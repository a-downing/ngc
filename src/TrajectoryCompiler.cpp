#include "machine/TrajectoryCompiler.h"
#include "machine/ArcInterpolation.h"
#include "utils.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <numbers>
#include <numeric>
#include <span>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <path_tempo/Planner.h>
#include <ruckig/ruckig.hpp>

namespace ngc {
    namespace {
        double dot(const vec3_t &a, const vec3_t &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
        vec3_t scale(const vec3_t &v, const double amount) { return { v.x*amount, v.y*amount, v.z*amount }; }
        position_t scaled(const position_t &value, const double amount) {
            return { value.x*amount, value.y*amount, value.z*amount,
                     value.a*amount, value.b*amount, value.c*amount };
        }
        position_t add(const position_t &a, const position_t &b) { return a + b; }
        position_t subtract(const position_t &a, const position_t &b) { return a - b; }

        position_t midpoint(const position_t &a, const position_t &b) {
            return scaled(add(a, b), 0.5);
        }

        template<typename T, std::size_t N>
        std::pair<std::array<T, N>, std::array<T, N>> splitBezier(
                const std::array<T, N> &control) {
            const auto middle = [](const T &a, const T &b) {
                if constexpr(std::same_as<T, double>) {
                    return std::midpoint(a, b);
                } else {
                    return midpoint(a, b);
                }
            };
            auto work = control;
            std::array<T, N> left{};
            std::array<T, N> right{};
            left.front() = work.front();
            right.back() = work.back();
            for (std::size_t level = 1; level < N; ++level) {
                for (std::size_t index = 0; index < N - level; ++index) {
                    work[index] = middle(work[index], work[index + 1]);
                }
                left[level] = work.front();
                right[N - level - 1] = work[N - level - 1];
            }
            return {left, right};
        }

        std::array<position_t, 4> bezierControls(const AxisPolynomialSpan &span) {
            return {
                span.origin,
                add(span.origin, scaled(span.coefficients[0], 1.0 / 3.0)),
                add(span.origin, add(
                    scaled(span.coefficients[0], 2.0 / 3.0),
                    scaled(span.coefficients[1], 1.0 / 3.0))),
                add(span.origin, add(span.coefficients[0],
                    add(span.coefficients[1], span.coefficients[2]))),
            };
        }

        double distanceToSegment(const position_t &point, const position_t &from, const position_t &to) {
            const vec3_t p { point.x, point.y, point.z };
            const vec3_t a { from.x, from.y, from.z };
            const vec3_t delta { to.x - from.x, to.y - from.y, to.z - from.z };
            const auto lengthSquared = dot(delta, delta);
            const auto parameter = lengthSquared > 0.0
                ? std::clamp(dot(p - a, delta) / lengthSquared, 0.0, 1.0) : 0.0;
            return (p - (a + scale(delta, parameter))).length();
        }

        struct PathSample { position_t position; position_t tangent; };
        struct TimeBoundary {
            double time;
            double distance;
            double velocity;
            double acceleration;
            double jerk=0.0;
        };

        class TimeLaw {
            // One initial boundary, two Ruckig brake phases, seven main phases,
            // and one possible terminal remainder.
            static constexpr std::size_t CAPACITY=11;
            std::array<TimeBoundary,CAPACITY> m_boundaries {};
            std::size_t m_size=0;

        public:
            TimeLaw()=default;
            TimeLaw(std::initializer_list<TimeBoundary> boundaries) {
                for(const auto &boundary:boundaries) push_back(boundary);
            }

            void push_back(const TimeBoundary &boundary) {
                if(m_size==CAPACITY) PANIC("local time law exceeded its fixed phase capacity");
                m_boundaries[m_size++]=boundary;
            }

            [[nodiscard]] std::size_t size() const { return m_size; }
            TimeBoundary &front() { return m_boundaries.front(); }
            const TimeBoundary &front() const { return m_boundaries.front(); }
            TimeBoundary &back() { return m_boundaries[m_size-1]; }
            const TimeBoundary &back() const { return m_boundaries[m_size-1]; }
            TimeBoundary &operator[](const std::size_t index) { return m_boundaries[index]; }
            const TimeBoundary &operator[](const std::size_t index) const {
                return m_boundaries[index];
            }
            auto begin() { return m_boundaries.begin(); }
            auto begin() const { return m_boundaries.begin(); }
            auto end() { return m_boundaries.begin()+m_size; }
            auto end() const { return m_boundaries.begin()+m_size; }
        };

        struct TimeLawWorkspace {
            path_tempo::ScalarTransitionPlanner planner;
        };

        enum class TimeLawPurpose {
            ExactStop,
        };

        struct TimeLawInstrumentation {
            TimeLawDiagnostics diagnostics;

            TimeLawCallDiagnostics &forPurpose(const TimeLawPurpose purpose) {
                switch(purpose) {
                    case TimeLawPurpose::ExactStop: return diagnostics.exactStop;
                }
                PANIC("invalid time-law instrumentation purpose");
            }

            TimeLawCallDiagnostics &begin(const TimeLawPurpose purpose,
                                          const bool correctionPass) {
                auto &result=forPurpose(purpose);
                ++result.calls;
                if(correctionPass) ++result.correctionPassCalls;
                return result;
            }
        };

        struct TimeLawCallTimer {
            TimeLawCallDiagnostics &diagnostics;
            std::chrono::steady_clock::time_point started=std::chrono::steady_clock::now();
            bool succeeded=false;

            ~TimeLawCallTimer() {
                diagnostics.seconds+=std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-started).count();
                if(succeeded) ++diagnostics.successes;
                else ++diagnostics.failures;
            }
        };

        struct TimeLawCompilationRecorder {
            TimeLawDiagnostics &destination;
            TimeLawInstrumentation instrumentation;

            explicit TimeLawCompilationRecorder(TimeLawDiagnostics &destinationValue)
                :destination(destinationValue) { }
            ~TimeLawCompilationRecorder() { destination=instrumentation.diagnostics; }
        };

        struct ScalarPhase {
            double a;
            double b;
            double c;
            double d;
            double duration;

            TimeBoundary at(const double u) const {
                return {
                    .time = duration * u,
                    .distance = ((a*u + b)*u + c)*u + d,
                    .velocity = (3.0*a*u*u + 2.0*b*u + c) / duration,
                    .acceleration = (6.0*a*u + 2.0*b) / (duration*duration),
                };
            }
        };

        ScalarPhase scalarPhase(const TimeBoundary &from, const TimeBoundary &to) {
            const auto duration = to.time - from.time;
            const auto delta = to.distance - from.distance;
            const auto c = from.velocity * duration;
            const auto endDerivative = to.velocity * duration;
            return {
                .a = -2.0*delta + c + endDerivative,
                .b = 3.0*delta - 2.0*c - endDerivative,
                .c = c,
                .d = from.distance,
                .duration = duration,
            };
        }

        AxisPolynomialSpan hermite(const SpanId id, const PathSample &from, const PathSample &to,
                                   const double fromVelocity, const double toVelocity, const double duration) {
            const auto v0t = scaled(from.tangent, fromVelocity * duration);
            const auto v1t = scaled(to.tangent, toVelocity * duration);
            const auto delta = subtract(to.position, from.position);
            AxisPolynomialSpan result;
            result.id = id;
            result.duration = duration;
            result.inverseDuration = 1.0 / duration;
            result.inverseDurationSquared = result.inverseDuration * result.inverseDuration;
            result.inverseDurationCubed = result.inverseDurationSquared * result.inverseDuration;
            result.origin = from.position;
            result.coefficients[0] = v0t;
            result.coefficients[1] = subtract(
                scaled(delta, 3.0), add(scaled(v0t, 2.0), v1t));
            result.coefficients[2] =
                add(scaled(delta, -2.0), add(v0t, v1t));
            return result;
        }

        struct KinematicPathState {
            position_t position{};
            position_t velocity{};
            position_t acceleration{};
        };

        struct LocalQuinticBezier {
            position_t origin{};
            std::array<position_t, 6> controls{};
        };

        LocalQuinticBezier localQuinticBezier(const KinematicPathState &from,
                                              const KinematicPathState &to,
                                              const double duration) {
            const auto durationSquared = duration * duration;
            const auto delta = subtract(to.position, from.position);
            LocalQuinticBezier result {
                .origin = from.position,
            };
            result.controls = {
                position_t{},
                scaled(from.velocity, duration / 5.0),
                add(scaled(from.velocity, 2.0 * duration / 5.0),
                    scaled(from.acceleration, durationSquared / 20.0)),
                add(delta, add(scaled(to.velocity, -2.0 * duration / 5.0),
                    scaled(to.acceleration, durationSquared / 20.0))),
                add(delta, scaled(to.velocity, -duration / 5.0)),
                delta,
            };

            return result;
        }

        std::array<position_t, 6> absoluteBezierControls(
                const LocalQuinticBezier &quintic) {
            auto result = quintic.controls;
            for (auto &control : result) {
                control = add(quintic.origin, control);
            }

            return result;
        }

        std::array<position_t, 6> quinticPowerCoefficients(
                const std::array<position_t, 6> &controls) {
            return {
                controls[0],
                scaled(subtract(controls[1], controls[0]), 5.0),
                scaled(add(subtract(controls[0], scaled(controls[1], 2.0)),
                    controls[2]), 10.0),
                scaled(add(subtract(scaled(controls[1], 3.0), controls[0]),
                    subtract(controls[3], scaled(controls[2], 3.0))), 10.0),
                scaled(add(subtract(add(controls[0], scaled(controls[2], 6.0)),
                    scaled(controls[1], 4.0)), subtract(controls[4],
                    scaled(controls[3], 4.0))), 5.0),
                add(subtract(add(scaled(controls[1], 5.0),
                    scaled(controls[3], 10.0)), controls[0]),
                    subtract(add(controls[5], scaled(controls[2], -10.0)),
                        scaled(controls[4], 5.0))),
            };
        }

        std::array<position_t, 6> absoluteQuinticPowerCoefficients(
                const LocalQuinticBezier &quintic) {
            auto result = quinticPowerCoefficients(quintic.controls);
            result.front() = add(quintic.origin, result.front());

            return result;
        }

        MotionState evaluateLocalNormalizedPolynomial(
                const position_t &origin,
                const std::array<position_t, 6> &coefficients,
                const double inverseDuration, const double parameter) {
            auto position = coefficients.back();
            for (std::size_t index = coefficients.size() - 1; index-- > 0;) {
                position = add(scaled(position, parameter), coefficients[index]);
            }
            auto velocity = scaled(coefficients.back(), 5.0);
            for (std::size_t index = coefficients.size() - 1; index-- > 1;) {
                velocity = add(scaled(velocity, parameter),
                    scaled(coefficients[index], static_cast<double>(index)));
            }
            auto acceleration = scaled(coefficients.back(), 20.0);
            for (std::size_t index = coefficients.size() - 1; index-- > 2;) {
                acceleration = add(scaled(acceleration, parameter),
                    scaled(coefficients[index],
                        static_cast<double>(index * (index - 1))));
            }

            return {
                .position = add(origin, position),
                .velocity = scaled(velocity, inverseDuration),
                .acceleration = scaled(
                    acceleration, inverseDuration * inverseDuration),
            };
        }

        template<std::size_t N>
        std::array<position_t, N - 1> derivativeBezierControls(
                const std::array<position_t, N> &controls, const double inverseDuration) {
            std::array<position_t, N - 1> result{};
            const auto scale = static_cast<double>(N - 1) * inverseDuration;
            for (std::size_t index = 0; index < result.size(); ++index) {
                result[index] = scaled(
                    subtract(controls[index + 1], controls[index]), scale);
            }
            return result;
        }

        template<std::size_t N>
        position_t evaluateBezierControls(
                const std::array<position_t, N> &controls, const double parameter) {
            auto work = controls;
            for (std::size_t remaining = N; remaining > 1; --remaining) {
                for (std::size_t control = 0; control + 1 < remaining; ++control) {
                    work[control] = add(scaled(work[control], 1.0 - parameter),
                        scaled(work[control + 1], parameter));
                }
            }
            return work.front();
        }

        position_t evaluateQuinticPosition(
                const LocalQuinticBezier &quintic, const double parameter) {
            return add(quintic.origin,
                evaluateBezierControls(quintic.controls, parameter));
        }

        double quinticEndpointResolutionJerkRatio(
                const LocalQuinticBezier &quintic, const double duration,
                const TrajectoryLimits &limits) {
            constexpr std::array components {
                &position_t::x, &position_t::y, &position_t::z,
                &position_t::a, &position_t::b, &position_t::c,
            };
            const auto terminal = add(quintic.origin, quintic.controls.back());
            position_t endpointResolution{};
            for (const auto component : components) {
                const auto from = quintic.origin.*component;
                const auto to = terminal.*component;
                const auto fromResolution =
                    std::nextafter(from, std::numeric_limits<double>::infinity()) - from;
                const auto toResolution =
                    std::nextafter(to, std::numeric_limits<double>::infinity()) - to;
                endpointResolution.*component =
                    std::abs(fromResolution) + std::abs(toResolution);
            }

            // Endpoint position resolution enters a quintic jerk-control numerator
            // with coefficient magnitude at most 60. Express that uncertainty in
            // the same time-domain path and axis jerk ratios used by certification.
            const auto jerkScale = 60.0 / (duration * duration * duration);
            auto result = jerkScale * endpointResolution.length() / limits.pathJerk;
            for (const auto component : components) {
                result = std::max(result, jerkScale
                    * std::abs(endpointResolution.*component)
                    / (limits.axisJerk.*component));
            }

            return result;
        }

        double evaluatePolynomial(
                const std::span<const double> coefficients, const double parameter) {
            auto result = 0.0;
            for (auto coefficient = coefficients.rbegin();
                    coefficient != coefficients.rend(); ++coefficient) {
                result = result * parameter + *coefficient;
            }
            return result;
        }

        std::vector<double> polynomialRootsInUnitInterval(
                std::vector<double> coefficients) {
            auto scale = 0.0;
            for (const auto coefficient : coefficients) {
                scale = std::max(scale, std::abs(coefficient));
            }
            if (scale == 0.0) {
                return {};
            }
            for (auto &coefficient : coefficients) {
                coefficient /= scale;
            }
            constexpr auto COEFFICIENT_TOLERANCE =
                64.0 * std::numeric_limits<double>::epsilon();
            while (coefficients.size() > 1
                  && std::abs(coefficients.back()) <= COEFFICIENT_TOLERANCE) {
                coefficients.pop_back();
            }
            if (coefficients.size() == 1) {
                return {};
            }
            if (coefficients.size() == 2) {
                const auto root = -coefficients[0] / coefficients[1];
                if (root >= 0.0 && root <= 1.0) {
                    return {root};
                }
                return {};
            }

            std::vector<double> derivative(coefficients.size() - 1);
            for (std::size_t index = 1; index < coefficients.size(); ++index) {
                derivative[index - 1] =
                    static_cast<double>(index) * coefficients[index];
            }
            auto boundaries = polynomialRootsInUnitInterval(std::move(derivative));
            boundaries.push_back(0.0);
            boundaries.push_back(1.0);
            std::ranges::sort(boundaries);
            const auto duplicate = [](const double left, const double right) {
                return std::abs(left - right) <= 1e-14;
            };
            boundaries.erase(
                std::unique(boundaries.begin(), boundaries.end(), duplicate),
                boundaries.end());

            std::vector<double> roots;
            constexpr auto VALUE_TOLERANCE = 1e-12;
            const auto add = [&](const double root) {
                roots.push_back(std::clamp(root, 0.0, 1.0));
            };
            for (const auto boundary : boundaries) {
                if (std::abs(evaluatePolynomial(coefficients, boundary))
                        <= VALUE_TOLERANCE) {
                    add(boundary);
                }
            }
            for (std::size_t interval = 1; interval < boundaries.size(); ++interval) {
                auto left = boundaries[interval - 1];
                auto right = boundaries[interval];
                auto leftValue = evaluatePolynomial(coefficients, left);
                const auto rightValue = evaluatePolynomial(coefficients, right);
                if (leftValue == 0.0 || rightValue == 0.0
                   || std::signbit(leftValue) == std::signbit(rightValue)) {
                    continue;
                }
                for (unsigned iteration = 0; iteration < 80; ++iteration) {
                    const auto middle = std::midpoint(left, right);
                    const auto middleValue =
                        evaluatePolynomial(coefficients, middle);
                    if (middleValue == 0.0) {
                        left = middle;
                        right = middle;
                        break;
                    }
                    if (std::signbit(leftValue) == std::signbit(middleValue)) {
                        left = middle;
                        leftValue = middleValue;
                    } else {
                        right = middle;
                    }
                }
                add(std::midpoint(left, right));
            }
            std::ranges::sort(roots);
            roots.erase(std::unique(roots.begin(), roots.end(), duplicate), roots.end());
            return roots;
        }

        struct QuinticJerkMaximum {
            double parameter = 0.0;
            position_t jerk{};
        };

        QuinticJerkMaximum maximumQuinticJerk(
                const std::array<position_t, 6> &controls, const double duration) {
            const auto inverseDuration = 1.0 / duration;
            const auto velocityControls =
                derivativeBezierControls(controls, inverseDuration);
            const auto accelerationControls =
                derivativeBezierControls(velocityControls, inverseDuration);
            const auto jerkControls =
                derivativeBezierControls(accelerationControls, inverseDuration);
            const auto constant = jerkControls[0];
            const auto linear =
                scaled(subtract(jerkControls[1], jerkControls[0]), 2.0);
            const auto quadratic = add(subtract(jerkControls[0],
                scaled(jerkControls[1], 2.0)), jerkControls[2]);
            const auto dot = [](const position_t &left, const position_t &right) {
                return left.x * right.x + left.y * right.y + left.z * right.z
                    + left.a * right.a + left.b * right.b + left.c * right.c;
            };
            const std::vector stationaryPolynomial {
                dot(constant, linear),
                2.0 * dot(constant, quadratic) + dot(linear, linear),
                3.0 * dot(linear, quadratic),
                2.0 * dot(quadratic, quadratic),
            };
            auto candidates = polynomialRootsInUnitInterval(stationaryPolynomial);
            candidates.push_back(0.0);
            candidates.push_back(1.0);
            QuinticJerkMaximum result;
            for (const auto parameter : candidates) {
                const auto jerk = evaluateBezierControls(jerkControls, parameter);
                if (jerk.length() > result.jerk.length()) {
                    result = {.parameter = parameter, .jerk = jerk};
                }
            }
            return result;
        }

        std::array<AxisPolynomialSpan,3> c2CubicChain(const SpanId firstId,
                                                      const KinematicPathState &from,
                                                      const KinematicPathState &to,
                                                      const double duration) {
            const auto h=duration/3.0;
            const auto inverseH=1.0/h;
            const auto accelerationDelta=scaled(subtract(to.acceleration,from.acceleration),inverseH);
            const auto velocityDelta=scaled(subtract(subtract(to.velocity,from.velocity),
                scaled(from.acceleration,3.0*h)),1.0/(h*h));
            const auto positionDelta=scaled(subtract(subtract(subtract(to.position,from.position),
                scaled(from.velocity,3.0*h)),scaled(from.acceleration,4.5*h*h)),1.0/(h*h*h));
            const auto velocityEquation=subtract(velocityDelta,scaled(accelerationDelta,0.5));
            const auto positionEquation=subtract(positionDelta,scaled(accelerationDelta,1.0/6.0));
            const auto jerk0=subtract(positionEquation,velocityEquation);
            const auto jerk1=subtract(velocityEquation,scaled(jerk0,2.0));
            const auto jerk2=subtract(subtract(accelerationDelta,jerk0),jerk1);
            const std::array jerks{jerk0,jerk1,jerk2};

            std::array<AxisPolynomialSpan,3> result;
            KinematicPathState state=from;
            for(std::size_t index=0;index<result.size();++index) {
                auto &span=result[index];
                span.id=firstId+index;
                span.duration=h;
                span.inverseDuration=inverseH;
                span.inverseDurationSquared=inverseH*inverseH;
                span.inverseDurationCubed=span.inverseDurationSquared*inverseH;
                span.origin = state.position;
                span.coefficients[0] = scaled(state.velocity, h);
                span.coefficients[1] =
                    scaled(state.acceleration, h * h / 2.0);
                span.coefficients[2] =
                    scaled(jerks[index], h * h * h / 6.0);
                KinematicPathState terminal {
                    .position = add(state.position, add(
                        add(span.coefficients[2], span.coefficients[1]),
                        span.coefficients[0])),
                    .velocity=add(state.velocity,add(scaled(state.acceleration,h),
                        scaled(jerks[index],h*h/2.0))),
                    .acceleration=add(state.acceleration,scaled(jerks[index],h)),
                };
                if(index+1==result.size()) terminal=to;
                state=terminal;
            }
            return result;
        }

        ScalarPhase localScalarPhase(const TimeBoundary &from,const TimeBoundary &to) {
            const auto duration=to.time-from.time;
            return {
                .a=from.jerk*duration*duration*duration/6.0,
                .b=from.acceleration*duration*duration/2.0,
                .c=from.velocity*duration,
                .d=from.distance,
                .duration=duration,
            };
        }

        std::array<double,6> axisArray(const position_t &value) {
            return {value.x,value.y,value.z,value.a,value.b,value.c};
        }

        position_t axisPosition(const std::array<double,6> &value) {
            return {value[0],value[1],value[2],value[3],value[4],value[5]};
        }

        AxisPolynomialSpan constantJerkSpan(const SpanId id,const MotionState &from,
                                            const position_t &jerk,const MotionState &to,
                                            const double duration) {
            AxisPolynomialSpan result;
            result.id=id;
            result.duration=duration;
            result.inverseDuration=1.0/duration;
            result.inverseDurationSquared=result.inverseDuration*result.inverseDuration;
            result.inverseDurationCubed=result.inverseDurationSquared*result.inverseDuration;
            result.origin = from.position;
            result.coefficients[0] = scaled(from.velocity, duration);
            result.coefficients[1] =
                scaled(from.acceleration, duration * duration / 2.0);
            result.coefficients[2] =
                scaled(jerk, duration * duration * duration / 6.0);
            (void)to;
            return result;
        }

        std::expected<std::vector<AxisPolynomialSpan>,std::string> stoppingSpans(
                const MotionState &from,const TrajectoryLimits &limits,SpanId &nextSpan) {
            ruckig::InputParameter<6> input;
            input.control_interface=ruckig::ControlInterface::Velocity;
            input.current_position=axisArray(from.position);
            input.current_velocity=axisArray(from.velocity);
            input.current_acceleration=axisArray(from.acceleration);
            input.target_velocity={0.0,0.0,0.0,0.0,0.0,0.0};
            input.target_acceleration={0.0,0.0,0.0,0.0,0.0,0.0};
            const auto conservative=1.0/std::sqrt(6.0);
            const auto acceleration=axisArray(limits.axisAcceleration);
            const auto jerk=axisArray(limits.axisJerk);
            for(std::size_t axis=0;axis<6;++axis) {
                input.max_acceleration[axis]=std::min(acceleration[axis],
                    limits.pathAcceleration*conservative);
                input.max_jerk[axis]=std::min(jerk[axis],limits.pathJerk*conservative);
            }
            ruckig::Ruckig<6> generator;
            ruckig::Trajectory<6> trajectory;
            if(generator.calculate(input,trajectory)!=ruckig::Result::Working)
                return std::unexpected("Ruckig failed to calculate a moving-boundary stop trajectory");

            std::vector<double> times{0.0,trajectory.get_duration()};
            const auto trajectoryProfiles=trajectory.get_profiles();
            const auto &profiles=trajectoryProfiles.front();
            for(const auto &profile:profiles) {
                auto elapsed=0.0;
                for(const auto duration:profile.brake.t) {
                    elapsed+=duration;
                    if(elapsed>1e-12&&elapsed<trajectory.get_duration()-1e-12)
                        times.push_back(elapsed);
                }
                for(const auto cumulative:profile.t_sum) {
                    const auto time=profile.brake.duration+cumulative;
                    if(time>1e-12&&time<trajectory.get_duration()-1e-12)
                        times.push_back(time);
                }
            }
            std::ranges::sort(times);
            times.erase(std::unique(times.begin(),times.end(),[](const double left,const double right) {
                return std::abs(left-right)<=1e-11;
            }),times.end());

            std::vector<AxisPolynomialSpan> result;
            result.reserve(times.size()-1);
            MotionState previous=from;
            for(std::size_t boundary=1;boundary<times.size();++boundary) {
                const auto start=times[boundary-1];
                const auto end=times[boundary];
                const auto duration=end-start;
                if(duration<=1e-12) continue;
                std::array<double,6> position{},velocity{},acceleration{},jerkAtMiddle{};
                std::size_t section=0;
                trajectory.at_time(end,position,velocity,acceleration);
                std::array<double,6> ignoredPosition{},ignoredVelocity{},ignoredAcceleration{};
                trajectory.at_time(std::midpoint(start,end),ignoredPosition,ignoredVelocity,
                    ignoredAcceleration,jerkAtMiddle,section);
                MotionState terminal{axisPosition(position),axisPosition(velocity),axisPosition(acceleration)};
                result.push_back(constantJerkSpan(nextSpan++,previous,axisPosition(jerkAtMiddle),
                    terminal,duration));
                previous=terminal;
            }
            if(result.empty())
                return std::unexpected("moving-boundary stop trajectory produced no polynomial spans");
            return result;
        }

        template<typename PositionAt>
        bool verifiesArcTolerance(const AxisPolynomialSpan &span, const double distance0, const double distance1,
                                  const double velocity0, const double velocity1, const double tolerance,
                                  const PositionAt &positionAt, const auto &referenceErrorAt) {
            // The scalar controls retain the time law's ordered association between this
            // polynomial and its source-arc interval as both curves are subdivided.
            const std::array<double, 4> scalarControl {
                distance0,
                distance0 + velocity0*span.duration / 3.0,
                distance1 - velocity1*span.duration / 3.0,
                distance1,
            };

            const auto verify = [&](const auto &self, const std::array<position_t, 4> &curve,
                                    const std::array<double, 4> &distance, const unsigned depth) -> bool {
                const auto source0 = positionAt(distance[0]);
                const auto source1 = positionAt(distance[3]);
                const auto referenceError = referenceErrorAt(distance[0], distance[3]);
                const auto availableError = tolerance - referenceError;
                // A segment capsule is convex. If every polynomial control point is in
                // this capsule, its entire hull is within availableError of the chord;
                // the circular/helical source interval is within referenceError of it.
                if(availableError >= 0.0
                   && std::ranges::all_of(curve, [&](const position_t &control) {
                       return distanceToSegment(control, source0, source1) <= availableError;
                   })) return true;

                if(depth == 20) return false;
                const auto [leftCurve, rightCurve] = splitBezier(curve);
                const auto [leftDistance, rightDistance] = splitBezier(distance);
                return self(self, leftCurve, leftDistance, depth + 1)
                    && self(self, rightCurve, rightDistance, depth + 1);
            };
            return verify(verify, bezierControls(span), scalarControl, 0);
        }

        template<std::size_t N, typename PositionAt>
        bool verifiesOrderedCurveTolerance(const std::array<position_t, N> &controls,
                                           const double distance0,const double distance1,
                                           const double tolerance,const PositionAt &positionAt,
                                           const auto &referenceErrorAt,
                                           const bool requireForwardProgress = false) {
            std::size_t visitedNodes=0;
            constexpr std::size_t MAX_VISITED_NODES=64;
            const auto verify=[&](const auto &self,const std::array<position_t,N> &curve,
                                  const double from,const double to,const unsigned depth) -> bool {
                if(++visitedNodes>MAX_VISITED_NODES) return false;
                const auto source0=positionAt(from);
                const auto source1=positionAt(to);
                const auto available=tolerance-referenceErrorAt(from,to);
                const auto geometryVerified =
                    available>=0.0&&std::ranges::all_of(curve,[&](const position_t &control) {
                        return distanceToSegment(control,source0,source1)<=available;
                    });
                auto progressVerified = !requireForwardProgress;
                if (geometryVerified && requireForwardProgress) {
                    const auto direction = subtract(source1, source0);
                    const auto directionSquared =
                        direction.x * direction.x + direction.y * direction.y
                        + direction.z * direction.z + direction.a * direction.a
                        + direction.b * direction.b + direction.c * direction.c;
                    const auto toleranceSquared = std::max(1e-24, directionSquared * 1e-12);
                    progressVerified = directionSquared > 1e-24;
                    for (std::size_t control = 1;
                            progressVerified && control < curve.size(); ++control) {
                        const auto delta = subtract(curve[control], curve[control - 1]);
                        const auto projection =
                            delta.x * direction.x + delta.y * direction.y
                            + delta.z * direction.z + delta.a * direction.a
                            + delta.b * direction.b + delta.c * direction.c;
                        progressVerified = projection >= -toleranceSquared;
                    }
                }
                if (geometryVerified && progressVerified) {
                    return true;
                }
                if(depth>=20) return false;
                const auto [left,right]=splitBezier(curve);
                const auto middle=std::midpoint(from,to);
                return self(self,left,from,middle,depth+1)
                    &&self(self,right,middle,to,depth+1);
            };
            return verify(verify,controls,distance0,distance1,0);
        }

        template<typename PositionAt>
        bool verifiesOrderedCurveTolerance(const AxisPolynomialSpan &span,const double distance0,
                                           const double distance1,const double tolerance,
                                           const PositionAt &positionAt,const auto &referenceErrorAt) {
            return verifiesOrderedCurveTolerance(bezierControls(span),distance0,distance1,
                tolerance,positionAt,referenceErrorAt);
        }

        double dotPosition(
                const position_t &left, const position_t &right) {
            return left.x * right.x + left.y * right.y
                + left.z * right.z + left.a * right.a
                + left.b * right.b + left.c * right.c;
        }

        double maximumDerivativeMagnitude(const AxisPolynomialSpan &span,
                                          const unsigned derivative) {
            std::vector<position_t> coefficients;
            if (derivative == 2) {
                coefficients = {
                    scaled(span.coefficients[1], 2.0),
                    scaled(span.coefficients[2], 6.0),
                    scaled(span.coefficients[3], 12.0),
                    scaled(span.coefficients[4], 20.0),
                };
            } else {
                coefficients = {
                    scaled(span.coefficients[2], 6.0),
                    scaled(span.coefficients[3], 24.0),
                    scaled(span.coefficients[4], 60.0),
                };
            }
            std::vector<double> squared(
                coefficients.size() * 2 - 1, 0.0);
            for (std::size_t left = 0; left < coefficients.size(); ++left) {
                for (std::size_t right = 0;
                        right < coefficients.size(); ++right) {
                    squared[left + right] +=
                        dotPosition(coefficients[left], coefficients[right]);
                }
            }
            std::vector<double> stationary(squared.size() - 1);
            for (std::size_t index = 1; index < squared.size(); ++index) {
                stationary[index - 1] =
                    static_cast<double>(index) * squared[index];
            }
            auto maximum = 0.0;
            const auto observe = [&](const double parameter) {
                const auto evaluation =
                    evaluateExecutionPolynomial(span, parameter);
                const auto value = derivative == 2
                    ? evaluation.state.acceleration : evaluation.jerk;
                maximum = std::max(maximum, value.length());
            };
            observe(0.0);
            observe(1.0);
            for (const auto root :
                    polynomialRootsInUnitInterval(std::move(stationary))) {
                observe(root);
            }
            return maximum;
        }

        double maximumLinearAcceleration(const AxisPolynomialSpan &span) {
            return maximumDerivativeMagnitude(span, 2);
        }

        double maximumLinearJerk(const AxisPolynomialSpan &span) {
            return maximumDerivativeMagnitude(span, 3);
        }

        std::size_t polynomialSeverityBin(const double ratio) {
            if (ratio <= 1.0) {
                return 0;
            }
            if (ratio <= 1.0 + 1e-9) {
                return 1;
            }
            if (ratio <= 1.001) {
                return 2;
            }
            if (ratio <= 1.01) {
                return 3;
            }
            if (ratio <= 1.05) {
                return 4;
            }
            if (ratio <= 1.25) {
                return 5;
            }
            if (ratio <= 1.5) {
                return 6;
            }
            if (ratio <= 2.0) {
                return 7;
            }
            return 8;
        }

        constexpr std::array AXIS_COMPONENTS {
            &position_t::x, &position_t::y, &position_t::z,
            &position_t::a, &position_t::b, &position_t::c,
        };

        struct QuinticConstraintBounds {
            double maximumRatio = 0.0;
            double maximumCorrectionRatio = 0.0;
            ContinuousPolynomialConstraintKind constraint =
                ContinuousPolynomialConstraintKind::PathAcceleration;
            std::size_t axis = std::numeric_limits<std::size_t>::max();
            double maximumVelocityRatio = 0.0;
            ContinuousPolynomialConstraintKind velocityConstraint =
                ContinuousPolynomialConstraintKind::PathVelocity;
            std::size_t velocityAxis = std::numeric_limits<std::size_t>::max();
            double maximumAccelerationRatio = 0.0;
            ContinuousPolynomialConstraintKind accelerationConstraint =
                ContinuousPolynomialConstraintKind::PathAcceleration;
            std::size_t accelerationAxis =
                std::numeric_limits<std::size_t>::max();
            double maximumJerkRatio = 0.0;
            ContinuousPolynomialConstraintKind jerkConstraint =
                ContinuousPolynomialConstraintKind::PathJerk;
            std::size_t jerkAxis = std::numeric_limits<std::size_t>::max();
        };

        template<std::size_t N, typename Magnitude>
        double certifiedBezierMaximum(const std::array<position_t, N> &controls,
                                      const double limit, const Magnitude &magnitude,
                                      std::size_t &visitedNodes, const unsigned depth = 0) {
            ++visitedNodes;
            auto upper = 0.0;
            for (const auto &control : controls) {
                upper = std::max(upper, magnitude(control));
            }
            if (upper <= limit || depth >= 12) {
                return upper;
            }
            const auto [left, right] = splitBezier(controls);
            const auto lower = std::max({magnitude(controls.front()),
                magnitude(left.back()), magnitude(controls.back())});
            if (upper - lower <= std::max(1e-12, upper * 1e-8)) {
                return upper;
            }
            return std::max(
                certifiedBezierMaximum(left, limit, magnitude, visitedNodes, depth + 1),
                certifiedBezierMaximum(right, limit, magnitude, visitedNodes, depth + 1));
        }

        QuinticConstraintBounds quinticConstraintBounds(
                const std::array<position_t, 6> &positionControls, const double duration,
                const double pathVelocityLimit, const TrajectoryLimits &limits,
                std::size_t &visitedNodes) {
            const auto inverseDuration = 1.0 / duration;
            const auto velocityControls = derivativeBezierControls(
                positionControls, inverseDuration);
            const auto accelerationControls = derivativeBezierControls(
                velocityControls, inverseDuration);
            const auto jerkControls = derivativeBezierControls(
                accelerationControls, inverseDuration);
            QuinticConstraintBounds result;

            const auto observe = [&](const double measured, const double limit,
                    const ContinuousPolynomialConstraintKind constraint,
                    const std::size_t axis) {
                const auto ratio = measured / limit;
                result.maximumRatio = std::max(result.maximumRatio, ratio);
                const auto acceleration =
                    constraint == ContinuousPolynomialConstraintKind::PathAcceleration
                    || constraint
                        == ContinuousPolynomialConstraintKind::AxisAcceleration;
                const auto jerk =
                    constraint == ContinuousPolynomialConstraintKind::PathJerk
                    || constraint == ContinuousPolynomialConstraintKind::AxisJerk;
                const auto correctionRatio =
                    ratio / ((acceleration || jerk)
                        ? trajectory_detail::DYNAMIC_LIMIT_RATIO : 1.0);
                if (correctionRatio > result.maximumCorrectionRatio) {
                    result.maximumCorrectionRatio = correctionRatio;
                    result.constraint = constraint;
                    result.axis = axis;
                }
                if (jerk) {
                    if (ratio > result.maximumJerkRatio) {
                        result.maximumJerkRatio = ratio;
                        result.jerkConstraint = constraint;
                        result.jerkAxis = axis;
                    }
                } else if (acceleration) {
                    if (ratio > result.maximumAccelerationRatio) {
                        result.maximumAccelerationRatio = ratio;
                        result.accelerationConstraint = constraint;
                        result.accelerationAxis = axis;
                    }
                } else {
                    if (ratio > result.maximumVelocityRatio) {
                        result.maximumVelocityRatio = ratio;
                        result.velocityConstraint = constraint;
                        result.velocityAxis = axis;
                    }
                }
            };

            const auto vectorMagnitude = [](const position_t &control) {
                return control.length();
            };
            observe(certifiedBezierMaximum(velocityControls, pathVelocityLimit,
                        vectorMagnitude, visitedNodes), pathVelocityLimit,
                ContinuousPolynomialConstraintKind::PathVelocity,
                std::numeric_limits<std::size_t>::max());
            observe(certifiedBezierMaximum(accelerationControls,
                        limits.pathAcceleration
                            * trajectory_detail::DYNAMIC_LIMIT_RATIO,
                        vectorMagnitude, visitedNodes), limits.pathAcceleration,
                ContinuousPolynomialConstraintKind::PathAcceleration,
                std::numeric_limits<std::size_t>::max());
            observe(certifiedBezierMaximum(jerkControls,
                        limits.pathJerk
                            * trajectory_detail::DYNAMIC_LIMIT_RATIO,
                        vectorMagnitude, visitedNodes), limits.pathJerk,
                ContinuousPolynomialConstraintKind::PathJerk,
                std::numeric_limits<std::size_t>::max());
            for (std::size_t axis = 0; axis < AXIS_COMPONENTS.size(); ++axis) {
                const auto component = AXIS_COMPONENTS[axis];
                const auto componentMagnitude = [&](const position_t &control) {
                    return std::abs(control.*component);
                };
                observe(certifiedBezierMaximum(velocityControls,
                            limits.axisVelocity.*component, componentMagnitude, visitedNodes),
                    limits.axisVelocity.*component,
                    ContinuousPolynomialConstraintKind::AxisVelocity, axis);
                observe(certifiedBezierMaximum(accelerationControls,
                            (limits.axisAcceleration.*component)
                                * trajectory_detail::DYNAMIC_LIMIT_RATIO,
                            componentMagnitude, visitedNodes),
                    limits.axisAcceleration.*component,
                    ContinuousPolynomialConstraintKind::AxisAcceleration, axis);
                observe(certifiedBezierMaximum(jerkControls,
                            (limits.axisJerk.*component)
                                * trajectory_detail::DYNAMIC_LIMIT_RATIO,
                            componentMagnitude, visitedNodes),
                    limits.axisJerk.*component,
                    ContinuousPolynomialConstraintKind::AxisJerk, axis);
            }
            return result;
        }

        double quinticAccelerationExcursionRatio(
                const std::array<position_t, 6> &positionControls,
                const double duration, const double servoPeriod,
                const TrajectoryLimits &limits) {
            const auto inverseDuration = 1.0 / duration;
            const auto velocityControls = derivativeBezierControls(
                positionControls, inverseDuration);
            const auto accelerationControls = derivativeBezierControls(
                velocityControls, inverseDuration);

            // A cubic acceleration curve stays inside its Bezier control hull.
            // The control-hull diameter therefore certifies every possible
            // acceleration excursion within this isolated quintic.
            return trajectory_detail::accelerationControlHullExcursionRatio(
                accelerationControls, servoPeriod, limits.pathJerk,
                limits.axisJerk);
        }

        QuinticConstraintBounds sampledQuinticConstraintBounds(
                const std::array<position_t, 6> &positionControls, const double duration,
                const double pathVelocityLimit, const TrajectoryLimits &limits) {
            const auto inverseDuration = 1.0 / duration;
            const auto velocityControls = derivativeBezierControls(
                positionControls, inverseDuration);
            const auto accelerationControls = derivativeBezierControls(
                velocityControls, inverseDuration);
            const auto jerkControls = derivativeBezierControls(
                accelerationControls, inverseDuration);
            QuinticConstraintBounds result;
            const auto observe = [&](const double measured, const double limit,
                    const ContinuousPolynomialConstraintKind constraint,
                    const std::size_t axis) {
                const auto ratio = measured / limit;
                if (ratio > result.maximumRatio) {
                    result.maximumRatio = ratio;
                    result.constraint = constraint;
                    result.axis = axis;
                }
            };
            constexpr std::size_t SAMPLE_INTERVALS = 128;
            for (std::size_t sample = 0; sample <= SAMPLE_INTERVALS; ++sample) {
                const auto parameter =
                    static_cast<double>(sample) / SAMPLE_INTERVALS;
                const auto velocity =
                    evaluateBezierControls(velocityControls, parameter);
                const auto acceleration =
                    evaluateBezierControls(accelerationControls, parameter);
                const auto jerk = evaluateBezierControls(jerkControls, parameter);
                observe(velocity.length(), pathVelocityLimit,
                    ContinuousPolynomialConstraintKind::PathVelocity,
                    std::numeric_limits<std::size_t>::max());
                observe(acceleration.length(), limits.pathAcceleration,
                    ContinuousPolynomialConstraintKind::PathAcceleration,
                    std::numeric_limits<std::size_t>::max());
                observe(jerk.length(), limits.pathJerk,
                    ContinuousPolynomialConstraintKind::PathJerk,
                    std::numeric_limits<std::size_t>::max());
                for (std::size_t axis = 0; axis < AXIS_COMPONENTS.size(); ++axis) {
                    const auto component = AXIS_COMPONENTS[axis];
                    observe(std::abs(velocity.*component),
                        limits.axisVelocity.*component,
                        ContinuousPolynomialConstraintKind::AxisVelocity, axis);
                    observe(std::abs(acceleration.*component),
                        limits.axisAcceleration.*component,
                        ContinuousPolynomialConstraintKind::AxisAcceleration, axis);
                    observe(std::abs(jerk.*component), limits.axisJerk.*component,
                        ContinuousPolynomialConstraintKind::AxisJerk, axis);
                }
            }
            return result;
        }

        bool positiveAxisLimits(const position_t &limits) {
            return std::ranges::all_of(AXIS_COMPONENTS, [&](const auto component) {
                return limits.*component > 0.0;
            });
        }

        double pathLimit(const position_t &maximumTangent, const position_t &axisLimits) {
            auto result = std::numeric_limits<double>::infinity();
            for(const auto component : AXIS_COMPONENTS) {
                const auto derivative = std::abs(maximumTangent.*component);
                if(derivative > 1e-15)
                    result = std::min(result, axisLimits.*component / derivative);
            }
            return result;
        }

        std::expected<TimeLaw, std::string> solveTimeLawBetween(TimeLawWorkspace &workspace,
                const double length,const double fromVelocity,const double fromAcceleration,
                const double toVelocity,const double toAcceleration,
                const double requestedVelocity,const double acceleration,const double jerk) {
            const auto transition=workspace.planner.solve({
                .piece=0,
                .length=length,
                .beginning={.velocity=fromVelocity,.acceleration=fromAcceleration},
                .ending={.velocity=toVelocity,.acceleration=toAcceleration},
                .maximumVelocity=requestedVelocity,
                .maximumAcceleration=acceleration,
                .maximumJerk=jerk,
            });
            if(!transition) return std::unexpected(transition.error().message);

            TimeLaw result {{0.0,0.0,fromVelocity,fromAcceleration,0.0}};
            auto phaseTime=0.0;
            for(const auto &segment:*transition) {
                auto &from=result.back();
                from.jerk=segment.jerk();
                phaseTime+=segment.duration;
                result.push_back({
                    phaseTime,
                    segment.position(segment.duration),
                    segment.velocity(segment.duration),
                    segment.acceleration(segment.duration),
                    0.0,
                });
            }
            result.front().time=0.0;
            result.front().distance=0.0;
            result.front().velocity=fromVelocity;
            result.front().acceleration=fromAcceleration;
            result.back().time=transition->duration();
            result.back().distance=length;
            result.back().velocity=toVelocity;
            result.back().acceleration=toAcceleration;
            result.back().jerk=0.0;

            return result;
        }

        std::expected<TimeLaw, std::string> timeLawBetween(TimeLawWorkspace &workspace,
                TimeLawInstrumentation &instrumentation,const TimeLawPurpose purpose,
                const bool correctionPass,
                const double length,const double fromVelocity,const double fromAcceleration,
                const double toVelocity,const double toAcceleration,
                const double requestedVelocity,const double acceleration,const double jerk) {
            auto &diagnostics=instrumentation.begin(purpose,correctionPass);
            TimeLawCallTimer timer {diagnostics};
            ++diagnostics.solverCalls;
            auto result=solveTimeLawBetween(workspace,length,fromVelocity,fromAcceleration,
                toVelocity,toAcceleration,requestedVelocity,acceleration,jerk);
            timer.succeeded=result.has_value();
            return result;
        }

        std::expected<TimeLaw, std::string> timeLaw(TimeLawWorkspace &workspace,
                TimeLawInstrumentation &instrumentation,
                const double length, const double requestedVelocity, const double acceleration, const double jerk) {
            if(std::isinf(acceleration)) return TimeLaw{
                {0.0,0.0,requestedVelocity,0.0},
                {length/requestedVelocity,length,requestedVelocity,0.0},
            };
            return timeLawBetween(workspace,instrumentation,TimeLawPurpose::ExactStop,false,
                                  length,0.0,0.0,0.0,0.0,
                                  requestedVelocity,acceleration,jerk);
        }

        double positionDot(const position_t &left, const position_t &right) {
            return left.x*right.x + left.y*right.y + left.z*right.z
                + left.a*right.a + left.b*right.b + left.c*right.c;
        }

        struct GeometryActivation {
            std::size_t input=0;
            double distance=0.0;
        };

        struct GeometryPiece {
            std::size_t input=0;
            PreparedPieceId preparedPiece=0;
            PreparedPieceKind preparedKind=PreparedPieceKind::RetainedLineSection;
            std::size_t knotInterval=std::numeric_limits<std::size_t>::max();
            std::size_t firstSourceInput=0;
            std::size_t lastSourceInput=0;
            std::size_t sourceInputCount=0;
            std::vector<GeometryActivation> activations{};
            double length=0.0;
            double curveFrom=0.0;
            double curveTo=0.0;
            double programmedVelocity=0.0;
            double staticVelocityLimit=std::numeric_limits<double>::infinity();
            bool linear=false;
            std::span<const PreparedGeometricSample> geometricSamples{};
            double geometricSampleDistanceOffset=0.0;
            std::function<PathSample(double)> sampleAt;
            std::function<position_t(double)> curvatureAt;
            std::function<position_t(double)> infiniteJerkCurvatureAt;
            std::function<position_t(double)> curvatureDerivativeAt;
            std::function<position_t(double)> positionAt;
            std::function<double(double,double)> chordErrorBound;
        };

        path_tempo::Vector<6> pathTempoVector(const position_t &value) {
            return {value.x,value.y,value.z,value.a,value.b,value.c};
        }

        std::expected<std::vector<TimeLaw>,std::string> localTimeLaws(
                const path_tempo::PlannedPath &planned,
                const std::span<const GeometryPiece> pieces) {
            std::vector<TimeLaw> result(pieces.size());
            std::vector<double> elapsed(pieces.size(),0.0);
            std::vector<double> offsets(pieces.size(),0.0);
            for (std::size_t piece=1;piece<pieces.size();++piece) {
                offsets[piece]=offsets[piece-1]+pieces[piece-1].length;
            }

            auto previousPiece=std::size_t {0};
            for (const auto &segment:planned.timeLaw.segments) {
                if (segment.piece==0||segment.piece>pieces.size()) {
                    return std::unexpected(std::format(
                        "PathTempo emitted unknown piece ID {}",segment.piece));
                }
                const auto piece=static_cast<std::size_t>(segment.piece-1);
                if (!result[piece].size()) {
                    if (piece<previousPiece) {
                        return std::unexpected(
                            "PathTempo emitted time-law pieces out of order");
                    }
                    previousPiece=piece;
                }
                const auto localDistance=segment.c0-offsets[piece];
                const auto tolerance=std::max(1e-10,pieces[piece].length*1e-10);
                auto &timing=result[piece];
                if (timing.size()==0) {
                    if (std::abs(localDistance)>tolerance) {
                        return std::unexpected(std::format(
                            "PathTempo piece {} starts at local distance {}",piece,
                            localDistance));
                    }
                    timing.push_back({
                        .time=0.0,
                        .distance=0.0,
                        .velocity=segment.velocity(0.0),
                        .acceleration=segment.acceleration(0.0),
                        .jerk=segment.jerk(),
                    });
                } else {
                    if (std::abs(timing.back().distance-localDistance)>tolerance) {
                        return std::unexpected(std::format(
                            "PathTempo piece {} has a discontinuous scalar phase",piece));
                    }
                    timing.back().jerk=segment.jerk();
                }
                elapsed[piece]+=segment.duration;
                timing.push_back({
                    .time=elapsed[piece],
                    .distance=segment.position(segment.duration)-offsets[piece],
                    .velocity=segment.velocity(segment.duration),
                    .acceleration=segment.acceleration(segment.duration),
                });
            }
            for (std::size_t piece=0;piece<pieces.size();++piece) {
                const auto tolerance=std::max(1e-10,pieces[piece].length*1e-10);
                if (result[piece].size()<2
                   ||std::abs(result[piece].back().distance-pieces[piece].length)>tolerance) {
                    return std::unexpected(std::format(
                        "PathTempo did not cover continuous piece {}",piece));
                }
                result[piece].back().distance=pieces[piece].length;
            }

            return result;
        }
    }

    namespace trajectory_detail {
        namespace {
            double maximumAbsolutePolynomial(
                    const std::vector<double> &values,
                    std::vector<double> stationary) {
                auto result = std::max(
                    std::abs(evaluatePolynomial(values, 0.0)),
                    std::abs(evaluatePolynomial(values, 1.0)));
                for (const auto root :
                        polynomialRootsInUnitInterval(std::move(stationary))) {
                    result = std::max(
                        result, std::abs(evaluatePolynomial(values, root)));
                }
                return result;
            }

            std::array<double, 5> componentCoefficients(
                    const AxisPolynomialSpan &span,
                    const double position_t::*component) {
                return {
                    span.coefficients[0].*component,
                    span.coefficients[1].*component,
                    span.coefficients[2].*component,
                    span.coefficients[3].*component,
                    span.coefficients[4].*component,
                };
            }
        }

        double maximumAxisVelocity(const AxisPolynomialSpan &span,
                const double position_t::*component) {
            const auto c = componentCoefficients(span, component);
            return maximumAbsolutePolynomial(
                {c[0], 2.0 * c[1], 3.0 * c[2],
                    4.0 * c[3], 5.0 * c[4]},
                {2.0 * c[1], 6.0 * c[2],
                    12.0 * c[3], 20.0 * c[4]})
                * span.inverseDuration;
        }

        double maximumAxisAcceleration(const AxisPolynomialSpan &span,
                const double position_t::*component) {
            const auto c = componentCoefficients(span, component);
            return maximumAbsolutePolynomial(
                {2.0 * c[1], 6.0 * c[2],
                    12.0 * c[3], 20.0 * c[4]},
                {6.0 * c[2], 24.0 * c[3], 60.0 * c[4]})
                * span.inverseDurationSquared;
        }

        double maximumAxisJerk(const AxisPolynomialSpan &span,
                const double position_t::*component) {
            const auto c = componentCoefficients(span, component);
            return maximumAbsolutePolynomial(
                {6.0 * c[2], 24.0 * c[3], 60.0 * c[4]},
                {24.0 * c[3], 120.0 * c[4]})
                * span.inverseDurationCubed;
        }

        double maximumPathAcceleration(const AxisPolynomialSpan &span) {
            return maximumLinearAcceleration(span);
        }

        double maximumPathJerk(const AxisPolynomialSpan &span) {
            return maximumLinearJerk(span);
        }

        double accelerationExcursionRatio(const AxisPolynomialSpan &span,
                const double servoPeriod, const TrajectoryLimits &limits) {
            const auto scale = span.inverseDurationSquared;
            const auto a0 = scaled(span.coefficients[1], 2.0 * scale);
            const auto a1 = scaled(span.coefficients[2], 6.0 * scale);
            const auto a2 = scaled(span.coefficients[3], 12.0 * scale);
            const auto a3 = scaled(span.coefficients[4], 20.0 * scale);
            const std::array controls {
                a0,
                add(a0, scaled(a1, 1.0 / 3.0)),
                add(add(a0, scaled(a1, 2.0 / 3.0)),
                    scaled(a2, 1.0 / 3.0)),
                add(add(a0, a1), add(a2, a3)),
            };

            return accelerationControlHullExcursionRatio(
                controls, servoPeriod, limits.pathJerk, limits.axisJerk);
        }
    }

    TrajectoryCompiler::TrajectoryCompiler(TrajectoryLimits limits) : m_limits(limits) { }

    void TrajectoryCompiler::reset(const EpochId epoch, const position_t &position) {
        m_epoch = epoch;
        m_nextChunk = 1;
        m_nextSpan = 1;
        m_nextExecutionMarker = 1;
        m_previousBranch = 0;
        m_position = position;
    }

    std::expected<PlanChunk, std::string> TrajectoryCompiler::compile(
            const MachineCommand &command, const PreparedPathPiece *preparedPiece) {
        m_lastTimeLawDiagnostics={};
        if(m_limits.pathAcceleration <= 0.0 || m_limits.pathJerk <= 0.0
           || m_limits.rapidSpeed <= 0.0 || m_limits.arcChordTolerance <= 0.0
           || !positiveAxisLimits(m_limits.axisVelocity)
           || !positiveAxisLimits(m_limits.axisAcceleration)
           || !positiveAxisLimits(m_limits.axisJerk))
            return std::unexpected("trajectory limits must be positive");
        if(std::holds_alternative<ProbeMove>(command))
            return std::unexpected("probe moves must be compiled as executor-owned triggered moves");

        PlanChunk chunk;
        chunk.epoch = m_epoch;
        chunk.id = m_nextChunk++;
        chunk.predecessorBranch = m_previousBranch;
        chunk.branch = chunk.id;
        TimeLawWorkspace timeLawWorkspace;
        TimeLawCompilationRecorder timeLawRecorder {m_lastTimeLawDiagnostics};
        CurveEvaluationWorkspace preparedWorkspace;
        if(preparedPiece && (!preparedPiece->curve||preparedPiece->length()<=1e-12))
            return std::unexpected("exact-stop prepared piece is invalid");

        const auto preparedSample = [&](const double distance) {
            const auto source = preparedPiece->curveFrom
                + std::clamp(distance, 0.0, preparedPiece->length());
            return PathSample{
                positionAtDistance(*preparedPiece->curve, source, preparedWorkspace),
                tangentAtDistance(*preparedPiece->curve, source, preparedWorkspace)};
        };
        const auto preparedMaximumTangent = [&] {
            position_t maximum{};
            if(!preparedPiece) return maximum;
            if(!preparedPiece->geometricSamples.empty()) {
                for(const auto &sample : preparedPiece->geometricSamples)
                    for(const auto component : AXIS_COMPONENTS)
                        maximum.*component = std::max(
                            maximum.*component, std::abs(sample.tangent.*component));
            } else {
                constexpr unsigned samples = 64;
                for(unsigned index = 0; index <= samples; ++index) {
                    const auto tangent = preparedSample(
                        preparedPiece->length() * index / static_cast<double>(samples)).tangent;
                    for(const auto component : AXIS_COMPONENTS)
                        maximum.*component = std::max(
                            maximum.*component, std::abs(tangent.*component));
                }
            }
            return maximum;
        };

        auto appendMotion = [&](const double length, const double speedPerMinute,
                                const position_t &maximumTangent,
                                const auto &sample, const auto &curvatureAt,
                                const bool curved,
                                const auto &verify) -> std::optional<std::string> {
            if(length <= 1e-12) {
                auto hold = hermite(m_nextSpan++, sample(0.0), sample(length), 0.0, 0.0, 1e-6);
                if(!chunk.normalMotion.push(hold)) return "trajectory chunk span capacity exceeded";
                return std::nullopt;
            }
            const auto requestedVelocity = std::min(speedPerMinute / 60.0,
                pathLimit(maximumTangent, m_limits.axisVelocity));
            if(requestedVelocity <= 0.0) return "motion speed must be positive";
            const auto acceleration = std::min(m_limits.pathAcceleration,
                pathLimit(maximumTangent, m_limits.axisAcceleration));
            const auto jerk = std::min(m_limits.pathJerk,
                pathLimit(maximumTangent, m_limits.axisJerk));
            const auto timing = timeLaw(timeLawWorkspace,timeLawRecorder.instrumentation,
                length,requestedVelocity,acceleration,jerk);
            if(!timing) return timing.error();
            const auto &boundaries = *timing;
            for(std::size_t phase = 1; phase < boundaries.size(); ++phase) {
                const auto &begin = boundaries[phase - 1];
                const auto &end = boundaries[phase];
                if(end.time - begin.time <= 1e-12) continue;
                const auto scalar = scalarPhase(begin, end);
                const auto emit = [&](const auto &self, const double u0, const double u1,
                                      const unsigned depth) -> std::optional<std::string> {
                    const auto from = scalar.at(u0);
                    const auto to = scalar.at(u1);
                    const auto duration = scalar.duration * (u1 - u0);
                    std::vector<AxisPolynomialSpan> spans;
                    if(curved) {
                        const auto state = [&](const TimeBoundary &boundary) {
                            const auto geometry = sample(boundary.distance);
                            const auto curvature = curvatureAt(boundary.distance);
                            return KinematicPathState{
                                .position = geometry.position,
                                .velocity = scaled(geometry.tangent, boundary.velocity),
                                .acceleration = add(
                                    scaled(geometry.tangent, boundary.acceleration),
                                    scaled(curvature, boundary.velocity * boundary.velocity)),
                            };
                        };
                        const auto chain = c2CubicChain(
                            m_nextSpan, state(from), state(to), duration);
                        spans.assign(chain.begin(), chain.end());
                    } else {
                        spans.push_back(hermite(m_nextSpan, sample(from.distance),
                            sample(to.distance), from.velocity, to.velocity, duration));
                    }
                    auto accepted = true;
                    for(std::size_t index = 0; index < spans.size(); ++index) {
                        const auto fraction0 = static_cast<double>(index) / spans.size();
                        const auto fraction1 = static_cast<double>(index + 1) / spans.size();
                        const auto proofFrom = scalar.at(std::lerp(u0, u1, fraction0));
                        const auto proofTo = scalar.at(std::lerp(u0, u1, fraction1));
                        if(!verify(spans[index], proofFrom.distance, proofTo.distance,
                                   proofFrom.velocity, proofTo.velocity)) {
                            accepted = false;
                            break;
                        }
                    }
                    if(!accepted) {
                        if(depth == 20) return "arc cubic tolerance verification did not converge";
                        const auto middle = std::midpoint(u0, u1);
                        if(auto result = self(self, u0, middle, depth + 1)) return result;
                        return self(self, middle, u1, depth + 1);
                    }
                    for(auto &span : spans) {
                        if(!chunk.normalMotion.push(span))
                            return "trajectory chunk span capacity exceeded";
                        ++m_nextSpan;
                    }
                    return std::nullopt;
                };
                if(auto result = emit(emit, 0.0, 1.0, 0)) return result;
            }

            double maximumAcceleration = 0.0;
            double maximumJerk = 0.0;
            auto axisScaleFactor = 1.0;
            for(const auto &span : chunk.normalMotion) {
                maximumAcceleration = std::max(maximumAcceleration, maximumLinearAcceleration(span));
                maximumJerk = std::max(maximumJerk, maximumLinearJerk(span));
                for(const auto component : AXIS_COMPONENTS) {
                    axisScaleFactor = std::max(axisScaleFactor,
                        trajectory_detail::maximumAxisVelocity(span, component)
                            / (m_limits.axisVelocity.*component));
                    axisScaleFactor = std::max(axisScaleFactor, std::sqrt(
                        trajectory_detail::maximumAxisAcceleration(span, component)
                            / (m_limits.axisAcceleration.*component)));
                    axisScaleFactor = std::max(axisScaleFactor, std::cbrt(
                        trajectory_detail::maximumAxisJerk(span, component)
                            / (m_limits.axisJerk.*component)));
                }
            }
            const auto scaleFactor = std::max({1.0, axisScaleFactor,
                std::sqrt(maximumAcceleration / m_limits.pathAcceleration),
                std::cbrt(maximumJerk / m_limits.pathJerk)});
            if(scaleFactor > 1.0 + 1e-9) {
                for(auto &span : chunk.normalMotion) {
                    span.duration *= scaleFactor;
                    span.inverseDuration /= scaleFactor;
                    span.inverseDurationSquared = span.inverseDuration * span.inverseDuration;
                    span.inverseDurationCubed = span.inverseDurationSquared * span.inverseDuration;
                }
            }
            return std::nullopt;
        };

        auto error = std::visit([&](const auto &value) -> std::optional<std::string> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, MoveLine>) {
                if(preparedPiece) {
                    const auto start = preparedSample(0.0).position;
                    const auto end = preparedSample(preparedPiece->length()).position;
                    if(subtract(start, value.from()).length() > 1e-8
                       || subtract(end, value.to()).length() > 1e-8)
                        return "exact-stop prepared line endpoints do not match its source entity";
                    const auto speed = value.speed() < 0.0 ? m_limits.rapidSpeed : value.speed();
                    const auto accept = [&](const AxisPolynomialSpan &span,
                                            const double from, const double to,
                                            double, double) {
                        if(preparedPiece->curve->geometricallyLinear) return true;
                        return verifiesOrderedCurveTolerance(span, from, to,
                            m_limits.arcChordTolerance,
                            [&](const double distance) {
                                return preparedSample(distance).position;
                            },
                            [&](const double a, const double b) {
                                return chordErrorBound(*preparedPiece->curve,
                                    preparedPiece->curveFrom + a,
                                    preparedPiece->curveFrom + b, preparedWorkspace);
                            });
                    };
                    const auto curvature = [&](const double distance) {
                        return curvatureAtDistance(*preparedPiece->curve,
                            preparedPiece->curveFrom + distance, preparedWorkspace);
                    };
                    if(auto result = appendMotion(preparedPiece->length(), speed,
                            preparedMaximumTangent(), preparedSample, curvature,
                            !preparedPiece->curve->geometricallyLinear, accept)) return result;
                    m_position = value.to();
                    return std::nullopt;
                }
                const auto source = value.from();
                const auto target = value.to();
                const auto delta = subtract(target, source);
                auto length = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z
                    + delta.a*delta.a + delta.b*delta.b + delta.c*delta.c);
                const auto tangent = length > 1e-12 ? scaled(delta, 1.0 / length) : position_t{};
                const auto sample = [&](const double distance) { return PathSample { add(source, scaled(tangent, distance)), tangent }; };
                const auto speed = value.speed() < 0.0 ? m_limits.rapidSpeed : value.speed();
                const auto accept = [](const AxisPolynomialSpan &, double, double, double, double) { return true; };
                const auto curvature = [](double) { return position_t{}; };
                if(auto result = appendMotion(length, speed, tangent, sample,
                        curvature, false, accept)) return result;
                m_position = target;
            } else if constexpr(std::same_as<T, ProbeMove>) {
                return "unreachable probe compilation path";
            } else if constexpr(std::same_as<T, MoveArc>) {
                if(preparedPiece) {
                    const auto start = preparedSample(0.0).position;
                    const auto end = preparedSample(preparedPiece->length()).position;
                    if(subtract(start, value.from()).length() > 1e-8
                       || subtract(end, value.to()).length() > 1e-8)
                        return "exact-stop prepared arc endpoints do not match its source entity";
                    auto arcSpeed = value.speed();
                    if(!std::isinf(m_limits.pathAcceleration)) {
                        auto maximumCurvature = 0.0;
                        if(!preparedPiece->geometricSamples.empty())
                            for(const auto &sample : preparedPiece->geometricSamples)
                                maximumCurvature = std::max(
                                    maximumCurvature, sample.curvature.length());
                        else for(unsigned index = 0; index <= 64; ++index)
                            maximumCurvature = std::max(maximumCurvature,
                                curvatureAtDistance(*preparedPiece->curve,
                                    preparedPiece->curveFrom + preparedPiece->length()
                                        * index / 64.0,
                                    preparedWorkspace).length());
                        if(maximumCurvature > 1e-15)
                            arcSpeed = std::min(arcSpeed, 60.0 * std::sqrt(
                                m_limits.pathAcceleration / maximumCurvature));
                    }
                    const auto accept = [&](const AxisPolynomialSpan &span,
                                            const double from, const double to,
                                            double, double) {
                        return verifiesOrderedCurveTolerance(span, from, to,
                            m_limits.arcChordTolerance,
                            [&](const double distance) {
                                return preparedSample(distance).position;
                            },
                            [&](const double a, const double b) {
                                return chordErrorBound(*preparedPiece->curve,
                                    preparedPiece->curveFrom + a,
                                    preparedPiece->curveFrom + b, preparedWorkspace);
                            });
                    };
                    const auto curvature = [&](const double distance) {
                        return curvatureAtDistance(*preparedPiece->curve,
                            preparedPiece->curveFrom + distance, preparedWorkspace);
                    };
                    if(auto result = appendMotion(preparedPiece->length(), arcSpeed,
                            preparedMaximumTangent(), preparedSample, curvature,
                            true, accept)) return result;
                    m_position = value.to();
                    return std::nullopt;
                }
                const simulation_detail::ArcReference reference(value);
                if(!reference.valid()) return value.axis().length() <= 1e-12
                    ? "arc axis must be nonzero" : "arc radius must be nonzero";
                const auto length = reference.length();
                const auto positionAt = [&](const double distance) { return reference.positionAtDistance(distance); };
                const auto sample = [&](const double distance) {
                    return PathSample { positionAt(distance), reference.tangentAtDistance(distance) };
                };
                position_t maximumTangent{};
                constexpr unsigned tangentIntervals = 256;
                for(unsigned index = 0; index <= tangentIntervals; ++index) {
                    const auto derivative = reference.derivative(
                        static_cast<double>(index) / tangentIntervals);
                    const auto magnitude = derivative.length();
                    const auto tangent = magnitude > 0.0 ? scaled(derivative, 1.0 / magnitude) : position_t{};
                    for(const auto component : AXIS_COMPONENTS)
                        maximumTangent.*component = std::max(maximumTangent.*component,
                            std::abs(tangent.*component));
                }
                auto arcSpeed = value.speed();
                if(!std::isinf(m_limits.pathAcceleration) && length > 1e-12) {
                    const auto curvature = reference.curvatureAccelerationBound();
                    if(curvature > 1e-15)
                        arcSpeed = std::min(arcSpeed, 60.0 * std::sqrt(m_limits.pathAcceleration / curvature));
                }
                const auto accept = [&](const AxisPolynomialSpan &span, const double distance0,
                                        const double distance1, const double velocity0, const double velocity1) {
                    return verifiesArcTolerance(span, distance0, distance1, velocity0, velocity1,
                        m_limits.arcChordTolerance, positionAt,
                        [&](const double from, const double to) { return reference.chordErrorBound(from, to); });
                };
                const auto curvature = [&](const double distance) {
                    return reference.curvatureAtDistance(distance);
                };
                if(auto result = appendMotion(length, arcSpeed, maximumTangent, sample,
                        curvature, true, accept)) return result;
                m_position = value.to();
            } else if constexpr(std::same_as<T, SpindleStart> || std::same_as<T, SpindleStop>) {
                const PathSample held { m_position, {} };
                if(!chunk.normalMotion.push(hermite(m_nextSpan++, held, held, 0.0, 0.0, 1e-6)))
                    return "trajectory chunk span capacity exceeded";
                SpindleEvent spindle;
                if constexpr(std::same_as<T, SpindleStart>) {
                    spindle = { true, value.direction(), value.speed() };
                }
                if(!chunk.events.push({ 0, spindle })) return "trajectory chunk event capacity exceeded";
            }
            return std::nullopt;
        }, command);
        if(error) return std::unexpected(std::move(*error));

        chunk.branchState =
            executionSpanEnd(chunk.normalMotion[chunk.normalMotion.size - 1]);
        chunk.branchState.velocity = {};
        chunk.branchState.acceleration = {};
        const PathSample held { chunk.branchState.position, {} };
        auto stop = hermite(m_nextSpan++, held, held, 0.0, 0.0, 1e-6);
        if(!chunk.stopTail.push(stop))
            return std::unexpected("trajectory chunk stop-tail capacity exceeded");
        chunk.stopState = chunk.branchState;
        if (!chunk.markers.push({
                .id = m_nextExecutionMarker++,
                .span = 0,
                .parameter = 0.0,
            })) {
            return std::unexpected(
                "trajectory chunk execution-marker capacity exceeded");
        }
        m_previousBranch = chunk.branch;
        return chunk;
    }

    std::expected<std::unique_ptr<ContinuousTrajectoryPlan>, std::string>
    TrajectoryCompiler::compileContinuous(
            const PreparedContinuousGeometry &geometry, const double blendScale,
            std::optional<MotionState> requestedStartState,
            std::optional<MotionState> requestedEndState,
            InfiniteJerkTrajectoryTimeResult *infiniteJerkTime) {
        m_lastTimeLawDiagnostics={};
        TimeLawCompilationRecorder timeLawRecorder {m_lastTimeLawDiagnostics};
        if(geometry.commands.empty()||geometry.pieces.empty())
            return std::unexpected("prepared continuous trajectory window is empty");
        if(blendScale<=0.0) return std::unexpected("continuous trajectory blend scale must be positive");
        if(m_limits.pathAcceleration<=0.0||m_limits.pathJerk<=0.0
           ||m_limits.arcChordTolerance<=0.0||!positiveAxisLimits(m_limits.axisVelocity)
           ||!positiveAxisLimits(m_limits.axisAcceleration)||!positiveAxisLimits(m_limits.axisJerk))
            return std::unexpected("trajectory limits must be positive");
        if(m_continuousPlanningEffort.maximumLocalCorrectionPasses==0
           ||m_continuousPlanningEffort.maximumLocalCorrectionPasses>128
           ||!std::isfinite(m_continuousPlanningEffort.quinticServoPeriod)
           ||m_continuousPlanningEffort.quinticServoPeriod<=0.0
           ||m_continuousPlanningEffort.quinticServoPeriod>1.0
           ||(m_continuousPlanningEffort.boundaryAccelerationMode
                  != ContinuousBoundaryAccelerationMode::Zero
              && m_continuousPlanningEffort.boundaryAccelerationMode
                  != ContinuousBoundaryAccelerationMode::Optimized))
            return std::unexpected("continuous planning effort is outside its bounded range");
        reportProgress();

        std::vector<GeometryPiece> pieces;
        auto timingPieceCount=std::size_t{0};
        for(const auto &prepared:geometry.pieces)
            timingPieceCount+=prepared.splineKnotIntervals.empty()
                ?std::size_t{1}:prepared.splineKnotIntervals.size();
        pieces.reserve(timingPieceCount);
        auto workspace=std::make_shared<CurveEvaluationWorkspace>();
        std::unordered_map<PreparedCommandId,std::size_t> inputById;
        inputById.reserve(geometry.commands.size());
        for(std::size_t input=0;input<geometry.commands.size();++input)
            if(!inputById.emplace(geometry.commands[input].id,input).second)
                return std::unexpected(std::format(
                    "prepared geometry contains duplicate command {}",
                    geometry.commands[input].id));
        const auto inputFor=[&](const PreparedCommandId id)
                ->std::expected<std::size_t,std::string> {
            const auto found=inputById.find(id);
            if(found==inputById.end())
                return std::unexpected(std::format(
                    "prepared piece references unknown command {}",id));
            return found->second;
        };
        for(const auto &prepared:geometry.pieces) {
            if(!prepared.curve||prepared.length()<=1e-12||prepared.programmedFeed<=0.0)
                return std::unexpected("prepared continuous path contains an invalid piece");
            if(prepared.geometricSamples.size()<2)
                return std::unexpected(std::format(
                    "prepared continuous piece {} has no usable geometric samples",
                    prepared.id));
            const auto primary=inputFor(prepared.primaryCommand);
            if(!primary) return std::unexpected(primary.error());
            auto firstSourceInput=*primary;
            auto lastSourceInput=*primary;
            auto sourceInputCount=std::size_t{0};
            for(const auto source:prepared.sourceCommands) {
                const auto input=inputFor(source);
                if(!input) return std::unexpected(input.error());
                if(sourceInputCount==0) firstSourceInput=*input;
                lastSourceInput=*input;
                ++sourceInputCount;
            }
            if(sourceInputCount==0) sourceInputCount=1;
            auto nextActivation=std::size_t{0};
            const auto appendTimingPiece=[&](
                    const double from,const double to,const double programmedVelocity,
                    const double staticVelocityLimit,const std::size_t knotInterval,
                    const std::span<const PreparedGeometricSample> samples)
                    ->std::expected<void,std::string> {
                const auto length=to-from;
                const auto sampleOffset=from-prepared.curveFrom;
                if(!std::isfinite(length)||length<=1e-12
                   ||!std::isfinite(programmedVelocity)||programmedVelocity<=0.0
                   ||std::isnan(staticVelocityLimit)||staticVelocityLimit<=0.0)
                    return std::unexpected(std::format(
                        "prepared continuous piece {} has an invalid timing interval",
                        prepared.id));
                if(samples.size()<2)
                    return std::unexpected(std::format(
                        "prepared continuous piece {} timing interval has no geometric samples",
                        prepared.id));
                const auto distanceTolerance=std::max(1e-10,prepared.length()*1e-10);
                if(!std::isfinite(samples.front().distance)
                   ||!std::isfinite(samples.back().distance)
                   ||std::abs(samples.front().distance-sampleOffset)>distanceTolerance
                   ||std::abs(samples.back().distance-(sampleOffset+length))>distanceTolerance)
                    return std::unexpected(std::format(
                        "prepared continuous piece {} timing samples do not match their interval",
                        prepared.id));
                for(std::size_t sample=1;sample<samples.size();++sample)
                    if(!std::isfinite(samples[sample].distance)
                       ||samples[sample].distance<=samples[sample-1].distance)
                        return std::unexpected(std::format(
                            "prepared continuous piece {} timing samples are not ordered",
                            prepared.id));

                std::vector<GeometryActivation> activations;
                const auto finalInterval=std::abs(to-prepared.curveTo)<=distanceTolerance;
                while(nextActivation<prepared.activationStations.size()) {
                    const auto &station=prepared.activationStations[nextActivation];
                    if(!std::isfinite(station.curveDistance)
                       ||station.curveDistance<from-distanceTolerance)
                        return std::unexpected(std::format(
                            "prepared continuous piece {} has an unordered activation station",
                            prepared.id));
                    if(station.curveDistance>=to-distanceTolerance
                       &&!(finalInterval&&station.curveDistance<=to+distanceTolerance)) break;
                    const auto input=inputFor(station.command);
                    if(!input) return std::unexpected(input.error());
                    activations.push_back({*input,
                        std::clamp(station.curveDistance-from,0.0,length)});
                    ++nextActivation;
                }

                const auto curve=prepared.curve;
                pieces.push_back({
                    .input=*primary,
                    .preparedPiece=prepared.id,
                    .preparedKind=prepared.kind,
                    .knotInterval=knotInterval,
                    .firstSourceInput=firstSourceInput,
                    .lastSourceInput=lastSourceInput,
                    .sourceInputCount=sourceInputCount,
                    .activations=std::move(activations),
                    .length=length,
                    .curveFrom=from,
                    .curveTo=to,
                    .programmedVelocity=programmedVelocity,
                    .staticVelocityLimit=staticVelocityLimit,
                    .linear=prepared.curve->geometricallyLinear,
                    .geometricSamples=samples,
                    .geometricSampleDistanceOffset=sampleOffset,
                    .sampleAt=[curve,workspace,from,length](const double distance) {
                        const auto source=from+std::clamp(distance,0.0,length);
                        return PathSample{positionAtDistance(*curve,source,*workspace),
                            tangentAtDistance(*curve,source,*workspace)};
                    },
                    .curvatureAt=[curve,workspace,from,length](const double distance) {
                        return curvatureAtDistance(*curve,
                            from+std::clamp(distance,0.0,length),*workspace);
                    },
                    .infiniteJerkCurvatureAt=infiniteJerkTime
                        ?std::function<position_t(double)>{[curve,workspace,from,length](
                                const double distance) {
                            return curvatureAtDistance(*curve,
                                from+std::clamp(distance,0.0,length),*workspace);
                        }}:std::function<position_t(double)>{},
                    .curvatureDerivativeAt=[curve,workspace,from,length,knotInterval](
                            const double distance) {
                        const auto source=from+std::clamp(distance,0.0,length);
                        if(knotInterval != std::numeric_limits<std::size_t>::max())
                            return curvatureDerivativeAtDistance(
                                *curve,source,*workspace,knotInterval);
                        return curvatureDerivativeAtDistance(*curve,source,*workspace);
                    },
                    .positionAt=[curve,workspace,from,length](const double distance) {
                        return positionAtDistance(*curve,
                            from+std::clamp(distance,0.0,length),*workspace);
                    },
                    .chordErrorBound=[curve,workspace,from,length](const double a,const double b) {
                        return chordErrorBound(*curve,from+std::clamp(a,0.0,length),
                            from+std::clamp(b,0.0,length),*workspace);
                    },
                });
                return {};
            };

            if(prepared.splineKnotIntervals.empty()) {
                if(auto appended=appendTimingPiece(prepared.curveFrom,prepared.curveTo,
                        prepared.programmedFeed,std::numeric_limits<double>::infinity(),
                        std::numeric_limits<std::size_t>::max(),prepared.geometricSamples);
                        !appended)
                    return std::unexpected(appended.error());
                if(nextActivation!=prepared.activationStations.size())
                    return std::unexpected(std::format(
                        "prepared continuous piece {} has activation stations outside its interval",
                        prepared.id));
                continue;
            }
            const auto expectedSampleCount = (PREPARED_CURVE_SAMPLE_INTERVALS + 1)
                * prepared.splineKnotIntervals.size();
            if(prepared.geometricSamples.size()!=expectedSampleCount)
                return std::unexpected(std::format(
                    "prepared spline {} has {} geometric samples; expected {}",
                    prepared.id,prepared.geometricSamples.size(),expectedSampleCount));
            const auto *spline = std::get_if<PreparedSplineCurve>(&prepared.curve->value);
            if(!spline || spline->controls.size() <= spline->degree)
                return std::unexpected(std::format(
                    "prepared piece {} has knot intervals but is not a spline",prepared.id));
            const auto parameterSpanCount = spline->controls.size() - spline->degree;
            auto expectedFrom=prepared.curveFrom;
            for(std::size_t intervalIndex=0;
                    intervalIndex<prepared.splineKnotIntervals.size();++intervalIndex) {
                const auto &interval=prepared.splineKnotIntervals[intervalIndex];
                if(std::abs(interval.curveFrom-expectedFrom)>1e-10
                   ||interval.curveTo<=interval.curveFrom)
                    return std::unexpected(std::format(
                        "prepared spline {} knot interval {} is not contiguous",
                        prepared.id,intervalIndex));
                if(interval.geometricSampleCount!=PREPARED_CURVE_SAMPLE_INTERVALS+1
                   ||interval.firstGeometricSample>prepared.geometricSamples.size()
                   ||interval.geometricSampleCount
                        >prepared.geometricSamples.size()-interval.firstGeometricSample)
                    return std::unexpected(std::format(
                        "prepared spline {} knot interval {} does not provide {} samples",
                        prepared.id,intervalIndex,
                        PREPARED_CURVE_SAMPLE_INTERVALS+1));
                if(interval.parameterSpan >= parameterSpanCount)
                    return std::unexpected(std::format(
                        "prepared spline {} knot interval {} has invalid parameter span {}",
                        prepared.id,intervalIndex,interval.parameterSpan));
                const auto samples=std::span{prepared.geometricSamples}.subspan(
                    interval.firstGeometricSample,interval.geometricSampleCount);
                if(auto appended=appendTimingPiece(interval.curveFrom,interval.curveTo,
                        interval.programmedFeed,interval.geometricVelocityLimit,
                        interval.parameterSpan,samples); !appended)
                    return std::unexpected(appended.error());
                expectedFrom=interval.curveTo;
            }
            if(std::abs(expectedFrom-prepared.curveTo)>1e-10)
                return std::unexpected(std::format(
                    "prepared spline {} knot intervals do not cover the curve",
                    prepared.id));
            if(nextActivation!=prepared.activationStations.size())
                return std::unexpected(std::format(
                    "prepared spline {} has activation stations outside its intervals",
                    prepared.id));
        }
        if(pieces.empty()) return std::unexpected("continuous path produced no geometry");

        CurveEvaluationWorkspace endpointWorkspace;
        const auto &last=geometry.pieces.back();
        const auto expectedEnd=positionAtDistance(
            *last.curve,last.curveTo,endpointWorkspace);
        const auto startState=requestedStartState.value_or(MotionState{m_position,{},{}});
        const auto endState=requestedEndState.value_or(MotionState{expectedEnd,{},{}});
        const auto scalarBoundary=[&](const GeometryPiece &piece,const double distance,
                                      const MotionState &state,const std::string_view name)
                -> std::expected<std::pair<double,double>,std::string> {
            const auto sample=piece.sampleAt(distance);
            const auto curvature=piece.curvatureAt(distance);
            const auto positionError=subtract(state.position,sample.position).length();
            const auto velocity=positionDot(state.velocity,sample.tangent);
            const auto velocityError=subtract(state.velocity,scaled(sample.tangent,velocity)).length();
            const auto tangentialAcceleration=subtract(
                state.acceleration,scaled(curvature,velocity*velocity));
            const auto acceleration=positionDot(tangentialAcceleration,sample.tangent);
            const auto accelerationError=subtract(
                tangentialAcceleration,scaled(sample.tangent,acceleration)).length();
            if(positionError>1e-8||velocity<-1e-10||velocityError>1e-7
               ||accelerationError>1e-7)
                return std::unexpected(std::format(
                    "continuous {} boundary is not a forward path PVA state: position error={} "
                    "velocity={} transverse velocity={} scalar acceleration={} transverse "
                    "acceleration={}",name,positionError,velocity,velocityError,acceleration,
                    accelerationError));
            return std::pair{std::max(0.0,velocity),acceleration};
        };
        auto scalarStart=scalarBoundary(pieces.front(),0.0,startState,"start");
        if(!scalarStart) return std::unexpected(scalarStart.error());
        auto scalarEnd=scalarBoundary(pieces.back(),pieces.back().length,endState,"end");
        if(!scalarEnd) return std::unexpected(scalarEnd.error());
        if(infiniteJerkTime) {
            std::vector<InfiniteJerkPathPiece> infiniteJerkPieces;
            infiniteJerkPieces.reserve(pieces.size());
            for(const auto &piece:pieces) infiniteJerkPieces.push_back({
                .length=piece.length,
                .velocityLimit=std::min(
                    piece.programmedVelocity,piece.staticVelocityLimit),
                .tangentAt=[sample=piece.sampleAt](const double distance) {
                    return sample(distance).tangent;
                },
                .curvatureAt=piece.infiniteJerkCurvatureAt
                    ?piece.infiniteJerkCurvatureAt:piece.curvatureAt,
            });
            auto measured=infiniteJerkTrajectoryTime(infiniteJerkPieces,{
                .pathAcceleration=m_limits.pathAcceleration,
                .axisVelocity=m_limits.axisVelocity,
                .axisAcceleration=m_limits.axisAcceleration,
            },scalarStart->first,scalarEnd->first);
            if(!measured) return std::unexpected(std::format(
                "smoothed-path infinite-jerk timing failed: {}",measured.error()));
            *infiniteJerkTime=*measured;
        }

        auto result=std::make_unique<ContinuousTrajectoryPlan>();
        const auto planningPathAcceleration=m_limits.pathAcceleration;
        const auto planningPathJerk=m_limits.pathJerk;
        const auto planningAxisAcceleration=m_limits.axisAcceleration;
        const auto planningAxisJerk=m_limits.axisJerk;
        std::vector<AxisPolynomialSpan> normalSpans;
        auto nextChunk=m_nextChunk;
        auto nextSpan=m_nextSpan;
        struct ResolvedActivation {
            SpanId span = 0;
            double parameter = 0.0;
        };
        std::vector<ResolvedActivation> resolvedActivations(
            geometry.commands.size());

        std::vector<path_tempo::InitialPieceLimits> effectiveLimits(pieces.size());

        const auto kinematicAt=[&](const std::size_t pieceIndex,const TimeBoundary &boundary) {
            const auto &piece=pieces[pieceIndex];
            auto local=std::clamp(boundary.distance,0.0,piece.length);
            if(local<1e-10) local=0.0;
            else if(piece.length-local<1e-10) local=piece.length;
            const auto sample=piece.sampleAt(local);
            const auto curvature = piece.linear
                ? position_t{} : piece.curvatureAt(local);
            return KinematicPathState {
                .position=sample.position,
                .velocity=scaled(sample.tangent,boundary.velocity),
                .acceleration=add(scaled(sample.tangent,boundary.acceleration),
                                  scaled(curvature,boundary.velocity*boundary.velocity)),
            };
        };

        const auto maximumLocalCorrectionPasses=
            m_continuousPlanningEffort.maximumLocalCorrectionPasses;
        bool constraintsVerified=false;
        std::string correctionHistory;
        std::vector<path_tempo::SampledPathPiece<6>> pathTempoStorage;
        pathTempoStorage.reserve(pieces.size());
        for (std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
            const auto &piece=pieces[pieceIndex];
            path_tempo::SampledPathPiece<6> timingPiece {
                .id=pieceIndex+1,
                .length=piece.length,
                .maxVelocity=piece.programmedVelocity,
                .initialLimits={.velocity=piece.staticVelocityLimit},
                .stations={},
            };
            timingPiece.stations.reserve(piece.geometricSamples.size());
            for (std::size_t sampleIndex=0;sampleIndex<piece.geometricSamples.size();
                    ++sampleIndex) {
                const auto &sample=piece.geometricSamples[sampleIndex];
                const auto distance=sampleIndex==0?0.0
                    :sampleIndex+1==piece.geometricSamples.size()?piece.length
                    :sample.distance-piece.geometricSampleDistanceOffset;
                timingPiece.stations.push_back({
                    .distance=distance,
                    .tangent=pathTempoVector(sample.tangent),
                    .curvature=pathTempoVector(sample.curvature),
                    .thirdDerivative=pathTempoVector(sample.curvatureDerivative),
                });
            }
            pathTempoStorage.push_back(std::move(timingPiece));
        }

        std::vector<path_tempo::PathPiece<6>> pathTempoPieces;
        pathTempoPieces.reserve(pathTempoStorage.size());
        for (const auto &piece:pathTempoStorage)
            pathTempoPieces.push_back(piece.view());

        const path_tempo::PathPlanningRequest<6> pathTempoRequest {
            .pieces = pathTempoPieces,
            .beginning = {
                .velocity = scalarStart->first,
                .acceleration = scalarStart->second,
            },
            .ending = {
                .velocity = scalarEnd->first,
                .acceleration = scalarEnd->second,
            },
            .limits = {
                .pathAcceleration = planningPathAcceleration,
                .pathJerk = planningPathJerk,
                .coordinateVelocity = pathTempoVector(m_limits.axisVelocity),
                .coordinateAcceleration = pathTempoVector(planningAxisAcceleration),
                .coordinateJerk = pathTempoVector(planningAxisJerk),
            },
            .settings = {
                .maximumCorrectionPasses = maximumLocalCorrectionPasses,
                .applySampledCorrections = true,
                .boundaryAccelerationMode =
                    m_continuousPlanningEffort.boundaryAccelerationMode
                            == ContinuousBoundaryAccelerationMode::Zero
                        ? path_tempo::BoundaryAccelerationMode::Zero
                        : path_tempo::BoundaryAccelerationMode::Optimized,
            },
        };
        auto materializationPass = 0U;
        const path_tempo::MaterializationCorrection materializationCorrection =
            [&](const path_tempo::PlannedPath &candidate)
                -> std::expected<std::vector<path_tempo::PieceCorrection>, std::string> {
            const auto correctionPass = materializationPass++;
            ++result->materialization.callbackPasses;
            result->materialization.candidatePieces+=pieces.size();
            const auto conversionStarted=std::chrono::steady_clock::now();
            reportProgress();
            const auto *planned = &candidate;
            if (planned->pieceBoundaries.size()!=pieces.size()+1
               ||planned->pieceLimits.size()!=pieces.size()) {
                return std::unexpected(
                    "PathTempo continuous timing returned inconsistent piece data");
            }

            std::vector<double> stationVelocity;
            std::vector<double> stationAcceleration;
            stationVelocity.reserve(planned->pieceBoundaries.size());
            stationAcceleration.reserve(planned->pieceBoundaries.size());
            for (const auto &boundary:planned->pieceBoundaries) {
                stationVelocity.push_back(boundary.velocity);
                stationAcceleration.push_back(boundary.acceleration);
            }
            auto convertedTiming=localTimeLaws(*planned,pieces);
            if (!convertedTiming) {
                return std::unexpected(convertedTiming.error());
            }
            auto pieceTiming=std::move(*convertedTiming);

            effectiveLimits=planned->pieceLimits;
            result->materialization.candidateConversionSeconds+=
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-conversionStarted).count();
            result->correctionPasses =
                static_cast<unsigned>(planned->diagnostics.correctionPasses);

            std::vector<double> correction(pieces.size(), 1.0);
            std::vector<double> quinticCorrection(pieces.size(), 1.0);
            {
                // Construct the production degree-aware normal-motion sequence.
                auto &quintic = result->materialization.quintic;
                auto correctionPasses = std::move(quintic.correctionPasses);
                quintic = {};
                quintic.correctionPasses = std::move(correctionPasses);
                quintic.ran = true;
                const auto quinticStarted = std::chrono::steady_clock::now();
                const auto requiredTimeScale = [](
                        const ContinuousPolynomialConstraintKind constraint,
                        const double ratio) {
                    switch (constraint) {
                        case ContinuousPolynomialConstraintKind::PathVelocity:
                        case ContinuousPolynomialConstraintKind::AxisVelocity:
                            return ratio;
                        case ContinuousPolynomialConstraintKind::PathAcceleration:
                        case ContinuousPolynomialConstraintKind::AxisAcceleration:
                            return std::sqrt(ratio);
                        case ContinuousPolynomialConstraintKind::PathJerk:
                        case ContinuousPolynomialConstraintKind::AxisJerk:
                            return std::cbrt(ratio);
                    }
                    return ratio;
                };
                const auto maximumQuinticSpans =
                    std::max<std::size_t>(8192,pieces.size() * 32);
                const auto maximumQuinticProofs =
                    std::max<std::size_t>(32768,pieces.size() * 64);
                std::vector<bool> shadowActivationOwned(
                    geometry.commands.size(), false);
                auto shadowGlobalTime = 0.0;
                const auto assignShadowActivations = [&](
                        const GeometryPiece &piece, const std::size_t pieceIndex,
                        const std::size_t firstSpan)
                        -> std::expected<void, std::string> {
                    const auto finalSpan = quintic.shadowSpans.size();
                    const auto tolerance = std::max(1e-12, piece.length * 1e-10);
                    const auto &boundaries = pieceTiming[pieceIndex];
                    const auto pieceDuration = boundaries.back().time;
                    const auto distanceAtTime = [&](const double requested) {
                        if (requested <= 0.0) {
                            return boundaries.front().distance;
                        }
                        if (requested >= pieceDuration) {
                            return boundaries.back().distance;
                        }
                        const auto upper = std::ranges::upper_bound(
                            boundaries, requested, {}, &TimeBoundary::time);
                        const auto phase = std::clamp<std::size_t>(
                            upper - boundaries.begin(), 1,
                            boundaries.size() - 1);
                        const auto scalar = localScalarPhase(
                            boundaries[phase - 1], boundaries[phase]);
                        return scalar.at(std::clamp(
                            (requested - boundaries[phase - 1].time)
                                / scalar.duration,
                            0.0, 1.0)).distance;
                    };
                    for (const auto &activation : piece.activations) {
                        if (activation.input >= shadowActivationOwned.size()) {
                            return std::unexpected(
                                "shadow quintic activation input is out of range");
                        }
                        if (shadowActivationOwned[activation.input]) {
                            continue;
                        }
                        auto owner = finalSpan;
                        for (auto span = firstSpan; span < finalSpan; ++span) {
                            const auto &candidate = quintic.shadowSpans[span];
                            const auto isFinal = span + 1 == finalSpan;
                            if (activation.distance
                                    < candidate.localDistanceTo - tolerance
                               || (isFinal && activation.distance
                                    <= candidate.localDistanceTo + tolerance)) {
                                owner = span;
                                break;
                            }
                        }
                        if (owner == finalSpan) {
                            return std::unexpected(std::format(
                                "shadow quintic sequence has no activation owner "
                                "for input {} on timing piece {}",
                                activation.input,
                                quintic.shadowSpans.empty()
                                    ? 0 : quintic.shadowSpans.back().timingPiece));
                        }
                        auto low = 0.0;
                        auto high = pieceDuration;
                        for (unsigned iteration = 0; iteration < 64; ++iteration) {
                            const auto middle = std::midpoint(low, high);
                            if (distanceAtTime(middle) < activation.distance) {
                                low = middle;
                            } else {
                                high = middle;
                            }
                        }
                        const auto activationTime =
                            activation.distance <= 0.0 ? 0.0
                            : activation.distance >= piece.length ? pieceDuration
                            : std::midpoint(low, high);
                        const auto &ownerSpan = quintic.shadowSpans[owner];
                        shadowActivationOwned[activation.input] = true;
                        quintic.shadowActivations.push_back({
                            .input = activation.input,
                            .span = owner,
                            .globalTime = shadowGlobalTime + activationTime,
                            .localDistance = activation.distance,
                            .parameter = std::clamp(
                                (activationTime - ownerSpan.pieceTimeFrom)
                                    * ownerSpan.inverseDuration,
                                0.0, 1.0),
                        });
                    }

                    return {};
                };
                for (std::size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex) {
                    const auto &piece = pieces[pieceIndex];
                    const auto &boundaries = pieceTiming[pieceIndex];
                    const auto pieceDuration = boundaries.back().time;
                    const auto stateAtTime = [&](const double requested) {
                        if (requested <= 0.0) {
                            return boundaries.front();
                        }
                        if (requested >= pieceDuration) {
                            return boundaries.back();
                        }
                        const auto upper = std::ranges::upper_bound(
                            boundaries, requested, {}, &TimeBoundary::time);
                        const auto phase = std::clamp<std::size_t>(
                            upper - boundaries.begin(), 1,
                            boundaries.size() - 1);
                        const auto scalar = localScalarPhase(
                            boundaries[phase - 1], boundaries[phase]);
                        auto result = scalar.at(std::clamp(
                            (requested - boundaries[phase - 1].time)
                                / scalar.duration,
                            0.0, 1.0));
                        result.time = requested;
                        return result;
                    };
                    const auto firstShadowSpan = quintic.shadowSpans.size();
                    struct QuinticCandidate {
                        bool accepted = false;
                        bool constraintsVerified = false;
                        bool geometryEvaluated = false;
                        bool geometryVerified = false;
                        bool progressVerified = false;
                        double maximumRatio = 0.0;
                        double maximumCorrectionRatio = 0.0;
                        double acceptanceRatio = 0.0;
                        double accelerationExcursionRatio = 0.0;
                        bool subServoJerkAccepted = false;
                        ContinuousPolynomialConstraintKind constraint =
                            ContinuousPolynomialConstraintKind::PathAcceleration;
                        std::size_t axis = std::numeric_limits<std::size_t>::max();
                    };
                    const auto pathVelocityLimit =
                        std::min(piece.programmedVelocity, piece.staticVelocityLimit);
                    const auto proveGeometry = [&](QuinticCandidate &candidate,
                                                   const LocalQuinticBezier &candidateQuintic,
                                                   const TimeBoundary &from,
                                                   const TimeBoundary &to) {
                        if (quintic.geometryProofs >= maximumQuinticProofs) {
                            quintic.resourceExhausted = true;
                            return;
                        }
                        const auto localFrom =
                            std::clamp(from.distance, 0.0, piece.length);
                        const auto localTo =
                            std::clamp(to.distance, 0.0, piece.length);
                        const auto absoluteControls =
                            absoluteBezierControls(candidateQuintic);
                        ++quintic.geometryProofs;
                        const auto geometryAndProgressVerified =
                            verifiesOrderedCurveTolerance(
                                absoluteControls, localFrom, localTo,
                                m_limits.arcChordTolerance,
                                piece.positionAt, piece.chordErrorBound, true);
                        auto geometryVerified = geometryAndProgressVerified;
                        if (!geometryAndProgressVerified) {
                            ++quintic.geometryProofs;
                            geometryVerified = verifiesOrderedCurveTolerance(
                                absoluteControls, localFrom, localTo,
                                m_limits.arcChordTolerance,
                                piece.positionAt, piece.chordErrorBound);
                            if (geometryVerified) {
                                ++quintic.forwardProgressFailures;
                            }
                        }
                        candidate.geometryEvaluated = true;
                        candidate.geometryVerified = geometryVerified;
                        candidate.progressVerified =
                            geometryAndProgressVerified;
                        if (!geometryVerified) {
                            ++quintic.geometryRefinements;
                        }
                        candidate.accepted =
                            candidate.constraintsVerified
                            && geometryAndProgressVerified;
                    };
                    const auto evaluateStates = [&](const TimeBoundary &from,
                                                    const TimeBoundary &to,
                                                    LocalQuinticBezier *retainedQuintic) {
                        QuinticCandidate candidate;
                        ++quintic.candidateEvaluations;
                        const auto duration = to.time - from.time;
                        if (duration <= 1e-12) {
                            ++quintic.numericallyUnrefinableIntervals;
                            return candidate;
                        }
                        const auto candidateQuintic = localQuinticBezier(
                            kinematicAt(pieceIndex, from), kinematicAt(pieceIndex, to), duration);
                        if (retainedQuintic) {
                            *retainedQuintic = candidateQuintic;
                        }
                        const auto bounds = quinticConstraintBounds(
                            candidateQuintic.controls, duration,
                            pathVelocityLimit, m_limits, quintic.constraintBoundNodes);
                        candidate.maximumRatio = bounds.maximumRatio;
                        candidate.maximumCorrectionRatio =
                            bounds.maximumCorrectionRatio;
                        candidate.constraint = bounds.constraint;
                        candidate.axis = bounds.axis;
                        const auto servoPeriod =
                            m_continuousPlanningEffort.quinticServoPeriod;
                        if (duration < servoPeriod
                           && bounds.maximumJerkRatio
                                > trajectory_detail::DYNAMIC_LIMIT_RATIO
                                    + trajectory_detail::
                                        POLYNOMIAL_RATIO_TOLERANCE) {
                            candidate.accelerationExcursionRatio =
                                quinticAccelerationExcursionRatio(
                                    candidateQuintic.controls, duration,
                                    servoPeriod, m_limits);
                        }
                        const auto classification =
                            trajectory_detail::classifyQuinticConstraints(
                                duration, bounds.maximumVelocityRatio,
                                bounds.maximumAccelerationRatio,
                                bounds.maximumJerkRatio,
                                candidate.accelerationExcursionRatio,
                                servoPeriod);
                        candidate.subServoJerkAccepted =
                            classification.subServoJerkAccepted;
                        candidate.constraintsVerified =
                            classification.constraintsVerified;
                        candidate.maximumCorrectionRatio =
                            classification.maximumFailedCorrectionRatio;
                        switch (classification.correctionCategory) {
                            case trajectory_detail::QuinticConstraintCategory::None:
                                break;
                            case trajectory_detail::QuinticConstraintCategory::Velocity:
                                candidate.constraint =
                                    bounds.velocityConstraint;
                                candidate.axis = bounds.velocityAxis;
                                break;
                            case trajectory_detail::QuinticConstraintCategory::Acceleration:
                                candidate.constraint =
                                    bounds.accelerationConstraint;
                                candidate.axis = bounds.accelerationAxis;
                                break;
                            case trajectory_detail::QuinticConstraintCategory::Jerk:
                                candidate.constraint = bounds.jerkConstraint;
                                candidate.axis = bounds.jerkAxis;
                                break;
                        }
                        candidate.acceptanceRatio = std::max({
                            bounds.maximumVelocityRatio,
                            bounds.maximumAccelerationRatio
                                / trajectory_detail::DYNAMIC_LIMIT_RATIO,
                            candidate.subServoJerkAccepted
                                ? candidate.accelerationExcursionRatio
                                    / trajectory_detail::DYNAMIC_LIMIT_RATIO
                                : bounds.maximumJerkRatio
                                    / trajectory_detail::DYNAMIC_LIMIT_RATIO});
                        if (!candidate.constraintsVerified) {
                            ++quintic.constraintRefinements;

                            return candidate;
                        }

                        proveGeometry(candidate, candidateQuintic, from, to);

                        return candidate;
                    };

                    const auto totalDuration = pieceDuration;
                    const auto stateAt = [&](const double u) {
                        return stateAtTime(totalDuration * u);
                    };

                    LocalQuinticBezier initialQuintic;
                    const auto initial = evaluateStates(
                        boundaries.front(), boundaries.back(), &initialQuintic);
                    ++quintic.initialSpans;
                    ++quintic.initialWorstRatioHistogram[
                        polynomialSeverityBin(initial.maximumRatio)];
                    if (!initial.constraintsVerified) {
                        ++quintic.initialViolatingSpans;
                    }
                    if (initial.maximumRatio > quintic.maximumInitialRatio) {
                        quintic.maximumInitialRatio = initial.maximumRatio;
                        quintic.worstInitialDuration = totalDuration;
                        quintic.worstInitialTimingPiece = pieceIndex;
                        quintic.worstInitialPreparedPiece = piece.preparedPiece;
                        quintic.worstInitialInput = piece.input;
                        quintic.worstInitialConstraint = initial.constraint;
                        quintic.worstInitialAxis = initial.axis;
                    }

                    struct AdaptiveLeaf {
                        double from = 0.0;
                        double to = 0.0;
                        bool accepted = false;
                        QuinticCandidate candidate;
                        LocalQuinticBezier quintic;
                    };
                    std::vector<AdaptiveLeaf> leaves;
                    const auto refine = [&](const auto &self, const double u0,
                                            const double u1, const unsigned depth,
                                            const QuinticCandidate *retainedCandidate,
                                            const LocalQuinticBezier *retainedQuintic) -> void {
                        if (quintic.resourceExhausted) {
                            return;
                        }
                        const auto from = stateAt(u0);
                        const auto to = stateAt(u1);
                        LocalQuinticBezier candidateQuintic;
                        QuinticCandidate candidate;
                        if (retainedCandidate && retainedQuintic) {
                            candidate = *retainedCandidate;
                            candidateQuintic = *retainedQuintic;
                        } else {
                            candidate =
                                evaluateStates(from, to, &candidateQuintic);
                        }
                        if (candidate.accepted) {
                            leaves.push_back({
                                u0, u1, true, candidate, candidateQuintic});
                            quintic.maximumDepth = std::max(quintic.maximumDepth, depth);
                            return;
                        }
                        if (depth >= 20) {
                            leaves.push_back({
                                u0, u1, false, candidate, candidateQuintic});
                            ++quintic.numericallyUnrefinableIntervals;
                            quintic.maximumDepth = std::max(quintic.maximumDepth, depth);
                            return;
                        }
                        auto split = std::midpoint(u0, u1);
                        // Keep estimated endpoint-resolution error below one percent
                        // of every jerk limit. This is specific to the local quintic;
                        // the cubic-chain duration guard has a different error model.
                        constexpr auto MAXIMUM_QUINTIC_ROUNDING_RATIO = 0.01;
                        const auto kinematicFrom = kinematicAt(pieceIndex, from);
                        const auto kinematicTo = kinematicAt(pieceIndex, to);
                        const auto stableSplit = [&](const double candidateSplit) {
                            if (candidateSplit <= u0 || candidateSplit >= u1) {
                                return false;
                            }
                            const auto middle =
                                kinematicAt(pieceIndex, stateAt(candidateSplit));
                            const auto leftDuration =
                                totalDuration * (candidateSplit - u0);
                            const auto rightDuration =
                                totalDuration * (u1 - candidateSplit);
                            const auto left = localQuinticBezier(
                                kinematicFrom, middle, leftDuration);
                            const auto right = localQuinticBezier(
                                middle, kinematicTo, rightDuration);
                            return quinticEndpointResolutionJerkRatio(
                                       left, leftDuration, m_limits)
                                    <= MAXIMUM_QUINTIC_ROUNDING_RATIO
                                && quinticEndpointResolutionJerkRatio(
                                       right, rightDuration, m_limits)
                                    <= MAXIMUM_QUINTIC_ROUNDING_RATIO;
                        };
                        std::optional<double> phaseSplit;
                        const auto fromTime = totalDuration * u0;
                        const auto toTime = totalDuration * u1;
                        const auto targetTime = totalDuration * split;
                        const auto phaseCandidate = std::ranges::lower_bound(
                            boundaries, targetTime, {}, &TimeBoundary::time);
                        const auto considerBoundary = [&](const auto iterator) {
                            if (iterator == boundaries.end()
                               || iterator->time <= fromTime
                               || iterator->time >= toTime) {
                                return;
                            }
                            const auto candidateSplit = iterator->time / totalDuration;
                            if (!stableSplit(candidateSplit)) {
                                return;
                            }
                            if (!phaseSplit
                               || std::abs(candidateSplit - split)
                                    < std::abs(*phaseSplit - split)) {
                                phaseSplit = candidateSplit;
                            }
                        };
                        considerBoundary(phaseCandidate);
                        if (phaseCandidate != boundaries.begin()) {
                            considerBoundary(phaseCandidate - 1);
                        }
                        if (phaseSplit) {
                            split = *phaseSplit;
                        } else if (!stableSplit(split)) {
                            leaves.push_back({
                                u0, u1, false, candidate, candidateQuintic});
                            ++quintic.numericallyUnrefinableIntervals;
                            quintic.maximumDepth = std::max(quintic.maximumDepth, depth);
                            return;
                        }
                        ++quintic.subdivisions;
                        self(self, u0, split, depth + 1, nullptr, nullptr);
                        self(self, split, u1, depth + 1, nullptr, nullptr);
                    };
                    if (initial.accepted) {
                        leaves.push_back({
                            0.0, 1.0, true, initial, initialQuintic});
                    } else {
                        refine(refine, 0.0, 1.0, 0,
                            &initial, &initialQuintic);
                    }

                    const auto allLeavesAccepted =
                        std::ranges::all_of(leaves, &AdaptiveLeaf::accepted);
                    if (!allLeavesAccepted) {
                        auto pieceFailures = std::size_t{0};
                        for (const auto &leaf : leaves) {
                            if (leaf.accepted) {
                                continue;
                            }
                            ++pieceFailures;
                            const auto from = stateAt(leaf.from);
                            const auto to = stateAt(leaf.to);
                            auto finalCandidate = leaf.candidate;
                            if (!finalCandidate.geometryEvaluated) {
                                proveGeometry(finalCandidate, leaf.quintic, from, to);
                            }
                            if (leaf.from == 0.0 && leaf.to == 1.0) {
                                ++quintic.wholePieceFailures;
                            } else if (leaf.from == 0.0) {
                                ++quintic.beginningBoundaryFailures;
                            } else if (leaf.to == 1.0) {
                                ++quintic.endingBoundaryFailures;
                            } else {
                                ++quintic.interiorFailures;
                            }
                            if (!finalCandidate.geometryVerified) {
                                ++quintic.failedGeometryChecks;
                            } else if (!finalCandidate.progressVerified) {
                                ++quintic.failedProgressChecks;
                            }
                            if (!finalCandidate.constraintsVerified) {
                                ++quintic.failedConstraintChecks;
                                ++quintic.failedConstraintKinds[
                                    static_cast<std::size_t>(
                                        finalCandidate.constraint)];
                                quinticCorrection[pieceIndex] = std::max(
                                    quinticCorrection[pieceIndex],
                                    requiredTimeScale(
                                        finalCandidate.constraint,
                                        finalCandidate.maximumCorrectionRatio));
                            }
                            const auto duration =
                                totalDuration * (leaf.to - leaf.from);
                            const auto &failedQuintic = leaf.quintic;
                            const auto sampled = sampledQuinticConstraintBounds(
                                failedQuintic.controls, duration,
                                pathVelocityLimit, m_limits);
                            const auto maximumJerk =
                                maximumQuinticJerk(failedQuintic.controls, duration);
                            const auto maximumJerkPieceTime =
                                from.time + duration * maximumJerk.parameter;
                            const auto phaseUpper = maximumJerk.parameter
                                    >= 1.0 - 1e-12
                                ? std::ranges::lower_bound(
                                    boundaries, maximumJerkPieceTime, {},
                                    &TimeBoundary::time)
                                : std::ranges::upper_bound(
                                    boundaries, maximumJerkPieceTime, {},
                                    &TimeBoundary::time);
                            const auto phase = std::clamp<std::size_t>(
                                phaseUpper - boundaries.begin(), 1,
                                boundaries.size() - 1);
                            const auto originalState =
                                stateAtTime(maximumJerkPieceTime);
                            const auto originalDistance = std::clamp(
                                originalState.distance, 0.0, piece.length);
                            const auto originalPathSample =
                                piece.sampleAt(originalDistance);
                            const auto originalCurvature =
                                piece.curvatureAt(originalDistance);
                            const auto originalThirdDerivative =
                                piece.curvatureDerivativeAt(originalDistance);
                            const auto originalScalarJerk =
                                boundaries[phase - 1].jerk;
                            const auto originalJerk = add(
                                add(scaled(originalPathSample.tangent,
                                        originalScalarJerk),
                                    scaled(originalCurvature,
                                        3.0 * originalState.velocity
                                            * originalState.acceleration)),
                                scaled(originalThirdDerivative,
                                    originalState.velocity * originalState.velocity
                                        * originalState.velocity));
                            ContinuousQuinticMaterializationDiagnostics::Failure failure {
                                .timingPiece = pieceIndex,
                                .preparedPiece = piece.preparedPiece,
                                .preparedKind = piece.preparedKind,
                                .knotInterval = piece.knotInterval,
                                .firstSourceInput = piece.firstSourceInput,
                                .lastSourceInput = piece.lastSourceInput,
                                .sourceInputCount = piece.sourceInputCount,
                                .pieceCurveFrom = piece.curveFrom,
                                .pieceCurveTo = piece.curveTo,
                                .intervalFrom = leaf.from,
                                .intervalTo = leaf.to,
                                .localDistanceFrom = std::clamp(
                                    from.distance, 0.0, piece.length),
                                .localDistanceTo = std::clamp(
                                    to.distance, 0.0, piece.length),
                                .duration = duration,
                                .certifiedRatio = finalCandidate.maximumRatio,
                                .sampledRatio = sampled.maximumRatio,
                                .maximumJerkParameter = maximumJerk.parameter,
                                .maximumJerkTime =
                                    duration * maximumJerk.parameter,
                                .maximumJerkPieceTime = maximumJerkPieceTime,
                                .maximumJerkRatio =
                                    maximumJerk.jerk.length() / m_limits.pathJerk,
                                .maximumJerk = maximumJerk.jerk,
                                .originalDistance = originalDistance,
                                .originalVelocity = originalState.velocity,
                                .originalAcceleration =
                                    originalState.acceleration,
                                .originalScalarJerk = originalScalarJerk,
                                .originalJerkRatio =
                                    originalJerk.length() / m_limits.pathJerk,
                                .originalJerk = originalJerk,
                                .constraint = finalCandidate.constraint,
                                .axis = finalCandidate.axis,
                                .bezierControls = absoluteBezierControls(failedQuintic),
                                .normalizedPowerCoefficients =
                                    absoluteQuinticPowerCoefficients(failedQuintic),
                                .samples = {},
                            };
                            constexpr std::size_t FAILURE_SAMPLE_INTERVALS = 256;
                            failure.samples.reserve(FAILURE_SAMPLE_INTERVALS + 1);
                            for (std::size_t sampleIndex = 0;
                                    sampleIndex <= FAILURE_SAMPLE_INTERVALS; ++sampleIndex) {
                                const auto parameter = static_cast<double>(sampleIndex)
                                    / FAILURE_SAMPLE_INTERVALS;
                                const auto sampleTime = from.time + duration * parameter;
                                const auto scalarState = stateAtTime(sampleTime);
                                const auto localDistance = std::clamp(
                                    scalarState.distance, 0.0, piece.length);
                                failure.samples.push_back({
                                    .parameter = parameter,
                                    .time = duration * parameter,
                                    .localDistance = localDistance,
                                    .quinticPosition =
                                        evaluateQuinticPosition(
                                            failedQuintic, parameter),
                                    .preparedPosition = piece.positionAt(localDistance),
                                });
                            }
                            quintic.failures.push_back(std::move(failure));
                            quintic.maximumFailedCertifiedRatio = std::max(
                                quintic.maximumFailedCertifiedRatio,
                                finalCandidate.maximumRatio);
                            quintic.maximumFailedSampledRatio = std::max(
                                quintic.maximumFailedSampledRatio,
                                sampled.maximumRatio);
                            const auto jerkLimit = effectiveLimits[pieceIndex].jerk;
                            const auto startRampDuration =
                                std::abs(from.acceleration) / jerkLimit;
                            const auto endRampDuration =
                                std::abs(to.acceleration) / jerkLimit;
                            if (std::abs(from.acceleration) > 1e-12
                               || std::abs(to.acceleration) > 1e-12) {
                                ++quintic.failedNonzeroBoundaryAccelerations;
                            }
                            if (quintic.firstFailureDuration == 0.0) {
                                quintic.firstFailureDuration = duration;
                                quintic.firstFailureFrom = leaf.from;
                                quintic.firstFailureTo = leaf.to;
                                quintic.firstFailureCertifiedRatio =
                                    finalCandidate.maximumRatio;
                                quintic.firstFailureSampledRatio =
                                    sampled.maximumRatio;
                                quintic.firstFailureStartAcceleration =
                                    from.acceleration;
                                quintic.firstFailureEndAcceleration =
                                    to.acceleration;
                                quintic.firstFailureStartRampDuration =
                                    startRampDuration;
                                quintic.firstFailureEndRampDuration =
                                    endRampDuration;
                                quintic.firstFailureTimingPiece = pieceIndex;
                                quintic.firstFailurePreparedPiece = piece.preparedPiece;
                            }
                        }
                        quintic.failedIntervals += std::max<std::size_t>(1, pieceFailures);
                        ++quintic.unresolvedPieces;
                        shadowGlobalTime += pieceDuration;
                        continue;
                    }
                    for (const auto &leaf : leaves) {
                        const auto fromTime = totalDuration * leaf.from;
                        const auto toTime = totalDuration * leaf.to;
                        const auto from = stateAt(leaf.from);
                        const auto to = stateAt(leaf.to);
                        const auto kinematicFrom =
                            kinematicAt(pieceIndex, from);
                        const auto kinematicTo =
                            kinematicAt(pieceIndex, to);
                        const auto coefficients =
                            quinticPowerCoefficients(
                                leaf.quintic.controls);
                        quintic.shadowSpans.push_back({
                            .timingPiece = pieceIndex,
                            .preparedPiece = piece.preparedPiece,
                            .preparedKind = piece.preparedKind,
                            .knotInterval = piece.knotInterval,
                            .firstSourceInput = piece.firstSourceInput,
                            .lastSourceInput = piece.lastSourceInput,
                            .sourceInputCount = piece.sourceInputCount,
                            .degree = 5,
                            .globalTimeFrom = shadowGlobalTime + fromTime,
                            .pieceTimeFrom = fromTime,
                            .pieceTimeTo = toTime,
                            .localDistanceFrom = std::clamp(
                                from.distance, 0.0, piece.length),
                            .localDistanceTo = std::clamp(
                                to.distance, 0.0, piece.length),
                            .duration = toTime - fromTime,
                            .inverseDuration = 1.0 / (toTime - fromTime),
                            .origin = leaf.quintic.origin,
                            .coefficients = coefficients,
                            .start = {
                                kinematicFrom.position,
                                kinematicFrom.velocity,
                                kinematicFrom.acceleration,
                            },
                            .end = {
                                kinematicTo.position,
                                kinematicTo.velocity,
                                kinematicTo.acceleration,
                            },
                            .pointwiseConstraintRatio =
                                leaf.candidate.maximumRatio,
                            .acceptanceRatio =
                                leaf.candidate.acceptanceRatio,
                            .accelerationExcursionRatio =
                                leaf.candidate.accelerationExcursionRatio,
                            .subServoJerkAccepted =
                                leaf.candidate.subServoJerkAccepted,
                        });
                        const auto internalPhaseBoundaries = std::ranges::count_if(
                            boundaries, [&](const TimeBoundary &boundary) {
                                return boundary.time > fromTime && boundary.time < toTime;
                            });
                        const auto phases =
                            static_cast<std::size_t>(internalPhaseBoundaries) + 1;
                        ++quintic.finalQuinticSpans;
                        quintic.absorbedScalarPhaseBoundaries +=
                            internalPhaseBoundaries;
                        quintic.maximumScalarPhasesPerQuintic =
                            std::max(quintic.maximumScalarPhasesPerQuintic, phases);
                        quintic.maximumAcceptedRatio = std::max(
                            quintic.maximumAcceptedRatio,
                            leaf.candidate.acceptanceRatio);
                        quintic.maximumAcceptedPointwiseRatio = std::max(
                            quintic.maximumAcceptedPointwiseRatio,
                            leaf.candidate.maximumRatio);
                        if (leaf.candidate.subServoJerkAccepted) {
                            ++quintic.subServoJerkAcceptedSpans;
                            quintic.maximumSubServoAccelerationExcursionRatio =
                                std::max(
                                    quintic.maximumSubServoAccelerationExcursionRatio,
                                    leaf.candidate.accelerationExcursionRatio);
                        }
                        if (leaf.to < 1.0) {
                            ++quintic.quinticSplits;
                        }
                    }
                    if (const auto assigned =
                            assignShadowActivations(
                                piece, pieceIndex, firstShadowSpan);
                            !assigned) {
                        return std::unexpected(assigned.error());
                    }
                    shadowGlobalTime += pieceDuration;
                    if (quintic.finalQuinticSpans >= maximumQuinticSpans) {
                        quintic.resourceExhausted = true;
                        break;
                    }
                }
                const auto completeShadow =
                    quintic.failedIntervals == 0 && !quintic.resourceExhausted;
                if (!completeShadow) {
                    quintic.shadowSpans.clear();
                    quintic.shadowActivations.clear();
                } else {
                    if (quintic.shadowSpans.empty()
                       || quintic.shadowSpans.size()
                            != quintic.finalQuinticSpans) {
                        return std::unexpected(
                            "complete shadow quintic sequence has inconsistent span counts");
                    }
                    auto expectedGlobalTime = 0.0;
                    auto previousPiece = std::numeric_limits<std::size_t>::max();
                    auto expectedPieceTime = 0.0;
                    auto expectedPieceDistance = 0.0;
                    const ContinuousQuinticMaterializationDiagnostics::ShadowSpan
                        *previousSpan = nullptr;
                    auto currentShadowSpan = std::size_t{0};
                    auto worstPositionSpan = std::size_t{0};
                    auto worstPositionCheck = std::string_view{"none"};
                    const auto observeStateError = [&](const MotionState &left,
                                                       const MotionState &right,
                                                       const std::string_view check) {
                        const auto positionError =
                            subtract(left.position, right.position).length();
                        if (positionError > quintic.maximumShadowPositionError) {
                            quintic.maximumShadowPositionError = positionError;
                            worstPositionSpan = currentShadowSpan;
                            worstPositionCheck = check;
                        }
                        quintic.maximumShadowVelocityError = std::max(
                            quintic.maximumShadowVelocityError,
                            subtract(left.velocity, right.velocity).length());
                        quintic.maximumShadowAccelerationError = std::max(
                            quintic.maximumShadowAccelerationError,
                            subtract(left.acceleration,
                                right.acceleration).length());
                    };
                    for (const auto &span : quintic.shadowSpans) {
                        if (span.timingPiece >= pieces.size()
                           || (span.degree != 3 && span.degree != 5)
                           || !std::isfinite(span.duration)
                           || span.duration <= 0.0
                           || !std::isfinite(span.inverseDuration)
                           || span.inverseDuration <= 0.0) {
                            return std::unexpected(
                                "shadow quintic sequence contains an invalid span");
                        }
                        if (span.timingPiece != previousPiece) {
                            if (previousPiece !=
                                    std::numeric_limits<std::size_t>::max()) {
                                const auto &completedPiece = pieces[previousPiece];
                                const auto completedDuration =
                                    pieceTiming[previousPiece].back().time;
                                const auto completedDistanceError =
                                    std::abs(expectedPieceDistance
                                        - completedPiece.length);
                                if (completedDistanceError
                                        > std::max(1e-10,
                                            completedPiece.length * 1e-9)) {
                                    return std::unexpected(std::format(
                                        "shadow timing piece {} distance coverage "
                                        "ends at {} instead of {} kind={} linear={}",
                                        previousPiece, expectedPieceDistance,
                                        completedPiece.length,
                                        static_cast<unsigned>(
                                            completedPiece.preparedKind),
                                        completedPiece.linear));
                                }
                                quintic.maximumShadowDistanceError = std::max(
                                    quintic.maximumShadowDistanceError,
                                    completedDistanceError);
                                quintic.maximumShadowTimeError = std::max(
                                    quintic.maximumShadowTimeError,
                                    std::abs(expectedPieceTime
                                        - completedDuration));
                            }
                            if (previousPiece
                                    != std::numeric_limits<std::size_t>::max()
                               && span.timingPiece != previousPiece + 1) {
                                return std::unexpected(
                                    "shadow quintic timing pieces are out of order");
                            }
                            previousPiece = span.timingPiece;
                            expectedPieceTime = 0.0;
                            expectedPieceDistance = 0.0;
                        }
                        quintic.maximumShadowTimeError = std::max(
                            quintic.maximumShadowTimeError,
                            std::abs(span.globalTimeFrom - expectedGlobalTime));
                        quintic.maximumShadowTimeError = std::max(
                            quintic.maximumShadowTimeError,
                            std::abs(span.pieceTimeFrom - expectedPieceTime));
                        quintic.maximumShadowDistanceError = std::max(
                            quintic.maximumShadowDistanceError,
                            std::abs(span.localDistanceFrom
                                - expectedPieceDistance));
                        const auto evaluatedStart =
                            evaluateLocalNormalizedPolynomial(
                                span.origin, span.coefficients,
                                span.inverseDuration, 0.0);
                        const auto evaluatedEnd =
                            evaluateLocalNormalizedPolynomial(
                                span.origin, span.coefficients,
                                span.inverseDuration, 1.0);
                        observeStateError(
                            evaluatedStart, span.start, "polynomial_start");
                        observeStateError(
                            evaluatedEnd, span.end, "polynomial_end");
                        if (previousSpan) {
                            observeStateError(
                                previousSpan->end, span.start, "continuity");
                        }
                        expectedGlobalTime += span.duration;
                        expectedPieceTime = span.pieceTimeTo;
                        expectedPieceDistance = span.localDistanceTo;
                        previousSpan = &span;
                        ++currentShadowSpan;
                    }
                    const auto &lastPiece = pieces.back();
                    quintic.maximumShadowDistanceError = std::max(
                        quintic.maximumShadowDistanceError,
                        std::abs(expectedPieceDistance - lastPiece.length));
                    quintic.maximumShadowTimeError = std::max(
                        quintic.maximumShadowTimeError,
                        std::abs(expectedPieceTime
                            - pieceTiming.back().back().time));
                    quintic.shadowDuration = expectedGlobalTime;
                    currentShadowSpan = 0;
                    observeStateError(quintic.shadowSpans.front().start,
                        startState, "path_start");
                    currentShadowSpan = quintic.shadowSpans.size() - 1;
                    observeStateError(quintic.shadowSpans.back().end,
                        endState, "path_end");

                    const auto timeTolerance =
                        std::max(1e-10, expectedGlobalTime * 1e-12);
                    const auto positionTolerance =
                        std::max(1e-10, m_limits.arcChordTolerance * 1e-6);
                    const auto distanceTolerance = std::max(
                        1e-10, std::accumulate(
                            pieces.begin(), pieces.end(), 0.0,
                            [](const double total, const GeometryPiece &piece) {
                                return total + piece.length;
                            }) * 1e-12);
                    constexpr auto VELOCITY_TOLERANCE = 1e-8;
                    const auto accelerationTolerance =
                        std::max(1e-8, m_limits.pathAcceleration * 1e-8);
                    if (quintic.maximumShadowTimeError > timeTolerance
                       || quintic.maximumShadowDistanceError > distanceTolerance
                       || quintic.maximumShadowPositionError > positionTolerance
                       || quintic.maximumShadowVelocityError > VELOCITY_TOLERANCE
                       || quintic.maximumShadowAccelerationError
                            > accelerationTolerance) {
                        return std::unexpected(std::format(
                            "shadow quintic sequence verification failed: "
                            "position={} velocity={} acceleration={} "
                            "time_coverage={} distance_coverage={} "
                            "worst_position_span={} check={} piece={} degree={} "
                            "previous_piece={} previous_degree={} "
                            "previous_end=[{},{},{}] start=[{},{},{}] "
                            "previous_z_coefficients=[{},{},{},{}] duration={}",
                            quintic.maximumShadowPositionError,
                            quintic.maximumShadowVelocityError,
                            quintic.maximumShadowAccelerationError,
                            quintic.maximumShadowTimeError,
                            quintic.maximumShadowDistanceError,
                            worstPositionSpan, worstPositionCheck,
                            quintic.shadowSpans[worstPositionSpan].timingPiece,
                            quintic.shadowSpans[worstPositionSpan].degree,
                            worstPositionSpan == 0 ? 0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .timingPiece,
                            worstPositionSpan == 0 ? 0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .degree,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .end.position.x,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .end.position.y,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .end.position.z,
                            quintic.shadowSpans[worstPositionSpan].start.position.x,
                            quintic.shadowSpans[worstPositionSpan].start.position.y,
                            quintic.shadowSpans[worstPositionSpan].start.position.z,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .origin.z,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .coefficients[1].z,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .coefficients[2].z,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .coefficients[3].z,
                            worstPositionSpan == 0 ? 0.0
                                : quintic.shadowSpans[worstPositionSpan - 1]
                                    .duration));
                    }

                    std::vector<bool> expectedActivations(
                        geometry.commands.size(), false);
                    for (const auto &piece : pieces) {
                        for (const auto &activation : piece.activations) {
                            expectedActivations[activation.input] = true;
                        }
                    }
                    std::vector<bool> observedActivations(
                        geometry.commands.size(), false);
                    auto previousActivationTime = 0.0;
                    for (const auto &activation : quintic.shadowActivations) {
                        if (activation.input >= observedActivations.size()
                           || activation.span >= quintic.shadowSpans.size()
                           || observedActivations[activation.input]
                           || !std::isfinite(activation.globalTime)
                           || !std::isfinite(activation.localDistance)
                           || !std::isfinite(activation.parameter)
                           || activation.parameter < 0.0
                           || activation.parameter > 1.0
                           || activation.globalTime + timeTolerance
                                < previousActivationTime) {
                            return std::unexpected(
                                "shadow quintic activation ownership is inconsistent");
                        }
                        const auto &owner =
                            quintic.shadowSpans[activation.span];
                        const auto activationTimeError = std::abs(
                            activation.globalTime
                                - (owner.globalTimeFrom
                                    + owner.duration * activation.parameter));
                        if (activationTimeError > timeTolerance
                           || activation.localDistance
                                < owner.localDistanceFrom - distanceTolerance
                           || activation.localDistance
                                > owner.localDistanceTo + distanceTolerance) {
                            return std::unexpected(
                                "shadow quintic activation station is outside "
                                "its owning span");
                        }
                        observedActivations[activation.input] = true;
                        previousActivationTime = activation.globalTime;
                    }
                    if (expectedActivations != observedActivations) {
                        return std::unexpected(
                            "shadow quintic sequence does not own every activation");
                    }

                    std::vector<double> shadowBoundaries;
                    shadowBoundaries.reserve(quintic.shadowSpans.size() - 1);
                    for (std::size_t span = 0;
                            span + 1 < quintic.shadowSpans.size(); ++span) {
                        shadowBoundaries.push_back(
                            quintic.shadowSpans[span].globalTimeFrom
                                + quintic.shadowSpans[span].duration);
                    }
                    quintic.maximumShadowSpansPerServoPeriod = 1;
                    auto firstBoundary = std::size_t{0};
                    for (std::size_t boundary = 0;
                            boundary < shadowBoundaries.size(); ++boundary) {
                        while (shadowBoundaries[boundary]
                                - shadowBoundaries[firstBoundary]
                                    > m_continuousPlanningEffort.quinticServoPeriod
                               && firstBoundary < boundary) {
                            ++firstBoundary;
                        }
                        quintic.maximumShadowSpansPerServoPeriod = std::max(
                            quintic.maximumShadowSpansPerServoPeriod,
                            boundary - firstBoundary + 2);
                    }

                    constexpr auto FNV_OFFSET =
                        std::uint64_t{14695981039346656037ULL};
                    constexpr auto FNV_PRIME =
                        std::uint64_t{1099511628211ULL};
                    auto fingerprint = FNV_OFFSET;
                    const auto appendFingerprint = [&](const std::uint64_t value) {
                        fingerprint ^= value;
                        fingerprint *= FNV_PRIME;
                    };
                    const auto appendDouble = [&](const double value) {
                        appendFingerprint(std::bit_cast<std::uint64_t>(value));
                    };
                    const auto appendPosition = [&](const position_t &value) {
                        appendDouble(value.x);
                        appendDouble(value.y);
                        appendDouble(value.z);
                        appendDouble(value.a);
                        appendDouble(value.b);
                        appendDouble(value.c);
                    };
                    for (const auto &span : quintic.shadowSpans) {
                        appendFingerprint(span.timingPiece);
                        appendFingerprint(span.preparedPiece);
                        appendFingerprint(span.degree);
                        appendDouble(span.globalTimeFrom);
                        appendDouble(span.duration);
                        appendDouble(span.localDistanceFrom);
                        appendDouble(span.localDistanceTo);
                        appendPosition(span.origin);
                        for (const auto &coefficient : span.coefficients) {
                            appendPosition(coefficient);
                        }
                    }
                    for (const auto &activation : quintic.shadowActivations) {
                        appendFingerprint(activation.input);
                        appendFingerprint(activation.span);
                        appendDouble(activation.globalTime);
                        appendDouble(activation.localDistance);
                        appendDouble(activation.parameter);
                    }
                    quintic.shadowFingerprint = fingerprint;

                    quintic.shadowSequenceVerified = true;
                }
                quintic.seconds = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - quinticStarted).count();
                const auto correctedPieces = std::ranges::count_if(
                    quinticCorrection, [](const double factor) {
                        return factor > 1.0 + 1e-9;
                    });
                const auto maximumRequiredTimeScale =
                    *std::ranges::max_element(quinticCorrection);
                quintic.correctionPasses.push_back({
                    .callbackPass = correctionPass,
                    .failedIntervals = quintic.failedIntervals,
                    .correctedPieces =
                        static_cast<std::size_t>(correctedPieces),
                    .candidateEvaluations = quintic.candidateEvaluations,
                    .geometryProofs = quintic.geometryProofs,
                    .maximumFailedRatio =
                        quintic.maximumFailedCertifiedRatio,
                    .maximumRequiredTimeScale =
                        maximumRequiredTimeScale,
                    .seconds = quintic.seconds,
                });
            }

            const auto correctionStarted = std::chrono::steady_clock::now();
            const auto &quintic = result->materialization.quintic;
            const auto &quinticPass = quintic.correctionPasses.back();
            correctionHistory += std::format(
                "{}pass {}: check=quintic failed_intervals={} "
                "corrected_pieces={} candidates={} geometry_proofs={} "
                "maximum_failed_ratio={} maximum_time_scale={} pieces=[",
                correctionHistory.empty() ? "" : "; ",
                correctionPass, quinticPass.failedIntervals,
                quinticPass.correctedPieces,
                quinticPass.candidateEvaluations,
                quinticPass.geometryProofs,
                quinticPass.maximumFailedRatio,
                quinticPass.maximumRequiredTimeScale);
            auto listed = std::size_t{0};
            constexpr auto MAXIMUM_LISTED_CORRECTIONS = 16U;
            for (std::size_t pieceIndex = 0;
                    pieceIndex < quinticCorrection.size(); ++pieceIndex) {
                if (quinticCorrection[pieceIndex] <= 1.0 + 1e-9) {
                    continue;
                }
                correction[pieceIndex] = quinticCorrection[pieceIndex];
                if (listed < MAXIMUM_LISTED_CORRECTIONS) {
                    correctionHistory += std::format(
                        "{}{{piece={} required_time_scale={}}}",
                        listed == 0 ? "" : ",", pieceIndex,
                        quinticCorrection[pieceIndex]);
                }
                ++listed;
            }
            if (listed > MAXIMUM_LISTED_CORRECTIONS) {
                correctionHistory += std::format(
                    ",... {} omitted",
                    listed - MAXIMUM_LISTED_CORRECTIONS);
            }
            correctionHistory += "]";

            const auto worst =
                *std::ranges::max_element(correction);
            if (worst <= 1.0 + 1e-9) {
                constraintsVerified = true;
                result->pieceTiming.clear();
                result->pieceTiming.reserve(pieces.size());
                for (std::size_t pieceIndex = 0;
                        pieceIndex < pieces.size(); ++pieceIndex) {
                    result->pieceTiming.push_back({
                        .input = pieces[pieceIndex].input,
                        .preparedPiece = pieces[pieceIndex].preparedPiece,
                        .preparedKind = pieces[pieceIndex].preparedKind,
                        .knotInterval = pieces[pieceIndex].knotInterval,
                        .length = pieces[pieceIndex].length,
                        .curveFrom = pieces[pieceIndex].curveFrom,
                        .curveTo = pieces[pieceIndex].curveTo,
                        .linear = pieces[pieceIndex].linear,
                        .startPosition = pieces[pieceIndex].positionAt(0.0),
                        .endPosition = pieces[pieceIndex].positionAt(
                            pieces[pieceIndex].length),
                        .programmedVelocityLimit =
                            pieces[pieceIndex].programmedVelocity,
                        .staticVelocityLimit =
                            pieces[pieceIndex].staticVelocityLimit,
                        .velocityLimit = effectiveLimits[pieceIndex].velocity,
                        .accelerationLimit =
                            effectiveLimits[pieceIndex].acceleration,
                        .jerkLimit = effectiveLimits[pieceIndex].jerk,
                        .entryVelocity = stationVelocity[pieceIndex],
                        .entryAcceleration = stationAcceleration[pieceIndex],
                        .exitVelocity = stationVelocity[pieceIndex + 1],
                        .exitAcceleration =
                            stationAcceleration[pieceIndex + 1],
                        .duration = pieceTiming[pieceIndex].back().time,
                    });
                }
                result->materialization.correctionCollectionSeconds +=
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now()
                            - correctionStarted).count();

                return std::vector<path_tempo::PieceCorrection>{};
            }

            std::vector<path_tempo::PieceCorrection> corrections;
            corrections.reserve(listed);
            for (std::size_t pieceIndex = 0;
                    pieceIndex < correction.size(); ++pieceIndex) {
                if (correction[pieceIndex] > 1.0 + 1e-9) {
                    corrections.push_back({
                        .piece = pieceIndex + 1,
                        .requiredTimeScale =
                            correction[pieceIndex] * 1.01,
                    });
                }
            }
            result->materialization.correctionCollectionSeconds +=
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()
                        - correctionStarted).count();

            return corrections;
        };

        path_tempo::PathPlanner pathPlanner;
        const auto planned = pathPlanner.solve(pathTempoRequest, materializationCorrection);
        if (!planned) {
            return std::unexpected(std::format(
                "PathTempo continuous timing failed: {}; materialization history: {}",
                planned.error().message,correctionHistory));
        }

        const auto diagnosedCorrectionPasses = planned->diagnostics.correctionPasses;
        if (materializationPass > diagnosedCorrectionPasses) {
            return std::unexpected(
                "PathTempo materialization callback count is inconsistent with its correction-pass diagnostics");
        }

        if (!constraintsVerified) {
            return std::unexpected(std::format(
                "continuous local constraint correction did not converge after {} passes: {}",
                maximumLocalCorrectionPasses,correctionHistory));
        }

        result->correctionHistory = correctionHistory;

        const auto &quintic =
            result->materialization.quintic;
        if (!quintic.shadowSequenceVerified
           || quintic.shadowSpans.empty()) {
            return std::unexpected(
                "production quintic materialization has no complete "
                "verified execution sequence");
        }
        normalSpans.clear();
        normalSpans.reserve(quintic.shadowSpans.size());
        nextSpan = m_nextSpan;
        for (const auto &shadow : quintic.shadowSpans) {
            if (shadow.degree != 5
               || shadow.coefficients[0].length() > 1e-18) {
                return std::unexpected(
                    "production quintic shadow span has an invalid "
                    "degree or local constant");
            }
            normalSpans.push_back({
                .id = nextSpan++,
                .degree = ExecutionPolynomialDegree::Quintic,
                .duration = shadow.duration,
                .inverseDuration = shadow.inverseDuration,
                .inverseDurationSquared =
                    shadow.inverseDuration * shadow.inverseDuration,
                .inverseDurationCubed =
                    shadow.inverseDuration * shadow.inverseDuration
                        * shadow.inverseDuration,
                .origin = shadow.origin,
                .coefficients = {
                    shadow.coefficients[1],
                    shadow.coefficients[2],
                    shadow.coefficients[3],
                    shadow.coefficients[4],
                    shadow.coefficients[5],
                },
            });
        }
        std::ranges::fill(
            resolvedActivations, ResolvedActivation{});
        for (const auto &activation : quintic.shadowActivations) {
            if (activation.input >= resolvedActivations.size()
               || activation.span >= normalSpans.size()
               || resolvedActivations[activation.input].span != 0) {
                return std::unexpected(
                    "production quintic activation ownership is "
                    "inconsistent");
            }
            resolvedActivations[activation.input] = {
                .span = normalSpans[activation.span].id,
                .parameter = activation.parameter,
            };
        }

        const auto emittedStart = executionSpanStart(normalSpans.front());
        const auto emittedEnd = executionSpanEnd(normalSpans.back());
        const auto boundaryMismatch=[](const MotionState &actual,const MotionState &expected) {
            return std::array{
                subtract(actual.position,expected.position).length(),
                subtract(actual.velocity,expected.velocity).length(),
                subtract(actual.acceleration,expected.acceleration).length(),
            };
        };
        const auto startMismatch=boundaryMismatch(emittedStart,startState);
        const auto endMismatch=boundaryMismatch(emittedEnd,endState);
        if(startMismatch[0]>1e-8||startMismatch[1]>1e-7||startMismatch[2]>1e-7
           ||endMismatch[0]>1e-8||endMismatch[1]>1e-7||endMismatch[2]>1e-7)
            return std::unexpected(std::format(
                "continuous emitted boundary PVA mismatch: start position={} velocity={} "
                "acceleration={}; end position={} velocity={} acceleration={}",
                startMismatch[0],startMismatch[1],startMismatch[2],
                endMismatch[0],endMismatch[1],endMismatch[2]));

        result->chunks.reserve((normalSpans.size()+MAX_NORMAL_SPANS_PER_CHUNK-1)
            /MAX_NORMAL_SPANS_PER_CHUNK);
        auto predecessor=m_previousBranch;
        for(std::size_t first=0;first<normalSpans.size();first+=MAX_NORMAL_SPANS_PER_CHUNK) {
            auto &chunk=result->chunks.emplace_back();
            chunk.epoch=m_epoch;
            chunk.id=nextChunk++;
            chunk.predecessorBranch=predecessor;
            chunk.branch=chunk.id;
            const auto last=std::min(first+MAX_NORMAL_SPANS_PER_CHUNK,normalSpans.size());
            for(std::size_t span=first;span<last;++span) (void)chunk.normalMotion.push(normalSpans[span]);
            chunk.branchState =
                executionSpanEnd(chunk.normalMotion[chunk.normalMotion.size - 1]);
            if(chunk.branchState.velocity.length()<=1e-10
               &&chunk.branchState.acceleration.length()<=1e-10) {
                chunk.branchState.velocity={};
                chunk.branchState.acceleration={};
                const PathSample held{chunk.branchState.position,{}};
                auto stop=hermite(nextSpan++,held,held,0.0,0.0,1e-6);
                if(!chunk.stopTail.push(stop))
                    return std::unexpected("continuous trajectory terminal stop-tail capacity exceeded");
                chunk.stopState=chunk.branchState;
            } else {
                auto stop=stoppingSpans(chunk.branchState,m_limits,nextSpan);
                if(!stop) return std::unexpected(std::format(
                    "continuous trajectory failed to generate stop tail for packet {} at staged span {}: {}",
                    result->chunks.size()-1,last,stop.error()));
                if(stop->size()>MAX_STOP_SPANS_PER_CHUNK)
                    return std::unexpected(std::format(
                        "continuous trajectory moving stop tail exceeds fixed capacity for packet {} at "
                        "staged span {}: required spans={} capacity={}",result->chunks.size()-1,last,
                        stop->size(),MAX_STOP_SPANS_PER_CHUNK));
                for(const auto &span:*stop) (void)chunk.stopTail.push(span);
                chunk.stopState = executionSpanEnd(
                    chunk.stopTail[chunk.stopTail.size - 1]);
                chunk.stopState.velocity={};
                chunk.stopState.acceleration={};
            }
            predecessor=chunk.branch;
        }

        m_nextChunk=nextChunk;
        m_nextSpan=nextSpan;
        m_previousBranch=result->chunks.back().branch;
        m_position=expectedEnd;
        struct ActivationSpanOwner {
            std::size_t chunk = 0;
            std::uint32_t span = 0;
        };
        std::unordered_map<SpanId, ActivationSpanOwner> activationOwners;
        activationOwners.reserve(normalSpans.size());
        for (std::size_t chunk = 0; chunk < result->chunks.size(); ++chunk) {
            for (std::uint32_t span = 0;
                    span < result->chunks[chunk].normalMotion.size; ++span) {
                activationOwners.emplace(
                    result->chunks[chunk].normalMotion[span].id,
                    ActivationSpanOwner{chunk, span});
            }
        }
        std::vector<std::vector<ExecutionMarker>> chunkMarkers(
            result->chunks.size());
        auto nextExecutionMarker = m_nextExecutionMarker;
        for (std::size_t input = 0;
                input < resolvedActivations.size(); ++input) {
            if (!geometry.commands[input].presentationActivation) {
                continue;
            }
            const auto &activation = resolvedActivations[input];
            const auto owner = activationOwners.find(activation.span);
            if (activation.span == 0 || owner == activationOwners.end()
               || !std::isfinite(activation.parameter)
               || activation.parameter < 0.0 || activation.parameter > 1.0) {
                return std::unexpected(std::format(
                    "prepared command {} has no emitted activation owner",
                    geometry.commands[input].id));
            }
            const auto marker = nextExecutionMarker++;
            chunkMarkers[owner->second.chunk].push_back({
                .id = marker,
                .span = owner->second.span,
                .parameter = activation.parameter,
            });
            result->activations.push_back({
                .input = input,
                .span = activation.span,
                .chunk = owner->second.chunk,
                .marker = marker,
                .parameter = activation.parameter,
            });
        }
        for (std::size_t chunk = 0; chunk < result->chunks.size(); ++chunk) {
            auto &markers = chunkMarkers[chunk];
            std::ranges::sort(markers, [](const auto &left, const auto &right) {
                return std::tuple{
                    left.span, left.parameter, left.id,
                } < std::tuple{
                    right.span, right.parameter, right.id,
                };
            });
            if (markers.size() > MAX_EXECUTION_MARKERS_PER_CHUNK) {
                return std::unexpected(std::format(
                    "continuous trajectory execution-marker capacity exceeded "
                    "for packet {}: required markers={} capacity={}",
                    chunk, markers.size(),
                    MAX_EXECUTION_MARKERS_PER_CHUNK));
            }
            for (const auto &marker : markers) {
                (void)result->chunks[chunk].markers.push(marker);
            }
            result->executionMarkers += markers.size();
            result->maximumExecutionMarkersPerChunk = std::max(
                result->maximumExecutionMarkersPerChunk, markers.size());
            result->interiorExecutionMarkers += std::ranges::count_if(
                markers, [](const auto &marker) {
                    return marker.parameter > 0.0
                        && marker.parameter < 1.0;
                });
        }
        std::ranges::sort(result->activations,{},[](const TimedCommandActivation &activation) {
            return std::tuple{
                activation.chunk, activation.span,
                activation.parameter, activation.input,
            };
        });
        m_nextExecutionMarker = nextExecutionMarker;
        result->timeLaw=timeLawRecorder.instrumentation.diagnostics;
        return result;
    }

    std::expected<TriggeredMove, std::string> TrajectoryCompiler::compileTriggeredMove(
            const ProbeMove &command, const DigitalInputId input, const InputCondition condition) {
        if(m_limits.pathAcceleration <= 0.0 || m_limits.pathJerk <= 0.0 || command.feed() <= 0.0
           || !positiveAxisLimits(m_limits.axisVelocity)
           || !positiveAxisLimits(m_limits.axisAcceleration)
           || !positiveAxisLimits(m_limits.axisJerk))
            return std::unexpected("triggered-move limits must be positive");
        const auto delta = subtract(command.target(), command.from());
        const auto length = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z
            + delta.a*delta.a + delta.b*delta.b + delta.c*delta.c);
        if(length <= 1e-12) return std::unexpected("triggered move must have nonzero length");
        const auto direction = scaled(delta, 1.0 / length);
        const auto magnitudes = [&](const double limit) {
            return position_t { std::abs(direction.x)*limit, std::abs(direction.y)*limit,
                std::abs(direction.z)*limit, std::abs(direction.a)*limit,
                std::abs(direction.b)*limit, std::abs(direction.c)*limit };
        };
        TriggeredMove move;
        move.epoch = m_epoch;
        move.id = m_nextChunk++;
        move.predecessorBranch = m_previousBranch;
        move.branch = move.id;
        move.moveId = command.id();
        move.target = command.target();
        move.limits.velocity = magnitudes(std::min(command.feed() / 60.0,
            pathLimit(direction, m_limits.axisVelocity)));
        move.limits.acceleration = magnitudes(std::min(m_limits.pathAcceleration,
            pathLimit(direction, m_limits.axisAcceleration)));
        move.limits.jerk = magnitudes(std::min(m_limits.pathJerk,
            pathLimit(direction, m_limits.axisJerk)));
        move.input = input;
        move.condition = condition;
        move.triggerRequired = command.errorIfNotFound();
        m_position = command.target();
        m_previousBranch = move.branch;
        return move;
    }
}
