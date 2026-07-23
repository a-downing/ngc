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
                span.d,
                add(span.d, scaled(span.c, 1.0 / 3.0)),
                add(span.d, add(scaled(span.c, 2.0 / 3.0), scaled(span.b, 1.0 / 3.0))),
                add(span.d, add(span.c, add(span.b, span.a))),
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
            result.a = add(scaled(delta, -2.0), add(v0t, v1t));
            result.b = subtract(scaled(delta, 3.0), add(scaled(v0t, 2.0), v1t));
            result.c = v0t;
            result.d = from.position;
            result.end.position = to.position;
            result.end.velocity = scaled(to.tangent, toVelocity);
            const auto secondAtEnd = add(scaled(result.a, 6.0), scaled(result.b, 2.0));
            result.end.acceleration = scaled(secondAtEnd, result.inverseDurationSquared);
            return result;
        }

        struct KinematicPathState {
            position_t position{};
            position_t velocity{};
            position_t acceleration{};
        };

        std::array<position_t, 6> quinticBezierControls(const KinematicPathState &from,
                                                        const KinematicPathState &to,
                                                        const double duration) {
            const auto durationSquared = duration * duration;
            return {
                from.position,
                add(from.position, scaled(from.velocity, duration / 5.0)),
                add(from.position, add(scaled(from.velocity, 2.0 * duration / 5.0),
                    scaled(from.acceleration, durationSquared / 20.0))),
                add(to.position, add(scaled(to.velocity, -2.0 * duration / 5.0),
                    scaled(to.acceleration, durationSquared / 20.0))),
                add(to.position, scaled(to.velocity, -duration / 5.0)),
                to.position,
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
                span.a=scaled(jerks[index],h*h*h/6.0);
                span.b=scaled(state.acceleration,h*h/2.0);
                span.c=scaled(state.velocity,h);
                span.d=state.position;
                KinematicPathState terminal {
                    .position=add(state.position,add(add(span.a,span.b),span.c)),
                    .velocity=add(state.velocity,add(scaled(state.acceleration,h),
                        scaled(jerks[index],h*h/2.0))),
                    .acceleration=add(state.acceleration,scaled(jerks[index],h)),
                };
                if(index+1==result.size()) terminal=to;
                span.end={terminal.position,terminal.velocity,terminal.acceleration};
                state=terminal;
            }
            return result;
        }

        double minimumNumericallyStableC2Duration(const KinematicPathState &from,
                                                   const KinematicPathState &to,
                                                   const TrajectoryLimits &limits) {
            constexpr std::array components {
                &position_t::x,&position_t::y,&position_t::z,
                &position_t::a,&position_t::b,&position_t::c,
            };
            auto coordinateScale=1.0;
            for(const auto component:components)
                coordinateScale=std::max({coordinateScale,
                    std::abs(from.position.*component),std::abs(to.position.*component)});
            auto result=0.0;
            const auto consider=[&](const double scale,const double jerkLimit) {
                if(!std::isfinite(jerkLimit)||jerkLimit<=0.0) return;
                // c2CubicChain divides the interval into three equal spans and
                // recovers their jerks from an endpoint-position residual divided
                // by h^3. Keep ordinary double-precision endpoint subtraction
                // comfortably below the relevant jerk scale.
                constexpr auto ROUNDING_BUDGET=1024.0*std::numeric_limits<double>::epsilon();
                result=std::max(result,3.0*std::cbrt(ROUNDING_BUDGET*scale/jerkLimit));
            };
            consider(coordinateScale,limits.pathJerk);
            for(const auto component:components)
                consider(std::max({1.0,std::abs(from.position.*component),
                    std::abs(to.position.*component)}),limits.axisJerk.*component);
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
            result.a=scaled(jerk,duration*duration*duration/6.0);
            result.b=scaled(from.acceleration,duration*duration/2.0);
            result.c=scaled(from.velocity,duration);
            result.d=from.position;
            result.end=to;
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
            result.back().end.velocity={};
            result.back().end.acceleration={};
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

        double maximumLinearAcceleration(const AxisPolynomialSpan &span) {
            const auto at = [&](const double u) {
                const auto value = scaled(add(scaled(span.a, 6.0*u), scaled(span.b, 2.0)),
                                          span.inverseDurationSquared);
                return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                    + value.a*value.a + value.b*value.b + value.c*value.c);
            };
            return std::max(at(0.0), at(1.0));
        }

        double maximumLinearJerk(const AxisPolynomialSpan &span) {
            const auto value = scaled(span.a, 6.0 * span.inverseDurationCubed);
            return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                + value.a*value.a + value.b*value.b + value.c*value.c);
        }

        std::size_t cubicSeverityBin(const double ratio) {
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
        constexpr std::array AXIS_NAMES {"X","Y","Z","A","B","C"};

        struct QuinticConstraintBounds {
            double maximumRatio = 0.0;
            ContinuousPolynomialConstraintKind constraint =
                ContinuousPolynomialConstraintKind::PathAcceleration;
            std::size_t axis = std::numeric_limits<std::size_t>::max();
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
                if (ratio > result.maximumRatio) {
                    result.maximumRatio = ratio;
                    result.constraint = constraint;
                    result.axis = axis;
                }
            };

            const auto vectorMagnitude = [](const position_t &control) {
                return control.length();
            };
            observe(certifiedBezierMaximum(velocityControls, pathVelocityLimit,
                        vectorMagnitude, visitedNodes), pathVelocityLimit,
                ContinuousPolynomialConstraintKind::PathVelocity,
                std::numeric_limits<std::size_t>::max());
            observe(certifiedBezierMaximum(accelerationControls, limits.pathAcceleration,
                        vectorMagnitude, visitedNodes), limits.pathAcceleration,
                ContinuousPolynomialConstraintKind::PathAcceleration,
                std::numeric_limits<std::size_t>::max());
            observe(certifiedBezierMaximum(jerkControls, limits.pathJerk,
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
                            limits.axisAcceleration.*component, componentMagnitude, visitedNodes),
                    limits.axisAcceleration.*component,
                    ContinuousPolynomialConstraintKind::AxisAcceleration, axis);
                observe(certifiedBezierMaximum(jerkControls,
                            limits.axisJerk.*component, componentMagnitude, visitedNodes),
                    limits.axisJerk.*component,
                    ContinuousPolynomialConstraintKind::AxisJerk, axis);
            }
            return result;
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

    TrajectoryCompiler::TrajectoryCompiler(TrajectoryLimits limits) : m_limits(limits) { }

    void TrajectoryCompiler::reset(const EpochId epoch, const position_t &position) {
        m_epoch = epoch;
        m_nextChunk = 1;
        m_nextSpan = 1;
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
                    span.end.velocity = scaled(span.end.velocity, 1.0 / scaleFactor);
                    span.end.acceleration = scaled(span.end.acceleration, 1.0 / (scaleFactor * scaleFactor));
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

        chunk.branchState = chunk.normalMotion[chunk.normalMotion.size - 1].end;
        chunk.branchState.velocity = {};
        chunk.branchState.acceleration = {};
        const PathSample held { chunk.branchState.position, {} };
        auto stop = hermite(m_nextSpan++, held, held, 0.0, 0.0, 1e-6);
        stop.end = chunk.branchState;
        if(!chunk.stopTail.push(stop))
            return std::unexpected("trajectory chunk stop-tail capacity exceeded");
        chunk.stopState = chunk.branchState;
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
           ||m_continuousPlanningEffort.geometryVerificationBudgetMultiplier<36
           ||m_continuousPlanningEffort.geometryVerificationBudgetMultiplier>256
           ||(m_continuousPlanningEffort.boundaryAccelerationMode
                  != ContinuousBoundaryAccelerationMode::Zero
              && m_continuousPlanningEffort.boundaryAccelerationMode
                  != ContinuousBoundaryAccelerationMode::Optimized)
           ||(m_continuousPlanningEffort.constraintCheckMode
                  != ContinuousConstraintCheckMode::Materialized
              && m_continuousPlanningEffort.constraintCheckMode
                  != ContinuousConstraintCheckMode::Sampled
              && m_continuousPlanningEffort.constraintCheckMode
                  != ContinuousConstraintCheckMode::GeometryDiagnostic))
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
        const auto geometryDiagnostic=m_continuousPlanningEffort.constraintCheckMode
            ==ContinuousConstraintCheckMode::GeometryDiagnostic;
        constexpr auto GEOMETRY_DIAGNOSTIC_LIMIT_SCALE=0.99;
        const auto planningLimitScale=
            geometryDiagnostic?GEOMETRY_DIAGNOSTIC_LIMIT_SCALE:1.0;
        const auto planningPathAcceleration=m_limits.pathAcceleration*planningLimitScale;
        const auto planningPathJerk=m_limits.pathJerk*planningLimitScale;
        const auto planningAxisAcceleration=scaled(m_limits.axisAcceleration,planningLimitScale);
        const auto planningAxisJerk=scaled(m_limits.axisJerk,planningLimitScale);
        std::vector<AxisPolynomialSpan> normalSpans;
        const auto maximumStagedNormalSpans=std::max<std::size_t>(8192,pieces.size()*8);
        const auto maximumGeometryVerificationAttemptsPerPass=
            std::max<std::size_t>(8192,pieces.size()*4);
        const auto maximumTotalGeometryVerificationAttempts=
            std::max<std::size_t>(32768,pieces.size()
                *m_continuousPlanningEffort.geometryVerificationBudgetMultiplier);
        auto nextChunk=m_nextChunk;
        auto nextSpan=m_nextSpan;
        std::vector<SpanId> activationSpans(geometry.commands.size(),0);

        std::vector<path_tempo::InitialPieceLimits> effectiveLimits(pieces.size());

        const auto kinematicAt=[&](const std::size_t pieceIndex,const TimeBoundary &boundary) {
            const auto &piece=pieces[pieceIndex];
            auto local=std::clamp(boundary.distance,0.0,piece.length);
            if(local<1e-10) local=0.0;
            else if(piece.length-local<1e-10) local=piece.length;
            const auto sample=piece.sampleAt(local);
            const auto curvature=piece.curvatureAt(local);
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
        std::size_t totalGeometryVerificationAttempts=0;
        std::vector<double> previousStationVelocity;
        std::vector<double> previousStationAcceleration;
        std::vector<TimeLaw> previousPieceTiming;
        std::vector<std::size_t> previouslyCorrectedPieces;
        const auto bitExactDouble=[](const double left,const double right) {
            return std::bit_cast<std::uint64_t>(left)
                ==std::bit_cast<std::uint64_t>(right);
        };
        const auto bitExactTimeLaw=[&](const TimeLaw &left,const TimeLaw &right) {
            if(left.size()!=right.size()) return false;
            for(std::size_t boundary=0;boundary<left.size();++boundary) {
                const auto &a=left[boundary];
                const auto &b=right[boundary];
                if(!bitExactDouble(a.time,b.time)
                   ||!bitExactDouble(a.distance,b.distance)
                   ||!bitExactDouble(a.velocity,b.velocity)
                   ||!bitExactDouble(a.acceleration,b.acceleration)
                   ||!bitExactDouble(a.jerk,b.jerk)) return false;
            }
            return true;
        };
        const auto bitExactLimits=[&](const path_tempo::InitialPieceLimits &left,
                                      const path_tempo::InitialPieceLimits &right) {
            return bitExactDouble(left.velocity,right.velocity)
                &&bitExactDouble(left.acceleration,right.acceleration)
                &&bitExactDouble(left.jerk,right.jerk);
        };
        struct ConstraintViolationDiagnostic {
            double factor=1.0;
            std::size_t localSpan=0;
            const char *constraint="none";
            const char *axis="path";
            double measured=0.0;
            double limit=0.0;
            double ratio=1.0;
            double duration=0.0;
            std::size_t scalarPhase=0;
            double scalarJerk=0.0;
            double phaseDistanceChange=0.0;
            double phaseVelocityChange=0.0;
            double phaseAccelerationChange=0.0;
            double sampleDistance=0.0;
            double sampleVelocity=0.0;
            double sampleAcceleration=0.0;
        };
        struct MaterializedPiece {
            bool valid=false;
            TimeLaw timing;
            path_tempo::InitialPieceLimits limits{};
            std::vector<AxisPolynomialSpan> spans;
            std::vector<std::pair<std::size_t,std::size_t>> activationOwners;
            double correction=1.0;
            ConstraintViolationDiagnostic violation;
            double geometryCorrection=1.0;
            ConstraintViolationDiagnostic geometryViolation;
        };
        std::vector<MaterializedPiece> materializedPieces(pieces.size());
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
                .applySampledCorrections =
                    usesPathTempoSampledCorrections(m_continuousPlanningEffort),
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

            const auto candidateDuration=std::accumulate(pieceTiming.begin(),pieceTiming.end(),0.0,
                [](const double total,const auto &timing) { return total+timing.back().time; });
            if(correctionPass>0&&!previouslyCorrectedPieces.empty()) {
                CorrectionPassLocalityDiagnostic locality {
                    .pass=correctionPass,
                    .pieceCount=pieces.size(),
                    .correctedPieces=previouslyCorrectedPieces.size(),
                    .firstCorrectedPiece=previouslyCorrectedPieces.front(),
                    .lastCorrectedPiece=previouslyCorrectedPieces.back(),
                };
                auto firstChangedStation=stationVelocity.size();
                auto lastChangedStation=std::size_t {0};
                for(std::size_t station=0;station<stationVelocity.size();++station) {
                    locality.maximumVelocityChange=std::max(locality.maximumVelocityChange,
                        std::abs(stationVelocity[station]-previousStationVelocity[station]));
                    locality.maximumAccelerationChange=std::max(
                        locality.maximumAccelerationChange,
                        std::abs(stationAcceleration[station]
                            -previousStationAcceleration[station]));
                    if(bitExactDouble(stationVelocity[station],previousStationVelocity[station])
                       &&bitExactDouble(stationAcceleration[station],
                           previousStationAcceleration[station])) continue;
                    ++locality.changedStations;
                    firstChangedStation=std::min(firstChangedStation,station);
                    lastChangedStation=station;
                }
                if(locality.changedStations>0) {
                    locality.firstChangedStation=firstChangedStation;
                    locality.lastChangedStation=lastChangedStation;
                }

                auto firstChangedPiece=pieces.size();
                auto lastChangedPiece=std::size_t {0};
                std::vector<bool> changedPieceTiming(pieces.size(),false);
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    locality.maximumDurationChange=std::max(locality.maximumDurationChange,
                        std::abs(pieceTiming[pieceIndex].back().time
                            -previousPieceTiming[pieceIndex].back().time));
                    if(bitExactTimeLaw(pieceTiming[pieceIndex],
                                      previousPieceTiming[pieceIndex])) {
                        ++locality.bitExactReusablePieceTimings;
                        continue;
                    }
                    ++locality.changedPieceTimings;
                    changedPieceTiming[pieceIndex]=true;
                    firstChangedPiece=std::min(firstChangedPiece,pieceIndex);
                    lastChangedPiece=pieceIndex;
                    const auto corrected=std::ranges::lower_bound(
                        previouslyCorrectedPieces,pieceIndex);
                    if(corrected==previouslyCorrectedPieces.end()||*corrected!=pieceIndex)
                        ++locality.changedUncorrectedPieceTimings;
                    auto distance=pieces.size();
                    if(corrected!=previouslyCorrectedPieces.end())
                        distance=std::min(distance,*corrected-pieceIndex);
                    if(corrected!=previouslyCorrectedPieces.begin())
                        distance=std::min(distance,
                            pieceIndex-*(corrected-1));
                    locality.maximumPropagationFromCorrectedPiece=std::max(
                        locality.maximumPropagationFromCorrectedPiece,distance);
                }
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();) {
                    const auto changed=changedPieceTiming[pieceIndex];
                    const auto begin=pieceIndex;
                    while(pieceIndex<pieces.size()
                          &&changedPieceTiming[pieceIndex]==changed) ++pieceIndex;
                    const auto length=pieceIndex-begin;
                    if(changed) {
                        ++locality.changedPieceTimingRuns;
                        locality.maximumChangedPieceTimingRun=std::max(
                            locality.maximumChangedPieceTimingRun,length);
                    } else {
                        ++locality.bitExactReusablePieceTimingRuns;
                        locality.maximumBitExactReusablePieceTimingRun=std::max(
                            locality.maximumBitExactReusablePieceTimingRun,length);
                    }
                }
                if(locality.changedPieceTimings>0) {
                    locality.firstChangedPieceTiming=firstChangedPiece;
                    locality.lastChangedPieceTiming=lastChangedPiece;
                    locality.bitExactReusablePrefixPieces=firstChangedPiece;
                    locality.bitExactReusableSuffixPieces=pieces.size()-lastChangedPiece-1;
                    if(firstChangedPiece<locality.firstCorrectedPiece)
                        locality.leftPropagationPieces=
                            locality.firstCorrectedPiece-firstChangedPiece;
                    if(lastChangedPiece>locality.lastCorrectedPiece)
                        locality.rightPropagationPieces=
                            lastChangedPiece-locality.lastCorrectedPiece;
                } else {
                    locality.bitExactReusablePrefixPieces=pieces.size();
                }
                timeLawRecorder.instrumentation.diagnostics.correctionPassLocality
                    .push_back(locality);
            }
            auto maximumStationAcceleration=0.0;
            for(const auto value:stationAcceleration)
                maximumStationAcceleration=std::max(maximumStationAcceleration,std::abs(value));
            std::size_t geometryVerificationAttempts=0;
            std::size_t stagedSpanCount=0;
            std::vector<std::size_t> pieceSpanOffsets(pieces.size());
            std::vector<double> correction(pieces.size(),1.0);
            std::vector<ConstraintViolationDiagnostic> violation(pieces.size());
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                if((pieceIndex&15U)==0) reportProgress();
                const auto &piece=pieces[pieceIndex];
                const auto &boundaries=pieceTiming[pieceIndex];
                pieceSpanOffsets[pieceIndex]=stagedSpanCount;
                auto &cached=materializedPieces[pieceIndex];
                const auto comparisonStarted=std::chrono::steady_clock::now();
                // Geometry, machine limits, tolerance, and the materialization
                // algorithm are immutable for this solve-local cache. Exact
                // time-law and effective-limit equality therefore completes
                // the candidate fingerprint.
                const auto reusable=m_continuousPlanningEffort.reuseMaterializedPieces
                    &&cached.valid
                    &&bitExactTimeLaw(cached.timing,boundaries)
                    &&bitExactLimits(cached.limits,effectiveLimits[pieceIndex]);
                result->materialization.cacheComparisonSeconds+=
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-comparisonStarted).count();
                if(reusable) {
                    ++result->materialization.reusedPieces;
                    if(stagedSpanCount>maximumStagedNormalSpans
                       ||cached.spans.size()>maximumStagedNormalSpans-stagedSpanCount)
                        return std::unexpected(std::format(
                            "continuous cached staged-span resource bound exceeded: staged={} "
                            "cached={} bound={} pieces={} piece={} input={}",stagedSpanCount,
                            cached.spans.size(),maximumStagedNormalSpans,pieces.size(),pieceIndex,
                            piece.input));
                    stagedSpanCount+=cached.spans.size();
                    correction[pieceIndex]=geometryDiagnostic
                        ?cached.geometryCorrection:cached.correction;
                    violation[pieceIndex]=geometryDiagnostic
                        ?cached.geometryViolation:cached.violation;
                    continue;
                }

                ++result->materialization.materializedPieces;
                MaterializedPiece materialized;
                materialized.timing=boundaries;
                materialized.limits=effectiveLimits[pieceIndex];
                const auto emit=[&](const auto &self,const auto &stateAt,
                                    const double totalDuration,const std::size_t interval,
                                    const double u0,const double u1,
                                    const unsigned depth) -> std::optional<std::string> {
                        if(stagedSpanCount+materialized.spans.size()>=maximumStagedNormalSpans)
                            return std::format(
                                "continuous staged-span resource bound exceeded: staged={} bound={} "
                                "pieces={} piece={} input={} interval={} depth={} jerk_limit={}",
                                stagedSpanCount+materialized.spans.size(),
                                maximumStagedNormalSpans,pieces.size(),
                                pieceIndex,piece.input,interval,depth,
                                effectiveLimits[pieceIndex].jerk);
                        ++geometryVerificationAttempts;
                        ++totalGeometryVerificationAttempts;
                        if((geometryVerificationAttempts&127U)==0) reportProgress();
                        if(geometryVerificationAttempts>maximumGeometryVerificationAttemptsPerPass)
                            return std::format(
                                "continuous per-pass geometry-verification resource bound exceeded: "
                                "pass={} attempts={} bound={} total_attempts={} staged={} pieces={} "
                                "piece={} input={} interval={} depth={} jerk_limit={}",correctionPass,
                                geometryVerificationAttempts,maximumGeometryVerificationAttemptsPerPass,
                                totalGeometryVerificationAttempts,
                                stagedSpanCount+materialized.spans.size(),pieces.size(),
                                pieceIndex,piece.input,interval,depth,
                                effectiveLimits[pieceIndex].jerk);
                        if(totalGeometryVerificationAttempts>maximumTotalGeometryVerificationAttempts)
                            return std::format(
                                "continuous cumulative geometry-verification resource bound exceeded: "
                                "pass={} total_attempts={} bound={} pass_attempts={} staged={} pieces={} "
                                "piece={} input={} interval={} depth={} jerk_limit={}",correctionPass,
                                totalGeometryVerificationAttempts,maximumTotalGeometryVerificationAttempts,
                                geometryVerificationAttempts,
                                stagedSpanCount+materialized.spans.size(),pieces.size(),pieceIndex,
                                piece.input,interval,depth,effectiveLimits[pieceIndex].jerk);
                        auto from=stateAt(u0);
                        auto to=stateAt(u1);
                        const auto duration=totalDuration*(u1-u0);
                        if(duration<=1e-12) return std::nullopt;
                        const auto localFrom=std::clamp(from.distance,0.0,piece.length);
                        const auto localTo=std::clamp(to.distance,0.0,piece.length);
                        if(localTo+std::max(1e-12,piece.length*1e-10)<localFrom)
                            return std::format(
                                "continuous local time law reverses inside curved emission: "
                                "pass={} piece={} input={} depth={} u=[{},{}] "
                                "distance=[{},{}] piece_length={} duration={}",
                                correctionPass,pieceIndex,piece.input,depth,u0,u1,
                                localFrom,localTo,piece.length,duration);
                        const auto constructionStarted=std::chrono::steady_clock::now();
                        std::vector<AxisPolynomialSpan> chain;
                        if(piece.linear) {
                            const auto start=piece.sampleAt(localFrom);
                            const auto finish=piece.sampleAt(localTo);
                            AxisPolynomialSpan span;
                            span.id=0;
                            span.duration=duration;
                            span.inverseDuration=1.0/duration;
                            span.inverseDurationSquared=span.inverseDuration*span.inverseDuration;
                            span.inverseDurationCubed=span.inverseDurationSquared*span.inverseDuration;
                            span.a=scaled(start.tangent,
                                (to.acceleration-from.acceleration)*duration*duration/6.0);
                            span.b=scaled(start.tangent,from.acceleration*duration*duration/2.0);
                            span.c=scaled(start.tangent,from.velocity*duration);
                            span.d=start.position;
                            span.end.position=finish.position;
                            span.end.velocity=scaled(finish.tangent,to.velocity);
                            span.end.acceleration=scaled(finish.tangent,to.acceleration);
                            chain.push_back(span);
                        } else {
                            const auto curved=c2CubicChain(0,kinematicAt(pieceIndex,from),
                                kinematicAt(pieceIndex,to),duration);
                            chain.assign(curved.begin(),curved.end());
                        }
                        result->materialization.cubicConstructionSeconds+=
                            std::chrono::duration<double>(
                                std::chrono::steady_clock::now()-constructionStarted).count();
                        if(stagedSpanCount+materialized.spans.size()+chain.size()
                                >maximumStagedNormalSpans)
                            return std::format(
                                "continuous staged-span resource bound exceeded: staged={} bound={} "
                                "pieces={} piece={} input={} interval={} depth={} jerk_limit={}",
                                stagedSpanCount+materialized.spans.size()+chain.size(),
                                maximumStagedNormalSpans,pieces.size(),pieceIndex,piece.input,
                                interval,depth,effectiveLimits[pieceIndex].jerk);
                        auto verified=true;
                        for(std::size_t chainSpan=0;chainSpan<chain.size();++chainSpan) {
                            const auto fraction0=static_cast<double>(chainSpan)/chain.size();
                            const auto fraction1=static_cast<double>(chainSpan+1)/chain.size();
                            const auto source0=stateAt(std::lerp(u0,u1,fraction0)).distance;
                            const auto source1=stateAt(std::lerp(u0,u1,fraction1)).distance;
                            const auto proofFrom=std::clamp(source0,localFrom,localTo);
                            const auto proofTo=std::clamp(source1,localFrom,localTo);
                            const auto verificationStarted=std::chrono::steady_clock::now();
                            const auto proof=verifiesOrderedCurveTolerance(chain[chainSpan],
                                proofFrom,proofTo,m_limits.arcChordTolerance,piece.positionAt,
                                piece.chordErrorBound);
                            result->materialization.geometryVerificationSeconds+=
                                std::chrono::duration<double>(
                                    std::chrono::steady_clock::now()-verificationStarted).count();
                            if(!proof) {
                                verified=false;
                                break;
                            }
                        }
                        if(!verified) {
                            if(depth>=20) return std::format(
                                "continuous local cubic tolerance verification did not converge at "
                                "piece {} source input {} phase {} recursion depth {} scalar u=[{},{}] "
                                "local distance=[{},{}] duration={} tolerance={}",pieceIndex,piece.input,
                                interval,depth,u0,u1,localFrom,localTo,duration,m_limits.arcChordTolerance);
                            auto split=std::midpoint(u0,u1);
                            if(!piece.linear) {
                                std::optional<double> phaseSplit;
                                const auto fromTime=totalDuration*u0;
                                const auto toTime=totalDuration*u1;
                                const auto targetTime=totalDuration*split;
                                const auto minimumSide=std::max(1e-12,totalDuration*1e-12);
                                const auto candidate=std::ranges::lower_bound(
                                    boundaries,targetTime,{},&TimeBoundary::time);
                                const auto considerBoundary=[&](const auto iterator) {
                                    if(iterator==boundaries.end()) return;
                                    const auto time=iterator->time;
                                    if(time<=fromTime+minimumSide||time>=toTime-minimumSide) return;
                                    const auto candidateSplit=time/totalDuration;
                                    const auto candidateState=kinematicAt(
                                        pieceIndex,stateAt(candidateSplit));
                                    if(time-fromTime<minimumNumericallyStableC2Duration(
                                            kinematicAt(pieceIndex,from),candidateState,m_limits)
                                       ||toTime-time<minimumNumericallyStableC2Duration(
                                            candidateState,kinematicAt(pieceIndex,to),m_limits))
                                        return;
                                    if(!phaseSplit
                                       ||std::abs(candidateSplit-std::midpoint(u0,u1))
                                          <std::abs(*phaseSplit-std::midpoint(u0,u1)))
                                        phaseSplit=candidateSplit;
                                };
                                considerBoundary(candidate);
                                if(candidate!=boundaries.begin()) considerBoundary(candidate-1);
                                if(phaseSplit) split=*phaseSplit;
                            }
                            if(auto error=self(self,stateAt,totalDuration,interval,u0,split,depth+1))
                                return error;
                            return self(self,stateAt,totalDuration,interval,split,u1,depth+1);
                        }
                        const auto firstLocalSpan=materialized.spans.size();
                        materialized.spans.insert(
                            materialized.spans.end(),chain.begin(),chain.end());
                        const auto activationTolerance=std::max(
                            1e-12,piece.length*1e-10);
                        const auto finalEmission=
                            localTo>=piece.length-activationTolerance;
                        for(const auto &activation:piece.activations) {
                            if(std::ranges::any_of(materialized.activationOwners,
                                    [&](const auto &owner) {
                                        return owner.first==activation.input;
                                    })
                               ||activation.distance<localFrom-activationTolerance
                               ||(activation.distance>=localTo-activationTolerance
                                  &&!(finalEmission&&activation.distance
                                        <=localTo+activationTolerance))) continue;
                            auto owningSpan=chain.size()-1;
                            for(std::size_t chainSpan=0;
                                    chainSpan+1<chain.size();++chainSpan) {
                                const auto fraction=static_cast<double>(chainSpan+1)
                                    /static_cast<double>(chain.size());
                                const auto boundaryDistance=stateAt(
                                    std::lerp(u0,u1,fraction)).distance;
                                if(activation.distance<boundaryDistance-activationTolerance) {
                                    owningSpan=chainSpan;
                                    break;
                                }
                            }
                            materialized.activationOwners.emplace_back(
                                activation.input,firstLocalSpan+owningSpan);
                        }
                        return std::nullopt;
                };
                if(piece.linear) {
                    for(std::size_t phase=1;phase<boundaries.size();++phase) {
                        const auto scalar=localScalarPhase(boundaries[phase-1],boundaries[phase]);
                        const auto stateAt=[&](const double u) { return scalar.at(u); };
                        if(auto error=emit(emit,stateAt,scalar.duration,phase,0.0,1.0,0))
                            return std::unexpected(*error);
                    }
                } else {
                    const auto totalDuration=boundaries.back().time;
                    const auto stateAt=[&](const double u) {
                        if(u<=0.0) return boundaries.front();
                        if(u>=1.0) return boundaries.back();
                        const auto time=totalDuration*u;
                        const auto upper=std::ranges::upper_bound(boundaries,time,{},&TimeBoundary::time);
                        const auto index=std::clamp<std::size_t>(upper-boundaries.begin(),1,
                            boundaries.size()-1);
                        const auto scalar=localScalarPhase(boundaries[index-1],boundaries[index]);
                        return scalar.at(std::clamp((time-boundaries[index-1].time)/scalar.duration,
                            0.0,1.0));
                    };
                    if(auto error=emit(emit,stateAt,totalDuration,0,0.0,1.0,0))
                        return std::unexpected(*error);
                }

                if(materialized.spans.empty())
                    return std::unexpected(std::format(
                        "continuous timing piece {} emitted no motion spans",pieceIndex));
                if(geometryDiagnostic) {
                    auto geometrySample=std::size_t {0};
                    const auto evaluateGeometry=[&](const TimeBoundary &state,
                                                    const double scalarJerk,
                                                    const std::size_t scalarPhase,
                                                    const TimeBoundary &phaseFrom,
                                                    const TimeBoundary &phaseTo) {
                        ++result->materialization.geometryDifferentialChecks;
                        const auto distance=std::clamp(state.distance,0.0,piece.length);
                        const auto pathSample=piece.sampleAt(distance);
                        const auto curvature=piece.curvatureAt(distance);
                        const auto thirdDerivative=piece.curvatureDerivativeAt(distance);
                        const auto velocity=scaled(pathSample.tangent,state.velocity);
                        const auto acceleration=add(
                            scaled(pathSample.tangent,state.acceleration),
                            scaled(curvature,state.velocity*state.velocity));
                        const auto jerk=add(
                            add(scaled(pathSample.tangent,scalarJerk),
                                scaled(curvature,3.0*state.velocity*state.acceleration)),
                            scaled(thirdDerivative,state.velocity*state.velocity*state.velocity));
                        const auto consider=[&](const double factor,const char *constraintName,
                                                const char *axis,const double measured,
                                                const double limit) {
                            if(factor<=materialized.geometryCorrection) return;
                            materialized.geometryCorrection=factor;
                            materialized.geometryViolation={
                                .factor=factor,.localSpan=geometrySample,
                                .constraint=constraintName,.axis=axis,.measured=measured,
                                .limit=limit,.ratio=measured/limit,
                                .duration=phaseTo.time-phaseFrom.time,
                                .scalarPhase=scalarPhase,.scalarJerk=scalarJerk,
                                .phaseDistanceChange=phaseTo.distance-phaseFrom.distance,
                                .phaseVelocityChange=phaseTo.velocity-phaseFrom.velocity,
                                .phaseAccelerationChange=
                                    phaseTo.acceleration-phaseFrom.acceleration,
                                .sampleDistance=state.distance,.sampleVelocity=state.velocity,
                                .sampleAcceleration=state.acceleration,
                            };
                        };
                        consider(std::sqrt(acceleration.length()/m_limits.pathAcceleration),
                            "path_acceleration","path",acceleration.length(),
                            m_limits.pathAcceleration);
                        consider(std::cbrt(jerk.length()/m_limits.pathJerk),
                            "path_jerk","path",jerk.length(),m_limits.pathJerk);
                        for(std::size_t axis=0;axis<AXIS_COMPONENTS.size();++axis) {
                            const auto component=AXIS_COMPONENTS[axis];
                            consider(std::abs(velocity.*component)
                                    /(m_limits.axisVelocity.*component),
                                "axis_velocity",AXIS_NAMES[axis],
                                std::abs(velocity.*component),m_limits.axisVelocity.*component);
                            consider(std::sqrt(std::abs(acceleration.*component)
                                    /(m_limits.axisAcceleration.*component)),
                                "axis_acceleration",AXIS_NAMES[axis],
                                std::abs(acceleration.*component),
                                m_limits.axisAcceleration.*component);
                            consider(std::cbrt(std::abs(jerk.*component)
                                    /(m_limits.axisJerk.*component)),
                                "axis_jerk",AXIS_NAMES[axis],std::abs(jerk.*component),
                                m_limits.axisJerk.*component);
                        }
                        ++geometrySample;
                    };
                    for(std::size_t phase=1;phase<boundaries.size();++phase) {
                        const auto scalar=localScalarPhase(
                            boundaries[phase-1],boundaries[phase]);
                        for(const auto u:std::array{0.0,0.5,1.0})
                            evaluateGeometry(scalar.at(u),boundaries[phase-1].jerk,
                                phase-1,boundaries[phase-1],boundaries[phase]);
                    }
                    for(std::size_t sampleIndex=0;sampleIndex<piece.geometricSamples.size();
                            ++sampleIndex) {
                        const auto requested=sampleIndex==0?0.0
                            :sampleIndex+1==piece.geometricSamples.size()?piece.length
                            :piece.geometricSamples[sampleIndex].distance
                                -piece.geometricSampleDistanceOffset;
                        const auto upper=std::ranges::lower_bound(
                            boundaries,requested,{},&TimeBoundary::distance);
                        const auto phase=std::clamp<std::size_t>(
                            upper-boundaries.begin(),1,boundaries.size()-1);
                        const auto scalar=localScalarPhase(
                            boundaries[phase-1],boundaries[phase]);
                        auto low=0.0;
                        auto high=1.0;
                        for(auto iteration=0;iteration<48;++iteration) {
                            const auto middle=std::midpoint(low,high);
                            if(scalar.at(middle).distance<requested) low=middle;
                            else high=middle;
                        }
                        evaluateGeometry(scalar.at(std::midpoint(low,high)),
                            boundaries[phase-1].jerk,phase-1,
                            boundaries[phase-1],boundaries[phase]);
                    }
                }
                if(m_continuousPlanningEffort.constraintCheckMode
                        !=ContinuousConstraintCheckMode::Sampled) {
                    const auto constraintStarted=std::chrono::steady_clock::now();
                    for(std::size_t localSpan=0;localSpan<materialized.spans.size();++localSpan) {
                        const auto &span=materialized.spans[localSpan];
                        ++result->materialization.exactConstraintSpanChecks;
                        const auto consider=[&](const double factor,const char *constraintName,
                                                const char *axis,const double measured,
                                                const double limit) {
                            if(factor<=materialized.correction) return;
                            materialized.correction=factor;
                            materialized.violation={
                                .factor=factor,.localSpan=localSpan,.constraint=constraintName,
                                .axis=axis,.measured=measured,.limit=limit,
                                .ratio=measured/limit,.duration=span.duration,
                            };
                        };
                        const auto pathAcceleration=maximumLinearAcceleration(span);
                        consider(std::sqrt(pathAcceleration/m_limits.pathAcceleration),
                            "path_acceleration","path",pathAcceleration,m_limits.pathAcceleration);
                        const auto pathJerk=maximumLinearJerk(span);
                        consider(std::cbrt(pathJerk/m_limits.pathJerk),
                            "path_jerk","path",pathJerk,m_limits.pathJerk);
                        for(std::size_t axis=0;axis<AXIS_COMPONENTS.size();++axis) {
                            const auto component=AXIS_COMPONENTS[axis];
                            const auto velocity=trajectory_detail::maximumAxisVelocity(span,component);
                            consider(velocity/(m_limits.axisVelocity.*component),"axis_velocity",
                                AXIS_NAMES[axis],velocity,m_limits.axisVelocity.*component);
                            const auto acceleration=
                                trajectory_detail::maximumAxisAcceleration(span,component);
                            consider(std::sqrt(acceleration/(m_limits.axisAcceleration.*component)),
                                "axis_acceleration",AXIS_NAMES[axis],acceleration,
                                m_limits.axisAcceleration.*component);
                            const auto jerk=trajectory_detail::maximumAxisJerk(span,component);
                            consider(std::cbrt(jerk/(m_limits.axisJerk.*component)),"axis_jerk",
                                AXIS_NAMES[axis],jerk,m_limits.axisJerk.*component);
                        }
                    }
                    result->materialization.exactConstraintSeconds+=
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now()-constraintStarted).count();
                }
                materialized.valid=true;
                cached=std::move(materialized);
                stagedSpanCount+=cached.spans.size();
                correction[pieceIndex]=geometryDiagnostic
                    ?cached.geometryCorrection:cached.correction;
                violation[pieceIndex]=geometryDiagnostic
                    ?cached.geometryViolation:cached.violation;
            }
            result->geometryVerificationAttempts=totalGeometryVerificationAttempts;
            result->geometryVerificationHighWater=std::max(
                result->geometryVerificationHighWater,geometryVerificationAttempts);
            if(stagedSpanCount==0)
                return std::unexpected("continuous locally timed trajectory emitted no motion spans");
            if(geometryDiagnostic) {
                result->materialization.axisCubicViolationPieces = 0;
                result->materialization.maximumAxisCubicTimeScale = 1.0;
                result->materialization.cubicViolations = {};
                auto &diagnostics = result->materialization.cubicViolations;
                constexpr auto MATERIAL_VIOLATION_TOLERANCE = 1e-9;
                for (std::size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex) {
                    const auto &piece = pieces[pieceIndex];
                    const auto &materialized = materializedPieces[pieceIndex];
                    auto pieceViolation = false;
                    result->materialization.maximumAxisCubicTimeScale = std::max(
                        result->materialization.maximumAxisCubicTimeScale,
                        materialized.correction);
                    for (std::size_t localSpan = 0;
                            localSpan < materialized.spans.size(); ++localSpan) {
                        const auto &span = materialized.spans[localSpan];
                        auto worstRatio = 0.0;
                        auto worstConstraint =
                            ContinuousPolynomialConstraintKind::PathAcceleration;
                        auto worstAxis = std::numeric_limits<std::size_t>::max();
                        const auto observe = [&](const double measured, const double limit,
                                ContinuousCubicConstraintSeverity &severity,
                                const ContinuousPolynomialConstraintKind constraint,
                                const std::size_t axis) {
                            const auto ratio = measured / limit;
                            if (ratio > worstRatio) {
                                worstRatio = ratio;
                                worstConstraint = constraint;
                                worstAxis = axis;
                            }
                            if (ratio <= 1.0 + MATERIAL_VIOLATION_TOLERANCE) {
                                return;
                            }
                            ++severity.violatingSpans;
                            severity.violatingDuration += span.duration;
                            if (ratio > severity.maximumRatio) {
                                severity.maximumRatio = ratio;
                                severity.worstSpanDuration = span.duration;
                                severity.worstTimingPiece = pieceIndex;
                                severity.worstLocalSpan = localSpan;
                            }
                        };
                        observe(maximumLinearAcceleration(span), m_limits.pathAcceleration,
                            diagnostics.pathAcceleration,
                            ContinuousPolynomialConstraintKind::PathAcceleration,
                            std::numeric_limits<std::size_t>::max());
                        observe(maximumLinearJerk(span), m_limits.pathJerk,
                            diagnostics.pathJerk, ContinuousPolynomialConstraintKind::PathJerk,
                            std::numeric_limits<std::size_t>::max());
                        for (std::size_t axis = 0; axis < AXIS_COMPONENTS.size(); ++axis) {
                            const auto component = AXIS_COMPONENTS[axis];
                            observe(trajectory_detail::maximumAxisVelocity(span, component),
                                m_limits.axisVelocity.*component,
                                diagnostics.axisVelocity[axis],
                                ContinuousPolynomialConstraintKind::AxisVelocity, axis);
                            observe(trajectory_detail::maximumAxisAcceleration(span, component),
                                m_limits.axisAcceleration.*component,
                                diagnostics.axisAcceleration[axis],
                                ContinuousPolynomialConstraintKind::AxisAcceleration, axis);
                            observe(trajectory_detail::maximumAxisJerk(span, component),
                                m_limits.axisJerk.*component, diagnostics.axisJerk[axis],
                                ContinuousPolynomialConstraintKind::AxisJerk, axis);
                        }

                        ++diagnostics.spans;
                        ++diagnostics.worstRatioHistogram[cubicSeverityBin(worstRatio)];
                        if (worstRatio <= 1.0 + MATERIAL_VIOLATION_TOLERANCE) {
                            continue;
                        }
                        pieceViolation = true;
                        ++diagnostics.violatingSpans;
                        diagnostics.violatingDuration += span.duration;
                        if(worstRatio > diagnostics.maximumRatio) {
                            diagnostics.maximumRatio = worstRatio;
                            diagnostics.worstSpanDuration = span.duration;
                            diagnostics.worstTimingPiece = pieceIndex;
                            diagnostics.worstPreparedPiece = piece.preparedPiece;
                            diagnostics.worstInput = piece.input;
                            diagnostics.worstLocalSpan = localSpan;
                            diagnostics.worstConstraint = worstConstraint;
                            diagnostics.worstAxis = worstAxis;
                        }
                    }
                    if (pieceViolation) {
                        ++diagnostics.violatingPieces;
                    }
                }
                result->materialization.axisCubicViolationPieces =
                    diagnostics.violatingPieces;

                // Benchmark an alternative NRT materialization without changing the emitted
                // cubic plan. Each accepted quintic matches endpoint PVA, stays within the
                // prepared-geometry tolerance, and has certified derivative-control bounds.
                auto &quintic = result->materialization.quinticPrototype;
                quintic = {};
                quintic.ran = true;
                const auto quinticStarted = std::chrono::steady_clock::now();
                const auto maximumPrototypeSpans =
                    std::max<std::size_t>(8192,pieces.size() * 32);
                const auto maximumPrototypeProofs =
                    std::max<std::size_t>(32768,pieces.size() * 64);
                for (std::size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex) {
                    const auto &piece = pieces[pieceIndex];
                    if (piece.linear) {
                        quintic.retainedLinearCubicSpans +=
                            materializedPieces[pieceIndex].spans.size();
                        continue;
                    }
                    const auto &boundaries = pieceTiming[pieceIndex];
                    struct GroupCandidate {
                        bool accepted = false;
                        bool constraintsVerified = false;
                        bool geometryVerified = false;
                        bool progressVerified = false;
                        double maximumRatio = 0.0;
                        ContinuousPolynomialConstraintKind constraint =
                            ContinuousPolynomialConstraintKind::PathAcceleration;
                        std::size_t axis = std::numeric_limits<std::size_t>::max();
                    };
                    const auto pathVelocityLimit =
                        std::min(piece.programmedVelocity, piece.staticVelocityLimit);
                    const auto evaluateStates = [&](const TimeBoundary &from,
                                                    const TimeBoundary &to) {
                        GroupCandidate candidate;
                        ++quintic.groupCandidateEvaluations;
                        if (quintic.geometryProofs >= maximumPrototypeProofs) {
                            quintic.resourceExhausted = true;
                            return candidate;
                        }
                        const auto duration = to.time - from.time;
                        if (duration <= 1e-12) {
                            ++quintic.numericallyUnrefinableIntervals;
                            return candidate;
                        }
                        const auto controls = quinticBezierControls(
                            kinematicAt(pieceIndex, from), kinematicAt(pieceIndex, to), duration);
                        const auto bounds = quinticConstraintBounds(controls, duration,
                            pathVelocityLimit, m_limits, quintic.constraintBoundNodes);
                        candidate.maximumRatio = bounds.maximumRatio;
                        candidate.constraint = bounds.constraint;
                        candidate.axis = bounds.axis;
                        const auto localFrom = std::clamp(from.distance, 0.0, piece.length);
                        const auto localTo = std::clamp(to.distance, 0.0, piece.length);
                        ++quintic.geometryProofs;
                        const auto geometryAndProgressVerified = verifiesOrderedCurveTolerance(
                            controls, localFrom, localTo, m_limits.arcChordTolerance,
                            piece.positionAt, piece.chordErrorBound, true);
                        auto geometryVerified = geometryAndProgressVerified;
                        if (!geometryAndProgressVerified) {
                            ++quintic.geometryProofs;
                            geometryVerified = verifiesOrderedCurveTolerance(
                                controls, localFrom, localTo, m_limits.arcChordTolerance,
                                piece.positionAt, piece.chordErrorBound);
                            if (geometryVerified) {
                                ++quintic.forwardProgressFailures;
                            }
                        }
                        candidate.geometryVerified = geometryVerified;
                        candidate.progressVerified = geometryAndProgressVerified;
                        const auto constraintsVerified =
                            bounds.maximumRatio <= 1.0 + MATERIAL_VIOLATION_TOLERANCE;
                        candidate.constraintsVerified = constraintsVerified;
                        if (!geometryVerified) {
                            ++quintic.geometryRefinements;
                        }
                        if (!constraintsVerified) {
                            ++quintic.constraintRefinements;
                        }
                        candidate.accepted =
                            geometryAndProgressVerified && constraintsVerified;
                        return candidate;
                    };

                    const auto totalDuration = boundaries.back().time;
                    const auto stateAtTime = [&](const double requested) {
                        if (requested <= 0.0) {
                            return boundaries.front();
                        }
                        if (requested >= totalDuration) {
                            return boundaries.back();
                        }
                        const auto upper = std::ranges::upper_bound(
                            boundaries, requested, {}, &TimeBoundary::time);
                        const auto phase = std::clamp<std::size_t>(
                            upper - boundaries.begin(), 1, boundaries.size() - 1);
                        const auto scalar = localScalarPhase(
                            boundaries[phase - 1], boundaries[phase]);
                        auto result = scalar.at(std::clamp(
                            (requested - boundaries[phase - 1].time) / scalar.duration,
                            0.0, 1.0));
                        result.time = requested;
                        return result;
                    };
                    const auto stateAt = [&](const double u) {
                        return stateAtTime(totalDuration * u);
                    };

                    const auto initial = evaluateStates(
                        boundaries.front(), boundaries.back());
                    ++quintic.initialCurvedSpans;
                    ++quintic.initialWorstRatioHistogram[
                        cubicSeverityBin(initial.maximumRatio)];
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
                        GroupCandidate candidate;
                    };
                    std::vector<AdaptiveLeaf> leaves;
                    const auto refine = [&](const auto &self, const double u0,
                                            const double u1, const unsigned depth) -> void {
                        if (quintic.resourceExhausted) {
                            return;
                        }
                        const auto from = stateAt(u0);
                        const auto to = stateAt(u1);
                        const auto candidate = evaluateStates(from, to);
                        if (candidate.accepted) {
                            leaves.push_back({u0, u1, true, candidate});
                            quintic.maximumDepth = std::max(quintic.maximumDepth, depth);
                            return;
                        }
                        if (depth >= 20) {
                            leaves.push_back({u0, u1, false, candidate});
                            ++quintic.numericallyUnrefinableIntervals;
                            quintic.maximumDepth = std::max(quintic.maximumDepth, depth);
                            return;
                        }
                        auto split = std::midpoint(u0, u1);
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
                            return leftDuration >= minimumNumericallyStableC2Duration(
                                       kinematicFrom, middle, m_limits)
                                && rightDuration >= minimumNumericallyStableC2Duration(
                                       middle, kinematicTo, m_limits);
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
                            leaves.push_back({u0, u1, false, candidate});
                            ++quintic.numericallyUnrefinableIntervals;
                            quintic.maximumDepth = std::max(quintic.maximumDepth, depth);
                            return;
                        }
                        ++quintic.subdivisions;
                        self(self, u0, split, depth + 1);
                        self(self, split, u1, depth + 1);
                    };
                    if (initial.accepted) {
                        leaves.push_back({0.0, 1.0, true, initial});
                    } else {
                        refine(refine, 0.0, 1.0, 0);
                    }

                    std::vector<double> groupNodes;
                    groupNodes.reserve(leaves.size() + 1);
                    groupNodes.push_back(0.0);
                    for (const auto &leaf : leaves) {
                        groupNodes.push_back(leaf.to);
                    }
                    const auto groupNodeCount = groupNodes.size();
                    std::vector<GroupCandidate> candidates(groupNodeCount * groupNodeCount);
                    std::vector<bool> candidateEvaluated(groupNodeCount * groupNodeCount);
                    for (std::size_t leaf = 0; leaf < leaves.size(); ++leaf) {
                        const auto candidateIndex =
                            leaf * groupNodeCount + leaf + 1;
                        candidates[candidateIndex] = leaves[leaf].candidate;
                        candidateEvaluated[candidateIndex] = true;
                    }
                    candidates[groupNodeCount - 1] = initial;
                    candidateEvaluated[groupNodeCount - 1] = true;
                    const auto evaluate = [&](const std::size_t fromNode,
                                              const std::size_t toNode) -> GroupCandidate & {
                        const auto candidateIndex =
                            fromNode * groupNodeCount + toNode;
                        if (!candidateEvaluated[candidateIndex]) {
                            candidates[candidateIndex] = evaluateStates(
                                stateAt(groupNodes[fromNode]),
                                stateAt(groupNodes[toNode]));
                            candidateEvaluated[candidateIndex] = true;
                        }
                        return candidates[candidateIndex];
                    };
                    const auto finalNode = groupNodeCount - 1;
                    std::vector<std::int8_t> solutionState(groupNodeCount);
                    std::vector<std::size_t> nextNode(groupNodeCount);
                    const auto solve = [&](const auto &self,
                                           const std::size_t fromNode) -> bool {
                        if (fromNode == finalNode) {
                            return true;
                        }
                        if (solutionState[fromNode] != 0) {
                            return solutionState[fromNode] > 0;
                        }
                        for (auto toNode = finalNode; toNode > fromNode; --toNode) {
                            if (evaluate(fromNode, toNode).accepted
                               && self(self, toNode)) {
                                nextNode[fromNode] = toNode;
                                solutionState[fromNode] = 1;
                                return true;
                            }
                            if (quintic.resourceExhausted) {
                                break;
                            }
                        }
                        solutionState[fromNode] = -1;
                        return false;
                    };
                    const auto grouped = solve(solve, 0);
                    if (!grouped) {
                        auto pieceFailures = std::size_t{0};
                        for (const auto &leaf : leaves) {
                            if (leaf.accepted) {
                                continue;
                            }
                            ++pieceFailures;
                            if (leaf.from == 0.0 && leaf.to == 1.0) {
                                ++quintic.wholePieceFailures;
                            } else if (leaf.from == 0.0) {
                                ++quintic.beginningBoundaryFailures;
                            } else if (leaf.to == 1.0) {
                                ++quintic.endingBoundaryFailures;
                            } else {
                                ++quintic.interiorFailures;
                            }
                            if (!leaf.candidate.geometryVerified) {
                                ++quintic.failedGeometryChecks;
                            } else if (!leaf.candidate.progressVerified) {
                                ++quintic.failedProgressChecks;
                            }
                            if (!leaf.candidate.constraintsVerified) {
                                ++quintic.failedConstraintChecks;
                                ++quintic.failedConstraintKinds[
                                    static_cast<std::size_t>(
                                        leaf.candidate.constraint)];
                            }
                            const auto from = stateAt(leaf.from);
                            const auto to = stateAt(leaf.to);
                            const auto duration =
                                totalDuration * (leaf.to - leaf.from);
                            const auto controls = quinticBezierControls(
                                kinematicAt(pieceIndex, from),
                                kinematicAt(pieceIndex, to), duration);
                            const auto sampled = sampledQuinticConstraintBounds(
                                controls, duration, pathVelocityLimit, m_limits);
                            quintic.maximumFailedCertifiedRatio = std::max(
                                quintic.maximumFailedCertifiedRatio,
                                leaf.candidate.maximumRatio);
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
                                    leaf.candidate.maximumRatio;
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
                        ++quintic.ungroupablePieces;
                        continue;
                    }
                    for (auto fromNode = std::size_t{0}; fromNode < finalNode;) {
                        const auto toNode = nextNode[fromNode];
                        const auto fromTime = totalDuration * groupNodes[fromNode];
                        const auto toTime = totalDuration * groupNodes[toNode];
                        const auto internalPhaseBoundaries = std::ranges::count_if(
                            boundaries, [&](const TimeBoundary &boundary) {
                                return boundary.time > fromTime && boundary.time < toTime;
                            });
                        const auto phases =
                            static_cast<std::size_t>(internalPhaseBoundaries) + 1;
                        const auto &candidate =
                            candidates[fromNode * groupNodeCount + toNode];
                        ++quintic.finalQuinticSpans;
                        quintic.groupedPhaseBoundaries += internalPhaseBoundaries;
                        quintic.maximumPhasesPerGroup =
                            std::max(quintic.maximumPhasesPerGroup, phases);
                        quintic.maximumAcceptedRatio = std::max(
                            quintic.maximumAcceptedRatio, candidate.maximumRatio);
                        if (toNode < finalNode) {
                            ++quintic.phaseBoundarySplits;
                        }
                        fromNode = toNode;
                    }
                    if (quintic.finalQuinticSpans >= maximumPrototypeSpans) {
                        quintic.resourceExhausted = true;
                        break;
                    }
                }
                quintic.seconds = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - quinticStarted).count();
            }

            const auto correctionStarted=std::chrono::steady_clock::now();
            const auto worstIterator=std::ranges::max_element(correction);
            const auto worst=*worstIterator;
            const auto worstPiece=static_cast<std::size_t>(worstIterator-correction.begin());
            const auto &worstViolation=violation[worstPiece];
            const auto worstStagedSpan=pieceSpanOffsets[worstPiece]+worstViolation.localSpan;
            const auto worstSpanId=m_nextSpan+worstStagedSpan;
            const auto violatingPieces=std::ranges::count_if(correction,[](const double factor) {
                return factor>1.0+1e-9;
            });
            const auto sampled=m_continuousPlanningEffort.constraintCheckMode
                ==ContinuousConstraintCheckMode::Sampled;
            if(geometryDiagnostic) {
                correctionHistory+=std::format(
                    "{}pass {}: factor={} violating_pieces={} piece={} input={} "
                    "check=actual_geometry "
                    "sample={} scalar_phase={} phase_duration={} phase_ds={} phase_dv={} "
                    "phase_da={} scalar_jerk={} sample_s={} sample_v={} sample_a={} "
                    "constraint={} axis={} measured={} limit={} "
                    "measured_over_limit={} candidate_duration={} max_station_acceleration={} "
                    "station_state=[v={} a={} -> v={} a={}] local_limits=[v={} a={} j={}]",
                    correctionHistory.empty()?"":"; ",correctionPass,worst,
                    violatingPieces,worstPiece,
                    pieces[worstPiece].input,worstViolation.localSpan,
                    worstViolation.scalarPhase,worstViolation.duration,
                    worstViolation.phaseDistanceChange,worstViolation.phaseVelocityChange,
                    worstViolation.phaseAccelerationChange,worstViolation.scalarJerk,
                    worstViolation.sampleDistance,worstViolation.sampleVelocity,
                    worstViolation.sampleAcceleration,
                    worstViolation.constraint,worstViolation.axis,worstViolation.measured,
                    worstViolation.limit,worstViolation.ratio,candidateDuration,
                    maximumStationAcceleration,stationVelocity[worstPiece],
                    stationAcceleration[worstPiece],stationVelocity[worstPiece+1],
                    stationAcceleration[worstPiece+1],effectiveLimits[worstPiece].velocity,
                    effectiveLimits[worstPiece].acceleration,
                    effectiveLimits[worstPiece].jerk);
                if(violatingPieces>0) {
                    const auto kindName=[](const PreparedPieceKind kind) {
                        switch(kind) {
                            case PreparedPieceKind::RetainedLineSection:
                                return std::string_view{"retained_line"};
                            case PreparedPieceKind::RetainedArcSection:
                                return std::string_view{"retained_arc"};
                            case PreparedPieceKind::JunctionBlend:
                                return std::string_view{"junction_blend"};
                            case PreparedPieceKind::ClusterSpline:
                                return std::string_view{"cluster_spline"};
                        }
                        return std::string_view{"unknown"};
                    };
                    const auto sourceBlock=[&](const std::size_t input) {
                        if(input>=geometry.commands.size()) return std::string{"<invalid>"};
                        const auto &blocks=geometry.commands[input].presentation.activeBlocks;
                        if(blocks.empty()) return std::string{"<none>"};
                        const auto &block=blocks.back();
                        return std::format("{}:{} block {} '{}'",block.source,block.line,
                            block.id,block.text);
                    };
                    correctionHistory+=" violations=[";
                    auto listed=std::size_t{0};
                    constexpr auto MAXIMUM_LISTED_VIOLATIONS=16U;
                    for(std::size_t pieceIndex=0;
                            pieceIndex<correction.size()
                                &&listed<MAXIMUM_LISTED_VIOLATIONS;++pieceIndex) {
                        if(correction[pieceIndex]<=1.0+1e-9) continue;
                        const auto &piece=pieces[pieceIndex];
                        const auto &pieceViolation=violation[pieceIndex];
                        const auto distance=std::clamp(
                            pieceViolation.sampleDistance,0.0,piece.length);
                        const auto pathSample=piece.sampleAt(distance);
                        const auto curvature=piece.curvatureAt(distance);
                        const auto thirdDerivative=piece.curvatureDerivativeAt(distance);
                        const PreparedGeometricSample *nearestStation=nullptr;
                        auto nearestStationDistance=0.0;
                        auto nearestStationError=std::numeric_limits<double>::infinity();
                        for(std::size_t sampleIndex=0;
                                sampleIndex<piece.geometricSamples.size();++sampleIndex) {
                            const auto stationDistance=sampleIndex==0?0.0
                                :sampleIndex+1==piece.geometricSamples.size()?piece.length
                                :piece.geometricSamples[sampleIndex].distance
                                    -piece.geometricSampleDistanceOffset;
                            const auto error=std::abs(stationDistance-distance);
                            if(error<nearestStationError) {
                                nearestStation=&piece.geometricSamples[sampleIndex];
                                nearestStationDistance=stationDistance;
                                nearestStationError=error;
                            }
                        }
                        const auto knot=piece.knotInterval
                                ==std::numeric_limits<std::size_t>::max()
                            ?std::string{"none"}:std::to_string(piece.knotInterval);
                        correctionHistory+=std::format(
                            "{}{{piece={} prepared_piece={} kind={} knot={} "
                            "sources={} first=\"{}\" last=\"{}\" curve=[{},{}] "
                            "local_s={} xyz=[{},{},{}] tangent=[{},{},{}] "
                            "curvature=[{},{},{}] curvature_magnitude={} "
                            "q3=[{},{},{}] q3_magnitude={} phase={} phase_duration={} "
                            "nearest_station_s={} nearest_station_error={} "
                            "station_q3=[{},{},{}] station_q3_magnitude={} "
                            "phase_ds={} scalar=[v={} a={} j={}] constraint={} axis={} "
                            "measured={} limit={} ratio={} factor={}}}",
                            listed==0?"":",",pieceIndex,piece.preparedPiece,
                            kindName(piece.preparedKind),knot,piece.sourceInputCount,
                            sourceBlock(piece.firstSourceInput),
                            sourceBlock(piece.lastSourceInput),piece.curveFrom,piece.curveTo,
                            distance,pathSample.position.x,pathSample.position.y,
                            pathSample.position.z,pathSample.tangent.x,pathSample.tangent.y,
                            pathSample.tangent.z,curvature.x,curvature.y,curvature.z,
                            curvature.length(),thirdDerivative.x,thirdDerivative.y,
                            thirdDerivative.z,thirdDerivative.length(),
                            pieceViolation.scalarPhase,pieceViolation.duration,
                            nearestStationDistance,nearestStationError,
                            nearestStation->curvatureDerivative.x,
                            nearestStation->curvatureDerivative.y,
                            nearestStation->curvatureDerivative.z,
                            nearestStation->curvatureDerivative.length(),
                            pieceViolation.phaseDistanceChange,pieceViolation.sampleVelocity,
                            pieceViolation.sampleAcceleration,pieceViolation.scalarJerk,
                            pieceViolation.constraint,pieceViolation.axis,
                            pieceViolation.measured,pieceViolation.limit,pieceViolation.ratio,
                            correction[pieceIndex]);
                        ++listed;
                    }
                    if(static_cast<std::size_t>(violatingPieces)>listed)
                        correctionHistory+=std::format(",... {} omitted",
                            static_cast<std::size_t>(violatingPieces)-listed);
                    correctionHistory+="]";
                }
            } else if(!sampled)
                correctionHistory+=std::format(
                    "{}pass {}: factor={} piece={} input={} geometry={} piece_length={} "
                    "span_id={} staged_span={} span_duration={} constraint={} axis={} measured={} "
                    "limit={} measured_over_limit={} timing_candidate={} candidate_duration={} "
                    "max_station_acceleration={} station_state=[v={} a={} -> v={} a={}] "
                    "local_limits=[v={} a={} j={}]",
                    correctionHistory.empty()?"":"; ",correctionPass,worst,worstPiece,
                    pieces[worstPiece].input,pieces[worstPiece].linear?"linear":"curved",
                    pieces[worstPiece].length,worstSpanId,worstStagedSpan,
                    worstViolation.duration,worstViolation.constraint,worstViolation.axis,
                    worstViolation.measured,worstViolation.limit,worstViolation.ratio,
                    "velocity-seed",candidateDuration,
                    maximumStationAcceleration,
                    stationVelocity[worstPiece],stationAcceleration[worstPiece],
                    stationVelocity[worstPiece+1],stationAcceleration[worstPiece+1],
                    effectiveLimits[worstPiece].velocity,
                    effectiveLimits[worstPiece].acceleration,
                    effectiveLimits[worstPiece].jerk);
            if(sampled||worst<=1.0+1e-9) {
                constraintsVerified = true;
                result->materialization.correctionCollectionSeconds+=
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-correctionStarted).count();

                const auto assemblyStarted=std::chrono::steady_clock::now();
                normalSpans.clear();
                normalSpans.reserve(stagedSpanCount);
                std::ranges::fill(activationSpans,SpanId{});
                nextSpan=m_nextSpan;
                for(const auto &materialized:materializedPieces) {
                    const auto firstSpan=normalSpans.size();
                    for(auto span:materialized.spans) {
                        span.id=nextSpan++;
                        normalSpans.push_back(span);
                    }
                    for(const auto &[input,localSpan]:materialized.activationOwners) {
                        if(input>=activationSpans.size()||localSpan>=materialized.spans.size())
                            return std::unexpected(
                                "continuous cached activation ownership is inconsistent");
                        if(activationSpans[input]!=0) continue;
                        activationSpans[input]=normalSpans[firstSpan+localSpan].id;
                    }
                }
                result->pieceTiming.clear();
                result->pieceTiming.reserve(pieces.size());
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    result->pieceTiming.push_back({
                        .input=pieces[pieceIndex].input,
                        .length=pieces[pieceIndex].length,
                        .linear=pieces[pieceIndex].linear,
                        .startPosition=pieces[pieceIndex].positionAt(0.0),
                        .endPosition=pieces[pieceIndex].positionAt(pieces[pieceIndex].length),
                        .programmedVelocityLimit=pieces[pieceIndex].programmedVelocity,
                        .staticVelocityLimit=pieces[pieceIndex].staticVelocityLimit,
                        .velocityLimit=effectiveLimits[pieceIndex].velocity,
                        .accelerationLimit=effectiveLimits[pieceIndex].acceleration,
                        .jerkLimit=effectiveLimits[pieceIndex].jerk,
                        .entryVelocity=stationVelocity[pieceIndex],
                        .entryAcceleration=stationAcceleration[pieceIndex],
                        .exitVelocity=stationVelocity[pieceIndex+1],
                        .exitAcceleration=stationAcceleration[pieceIndex+1],
                        .duration=pieceTiming[pieceIndex].back().time,
                    });
                }
                result->materialization.finalAssemblySeconds+=
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-assemblyStarted).count();

                return std::vector<path_tempo::PieceCorrection>{};
            }

            previousStationVelocity = stationVelocity;
            previousStationAcceleration = stationAcceleration;
            previousPieceTiming = pieceTiming;
            previouslyCorrectedPieces.clear();

            std::vector<path_tempo::PieceCorrection> corrections;
            corrections.reserve(pieces.size());
            for (std::size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex) {
                if (correction[pieceIndex] <= 1.0 + 1e-9) {
                    continue;
                }

                previouslyCorrectedPieces.push_back(pieceIndex);
                corrections.push_back({
                    .piece = pieceIndex + 1,
                    .requiredTimeScale = correction[pieceIndex] * 1.01,
                });
            }
            result->materialization.correctionCollectionSeconds+=
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-correctionStarted).count();

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
        if ((!usesPathTempoSampledCorrections(m_continuousPlanningEffort)
                &&m_continuousPlanningEffort.constraintCheckMode
                    ==ContinuousConstraintCheckMode::Materialized
                &&materializationPass!=diagnosedCorrectionPasses)
            ||materializationPass>diagnosedCorrectionPasses) {
            return std::unexpected(
                "PathTempo materialization callback count is inconsistent with its correction-pass diagnostics");
        }

        if (!constraintsVerified) {
            return std::unexpected(std::format(
                "continuous local constraint correction did not converge after {} passes: {}",
                maximumLocalCorrectionPasses,correctionHistory));
        }

        result->correctionHistory = correctionHistory;

        const MotionState emittedStart{
            normalSpans.front().d,
            scaled(normalSpans.front().c,normalSpans.front().inverseDuration),
            scaled(normalSpans.front().b,2.0*normalSpans.front().inverseDurationSquared),
        };
        const auto &emittedEnd=normalSpans.back().end;
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
            chunk.branchState=chunk.normalMotion[chunk.normalMotion.size-1].end;
            if(chunk.branchState.velocity.length()<=1e-10
               &&chunk.branchState.acceleration.length()<=1e-10) {
                chunk.branchState.velocity={};
                chunk.branchState.acceleration={};
                const PathSample held{chunk.branchState.position,{}};
                auto stop=hermite(nextSpan++,held,held,0.0,0.0,1e-6);
                stop.end=chunk.branchState;
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
                chunk.stopState=chunk.stopTail[chunk.stopTail.size-1].end;
                chunk.stopState.velocity={};
                chunk.stopState.acceleration={};
            }
            predecessor=chunk.branch;
        }

        m_nextChunk=nextChunk;
        m_nextSpan=nextSpan;
        m_previousBranch=result->chunks.back().branch;
        m_position=expectedEnd;
        std::unordered_map<SpanId,std::size_t> activationChunk;
        activationChunk.reserve(normalSpans.size());
        for(std::size_t chunk=0;chunk<result->chunks.size();++chunk)
            for(const auto &span:result->chunks[chunk].normalMotion)
                activationChunk.emplace(span.id,chunk);
        for(std::size_t input=0;input<activationSpans.size();++input) {
            if(!geometry.commands[input].presentationActivation) continue;
            const auto span=activationSpans[input];
            const auto owner=activationChunk.find(span);
            if(span==0||owner==activationChunk.end())
                return std::unexpected(std::format(
                    "prepared command {} has no emitted activation owner",
                    geometry.commands[input].id));
            result->activations.push_back({input,span,owner->second});
        }
        std::ranges::sort(result->activations,{},[](const TimedCommandActivation &activation) {
            return std::pair{activation.chunk,activation.input};
        });
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
