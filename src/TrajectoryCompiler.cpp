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
#include <vector>

#include <highs/Highs.h>
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

        struct SparseLpBuilder {
            HighsLp model;

            explicit SparseLpBuilder(const std::size_t variables) {
                model.num_col_=static_cast<HighsInt>(variables);
                model.num_row_=0;
                model.sense_=ObjSense::kMinimize;
                model.offset_=0.0;
                model.col_cost_.assign(variables,0.0);
                model.col_lower_.assign(variables,-kHighsInf);
                model.col_upper_.assign(variables,kHighsInf);
                model.a_matrix_.format_=MatrixFormat::kRowwise;
                model.a_matrix_.start_.assign(1,0);
            }

            void addRow(const double lower,const double upper,
                        const std::initializer_list<std::pair<std::size_t,double>> entries) {
                for(const auto &[column,value]:entries) {
                    if(std::abs(value)<=1e-18) continue;
                    if(column>=static_cast<std::size_t>(model.num_col_))
                        PANIC("SCP LP row references an invalid column");
                    model.a_matrix_.index_.push_back(static_cast<HighsInt>(column));
                    model.a_matrix_.value_.push_back(value);
                }
                model.a_matrix_.start_.push_back(
                    static_cast<HighsInt>(model.a_matrix_.index_.size()));
                model.row_lower_.push_back(lower);
                model.row_upper_.push_back(upper);
                ++model.num_row_;
            }
        };

        template<typename T>
        std::pair<std::array<T, 4>, std::array<T, 4>> splitBezier(const std::array<T, 4> &control) {
            const auto middle = [](const T &a, const T &b) {
                if constexpr(std::same_as<T, double>) return std::midpoint(a, b);
                else return midpoint(a, b);
            };
            const auto p01 = middle(control[0], control[1]);
            const auto p12 = middle(control[1], control[2]);
            const auto p23 = middle(control[2], control[3]);
            const auto p012 = middle(p01, p12);
            const auto p123 = middle(p12, p23);
            const auto p0123 = middle(p012, p123);
            return {{ control[0], p01, p012, p0123 }, { p0123, p123, p23, control[3] }};
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
            bool ruckigBrakePhase=false;
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
            ruckig::InputParameter<1> input;
            ruckig::Ruckig<1> generator;
            ruckig::Trajectory<1> trajectory;
        };

        enum class TimeLawPurpose {
            ExactStop,
            ContinuousSeed,
        };

        using TimeLawInputKey=std::array<std::uint64_t,8>;

        struct TimeLawInputHash {
            std::size_t operator()(const TimeLawInputKey &values) const {
                auto result=std::size_t {0xcbf29ce484222325ULL};
                for(const auto value:values) {
                    result^=static_cast<std::size_t>(value);
                    result*=std::size_t {0x100000001b3ULL};
                    if constexpr(sizeof(std::size_t)<sizeof(value))
                        result^=static_cast<std::size_t>(value>>32U);
                }
                return result;
            }
        };

        TimeLawInputKey timeLawInputKey(const double length,const double fromVelocity,
                const double fromAcceleration,const double toVelocity,
                const double toAcceleration,const double requestedVelocity,
                const double acceleration,const double jerk) {
            return {
                std::bit_cast<std::uint64_t>(length),
                std::bit_cast<std::uint64_t>(fromVelocity),
                std::bit_cast<std::uint64_t>(fromAcceleration),
                std::bit_cast<std::uint64_t>(toVelocity),
                std::bit_cast<std::uint64_t>(toAcceleration),
                std::bit_cast<std::uint64_t>(requestedVelocity),
                std::bit_cast<std::uint64_t>(acceleration),
                std::bit_cast<std::uint64_t>(jerk),
            };
        }

        struct TimeLawCacheEntry {
            TimeLawInputKey key {};
            double duration=0.0;
            bool successful=false;
            bool valid=false;
        };

        constexpr std::size_t SMALL_TIME_LAW_CACHE_SIZE=32768;
        // A 128K direct-mapped table is about 11 MiB and was the measured knee
        // for large horizons. A 256K table removed more solver calls but did not
        // improve wall time. Smaller rolling probes avoid initializing it.
        constexpr std::size_t LARGE_TIME_LAW_CACHE_SIZE=131072;

        TimeLawCacheEntry *threadTimeLawCache() {
            thread_local auto cache=
                std::make_unique<TimeLawCacheEntry[]>(LARGE_TIME_LAW_CACHE_SIZE);
            return cache.get();
        }

        struct TimeLawInstrumentation {
            TimeLawDiagnostics diagnostics;
            std::unique_ptr<TimeLawCacheEntry[]> cache;
            std::size_t cacheSize=SMALL_TIME_LAW_CACHE_SIZE;
            bool shareAcrossCompilations=false;

            void configureCache(const std::size_t requestedSize,const bool share) {
                if(cache) PANIC("time-law cache configured after first use");
                if(requestedSize==0||(requestedSize&(requestedSize-1))!=0)
                    PANIC("time-law cache size must be a power of two");
                cacheSize=requestedSize;
                shareAcrossCompilations=share;
            }

            TimeLawCacheEntry *entries() {
                if(shareAcrossCompilations) return threadTimeLawCache();
                if(!cache) cache=std::make_unique<TimeLawCacheEntry[]>(cacheSize);
                return cache.get();
            }

            TimeLawCallDiagnostics &forPurpose(const TimeLawPurpose purpose) {
                switch(purpose) {
                    case TimeLawPurpose::ExactStop: return diagnostics.exactStop;
                    case TimeLawPurpose::ContinuousSeed: return diagnostics.continuousSeed;
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

            TimeLawCacheEntry *find(const TimeLawInputKey &key,
                                    TimeLawCallDiagnostics &diagnosticsForPurpose) {
                auto &entry=entries()[TimeLawInputHash{}(key)&(cacheSize-1)];
                if(entry.valid&&entry.key==key) {
                    ++diagnosticsForPurpose.cacheHits;
                    if(entry.successful) ++diagnosticsForPurpose.cacheSuccessfulHits;
                    else ++diagnosticsForPurpose.cacheFailureHits;
                    return &entry;
                }
                ++diagnosticsForPurpose.cacheMisses;
                if(entry.valid) ++diagnosticsForPurpose.cacheCollisions;
                return nullptr;
            }

            void store(const TimeLawInputKey &key,const bool successful,
                       const double duration) {
                auto &entry=entries()[TimeLawInputHash{}(key)&(cacheSize-1)];
                entry={key,duration,successful,true};
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

        template<typename PositionAt>
        bool verifiesOrderedCurveTolerance(const AxisPolynomialSpan &span,const double distance0,
                                           const double distance1,const double tolerance,
                                           const PositionAt &positionAt,const auto &referenceErrorAt) {
            std::size_t visitedNodes=0;
            constexpr std::size_t MAX_VISITED_NODES=64;
            const auto verify=[&](const auto &self,const std::array<position_t,4> &curve,
                                  const double from,const double to,const unsigned depth) -> bool {
                if(++visitedNodes>MAX_VISITED_NODES) return false;
                const auto source0=positionAt(from);
                const auto source1=positionAt(to);
                const auto available=tolerance-referenceErrorAt(from,to);
                if(available>=0.0&&std::ranges::all_of(curve,[&](const position_t &control) {
                    return distanceToSegment(control,source0,source1)<=available;
                })) return true;
                if(depth>=20) return false;
                const auto [left,right]=splitBezier(curve);
                const auto middle=std::midpoint(from,to);
                return self(self,left,from,middle,depth+1)
                    &&self(self,right,middle,to,depth+1);
            };
            return verify(verify,bezierControls(span),distance0,distance1,0);
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

        constexpr std::array AXIS_COMPONENTS {
            &position_t::x, &position_t::y, &position_t::z,
            &position_t::a, &position_t::b, &position_t::c,
        };
        constexpr std::array AXIS_NAMES {"X","Y","Z","A","B","C"};

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
            if(length<=0.0||fromVelocity<0.0||toVelocity<0.0||requestedVelocity<=0.0
               ||std::abs(fromAcceleration)>acceleration*(1.0+1e-10)
               ||std::abs(toAcceleration)>acceleration*(1.0+1e-10)
               ||acceleration<=0.0||jerk<=0.0)
                return std::unexpected("local time law received invalid distance, state, or limits");
            if(std::isinf(acceleration)) {
                if(std::abs(fromVelocity-toVelocity)>1e-12||fromVelocity<=0.0
                   ||std::abs(fromAcceleration)>1e-12||std::abs(toAcceleration)>1e-12)
                    return std::unexpected(
                        "unbounded-acceleration local time law requires equal positive speeds and zero acceleration");
                return TimeLaw {
                    {0.0,0.0,fromVelocity,0.0},
                    {length/fromVelocity,length,toVelocity,0.0},
                };
            }
            auto &input=workspace.input;
            input.current_position={0.0};
            input.current_velocity={fromVelocity};
            input.current_acceleration={fromAcceleration};
            input.target_position={length};
            input.target_velocity={toVelocity};
            input.target_acceleration={toAcceleration};
            input.max_velocity = {requestedVelocity}; input.max_acceleration = {acceleration}; input.max_jerk = {jerk};
            auto &generator=workspace.generator;
            auto &trajectory=workspace.trajectory;
            if(generator.calculate(input, trajectory) != ruckig::Result::Working)
                return std::unexpected(std::format(
                    "Ruckig failed local position timing: length={} state (v={}, a={}) -> "
                    "(v={}, a={}) limits v={} a={} j={}",length,fromVelocity,fromAcceleration,
                    toVelocity,toAcceleration,requestedVelocity,acceleration,jerk));
            const auto profiles=trajectory.get_profiles();
            const auto &profile=profiles.front().front();
            TimeLaw result {{
                0.0,0.0,fromVelocity,fromAcceleration,0.0}};
            auto phaseTime=0.0;
            const auto appendPhase=[&](const double duration,const double phaseJerk,
                                       const bool ruckigBrakePhase=false) {
                if(duration<=1e-12) return;
                auto &from=result.back();
                from.jerk=phaseJerk;
                from.ruckigBrakePhase=ruckigBrakePhase;
                phaseTime+=duration;
                result.push_back({
                    phaseTime,
                    from.distance+from.velocity*duration
                        +0.5*from.acceleration*duration*duration
                        +phaseJerk*duration*duration*duration/6.0,
                    from.velocity+from.acceleration*duration
                        +0.5*phaseJerk*duration*duration,
                    from.acceleration+phaseJerk*duration,
                    0.0,
                });
            };
            for(std::size_t phase=0;phase<profile.brake.t.size();++phase)
                appendPhase(profile.brake.t[phase],profile.brake.j[phase],true);
            for(std::size_t phase=0;phase<profile.t.size();++phase) {
                appendPhase(profile.t[phase],profile.j[phase]);
            }
            appendPhase(trajectory.get_duration()-phaseTime,0.0);
            result.front().time=0.0;
            result.front().distance=0.0;
            result.front().velocity=fromVelocity;
            result.front().acceleration=fromAcceleration;
            result.back().time=trajectory.get_duration();
            result.back().distance=length;
            result.back().velocity=toVelocity;
            result.back().acceleration=toAcceleration;
            result.back().jerk=0.0;
            const auto distanceTolerance=std::max(1e-12,length*1e-9);
            const auto velocityTolerance=std::max(1e-12,requestedVelocity*1e-10);
            for(std::size_t boundary=0;boundary<result.size();++boundary) {
                const auto &state=result[boundary];
                if(!std::isfinite(state.time)||!std::isfinite(state.distance)
                   ||!std::isfinite(state.velocity)||!std::isfinite(state.acceleration)
                   ||state.distance < -distanceTolerance
                   ||state.distance > length+distanceTolerance
                   ||state.velocity < -distanceTolerance
                   ||(boundary>0&&(state.time<=result[boundary-1].time
                       ||state.distance<result[boundary-1].distance-distanceTolerance)))
                    return std::unexpected(std::format(
                        "Ruckig produced a non-monotone local path law at boundary {} of {}: "
                        "time={} distance={} velocity={} acceleration={} previous time={} "
                        "previous distance={} length={} tolerance={}",boundary,result.size(),
                        state.time,state.distance,state.velocity,state.acceleration,
                        boundary?result[boundary-1].time:0.0,
                        boundary?result[boundary-1].distance:0.0,length,distanceTolerance));
            }
            for(std::size_t phase=1;phase<result.size();++phase) {
                const auto &from=result[phase-1];
                const auto duration=result[phase].time-from.time;
                auto minimumVelocity=std::min(from.velocity,result[phase].velocity);
                if(std::abs(from.jerk)>1e-15) {
                    const auto extremum=-from.acceleration/from.jerk;
                    if(extremum>0.0&&extremum<duration)
                        minimumVelocity=std::min(minimumVelocity,from.velocity
                            +from.acceleration*extremum
                            +0.5*from.jerk*extremum*extremum);
                }
                if(minimumVelocity < -velocityTolerance)
                    return std::unexpected(std::format(
                        "Ruckig local position timing reverses path direction in phase {}: "
                        "minimum_velocity={} tolerance={} duration={} length={} "
                        "state (v={}, a={}) -> (v={}, a={}) limits v={} a={} j={}",
                        phase-1,minimumVelocity,velocityTolerance,duration,length,
                        fromVelocity,fromAcceleration,toVelocity,toAcceleration,
                        requestedVelocity,acceleration,jerk));
            }
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

        struct CandidateTimeLaw {
            std::optional<TimeLaw> timing;
            TimeLawInputKey key {};
            double duration=0.0;
            bool successful=false;
            bool cacheHit=false;
        };

        CandidateTimeLaw candidateTimeLawBetween(TimeLawWorkspace &workspace,
                TimeLawInstrumentation &instrumentation,const bool useCache,
                const TimeLawPurpose purpose,
                const bool correctionPass,
                const double length,const double fromVelocity,const double fromAcceleration,
                const double toVelocity,const double toAcceleration,
                const double requestedVelocity,const double acceleration,const double jerk) {
            auto &diagnostics=instrumentation.begin(purpose,correctionPass);
            TimeLawCallTimer timer {diagnostics};
            const auto key=timeLawInputKey(length,fromVelocity,fromAcceleration,toVelocity,
                toAcceleration,requestedVelocity,acceleration,jerk);
            if(useCache) {
                if(const auto *cached=instrumentation.find(key,diagnostics)) {
                    timer.succeeded=cached->successful;
                    return {.timing=std::nullopt,.key=key,.duration=cached->duration,
                        .successful=cached->successful,.cacheHit=true};
                }
            }
            ++diagnostics.solverCalls;
            auto result=solveTimeLawBetween(workspace,length,fromVelocity,fromAcceleration,
                toVelocity,toAcceleration,requestedVelocity,acceleration,jerk);
            if(!result) {
                if(useCache) instrumentation.store(key,false,0.0);
                return {.timing=std::nullopt,.key=key,.duration=0.0,
                    .successful=false,.cacheHit=false};
            }
            const auto duration=result->back().time;
            if(useCache) instrumentation.store(key,true,duration);
            timer.succeeded=true;
            return {.timing=*result,.key=key,.duration=duration,
                .successful=true,.cacheHit=false};
        }

        std::expected<TimeLaw,std::string> materializeCandidateTimeLaw(
                TimeLawWorkspace &workspace,
                TimeLawInstrumentation &instrumentation,const TimeLawPurpose purpose,
                const CandidateTimeLaw &candidate,
                const double length,const double fromVelocity,const double fromAcceleration,
                const double toVelocity,const double toAcceleration,
                const double requestedVelocity,const double acceleration,const double jerk) {
            const auto key=timeLawInputKey(length,fromVelocity,fromAcceleration,toVelocity,
                toAcceleration,requestedVelocity,acceleration,jerk);
            if(!candidate.successful)
                return std::unexpected("cannot materialize an unsuccessful cached time law");
            if(candidate.key!=key)
                return std::unexpected("cached time-law winner does not match its materialization key");
            auto &diagnostics=instrumentation.forPurpose(purpose);
            ++diagnostics.cacheMaterializations;
            ++diagnostics.solverCalls;
            const auto started=std::chrono::steady_clock::now();
            auto result=solveTimeLawBetween(workspace,length,fromVelocity,fromAcceleration,
                toVelocity,toAcceleration,requestedVelocity,acceleration,jerk);
            diagnostics.seconds+=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-started).count();
            if(!result)
                return std::unexpected(
                    "cached successful time-law winner failed exact materialization");
            if(std::bit_cast<std::uint64_t>(result->back().time)
                    !=std::bit_cast<std::uint64_t>(candidate.duration))
                return std::unexpected(
                    "cached time-law winner duration changed during exact materialization");
            return *result;
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

        double velocityTransitionDistance(const double fromVelocity,const double toVelocity,
                                          const double acceleration,const double jerk) {
            const auto change=std::abs(toVelocity-fromVelocity);
            if(change<=1e-15) return 0.0;
            const auto threshold=acceleration*acceleration/jerk;
            const auto duration=change<=threshold
                ? 2.0*std::sqrt(change/jerk)
                : change/acceleration+acceleration/jerk;
            return std::midpoint(fromVelocity,toVelocity)*duration;
        }

        double reachableVelocity(const double fixedVelocity,const double cap,const double length,
                                 const double acceleration,const double jerk) {
            if(cap<=fixedVelocity) return cap;
            // Leave a small positive cruise-distance reserve. Asking Ruckig for
            // a transition at the exact minimum-distance boundary is numerically
            // ambiguous and can select a much longer reversing profile.
            const auto available=std::max(0.0,length-std::max(1e-12,length*1e-6));
            if(velocityTransitionDistance(fixedVelocity,cap,acceleration,jerk)<=available) return cap;
            auto low=fixedVelocity;
            auto high=cap;
            for(unsigned iteration=0;iteration<52;++iteration) {
                const auto middle=std::midpoint(low,high);
                if(velocityTransitionDistance(fixedVelocity,middle,acceleration,jerk)<=available)
                    low=middle;
                else high=middle;
            }
            return low;
        }

        double positionDot(const position_t &left, const position_t &right) {
            return left.x*right.x + left.y*right.y + left.z*right.z
                + left.a*right.a + left.b*right.b + left.c*right.c;
        }

        struct CoupledEndpointGeometry {
            position_t tangent;
            position_t curvature;
            position_t curvatureDerivative;
        };

        enum class CoupledEndpointResult { Feasible, Acceleration, Jerk };

       struct GeometryPiece {
            std::size_t input=0;
            std::vector<std::size_t> activationInputs{};
            double length=0.0;
            double speed=0.0;
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
        if(preparedPiece && (!preparedPiece->curve
           ||(!preparedPiece->geometricallyLinear && preparedPiece->length() <= 1e-12)))
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
                        if(preparedPiece->geometricallyLinear) return true;
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
                            !preparedPiece->geometricallyLinear, accept)) return result;
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
           ||m_continuousPlanningEffort.scpIterations==0
           ||m_continuousPlanningEffort.scpIterations>32
           ||m_continuousPlanningEffort.scpLineSearchSteps==0
           ||m_continuousPlanningEffort.scpLineSearchSteps>24
           ||(m_continuousPlanningEffort.timeLawCacheEntries!=0
                &&((m_continuousPlanningEffort.timeLawCacheEntries
                        &(m_continuousPlanningEffort.timeLawCacheEntries-1))!=0
                   ||m_continuousPlanningEffort.timeLawCacheEntries
                        >LARGE_TIME_LAW_CACHE_SIZE))
           ||m_continuousPlanningEffort.scpSimplexIterationLimitMultiplier==0
           ||m_continuousPlanningEffort.scpSimplexIterationLimitMultiplier>4096
           ||!std::isfinite(m_continuousPlanningEffort.scpVelocityTrustFraction)
           ||m_continuousPlanningEffort.scpVelocityTrustFraction<=0.0
           ||m_continuousPlanningEffort.scpVelocityTrustFraction>1.0
           ||!std::isfinite(m_continuousPlanningEffort.scpAccelerationTrustFraction)
           ||m_continuousPlanningEffort.scpAccelerationTrustFraction<=0.0
           ||m_continuousPlanningEffort.scpAccelerationTrustFraction>2.0
           ||!std::isfinite(m_continuousPlanningEffort.scpSolveTimeLimit)
           ||m_continuousPlanningEffort.scpSolveTimeLimit<=0.0
           ||m_continuousPlanningEffort.scpSolveTimeLimit>60.0
           ||!std::isfinite(m_continuousPlanningEffort
                .curvatureDerivativeVelocityCapMultiplier)
           ||m_continuousPlanningEffort.curvatureDerivativeVelocityCapMultiplier<=0.0)
            return std::unexpected("continuous planning effort is outside its bounded range");
        reportProgress();

        std::vector<GeometryPiece> pieces;
        auto timingPieceCount=std::size_t{0};
        for(const auto &prepared:geometry.pieces)
            timingPieceCount+=prepared.kind==PreparedPieceKind::ClusterSpline
                ?prepared.clusterKnotIntervals.size():1;
        pieces.reserve(timingPieceCount);
        auto workspace=std::make_shared<CurveEvaluationWorkspace>();
        const auto inputFor=[&](const PreparedCommandId id)
                ->std::expected<std::size_t,std::string> {
            const auto found=std::ranges::find_if(geometry.commands,
                [id](const auto &command) { return command.id==id; });
            if(found==geometry.commands.end())
                return std::unexpected(std::format(
                    "prepared piece references unknown command {}",id));
            return static_cast<std::size_t>(found-geometry.commands.begin());
        };
        for(const auto &prepared:geometry.pieces) {
            if(!prepared.curve||prepared.length()<=1e-12||prepared.programmedFeed<=0.0)
                return std::unexpected("prepared continuous path contains an invalid piece");
            if(prepared.geometricSamples.size()<2)
                return std::unexpected(std::format(
                    "prepared continuous piece {} has no usable geometric samples",
                    prepared.id));
            const auto primaryId=prepared.activationCommands.empty()
                ?prepared.primaryCommand:prepared.activationCommands.front();
            const auto primary=inputFor(primaryId);
            if(!primary) return std::unexpected(primary.error());
            std::vector<std::size_t> activations;
            for(const auto id:prepared.activationCommands) {
                const auto input=inputFor(id);
                if(!input) return std::unexpected(input.error());
                if(*input!=*primary) activations.push_back(*input);
            }
            const auto appendTimingPiece=[&](
                    const double from,const double to,const double speed,
                    const std::span<const PreparedGeometricSample> samples,
                    std::vector<std::size_t> activationInputs)
                    ->std::expected<void,std::string> {
                const auto length=to-from;
                const auto sampleOffset=from-prepared.curveFrom;
                if(!std::isfinite(length)||length<=1e-12
                   ||!std::isfinite(speed)||speed<=0.0)
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

                const auto curve=prepared.curve;
                pieces.push_back({
                    .input=*primary,
                    .activationInputs=std::move(activationInputs),
                    .length=length,
                    .speed=speed,
                    .linear=prepared.geometricallyLinear,
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
                    .curvatureDerivativeAt=[curve,workspace,from,length](const double distance) {
                        return curvatureDerivativeAtDistance(*curve,
                            from+std::clamp(distance,0.0,length),*workspace);
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

            if(prepared.kind!=PreparedPieceKind::ClusterSpline) {
                if(auto appended=appendTimingPiece(prepared.curveFrom,prepared.curveTo,
                        prepared.programmedFeed,prepared.geometricSamples,
                        std::move(activations)); !appended)
                    return std::unexpected(appended.error());
                continue;
            }
            if(prepared.clusterKnotIntervals.empty())
                return std::unexpected(std::format(
                    "prepared cluster spline {} has no knot intervals",prepared.id));
            const auto expectedSampleCount=16*prepared.clusterKnotIntervals.size()+1;
            if(prepared.geometricSamples.size()!=expectedSampleCount)
                return std::unexpected(std::format(
                    "prepared cluster spline {} has {} geometric samples; expected {}",
                    prepared.id,prepared.geometricSamples.size(),expectedSampleCount));
            auto expectedFrom=prepared.curveFrom;
            for(std::size_t intervalIndex=0;
                    intervalIndex<prepared.clusterKnotIntervals.size();++intervalIndex) {
                const auto &interval=prepared.clusterKnotIntervals[intervalIndex];
                if(std::abs(interval.curveFrom-expectedFrom)>1e-10
                   ||interval.curveTo<=interval.curveFrom)
                    return std::unexpected(std::format(
                        "prepared cluster spline {} knot interval {} is not contiguous",
                        prepared.id,intervalIndex));
                if(interval.geometricSampleCount!=17
                   ||interval.firstGeometricSample>prepared.geometricSamples.size()
                   ||interval.geometricSampleCount
                        >prepared.geometricSamples.size()-interval.firstGeometricSample)
                    return std::unexpected(std::format(
                        "prepared cluster spline {} knot interval {} does not provide 17 samples",
                        prepared.id,intervalIndex));
                const auto samples=std::span{prepared.geometricSamples}.subspan(
                    interval.firstGeometricSample,interval.geometricSampleCount);
                auto intervalActivations=intervalIndex==0
                    ?std::move(activations):std::vector<std::size_t>{};
                if(auto appended=appendTimingPiece(interval.curveFrom,interval.curveTo,
                        interval.programmedFeed,samples,std::move(intervalActivations)); !appended)
                    return std::unexpected(appended.error());
                expectedFrom=interval.curveTo;
            }
            if(std::abs(expectedFrom-prepared.curveTo)>1e-10)
                return std::unexpected(std::format(
                    "prepared cluster spline {} knot intervals do not cover the curve",
                    prepared.id));
        }
        if(pieces.empty()) return std::unexpected("continuous path produced no geometry");
        const auto automaticTimeLawCacheSize=
            pieces.size()>1024?LARGE_TIME_LAW_CACHE_SIZE:SMALL_TIME_LAW_CACHE_SIZE;
        timeLawRecorder.instrumentation.configureCache(
            m_continuousPlanningEffort.timeLawCacheEntries==0
                ?automaticTimeLawCacheSize
                :m_continuousPlanningEffort.timeLawCacheEntries,
            m_continuousPlanningEffort.shareTimeLawCacheAcrossCompilations);

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
                .velocityLimit=piece.speed,
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
        std::vector<AxisPolynomialSpan> normalSpans;
        std::vector<std::size_t> normalSpanPieces;
        const auto maximumStagedNormalSpans=std::max<std::size_t>(8192,pieces.size()*8);
        const auto maximumGeometryVerificationAttemptsPerPass=
            std::max<std::size_t>(8192,pieces.size()*4);
        const auto maximumTotalGeometryVerificationAttempts=
            std::max<std::size_t>(32768,pieces.size()
                *m_continuousPlanningEffort.geometryVerificationBudgetMultiplier);
        auto nextChunk=m_nextChunk;
        auto nextSpan=m_nextSpan;
        std::vector<SpanId> activationSpans(geometry.commands.size(),0);

        struct LocalLimits {
            double velocity;
            double acceleration;
            double jerk;
            ContinuousVelocityLimitCause velocityCause=
                ContinuousVelocityLimitCause::ProgrammedFeed;
        };
        std::vector<LocalLimits> localLimits;
        localLimits.reserve(pieces.size());
        struct CurvatureDerivativeDiagnostic {
            double distance=0.0;
            double analyticMagnitude=0.0;
            double curvatureMagnitude=0.0;
            double tangentialMagnitude=0.0;
            double normalMagnitude=0.0;
            double coarseMagnitude=0.0;
            double fineMagnitude=0.0;
            double coarseStep=0.0;
            double fineStep=0.0;
        };
        std::vector<CurvatureDerivativeDiagnostic> curvatureDerivativeDiagnostics;
        curvatureDerivativeDiagnostics.reserve(pieces.size());
        for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
            if((pieceIndex&15U)==0) reportProgress();
            const auto &piece=pieces[pieceIndex];
            LocalLimits limits{piece.speed,m_limits.pathAcceleration,m_limits.pathJerk};
            const auto limitVelocity=[&](const double candidate,
                                         const ContinuousVelocityLimitCause cause) {
                if(candidate<limits.velocity) {
                    limits.velocity=candidate;
                    limits.velocityCause=cause;
                }
            };
            CurvatureDerivativeDiagnostic derivativeDiagnostic;
            for(const auto &geometric:piece.geometricSamples) {
                const auto distance=geometric.distance-piece.geometricSampleDistanceOffset;
                const PathSample sample{geometric.position,geometric.tangent};
                const auto &curvature=geometric.curvature;
                const auto &curvatureDerivative=geometric.curvatureDerivative;
                const auto derivativeMagnitude=curvatureDerivative.length();
                if(derivativeMagnitude>derivativeDiagnostic.analyticMagnitude) {
                    derivativeDiagnostic.distance=distance;
                    derivativeDiagnostic.analyticMagnitude=derivativeMagnitude;
                }
                for(const auto component:AXIS_COMPONENTS) {
                    const auto tangent=std::abs(sample.tangent.*component);
                    if(tangent>1e-15) {
                        limitVelocity(m_limits.axisVelocity.*component/tangent,
                            ContinuousVelocityLimitCause::AxisVelocity);
                        limits.acceleration=std::min(limits.acceleration,
                            m_limits.axisAcceleration.*component/tangent);
                        limits.jerk=std::min(limits.jerk,m_limits.axisJerk.*component/tangent);
                    }
                    const auto axisCurvature=std::abs(curvature.*component);
                    if(axisCurvature>1e-15)
                        limitVelocity(std::sqrt(
                            m_limits.axisAcceleration.*component/axisCurvature),
                            ContinuousVelocityLimitCause::AxisCentripetalAcceleration);
                }
                const auto curvatureMagnitude=curvature.length();
                if(curvatureMagnitude>1e-15)
                    limitVelocity(std::sqrt(m_limits.pathAcceleration/curvatureMagnitude),
                        ContinuousVelocityLimitCause::PathCentripetalAcceleration);
                if(m_continuousPlanningEffort.applyCurvatureDerivativeVelocityCap) {
                    const auto derivativeMultiplier=m_continuousPlanningEffort
                        .curvatureDerivativeVelocityCapMultiplier;
                    if(derivativeMagnitude>1e-15)
                        limitVelocity(derivativeMultiplier
                                *std::cbrt(m_limits.pathJerk/derivativeMagnitude),
                            ContinuousVelocityLimitCause::PathCurvatureDerivativeJerk);
                    for(const auto component:AXIS_COMPONENTS) {
                        const auto axisDerivative=std::abs(curvatureDerivative.*component);
                        if(axisDerivative>1e-15)
                            limitVelocity(derivativeMultiplier*std::cbrt(
                                    m_limits.axisJerk.*component/axisDerivative),
                                ContinuousVelocityLimitCause::AxisCurvatureDerivativeJerk);
                    }
                }
            }
            const auto finiteDifferenceMagnitude=[&](const double requestedStep) {
                const auto step=std::clamp(requestedStep,1e-10,piece.length);
                const auto distance=derivativeDiagnostic.distance;
                position_t difference{};
                double denominator=0.0;
                if(distance<=step) {
                    difference=subtract(piece.curvatureAt(std::min(piece.length,distance+step)),
                                        piece.curvatureAt(distance));
                    denominator=std::min(piece.length,distance+step)-distance;
                } else if(piece.length-distance<=step) {
                    difference=subtract(piece.curvatureAt(distance),
                                        piece.curvatureAt(std::max(0.0,distance-step)));
                    denominator=distance-std::max(0.0,distance-step);
                } else {
                    difference=subtract(piece.curvatureAt(distance+step),
                                        piece.curvatureAt(distance-step));
                    denominator=2.0*step;
                }
                return denominator>1e-15?scaled(difference,1.0/denominator).length():0.0;
            };
            if(m_continuousPlanningEffort.measureCurvatureDerivativeNumerics) {
                derivativeDiagnostic.coarseStep=std::max(1e-9,piece.length*1e-4);
                derivativeDiagnostic.fineStep=std::max(1e-10,piece.length*1e-5);
                derivativeDiagnostic.coarseMagnitude=
                    finiteDifferenceMagnitude(derivativeDiagnostic.coarseStep);
                derivativeDiagnostic.fineMagnitude=
                    finiteDifferenceMagnitude(derivativeDiagnostic.fineStep);
                const auto sample=piece.sampleAt(derivativeDiagnostic.distance);
                const auto curvature=piece.curvatureAt(derivativeDiagnostic.distance);
                const auto derivative=piece.curvatureDerivativeAt(
                    derivativeDiagnostic.distance);
                const auto tangential=positionDot(sample.tangent,derivative);
                derivativeDiagnostic.curvatureMagnitude=curvature.length();
                derivativeDiagnostic.tangentialMagnitude=std::abs(tangential);
                derivativeDiagnostic.normalMagnitude=
                    subtract(derivative,scaled(sample.tangent,tangential)).length();
            }
            if(limits.velocity<=0.0||limits.acceleration<=0.0||limits.jerk<=0.0
               ||!std::isfinite(limits.velocity)||!std::isfinite(limits.acceleration)
               ||!std::isfinite(limits.jerk))
                return std::unexpected(std::format(
                    "continuous piece {} for input {} has invalid local limits v={} a={} j={}",
                    pieceIndex,piece.input,limits.velocity,limits.acceleration,limits.jerk));
            localLimits.push_back(limits);
            curvatureDerivativeDiagnostics.push_back(derivativeDiagnostic);
        }
        const auto initialLocalLimits=localLimits;

        // These one-sided endpoint values are geometry-only and remain valid
        // through every local-limit correction pass. Candidate evaluation used
        // to repeat the exact inverse/sample path for both sides of every
        // station even though only scalar velocity and acceleration changed.
        std::vector<CoupledEndpointGeometry> pieceStartGeometry;
        std::vector<CoupledEndpointGeometry> pieceEndGeometry;
        pieceStartGeometry.reserve(pieces.size());
        pieceEndGeometry.reserve(pieces.size());
        for(const auto &piece:pieces) {
            const auto &start=piece.geometricSamples.front();
            const auto &end=piece.geometricSamples.back();
            pieceStartGeometry.push_back({start.tangent,start.curvature,
                start.curvatureDerivative});
            pieceEndGeometry.push_back({end.tangent,end.curvature,
                end.curvatureDerivative});
        }
        timeLawRecorder.instrumentation.diagnostics.endpointFeasibility
            .cachedGeometryEndpoints+=2*pieces.size();

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
        TimeLawWorkspace timeLawWorkspace;
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
                   ||!bitExactDouble(a.jerk,b.jerk)
                   ||a.ruckigBrakePhase!=b.ruckigBrakePhase) return false;
            }
            return true;
        };
        std::optional<HighsBasis> reusableScpBasis;
        for(unsigned correctionPass=0;correctionPass<maximumLocalCorrectionPasses;
                ++correctionPass) {
            reportProgress();
            result->correctionPasses=correctionPass+1;
            std::vector<double> stationCaps(pieces.size()+1,std::numeric_limits<double>::infinity());
            stationCaps.front()=scalarStart->first;
            stationCaps.back()=scalarEnd->first;
            for(std::size_t station=1;station<pieces.size();++station)
                stationCaps[station]=std::min(localLimits[station-1].velocity,
                                              localLimits[station].velocity);
            if(stationCaps.front()>localLimits.front().velocity*(1.0+1e-10)
               ||stationCaps.back()>localLimits.back().velocity*(1.0+1e-10))
                return std::unexpected(std::format(
                    "continuous boundary velocity exceeds a local path cap: start={}/{} end={}/{}",
                    stationCaps.front(),localLimits.front().velocity,
                    stationCaps.back(),localLimits.back().velocity));
            std::vector<double> stationVelocity=stationCaps;
            for(unsigned reachabilityPass=0;reachabilityPass<8;++reachabilityPass) {
                auto maximumChange=0.0;
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    const auto reachable=reachableVelocity(stationVelocity[pieceIndex],
                        std::min(stationCaps[pieceIndex+1],localLimits[pieceIndex].velocity),
                        pieces[pieceIndex].length,localLimits[pieceIndex].acceleration,
                        localLimits[pieceIndex].jerk);
                    if(pieceIndex+1==pieces.size()) {
                        if(reachable+1e-10<stationVelocity.back())
                            return std::unexpected(std::format(
                                "continuous fixed end velocity is not forward reachable: "
                                "requested={} reachable={}",stationVelocity.back(),reachable));
                        continue;
                    }
                    const auto reduced=std::min(stationVelocity[pieceIndex+1],reachable);
                    maximumChange=std::max(maximumChange,stationVelocity[pieceIndex+1]-reduced);
                    stationVelocity[pieceIndex+1]=reduced;
                }
                for(std::size_t pieceIndex=pieces.size();pieceIndex-->0;) {
                    const auto reachable=reachableVelocity(stationVelocity[pieceIndex+1],
                        std::min(stationCaps[pieceIndex],localLimits[pieceIndex].velocity),
                        pieces[pieceIndex].length,localLimits[pieceIndex].acceleration,
                        localLimits[pieceIndex].jerk);
                    if(pieceIndex==0) {
                        if(reachable+1e-10<stationVelocity.front())
                            return std::unexpected(std::format(
                                "continuous fixed start velocity cannot reach the remaining path: "
                                "requested={} backward reachable={}",stationVelocity.front(),reachable));
                        continue;
                    }
                    const auto reduced=std::min(stationVelocity[pieceIndex],reachable);
                    maximumChange=std::max(maximumChange,stationVelocity[pieceIndex]-reduced);
                    stationVelocity[pieceIndex]=reduced;
                }
                if(maximumChange<=1e-11) break;
            }

            std::vector<double> stationAcceleration(pieces.size()+1,0.0);
            stationAcceleration.front()=scalarStart->second;
            stationAcceleration.back()=scalarEnd->second;
            std::vector<TimeLaw> pieceTiming(pieces.size());
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                if((pieceIndex&31U)==0) reportProgress();
                auto timing=timeLawBetween(timeLawWorkspace,timeLawRecorder.instrumentation,
                    TimeLawPurpose::ContinuousSeed,correctionPass>0,
                    pieces[pieceIndex].length,
                    stationVelocity[pieceIndex],0.0,stationVelocity[pieceIndex+1],0.0,
                    localLimits[pieceIndex].velocity,localLimits[pieceIndex].acceleration,
                    localLimits[pieceIndex].jerk);
                if(!timing) return std::unexpected(std::format(
                    "continuous reachability seed timing failed at piece {} input {}: {}",
                    pieceIndex,pieces[pieceIndex].input,timing.error()));
                pieceTiming[pieceIndex]=*timing;
            }
            result->velocityOnlySeedDuration=std::accumulate(
                pieceTiming.begin(),pieceTiming.end(),0.0,
                [](const double total,const auto &timing) { return total+timing.back().time; });

            const auto stateWithinCoupledLimits=[&](const CoupledEndpointGeometry &geometry,
                    const std::size_t pieceIndex,const double velocity,
                    const double acceleration) {
                const auto axisAcceleration=add(scaled(geometry.tangent,acceleration),
                    scaled(geometry.curvature,velocity*velocity));
                if(axisAcceleration.length()>m_limits.pathAcceleration*(1.0+1e-10))
                    return CoupledEndpointResult::Acceleration;
                for(const auto component:AXIS_COMPONENTS)
                    if(std::abs(axisAcceleration.*component)
                       >m_limits.axisAcceleration.*component*(1.0+1e-10))
                        return CoupledEndpointResult::Acceleration;

                const auto geometricJerk=add(
                    scaled(geometry.curvature,3.0*velocity*acceleration),
                    scaled(geometry.curvatureDerivative,velocity*velocity*velocity));
                auto minimumScalarJerk=-localLimits[pieceIndex].jerk;
                auto maximumScalarJerk=localLimits[pieceIndex].jerk;
                for(const auto component:AXIS_COMPONENTS) {
                    const auto tangent=geometry.tangent.*component;
                    const auto geometric=geometricJerk.*component;
                    const auto limit=m_limits.axisJerk.*component;
                    if(std::abs(tangent)<=1e-15) {
                        if(std::abs(geometric)>limit*(1.0+1e-10))
                            return CoupledEndpointResult::Jerk;
                        continue;
                    }
                    auto lower=(-limit-geometric)/tangent;
                    auto upper=(limit-geometric)/tangent;
                    if(lower>upper) std::swap(lower,upper);
                    minimumScalarJerk=std::max(minimumScalarJerk,lower);
                    maximumScalarJerk=std::min(maximumScalarJerk,upper);
                }
                if(minimumScalarJerk>maximumScalarJerk)
                    return CoupledEndpointResult::Jerk;
                const auto scalarJerk=std::clamp(-positionDot(geometry.tangent,geometricJerk),
                    minimumScalarJerk,maximumScalarJerk);
                return add(scaled(geometry.tangent,scalarJerk),geometricJerk).length()
                    <=m_limits.pathJerk*(1.0+1e-10)
                    ?CoupledEndpointResult::Feasible:CoupledEndpointResult::Jerk;
            };

            const auto feasibleEndpointScalarJerk=[&](
                    const CoupledEndpointGeometry &geometry,const std::size_t pieceIndex,
                    const double velocity,const double acceleration)
                    ->std::optional<double> {
                const auto geometricJerk=add(
                    scaled(geometry.curvature,3.0*velocity*acceleration),
                    scaled(geometry.curvatureDerivative,velocity*velocity*velocity));
                auto lower=-localLimits[pieceIndex].jerk;
                auto upper=localLimits[pieceIndex].jerk;
                for(const auto component:AXIS_COMPONENTS) {
                    const auto tangent=geometry.tangent.*component;
                    const auto geometric=geometricJerk.*component;
                    const auto limit=m_limits.axisJerk.*component;
                    if(!std::isfinite(limit)) continue;
                    if(std::abs(tangent)<=1e-15) {
                        if(std::abs(geometric)>limit*(1.0+1e-10)) return std::nullopt;
                        continue;
                    }
                    auto componentLower=(-limit-geometric)/tangent;
                    auto componentUpper=(limit-geometric)/tangent;
                    if(componentLower>componentUpper)
                        std::swap(componentLower,componentUpper);
                    lower=std::max(lower,componentLower);
                    upper=std::min(upper,componentUpper);
                }
                if(lower>upper) return std::nullopt;
                const auto result=std::clamp(-positionDot(geometry.tangent,geometricJerk),
                    lower,upper);
                if(add(scaled(geometry.tangent,result),geometricJerk).length()
                    >m_limits.pathJerk*(1.0+1e-10)) return std::nullopt;
                return result;
            };

            const auto materializeScpCandidate=[&](const std::vector<double> &velocity,
                    const std::vector<double> &acceleration,std::vector<TimeLaw> &timing,
                    double &duration) {
                if(velocity.size()!=pieces.size()+1||acceleration.size()!=velocity.size())
                    PANIC("SCP candidate station count does not match continuous geometry");
                for(std::size_t station=0;station<velocity.size();++station) {
                    if(station>0&&stateWithinCoupledLimits(pieceEndGeometry[station-1],
                            station-1,velocity[station],acceleration[station])
                                !=CoupledEndpointResult::Feasible) return false;
                    if(station<pieces.size()&&stateWithinCoupledLimits(
                            pieceStartGeometry[station],station,velocity[station],
                            acceleration[station])!=CoupledEndpointResult::Feasible) return false;
                }
                std::vector<TimeLaw> candidateTiming(pieces.size());
                auto candidateDuration=0.0;
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    ++result->scpMaterializationAttempts;
                    auto candidate=timeLawBetween(timeLawWorkspace,
                        timeLawRecorder.instrumentation,TimeLawPurpose::ContinuousSeed,
                        correctionPass>0,pieces[pieceIndex].length,
                        velocity[pieceIndex],acceleration[pieceIndex],
                        velocity[pieceIndex+1],acceleration[pieceIndex+1],
                        localLimits[pieceIndex].velocity,
                        localLimits[pieceIndex].acceleration,
                        localLimits[pieceIndex].jerk);
                    if(!candidate) return false;
                    candidateDuration+=candidate->back().time;
                    candidateTiming[pieceIndex]=*candidate;
                }
                timing=std::move(candidateTiming);
                duration=candidateDuration;
                return true;
            };

            // The velocity-only reachability envelope is a cheap feasible
            // reference for the sequential LP. Materialize its actual boundary
            // accelerations before asking HiGHS for an improving step.
            auto scpDuration=0.0;
            if(!materializeScpCandidate(stationVelocity,stationAcceleration,
                    pieceTiming,scpDuration))
                return std::unexpected(std::format(
                    "continuous SCP could not materialize its initial fixed-boundary "
                    "reference on correction pass {}",correctionPass));

            const auto velocityColumn=[](const std::size_t station) { return station; };
            const auto accelerationColumn=[stationCount=stationVelocity.size()](
                    const std::size_t station) { return stationCount+station; };
            const auto jerkColumn=[stationCount=stationVelocity.size()](
                    const std::size_t pieceIndex,const bool end) {
                return 2*stationCount+2*pieceIndex+(end?1U:0U);
            };
            const auto accelerationDeviationColumn=[stationCount=stationVelocity.size(),
                    pieceCount=pieces.size()](const std::size_t station) {
                return 2*stationCount+2*pieceCount+station;
            };
            const auto variableCount=3*stationVelocity.size()+2*pieces.size();
            const auto scpStarted=std::chrono::steady_clock::now();

            for(unsigned scpIteration=0;
                    scpIteration<m_continuousPlanningEffort.scpIterations;++scpIteration) {
                reportProgress();
                SparseLpBuilder lp(variableCount);
                const auto &referenceVelocity=stationVelocity;
                const auto &referenceAcceleration=stationAcceleration;

                std::vector<std::size_t> stationDeviationRowOffsets(
                    referenceVelocity.size());
                for(std::size_t station=0;station<referenceVelocity.size();++station) {
                    const auto vColumn=velocityColumn(station);
                    const auto aColumn=accelerationColumn(station);
                    auto velocityLower=std::max(0.0,referenceVelocity[station]
                        -m_continuousPlanningEffort.scpVelocityTrustFraction
                            *std::max(1e-6,stationCaps[station]));
                    auto velocityUpper=std::min(stationCaps[station],referenceVelocity[station]
                        +m_continuousPlanningEffort.scpVelocityTrustFraction
                            *std::max(1e-6,stationCaps[station]));
                    auto accelerationLimit=std::numeric_limits<double>::infinity();
                    if(station>0) accelerationLimit=std::min(accelerationLimit,
                        localLimits[station-1].acceleration);
                    if(station<pieces.size()) accelerationLimit=std::min(accelerationLimit,
                        localLimits[station].acceleration);
                    accelerationLimit*=0.95;
                    auto accelerationLower=std::max(-accelerationLimit,
                        referenceAcceleration[station]
                            -m_continuousPlanningEffort.scpAccelerationTrustFraction
                                *accelerationLimit);
                    auto accelerationUpper=std::min(accelerationLimit,
                        referenceAcceleration[station]
                            +m_continuousPlanningEffort.scpAccelerationTrustFraction
                                *accelerationLimit);
                    if(station==0) {
                        velocityLower=velocityUpper=scalarStart->first;
                        accelerationLower=accelerationUpper=scalarStart->second;
                    } else if(station+1==referenceVelocity.size()) {
                        velocityLower=velocityUpper=scalarEnd->first;
                        accelerationLower=accelerationUpper=scalarEnd->second;
                    }
                    lp.model.col_lower_[vColumn]=velocityLower;
                    lp.model.col_upper_[vColumn]=velocityUpper;
                    lp.model.col_lower_[aColumn]=accelerationLower;
                    lp.model.col_upper_[aColumn]=accelerationUpper;
                    const auto deviationColumn=accelerationDeviationColumn(station);
                    lp.model.col_lower_[deviationColumn]=0.0;
                    lp.model.col_upper_[deviationColumn]=kHighsInf;
                    lp.model.col_cost_[deviationColumn]=1e-6;

                    auto targetAcceleration=referenceAcceleration[station];
                    if(station>0&&station<pieces.size()) {
                        const auto leftSlope=(referenceVelocity[station]
                            *referenceVelocity[station]-referenceVelocity[station-1]
                                *referenceVelocity[station-1])
                            /(2.0*pieces[station-1].length);
                        const auto rightSlope=(referenceVelocity[station+1]
                            *referenceVelocity[station+1]-referenceVelocity[station]
                                *referenceVelocity[station])
                            /(2.0*pieces[station].length);
                        targetAcceleration=leftSlope*rightSlope>0.0
                            ?std::copysign(std::min(std::abs(leftSlope),
                                std::abs(rightSlope)),leftSlope):0.0;
                        targetAcceleration=std::clamp(targetAcceleration,
                            accelerationLower,accelerationUpper);
                    }
                    stationDeviationRowOffsets[station]=lp.model.num_row_;
                    lp.addRow(-targetAcceleration,kHighsInf,{
                        {aColumn,-1.0},{deviationColumn,1.0},
                    });
                    lp.addRow(targetAcceleration,kHighsInf,{
                        {aColumn,1.0},{deviationColumn,1.0},
                    });
                }
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    for(const auto end:{false,true}) {
                        const auto column=jerkColumn(pieceIndex,end);
                        lp.model.col_lower_[column]=-localLimits[pieceIndex].jerk;
                        lp.model.col_upper_[column]=localLimits[pieceIndex].jerk;
                    }
                    const auto speedSum=std::max(1e-6,
                        referenceVelocity[pieceIndex]+referenceVelocity[pieceIndex+1]);
                    const auto objectiveDerivative=
                        -2.0*pieces[pieceIndex].length/(speedSum*speedSum);
                    lp.model.col_cost_[velocityColumn(pieceIndex)]+=objectiveDerivative;
                    lp.model.col_cost_[velocityColumn(pieceIndex+1)]+=objectiveDerivative;
                }

                const auto addAccelerationConstraints=[&](
                        const CoupledEndpointGeometry &geometry,const std::size_t station,
                        const double referenceV) {
                    const auto vColumn=velocityColumn(station);
                    const auto aColumn=accelerationColumn(station);
                    for(std::size_t axis=0;axis<AXIS_COMPONENTS.size();++axis) {
                        const auto component=AXIS_COMPONENTS[axis];
                        const auto velocityCoefficient=
                            2.0*geometry.curvature.*component*referenceV;
                        const auto accelerationCoefficient=geometry.tangent.*component;
                        const auto constant=-(geometry.curvature.*component)
                            *referenceV*referenceV;
                        const auto limit=m_limits.axisAcceleration.*component;
                        if(!std::isfinite(limit)) continue;
                        lp.addRow(-limit-constant,limit-constant,{
                            {vColumn,velocityCoefficient},
                            {aColumn,accelerationCoefficient},
                        });
                    }
                    const auto referenceVector=add(
                        scaled(geometry.tangent,referenceAcceleration[station]),
                        scaled(geometry.curvature,referenceV*referenceV));
                    const auto referenceNorm=referenceVector.length();
                    if(referenceNorm>1e-12) {
                        const auto direction=scaled(referenceVector,1.0/referenceNorm);
                        const auto velocityCoefficient=
                            2.0*positionDot(direction,geometry.curvature)*referenceV;
                        const auto accelerationCoefficient=
                            positionDot(direction,geometry.tangent);
                        const auto constant=-positionDot(direction,geometry.curvature)
                            *referenceV*referenceV;
                        lp.addRow(-kHighsInf,m_limits.pathAcceleration-constant,{
                            {vColumn,velocityCoefficient},
                            {aColumn,accelerationCoefficient},
                        });
                    }
                };

                const auto addJerkConstraints=[&](const CoupledEndpointGeometry &geometry,
                        const std::size_t station,const std::size_t pieceIndex,
                        const bool end,const double referenceJerk) {
                    const auto referenceV=referenceVelocity[station];
                    const auto referenceA=referenceAcceleration[station];
                    const auto vColumn=velocityColumn(station);
                    const auto aColumn=accelerationColumn(station);
                    const auto jColumn=jerkColumn(pieceIndex,end);
                    position_t velocityDerivative{};
                    position_t accelerationDerivative{};
                    position_t geometricReference{};
                    for(const auto component:AXIS_COMPONENTS) {
                        velocityDerivative.*component=
                            3.0*geometry.curvature.*component*referenceA
                            +3.0*geometry.curvatureDerivative.*component
                                *referenceV*referenceV;
                        accelerationDerivative.*component=
                            3.0*geometry.curvature.*component*referenceV;
                        geometricReference.*component=
                            3.0*geometry.curvature.*component*referenceV*referenceA
                            +geometry.curvatureDerivative.*component
                                *referenceV*referenceV*referenceV;
                    }
                    const auto constant=subtract(geometricReference,
                        add(scaled(velocityDerivative,referenceV),
                            scaled(accelerationDerivative,referenceA)));
                    for(const auto component:AXIS_COMPONENTS) {
                        const auto limit=m_limits.axisJerk.*component;
                        if(!std::isfinite(limit)) continue;
                        lp.addRow(-limit-constant.*component,limit-constant.*component,{
                            {vColumn,velocityDerivative.*component},
                            {aColumn,accelerationDerivative.*component},
                            {jColumn,geometry.tangent.*component},
                        });
                    }
                    const auto referenceVector=add(
                        scaled(geometry.tangent,referenceJerk),geometricReference);
                    const auto referenceNorm=referenceVector.length();
                    if(referenceNorm>1e-12) {
                        const auto direction=scaled(referenceVector,1.0/referenceNorm);
                        lp.addRow(-kHighsInf,m_limits.pathJerk-positionDot(direction,constant),{
                            {vColumn,positionDot(direction,velocityDerivative)},
                            {aColumn,positionDot(direction,accelerationDerivative)},
                            {jColumn,positionDot(direction,geometry.tangent)},
                        });
                    }
                };

                std::vector<std::array<std::size_t,6>> scpPieceRowOffsets(pieces.size());
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    const auto from=pieceIndex;
                    const auto to=pieceIndex+1;
                    const auto referenceFrom=referenceVelocity[from];
                    const auto referenceTo=referenceVelocity[to];
                    scpPieceRowOffsets[pieceIndex][0]=lp.model.num_row_;
                    if(m_continuousPlanningEffort.addScpAdjacentReachabilityRows) {
                        const auto referenceSquaredDifference=
                            referenceTo*referenceTo-referenceFrom*referenceFrom;
                        const auto accelerationEnergyLimit=2.0
                            *localLimits[pieceIndex].acceleration
                            *pieces[pieceIndex].length;
                        lp.addRow(-kHighsInf,
                            accelerationEnergyLimit+referenceSquaredDifference,{
                                {velocityColumn(to),2.0*referenceTo},
                                {velocityColumn(from),-2.0*referenceFrom},
                            });
                        lp.addRow(-kHighsInf,
                            accelerationEnergyLimit-referenceSquaredDifference,{
                                {velocityColumn(from),2.0*referenceFrom},
                                {velocityColumn(to),-2.0*referenceTo},
                            });
                        result->scpAdjacentReachabilityRows+=2;
                    }
                    scpPieceRowOffsets[pieceIndex][1]=lp.model.num_row_;
                    addAccelerationConstraints(pieceStartGeometry[pieceIndex],from,
                        referenceFrom);
                    scpPieceRowOffsets[pieceIndex][2]=lp.model.num_row_;
                    addAccelerationConstraints(pieceEndGeometry[pieceIndex],to,referenceTo);
                    scpPieceRowOffsets[pieceIndex][3]=lp.model.num_row_;
                    addJerkConstraints(pieceStartGeometry[pieceIndex],from,pieceIndex,
                        false,0.0);
                    scpPieceRowOffsets[pieceIndex][4]=lp.model.num_row_;
                    addJerkConstraints(pieceEndGeometry[pieceIndex],to,pieceIndex,
                        true,0.0);
                    scpPieceRowOffsets[pieceIndex][5]=lp.model.num_row_;
                }

                // Every sequential linearization must contain its reference
                // point. Check that invariant before attributing an
                // infeasibility to the LP solver.
                std::vector<double> referenceValues(variableCount,0.0);
                for(std::size_t station=0;station<referenceVelocity.size();++station) {
                    referenceValues[velocityColumn(station)]=referenceVelocity[station];
                    referenceValues[accelerationColumn(station)]=referenceAcceleration[station];
                    const auto firstDeviationRow=stationDeviationRowOffsets[station];
                    const auto targetFromFirstRow=-lp.model.row_lower_[firstDeviationRow];
                    referenceValues[accelerationDeviationColumn(station)]=
                        std::abs(referenceAcceleration[station]-targetFromFirstRow);
                }
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    const auto startJerk=feasibleEndpointScalarJerk(
                        pieceStartGeometry[pieceIndex],pieceIndex,
                        referenceVelocity[pieceIndex],referenceAcceleration[pieceIndex]);
                    const auto endJerk=feasibleEndpointScalarJerk(
                        pieceEndGeometry[pieceIndex],pieceIndex,
                        referenceVelocity[pieceIndex+1],referenceAcceleration[pieceIndex+1]);
                    if(!startJerk||!endJerk)
                        PANIC("verified SCP reference has no feasible endpoint scalar jerk");
                    referenceValues[jerkColumn(pieceIndex,false)]=*startJerk;
                    referenceValues[jerkColumn(pieceIndex,true)]=*endJerk;
                }
                auto referenceViolation=0.0;
                auto referenceViolationRow=std::size_t {0};
                auto referenceViolationValue=0.0;
                for(std::size_t row=0;row<static_cast<std::size_t>(lp.model.num_row_);++row) {
                    auto value=0.0;
                    const auto begin=lp.model.a_matrix_.start_[row];
                    const auto end=lp.model.a_matrix_.start_[row+1];
                    for(auto entry=begin;entry<end;++entry)
                        value+=lp.model.a_matrix_.value_[entry]
                            *referenceValues[lp.model.a_matrix_.index_[entry]];
                    const auto violation=std::max({0.0,lp.model.row_lower_[row]-value,
                        value-lp.model.row_upper_[row]});
                    if(violation>referenceViolation) {
                        referenceViolation=violation;
                        referenceViolationRow=row;
                        referenceViolationValue=value;
                    }
                }
                if(referenceViolation>1e-7) {
                    if(referenceViolationRow<scpPieceRowOffsets.front()[0]) {
                        auto violationStation=std::size_t {0};
                        while(violationStation+1<stationDeviationRowOffsets.size()
                              &&referenceViolationRow
                                    >=stationDeviationRowOffsets[violationStation+1])
                            ++violationStation;
                        return std::unexpected(std::format(
                            "continuous SCP linearization excludes its reference: row={} "
                            "station={} group=acceleration_deviation group_row={} "
                            "violation={} value={} bounds=[{},{}] reference=[v={} a={}] "
                            "correction_pass={} iteration={}",
                            referenceViolationRow,violationStation,
                            referenceViolationRow
                                -stationDeviationRowOffsets[violationStation],
                            referenceViolation,referenceViolationValue,
                            lp.model.row_lower_[referenceViolationRow],
                            lp.model.row_upper_[referenceViolationRow],
                            referenceVelocity[violationStation],
                            referenceAcceleration[violationStation],
                            correctionPass,scpIteration));
                    }
                    auto violationPiece=std::size_t {0};
                    while(violationPiece+1<scpPieceRowOffsets.size()
                          &&referenceViolationRow>=scpPieceRowOffsets[violationPiece][5])
                        ++violationPiece;
                    static constexpr std::array rowGroups{
                        "adjacent_reachability","start_acceleration","end_acceleration",
                        "start_jerk","end_jerk"};
                    auto violationGroup=std::size_t {0};
                    while(violationGroup+1<rowGroups.size()
                          &&referenceViolationRow
                              >=scpPieceRowOffsets[violationPiece][violationGroup+1])
                        ++violationGroup;
                    const auto violationBegin=
                        lp.model.a_matrix_.start_[referenceViolationRow];
                    const auto violationEnd=
                        lp.model.a_matrix_.start_[referenceViolationRow+1];
                    const auto firstEntry=violationBegin<violationEnd?violationBegin:0;
                    const auto secondEntry=violationBegin+1<violationEnd
                        ?violationBegin+1:firstEntry;
                    const auto thirdEntry=violationBegin+2<violationEnd
                        ?violationBegin+2:secondEntry;
                    return std::unexpected(std::format(
                        "continuous SCP linearization excludes its reference: row={} piece={} "
                        "group={} group_row={} violation={} value={} bounds=[{},{}] "
                        "reference=[v={} a={}] curvature={} tangent={} pieces={} "
                        "entries={} first=[column={} coefficient={} reference={}] "
                        "second=[column={} coefficient={} reference={}] "
                        "third=[column={} coefficient={} reference={}] correction_pass={} iteration={}",
                        referenceViolationRow,violationPiece,rowGroups[violationGroup],
                        referenceViolationRow
                            -scpPieceRowOffsets[violationPiece][violationGroup],
                        referenceViolation,referenceViolationValue,
                        lp.model.row_lower_[referenceViolationRow],
                        lp.model.row_upper_[referenceViolationRow],
                        referenceVelocity[violationPiece],
                        referenceAcceleration[violationPiece],
                        pieceStartGeometry[violationPiece].curvature.x,
                        pieceStartGeometry[violationPiece].tangent.x,
                        pieces.size(),violationEnd-violationBegin,
                        lp.model.a_matrix_.index_[firstEntry],
                        lp.model.a_matrix_.value_[firstEntry],
                        referenceValues[lp.model.a_matrix_.index_[firstEntry]],
                        lp.model.a_matrix_.index_[secondEntry],
                        lp.model.a_matrix_.value_[secondEntry],
                        referenceValues[lp.model.a_matrix_.index_[secondEntry]],
                        lp.model.a_matrix_.index_[thirdEntry],
                        lp.model.a_matrix_.value_[thirdEntry],
                        referenceValues[lp.model.a_matrix_.index_[thirdEntry]],
                        correctionPass,scpIteration));
                }

                Highs highs;
                if(highs.setOptionValue("output_flag",false)!=HighsStatus::kOk
                   ||highs.setOptionValue("threads",HighsInt {1})!=HighsStatus::kOk
                   ||highs.setOptionValue("solver",std::string {"simplex"})
                        !=HighsStatus::kOk
                   ||highs.setOptionValue("time_limit",
                        m_continuousPlanningEffort.scpSolveTimeLimit)!=HighsStatus::kOk
                   ||highs.setOptionValue("simplex_iteration_limit",
                        static_cast<HighsInt>(m_continuousPlanningEffort
                            .scpSimplexIterationLimitMultiplier*variableCount))
                                !=HighsStatus::kOk)
                    return std::unexpected("continuous SCP could not configure HiGHS");
                const auto passStatus=highs.passModel(lp.model);
                if(passStatus==HighsStatus::kError) {
                    return std::unexpected(std::format(
                        "continuous SCP could not pass its LP to HiGHS: columns={} rows={} "
                        "nonzeros={}",lp.model.num_col_,lp.model.num_row_,
                        lp.model.a_matrix_.index_.size()));
                }
                if(m_continuousPlanningEffort.reuseScpBasis&&reusableScpBasis) {
                    ++result->scpBasisReuseAttempts;
                    if(reusableScpBasis->valid
                       &&reusableScpBasis->col_status.size()
                            ==static_cast<std::size_t>(lp.model.num_col_)
                       &&reusableScpBasis->row_status.size()
                            ==static_cast<std::size_t>(lp.model.num_row_)) {
                        if(highs.setBasis(*reusableScpBasis,"NGC SCP reuse")
                                !=HighsStatus::kOk)
                            return std::unexpected(std::format(
                                "continuous SCP could not apply a dimension-checked HiGHS "
                                "basis on correction pass {} iteration {}: columns={} rows={}",
                                correctionPass,scpIteration,lp.model.num_col_,lp.model.num_row_));
                        ++result->scpBasisReuseApplied;
                    } else {
                        ++result->scpBasisDimensionMismatches;
                        reusableScpBasis.reset();
                    }
                }
                ++result->scpSolves;
                const auto solveStatus=highs.run();
                const auto &solveInfo=highs.getInfo();
                if(solveInfo.simplex_iteration_count>0)
                    result->scpSimplexIterations+=
                        static_cast<std::size_t>(solveInfo.simplex_iteration_count);
                const auto modelStatus=highs.getModelStatus();
                auto solveClassification=trajectory_detail::ScpSolveClassification::Failure;
                if(solveStatus!=HighsStatus::kError) {
                    if(modelStatus==HighsModelStatus::kOptimal
                       &&solveStatus==HighsStatus::kOk)
                        solveClassification=trajectory_detail::ScpSolveClassification::Optimal;
                    else if(modelStatus==HighsModelStatus::kTimeLimit)
                        solveClassification=trajectory_detail::ScpSolveClassification::TimeLimit;
                    else if(modelStatus==HighsModelStatus::kIterationLimit)
                        solveClassification=
                            trajectory_detail::ScpSolveClassification::IterationLimit;
                }
                const auto solveAction=trajectory_detail::scpSolveAction(solveClassification);
                if(solveAction==trajectory_detail::ScpSolveAction::RetainReference) {
                    auto &fallback=result->scpResourceFallback;
                    if(fallback.occurrences==0) {
                        fallback.reason=solveClassification
                                ==trajectory_detail::ScpSolveClassification::TimeLimit
                            ?ScpResourceFallbackReason::TimeLimit
                            :ScpResourceFallbackReason::IterationLimit;
                        fallback.correctionPass=correctionPass;
                        fallback.scpIteration=scpIteration;
                    }
                    ++fallback.occurrences;
                    break;
                }
                if(solveAction==trajectory_detail::ScpSolveAction::Fail) {
                    return std::unexpected(std::format(
                        "continuous SCP HiGHS solve failed on correction pass {} iteration {}: {} "
                        "pieces={} start=[v={} a={}] end=[v={} a={}]",
                        correctionPass,scpIteration,
                        highs.modelStatusToString(modelStatus),pieces.size(),
                        scalarStart->first,scalarStart->second,
                        scalarEnd->first,scalarEnd->second));
                }
                const auto &solution=highs.getSolution();
                if(!solution.value_valid
                   ||solution.col_value.size()!=static_cast<std::size_t>(variableCount))
                    return std::unexpected("continuous SCP HiGHS solution has no primal values");
                if(m_continuousPlanningEffort.reuseScpBasis) {
                    const auto &basis=highs.getBasis();
                    if(basis.valid
                       &&basis.col_status.size()==static_cast<std::size_t>(lp.model.num_col_)
                       &&basis.row_status.size()==static_cast<std::size_t>(lp.model.num_row_))
                        reusableScpBasis=basis;
                    else
                        reusableScpBasis.reset();
                }

                std::vector<double> proposedVelocity(referenceVelocity.size());
                std::vector<double> proposedAcceleration(referenceAcceleration.size());
                for(std::size_t station=0;station<referenceVelocity.size();++station) {
                    proposedVelocity[station]=std::clamp(
                        solution.col_value[velocityColumn(station)],0.0,stationCaps[station]);
                    proposedAcceleration[station]=
                        solution.col_value[accelerationColumn(station)];
                }
                // A whole-horizon line search is overly brittle here: each
                // materialized piece contains several jerk phases, so one bad
                // station can reject an otherwise useful LP direction. Treat
                // internal stations as a working set instead. A station is
                // committed only when both adjacent exact scalar transitions
                // remain feasible and their combined duration does not grow.
                auto acceptedAny=false;
                for(std::size_t station=1;station+1<stationVelocity.size();++station) {
                    ++result->scpStationProposals;
                    const auto leftPiece=station-1;
                    const auto rightPiece=station;
                    const auto oldDuration=pieceTiming[leftPiece].back().time
                        +pieceTiming[rightPiece].back().time;
                    for(unsigned lineSearch=0;
                            lineSearch<m_continuousPlanningEffort.scpLineSearchSteps;
                            ++lineSearch) {
                        ++result->scpLineSearchTrials;
                        const auto fraction=std::ldexp(1.0,-static_cast<int>(lineSearch));
                        const auto trialVelocity=std::lerp(stationVelocity[station],
                            proposedVelocity[station],fraction);
                        const auto trialAcceleration=std::lerp(stationAcceleration[station],
                            proposedAcceleration[station],fraction);
                        if(stateWithinCoupledLimits(pieceEndGeometry[leftPiece],leftPiece,
                                trialVelocity,trialAcceleration)
                                    !=CoupledEndpointResult::Feasible
                           ||stateWithinCoupledLimits(pieceStartGeometry[rightPiece],rightPiece,
                                trialVelocity,trialAcceleration)
                                    !=CoupledEndpointResult::Feasible)
                            continue;

                        auto left=candidateTimeLawBetween(timeLawWorkspace,
                            timeLawRecorder.instrumentation,
                            m_continuousPlanningEffort.cacheScpLineSearchTrials,
                            TimeLawPurpose::ContinuousSeed,
                            correctionPass>0,pieces[leftPiece].length,
                            stationVelocity[leftPiece],stationAcceleration[leftPiece],
                            trialVelocity,trialAcceleration,
                            localLimits[leftPiece].velocity,
                            localLimits[leftPiece].acceleration,
                            localLimits[leftPiece].jerk);
                        if(!left.cacheHit) ++result->scpMaterializationAttempts;
                        if(!left.successful) continue;
                        auto right=candidateTimeLawBetween(timeLawWorkspace,
                            timeLawRecorder.instrumentation,
                            m_continuousPlanningEffort.cacheScpLineSearchTrials,
                            TimeLawPurpose::ContinuousSeed,
                            correctionPass>0,pieces[rightPiece].length,
                            trialVelocity,trialAcceleration,
                            stationVelocity[rightPiece+1],stationAcceleration[rightPiece+1],
                            localLimits[rightPiece].velocity,
                            localLimits[rightPiece].acceleration,
                            localLimits[rightPiece].jerk);
                        if(!right.cacheHit) ++result->scpMaterializationAttempts;
                        if(!right.successful) continue;
                        const auto newDuration=left.duration+right.duration;
                        if(newDuration>oldDuration*(1.0+1e-10)) continue;

                        if(!left.timing) {
                            ++result->scpMaterializationAttempts;
                            auto materialized=materializeCandidateTimeLaw(timeLawWorkspace,
                                timeLawRecorder.instrumentation,
                                TimeLawPurpose::ContinuousSeed,left,
                                pieces[leftPiece].length,stationVelocity[leftPiece],
                                stationAcceleration[leftPiece],trialVelocity,trialAcceleration,
                                localLimits[leftPiece].velocity,
                                localLimits[leftPiece].acceleration,
                                localLimits[leftPiece].jerk);
                            if(!materialized)
                                return std::unexpected(std::format(
                                    "continuous SCP left trial cache materialization failed "
                                    "on correction pass {} iteration {} station {}: {}",
                                    correctionPass,scpIteration,station,materialized.error()));
                            left.timing=std::move(*materialized);
                        }
                        if(!right.timing) {
                            ++result->scpMaterializationAttempts;
                            auto materialized=materializeCandidateTimeLaw(timeLawWorkspace,
                                timeLawRecorder.instrumentation,
                                TimeLawPurpose::ContinuousSeed,right,
                                pieces[rightPiece].length,trialVelocity,trialAcceleration,
                                stationVelocity[rightPiece+1],stationAcceleration[rightPiece+1],
                                localLimits[rightPiece].velocity,
                                localLimits[rightPiece].acceleration,
                                localLimits[rightPiece].jerk);
                            if(!materialized)
                                return std::unexpected(std::format(
                                    "continuous SCP right trial cache materialization failed "
                                    "on correction pass {} iteration {} station {}: {}",
                                    correctionPass,scpIteration,station,materialized.error()));
                            right.timing=std::move(*materialized);
                        }

                        stationVelocity[station]=trialVelocity;
                        stationAcceleration[station]=trialAcceleration;
                        pieceTiming[leftPiece]=std::move(*left.timing);
                        pieceTiming[rightPiece]=std::move(*right.timing);
                        scpDuration+=newDuration-oldDuration;
                        ++result->scpAcceptedSteps;
                        acceptedAny=true;
                        break;
                    }
                }
                if(!acceptedAny) break;
            }
            result->scpSeconds+=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-scpStarted).count();

            const auto optimizedDuration=std::accumulate(pieceTiming.begin(),pieceTiming.end(),0.0,
                [](const double total,const auto &timing) { return total+timing.back().time; });
            result->ruckigBrakePhases=std::accumulate(
                pieceTiming.begin(),pieceTiming.end(),std::size_t {0},
                [](const std::size_t total,const auto &timing) {
                    return total+std::ranges::count_if(timing,[](const auto &boundary) {
                        return boundary.ruckigBrakePhase;
                    });
                });
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
            result->pieceTiming.clear();
            result->pieceTiming.reserve(pieces.size());
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                result->pieceTiming.push_back({
                    .input=pieces[pieceIndex].input,
                    .length=pieces[pieceIndex].length,
                    .linear=pieces[pieceIndex].linear,
                    .startPosition=pieces[pieceIndex].positionAt(0.0),
                    .endPosition=pieces[pieceIndex].positionAt(pieces[pieceIndex].length),
                    .programmedVelocityLimit=pieces[pieceIndex].speed,
                    .initialVelocityLimit=initialLocalLimits[pieceIndex].velocity,
                    .initialVelocityLimitCause=initialLocalLimits[pieceIndex].velocityCause,
                    .initialAccelerationLimit=initialLocalLimits[pieceIndex].acceleration,
                    .initialJerkLimit=initialLocalLimits[pieceIndex].jerk,
                    .velocityLimit=localLimits[pieceIndex].velocity,
                    .accelerationLimit=localLimits[pieceIndex].acceleration,
                    .jerkLimit=localLimits[pieceIndex].jerk,
                    .entryVelocity=stationVelocity[pieceIndex],
                    .entryAcceleration=stationAcceleration[pieceIndex],
                    .exitVelocity=stationVelocity[pieceIndex+1],
                    .exitAcceleration=stationAcceleration[pieceIndex+1],
                    .duration=pieceTiming[pieceIndex].back().time,
                    .curvatureDerivativeSampleDistance=
                        curvatureDerivativeDiagnostics[pieceIndex].distance,
                    .curvatureDerivativeMagnitude=
                        curvatureDerivativeDiagnostics[pieceIndex].analyticMagnitude,
                    .curvatureMagnitudeAtDerivativeSample=
                        curvatureDerivativeDiagnostics[pieceIndex].curvatureMagnitude,
                    .curvatureDerivativeTangentialMagnitude=
                        curvatureDerivativeDiagnostics[pieceIndex].tangentialMagnitude,
                    .curvatureDerivativeNormalMagnitude=
                        curvatureDerivativeDiagnostics[pieceIndex].normalMagnitude,
                    .curvatureDerivativeFiniteDifferenceCoarse=
                        curvatureDerivativeDiagnostics[pieceIndex].coarseMagnitude,
                    .curvatureDerivativeFiniteDifferenceFine=
                        curvatureDerivativeDiagnostics[pieceIndex].fineMagnitude,
                    .curvatureDerivativeFiniteDifferenceCoarseStep=
                        curvatureDerivativeDiagnostics[pieceIndex].coarseStep,
                    .curvatureDerivativeFiniteDifferenceFineStep=
                        curvatureDerivativeDiagnostics[pieceIndex].fineStep,
                });
            }

            normalSpans.clear();
            normalSpanPieces.clear();
            std::size_t geometryVerificationAttempts=0;
            std::ranges::fill(activationSpans,SpanId{});
            nextSpan=m_nextSpan;
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                if((pieceIndex&15U)==0) reportProgress();
                const auto &piece=pieces[pieceIndex];
                const auto &boundaries=pieceTiming[pieceIndex];
                const auto emit=[&](const auto &self,const auto &stateAt,
                                    const double totalDuration,const std::size_t interval,
                                    const double u0,const double u1,
                                    const unsigned depth) -> std::optional<std::string> {
                        if(normalSpans.size()>=maximumStagedNormalSpans)
                            return std::format(
                                "continuous staged-span resource bound exceeded: staged={} bound={} "
                                "pieces={} piece={} input={} interval={} depth={} jerk_limit={}",
                                normalSpans.size(),maximumStagedNormalSpans,pieces.size(),
                                pieceIndex,piece.input,interval,depth,localLimits[pieceIndex].jerk);
                        ++geometryVerificationAttempts;
                        ++totalGeometryVerificationAttempts;
                        if((geometryVerificationAttempts&127U)==0) reportProgress();
                        if(geometryVerificationAttempts>maximumGeometryVerificationAttemptsPerPass)
                            return std::format(
                                "continuous per-pass geometry-verification resource bound exceeded: "
                                "pass={} attempts={} bound={} total_attempts={} staged={} pieces={} "
                                "piece={} input={} interval={} depth={} jerk_limit={}",correctionPass,
                                geometryVerificationAttempts,maximumGeometryVerificationAttemptsPerPass,
                                totalGeometryVerificationAttempts,normalSpans.size(),pieces.size(),
                                pieceIndex,piece.input,interval,depth,localLimits[pieceIndex].jerk);
                        if(totalGeometryVerificationAttempts>maximumTotalGeometryVerificationAttempts)
                            return std::format(
                                "continuous cumulative geometry-verification resource bound exceeded: "
                                "pass={} total_attempts={} bound={} pass_attempts={} staged={} pieces={} "
                                "piece={} input={} interval={} depth={} jerk_limit={}",correctionPass,
                                totalGeometryVerificationAttempts,maximumTotalGeometryVerificationAttempts,
                                geometryVerificationAttempts,normalSpans.size(),pieces.size(),pieceIndex,
                                piece.input,interval,depth,localLimits[pieceIndex].jerk);
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
                        std::vector<AxisPolynomialSpan> chain;
                        if(piece.linear) {
                            const auto start=piece.sampleAt(localFrom);
                            const auto finish=piece.sampleAt(localTo);
                            AxisPolynomialSpan span;
                            span.id=nextSpan;
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
                            const auto curved=c2CubicChain(nextSpan,kinematicAt(pieceIndex,from),
                                kinematicAt(pieceIndex,to),duration);
                            chain.assign(curved.begin(),curved.end());
                        }
                        auto verified=true;
                        for(std::size_t chainSpan=0;chainSpan<chain.size();++chainSpan) {
                            const auto fraction0=static_cast<double>(chainSpan)/chain.size();
                            const auto fraction1=static_cast<double>(chainSpan+1)/chain.size();
                            const auto source0=stateAt(std::lerp(u0,u1,fraction0)).distance;
                            const auto source1=stateAt(std::lerp(u0,u1,fraction1)).distance;
                            const auto proofFrom=std::clamp(source0,localFrom,localTo);
                            const auto proofTo=std::clamp(source1,localFrom,localTo);
                            const auto proof=verifiesOrderedCurveTolerance(chain[chainSpan],
                                proofFrom,proofTo,m_limits.arcChordTolerance,piece.positionAt,
                                piece.chordErrorBound);
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
                        normalSpans.insert(normalSpans.end(),chain.begin(),chain.end());
                        normalSpanPieces.insert(normalSpanPieces.end(),chain.size(),pieceIndex);
                        if(activationSpans[piece.input]==0)
                            activationSpans[piece.input]=chain.front().id;
                        for(const auto input:piece.activationInputs)
                            if(activationSpans[input]==0)
                                activationSpans[input]=chain.front().id;
                        nextSpan+=chain.size();
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
            }
            result->geometryVerificationAttempts=totalGeometryVerificationAttempts;
            result->geometryVerificationHighWater=std::max(
                result->geometryVerificationHighWater,geometryVerificationAttempts);
            if(normalSpans.empty())
                return std::unexpected("continuous locally timed trajectory emitted no motion spans");

            struct ConstraintViolationDiagnostic {
                double factor=1.0;
                std::size_t stagedSpan=0;
                SpanId spanId=0;
                const char *constraint="none";
                const char *axis="path";
                double measured=0.0;
                double limit=0.0;
                double ratio=1.0;
                double duration=0.0;
            };
            std::vector<double> correction(pieces.size(),1.0);
            std::vector<ConstraintViolationDiagnostic> violation(pieces.size());
            for(std::size_t spanIndex=0;spanIndex<normalSpans.size();++spanIndex) {
                const auto &span=normalSpans[spanIndex];
                const auto pieceIndex=normalSpanPieces[spanIndex];
                const auto consider=[&](const double factor,const char *constraint,
                                        const char *axis,const double measured,const double limit) {
                    if(factor<=correction[pieceIndex]) return;
                    correction[pieceIndex]=factor;
                    violation[pieceIndex]={
                        .factor=factor,.stagedSpan=spanIndex,.spanId=span.id,
                        .constraint=constraint,.axis=axis,.measured=measured,.limit=limit,
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
                    const auto acceleration=trajectory_detail::maximumAxisAcceleration(span,component);
                    consider(std::sqrt(acceleration/(m_limits.axisAcceleration.*component)),
                        "axis_acceleration",AXIS_NAMES[axis],acceleration,
                        m_limits.axisAcceleration.*component);
                    const auto jerk=trajectory_detail::maximumAxisJerk(span,component);
                    consider(std::cbrt(jerk/(m_limits.axisJerk.*component)),"axis_jerk",
                        AXIS_NAMES[axis],jerk,m_limits.axisJerk.*component);
                }
            }
            const auto worstIterator=std::ranges::max_element(correction);
            const auto worst=*worstIterator;
            const auto worstPiece=static_cast<std::size_t>(worstIterator-correction.begin());
            const auto &worstViolation=violation[worstPiece];
            correctionHistory+=std::format(
                "{}pass {}: factor={} piece={} input={} geometry={} piece_length={} "
                "span_id={} staged_span={} span_duration={} constraint={} axis={} measured={} "
                "limit={} measured_over_limit={} timing_candidate={} optimized_duration={} "
                "scp_iterations={} scp_accepted_steps={} "
                "max_station_acceleration={} station_state=[v={} a={} -> v={} a={}] "
                "local_limits=[v={} a={} j={}]",
                correctionHistory.empty()?"":"; ",correctionPass,worst,worstPiece,
                pieces[worstPiece].input,pieces[worstPiece].linear?"linear":"curved",
                pieces[worstPiece].length,worstViolation.spanId,worstViolation.stagedSpan,
                worstViolation.duration,worstViolation.constraint,worstViolation.axis,
                worstViolation.measured,worstViolation.limit,worstViolation.ratio,
                "sparse-scp",optimizedDuration,
                m_continuousPlanningEffort.scpIterations,result->scpAcceptedSteps,
                maximumStationAcceleration,
                stationVelocity[worstPiece],stationAcceleration[worstPiece],
                stationVelocity[worstPiece+1],stationAcceleration[worstPiece+1],
                localLimits[worstPiece].velocity,
                localLimits[worstPiece].acceleration,localLimits[worstPiece].jerk);
            if(worst<=1.0+1e-9) {
                constraintsVerified=true;
                break;
            }
            previousStationVelocity=stationVelocity;
            previousStationAcceleration=stationAcceleration;
            previousPieceTiming=pieceTiming;
            previouslyCorrectedPieces.clear();
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                if(correction[pieceIndex]<=1.0+1e-9) continue;
                previouslyCorrectedPieces.push_back(pieceIndex);
                const auto factor=correction[pieceIndex]*1.01;
                localLimits[pieceIndex].velocity/=factor;
                localLimits[pieceIndex].acceleration/=factor*factor;
                localLimits[pieceIndex].jerk/=factor*factor*factor;
            }
        }
        if(!constraintsVerified)
            return std::unexpected(std::format(
                "continuous local constraint correction did not converge after {} passes: {}",
                maximumLocalCorrectionPasses,correctionHistory));
        result->correctionHistory=correctionHistory;

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

        for(std::size_t input=1;input<activationSpans.size();++input)
            if(activationSpans[input]==0) activationSpans[input]=activationSpans[input-1];
        m_nextChunk=nextChunk;
        m_nextSpan=nextSpan;
        m_previousBranch=result->chunks.back().branch;
        m_position=expectedEnd;
        result->activationSpans=std::move(activationSpans);
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
