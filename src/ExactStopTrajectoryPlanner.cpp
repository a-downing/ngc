#include "machine/ExactStopTrajectoryPlanner.h"
#include "machine/ArcInterpolation.h"
#include "machine/SplineHandleOptimization.h"
#include "utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <numbers>
#include <numeric>
#include <tuple>
#include <type_traits>
#include <vector>

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
            const auto &profiles=trajectory.get_profiles_ref().front();
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

        double maximumAxisVelocity(const AxisPolynomialSpan &span, const double position_t::*component) {
            const auto at = [&](const double u) {
                return std::abs((3.0*span.a.*component*u*u + 2.0*span.b.*component*u
                    + span.c.*component) * span.inverseDuration);
            };
            auto result = std::max(at(0.0), at(1.0));
            if(std::abs(span.a.*component) > 1e-15) {
                const auto stationary = -(span.b.*component) / (3.0*(span.a.*component));
                if(stationary > 0.0 && stationary < 1.0) result = std::max(result, at(stationary));
            }
            return result;
        }

        double maximumAxisAcceleration(const AxisPolynomialSpan &span, const double position_t::*component) {
            const auto at = [&](const double u) {
                return std::abs((6.0*span.a.*component*u + 2.0*span.b.*component)
                    * span.inverseDurationSquared);
            };
            return std::max(at(0.0), at(1.0));
        }

        double maximumAxisJerk(const AxisPolynomialSpan &span, const double position_t::*component) {
            return std::abs(6.0*span.a.*component * span.inverseDurationCubed);
        }

        std::expected<TimeLaw, std::string> timeLawBetween(TimeLawWorkspace &workspace,
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
            const auto &profiles=trajectory.get_profiles_ref();
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

        std::expected<TimeLaw, std::string> timeLaw(TimeLawWorkspace &workspace,
                const double length, const double requestedVelocity, const double acceleration, const double jerk) {
            if(std::isinf(acceleration)) return TimeLaw{
                {0.0,0.0,requestedVelocity,0.0},
                {length/requestedVelocity,length,requestedVelocity,0.0},
            };
            return timeLawBetween(workspace,length,0.0,0.0,0.0,0.0,
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

        bool finitePosition(const position_t &value) {
            return std::ranges::all_of(AXIS_COMPONENTS,[&](const auto component) {
                return std::isfinite(value.*component);
            });
        }

        position_t evaluateBSpline(const std::span<const position_t> controls,
                                   const std::span<const double> knots,
                                   const std::size_t degree, const double requestedParameter) {
            const auto endParameter=knots[controls.size()];
            if(requestedParameter<=knots[degree]) return controls.front();
            if(requestedParameter>=endParameter) return controls.back();
            const auto parameter=std::clamp(requestedParameter,knots[degree],endParameter);
            auto span=degree;
            while(span+1<controls.size()&&parameter>=knots[span+1]) ++span;
            std::array<position_t,4> work{};
            for(std::size_t index=0;index<=degree;++index)
                work[index]=controls[span-degree+index];
            for(std::size_t level=1;level<=degree;++level) {
                for(std::size_t index=degree;index>=level;--index) {
                    const auto knotIndex=span-degree+index;
                    const auto denominator=knots[knotIndex+degree-level+1]-knots[knotIndex];
                    const auto alpha=(parameter-knots[knotIndex])/denominator;
                    work[index]=add(scaled(work[index-1],1.0-alpha),scaled(work[index],alpha));
                }
            }
            return work[degree];
        }

        template<typename Function>
        double integrateAdaptive(const Function &function, const double from, const double to,
                                 const double tolerance, const unsigned depth=0) {
            const auto middle=std::midpoint(from,to);
            const auto fromValue=function(from);
            const auto middleValue=function(middle);
            const auto toValue=function(to);
            const auto estimate=(to-from)*(fromValue+4.0*middleValue+toValue)/6.0;
            const auto recurse=[&](const auto &self,const double a,const double b,
                                   const double fa,const double fm,const double fb,
                                   const double whole,const double epsilon,
                                   const unsigned level) -> double {
                const auto m=std::midpoint(a,b);
                const auto lm=std::midpoint(a,m);
                const auto rm=std::midpoint(m,b);
                const auto flm=function(lm);
                const auto frm=function(rm);
                const auto left=(m-a)*(fa+4.0*flm+fm)/6.0;
                const auto right=(b-m)*(fm+4.0*frm+fb)/6.0;
                const auto refined=left+right;
                if(level>=20||std::abs(refined-whole)<=15.0*epsilon)
                    return refined+(refined-whole)/15.0;
                return self(self,a,m,fa,flm,fm,left,epsilon*0.5,level+1)
                    +self(self,m,b,fm,frm,fb,right,epsilon*0.5,level+1);
            };
            return recurse(recurse,from,to,fromValue,middleValue,toValue,estimate,tolerance,depth);
        }

        class CubicBSplineReference {
            static constexpr std::size_t LENGTH_INTERVALS_PER_SPAN=32;
            std::vector<double> m_knots;
            std::vector<double> m_derivativeKnots;
            std::vector<double> m_secondKnots;
            std::vector<double> m_thirdKnots;
            std::vector<position_t> m_controls;
            std::vector<position_t> m_derivativeControls;
            std::vector<position_t> m_secondControls;
            std::vector<position_t> m_thirdControls;
            struct LengthNode { double parameter=0.0; double distance=0.0; };
            std::vector<LengthNode> m_nodes;
            std::vector<double> m_spanDistances;
            double m_parameterMaximum=0.0;
            double m_length=0.0;

            double speed(const double parameter) const { return derivative(parameter).length(); }
            double integratedLength(const double from,const double to) const {
                if(to<=from) return 0.0;
                return integrateAdaptive([&](const double u) { return speed(u); },from,to,
                    1e-12*std::max(1.0,to-from));
            }
        public:
            explicit CubicBSplineReference(std::vector<position_t> controls)
                : m_controls(std::move(controls)) {
                if(m_controls.size()<4) return;
                m_parameterMaximum=static_cast<double>(m_controls.size()-3);
                m_knots.assign(m_controls.size()+4,m_parameterMaximum);
                std::fill_n(m_knots.begin(),4,0.0);
                for(std::size_t index=4;index<m_controls.size();++index)
                    m_knots[index]=static_cast<double>(index-3);
                m_derivativeKnots.assign(m_knots.begin()+1,m_knots.end()-1);
                m_secondKnots.assign(m_derivativeKnots.begin()+1,m_derivativeKnots.end()-1);
                m_thirdKnots.assign(m_secondKnots.begin()+1,m_secondKnots.end()-1);
                m_derivativeControls.resize(m_controls.size()-1);
                m_secondControls.resize(m_controls.size()-2);
                m_thirdControls.resize(m_controls.size()-3);
                for(std::size_t index=0;index<m_derivativeControls.size();++index) {
                    const auto denominator=m_knots[index+4]-m_knots[index+1];
                    m_derivativeControls[index]=scaled(
                        subtract(m_controls[index+1],m_controls[index]),3.0/denominator);
                }
                for(std::size_t index=0;index<m_secondControls.size();++index) {
                    const auto denominator=m_derivativeKnots[index+3]
                        -m_derivativeKnots[index+1];
                    m_secondControls[index]=scaled(
                        subtract(m_derivativeControls[index+1],m_derivativeControls[index]),
                        2.0/denominator);
                }
                for(std::size_t index=0;index<m_thirdControls.size();++index) {
                    const auto denominator=m_secondKnots[index+2]-m_secondKnots[index+1];
                    m_thirdControls[index]=scaled(
                        subtract(m_secondControls[index+1],m_secondControls[index]),
                        1.0/denominator);
                }
                const auto intervals=LENGTH_INTERVALS_PER_SPAN*(m_controls.size()-3);
                m_nodes.resize(intervals+1);
                m_spanDistances.reserve(m_controls.size()-2);
                m_spanDistances.push_back(0.0);
                for(std::size_t index=1;index<=intervals;++index) {
                    const auto from=m_parameterMaximum*static_cast<double>(index-1)/intervals;
                    const auto to=m_parameterMaximum*static_cast<double>(index)/intervals;
                    m_length+=integratedLength(from,to);
                    m_nodes[index]={to,m_length};
                    if(index%LENGTH_INTERVALS_PER_SPAN==0)
                        m_spanDistances.push_back(m_length);
                }
            }

            template<std::size_t Count>
            explicit CubicBSplineReference(const std::array<position_t,Count> &controls)
                : CubicBSplineReference(std::vector<position_t>(controls.begin(),controls.end())) {}

            double length() const { return m_length; }
            std::span<const double> spanDistances() const { return m_spanDistances; }
            position_t position(const double parameter) const {
                return evaluateBSpline(m_controls,m_knots,3,parameter);
            }
            position_t derivative(const double parameter) const {
                return evaluateBSpline(m_derivativeControls,m_derivativeKnots,2,parameter);
            }
            position_t secondDerivative(const double parameter) const {
                return evaluateBSpline(m_secondControls,m_secondKnots,1,parameter);
            }
            position_t thirdDerivative(const double parameter) const {
                return evaluateBSpline(m_thirdControls,m_thirdKnots,0,parameter);
            }
            double parameterAtDistance(const double requestedDistance) const {
                if(!std::isfinite(requestedDistance)||!std::isfinite(m_length)||m_length<=1e-15)
                    return m_parameterMaximum;
                const auto distance=std::clamp(requestedDistance,0.0,m_length);
                if(distance<=0.0) return 0.0;
                if(distance>=m_length) return m_parameterMaximum;
                const auto upper=std::lower_bound(m_nodes.begin(),m_nodes.end(),distance,
                    [](const LengthNode &node,const double value) { return node.distance<value; });
                const auto lower=upper-1;
                auto from=lower->parameter;
                auto to=upper->parameter;
                auto parameter=std::lerp(from,to,
                    (distance-lower->distance)/(upper->distance-lower->distance));
                const auto distanceTolerance=1e-12*std::max(1.0,m_length);
                for(unsigned iteration=0;iteration<12;++iteration) {
                    const auto parameterDistance=
                        lower->distance+integratedLength(lower->parameter,parameter);
                    const auto error=parameterDistance-distance;
                    if(std::abs(error)<=distanceTolerance) return parameter;
                    if(error<0.0) from=parameter;
                    else to=parameter;
                    const auto localSpeed=speed(parameter);
                    const auto newton=localSpeed>1e-15?parameter-error/localSpeed:parameter;
                    parameter=newton>from&&newton<to?newton:std::midpoint(from,to);
                }
                return parameter;
            }
            position_t positionAtDistance(const double distance) const {
                return position(parameterAtDistance(distance));
            }
            position_t tangentAtDistance(const double distance) const {
                const auto value=derivative(parameterAtDistance(distance));
                const auto magnitude=value.length();
                return magnitude>1e-15?scaled(value,1.0/magnitude):position_t{};
            }
            position_t curvatureAtDistance(const double distance) const {
                const auto parameter=parameterAtDistance(distance);
                const auto velocity=derivative(parameter);
                const auto speed=velocity.length();
                if(speed<=1e-15) return {};
                const auto tangent=scaled(velocity,1.0/speed);
                const auto acceleration=secondDerivative(parameter);
                return scaled(subtract(acceleration,
                    scaled(tangent,positionDot(acceleration,tangent))),1.0/(speed*speed));
            }
            position_t curvatureDerivativeAtDistance(const double distance) const {
                const auto parameter=parameterAtDistance(distance);
                const auto first=derivative(parameter);
                const auto second=secondDerivative(parameter);
                const auto third=thirdDerivative(parameter);
                const auto speed=first.length();
                if(speed<=1e-15) return {};
                const auto firstSecond=positionDot(first,second);
                const auto firstThird=positionDot(first,third);
                const auto secondSquared=positionDot(second,second);
                const auto inverseSpeed2=1.0/(speed*speed);
                const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
                const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
                auto parameterDerivative=add(
                    scaled(third,inverseSpeed2),
                    add(scaled(second,-3.0*firstSecond*inverseSpeed4),
                        add(scaled(first,-(secondSquared+firstThird)*inverseSpeed4),
                            scaled(first,4.0*firstSecond*firstSecond*inverseSpeed6))));
                return scaled(parameterDerivative,1.0/speed);
            }
            double chordErrorBound(const double fromDistance,const double toDistance) const {
                const auto from=parameterAtDistance(fromDistance);
                const auto to=parameterAtDistance(toDistance);
                auto maximumSecond=std::max(secondDerivative(from).length(),secondDerivative(to).length());
                for(auto knot=std::floor(from)+1.0;knot<to;knot+=1.0)
                    maximumSecond=std::max(maximumSecond,secondDerivative(knot).length());
                const auto width=to-from;
                return maximumSecond*width*width/8.0;
            }
        };

        struct ContinuousEntity {
            std::size_t input=0;
            double length=0.0;
            double speed=0.0;
            bool linear=false;
            std::function<position_t(double)> positionAt;
            std::function<position_t(double)> tangentAt;
            std::function<position_t(double)> curvatureAt;
            std::function<position_t(double)> curvatureDerivativeAt;
            std::function<double(double,double)> chordErrorBound;
        };

        struct GeometryPiece {
            std::size_t input=0;
            std::vector<std::size_t> activationInputs{};
            double length=0.0;
            double speed=0.0;
            bool linear=false;
            std::function<PathSample(double)> sampleAt;
            std::function<position_t(double)> curvatureAt;
            std::function<position_t(double)> curvatureDerivativeAt;
            std::function<position_t(double)> positionAt;
            std::function<double(double,double)> chordErrorBound;
        };
    }

    ExactStopTrajectoryPlanner::ExactStopTrajectoryPlanner(TrajectoryLimits limits) : m_limits(limits) { }

    void ExactStopTrajectoryPlanner::reset(const EpochId epoch, const position_t &position) {
        m_epoch = epoch;
        m_nextChunk = 1;
        m_nextSpan = 1;
        m_previousBranch = 0;
        m_position = position;
    }

    std::expected<PlanChunk, std::string> ExactStopTrajectoryPlanner::compile(const MachineCommand &command) {
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

        auto appendMotion = [&](const double length, const double speedPerMinute,
                                const position_t &maximumTangent,
                                const auto &sample, const auto &verify) -> std::optional<std::string> {
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
            const auto timing = timeLaw(timeLawWorkspace,length,requestedVelocity,acceleration,jerk);
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
                    auto span = hermite(m_nextSpan++, sample(from.distance), sample(to.distance),
                                        from.velocity, to.velocity, duration);
                    if(!verify(span, from.distance, to.distance, from.velocity, to.velocity)) {
                        --m_nextSpan;
                        if(depth == 20) return "arc cubic tolerance verification did not converge";
                        const auto middle = std::midpoint(u0, u1);
                        if(auto result = self(self, u0, middle, depth + 1)) return result;
                        return self(self, middle, u1, depth + 1);
                    }
                    if(!chunk.normalMotion.push(span)) return "trajectory chunk span capacity exceeded";
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
                        maximumAxisVelocity(span, component) / (m_limits.axisVelocity.*component));
                    axisScaleFactor = std::max(axisScaleFactor, std::sqrt(
                        maximumAxisAcceleration(span, component) / (m_limits.axisAcceleration.*component)));
                    axisScaleFactor = std::max(axisScaleFactor, std::cbrt(
                        maximumAxisJerk(span, component) / (m_limits.axisJerk.*component)));
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
                const auto source = value.from();
                const auto target = value.to();
                const auto delta = subtract(target, source);
                auto length = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z
                    + delta.a*delta.a + delta.b*delta.b + delta.c*delta.c);
                const auto tangent = length > 1e-12 ? scaled(delta, 1.0 / length) : position_t{};
                const auto sample = [&](const double distance) { return PathSample { add(source, scaled(tangent, distance)), tangent }; };
                const auto speed = value.speed() < 0.0 ? m_limits.rapidSpeed : value.speed();
                const auto accept = [](const AxisPolynomialSpan &, double, double, double, double) { return true; };
                if(auto result = appendMotion(length, speed, tangent, sample, accept)) return result;
                m_position = target;
            } else if constexpr(std::same_as<T, ProbeMove>) {
                return "unreachable probe compilation path";
            } else if constexpr(std::same_as<T, MoveArc>) {
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
                if(auto result = appendMotion(length, arcSpeed, maximumTangent, sample, accept)) return result;
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
    ExactStopTrajectoryPlanner::compileContinuous(
            const std::span<const MachineCommand> commands, const double blendScale,
            ContinuousAccelerationOracleModel *oracleModel,
            std::optional<MotionState> requestedStartState,
            std::optional<MotionState> requestedEndState,
            const std::span<const double> scaleOverrides,
            const unsigned requestedVelocitySearchIterations) {
        if(commands.empty()) return std::unexpected("continuous trajectory window is empty");
        if(blendScale<=0.0) return std::unexpected("continuous trajectory blend scale must be positive");
        if(m_limits.pathAcceleration<=0.0||m_limits.pathJerk<=0.0
           ||m_limits.arcChordTolerance<=0.0||!positiveAxisLimits(m_limits.axisVelocity)
           ||!positiveAxisLimits(m_limits.axisAcceleration)||!positiveAxisLimits(m_limits.axisJerk))
            return std::unexpected("trajectory limits must be positive");
        constexpr std::size_t MAX_REACHABILITY_ACCELERATION_CANDIDATES=32;
        if(m_continuousPlanningEffort.reachabilitySweeps==0
           ||m_continuousPlanningEffort.accelerationCandidates<6
           ||m_continuousPlanningEffort.accelerationCandidates
                >MAX_REACHABILITY_ACCELERATION_CANDIDATES
           ||m_continuousPlanningEffort.candidateBudgetMultiplier==0
           ||m_continuousPlanningEffort.candidateBudgetMultiplier>1024
           ||!std::isfinite(m_continuousPlanningEffort
                .curvatureDerivativeVelocityCapMultiplier)
           ||m_continuousPlanningEffort.curvatureDerivativeVelocityCapMultiplier<=0.0)
            return std::unexpected("continuous planning effort is outside its bounded range");
        reportProgress();

        std::vector<ContinuousEntity> entities;
        entities.reserve(commands.size());
        position_t expectedStart=m_position;
        for(std::size_t input=0;input<commands.size();++input) {
            if((input&15U)==0) reportProgress();
            auto entity=std::visit([&](const auto &command)
                    -> std::expected<ContinuousEntity,std::string> {
                using T=std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T,MoveLine>) {
                    if(command.speed()<=0.0||command.machineCoordinates())
                        return std::unexpected("continuous line must be a positive-feed non-G53 move");
                    if(const auto mismatch=subtract(command.from(),expectedStart).length();mismatch>1e-8)
                        return std::unexpected(std::format(
                            "continuous line input {} does not begin at the planned position: "
                            "command start [X={} Y={} Z={} A={} B={} C={}], planned [X={} Y={} Z={} A={} B={} C={}], "
                            "distance={} tolerance=1e-8",input,command.from().x,command.from().y,
                            command.from().z,command.from().a,command.from().b,command.from().c,
                            expectedStart.x,expectedStart.y,expectedStart.z,expectedStart.a,
                            expectedStart.b,expectedStart.c,mismatch));
                    const auto delta=subtract(command.to(),command.from());
                    const auto length=delta.length();
                    if(length<=1e-12) return std::unexpected("continuous path contains a zero-length line");
                    const auto tangent=scaled(delta,1.0/length);
                    return ContinuousEntity {
                        .input=input,.length=length,.speed=command.speed(),
                        .linear=true,
                        .positionAt=[from=command.from(),tangent,length](const double distance) {
                            return add(from,scaled(tangent,std::clamp(distance,0.0,length)));
                        },
                        .tangentAt=[tangent](double) { return tangent; },
                        .curvatureAt=[](double) { return position_t{}; },
                        .curvatureDerivativeAt=[](double) { return position_t{}; },
                        .chordErrorBound=[](double,double) { return 0.0; },
                    };
                } else if constexpr(std::same_as<T,MoveArc>) {
                    if(command.speed()<=0.0)
                        return std::unexpected("continuous arc must have a positive feed");
                    if(const auto mismatch=subtract(command.from(),expectedStart).length();mismatch>1e-8)
                        return std::unexpected(std::format(
                            "continuous arc input {} does not begin at the planned position: "
                            "command start [X={} Y={} Z={} A={} B={} C={}], planned [X={} Y={} Z={} A={} B={} C={}], "
                            "distance={} tolerance=1e-8",input,command.from().x,command.from().y,
                            command.from().z,command.from().a,command.from().b,command.from().c,
                            expectedStart.x,expectedStart.y,expectedStart.z,expectedStart.a,
                            expectedStart.b,expectedStart.c,mismatch));
                    auto reference=std::make_shared<simulation_detail::ArcReference>(command);
                    if(!reference->valid()||reference->length()<=1e-12)
                        return std::unexpected("continuous path contains an invalid arc");
                    const std::function<position_t(double)> curvatureAt=[reference](const double distance) {
                        const auto length=reference->length();
                        const auto local=std::clamp(distance,0.0,length);
                        const auto step=std::clamp(length*1e-5,1e-8,
                            std::max(length*1e-3,1e-8));
                        const auto from=std::max(0.0,local-step);
                        const auto to=std::min(length,local+step);
                        if(to-from<=1e-15) return position_t{};
                        const auto tangent=reference->tangentAtDistance(local);
                        auto curvature=scaled(subtract(reference->tangentAtDistance(to),
                            reference->tangentAtDistance(from)),1.0/(to-from));
                        return subtract(curvature,
                            scaled(tangent,positionDot(curvature,tangent)));
                    };
                    return ContinuousEntity {
                        .input=input,.length=reference->length(),.speed=command.speed(),
                        .linear=false,
                        .positionAt=[reference](const double distance) {
                            return reference->positionAtDistance(distance);
                        },
                        .tangentAt=[reference](const double distance) {
                            return reference->tangentAtDistance(distance);
                        },
                        .curvatureAt=curvatureAt,
                        .curvatureDerivativeAt=[reference,curvatureAt](const double distance) {
                            const auto length=reference->length();
                            const auto step=std::clamp(length*1e-3,1e-7,
                                std::max(length*1e-2,1e-7));
                            const auto local=std::clamp(distance,0.0,length);
                            const auto from=std::max(0.0,local-step);
                            const auto to=std::min(length,local+step);
                            if(to-from<=1e-15) return position_t{};
                            return scaled(subtract(curvatureAt(to),curvatureAt(from)),1.0/(to-from));
                        },
                        .chordErrorBound=[reference](const double from,const double to) {
                            return reference->chordErrorBound(from,to);
                        },
                    };
                } else return std::unexpected("continuous path accepts only lines and arcs");
            },commands[input]);
            if(!entity) return std::unexpected(entity.error());
            entities.push_back(std::move(*entity));
            expectedStart=std::visit([](const auto &command) -> position_t {
                using T=std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T,MoveLine>||std::same_as<T,MoveArc>) return command.to();
                else return {};
            },commands[input]);
        }

        if(!scaleOverrides.empty()&&scaleOverrides.size()!=entities.size())
            return std::unexpected("continuous scale-override count does not match its command count");
        std::vector<double> scales;
        scales.reserve(entities.size());
        for(std::size_t index=0;index<entities.size();++index) {
            const auto automatic=std::min(blendScale,entities[index].length/6.0);
            const auto scale=!scaleOverrides.empty()&&scaleOverrides[index]>0.0
                ?scaleOverrides[index]:automatic;
            if(!std::isfinite(scale)||scale<=0.0||scale>entities[index].length/3.0*(1.0+1e-10))
                return std::unexpected(std::format(
                    "continuous entity {} has invalid retained scale {} for length {}",
                    index,scale,entities[index].length));
            scales.push_back(scale);
        }

        std::vector<double> entityLengths;
        entityLengths.reserve(entities.size());
        for(const auto &entity:entities) entityLengths.push_back(entity.length);
        std::vector<ContinuousTrajectoryPlan::SplineGeometry> splineGeometryDiagnostics;
        const auto shortEntityClusters=spline_detail::detectShortEntitySplineClusters(
            entityLengths,blendScale);
        constexpr auto NO_CLUSTER=std::numeric_limits<std::size_t>::max();
        std::vector<std::size_t> clusterRight(entities.size(),NO_CLUSTER);
        std::vector<bool> collapsedEntity(entities.size(),false);
        std::vector<bool> suppressedJunction(entities.empty()?0:entities.size()-1,false);
        for(const auto &cluster:shortEntityClusters) {
            clusterRight[cluster.left]=cluster.right;
            for(auto entity=cluster.firstInterior;entity<=cluster.lastInterior;++entity)
                collapsedEntity[entity]=true;
            for(auto junction=cluster.left;junction<cluster.right;++junction)
                suppressedJunction[junction]=true;
        }

        std::vector<std::optional<position_t>> shortLineMidpointCurvatures(entities.size());
        const auto isFullyReplacedShortLine=[&](const std::size_t index) {
            return entities[index].linear
                &&std::abs(6.0*scales[index]-entities[index].length)
                    <=1e-9*std::max(1.0,entities[index].length);
        };
        for(std::size_t index=1;index+1<entities.size();++index) {
            if(collapsedEntity[index-1]||collapsedEntity[index]||collapsedEntity[index+1]
               ||!isFullyReplacedShortLine(index-1)||!isFullyReplacedShortLine(index)
               ||!isFullyReplacedShortLine(index+1)) continue;
            const auto minimumLength=std::min({entities[index-1].length,entities[index].length,
                entities[index+1].length});
            const auto maximumLength=std::max({entities[index-1].length,entities[index].length,
                entities[index+1].length});
            if(minimumLength<=1e-12||maximumLength>1.5*minimumLength) continue;
            const auto previous=entities[index-1].tangentAt(entities[index-1].length);
            const auto current=entities[index].tangentAt(0.5*entities[index].length);
            const auto next=entities[index+1].tangentAt(0.0);
            const auto line=std::get_if<MoveLine>(&commands[index]);
            if(!line) continue;
            const auto delta=subtract(line->to(),line->from());
            const auto xyzLength=std::sqrt(delta.x*delta.x+delta.y*delta.y+delta.z*delta.z);
            const auto curvature=spline_detail::inferShortLineMidpointCurvature(
                {previous.x,previous.y,previous.z},{current.x,current.y,current.z},
                {next.x,next.y,next.z},xyzLength);
            if(curvature) shortLineMidpointCurvatures[index]=position_t{
                (*curvature)[0],(*curvature)[1],(*curvature)[2],0.0,0.0,0.0};
        }
        const auto blendCurvatureAt=[&](const std::size_t index,const double distance) {
            if(shortLineMidpointCurvatures[index]
               &&std::abs(distance-0.5*entities[index].length)
                    <=1e-9*std::max(1.0,entities[index].length))
                return *shortLineMidpointCurvatures[index];
            return entities[index].curvatureAt(distance);
        };

        const auto trimmedPiece=[](const ContinuousEntity &entity,const double from,const double to) {
            const auto length=std::max(0.0,to-from);
            return GeometryPiece {
                .input=entity.input,.length=length,.speed=entity.speed/60.0,
                .linear=entity.linear,
                .sampleAt=[entity,from,length](const double distance) {
                    const auto source=from+std::clamp(distance,0.0,length);
                    return PathSample{entity.positionAt(source),entity.tangentAt(source)};
                },
                .curvatureAt=[entity,from,length](const double distance) {
                    return entity.curvatureAt(from+std::clamp(distance,0.0,length));
                },
                .curvatureDerivativeAt=[entity,from,length](const double distance) {
                    return entity.curvatureDerivativeAt(from+std::clamp(distance,0.0,length));
                },
                .positionAt=[entity,from,length](const double distance) {
                    return entity.positionAt(from+std::clamp(distance,0.0,length));
                },
                .chordErrorBound=[entity,from,length](const double a,const double b) {
                    return entity.chordErrorBound(from+std::clamp(a,0.0,length),
                                                   from+std::clamp(b,0.0,length));
                },
            };
        };

        const auto buildClusterControls=[&](const std::size_t left,const std::size_t right)
                -> std::expected<std::vector<position_t>,std::string> {
            const auto leftDistance=entities[left].length-3.0*blendScale;
            const auto rightDistance=3.0*blendScale;
            const PathSample start{entities[left].positionAt(leftDistance),
                                   entities[left].tangentAt(leftDistance)};
            const PathSample end{entities[right].positionAt(rightDistance),
                                 entities[right].tangentAt(rightDistance)};
            const auto startCurvature=blendCurvatureAt(left,leftDistance);
            const auto endCurvature=blendCurvatureAt(right,rightDistance);
            const auto fittedHandle=[blendScale](const PathSample &endpoint,
                                       const position_t &curvature,
                                       const position_t &twoStepsInside,
                                       const double fallbackTangentDistance) {
                const auto delta=subtract(twoStepsInside,endpoint.position);
                auto tangentDistance=positionDot(delta,endpoint.tangent);
                if(tangentDistance*fallbackTangentDistance<=0.0)
                    tangentDistance=fallbackTangentDistance;
                const auto normalDelta=subtract(delta,scaled(endpoint.tangent,tangentDistance));
                const auto curvatureSquared=positionDot(curvature,curvature);
                auto handle=blendScale;
                if(curvatureSquared>1e-18) {
                    const auto handleSquared=positionDot(normalDelta,curvature)
                        /(3.0*curvatureSquared);
                    if(handleSquared>1e-18) handle=std::clamp(
                        std::sqrt(handleSquared),blendScale*0.25,blendScale*2.0);
                }
                return std::pair{handle,tangentDistance};
            };
            auto [incomingHandle,incomingTangentDistance]=fittedHandle(
                start,startCurvature,entities[left].positionAt(entities[left].length-blendScale),
                2.0*blendScale);
            auto [outgoingHandle,outgoingTangentDistance]=fittedHandle(
                end,endCurvature,entities[right].positionAt(blendScale),-2.0*blendScale);
            const auto xyzLength=[](const position_t &value) {
                return std::sqrt(value.x*value.x+value.y*value.y+value.z*value.z);
            };
            if(xyzLength(startCurvature)>1e-12||xyzLength(endCurvature)>1e-12) {
                const auto endpoint=[](const PathSample &sample,const position_t &curvature) {
                    return spline_detail::Endpoint3{
                        .position={sample.position.x,sample.position.y,sample.position.z},
                        .tangent={sample.tangent.x,sample.tangent.y,sample.tangent.z},
                        .curvature={curvature.x,curvature.y,curvature.z},
                    };
                };
                std::tie(incomingHandle,outgoingHandle)=spline_detail::optimizeHandles(
                    endpoint(start,startCurvature),endpoint(end,endCurvature),
                    incomingTangentDistance,outgoingTangentDistance,
                    incomingHandle,outgoingHandle,blendScale,blendScale);
            }
            std::vector<position_t> controls;
            std::vector<double> interiorLengths;
            interiorLengths.reserve(right-left-1);
            auto interiorLength=0.0;
            for(auto index=left+1;index<right;++index) {
                interiorLength+=entities[index].length;
                interiorLengths.push_back(entities[index].length);
            }
            std::vector<double> distances;
            if(interiorLength>=6.0*blendScale) {
                distances=spline_detail::evenlySpacedCompositeControlDistances(
                    interiorLengths,blendScale);
                if(distances.empty())
                    return std::unexpected("continuous short-entity spline has no control samples");
            }
            controls.reserve(distances.size()+6);
            controls.push_back(start.position);
            controls.push_back(add(start.position,scaled(start.tangent,incomingHandle)));
            controls.push_back(add(add(start.position,
                scaled(start.tangent,incomingTangentDistance)),
                scaled(startCurvature,3.0*incomingHandle*incomingHandle)));
            auto entity=left+1;
            auto entityStart=0.0;
            for(const auto distance:distances) {
                while(entity+1<right
                      &&distance>entityStart+entities[entity].length) {
                    entityStart+=entities[entity].length;
                    ++entity;
                }
                controls.push_back(entities[entity].positionAt(
                    std::clamp(distance-entityStart,0.0,entities[entity].length)));
            }
            controls.push_back(add(add(end.position,
                scaled(end.tangent,outgoingTangentDistance)),
                scaled(endCurvature,3.0*outgoingHandle*outgoingHandle)));
            controls.push_back(subtract(end.position,scaled(end.tangent,outgoingHandle)));
            controls.push_back(end.position);
            if(controls.size()>6) {
                std::vector<std::array<double,6>> sourceControls;
                sourceControls.reserve(controls.size());
                for(const auto &control:controls)
                    sourceControls.push_back({control.x,control.y,control.z,
                                              control.a,control.b,control.c});
                const auto conditioned=spline_detail::conditionCubicSplineInteriorControls<6>(
                    sourceControls,blendScale);
                for(std::size_t control=0;control<controls.size();++control)
                    controls[control]={conditioned[control][0],conditioned[control][1],
                        conditioned[control][2],conditioned[control][3],
                        conditioned[control][4],conditioned[control][5]};
            }
            if(!std::ranges::all_of(controls,finitePosition))
                return std::unexpected("continuous evenly-spaced spline controls are not finite");
            return controls;
        };

        const auto clusterSplinePiece=[&](std::vector<position_t> controls,
                const std::size_t left,const std::size_t right,
                std::vector<std::size_t> activations)
                -> std::expected<std::vector<GeometryPiece>,std::string> {
            if(!std::ranges::all_of(controls,finitePosition))
                return std::unexpected("continuous evenly-spaced spline controls are not finite");
            auto capturedControls=m_continuousPlanningEffort.captureSplineGeometry
                ?controls:std::vector<position_t>{};
            auto spline=std::make_shared<CubicBSplineReference>(std::move(controls));
            if(!std::isfinite(spline->length())||spline->length()<=1e-12)
                return std::unexpected("continuous path produced a zero-length evenly-spaced spline");
            std::vector<GeometryPiece> result;
            const auto spanDistances=spline->spanDistances();
            constexpr std::size_t timingSpansPerPiece=3;
            struct FeedSection {
                double end=0.0;
                double speed=0.0;
            };
            std::vector<FeedSection> feedSections;
            feedSections.reserve(right-left+1);
            auto sourceLength=0.0;
            const auto appendFeedSection=[&](const double length,const double speed) {
                sourceLength+=length;
                feedSections.push_back({sourceLength,speed/60.0});
            };
            appendFeedSection(3.0*blendScale,entities[left].speed);
            for(auto entity=left+1;entity<right;++entity)
                appendFeedSection(entities[entity].length,entities[entity].speed);
            appendFeedSection(3.0*blendScale,entities[right].speed);
            if(!std::isfinite(sourceLength)||sourceLength<=1e-12)
                return std::unexpected("continuous short-entity spline has invalid source length");

            std::vector<double> pieceBoundaries{0.0};
            pieceBoundaries.reserve((spanDistances.size()-2)/timingSpansPerPiece
                +feedSections.size()+2);
            for(std::size_t span=0;span+1<spanDistances.size();
                    span+=timingSpansPerPiece) {
                const auto finalSpan=std::min(
                    span+timingSpansPerPiece,spanDistances.size()-1);
                pieceBoundaries.push_back(spanDistances[finalSpan]);
            }
            for(std::size_t section=0;section+1<feedSections.size();++section) {
                if(std::abs(feedSections[section].speed-feedSections[section+1].speed)
                    <=1e-12) continue;
                pieceBoundaries.push_back(
                    spline->length()*feedSections[section].end/sourceLength);
            }
            std::ranges::sort(pieceBoundaries);
            const auto duplicateBoundary=[&](const double first,const double second) {
                return std::abs(first-second)<=1e-12*std::max(1.0,spline->length());
            };
            pieceBoundaries.erase(
                std::unique(pieceBoundaries.begin(),pieceBoundaries.end(),duplicateBoundary),
                pieceBoundaries.end());
            if(m_continuousPlanningEffort.captureSplineGeometry)
                splineGeometryDiagnostics.push_back({
                    .firstInput=left,
                    .lastInput=right,
                    .controls=std::move(capturedControls),
                    .pieceBoundaries=pieceBoundaries,
                });
            result.reserve(pieceBoundaries.size()-1);
            for(std::size_t boundary=0;boundary+1<pieceBoundaries.size();++boundary) {
                const auto offset=pieceBoundaries[boundary];
                const auto length=pieceBoundaries[boundary+1]-offset;
                if(!std::isfinite(length)||length<=1e-12)
                    return std::unexpected("continuous evenly-spaced spline has a zero-length knot span");
                const auto sourceMiddle=sourceLength
                    *std::midpoint(offset,pieceBoundaries[boundary+1])/spline->length();
                const auto section=std::ranges::find_if(feedSections,
                    [sourceMiddle](const FeedSection &candidate) {
                        return sourceMiddle<=candidate.end;
                    });
                if(section==feedSections.end())
                    return std::unexpected("continuous short-entity spline feed mapping failed");
                result.push_back({
                    .input=left+1,
                    .activationInputs=boundary==0
                        ?std::move(activations):std::vector<std::size_t>{},
                    .length=length,.speed=section->speed,.linear=false,
                    .sampleAt=[spline,offset,length](const double distance) {
                        const auto source=offset+std::clamp(distance,0.0,length);
                        return PathSample{spline->positionAtDistance(source),
                                          spline->tangentAtDistance(source)};
                    },
                    .curvatureAt=[spline,offset,length](const double distance) {
                        return spline->curvatureAtDistance(
                            offset+std::clamp(distance,0.0,length));
                    },
                    .curvatureDerivativeAt=[spline,offset,length](const double distance) {
                        return spline->curvatureDerivativeAtDistance(
                            offset+std::clamp(distance,0.0,length));
                    },
                    .positionAt=[spline,offset,length](const double distance) {
                        return spline->positionAtDistance(
                            offset+std::clamp(distance,0.0,length));
                    },
                    .chordErrorBound=[spline,offset,length](const double from,const double to) {
                        return spline->chordErrorBound(
                            offset+std::clamp(from,0.0,length),
                            offset+std::clamp(to,0.0,length));
                    },
                });
            }
            return result;
        };

        std::vector<GeometryPiece> pieces;
        pieces.reserve(entities.size()*2-1);
        for(std::size_t index=0;index<entities.size();++index) {
            if((index&15U)==0) reportProgress();
            if(collapsedEntity[index]) continue;
            const auto exactFrom=index==0?0.0:3.0*scales[index];
            const auto exactTo=index+1==entities.size()
                ?entities[index].length:entities[index].length-3.0*scales[index];
            if(exactTo-exactFrom>1e-12)
                pieces.push_back(trimmedPiece(entities[index],exactFrom,exactTo));
            if(index+1==entities.size()) continue;

            const auto cluster=clusterRight[index]!=NO_CLUSTER;
            if(!cluster&&suppressedJunction[index]) continue;
            const auto outgoingIndex=cluster?clusterRight[index]:index+1;

            if(cluster) {
                const auto controls=buildClusterControls(index,outgoingIndex);
                if(!controls) return std::unexpected(std::format(
                    "continuous evenly-spaced cluster {}..{} failed: {}",
                    index,outgoingIndex,controls.error()));
                std::vector<std::size_t> activations;
                for(auto input=index+1;input<=outgoingIndex;++input) {
                    activations.push_back(input);
                }
                auto splinePieces=clusterSplinePiece(std::move(*controls),index,outgoingIndex,
                    std::move(activations));
                if(!splinePieces) return std::unexpected(splinePieces.error());
                pieces.insert(pieces.end(),
                    std::make_move_iterator(splinePieces->begin()),
                    std::make_move_iterator(splinePieces->end()));
                continue;
            }

            const auto &incoming=entities[index];
            const auto &outgoing=entities[outgoingIndex];
            const auto incomingScale=scales[index];
            const auto outgoingScale=scales[outgoingIndex];
            const auto startDistance=incoming.length-3.0*incomingScale;
            const auto endDistance=3.0*outgoingScale;
            const PathSample start{incoming.positionAt(startDistance),incoming.tangentAt(startDistance)};
            const PathSample end{outgoing.positionAt(endDistance),outgoing.tangentAt(endDistance)};
            const auto startCurvature=blendCurvatureAt(index,startDistance);
            const auto endCurvature=blendCurvatureAt(outgoingIndex,endDistance);
            const auto fittedHandle=[](const PathSample &endpoint,const position_t &curvature,
                                       const position_t &twoStepsInside,const double fallbackScale,
                                       const double fallbackTangentDistance) {
                const auto delta=subtract(twoStepsInside,endpoint.position);
                auto tangentDistance=positionDot(delta,endpoint.tangent);
                if(tangentDistance*fallbackTangentDistance<=0.0)
                    tangentDistance=fallbackTangentDistance;
                const auto normalDelta=subtract(delta,scaled(endpoint.tangent,tangentDistance));
                const auto curvatureSquared=positionDot(curvature,curvature);
                auto handle=fallbackScale;
                if(curvatureSquared>1e-18) {
                    const auto handleSquared=positionDot(normalDelta,curvature)/(3.0*curvatureSquared);
                    if(handleSquared>1e-18) handle=std::clamp(std::sqrt(handleSquared),
                        fallbackScale*0.25,fallbackScale*2.0);
                }
                return std::pair{handle,tangentDistance};
            };
            auto [incomingHandle,incomingTangentDistance]=fittedHandle(
                start,startCurvature,incoming.positionAt(incoming.length-incomingScale),
                incomingScale,2.0*incomingScale);
            auto [outgoingHandle,outgoingTangentDistance]=fittedHandle(
                end,endCurvature,outgoing.positionAt(outgoingScale),
                outgoingScale,-2.0*outgoingScale);
            const auto xyzLength=[](const position_t &value) {
                return std::sqrt(value.x*value.x+value.y*value.y+value.z*value.z);
            };
            if(xyzLength(startCurvature)>1e-12||xyzLength(endCurvature)>1e-12) {
                const auto endpoint=[](const PathSample &sample,const position_t &curvature) {
                    return spline_detail::Endpoint3 {
                        .position={sample.position.x,sample.position.y,sample.position.z},
                        .tangent={sample.tangent.x,sample.tangent.y,sample.tangent.z},
                        .curvature={curvature.x,curvature.y,curvature.z},
                    };
                };
                std::tie(incomingHandle,outgoingHandle)=spline_detail::optimizeHandles(
                    endpoint(start,startCurvature),endpoint(end,endCurvature),
                    incomingTangentDistance,outgoingTangentDistance,
                    incomingHandle,outgoingHandle,incomingScale,outgoingScale);
            }
            const std::array<position_t,6> controls {
                start.position,
                add(start.position,scaled(start.tangent,incomingHandle)),
                add(add(start.position,scaled(start.tangent,incomingTangentDistance)),
                    scaled(startCurvature,3.0*incomingHandle*incomingHandle)),
                add(add(end.position,scaled(end.tangent,outgoingTangentDistance)),
                    scaled(endCurvature,3.0*outgoingHandle*outgoingHandle)),
                subtract(end.position,scaled(end.tangent,outgoingHandle)),
                end.position,
            };
            if(!std::ranges::all_of(controls,finitePosition))
                return std::unexpected("continuous B-spline controls are not finite");
            auto spline=std::make_shared<CubicBSplineReference>(controls);
            if(!std::isfinite(spline->length())||spline->length()<=1e-12)
                return std::unexpected("continuous path produced a zero-length B-spline blend");
            if(m_continuousPlanningEffort.captureSplineGeometry)
                splineGeometryDiagnostics.push_back({
                    .firstInput=index,
                    .lastInput=outgoingIndex,
                    .controls=std::vector<position_t>(controls.begin(),controls.end()),
                    .pieceBoundaries={0.0,spline->length()},
                });
            const auto linear=!cluster&&incoming.linear&&outgoing.linear
                &&subtract(start.tangent,end.tangent).length()<=1e-12;
            auto splineSpeed=(incoming.speed+outgoing.speed)/120.0;
            std::vector<std::size_t> splineActivations;
            if(cluster) {
                auto minimumFeed=incoming.speed;
                for(auto input=index+1;input<=outgoingIndex;++input) {
                    minimumFeed=std::min(minimumFeed,entities[input].speed);
                    splineActivations.push_back(input);
                }
                splineSpeed=minimumFeed/60.0;
            }
            pieces.push_back({
                .input=cluster?index+1:outgoingIndex,
                .activationInputs=std::move(splineActivations),
                .length=spline->length(),
                .speed=splineSpeed,
                .linear=linear,
                .sampleAt=[spline,start,end](const double distance) {
                    if(distance<=1e-10) return start;
                    if(spline->length()-distance<=1e-10) return end;
                    return PathSample{spline->positionAtDistance(distance),
                                      spline->tangentAtDistance(distance)};
                },
                .curvatureAt=[spline,startCurvature,endCurvature](const double distance) {
                    if(distance<=1e-10) return startCurvature;
                    if(spline->length()-distance<=1e-10) return endCurvature;
                    return spline->curvatureAtDistance(distance);
                },
                .curvatureDerivativeAt=[spline](const double distance) {
                    return spline->curvatureDerivativeAtDistance(distance);
                },
                .positionAt=[spline](const double distance) {
                    return spline->positionAtDistance(distance);
                },
                .chordErrorBound=[spline](const double from,const double to) {
                    return spline->chordErrorBound(from,to);
                },
            });
        }
        if(pieces.empty()) return std::unexpected("continuous path produced no geometry");

        const auto expectedEnd=expectedStart;
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

        auto result=std::make_unique<ContinuousTrajectoryPlan>();
        std::vector<AxisPolynomialSpan> normalSpans;
        std::vector<std::size_t> normalSpanPieces;
        const auto maximumStagedNormalSpans=std::max<std::size_t>(8192,pieces.size()*8);
        const auto maximumGeometryVerificationAttemptsPerPass=
            std::max<std::size_t>(8192,pieces.size()*3);
        const auto maximumTotalGeometryVerificationAttempts=
            std::max<std::size_t>(32768,pieces.size()*36);
        std::size_t curvedReachabilityStations=0;
        for(std::size_t station=1;station<pieces.size();++station)
            if(!pieces[station-1].linear||!pieces[station].linear)
                ++curvedReachabilityStations;
        const auto linearReachabilityStations=pieces.size()-1-curvedReachabilityStations;
        const auto candidateBudgetMultiplier=
            m_continuousPlanningEffort.candidateBudgetMultiplier;
        const auto maximumCurvedCandidateEvaluationsPerPass=candidateBudgetMultiplier
            *std::max<std::size_t>(32768,curvedReachabilityStations*128);
        const auto maximumLinearCandidateEvaluationsPerPass=candidateBudgetMultiplier
            *std::max<std::size_t>(32768,linearReachabilityStations*512);
        const auto maximumTotalCurvedCandidateEvaluations=candidateBudgetMultiplier
            *std::max<std::size_t>(524288,curvedReachabilityStations*384);
        const auto maximumTotalLinearCandidateEvaluations=candidateBudgetMultiplier
            *std::max<std::size_t>(131072,linearReachabilityStations*2048);
        auto nextChunk=m_nextChunk;
        auto nextSpan=m_nextSpan;
        std::vector<SpanId> activationSpans(commands.size(),0);

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
            constexpr unsigned SAMPLES=64;
            CurvatureDerivativeDiagnostic derivativeDiagnostic;
            for(unsigned sampleIndex=0;sampleIndex<=SAMPLES;++sampleIndex) {
                const auto distance=piece.length*static_cast<double>(sampleIndex)/SAMPLES;
                const auto sample=piece.sampleAt(distance);
                const auto curvature=piece.curvatureAt(distance);
                const auto curvatureDerivative=piece.curvatureDerivativeAt(distance);
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

        constexpr unsigned MAX_LOCAL_CORRECTION_PASSES=24;
        bool constraintsVerified=false;
        std::string correctionHistory;
        std::size_t totalGeometryVerificationAttempts=0;
        std::size_t totalReachabilityCandidateEvaluations=0;
        std::size_t totalCurvedCandidateEvaluations=0;
        std::size_t totalLinearCandidateEvaluations=0;
        TimeLawWorkspace timeLawWorkspace;
        for(unsigned correctionPass=0;correctionPass<MAX_LOCAL_CORRECTION_PASSES;++correctionPass) {
            reportProgress();
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
                auto timing=timeLawBetween(timeLawWorkspace,pieces[pieceIndex].length,
                    stationVelocity[pieceIndex],0.0,stationVelocity[pieceIndex+1],0.0,
                    localLimits[pieceIndex].velocity,localLimits[pieceIndex].acceleration,
                    localLimits[pieceIndex].jerk);
                if(!timing) return std::unexpected(std::format(
                    "continuous reachability seed timing failed at piece {} input {}: {}",
                    pieceIndex,pieces[pieceIndex].input,timing.error()));
                pieceTiming[pieceIndex]=std::move(*timing);
            }
            result->velocityOnlySeedDuration=std::accumulate(
                pieceTiming.begin(),pieceTiming.end(),0.0,
                [](const double total,const auto &timing) { return total+timing.back().time; });

            const auto stateWithinCoupledLimits=[&](const std::size_t pieceIndex,
                    const double distance,const double velocity,const double acceleration) {
                const auto sample=pieces[pieceIndex].sampleAt(distance);
                const auto curvature=pieces[pieceIndex].curvatureAt(distance);
                const auto axisAcceleration=add(scaled(sample.tangent,acceleration),
                    scaled(curvature,velocity*velocity));
                if(axisAcceleration.length()>m_limits.pathAcceleration*(1.0+1e-10)) return false;
                for(const auto component:AXIS_COMPONENTS)
                    if(std::abs(axisAcceleration.*component)
                       >m_limits.axisAcceleration.*component*(1.0+1e-10)) return false;

                const auto geometricJerk=add(scaled(curvature,3.0*velocity*acceleration),
                    scaled(pieces[pieceIndex].curvatureDerivativeAt(distance),
                           velocity*velocity*velocity));
                auto minimumScalarJerk=-localLimits[pieceIndex].jerk;
                auto maximumScalarJerk=localLimits[pieceIndex].jerk;
                for(const auto component:AXIS_COMPONENTS) {
                    const auto tangent=sample.tangent.*component;
                    const auto geometric=geometricJerk.*component;
                    const auto limit=m_limits.axisJerk.*component;
                    if(std::abs(tangent)<=1e-15) {
                        if(std::abs(geometric)>limit*(1.0+1e-10)) return false;
                        continue;
                    }
                    auto lower=(-limit-geometric)/tangent;
                    auto upper=(limit-geometric)/tangent;
                    if(lower>upper) std::swap(lower,upper);
                    minimumScalarJerk=std::max(minimumScalarJerk,lower);
                    maximumScalarJerk=std::min(maximumScalarJerk,upper);
                }
                if(minimumScalarJerk>maximumScalarJerk) return false;
                const auto scalarJerk=std::clamp(-positionDot(sample.tangent,geometricJerk),
                    minimumScalarJerk,maximumScalarJerk);
                return add(scaled(sample.tangent,scalarJerk),geometricJerk).length()
                    <=m_limits.pathJerk*(1.0+1e-10);
            };

            const auto reachabilitySweeps=m_continuousPlanningEffort.reachabilitySweeps;
            const auto reachabilityAccelerationCandidates=
                m_continuousPlanningEffort.accelerationCandidates;
            // Production rolling callers may request a coarser bounded refinement.
            // The standalone diagnostic exporter can explicitly disable the large-
            // horizon cap while leaving production defaults unchanged.
            const auto requestedEffortVelocitySearchIterations=std::max(
                requestedVelocitySearchIterations,
                m_continuousPlanningEffort.minimumVelocitySearchIterations);
            const auto reachabilityVelocitySearchIterations=
                m_continuousPlanningEffort.capLargeHorizonVelocitySearch
                    &&curvedReachabilityStations>1024
                    ?std::min(4U,std::max(1U,requestedEffortVelocitySearchIterations))
                    :std::max(1U,requestedEffortVelocitySearchIterations);
            std::size_t reachabilityCandidateEvaluations=0;
            std::size_t curvedCandidateEvaluations=0;
            std::size_t linearCandidateEvaluations=0;
            std::size_t activeStationVisits=0;
            std::size_t velocityCandidateSets=0;
            std::size_t capVelocitySearches=0;
            std::size_t binaryVelocitySearchSteps=0;
            std::size_t improvedStationVisits=0;
            bool reachabilityCandidateBudgetExceeded=false;
            const auto optimizeStation=[&](const std::size_t station) {
                if(reachabilityCandidateBudgetExceeded) return false;
                const auto leftPiece=station-1;
                const auto rightPiece=station;
                const auto leftSeedSlope=(stationVelocity[station]*stationVelocity[station]
                    -stationVelocity[station-1]*stationVelocity[station-1])
                    /(2.0*pieces[leftPiece].length);
                const auto rightSeedSlope=(stationVelocity[station+1]*stationVelocity[station+1]
                    -stationVelocity[station]*stationVelocity[station])
                    /(2.0*pieces[rightPiece].length);
                const auto atCap=stationCaps[station]-stationVelocity[station]
                    <=std::max(1e-10,stationCaps[station]*1e-9);
                if(atCap||leftSeedSlope*rightSeedSlope<=0.0) return false;
                ++activeStationVisits;
                auto bestVelocity=stationVelocity[station];
                auto bestAcceleration=stationAcceleration[station];
                auto bestLeft=pieceTiming[leftPiece];
                auto bestRight=pieceTiming[rightPiece];
                auto bestDuration=bestLeft.back().time+bestRight.back().time;
                auto improved=false;
                const auto accelerationLimit=0.95*std::min(
                    localLimits[leftPiece].acceleration,localLimits[rightPiece].acceleration);
                const auto curvedCandidate=!pieces[leftPiece].linear||!pieces[rightPiece].linear;

                const auto evaluate=[&](const double velocity,const double acceleration) {
                    ++reachabilityCandidateEvaluations;
                    ++totalReachabilityCandidateEvaluations;
                    if((reachabilityCandidateEvaluations&127U)==0) reportProgress();
                    auto &passEvaluations=curvedCandidate
                        ?curvedCandidateEvaluations:linearCandidateEvaluations;
                    auto &totalEvaluations=curvedCandidate
                        ?totalCurvedCandidateEvaluations:totalLinearCandidateEvaluations;
                    ++passEvaluations;
                    ++totalEvaluations;
                    const auto passBound=curvedCandidate
                        ?maximumCurvedCandidateEvaluationsPerPass
                        :maximumLinearCandidateEvaluationsPerPass;
                    const auto totalBound=curvedCandidate
                        ?maximumTotalCurvedCandidateEvaluations
                        :maximumTotalLinearCandidateEvaluations;
                    if(passEvaluations>passBound||totalEvaluations>totalBound) {
                        reachabilityCandidateBudgetExceeded=true;
                        return false;
                    }
                    if(velocity<0.0||velocity>stationCaps[station]*(1.0+1e-10)
                       ||std::abs(acceleration)>accelerationLimit*(1.0+1e-10)) return false;
                    if(!stateWithinCoupledLimits(leftPiece,pieces[leftPiece].length,
                            velocity,acceleration)
                       ||!stateWithinCoupledLimits(rightPiece,0.0,velocity,acceleration))
                        return false;
                    auto left=timeLawBetween(timeLawWorkspace,pieces[leftPiece].length,
                        stationVelocity[station-1],stationAcceleration[station-1],
                        velocity,acceleration,localLimits[leftPiece].velocity,
                        localLimits[leftPiece].acceleration,localLimits[leftPiece].jerk);
                    if(!left) return false;
                    auto right=timeLawBetween(timeLawWorkspace,pieces[rightPiece].length,
                        velocity,acceleration,stationVelocity[station+1],
                        stationAcceleration[station+1],localLimits[rightPiece].velocity,
                        localLimits[rightPiece].acceleration,localLimits[rightPiece].jerk);
                    if(!right) return false;
                    const auto duration=left->back().time+right->back().time;
                    if(duration<bestDuration-1e-12) {
                        bestDuration=duration;
                        bestVelocity=velocity;
                        bestAcceleration=acceleration;
                        bestLeft=std::move(*left);
                        bestRight=std::move(*right);
                        improved=true;
                    }
                    return true;
                };
                const auto evaluateVelocity=[&](const double velocity) {
                    ++velocityCandidateSets;
                    const auto leftSlope=(velocity*velocity
                        -stationVelocity[station-1]*stationVelocity[station-1])
                        /(2.0*pieces[leftPiece].length);
                    const auto rightSlope=(stationVelocity[station+1]*stationVelocity[station+1]
                        -velocity*velocity)/(2.0*pieces[rightPiece].length);
                    const auto minmod=leftSlope*rightSlope>0.0
                        ?std::copysign(std::min(std::abs(leftSlope),std::abs(rightSlope)),leftSlope)
                        :0.0;
                    std::array<double,MAX_REACHABILITY_ACCELERATION_CANDIDATES> candidates {};
                    std::size_t candidateCount=0;
                    candidates[candidateCount++]=stationAcceleration[station];
                    candidates[candidateCount++]=0.0;
                    candidates[candidateCount++]=minmod;
                    candidates[candidateCount++]=std::midpoint(leftSlope,rightSlope);
                    candidates[candidateCount++]=leftSlope;
                    candidates[candidateCount++]=rightSlope;
                    const auto gridCandidates=reachabilityAccelerationCandidates-candidateCount;
                    for(std::size_t grid=0;grid<gridCandidates;++grid) {
                        const auto fraction=gridCandidates==1?0.5
                            :static_cast<double>(grid)/static_cast<double>(gridCandidates-1);
                        candidates[candidateCount++]=std::lerp(
                            -accelerationLimit,accelerationLimit,fraction);
                    }
                    auto feasible=false;
                    std::array<double,MAX_REACHABILITY_ACCELERATION_CANDIDATES> uniqueCandidates {};
                    std::size_t uniqueCandidateCount=0;
                    const auto accelerationTolerance=std::max(1e-12,accelerationLimit*1e-12);
                    for(std::size_t candidateIndex=0;candidateIndex<candidateCount;
                        ++candidateIndex) {
                        const auto candidate=candidates[candidateIndex];
                        const auto clamped=
                            std::clamp(candidate,-accelerationLimit,accelerationLimit);
                        auto duplicate=false;
                        for(std::size_t index=0;index<uniqueCandidateCount;++index)
                            if(std::abs(uniqueCandidates[index]-clamped)
                               <=accelerationTolerance) {
                                duplicate=true;
                                break;
                            }
                        if(!duplicate) uniqueCandidates[uniqueCandidateCount++]=clamped;
                    }
                    const auto velocityTolerance=
                        std::max(1e-12,stationCaps[station]*1e-12);
                    for(std::size_t index=0;index<uniqueCandidateCount;++index) {
                        const auto candidate=uniqueCandidates[index];
                        if(std::abs(velocity-stationVelocity[station])<=velocityTolerance
                           &&std::abs(candidate-stationAcceleration[station])
                               <=accelerationTolerance) {
                            feasible=true;
                            continue;
                        }
                        feasible=evaluate(velocity,candidate)||feasible;
                        if(reachabilityCandidateBudgetExceeded) break;
                    }
                    return feasible;
                };
                (void)evaluateVelocity(stationVelocity[station]);
                const auto cap=stationCaps[station];
                if(!reachabilityCandidateBudgetExceeded
                   &&cap>stationVelocity[station]+1e-10) {
                    ++capVelocitySearches;
                    if(!evaluateVelocity(cap)) {
                        auto low=stationVelocity[station];
                        auto high=cap;
                        for(unsigned iteration=0;
                            iteration<reachabilityVelocitySearchIterations;++iteration) {
                            ++binaryVelocitySearchSteps;
                            const auto middle=std::midpoint(low,high);
                            if(evaluateVelocity(middle)) low=middle;
                            else high=middle;
                            if(reachabilityCandidateBudgetExceeded) break;
                        }
                    }
                }
                stationVelocity[station]=bestVelocity;
                stationAcceleration[station]=bestAcceleration;
                pieceTiming[leftPiece]=std::move(bestLeft);
                pieceTiming[rightPiece]=std::move(bestRight);
                if(improved) ++improvedStationVisits;
                return improved;
            };
            for(unsigned sweep=0;sweep<reachabilitySweeps;++sweep) {
                auto sweepImproved=false;
                for(std::size_t station=1;station<pieces.size();++station)
                    sweepImproved=optimizeStation(station)||sweepImproved;
                for(std::size_t station=pieces.size();station-->1;)
                    sweepImproved=optimizeStation(station)||sweepImproved;
                if(!sweepImproved||reachabilityCandidateBudgetExceeded) break;
            }
            if(reachabilityCandidateBudgetExceeded)
                return std::unexpected(std::format(
                    "continuous acceleration-aware reachability resource bound exceeded: "
                    "pass={} pass_evaluations={} curved_pass={}/{} linear_pass={}/{} "
                    "total_evaluations={} curved_total={}/{} linear_total={}/{} pieces={} "
                    "curved_stations={} linear_stations={} active_visits={} improvements={} "
                    "velocity_sets={} cap_searches={} binary_steps={}",correctionPass,
                    reachabilityCandidateEvaluations,curvedCandidateEvaluations,
                    maximumCurvedCandidateEvaluationsPerPass,linearCandidateEvaluations,
                    maximumLinearCandidateEvaluationsPerPass,totalReachabilityCandidateEvaluations,
                    totalCurvedCandidateEvaluations,maximumTotalCurvedCandidateEvaluations,
                    totalLinearCandidateEvaluations,maximumTotalLinearCandidateEvaluations,
                    pieces.size(),curvedReachabilityStations,linearReachabilityStations,
                    activeStationVisits,improvedStationVisits,velocityCandidateSets,
                    capVelocitySearches,binaryVelocitySearchSteps));
            result->reachabilityCandidateEvaluations=totalReachabilityCandidateEvaluations;
            const auto reachabilityDuration=std::accumulate(pieceTiming.begin(),pieceTiming.end(),0.0,
                [](const double total,const auto &timing) { return total+timing.back().time; });
            result->accelerationAwareDuration=reachabilityDuration;
            result->ruckigBrakePhases=std::accumulate(
                pieceTiming.begin(),pieceTiming.end(),std::size_t {0},
                [](const std::size_t total,const auto &timing) {
                    return total+std::ranges::count_if(timing,[](const auto &boundary) {
                        return boundary.ruckigBrakePhase;
                    });
                });
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
                    const auto velocity=maximumAxisVelocity(span,component);
                    consider(velocity/(m_limits.axisVelocity.*component),"axis_velocity",
                        AXIS_NAMES[axis],velocity,m_limits.axisVelocity.*component);
                    const auto acceleration=maximumAxisAcceleration(span,component);
                    consider(std::sqrt(acceleration/(m_limits.axisAcceleration.*component)),
                        "axis_acceleration",AXIS_NAMES[axis],acceleration,
                        m_limits.axisAcceleration.*component);
                    const auto jerk=maximumAxisJerk(span,component);
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
                "limit={} measured_over_limit={} timing_candidate={} reachability_duration={} "
                "reachability_sweeps={} acceleration_candidates={} "
                "max_station_acceleration={} station_state=[v={} a={} -> v={} a={}] "
                "local_limits=[v={} a={} j={}]",
                correctionHistory.empty()?"":"; ",correctionPass,worst,worstPiece,
                pieces[worstPiece].input,pieces[worstPiece].linear?"linear":"curved",
                pieces[worstPiece].length,worstViolation.spanId,worstViolation.stagedSpan,
                worstViolation.duration,worstViolation.constraint,worstViolation.axis,
                worstViolation.measured,worstViolation.limit,worstViolation.ratio,
                "acceleration-aware",reachabilityDuration,reachabilitySweeps,
                reachabilityAccelerationCandidates,
                maximumStationAcceleration,
                stationVelocity[worstPiece],stationAcceleration[worstPiece],
                stationVelocity[worstPiece+1],stationAcceleration[worstPiece+1],
                localLimits[worstPiece].velocity,
                localLimits[worstPiece].acceleration,localLimits[worstPiece].jerk);
            if(worst<=1.0+1e-9) {
                constraintsVerified=true;
                break;
            }
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                if(correction[pieceIndex]<=1.0+1e-9) continue;
                const auto factor=correction[pieceIndex]*1.01;
                localLimits[pieceIndex].velocity/=factor;
                localLimits[pieceIndex].acceleration/=factor*factor;
                localLimits[pieceIndex].jerk/=factor*factor*factor;
            }
        }
        if(!constraintsVerified)
            return std::unexpected(std::format(
                "continuous local constraint correction did not converge after {} passes: {}",
                MAX_LOCAL_CORRECTION_PASSES,correctionHistory));

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

        if(oracleModel) {
            ContinuousAccelerationOracleModel model;
            model.pathAcceleration=m_limits.pathAcceleration;
            model.axisAcceleration=m_limits.axisAcceleration;
            model.pathJerk=m_limits.pathJerk;
            model.axisJerk=m_limits.axisJerk;
            model.pieceTiming=result->pieceTiming;
            for(const auto &timing:result->pieceTiming) model.plannerDuration+=timing.duration;
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                const auto &piece=pieces[pieceIndex];
                // Thirty-two intervals per geometry piece let the oracle represent
                // acceleration, cruise, and braking inside long retained lines,
                // while also exposing curved-piece acceleration cones. This work
                // exists only when the optional development model is requested.
                constexpr std::size_t subdivisions=32;
                const auto length=piece.length/static_cast<double>(subdivisions);
                for(std::size_t subdivision=0;subdivision<subdivisions;++subdivision) {
                    const auto distance=length*(static_cast<double>(subdivision)+0.5);
                    const auto sample=piece.sampleAt(distance);
                    model.segments.push_back({
                        .piece=pieceIndex,
                        .input=piece.input,
                        .length=length,
                        .velocityLimit=localLimits[pieceIndex].velocity,
                        .tangent=sample.tangent,
                        .curvature=piece.curvatureAt(distance),
                        .curvatureDerivative=piece.curvatureDerivativeAt(distance),
                    });
                }
            }
            *oracleModel=std::move(model);
        }

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
        result->splineGeometry=std::move(splineGeometryDiagnostics);
        return result;
    }

    std::expected<TriggeredMove, std::string> ExactStopTrajectoryPlanner::compileTriggeredMove(
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
