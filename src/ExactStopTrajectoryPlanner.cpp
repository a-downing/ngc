#include "machine/ExactStopTrajectoryPlanner.h"
#include "machine/ArcInterpolation.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <memory>
#include <numbers>
#include <numeric>
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
        struct TimeBoundary { double time; double distance; double velocity; double acceleration; };

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
                .a=(to.acceleration-from.acceleration)*duration*duration/6.0,
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
            const auto profiles=trajectory.get_profiles().front();
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
            const auto verify=[&](const auto &self,const std::array<position_t,4> &curve,
                                  const double from,const double to,const unsigned depth) -> bool {
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

        std::expected<std::vector<TimeBoundary>, std::string> timeLawBetween(
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
                return std::vector<TimeBoundary> {
                    {0.0,0.0,fromVelocity,0.0},
                    {length/fromVelocity,length,toVelocity,0.0},
                };
            }
            ruckig::InputParameter<1> input;
            input.current_position={0.0};
            input.current_velocity={fromVelocity};
            input.current_acceleration={fromAcceleration};
            input.target_position={length};
            input.target_velocity={toVelocity};
            input.target_acceleration={toAcceleration};
            input.max_velocity = {requestedVelocity}; input.max_acceleration = {acceleration}; input.max_jerk = {jerk};
            ruckig::Ruckig<1> generator;
            ruckig::Trajectory<1> trajectory;
            if(generator.calculate(input, trajectory) != ruckig::Result::Working)
                return std::unexpected(std::format(
                    "Ruckig failed local position timing: length={} state (v={}, a={}) -> "
                    "(v={}, a={}) limits v={} a={} j={}",length,fromVelocity,fromAcceleration,
                    toVelocity,toAcceleration,requestedVelocity,acceleration,jerk));
            std::vector<double> times {0.0};
            for(const auto duration : trajectory.get_profiles().front().front().t) {
                const auto next = times.back() + duration;
                if(next > times.back() + 1e-12) times.push_back(next);
            }
            if(times.back()<trajectory.get_duration()-1e-12)
                times.push_back(trajectory.get_duration());
            std::vector<TimeBoundary> result;
            for(const auto time : times) {
                double position, velocity, scalarAcceleration;
                trajectory.at_time(time, position, velocity, scalarAcceleration);
                result.push_back({time, position, velocity, scalarAcceleration});
            }
            result.front()={0.0,0.0,fromVelocity,fromAcceleration};
            result.back()={trajectory.get_duration(),length,toVelocity,toAcceleration};
            const auto distanceTolerance=std::max(1e-12,length*1e-9);
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
            return result;
        }

        std::expected<std::vector<TimeBoundary>, std::string> timeLaw(
                const double length, const double requestedVelocity, const double acceleration, const double jerk) {
            if(std::isinf(acceleration)) return std::vector<TimeBoundary>{
                {0.0,0.0,requestedVelocity,0.0},
                {length/requestedVelocity,length,requestedVelocity,0.0},
            };
            return timeLawBetween(length,0.0,0.0,0.0,0.0,
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

        template<std::size_t ControlCount, std::size_t KnotCount>
        position_t evaluateBSpline(const std::array<position_t,ControlCount> &controls,
                                   const std::array<double,KnotCount> &knots,
                                   const std::size_t degree, const double requestedParameter) {
            const auto endParameter=knots[ControlCount];
            if(requestedParameter<=knots[degree]) return controls.front();
            if(requestedParameter>=endParameter) return controls.back();
            const auto parameter=std::clamp(requestedParameter,knots[degree],endParameter);
            auto span=degree;
            while(span+1<ControlCount&&parameter>=knots[span+1]) ++span;
            std::array<position_t,ControlCount> work{};
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
            static constexpr std::size_t LENGTH_INTERVALS=96;
            static constexpr std::array<double,10> KNOTS {0,0,0,0,1,2,3,3,3,3};
            static constexpr std::array<double,8> DERIVATIVE_KNOTS {0,0,0,1,2,3,3,3};
            static constexpr std::array<double,6> SECOND_KNOTS {0,0,1,2,3,3};
            std::array<position_t,6> m_controls{};
            std::array<position_t,5> m_derivativeControls{};
            std::array<position_t,4> m_secondControls{};
            std::array<position_t,3> m_thirdControls{};
            struct LengthNode { double parameter=0.0; double distance=0.0; };
            std::array<LengthNode,LENGTH_INTERVALS+1> m_nodes{};
            double m_length=0.0;

            double speed(const double parameter) const { return derivative(parameter).length(); }
            double integratedLength(const double from,const double to) const {
                if(to<=from) return 0.0;
                return integrateAdaptive([&](const double u) { return speed(u); },from,to,
                    1e-12*std::max(1.0,to-from));
            }

        public:
            explicit CubicBSplineReference(const std::array<position_t,6> &controls)
                : m_controls(controls) {
                for(std::size_t index=0;index<m_derivativeControls.size();++index) {
                    const auto denominator=KNOTS[index+4]-KNOTS[index+1];
                    m_derivativeControls[index]=scaled(
                        subtract(m_controls[index+1],m_controls[index]),3.0/denominator);
                }
                for(std::size_t index=0;index<m_secondControls.size();++index) {
                    const auto denominator=DERIVATIVE_KNOTS[index+3]-DERIVATIVE_KNOTS[index+1];
                    m_secondControls[index]=scaled(
                        subtract(m_derivativeControls[index+1],m_derivativeControls[index]),
                        2.0/denominator);
                }
                for(std::size_t index=0;index<m_thirdControls.size();++index) {
                    const auto denominator=SECOND_KNOTS[index+2]-SECOND_KNOTS[index+1];
                    m_thirdControls[index]=scaled(
                        subtract(m_secondControls[index+1],m_secondControls[index]),
                        1.0/denominator);
                }
                for(std::size_t index=1;index<=LENGTH_INTERVALS;++index) {
                    const auto from=3.0*static_cast<double>(index-1)/LENGTH_INTERVALS;
                    const auto to=3.0*static_cast<double>(index)/LENGTH_INTERVALS;
                    m_length+=integratedLength(from,to);
                    m_nodes[index]={to,m_length};
                }
            }

            double length() const { return m_length; }
            position_t position(const double parameter) const {
                return evaluateBSpline(m_controls,KNOTS,3,parameter);
            }
            position_t derivative(const double parameter) const {
                return evaluateBSpline(m_derivativeControls,DERIVATIVE_KNOTS,2,parameter);
            }
            position_t secondDerivative(const double parameter) const {
                return evaluateBSpline(m_secondControls,SECOND_KNOTS,1,parameter);
            }
            position_t thirdDerivative(const double parameter) const {
                const auto segment=std::min<std::size_t>(m_thirdControls.size()-1,
                    static_cast<std::size_t>(std::floor(std::clamp(parameter,0.0,3.0))));
                return m_thirdControls[segment];
            }
            double parameterAtDistance(const double requestedDistance) const {
                if(!std::isfinite(requestedDistance)||!std::isfinite(m_length)||m_length<=1e-15)
                    return 3.0;
                const auto distance=std::clamp(requestedDistance,0.0,m_length);
                if(distance<=0.0) return 0.0;
                if(distance>=m_length) return 3.0;
                const auto upper=std::lower_bound(m_nodes.begin(),m_nodes.end(),distance,
                    [](const LengthNode &node,const double value) { return node.distance<value; });
                const auto lower=upper-1;
                auto from=lower->parameter;
                auto to=upper->parameter;
                for(unsigned iteration=0;iteration<44;++iteration) {
                    const auto middle=std::midpoint(from,to);
                    const auto middleDistance=lower->distance+integratedLength(lower->parameter,middle);
                    if(middleDistance<distance) from=middle;
                    else to=middle;
                }
                return std::midpoint(from,to);
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
                for(const auto knot:{1.0,2.0}) if(knot>from&&knot<to)
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
            const auto timing = timeLaw(length, requestedVelocity, acceleration, jerk);
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
            ContinuousAccelerationOracleModel *oracleModel) {
        if(commands.empty()) return std::unexpected("continuous trajectory window is empty");
        if(blendScale<=0.0) return std::unexpected("continuous trajectory blend scale must be positive");
        if(m_limits.pathAcceleration<=0.0||m_limits.pathJerk<=0.0
           ||m_limits.arcChordTolerance<=0.0||!positiveAxisLimits(m_limits.axisVelocity)
           ||!positiveAxisLimits(m_limits.axisAcceleration)||!positiveAxisLimits(m_limits.axisJerk))
            return std::unexpected("trajectory limits must be positive");

        std::vector<ContinuousEntity> entities;
        entities.reserve(commands.size());
        position_t expectedStart=m_position;
        for(std::size_t input=0;input<commands.size();++input) {
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

        std::vector<double> scales;
        scales.reserve(entities.size());
        for(const auto &entity:entities) scales.push_back(std::min(blendScale,entity.length/6.0));

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

        std::vector<GeometryPiece> pieces;
        pieces.reserve(entities.size()*2-1);
        for(std::size_t index=0;index<entities.size();++index) {
            const auto exactFrom=index==0?0.0:3.0*scales[index];
            const auto exactTo=index+1==entities.size()?entities[index].length
                :entities[index].length-3.0*scales[index];
            if(exactTo-exactFrom>1e-12)
                pieces.push_back(trimmedPiece(entities[index],exactFrom,exactTo));
            if(index+1==entities.size()) continue;

            const auto &incoming=entities[index];
            const auto &outgoing=entities[index+1];
            const auto incomingScale=scales[index];
            const auto outgoingScale=scales[index+1];
            const auto startDistance=incoming.length-3.0*incomingScale;
            const auto endDistance=3.0*outgoingScale;
            const PathSample start{incoming.positionAt(startDistance),incoming.tangentAt(startDistance)};
            const PathSample end{outgoing.positionAt(endDistance),outgoing.tangentAt(endDistance)};
            const auto startCurvature=incoming.curvatureAt(startDistance);
            const auto endCurvature=outgoing.curvatureAt(endDistance);
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
            const auto [incomingHandle,incomingTangentDistance]=fittedHandle(
                start,startCurvature,incoming.positionAt(incoming.length-incomingScale),
                incomingScale,2.0*incomingScale);
            const auto [outgoingHandle,outgoingTangentDistance]=fittedHandle(
                end,endCurvature,outgoing.positionAt(outgoingScale),
                outgoingScale,-2.0*outgoingScale);
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
            const auto linear=incoming.linear&&outgoing.linear
                &&subtract(start.tangent,end.tangent).length()<=1e-12;
            pieces.push_back({
                .input=index+1,.length=spline->length(),
                .speed=(incoming.speed+outgoing.speed)/120.0,
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

        auto result=std::make_unique<ContinuousTrajectoryPlan>();
        std::vector<AxisPolynomialSpan> normalSpans;
        std::vector<std::size_t> normalSpanPieces;
        auto nextChunk=m_nextChunk;
        auto nextSpan=m_nextSpan;
        std::vector<SpanId> activationSpans(commands.size(),0);

        struct LocalLimits { double velocity; double acceleration; double jerk; };
        std::vector<LocalLimits> localLimits;
        localLimits.reserve(pieces.size());
        for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
            const auto &piece=pieces[pieceIndex];
            LocalLimits limits{piece.speed,m_limits.pathAcceleration,m_limits.pathJerk};
            constexpr unsigned SAMPLES=64;
            for(unsigned sampleIndex=0;sampleIndex<=SAMPLES;++sampleIndex) {
                const auto distance=piece.length*static_cast<double>(sampleIndex)/SAMPLES;
                const auto sample=piece.sampleAt(distance);
                const auto curvature=piece.curvatureAt(distance);
                const auto curvatureDerivative=piece.curvatureDerivativeAt(distance);
                for(const auto component:AXIS_COMPONENTS) {
                    const auto tangent=std::abs(sample.tangent.*component);
                    if(tangent>1e-15) {
                        limits.velocity=std::min(limits.velocity,
                            m_limits.axisVelocity.*component/tangent);
                        limits.acceleration=std::min(limits.acceleration,
                            m_limits.axisAcceleration.*component/tangent);
                        limits.jerk=std::min(limits.jerk,m_limits.axisJerk.*component/tangent);
                    }
                    const auto axisCurvature=std::abs(curvature.*component);
                    if(axisCurvature>1e-15)
                        limits.velocity=std::min(limits.velocity,std::sqrt(
                            m_limits.axisAcceleration.*component/axisCurvature));
                }
                const auto curvatureMagnitude=curvature.length();
                if(curvatureMagnitude>1e-15)
                    limits.velocity=std::min(limits.velocity,
                        std::sqrt(m_limits.pathAcceleration/curvatureMagnitude));
                const auto derivativeMagnitude=curvatureDerivative.length();
                if(derivativeMagnitude>1e-15)
                    limits.velocity=std::min(limits.velocity,
                        std::cbrt(m_limits.pathJerk/derivativeMagnitude));
                for(const auto component:AXIS_COMPONENTS) {
                    const auto axisDerivative=std::abs(curvatureDerivative.*component);
                    if(axisDerivative>1e-15)
                        limits.velocity=std::min(limits.velocity,std::cbrt(
                            m_limits.axisJerk.*component/axisDerivative));
                }
            }
            if(limits.velocity<=0.0||limits.acceleration<=0.0||limits.jerk<=0.0
               ||!std::isfinite(limits.velocity)||!std::isfinite(limits.acceleration)
               ||!std::isfinite(limits.jerk))
                return std::unexpected(std::format(
                    "continuous piece {} for input {} has invalid local limits v={} a={} j={}",
                    pieceIndex,piece.input,limits.velocity,limits.acceleration,limits.jerk));
            localLimits.push_back(limits);
        }

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

        constexpr unsigned MAX_LOCAL_CORRECTION_PASSES=12;
        bool constraintsVerified=false;
        std::string correctionHistory;
        for(unsigned correctionPass=0;correctionPass<MAX_LOCAL_CORRECTION_PASSES;++correctionPass) {
            std::vector<double> stationCaps(pieces.size()+1,std::numeric_limits<double>::infinity());
            stationCaps.front()=0.0;
            stationCaps.back()=0.0;
            for(std::size_t station=1;station<pieces.size();++station)
                stationCaps[station]=std::min(localLimits[station-1].velocity,
                                              localLimits[station].velocity);
            std::vector<double> stationVelocity=stationCaps;
            for(unsigned reachabilityPass=0;reachabilityPass<8;++reachabilityPass) {
                auto maximumChange=0.0;
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    const auto reachable=reachableVelocity(stationVelocity[pieceIndex],
                        std::min(stationCaps[pieceIndex+1],localLimits[pieceIndex].velocity),
                        pieces[pieceIndex].length,localLimits[pieceIndex].acceleration,
                        localLimits[pieceIndex].jerk);
                    const auto reduced=std::min(stationVelocity[pieceIndex+1],reachable);
                    maximumChange=std::max(maximumChange,stationVelocity[pieceIndex+1]-reduced);
                    stationVelocity[pieceIndex+1]=reduced;
                }
                for(std::size_t pieceIndex=pieces.size();pieceIndex-->0;) {
                    const auto reachable=reachableVelocity(stationVelocity[pieceIndex+1],
                        std::min(stationCaps[pieceIndex],localLimits[pieceIndex].velocity),
                        pieces[pieceIndex].length,localLimits[pieceIndex].acceleration,
                        localLimits[pieceIndex].jerk);
                    const auto reduced=std::min(stationVelocity[pieceIndex],reachable);
                    maximumChange=std::max(maximumChange,stationVelocity[pieceIndex]-reduced);
                    stationVelocity[pieceIndex]=reduced;
                }
                if(maximumChange<=1e-11) break;
            }

            const auto zeroAccelerationStationVelocity=stationVelocity;
            std::vector<double> stationAcceleration(pieces.size()+1,0.0);
            // Estimate one shared scalar
            // acceleration from adjacent spatial velocity slopes.  The minmod
            // choice carries acceleration only while both sides agree on its
            // direction and returns to zero at speed peaks and valleys.
            for(std::size_t station=1;station<pieces.size();++station) {
                const auto left=(stationVelocity[station]*stationVelocity[station]
                    -stationVelocity[station-1]*stationVelocity[station-1])
                    /(2.0*pieces[station-1].length);
                const auto right=(stationVelocity[station+1]*stationVelocity[station+1]
                    -stationVelocity[station]*stationVelocity[station])
                    /(2.0*pieces[station].length);
                const auto atCap=stationCaps[station]-stationVelocity[station]
                    <=std::max(1e-10,stationCaps[station]*1e-9);
                if(!atCap&&left*right>0.0) {
                    const auto magnitude=std::min(std::abs(left),std::abs(right));
                    const auto limit=0.95*std::min(localLimits[station-1].acceleration,
                                                   localLimits[station].acceleration);
                    stationAcceleration[station]=std::copysign(std::min(magnitude,limit),left);
                }
            }

            // Ruckig remains the exact local feasibility authority.  If the
            // conservative slope estimate still makes an adjacent position solve
            // infeasible, reduce all internal station accelerations together.  The
            // zero-acceleration profile is the proven baseline and must succeed;
            // reaching it without a solve is a fatal planning error.
            std::vector<std::vector<TimeBoundary>> pieceTiming(pieces.size());
            std::string timingFailure;
            bool timingSolved=false;
            unsigned accelerationReductions=0;
            for(unsigned accelerationAttempt=0;accelerationAttempt<16;++accelerationAttempt) {
                timingFailure.clear();
                timingSolved=true;
                for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                    auto timing=timeLawBetween(pieces[pieceIndex].length,
                        stationVelocity[pieceIndex],stationAcceleration[pieceIndex],
                        stationVelocity[pieceIndex+1],stationAcceleration[pieceIndex+1],
                        localLimits[pieceIndex].velocity,localLimits[pieceIndex].acceleration,
                        localLimits[pieceIndex].jerk);
                    if(!timing) {
                        timingFailure=std::format("piece {} input {}: {}",pieceIndex,
                            pieces[pieceIndex].input,timing.error());
                        timingSolved=false;
                        break;
                    }
                    pieceTiming[pieceIndex]=std::move(*timing);
                }
                if(timingSolved) {
                    accelerationReductions=accelerationAttempt;
                    break;
                }
                for(std::size_t station=1;station+1<stationAcceleration.size();++station)
                    stationAcceleration[station]*=0.5;
            }
            if(!timingSolved) return std::unexpected(std::format(
                "continuous acceleration-carrying timing failed after 16 reductions; {}",
                timingFailure));
            std::vector<std::vector<TimeBoundary>> zeroAccelerationTiming(pieces.size());
            auto carriedDuration=0.0;
            auto zeroAccelerationDuration=0.0;
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                carriedDuration+=pieceTiming[pieceIndex].back().time;
                auto timing=timeLawBetween(pieces[pieceIndex].length,
                    zeroAccelerationStationVelocity[pieceIndex],0.0,
                    zeroAccelerationStationVelocity[pieceIndex+1],0.0,
                    localLimits[pieceIndex].velocity,localLimits[pieceIndex].acceleration,
                    localLimits[pieceIndex].jerk);
                if(!timing) return std::unexpected(std::format(
                    "continuous zero-acceleration comparison timing failed at piece {} input {}: {}",
                    pieceIndex,pieces[pieceIndex].input,timing.error()));
                zeroAccelerationDuration+=timing->back().time;
                zeroAccelerationTiming[pieceIndex]=std::move(*timing);
            }
            // Carrying acceleration is an optimization, never permission to make
            // the already-feasible profile slower.  Compare complete scalar
            // horizons and retain the proven zero-acceleration timing when the
            // estimated shared accelerations do not reduce traversal time.
            const auto accelerationCandidateSelected=
                carriedDuration<zeroAccelerationDuration-1e-12;
            if(carriedDuration>=zeroAccelerationDuration-1e-12) {
                stationVelocity=zeroAccelerationStationVelocity;
                std::ranges::fill(stationAcceleration,0.0);
                pieceTiming=std::move(zeroAccelerationTiming);
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
                    .velocityLimit=localLimits[pieceIndex].velocity,
                    .accelerationLimit=localLimits[pieceIndex].acceleration,
                    .jerkLimit=localLimits[pieceIndex].jerk,
                    .entryVelocity=stationVelocity[pieceIndex],
                    .entryAcceleration=stationAcceleration[pieceIndex],
                    .exitVelocity=stationVelocity[pieceIndex+1],
                    .exitAcceleration=stationAcceleration[pieceIndex+1],
                    .duration=pieceTiming[pieceIndex].back().time,
                });
            }

            normalSpans.clear();
            normalSpanPieces.clear();
            std::ranges::fill(activationSpans,SpanId{});
            nextSpan=m_nextSpan;
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                const auto &piece=pieces[pieceIndex];
                const auto &boundaries=pieceTiming[pieceIndex];
                const auto emit=[&](const auto &self,const auto &stateAt,
                                    const double totalDuration,const std::size_t interval,
                                    const double u0,const double u1,
                                    const unsigned depth) -> std::optional<std::string> {
                        auto from=stateAt(u0);
                        auto to=stateAt(u1);
                        const auto duration=totalDuration*(u1-u0);
                        if(duration<=1e-12) return std::nullopt;
                        const auto localFrom=std::clamp(from.distance,0.0,piece.length);
                        const auto localTo=std::clamp(to.distance,0.0,piece.length);
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
                            if(!verifiesOrderedCurveTolerance(chain[chainSpan],
                                std::clamp(source0,localFrom,localTo),
                                std::clamp(source1,localFrom,localTo),m_limits.arcChordTolerance,
                                piece.positionAt,piece.chordErrorBound)) {
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
                            const auto middle=std::midpoint(u0,u1);
                            if(auto error=self(self,stateAt,totalDuration,interval,u0,middle,depth+1))
                                return error;
                            return self(self,stateAt,totalDuration,interval,middle,u1,depth+1);
                        }
                        normalSpans.insert(normalSpans.end(),chain.begin(),chain.end());
                        normalSpanPieces.insert(normalSpanPieces.end(),chain.size(),pieceIndex);
                        if(activationSpans[piece.input]==0)
                            activationSpans[piece.input]=chain.front().id;
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
                "limit={} measured_over_limit={} timing_candidate={} carried_duration={} "
                "zero_acceleration_duration={} selected_duration={} acceleration_reductions={} "
                "max_station_acceleration={} station_state=[v={} a={} -> v={} a={}] "
                "local_limits=[v={} a={} j={}]",
                correctionHistory.empty()?"":"; ",correctionPass,worst,worstPiece,
                pieces[worstPiece].input,pieces[worstPiece].linear?"linear":"curved",
                pieces[worstPiece].length,worstViolation.spanId,worstViolation.stagedSpan,
                worstViolation.duration,worstViolation.constraint,worstViolation.axis,
                worstViolation.measured,worstViolation.limit,worstViolation.ratio,
                accelerationCandidateSelected?"acceleration-carrying":"zero-acceleration",
                carriedDuration,zeroAccelerationDuration,
                accelerationCandidateSelected?carriedDuration:zeroAccelerationDuration,
                accelerationReductions,maximumStationAcceleration,
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
                const auto factor=correction[pieceIndex]*1.001;
                localLimits[pieceIndex].velocity/=factor;
                localLimits[pieceIndex].acceleration/=factor*factor;
                localLimits[pieceIndex].jerk/=factor*factor*factor;
            }
        }
        if(!constraintsVerified)
            return std::unexpected(std::format(
                "continuous local constraint correction did not converge after {} passes: {}",
                MAX_LOCAL_CORRECTION_PASSES,correctionHistory));

        if(oracleModel) {
            ContinuousAccelerationOracleModel model;
            model.pathAcceleration=m_limits.pathAcceleration;
            model.axisAcceleration=m_limits.axisAcceleration;
            for(const auto &timing:result->pieceTiming) model.plannerDuration+=timing.duration;
            for(std::size_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex) {
                const auto &piece=pieces[pieceIndex];
                // Sixteen intervals per geometry piece let the oracle represent
                // acceleration, cruise, and braking inside long retained lines,
                // while also exposing curved-piece acceleration cones. This work
                // exists only when the optional development model is requested.
                constexpr std::size_t subdivisions=16;
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
            if(last==normalSpans.size()) {
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
        m_position=expectedStart;
        result->activationSpans=std::move(activationSpans);
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
