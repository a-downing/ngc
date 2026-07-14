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

        AxisPolynomialSpan bezierSpan(const SpanId id,const std::array<position_t,4> &control,
                                      const double duration) {
            AxisPolynomialSpan result;
            result.id=id;
            result.duration=duration;
            result.inverseDuration=1.0/duration;
            result.inverseDurationSquared=result.inverseDuration*result.inverseDuration;
            result.inverseDurationCubed=result.inverseDurationSquared*result.inverseDuration;
            result.d=control[0];
            result.c=scaled(subtract(control[1],control[0]),3.0);
            result.b=scaled(add(subtract(control[2],scaled(control[1],2.0)),control[0]),3.0);
            result.a=add(subtract(control[3],scaled(control[2],3.0)),
                         add(scaled(control[1],3.0),scaled(control[0],-1.0)));
            result.end.position=control[3];
            result.end.velocity=scaled(subtract(control[3],control[2]),3.0/duration);
            result.end.acceleration=scaled(
                add(subtract(control[3],scaled(control[2],2.0)),control[1]),
                6.0/(duration*duration));
            return result;
        }

        std::array<AxisPolynomialSpan,3> c2CubicChain(const SpanId firstId,
                                                      const KinematicPathState &from,
                                                      const KinematicPathState &to,
                                                      const double duration) {
            const auto spanDuration=duration/3.0;
            const auto a0=from.position;
            const auto a1=add(a0,scaled(from.velocity,spanDuration/3.0));
            const auto a2=add(subtract(scaled(a1,2.0),a0),
                              scaled(from.acceleration,spanDuration*spanDuration/6.0));
            const auto c3=to.position;
            const auto c2=subtract(c3,scaled(to.velocity,spanDuration/3.0));
            const auto c1=add(subtract(scaled(c2,2.0),c3),
                              scaled(to.acceleration,spanDuration*spanDuration/6.0));
            const auto a3=scaled(add(subtract(scaled(a2,7.0),scaled(a1,2.0)),
                                     subtract(scaled(c1,2.0),c2)),1.0/6.0);
            const auto b0=a3;
            const auto b1=subtract(scaled(a3,2.0),a2);
            const auto b2=add(subtract(scaled(a3,4.0),scaled(a2,4.0)),a1);
            const auto b3=add(c1,scaled(subtract(b1,c2),0.25));
            const auto c0=b3;
            return {
                bezierSpan(firstId,{a0,a1,a2,a3},spanDuration),
                bezierSpan(firstId+1,{b0,b1,b2,b3},spanDuration),
                bezierSpan(firstId+2,{c0,c1,c2,c3},spanDuration),
            };
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

        std::expected<std::vector<TimeBoundary>, std::string> timeLaw(
                const double length, const double requestedVelocity, const double acceleration, const double jerk) {
            if(std::isinf(acceleration)) {
                return std::vector<TimeBoundary> {
                    { 0.0, 0.0, requestedVelocity, 0.0 },
                    { length / requestedVelocity, length, requestedVelocity, 0.0 },
                };
            }
            ruckig::InputParameter<1> input;
            input.current_position = {0.0}; input.current_velocity = {0.0}; input.current_acceleration = {0.0};
            input.target_position = {length}; input.target_velocity = {0.0}; input.target_acceleration = {0.0};
            input.max_velocity = {requestedVelocity}; input.max_acceleration = {acceleration}; input.max_jerk = {jerk};
            ruckig::Ruckig<1> generator;
            ruckig::Trajectory<1> trajectory;
            if(generator.calculate(input, trajectory) != ruckig::Result::Working)
                return std::unexpected("Ruckig failed to calculate exact-stop timing");
            std::vector<double> times {0.0};
            for(const auto duration : trajectory.get_profiles().front().front().t) {
                const auto next = times.back() + duration;
                if(next > times.back() + 1e-12) times.push_back(next);
            }
            std::vector<TimeBoundary> result;
            for(const auto time : times) {
                double position, velocity, scalarAcceleration;
                trajectory.at_time(time, position, velocity, scalarAcceleration);
                result.push_back({time, position, velocity, scalarAcceleration});
            }
            result.back() = {trajectory.get_duration(), length, 0.0, 0.0};
            return result;
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
            std::function<position_t(double)> positionAt;
            std::function<position_t(double)> tangentAt;
            std::function<position_t(double)> curvatureAt;
            std::function<double(double,double)> chordErrorBound;
        };

        struct GeometryPiece {
            std::size_t input=0;
            double length=0.0;
            std::function<PathSample(double)> sampleAt;
            std::function<position_t(double)> curvatureAt;
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
            const std::span<const MachineCommand> commands, const double blendScale) {
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
                    if(subtract(command.from(),expectedStart).length()>1e-8)
                        return std::unexpected("continuous line does not begin at the planned position");
                    const auto delta=subtract(command.to(),command.from());
                    const auto length=delta.length();
                    if(length<=1e-12) return std::unexpected("continuous path contains a zero-length line");
                    const auto tangent=scaled(delta,1.0/length);
                    return ContinuousEntity {
                        .input=input,.length=length,.speed=command.speed(),
                        .positionAt=[from=command.from(),tangent,length](const double distance) {
                            return add(from,scaled(tangent,std::clamp(distance,0.0,length)));
                        },
                        .tangentAt=[tangent](double) { return tangent; },
                        .curvatureAt=[](double) { return position_t{}; },
                        .chordErrorBound=[](double,double) { return 0.0; },
                    };
                } else if constexpr(std::same_as<T,MoveArc>) {
                    if(command.speed()<=0.0)
                        return std::unexpected("continuous arc must have a positive feed");
                    if(subtract(command.from(),expectedStart).length()>1e-8)
                        return std::unexpected("continuous arc does not begin at the planned position");
                    auto reference=std::make_shared<simulation_detail::ArcReference>(command);
                    if(!reference->valid()||reference->length()<=1e-12)
                        return std::unexpected("continuous path contains an invalid arc");
                    return ContinuousEntity {
                        .input=input,.length=reference->length(),.speed=command.speed(),
                        .positionAt=[reference](const double distance) {
                            return reference->positionAtDistance(distance);
                        },
                        .tangentAt=[reference](const double distance) {
                            return reference->tangentAtDistance(distance);
                        },
                        .curvatureAt=[reference](const double distance) {
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
                .input=entity.input,.length=length,
                .sampleAt=[entity,from,length](const double distance) {
                    const auto source=from+std::clamp(distance,0.0,length);
                    return PathSample{entity.positionAt(source),entity.tangentAt(source)};
                },
                .curvatureAt=[entity,from,length](const double distance) {
                    return entity.curvatureAt(from+std::clamp(distance,0.0,length));
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
            pieces.push_back({
                .input=index+1,.length=spline->length(),
                .sampleAt=[spline](const double distance) {
                    return PathSample{spline->positionAtDistance(distance),
                                      spline->tangentAtDistance(distance)};
                },
                .curvatureAt=[spline](const double distance) {
                    return spline->curvatureAtDistance(distance);
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

        std::vector<double> offsets{0.0};
        offsets.reserve(pieces.size()+1);
        for(const auto &piece:pieces) offsets.push_back(offsets.back()+piece.length);
        const auto totalLength=offsets.back();
        const auto pieceIndexAt=[&](const double distance) {
            if(distance>=totalLength) return pieces.size()-1;
            const auto upper=std::upper_bound(offsets.begin(),offsets.end(),std::max(0.0,distance));
            return std::min<std::size_t>(pieces.size()-1,
                static_cast<std::size_t>(std::distance(offsets.begin(),upper)-1));
        };
        position_t maximumTangent{};
        for(const auto &piece:pieces) {
            constexpr unsigned SAMPLES=64;
            for(unsigned sample=0;sample<=SAMPLES;++sample) {
                const auto tangent=piece.sampleAt(piece.length*static_cast<double>(sample)/SAMPLES).tangent;
                for(const auto component:AXIS_COMPONENTS)
                    maximumTangent.*component=std::max(maximumTangent.*component,
                        std::abs(tangent.*component));
            }
        }
        auto requestedVelocity=std::numeric_limits<double>::infinity();
        for(const auto &entity:entities) requestedVelocity=std::min(requestedVelocity,entity.speed/60.0);
        requestedVelocity=std::min(requestedVelocity,pathLimit(maximumTangent,m_limits.axisVelocity));
        if(requestedVelocity<=0.0||!std::isfinite(requestedVelocity))
            return std::unexpected("continuous path has no valid velocity");
        const auto timing=timeLaw(totalLength,requestedVelocity,m_limits.pathAcceleration,m_limits.pathJerk);
        if(!timing) return std::unexpected(timing.error());

        auto result=std::make_unique<ContinuousTrajectoryPlan>();
        auto &chunk=result->chunk;
        chunk.epoch=m_epoch;
        chunk.id=m_nextChunk;
        chunk.predecessorBranch=m_previousBranch;
        chunk.branch=chunk.id;
        auto nextSpan=m_nextSpan;
        std::vector<SpanId> activationSpans(commands.size(),0);
        const auto kinematicAt=[&](const TimeBoundary &boundary) {
            auto distance=boundary.distance;
            const auto nearby=std::lower_bound(offsets.begin(),offsets.end(),distance);
            if(nearby!=offsets.end()&&std::abs(*nearby-distance)<1e-9) distance=*nearby;
            else if(nearby!=offsets.begin()&&std::abs(*std::prev(nearby)-distance)<1e-9)
                distance=*std::prev(nearby);
            const auto pieceIndex=pieceIndexAt(distance);
            const auto &piece=pieces[pieceIndex];
            auto local=std::clamp(distance-offsets[pieceIndex],0.0,piece.length);
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
        const auto &boundaries=*timing;
        for(std::size_t phase=1;phase<boundaries.size();++phase) {
            const auto scalar=scalarPhase(boundaries[phase-1],boundaries[phase]);
            std::vector<double> cuts{0.0,1.0};
            for(std::size_t boundary=1;boundary+1<offsets.size();++boundary) {
                if(offsets[boundary]<=boundaries[phase-1].distance+1e-12
                   ||offsets[boundary]>=boundaries[phase].distance-1e-12) continue;
                auto from=0.0,to=1.0;
                for(unsigned iteration=0;iteration<52;++iteration) {
                    const auto middle=std::midpoint(from,to);
                    if(scalar.at(middle).distance<offsets[boundary]) from=middle;
                    else to=middle;
                }
                cuts.push_back(std::midpoint(from,to));
            }
            std::ranges::sort(cuts);
            for(std::size_t cut=1;cut<cuts.size();++cut) {
                const auto emit=[&](const auto &self,const double u0,const double u1,
                                    const unsigned depth) -> std::optional<std::string> {
                    const auto from=scalar.at(u0);
                    const auto to=scalar.at(u1);
                    const auto duration=scalar.duration*(u1-u0);
                    if(duration<=1e-12) return std::nullopt;
                    const auto middleDistance=scalar.at(std::midpoint(u0,u1)).distance;
                    const auto pieceIndex=pieceIndexAt(middleDistance);
                    const auto localFrom=std::clamp(from.distance-offsets[pieceIndex],0.0,pieces[pieceIndex].length);
                    const auto localTo=std::clamp(to.distance-offsets[pieceIndex],0.0,pieces[pieceIndex].length);
                    const auto chain=c2CubicChain(nextSpan,kinematicAt(from),kinematicAt(to),duration);
                    auto verified=true;
                    for(std::size_t chainSpan=0;chainSpan<chain.size();++chainSpan) {
                        const auto fraction0=static_cast<double>(chainSpan)/chain.size();
                        const auto fraction1=static_cast<double>(chainSpan+1)/chain.size();
                        const auto source0=scalar.at(std::lerp(u0,u1,fraction0)).distance-offsets[pieceIndex];
                        const auto source1=scalar.at(std::lerp(u0,u1,fraction1)).distance-offsets[pieceIndex];
                        if(!verifiesOrderedCurveTolerance(chain[chainSpan],
                            std::clamp(source0,localFrom,localTo),std::clamp(source1,localFrom,localTo),
                            m_limits.arcChordTolerance,pieces[pieceIndex].positionAt,
                            pieces[pieceIndex].chordErrorBound)) {
                            verified=false;
                            break;
                        }
                    }
                    if(!verified) {
                        if(depth>=20) return "continuous cubic tolerance verification did not converge";
                        const auto middle=std::midpoint(u0,u1);
                        if(auto error=self(self,u0,middle,depth+1)) return error;
                        return self(self,middle,u1,depth+1);
                    }
                    if(chunk.normalMotion.size+chain.size()>MAX_NORMAL_SPANS_PER_CHUNK)
                        return "continuous trajectory chunk span capacity exceeded";
                    for(const auto &span:chain) (void)chunk.normalMotion.push(span);
                    if(activationSpans[pieces[pieceIndex].input]==0)
                        activationSpans[pieces[pieceIndex].input]=chain.front().id;
                    nextSpan+=chain.size();
                    return std::nullopt;
                };
                if(auto error=emit(emit,cuts[cut-1],cuts[cut],0)) return std::unexpected(*error);
            }
        }
        if(chunk.normalMotion.size==0) return std::unexpected("continuous trajectory emitted no motion spans");

        double maximumAcceleration=0.0;
        double maximumJerk=0.0;
        auto axisScaleFactor=1.0;
        for(const auto &span:chunk.normalMotion) {
            maximumAcceleration=std::max(maximumAcceleration,maximumLinearAcceleration(span));
            maximumJerk=std::max(maximumJerk,maximumLinearJerk(span));
            for(const auto component:AXIS_COMPONENTS) {
                axisScaleFactor=std::max(axisScaleFactor,
                    maximumAxisVelocity(span,component)/(m_limits.axisVelocity.*component));
                axisScaleFactor=std::max(axisScaleFactor,std::sqrt(
                    maximumAxisAcceleration(span,component)/(m_limits.axisAcceleration.*component)));
                axisScaleFactor=std::max(axisScaleFactor,std::cbrt(
                    maximumAxisJerk(span,component)/(m_limits.axisJerk.*component)));
            }
        }
        const auto scaleFactor=std::max({1.0,axisScaleFactor,
            std::sqrt(maximumAcceleration/m_limits.pathAcceleration),
            std::cbrt(maximumJerk/m_limits.pathJerk)});
        if(scaleFactor>1.0+1e-9) {
            for(auto &span:chunk.normalMotion) {
                span.duration*=scaleFactor;
                span.inverseDuration/=scaleFactor;
                span.inverseDurationSquared=span.inverseDuration*span.inverseDuration;
                span.inverseDurationCubed=span.inverseDurationSquared*span.inverseDuration;
                span.end.velocity=scaled(span.end.velocity,1.0/scaleFactor);
                span.end.acceleration=scaled(span.end.acceleration,1.0/(scaleFactor*scaleFactor));
            }
        }

        chunk.branchState=chunk.normalMotion[chunk.normalMotion.size-1].end;
        chunk.branchState.velocity={};
        chunk.branchState.acceleration={};
        const PathSample held{chunk.branchState.position,{}};
        auto stop=hermite(nextSpan++,held,held,0.0,0.0,1e-6);
        stop.end=chunk.branchState;
        if(!chunk.stopTail.push(stop))
            return std::unexpected("continuous trajectory stop-tail capacity exceeded");
        chunk.stopState=chunk.branchState;

        for(std::size_t input=1;input<activationSpans.size();++input)
            if(activationSpans[input]==0) activationSpans[input]=activationSpans[input-1];
        m_nextChunk++;
        m_nextSpan=nextSpan;
        m_previousBranch=chunk.branch;
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
