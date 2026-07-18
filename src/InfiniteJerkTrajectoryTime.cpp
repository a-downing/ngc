#include "machine/InfiniteJerkTrajectoryTime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <numeric>
#include <optional>
#include <ranges>
#include <vector>

namespace ngc {
    namespace {
        constexpr std::array AXIS_COMPONENTS {
            &position_t::x,&position_t::y,&position_t::z,
            &position_t::a,&position_t::b,&position_t::c,
        };

        struct AccelerationRange {
            double minimum = -std::numeric_limits<double>::infinity();
            double maximum = std::numeric_limits<double>::infinity();
        };

        struct Interval {
            const InfiniteJerkPathPiece *piece = nullptr;
            double from = 0.0;
            double length = 0.0;
        };

        struct ResolutionResult {
            double duration = 0.0;
            double maximumVelocity = 0.0;
            std::size_t intervals = 0;
        };

        double dot(const position_t &left,const position_t &right) {
            auto result=0.0;
            for(const auto component:AXIS_COMPONENTS)
                result+=(left.*component)*(right.*component);
            return result;
        }

        bool finitePosition(const position_t &value) {
            return std::ranges::all_of(AXIS_COMPONENTS,[&](const auto component) {
                return std::isfinite(value.*component);
            });
        }

        bool positiveLimit(const double value) {
            return value>0.0&&!std::isnan(value);
        }

        std::optional<AccelerationRange> accelerationRange(
                const position_t &tangent,const position_t &curvature,
                const double squaredVelocity,const InfiniteJerkTrajectoryLimits &limits) {
            AccelerationRange result;
            const auto intersect=[&](double lower,double upper) {
                if(lower>upper) std::swap(lower,upper);
                result.minimum=std::max(result.minimum,lower);
                result.maximum=std::min(result.maximum,upper);
                return result.minimum<=result.maximum+1e-12*std::max(
                    {1.0,std::abs(result.minimum),std::abs(result.maximum)});
            };

            if(std::isfinite(limits.pathAcceleration)) {
                const auto tangentSquared=dot(tangent,tangent);
                const auto tangentCurvature=dot(tangent,curvature);
                const auto curvatureSquared=dot(curvature,curvature);
                if(tangentSquared<=1e-24) return std::nullopt;
                const auto linear=tangentCurvature*squaredVelocity;
                const auto constant=curvatureSquared*squaredVelocity*squaredVelocity
                    -limits.pathAcceleration*limits.pathAcceleration;
                auto discriminant=linear*linear-tangentSquared*constant;
                const auto tolerance=1e-12*std::max(
                    1.0,linear*linear+std::abs(tangentSquared*constant));
                if(discriminant<-tolerance) return std::nullopt;
                discriminant=std::max(0.0,discriminant);
                const auto root=std::sqrt(discriminant);
                if(!intersect((-linear-root)/tangentSquared,
                              (-linear+root)/tangentSquared)) return std::nullopt;
            }

            for(const auto component:AXIS_COMPONENTS) {
                const auto limit=limits.axisAcceleration.*component;
                if(!std::isfinite(limit)) continue;
                const auto tangentComponent=tangent.*component;
                const auto geometric=(curvature.*component)*squaredVelocity;
                if(std::abs(tangentComponent)<=1e-15) {
                    if(std::abs(geometric)>limit*(1.0+1e-12)) return std::nullopt;
                    continue;
                }
                if(!intersect((-limit-geometric)/tangentComponent,
                              ( limit-geometric)/tangentComponent)) return std::nullopt;
            }
            if(result.minimum>result.maximum) return std::nullopt;
            return result;
        }

        std::expected<double,std::string> velocitySquaredLimit(
                const InfiniteJerkPathPiece &piece,const double distance,
                const InfiniteJerkTrajectoryLimits &limits) {
            const auto local=std::clamp(distance,0.0,piece.length);
            const auto tangent=piece.tangentAt(local);
            const auto curvature=piece.curvatureAt(local);
            if(!finitePosition(tangent)||!finitePosition(curvature))
                return std::unexpected("infinite-jerk path geometry is not finite");
            const auto tangentLength=std::sqrt(dot(tangent,tangent));
            if(std::abs(tangentLength-1.0)>1e-7)
                return std::unexpected(std::format(
                    "infinite-jerk path tangent is not unit length: magnitude={}",tangentLength));

            auto velocityLimit=piece.velocityLimit;
            for(const auto component:AXIS_COMPONENTS) {
                const auto axisLimit=limits.axisVelocity.*component;
                const auto magnitude=std::abs(tangent.*component);
                if(std::isfinite(axisLimit)&&magnitude>1e-15)
                    velocityLimit=std::min(velocityLimit,axisLimit/magnitude);
            }
            if(!std::isfinite(velocityLimit)||velocityLimit<=0.0)
                return std::unexpected("infinite-jerk path has no finite positive velocity limit");
            const auto upper=velocityLimit*velocityLimit;
            if(accelerationRange(tangent,curvature,upper,limits)) return upper;
            if(!accelerationRange(tangent,curvature,0.0,limits))
                return std::unexpected("infinite-jerk path has no feasible acceleration state at zero speed");
            auto lower=0.0;
            auto higher=upper;
            for(unsigned iteration=0;iteration<48;++iteration) {
                const auto middle=std::midpoint(lower,higher);
                if(accelerationRange(tangent,curvature,middle,limits)) lower=middle;
                else higher=middle;
            }
            return lower;
        }

        std::expected<std::optional<AccelerationRange>,std::string> localAccelerationRange(
                const Interval &interval,const double fraction,const double squaredVelocity,
                const InfiniteJerkTrajectoryLimits &limits) {
            const auto distance=interval.from+std::clamp(fraction,0.0,1.0)*interval.length;
            const auto tangent=interval.piece->tangentAt(distance);
            const auto curvature=interval.piece->curvatureAt(distance);
            if(!finitePosition(tangent)||!finitePosition(curvature))
                return std::unexpected("infinite-jerk path geometry is not finite during integration");
            return accelerationRange(tangent,curvature,std::max(0.0,squaredVelocity),limits);
        }

        std::expected<double,std::string> integrateEnvelope(
                const Interval &interval,const double initialSquaredVelocity,
                const bool forward,const InfiniteJerkTrajectoryLimits &limits) {
            const auto derivative=[&](const double fraction,double squaredVelocity)
                    ->std::expected<double,std::string> {
                squaredVelocity=std::max(0.0,squaredVelocity);
                auto range=localAccelerationRange(interval,fraction,squaredVelocity,limits);
                if(!range) return std::unexpected(range.error());
                if(!*range) {
                    const auto distance=interval.from
                        +std::clamp(fraction,0.0,1.0)*interval.length;
                    auto cap=velocitySquaredLimit(*interval.piece,distance,limits);
                    if(!cap) return std::unexpected(cap.error());
                    range=localAccelerationRange(interval,fraction,
                        std::min(squaredVelocity,*cap)*(1.0-1e-13),limits);
                    if(!range||!*range)
                        return std::unexpected(
                            "infinite-jerk acceleration envelope is numerically empty");
                }
                return 2.0*(forward?(**range).maximum:(**range).minimum);
            };
            const auto step=(forward?1.0:-1.0)*interval.length;
            const auto startFraction=forward?0.0:1.0;
            const auto endFraction=forward?1.0:0.0;
            const auto nonnegative=[](const double value) { return std::max(0.0,value); };
            auto k1=derivative(startFraction,initialSquaredVelocity);
            if(!k1) return std::unexpected(k1.error());
            auto k2=derivative(0.5,nonnegative(initialSquaredVelocity+0.5*step* *k1));
            if(!k2) return std::unexpected(k2.error());
            auto k3=derivative(0.5,nonnegative(initialSquaredVelocity+0.5*step* *k2));
            if(!k3) return std::unexpected(k3.error());
            auto k4=derivative(endFraction,nonnegative(initialSquaredVelocity+step* *k3));
            if(!k4) return std::unexpected(k4.error());
            return nonnegative(initialSquaredVelocity
                +step*(*k1+2.0* *k2+2.0* *k3+*k4)/6.0);
        }

        std::expected<ResolutionResult,std::string> solveResolution(
                const std::span<const InfiniteJerkPathPiece> pieces,
                const InfiniteJerkTrajectoryLimits &limits,
                const double startSquaredVelocity,const double endSquaredVelocity,
                const std::size_t subdivisionsPerPiece,const std::size_t maximumIntervals) {
            std::vector<Interval> intervals;
            std::vector<double> stationCaps;
            stationCaps.push_back(std::numeric_limits<double>::infinity());
            for(const auto &piece:pieces) {
                // Even constant geometry needs stations: the optimal profile can
                // switch between acceleration, cruise, and braking inside it.
                const auto subdivisions=subdivisionsPerPiece;
                if(subdivisions>maximumIntervals-intervals.size())
                    return std::unexpected(std::format(
                        "infinite-jerk timing interval bound exceeded: requested more than {}",
                        maximumIntervals));
                for(std::size_t subdivision=0;subdivision<=subdivisions;++subdivision) {
                    const auto distance=piece.length*static_cast<double>(subdivision)
                        /static_cast<double>(subdivisions);
                    auto cap=velocitySquaredLimit(piece,distance,limits);
                    if(!cap) return std::unexpected(cap.error());
                    if(subdivision==0) stationCaps.back()=std::min(stationCaps.back(),*cap);
                    else stationCaps.push_back(*cap);
                    if(subdivision<subdivisions) intervals.push_back({
                        &piece,distance,piece.length/static_cast<double>(subdivisions)});
                }
            }
            if(startSquaredVelocity>stationCaps.front()*(1.0+1e-12)
               ||endSquaredVelocity>stationCaps.back()*(1.0+1e-12))
                return std::unexpected("infinite-jerk boundary velocity exceeds the path limit");

            std::vector<double> forward(stationCaps.size(),0.0);
            forward.front()=startSquaredVelocity;
            for(std::size_t index=0;index<intervals.size();++index) {
                auto propagated=integrateEnvelope(intervals[index],forward[index],true,limits);
                if(!propagated) return std::unexpected(std::format(
                    "{} at interval {} from_distance={} length={} state={} target_cap={}",
                    propagated.error(),index,intervals[index].from,intervals[index].length,
                    forward[index],stationCaps[index+1]));
                forward[index+1]=std::min(stationCaps[index+1],*propagated);
            }

            std::vector<double> profile(stationCaps.size(),0.0);
            profile.back()=endSquaredVelocity;
            for(std::size_t index=intervals.size();index-->0;) {
                auto propagated=integrateEnvelope(intervals[index],profile[index+1],false,limits);
                if(!propagated) return std::unexpected(std::format(
                    "{} at interval {} from_distance={} length={} target_state={} source_cap={}",
                    propagated.error(),index,intervals[index].from,intervals[index].length,
                    profile[index+1],std::min(stationCaps[index],forward[index])));
                profile[index]=std::min({stationCaps[index],forward[index],*propagated});
            }

            ResolutionResult result;
            result.intervals=intervals.size();
            for(std::size_t index=0;index<intervals.size();++index) {
                const auto fromVelocity=std::sqrt(std::max(0.0,profile[index]));
                const auto toVelocity=std::sqrt(std::max(0.0,profile[index+1]));
                const auto velocitySum=fromVelocity+toVelocity;
                if(velocitySum<=1e-15)
                    return std::unexpected(std::format(
                        "infinite-jerk timing produced a stationary positive-length interval {}",
                        index));
                result.duration+=2.0*intervals[index].length/velocitySum;
                result.maximumVelocity=std::max({result.maximumVelocity,fromVelocity,toVelocity});
            }
            return result;
        }
    }

    std::expected<InfiniteJerkTrajectoryTimeResult, std::string>
    infiniteJerkTrajectoryTime(
            const std::span<const InfiniteJerkPathPiece> pieces,
            const InfiniteJerkTrajectoryLimits &limits,
            const double startVelocity,const double endVelocity,
            const InfiniteJerkTrajectoryTimeOptions &options) {
        if(pieces.empty()) return std::unexpected("infinite-jerk path is empty");
        if(!positiveLimit(limits.pathAcceleration)
           ||!std::ranges::all_of(AXIS_COMPONENTS,[&](const auto component) {
               return positiveLimit(limits.axisVelocity.*component)
                   &&positiveLimit(limits.axisAcceleration.*component);
           })) return std::unexpected("infinite-jerk trajectory limits must be positive");
        const auto hasFiniteAcceleration=std::isfinite(limits.pathAcceleration)
            ||std::ranges::any_of(AXIS_COMPONENTS,[&](const auto component) {
                return std::isfinite(limits.axisAcceleration.*component);
            });
        if(!hasFiniteAcceleration)
            return std::unexpected(
                "infinite-jerk timing requires at least one finite acceleration limit");
        if(!std::isfinite(startVelocity)||startVelocity<0.0
           ||!std::isfinite(endVelocity)||endVelocity<0.0)
            return std::unexpected("infinite-jerk boundary velocities must be finite and non-negative");
        if(options.initialSubdivisions==0||options.maximumRefinements==0
           ||options.maximumIntervals==0||!std::isfinite(options.relativeTolerance)
           ||options.relativeTolerance<=0.0||!std::isfinite(options.absoluteTolerance)
           ||options.absoluteTolerance<=0.0)
            return std::unexpected("infinite-jerk timing options are invalid");
        for(const auto &piece:pieces)
            if(!std::isfinite(piece.length)||piece.length<=0.0
               ||!std::isfinite(piece.velocityLimit)||piece.velocityLimit<=0.0
               ||!piece.tangentAt||!piece.curvatureAt)
                return std::unexpected("infinite-jerk path piece is invalid");

        const auto startSquared=startVelocity*startVelocity;
        const auto endSquared=endVelocity*endVelocity;
        std::optional<ResolutionResult> previous;
        std::optional<std::string> lastResolutionError;
        auto lastDifference=std::numeric_limits<double>::infinity();
        for(std::size_t refinement=0;refinement<=options.maximumRefinements;++refinement) {
            if(refinement>=std::numeric_limits<std::size_t>::digits
               ||options.initialSubdivisions
                    >options.maximumIntervals/(std::size_t {1}<<refinement))
                return std::unexpected("infinite-jerk timing subdivision count overflowed");
            const auto subdivisions=options.initialSubdivisions
                *(std::size_t {1}<<refinement);
            auto current=solveResolution(pieces,limits,startSquared,endSquared,
                                         subdivisions,options.maximumIntervals);
            if(!current) {
                lastResolutionError=current.error();
                continue;
            }
            if(previous) {
                const auto difference=std::abs(current->duration-previous->duration);
                lastDifference=difference;
                const auto tolerance=options.absoluteTolerance
                    +options.relativeTolerance*current->duration;
                if(difference<=tolerance)
                    return InfiniteJerkTrajectoryTimeResult {
                        .duration=current->duration,
                        .estimatedDurationError=difference,
                        .maximumVelocity=current->maximumVelocity,
                        .intervals=current->intervals,
                        .refinements=refinement,
                    };
            }
            previous=*current;
        }
        if(!previous) return std::unexpected(std::format(
            "infinite-jerk timing produced no feasible refined envelope: {}",
            lastResolutionError.value_or("unknown resolution failure")));
        return std::unexpected(std::format(
            "infinite-jerk timing did not converge after {} refinements; "
            "last duration={} last refinement delta={}",
            options.maximumRefinements,previous->duration,lastDifference));
    }
}
