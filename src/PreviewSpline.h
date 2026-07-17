#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <tuple>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "machine/SplineHandleOptimization.h"
#include "machine/SplineReconstruction.h"

namespace ngc::experimental {
    struct JunctionState {
        glm::dvec3 position{};
        glm::dvec3 tangent{};   // Unit tangent in traversal direction.
        glm::dvec3 curvature{}; // d^2 position / d arc-length^2.
        glm::dvec3 curvatureDerivative{};
    };

    struct JunctionEntity {
        double length = 0.0;
        std::function<JunctionState(double)> stateAtDistance;
        double programmedSpeed = 0.0; // Canonical units per minute.
        bool linear = false;
        std::optional<glm::dvec3> midpointCurvature = std::nullopt;
    };

    struct JunctionBlend {
        // One open-clamped, uniform degree-three B-spline. Ordinary junctions
        // have six controls; longer short-entity clusters have one directly
        // sampled interior control per replaced entity.
        std::vector<glm::dvec3> controlPoints;
        std::size_t degree = 3;
        double incomingTrim = 0.0;
        double outgoingTrim = 0.0;
        double incomingScale = 0.0;
        double outgoingScale = 0.0;
    };

    inline double entityBlendScale(const JunctionEntity &entity, const double programmedScale) {
        return std::min(programmedScale, entity.length / 6.0);
    }

    // Build one six-control clamped curve from the endpoint jets at the 3P
    // trims. For lines this places the controls exactly at incoming 3P, 2P, P
    // and outgoing P, 2P, 3P. Curved-entity interior controls leave the entity
    // as needed to match its tangent and geometric curvature. The junction
    // itself is deliberately not a control point.
    inline std::optional<JunctionBlend> fitJunction(const JunctionEntity &incoming,
                                                     const JunctionEntity &outgoing,
                                                     const double programmedScale,
                                                     const bool requireCommonEndpoint=true) {
        constexpr double MINIMUM_LENGTH = 1e-12;
        if(programmedScale <= 0.0 || incoming.length <= MINIMUM_LENGTH
           || outgoing.length <= MINIMUM_LENGTH) return std::nullopt;

        const auto incomingEnd = incoming.stateAtDistance(incoming.length);
        const auto outgoingStart = outgoing.stateAtDistance(0.0);
        const auto positionScale = std::max({programmedScale, incoming.length, outgoing.length, 1.0});
        if(requireCommonEndpoint
           &&glm::length(incomingEnd.position-outgoingStart.position)>positionScale*1e-9)
            return std::nullopt;

        const auto incomingScale = entityBlendScale(incoming, programmedScale);
        const auto outgoingScale = entityBlendScale(outgoing, programmedScale);
        if(incomingScale <= 0.0 || outgoingScale <= 0.0) return std::nullopt;

        const auto incomingTrim = 3.0 * incomingScale;
        const auto outgoingTrim = 3.0 * outgoingScale;
        const auto blendState=[](const JunctionEntity &entity,const double distance) {
            auto state=entity.stateAtDistance(distance);
            if(entity.midpointCurvature
               &&std::abs(distance-0.5*entity.length)<=1e-9*std::max(1.0,entity.length))
                state.curvature=*entity.midpointCurvature;
            return state;
        };
        const auto start = blendState(incoming,incoming.length - incomingTrim);
        const auto end = blendState(outgoing,outgoingTrim);
        const auto fittedHandle = [](const JunctionState &endpoint,
                                     const glm::dvec3 &twoStepsInside,
                                     const double fallbackScale,
                                     const double fallbackTangentDistance) {
            const auto delta = twoStepsInside - endpoint.position;
            auto tangentDistance = glm::dot(delta, endpoint.tangent);
            if(tangentDistance * fallbackTangentDistance <= 0.0)
                tangentDistance = fallbackTangentDistance;
            const auto normalDelta = delta - endpoint.tangent * tangentDistance;
            const auto curvatureSquared = glm::dot(endpoint.curvature, endpoint.curvature);
            auto handle = fallbackScale;
            if(curvatureSquared > 1e-18) {
                const auto handleSquared = glm::dot(normalDelta, endpoint.curvature)
                    / (3.0 * curvatureSquared);
                if(handleSquared > 1e-18) handle = std::clamp(
                    std::sqrt(handleSquared), fallbackScale * 0.25, fallbackScale * 2.0);
            }
            return std::pair {handle, tangentDistance};
        };
        auto [incomingHandle, incomingTangentDistance] = fittedHandle(
            start, incoming.stateAtDistance(incoming.length - incomingScale).position,
            incomingScale, 2.0 * incomingScale);
        auto [outgoingHandle, outgoingTangentDistance] = fittedHandle(
            end, outgoing.stateAtDistance(outgoingScale).position,
            outgoingScale, -2.0 * outgoingScale);
        if(glm::length(start.curvature)>1e-12||glm::length(end.curvature)>1e-12) {
            const auto endpoint=[](const JunctionState &state) {
                return spline_detail::Endpoint3 {
                    .position={state.position.x,state.position.y,state.position.z},
                    .tangent={state.tangent.x,state.tangent.y,state.tangent.z},
                    .curvature={state.curvature.x,state.curvature.y,state.curvature.z},
                };
            };
            std::tie(incomingHandle,outgoingHandle)=spline_detail::optimizeHandles(
                endpoint(start),endpoint(end),incomingTangentDistance,outgoingTangentDistance,
                incomingHandle,outgoingHandle,incomingScale,outgoingScale);
        }
        return JunctionBlend {
            .controlPoints = {
                start.position,
                start.position + start.tangent * incomingHandle,
                start.position + start.tangent * incomingTangentDistance
                    + start.curvature * (3.0 * incomingHandle * incomingHandle),
                end.position + end.tangent * outgoingTangentDistance
                    + end.curvature * (3.0 * outgoingHandle * outgoingHandle),
                end.position - end.tangent * outgoingHandle,
                end.position,
            },
            .incomingTrim = incomingTrim,
            .outgoingTrim = outgoingTrim,
            .incomingScale = incomingScale,
            .outgoingScale = outgoingScale,
        };
    }

    inline std::optional<JunctionBlend> buildEvenlySpacedControlCluster(
            const std::span<const JunctionEntity> entities,const std::size_t left,
            const std::size_t right,const double programmedScale) {
        if(programmedScale<=0.0||right<=left+1||right>=entities.size()) return std::nullopt;
        const auto outer=fitJunction(
            entities[left],entities[right],programmedScale,false);
        if(!outer) return std::nullopt;
        JunctionBlend result{
            .controlPoints={},
            .degree=3,
            .incomingTrim=outer->incomingTrim,
            .outgoingTrim=outer->outgoingTrim,
            .incomingScale=outer->incomingScale,
            .outgoingScale=outer->outgoingScale,
        };
        std::vector<double> interiorLengths;
        interiorLengths.reserve(right-left-1);
        auto interiorLength=0.0;
        for(auto index=left+1;index<right;++index) {
            interiorLength+=entities[index].length;
            interiorLengths.push_back(entities[index].length);
        }
        if(interiorLength<6.0*programmedScale) return outer;
        const auto distances=spline_detail::evenlySpacedCompositeControlDistances(
            interiorLengths,programmedScale);
        if(distances.empty()) return std::nullopt;
        result.controlPoints.reserve(distances.size()+6);
        result.controlPoints.insert(result.controlPoints.end(),
            outer->controlPoints.begin(),outer->controlPoints.begin()+3);
        auto entity=left+1;
        auto entityStart=0.0;
        for(const auto distance:distances) {
            while(entity+1<right
                  &&distance>entityStart+entities[entity].length) {
                entityStart+=entities[entity].length;
                ++entity;
            }
            result.controlPoints.push_back(entities[entity].stateAtDistance(
                std::clamp(distance-entityStart,0.0,entities[entity].length)).position);
        }
        result.controlPoints.insert(result.controlPoints.end(),
            outer->controlPoints.end()-3,outer->controlPoints.end());

        const auto state=[&](const std::size_t entity,const double distance) {
            auto value=entities[entity].stateAtDistance(distance);
            if(entities[entity].midpointCurvature
               &&std::abs(distance-0.5*entities[entity].length)
                    <=1e-9*std::max(1.0,entities[entity].length))
                value.curvature=*entities[entity].midpointCurvature;
            return value;
        };
        struct Segment {
            std::size_t entity=0;
            double entityFrom=0.0;
            double length=0.0;
            double sourceFrom=0.0;
        };
        std::vector<Segment> segments;
        segments.reserve(right-left+1);
        auto sourceLength=0.0;
        const auto append=[&](const std::size_t entity,const double from,const double to) {
            segments.push_back({entity,from,to-from,sourceLength});
            sourceLength+=to-from;
        };
        append(left,entities[left].length-result.incomingTrim,entities[left].length);
        for(auto entity=left+1;entity<right;++entity)
            append(entity,0.0,entities[entity].length);
        append(right,0.0,result.outgoingTrim);
        const auto position=[](const glm::dvec3 &value) {
            return position_t{value.x,value.y,value.z,0.0,0.0,0.0};
        };
        const auto segmentFor=[segments,sourceLength](const double requested) {
            const auto distance=std::clamp(requested,0.0,sourceLength);
            const auto found=std::ranges::find_if(segments,[distance](const auto &segment) {
                return distance<=segment.sourceFrom+segment.length;
            });
            return found==segments.end()?segments.back():*found;
        };
        spline_detail::SplineReconstructionSource source;
        source.length=sourceLength;
        source.positionAt=[&,segmentFor,sourceLength,position](const double requested) {
            const auto distance=std::clamp(requested,0.0,sourceLength);
            const auto segment=segmentFor(distance);
            return position(entities[segment.entity].stateAtDistance(segment.entityFrom
                +std::clamp(distance-segment.sourceFrom,0.0,segment.length)).position);
        };
        const auto start=state(left,entities[left].length-result.incomingTrim);
        const auto end=state(right,result.outgoingTrim);
        source.startTangent=position(start.tangent);
        source.startCurvature=position(start.curvature);
        source.startCurvatureDerivative=position(start.curvatureDerivative);
        source.endTangent=position(end.tangent);
        source.endCurvature=position(end.curvature);
        source.endCurvatureDerivative=position(end.curvatureDerivative);
        std::vector<position_t> cubicControls;
        cubicControls.reserve(result.controlPoints.size());
        for(const auto &control:result.controlPoints) cubicControls.push_back(position(control));
        const auto reconstructed=spline_detail::reconstructSpline(
            cubicControls,source,programmedScale,false);
        if(!reconstructed) return std::nullopt;
        result.degree=reconstructed->degree;
        result.controlPoints.clear();
        result.controlPoints.reserve(reconstructed->controls.size());
        for(const auto &control:reconstructed->controls)
            result.controlPoints.emplace_back(control.x,control.y,control.z);
        return result;
    }

    inline glm::dvec3 evaluateClampedBSpline(const std::vector<glm::dvec3> &controls,
                                             const std::size_t degree,
                                             const double parameter) {
        if(controls.size()<=degree||degree>5) return {};
        const auto maximumParameter=static_cast<double>(controls.size()-degree);
        std::vector<double> knots(controls.size()+degree+1,maximumParameter);
        for(std::size_t index=0;index<=degree;++index) knots[index]=0.0;
        for(std::size_t index=degree+1;index<controls.size();++index)
            knots[index]=static_cast<double>(index-degree);
        if(parameter <= 0.0) return controls.front();
        if(parameter >= maximumParameter) return controls.back();

        const auto span=std::min(controls.size()-1,
            degree+static_cast<std::size_t>(std::floor(parameter)));
        std::array<glm::dvec3,6> work;
        for(std::size_t index=0;index<=degree;++index)
            work[index]=controls[span-degree+index];
        for(std::size_t level=1;level<=degree;++level) {
            for(std::size_t index=degree;index>=level;--index) {
                const auto knotIndex=span-degree+index;
                const auto denominator=knots[knotIndex+degree-level+1]-knots[knotIndex];
                const auto alpha=(parameter-knots[knotIndex])/denominator;
                work[index]=work[index-1]*(1.0-alpha)+work[index]*alpha;
            }
        }
        return work[degree];
    }

    struct GeometricJerkCombSample {
        glm::dvec3 position{};
        glm::dvec3 normalDirection{};
        double magnitude = 0.0;
        double normalMagnitude = 0.0;
        double tangentialMagnitude = 0.0;
        double geometricSpeedLimit = std::numeric_limits<double>::infinity();
        double programmedSpeed = 0.0;
    };

    struct ClusterFeedSection {
        double length = 0.0;
        double speed = 0.0; // Machine units per second.
    };

    // Preview-only analytic samples of the complete arc-length q''' vector.
    // Tooth direction is a readable path-normal indicator; the tooth length,
    // not that display direction, represents |q'''|.
    inline std::vector<GeometricJerkCombSample> sampleGeometricJerkComb(
            const JunctionBlend &blend,
            const std::span<const ClusterFeedSection> feedSections={},
            const double pathJerk=std::numeric_limits<double>::infinity(),
            const unsigned intervalsPerPiece=64) {
        struct Curve {
            std::size_t degree=0;
            std::vector<glm::dvec3> controls;
            std::vector<double> knots;
        };
        if(blend.degree<3||blend.controlPoints.size()<=blend.degree
           ||intervalsPerPiece==0) return {};
        const auto curve=[&] {
            Curve result{blend.degree,blend.controlPoints,{}};
            const auto maximum=static_cast<double>(blend.controlPoints.size()-blend.degree);
            result.knots.assign(blend.controlPoints.size()+blend.degree+1,maximum);
            for(std::size_t index=0;index<=blend.degree;++index) result.knots[index]=0.0;
            for(std::size_t index=blend.degree+1;index<blend.controlPoints.size();++index)
                result.knots[index]=static_cast<double>(index-blend.degree);
            return result;
        }();
        const auto derivative=[](const Curve &source) {
            Curve result;
            if(source.degree==0||source.controls.size()<2) return result;
            result.degree=source.degree-1;
            result.controls.reserve(source.controls.size()-1);
            for(std::size_t index=0;index+1<source.controls.size();++index) {
                const auto denominator=source.knots[index+source.degree+1]
                    -source.knots[index+1];
                result.controls.push_back((source.controls[index+1]-source.controls[index])
                    *(static_cast<double>(source.degree)/denominator));
            }
            result.knots.assign(source.knots.begin()+1,source.knots.end()-1);
            return result;
        };
        const auto first=derivative(curve);
        const auto second=derivative(first);
        const auto third=derivative(second);
        const auto evaluate=[](const Curve &source,const double requested) {
            if(source.controls.empty()) return glm::dvec3{};
            const auto minimum=source.knots[source.degree];
            const auto maximum=source.knots[source.controls.size()];
            if(requested<=minimum) return source.controls.front();
            if(requested>=maximum) return source.controls.back();
            const auto upper=std::ranges::upper_bound(source.knots,requested);
            const auto span=std::clamp<std::size_t>(
                static_cast<std::size_t>(upper-source.knots.begin()-1),
                source.degree,source.controls.size()-1);
            std::array<glm::dvec3,6> work{};
            for(std::size_t index=0;index<=source.degree;++index)
                work[index]=source.controls[span-source.degree+index];
            for(std::size_t level=1;level<=source.degree;++level)
                for(std::size_t index=source.degree;index>=level;--index) {
                    const auto knot=span-source.degree+index;
                    const auto denominator=source.knots[knot+source.degree-level+1]
                        -source.knots[knot];
                    const auto alpha=(requested-source.knots[knot])/denominator;
                    work[index]=work[index-1]*(1.0-alpha)+work[index]*alpha;
                }
            return work[source.degree];
        };
        const auto knotSpans=blend.controlPoints.size()-blend.degree;
        constexpr std::size_t LENGTH_INTERVALS_PER_KNOT_SPAN=32;
        const auto lengthIntervals=knotSpans*LENGTH_INTERVALS_PER_KNOT_SPAN;
        std::vector<double> parameters(lengthIntervals+1);
        std::vector<double> distances(lengthIntervals+1);
        const auto speedAt=[&](const double parameter) {
            return glm::length(evaluate(first,parameter));
        };
        for(std::size_t interval=0;interval<lengthIntervals;++interval) {
            const auto from=static_cast<double>(interval)/LENGTH_INTERVALS_PER_KNOT_SPAN;
            const auto to=static_cast<double>(interval+1)/LENGTH_INTERVALS_PER_KNOT_SPAN;
            const auto middle=std::midpoint(from,to);
            parameters[interval]=from;
            distances[interval+1]=distances[interval]
                +(to-from)*(speedAt(from)+4.0*speedAt(middle)+speedAt(to))/6.0;
        }
        parameters.back()=static_cast<double>(knotSpans);
        const auto totalLength=distances.back();
        if(!std::isfinite(totalLength)||totalLength<=1e-15) return {};
        std::vector<double> pieceBoundaries{0.0,totalLength};
        pieceBoundaries.reserve(knotSpans+feedSections.size()+1);
        for(std::size_t span=1;span<knotSpans;++span)
            pieceBoundaries.push_back(
                distances[span*LENGTH_INTERVALS_PER_KNOT_SPAN]);
        auto sourceLength=0.0;
        for(const auto &section:feedSections) sourceLength+=section.length;
        if(sourceLength>1e-15) {
            auto sourceBoundary=0.0;
            for(std::size_t section=0;section+1<feedSections.size();++section) {
                sourceBoundary+=feedSections[section].length;
                if(std::abs(feedSections[section].speed-feedSections[section+1].speed)>1e-12)
                    pieceBoundaries.push_back(totalLength*sourceBoundary/sourceLength);
            }
        }
        std::ranges::sort(pieceBoundaries);
        pieceBoundaries.erase(std::unique(pieceBoundaries.begin(),pieceBoundaries.end(),
            [totalLength](const double left,const double right) {
                return std::abs(left-right)<=1e-12*std::max(1.0,totalLength);
            }),pieceBoundaries.end());
        const auto parameterAtDistance=[&](const double requested) {
            const auto distance=std::clamp(requested,0.0,totalLength);
            const auto upper=std::ranges::upper_bound(distances,distance);
            if(upper==distances.begin()) return parameters.front();
            if(upper==distances.end()) return parameters.back();
            const auto right=static_cast<std::size_t>(upper-distances.begin());
            const auto left=right-1;
            const auto width=distances[right]-distances[left];
            const auto fraction=width>1e-15?(distance-distances[left])/width:0.0;
            return std::lerp(parameters[left],parameters[right],fraction);
        };
        const auto feedAtDistance=[&](const double distance) {
            if(feedSections.empty()||sourceLength<=1e-15) return 0.0;
            const auto sourceDistance=sourceLength*distance/totalLength;
            auto end=0.0;
            for(const auto &section:feedSections) {
                end+=section.length;
                if(sourceDistance<=end) return section.speed;
            }
            return feedSections.back().speed;
        };
        std::vector<GeometricJerkCombSample> result;
        result.reserve((pieceBoundaries.size()-1)*(intervalsPerPiece+1));
        for(std::size_t piece=0;piece+1<pieceBoundaries.size();++piece)
        for(std::size_t sample=0;sample<=intervalsPerPiece;++sample) {
            const auto distance=std::lerp(pieceBoundaries[piece],pieceBoundaries[piece+1],
                static_cast<double>(sample)/intervalsPerPiece);
            const auto parameter=parameterAtDistance(distance);
            const auto position=evaluate(curve,parameter);
            const auto r1=evaluate(first,parameter);
            const auto r2=evaluate(second,parameter);
            const auto r3=evaluate(third,parameter);
            const auto speed=glm::length(r1);
            if(!std::isfinite(speed)||speed<=1e-15) continue;
            const auto tangent=r1/speed;
            const auto firstSecond=glm::dot(r1,r2);
            const auto inverseSpeed2=1.0/(speed*speed);
            const auto inverseSpeed4=inverseSpeed2*inverseSpeed2;
            const auto inverseSpeed6=inverseSpeed4*inverseSpeed2;
            const auto curvature=r2*inverseSpeed2-r1*(firstSecond*inverseSpeed4);
            const auto parameterCurvatureDerivative=
                r3*inverseSpeed2-r2*(3.0*firstSecond*inverseSpeed4)
                -r1*((glm::dot(r2,r2)+glm::dot(r1,r3))*inverseSpeed4)
                +r1*(4.0*firstSecond*firstSecond*inverseSpeed6);
            const auto geometricJerk=parameterCurvatureDerivative/speed;
            const auto tangential=glm::dot(tangent,geometricJerk);
            const auto normal=geometricJerk-tangent*tangential;
            const auto magnitude=glm::length(geometricJerk);
            if(!std::isfinite(magnitude)) continue;
            auto normalDirection=glm::dvec3{};
            const auto curvatureMagnitude=glm::length(curvature);
            const auto normalMagnitude=glm::length(normal);
            if(curvatureMagnitude>1e-15) normalDirection=curvature/curvatureMagnitude;
            else if(normalMagnitude>1e-15) normalDirection=normal/normalMagnitude;
            else {
                const auto reference=std::abs(tangent.z)<0.9
                    ?glm::dvec3(0.0,0.0,1.0):glm::dvec3(0.0,1.0,0.0);
                normalDirection=glm::normalize(glm::cross(reference,tangent));
            }
            const auto geometricSpeedLimit=magnitude>1e-30&&std::isfinite(pathJerk)
                ?std::cbrt(pathJerk/magnitude):std::numeric_limits<double>::infinity();
            result.push_back({position,normalDirection,magnitude,normalMagnitude,
                              std::abs(tangential),geometricSpeedLimit,
                              feedAtDistance(std::midpoint(
                                  pieceBoundaries[piece],pieceBoundaries[piece+1]))});
        }
        return result;
    }

    inline void tessellateJunction(const JunctionBlend &blend, std::vector<glm::dvec3> &lineVertices) {
        const auto parameterMaximum=static_cast<double>(
            blend.controlPoints.size()-blend.degree);
        const auto sampleCount=24*static_cast<int>(
            blend.controlPoints.size()-blend.degree);
        auto previous = blend.controlPoints.front();
        for(int i = 1; i <= sampleCount; ++i) {
            const auto parameter=parameterMaximum*static_cast<double>(i)/sampleCount;
            const auto current=evaluateClampedBSpline(
                blend.controlPoints,blend.degree,parameter);
            lineVertices.push_back(previous);
            lineVertices.push_back(current);
            previous = current;
        }
    }
}
