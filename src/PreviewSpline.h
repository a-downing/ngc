#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <tuple>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "machine/SplineHandleOptimization.h"

namespace ngc::experimental {
    struct JunctionState {
        glm::dvec3 position{};
        glm::dvec3 tangent{};   // Unit tangent in traversal direction.
        glm::dvec3 curvature{}; // d^2 position / d arc-length^2.
    };

    struct JunctionEntity {
        double length = 0.0;
        std::function<JunctionState(double)> stateAtDistance;
        bool linear = false;
        std::optional<glm::dvec3> midpointCurvature = std::nullopt;
    };

    struct JunctionBlend {
        // One open-clamped, uniform degree-three B-spline. Ordinary junctions
        // have six controls; longer short-entity clusters have one directly
        // sampled interior control per replaced entity.
        std::vector<glm::dvec3> controlPoints;
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
        return result;
    }

    inline glm::dvec3 evaluateClampedCubicBSpline(const std::vector<glm::dvec3> &controls,
                                                   const double parameter) {
        constexpr std::size_t DEGREE = 3;
        if(controls.size()<4) return {};
        const auto maximumParameter=static_cast<double>(controls.size()-3);
        std::vector<double> knots(controls.size()+DEGREE+1,maximumParameter);
        for(std::size_t index=0;index<=DEGREE;++index) knots[index]=0.0;
        for(std::size_t index=DEGREE+1;index<controls.size();++index)
            knots[index]=static_cast<double>(index-DEGREE);
        if(parameter <= 0.0) return controls.front();
        if(parameter >= maximumParameter) return controls.back();

        const auto span=std::min(controls.size()-1,
            DEGREE+static_cast<std::size_t>(std::floor(parameter)));
        std::array<glm::dvec3, DEGREE+1> work;
        for(std::size_t index=0;index<=DEGREE;++index)
            work[index]=controls[span-DEGREE+index];
        for(std::size_t level=1;level<=DEGREE;++level) {
            for(std::size_t index=DEGREE;index>=level;--index) {
                const auto knotIndex=span-DEGREE+index;
                const auto denominator=knots[knotIndex+DEGREE-level+1]-knots[knotIndex];
                const auto alpha=(parameter-knots[knotIndex])/denominator;
                work[index]=work[index-1]*(1.0-alpha)+work[index]*alpha;
            }
        }
        return work[DEGREE];
    }

    inline void tessellateJunction(const JunctionBlend &blend, std::vector<glm::dvec3> &lineVertices) {
        const auto parameterMaximum=static_cast<double>(blend.controlPoints.size()-3);
        const auto sampleCount=24*static_cast<int>(blend.controlPoints.size()-3);
        auto previous = blend.controlPoints.front();
        for(int i = 1; i <= sampleCount; ++i) {
            const auto parameter=parameterMaximum*static_cast<double>(i)/sampleCount;
            const auto current=evaluateClampedCubicBSpline(blend.controlPoints,parameter);
            lineVertices.push_back(previous);
            lineVertices.push_back(current);
            previous = current;
        }
    }
}
