#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

namespace ngc::experimental {
    struct JunctionState {
        glm::dvec3 position{};
        glm::dvec3 tangent{};   // Unit tangent in traversal direction.
        glm::dvec3 curvature{}; // d^2 position / d arc-length^2.
    };

    struct JunctionEntity {
        double length = 0.0;
        std::function<JunctionState(double)> stateAtDistance;
    };

    struct JunctionBlend {
        // One open-clamped degree-three B-spline with six controls.
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
                                                     const double programmedScale) {
        constexpr double MINIMUM_LENGTH = 1e-12;
        if(programmedScale <= 0.0 || incoming.length <= MINIMUM_LENGTH
           || outgoing.length <= MINIMUM_LENGTH) return std::nullopt;

        const auto incomingEnd = incoming.stateAtDistance(incoming.length);
        const auto outgoingStart = outgoing.stateAtDistance(0.0);
        const auto positionScale = std::max({programmedScale, incoming.length, outgoing.length, 1.0});
        if(glm::length(incomingEnd.position - outgoingStart.position) > positionScale * 1e-9)
            return std::nullopt;

        const auto incomingScale = entityBlendScale(incoming, programmedScale);
        const auto outgoingScale = entityBlendScale(outgoing, programmedScale);
        if(incomingScale <= 0.0 || outgoingScale <= 0.0) return std::nullopt;

        const auto incomingTrim = 3.0 * incomingScale;
        const auto outgoingTrim = 3.0 * outgoingScale;
        const auto start = incoming.stateAtDistance(incoming.length - incomingTrim);
        const auto end = outgoing.stateAtDistance(outgoingTrim);
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
        const auto [incomingHandle, incomingTangentDistance] = fittedHandle(
            start, incoming.stateAtDistance(incoming.length - incomingScale).position,
            incomingScale, 2.0 * incomingScale);
        const auto [outgoingHandle, outgoingTangentDistance] = fittedHandle(
            end, outgoing.stateAtDistance(outgoingScale).position,
            outgoingScale, -2.0 * outgoingScale);
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

    inline glm::dvec3 evaluateClampedCubicBSpline(const std::vector<glm::dvec3> &controls,
                                                   const double parameter) {
        constexpr std::size_t DEGREE = 3;
        constexpr std::array knots {0.0,0.0,0.0,0.0,1.0,2.0,3.0,3.0,3.0,3.0};
        if(parameter <= 0.0) return controls.front();
        if(parameter >= 3.0) return controls.back();

        const auto span = parameter < 1.0 ? std::size_t {3}
                        : parameter < 2.0 ? std::size_t {4} : std::size_t {5};
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
        constexpr int SAMPLE_COUNT = 72;
        auto previous = blend.controlPoints.front();
        for(int i = 1; i <= SAMPLE_COUNT; ++i) {
            const auto parameter=3.0*static_cast<double>(i)/SAMPLE_COUNT;
            const auto current=evaluateClampedCubicBSpline(blend.controlPoints,parameter);
            lineVertices.push_back(previous);
            lineVertices.push_back(current);
            previous = current;
        }
    }
}
