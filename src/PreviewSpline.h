#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <numbers>
#include <ranges>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

namespace ngc::experimental {
    struct JunctionState {
        glm::dvec3 position{};
        glm::dvec3 tangent{};          // Unit tangent in traversal direction.
        glm::dvec3 curvature{};        // d^2 position / d arc-length^2.
    };

    struct JunctionEntity {
        double length = 0.0;
        std::function<JunctionState(double)> stateAtDistance;
    };

    struct JunctionBlend {
        // A quintic Bezier is an open, clamped degree-five B-spline. Degree
        // five provides enough freedom to match position, tangent, and
        // curvature independently at both trimmed entity endpoints.
        std::vector<glm::dvec3> controlPoints;
        double incomingTrim = 0.0;
        double outgoingTrim = 0.0;
    };

    struct ClusterBlend {
        std::vector<JunctionBlend> spans;
    };

    inline double distanceToSegment(const glm::dvec3 &point, const glm::dvec3 &from, const glm::dvec3 &to) {
        const auto direction = to - from;
        const auto lengthSquared = glm::dot(direction, direction);
        const auto parameter = lengthSquared > 1e-18
            ? std::clamp(glm::dot(point - from, direction) / lengthSquared, 0.0, 1.0) : 0.0;
        return glm::length(point - (from + parameter * direction));
    }

    inline std::pair<std::vector<glm::dvec3>, std::vector<glm::dvec3>> splitBezier(
            const std::vector<glm::dvec3> &controlPoints) {
        std::vector<std::vector<glm::dvec3>> levels { controlPoints };
        while(levels.back().size() > 1) {
            std::vector<glm::dvec3> next;
            for(std::size_t i = 1; i < levels.back().size(); ++i) {
                next.push_back((levels.back()[i - 1] + levels.back()[i]) * 0.5);
            }
            levels.push_back(std::move(next));
        }
        std::vector<glm::dvec3> left, right;
        left.reserve(controlPoints.size());
        right.reserve(controlPoints.size());
        for(const auto &level : levels) left.push_back(level.front());
        for(auto level = levels.rbegin(); level != levels.rend(); ++level) right.push_back(level->back());
        return { std::move(left), std::move(right) };
    }

    inline bool controlHullInsideTube(const std::vector<glm::dvec3> &controls,
                                      const std::vector<glm::dvec3> &reference,
                                      const double tolerance, const int depth = 0) {
        for(std::size_t segment = 1; segment < reference.size(); ++segment) {
            if(std::ranges::all_of(controls, [&](const glm::dvec3 &point) {
                return distanceToSegment(point, reference[segment - 1], reference[segment]) <= tolerance;
            })) return true;
        }
        if(depth >= 18) return false;
        const auto [left, right] = splitBezier(controls);
        return controlHullInsideTube(left, reference, tolerance, depth + 1)
            && controlHullInsideTube(right, reference, tolerance, depth + 1);
    }

    inline std::vector<double> referenceParameters(const std::vector<glm::dvec3> &reference) {
        std::vector<double> parameters(reference.size(), 0.0);
        for(std::size_t i = 1; i < reference.size(); ++i) {
            parameters[i] = parameters[i - 1] + glm::length(reference[i] - reference[i - 1]);
        }
        const auto total = parameters.back();
        if(total > 1e-15) for(auto &parameter : parameters) parameter /= total;
        parameters.back() = 1.0;
        return parameters;
    }

    inline bool controlHullInsideOrderedTube(const std::vector<glm::dvec3> &controls,
                                             const std::vector<glm::dvec3> &reference,
                                             const std::vector<double> &parameters,
                                             const double u0, const double u1,
                                             const double tolerance, const int depth = 0) {
        for(std::size_t segment = 1; segment < reference.size(); ++segment) {
            if(parameters[segment] + 1e-12 < u0 || parameters[segment - 1] - 1e-12 > u1) continue;
            if(std::ranges::all_of(controls, [&](const glm::dvec3 &point) {
                return distanceToSegment(point, reference[segment - 1], reference[segment]) <= tolerance;
            })) return true;
        }
        if(depth >= 18) return false;
        const auto [left, right] = splitBezier(controls);
        const auto middle = (u0 + u1) * 0.5;
        return controlHullInsideOrderedTube(left, reference, parameters, u0, middle, tolerance, depth + 1)
            && controlHullInsideOrderedTube(right, reference, parameters, middle, u1, tolerance, depth + 1);
    }

    inline std::vector<glm::dvec3> junctionReference(const JunctionEntity &incoming,
                                                     const JunctionEntity &outgoing,
                                                     const double incomingTrim,
                                                     const double outgoingTrim,
                                                     const double tolerance) {
        std::vector<glm::dvec3> reference;
        const auto appendInterval = [&](const JunctionEntity &entity, const double from, const double to) {
            const auto interval = std::max(to - from, 0.0);
            const auto count = std::clamp(static_cast<int>(std::ceil(interval / std::max(tolerance * 0.25, 1e-9))), 4, 256);
            for(int i = 0; i <= count; ++i) {
                const auto distance = from + interval * static_cast<double>(i) / count;
                const auto point = entity.stateAtDistance(distance).position;
                if(reference.empty() || glm::length(reference.back() - point) > 1e-12) reference.push_back(point);
            }
        };
        appendInterval(incoming, incoming.length - incomingTrim, incoming.length);
        appendInterval(outgoing, 0.0, outgoingTrim);
        return reference;
    }

    inline std::optional<JunctionBlend> fitJunction(const JunctionEntity &incoming,
                                                    const JunctionEntity &outgoing,
                                                    const double tolerance) {
        if(tolerance <= 0.0 || incoming.length < 2.0 * tolerance || outgoing.length < 2.0 * tolerance) {
            return std::nullopt;
        }
        const auto incomingEnd = incoming.stateAtDistance(incoming.length);
        const auto outgoingStart = outgoing.stateAtDistance(0.0);
        if(glm::length(incomingEnd.position - outgoingStart.position) > std::max(tolerance * 0.01, 1e-9)) {
            return std::nullopt;
        }
        const auto cosine = std::clamp(glm::dot(incomingEnd.tangent, outgoingStart.tangent), -1.0, 1.0);
        const auto turn = std::acos(cosine);
        if(turn < 1e-5 || turn > std::numbers::pi - 1e-5) return std::nullopt;

        const auto angleDistance = tolerance / std::max(std::tan(turn * 0.25), 1e-6);
        auto trim = std::min({ angleDistance, incoming.length * 0.25, outgoing.length * 0.25 });
        const auto minimumTrim = std::max(std::min(incoming.length, outgoing.length) * 1e-6, 1e-12);
        for(int attempt = 0; attempt < 20 && trim >= minimumTrim; ++attempt, trim *= 0.5) {
            const auto start = incoming.stateAtDistance(incoming.length - trim);
            const auto end = outgoing.stateAtDistance(trim);
            const auto parameterLength = 2.0 * trim;
            const auto startVelocity = start.tangent * parameterLength;
            const auto endVelocity = end.tangent * parameterLength;
            const auto startAcceleration = start.curvature * parameterLength * parameterLength;
            const auto endAcceleration = end.curvature * parameterLength * parameterLength;
            std::vector<glm::dvec3> controls(6);
            controls[0] = start.position;
            controls[1] = controls[0] + startVelocity / 5.0;
            controls[2] = startAcceleration / 20.0 + 2.0 * controls[1] - controls[0];
            controls[5] = end.position;
            controls[4] = controls[5] - endVelocity / 5.0;
            controls[3] = endAcceleration / 20.0 - controls[5] + 2.0 * controls[4];

            const auto reference = junctionReference(incoming, outgoing, trim, trim, tolerance);
            // Half the budget is reserved for entity-to-reference sampling.
            if(controlHullInsideTube(controls, reference, tolerance * 0.5)) {
                return JunctionBlend { .controlPoints = std::move(controls),
                                       .incomingTrim = trim, .outgoingTrim = trim };
            }
        }
        return std::nullopt;
    }

    inline JunctionBlend quinticBetween(const JunctionState &start, const JunctionState &end,
                                         const double parameterLength) {
        const auto startVelocity = start.tangent * parameterLength;
        const auto endVelocity = end.tangent * parameterLength;
        const auto startAcceleration = start.curvature * parameterLength * parameterLength;
        const auto endAcceleration = end.curvature * parameterLength * parameterLength;
        JunctionBlend blend;
        blend.controlPoints.resize(6);
        blend.controlPoints[0] = start.position;
        blend.controlPoints[1] = blend.controlPoints[0] + startVelocity / 5.0;
        blend.controlPoints[2] = startAcceleration / 20.0 + 2.0 * blend.controlPoints[1] - blend.controlPoints[0];
        blend.controlPoints[5] = end.position;
        blend.controlPoints[4] = blend.controlPoints[5] - endVelocity / 5.0;
        blend.controlPoints[3] = endAcceleration / 20.0 - blend.controlPoints[5] + 2.0 * blend.controlPoints[4];
        return blend;
    }

    inline bool fitClusterRecursive(const std::vector<glm::dvec3> &reference,
                                    const std::size_t first, const std::size_t last,
                                    const JunctionState &start, const JunctionState &end,
                                    const double tolerance, std::vector<JunctionBlend> &spans,
                                    const int depth = 0) {
        double length = 0.0;
        for(std::size_t i = first + 1; i <= last; ++i) length += glm::length(reference[i] - reference[i - 1]);
        const auto chordLength = glm::length(end.position - start.position);
        // Using the complete winding subpath length as the endpoint derivative
        // scale creates large handles even when the endpoints are nearby.
        // Chord-limited scaling preserves endpoint geometry without lobes.
        const auto derivativeScale = std::min(length, std::max(chordLength * 1.5, tolerance));
        auto blend = quinticBetween(start, end, std::max(derivativeScale, 1e-9));
        const std::vector<glm::dvec3> localReference(reference.begin() + static_cast<std::ptrdiff_t>(first),
                                                     reference.begin() + static_cast<std::ptrdiff_t>(last + 1));
        const auto parameters = referenceParameters(localReference);
        const auto verified = controlHullInsideOrderedTube(
            blend.controlPoints, localReference, parameters, 0.0, 1.0, tolerance);
        if(verified || depth >= 16 || last <= first + 2) {
            if(!verified) return false;
            spans.push_back(std::move(blend));
            return true;
        }

        const auto split = first + (last - first) / 2;
        const auto tangent = glm::normalize(reference[split + 1] - reference[split - 1]);
        const JunctionState middle {
            .position = reference[split],
            .tangent = tangent,
            // This is an artificial fitting boundary, not a source entity
            // boundary with meaningful curvature. Zero curvature avoids
            // amplifying polygon noise into large quintic control offsets.
            .curvature = {},
        };
        if(!fitClusterRecursive(reference, first, split, start, middle, tolerance, spans, depth + 1)) return false;
        return fitClusterRecursive(reference, split, last, middle, end, tolerance, spans, depth + 1);
    }

    inline std::optional<ClusterBlend> fitCluster(const std::vector<JunctionEntity> &entities,
                                                  const double tolerance) {
        if(entities.size() < 3 || entities.front().length < 2.0 * tolerance
            || entities.back().length < 2.0 * tolerance) return std::nullopt;
        const auto trim = std::min({ tolerance * 2.0, entities.front().length * 0.25,
                                     entities.back().length * 0.25 });
        const auto start = entities.front().stateAtDistance(entities.front().length - trim);
        const auto end = entities.back().stateAtDistance(trim);
        std::vector<glm::dvec3> reference;
        const auto append = [&](const JunctionEntity &entity, const double from, const double to) {
            const auto count = std::clamp(static_cast<int>(std::ceil((to - from) / std::max(tolerance * 0.25, 1e-9))), 2, 128);
            for(int i = 0; i <= count; ++i) {
                const auto point = entity.stateAtDistance(from + (to - from) * static_cast<double>(i) / count).position;
                if(reference.empty() || glm::length(reference.back() - point) > 1e-12) reference.push_back(point);
            }
        };
        append(entities.front(), entities.front().length - trim, entities.front().length);
        for(std::size_t i = 1; i + 1 < entities.size(); ++i) append(entities[i], 0.0, entities[i].length);
        append(entities.back(), 0.0, trim);
        if(reference.size() < 3) return std::nullopt;
        ClusterBlend result;
        // P remains the hard outer bound. Target one tenth of it so accepted
        // cluster geometry stays close to the source instead of merely legal.
        if(!fitClusterRecursive(reference, 0, reference.size() - 1, start, end,
                                tolerance * 0.1, result.spans) || result.spans.empty()) return std::nullopt;
        return result;
    }

    inline glm::dvec3 evaluateBezier(const std::vector<glm::dvec3> &controls, const double u) {
        auto work = controls;
        for(std::size_t level = 1; level < controls.size(); ++level) {
            for(std::size_t i = 0; i + level < controls.size(); ++i) {
                work[i] = work[i] * (1.0 - u) + work[i + 1] * u;
            }
        }
        return work.front();
    }

    inline void tessellateJunction(const JunctionBlend &blend, std::vector<glm::dvec3> &lineVertices) {
        constexpr int SAMPLE_COUNT = 48;
        auto previous = blend.controlPoints.front();
        for(int i = 1; i <= SAMPLE_COUNT; ++i) {
            const auto current = evaluateBezier(blend.controlPoints, static_cast<double>(i) / SAMPLE_COUNT);
            lineVertices.push_back(previous);
            lineVertices.push_back(current);
            previous = current;
        }
    }
}
