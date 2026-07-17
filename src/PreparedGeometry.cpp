#include "machine/PreparedGeometry.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>

#include "machine/SplineHandleOptimization.h"
#include "machine/SplineReconstruction.h"

namespace ngc {
    namespace {
        position_t scaled(const position_t &value, const double factor) {
            return { value.x * factor, value.y * factor, value.z * factor,
                     value.a * factor, value.b * factor, value.c * factor };
        }

        double dot(const position_t &left, const position_t &right) {
            return left.x * right.x + left.y * right.y + left.z * right.z
                + left.a * right.a + left.b * right.b + left.c * right.c;
        }

        std::vector<double> openKnots(const std::size_t controls, const std::size_t degree) {
            const auto spans = controls - degree;
            std::vector<double> knots(controls + degree + 1, static_cast<double>(spans));
            std::fill_n(knots.begin(), degree + 1, 0.0);
            for(std::size_t index = degree + 1; index < controls; ++index)
                knots[index] = static_cast<double>(index - degree);
            return knots;
        }

        position_t splineAt(const std::size_t degree,
                            const std::span<const position_t> controls,
                            const std::span<const double> knots,
                            double requested,
                            const std::optional<std::size_t> parameterSpan = std::nullopt) {
            constexpr std::size_t MAX_SPLINE_DEGREE = 5;
            if(controls.empty() || degree >= controls.size()) return {};
            if(degree > MAX_SPLINE_DEGREE)
                throw std::runtime_error("prepared spline degree exceeds bounded evaluator storage");
            const auto first = knots[degree];
            const auto last = knots[controls.size()];
            const auto parameter = std::clamp(requested, first, last);
            if(parameter <= first) return controls.front();
            if(parameter >= last) return controls.back();

            auto span = controls.size() - 1;
            if(parameterSpan) {
                span = std::min(degree + *parameterSpan, controls.size() - 1);
            } else {
                const auto upper = std::upper_bound(
                    knots.begin() + static_cast<std::ptrdiff_t>(degree),
                    knots.begin() + static_cast<std::ptrdiff_t>(controls.size() + 1),
                    parameter);
                span = std::clamp<std::size_t>(
                    std::distance(knots.begin(), upper) - 1, degree, controls.size() - 1);
            }
            std::array<position_t, MAX_SPLINE_DEGREE + 1> values{};
            for(std::size_t index = 0; index <= degree; ++index)
                values[index] = controls[span - degree + index];
            for(std::size_t order = 1; order <= degree; ++order) {
                for(std::size_t index = degree; index >= order; --index) {
                    const auto source = span - degree + index;
                    const auto denominator = knots[source + degree + 1 - order]
                        - knots[source];
                    const auto alpha = denominator > 0.0
                        ? (parameter - knots[source]) / denominator : 0.0;
                    values[index] = scaled(values[index - 1], 1.0 - alpha)
                        + scaled(values[index], alpha);
                }
            }
            return values[degree];
        }

        position_t splineAt(const PreparedSplineCurve &spline, const double requested) {
            return splineAt(spline.degree, spline.controls, spline.knots, requested);
        }

        PreparedSplineDerivative derivativeSpline(const std::size_t degree,
                                                   const std::span<const position_t> controls,
                                                   const std::span<const double> knots) {
            PreparedSplineDerivative result{degree,
                std::vector<position_t>(controls.begin(), controls.end()),
                std::vector<double>(knots.begin(), knots.end())};
            if(result.degree > 0 && result.controls.size() > 1) {
                std::vector<position_t> next(result.controls.size() - 1);
                for(std::size_t index = 0; index < next.size(); ++index) {
                    const auto denominator = result.knots[index + result.degree + 1]
                        - result.knots[index + 1];
                    next[index] = denominator > 0.0
                        ? scaled(result.controls[index + 1] - result.controls[index],
                                 static_cast<double>(result.degree) / denominator) : position_t{};
                }
                result.controls = std::move(next);
                result.knots = { result.knots.begin() + 1, result.knots.end() - 1 };
                --result.degree;
            }
            return result;
        }

        std::size_t splineParameterSpan(const PreparedSplineCurve &spline,
                                        const double parameter) {
            const auto spans = spline.controls.size() - spline.degree;
            if(spans <= 1) return 0;
            return std::min(spans - 1, static_cast<std::size_t>(
                std::floor(std::clamp(parameter, 0.0, static_cast<double>(spans)))));
        }

        double derivativeSpeed(const PreparedSplineCurve &spline, const double parameter,
                               const std::optional<std::size_t> parameterSpan = std::nullopt) {
            const auto &derivative = spline.derivatives.front();
            return derivative.controls.empty() ? 0.0
                : splineAt(derivative.degree, derivative.controls,
                           derivative.knots, parameter, parameterSpan).length();
        }

        double integrateSpeed(const PreparedSplineCurve &spline, double from, double to,
                              double *endingSpeed = nullptr,
                              const std::optional<std::size_t> parameterSpan = std::nullopt) {
            if(to <= from) return 0.0;
            constexpr std::size_t intervals = 8;
            const auto step = (to - from) / static_cast<double>(intervals);
            const auto toSpeed = derivativeSpeed(spline, to, parameterSpan);
            if(endingSpeed) *endingSpeed = toSpeed;
            auto total = derivativeSpeed(spline, from, parameterSpan) + toSpeed;
            for(std::size_t index = 1; index < intervals; ++index)
                total += (index & 1U ? 4.0 : 2.0)
                    * derivativeSpeed(spline, from + step * static_cast<double>(index),
                                      parameterSpan);
            return total * step / 3.0;
        }

        std::size_t splineTableIndex(const PreparedSplineCurve &spline, const double distance) {
            if(spline.distances.size() < 2) return 0;
            const auto found = std::upper_bound(spline.distances.begin(), spline.distances.end(), distance);
            if(found == spline.distances.begin()) return 0;
            return std::min<std::size_t>(spline.distances.size() - 2,
                static_cast<std::size_t>(std::distance(spline.distances.begin(), found) - 1));
        }

        std::size_t inverseCacheIndex(const double distance) {
            auto bits = std::bit_cast<std::uint64_t>(distance);
            bits ^= bits >> 33;
            bits *= 0xff51afd7ed558ccdULL;
            bits ^= bits >> 33;
            return static_cast<std::size_t>(bits % 16U);
        }

        CurveEvaluationWorkspace::SplineEntry &splineWorkspace(
                const PreparedCurve &curve, CurveEvaluationWorkspace &workspace) {
            const auto found = std::ranges::find_if(workspace.splines,
                [&](const auto &entry) { return entry.curve == &curve; });
            if(found != workspace.splines.end()) return *found;
            workspace.splines.push_back({ .curve = &curve });
            return workspace.splines.back();
        }

        struct OrderedSplineInverseState {
            std::size_t tableIndex = 0;
            std::size_t parameterSpan = 0;
            double previousDistance = -std::numeric_limits<double>::infinity();
        };

        double splineParameterAtDistance(const PreparedCurve &curve,
                                         const PreparedSplineCurve &spline,
                                         double distance,
                                         CurveEvaluationWorkspace &workspace,
                                         OrderedSplineInverseState *ordered = nullptr) {
            if(spline.distances.empty() || spline.parameters.empty()) return 0.0;
            const auto length = spline.distances.back();
            distance = std::clamp(distance, 0.0, length);
            if(distance == 0.0) {
                if(ordered) {
                    ordered->tableIndex = 0;
                    ordered->parameterSpan = 0;
                    ordered->previousDistance = distance;
                }
                return spline.parameters.front();
            }
            if(distance == length) {
                if(ordered) {
                    ordered->tableIndex = spline.distances.size() - 2;
                    ordered->parameterSpan = spline.controls.size() - spline.degree - 1;
                    ordered->previousDistance = distance;
                }
                return spline.parameters.back();
            }
            auto &cache = splineWorkspace(curve, workspace).inverseCache[
                inverseCacheIndex(distance)];
            if(cache.valid && cache.distance == distance) {
                if(ordered) {
                    ordered->parameterSpan = splineParameterSpan(spline, cache.parameter);
                    ordered->previousDistance = distance;
                }
                return cache.parameter;
            }
            std::size_t index = 0;
            if(ordered && distance >= ordered->previousDistance) {
                index = std::min(ordered->tableIndex, spline.distances.size() - 2);
                while(index + 1 < spline.distances.size() - 1
                      && spline.distances[index + 1] <= distance) ++index;
                ordered->tableIndex = index;
            } else {
                index = splineTableIndex(spline, distance);
                if(ordered) ordered->tableIndex = index;
            }
            const auto integrationStart = spline.parameters[index];
            const auto parameterSpan = splineParameterSpan(
                spline, std::midpoint(integrationStart, spline.parameters[index + 1]));
            if(ordered) ordered->parameterSpan = parameterSpan;
            auto low = integrationStart;
            auto high = spline.parameters[index + 1];
            const auto bracket = spline.distances[index + 1] - spline.distances[index];
            auto parameter = bracket > 0.0
                ? std::lerp(low, high, (distance - spline.distances[index]) / bracket)
                : low;
            // The table is a deterministic bracket only. Reintegrate against
            // the actual curve before accepting the inverse.
            for(unsigned iteration = 0; iteration < 12; ++iteration) {
                double speed = 0.0;
                const auto traveled = spline.distances[index]
                    + integrateSpeed(spline, integrationStart, parameter, &speed,
                                     parameterSpan);
                if(std::abs(traveled - distance) <= 1e-12 * std::max(1.0, length)) {
                    cache = { distance, parameter, true };
                    if(ordered) ordered->previousDistance = distance;
                    return parameter;
                }
                if(traveled < distance) low = parameter; else high = parameter;
                const auto next = speed > 1e-15
                    ? parameter - (traveled - distance) / speed
                    : std::midpoint(low, high);
                parameter = std::clamp(next, low, high);
            }
            cache = { distance, parameter, true };
            if(ordered) ordered->previousDistance = distance;
            return parameter;
        }

        std::shared_ptr<const PreparedCurve> lineCurve(const position_t &from,
                                                        const position_t &to) {
            return std::make_shared<const PreparedCurve>(PreparedCurve{
                PreparedLineCurve{from, to}, (to - from).length()});
        }

        std::shared_ptr<const PreparedCurve> arcCurve(const MoveArc &arc) {
            simulation_detail::ArcReference reference(arc);
            if(!reference.valid() || reference.length() <= 1e-12) return {};
            return std::make_shared<const PreparedCurve>(PreparedCurve{
                PreparedArcCurve{arc}, reference.length()});
        }

        std::shared_ptr<const PreparedCurve> splineCurve(std::vector<position_t> controls,
                                                         std::size_t degree,
                                                         const std::size_t intervalsPerSpan) {
            if(degree == 0 || controls.size() <= degree) return {};
            PreparedSplineCurve spline;
            spline.degree = degree;
            spline.controls = std::move(controls);
            spline.knots = openKnots(spline.controls.size(), degree);
            spline.derivatives[0] = derivativeSpline(
                spline.degree, spline.controls, spline.knots);
            for(std::size_t order = 1; order < spline.derivatives.size(); ++order) {
                const auto &previous = spline.derivatives[order - 1];
                spline.derivatives[order] = derivativeSpline(
                    previous.degree, previous.controls, previous.knots);
            }
            const auto spans = spline.controls.size() - degree;
            const auto displacement = spline.controls.back() - spline.controls.front();
            const auto displacementSquared = dot(displacement, displacement);
            auto collinear = displacementSquared > 1e-24;
            for(const auto &control : spline.controls) {
                const auto fraction = displacementSquared > 0.0
                    ? dot(control - spline.controls.front(), displacement) / displacementSquared : 0.0;
                if((control - (spline.controls.front() + scaled(displacement, fraction))).length()
                        > 1e-10 * std::max(1.0, displacement.length())) {
                    collinear = false;
                    break;
                }
            }
            if(collinear) {
                // Collinear junctions are geometrically exact lines. Retain
                // the spline controls/knots for presentation identity, but
                // use an exact chord-length bracket and avoid rebuilding a
                // costly numerical table for every straight G64 junction.
                spline.parameters = { 0.0, static_cast<double>(spans) };
                spline.distances = { 0.0, displacement.length() };
                return std::make_shared<const PreparedCurve>(PreparedCurve{
                    std::move(spline), displacement.length()});
            }
            const auto intervals = spans * std::max<std::size_t>(1, intervalsPerSpan);
            spline.parameters.reserve(intervals + 1);
            spline.distances.reserve(intervals + 1);
            spline.parameters.push_back(0.0);
            spline.distances.push_back(0.0);
            auto distance = 0.0;
            for(std::size_t index = 1; index <= intervals; ++index) {
                const auto parameter = static_cast<double>(spans) * index / intervals;
                const auto parameterSpan = std::min(
                    spans - 1, (index - 1) / std::max<std::size_t>(1, intervalsPerSpan));
                distance += integrateSpeed(spline, spline.parameters.back(), parameter,
                                           nullptr, parameterSpan);
                spline.parameters.push_back(parameter);
                spline.distances.push_back(distance);
            }
            if(!std::isfinite(distance) || distance <= 1e-12) return {};
            for(const auto &control : spline.derivatives[1].controls)
                spline.maximumSecondDerivative = std::max(
                    spline.maximumSecondDerivative, control.length());
            return std::make_shared<const PreparedCurve>(PreparedCurve{
                std::move(spline), distance});
        }

        const simulation_detail::ArcReference *arcReference(const PreparedCurve &curve,
                                                              CurveEvaluationWorkspace &workspace) {
            for(auto &entry : workspace.arcs)
                if(entry.curve == &curve) return entry.reference.get();
            const auto *arc = std::get_if<PreparedArcCurve>(&curve.value);
            if(!arc) return nullptr;
            workspace.arcs.push_back({ &curve,
                std::make_unique<simulation_detail::ArcReference>(arc->arc) });
            return workspace.arcs.back().reference.get();
        }

        position_t derivativeAt(const PreparedSplineCurve &spline, double parameter,
                                unsigned order,
                                const std::optional<std::size_t> parameterSpan = std::nullopt) {
            if(order == 0)
                return splineAt(spline.degree, spline.controls, spline.knots,
                                parameter, parameterSpan);
            if(order > spline.derivatives.size()) return {};
            const auto &derivative = spline.derivatives[order - 1];
            if(derivative.controls.empty()) return {};
            return splineAt(derivative.degree, derivative.controls,
                            derivative.knots, parameter, parameterSpan);
        }

        PreparedGeometryBoundary splineBoundaryAtParameter(
                const PreparedSplineCurve &spline, const double parameter,
                const std::size_t parameterSpan) {
            PreparedGeometryBoundary result;
            result.position = derivativeAt(spline, parameter, 0, parameterSpan);
            const auto first = derivativeAt(spline, parameter, 1, parameterSpan);
            const auto second = derivativeAt(spline, parameter, 2, parameterSpan);
            const auto third = derivativeAt(spline, parameter, 3, parameterSpan);
            const auto speed = first.length();
            if(speed <= 1e-15) return result;

            result.tangent = scaled(first, 1.0 / speed);
            result.curvature = scaled(
                second - scaled(result.tangent, dot(second, result.tangent)),
                1.0 / (speed * speed));

            const auto firstSecond = dot(first, second);
            const auto secondSquared = dot(second, second);
            const auto firstThird = dot(first, third);
            const auto inverseSpeed2 = 1.0 / (speed * speed);
            const auto inverseSpeed4 = inverseSpeed2 * inverseSpeed2;
            const auto inverseSpeed6 = inverseSpeed4 * inverseSpeed2;
            const auto parameterDerivative = scaled(third, inverseSpeed2)
                + scaled(second, -3.0 * firstSecond * inverseSpeed4)
                + scaled(first, -(secondSquared + firstThird) * inverseSpeed4)
                + scaled(first, 4.0 * firstSecond * firstSecond * inverseSpeed6);
            result.curvatureDerivative = scaled(parameterDerivative, 1.0 / speed);
            return result;
        }

        PreparedGeometryBoundary boundaryAt(const PreparedPathPiece &piece,
                                             CurveEvaluationWorkspace &workspace,
                                             const double distance) {
            return {
                positionAtDistance(*piece.curve, piece.curveFrom + distance, workspace),
                tangentAtDistance(*piece.curve, piece.curveFrom + distance, workspace),
                curvatureAtDistance(*piece.curve, piece.curveFrom + distance, workspace),
                curvatureDerivativeAtDistance(*piece.curve, piece.curveFrom + distance, workspace),
            };
        }

        void samplePiece(PreparedPathPiece &piece, CurveEvaluationWorkspace &workspace) {
            constexpr unsigned SAMPLES = 64;
            piece.geometricSamples.reserve(SAMPLES + 1);
            OrderedSplineInverseState orderedSpline;
            const auto *spline = std::get_if<PreparedSplineCurve>(&piece.curve->value);
            for(unsigned index = 0; index <= SAMPLES; ++index) {
                const auto distance = piece.length() * index / static_cast<double>(SAMPLES);
                PreparedGeometryBoundary boundary;
                if(spline) {
                    const auto parameter = splineParameterAtDistance(
                        *piece.curve, *spline, piece.curveFrom + distance,
                        workspace, &orderedSpline);
                    boundary = splineBoundaryAtParameter(
                        *spline, parameter, orderedSpline.parameterSpan);
                } else boundary = boundaryAt(piece, workspace, distance);
                const auto tangential = dot(boundary.tangent, boundary.curvatureDerivative);
                const auto normal = boundary.curvatureDerivative - scaled(boundary.tangent, tangential);
                piece.geometricSamples.push_back({ distance, boundary.position, boundary.tangent,
                    boundary.curvature, boundary.curvatureDerivative, normal.length(),
                    boundary.curvatureDerivative.length() });
            }
        }

        struct SourceEntity {
            PreparedCommandId id = 0;
            std::shared_ptr<const PreparedCurve> curve;
            double feed = 0.0;
            bool linear = false;
            double length = 0.0;
        };

        double sourceScale(const SourceEntity &entity, const double blendScale) {
            return std::min(blendScale, entity.length / 6.0);
        }

        position_t sourcePosition(const SourceEntity &entity, double distance,
                                  CurveEvaluationWorkspace &workspace) {
            return positionAtDistance(*entity.curve, distance, workspace);
        }

        position_t sourceTangent(const SourceEntity &entity, double distance,
                                 CurveEvaluationWorkspace &workspace) {
            return tangentAtDistance(*entity.curve, distance, workspace);
        }

        std::vector<position_t> curvatureMatchedSixControlBlend(
                                                const SourceEntity &incoming,
                                                const SourceEntity &outgoing,
                                                const double incomingScale,
                                                const double outgoingScale,
                                                CurveEvaluationWorkspace &workspace) {
            const auto startDistance = incoming.length - 3.0 * incomingScale;
            const auto endDistance = 3.0 * outgoingScale;
            const auto start = sourcePosition(incoming, startDistance, workspace);
            const auto end = sourcePosition(outgoing, endDistance, workspace);
            const auto startTangent = sourceTangent(incoming, startDistance, workspace);
            const auto endTangent = sourceTangent(outgoing, endDistance, workspace);
            const auto startCurvature = curvatureAtDistance(
                *incoming.curve, startDistance, workspace);
            const auto endCurvature = curvatureAtDistance(
                *outgoing.curve, endDistance, workspace);
            const auto fittedHandle = [](const position_t &endpoint,
                                         const position_t &tangent,
                                         const position_t &curvature,
                                         const position_t &twoStepsInside,
                                         const double fallbackScale,
                                         const double fallbackTangentDistance) {
                const auto delta = twoStepsInside - endpoint;
                auto tangentDistance = dot(delta, tangent);
                if(tangentDistance * fallbackTangentDistance <= 0.0)
                    tangentDistance = fallbackTangentDistance;
                const auto normalDelta = delta - scaled(tangent, tangentDistance);
                const auto curvatureSquared = dot(curvature, curvature);
                auto handle = fallbackScale;
                if(curvatureSquared > 1e-18) {
                    const auto handleSquared = dot(normalDelta, curvature)
                        / (3.0 * curvatureSquared);
                    if(handleSquared > 1e-18)
                        handle = std::clamp(std::sqrt(handleSquared),
                            fallbackScale * 0.25, fallbackScale * 2.0);
                }
                return std::pair{handle, tangentDistance};
            };
            auto [incomingHandle, incomingTangentDistance] = fittedHandle(
                start, startTangent, startCurvature,
                sourcePosition(incoming, incoming.length - incomingScale, workspace),
                incomingScale, 2.0 * incomingScale);
            auto [outgoingHandle, outgoingTangentDistance] = fittedHandle(
                end, endTangent, endCurvature,
                sourcePosition(outgoing, outgoingScale, workspace),
                outgoingScale, -2.0 * outgoingScale);
            const auto xyzLength = [](const position_t &value) {
                return std::hypot(value.x, value.y, value.z);
            };
            if(xyzLength(startCurvature) > 1e-12 || xyzLength(endCurvature) > 1e-12) {
                const auto endpoint = [](const position_t &position,
                                         const position_t &tangent,
                                         const position_t &curvature) {
                    return spline_detail::Endpoint3{
                        .position={position.x, position.y, position.z},
                        .tangent={tangent.x, tangent.y, tangent.z},
                        .curvature={curvature.x, curvature.y, curvature.z},
                    };
                };
                std::tie(incomingHandle, outgoingHandle) = spline_detail::optimizeHandles(
                    endpoint(start, startTangent, startCurvature),
                    endpoint(end, endTangent, endCurvature),
                    incomingTangentDistance, outgoingTangentDistance,
                    incomingHandle, outgoingHandle, incomingScale, outgoingScale);
            }
            return {
                start,
                start + scaled(startTangent, incomingHandle),
                start + scaled(startTangent, incomingTangentDistance)
                    + scaled(startCurvature, 3.0 * incomingHandle * incomingHandle),
                end + scaled(endTangent, outgoingTangentDistance)
                    + scaled(endCurvature, 3.0 * outgoingHandle * outgoingHandle),
                end - scaled(endTangent, outgoingHandle),
                end,
            };
        }
    }

    double curveLength(const PreparedCurve &curve) { return curve.length; }

    std::shared_ptr<const PreparedCurve> prepareDisplayCurve(const MachineCommand &command) {
        return std::visit([](const auto &value) -> std::shared_ptr<const PreparedCurve> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, MoveLine>) return lineCurve(value.from(), value.to());
            else if constexpr(std::same_as<T, MoveArc>) return arcCurve(value);
            else if constexpr(std::same_as<T, ProbeMove>)
                return lineCurve(value.from(), value.target());
            else return {};
        }, command);
    }

    position_t positionAtDistance(const PreparedCurve &curve, const double requested,
                                  CurveEvaluationWorkspace &workspace) {
        const auto distance = std::clamp(requested, 0.0, curve.length);
        return std::visit([&](const auto &value) -> position_t {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, PreparedLineCurve>) {
                const auto fraction = curve.length > 1e-15 ? distance / curve.length : 0.0;
                return value.from + scaled(value.to - value.from, fraction);
            } else if constexpr(std::same_as<T, PreparedArcCurve>) {
                const auto *reference = arcReference(curve, workspace);
                return reference ? reference->positionAtDistance(distance) : value.arc.from();
            } else {
                return splineAt(value, splineParameterAtDistance(curve, value, distance, workspace));
            }
        }, curve.value);
    }

    position_t tangentAtDistance(const PreparedCurve &curve, const double requested,
                                 CurveEvaluationWorkspace &workspace) {
        const auto distance = std::clamp(requested, 0.0, curve.length);
        return std::visit([&](const auto &value) -> position_t {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, PreparedLineCurve>) {
                return curve.length > 1e-15 ? scaled(value.to - value.from, 1.0 / curve.length)
                                            : position_t{};
            } else if constexpr(std::same_as<T, PreparedArcCurve>) {
                const auto *reference = arcReference(curve, workspace);
                return reference ? reference->tangentAtDistance(distance) : position_t{};
            } else {
                const auto parameter = splineParameterAtDistance(curve, value, distance, workspace);
                const auto derivative = derivativeAt(value, parameter, 1);
                return derivative.length() > 1e-15 ? scaled(derivative, 1.0 / derivative.length())
                                                   : position_t{};
            }
        }, curve.value);
    }

    position_t curvatureAtDistance(const PreparedCurve &curve, const double requested,
                                   CurveEvaluationWorkspace &workspace) {
        const auto distance = std::clamp(requested, 0.0, curve.length);
        return std::visit([&](const auto &value) -> position_t {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, PreparedLineCurve>) return {};
            else if constexpr(std::same_as<T, PreparedArcCurve>) {
                const auto *reference = arcReference(curve, workspace);
                return reference ? reference->curvatureAtDistance(distance) : position_t{};
            } else {
                const auto parameter = splineParameterAtDistance(curve, value, distance, workspace);
                const auto first = derivativeAt(value, parameter, 1);
                const auto second = derivativeAt(value, parameter, 2);
                const auto speed = first.length();
                if(speed <= 1e-15) return position_t{};
                const auto tangent = scaled(first, 1.0 / speed);
                return scaled(second - scaled(tangent, dot(second, tangent)),
                              1.0 / (speed * speed));
            }
        }, curve.value);
    }

    position_t curvatureDerivativeAtDistance(const PreparedCurve &curve, const double requested,
                                              CurveEvaluationWorkspace &workspace) {
        const auto distance = std::clamp(requested, 0.0, curve.length);
        return std::visit([&](const auto &value) -> position_t {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, PreparedLineCurve>) return {};
            else if constexpr(std::same_as<T, PreparedArcCurve>) {
                const auto *reference = arcReference(curve, workspace);
                return reference
                    ? reference->curvatureDerivativeAtDistance(distance) : position_t{};
            } else {
                const auto parameter = splineParameterAtDistance(curve, value, distance, workspace);
                const auto first = derivativeAt(value, parameter, 1);
                const auto second = derivativeAt(value, parameter, 2);
                const auto third = derivativeAt(value, parameter, 3);
                const auto speed = first.length();
                if(speed <= 1e-15) return {};
                const auto firstSecond = dot(first, second);
                const auto secondSquared = dot(second, second);
                const auto firstThird = dot(first, third);
                const auto inverseSpeed2 = 1.0 / (speed * speed);
                const auto inverseSpeed4 = inverseSpeed2 * inverseSpeed2;
                const auto inverseSpeed6 = inverseSpeed4 * inverseSpeed2;
                const auto parameterDerivative = scaled(third, inverseSpeed2)
                    + scaled(second, -3.0 * firstSecond * inverseSpeed4)
                    + scaled(first, -(secondSquared + firstThird) * inverseSpeed4)
                    + scaled(first, 4.0 * firstSecond * firstSecond * inverseSpeed6);
                return scaled(parameterDerivative, 1.0 / speed);
            }
        }, curve.value);
    }

    double chordErrorBound(const PreparedCurve &curve, const double fromDistance,
                           const double toDistance, CurveEvaluationWorkspace &workspace) {
        const auto from = std::clamp(fromDistance, 0.0, curve.length);
        const auto to = std::clamp(toDistance, 0.0, curve.length);
        if(to <= from) return 0.0;
        return std::visit([&](const auto &value) -> double {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, PreparedLineCurve>) return 0.0;
            else if constexpr(std::same_as<T, PreparedArcCurve>) {
                const auto *reference = arcReference(curve, workspace);
                return reference ? reference->chordErrorBound(from, to) : 0.0;
            } else {
                const auto first = splineParameterAtDistance(curve, value, from, workspace);
                const auto second = splineParameterAtDistance(curve, value, to, workspace);
                return value.maximumSecondDerivative * (second - first) * (second - first) / 8.0;
            }
        }, curve.value);
    }

    std::expected<PreparedContinuousGeometry, std::string> prepareContinuousGeometry(
            const std::span<const PreparedCommandRecord> records, const double blendScale,
            const position_t expectedStart, const GeometryPreparationEffort &effort,
            const ContinuousGeometryBoundaries &boundaries) {
        if(records.empty()) return std::unexpected("continuous geometry window is empty");
        if(!std::isfinite(blendScale) || blendScale <= 0.0)
            return std::unexpected("continuous geometry blend scale must be positive");

        PreparedContinuousGeometry result;
        result.commands.assign(records.begin(), records.end());
        result.diagnostics.sourceCommands = records.size();
        std::vector<SourceEntity> entities;
        entities.reserve(records.size());
        CurveEvaluationWorkspace workspace;
        auto expected = expectedStart;
        for(const auto &record : records) {
            const auto entity = std::visit([&](const auto &command)
                    -> std::expected<SourceEntity, std::string> {
                using T = std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T, MoveLine>) {
                    if(command.speed() <= 0.0 || command.machineCoordinates())
                        return std::unexpected("continuous line must be a positive-feed non-G53 move");
                    if((command.from() - expected).length() > 1e-8)
                        return std::unexpected("continuous geometry source endpoints are discontinuous");
                    auto curve = lineCurve(command.from(), command.to());
                    if(!curve || curve->length <= 1e-12)
                        return std::unexpected("continuous path contains a zero-length line");
                    return SourceEntity{record.id, std::move(curve), command.speed() / 60.0,
                                        true, (command.to() - command.from()).length()};
                } else if constexpr(std::same_as<T, MoveArc>) {
                    if(command.speed() <= 0.0)
                        return std::unexpected("continuous arc must have a positive feed");
                    if((command.from() - expected).length() > 1e-8)
                        return std::unexpected("continuous geometry source endpoints are discontinuous");
                    auto curve = arcCurve(command);
                    if(!curve) return std::unexpected("continuous path contains an invalid arc");
                    const auto length = curveLength(*curve);
                    return SourceEntity{record.id, std::move(curve), command.speed() / 60.0,
                                        false, length};
                } else {
                    return std::unexpected("continuous geometry accepts only lines and arcs");
                }
            }, record.command);
            if(!entity) return std::unexpected(entity.error());
            expected = std::visit([](const auto &command) -> position_t {
                using T = std::decay_t<decltype(command)>;
                if constexpr(std::same_as<T, MoveLine> || std::same_as<T, MoveArc>) return command.to();
                else return {};
            }, record.command);
            entities.push_back(*entity);
        }

        const auto entityLengths = [&] {
            std::vector<double> values;
            values.reserve(entities.size());
            for(const auto &entity : entities) values.push_back(entity.length);
            return values;
        }();
        const auto clusters = spline_detail::detectShortEntitySplineClusters(
            entityLengths, blendScale);
        std::vector<std::size_t> clusterRight(entities.size(), std::numeric_limits<std::size_t>::max());
        for(const auto &cluster : clusters) clusterRight[cluster.left] = cluster.right;

        auto addPiece = [&](const PreparedPieceKind kind, const std::shared_ptr<const PreparedCurve> &curve,
                            const double from, const double to, const double feed,
                            const bool linear, const PreparedCommandId primary,
                            std::vector<PreparedCommandId> activations) -> std::expected<void, std::string> {
            if(!curve || to - from <= 1e-12 || !std::isfinite(to - from))
                return std::unexpected("prepared geometry contains a zero-length piece");
            PreparedPathPiece piece;
            piece.id = static_cast<PreparedPieceId>(result.pieces.size() + 1);
            piece.kind = kind;
            piece.curve = curve;
            piece.curveFrom = from;
            piece.curveTo = to;
            piece.programmedFeed = feed;
            piece.geometricallyLinear = linear;
            piece.primaryCommand = primary;
            piece.activationCommands = std::move(activations);
            if(effort.generateSamples) samplePiece(piece, workspace);
            result.diagnostics.pathLength += piece.length();
            if(feed > 0.0) result.diagnostics.nominalDuration += piece.length() / feed;
            ++result.diagnostics.preparedPieces;
            switch(kind) {
                case PreparedPieceKind::RetainedLineSection: ++result.diagnostics.retainedLineSections; break;
                case PreparedPieceKind::RetainedArcSection: ++result.diagnostics.retainedArcSections; break;
                case PreparedPieceKind::JunctionBlend: ++result.diagnostics.junctionBlends; break;
                case PreparedPieceKind::ClusterSpline: ++result.diagnostics.clusterSplines; break;
            }
            result.pieces.push_back(std::move(piece));
            return {};
        };

        for(std::size_t index = 0; index < entities.size(); ++index) {
            const auto incomingScale = sourceScale(entities[index], blendScale);
            const auto from = index == 0 && !boundaries.incomingReplacement
                ? 0.0 : 3.0 * incomingScale;
            const auto to = index + 1 == entities.size() ? entities[index].length
                : entities[index].length - 3.0 * incomingScale;
            const auto cluster = clusterRight[index];
            const auto collapsed = std::ranges::any_of(clusters, [&](const auto &candidate) {
                return index > candidate.left && index < candidate.right;
            });
            const auto deferredFinal = boundaries.deferFinalRetainedSection
                && index + 1 == entities.size();
            if(!collapsed && !deferredFinal && to > from + 1e-12) {
                const auto kind = entities[index].linear ? PreparedPieceKind::RetainedLineSection
                                                          : PreparedPieceKind::RetainedArcSection;
                if(auto added = addPiece(kind, entities[index].curve, from, to,
                                         entities[index].feed, entities[index].linear,
                                         entities[index].id, {entities[index].id}); !added)
                    return std::unexpected(added.error());
            }
            if(index + 1 == entities.size()) continue;
            if(cluster != std::numeric_limits<std::size_t>::max()) {
                const auto right = cluster;
                const auto leftScale = sourceScale(entities[index], blendScale);
                const auto rightScale = sourceScale(entities[right], blendScale);
                auto controls = curvatureMatchedSixControlBlend(
                    entities[index], entities[right], leftScale, rightScale, workspace);
                if(right > index + 1) {
                    std::vector<double> interiorLengths;
                    auto interiorLength = 0.0;
                    for(std::size_t entity = index + 1; entity < right; ++entity) {
                        interiorLengths.push_back(entities[entity].length);
                        interiorLength += entities[entity].length;
                    }
                    if(interiorLength >= 6.0 * blendScale) {
                        const auto distances = spline_detail::evenlySpacedCompositeControlDistances(
                            interiorLengths, blendScale);
                        if(distances.empty()) return std::unexpected(
                            "continuous short-entity spline has no control samples");
                        std::vector<position_t> interior;
                        auto entity = index + 1;
                        auto entityStart = 0.0;
                        for(const auto distance : distances) {
                            while(entity + 1 < right
                                  && distance > entityStart + entities[entity].length) {
                                entityStart += entities[entity].length;
                                ++entity;
                            }
                            interior.push_back(sourcePosition(entities[entity],
                                std::clamp(distance - entityStart, 0.0,
                                           entities[entity].length), workspace));
                        }
                        controls.insert(controls.begin() + 3,
                                        interior.begin(), interior.end());
                    }
                }
                std::size_t splineDegree = 3;
                if(controls.size() > 6) {
                    // The shared solver is intentionally selected here, just as
                    // it is for timed cluster preparation. The source callback
                    // is immutable and contains no dynamic motion limits.
                    spline_detail::SplineReconstructionSource source;
                    source.length = 3.0 * leftScale + 3.0 * rightScale;
                    for(std::size_t entity = index + 1; entity < right; ++entity)
                        source.length += entities[entity].length;
                    source.positionAt = [&](double distance) {
                        const auto leftLength = 3.0 * leftScale;
                        if(distance <= leftLength)
                            return sourcePosition(entities[index], entities[index].length - leftLength + distance,
                                workspace);
                        distance -= leftLength;
                        for(std::size_t entity = index + 1; entity < right; ++entity) {
                            if(distance <= entities[entity].length)
                                return sourcePosition(entities[entity], distance, workspace);
                            distance -= entities[entity].length;
                        }
                        return sourcePosition(entities[right], std::min(distance, 3.0 * rightScale), workspace);
                    };
                    source.chordErrorBound = [](double, double) { return 0.0; };
                    source.boundaries = {0.0, 3.0 * leftScale};
                    auto boundary = 3.0 * leftScale;
                    for(std::size_t entity = index + 1; entity < right; ++entity) {
                        boundary += entities[entity].length;
                        source.boundaries.push_back(boundary);
                    }
                    source.boundaries.push_back(source.length);
                    const auto startDistance = entities[index].length - 3.0 * leftScale;
                    const auto endDistance = 3.0 * rightScale;
                    source.startTangent = tangentAtDistance(
                        *entities[index].curve, startDistance, workspace);
                    source.startCurvature = curvatureAtDistance(
                        *entities[index].curve, startDistance, workspace);
                    source.startCurvatureDerivative = curvatureDerivativeAtDistance(
                        *entities[index].curve, startDistance, workspace);
                    source.endTangent = tangentAtDistance(
                        *entities[right].curve, endDistance, workspace);
                    source.endCurvature = curvatureAtDistance(
                        *entities[right].curve, endDistance, workspace);
                    source.endCurvatureDerivative = curvatureDerivativeAtDistance(
                        *entities[right].curve, endDistance, workspace);
                    auto fitted = spline_detail::reconstructSpline(
                        controls, source, blendScale, effort.certifySourceTube);
                    if(!fitted) return std::unexpected(fitted.error());
                    splineDegree = fitted->degree;
                    controls = std::move(fitted->controls);
                }
                auto curve = splineCurve(std::move(controls), splineDegree,
                    effort.lengthTableIntervalsPerKnotSpan);
                if(!curve) return std::unexpected("short-entity cluster spline construction failed");
                std::vector<PreparedCommandId> activations;
                for(std::size_t command = index + 1; command <= right; ++command)
                    activations.push_back(entities[command].id);
                if(auto added = addPiece(PreparedPieceKind::ClusterSpline, curve, 0.0,
                                         curve->length, (entities[index].feed + entities[right].feed) / 2.0,
                                         false, entities[index].id, std::move(activations)); !added)
                    return std::unexpected(added.error());
                index = right - 1;
                continue;
            }
            const auto &incoming = entities[index];
            const auto &outgoing = entities[index + 1];
            const auto controls = curvatureMatchedSixControlBlend(incoming, outgoing,
                sourceScale(incoming, blendScale), sourceScale(outgoing, blendScale), workspace);
            auto curve = splineCurve(controls, 3,
                effort.lengthTableIntervalsPerKnotSpan);
            if(!curve) return std::unexpected("junction blend spline construction failed");
            if(auto added = addPiece(PreparedPieceKind::JunctionBlend, curve, 0.0, curve->length,
                                     (incoming.feed + outgoing.feed) / 2.0, false,
                                     outgoing.id, {outgoing.id}); !added)
                return std::unexpected(added.error());
        }
        if(result.pieces.empty()) return std::unexpected("continuous path produced no geometry");
        result.beginning = boundaryAt(result.pieces.front(), workspace, 0.0);
        result.ending = boundaryAt(result.pieces.back(), workspace, result.pieces.back().length());
        return result;
    }
}
