#pragma once

#include <array>
#include <cstdint>
#include <concepts>
#include <expected>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "evaluator/InterpreterStatus.h"
#include "evaluator/InterpreterSession.h"
#include "machine/ArcInterpolation.h"
#include "machine/MachineCommand.h"
#include "machine/SplineReconstruction.h"

namespace ngc {
    using GeometryEpoch = std::uint64_t;
    using GeometrySequence = std::uint64_t;
    using PreparedCommandId = std::uint64_t;
    using PreparedPieceId = std::uint64_t;
    using ContinuousChainId = std::uint64_t;
    using SynchronizationFenceId = std::uint64_t;

    enum class ExecutablePathMode { ExactStop, Continuous };

    struct TrajectoryPlanningMetadata {
        ExecutablePathMode pathMode = ExecutablePathMode::ExactStop;
        std::optional<double> pathTolerance;
    };

    struct TrajectoryCommandPresentation {
        ToolGeometry tool{};
        position_t activeToolOffset{};
        std::optional<WorkCoordinateSystem> workCoordinateSystem;
        std::vector<std::string> modalGCodes;
        std::vector<BlockExecution> activeBlocks;
    };

    inline bool sameProtectedTrajectoryPresentation(
            const TrajectoryCommandPresentation &left,
            const TrajectoryCommandPresentation &right) {
        const auto samePosition = [](const position_t &a, const position_t &b) {
            return (a - b).length() <= 1e-12;
        };
        if(!samePosition(left.activeToolOffset, right.activeToolOffset)) return false;
        if(left.tool.number != right.tool.number || left.tool.diameter != right.tool.diameter
           || !samePosition(left.tool.offset, right.tool.offset)) return false;
        if(left.workCoordinateSystem.has_value() != right.workCoordinateSystem.has_value()) return false;
        if(left.workCoordinateSystem
           && (left.workCoordinateSystem->name != right.workCoordinateSystem->name
               || !samePosition(left.workCoordinateSystem->offset,
                                right.workCoordinateSystem->offset))) return false;
        const auto protectedModes = [](const std::vector<std::string> &modes) {
            std::vector<std::string_view> result;
            for(const auto &mode : modes) {
                if(mode == "G0" || mode == "G1" || mode == "G2" || mode == "G3"
                   || mode == "G38.2" || mode == "G38.3" || mode == "G38.4"
                   || mode == "G38.5") continue;
                result.push_back(mode);
            }
            return result;
        };
        return protectedModes(left.modalGCodes) == protectedModes(right.modalGCodes);
    }

    struct PreparedLineCurve {
        position_t from{};
        position_t to{};
    };

    struct PreparedArcCurve {
        MoveArc arc;
    };

    struct PreparedSplineDerivative {
        std::size_t degree = 0;
        std::vector<position_t> controls;
        std::vector<double> knots;
    };

    struct PreparedSplineCurve {
        std::size_t degree = 0;
        std::vector<position_t> controls;
        std::vector<double> knots;
        std::array<PreparedSplineDerivative, 3> derivatives;
        // Immutable construction-time arc-length table. It is a bracket only;
        // consumers still certify inverse queries against the curve.
        std::vector<double> parameters;
        std::vector<double> distances;
        double maximumSecondDerivative = 0.0;
    };

    using PreparedCurveValue = std::variant<PreparedLineCurve, PreparedArcCurve,
                                             PreparedSplineCurve>;

    struct PreparedCurve {
        PreparedCurveValue value;
        double length = 0.0;
        bool geometricallyLinear = false;
    };

    struct CurveEvaluationWorkspace {
        struct ArcEntry {
            const PreparedCurve *curve = nullptr;
            std::unique_ptr<simulation_detail::ArcReference> reference;
        };
        std::vector<ArcEntry> arcs;

        struct SplineInverseCacheEntry {
            double distance = 0.0;
            double parameter = 0.0;
            bool valid = false;
        };
        struct SplineEntry {
            const PreparedCurve *curve = nullptr;
            std::array<SplineInverseCacheEntry, 16> inverseCache{};
        };
        std::vector<SplineEntry> splines;

        void clear() { arcs.clear(); splines.clear(); }
    };

    double curveLength(const PreparedCurve &curve);
    position_t positionAtDistance(const PreparedCurve &curve, double distance,
                                  CurveEvaluationWorkspace &workspace);
    position_t tangentAtDistance(const PreparedCurve &curve, double distance,
                                 CurveEvaluationWorkspace &workspace);
    position_t curvatureAtDistance(const PreparedCurve &curve, double distance,
                                   CurveEvaluationWorkspace &workspace);
    position_t curvatureDerivativeAtDistance(const PreparedCurve &curve, double distance,
                                              CurveEvaluationWorkspace &workspace);
    double chordErrorBound(const PreparedCurve &curve, double fromDistance,
                           double toDistance, CurveEvaluationWorkspace &workspace);
    std::shared_ptr<const PreparedCurve> prepareDisplayCurve(const MachineCommand &command);

    struct PreparedGeometryBoundary {
        position_t position{};
        position_t tangent{};
        position_t curvature{};
        position_t curvatureDerivative{};
    };

    enum class PreparedPieceKind {
        RetainedLineSection,
        RetainedArcSection,
        JunctionBlend,
        ClusterSpline,
    };

    struct PreparedGeometricSample {
        double distance = 0.0;
        position_t tangent{};
        position_t curvature{};
        position_t curvatureDerivative{};
    };

    struct PreparedClusterKnotInterval {
        double curveFrom = 0.0;
        double curveTo = 0.0;
        double programmedFeed = 0.0;
        // Producer-computed static geometric velocity cap. Rolling horizon
        // selection combines it with programmed feed; exact dynamic timing
        // and future runtime feed scaling remain planner-owned.
        double geometricVelocityLimit = std::numeric_limits<double>::infinity();
        std::size_t firstGeometricSample = 0;
        std::size_t geometricSampleCount = 0;
    };

    struct PreparedSourceInterval {
        PreparedCommandId command = 0;
        std::shared_ptr<const PreparedCurve> curve;
        double curveFrom = 0.0;
        double curveTo = 0.0;
    };

    struct PreparedActivationStation {
        PreparedCommandId command = 0;
        // Absolute arc distance on the immutable prepared curve. The geometry
        // producer owns this source-to-curve association; timing later maps it
        // to an emitted execution span.
        double curveDistance = 0.0;
    };

    struct PreparedPathPiece {
        PreparedPieceId id = 0;
        PreparedPieceKind kind = PreparedPieceKind::RetainedLineSection;
        std::shared_ptr<const PreparedCurve> curve;
        double curveFrom = 0.0;
        double curveTo = 0.0;
        double programmedFeed = 0.0;
        PreparedCommandId primaryCommand = 0;
        std::vector<PreparedActivationStation> activationStations;
        // Source entities whose geometry this piece represents. This is
        // deliberately separate from activationStations: a junction blend
        // involves both adjacent source entities while presentation activates
        // only at the owning command boundary.
        std::vector<PreparedCommandId> sourceCommands;
        // Exact portions of source entities replaced by a junction blend or
        // cluster spline. Preview uses these immutable prepared curves to show
        // the replaced geometry without reconstructing it independently.
        std::vector<PreparedSourceInterval> replacedSourceIntervals;
        std::vector<PreparedGeometricSample> geometricSamples;
        // Continuous timing treats each cluster-spline knot interval as one
        // timing interval and requires its prepared samples and feed. The
        // cluster-wide programmedFeed remains presentation/slice metadata.
        std::vector<PreparedClusterKnotInterval> clusterKnotIntervals;

        double length() const { return std::max(0.0, curveTo - curveFrom); }
    };

    struct PreparedCommandRecord {
        PreparedCommandId id = 0;
        MachineCommand command = SpindleStop{};
        TrajectoryPlanningMetadata metadata;
        TrajectoryCommandPresentation presentation;
        bool presentationActivation = true;
        double continuousScaleOverride = 0.0;
    };

    struct GeometryPreparationEffort {
        bool certifySourceTube = true;
        bool generateSamples = true;
        std::size_t lengthTableIntervalsPerKnotSpan = 32;
        spline_detail::SplineFitSolver splineFitSolver =
            spline_detail::continuousSplineFitSolver();
        spline_detail::SplineVelocityLimits splineVelocityLimits;
    };

    // Selects the open boundaries of an incremental continuous-geometry
    // preparation. An incoming replacement means the first source entity's
    // first 3P has already been replaced. Deferring the final retained
    // primitive section permits construction of the replacement entering that
    // source without constructing or sampling that not-yet-final section.
    struct ContinuousGeometryBoundaries {
        bool incomingReplacement = false;
        bool deferFinalRetainedSection = false;
    };

    struct GeometryPreparationDiagnostics {
        double nominalDuration = 0.0;
    };

    struct PreparedContinuousGeometry {
        std::vector<PreparedCommandRecord> commands;
        std::vector<PreparedPathPiece> pieces;
        GeometryPreparationDiagnostics diagnostics;
    };

    struct PreparedGeometrySlice {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        ContinuousChainId chain = 0;
        std::vector<PreparedCommandRecord> commands;
        std::vector<PreparedPathPiece> pieces;
        double nominalDuration = 0.0;
    };

    struct PreparedStandaloneCommand {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        PreparedCommandRecord command;
        std::shared_ptr<const PreparedCurve> displayGeometry;
    };

    struct PreparedContinuousEnd {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        ContinuousChainId chain = 0;
    };

    struct PreparedBlockLifecycleMessage {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        InterpreterBlockLifecycle lifecycle;
    };

    struct PreparedSynchronizationFence {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        SynchronizationFenceId fence = 0;
    };

    struct PreparedProbeFence {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        std::uint64_t commandId = 0;
    };

    struct PreparedStatusMessage {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        InterpreterStatusMessage status;
    };

    struct PreparedProgramEnd {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
    };

    struct PreparedFailure {
        GeometryEpoch epoch = 0;
        GeometrySequence sequence = 0;
        std::string error;
    };

    using PreparedStreamMessage = std::variant<PreparedGeometrySlice,
        PreparedStandaloneCommand, PreparedContinuousEnd, PreparedBlockLifecycleMessage,
        PreparedSynchronizationFence, PreparedProbeFence, PreparedStatusMessage,
        PreparedProgramEnd, PreparedFailure>;

    struct PreparedPreviewScene {
        std::vector<PreparedGeometrySlice> continuousSlices;
        std::vector<PreparedStandaloneCommand> standaloneCommands;
        std::vector<PreparedContinuousEnd> geometryEnds;
        std::vector<TrajectoryCommandPresentation> presentations;
        std::uint64_t revision = 0;
    };

    // Direct NRT sink used by Preview. It stores complete immutable slices and
    // publishes a new revision only when the producer reaches program end.
    class PreviewGeometryCollector {
        PreparedPreviewScene m_scene;

    public:
        void clear() {
            m_scene.continuousSlices.clear();
            m_scene.standaloneCommands.clear();
            m_scene.geometryEnds.clear();
            m_scene.presentations.clear();
        }

        void consume(PreparedStreamMessage message) {
            std::visit([&](auto &&value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, PreparedGeometrySlice>) {
                    m_scene.continuousSlices.push_back(std::move(value));
                    for(const auto &command : m_scene.continuousSlices.back().commands)
                        if(command.presentationActivation)
                            m_scene.presentations.push_back(command.presentation);
                } else if constexpr(std::same_as<T, PreparedStandaloneCommand>) {
                    m_scene.standaloneCommands.push_back(std::move(value));
                    const auto &stored = m_scene.standaloneCommands.back();
                    if(stored.command.presentationActivation)
                        m_scene.presentations.push_back(stored.command.presentation);
                } else if constexpr(std::same_as<T, PreparedContinuousEnd>) {
                    m_scene.geometryEnds.push_back(std::move(value));
                }
            }, std::move(message));
        }

        PreparedPreviewScene finish() {
            ++m_scene.revision;
            return std::move(m_scene);
        }

        const PreparedPreviewScene &scene() const { return m_scene; }
    };

    struct ReleaseSynchronization {
        GeometryEpoch epoch = 0;
        SynchronizationFenceId fence = 0;
    };

    struct DeliverProbeResult {
        GeometryEpoch epoch = 0;
        ProbeResult result{};
    };

    struct AbortGeometryRun {
        GeometryEpoch epoch = 0;
        std::string error;
    };

    using GeometryFeedback = std::variant<ReleaseSynchronization, DeliverProbeResult,
                                          AbortGeometryRun>;

    std::expected<PreparedContinuousGeometry, std::string> prepareContinuousGeometry(
        std::span<const PreparedCommandRecord> commands, double blendScale,
        position_t expectedStart = {},
        const GeometryPreparationEffort &effort = {},
        const ContinuousGeometryBoundaries &boundaries = {});

    std::expected<PreparedContinuousGeometry, std::string> prepareExactStopGeometry(
        std::span<const PreparedCommandRecord> commands,
        position_t expectedStart = {},
        const GeometryPreparationEffort &effort = {});
}
