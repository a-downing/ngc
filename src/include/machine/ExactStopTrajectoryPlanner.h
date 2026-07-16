#pragma once

#include <expected>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "machine/MotionBackend.h"

namespace ngc {
    enum class ContinuousVelocityLimitCause {
        ProgrammedFeed,
        AxisVelocity,
        AxisCentripetalAcceleration,
        PathCentripetalAcceleration,
        AxisCurvatureDerivativeJerk,
        PathCurvatureDerivativeJerk,
    };

    struct ContinuousPlanningEffort {
        unsigned reachabilitySweeps = 3;
        unsigned minimumVelocitySearchIterations = 0;
        std::size_t accelerationCandidates = 6;
        std::size_t candidateBudgetMultiplier = 1;
        bool capLargeHorizonVelocitySearch = true;
        double curvatureDerivativeVelocityCapMultiplier = 1.0;
        bool applyCurvatureDerivativeVelocityCap = true;
        bool measureCurvatureDerivativeNumerics = false;
        bool captureSplineGeometry = false;
    };

    struct ContinuousPieceTimingDiagnostic {
        std::size_t input = 0;
        double length = 0.0;
        bool linear = false;
        position_t startPosition{};
        position_t endPosition{};
        double programmedVelocityLimit = 0.0;
        double initialVelocityLimit = 0.0;
        ContinuousVelocityLimitCause initialVelocityLimitCause =
            ContinuousVelocityLimitCause::ProgrammedFeed;
        double initialAccelerationLimit = 0.0;
        double initialJerkLimit = 0.0;
        double velocityLimit = 0.0;
        double accelerationLimit = 0.0;
        double jerkLimit = 0.0;
        double entryVelocity = 0.0;
        double entryAcceleration = 0.0;
        double exitVelocity = 0.0;
        double exitAcceleration = 0.0;
        double duration = 0.0;
        // NRT-only numerical diagnostics at the sample producing the largest
        // analytic curvature-derivative magnitude for this geometry piece.
        double curvatureDerivativeSampleDistance = 0.0;
        double curvatureDerivativeMagnitude = 0.0;
        double curvatureMagnitudeAtDerivativeSample = 0.0;
        double curvatureDerivativeTangentialMagnitude = 0.0;
        double curvatureDerivativeNormalMagnitude = 0.0;
        double curvatureDerivativeFiniteDifferenceCoarse = 0.0;
        double curvatureDerivativeFiniteDifferenceFine = 0.0;
        double curvatureDerivativeFiniteDifferenceCoarseStep = 0.0;
        double curvatureDerivativeFiniteDifferenceFineStep = 0.0;
    };

    struct ContinuousAccelerationOracleSegment {
        std::size_t piece = 0;
        std::size_t input = 0;
        double length = 0.0;
        double velocityLimit = 0.0;
        position_t tangent{};
        position_t curvature{};
        position_t curvatureDerivative{};
    };

    // Optional NRT-only data for the standalone Clarabel comparison tool.
    // This is deliberately not part of PlanChunk or MotionBackend.
    struct ContinuousAccelerationOracleModel {
        double pathAcceleration = 0.0;
        position_t axisAcceleration{};
        double pathJerk = 0.0;
        position_t axisJerk{};
        double plannerDuration = 0.0;
        std::vector<ContinuousPieceTimingDiagnostic> pieceTiming;
        std::vector<ContinuousAccelerationOracleSegment> segments;
    };

    struct ContinuousTrajectoryPlan {
        struct SplineGeometry {
            std::size_t firstInput = 0;
            std::size_t lastInput = 0;
            std::vector<position_t> controls;
            std::vector<double> pieceBoundaries;
        };
        std::vector<PlanChunk> chunks;
        std::vector<SpanId> activationSpans;
        // NRT-only development evidence; never crosses MotionBackend.
        std::vector<ContinuousPieceTimingDiagnostic> pieceTiming;
        double velocityOnlySeedDuration = 0.0;
        double accelerationAwareDuration = 0.0;
        std::size_t ruckigBrakePhases = 0;
        std::size_t reachabilityCandidateEvaluations = 0;
        std::size_t geometryVerificationAttempts = 0;
        std::size_t geometryVerificationHighWater = 0;
        // Optional NRT development snapshot; never crosses MotionBackend.
        std::vector<SplineGeometry> splineGeometry;
    };

    struct TrajectoryLimits {
        static position_t unlimitedAxes() {
            constexpr auto infinity = std::numeric_limits<double>::infinity();
            return { infinity, infinity, infinity, infinity, infinity, infinity };
        }

        // Aggregate path limits remain additional caps. Axis limits are the
        // final authority for every emitted axis polynomial.
        double pathAcceleration = 10.0;
        double rapidSpeed = 100.0; // Machine units per minute.
        double arcChordTolerance = 0.0001;
        double pathJerk = 100.0;
        position_t axisVelocity = unlimitedAxes();       // Machine units per second.
        position_t axisAcceleration = unlimitedAxes();   // Machine units per second squared.
        position_t axisJerk = unlimitedAxes();           // Machine units per second cubed.
        // Minimum NRT rolling-horizon duration; proof may extend it.
        double lookaheadDuration = 2.0;
    };

    class ExactStopTrajectoryPlanner {
        TrajectoryLimits m_limits;
        ContinuousPlanningEffort m_continuousPlanningEffort;
        EpochId m_epoch = 1;
        ChunkId m_nextChunk = 1;
        SpanId m_nextSpan = 1;
        BranchSequence m_previousBranch = 0;
        position_t m_position{};
        std::function<void()> m_progressCallback;

    public:
        explicit ExactStopTrajectoryPlanner(TrajectoryLimits limits = {});

        void reset(EpochId epoch, const position_t &position = {});
        const TrajectoryLimits &limits() const { return m_limits; }
        void setLimits(const TrajectoryLimits &limits) { m_limits = limits; }
        void setContinuousPlanningEffort(const ContinuousPlanningEffort &effort) {
            m_continuousPlanningEffort=effort;
        }
        void setProgressCallback(std::function<void()> callback) {
            m_progressCallback=std::move(callback);
        }
        const std::function<void()> &progressCallback() const { return m_progressCallback; }
        void reportProgress() const {
            if(m_progressCallback) m_progressCallback();
        }
        const position_t &plannedPosition() const { return m_position; }

        std::expected<PlanChunk, std::string> compile(const MachineCommand &command);
        std::expected<std::unique_ptr<ContinuousTrajectoryPlan>, std::string> compileContinuous(
            std::span<const MachineCommand> commands, double blendScale,
            ContinuousAccelerationOracleModel *oracleModel = nullptr,
            std::optional<MotionState> startState = std::nullopt,
            std::optional<MotionState> endState = std::nullopt,
            std::span<const double> scaleOverrides = {},
            unsigned velocitySearchIterations = 12);
        std::expected<TriggeredMove, std::string> compileTriggeredMove(
            const ProbeMove &command, DigitalInputId input = 0,
            InputCondition condition = InputCondition::Active);
    };
}
