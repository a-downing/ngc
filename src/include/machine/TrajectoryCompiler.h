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
#include "machine/ArcInterpolation.h"
#include "machine/InfiniteJerkTrajectoryTime.h"
#include "machine/PreparedGeometry.h"

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
        // Production defaults remain bounded. Standalone offline profiles may
        // raise these ceilings to measure difficult high-jerk horizons.
        unsigned maximumLocalCorrectionPasses = 32;
        std::size_t geometryVerificationBudgetMultiplier = 36;
        bool capLargeHorizonVelocitySearch = true;
        double curvatureDerivativeVelocityCapMultiplier = 1.0;
        bool applyCurvatureDerivativeVelocityCap = true;
        bool measureCurvatureDerivativeNumerics = false;
        bool measureStationVisitReplay = false;
        bool enableStationVisitReplay = true;
        bool shareTimeLawCacheAcrossCompilations = true;
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

    // NRT-only cost evidence for calls into the scalar Ruckig position solver.
    // Candidate calls use a compile-local bit-exact result cache. A cached
    // successful duration is materialized again only if it becomes the new
    // selected timing, so retained phase data still comes from Ruckig and the
    // complete validation path.
    struct TimeLawCallDiagnostics {
        std::size_t calls = 0;
        std::size_t successes = 0;
        std::size_t failures = 0;
        std::size_t solverCalls = 0;
        std::size_t cacheHits = 0;
        std::size_t cacheMisses = 0;
        std::size_t cacheCollisions = 0;
        std::size_t cacheMaterializations = 0;
        std::size_t correctionPassCalls = 0;
        double seconds = 0.0;

        TimeLawCallDiagnostics &operator+=(const TimeLawCallDiagnostics &other) {
            calls+=other.calls;
            successes+=other.successes;
            failures+=other.failures;
            solverCalls+=other.solverCalls;
            cacheHits+=other.cacheHits;
            cacheMisses+=other.cacheMisses;
            cacheCollisions+=other.cacheCollisions;
            cacheMaterializations+=other.cacheMaterializations;
            correctionPassCalls+=other.correctionPassCalls;
            seconds+=other.seconds;
            return *this;
        }
    };

    // NRT-only evidence for the cached one-sided coupled endpoint checks that
    // run before local Ruckig transition solves.
    struct EndpointFeasibilityDiagnostics {
        std::size_t cachedGeometryEndpoints = 0;
        std::size_t candidateChecks = 0;
        std::size_t accelerationRejections = 0;
        std::size_t jerkRejections = 0;

        EndpointFeasibilityDiagnostics &operator+=(const EndpointFeasibilityDiagnostics &other) {
            cachedGeometryEndpoints+=other.cachedGeometryEndpoints;
            candidateChecks+=other.candidateChecks;
            accelerationRejections+=other.accelerationRejections;
            jerkRejections+=other.jerkRejections;
            return *this;
        }
    };

    // NRT-only pass-to-pass evidence for deciding whether exact-constraint
    // correction can safely reuse an unchanged prefix or suffix. Every record
    // compares one correction replan with the complete preceding pass using
    // bit-exact scalar station states and scalar TimeLaw phase boundaries.
    struct CorrectionPassLocalityDiagnostic {
        unsigned pass = 0;
        std::size_t pieceCount = 0;
        std::size_t correctedPieces = 0;
        std::size_t firstCorrectedPiece = 0;
        std::size_t lastCorrectedPiece = 0;
        std::size_t changedStations = 0;
        std::size_t firstChangedStation = 0;
        std::size_t lastChangedStation = 0;
        std::size_t changedPieceTimings = 0;
        std::size_t changedUncorrectedPieceTimings = 0;
        std::size_t changedPieceTimingRuns = 0;
        std::size_t maximumChangedPieceTimingRun = 0;
        std::size_t firstChangedPieceTiming = 0;
        std::size_t lastChangedPieceTiming = 0;
        std::size_t bitExactReusablePieceTimings = 0;
        std::size_t bitExactReusablePieceTimingRuns = 0;
        std::size_t maximumBitExactReusablePieceTimingRun = 0;
        std::size_t bitExactReusablePrefixPieces = 0;
        std::size_t bitExactReusableSuffixPieces = 0;
        std::size_t leftPropagationPieces = 0;
        std::size_t rightPropagationPieces = 0;
        std::size_t maximumPropagationFromCorrectedPiece = 0;
        double maximumVelocityChange = 0.0;
        double maximumAccelerationChange = 0.0;
        double maximumDurationChange = 0.0;
    };

    // NRT-only shadow evidence for replaying an acceleration-aware station
    // visit on the following correction pass. Shadow matches still execute the
    // visit and compare its complete output, so these counters do not affect
    // trajectory selection or resource-budget authority.
    struct StationVisitReplayDiagnostics {
        std::size_t activeVisits = 0;
        std::size_t comparableVisits = 0;
        std::size_t exactInputMatches = 0;
        std::size_t exactOutputMatches = 0;
        std::size_t outputMismatches = 0;
        std::size_t potentialCandidateEvaluations = 0;
        std::size_t potentialEndpointChecks = 0;
        std::size_t potentialTimeLawCalls = 0;
        std::size_t potentialSolverCalls = 0;
        std::size_t potentialMaterializations = 0;
        double potentialTimeLawSeconds = 0.0;
        double potentialVisitSeconds = 0.0;

        StationVisitReplayDiagnostics &operator+=(
                const StationVisitReplayDiagnostics &other) {
            activeVisits+=other.activeVisits;
            comparableVisits+=other.comparableVisits;
            exactInputMatches+=other.exactInputMatches;
            exactOutputMatches+=other.exactOutputMatches;
            outputMismatches+=other.outputMismatches;
            potentialCandidateEvaluations+=other.potentialCandidateEvaluations;
            potentialEndpointChecks+=other.potentialEndpointChecks;
            potentialTimeLawCalls+=other.potentialTimeLawCalls;
            potentialSolverCalls+=other.potentialSolverCalls;
            potentialMaterializations+=other.potentialMaterializations;
            potentialTimeLawSeconds+=other.potentialTimeLawSeconds;
            potentialVisitSeconds+=other.potentialVisitSeconds;
            return *this;
        }
    };

    struct TimeLawDiagnostics {
        TimeLawCallDiagnostics exactStop;
        TimeLawCallDiagnostics continuousSeed;
        TimeLawCallDiagnostics stationCurrentVelocity;
        TimeLawCallDiagnostics stationCapVelocity;
        TimeLawCallDiagnostics stationVelocityBracket;
        EndpointFeasibilityDiagnostics endpointFeasibility;
        std::vector<CorrectionPassLocalityDiagnostic> correctionPassLocality;
        StationVisitReplayDiagnostics stationVisitReplay;

        TimeLawDiagnostics &operator+=(const TimeLawDiagnostics &other) {
            exactStop+=other.exactStop;
            continuousSeed+=other.continuousSeed;
            stationCurrentVelocity+=other.stationCurrentVelocity;
            stationCapVelocity+=other.stationCapVelocity;
            stationVelocityBracket+=other.stationVelocityBracket;
            endpointFeasibility+=other.endpointFeasibility;
            correctionPassLocality.insert(correctionPassLocality.end(),
                other.correctionPassLocality.begin(),other.correctionPassLocality.end());
            stationVisitReplay+=other.stationVisitReplay;
            return *this;
        }
    };

    inline TimeLawCallDiagnostics totalTimeLawCalls(const TimeLawDiagnostics &diagnostics) {
        auto total=diagnostics.exactStop;
        total+=diagnostics.continuousSeed;
        total+=diagnostics.stationCurrentVelocity;
        total+=diagnostics.stationCapVelocity;
        total+=diagnostics.stationVelocityBracket;
        return total;
    }

    struct ContinuousTrajectoryPlan {
        std::vector<PlanChunk> chunks;
        std::vector<SpanId> activationSpans;
        // NRT-only development evidence; never crosses MotionBackend.
        std::vector<ContinuousPieceTimingDiagnostic> pieceTiming;
        double velocityOnlySeedDuration = 0.0;
        double accelerationAwareDuration = 0.0;
        std::string correctionHistory;
        std::size_t ruckigBrakePhases = 0;
        std::size_t reachabilityCandidateEvaluations = 0;
        std::size_t geometryVerificationAttempts = 0;
        std::size_t geometryVerificationHighWater = 0;
        TimeLawDiagnostics timeLaw;
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

    class TrajectoryCompiler {
        TrajectoryLimits m_limits;
        ContinuousPlanningEffort m_continuousPlanningEffort;
        EpochId m_epoch = 1;
        ChunkId m_nextChunk = 1;
        SpanId m_nextSpan = 1;
        BranchSequence m_previousBranch = 0;
        position_t m_position{};
        std::function<void()> m_progressCallback;
        TimeLawDiagnostics m_lastTimeLawDiagnostics;

    public:
        explicit TrajectoryCompiler(TrajectoryLimits limits = {});

        void reset(EpochId epoch, const position_t &position = {});
        const TrajectoryLimits &limits() const { return m_limits; }
        void setLimits(const TrajectoryLimits &limits) { m_limits = limits; }
        void setContinuousPlanningEffort(const ContinuousPlanningEffort &effort) {
            m_continuousPlanningEffort=effort;
        }
        const ContinuousPlanningEffort &continuousPlanningEffort() const {
            return m_continuousPlanningEffort;
        }
        void setProgressCallback(std::function<void()> callback) {
            m_progressCallback=std::move(callback);
        }
        const std::function<void()> &progressCallback() const { return m_progressCallback; }
        const TimeLawDiagnostics &lastTimeLawDiagnostics() const {
            return m_lastTimeLawDiagnostics;
        }
        void reportProgress() const {
            if(m_progressCallback) m_progressCallback();
        }
        const position_t &plannedPosition() const { return m_position; }

        std::expected<PlanChunk, std::string> compile(
            const MachineCommand &command,
            const PreparedPathPiece *preparedPiece = nullptr);
        // Prepared geometry is immutable and may be constructed by a different
        // NRT thread. Timing consumes its command metadata here while keeping
        // all dynamic limits, time laws, proof, packetization, and stop-tail
        // generation on the planning owner.
        std::expected<std::unique_ptr<ContinuousTrajectoryPlan>, std::string> compileContinuous(
            const PreparedContinuousGeometry &geometry, double blendScale,
            std::optional<MotionState> startState = std::nullopt,
            std::optional<MotionState> endState = std::nullopt,
            unsigned velocitySearchIterations = 12,
            InfiniteJerkTrajectoryTimeResult *infiniteJerkTime = nullptr);
        std::expected<TriggeredMove, std::string> compileTriggeredMove(
            const ProbeMove &command, DigitalInputId input = 0,
            InputCondition condition = InputCondition::Active);
    };
}
