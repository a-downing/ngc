#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <expected>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "machine/MotionBackend.h"
#include "machine/ArcInterpolation.h"
#include "machine/InfiniteJerkTrajectoryTime.h"
#include "machine/PreparedGeometry.h"

namespace ngc {
    enum class ScpResourceFallbackReason : std::uint8_t {
        None,
        TimeLimit,
        IterationLimit,
    };

    inline std::string_view name(const ScpResourceFallbackReason reason) {
        switch(reason) {
            case ScpResourceFallbackReason::None: return "none";
            case ScpResourceFallbackReason::TimeLimit: return "time-limit";
            case ScpResourceFallbackReason::IterationLimit: return "iteration-limit";
        }
        return "unknown";
    }

    namespace trajectory_detail {
        // HiGHS removes entries whose magnitude is less than or equal to its
        // small_matrix_value. The builder and configured solver share this one
        // cutoff so passModel cannot silently loosen the SCP model.
        inline constexpr double SCP_SMALL_MATRIX_VALUE=1e-12;

        constexpr bool scpRetainsMatrixCoefficient(const double value) {
            return value>SCP_SMALL_MATRIX_VALUE||value<-SCP_SMALL_MATRIX_VALUE;
        }

        enum class ScpSolveClassification : std::uint8_t {
            Optimal,
            TimeLimit,
            IterationLimit,
            Failure,
        };

        enum class ScpSolveAction : std::uint8_t {
            AcceptOptimal,
            RetainReference,
            Fail,
        };

        constexpr ScpSolveAction scpSolveAction(const ScpSolveClassification classification) {
            switch(classification) {
                case ScpSolveClassification::Optimal: return ScpSolveAction::AcceptOptimal;
                case ScpSolveClassification::TimeLimit:
                case ScpSolveClassification::IterationLimit:
                    return ScpSolveAction::RetainReference;
                case ScpSolveClassification::Failure: return ScpSolveAction::Fail;
            }
            return ScpSolveAction::Fail;
        }

        inline double maximumAxisVelocity(const AxisPolynomialSpan &span,
                                          const double position_t::*component) {
            const auto at = [&](const double u) {
                return std::abs((3.0*span.a.*component*u*u + 2.0*span.b.*component*u
                    + span.c.*component) * span.inverseDuration);
            };
            auto result = std::max(at(0.0), at(1.0));
            if(std::abs(span.a.*component) > 1e-15) {
                const auto stationary = -(span.b.*component) / (3.0*(span.a.*component));
                if(stationary > 0.0 && stationary < 1.0)
                    result = std::max(result, at(stationary));
            }
            return result;
        }

        inline double maximumAxisAcceleration(const AxisPolynomialSpan &span,
                                              const double position_t::*component) {
            const auto at = [&](const double u) {
                return std::abs((6.0*span.a.*component*u + 2.0*span.b.*component)
                    * span.inverseDurationSquared);
            };
            return std::max(at(0.0), at(1.0));
        }

        inline double maximumAxisJerk(const AxisPolynomialSpan &span,
                                      const double position_t::*component) {
            return std::abs(6.0*span.a.*component * span.inverseDurationCubed);
        }
    }

    enum class ContinuousVelocityLimitCause {
        ProgrammedFeed,
        AxisVelocity,
        AxisCentripetalAcceleration,
        PathCentripetalAcceleration,
        AxisCurvatureDerivativeJerk,
        PathCurvatureDerivativeJerk,
    };

    struct ContinuousPlanningEffort {
        unsigned maximumLocalCorrectionPasses = 32;
        std::size_t geometryVerificationBudgetMultiplier = 36;
        double curvatureDerivativeVelocityCapMultiplier = 1.0;
        bool applyCurvatureDerivativeVelocityCap = true;
        bool measureCurvatureDerivativeNumerics = false;
        bool shareTimeLawCacheAcrossCompilations = true;
        // Measurement controls for the compile-local scalar time-law cache.
        // Production uses cached duration-only SCP line-search trials and the
        // automatic horizon-sized cache. Tests and offline benchmarks may
        // disable the trial cache or force a smaller power-of-two table.
        bool cacheScpLineSearchTrials = true;
        std::size_t timeLawCacheEntries = 0;
        // Experimental HiGHS-backed sequential linearization. These bounds
        // are NRT planning limits, not RT execution data.
        bool addScpAdjacentReachabilityRows = false;
        bool reuseScpBasis = true;
        unsigned scpIterations = 1;
        unsigned scpLineSearchSteps = 8;
        std::size_t scpSimplexIterationLimitMultiplier = 64;
        double scpVelocityTrustFraction = 0.35;
        double scpAccelerationTrustFraction = 0.75;
        double scpSolveTimeLimit = 0.5;
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
        std::size_t cacheSuccessfulHits = 0;
        std::size_t cacheFailureHits = 0;
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
            cacheSuccessfulHits+=other.cacheSuccessfulHits;
            cacheFailureHits+=other.cacheFailureHits;
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

    struct TimeLawDiagnostics {
        TimeLawCallDiagnostics exactStop;
        TimeLawCallDiagnostics continuousSeed;
        EndpointFeasibilityDiagnostics endpointFeasibility;
        std::vector<CorrectionPassLocalityDiagnostic> correctionPassLocality;

        TimeLawDiagnostics &operator+=(const TimeLawDiagnostics &other) {
            exactStop+=other.exactStop;
            continuousSeed+=other.continuousSeed;
            endpointFeasibility+=other.endpointFeasibility;
            correctionPassLocality.insert(correctionPassLocality.end(),
                other.correctionPassLocality.begin(),other.correctionPassLocality.end());
            return *this;
        }
    };

    inline TimeLawCallDiagnostics totalTimeLawCalls(const TimeLawDiagnostics &diagnostics) {
        auto total=diagnostics.exactStop;
        total+=diagnostics.continuousSeed;
        return total;
    }

    struct ContinuousTrajectoryPlan {
        std::vector<PlanChunk> chunks;
        std::vector<SpanId> activationSpans;
        // NRT-only development evidence; never crosses MotionBackend.
        std::vector<ContinuousPieceTimingDiagnostic> pieceTiming;
        double velocityOnlySeedDuration = 0.0;
        std::string correctionHistory;
        std::size_t ruckigBrakePhases = 0;
        std::size_t geometryVerificationAttempts = 0;
        std::size_t geometryVerificationHighWater = 0;
        std::size_t scpSolves = 0;
        std::size_t scpSimplexIterations = 0;
        std::size_t scpAdjacentReachabilityRows = 0;
        std::size_t scpBasisReuseAttempts = 0;
        std::size_t scpBasisReuseApplied = 0;
        std::size_t scpBasisDimensionMismatches = 0;
        std::size_t scpStationProposals = 0;
        std::size_t scpLineSearchTrials = 0;
        std::size_t scpAcceptedSteps = 0;
        std::size_t scpMaterializationAttempts = 0;
        double scpSeconds = 0.0;
        unsigned correctionPasses = 0;
        // Bounded NRT evidence for the first expected solver-resource fallback
        // in this plan. occurrences includes later correction-pass fallbacks.
        struct ResourceFallbackDiagnostic {
            ScpResourceFallbackReason reason = ScpResourceFallbackReason::None;
            unsigned correctionPass = 0;
            unsigned scpIteration = 0;
            std::size_t occurrences = 0;
        } scpResourceFallback;
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
            InfiniteJerkTrajectoryTimeResult *infiniteJerkTime = nullptr);
        std::expected<TriggeredMove, std::string> compileTriggeredMove(
            const ProbeMove &command, DigitalInputId input = 0,
            InputCondition condition = InputCondition::Active);
    };
}
