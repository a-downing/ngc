#pragma once

#include <expected>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "machine/MotionBackend.h"

namespace ngc {
    struct ContinuousPieceTimingDiagnostic {
        std::size_t input = 0;
        double length = 0.0;
        double velocityLimit = 0.0;
        double accelerationLimit = 0.0;
        double jerkLimit = 0.0;
        double entryVelocity = 0.0;
        double entryAcceleration = 0.0;
        double exitVelocity = 0.0;
        double exitAcceleration = 0.0;
        double duration = 0.0;
    };

    struct ContinuousAccelerationOracleSegment {
        std::size_t piece = 0;
        std::size_t input = 0;
        double length = 0.0;
        double velocityLimit = 0.0;
        position_t tangent{};
        position_t curvature{};
    };

    // Optional NRT-only data for the standalone Clarabel comparison tool.
    // This is deliberately not part of PlanChunk or MotionBackend.
    struct ContinuousAccelerationOracleModel {
        double pathAcceleration = 0.0;
        position_t axisAcceleration{};
        double plannerDuration = 0.0;
        std::vector<ContinuousAccelerationOracleSegment> segments;
    };

    struct ContinuousTrajectoryPlan {
        std::vector<PlanChunk> chunks;
        std::vector<SpanId> activationSpans;
        // NRT-only development evidence; never crosses MotionBackend.
        std::vector<ContinuousPieceTimingDiagnostic> pieceTiming;
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
    };

    class ExactStopTrajectoryPlanner {
        TrajectoryLimits m_limits;
        EpochId m_epoch = 1;
        ChunkId m_nextChunk = 1;
        SpanId m_nextSpan = 1;
        BranchSequence m_previousBranch = 0;
        position_t m_position{};

    public:
        explicit ExactStopTrajectoryPlanner(TrajectoryLimits limits = {});

        void reset(EpochId epoch, const position_t &position = {});
        const TrajectoryLimits &limits() const { return m_limits; }
        void setLimits(const TrajectoryLimits &limits) { m_limits = limits; }
        const position_t &plannedPosition() const { return m_position; }

        std::expected<PlanChunk, std::string> compile(const MachineCommand &command);
        std::expected<std::unique_ptr<ContinuousTrajectoryPlan>, std::string> compileContinuous(
            std::span<const MachineCommand> commands, double blendScale,
            ContinuousAccelerationOracleModel *oracleModel = nullptr);
        std::expected<TriggeredMove, std::string> compileTriggeredMove(
            const ProbeMove &command, DigitalInputId input = 0,
            InputCondition condition = InputCondition::Active);
    };
}
