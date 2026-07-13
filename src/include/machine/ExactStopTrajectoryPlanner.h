#pragma once

#include <expected>
#include <limits>
#include <optional>
#include <string>

#include "machine/MotionBackend.h"

namespace ngc {
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
        std::expected<TriggeredMove, std::string> compileTriggeredMove(
            const ProbeMove &command, DigitalInputId input = 0,
            InputCondition condition = InputCondition::Active);
    };
}
