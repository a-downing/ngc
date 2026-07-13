#pragma once

#include <expected>
#include <optional>
#include <string>

#include "machine/MotionBackend.h"

namespace ngc {
    struct TrajectoryLimits {
        // Machine units per second squared. This is deliberately a configuration
        // object even though the initial value is hardcoded.
        double pathAcceleration = 10.0;
        double rapidSpeed = 100.0; // Machine units per minute.
        double arcChordTolerance = 0.0001;
        double pathJerk = 100.0;
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
    };
}
