#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "machine/MotionBackend.h"

namespace ngc {
    struct ExecutedTrajectorySpan {
        EpochId epoch = 0;
        ChunkId chunk = 0;
        SpanId span = 0;
        bool stopTail = false;
        std::vector<position_t> positions;
    };

    struct MockTrajectorySnapshot {
        std::uint64_t revision = 0;
        std::vector<ExecutedTrajectorySpan> spans;
    };

    struct ExecutedJerkSample {
        EpochId epoch = 0;
        ChunkId chunk = 0;
        SpanId span = 0;
        position_t position{};
        position_t velocity{};
        position_t acceleration{};
        position_t jerk{};
        double magnitude = 0.0;
        double physicalTime = 0.0;
        double referenceTime = 0.0;
        double executionRate = 1.0;
        double executionRateAcceleration = 0.0;
        double executionRateJerk = 0.0;
        bool feedHolding = false;
        bool stopTail = false;
    };

    // Development-only retained diagnostics. This is deliberately separate from
    // MotionBackend and has no physical-RT implementation requirement.
    class MockTrajectoryDiagnostics {
    public:
        virtual ~MockTrajectoryDiagnostics() = default;
        virtual void clearTrajectoryDiagnostics() = 0;
        virtual MockTrajectorySnapshot trajectorySnapshot() const = 0;
        // Incremental mock-servo samples for NRT visualization. Taking the
        // samples removes them from the pending diagnostic queue.
        virtual std::vector<ExecutedJerkSample> takeExecutedJerkSamples() = 0;
    };
}
