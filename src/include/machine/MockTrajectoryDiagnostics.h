#pragma once

#include <algorithm>
#include <cmath>
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
        double executedUntil = 1.0;
        AxisPolynomialSpan polynomial{};
        position_t terminalPosition{};
    };

    struct MockTrajectorySnapshot {
        std::uint64_t revision = 0;
        std::vector<ExecutedTrajectorySpan> spans;
    };

    inline std::size_t diagnosticServoSegmentCount(const ExecutedTrajectorySpan &span,
                                                   const double servoPeriod) {
        const auto executedDuration = span.polynomial.duration * span.executedUntil;
        return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(executedDuration / servoPeriod)));
    }

    // Development-only retained diagnostics. This is deliberately separate from
    // MotionBackend and has no physical-RT implementation requirement.
    class MockTrajectoryDiagnostics {
    public:
        virtual ~MockTrajectoryDiagnostics() = default;
        virtual void clearTrajectoryDiagnostics() = 0;
        virtual MockTrajectorySnapshot trajectorySnapshot() const = 0;
    };
}
