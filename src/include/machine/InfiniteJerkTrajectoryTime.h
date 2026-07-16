#pragma once

#include <cstddef>
#include <expected>
#include <functional>
#include <span>
#include <string>

#include "machine/MachineCommand.h"

namespace ngc {
    // One fixed-geometry, arc-length-parameterized section of a toolpath.
    // Velocity limits use machine units per second. The callbacks return the
    // exact first and second derivatives dq/ds and d2q/ds2 used by the path.
    struct InfiniteJerkPathPiece {
        double length = 0.0;
        double velocityLimit = 0.0;
        std::function<position_t(double)> tangentAt;
        std::function<position_t(double)> curvatureAt;
    };

    struct InfiniteJerkTrajectoryLimits {
        double pathAcceleration = 0.0;
        position_t axisVelocity{};
        position_t axisAcceleration{};
    };

    struct InfiniteJerkTrajectoryTimeOptions {
        std::size_t initialSubdivisions = 4;
        std::size_t maximumRefinements = 9;
        std::size_t maximumIntervals = 4 * 1024 * 1024;
        double relativeTolerance = 1e-5;
        double absoluteTolerance = 1e-8;
    };

    struct InfiniteJerkTrajectoryTimeResult {
        double duration = 0.0;
        // Absolute duration change between the final two grid refinements.
        double estimatedDurationError = 0.0;
        double maximumVelocity = 0.0;
        std::size_t intervals = 0;
        std::size_t refinements = 0;
    };

    // Computes the acceleration-limited minimum-time traversal of a fixed path
    // when scalar acceleration may change instantaneously (infinite jerk).
    // The acceleration constraints are derived analytically from
    // q'(s) a + q''(s) v^2. Numerical root finding and refinement integrate the
    // resulting continuous forward/backward maximum-velocity envelope; no
    // external optimizer or jerk-limited local solver is involved.
    std::expected<InfiniteJerkTrajectoryTimeResult, std::string>
    infiniteJerkTrajectoryTime(
        std::span<const InfiniteJerkPathPiece> pieces,
        const InfiniteJerkTrajectoryLimits &limits,
        double startVelocity = 0.0,
        double endVelocity = 0.0,
        const InfiniteJerkTrajectoryTimeOptions &options = {});
}
