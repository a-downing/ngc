#pragma once

#include <cstdint>

#include "machine/MotionBackend.h"

namespace ngc {
    struct BackendCapabilities { };

    struct StationaryBackendState {
        MotionState commanded;
        MotionState feedback;
        JointMotionState commandedJoints;
        JointMotionState feedbackJoints;
    };

    class BackendRuntime {
    public:
        virtual ~BackendRuntime() = default;

        virtual MotionBackend &endpoint() noexcept = 0;
        virtual void start() = 0;
        virtual void stop() = 0;
        [[nodiscard]] virtual BackendCapabilities capabilities() const noexcept = 0;
        [[nodiscard]] virtual bool restoreStationaryState(
            const StationaryBackendState &) noexcept {
            return false;
        }

        [[nodiscard]] virtual bool prepareTriggeredJointMove(
            const TriggeredJointMove &) noexcept = 0;
        virtual void serviceImmediate() = 0;
        [[nodiscard]] virtual std::uint64_t advanceServiceMotionPeriod() = 0;
        virtual void waitForServiceMotion() = 0;
    };
}
