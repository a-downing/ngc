#pragma once

#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "machine/MachineConfiguration.h"
#include "machine/ExecutorDemandController.h"
#include "machine/MotionBackend.h"
#include "machine/ServicedMotionOperation.h"

namespace ngc {
    enum class JoggingOutcome { Completed, Stopped };

    struct JoggingObservation {
        position_t machinePosition{};
        JointMotionState joints{};
        double commandProgress = 0.0;
        bool hasActiveMotion = false;
        std::uint64_t servoTicks = 0;
        BackendState backendState = BackendState::Disabled;
        std::uint32_t backendFaultCode = 0;
    };

    struct JoggingResult {
        JoggingOutcome outcome = JoggingOutcome::Completed;
        JoggingObservation observation{};
        JogStopReason stopReason = JogStopReason::TargetReached;
    };

    struct JoggingRuntimeCallbacks {
        std::function<bool()> shutdownRequested;
        std::function<void()> serviceImmediate;
        std::function<std::uint64_t()> advanceServiceMotionPeriod;
        std::function<void()> waitForServiceMotion;
        std::function<void()> stopRuntime;
        std::function<void()> faultSession;
        std::function<void(const JoggingObservation &)> observe;
    };

    class JoggingController {
    public:
        JoggingController(std::vector<AxisConfiguration> axes,
                          std::vector<JointConfiguration> joints,
                          MotionBackend &backend,
                          ExecutorDemandController &demand);

        [[nodiscard]] std::expected<JoggingResult, std::string> run(
            EpochId epoch, const position_t &startingPosition,
            const ControlRequest &firstRequest,
            const std::function<std::optional<ControlRequest>()> &nextControl,
            const JoggingRuntimeCallbacks &callbacks);

    private:
        [[nodiscard]] const JointConfiguration *configuredJoint(JointId id) const;
        [[nodiscard]] std::expected<bool, std::string> setJointPositions(
            JointMask joints, const JointVector &positions,
            ServicedMotionOperation &operation);
        void observeSnapshot(
            const ExecutionSnapshot &snapshot, std::uint64_t servoTicks,
            const JoggingRuntimeCallbacks &callbacks);

        std::vector<AxisConfiguration> m_axes;
        std::vector<JointConfiguration> m_joints;
        MotionBackend &m_backend;
        ExecutorDemandController &m_demand;
        JoggingObservation m_observation;
        RequestId m_nextRequest = 1;
    };
}
