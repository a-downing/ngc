#pragma once

#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "machine/MachineConfiguration.h"
#include "machine/MotionBackend.h"

namespace ngc {
    enum class HomingOutcome { Completed, Stopped };

    struct HomingObservation {
        position_t machinePosition{};
        JointMotionState joints{};
        double commandProgress = 0.0;
        bool hasActiveMotion = false;
        std::uint64_t servoTicks = 0;
    };

    struct HomingResult {
        HomingOutcome outcome = HomingOutcome::Completed;
        HomingObservation observation{};
        JointMask homedJoints = 0;
    };

    // NRT lifetime and input-policy hooks surrounding the bounded MotionBackend
    // endpoint. Simulation uses these for synthetic switches and service-clock
    // stepping; a physical runtime may prepare no synthetic input and wait for
    // its independently serviced backend.
    struct HomingRuntimeCallbacks {
        std::function<bool()> stopRequested;
        std::function<bool(const TriggeredJointMove &)> prepareTriggeredMove;
        std::function<void()> serviceImmediate;
        std::function<std::uint64_t()> advanceServiceMotionPeriod;
        std::function<void()> waitForServiceMotion;
        std::function<void(const HomingObservation &)> observe;
    };

    class HomingController {
    public:
        HomingController(std::vector<AxisConfiguration> axes,
                         std::vector<JointConfiguration> joints,
                         HomingConfiguration homing, MotionBackend &backend);

        [[nodiscard]] bool available() const noexcept;
        [[nodiscard]] std::expected<HomingResult, std::string> run(
            EpochId epoch, const position_t &startingPosition,
            const HomingRuntimeCallbacks &callbacks);

    private:
        [[nodiscard]] const JointConfiguration *configuredJoint(JointId id) const;
        [[nodiscard]] TriggeredJointMove makeMove(
            const HomingGroupConfiguration &group, EpochId epoch, bool triggered,
            bool slow, bool backoff);
        [[nodiscard]] bool submitControl(
            const ControlRequest &request, const HomingRuntimeCallbacks &callbacks);
        [[nodiscard]] std::expected<bool, std::string> setJointPositions(
            JointMask joints, const JointVector &positions,
            const HomingRuntimeCallbacks &callbacks);
        [[nodiscard]] std::expected<TriggeredJointMoveCompleted, std::string> executeMove(
            const TriggeredJointMove &move, const HomingRuntimeCallbacks &callbacks);
        void takeSnapshots(const HomingRuntimeCallbacks &callbacks);
        void observeSnapshot(
            const ExecutionSnapshot &snapshot, const HomingRuntimeCallbacks &callbacks);
        [[nodiscard]] HomingResult result(HomingOutcome outcome, JointMask homedJoints) const;

        std::vector<AxisConfiguration> m_axes;
        std::vector<JointConfiguration> m_joints;
        HomingConfiguration m_homing;
        MotionBackend &m_backend;
        HomingObservation m_observation;
        RequestId m_nextRequest = 1;
        ChunkId m_nextChunk = 1;
        BranchSequence m_branch = 0;
        TriggeredMoveId m_nextMove = 1;
        std::optional<RequestId> m_stopRequest;
    };
}
