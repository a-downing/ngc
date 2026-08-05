#pragma once

#include <memory>
#include <vector>

#include "machine/BackendRuntime.h"
#include "machine/MachineConfiguration.h"
#include "machine/MockTrajectoryDiagnostics.h"

namespace ngc {
    // Thin simulation policy around ProductionExecutorCore. Motion and control
    // semantics remain owned by the production core; this layer provides only
    // synthetic inputs, fixed-tick advancement, and retained NRT diagnostics.
    class SimulationExecutor final : public MotionBackend {
    public:
        SimulationExecutor(
            double servoPeriod,
            const FeedHoldConfiguration &feedHold = {},
            const TrajectoryLimits &trajectory = {},
            const std::vector<AxisConfiguration> &axes = {},
            const std::vector<JointConfiguration> &joints = {});
        SimulationExecutor(
            double servoPeriod,
            const MachineConfiguration &configuration);
        ~SimulationExecutor() override;
        SimulationExecutor(const SimulationExecutor &) = delete;
        SimulationExecutor &operator=(const SimulationExecutor &) = delete;

        PublishResult tryPublish(const ExecutionItem &item) noexcept override;
        DemandPublishResult publishDemand(
            const ExecutorDemand &demand) noexcept override;
        SubmitResult trySubmit(const ControlRequest &request) noexcept override;
        bool tryTakeEvent(ExecutionEvent &event) noexcept override;
        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override;

        void restoreStationaryState(
            const StationaryBackendState &state) noexcept;
        void serviceImmediate() noexcept;
        bool advanceTick(bool publishSnapshot = true) noexcept;
        bool advance(double seconds, bool publishSnapshot = true);
        void runUntilIdle(double tickSeconds = 0.001);
        bool configureSyntheticInput(
            TriggeredMoveId move,
            const position_t &position) noexcept;
        bool configureSyntheticJointInput(
            TriggeredMoveId move, JointId joint,
            double position) noexcept;
        void latchEmergencyStop() noexcept;
        void resetEmergencyStop() noexcept;

        [[nodiscard]] double lastProgramSeconds() const noexcept;
        [[nodiscard]] double currentJerkMagnitude() const noexcept;
        void clearTrajectoryDiagnostics();
        [[nodiscard]] MockTrajectorySnapshot trajectorySnapshot() const;
        std::vector<ExecutedJerkSample> takeExecutedJerkSamples();

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
