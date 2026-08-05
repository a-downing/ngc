#include "machine/MockMotionBackend.h"

#include <utility>

#include "machine/ExecutorDemandController.h"
#include "machine/SimulationExecutor.h"
#include "machine/SpscChannel.h"

namespace ngc {
    class MockMotionBackend::Impl {
    public:
        Impl(const FeedHoldConfiguration &feedHold,
             const TrajectoryLimits &trajectory,
             const std::vector<AxisConfiguration> &axes = {},
             const std::vector<JointConfiguration> &joints = {})
            : executor(0.001, feedHold, trajectory, axes, joints),
              demand(executor) { }

        SubmitResult submit(const ControlRequest &request) noexcept {
            return std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::same_as<T, ResetRequest>) {
                    epoch = value.nextEpoch;
                    return completeDemand(
                        value.id, demand.request(
                            epoch, ExecutorDemandMode::Idle));
                } else if constexpr (std::same_as<T, EnableRequest>) {
                    if (epoch == 0) {
                        return executor.trySubmit(request);
                    }
                    return completeDemand(
                        value.id, demand.request(
                            epoch, ExecutorDemandMode::Idle));
                } else if constexpr (std::same_as<T, StartRequest>) {
                    epoch = value.epoch;
                    return completeDemand(
                        value.id, demand.request(
                            epoch, ExecutorDemandMode::Run));
                } else if constexpr (std::same_as<T, ResumeRequest>) {
                    if (demand.mode() == ExecutorDemandMode::Stop
                        && demand.epoch() == value.epoch) {
                        return completeDemand(value.id, false);
                    }
                    epoch = value.epoch;
                    return completeDemand(
                        value.id, demand.request(
                            epoch, ExecutorDemandMode::Run));
                } else if constexpr (std::same_as<T, FeedHoldRequest>) {
                    return completeDemand(
                        value.id, demand.request(
                            epoch, ExecutorDemandMode::FeedHold));
                } else if constexpr (
                    std::same_as<T, ControlledStopRequest>
                    || std::same_as<T, AbortRequest>) {
                    return completeDemand(
                        value.id, demand.request(
                            epoch, ExecutorDemandMode::Stop));
                } else if constexpr (std::same_as<T, DisableRequest>) {
                    return completeDemand(
                        value.id, demand.request(
                            0, ExecutorDemandMode::Disabled));
                } else {
                    return executor.trySubmit(request);
                }
            }, request);
        }

        SubmitResult completeDemand(
            const RequestId request, const bool accepted) noexcept {
            return events.tryPush(RequestCompleted{request, accepted})
                ? SubmitResult::Submitted : SubmitResult::Full;
        }

        SimulationExecutor executor;
        ExecutorDemandController demand;
        SpscChannel<ExecutionEvent, 16> events;
        EpochId epoch = 0;
    };

    MockMotionBackend::MockMotionBackend(
        const FeedHoldConfiguration &feedHold,
        const TrajectoryLimits &trajectory)
        : m_impl(std::make_unique<Impl>(feedHold, trajectory)) { }

    MockMotionBackend::MockMotionBackend(
        const std::vector<AxisConfiguration> &axes,
        const std::vector<JointConfiguration> &joints)
        : m_impl(std::make_unique<Impl>(
              FeedHoldConfiguration{}, TrajectoryLimits{}, axes, joints)) { }

    MockMotionBackend::MockMotionBackend(
        const FeedHoldConfiguration &feedHold,
        const TrajectoryLimits &trajectory,
        const std::vector<AxisConfiguration> &axes,
        const std::vector<JointConfiguration> &joints)
        : m_impl(std::make_unique<Impl>(
              feedHold, trajectory, axes, joints)) { }

    MockMotionBackend::~MockMotionBackend() = default;

    PublishResult MockMotionBackend::tryPublish(
        const ExecutionItem &item) noexcept {
        return m_impl->executor.tryPublish(item);
    }

    DemandPublishResult MockMotionBackend::publishDemand(
        const ExecutorDemand &demand) noexcept {
        return m_impl->executor.publishDemand(demand);
    }

    SubmitResult MockMotionBackend::trySubmit(
        const ControlRequest &request) noexcept {
        return m_impl->submit(request);
    }

    bool MockMotionBackend::tryTakeEvent(
        ExecutionEvent &event) noexcept {
        return m_impl->events.tryPop(event)
            || m_impl->executor.tryTakeEvent(event);
    }

    bool MockMotionBackend::tryTakeSnapshot(
        ExecutionSnapshot &snapshot) noexcept {
        return m_impl->executor.tryTakeSnapshot(snapshot);
    }

    void MockMotionBackend::restoreStationaryState(
        const StationaryBackendState &state) noexcept {
        m_impl->executor.restoreStationaryState(state);
    }

    void MockMotionBackend::latchEmergencyStop() noexcept {
        m_impl->executor.latchEmergencyStop();
    }

    void MockMotionBackend::resetEmergencyStop() noexcept {
        m_impl->executor.resetEmergencyStop();
    }

    void MockMotionBackend::advance(const double seconds) {
        (void)m_impl->executor.advance(seconds);
    }

    bool MockMotionBackend::advanceTick(
        const double seconds, const bool publishSnapshot) {
        return m_impl->executor.advance(seconds, publishSnapshot);
    }

    double MockMotionBackend::lastAdvanceProgramSeconds() const noexcept {
        return m_impl->executor.lastProgramSeconds();
    }

    double MockMotionBackend::currentProgramJerkMagnitude() const noexcept {
        return m_impl->executor.currentJerkMagnitude();
    }

    void MockMotionBackend::runUntilIdle() {
        m_impl->executor.runUntilIdle();
    }

    void MockMotionBackend::runUntilIdle(const double tickSeconds) {
        m_impl->executor.runUntilIdle(tickSeconds);
    }

    bool MockMotionBackend::configureSyntheticInput(
        const TriggeredMoveId move,
        const position_t &transitionPosition) noexcept {
        return m_impl->executor.configureSyntheticInput(
            move, transitionPosition);
    }

    bool MockMotionBackend::configureSyntheticJointInput(
        const TriggeredMoveId move, const JointId joint,
        const double transitionPosition) noexcept {
        return m_impl->executor.configureSyntheticJointInput(
            move, joint, transitionPosition);
    }

    void MockMotionBackend::clearTrajectoryDiagnostics() {
        m_impl->executor.clearTrajectoryDiagnostics();
    }

    MockTrajectorySnapshot MockMotionBackend::trajectorySnapshot() const {
        return m_impl->executor.trajectorySnapshot();
    }

    std::vector<ExecutedJerkSample>
    MockMotionBackend::takeExecutedJerkSamples() {
        return m_impl->executor.takeExecutedJerkSamples();
    }
}
