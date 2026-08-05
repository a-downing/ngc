#include "machine/InProcessSimulationRuntime.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ranges>
#include <utility>

#include "WindowsServoPacer.h"
#include "machine/HostedExecutorRuntime.h"
#include "machine/ProductionExecutorCore.h"
#include "machine/SpscChannel.h"
#include "machine/SimulationExecutor.h"

namespace ngc {
    namespace {
        constexpr std::size_t SYNTHETIC_INPUT_CAPACITY = 16;

        double dot(const position_t &left, const position_t &right) noexcept {
            return left.x * right.x + left.y * right.y
                + left.z * right.z + left.a * right.a
                + left.b * right.b + left.c * right.c;
        }

        double magnitude(const position_t &value) noexcept {
            return std::sqrt(dot(value, value));
        }

        bool inputLevel(const InputCondition condition,
                        const bool conditionMet) noexcept {
            return condition == InputCondition::Active
                    || condition == InputCondition::RisingEdge
                ? conditionMet : !conditionMet;
        }

        ProductionExecutorConfiguration simulationExecutorConfiguration(
            const MachineConfiguration &configuration) {
            auto simulation = configuration;
            simulation.machineExecutor.reset();
            auto result =
                hostedExecutorRuntimeConfiguration(simulation).executor;
            result.controlledStopLimits.velocity =
                simulation.trajectory.axisVelocity;
            result.controlledStopLimits.acceleration =
                simulation.trajectory.axisAcceleration;
            result.controlledStopLimits.jerk =
                simulation.trajectory.axisJerk;
            result.axisPosition = simulation.trajectory.axisPosition;
            const auto sanitize = [](double &value) {
                if (std::isnan(value) || value <= 0.0) {
                    value = std::numeric_limits<double>::infinity();
                }
            };
            sanitize(result.feedHold.pathAcceleration);
            sanitize(result.feedHold.axisAcceleration.x);
            sanitize(result.feedHold.axisAcceleration.y);
            sanitize(result.feedHold.axisAcceleration.z);
            sanitize(result.feedHold.axisAcceleration.a);
            sanitize(result.feedHold.axisAcceleration.b);
            sanitize(result.feedHold.axisAcceleration.c);
            const auto finiteStopLimit = [](double &value) {
                if (!std::isfinite(value) || value <= 0.0) {
                    value = 1.0e9;
                }
            };
            const auto sanitizeStop = [&](position_t &value) {
                finiteStopLimit(value.x);
                finiteStopLimit(value.y);
                finiteStopLimit(value.z);
                finiteStopLimit(value.a);
                finiteStopLimit(value.b);
                finiteStopLimit(value.c);
            };
            sanitizeStop(result.controlledStopLimits.velocity);
            sanitizeStop(result.controlledStopLimits.acceleration);
            sanitizeStop(result.controlledStopLimits.jerk);

            return result;
        }
    }

    class SimulationExecutor::Impl final : public MotionBackend {
        struct TriggerDescription {
            TriggeredMoveId move = 0;
            position_t axisTarget{};
            FixedArray<JointTrigger, MAX_JOINTS> jointTriggers;
            JointVector jointTarget{};
            DigitalInputId axisInput = 0;
            InputCondition axisCondition = InputCondition::Active;
            bool jointSpace = false;
        };

        struct SyntheticTransition {
            TriggeredMoveId move = 0;
            JointId joint = MAX_JOINTS;
            position_t axisPosition{};
            double jointPosition = 0.0;
        };

    public:
        Impl(const double servoPeriod,
             ProductionExecutorConfiguration configuration,
             const std::vector<JointConfiguration> &joints = {})
            : m_core(servoPeriod, std::move(configuration)) {
            m_jointSearchDirection.fill(1.0);
            m_jointActiveCondition.fill(InputCondition::Active);
            for (const auto &joint : joints) {
                m_jointSearchDirection[joint.id] = std::copysign(
                    1.0, joint.homing.searchVelocity
                        * joint.coordinateScale);
                m_jointActiveCondition[joint.id] =
                    joint.homing.condition;
            }
        }

        PublishResult tryPublish(const ExecutionItem &item) noexcept override {
            const auto result = m_core.tryPublish(item);
            if (result != PublishResult::Published) {
                return result;
            }

            if (const auto *move = std::get_if<TriggeredMove>(&item)) {
                (void)m_descriptions.tryPush({
                    .move = move->moveId,
                    .axisTarget = move->target,
                    .jointTriggers = {},
                    .jointTarget = {},
                    .axisInput = move->input,
                    .axisCondition = move->condition,
                    .jointSpace = false,
                });
            } else if (const auto *move =
                           std::get_if<TriggeredJointMove>(&item)) {
                (void)m_descriptions.tryPush({
                    .move = move->moveId,
                    .jointTriggers = move->triggers,
                    .jointTarget = move->target,
                    .jointSpace = true,
                });
            }

            return result;
        }

        DemandPublishResult publishDemand(
            const ExecutorDemand &demand) noexcept override {
            return m_core.publishDemand(demand);
        }

        SubmitResult trySubmit(const ControlRequest &request) noexcept override {
            return m_core.trySubmit(request);
        }

        bool tryTakeEvent(ExecutionEvent &event) noexcept override {
            return m_core.tryTakeEvent(event);
        }

        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override {
            return m_core.tryTakeSnapshot(snapshot);
        }

        void restoreStationaryState(
            const StationaryBackendState &state) noexcept {
            m_core.restoreStationaryState(
                state.commanded, state.feedback,
                state.commandedJoints, state.feedbackJoints);
        }

        void serviceImmediate() noexcept {
            m_core.serviceImmediate();
        }

        bool configureSyntheticInput(
            const TriggeredMoveId move,
            const position_t &position) noexcept {
            return m_transitions.tryPush({
                .move = move,
                .axisPosition = position,
            });
        }

        bool configureSyntheticJointInput(
            const TriggeredMoveId move, const JointId joint,
            const double position) noexcept {
            return m_transitions.tryPush({
                .move = move,
                .joint = joint,
                .jointPosition = position,
            });
        }

        void latchEmergencyStop() noexcept {
            m_core.latchEmergencyStop();
        }

        void resetEmergencyStop() noexcept {
            m_core.resetEmergencyStop();
        }

        bool advanceTick(const bool publishSnapshot) noexcept {
            drainSyntheticConfiguration();
            applySyntheticInputs();
            m_core.servoTick(publishSnapshot);
            const auto observation = m_core.lastTickObservation();
            recordObservation(observation);
            if (observation.completedMove != 0) {
                removeMove(observation.completedMove);
            }

            return observation.crossedChunk;
        }

        bool advance(const double seconds, const bool publishSnapshot) {
            if (seconds <= 0.0) {
                serviceImmediate();

                return false;
            }

            const auto ticks = static_cast<std::uint64_t>(std::ceil(
                seconds / m_core.servoPeriod() - 1e-12));
            auto crossed = false;
            for (std::uint64_t tick = 0; tick < ticks; ++tick) {
                crossed = advanceTick(
                    publishSnapshot && tick + 1 == ticks) || crossed;
            }

            return crossed;
        }

        void runUntilIdle(const double tickSeconds) {
            const auto ticksPerStep = std::max<std::uint64_t>(
                1, static_cast<std::uint64_t>(
                    std::ceil(
                        tickSeconds / m_core.servoPeriod() - 1e-12)));
            for (std::size_t guard = 0; guard < 10'000'000; ++guard) {
                for (std::uint64_t tick = 0; tick < ticksPerStep; ++tick) {
                    (void)advanceTick(false);
                    const auto snapshot = m_core.currentSnapshot();
                    if ((snapshot.state == BackendState::Held
                         || snapshot.state == BackendState::Disabled
                         || snapshot.state == BackendState::Faulted)
                        && snapshot.queuedExecutionItems == 0) {
                        return;
                    }
                }
            }
        }

        [[nodiscard]] double lastProgramSeconds() const noexcept {
            return m_lastProgramSeconds;
        }

        [[nodiscard]] double currentJerkMagnitude() const noexcept {
            return m_currentJerkMagnitude.load(std::memory_order_relaxed);
        }

        void clearTrajectoryDiagnostics() {
            std::scoped_lock lock(m_diagnosticsMutex);
            m_jerkSamples.clear();
            m_trajectory = {};
            m_physicalTime = 0.0;
            m_referenceTime = 0.0;
            m_lastProgramSeconds = 0.0;
            m_currentJerkMagnitude.store(0.0, std::memory_order_relaxed);
        }

        std::vector<ExecutedJerkSample> takeExecutedJerkSamples() {
            std::scoped_lock lock(m_diagnosticsMutex);

            return std::exchange(m_jerkSamples, {});
        }

        MockTrajectorySnapshot trajectorySnapshot() const {
            std::scoped_lock lock(m_diagnosticsMutex);

            return m_trajectory;
        }

    private:
        void drainSyntheticConfiguration() noexcept {
            TriggerDescription description;
            while (m_descriptions.tryPop(description)) {
                (void)m_pendingDescriptions.push(description);
            }
            SyntheticTransition transition;
            while (m_transitions.tryPop(transition)) {
                (void)m_pendingTransitions.push(transition);
            }
        }

        const TriggerDescription *descriptionFor(
                const TriggeredMoveId active) const noexcept {
            if (active != 0) {
                for (const auto &description : m_pendingDescriptions) {
                    if (description.move == active) {
                        return &description;
                    }
                }
            }

            return m_pendingDescriptions.size != 0
                ? &m_pendingDescriptions[0] : nullptr;
        }

        void applySyntheticInputs() noexcept {
            LogicalDigitalInputImage inputs;
            const auto context = m_core.motionContext();
            const auto *description = descriptionFor(context.move);
            if (description == nullptr) {
                m_core.setDigitalInputSamples(inputs);
                return;
            }

            for (const auto &transition : m_pendingTransitions) {
                if (transition.move != description->move) {
                    continue;
                }
                if (!description->jointSpace
                    && transition.joint == MAX_JOINTS) {
                    const auto target = context.move == description->move
                        ? context.axisTarget : description->axisTarget;
                    const auto current = context.axisPosition;
                    const auto conditionMet = dot(
                        target - current,
                        transition.axisPosition - current) <= 1e-12;
                    inputs[description->axisInput] = inputLevel(
                        description->axisCondition, conditionMet);
                } else if (description->jointSpace
                           && transition.joint < MAX_JOINTS) {
                    const auto joint = transition.joint;
                    const auto current = context.jointPosition[joint];
                    const auto conditionMet = m_jointSearchDirection[joint]
                        * (current - transition.jointPosition) >= -1e-12;
                    for (const auto &trigger :
                         description->jointTriggers) {
                        if (trigger.joint == joint) {
                            inputs[trigger.input] = inputLevel(
                                m_jointActiveCondition[joint],
                                conditionMet);
                        }
                    }
                }
            }
            m_core.setDigitalInputSamples(inputs);
        }

        void removeMove(const TriggeredMoveId move) noexcept {
            for (std::uint32_t index = 0;
                 index < m_pendingDescriptions.size;) {
                if (m_pendingDescriptions[index].move != move) {
                    ++index;
                    continue;
                }
                m_pendingDescriptions[index] = m_pendingDescriptions[
                    m_pendingDescriptions.size - 1];
                --m_pendingDescriptions.size;
            }
            for (std::uint32_t index = 0;
                 index < m_pendingTransitions.size;) {
                if (m_pendingTransitions[index].move != move) {
                    ++index;
                    continue;
                }
                m_pendingTransitions[index] = m_pendingTransitions[
                    m_pendingTransitions.size - 1];
                --m_pendingTransitions.size;
            }
        }

        void recordObservation(
            const ProductionExecutorTickObservation &observation) {
            m_lastProgramSeconds = observation.programSeconds;
            if (!observation.plannedMotion) {
                m_currentJerkMagnitude.store(
                    0.0, std::memory_order_relaxed);
                return;
            }

            const auto jerkMagnitude = magnitude(observation.jerk);
            m_currentJerkMagnitude.store(
                jerkMagnitude, std::memory_order_relaxed);
            std::scoped_lock lock(m_diagnosticsMutex);
            if (m_trajectory.spans.empty()
                || m_trajectory.spans.back().epoch != observation.epoch
                || m_trajectory.spans.back().chunk != observation.chunk
                || m_trajectory.spans.back().span != observation.span
                || m_trajectory.spans.back().stopTail
                    != observation.stopTail) {
                if (!m_trajectory.spans.empty()
                    && (m_trajectory.spans.back().positions.empty()
                        || (m_trajectory.spans.back().positions.back()
                            - observation.spanStart).length() > 1e-12)) {
                    m_trajectory.spans.back().positions.push_back(
                        observation.spanStart);
                }
                ExecutedTrajectorySpan span{
                    .epoch = observation.epoch,
                    .chunk = observation.chunk,
                    .span = observation.span,
                    .stopTail = observation.stopTail,
                    .positions = {},
                };
                span.positions.push_back(observation.spanStart);
                m_trajectory.spans.push_back(std::move(span));
            }
            m_trajectory.spans.back().positions.push_back(
                observation.commanded.position);
            ++m_trajectory.revision;
            m_physicalTime += observation.programSeconds;
            m_referenceTime += observation.programSeconds
                * observation.executionRate;
            m_jerkSamples.push_back({
                .epoch = observation.epoch,
                .chunk = observation.chunk,
                .span = observation.span,
                .position = observation.commanded.position,
                .velocity = observation.commanded.velocity,
                .acceleration = observation.commanded.acceleration,
                .jerk = observation.jerk,
                .magnitude = jerkMagnitude,
                .physicalTime = m_physicalTime,
                .referenceTime = m_referenceTime,
                .executionRate = observation.executionRate,
                .executionRateAcceleration =
                    observation.executionRateAcceleration,
                .executionRateJerk = observation.executionRateJerk,
                .feedHolding = observation.feedHolding,
                .stopTail = observation.stopTail,
            });
        }

        ProductionExecutorCore m_core;
        SpscChannel<TriggerDescription, SYNTHETIC_INPUT_CAPACITY>
            m_descriptions;
        SpscChannel<SyntheticTransition, SYNTHETIC_INPUT_CAPACITY>
            m_transitions;
        FixedArray<TriggerDescription, SYNTHETIC_INPUT_CAPACITY>
            m_pendingDescriptions;
        FixedArray<SyntheticTransition, SYNTHETIC_INPUT_CAPACITY>
            m_pendingTransitions;
        std::atomic<double> m_currentJerkMagnitude{0.0};
        mutable std::mutex m_diagnosticsMutex;
        std::vector<ExecutedJerkSample> m_jerkSamples;
        MockTrajectorySnapshot m_trajectory;
        double m_lastProgramSeconds = 0.0;
        double m_physicalTime = 0.0;
        double m_referenceTime = 0.0;
        std::array<double, MAX_JOINTS> m_jointSearchDirection{};
        std::array<InputCondition, MAX_JOINTS> m_jointActiveCondition{};
    };

    SimulationExecutor::SimulationExecutor(
        const double servoPeriod,
        const FeedHoldConfiguration &feedHold,
        const TrajectoryLimits &trajectory,
        const std::vector<AxisConfiguration> &axes,
        const std::vector<JointConfiguration> &joints) {
        MachineConfiguration configuration;
        configuration.feedHold = feedHold;
        configuration.trajectory = trajectory;
        configuration.simulation.servoPeriod = servoPeriod;
        configuration.axes = axes;
        configuration.joints = joints;
        m_impl = std::make_unique<Impl>(
            servoPeriod,
            simulationExecutorConfiguration(configuration), joints);
    }

    SimulationExecutor::SimulationExecutor(
        const double servoPeriod,
        const MachineConfiguration &configuration)
        : m_impl(std::make_unique<Impl>(
              servoPeriod,
              simulationExecutorConfiguration(configuration),
              configuration.joints)) { }

    SimulationExecutor::~SimulationExecutor() = default;

    PublishResult SimulationExecutor::tryPublish(
        const ExecutionItem &item) noexcept {
        return m_impl->tryPublish(item);
    }

    DemandPublishResult SimulationExecutor::publishDemand(
        const ExecutorDemand &demand) noexcept {
        return m_impl->publishDemand(demand);
    }

    SubmitResult SimulationExecutor::trySubmit(
        const ControlRequest &request) noexcept {
        return m_impl->trySubmit(request);
    }

    bool SimulationExecutor::tryTakeEvent(
        ExecutionEvent &event) noexcept {
        return m_impl->tryTakeEvent(event);
    }

    bool SimulationExecutor::tryTakeSnapshot(
        ExecutionSnapshot &snapshot) noexcept {
        return m_impl->tryTakeSnapshot(snapshot);
    }

    void SimulationExecutor::restoreStationaryState(
        const StationaryBackendState &state) noexcept {
        m_impl->restoreStationaryState(state);
    }

    void SimulationExecutor::serviceImmediate() noexcept {
        m_impl->serviceImmediate();
    }

    bool SimulationExecutor::advanceTick(
        const bool publishSnapshot) noexcept {
        return m_impl->advanceTick(publishSnapshot);
    }

    bool SimulationExecutor::advance(
        const double seconds, const bool publishSnapshot) {
        return m_impl->advance(seconds, publishSnapshot);
    }

    void SimulationExecutor::runUntilIdle(const double tickSeconds) {
        m_impl->runUntilIdle(tickSeconds);
    }

    bool SimulationExecutor::configureSyntheticInput(
        const TriggeredMoveId move,
        const position_t &position) noexcept {
        return m_impl->configureSyntheticInput(move, position);
    }

    bool SimulationExecutor::configureSyntheticJointInput(
        const TriggeredMoveId move, const JointId joint,
        const double position) noexcept {
        return m_impl->configureSyntheticJointInput(move, joint, position);
    }

    void SimulationExecutor::latchEmergencyStop() noexcept {
        m_impl->latchEmergencyStop();
    }

    void SimulationExecutor::resetEmergencyStop() noexcept {
        m_impl->resetEmergencyStop();
    }

    double SimulationExecutor::lastProgramSeconds() const noexcept {
        return m_impl->lastProgramSeconds();
    }

    double SimulationExecutor::currentJerkMagnitude() const noexcept {
        return m_impl->currentJerkMagnitude();
    }

    void SimulationExecutor::clearTrajectoryDiagnostics() {
        m_impl->clearTrajectoryDiagnostics();
    }

    MockTrajectorySnapshot SimulationExecutor::trajectorySnapshot() const {
        return m_impl->trajectorySnapshot();
    }

    std::vector<ExecutedJerkSample>
    SimulationExecutor::takeExecutedJerkSamples() {
        return m_impl->takeExecutedJerkSamples();
    }

    InProcessSimulationRuntime::InProcessSimulationRuntime(const TrajectoryLimits &limits,
                                                           const SimulationTiming &timing)
        : m_executor(std::make_unique<SimulationExecutor>(
              timing.servoPeriod, FeedHoldConfiguration{}, limits)),
          m_servoPeriod(timing.servoPeriod),
          m_schedulerPeriod(timing.schedulerPeriod),
          m_servoTicksPerSchedulerPeriod(static_cast<std::uint32_t>(
              std::max(1.0, std::round(timing.schedulerPeriod / timing.servoPeriod)))) { }

    InProcessSimulationRuntime::InProcessSimulationRuntime(const MachineConfiguration &configuration)
        : m_executor(std::make_unique<SimulationExecutor>(
              configuration.simulation.servoPeriod, configuration)),
          m_joints(configuration.joints),
          m_servoPeriod(configuration.simulation.servoPeriod),
          m_schedulerPeriod(configuration.simulation.schedulerPeriod),
          m_servoTicksPerSchedulerPeriod(static_cast<std::uint32_t>(std::max(
              1.0, std::round(configuration.simulation.schedulerPeriod
                              / configuration.simulation.servoPeriod)))) { }

    InProcessSimulationRuntime::~InProcessSimulationRuntime() {
        stop();
    }

    MotionBackend &InProcessSimulationRuntime::endpoint() noexcept {
        return *m_executor;
    }

    BackendCapabilities InProcessSimulationRuntime::capabilities() const noexcept {
        return {};
    }

    bool InProcessSimulationRuntime::restoreStationaryState(
        const StationaryBackendState &state) noexcept {
        std::scoped_lock lock(m_schedulerMutex);
        if (m_started) {
            return false;
        }

        m_executor->restoreStationaryState(state);

        return true;
    }

    bool InProcessSimulationRuntime::prepareTriggeredJointMove(
        const TriggeredJointMove &move) noexcept {
        for (const auto &trigger : move.triggers) {
            const auto found = std::ranges::find(
                m_joints, trigger.joint, &JointConfiguration::id);
            if (found == m_joints.end()) {
                return false;
            }
            const auto position =
                found->homing.switchPosition * found->coordinateScale;
            if (!configureSyntheticJointInput(move.moveId, trigger.joint, position)) {
                return false;
            }
        }

        return true;
    }

    void InProcessSimulationRuntime::serviceImmediate() {
        switch (requestService(0, ServiceRequestKind::Immediate)) {
            case ServiceRequestResult::Direct:
                serviceEmergencyStop();
                m_executor->serviceImmediate();
                break;
            case ServiceRequestResult::Completed:
            case ServiceRequestResult::Unavailable:
                break;
        }
    }

    void InProcessSimulationRuntime::waitForServiceMotion() {
        std::this_thread::sleep_for(std::chrono::duration<double>(m_schedulerPeriod));
    }

    void InProcessSimulationRuntime::start() {
        std::scoped_lock lock(m_schedulerMutex);
        if (m_started) {
            return;
        }

        m_stopping.store(false, std::memory_order_release);
        m_pacingError.store(0, std::memory_order_release);
        m_schedulerThread = std::thread(&InProcessSimulationRuntime::runScheduler, this);
        m_started = true;
    }

    void InProcessSimulationRuntime::stop() {
        {
            std::scoped_lock lock(m_schedulerMutex);
            if (!m_started) {
                return;
            }
            m_stopping.store(true, std::memory_order_release);
            m_timedExecutionActive.store(false, std::memory_order_release);
            m_executorRefillRequested.store(false, std::memory_order_release);
            m_nrtRefillActive.store(false, std::memory_order_release);
            m_rollingSupplyActive.store(false, std::memory_order_release);
        }
        m_schedulerCv.notify_all();
        m_schedulerThread.join();

        std::scoped_lock lock(m_schedulerMutex);
        m_started = false;
        discardServiceRequests();
        m_executorBatchActive.store(false, std::memory_order_release);
    }

    bool InProcessSimulationRuntime::beginTimedExecution() {
        {
            std::scoped_lock lock(m_schedulerMutex);
            if (!m_started || m_stopping.load(std::memory_order_acquire)
                || m_timedExecutionActive.load(std::memory_order_acquire)
                || m_pacingError.load(std::memory_order_acquire) != 0) {
                return false;
            }

            resetTimedDiagnostics();
            m_executor->clearTrajectoryDiagnostics();
            m_timedExecutionActive.store(true, std::memory_order_release);
        }
        m_schedulerCv.notify_all();

        return true;
    }

    void InProcessSimulationRuntime::endTimedExecution() {
        m_timedExecutionActive.store(false, std::memory_order_release);
        m_executorRefillRequested.store(false, std::memory_order_release);
        m_nrtRefillActive.store(false, std::memory_order_release);
        m_rollingSupplyActive.store(false, std::memory_order_release);
        m_schedulerCv.notify_all();

        while (m_executorBatchActive.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }

    void InProcessSimulationRuntime::setTickMultiplier(const int multiplier) noexcept {
        m_tickMultiplier.store(
            static_cast<std::uint32_t>(std::clamp(multiplier, 1, 1000)),
            std::memory_order_relaxed);
    }

    std::uint32_t InProcessSimulationRuntime::tickMultiplier() const noexcept {
        return m_tickMultiplier.load(std::memory_order_relaxed);
    }

    double InProcessSimulationRuntime::servoPeriod() const noexcept {
        return m_servoPeriod;
    }

    double InProcessSimulationRuntime::schedulerPeriod() const noexcept {
        return m_schedulerPeriod;
    }

    std::uint32_t InProcessSimulationRuntime::servoTicksPerSchedulerPeriod() const noexcept {
        return m_servoTicksPerSchedulerPeriod;
    }

    SimulationRuntimeSnapshot InProcessSimulationRuntime::snapshot() const noexcept {
        return {
            .servoPeriodSeconds = m_servoPeriod,
            .schedulerPeriodSeconds = m_schedulerPeriod,
            .servoTicksPerSchedulerPeriod = m_servoTicksPerSchedulerPeriod,
            .tickMultiplier = m_tickMultiplier.load(std::memory_order_relaxed),
            .servoTicks = m_servoTicks.load(std::memory_order_relaxed),
            .programElapsedSeconds = m_programElapsedSeconds.load(std::memory_order_relaxed),
            .executedPathJerk = m_executor->currentJerkMagnitude(),
            .deadlineMisses = m_deadlineMisses.load(std::memory_order_relaxed),
            .lastWakeLatenessSeconds = m_lastWakeLateness.load(std::memory_order_relaxed),
            .maximumWakeLatenessSeconds =
                m_maximumWakeLateness.load(std::memory_order_relaxed),
            .maximumTickExecutionSeconds =
                m_maximumTickExecution.load(std::memory_order_relaxed),
            .pacingError = m_pacingError.load(std::memory_order_acquire),
        };
    }

    void InProcessSimulationRuntime::setNrtRefillActive(const bool active) noexcept {
        m_nrtRefillActive.store(active, std::memory_order_release);
    }

    bool InProcessSimulationRuntime::executorBatchActive() const noexcept {
        return m_executorBatchActive.load(std::memory_order_acquire);
    }

    void InProcessSimulationRuntime::releaseRefillOpportunity() noexcept {
        m_executorRefillRequested.store(false, std::memory_order_release);
    }

    void InProcessSimulationRuntime::setRollingSupplyActive(const bool active) noexcept {
        m_rollingSupplyActive.store(active, std::memory_order_release);
    }

    void InProcessSimulationRuntime::advanceImmediate(const double seconds) {
        if (seconds <= 0.0) {
            serviceImmediate();
            return;
        }
        const auto ticks = static_cast<std::uint64_t>(
            std::ceil(seconds / m_servoPeriod));

        switch (requestService(ticks, ServiceRequestKind::Motion)) {
            case ServiceRequestResult::Direct:
                static_cast<void>(advanceServiceTicks(ticks, false));
                break;
            case ServiceRequestResult::Completed:
            case ServiceRequestResult::Unavailable:
                break;
        }
    }

    std::uint64_t InProcessSimulationRuntime::advanceServiceMotionPeriod() {
        const auto ticks = static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod)
            * m_tickMultiplier.load(std::memory_order_relaxed);

        switch (requestService(ticks, ServiceRequestKind::Motion)) {
            case ServiceRequestResult::Direct:
                return advanceServiceTicks(ticks, false);
            case ServiceRequestResult::Completed:
                return ticks;
            case ServiceRequestResult::Unavailable:
                return 0;
        }
    }

    bool InProcessSimulationRuntime::configureSyntheticInput(
        const TriggeredMoveId move, const position_t &transitionPosition) noexcept {
        return m_executor->configureSyntheticInput(move, transitionPosition);
    }

    bool InProcessSimulationRuntime::configureSyntheticJointInput(
        const TriggeredMoveId move, const JointId joint,
        const double transitionPosition) noexcept {
        return m_executor->configureSyntheticJointInput(
            move, joint, transitionPosition);
    }

    void InProcessSimulationRuntime::requestEmergencyStop(
        const EmergencyStopSource source) noexcept {
        m_emergencyStopInterface.request(source);
        m_schedulerCv.notify_all();
    }

    void InProcessSimulationRuntime::releaseEmergencyStop(
        const EmergencyStopSource source) noexcept {
        m_emergencyStopInterface.release(source);
    }

    std::uint64_t InProcessSimulationRuntime::requestEmergencyStopReset() noexcept {
        const auto generation = m_emergencyStopInterface.requestReset();
        m_schedulerCv.notify_all();

        return generation;
    }

    EmergencyStopStatus InProcessSimulationRuntime::emergencyStopStatus() const noexcept {
        return m_emergencyStopInterface.status();
    }

    void InProcessSimulationRuntime::serviceEmergencyStop() noexcept {
        const auto wasLatched = m_emergencyStopState.latched();
        const auto isLatched = m_emergencyStopState.update();
        if (isLatched && !wasLatched) {
            m_executor->latchEmergencyStop();
        } else if (!isLatched && wasLatched) {
            m_executor->resetEmergencyStop();
        }
    }

    InProcessSimulationRuntime::ServiceRequestResult
    InProcessSimulationRuntime::requestService(
        const std::uint64_t ticks, const ServiceRequestKind kind) {
        std::unique_lock lock(m_schedulerMutex);
        if (!m_started) {
            return ServiceRequestResult::Direct;
        }
        if (m_stopping.load(std::memory_order_acquire)) {
            return ServiceRequestResult::Unavailable;
        }

        const auto request = ServiceRequest {
            ++m_nextServiceRequest, ticks, kind,
        };
        while (m_serviceRequest.has_value()
               && !m_stopping.load(std::memory_order_acquire)) {
            m_schedulerCv.wait(lock);
        }
        if (m_stopping.load(std::memory_order_acquire)) {
            return ServiceRequestResult::Unavailable;
        }

        m_serviceRequest = request;
        m_schedulerCv.notify_all();
        m_schedulerCv.wait(lock, [&] {
            return m_completedServiceRequest >= request.generation
                || m_stopping.load(std::memory_order_acquire);
        });

        if (m_stopping.load(std::memory_order_acquire)) {
            return ServiceRequestResult::Unavailable;
        }
        if (m_completedServiceRequest >= request.generation) {
            return ServiceRequestResult::Completed;
        }

        return ServiceRequestResult::Unavailable;
    }

    void InProcessSimulationRuntime::serviceRequestedWork(
        const ServiceRequest &request) noexcept {
        if (request.kind == ServiceRequestKind::Immediate) {
            if (!m_stopping.load(std::memory_order_acquire)) {
                serviceEmergencyStop();
                if (!m_stopping.load(std::memory_order_acquire)) {
                    m_executor->serviceImmediate();
                }
            }
        } else {
            static_cast<void>(advanceServiceTicks(request.ticks, true));
        }

        {
            std::scoped_lock lock(m_schedulerMutex);
            m_completedServiceRequest = request.generation;
        }
        m_schedulerCv.notify_all();
    }

    std::uint64_t InProcessSimulationRuntime::advanceServiceTicks(
        const std::uint64_t ticks, const bool stopWhenRequested) noexcept {
        auto advanced = std::uint64_t {0};
        for (; advanced < ticks; ++advanced) {
            if (stopWhenRequested
                && m_stopping.load(std::memory_order_acquire)) {
                break;
            }

            serviceEmergencyStop();
            if (stopWhenRequested
                && m_stopping.load(std::memory_order_acquire)) {
                break;
            }

            (void)m_executor->advanceTick(advanced + 1 == ticks);
        }

        return advanced;
    }

    void InProcessSimulationRuntime::discardServiceRequests() noexcept {
        m_serviceRequest.reset();
    }

    bool InProcessSimulationRuntime::emergencyStopWorkPending() const noexcept {
        const auto status = m_emergencyStopInterface.status();
        const auto requested = m_emergencyStopInterface.requestedSources();

        return (requested & ~status.latchedSources) != 0
            || m_emergencyStopInterface.resetGeneration()
                != status.acknowledgedResetGeneration;
    }

    void InProcessSimulationRuntime::clearTrajectoryDiagnostics() {
        m_executor->clearTrajectoryDiagnostics();
    }

    std::vector<ExecutedJerkSample> InProcessSimulationRuntime::takeExecutedJerkSamples() {
        return m_executor->takeExecutedJerkSamples();
    }

    void InProcessSimulationRuntime::runScheduler() {
        using clock = std::chrono::steady_clock;

        WindowsServoPacer pacer(m_schedulerPeriod);
        if (!pacer.valid()) {
            {
                std::scoped_lock lock(m_schedulerMutex);
                m_pacingError.store(pacer.errorCode(), std::memory_order_release);
                m_stopping.store(true, std::memory_order_release);
                discardServiceRequests();
            }
            m_schedulerCv.notify_all();

            return;
        }

        for (;;) {
            std::optional<ServiceRequest> serviceRequest;
            {
                std::unique_lock lock(m_schedulerMutex);
                m_schedulerCv.wait(lock, [&] {
                    return m_stopping.load(std::memory_order_acquire)
                        || m_timedExecutionActive.load(std::memory_order_acquire)
                        || emergencyStopWorkPending()
                        || m_serviceRequest.has_value();
                });
                if (m_stopping.load(std::memory_order_acquire)) {
                    discardServiceRequests();
                    m_schedulerCv.notify_all();

                    return;
                }
                if (!m_timedExecutionActive.load(std::memory_order_acquire)) {
                    if (emergencyStopWorkPending()) {
                        serviceEmergencyStop();
                    } else if (m_serviceRequest.has_value()) {
                        serviceRequest = *m_serviceRequest;
                        m_serviceRequest.reset();
                    }
                } else {
                    pacer.reset();
                }
            }
            if (serviceRequest.has_value()) {
                serviceRequestedWork(*serviceRequest);
                continue;
            }

            while (m_timedExecutionActive.load(std::memory_order_acquire)
                   && !m_stopping.load(std::memory_order_acquire)) {
                WindowsServoPacer::WaitResult timing;
                if (!pacer.wait(timing)) {
                    m_pacingError.store(pacer.errorCode(), std::memory_order_release);
                    m_timedExecutionActive.store(false, std::memory_order_release);
                    break;
                }
                if (!m_timedExecutionActive.load(std::memory_order_acquire)) {
                    continue;
                }

                m_lastWakeLateness.store(timing.latenessSeconds, std::memory_order_relaxed);
                updateMaximum(m_maximumWakeLateness, timing.latenessSeconds);
                m_deadlineMisses.fetch_add(timing.missedPeriods, std::memory_order_relaxed);
                const auto multiplier = m_tickMultiplier.load(std::memory_order_relaxed);
                const auto ticksThisPeriod =
                    static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod) * multiplier;

                if (multiplier > 1) {
                    for (;;) {
                        while ((m_nrtRefillActive.load(std::memory_order_acquire)
                                || m_rollingSupplyActive.load(std::memory_order_acquire))
                               && m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            std::this_thread::yield();
                        }
                        if (!m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            break;
                        }
                        m_executorBatchActive.store(true, std::memory_order_release);
                        if (!m_nrtRefillActive.load(std::memory_order_acquire)
                            && !m_rollingSupplyActive.load(std::memory_order_acquire)) {
                            break;
                        }
                        m_executorBatchActive.store(false, std::memory_order_release);
                    }
                    if (!m_timedExecutionActive.load(std::memory_order_relaxed)) {
                        m_executorBatchActive.store(false, std::memory_order_release);
                        continue;
                    }
                } else {
                    m_executorBatchActive.store(true, std::memory_order_release);
                }

                for (std::uint64_t tick = 0;
                     tick < ticksThisPeriod
                     && m_timedExecutionActive.load(std::memory_order_relaxed);
                     ++tick) {
                    const auto started = clock::now();
                    serviceEmergencyStop();
                    const auto crossedChunk =
                        m_executor->advanceTick(tick + 1 == ticksThisPeriod);
                    m_programElapsedSeconds.fetch_add(
                        m_executor->lastProgramSeconds(), std::memory_order_relaxed);
                    const auto duration =
                        std::chrono::duration<double>(clock::now() - started).count();
                    updateMaximum(m_maximumTickExecution, duration);
                    m_servoTicks.fetch_add(1, std::memory_order_relaxed);
                    if (crossedChunk && tick + 1 < ticksThisPeriod) {
                        m_executorRefillRequested.store(true, std::memory_order_release);
                        m_executorBatchActive.store(false, std::memory_order_release);
                        while (m_executorRefillRequested.load(std::memory_order_acquire)
                               && m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            std::this_thread::yield();
                        }
                        if (!m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            break;
                        }
                        m_executorBatchActive.store(true, std::memory_order_release);
                    }
                }
                m_executorBatchActive.store(false, std::memory_order_release);
            }
        }
    }

    void InProcessSimulationRuntime::resetTimedDiagnostics() noexcept {
        m_executorBatchActive.store(false, std::memory_order_relaxed);
        m_executorRefillRequested.store(false, std::memory_order_relaxed);
        m_nrtRefillActive.store(false, std::memory_order_relaxed);
        m_rollingSupplyActive.store(false, std::memory_order_relaxed);
        m_servoTicks.store(0, std::memory_order_relaxed);
        m_programElapsedSeconds.store(0.0, std::memory_order_relaxed);
        m_deadlineMisses.store(0, std::memory_order_relaxed);
        m_lastWakeLateness.store(0.0, std::memory_order_relaxed);
        m_maximumWakeLateness.store(0.0, std::memory_order_relaxed);
        m_maximumTickExecution.store(0.0, std::memory_order_relaxed);
    }

    void InProcessSimulationRuntime::updateMaximum(
        std::atomic<double> &target, const double value) noexcept {
        auto current = target.load(std::memory_order_relaxed);
        while (current < value && !target.compare_exchange_weak(
                   current, value, std::memory_order_relaxed,
                   std::memory_order_relaxed)) { }
    }
}
