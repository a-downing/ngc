#include "machine/ProductionExecutorCore.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

namespace ngc {
    namespace {
        constexpr std::uint32_t EVENT_OVERFLOW_FAULT = 1;
        constexpr std::uint32_t INVALID_EXECUTION_STATE_FAULT = 2;

        bool zeroHighCubicControls(const AxisPolynomialSpan &span) noexcept {
            return span.coefficients[3].length() == 0.0
                && span.coefficients[4].length() == 0.0;
        }

        bool finitePosition(const position_t &position) noexcept {
            return std::isfinite(position.x) && std::isfinite(position.y)
                && std::isfinite(position.z) && std::isfinite(position.a)
                && std::isfinite(position.b) && std::isfinite(position.c);
        }

        bool finiteMotionState(const MotionState &state) noexcept {
            return finitePosition(state.position)
                && finitePosition(state.velocity)
                && finitePosition(state.acceleration);
        }

        double magnitude(const position_t &value) noexcept {
            return std::sqrt(value.x * value.x + value.y * value.y
                + value.z * value.z + value.a * value.a
                + value.b * value.b + value.c * value.c);
        }

        double dot(const position_t &left, const position_t &right) noexcept {
            return left.x * right.x + left.y * right.y
                + left.z * right.z + left.a * right.a
                + left.b * right.b + left.c * right.c;
        }

        position_t scaled(const position_t &value, const double scale) noexcept {
            return {
                value.x * scale, value.y * scale, value.z * scale,
                value.a * scale, value.b * scale, value.c * scale,
            };
        }

        bool approximatelyEqual(const double actual,
                                const double expected) noexcept {
            return std::abs(actual - expected)
                <= 1e-12 * std::max(1.0, std::abs(expected));
        }

        bool validInputCondition(const InputCondition condition) noexcept {
            switch (condition) {
                case InputCondition::Active:
                case InputCondition::Inactive:
                case InputCondition::RisingEdge:
                case InputCondition::FallingEdge: return true;
            }

            return false;
        }
    }

    ProductionExecutorCore::ProductionExecutorCore(const double servoPeriod)
        : m_servoPeriod(servoPeriod) {
        if (!std::isfinite(servoPeriod) || servoPeriod <= 0.0) {
            throw std::invalid_argument(
                "production executor servo period must be finite and positive");
        }
    }

    PublishResult ProductionExecutorCore::tryPublish(
        const ExecutionItem &item) noexcept {
        if (!validExecutionItem(item)) {
            return PublishResult::Invalid;
        }

        for (std::uint8_t index = 0; index < m_planSlots.size(); ++index) {
            auto expected = false;
            if (!m_planSlots[index].occupied.compare_exchange_strong(
                    expected, true, std::memory_order_acquire)) {
                continue;
            }

            auto &slot = m_planSlots[index];
            slot.item = item;
            slot.normalMotionNanoseconds = normalMotionNanoseconds(item);
            m_queuedNormalMotionNanoseconds.fetch_add(
                slot.normalMotionNanoseconds, std::memory_order_release);
            m_queuedExecutionItems.fetch_add(1, std::memory_order_release);
            if (m_plans.tryPush(index)) {
                return PublishResult::Published;
            }

            m_queuedNormalMotionNanoseconds.fetch_sub(
                slot.normalMotionNanoseconds, std::memory_order_acq_rel);
            m_queuedExecutionItems.fetch_sub(1, std::memory_order_acq_rel);
            slot.occupied.store(false, std::memory_order_release);

            return PublishResult::Full;
        }

        return PublishResult::Full;
    }

    SubmitResult ProductionExecutorCore::trySubmit(
        const ControlRequest &request) noexcept {
        return m_controls.tryPush(request)
            ? SubmitResult::Submitted : SubmitResult::Full;
    }

    bool ProductionExecutorCore::tryTakeEvent(ExecutionEvent &event) noexcept {
        return m_events.tryPop(event);
    }

    bool ProductionExecutorCore::tryTakeSnapshot(
        ExecutionSnapshot &snapshot) noexcept {
        return m_snapshots.tryPop(snapshot);
    }

    void ProductionExecutorCore::restoreStationaryState(
        const MotionState &commanded, const MotionState &feedback) noexcept {
        discardExecution();

        ControlRequest control;
        while (m_controls.tryPop(control)) { }
        ExecutionEvent event;
        while (m_events.tryPop(event)) { }
        ExecutionSnapshot snapshot;
        while (m_snapshots.tryPop(snapshot)) { }

        m_snapshot = {};
        m_snapshot.state = BackendState::Disabled;
        m_snapshot.commanded = commanded;
        m_snapshot.feedback = feedback;
        m_faultEventEmitted = false;
    }

    void ProductionExecutorCore::setDigitalInputSample(
        const DigitalInputId input, const bool active) noexcept {
        m_digitalInputs[input] = active;
    }

    void ProductionExecutorCore::servoTick(const bool shouldPublishSnapshot) noexcept {
        serviceControls();

        if (m_snapshot.state == BackendState::Running
            || m_snapshot.state == BackendState::Holding) {
            if (!m_active.has_value()) {
                activateNext();
            }
            if (m_active.has_value()
                && (m_snapshot.state == BackendState::Running
                    || m_snapshot.state == BackendState::Holding)) {
                advanceActive(m_servoPeriod);
            }
        }

        m_previousDigitalInputs = m_digitalInputs;

        if (shouldPublishSnapshot || m_snapshot.state == BackendState::Held
            || m_snapshot.state == BackendState::Faulted
            || m_snapshot.state == BackendState::Disabled) {
            publishSnapshot();
        }
    }

    double ProductionExecutorCore::servoPeriod() const noexcept {
        return m_servoPeriod;
    }

    bool ProductionExecutorCore::validExecutionItem(
        const ExecutionItem &item) noexcept {
        if (const auto *chunk = std::get_if<PlanChunk>(&item)) {
            return validPlanChunk(*chunk);
        }
        if (const auto *move = std::get_if<TriggeredMove>(&item)) {
            return validTriggeredMove(*move);
        }
        if (const auto *move = std::get_if<TriggeredJointMove>(&item)) {
            return validTriggeredJointMove(*move);
        }

        return false;
    }

    bool ProductionExecutorCore::validPlanChunk(
        const PlanChunk &chunk) noexcept {
        if (chunk.epoch == 0 || chunk.id == 0 || chunk.branch == 0
            || chunk.normalMotion.size == 0 || chunk.stopTail.size == 0
            || chunk.events.size != 0
            || !finiteMotionState(chunk.branchState)
            || !finiteMotionState(chunk.stopState)) {
            return false;
        }

        const auto validSpan = [](const AxisPolynomialSpan &span) {
            if (span.id == 0
                || (span.degree != ExecutionPolynomialDegree::Cubic
                    && span.degree != ExecutionPolynomialDegree::Quintic)
                || !std::isfinite(span.duration) || span.duration <= 0.0
                || !finitePosition(span.origin)) {
                return false;
            }
            for (const auto &coefficient : span.coefficients) {
                if (!finitePosition(coefficient)) {
                    return false;
                }
            }

            const auto inverseDuration = 1.0 / span.duration;
            return std::isfinite(span.inverseDuration)
                && approximatelyEqual(
                    span.inverseDuration, inverseDuration)
                && std::isfinite(span.inverseDurationSquared)
                && approximatelyEqual(
                    span.inverseDurationSquared,
                    inverseDuration * inverseDuration)
                && std::isfinite(span.inverseDurationCubed)
                && approximatelyEqual(
                    span.inverseDurationCubed,
                    inverseDuration * inverseDuration * inverseDuration)
                && (span.degree != ExecutionPolynomialDegree::Cubic
                    || zeroHighCubicControls(span));
        };
        if (!std::ranges::all_of(chunk.normalMotion, validSpan)
            || !std::ranges::all_of(chunk.stopTail, validSpan)) {
            return false;
        }

        std::optional<std::pair<std::uint32_t, double>> previous;
        for (const auto &marker : chunk.markers) {
            const auto location = std::pair{marker.span, marker.parameter};
            if (marker.id == 0 || marker.span >= chunk.normalMotion.size
                || !std::isfinite(marker.parameter)
                || marker.parameter < 0.0 || marker.parameter > 1.0
                || (previous.has_value() && location < *previous)) {
                return false;
            }
            previous = location;
        }

        return true;
    }

    bool ProductionExecutorCore::validTriggeredMove(
        const TriggeredMove &move) noexcept {
        return move.epoch != 0 && move.id != 0 && move.branch != 0
            && move.moveId != 0 && finitePosition(move.target)
            && std::isfinite(magnitude(move.limits.velocity))
            && magnitude(move.limits.velocity) > 0.0
            && std::isfinite(magnitude(move.limits.acceleration))
            && magnitude(move.limits.acceleration) > 0.0
            && std::isfinite(magnitude(move.limits.jerk))
            && magnitude(move.limits.jerk) > 0.0
            && validInputCondition(move.condition);
    }

    bool ProductionExecutorCore::validTriggeredJointMove(
        const TriggeredJointMove &move) noexcept {
        constexpr auto validJointMask =
            static_cast<JointMask>((JointMask{1} << MAX_JOINTS) - 1);
        if (move.epoch == 0 || move.id == 0 || move.branch == 0
            || move.moveId == 0 || move.joints == 0
            || (move.joints & ~validJointMask) != 0
            || (move.targetMode != JointTargetMode::Absolute
                && move.targetMode != JointTargetMode::Relative)) {
            return false;
        }

        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            const auto mask = static_cast<JointMask>(JointMask{1} << joint);
            if ((move.joints & mask) == 0) {
                continue;
            }
            if (!std::isfinite(move.target[joint])
                || !std::isfinite(move.limits.velocity[joint])
                || move.limits.velocity[joint] <= 0.0
                || !std::isfinite(move.limits.acceleration[joint])
                || move.limits.acceleration[joint] <= 0.0
                || !std::isfinite(move.limits.jerk[joint])
                || move.limits.jerk[joint] <= 0.0) {
                return false;
            }
        }

        JointMask triggeredJoints = 0;
        for (const auto &trigger : move.triggers) {
            if (trigger.joint >= MAX_JOINTS
                || !validInputCondition(trigger.condition)
                || !std::isfinite(trigger.debounce)
                || trigger.debounce < 0.0) {
                return false;
            }

            const auto mask =
                static_cast<JointMask>(JointMask{1} << trigger.joint);
            if ((move.joints & mask) == 0
                || (triggeredJoints & mask) != 0) {
                return false;
            }
            triggeredJoints |= mask;
        }

        return !move.triggerRequired || triggeredJoints == move.joints;
    }

    std::uint64_t ProductionExecutorCore::normalMotionNanoseconds(
        const ExecutionItem &item) noexcept {
        const auto *chunk = std::get_if<PlanChunk>(&item);
        if (chunk == nullptr) {
            return 0;
        }

        auto seconds = 0.0;
        for (const auto &span : chunk->normalMotion) {
            seconds += std::max(span.duration, 0.0);
        }

        return secondsToNanoseconds(seconds);
    }

    EpochId ProductionExecutorCore::itemEpoch(
        const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.epoch;
        }, item);
    }

    ChunkId ProductionExecutorCore::itemId(
        const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.id;
        }, item);
    }

    BranchSequence ProductionExecutorCore::itemPredecessor(
        const ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.predecessorBranch;
        }, item);
    }

    std::uint64_t ProductionExecutorCore::secondsToNanoseconds(
        const double seconds) noexcept {
        constexpr auto scale = 1.0e9;
        if (!std::isfinite(seconds) || seconds <= 0.0) {
            return 0;
        }

        const auto maximum =
            static_cast<double>(std::numeric_limits<std::uint64_t>::max());
        if (seconds >= maximum / scale) {
            return std::numeric_limits<std::uint64_t>::max();
        }

        return static_cast<std::uint64_t>(std::llround(seconds * scale));
    }

    void ProductionExecutorCore::serviceControls() noexcept {
        ControlRequest request;
        while (m_controls.tryPop(request)) {
            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                auto success = false;
                if constexpr (std::same_as<T, EnableRequest>) {
                    success = m_snapshot.state == BackendState::Disabled
                        || m_snapshot.state == BackendState::Held;
                    if (success) {
                        m_snapshot.state = BackendState::Held;
                        m_snapshot.faultCode = 0;
                        m_faultEventEmitted = false;
                    }
                } else if constexpr (std::same_as<T, DisableRequest>) {
                    discardExecution();
                    m_snapshot.state = BackendState::Disabled;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    success = true;
                } else if constexpr (std::same_as<T, StartRequest>) {
                    success = m_snapshot.state == BackendState::Held
                        && value.epoch != 0 && value.epoch == m_snapshot.epoch;
                    if (success) {
                        m_snapshot.state = BackendState::Running;
                    }
                } else if constexpr (std::same_as<T, ResumeRequest>) {
                    success = m_snapshot.state == BackendState::Held
                        && !m_active.has_value() && value.epoch != 0
                        && value.epoch == m_snapshot.epoch
                        && m_queuedExecutionItems.load(
                            std::memory_order_acquire) != 0;
                    if (success) {
                        m_snapshot.state = BackendState::Running;
                    }
                } else if constexpr (std::same_as<T, ControlledStopRequest>) {
                    success = m_snapshot.state == BackendState::Running
                        && m_active.has_value();
                    if (success
                        && std::holds_alternative<TriggeredMove>(
                            m_planSlots[*m_active].item)) {
                        if (m_triggered.stopping) {
                            success = false;
                        } else if (!beginTriggeredStop(
                                       TriggeredMoveStatus::Aborted)) {
                            success = false;
                            faultTriggered();
                        }
                    } else if (success
                               && std::holds_alternative<TriggeredJointMove>(
                                   m_planSlots[*m_active].item)) {
                        const auto &move = activeTriggeredJointMove();
                        m_triggeredJointCompletionStatus =
                            TriggeredMoveStatus::Aborted;
                        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                            const auto mask =
                                static_cast<JointMask>(JointMask{1} << joint);
                            auto &runtime = m_triggeredJoints[joint];
                            if ((move.joints & mask) == 0 || runtime.finished
                                || runtime.stopping) {
                                continue;
                            }
                            if (!beginTriggeredJointStop(
                                    move, joint, false)) {
                                success = false;
                                faultTriggered();
                                break;
                            }
                        }
                    } else {
                        success = false;
                    }
                    if (success) {
                        m_snapshot.state = BackendState::Holding;
                    }
                } else if constexpr (std::same_as<T, AbortRequest>) {
                    discardExecution();
                    m_snapshot.state = BackendState::Held;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    success = true;
                } else if constexpr (std::same_as<T, ResetRequest>) {
                    discardExecution();
                    const auto commanded = m_snapshot.commanded;
                    const auto feedback = m_snapshot.feedback;
                    const auto commandedJoints = m_snapshot.commandedJoints;
                    const auto feedbackJoints = m_snapshot.feedbackJoints;
                    m_snapshot = {};
                    m_snapshot.epoch = value.nextEpoch;
                    m_snapshot.commanded = commanded;
                    m_snapshot.feedback = feedback;
                    m_snapshot.commandedJoints = commandedJoints;
                    m_snapshot.feedbackJoints = feedbackJoints;
                    m_faultEventEmitted = false;
                    success = value.nextEpoch != 0;
                } else if constexpr (std::same_as<T, SetJointPositionRequest>) {
                    constexpr auto validJointMask =
                        static_cast<JointMask>(
                            (JointMask{1} << MAX_JOINTS) - 1);
                    success = m_snapshot.state == BackendState::Held
                        && !m_active.has_value() && value.joints != 0
                        && (value.joints & ~validJointMask) == 0;
                    for (JointId joint = 0;
                         success && joint < MAX_JOINTS; ++joint) {
                        const auto mask =
                            static_cast<JointMask>(JointMask{1} << joint);
                        if ((value.joints & mask) != 0
                            && !std::isfinite(value.position[joint])) {
                            success = false;
                        }
                    }
                    if (success) {
                        for (JointId joint = 0;
                             joint < MAX_JOINTS; ++joint) {
                            const auto mask =
                                static_cast<JointMask>(
                                    JointMask{1} << joint);
                            if ((value.joints & mask) == 0) {
                                continue;
                            }
                            m_snapshot.commandedJoints.position[joint] =
                                value.position[joint];
                            m_snapshot.commandedJoints.velocity[joint] = 0.0;
                            m_snapshot.commandedJoints.acceleration[joint] =
                                0.0;
                        }
                        m_snapshot.feedbackJoints =
                            m_snapshot.commandedJoints;
                    }
                }

                emit(RequestCompleted{value.id, success});
            }, request);
        }
    }

    void ProductionExecutorCore::activateNext() noexcept {
        std::uint8_t index = 0;
        if (!m_plans.tryPop(index)) {
            return;
        }
        accountForDequeued(index);

        const auto &item = m_planSlots[index].item;
        if (itemEpoch(item) != m_snapshot.epoch) {
            emit(ChunkRejected{itemEpoch(item), itemId(item)});
            release(index);

            return;
        }

        m_active = index;
        m_stopping = false;
        m_span = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
        m_triggered = {};
        m_triggeredJoints = {};
        m_triggeredJointMask = 0;
        m_triggeredJointCompletionStatus =
            TriggeredMoveStatus::ReachedTarget;
        m_snapshot.activeChunk = itemId(item);
        m_snapshot.activeSpan = 0;
        emit(ChunkAccepted{itemEpoch(item), itemId(item)});
        if (std::holds_alternative<PlanChunk>(item)) {
            emitExecutionMarkersThrough(0.0);
        } else if (std::holds_alternative<TriggeredMove>(item)
                   && !initializeTriggered()) {
            faultTriggered();
        } else if (std::holds_alternative<TriggeredJointMove>(item)
                   && !initializeTriggeredJoints()) {
            faultTriggered();
        }
    }

    void ProductionExecutorCore::advanceActive(double seconds) noexcept {
        while (m_active.has_value() && seconds > 0.0
               && (m_snapshot.state == BackendState::Running
                   || m_snapshot.state == BackendState::Holding)) {
            const auto &item = m_planSlots[*m_active].item;
            if (std::holds_alternative<PlanChunk>(item)) {
                advancePlan(seconds);
            } else if (std::holds_alternative<TriggeredMove>(item)) {
                advanceTriggered(seconds);
            } else if (std::holds_alternative<TriggeredJointMove>(item)) {
                advanceTriggeredJoints(seconds);
            } else {
                fault(INVALID_EXECUTION_STATE_FAULT);
            }
        }
    }

    void ProductionExecutorCore::advancePlan(double &seconds) noexcept {
        while (m_active.has_value() && seconds > 0.0
               && m_snapshot.state == BackendState::Running
               && std::holds_alternative<PlanChunk>(
                   m_planSlots[*m_active].item)) {
            const auto &span = currentSpan();
            const auto remaining =
                std::max(span.duration - m_spanElapsed, 0.0);
            const auto consumed = std::min(seconds, remaining);
            seconds -= consumed;
            m_spanElapsed += consumed;

            const auto parameter = std::clamp(
                m_spanElapsed * span.inverseDuration, 0.0, 1.0);
            m_snapshot.activeSpan = span.id;
            m_snapshot.spanProgress = parameter;
            m_snapshot.commanded =
                evaluateExecutionPolynomial(span, parameter).state;
            m_snapshot.feedback = m_snapshot.commanded;
            emitExecutionMarkersThrough(parameter);
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }

            if (m_spanElapsed + 1e-12 < span.duration) {
                return;
            }
            completeSpan();
        }
    }

    bool ProductionExecutorCore::initializeTriggered() noexcept {
        const auto &move = activeTriggeredMove();
        m_triggered = {};
        m_triggered.start = m_snapshot.commanded.position;
        const auto delta = move.target - m_triggered.start;
        m_triggered.length = magnitude(delta);
        if (m_triggered.length <= 1e-12) {
            return true;
        }
        m_triggered.direction = scaled(delta, 1.0 / m_triggered.length);

        ruckig::InputParameter<1> input;
        input.current_position = {0.0};
        input.current_velocity = {
            dot(m_snapshot.commanded.velocity, m_triggered.direction),
        };
        input.current_acceleration = {
            dot(m_snapshot.commanded.acceleration, m_triggered.direction),
        };
        input.target_position = {m_triggered.length};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {magnitude(move.limits.velocity)};
        input.max_acceleration = {magnitude(move.limits.acceleration)};
        input.max_jerk = {magnitude(move.limits.jerk)};
        ruckig::Ruckig<1> generator;

        return generator.calculate(input, m_triggered.trajectory)
            == ruckig::Result::Working;
    }

    bool ProductionExecutorCore::beginTriggeredStop(
        const TriggeredMoveStatus status) noexcept {
        const auto &move = activeTriggeredMove();
        m_triggered.stopOrigin = m_snapshot.commanded;
        m_triggered.triggerState = m_snapshot.commanded;
        m_triggered.completionStatus = status;
        m_triggered.stopping = true;
        m_triggered.elapsed = 0.0;
        const auto scalarVelocity =
            dot(m_snapshot.commanded.velocity, m_triggered.direction);
        const auto scalarAcceleration =
            dot(m_snapshot.commanded.acceleration, m_triggered.direction);
        if (std::abs(scalarVelocity) <= 1e-12
            && std::abs(scalarAcceleration) <= 1e-12) {
            return true;
        }

        ruckig::InputParameter<1> input;
        input.control_interface = ruckig::ControlInterface::Velocity;
        input.current_position = {0.0};
        input.current_velocity = {scalarVelocity};
        input.current_acceleration = {scalarAcceleration};
        input.target_position = {0.0};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {
            std::max(magnitude(move.limits.velocity),
                     std::abs(scalarVelocity)),
        };
        input.max_acceleration = {magnitude(move.limits.acceleration)};
        input.max_jerk = {magnitude(move.limits.jerk)};
        ruckig::Ruckig<1> generator;

        return generator.calculate(input, m_triggered.trajectory)
            == ruckig::Result::Working;
    }

    MotionState ProductionExecutorCore::triggeredStateAt(
        const double elapsed, const position_t &origin) const noexcept {
        double position = 0.0;
        double velocity = 0.0;
        double acceleration = 0.0;
        m_triggered.trajectory.at_time(
            elapsed, position, velocity, acceleration);

        return {
            origin + scaled(m_triggered.direction, position),
            scaled(m_triggered.direction, velocity),
            scaled(m_triggered.direction, acceleration),
        };
    }

    bool ProductionExecutorCore::triggeredInputConditionMet(
        const TriggeredMove &move) const noexcept {
        const auto current = m_digitalInputs[move.input];
        const auto previous = m_previousDigitalInputs[move.input];
        switch (move.condition) {
            case InputCondition::Active: return current;
            case InputCondition::Inactive: return !current;
            case InputCondition::RisingEdge: return current && !previous;
            case InputCondition::FallingEdge: return !current && previous;
        }

        return false;
    }

    void ProductionExecutorCore::advanceTriggered(double &seconds) noexcept {
        const auto move = activeTriggeredMove();
        if (!m_triggered.stopping && triggeredInputConditionMet(move)) {
            if (!beginTriggeredStop(TriggeredMoveStatus::Triggered)) {
                faultTriggered();

                return;
            }
            if (m_triggered.trajectory.get_duration() <= 1e-12) {
                completeTriggered(TriggeredMoveStatus::Triggered);

                return;
            }
        }

        if (m_triggered.length <= 1e-12 && !m_triggered.stopping) {
            m_snapshot.commanded.position = move.target;
            m_snapshot.commanded.velocity = {};
            m_snapshot.commanded.acceleration = {};
            m_snapshot.feedback = m_snapshot.commanded;
            completeTriggered(TriggeredMoveStatus::ReachedTarget);

            return;
        }

        const auto duration = m_triggered.trajectory.get_duration();
        const auto consumed =
            std::min(seconds, std::max(duration - m_triggered.elapsed, 0.0));
        m_triggered.elapsed += consumed;
        const auto origin = m_triggered.stopping
            ? m_triggered.stopOrigin.position : m_triggered.start;
        m_snapshot.commanded = triggeredStateAt(m_triggered.elapsed, origin);
        m_snapshot.feedback = m_snapshot.commanded;
        m_snapshot.spanProgress = duration > 0.0
            ? std::clamp(m_triggered.elapsed / duration, 0.0, 1.0) : 1.0;
        seconds -= consumed;

        if (m_triggered.elapsed + 1e-12 < duration) {
            return;
        }
        if (m_triggered.stopping) {
            m_snapshot.commanded.velocity = {};
            m_snapshot.commanded.acceleration = {};
            m_snapshot.feedback = m_snapshot.commanded;
            completeTriggered(m_triggered.completionStatus);

            return;
        }

        m_snapshot.commanded.position = move.target;
        m_snapshot.commanded.velocity = {};
        m_snapshot.commanded.acceleration = {};
        m_snapshot.feedback = m_snapshot.commanded;
        completeTriggered(TriggeredMoveStatus::ReachedTarget);
    }

    void ProductionExecutorCore::completeTriggered(
        const TriggeredMoveStatus status) noexcept {
        const auto move = activeTriggeredMove();
        const auto stopped = m_snapshot.commanded;
        const auto trigger = status == TriggeredMoveStatus::Triggered
            ? m_triggered.triggerState : stopped;
        emit(TriggeredMoveCompleted{
            move.epoch, move.moveId, status, trigger, stopped,
        });
        emit(BranchSelected{
            move.epoch, move.branch, BranchChoice::Stop, 0,
        });
        emit(ChunkRetired{move.epoch, move.id});
        m_snapshot.state = BackendState::Held;
        m_snapshot.lastBranch = move.branch;
        emit(BackendHeld{
            move.epoch, stopped,
            status == TriggeredMoveStatus::Aborted
                ? BackendHoldReason::ControlledStop
                : BackendHoldReason::StopBranch,
        });
        release(*m_active);
        m_active.reset();
    }

    void ProductionExecutorCore::faultTriggered() noexcept {
        fault(INVALID_EXECUTION_STATE_FAULT);
    }

    bool ProductionExecutorCore::initializeTriggeredJoints() noexcept {
        const auto &move = activeTriggeredJointMove();
        m_triggeredJoints = {};
        m_triggeredJointMask = 0;
        m_triggeredJointCompletionStatus =
            TriggeredMoveStatus::ReachedTarget;
        m_snapshot.activeJoints = move.joints;
        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            const auto mask = static_cast<JointMask>(JointMask{1} << joint);
            if ((move.joints & mask) != 0
                && !initializeTriggeredJoint(move, joint)) {
                return false;
            }
        }

        return true;
    }

    bool ProductionExecutorCore::initializeTriggeredJoint(
        const TriggeredJointMove &move, const JointId joint) noexcept {
        auto &runtime = m_triggeredJoints[joint];
        runtime = {};
        const auto &state = m_snapshot.commandedJoints;
        runtime.target = move.targetMode == JointTargetMode::Relative
            ? state.position[joint] + move.target[joint] : move.target[joint];
        if (std::abs(runtime.target - state.position[joint]) <= 1e-12
            && std::abs(state.velocity[joint]) <= 1e-12
            && std::abs(state.acceleration[joint]) <= 1e-12) {
            runtime.finished = true;

            return true;
        }

        ruckig::InputParameter<1> input;
        input.current_position = {state.position[joint]};
        input.current_velocity = {state.velocity[joint]};
        input.current_acceleration = {state.acceleration[joint]};
        input.target_position = {runtime.target};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {move.limits.velocity[joint]};
        input.max_acceleration = {move.limits.acceleration[joint]};
        input.max_jerk = {move.limits.jerk[joint]};
        ruckig::Ruckig<1> generator;

        return generator.calculate(input, runtime.trajectory)
            == ruckig::Result::Working;
    }

    bool ProductionExecutorCore::beginTriggeredJointStop(
        const TriggeredJointMove &move, const JointId joint,
        const bool triggered) noexcept {
        auto &runtime = m_triggeredJoints[joint];
        const auto &state = m_snapshot.commandedJoints;
        runtime.stopping = true;
        runtime.triggerPosition = state.position[joint];
        runtime.triggerVelocity = state.velocity[joint];
        runtime.triggerAcceleration = state.acceleration[joint];
        runtime.elapsed = 0.0;
        if (triggered) {
            m_triggeredJointMask |=
                static_cast<JointMask>(JointMask{1} << joint);
        }
        if (std::abs(runtime.triggerVelocity) <= 1e-12
            && std::abs(runtime.triggerAcceleration) <= 1e-12) {
            runtime.finished = true;

            return true;
        }

        ruckig::InputParameter<1> input;
        input.control_interface = ruckig::ControlInterface::Velocity;
        input.current_position = {runtime.triggerPosition};
        input.current_velocity = {runtime.triggerVelocity};
        input.current_acceleration = {runtime.triggerAcceleration};
        input.target_position = {runtime.triggerPosition};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {
            std::max(move.limits.velocity[joint],
                     std::abs(runtime.triggerVelocity)),
        };
        input.max_acceleration = {move.limits.acceleration[joint]};
        input.max_jerk = {move.limits.jerk[joint]};
        ruckig::Ruckig<1> generator;

        return generator.calculate(input, runtime.trajectory)
            == ruckig::Result::Working;
    }

    bool ProductionExecutorCore::triggeredJointInputQualified(
        const JointTrigger &trigger,
        TriggeredJointRuntime &runtime) noexcept {
        const auto current =
            static_cast<bool>(m_digitalInputs[trigger.input]);
        const auto previous =
            static_cast<bool>(m_previousDigitalInputs[trigger.input]);
        const auto edgeCondition =
            trigger.condition == InputCondition::RisingEdge
            || trigger.condition == InputCondition::FallingEdge;
        const auto conditionStarted = [&] {
            switch (trigger.condition) {
                case InputCondition::Active: return current;
                case InputCondition::Inactive: return !current;
                case InputCondition::RisingEdge: return current && !previous;
                case InputCondition::FallingEdge: return !current && previous;
            }

            return false;
        }();
        const auto resultingLevel = [&] {
            switch (trigger.condition) {
                case InputCondition::Active:
                case InputCondition::RisingEdge: return current;
                case InputCondition::Inactive:
                case InputCondition::FallingEdge: return !current;
            }

            return false;
        }();

        if (edgeCondition && conditionStarted) {
            runtime.triggerPending = true;
            runtime.debounceElapsed = 0.0;
        } else if (!edgeCondition) {
            runtime.triggerPending = conditionStarted;
        }
        if (!runtime.triggerPending || !resultingLevel) {
            runtime.triggerPending = false;
            runtime.debounceElapsed = 0.0;

            return false;
        }

        runtime.debounceElapsed += m_servoPeriod;

        return runtime.debounceElapsed + 1e-12 >= trigger.debounce;
    }

    void ProductionExecutorCore::advanceTriggeredJoints(
        double &seconds) noexcept {
        const auto move = activeTriggeredJointMove();
        for (const auto &trigger : move.triggers) {
            auto &runtime = m_triggeredJoints[trigger.joint];
            if (!runtime.finished && !runtime.stopping
                && triggeredJointInputQualified(trigger, runtime)
                && !beginTriggeredJointStop(
                    move, trigger.joint, true)) {
                faultTriggered();

                return;
            }
        }

        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            const auto mask = static_cast<JointMask>(JointMask{1} << joint);
            if ((move.joints & mask) == 0) {
                continue;
            }

            auto &runtime = m_triggeredJoints[joint];
            if (runtime.finished) {
                continue;
            }

            const auto duration = runtime.trajectory.get_duration();
            const auto consumed =
                std::min(seconds, std::max(duration - runtime.elapsed, 0.0));
            runtime.elapsed += consumed;
            double position = 0.0;
            double velocity = 0.0;
            double acceleration = 0.0;
            runtime.trajectory.at_time(
                runtime.elapsed, position, velocity, acceleration);
            m_snapshot.commandedJoints.position[joint] = position;
            m_snapshot.commandedJoints.velocity[joint] = velocity;
            m_snapshot.commandedJoints.acceleration[joint] = acceleration;
            if (runtime.elapsed + 1e-12 < duration) {
                continue;
            }

            if (!runtime.stopping) {
                m_snapshot.commandedJoints.position[joint] = runtime.target;
            }
            m_snapshot.commandedJoints.velocity[joint] = 0.0;
            m_snapshot.commandedJoints.acceleration[joint] = 0.0;
            runtime.finished = true;
        }
        m_snapshot.feedbackJoints = m_snapshot.commandedJoints;
        seconds = 0.0;

        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            const auto mask = static_cast<JointMask>(JointMask{1} << joint);
            if ((move.joints & mask) != 0
                && !m_triggeredJoints[joint].finished) {
                return;
            }
        }
        completeTriggeredJoints();
    }

    void ProductionExecutorCore::completeTriggeredJoints() noexcept {
        const auto move = activeTriggeredJointMove();
        auto triggerState = m_snapshot.commandedJoints;
        JointMask expectedTriggers = 0;
        for (const auto &trigger : move.triggers) {
            const auto mask =
                static_cast<JointMask>(JointMask{1} << trigger.joint);
            expectedTriggers |= mask;
            if ((m_triggeredJointMask & mask) == 0) {
                continue;
            }

            const auto &runtime = m_triggeredJoints[trigger.joint];
            triggerState.position[trigger.joint] = runtime.triggerPosition;
            triggerState.velocity[trigger.joint] = runtime.triggerVelocity;
            triggerState.acceleration[trigger.joint] =
                runtime.triggerAcceleration;
        }
        const auto status =
            m_triggeredJointCompletionStatus
                == TriggeredMoveStatus::Aborted
            ? TriggeredMoveStatus::Aborted
            : expectedTriggers != 0
                && (m_triggeredJointMask & expectedTriggers)
                    == expectedTriggers
                ? TriggeredMoveStatus::Triggered
                : TriggeredMoveStatus::ReachedTarget;
        emit(TriggeredJointMoveCompleted{
            move.epoch, move.moveId, status, m_triggeredJointMask,
            triggerState, m_snapshot.commandedJoints,
        });
        emit(BranchSelected{
            move.epoch, move.branch, BranchChoice::Stop, 0,
        });
        emit(ChunkRetired{move.epoch, move.id});
        m_snapshot.state = BackendState::Held;
        m_snapshot.lastBranch = move.branch;
        m_snapshot.activeJoints = 0;
        emit(BackendHeld{
            move.epoch, m_snapshot.commanded,
            status == TriggeredMoveStatus::Aborted
                ? BackendHoldReason::ControlledStop
                : BackendHoldReason::StopBranch,
        });
        release(*m_active);
        m_active.reset();
    }

    void ProductionExecutorCore::completeSpan() noexcept {
        m_spanElapsed = 0.0;
        ++m_span;
        const auto count = m_stopping
            ? activeChunk().stopTail.size : activeChunk().normalMotion.size;
        if (m_span < count) {
            emitExecutionMarkersThrough(0.0);

            return;
        }

        if (!m_stopping) {
            selectContinuationOrStop();

            return;
        }

        const auto chunk = activeChunk();
        m_snapshot.commanded = chunk.stopState;
        m_snapshot.feedback = m_snapshot.commanded;
        m_snapshot.lastBranch = chunk.branch;
        m_snapshot.state = BackendState::Held;
        emit(ChunkRetired{chunk.epoch, chunk.id});
        emit(BackendHeld{
            chunk.epoch, chunk.stopState, BackendHoldReason::StopBranch,
        });
        release(*m_active);
        m_active.reset();
    }

    void ProductionExecutorCore::selectContinuationOrStop() noexcept {
        std::uint8_t continuationIndex = 0;
        const auto hasContinuation = m_plans.tryPop(continuationIndex);
        if (hasContinuation) {
            accountForDequeued(continuationIndex);
        }

        const auto currentIndex = *m_active;
        const auto current = activeChunk();
        if (hasContinuation) {
            const auto &continuation = m_planSlots[continuationIndex].item;
            if (itemEpoch(continuation) == current.epoch
                && itemPredecessor(continuation) == current.branch) {
                emit(BranchSelected{
                    current.epoch, current.branch, BranchChoice::Continue,
                    itemId(continuation),
                });
                emit(ChunkRetired{current.epoch, current.id});
                m_active = continuationIndex;
                release(currentIndex);
                m_stopping = false;
                m_span = 0;
                m_nextMarker = 0;
                m_spanElapsed = 0.0;
                m_snapshot.activeChunk = itemId(continuation);
                m_snapshot.activeSpan = 0;
                emit(ChunkAccepted{
                    itemEpoch(continuation), itemId(continuation),
                });
                if (std::holds_alternative<PlanChunk>(continuation)) {
                    emitExecutionMarkersThrough(0.0);
                } else if (std::holds_alternative<TriggeredMove>(continuation)
                           && !initializeTriggered()) {
                    faultTriggered();
                } else if (std::holds_alternative<TriggeredJointMove>(
                               continuation)
                           && !initializeTriggeredJoints()) {
                    faultTriggered();
                }

                return;
            }

            emit(ChunkRejected{
                itemEpoch(continuation), itemId(continuation),
            });
            release(continuationIndex);
        }

        emit(BranchSelected{
            current.epoch, current.branch, BranchChoice::Stop, 0,
        });
        m_stopping = true;
        m_span = 0;
        m_spanElapsed = 0.0;
    }

    void ProductionExecutorCore::emitExecutionMarkersThrough(
        const double parameter) noexcept {
        if (m_stopping || !m_active.has_value()) {
            return;
        }

        const auto &chunk = activeChunk();
        while (m_nextMarker < chunk.markers.size) {
            const auto &marker = chunk.markers[m_nextMarker];
            if (marker.span > m_span
                || (marker.span == m_span
                    && marker.parameter > parameter)) {
                return;
            }
            if (marker.span < m_span
                || marker.span >= chunk.normalMotion.size) {
                fault(INVALID_EXECUTION_STATE_FAULT);

                return;
            }

            emit(ExecutionMarkerReached{
                .epoch = chunk.epoch,
                .chunk = chunk.id,
                .marker = marker.id,
                .span = chunk.normalMotion[m_span].id,
                .parameter = marker.parameter,
            });
            ++m_nextMarker;
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }
        }
    }

    void ProductionExecutorCore::emit(
        const ExecutionEvent &event) noexcept {
        if (!m_events.tryPush(event)) {
            fault(EVENT_OVERFLOW_FAULT);
        }
    }

    void ProductionExecutorCore::fault(const std::uint32_t code) noexcept {
        m_snapshot.state = BackendState::Faulted;
        m_snapshot.faultCode = code;
        if (!m_faultEventEmitted) {
            m_faultEventEmitted = m_events.tryPush(BackendFault{code});
        }
    }

    void ProductionExecutorCore::discardExecution() noexcept {
        if (m_active.has_value()) {
            release(*m_active);
            m_active.reset();
        }

        std::uint8_t index = 0;
        while (m_plans.tryPop(index)) {
            accountForDequeued(index);
            release(index);
        }
        m_stopping = false;
        m_span = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
        m_triggered = {};
        m_triggeredJoints = {};
        m_triggeredJointMask = 0;
        m_triggeredJointCompletionStatus =
            TriggeredMoveStatus::ReachedTarget;
        m_snapshot.activeJoints = 0;
    }

    void ProductionExecutorCore::release(const std::uint8_t index) noexcept {
        m_planSlots[index].occupied.store(false, std::memory_order_release);
    }

    void ProductionExecutorCore::accountForDequeued(
        const std::uint8_t index) noexcept {
        m_queuedNormalMotionNanoseconds.fetch_sub(
            m_planSlots[index].normalMotionNanoseconds,
            std::memory_order_acq_rel);
        m_queuedExecutionItems.fetch_sub(1, std::memory_order_acq_rel);
    }

    void ProductionExecutorCore::publishSnapshot() noexcept {
        refreshSnapshot();
        static_cast<void>(m_snapshots.tryPush(m_snapshot));
    }

    void ProductionExecutorCore::refreshSnapshot() noexcept {
        constexpr auto nanosecondsToSeconds = 1.0e-9;
        m_snapshot.activeNormalMotionRemainingSeconds = 0.0;
        m_snapshot.stopBranchRemainingSeconds = 0.0;
        m_snapshot.queuedNormalMotionSeconds =
            nanosecondsToSeconds * static_cast<double>(
                m_queuedNormalMotionNanoseconds.load(
                    std::memory_order_acquire));
        m_snapshot.queuedExecutionItems =
            m_queuedExecutionItems.load(std::memory_order_acquire);
        if (m_active.has_value()
            && std::holds_alternative<PlanChunk>(
                m_planSlots[*m_active].item)) {
            const auto &chunk = activeChunk();
            if (m_stopping) {
                m_snapshot.stopBranchRemainingSeconds =
                    remainingDuration(chunk.stopTail, m_span, m_spanElapsed);
            } else {
                m_snapshot.activeNormalMotionRemainingSeconds =
                    remainingDuration(
                        chunk.normalMotion, m_span, m_spanElapsed);
                m_snapshot.stopBranchRemainingSeconds =
                    remainingDuration(chunk.stopTail, 0, 0.0);
            }
        }
        m_snapshot.committedNormalMotionSeconds =
            m_snapshot.state == BackendState::Running && !m_stopping
            ? m_snapshot.activeNormalMotionRemainingSeconds
                + m_snapshot.queuedNormalMotionSeconds
            : 0.0;
    }

    PlanChunk &ProductionExecutorCore::activeChunk() noexcept {
        return std::get<PlanChunk>(m_planSlots[*m_active].item);
    }

    const PlanChunk &ProductionExecutorCore::activeChunk() const noexcept {
        return std::get<PlanChunk>(m_planSlots[*m_active].item);
    }

    TriggeredMove &ProductionExecutorCore::activeTriggeredMove() noexcept {
        return std::get<TriggeredMove>(m_planSlots[*m_active].item);
    }

    const TriggeredMove &
    ProductionExecutorCore::activeTriggeredMove() const noexcept {
        return std::get<TriggeredMove>(m_planSlots[*m_active].item);
    }

    TriggeredJointMove &
    ProductionExecutorCore::activeTriggeredJointMove() noexcept {
        return std::get<TriggeredJointMove>(
            m_planSlots[*m_active].item);
    }

    const TriggeredJointMove &
    ProductionExecutorCore::activeTriggeredJointMove() const noexcept {
        return std::get<TriggeredJointMove>(
            m_planSlots[*m_active].item);
    }

    const AxisPolynomialSpan &
    ProductionExecutorCore::currentSpan() const noexcept {
        const auto &chunk = activeChunk();

        return m_stopping
            ? chunk.stopTail[m_span] : chunk.normalMotion[m_span];
    }
}
