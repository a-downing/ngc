#include "machine/ProductionExecutorCore.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "ExecutionItemOperations.h"

namespace ngc {
    namespace {
        constexpr std::uint32_t EVENT_OVERFLOW_FAULT = 1;
        constexpr std::uint32_t INVALID_EXECUTION_STATE_FAULT = 2;
        constexpr std::uint32_t JOG_GENERATION_FAULT = 3;
        constexpr std::uint32_t FEED_RETIMING_FAULT = 4;
        constexpr std::uint32_t FEED_RETIMING_STOP_BRANCH_FAULT = 5;
        constexpr std::uint32_t PLAN_START_DISCONTINUITY_FAULT = 6;
        constexpr double DYNAMIC_LIMIT_TOLERANCE = 1.01;

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

        double &axisComponent(position_t &position,
                              const AxisId axis) noexcept {
            switch (axis) {
                case AxisId::X: return position.x;
                case AxisId::Y: return position.y;
                case AxisId::Z: return position.z;
                case AxisId::A: return position.a;
                case AxisId::B: return position.b;
                case AxisId::C: return position.c;
            }

            return position.x;
        }

        double axisComponent(const position_t &position,
                             const AxisId axis) noexcept {
            auto copy = position;

            return axisComponent(copy, axis);
        }

        std::array<double, 6> axisValues(
            const position_t &value) noexcept {
            return {
                value.x, value.y, value.z,
                value.a, value.b, value.c,
            };
        }

        bool samePosition(const position_t &left, const position_t &right) noexcept {
            return (left - right).length() <= 1e-9;
        }

        position_t axisPosition(
            const std::array<double, 6> &value) noexcept {
            return {
                value[0], value[1], value[2],
                value[3], value[4], value[5],
            };
        }

        bool validPositiveLimit(const double value) noexcept {
            return !std::isnan(value) && value > 0.0;
        }

        bool validAxisLimits(const position_t &limits) noexcept {
            return validPositiveLimit(limits.x)
                && validPositiveLimit(limits.y)
                && validPositiveLimit(limits.z)
                && validPositiveLimit(limits.a)
                && validPositiveLimit(limits.b)
                && validPositiveLimit(limits.c);
        }

        bool constrainLinearInterval(
            const position_t &base, const position_t &direction,
            const position_t &axisLimits, const double aggregateLimit,
            double &lower, double &upper) noexcept {
            const auto constrainAxis = [&](const double offset,
                                           const double slope,
                                           const double limit) {
                if (std::isinf(limit)) {
                    return true;
                }
                const auto tolerated = limit * DYNAMIC_LIMIT_TOLERANCE;
                if (std::abs(slope) <= 1e-12) {
                    return std::abs(offset) <= tolerated + 1e-12;
                }

                auto first = (-tolerated - offset) / slope;
                auto second = (tolerated - offset) / slope;
                if (first > second) {
                    std::swap(first, second);
                }
                lower = std::max(lower, first);
                upper = std::min(upper, second);

                return lower <= upper + 1e-12;
            };

            if (!constrainAxis(base.x, direction.x, axisLimits.x)
                || !constrainAxis(base.y, direction.y, axisLimits.y)
                || !constrainAxis(base.z, direction.z, axisLimits.z)
                || !constrainAxis(base.a, direction.a, axisLimits.a)
                || !constrainAxis(base.b, direction.b, axisLimits.b)
                || !constrainAxis(base.c, direction.c, axisLimits.c)) {
                return false;
            }
            if (std::isinf(aggregateLimit)) {
                return true;
            }

            const auto tolerated =
                aggregateLimit * DYNAMIC_LIMIT_TOLERANCE;
            const auto quadratic = dot(direction, direction);
            const auto linear = 2.0 * dot(base, direction);
            const auto constant =
                dot(base, base) - tolerated * tolerated;
            if (quadratic <= 1e-24) {
                return constant <= 1e-12;
            }

            const auto discriminant =
                linear * linear - 4.0 * quadratic * constant;
            if (discriminant < -1e-12) {
                return false;
            }

            const auto root = std::sqrt(std::max(discriminant, 0.0));
            lower = std::max(
                lower, (-linear - root) / (2.0 * quadratic));
            upper = std::min(
                upper, (-linear + root) / (2.0 * quadratic));

            return lower <= upper + 1e-12;
        }

    }

    ProductionExecutorCore::ProductionExecutorCore(
        const double servoPeriod,
        ProductionExecutorConfiguration configuration)
        : m_configuration(std::move(configuration)),
          m_servoPeriod(servoPeriod) {
        if (!std::isfinite(servoPeriod) || servoPeriod <= 0.0) {
            throw std::invalid_argument(
                "production executor servo period must be finite and positive");
        }
        if (m_configuration.maximumJogLeaseTicks == 0) {
            throw std::invalid_argument(
                "production executor maximum jog lease must be positive");
        }
        const auto &feedHold = m_configuration.feedHold;
        const auto hasFeedHoldAcceleration =
            feedHold.tangentialAcceleration != 0.0;
        const auto hasFeedHoldJerk = feedHold.tangentialJerk != 0.0;
        if (hasFeedHoldAcceleration != hasFeedHoldJerk
            || (hasFeedHoldAcceleration
                && (!std::isfinite(feedHold.tangentialAcceleration)
                    || feedHold.tangentialAcceleration <= 0.0
                    || !std::isfinite(feedHold.tangentialJerk)
                    || feedHold.tangentialJerk <= 0.0))
            || !validPositiveLimit(feedHold.pathAcceleration)
            || !validAxisLimits(feedHold.axisAcceleration)) {
            throw std::invalid_argument(
                "production executor feed-hold limits must be positive "
                "and finite or infinite");
        }

        constexpr auto validJointMask =
            static_cast<JointMask>((JointMask{1} << MAX_JOINTS) - 1);
        for (const auto &axis : m_configuration.axes) {
            if ((axis.joints & ~validJointMask) != 0
                || (axis.joints & m_configuredJoints) != 0) {
                throw std::invalid_argument(
                    "production executor axis mappings must contain "
                    "distinct valid joints");
            }
            for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                const auto mask =
                    static_cast<JointMask>(JointMask{1} << joint);
                if ((axis.joints & mask) != 0
                    && (!std::isfinite(axis.coordinateScale[joint])
                        || std::abs(axis.coordinateScale[joint]) <= 1e-12)) {
                    throw std::invalid_argument(
                        "production executor axis mapping scales must be "
                        "finite and nonzero");
                }
            }
            m_configuredJoints |= axis.joints;
        }
    }

    PublishResult ProductionExecutorCore::tryPublish(
        const ExecutionItem &item) noexcept {
        if (!execution_item::valid(item)) {
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
            slot.normalMotionNanoseconds =
                execution_item::normalMotionNanoseconds(item);
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
        const MotionState &commanded, const MotionState &feedback,
        const JointMotionState &commandedJoints,
        const JointMotionState &feedbackJoints) noexcept {
        discardExecution();
        if (m_jog.has_value()) {
            m_jog.reset();
        }

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
        m_snapshot.commandedJoints = commandedJoints;
        m_snapshot.feedbackJoints = feedbackJoints;
        m_planStop.reset();
        m_controlledStoppedEpoch = 0;
        m_stopTailFaultCode = 0;
        m_outputState = {};
        m_faultEventEmitted = false;
    }

    void ProductionExecutorCore::serviceImmediate() noexcept {
        serviceControls();
        publishSnapshot();
    }

    void ProductionExecutorCore::setDigitalInputSample(
        const DigitalInputId input, const bool active) noexcept {
        m_digitalInputs[input] = active;
    }

    void ProductionExecutorCore::setDigitalInputSamples(
        const LogicalDigitalInputImage &inputs) noexcept {
        m_digitalInputs = inputs;
    }

    void ProductionExecutorCore::servoTick(const bool shouldPublishSnapshot) noexcept {
        serviceControls();

        if (m_snapshot.state == BackendState::Running
            || m_snapshot.state == BackendState::Holding) {
            if (m_jog.has_value()) {
                advanceJog(m_servoPeriod);
            } else if (!m_active.has_value()) {
                activateNext();
            }
            if (!m_jog.has_value() && m_active.has_value()
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

    void ProductionExecutorCore::reportHostFault(
        const std::uint32_t code) noexcept {
        fault(code);
        publishSnapshot();
    }

    void ProductionExecutorCore::latchEmergencyStop(const std::uint32_t code) noexcept {
        discardExecution();
        if (m_jog.has_value()) {
            abandonJog(JogStopReason::Aborted);
        }
        m_snapshot.commanded.velocity = {};
        m_snapshot.commanded.acceleration = {};
        m_snapshot.commandedJoints.velocity = {};
        m_snapshot.commandedJoints.acceleration = {};
        fault(code);
        publishSnapshot();
    }

    void ProductionExecutorCore::resetEmergencyStop() noexcept {
        if (m_snapshot.state != BackendState::Faulted) {
            return;
        }

        m_snapshot.state = BackendState::Disabled;
        m_snapshot.faultCode = 0;
        m_outputState = {};
        m_faultEventEmitted = false;
        publishSnapshot();
    }

    double ProductionExecutorCore::servoPeriod() const noexcept {
        return m_servoPeriod;
    }

    ProductionExecutorMotionContext
    ProductionExecutorCore::motionContext() const noexcept {
        ProductionExecutorMotionContext result;
        result.axisPosition = m_snapshot.commanded.position;
        result.jointPosition = m_snapshot.commandedJoints.position;
        if (!m_active.has_value()) {
            return result;
        }

        const auto &item = m_planSlots[*m_active].item;
        if (const auto *move = std::get_if<TriggeredMove>(&item)) {
            result.flags |= PRODUCTION_EXECUTOR_MOTION_IS_PROBE;
            result.axisStart = m_triggered.start;
            result.axisTarget = move->target;
        } else if (const auto *move =
                       std::get_if<TriggeredJointMove>(&item)) {
            result.flags |= PRODUCTION_EXECUTOR_MOTION_IS_HOMING;
            result.moveJoints = move->joints;
            for (const auto &trigger : move->triggers) {
                result.triggerJoints |=
                    static_cast<JointMask>(
                        JointMask{1} << trigger.joint);
            }
            for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                const auto mask =
                    static_cast<JointMask>(JointMask{1} << joint);
                if ((move->joints & mask) == 0) {
                    continue;
                }
                result.jointStart[joint] =
                    m_triggeredJoints[joint].start;
                result.jointTarget[joint] =
                    m_triggeredJoints[joint].target;
            }
        }

        return result;
    }

    ProductionExecutorOutputState ProductionExecutorCore::outputState() const noexcept {
        auto result = m_outputState;
        result.commandedJoints = m_snapshot.commandedJoints;
        result.motion = motionContext();
        result.executorEnabled =
            m_snapshot.state != BackendState::Disabled
            && m_snapshot.state != BackendState::Faulted;

        return result;
    }

    void ProductionExecutorCore::serviceControls() noexcept {
        ControlRequest request;
        while (m_controls.tryPop(request)) {
            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                auto success = false;
                if constexpr (std::same_as<T, EnableRequest>) {
                    success = !m_jog.has_value()
                        && (m_snapshot.state == BackendState::Disabled
                            || m_snapshot.state == BackendState::Held);
                    if (success) {
                        m_snapshot.state = BackendState::Held;
                        m_snapshot.faultCode = 0;
                        m_faultEventEmitted = false;
                    }
                } else if constexpr (std::same_as<T, DisableRequest>) {
                    if (m_jog.has_value()) {
                        abandonJog(JogStopReason::Disabled);
                    }
                    discardExecution();
                    m_snapshot.state = BackendState::Disabled;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    m_snapshot.commandedJoints.velocity = {};
                    m_snapshot.commandedJoints.acceleration = {};
                    m_snapshot.feedbackJoints =
                        m_snapshot.commandedJoints;
                    m_outputState = {};
                    success = true;
                } else if constexpr (std::same_as<T, StartRequest>) {
                    success = !m_jog.has_value()
                        && m_snapshot.state == BackendState::Held
                        && !m_feedRetiming.held
                        && value.epoch != 0 && value.epoch == m_snapshot.epoch
                        && value.epoch != m_controlledStoppedEpoch;
                    if (success) {
                        resetFeedRetiming();
                        m_snapshot.state = BackendState::Running;
                    }
                } else if constexpr (std::same_as<T, ResumeRequest>) {
                    const auto feedHeld = m_feedRetiming.held;
                    success = !m_jog.has_value()
                        && m_snapshot.state == BackendState::Held
                        && value.epoch != 0
                        && value.epoch == m_snapshot.epoch
                        && value.epoch != m_controlledStoppedEpoch;
                    if (success && feedHeld) {
                        success = m_active.has_value() && !m_stopping;
                        if (success && std::holds_alternative<PlanChunk>(
                                m_planSlots[*m_active].item)) {
                            m_feedRetiming.held = false;
                            m_feedRetiming.resuming = true;
                            m_feedRetiming.acceleration = 0.0;
                            m_feedRetiming.jerk = 0.0;
                            m_snapshot.state = BackendState::Running;
                        } else if (success
                                   && std::holds_alternative<TriggeredMove>(
                                       m_planSlots[*m_active].item)) {
                            success = !m_triggered.stopping
                                && initializeTriggered();
                            if (success) {
                                resetFeedRetiming();
                                m_snapshot.state = BackendState::Running;
                            }
                        } else {
                            success = false;
                        }
                    } else if (success) {
                        success = !m_active.has_value()
                            && m_queuedExecutionItems.load(
                                std::memory_order_acquire) != 0;
                    }
                    if (success && !feedHeld) {
                        resetFeedRetiming();
                        m_snapshot.state = BackendState::Running;
                    }
                } else if constexpr (std::same_as<T, FeedHoldRequest>) {
                    success = !m_jog.has_value()
                        && m_snapshot.state == BackendState::Running
                        && !m_feedRetiming.resuming
                        && m_active.has_value() && !m_stopping;
                    if (success && std::holds_alternative<PlanChunk>(
                            m_planSlots[*m_active].item)) {
                        success = feedHoldAvailable();
                    } else if (success
                               && std::holds_alternative<TriggeredMove>(
                                   m_planSlots[*m_active].item)) {
                        success = !m_triggered.stopping
                            && beginTriggeredStop(
                                TriggeredMoveStatus::ReachedTarget, true);
                    } else {
                        success = false;
                    }
                    if (success) {
                        m_feedRetiming.holding = true;
                        m_feedRetiming.held = false;
                        m_snapshot.state = BackendState::Holding;
                    }
                } else if constexpr (std::same_as<T, ControlledStopRequest>) {
                    const auto stoppingJog = m_jog.has_value();
                    if (stoppingJog) {
                        success =
                            beginJogStop(JogStopReason::RequestedStop);
                    } else {
                        success = (m_snapshot.state == BackendState::Running
                                || m_snapshot.state == BackendState::Holding
                                || (m_snapshot.state == BackendState::Held
                                    && m_feedRetiming.held))
                            && m_active.has_value();
                    }
                    if (success && !stoppingJog
                        && std::holds_alternative<PlanChunk>(
                            m_planSlots[*m_active].item)) {
                        success = !m_stopping && beginPlanStop();
                        if (success) {
                            resetFeedRetiming();
                        }
                    } else if (success && !stoppingJog
                        && std::holds_alternative<TriggeredMove>(
                            m_planSlots[*m_active].item)) {
                        if (m_triggered.stopping) {
                            success = false;
                        } else if (!beginTriggeredStop(
                                       TriggeredMoveStatus::Aborted)) {
                            success = false;
                            faultTriggered();
                        } else {
                            resetFeedRetiming();
                        }
                    } else if (success && !stoppingJog
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
                    } else if (!stoppingJog) {
                        success = false;
                    }
                    if (success && !stoppingJog) {
                        m_snapshot.state = BackendState::Holding;
                    }
                } else if constexpr (std::same_as<T, AbortRequest>) {
                    if (m_jog.has_value()) {
                        success = beginJogStop(JogStopReason::Aborted);
                    } else {
                        discardExecution();
                        m_snapshot.state = BackendState::Held;
                        m_snapshot.commanded.velocity = {};
                        m_snapshot.commanded.acceleration = {};
                        m_snapshot.feedback = m_snapshot.commanded;
                        success = true;
                    }
                    if (success) {
                        m_outputState = {};
                    }
                } else if constexpr (std::same_as<T, ResetRequest>) {
                    const auto enabled =
                        m_snapshot.state != BackendState::Disabled;
                    if (m_jog.has_value()) {
                        abandonJog(JogStopReason::Aborted);
                    }
                    discardExecution();
                    const auto commanded = m_snapshot.commanded;
                    const auto feedback = m_snapshot.feedback;
                    const auto commandedJoints = m_snapshot.commandedJoints;
                    const auto feedbackJoints = m_snapshot.feedbackJoints;
                    m_snapshot = {};
                    m_snapshot.state = enabled
                        ? BackendState::Held : BackendState::Disabled;
                    m_snapshot.epoch = value.nextEpoch;
                    m_snapshot.commanded = commanded;
                    m_snapshot.feedback = feedback;
                    m_snapshot.commandedJoints = commandedJoints;
                    m_snapshot.feedbackJoints = feedbackJoints;
                    m_planStop.reset();
                    m_controlledStoppedEpoch = 0;
                    m_outputState = {};
                    m_faultEventEmitted = false;
                    success = value.nextEpoch != 0;
                } else if constexpr (std::same_as<T, SetJointPositionRequest>) {
                    constexpr auto validJointMask =
                        static_cast<JointMask>(
                            (JointMask{1} << MAX_JOINTS) - 1);
                    success = !m_jog.has_value()
                        && m_snapshot.state == BackendState::Held
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
                } else if constexpr (
                    std::same_as<T, StartContinuousJogRequest>
                    || std::same_as<T, StartIncrementalJogRequest>) {
                    success = initializeJog(value);
                } else if constexpr (
                    std::same_as<T, RenewJogLeaseRequest>) {
                    success = m_jog.has_value() && m_jog->continuous
                        && !m_jog->stopping && m_jog->id == value.jog;
                    if (success) {
                        m_jog->leaseTicks = m_jog->leasePeriod;
                    }
                } else if constexpr (
                    std::same_as<T, SetContinuousJogVelocityRequest>) {
                    success = m_jog.has_value() && m_jog->id == value.jog
                        && setContinuousJogVelocity(value.signedVelocity);
                } else if constexpr (std::same_as<T, StopJogRequest>) {
                    success = m_jog.has_value() && m_jog->id == value.jog
                        && beginJogStop(JogStopReason::RequestedStop);
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
        if (execution_item::epoch(item) != m_snapshot.epoch) {
            emit(ChunkRejected{
                execution_item::epoch(item), execution_item::id(item)});
            release(index);

            return;
        }
        if (const auto *chunk = std::get_if<PlanChunk>(&item);
            chunk != nullptr
            && !samePosition(
                executionSpanStart(chunk->normalMotion[0]).position,
                m_snapshot.commanded.position)) {
            emit(ChunkRejected{chunk->epoch, chunk->id});
            release(index);
            fault(PLAN_START_DISCONTINUITY_FAULT);

            return;
        }

        m_active = index;
        m_stopping = false;
        m_span = 0;
        m_nextScheduledEvent = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
        m_triggered = {};
        m_triggeredJoints = {};
        m_triggeredJointMask = 0;
        m_triggeredJointCompletionStatus =
            TriggeredMoveStatus::ReachedTarget;
        m_stopTailFaultCode = 0;
        m_snapshot.activeChunk = execution_item::id(item);
        m_snapshot.activeSpan = 0;
        emit(ChunkAccepted{
            execution_item::epoch(item), execution_item::id(item)});
        if (std::holds_alternative<PlanChunk>(item)) {
            applyScheduledEventsForCurrentSpan();
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }
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
            if (m_planStop.has_value()) {
                advancePlanStop(seconds);
            } else if (std::holds_alternative<PlanChunk>(item)) {
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

    bool ProductionExecutorCore::feedHoldAvailable() const noexcept {
        return m_configuration.feedHold.tangentialAcceleration > 0.0
            && m_configuration.feedHold.tangentialJerk > 0.0;
    }

    bool ProductionExecutorCore::feedRetimingAccelerationInterval(
        const position_t &referenceVelocity,
        const position_t &referenceAcceleration,
        double &lower, double &upper) const noexcept {
        lower = -std::numeric_limits<double>::infinity();
        upper = std::numeric_limits<double>::infinity();
        const auto base = scaled(
            referenceAcceleration,
            m_feedRetiming.rate * m_feedRetiming.rate);

        return constrainLinearInterval(
            base, referenceVelocity,
            m_configuration.feedHold.axisAcceleration,
            m_configuration.feedHold.pathAcceleration,
            lower, upper);
    }

    double ProductionExecutorCore::feedRetimingReferenceAdvance(
        const double physicalSeconds) noexcept {
        const auto &span = currentSpan();
        const auto parameter = std::clamp(
            m_spanElapsed * span.inverseDuration, 0.0, 1.0);
        const auto reference =
            evaluateExecutionPolynomial(span, parameter);
        const auto referenceSpeed =
            magnitude(reference.state.velocity);
        const auto physicalReferenceAcceleration = scaled(
            reference.state.acceleration,
            m_feedRetiming.rate * m_feedRetiming.rate);
        if (referenceSpeed <= 1e-10
            && magnitude(physicalReferenceAcceleration) <= 1e-9) {
            m_feedRetiming.rate =
                m_feedRetiming.resuming ? 1.0 : 0.0;
            m_feedRetiming.acceleration = 0.0;
            m_feedRetiming.jerk = 0.0;
            m_feedRetiming.resuming = false;

            return m_feedRetiming.rate * physicalSeconds;
        }

        auto accelerationLower = 0.0;
        auto accelerationUpper = 0.0;
        if (!feedRetimingAccelerationInterval(
                reference.state.velocity,
                reference.state.acceleration,
                accelerationLower, accelerationUpper)) {
            fault(FEED_RETIMING_FAULT);

            return 0.0;
        }

        const auto safeReferenceSpeed =
            std::max(referenceSpeed, 1e-9);
        const auto accelerationMagnitude =
            m_configuration.feedHold.tangentialAcceleration
            / safeReferenceSpeed;
        const auto jerkMagnitude =
            m_configuration.feedHold.tangentialJerk
            / safeReferenceSpeed;
        const auto release = m_feedRetiming.resuming
            ? m_feedRetiming.acceleration > 0.0
                && 1.0 - m_feedRetiming.rate
                    <= m_feedRetiming.acceleration
                        * m_feedRetiming.acceleration
                        / (2.0 * std::max(jerkMagnitude, 1e-12))
                        + 1e-12
            : m_feedRetiming.acceleration < 0.0
                && m_feedRetiming.rate
                    <= m_feedRetiming.acceleration
                        * m_feedRetiming.acceleration
                        / (2.0 * std::max(jerkMagnitude, 1e-12))
                        + 1e-12;
        const auto targetAcceleration = std::clamp(
            m_feedRetiming.resuming
                ? accelerationMagnitude : -accelerationMagnitude,
            accelerationLower, accelerationUpper);
        const auto requestedJerk = m_feedRetiming.resuming
            ? (release ? -jerkMagnitude : jerkMagnitude)
            : (release ? jerkMagnitude : -jerkMagnitude);
        const auto previousAcceleration =
            m_feedRetiming.acceleration;
        auto nextAcceleration = previousAcceleration
            + requestedJerk * physicalSeconds;
        if (m_feedRetiming.resuming) {
            if (release) {
                nextAcceleration = std::max(nextAcceleration, 0.0);
            } else if (nextAcceleration > targetAcceleration) {
                nextAcceleration = targetAcceleration;
            }
        } else if (release) {
            nextAcceleration = std::min(nextAcceleration, 0.0);
        } else if (nextAcceleration < targetAcceleration) {
            nextAcceleration = targetAcceleration;
        }
        nextAcceleration = std::clamp(
            nextAcceleration, accelerationLower, accelerationUpper);

        auto nextRate = m_feedRetiming.rate
            + 0.5 * (previousAcceleration + nextAcceleration)
                * physicalSeconds;
        if (!m_feedRetiming.resuming && nextRate <= 1e-10) {
            nextRate = 0.0;
            nextAcceleration = 0.0;
        } else if (m_feedRetiming.resuming
                   && nextRate >= 1.0 - 1e-10) {
            nextRate = 1.0;
            nextAcceleration = 0.0;
            m_feedRetiming.resuming = false;
        }
        nextRate = std::clamp(nextRate, 0.0, 1.0);

        m_feedRetiming.jerk =
            (nextAcceleration - previousAcceleration)
            / physicalSeconds;
        const auto referenceAdvance =
            0.5 * (m_feedRetiming.rate + nextRate)
            * physicalSeconds;
        m_feedRetiming.rate = nextRate;
        m_feedRetiming.acceleration = nextAcceleration;

        return referenceAdvance;
    }

    MotionState ProductionExecutorCore::retimedState(
        const ExecutionPolynomialEvaluation &reference) const noexcept {
        const auto rate = m_feedRetiming.rate;

        return {
            reference.state.position,
            scaled(reference.state.velocity, rate),
            scaled(reference.state.acceleration, rate * rate)
                + scaled(reference.state.velocity,
                         m_feedRetiming.acceleration),
        };
    }

    void ProductionExecutorCore::finishFeedHold() noexcept {
        m_feedRetiming.holding = false;
        m_feedRetiming.held = true;
        m_feedRetiming.rate = 0.0;
        m_feedRetiming.acceleration = 0.0;
        m_feedRetiming.jerk = 0.0;
        m_snapshot.state = BackendState::Held;
        m_snapshot.commanded.velocity = {};
        m_snapshot.commanded.acceleration = {};
        m_snapshot.feedback = m_snapshot.commanded;
        m_snapshot.executionRate = 0.0;
        m_snapshot.executionRateAcceleration = 0.0;
        emit(BackendHeld{
            m_snapshot.epoch, m_snapshot.commanded,
            BackendHoldReason::FeedHold,
        });
    }

    void ProductionExecutorCore::resetFeedRetiming() noexcept {
        m_feedRetiming = {};
        m_snapshot.executionRate = 1.0;
        m_snapshot.executionRateAcceleration = 0.0;
    }

    void ProductionExecutorCore::faultFeedRetimingAtStopBranch() noexcept {
        discardExecution();
        fault(FEED_RETIMING_STOP_BRANCH_FAULT);
    }

    bool ProductionExecutorCore::beginPlanStop() noexcept {
        const auto &limits = m_configuration.controlledStopLimits;
        PlanStopRuntime runtime;
        runtime.origin = m_snapshot.commanded;

        ruckig::InputParameter<6> input;
        input.control_interface = ruckig::ControlInterface::Velocity;
        input.current_position = {};
        input.current_velocity = axisValues(runtime.origin.velocity);
        input.current_acceleration =
            axisValues(runtime.origin.acceleration);
        input.target_position = {};
        input.target_velocity = {};
        input.target_acceleration = {};
        const auto velocityLimits = axisValues(limits.velocity);
        const auto accelerationLimits =
            axisValues(limits.acceleration);
        const auto jerkLimits = axisValues(limits.jerk);
        auto movingAxes = std::size_t{0};
        for (auto axis = std::size_t{0}; axis < 6; ++axis) {
            const auto moving =
                std::abs(input.current_velocity[axis]) > 1e-12
                || std::abs(input.current_acceleration[axis]) > 1e-12;
            input.enabled[axis] = moving;
            if (!moving) {
                input.max_velocity[axis] = 1.0;
                input.max_acceleration[axis] = 1.0;
                input.max_jerk[axis] = 1.0;
                continue;
            }
            if (!std::isfinite(velocityLimits[axis])
                || velocityLimits[axis] <= 0.0
                || !std::isfinite(accelerationLimits[axis])
                || accelerationLimits[axis] <= 0.0
                || !std::isfinite(jerkLimits[axis])
                || jerkLimits[axis] <= 0.0) {
                return false;
            }

            input.max_velocity[axis] = velocityLimits[axis];
            input.max_acceleration[axis] = accelerationLimits[axis];
            input.max_jerk[axis] = jerkLimits[axis];
            ++movingAxes;
        }
        if (movingAxes == 0) {
            runtime.stationary = true;
            m_planStop = std::move(runtime);

            return true;
        }

        ruckig::Ruckig<6> generator;
        if (generator.calculate(input, runtime.trajectory)
            != ruckig::Result::Working) {
            return false;
        }

        m_planStop = std::move(runtime);

        return true;
    }

    void ProductionExecutorCore::advancePlanStop(double &seconds) noexcept {
        auto &stop = *m_planStop;
        if (stop.stationary) {
            completePlanStop();

            return;
        }

        const auto duration = stop.trajectory.get_duration();
        const auto consumed =
            std::min(seconds, std::max(duration - stop.elapsed, 0.0));
        stop.elapsed += consumed;

        std::array<double, 6> position{};
        std::array<double, 6> velocity{};
        std::array<double, 6> acceleration{};
        stop.trajectory.at_time(
            stop.elapsed, position, velocity, acceleration);
        m_snapshot.commanded = {
            stop.origin.position + axisPosition(position),
            axisPosition(velocity),
            axisPosition(acceleration),
        };
        m_snapshot.feedback = m_snapshot.commanded;
        m_snapshot.spanProgress = duration > 0.0
            ? std::clamp(stop.elapsed / duration, 0.0, 1.0) : 1.0;
        seconds -= consumed;

        if (stop.elapsed + 1e-12 < duration) {
            return;
        }

        completePlanStop();
    }

    void ProductionExecutorCore::completePlanStop() noexcept {
        const auto epoch = activeChunk().epoch;
        m_snapshot.commanded.velocity = {};
        m_snapshot.commanded.acceleration = {};
        m_snapshot.feedback = m_snapshot.commanded;

        emit(ChunkRetired{epoch, activeChunk().id});
        release(*m_active);
        m_active.reset();

        std::uint8_t index = 0;
        while (m_plans.tryPop(index)) {
            accountForDequeued(index);
            emit(ChunkRetired{
                execution_item::epoch(m_planSlots[index].item),
                execution_item::id(m_planSlots[index].item),
            });
            release(index);
        }

        m_planStop.reset();
        m_controlledStoppedEpoch = epoch;
        m_stopping = false;
        m_span = 0;
        m_nextScheduledEvent = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
        m_snapshot.state = BackendState::Held;
        emit(BackendHeld{
            epoch, m_snapshot.commanded,
            BackendHoldReason::ControlledStop,
        });
    }

    void ProductionExecutorCore::advancePlan(double &seconds) noexcept {
        const auto retiming =
            m_feedRetiming.holding || m_feedRetiming.resuming;
        auto referenceSeconds = seconds;
        if (retiming) {
            referenceSeconds =
                feedRetimingReferenceAdvance(seconds);
            seconds = 0.0;
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }
        }

        while (m_active.has_value() && referenceSeconds > 0.0
               && (m_snapshot.state == BackendState::Running
                   || m_snapshot.state == BackendState::Holding)
               && std::holds_alternative<PlanChunk>(
                   m_planSlots[*m_active].item)) {
            const auto &span = currentSpan();
            const auto remaining =
                std::max(span.duration - m_spanElapsed, 0.0);
            const auto consumed =
                std::min(referenceSeconds, remaining);
            referenceSeconds -= consumed;
            if (!retiming) {
                seconds -= consumed;
            }
            m_spanElapsed += consumed;

            const auto parameter = std::clamp(
                m_spanElapsed * span.inverseDuration, 0.0, 1.0);
            m_snapshot.activeSpan = span.id;
            m_snapshot.spanProgress = parameter;
            const auto reference =
                evaluateExecutionPolynomial(span, parameter);
            if (retiming) {
                m_snapshot.commanded = retimedState(reference);
            } else {
                m_snapshot.commanded = reference.state;
            }
            m_snapshot.feedback = m_snapshot.commanded;
            emitExecutionMarkersThrough(parameter);
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }

            if (m_spanElapsed + 1e-12 < span.duration) {
                break;
            }
            completeSpan();
        }

        m_snapshot.executionRate = m_feedRetiming.rate;
        m_snapshot.executionRateAcceleration =
            m_feedRetiming.acceleration;
        if (m_feedRetiming.holding
            && m_feedRetiming.rate == 0.0
            && m_snapshot.state != BackendState::Faulted) {
            finishFeedHold();
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
        const TriggeredMoveStatus status, const bool feedHold) noexcept {
        const auto &move = activeTriggeredMove();
        m_triggered.stopOrigin = m_snapshot.commanded;
        if (!feedHold) {
            m_triggered.triggerState = m_snapshot.commanded;
        }
        m_triggered.completionStatus = status;
        m_triggered.stopping = true;
        m_triggered.feedHoldStopping = feedHold;
        m_triggered.elapsed = 0.0;
        const auto scalarVelocity =
            dot(m_snapshot.commanded.velocity, m_triggered.direction);
        const auto scalarAcceleration =
            dot(m_snapshot.commanded.acceleration, m_triggered.direction);
        if (std::abs(scalarVelocity) <= 1e-12
            && std::abs(scalarAcceleration) <= 1e-12) {
            m_triggered.trajectory = {};

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
        if (triggeredInputConditionMet(move)) {
            if (!m_triggered.stopping) {
                if (!beginTriggeredStop(TriggeredMoveStatus::Triggered)) {
                    faultTriggered();

                    return;
                }
                if (m_triggered.trajectory.get_duration() <= 1e-12) {
                    completeTriggered(TriggeredMoveStatus::Triggered);

                    return;
                }
            } else if (m_triggered.feedHoldStopping) {
                m_triggered.triggerState = m_snapshot.commanded;
                m_triggered.completionStatus =
                    TriggeredMoveStatus::Triggered;
                m_triggered.feedHoldStopping = false;
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
        if (m_triggered.stopping && duration <= 1e-12) {
            m_snapshot.commanded.velocity = {};
            m_snapshot.commanded.acceleration = {};
            m_snapshot.feedback = m_snapshot.commanded;
            if (m_triggered.feedHoldStopping) {
                finishTriggeredFeedHold();
            } else {
                completeTriggered(m_triggered.completionStatus);
            }

            return;
        }
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
            if (m_triggered.feedHoldStopping) {
                finishTriggeredFeedHold();
            } else {
                completeTriggered(m_triggered.completionStatus);
            }

            return;
        }

        m_snapshot.commanded.position = move.target;
        m_snapshot.commanded.velocity = {};
        m_snapshot.commanded.acceleration = {};
        m_snapshot.feedback = m_snapshot.commanded;
        completeTriggered(TriggeredMoveStatus::ReachedTarget);
    }

    void ProductionExecutorCore::finishTriggeredFeedHold() noexcept {
        m_triggered.stopping = false;
        m_triggered.feedHoldStopping = false;
        finishFeedHold();
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
        resetFeedRetiming();
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
        runtime.start = state.position[joint];
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

    bool ProductionExecutorCore::triggeredJointInputConditionMet(
        const JointTrigger &trigger) const noexcept {
        const auto current =
            static_cast<bool>(m_digitalInputs[trigger.input]);
        const auto previous =
            static_cast<bool>(m_previousDigitalInputs[trigger.input]);
        switch (trigger.condition) {
            case InputCondition::Active: return current;
            case InputCondition::Inactive: return !current;
            case InputCondition::RisingEdge: return current && !previous;
            case InputCondition::FallingEdge: return !current && previous;
        }

        return false;
    }

    void ProductionExecutorCore::advanceTriggeredJoints(
        double &seconds) noexcept {
        const auto move = activeTriggeredJointMove();
        for (const auto &trigger : move.triggers) {
            auto &runtime = m_triggeredJoints[trigger.joint];
            if (!runtime.finished && !runtime.stopping
                && triggeredJointInputConditionMet(trigger)
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

    bool ProductionExecutorCore::validJogTarget(
        const JogTarget &target) const noexcept {
        constexpr auto validJointMask =
            static_cast<JointMask>((JointMask{1} << MAX_JOINTS) - 1);
        if ((target.joints & ~validJointMask) != 0) {
            return false;
        }
        if (target.type == JogTargetType::Axis) {
            return target.joints == 0 && axisJoints(target.axis) != 0;
        }
        if (target.joints == 0
            || (m_configuredJoints != 0
                && (target.joints & ~m_configuredJoints) != 0)) {
            return false;
        }
        if (target.type == JogTargetType::Joint) {
            return (target.joints & (target.joints - 1)) == 0;
        }

        return target.type == JogTargetType::JointGroup;
    }

    bool ProductionExecutorCore::validJogLimits(
        const JogMotionLimits &limits) noexcept {
        return std::isfinite(limits.velocity) && limits.velocity > 0.0
            && std::isfinite(limits.acceleration)
            && limits.acceleration > 0.0
            && std::isfinite(limits.jerk) && limits.jerk > 0.0;
    }

    bool ProductionExecutorCore::validJogTravel(
        const JogTravelRange &travel) noexcept {
        return !travel.enabled
            || (std::isfinite(travel.minimum)
                && std::isfinite(travel.maximum)
                && travel.minimum <= travel.maximum);
    }

    JointMask ProductionExecutorCore::axisJoints(
        const AxisId axis) const noexcept {
        const auto index = static_cast<std::size_t>(axis);
        if (index >= m_configuration.axes.size()) {
            return 0;
        }

        return m_configuration.axes[index].joints;
    }

    double ProductionExecutorCore::jogCoordinate(
        const JogTarget &target) const noexcept {
        if (target.type == JogTargetType::Axis) {
            return axisComponent(m_snapshot.commanded.position, target.axis);
        }
        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            if ((target.joints & (JointMask{1} << joint)) != 0) {
                return m_snapshot.commandedJoints.position[joint];
            }
        }

        return 0.0;
    }

    double ProductionExecutorCore::jogVelocity(
        const JogTarget &target) const noexcept {
        if (target.type == JogTargetType::Axis) {
            return axisComponent(m_snapshot.commanded.velocity, target.axis);
        }
        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            if ((target.joints & (JointMask{1} << joint)) != 0) {
                return m_snapshot.commandedJoints.velocity[joint];
            }
        }

        return 0.0;
    }

    double ProductionExecutorCore::jogAcceleration(
        const JogTarget &target) const noexcept {
        if (target.type == JogTargetType::Axis) {
            return axisComponent(
                m_snapshot.commanded.acceleration, target.axis);
        }
        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            if ((target.joints & (JointMask{1} << joint)) != 0) {
                return m_snapshot.commandedJoints.acceleration[joint];
            }
        }

        return 0.0;
    }

    void ProductionExecutorCore::applyJogState() noexcept {
        const auto &jog = *m_jog;
        if (jog.target.type == JogTargetType::Axis) {
            axisComponent(m_snapshot.commanded.position, jog.target.axis) =
                jog.axisOrigin + jog.position;
            axisComponent(m_snapshot.commanded.velocity, jog.target.axis) =
                jog.velocity;
            axisComponent(
                m_snapshot.commanded.acceleration, jog.target.axis) =
                jog.acceleration;
            m_snapshot.feedback = m_snapshot.commanded;

            const auto &mapping = m_configuration.axes[
                static_cast<std::size_t>(jog.target.axis)];
            for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                const auto mask =
                    static_cast<JointMask>(JointMask{1} << joint);
                if ((mapping.joints & mask) == 0) {
                    continue;
                }
                m_snapshot.commandedJoints.position[joint] =
                    jog.jointOrigin[joint]
                    + jog.position * mapping.coordinateScale[joint];
                m_snapshot.commandedJoints.velocity[joint] =
                    jog.velocity * mapping.coordinateScale[joint];
                m_snapshot.commandedJoints.acceleration[joint] =
                    jog.acceleration * mapping.coordinateScale[joint];
            }
            m_snapshot.feedbackJoints = m_snapshot.commandedJoints;

            return;
        }

        for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
            const auto mask =
                static_cast<JointMask>(JointMask{1} << joint);
            if ((jog.target.joints & mask) == 0) {
                continue;
            }
            m_snapshot.commandedJoints.position[joint] =
                jog.jointOrigin[joint] + jog.position;
            m_snapshot.commandedJoints.velocity[joint] = jog.velocity;
            m_snapshot.commandedJoints.acceleration[joint] =
                jog.acceleration;
        }
        m_snapshot.feedbackJoints = m_snapshot.commandedJoints;
    }

    bool ProductionExecutorCore::calculateJogPosition(
        const double distance, const double velocity) noexcept {
        auto &jog = *m_jog;
        ruckig::InputParameter<1> input;
        input.current_position = {jog.position};
        input.current_velocity = {jog.velocity};
        input.current_acceleration = {jog.acceleration};
        input.target_position = {distance};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {
            std::max(
                std::min(velocity, jog.limits.velocity),
                std::abs(jog.velocity)),
        };
        input.max_acceleration = {jog.limits.acceleration};
        input.max_jerk = {jog.limits.jerk};
        ruckig::Ruckig<1> generator;

        return generator.calculate(input, jog.trajectory)
            == ruckig::Result::Working;
    }

    bool ProductionExecutorCore::calculateJogVelocity(
        const double targetVelocity) noexcept {
        auto &jog = *m_jog;
        const auto &limits = jog.stopping
            ? jog.stopLimits : jog.limits;
        ruckig::InputParameter<1> input;
        input.control_interface = ruckig::ControlInterface::Velocity;
        input.current_position = {jog.position};
        input.current_velocity = {jog.velocity};
        input.current_acceleration = {jog.acceleration};
        input.target_position = {jog.position};
        input.target_velocity = {targetVelocity};
        input.target_acceleration = {0.0};
        input.max_velocity = {
            std::max(limits.velocity, std::abs(jog.velocity)),
        };
        input.max_acceleration = {limits.acceleration};
        input.max_jerk = {limits.jerk};
        ruckig::Ruckig<1> generator;

        return generator.calculate(input, jog.trajectory)
            == ruckig::Result::Working;
    }

    bool ProductionExecutorCore::initializeJog(
        const StartContinuousJogRequest &request) noexcept {
        if (m_snapshot.state != BackendState::Held
            || m_active.has_value() || m_jog.has_value()
            || request.jog == 0 || !validJogTarget(request.target)
            || !validJogLimits(request.limits)
            || !validJogLimits(request.stopLimits)
            || !validJogTravel(request.travel)
            || !std::isfinite(request.signedVelocity)
            || std::abs(request.signedVelocity) <= 1e-12
            || request.leaseTicks == 0) {
            return false;
        }

        const auto coordinate = jogCoordinate(request.target);
        if (request.travel.enabled
            && (coordinate < request.travel.minimum - 1e-12
                || coordinate > request.travel.maximum + 1e-12)) {
            return false;
        }

        m_jog.emplace();
        auto &jog = *m_jog;
        jog.id = request.jog;
        jog.target = request.target;
        jog.limits = request.limits;
        jog.stopLimits = request.stopLimits;
        jog.travel = request.travel;
        jog.axisOrigin = coordinate;
        jog.jointOrigin = m_snapshot.commandedJoints.position;
        jog.velocity = jogVelocity(request.target);
        jog.acceleration = jogAcceleration(request.target);
        jog.continuous = true;
        jog.leasePeriod = std::min(
            request.leaseTicks,
            m_configuration.maximumJogLeaseTicks);
        jog.leaseTicks = jog.leasePeriod;
        jog.cruiseVelocity = std::clamp(
            request.signedVelocity, -jog.limits.velocity,
            jog.limits.velocity);

        auto calculated = false;
        if (jog.travel.enabled) {
            const auto target = jog.cruiseVelocity < 0.0
                ? jog.travel.minimum : jog.travel.maximum;
            const auto distance = target - coordinate;
            calculated = std::abs(distance) > 1e-12
                && ((jog.cruiseVelocity < 0.0 && distance < 0.0)
                    || (jog.cruiseVelocity > 0.0 && distance > 0.0))
                && calculateJogPosition(
                    distance, std::abs(jog.cruiseVelocity));
        } else {
            calculated = calculateJogVelocity(jog.cruiseVelocity);
        }
        if (!calculated) {
            m_jog.reset();

            return false;
        }

        m_snapshot.state = BackendState::Running;
        m_snapshot.activeJoints = request.target.type == JogTargetType::Axis
            ? axisJoints(request.target.axis) : request.target.joints;

        return true;
    }

    bool ProductionExecutorCore::initializeJog(
        const StartIncrementalJogRequest &request) noexcept {
        if (m_snapshot.state != BackendState::Held
            || m_active.has_value() || m_jog.has_value()
            || request.jog == 0 || !validJogTarget(request.target)
            || !validJogLimits(request.limits)
            || !validJogLimits(request.stopLimits)
            || !validJogTravel(request.travel)
            || !std::isfinite(request.distance)
            || std::abs(request.distance) <= 1e-12
            || !std::isfinite(request.velocity)
            || request.velocity <= 0.0) {
            return false;
        }

        const auto coordinate = jogCoordinate(request.target);
        if (request.travel.enabled
            && (coordinate < request.travel.minimum - 1e-12
                || coordinate > request.travel.maximum + 1e-12)) {
            return false;
        }
        auto distance = request.distance;
        if (request.travel.enabled) {
            distance = std::clamp(
                coordinate + distance, request.travel.minimum,
                request.travel.maximum) - coordinate;
        }
        if (std::abs(distance) <= 1e-12) {
            return false;
        }

        m_jog.emplace();
        auto &jog = *m_jog;
        jog.id = request.jog;
        jog.target = request.target;
        jog.limits = request.limits;
        jog.stopLimits = request.stopLimits;
        jog.travel = request.travel;
        jog.axisOrigin = coordinate;
        jog.jointOrigin = m_snapshot.commandedJoints.position;
        jog.velocity = jogVelocity(request.target);
        jog.acceleration = jogAcceleration(request.target);
        if (!calculateJogPosition(distance, request.velocity)) {
            m_jog.reset();

            return false;
        }

        m_snapshot.state = BackendState::Running;
        m_snapshot.activeJoints = request.target.type == JogTargetType::Axis
            ? axisJoints(request.target.axis) : request.target.joints;

        return true;
    }

    bool ProductionExecutorCore::setContinuousJogVelocity(
        const double signedVelocity) noexcept {
        if (!m_jog.has_value() || !m_jog->continuous
            || m_jog->stopping || !std::isfinite(signedVelocity)
            || std::abs(signedVelocity) <= 1e-12) {
            return false;
        }

        auto &jog = *m_jog;
        jog.cruiseVelocity = std::clamp(
            signedVelocity, -jog.limits.velocity, jog.limits.velocity);
        auto calculated = false;
        if (jog.travel.enabled) {
            const auto coordinate = jog.axisOrigin + jog.position;
            const auto target = jog.cruiseVelocity < 0.0
                ? jog.travel.minimum : jog.travel.maximum;
            const auto distance = target - jog.axisOrigin;
            calculated =
                ((jog.cruiseVelocity < 0.0 && target < coordinate - 1e-12)
                    || (jog.cruiseVelocity > 0.0
                        && target > coordinate + 1e-12))
                && calculateJogPosition(
                    distance, std::abs(jog.cruiseVelocity));
        } else {
            calculated = calculateJogVelocity(jog.cruiseVelocity);
        }
        if (!calculated) {
            return false;
        }

        jog.elapsed = 0.0;
        jog.cruising = false;
        jog.leaseTicks = jog.leasePeriod;

        return true;
    }

    bool ProductionExecutorCore::beginJogStop(
        const JogStopReason reason) noexcept {
        if (!m_jog.has_value() || m_jog->stopping) {
            return false;
        }

        auto &jog = *m_jog;
        jog.stopping = true;
        jog.cruising = false;
        jog.stopReason = reason;
        jog.elapsed = 0.0;
        if (std::abs(jog.velocity) <= 1e-12
            && std::abs(jog.acceleration) <= 1e-12) {
            jog.velocity = 0.0;
            jog.acceleration = 0.0;
            applyJogState();
            completeJog();

            return true;
        }
        if (!calculateJogVelocity(0.0)) {
            faultJog();

            return false;
        }

        return true;
    }

    void ProductionExecutorCore::advanceJog(const double seconds) noexcept {
        if (!m_jog.has_value() || seconds <= 0.0) {
            return;
        }
        if (m_jog->continuous && !m_jog->stopping
            && m_jog->leaseTicks != 0) {
            --m_jog->leaseTicks;
            if (m_jog->leaseTicks == 0
                && !beginJogStop(JogStopReason::LeaseExpired)) {
                return;
            }
            if (!m_jog.has_value()) {
                return;
            }
        }

        auto &jog = *m_jog;
        if (jog.cruising) {
            jog.position += jog.cruiseVelocity * seconds;
            jog.velocity = jog.cruiseVelocity;
            jog.acceleration = 0.0;
            applyJogState();

            return;
        }

        const auto duration = jog.trajectory.get_duration();
        const auto consumed =
            std::min(seconds, std::max(duration - jog.elapsed, 0.0));
        jog.elapsed += consumed;
        jog.trajectory.at_time(
            jog.elapsed, jog.position, jog.velocity, jog.acceleration);
        applyJogState();
        if (jog.elapsed + 1e-12 < duration) {
            return;
        }

        const auto remaining = seconds - consumed;
        if (jog.stopping) {
            jog.velocity = 0.0;
            jog.acceleration = 0.0;
            applyJogState();
            completeJog();

            return;
        }
        if (jog.continuous && !jog.travel.enabled) {
            jog.cruising = true;
            if (remaining > 0.0) {
                jog.position += jog.cruiseVelocity * remaining;
                jog.velocity = jog.cruiseVelocity;
                jog.acceleration = 0.0;
                applyJogState();
            }

            return;
        }

        jog.velocity = 0.0;
        jog.acceleration = 0.0;
        jog.stopReason = jog.continuous
            ? JogStopReason::LimitReached : JogStopReason::TargetReached;
        applyJogState();
        completeJog();
    }

    void ProductionExecutorCore::completeJog() noexcept {
        const auto jog = *m_jog;
        m_snapshot.activeJoints = 0;
        m_snapshot.state = BackendState::Held;
        emit(JogStopped{
            jog.id, jog.target, jog.stopReason,
            m_snapshot.commanded, m_snapshot.commandedJoints,
        });
        m_jog.reset();
    }

    void ProductionExecutorCore::abandonJog(
        const JogStopReason reason) noexcept {
        if (!m_jog.has_value()) {
            return;
        }

        m_jog->velocity = 0.0;
        m_jog->acceleration = 0.0;
        m_jog->stopReason = reason;
        applyJogState();
        completeJog();
    }

    void ProductionExecutorCore::faultJog() noexcept {
        if (m_jog.has_value()) {
            m_jog->stopReason = JogStopReason::Fault;
            const auto jog = *m_jog;
            m_snapshot.activeJoints = 0;
            emit(JogStopped{
                jog.id, jog.target, jog.stopReason,
                m_snapshot.commanded, m_snapshot.commandedJoints,
            });
            m_jog.reset();
        }
        fault(JOG_GENERATION_FAULT);
    }

    void ProductionExecutorCore::completeSpan() noexcept {
        m_spanElapsed = 0.0;
        ++m_span;
        const auto count = m_stopping
            ? activeChunk().stopTail.size : activeChunk().normalMotion.size;
        if (m_span < count) {
            applyScheduledEventsForCurrentSpan();
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }
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
        if (m_stopTailFaultCode != 0) {
            const auto faultCode = m_stopTailFaultCode;
            emit(ChunkRetired{chunk.epoch, chunk.id});
            discardExecution();
            fault(faultCode);

            return;
        }
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
            if (execution_item::epoch(continuation) == current.epoch
                && execution_item::predecessor(continuation)
                    == current.branch) {
                emit(BranchSelected{
                    current.epoch, current.branch, BranchChoice::Continue,
                    execution_item::id(continuation),
                });
                emit(ChunkRetired{current.epoch, current.id});
                m_active = continuationIndex;
                release(currentIndex);
                m_stopping = false;
                m_span = 0;
                m_nextScheduledEvent = 0;
                m_nextMarker = 0;
                m_spanElapsed = 0.0;
                m_snapshot.activeChunk = execution_item::id(continuation);
                m_snapshot.activeSpan = 0;
                emit(ChunkAccepted{
                    execution_item::epoch(continuation),
                    execution_item::id(continuation),
                });
                if (std::holds_alternative<PlanChunk>(continuation)) {
                    applyScheduledEventsForCurrentSpan();
                    if (m_snapshot.state == BackendState::Faulted) {
                        return;
                    }
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
                execution_item::epoch(continuation),
                execution_item::id(continuation),
            });
            release(continuationIndex);
            m_stopTailFaultCode =
                PLAN_CONTINUATION_DISCONTINUITY_FAULT;
        } else if (current.stopTailPolicy
                   == StopTailPolicy::ContinuationRequired) {
            m_stopTailFaultCode = PLAN_UNDERRUN_FAULT;
        }

        if (m_feedRetiming.holding || m_feedRetiming.resuming) {
            faultFeedRetimingAtStopBranch();

            return;
        }

        emit(BranchSelected{
            current.epoch, current.branch, BranchChoice::Stop, 0,
        });
        m_stopping = true;
        m_span = 0;
        m_spanElapsed = 0.0;
    }

    void ProductionExecutorCore::applyScheduledEventsForCurrentSpan() noexcept {
        if (m_stopping || !m_active.has_value()) {
            return;
        }

        const auto &chunk = activeChunk();
        while (m_nextScheduledEvent < chunk.events.size) {
            const auto &event = chunk.events[m_nextScheduledEvent];
            if (event.span > m_span) {
                return;
            }
            if (event.span < m_span
                || event.span >= chunk.normalMotion.size) {
                fault(INVALID_EXECUTION_STATE_FAULT);

                return;
            }

            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::same_as<T, SpindleEvent>) {
                    m_outputState.spindle = value;
                }
            }, event.value);
            ++m_nextScheduledEvent;
        }
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
        m_outputState = {};
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
        m_planStop.reset();
        resetFeedRetiming();
        m_span = 0;
        m_nextScheduledEvent = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
        m_triggered = {};
        m_triggeredJoints = {};
        m_triggeredJointMask = 0;
        m_triggeredJointCompletionStatus =
            TriggeredMoveStatus::ReachedTarget;
        m_stopTailFaultCode = 0;
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
