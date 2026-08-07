#include "machine/HomingController.h"

#include "machine/MachineAxis.h"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace ngc {
    namespace {
        InputCondition releasedCondition(
            const InputCondition condition) noexcept {
            switch (condition) {
                case InputCondition::Active:
                case InputCondition::RisingEdge:
                    return InputCondition::Inactive;
                case InputCondition::Inactive:
                case InputCondition::FallingEdge:
                    return InputCondition::Active;
            }

            return InputCondition::Inactive;
        }
    }

    HomingController::HomingController(std::vector<AxisConfiguration> axes,
                                       std::vector<JointConfiguration> joints,
                                       HomingConfiguration homing, MotionBackend &backend,
                                       ExecutorDemandController &demand)
        : m_axes(std::move(axes)), m_joints(std::move(joints)),
          m_homing(std::move(homing)), m_backend(backend), m_demand(demand) { }

    bool HomingController::available() const noexcept {
        return !m_joints.empty() && !m_homing.groups.empty();
    }

    std::expected<HomingResult, std::string> HomingController::run(
        const EpochId epoch, const position_t &startingPosition,
        const HomingRuntimeCallbacks &callbacks) {
        if (!available()) {
            return std::unexpected("homing is not configured");
        }
        if (!callbacks.stopRequested || !callbacks.prepareTriggeredMove
            || !callbacks.serviceImmediate || !callbacks.advanceServiceMotionPeriod
            || !callbacks.waitForServiceMotion) {
            return std::unexpected("homing runtime callbacks are incomplete");
        }

        m_observation = {.machinePosition = startingPosition};
        m_nextRequest = 1;
        m_nextChunk = 1;
        m_branch = 0;
        m_nextMove = 1;
        m_stopRequested = false;
        ServicedMotionOperation operation(
            m_backend, m_demand, {
                .serviceImmediate = callbacks.serviceImmediate,
                .advanceServiceMotionPeriod = callbacks.advanceServiceMotionPeriod,
                .waitForServiceMotion = callbacks.waitForServiceMotion,
                .observeSnapshot = [&](const ExecutionSnapshot &snapshot,
                                       const std::uint64_t servoTicks) {
                    observeSnapshot(snapshot, servoTicks, callbacks);
                },
                .stopRuntime = callbacks.stopRuntime,
                .faultSession = callbacks.faultSession,
            });

        JointMask allJoints = 0;
        JointVector initial;
        for (const auto &joint : m_joints) {
            allJoints |= JointMask {1} << joint.id;
            initial[joint.id] =
                machineAxisPositionComponent(startingPosition, joint.axis) * joint.coordinateScale;
        }

        const auto begun = operation.begin(epoch, ExecutorDemandMode::Idle);
        if (!begun) {
            return std::unexpected("failed to initialize the motion backend for homing");
        }
        const auto initialized = setJointPositions(allJoints, initial, operation);
        if (!initialized || !*initialized) {
            return std::unexpected(initialized
                ? "failed to initialize the motion backend for homing"
                : initialized.error());
        }
        if (!operation.requestDemand(ExecutorDemandMode::Run)) {
            return std::unexpected(
                "failed to start the motion backend for homing");
        }
        operation.motionMayBeActive();

        for (const auto &group : m_homing.groups) {
            auto fast = makeMove(group, epoch, true, false, false);
            assignJointPositionEnvelope(
                fast, m_observation.joints.position);
            const auto fastResult = executeMove(fast, operation, callbacks);
            if (!fastResult) {
                return operation.finish<HomingResult>(
                    std::unexpected(fastResult.error()));
            }
            if (fastResult->status == TriggeredMoveStatus::Aborted) {
                return operation.finish<HomingResult>(
                    result(HomingOutcome::Stopped, 0));
            }

            auto backoff = makeMove(group, epoch, false, false, true);
            backoff.targetMode = JointTargetMode::Absolute;
            for (const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if (!joint) {
                    continue;
                }
                const auto searchDirection = std::copysign(
                    1.0, joint->homing.searchVelocity * joint->coordinateScale);
                backoff.target[id] = fastResult->triggerState.position[id]
                    - searchDirection * joint->homing.backoffDistance
                        * std::abs(joint->coordinateScale);
            }
            assignJointPositionEnvelope(
                backoff, fastResult->stoppedState.position);
            const auto backoffResult = executeMove(backoff, operation, callbacks);
            if (!backoffResult) {
                return operation.finish<HomingResult>(
                    std::unexpected(backoffResult.error()));
            }
            if (backoffResult->status == TriggeredMoveStatus::Aborted) {
                return operation.finish<HomingResult>(
                    result(HomingOutcome::Stopped, 0));
            }
            if (backoffResult->status != TriggeredMoveStatus::ReachedTarget) {
                return operation.finish<HomingResult>(std::unexpected(
                    "fixed homing backoff did not complete"));
            }
            for (const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if (!joint) {
                    continue;
                }
                const auto searchDirection = std::copysign(
                    1.0, joint->homing.searchVelocity * joint->coordinateScale);
                if (searchDirection
                    * (backoffResult->stoppedState.position[id]
                       - fastResult->triggerState.position[id]) >= 0.0) {
                    return operation.finish<HomingResult>(std::unexpected(
                        "homing backoff did not move behind the fast trigger"));
                }
            }

            auto releaseCheck = makeReleaseCheck(
                group, epoch, backoffResult->stoppedState);
            assignJointPositionEnvelope(
                releaseCheck, backoffResult->stoppedState.position);
            const auto releaseResult = executeMove(
                releaseCheck, operation, callbacks);
            if (!releaseResult) {
                return operation.finish<HomingResult>(
                    std::unexpected(releaseResult.error()));
            }
            if (releaseResult->status == TriggeredMoveStatus::Aborted) {
                return operation.finish<HomingResult>(
                    result(HomingOutcome::Stopped, 0));
            }

            auto slow = makeMove(group, epoch, true, true, false);
            assignJointPositionEnvelope(
                slow, releaseResult->stoppedState.position);
            const auto slowResult = executeMove(slow, operation, callbacks);
            if (!slowResult) {
                return operation.finish<HomingResult>(
                    std::unexpected(slowResult.error()));
            }
            if (slowResult->status == TriggeredMoveStatus::Aborted) {
                return operation.finish<HomingResult>(
                    result(HomingOutcome::Stopped, 0));
            }

            auto calibrated = slowResult->stoppedState.position;
            for (const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if (!joint) {
                    continue;
                }
                const auto desiredSwitch =
                    joint->homing.switchPosition * joint->coordinateScale;
                calibrated[id] += desiredSwitch - slowResult->triggerState.position[id];
            }
            const auto established = setJointPositions(
                slow.triggerRequired ? slow.joints : 0, calibrated, operation);
            if (!established || !*established) {
                return operation.finish<HomingResult>(std::unexpected(established
                    ? "failed to establish joint coordinates after slow homing search"
                    : established.error()));
            }

            auto finalMove = makeMove(group, epoch, false, false, false);
            assignJointPositionEnvelope(
                finalMove, m_observation.joints.position);
            const auto finalResult = executeMove(finalMove, operation, callbacks);
            if (!finalResult) {
                return operation.finish<HomingResult>(
                    std::unexpected(finalResult.error()));
            }
            if (finalResult->status == TriggeredMoveStatus::Aborted) {
                return operation.finish<HomingResult>(
                    result(HomingOutcome::Stopped, 0));
            }
            if (finalResult->status != TriggeredMoveStatus::ReachedTarget) {
                return operation.finish<HomingResult>(std::unexpected(
                    "final move to the configured home position did not complete"));
            }
        }

        return operation.finish<HomingResult>(
            result(HomingOutcome::Completed, allJoints));
    }

    const JointConfiguration *HomingController::configuredJoint(const JointId id) const {
        const auto found = std::ranges::find(m_joints, id, &JointConfiguration::id);

        return found == m_joints.end() ? nullptr : &*found;
    }

    TriggeredJointMove HomingController::makeMove(
        const HomingGroupConfiguration &group, const EpochId epoch,
        const bool triggered, const bool slow, const bool backoff) {
        TriggeredJointMove move;
        move.epoch = epoch;
        move.id = m_nextChunk++;
        move.predecessorBranch = m_branch;
        move.branch = ++m_branch;
        move.moveId = m_nextMove++;
        move.targetMode =
            backoff || triggered ? JointTargetMode::Relative : JointTargetMode::Absolute;
        for (const auto id : group.joints) {
            const auto *joint = configuredJoint(id);
            if (!joint) {
                continue;
            }

            move.joints |= JointMask {1} << id;
            const auto scale = joint->coordinateScale;
            const auto searchDirection =
                std::copysign(1.0, joint->homing.searchVelocity * scale);
            const auto coordinateRange = jointCoordinateRange(*joint);
            const auto range = coordinateRange.maximum - coordinateRange.minimum;
            if (backoff) {
                move.target[id] =
                    -searchDirection * joint->homing.backoffDistance * std::abs(scale);
            } else if (triggered) {
                move.target[id] = searchDirection * (slow
                    ? std::max(2.0 * joint->homing.backoffDistance,
                               0.01) * std::abs(scale)
                    : range + 2.0 * joint->homing.backoffDistance * std::abs(scale));
            } else {
                move.target[id] = joint->homing.homePosition * scale;
            }

            const auto phaseVelocity = backoff
                ? std::abs(joint->homing.searchVelocity * scale)
                : slow ? std::abs(joint->homing.latchVelocity * scale)
                : triggered ? std::abs(joint->homing.searchVelocity * scale)
                : joint->homing.finalVelocity == 0.0
                    ? joint->maxVelocity
                    : std::abs(joint->homing.finalVelocity * scale);
            move.limits.velocity[id] = std::min(joint->maxVelocity, phaseVelocity);
            move.limits.acceleration[id] = joint->maxAcceleration;
            move.limits.jerk[id] = joint->maxJerk;
            if (triggered) {
                (void)move.triggers.push({
                    id, joint->homing.input, joint->homing.condition,
                });
            }
        }
        move.triggerRequired = triggered;

        return move;
    }

    TriggeredJointMove HomingController::makeReleaseCheck(
        const HomingGroupConfiguration &group, const EpochId epoch,
        const JointMotionState &state) {
        auto move = makeMove(group, epoch, false, false, true);
        move.targetMode = JointTargetMode::Absolute;
        move.triggerRequired = true;
        move.checkTriggersAtStart = true;
        for (const auto id : group.joints) {
            const auto *joint = configuredJoint(id);
            if (!joint) {
                continue;
            }

            move.target[id] = state.position[id];
            (void)move.triggers.push({
                id, joint->homing.input,
                releasedCondition(joint->homing.condition),
            });
        }

        return move;
    }

    std::expected<bool, std::string> HomingController::setJointPositions(
        const JointMask joints, const JointVector &positions,
        ServicedMotionOperation &operation) {
        const auto request = m_nextRequest++;
        if (!operation.submit(SetJointPositionRequest {request, joints, positions})) {
            return false;
        }

        const auto event = operation.serviceUntil(
            [&](const ExecutionEvent &candidate) {
                const auto *completed = std::get_if<RequestCompleted>(&candidate);

                return completed && completed->request == request;
            },
            "joint-coordinate establishment exceeded its bounded service iteration limit");
        if (!event) {
            auto error = event.error();
            if (error.starts_with("motion backend fault ")) {
                error.replace(0, std::string("motion backend").size(), "homing backend");
            }

            return std::unexpected(std::move(error));
        }

        return std::get<RequestCompleted>(*event).succeeded;
    }

    std::expected<TriggeredJointMoveCompleted, std::string> HomingController::executeMove(
        const TriggeredJointMove &move, ServicedMotionOperation &operation,
        const HomingRuntimeCallbacks &callbacks) {
        operation.discardPendingEvents();
        if (!callbacks.prepareTriggeredMove(move)) {
            return std::unexpected("homing runtime failed to prepare a triggered move");
        }
        if (!operation.publish(ExecutionItem {move})) {
            return std::unexpected("motion backend rejected a homing move");
        }
        operation.motionMayBeActive();
        const auto event = operation.serviceUntil(
            [&](const ExecutionEvent &candidate) {
                const auto *completed =
                    std::get_if<TriggeredJointMoveCompleted>(&candidate);

                return completed && completed->move == move.moveId;
            },
            [&]() -> std::optional<std::string> {
                if (callbacks.stopRequested() && !m_stopRequested) {
                    if (!operation.requestDemand(ExecutorDemandMode::Stop)) {
                        return "motion backend demand mailbox rejected the homing stop";
                    }
                    m_stopRequested = true;
                }

                return std::nullopt;
            },
            "homing move exceeded its bounded service iteration limit");
        if (!event) {
            auto error = event.error();
            if (error.starts_with("motion backend fault ")) {
                error.replace(0, std::string("motion backend").size(), "homing backend");
            }

            return std::unexpected(std::move(error));
        }

        const auto completed = std::get<TriggeredJointMoveCompleted>(*event);
        operation.observeTerminalJoints(
            completed.stoppedState,
            completed.status == TriggeredMoveStatus::Fault
                ? BackendState::Faulted : BackendState::Held);
        if (completed.status == TriggeredMoveStatus::Fault) {
            return std::unexpected("homing backend fault while completing a move");
        }

        return completed;
    }

    void HomingController::observeSnapshot(
        const ExecutionSnapshot &snapshot, const std::uint64_t servoTicks,
        const HomingRuntimeCallbacks &callbacks) {
        m_observation.joints = snapshot.commandedJoints;
        for (const auto &axis : m_axes) {
            double sum = 0.0;
            std::size_t count = 0;
            for (const auto id : axis.joints) {
                const auto *joint = configuredJoint(id);
                if (!joint || std::abs(joint->coordinateScale) <= 1e-12) {
                    continue;
                }
                sum += snapshot.commandedJoints.position[id] / joint->coordinateScale;
                ++count;
            }
            if (count != 0) {
                machineAxisPositionComponent(m_observation.machinePosition, axis.axis) = sum / count;
            }
        }
        m_observation.commandProgress = snapshot.spanProgress;
        m_observation.servoTicks = servoTicks;
        m_observation.hasActiveMotion =
            snapshot.state == BackendState::Running && snapshot.activeJoints != 0;
        m_observation.backendState = snapshot.state;
        m_observation.backendFaultCode = snapshot.faultCode;
        if (callbacks.observe) {
            callbacks.observe(m_observation);
        }
    }

    HomingResult HomingController::result(
        const HomingOutcome outcome, const JointMask homedJoints) const {
        return {
            .outcome = outcome,
            .observation = m_observation,
            .homedJoints = homedJoints,
        };
    }
}
