#include "machine/HomingController.h"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace ngc {
    namespace {
        double &axisComponent(position_t &position, const Machine::Axis axis) {
            switch (axis) {
                case Machine::Axis::X: return position.x;
                case Machine::Axis::Y: return position.y;
                case Machine::Axis::Z: return position.z;
                case Machine::Axis::A: return position.a;
                case Machine::Axis::B: return position.b;
                case Machine::Axis::C: return position.c;
            }

            return position.x;
        }

        double axisComponent(const position_t &position, const Machine::Axis axis) {
            switch (axis) {
                case Machine::Axis::X: return position.x;
                case Machine::Axis::Y: return position.y;
                case Machine::Axis::Z: return position.z;
                case Machine::Axis::A: return position.a;
                case Machine::Axis::B: return position.b;
                case Machine::Axis::C: return position.c;
            }

            return position.x;
        }
    }

    HomingController::HomingController(std::vector<AxisConfiguration> axes,
                                       std::vector<JointConfiguration> joints,
                                       HomingConfiguration homing, MotionBackend &backend)
        : m_axes(std::move(axes)), m_joints(std::move(joints)),
          m_homing(std::move(homing)), m_backend(backend) { }

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
        m_stopRequest.reset();

        JointMask allJoints = 0;
        JointVector initial;
        for (const auto &joint : m_joints) {
            allJoints |= JointMask {1} << joint.id;
            initial[joint.id] =
                axisComponent(startingPosition, joint.axis) * joint.coordinateScale;
        }

        if (!submitControl(ResetRequest {m_nextRequest++, epoch}, callbacks)
            || !submitControl(EnableRequest {m_nextRequest++}, callbacks)) {
            return std::unexpected("failed to initialize the motion backend for homing");
        }
        const auto initialized = setJointPositions(allJoints, initial, callbacks);
        if (!initialized || !*initialized) {
            return std::unexpected(initialized
                ? "failed to initialize the motion backend for homing"
                : initialized.error());
        }

        for (const auto &group : m_homing.groups) {
            const auto fast = makeMove(group, epoch, true, false, false);
            const auto fastResult = executeMove(fast, callbacks);
            if (!fastResult) {
                return std::unexpected(fastResult.error());
            }
            if (fastResult->status == TriggeredMoveStatus::Aborted) {
                return result(HomingOutcome::Stopped, 0);
            }
            if (fastResult->status != TriggeredMoveStatus::Triggered) {
                return std::unexpected(
                    "fast homing search reached its travel limit before the switch");
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
            const auto backoffResult = executeMove(backoff, callbacks);
            if (!backoffResult) {
                return std::unexpected(backoffResult.error());
            }
            if (backoffResult->status == TriggeredMoveStatus::Aborted) {
                return result(HomingOutcome::Stopped, 0);
            }
            if (backoffResult->status != TriggeredMoveStatus::ReachedTarget) {
                return std::unexpected("fixed homing backoff did not complete");
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
                    return std::unexpected(
                        "homing backoff did not move behind the fast trigger");
                }
            }

            const auto slow = makeMove(group, epoch, true, true, false);
            const auto slowResult = executeMove(slow, callbacks);
            if (!slowResult) {
                return std::unexpected(slowResult.error());
            }
            if (slowResult->status == TriggeredMoveStatus::Aborted) {
                return result(HomingOutcome::Stopped, 0);
            }
            if (slowResult->status != TriggeredMoveStatus::Triggered) {
                return std::unexpected(
                    "slow homing search reached its travel limit before the switch");
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
                slow.triggerRequired ? slow.joints : 0, calibrated, callbacks);
            if (!established || !*established) {
                return std::unexpected(established
                    ? "failed to establish joint coordinates after slow homing search"
                    : established.error());
            }

            const auto finalMove = makeMove(group, epoch, false, false, false);
            const auto finalResult = executeMove(finalMove, callbacks);
            if (!finalResult) {
                return std::unexpected(finalResult.error());
            }
            if (finalResult->status == TriggeredMoveStatus::Aborted) {
                return result(HomingOutcome::Stopped, 0);
            }
            if (finalResult->status != TriggeredMoveStatus::ReachedTarget) {
                return std::unexpected(
                    "final move to the configured home position did not complete");
            }
        }

        return result(HomingOutcome::Completed, allJoints);
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
            const auto range = (joint->maximum - joint->minimum) * std::abs(scale);
            if (backoff) {
                move.target[id] =
                    -searchDirection * joint->homing.backoffDistance * std::abs(scale);
            } else if (triggered) {
                move.target[id] = searchDirection * (slow
                    ? std::max(2.0 * joint->homing.backoffDistance * std::abs(scale), 0.01)
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

    bool HomingController::submitControl(
        const ControlRequest &request, const HomingRuntimeCallbacks &callbacks) {
        if (m_backend.trySubmit(request) != SubmitResult::Submitted) {
            return false;
        }
        callbacks.serviceImmediate();

        return true;
    }

    std::expected<bool, std::string> HomingController::setJointPositions(
        const JointMask joints, const JointVector &positions,
        const HomingRuntimeCallbacks &callbacks) {
        const auto request = m_nextRequest++;
        if (!submitControl(SetJointPositionRequest {request, joints, positions}, callbacks)) {
            return false;
        }

        for (std::size_t guard = 0; guard < 10000000; ++guard) {
            ExecutionEvent event;
            while (m_backend.tryTakeEvent(event)) {
                if (const auto *result = std::get_if<RequestCompleted>(&event);
                    result && result->request == request) {
                    takeSnapshots(callbacks);

                    return result->succeeded;
                }
                if (const auto *fault = std::get_if<BackendFault>(&event)) {
                    return std::unexpected(
                        "homing backend fault " + std::to_string(fault->code));
                }
            }
            m_observation.servoTicks += callbacks.advanceServiceMotionPeriod();
            takeSnapshots(callbacks);
            callbacks.waitForServiceMotion();
        }

        return std::unexpected(
            "joint-coordinate establishment exceeded its bounded service iteration limit");
    }

    std::expected<TriggeredJointMoveCompleted, std::string> HomingController::executeMove(
        const TriggeredJointMove &move, const HomingRuntimeCallbacks &callbacks) {
        ExecutionEvent discarded;
        while (m_backend.tryTakeEvent(discarded)) { }
        if (!callbacks.prepareTriggeredMove(move)) {
            return std::unexpected("homing runtime failed to prepare a triggered move");
        }
        if (m_backend.tryPublish(ExecutionItem {move}) != PublishResult::Published) {
            return std::unexpected("motion backend rejected a homing move");
        }
        if (!submitControl(ResumeRequest {m_nextRequest++, move.epoch}, callbacks)) {
            return std::unexpected("motion backend control channel is full while starting homing");
        }

        for (std::size_t guard = 0; guard < 10000000; ++guard) {
            if (callbacks.stopRequested() && !m_stopRequest) {
                const auto request = m_nextRequest++;
                if (!submitControl(ControlledStopRequest {request}, callbacks)) {
                    return std::unexpected(
                        "motion backend control channel is full while stopping homing");
                }
                m_stopRequest = request;
            }

            m_observation.servoTicks += callbacks.advanceServiceMotionPeriod();
            takeSnapshots(callbacks);

            ExecutionEvent event;
            while (m_backend.tryTakeEvent(event)) {
                if (const auto *completed =
                        std::get_if<TriggeredJointMoveCompleted>(&event);
                    completed && completed->move == move.moveId) {
                    ExecutionSnapshot stopped;
                    stopped.state = BackendState::Held;
                    stopped.commandedJoints = completed->stoppedState;
                    stopped.spanProgress = 1.0;
                    observeSnapshot(stopped, callbacks);

                    return *completed;
                }
                if (const auto *completed = std::get_if<RequestCompleted>(&event);
                    completed && m_stopRequest
                    && completed->request == *m_stopRequest && !completed->succeeded) {
                    return std::unexpected(
                        "motion backend rejected the controlled homing stop");
                }
                if (const auto *fault = std::get_if<BackendFault>(&event)) {
                    return std::unexpected(
                        "homing backend fault " + std::to_string(fault->code));
                }
            }
            callbacks.waitForServiceMotion();
        }

        return std::unexpected("homing move exceeded its bounded service iteration limit");
    }

    void HomingController::takeSnapshots(const HomingRuntimeCallbacks &callbacks) {
        ExecutionSnapshot snapshot;
        while (m_backend.tryTakeSnapshot(snapshot)) {
            observeSnapshot(snapshot, callbacks);
        }
    }

    void HomingController::observeSnapshot(
        const ExecutionSnapshot &snapshot, const HomingRuntimeCallbacks &callbacks) {
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
                axisComponent(m_observation.machinePosition, axis.axis) = sum / count;
            }
        }
        m_observation.commandProgress = snapshot.spanProgress;
        m_observation.hasActiveMotion =
            snapshot.state == BackendState::Running && snapshot.activeJoints != 0;
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
