#include "machine/JoggingController.h"

#include <cmath>
#include <limits>
#include <ranges>
#include <type_traits>

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

        std::optional<JogId> jogId(const ControlRequest &request) {
            return std::visit([](const auto &value) -> std::optional<JogId> {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::same_as<T, StartContinuousJogRequest>
                              || std::same_as<T, StartIncrementalJogRequest>) {
                    return value.jog;
                }

                return std::nullopt;
            }, request);
        }

        RequestId requestId(const ControlRequest &request) {
            return std::visit([](const auto &value) {
                return value.id;
            }, request);
        }
    }

    JoggingController::JoggingController(std::vector<AxisConfiguration> axes,
                                         std::vector<JointConfiguration> joints,
                                         MotionBackend &backend,
                                         ExecutorDemandController &demand)
        : m_axes(std::move(axes)), m_joints(std::move(joints)),
          m_backend(backend), m_demand(demand) { }

    std::expected<JoggingResult, std::string> JoggingController::run(
        const EpochId epoch, const position_t &startingPosition,
        const ControlRequest &firstRequest,
        const std::function<std::optional<ControlRequest>()> &nextControl,
        const JoggingRuntimeCallbacks &callbacks) {
        const auto jog = jogId(firstRequest);
        if (!jog || *jog == 0) {
            return std::unexpected("jogging requires a nonzero start-jog token");
        }
        if (!nextControl || !callbacks.shutdownRequested || !callbacks.serviceImmediate
            || !callbacks.advanceServiceMotionPeriod || !callbacks.waitForServiceMotion) {
            return std::unexpected("jogging runtime callbacks are incomplete");
        }

        m_backend.discardPendingOutput();
        m_observation = {.machinePosition = startingPosition};
        m_nextRequest = std::numeric_limits<RequestId>::max() - 16;

        JointMask allJoints = 0;
        JointVector initial;
        for (const auto &joint : m_joints) {
            allJoints |= JointMask {1} << joint.id;
            initial[joint.id] =
                axisComponent(startingPosition, joint.axis) * joint.coordinateScale;
        }

        if (!m_demand.request(epoch, ExecutorDemandMode::Jog)) {
            return std::unexpected("failed to initialize the motion backend for jogging");
        }
        callbacks.serviceImmediate();
        const auto initialized = setJointPositions(allJoints, initial, callbacks);
        if (!initialized || !*initialized) {
            return std::unexpected(initialized
                ? "failed to initialize the motion backend for jogging"
                : initialized.error());
        }
        if (!submitControl(firstRequest, callbacks)) {
            return std::unexpected("motion backend control channel is full while starting jogging");
        }

        const auto firstRequestId = requestId(firstRequest);
        bool stopSubmitted = false;
        bool sessionStopped = false;
        for (std::size_t guard = 0; guard < 10000000; ++guard) {
            while (const auto control = nextControl()) {
                if (m_backend.trySubmit(*control) != SubmitResult::Submitted) {
                    return std::unexpected("motion backend jog control channel is full");
                }
            }
            if (callbacks.shutdownRequested() && !stopSubmitted) {
                if (m_backend.trySubmit(StopJogRequest {m_nextRequest--, *jog})
                    != SubmitResult::Submitted) {
                    return std::unexpected(
                        "motion backend control channel is full while stopping jogging");
                }
                stopSubmitted = true;
                sessionStopped = true;
            }

            m_observation.servoTicks += callbacks.advanceServiceMotionPeriod();
            takeSnapshots(callbacks);

            ExecutionEvent event;
            while (m_backend.tryTakeEvent(event)) {
                if (const auto *completed = std::get_if<RequestCompleted>(&event);
                    completed && !completed->succeeded
                    && completed->request == firstRequestId) {
                    return std::unexpected("motion backend rejected the start-jog request");
                }
                if (const auto *stopped = std::get_if<JogStopped>(&event);
                    stopped && stopped->jog == *jog) {
                    m_observation.joints = stopped->jointState;
                    m_observation.hasActiveMotion = false;

                    return JoggingResult {
                        .outcome = sessionStopped
                            ? JoggingOutcome::Stopped : JoggingOutcome::Completed,
                        .observation = m_observation,
                        .stopReason = stopped->reason,
                    };
                }
                if (const auto *fault = std::get_if<BackendFault>(&event)) {
                    return std::unexpected(
                        "jogging backend fault " + std::to_string(fault->code));
                }
            }
            callbacks.waitForServiceMotion();
        }

        return std::unexpected("jogging exceeded its bounded service iteration limit");
    }

    const JointConfiguration *JoggingController::configuredJoint(const JointId id) const {
        const auto found = std::ranges::find(m_joints, id, &JointConfiguration::id);

        return found == m_joints.end() ? nullptr : &*found;
    }

    bool JoggingController::submitControl(
        const ControlRequest &request, const JoggingRuntimeCallbacks &callbacks) {
        if (m_backend.trySubmit(request) != SubmitResult::Submitted) {
            return false;
        }
        callbacks.serviceImmediate();

        return true;
    }

    std::expected<bool, std::string> JoggingController::setJointPositions(
        const JointMask joints, const JointVector &positions,
        const JoggingRuntimeCallbacks &callbacks) {
        const auto request = m_nextRequest--;
        if (!submitControl(SetJointPositionRequest {request, joints, positions}, callbacks)) {
            return false;
        }

        for (std::size_t guard = 0; guard < 10000000; ++guard) {
            ExecutionEvent event;
            while (m_backend.tryTakeEvent(event)) {
                if (const auto *completed = std::get_if<RequestCompleted>(&event);
                    completed && completed->request == request) {
                    takeSnapshots(callbacks);

                    return completed->succeeded;
                }
                if (const auto *fault = std::get_if<BackendFault>(&event)) {
                    return std::unexpected(
                        "jogging backend fault " + std::to_string(fault->code));
                }
            }
            m_observation.servoTicks += callbacks.advanceServiceMotionPeriod();
            takeSnapshots(callbacks);
            callbacks.waitForServiceMotion();
        }

        return std::unexpected(
            "joint-coordinate initialization exceeded its bounded service iteration limit");
    }

    void JoggingController::takeSnapshots(const JoggingRuntimeCallbacks &callbacks) {
        ExecutionSnapshot snapshot;
        while (m_backend.tryTakeSnapshot(snapshot)) {
            observeSnapshot(snapshot, callbacks);
        }
    }

    void JoggingController::observeSnapshot(
        const ExecutionSnapshot &snapshot, const JoggingRuntimeCallbacks &callbacks) {
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
}
