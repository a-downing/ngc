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

        m_observation = {.machinePosition = startingPosition};
        m_nextRequest = std::numeric_limits<RequestId>::max() - 16;
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
                axisComponent(startingPosition, joint.axis) * joint.coordinateScale;
        }

        const auto begun = operation.begin(epoch, ExecutorDemandMode::Jog);
        if (!begun) {
            return std::unexpected("failed to initialize the motion backend for jogging");
        }
        const auto initialized = setJointPositions(allJoints, initial, operation);
        if (!initialized || !*initialized) {
            return std::unexpected(initialized
                ? "failed to initialize the motion backend for jogging"
                : initialized.error());
        }
        if (!operation.submit(firstRequest)) {
            return std::unexpected("motion backend control channel is full while starting jogging");
        }
        operation.motionMayBeActive();

        const auto firstRequestId = requestId(firstRequest);
        bool stopSubmitted = false;
        bool sessionStopped = false;
        const auto event = operation.serviceUntil(
            [&](const ExecutionEvent &candidate) {
                const auto *completed = std::get_if<RequestCompleted>(&candidate);
                if (completed && !completed->succeeded
                    && completed->request == firstRequestId) {
                    return true;
                }
                const auto *stopped = std::get_if<JogStopped>(&candidate);

                return stopped && stopped->jog == *jog;
            },
            [&]() -> std::optional<std::string> {
                while (const auto control = nextControl()) {
                    if (!operation.submit(*control)) {
                        return "motion backend jog control channel is full";
                    }
                }
                if (callbacks.shutdownRequested() && !stopSubmitted) {
                    if (!operation.submit(StopJogRequest {m_nextRequest--, *jog})) {
                        return "motion backend control channel is full while stopping jogging";
                    }
                    stopSubmitted = true;
                    sessionStopped = true;
                }

                return std::nullopt;
            },
            "jogging exceeded its bounded service iteration limit");
        if (!event) {
            auto error = event.error();
            if (error.starts_with("motion backend fault ")) {
                error.replace(0, std::string("motion backend").size(), "jogging backend");
            }

            return operation.finish<JoggingResult>(std::unexpected(std::move(error)));
        }
        if (const auto *completed = std::get_if<RequestCompleted>(&*event)) {
            return operation.finish<JoggingResult>(std::unexpected(
                completed->succeeded
                    ? "jogging ended without a terminal jog event"
                    : "motion backend rejected the start-jog request"));
        }

        const auto &stopped = std::get<JogStopped>(*event);
        const auto terminalState = stopped.reason == JogStopReason::Fault
            ? BackendState::Faulted
            : stopped.reason == JogStopReason::Disabled
                ? BackendState::Disabled : BackendState::Held;
        operation.observeTerminalJoints(stopped.jointState, terminalState);
        m_observation.hasActiveMotion = false;
        if (stopped.reason == JogStopReason::Fault) {
            return operation.finish<JoggingResult>(std::unexpected(
                "jogging backend fault while stopping the active jog"));
        }

        return operation.finish<JoggingResult>(JoggingResult {
            .outcome = sessionStopped
                ? JoggingOutcome::Stopped : JoggingOutcome::Completed,
            .observation = m_observation,
            .stopReason = stopped.reason,
        });
    }

    const JointConfiguration *JoggingController::configuredJoint(const JointId id) const {
        const auto found = std::ranges::find(m_joints, id, &JointConfiguration::id);

        return found == m_joints.end() ? nullptr : &*found;
    }

    std::expected<bool, std::string> JoggingController::setJointPositions(
        const JointMask joints, const JointVector &positions,
        ServicedMotionOperation &operation) {
        const auto request = m_nextRequest--;
        if (!operation.submit(SetJointPositionRequest {request, joints, positions})) {
            return false;
        }

        const auto event = operation.serviceUntil(
            [&](const ExecutionEvent &candidate) {
                const auto *completed = std::get_if<RequestCompleted>(&candidate);

                return completed && completed->request == request;
            },
            "joint-coordinate initialization exceeded its bounded service iteration limit");
        if (!event) {
            auto error = event.error();
            if (error.starts_with("motion backend fault ")) {
                error.replace(0, std::string("motion backend").size(), "jogging backend");
            }

            return std::unexpected(std::move(error));
        }

        return std::get<RequestCompleted>(*event).succeeded;
    }

    void JoggingController::observeSnapshot(
        const ExecutionSnapshot &snapshot, const std::uint64_t servoTicks,
        const JoggingRuntimeCallbacks &callbacks) {
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
        m_observation.servoTicks = servoTicks;
        m_observation.hasActiveMotion =
            snapshot.state == BackendState::Running && snapshot.activeJoints != 0;
        m_observation.backendState = snapshot.state;
        m_observation.backendFaultCode = snapshot.faultCode;
        if (callbacks.observe) {
            callbacks.observe(m_observation);
        }
    }
}
