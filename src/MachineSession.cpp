#include "machine/MachineSession.h"

#include <stdexcept>

namespace ngc {
    SessionCommandQueue::SessionCommandQueue(const std::size_t capacity) : m_capacity(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("session command queue capacity must be positive");
        }
    }

    bool SessionCommandQueue::tryPush(SessionCommand command) {
        std::scoped_lock lock(m_mutex);
        if (m_commands.size() >= m_capacity) {
            return false;
        }

        m_commands.push_back(std::move(command));

        return true;
    }

    std::optional<SessionCommand> SessionCommandQueue::tryPop() {
        std::scoped_lock lock(m_mutex);
        if (m_commands.empty()) {
            return std::nullopt;
        }

        auto command = std::move(m_commands.front());
        m_commands.pop_front();

        return command;
    }

    void SessionCommandQueue::clear() noexcept {
        std::scoped_lock lock(m_mutex);
        m_commands.clear();
    }

    bool SessionCommandQueue::empty() const noexcept {
        std::scoped_lock lock(m_mutex);
        return m_commands.empty();
    }

    std::size_t SessionCommandQueue::size() const noexcept {
        std::scoped_lock lock(m_mutex);
        return m_commands.size();
    }

    std::size_t SessionCommandQueue::capacity() const noexcept {
        return m_capacity;
    }

    ExecutionCoordinator::ExecutionCoordinator(const std::size_t commandCapacity)
        : m_commands(commandCapacity) { }

    bool ExecutionCoordinator::powerOn() noexcept {
        auto expected = MachinePowerState::Off;
        if (!m_powerState.compare_exchange_strong(expected, MachinePowerState::Starting)) {
            return false;
        }

        m_activity.store(MachineActivity::Idle);
        m_powerState.store(MachinePowerState::On);

        return true;
    }

    bool ExecutionCoordinator::powerOff() noexcept {
        if (m_powerState.load() == MachinePowerState::Off) {
            return true;
        }
        if (m_powerState.load() != MachinePowerState::On
            || m_activity.load() != MachineActivity::Idle
            || !m_commands.empty()) {
            return false;
        }

        m_powerState.store(MachinePowerState::Stopping);
        m_powerState.store(MachinePowerState::Off);

        return true;
    }

    bool ExecutionCoordinator::beginActivity(const MachineActivity activity) noexcept {
        if (m_powerState.load() != MachinePowerState::On) {
            return false;
        }
        if (activity == MachineActivity::Idle || activity == MachineActivity::Faulted
            || activity == MachineActivity::Stopping) {
            return false;
        }

        auto expected = MachineActivity::Idle;
        if (!m_activity.compare_exchange_strong(expected, activity)) {
            return false;
        }

        return true;
    }

    void ExecutionCoordinator::setActivity(const MachineActivity activity) noexcept {
        m_activity.store(activity);
    }

    void ExecutionCoordinator::finishActivity() noexcept {
        if (m_powerState.load() == MachinePowerState::On) {
            m_activity.store(MachineActivity::Idle);
        }
    }

    void ExecutionCoordinator::fault() noexcept {
        m_powerState.store(MachinePowerState::Faulted);
        m_activity.store(MachineActivity::Faulted);
    }

    MachinePowerState ExecutionCoordinator::powerState() const noexcept {
        return m_powerState.load();
    }

    MachineActivity ExecutionCoordinator::activity() const noexcept {
        return m_activity.load();
    }

    MachineSession::MachineSession(const Machine::Unit unit, const InterpretationMode mode,
                                   MotionBackend &backend, const TrajectoryLimits &limits,
                                   GeometryStreamPolicy geometryPolicy)
        : m_interpreter(unit, mode),
          m_geometryPolicy(std::move(geometryPolicy)),
          m_backend(backend),
          m_driver(backend, m_geometryForward, m_geometryFeedback, m_geometryCancelled, limits),
          m_programExecution(backend, m_driver, m_coordinator.commands()),
          m_limits(limits) { }

    MachineSession::~MachineSession() {
        (void)finishExecutionEpoch();
    }

    bool MachineSession::powerOn() noexcept {
        return m_coordinator.powerOn();
    }

    bool MachineSession::powerOff() noexcept {
        if (executionEpochActive()) {
            return false;
        }

        return m_coordinator.powerOff();
    }

    ProgramOperationUpdate MachineSession::programOperationUpdate() const {
        ProgramOperationUpdate result;
        result.stoppedPosition = m_programExecution.stoppedPosition();
        const auto controlState = m_programExecution.state();
        const auto driverState = m_driver.state();
        if (controlState == ProgramExecutionState::Error) {
            result.state = ProgramOperationState::Error;
            result.error = m_programExecution.error().value_or(
                "machine-session program execution failed");
        } else if (controlState == ProgramExecutionState::StopComplete) {
            result.state = ProgramOperationState::StopComplete;
        } else if (driverState == PreparedDriverState::Error) {
            result.state = ProgramOperationState::Error;
            result.error = m_driver.error().value_or(
                "prepared trajectory execution failed");
        } else if (driverState == PreparedDriverState::Completed) {
            result.state = ProgramOperationState::Completed;
        } else if (controlState == ProgramExecutionState::Paused
                   || driverState == PreparedDriverState::ProgramPaused) {
            result.state = ProgramOperationState::Paused;
        } else if (controlState == ProgramExecutionState::Holding) {
            result.state = ProgramOperationState::Holding;
        }

        return result;
    }

    std::expected<GeometryEpoch, std::string> MachineSession::beginExecutionEpoch(
        StartProgram start, ToolTable tools, const position_t &startingPosition) {
        if (executionEpochActive()) {
            return std::unexpected("a program or MDI execution epoch is already active");
        }
        if (m_coordinator.powerState() != MachinePowerState::On) {
            return std::unexpected("the machine session is not powered on");
        }
        if (m_coordinator.activity() == MachineActivity::Idle
            && !m_coordinator.beginActivity(start.activity)) {
            return std::unexpected("the machine session rejected the requested activity");
        }
        if (m_coordinator.activity() != start.activity) {
            return std::unexpected("another machine-session activity owns motion");
        }

        m_interpreter.setPrograms(start.programs);
        m_interpreter.machine().toolTable() = std::move(tools);
        m_interpreter.compile([](const auto &callback) { callback(); });
        if (start.preserveState) {
            m_interpreter.beginContinuation();
        } else {
            m_interpreter.begin();
        }

        m_geometryCancelled.store(false, std::memory_order_release);
        m_driver.setLimits(m_limits);
        const auto epoch = nextEpoch();
        if (!m_driver.begin(epoch, startingPosition)) {
            m_interpreter.reportError(
                "trajectory driver failed to initialize its backend control channels");
            m_interpreter.stop();
            m_coordinator.finishActivity();

            return std::unexpected("motion backend control channel is full");
        }
        m_programExecution.begin(epoch);

        m_geometryProducer = std::make_unique<GeometryStreamProducer>(
            m_interpreter, m_geometryForward, m_geometryFeedback, m_geometryCancelled,
            m_geometryPolicy);
        m_geometryThread = std::thread([this, epoch] {
            (void)m_geometryProducer->run(epoch);
        });

        return epoch;
    }

    GeometryStreamDiagnostics MachineSession::finishExecutionEpoch() {
        m_geometryCancelled.store(true, std::memory_order_release);
        m_interpreter.requestStop();
        m_geometryForward.notifyAll();
        m_geometryFeedback.notifyAll();
        if (m_geometryThread.joinable()) {
            m_geometryThread.join();
        }

        GeometryStreamDiagnostics diagnostics;
        if (m_geometryProducer) {
            diagnostics = m_geometryProducer->diagnostics();
        }
        m_geometryProducer.reset();
        m_programExecution.finish();
        PreparedForwardMessage forward;
        while (m_geometryForward.tryPop(forward)) { }
        PreparedFeedbackMessage feedback;
        while (m_geometryFeedback.tryPop(feedback)) { }

        return diagnostics;
    }

    void MachineSession::configureHoming(std::vector<AxisConfiguration> axes,
                                         std::vector<JointConfiguration> joints,
                                         HomingConfiguration homing) {
        m_homingController = std::make_unique<HomingController>(
            std::move(axes), std::move(joints), std::move(homing), m_backend);
    }

    void MachineSession::configureJogging(std::vector<AxisConfiguration> axes,
                                          std::vector<JointConfiguration> joints) {
        m_joggingController = std::make_unique<JoggingController>(
            std::move(axes), std::move(joints), m_backend);
    }

    bool MachineSession::homingAvailable() const noexcept {
        return m_homingController && m_homingController->available();
    }

    std::expected<HomingResult, std::string> MachineSession::runHoming(
        const position_t &startingPosition, const HomingRuntimeCallbacks &callbacks) {
        if (!m_homingController) {
            return std::unexpected("homing is not configured");
        }
        if (m_coordinator.activity() != MachineActivity::Homing) {
            return std::unexpected("homing does not own the machine session");
        }

        const auto homing = m_homingController->run(nextEpoch(), startingPosition, callbacks);
        if (homing) {
            m_interpreter.machine().synchronizePosition(homing->observation.machinePosition);
            if (homing->outcome == HomingOutcome::Completed) {
                m_homedJoints = homing->homedJoints;
            }
        }
        m_coordinator.finishActivity();

        return homing;
    }

    JointMask MachineSession::homedJoints() const noexcept {
        return m_homedJoints;
    }

    std::expected<JoggingResult, std::string> MachineSession::runJogging(
        const position_t &startingPosition, const ControlRequest &firstRequest,
        const JoggingRuntimeCallbacks &callbacks) {
        if (!m_joggingController) {
            return std::unexpected("jogging is not configured");
        }
        if (m_coordinator.activity() != MachineActivity::Jogging) {
            return std::unexpected("jogging does not own the machine session");
        }

        bool sessionStopRequested = false;
        const auto nextControl = [&]() -> std::optional<ControlRequest> {
            while (const auto command = m_coordinator.commands().tryPop()) {
                if (const auto *renew = std::get_if<RenewJog>(&*command)) {
                    return RenewJogLeaseRequest {renew->request, renew->jog};
                }
                if (const auto *update = std::get_if<SetJogVelocity>(&*command)) {
                    return update->request;
                }
                if (const auto *stop = std::get_if<StopJog>(&*command)) {
                    return StopJogRequest {stop->request, stop->jog};
                }
                if (std::holds_alternative<Stop>(*command)) {
                    sessionStopRequested = true;
                }
            }

            return std::nullopt;
        };
        auto controllerCallbacks = callbacks;
        controllerCallbacks.shutdownRequested = [&] {
            return sessionStopRequested
                || (callbacks.shutdownRequested && callbacks.shutdownRequested());
        };
        const auto result = m_joggingController->run(
            nextEpoch(), startingPosition, firstRequest, nextControl, controllerCallbacks);
        if (result) {
            m_interpreter.machine().synchronizePosition(result->observation.machinePosition);
        }
        m_coordinator.finishActivity();

        return result;
    }

    void MachineSession::requestGeometryStop() {
        m_interpreter.requestStop();
        m_geometryForward.notifyAll();
        m_geometryFeedback.notifyAll();
    }

    bool MachineSession::executionEpochActive() const noexcept {
        return m_geometryProducer != nullptr || m_geometryThread.joinable();
    }

    GeometryEpoch MachineSession::nextEpoch() noexcept {
        return m_nextEpoch++;
    }

    bool MachineSession::applyProgramPresentationUpdate() {
        auto presentation = m_driver.takePresentationUpdate();
        if (!presentation) {
            return false;
        }
        m_presentationTracker.setActivePresentation(*presentation);

        return true;
    }

    void MachineSession::completeProgramPresentation() {
        m_presentationTracker.completeDeferredBlocks();
    }

    const GeometryStreamProducer *MachineSession::geometryProducer() const noexcept {
        return m_geometryProducer.get();
    }
}
