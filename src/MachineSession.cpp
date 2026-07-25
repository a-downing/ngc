#include "machine/MachineSession.h"

#include <stdexcept>

#include "memory/ParameterStore.h"

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

    bool ExecutionCoordinator::beginPowerOn() noexcept {
        auto expected = MachinePowerState::Off;
        if (!m_powerState.compare_exchange_strong(expected, MachinePowerState::Starting)) {
            return false;
        }

        m_activity.store(MachineActivity::Idle);

        return true;
    }

    void ExecutionCoordinator::completePowerOn() noexcept {
        if (m_powerState.load() == MachinePowerState::Starting) {
            m_powerState.store(MachinePowerState::On);
        }
    }

    bool ExecutionCoordinator::beginPowerOff() noexcept {
        if (m_powerState.load() != MachinePowerState::On
            || m_activity.load() != MachineActivity::Idle
            || !m_commands.empty()) {
            return false;
        }

        m_powerState.store(MachinePowerState::Stopping);

        return true;
    }

    void ExecutionCoordinator::completePowerOff() noexcept {
        if (m_powerState.load() == MachinePowerState::Stopping) {
            m_powerState.store(MachinePowerState::Off);
        }
    }

    bool ExecutionCoordinator::powerOn() noexcept {
        if (!beginPowerOn()) {
            return false;
        }

        completePowerOn();

        return true;
    }

    bool ExecutionCoordinator::powerOff() noexcept {
        if (m_powerState.load() == MachinePowerState::Off) {
            return true;
        }
        if (!beginPowerOff()) {
            return false;
        }

        completePowerOff();

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
                                   BackendRuntime &runtime, const TrajectoryLimits &limits,
                                   GeometryStreamPolicy geometryPolicy)
        : m_unit(unit),
          m_interpreter(unit, mode),
          m_geometryPolicy(std::move(geometryPolicy)),
          m_runtime(runtime),
          m_backend(runtime.endpoint()),
          m_driver(m_backend, m_geometryForward, m_geometryFeedback, m_geometryCancelled, limits),
          m_programExecution(m_backend, m_driver, m_coordinator.commands()),
          m_limits(limits) { }

    MachineSession::~MachineSession() {
        (void)finishExecutionEpoch(ExecutionEpochOutcome::Abandoned);
        m_coordinator.commands().clear();
        m_coordinator.finishActivity();
        (void)powerOff();
    }

    bool MachineSession::powerOn() noexcept {
        if (!m_coordinator.beginPowerOn()) {
            return false;
        }

        try {
            m_runtime.start();
        } catch (...) {
            try {
                m_runtime.stop();
            } catch (...) { }
            m_coordinator.fault();

            return false;
        }

        m_coordinator.completePowerOn();

        return true;
    }

    bool MachineSession::powerOff() noexcept {
        if (executionEpochActive()) {
            return false;
        }
        if (m_coordinator.powerState() == MachinePowerState::Off) {
            return true;
        }
        if (!m_coordinator.beginPowerOff()) {
            return false;
        }

        try {
            m_runtime.stop();
        } catch (...) {
            m_coordinator.fault();

            return false;
        }

        m_coordinator.completePowerOff();

        return true;
    }

    bool MachineSession::queueProgram(StartProgram start) {
        if (start.programs.empty()
            || (start.activity != MachineActivity::Program
                && start.activity != MachineActivity::Mdi)) {
            return false;
        }
        if (!m_coordinator.beginActivity(start.activity)) {
            return false;
        }
        if (!m_coordinator.commands().tryPush(std::move(start))) {
            m_coordinator.finishActivity();

            return false;
        }

        return true;
    }

    bool MachineSession::queueHoming() {
        if (!homingAvailable() || !m_coordinator.beginActivity(MachineActivity::Homing)) {
            return false;
        }
        if (!m_coordinator.commands().tryPush(StartHoming {})) {
            m_coordinator.finishActivity();

            return false;
        }

        return true;
    }

    bool MachineSession::queueJog(ControlRequest request) {
        const auto startsJog = std::holds_alternative<StartContinuousJogRequest>(request)
            || std::holds_alternative<StartIncrementalJogRequest>(request);
        if (!startsJog || !m_joggingController
            || !m_coordinator.beginActivity(MachineActivity::Jogging)) {
            return false;
        }
        if (!m_coordinator.commands().tryPush(StartJog { std::move(request) })) {
            m_coordinator.finishActivity();

            return false;
        }

        return true;
    }

    std::expected<std::optional<DispatchedSessionOperation>, std::string>
    MachineSession::dispatchNextOperation() {
        while (auto command = m_coordinator.commands().tryPop()) {
            if (auto *start = std::get_if<StartProgram>(&*command)) {
                if (m_coordinator.activity() != start->activity) {
                    m_coordinator.finishActivity();

                    return std::unexpected(
                        "queued program activity does not own the machine session");
                }

                return DispatchedSessionOperation { std::move(*start) };
            }
            if (auto *mdi = std::get_if<ExecuteMdi>(&*command)) {
                if (m_coordinator.activity() != MachineActivity::Mdi) {
                    m_coordinator.finishActivity();

                    return std::unexpected(
                        "queued MDI activity does not own the machine session");
                }

                return DispatchedSessionOperation {
                    StartProgram {
                        .programs = {{std::move(mdi->program), std::move(mdi->source)}},
                        .preserveState = true,
                        .activity = MachineActivity::Mdi,
                    },
                };
            }
            if (std::holds_alternative<StartHoming>(*command)) {
                if (m_coordinator.activity() != MachineActivity::Homing) {
                    m_coordinator.finishActivity();

                    return std::unexpected(
                        "queued homing activity does not own the machine session");
                }

                return DispatchedSessionOperation { StartHoming {} };
            }
            if (auto *jog = std::get_if<StartJog>(&*command)) {
                if (m_coordinator.activity() != MachineActivity::Jogging) {
                    m_coordinator.finishActivity();

                    return std::unexpected(
                        "queued jogging activity does not own the machine session");
                }

                return DispatchedSessionOperation { StartJog { std::move(jog->request) } };
            }
            if (std::holds_alternative<Stop>(*command)) {
                m_coordinator.finishActivity();

                return DispatchedSessionOperation { Stop {} };
            }
        }

        return std::optional<DispatchedSessionOperation> {};
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
        StartProgram start, const position_t &startingPosition) {
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

    ExecutionEpochFinish MachineSession::finishExecutionEpoch(const ExecutionEpochOutcome outcome) {
        m_geometryCancelled.store(true, std::memory_order_release);
        m_interpreter.requestStop();
        m_geometryForward.notifyAll();
        m_geometryFeedback.notifyAll();
        if (m_geometryThread.joinable()) {
            m_geometryThread.join();
        }

        ExecutionEpochFinish result;
        if (m_geometryProducer) {
            result.diagnostics = m_geometryProducer->diagnostics();
        }
        m_geometryProducer.reset();
        m_programExecution.finish();
        PreparedForwardMessage forward;
        while (m_geometryForward.tryPop(forward)) { }
        PreparedFeedbackMessage feedback;
        while (m_geometryFeedback.tryPop(feedback)) { }
        if (m_coordinator.activity() == MachineActivity::Program
            || m_coordinator.activity() == MachineActivity::Mdi) {
            m_coordinator.finishActivity();
        }

        if (outcome != ExecutionEpochOutcome::Abandoned) {
            if (auto persisted = persistToolTableAtBoundary(); !persisted) {
                result.persistenceError = persisted.error();
            }
        }
        if (outcome == ExecutionEpochOutcome::Completed) {
            if (auto persisted = persistParametersAtBoundary();
                !persisted && !result.persistenceError) {
                result.persistenceError = persisted.error();
            }
        }

        return result;
    }

    bool MachineSession::controllerDataMutable() const noexcept {
        return !executionEpochActive()
            && m_coordinator.activity() == MachineActivity::Idle
            && m_coordinator.commands().empty();
    }

    bool MachineSession::setToolTable(const ToolTable &tools) {
        if (!controllerDataMutable()) {
            return false;
        }

        m_interpreter.machine().toolTable() = tools;
        m_observedToolTable = tools;
        m_toolTableInitialized = true;
        ++m_toolTableRevision;

        return true;
    }

    bool MachineSession::toolTableInitialized() const noexcept {
        return m_toolTableInitialized;
    }

    ToolTable MachineSession::toolTable() const {
        return m_interpreter.machine().toolTable();
    }

    std::pair<ToolTable, std::uint64_t> MachineSession::toolTableSnapshot() const {
        return {m_interpreter.machine().toolTable(), m_toolTableRevision};
    }

    std::expected<void, std::string> MachineSession::setToolTableStorePath(
        const std::filesystem::path &path) {
        if (!controllerDataMutable()) {
            return std::unexpected(
                "cannot configure the tool-table store while motion owns the machine");
        }
        m_toolTableStorePath = path;

        return {};
    }

    std::expected<void, std::string> MachineSession::saveToolTable(
        const std::filesystem::path &path) const {
        if (!controllerDataMutable()) {
            return std::unexpected("cannot save the tool table while motion owns the machine");
        }

        return m_interpreter.machine().toolTable().save(path);
    }

    std::expected<void, std::string> MachineSession::setPersistentParameterStorePath(
        const std::filesystem::path &path) {
        if (!controllerDataMutable()) {
            return std::unexpected(
                "cannot configure persistent parameters while motion owns the machine");
        }
        m_parameterStorePath = path;

        return {};
    }

    std::expected<void, std::string> MachineSession::loadPersistentParameters(
        const std::filesystem::path &path) {
        if (!controllerDataMutable()) {
            return std::unexpected(
                "cannot load persistent parameters while motion owns the machine");
        }

        auto loaded = ngc::loadPersistentParameters(
            path, m_unit, m_interpreter.machine().memory());
        if (loaded) {
            m_parameterStorePath = path;
            m_interpreter.machine().beginProgramRun();
        }

        return loaded;
    }

    std::expected<void, std::string> MachineSession::savePersistentParameters(
        const std::filesystem::path &path) const {
        if (!controllerDataMutable()) {
            return std::unexpected(
                "cannot save persistent parameters while motion owns the machine");
        }

        return ngc::savePersistentParameters(
            path, m_unit, m_interpreter.machine().memory());
    }

    std::expected<void, std::string> MachineSession::persistParametersAtBoundary() const {
        if (!m_parameterStorePath) {
            return {};
        }

        return ngc::savePersistentParameters(
            *m_parameterStorePath, m_unit, m_interpreter.machine().memory());
    }

    std::expected<void, std::string> MachineSession::persistToolTableAtBoundary() {
        const auto &updated = m_interpreter.machine().toolTable();
        if (m_toolTableInitialized && updated == m_observedToolTable) {
            return {};
        }

        m_observedToolTable = updated;
        m_toolTableInitialized = true;
        ++m_toolTableRevision;
        if (!m_toolTableStorePath) {
            return {};
        }

        return updated.save(*m_toolTableStorePath);
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
