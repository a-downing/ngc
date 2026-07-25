#include "machine/MachineSession.h"

#include <stdexcept>

namespace ngc {
    SessionCommandQueue::SessionCommandQueue(const std::size_t capacity) : m_capacity(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("session command queue capacity must be positive");
        }
    }

    bool SessionCommandQueue::tryPush(SessionCommand command) {
        if (m_commands.size() >= m_capacity) {
            return false;
        }

        m_commands.push_back(std::move(command));

        return true;
    }

    std::optional<SessionCommand> SessionCommandQueue::tryPop() {
        if (m_commands.empty()) {
            return std::nullopt;
        }

        auto command = std::move(m_commands.front());
        m_commands.pop_front();

        return command;
    }

    void SessionCommandQueue::clear() noexcept {
        m_commands.clear();
    }

    bool SessionCommandQueue::empty() const noexcept {
        return m_commands.empty();
    }

    std::size_t SessionCommandQueue::size() const noexcept {
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
          m_driver(backend, m_geometryForward, m_geometryFeedback, m_geometryCancelled, limits),
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
        PreparedForwardMessage forward;
        while (m_geometryForward.tryPop(forward)) { }
        PreparedFeedbackMessage feedback;
        while (m_geometryFeedback.tryPop(feedback)) { }

        return diagnostics;
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

    const GeometryStreamProducer *MachineSession::geometryProducer() const noexcept {
        return m_geometryProducer.get();
    }
}
