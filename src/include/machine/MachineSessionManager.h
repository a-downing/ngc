#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <expected>
#include <limits>
#include <memory>
#include <mutex>
#include <print>
#include <ranges>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <string>
#include <vector>

#include "config/ConfigurationFingerprint.h"
#include "machine/GeometryStreamProducer.h"
#ifdef __linux__
#include "machine/ExternalExecutorRuntime.h"
#endif
#include "machine/InProcessSimulationRuntime.h"
#include "machine/MachineControl.h"
#include "machine/MachineConfiguration.h"
#include "machine/MachineSessionCommand.h"
#include "machine/MachineSession.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"
#include "machine/SimulationPresentation.h"
#include "machine/ToolTable.h"
#include "memory/ParameterStore.h"

namespace ngc {

namespace detail {

class SessionBackendRuntime final : public ngc::BackendRuntime {
#ifdef __linux__
    static ngc::ExternalExecutorRuntimeConfiguration externalConfiguration(
            const ngc::MachineConfiguration &configuration) {
        const auto &backend = *configuration.machineExecutor;
        auto backendFingerprint =
            std::optional<std::uint64_t>{};
        if (backend.backendConfiguration.has_value()) {
            const auto fingerprint =
                ngc::toml_configuration::fileFingerprint(
                    *backend.backendConfiguration);
            if (!fingerprint) {
                throw std::runtime_error(
                    "failed to fingerprint backend "
                    "configuration: " + fingerprint.error());
            }
            backendFingerprint = *fingerprint;
        }
        const ngc::IpcIdentity identity{
            .configurationFingerprint =
                ngc::toml_configuration::combinedFingerprint(
                    configuration.sourceFingerprint,
                    backendFingerprint,
                    backend.servoPeriod),
            .sessionGeneration = 1,
            .epochGeneration = 1,
            .authorityGeneration = 1,
        };

        ngc::ExternalExecutorRuntimeConfiguration result{
            .peerExecutable = backend.executable,
            .identity = identity,
            .peerExpectedIdentity = identity,
            .peerArguments = {
                "--machine-configuration",
                backend.machineConfiguration.string(),
            },
        };
        if (backend.backendConfiguration.has_value()) {
            result.peerArguments.push_back(
                "--backend-configuration");
            result.peerArguments.push_back(
                backend.backendConfiguration->string());
        }

        return result;
    }
#endif

    static std::unique_ptr<ngc::BackendRuntime> configuredRuntime(
            const ngc::MachineConfiguration &configuration,
            const bool external) {
#ifdef __linux__
        if (external) {
            return std::make_unique<ngc::ExternalExecutorRuntime>(
                externalConfiguration(configuration));
        }
#else
        if (external) {
            throw std::runtime_error(
                "the external Machine executor is supported only on Linux");
        }
#endif

        return std::make_unique<ngc::InProcessSimulationRuntime>(
            configuration);
    }

public:
    explicit SessionBackendRuntime(
            const ngc::TrajectoryLimits &limits,
            const ngc::SimulationTiming &timing)
        : m_runtime(std::make_unique<ngc::InProcessSimulationRuntime>(
              limits, timing)),
          m_simulation(static_cast<ngc::InProcessSimulationRuntime *>(
              m_runtime.get())),
          m_servoPeriod(timing.servoPeriod) { }

    SessionBackendRuntime(
            const ngc::MachineConfiguration &configuration,
            const bool external)
        : m_runtime(configuredRuntime(configuration, external)),
          m_simulation(external ? nullptr
                                : static_cast<ngc::InProcessSimulationRuntime *>(
                                      m_runtime.get())),
          m_servoPeriod(external
              ? configuration.machineExecutor->servoPeriod
              : configuration.simulation.servoPeriod) { }

    ngc::MotionBackend &endpoint() noexcept override {
        return m_runtime->endpoint();
    }

    void start() override {
        m_runtime->start();
    }

    void stop() override {
        m_runtime->stop();
    }

    [[nodiscard]] ngc::BackendCapabilities capabilities() const noexcept override {
        return m_runtime->capabilities();
    }

    [[nodiscard]] bool restoreStationaryState(
            const ngc::StationaryBackendState &state) noexcept override {
        return m_runtime->restoreStationaryState(state);
    }

    [[nodiscard]] bool prepareTriggeredJointMove(
            const ngc::TriggeredJointMove &move) noexcept override {
        return m_runtime->prepareTriggeredJointMove(move);
    }

    void serviceImmediate() override {
        m_runtime->serviceImmediate();
    }

    [[nodiscard]] std::uint64_t advanceServiceMotionPeriod() override {
        return m_runtime->advanceServiceMotionPeriod();
    }

    void waitForServiceMotion() override {
        m_runtime->waitForServiceMotion();
    }

    bool tryTakeRealtimeTiming(
            ngc::RealtimeTimingSummary &summary) noexcept override {
        return m_runtime->tryTakeRealtimeTiming(summary);
    }

    void requestEmergencyStop(const ngc::EmergencyStopSource source) noexcept override {
        m_runtime->requestEmergencyStop(source);
    }

    void releaseEmergencyStop(const ngc::EmergencyStopSource source) noexcept override {
        m_runtime->releaseEmergencyStop(source);
    }

    [[nodiscard]] std::uint64_t requestEmergencyStopReset() noexcept override {
        return m_runtime->requestEmergencyStopReset();
    }

    [[nodiscard]] ngc::EmergencyStopStatus emergencyStopStatus() const noexcept override {
        return m_runtime->emergencyStopStatus();
    }

    bool beginTimedExecution() {
        return m_simulation ? m_simulation->beginTimedExecution() : true;
    }

    void endTimedExecution() {
        if (m_simulation) {
            m_simulation->endTimedExecution();
        }
    }

    void setTickMultiplier(const int multiplier) noexcept {
        if (m_simulation) {
            m_simulation->setTickMultiplier(multiplier);
        }
    }

    [[nodiscard]] std::uint32_t tickMultiplier() const noexcept {
        return m_simulation ? m_simulation->tickMultiplier() : 1;
    }

    [[nodiscard]] double servoPeriod() const noexcept {
        return m_servoPeriod;
    }

    [[nodiscard]] ngc::SimulationRuntimeSnapshot snapshot() const noexcept {
        if (m_simulation) {
            return m_simulation->snapshot();
        }

        ngc::SimulationRuntimeSnapshot result;
        result.servoPeriodSeconds = m_servoPeriod;
        result.schedulerPeriodSeconds = m_servoPeriod;
        result.servoTicksPerSchedulerPeriod = 1;

        return result;
    }

    [[nodiscard]] bool executorBatchActive() const noexcept {
        return m_simulation && m_simulation->executorBatchActive();
    }

    void setNrtRefillActive(const bool active) noexcept {
        if (m_simulation) {
            m_simulation->setNrtRefillActive(active);
        }
    }

    void releaseRefillOpportunity() noexcept {
        if (m_simulation) {
            m_simulation->releaseRefillOpportunity();
        }
    }

    void setRollingSupplyActive(const bool active) noexcept {
        if (m_simulation) {
            m_simulation->setRollingSupplyActive(active);
        }
    }

    bool configureSyntheticInput(
            const ngc::TriggeredMoveId move,
            const ngc::position_t &position) noexcept {
        return m_simulation
            && m_simulation->configureSyntheticInput(move, position);
    }

    std::vector<ngc::ExecutedJerkSample> takeExecutedJerkSamples() {
        return m_simulation
            ? m_simulation->takeExecutedJerkSamples()
            : std::vector<ngc::ExecutedJerkSample>{};
    }

    [[nodiscard]] bool simulationDiagnosticsAvailable() const noexcept {
        return m_simulation != nullptr;
    }

private:
    std::unique_ptr<ngc::BackendRuntime> m_runtime;
    ngc::InProcessSimulationRuntime *m_simulation = nullptr;
    double m_servoPeriod = 0.001;
};

class MachineSessionHost {
    static ngc::GeometryStreamPolicy geometryPolicy(const ngc::TrajectoryLimits &limits) {
        ngc::GeometryStreamPolicy result;
        result.splineVelocityLimits={
            .pathAcceleration=limits.pathAcceleration,
            .pathJerk=limits.pathJerk,
            .axisVelocity=limits.axisVelocity,
            .axisAcceleration=limits.axisAcceleration,
            .axisJerk=limits.axisJerk,
        };
        return result;
    }

    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;
    mutable std::mutex m_timedSnapshotMutex;
    std::condition_variable m_timedSnapshotCv;
    std::thread m_timedSnapshotThread;
    bool m_stopTimedSnapshotService = false;
    std::optional<ngc::ExecutionSnapshot> m_latestTimedBackendSnapshot;
    std::optional<ngc::RealtimeTimingSummary> m_latestRealtimeTiming;
    SessionBackendRuntime m_runtime;
    ngc::MachineSession m_machineSession;
    ngc::TrajectoryLimits m_limits;
    ngc::SimulationSnapshot m_snapshot;
    bool m_join = false;
    bool m_stop = false;
    bool m_running = false;
    bool m_programRunning = false;
    std::optional<ngc::JogId> m_activeJog;
    std::vector<ngc::AxisConfiguration> m_axes;
    std::vector<ngc::JointConfiguration> m_joints;
    std::uint32_t m_tickMultiplier = 1;
    ngc::ParameterSnapshot m_parameterSnapshot;
    enum class PowerOperation { On, Off };
    std::optional<PowerOperation> m_pendingPowerOperation;
    bool m_powerRequestActive = false;
    bool m_powerResultReady = false;
    SessionCommandResult m_powerResult;
    MachineControlAuthority m_controlAuthority {
        .target = MachineControlTarget::Simulation,
        .generation = 1,
    };

    [[nodiscard]] bool motionOwnedOrQueued() const noexcept {
        return m_running || !m_machineSession.coordinator().commands().empty()
            || m_machineSession.coordinator().activity() != ngc::MachineActivity::Idle
            || m_activeJog.has_value() || m_machineSession.servicedMotionOwned();
    }

    [[nodiscard]] bool hasControlAuthorityLocked(
            const MachineControlAuthority authority) const noexcept {
        return authority == m_controlAuthority;
    }

public:
    explicit MachineSessionHost(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                              const ngc::TrajectoryLimits limits = {},
                              const ngc::SimulationTiming timing = {},
                              const MachineControlTarget target = MachineControlTarget::Simulation)
        : m_runtime(limits, timing),
          m_machineSession(unit, target == MachineControlTarget::Simulation
                                    ? ngc::InterpretationMode::Simulation
                                    : ngc::InterpretationMode::MachineRun, m_runtime,
                           limits, geometryPolicy(limits)),
          m_limits(limits) {
        m_controlAuthority.target = target;
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        refreshParameterSnapshot();
        m_thread = std::thread(&MachineSessionHost::work, this);
    }
    explicit MachineSessionHost(
            const ngc::MachineConfiguration &configuration,
            const MachineControlTarget target = MachineControlTarget::Simulation,
            const bool useConfiguredMachineExecutor = false)
        : m_runtime(configuration, useConfiguredMachineExecutor),
          m_machineSession(configuration.unit,
                           target == MachineControlTarget::Simulation
                               ? ngc::InterpretationMode::Simulation
                               : ngc::InterpretationMode::MachineRun, m_runtime, configuration.trajectory,
                           geometryPolicy(configuration.trajectory)),
          m_limits(configuration.trajectory),
          m_axes(configuration.axes), m_joints(configuration.joints),
          m_tickMultiplier(1) {
        m_controlAuthority.target = target;
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        m_machineSession.configureJogging(configuration.axes, configuration.joints);
        m_machineSession.configureHoming(
            configuration.axes, configuration.joints, configuration.homing);
        clearActiveTool();
        refreshParameterSnapshot();
        m_thread = std::thread(&MachineSessionHost::work, this);
    }
    ~MachineSessionHost() { join(); }
    MachineSessionHost(const MachineSessionHost &) = delete;
    MachineSessionHost &operator=(const MachineSessionHost &) = delete;

    MachineSessionManagerState state() const {
        std::scoped_lock lock(m_mutex);

        return {
            .authority = m_controlAuthority,
            .simulationAvailable =
                m_controlAuthority.target == MachineControlTarget::Simulation,
            .machineAvailable = m_controlAuthority.target == MachineControlTarget::Machine,
        };
    }

    std::expected<MachineControlAuthority, std::string> selectControlTarget(
        const MachineControlTarget target) {
        std::scoped_lock lock(m_mutex);
        if (target != m_controlAuthority.target) {
            return std::unexpected("the requested target belongs to another machine session");
        }

        return m_controlAuthority;
    }

    bool hasControlAuthority(const MachineControlAuthority authority) const {
        std::scoped_lock lock(m_mutex);

        return authority == m_controlAuthority;
    }

    SessionCommandResult powerOn(const MachineControlAuthority authority) {
        std::unique_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        const auto state = m_machineSession.coordinator().powerState();
        if (m_powerRequestActive) {
            return { SessionCommandRejection::PowerTransitionInProgress };
        }
        if (state == ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionAlreadyPowered };
        }
        if (state != ngc::MachinePowerState::Off) {
            return { SessionCommandRejection::CommandUnavailable };
        }

        m_powerRequestActive = true;
        m_powerResultReady = false;
        m_pendingPowerOperation = PowerOperation::On;
        m_snapshot.powerState = ngc::MachinePowerState::Starting;
        m_cv.notify_all();
        m_cv.wait(lock, [&] {
            return m_powerResultReady || m_join;
        });
        if (!m_powerResultReady) {
            m_powerRequestActive = false;
            return { SessionCommandRejection::CommandUnavailable };
        }

        const auto result = m_powerResult;
        m_powerRequestActive = false;
        m_powerResultReady = false;
        m_cv.notify_all();

        return result;
    }

    SessionCommandResult powerOff(const MachineControlAuthority authority) {
        std::unique_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        const auto state = m_machineSession.coordinator().powerState();
        if (m_powerRequestActive) {
            return { SessionCommandRejection::PowerTransitionInProgress };
        }
        if (state == ngc::MachinePowerState::Off) {
            return { SessionCommandRejection::SessionAlreadyOff };
        }
        if (state != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::CommandUnavailable };
        }
        if (motionOwnedOrQueued()) {
            return { SessionCommandRejection::MotionOwned };
        }

        m_powerRequestActive = true;
        m_powerResultReady = false;
        m_pendingPowerOperation = PowerOperation::Off;
        m_snapshot.powerState = ngc::MachinePowerState::Stopping;
        m_cv.notify_all();
        m_cv.wait(lock, [&] {
            return m_powerResultReady || m_join;
        });
        if (!m_powerResultReady) {
            m_powerRequestActive = false;
            return { SessionCommandRejection::CommandUnavailable };
        }

        const auto result = m_powerResult;
        m_powerRequestActive = false;
        m_powerResultReady = false;
        m_cv.notify_all();

        return result;
    }

    bool emergencyStop() {
        std::scoped_lock lock(m_mutex);
        const auto state = m_machineSession.coordinator().powerState();
        if (state == ngc::MachinePowerState::Off) {
            return false;
        }

        m_runtime.requestEmergencyStop(ngc::EmergencyStopSource::Gui);
        m_machineSession.coordinator().commands().clear();
        m_machineSession.coordinator().fault();
        m_snapshot.powerState = ngc::MachinePowerState::Faulted;
        m_snapshot.machineActivity = ngc::MachineActivity::Faulted;
        m_snapshot.status = ngc::SimulationStatus::Error;
        m_snapshot.error = "GUI emergency stop is latched";
        m_snapshot.emergencyStopActiveSources |= emergencyStopSourceMask(
            ngc::EmergencyStopSource::Gui);
        m_snapshot.emergencyStopLatchedSources |= emergencyStopSourceMask(
            ngc::EmergencyStopSource::Gui);
        m_cv.notify_all();

        return true;
    }

    std::expected<void, std::string> resetEmergencyStop() {
        std::unique_lock lock(m_mutex);
        if (m_machineSession.coordinator().powerState()
                != ngc::MachinePowerState::Faulted) {
            return std::unexpected("the session is not emergency-stop faulted");
        }
        if (m_running || m_machineSession.executionEpochActive()) {
            return std::unexpected(
                "the faulted operation has not finished shutting down");
        }

        const auto guiSource = emergencyStopSourceMask(
            ngc::EmergencyStopSource::Gui);
        const bool guiEmergencyStop =
            (m_snapshot.emergencyStopLatchedSources & guiSource) != 0;

        lock.unlock();

        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::seconds(2);
        auto status = ngc::EmergencyStopStatus{};
        if (guiEmergencyStop) {
            while (std::chrono::steady_clock::now() < deadline) {
                status = m_runtime.emergencyStopStatus();
                if ((status.latchedSources & guiSource) != 0) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if ((status.latchedSources & guiSource) == 0) {
                return std::unexpected(
                    "the executor did not acknowledge the GUI emergency stop");
            }
            m_runtime.releaseEmergencyStop(ngc::EmergencyStopSource::Gui);
        }

        const auto resetGeneration = m_runtime.requestEmergencyStopReset();
        while (std::chrono::steady_clock::now() < deadline) {
            status = m_runtime.emergencyStopStatus();
            if (status.acknowledgedResetGeneration == resetGeneration) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (resetGeneration == 0
            || status.acknowledgedResetGeneration != resetGeneration) {
            return std::unexpected(
                "the executor did not acknowledge emergency-stop reset");
        }
        if (status.resetResult
                == ngc::EmergencyStopResetResult::BlockedByActiveSource) {
            return std::unexpected(
                "an emergency-stop source is still active");
        }
        if (status.resetResult != ngc::EmergencyStopResetResult::Cleared) {
            return std::unexpected(
                "the executor did not clear its emergency-stop latch");
        }

        m_runtime.stop();

        lock.lock();
        if (!m_machineSession.coordinator().resetFaultToOff()) {
            return std::unexpected(
                "the session left its faulted state during reset");
        }
        m_snapshot.powerState = ngc::MachinePowerState::Off;
        m_snapshot.machineActivity = ngc::MachineActivity::Idle;
        m_snapshot.status = ngc::SimulationStatus::Stopped;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.programOperation = ngc::ProgramOperationPresentation::Failed;
        m_snapshot.trajectoryBackendState = ngc::BackendState::Disabled;
        m_snapshot.trajectoryBackendFaultCode = 0;
        m_snapshot.emergencyStopActiveSources = 0;
        m_snapshot.emergencyStopLatchedSources = 0;
        m_snapshot.error.clear();

        return {};
    }

    SessionCommandResult start(const MachineControlAuthority authority,
                               const std::vector<std::tuple<std::string, std::string>> &programs,
                               const ngc::ToolTable &tools, const bool preserveState = false) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        if (m_snapshot.trajectoryBackendState == ngc::BackendState::Faulted
            || m_snapshot.trajectoryBackendFaultCode != 0) {
            return { SessionCommandRejection::BackendFaulted };
        }
        if (motionOwnedOrQueued()) {
            return { SessionCommandRejection::MotionOwned };
        }
        if (programs.empty()) {
            return { SessionCommandRejection::EmptyProgram };
        }
        if (m_machineSession.programMotionRequiresHoming()) {
            return { SessionCommandRejection::HomingRequired };
        }
        const auto activity = std::get<1>(programs.back()) == "<MDI>"
            ? ngc::MachineActivity::Mdi : ngc::MachineActivity::Program;
        if (!m_machineSession.toolTableInitialized()
            && !m_machineSession.setToolTable(tools)) {
            return { SessionCommandRejection::ToolTableUnavailable };
        }
        if (!m_machineSession.queueProgram(ngc::StartProgram {
                .programs = programs,
                .preserveState = preserveState,
                .activity = activity,
            })) {
            return { SessionCommandRejection::CommandUnavailable };
        }
        m_stop = false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Program;
        m_snapshot.programOperation = ngc::ProgramOperationPresentation::Running;
        m_snapshot.operatorAlert.reset();
        m_cv.notify_all();

        return {};
    }

    SessionCommandResult home(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        if (m_snapshot.trajectoryBackendState == ngc::BackendState::Faulted
            || m_snapshot.trajectoryBackendFaultCode != 0) {
            return { SessionCommandRejection::BackendFaulted };
        }
        if (motionOwnedOrQueued()) {
            return { SessionCommandRejection::MotionOwned };
        }
        if (!m_machineSession.homingAvailable()) {
            return { SessionCommandRejection::HomingUnavailable };
        }
        if (!m_machineSession.queueHoming()) {
            return { SessionCommandRejection::CommandUnavailable };
        }
        m_snapshot.homedJoints = m_machineSession.homedJoints();
        m_stop = false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Homing;
        m_snapshot.error.clear();
        m_cv.notify_all();

        return {};
    }

    bool homingAvailable() const {
        std::scoped_lock lock(m_mutex);
        return m_machineSession.homingAvailable();
    }

    SessionCommandResult startJog(const MachineControlAuthority authority,
                                  const ngc::ControlRequest &request) {
        const auto jog = std::visit([](const auto &value) -> std::optional<ngc::JogId> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, ngc::StartContinuousJogRequest>
                         || std::same_as<T, ngc::StartIncrementalJogRequest>) return value.jog;
            return std::nullopt;
        }, request);
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        if (m_snapshot.trajectoryBackendState == ngc::BackendState::Faulted
            || m_snapshot.trajectoryBackendFaultCode != 0) {
            return { SessionCommandRejection::BackendFaulted };
        }
        if (!jog || *jog == 0) {
            return { SessionCommandRejection::InvalidJogRequest };
        }
        if (motionOwnedOrQueued()) {
            return { SessionCommandRejection::MotionOwned };
        }
        if (!m_machineSession.queueJog(request)) {
            return { SessionCommandRejection::CommandUnavailable };
        }
        m_stop = false;
        m_activeJog = *jog;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Jogging;
        m_snapshot.jogging = true;
        m_snapshot.lastJogStopReason.reset();
        m_cv.notify_all();

        return {};
    }

    bool renewJog(const MachineControlAuthority authority, const ngc::RequestId request,
                  const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return false;
        }
        if(!m_activeJog || *m_activeJog != jog) return false;
        if(m_machineSession.coordinator().commands().anyOf([&](const auto &command) {
            const auto *renewal = std::get_if<ngc::RenewJog>(&command);
            return renewal && renewal->jog == jog;
        })) return true;
        if (!m_machineSession.coordinator().commands().tryPush(ngc::RenewJog { request, jog })) return false;
        m_cv.notify_all();
        return true;
    }

    bool setJogVelocity(const MachineControlAuthority authority,
                        const ngc::SetContinuousJogVelocityRequest &request) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return false;
        }
        if(!m_activeJog || *m_activeJog != request.jog) return false;
        m_machineSession.coordinator().commands().eraseIf([&](const auto &command) {
            if(const auto *renewal = std::get_if<ngc::RenewJog>(&command))
                return renewal->jog == request.jog;
            if(const auto *update = std::get_if<ngc::SetJogVelocity>(&command))
                return update->request.jog == request.jog;
            return false;
        });
        if (!m_machineSession.coordinator().commands().tryPush(ngc::SetJogVelocity { request })) return false;
        m_cv.notify_all();
        return true;
    }

    std::expected<void, std::string>
    setActiveWorkCoordinate(const MachineControlAuthority authority,
                            const ngc::Machine::Axis axis, const double workPosition) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (motionOwnedOrQueued()
            || m_snapshot.status == ngc::SimulationStatus::Error) {
            return std::unexpected("cannot change a work offset while motion owns the machine");
        }
        if(!std::isfinite(workPosition))
            return std::unexpected("requested work coordinate is not finite");
        const auto machinePosition = axisComponent(m_snapshot.machinePosition, axis);
        const auto toolOffset = axisComponent(
            m_machineSession.presentationTracker().snapshot().activePresentation.activeToolOffset, axis);
        const auto offset = machinePosition - toolOffset - workPosition;
        m_machineSession.interpreter().machine().setActiveWorkOffset(axis, offset);
        const ngc::WorkCoordinateSystem updated {
            std::string(ngc::name(*m_machineSession.interpreter().machine().state().modeCoordSys)),
            m_machineSession.interpreter().machine().workOffset(),
        };
        m_machineSession.presentationTracker().setActiveWorkCoordinateSystem(updated);
        const auto saved = m_machineSession.persistParametersAtBoundary();
        if (!saved) {
            return std::unexpected(saved.error());
        }

        return {};
    }

    bool stopJog(const MachineControlAuthority authority, const ngc::RequestId request,
                 const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return false;
        }
        if(!m_activeJog || *m_activeJog != jog) return false;
        m_machineSession.coordinator().commands().eraseIf([&](const auto &command) {
            const auto *renewal = std::get_if<ngc::RenewJog>(&command);
            if(renewal) return renewal->jog == jog;
            const auto *update = std::get_if<ngc::SetJogVelocity>(&command);
            return update && update->request.jog == jog;
        });
        if(m_machineSession.coordinator().commands().anyOf([&](const auto &command) {
            const auto *stop = std::get_if<ngc::StopJog>(&command);
            return stop && stop->jog == jog;
        })) return true;
        if (!m_machineSession.coordinator().commands().tryPush(ngc::StopJog { request, jog })) return false;
        m_cv.notify_all();
        return true;
    }

    SessionCommandResult feedHold(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        const auto &control = m_machineSession.programExecution();
        if (!m_running || !m_programRunning) {
            return { SessionCommandRejection::ProgramNotRunning };
        }
        if (control.paused()) {
            return { SessionCommandRejection::ProgramAlreadyPaused };
        }
        if (control.feedHoldInProgress()) {
            return { SessionCommandRejection::FeedHoldInProgress };
        }
        if (control.feedResumeInProgress()) {
            return { SessionCommandRejection::FeedResumeInProgress };
        }
        if (m_machineSession.coordinator().commands().contains<ngc::FeedHold>()) {
            return { SessionCommandRejection::CommandAlreadyQueued };
        }
        if (!m_machineSession.coordinator().commands().tryPush(ngc::FeedHold {})) {
            return { SessionCommandRejection::CommandQueueFull };
        }
        m_snapshot.status = ngc::SimulationStatus::Holding;
        m_snapshot.programOperation = ngc::ProgramOperationPresentation::FeedHoldPending;
        m_cv.notify_all();

        return {};
    }
    SessionCommandResult resume(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        const auto &control = m_machineSession.programExecution();
        if (!m_running || !m_programRunning) {
            return { SessionCommandRejection::ProgramNotRunning };
        }
        if (!control.paused()) {
            return { SessionCommandRejection::ProgramNotPaused };
        }
        if (m_machineSession.coordinator().commands().contains<ngc::Resume>()) {
            return { SessionCommandRejection::CommandAlreadyQueued };
        }
        if (control.programPaused()) {
            if (!m_machineSession.coordinator().commands().tryPush(ngc::Resume {})) {
                return { SessionCommandRejection::CommandQueueFull };
            }
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.programOperation = ngc::ProgramOperationPresentation::Running;
            m_snapshot.operatorAlert.reset();
            m_cv.notify_all();

            return {};
        }
        if (!m_machineSession.coordinator().commands().tryPush(ngc::Resume {})) {
            return { SessionCommandRejection::CommandQueueFull };
        }
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.programOperation = ngc::ProgramOperationPresentation::Resuming;
        m_cv.notify_all();

        return {};
    }
    SessionCommandResult stop(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return { SessionCommandRejection::StaleControlAuthority };
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        if (!motionOwnedOrQueued()) {
            return { SessionCommandRejection::NoMotionOwner };
        }
        m_machineSession.coordinator().commands().clear();
        if (!m_machineSession.coordinator().commands().tryPush(ngc::Stop {})) {
            return { SessionCommandRejection::CommandQueueFull };
        }
        m_snapshot.operatorAlert.reset();
        if (m_running) {
            m_snapshot.status = ngc::SimulationStatus::Holding;
        }
        if (m_snapshot.activity == ngc::SimulationActivity::Program) {
            m_snapshot.programOperation = ngc::ProgramOperationPresentation::Stopping;
        }
        m_cv.notify_all();

        return {};
    }
    void setTickMultiplier(const int multiplier) {
        std::scoped_lock lock(m_mutex);
        m_tickMultiplier = static_cast<std::uint32_t>(std::clamp(multiplier, 1, 1000));
        m_runtime.setTickMultiplier(multiplier);
        m_snapshot.tickMultiplier = m_tickMultiplier;
    }
    bool setSplineFitSolver(const ngc::spline_detail::SplineFitSolver solver) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return false;
        }
        m_machineSession.geometryPolicy().splineFitSolver = solver;
        return true;
    }
    bool setContinuousPlanningEffort(const ngc::ContinuousPlanningEffort &effort) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return false;
        }
        auto configuredEffort = effort;
        configuredEffort.quinticServoPeriod = m_runtime.servoPeriod();
        m_machineSession.driver().setContinuousPlanningEffort(configuredEffort);
        return true;
    }
    bool setContinuousDiagnosticCallback(std::function<void(
            const ngc::ContinuousTrajectoryPlan &,
            std::span<const ngc::TrajectoryPlannerInput>)> callback) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return false;
        }
        m_machineSession.driver().setContinuousDiagnosticCallback(std::move(callback));
        return true;
    }
    void setRapidSpeed(const double speed) {
        std::scoped_lock lock(m_mutex);
        m_limits.rapidSpeed = std::max(speed, 1e-6);
        if (!motionOwnedOrQueued()) {
            m_machineSession.driver().setLimits(m_limits);
        }
    }
    ngc::SimulationSnapshot snapshot() const {
        std::scoped_lock lock(m_mutex);
        auto result = m_snapshot;
        if (const auto timing = latestRealtimeTiming()) {
            result.realtimeTiming = *timing;
        }
        const auto emergencyStop = m_runtime.emergencyStopStatus();
        result.emergencyStopActiveSources = emergencyStop.activeSources;
        result.emergencyStopLatchedSources = emergencyStop.latchedSources;
        if (m_programRunning) {
            if (const auto backend = latestTimedBackendSnapshot()) {
                applyBackendObservation(result, *backend);
            }
            const auto operation = m_machineSession.programExecution().presentation();
            const auto queuedTransition =
                (result.programOperation
                    == ngc::ProgramOperationPresentation::FeedHoldPending
                 && operation == ngc::ProgramOperationPresentation::Running)
                || (result.programOperation
                        == ngc::ProgramOperationPresentation::Resuming
                    && operation == ngc::ProgramOperationPresentation::Held)
                || (result.programOperation
                        == ngc::ProgramOperationPresentation::Stopping
                    && operation != ngc::ProgramOperationPresentation::Stopping
                    && operation != ngc::ProgramOperationPresentation::Stopped
                    && operation != ngc::ProgramOperationPresentation::Failed);
            if (operation != ngc::ProgramOperationPresentation::Inactive
                && !queuedTransition) {
                result.programOperation = operation;
            }
        } else if (result.activity == ngc::SimulationActivity::Homing) {
            if (const auto observation = m_machineSession.homingObservation()) {
                result.machinePosition = observation->machinePosition;
                result.joints = observation->joints;
                result.commandProgress = observation->commandProgress;
                result.hasActiveMotion = observation->hasActiveMotion;
                result.servoTicks = observation->servoTicks;
                result.trajectoryBackendState = observation->backendState;
                result.trajectoryBackendFaultCode = observation->backendFaultCode;
            }
        } else if (result.activity == ngc::SimulationActivity::Jogging) {
            if (const auto observation = m_machineSession.joggingObservation()) {
                result.machinePosition = observation->machinePosition;
                result.joints = observation->joints;
                result.commandProgress = observation->commandProgress;
                result.hasActiveMotion = observation->hasActiveMotion;
                result.servoTicks = observation->servoTicks;
                result.trajectoryBackendState = observation->backendState;
                result.trajectoryBackendFaultCode = observation->backendFaultCode;
            }
        }
        if (result.status == ngc::SimulationStatus::Paused
            && !m_machineSession.programExecution().programPaused()
            && result.trajectoryBackendState != ngc::BackendState::Held) {
            result.status = ngc::SimulationStatus::Holding;
        }
        if (m_runtime.simulationDiagnosticsAvailable()) {
            result.simulationDiagnostics = ngc::SimulationDiagnostics {
                .servoPeriodSeconds = result.servoPeriodSeconds,
                .schedulerPeriodSeconds = result.schedulerPeriodSeconds,
                .servoTicksPerSchedulerPeriod = result.servoTicksPerSchedulerPeriod,
                .tickMultiplier = result.tickMultiplier,
                .servoTicks = result.servoTicks,
                .programElapsedSeconds = result.programElapsedSeconds,
                .executedPathJerk = result.executedPathJerk,
                .deadlineMisses = result.deadlineMisses,
                .lastWakeLatenessSeconds = result.lastWakeLatenessSeconds,
                .maximumWakeLatenessSeconds = result.maximumWakeLatenessSeconds,
                .maximumTickExecutionSeconds = result.maximumTickExecutionSeconds,
            };
        } else {
            result.simulationDiagnostics.reset();
        }
        if (!m_powerRequestActive) {
            result.powerState = m_machineSession.coordinator().powerState();
        }
        if (result.powerState == ngc::MachinePowerState::Stopping) {
            result.machineActivity = ngc::MachineActivity::Stopping;
        } else if (result.powerState == ngc::MachinePowerState::Faulted
                   || result.status == ngc::SimulationStatus::Error) {
            result.machineActivity = ngc::MachineActivity::Faulted;
        } else if (result.status == ngc::SimulationStatus::Holding
                   || (result.status == ngc::SimulationStatus::Paused
                       && !m_machineSession.programExecution().programPaused())) {
            result.machineActivity = ngc::MachineActivity::Holding;
        } else {
            result.machineActivity = m_machineSession.coordinator().activity();
        }
        const auto &presentation = m_machineSession.presentationTracker().snapshot();
        result.activePresentation = presentation.activePresentation;
        result.spindleRunning = presentation.spindleRunning;
        result.spindleSpeed = presentation.spindleSpeed;
        result.spindleDirection = presentation.spindleDirection;
        result.usedWorkCoordinateSystems = presentation.usedWorkCoordinateSystems;
        result.completedBlocks = presentation.completedBlocks;
        result.completedLineFlags = presentation.completedLineFlags;

        return result;
    }

    bool setToolTable(const MachineControlAuthority authority, const ngc::ToolTable &tools) {
        std::scoped_lock lock(m_mutex);
        return hasControlAuthorityLocked(authority) && !motionOwnedOrQueued()
            && m_machineSession.setToolTable(tools);
    }

    ngc::ToolTable toolTable() const {
        std::scoped_lock lock(m_mutex);

        return m_machineSession.toolTable();
    }

    std::pair<ngc::ToolTable, std::uint64_t> toolTableSnapshot() const {
        std::scoped_lock lock(m_mutex);

        return m_machineSession.toolTableSnapshot();
    }

    ngc::ParameterSnapshot parameterSnapshot() const {
        std::scoped_lock lock(m_mutex);

        return m_parameterSnapshot;
    }

    bool controllerDataMutable() const {
        std::scoped_lock lock(m_mutex);

        return !motionOwnedOrQueued() && m_machineSession.controllerDataMutable();
    }

    std::expected<ngc::MachineSessionCheckpoint, std::string> checkpoint(
            const MachineControlAuthority authority) const {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return std::unexpected(
                "Machine-to-Simulation branching requires Machine to be powered on");
        }
        if (motionOwnedOrQueued() || m_snapshot.hasActiveMotion
            || m_snapshot.trajectoryBackendQueuedExecutionItems != 0
            || m_snapshot.trajectoryBackendVelocity > 1e-9
            || m_snapshot.trajectoryBackendAcceleration > 1e-9) {
            return std::unexpected(
                "Machine-to-Simulation branching requires Machine to be stationary and idle");
        }
        if (m_snapshot.status == ngc::SimulationStatus::Error
            || m_snapshot.trajectoryBackendState == ngc::BackendState::Faulted
            || m_snapshot.trajectoryBackendFaultCode != 0) {
            return std::unexpected(
                "Machine-to-Simulation branching requires fault-free Machine position confidence");
        }

        ngc::JointMask configuredJoints = 0;
        for (const auto &joint : m_joints) {
            configuredJoints |= ngc::JointMask {1} << joint.id;
        }
        if ((m_machineSession.homedJoints() & configuredJoints) != configuredJoints) {
            return std::unexpected(
                "Machine-to-Simulation branching requires every configured Machine joint to be homed");
        }

        const ngc::StationaryBackendState backend {
            .commanded = {
                .position = m_snapshot.machinePosition,
            },
            .feedback = {
                .position = m_snapshot.machinePosition,
            },
            .commandedJoints = m_snapshot.joints,
            .feedbackJoints = m_snapshot.joints,
        };

        return m_machineSession.checkpoint(backend);
    }

    std::expected<void, std::string> restoreCheckpoint(
            const ngc::MachineSessionCheckpoint &checkpoint) {
        std::scoped_lock lock(m_mutex);
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::Off
            || motionOwnedOrQueued()) {
            return std::unexpected(
                "Machine-to-Simulation import requires Simulation to be powered off and idle");
        }

        auto restored = m_machineSession.restoreCheckpoint(checkpoint);
        if (!restored) {
            return restored;
        }

        m_snapshot = {};
        copyRuntimeTimingSnapshot();
        m_snapshot.powerState = ngc::MachinePowerState::Off;
        m_snapshot.status = ngc::SimulationStatus::Stopped;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.machineActivity = ngc::MachineActivity::Idle;
        m_snapshot.machinePosition = checkpoint.backend.commanded.position;
        m_snapshot.joints = checkpoint.backend.commandedJoints;
        m_snapshot.homedJoints = checkpoint.homedJoints;
        refreshParameterSnapshot();

        return {};
    }

    std::expected<void, std::string> setToolTableStorePath(
            const MachineControlAuthority authority, const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot configure the tool-table store while motion owns the machine");
        }
        return m_machineSession.setToolTableStorePath(path);
    }

    std::expected<void, std::string> saveToolTable(
            const MachineControlAuthority authority, const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot save the tool table while motion owns the machine");
        }

        return m_machineSession.saveToolTable(path);
    }

    std::expected<void, std::string> setPersistentParameterStorePath(
            const MachineControlAuthority authority, const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot configure persistent parameters while motion owns the machine");
        }
        return m_machineSession.setPersistentParameterStorePath(path);
    }
    std::expected<void, std::string> loadPersistentParameters(
            const MachineControlAuthority authority, const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (motionOwnedOrQueued()) {
            return std::unexpected("cannot load persistent parameters while motion owns the machine");
        }

        auto loaded = m_machineSession.loadPersistentParameters(path);
        if (loaded) {
            refreshParameterSnapshot();
            m_machineSession.presentationTracker().setActivePresentation(sessionPresentation());
        }

        return loaded;
    }

    std::expected<void, std::string> savePersistentParameters(
            const MachineControlAuthority authority, const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (!hasControlAuthorityLocked(authority)) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (motionOwnedOrQueued()) {
            return std::unexpected("cannot save persistent parameters while motion owns the machine");
        }

        return m_machineSession.savePersistentParameters(path);
    }
    std::vector<ngc::ExecutedJerkSample> takeExecutedJerkSamples() {
        auto samples = m_runtime.takeExecutedJerkSamples();
        std::scoped_lock lock(m_mutex);
        for(auto &sample:samples) {
            if (const auto toolOffset = m_machineSession.presentationTracker().toolOffsetForChunk(
                    sample.epoch, sample.chunk)) {
                sample.position = sample.position - *toolOffset;
            }
        }
        return samples;
    }

    void join() {
        {
            std::scoped_lock lock(m_mutex);
            if (!m_thread.joinable()) {
                return;
            }
            m_snapshot.powerState = ngc::MachinePowerState::Stopping;
            m_join = true;
            m_pendingPowerOperation.reset();
            m_powerResultReady = false;
            if (motionOwnedOrQueued()) {
                m_machineSession.coordinator().commands().clear();
                (void)m_machineSession.coordinator().commands().tryPush(ngc::Stop {});
            }
            m_cv.notify_all();
        }
        m_thread.join();
        {
            std::scoped_lock lock(m_mutex);
            m_machineSession.coordinator().finishActivity();
            if (m_machineSession.coordinator().powerState()
                    == ngc::MachinePowerState::Faulted) {
                m_runtime.stop();
                (void)m_machineSession.coordinator().resetFaultToOff();
            } else {
                (void)m_machineSession.powerOff();
            }
            m_snapshot.powerState = m_machineSession.coordinator().powerState();
            m_snapshot.machineActivity = ngc::MachineActivity::Idle;
            m_cv.notify_all();
        }
    }

private:
    void startTimedSnapshotService() {
        {
            std::scoped_lock lock(m_timedSnapshotMutex);
            m_stopTimedSnapshotService = false;
            m_latestTimedBackendSnapshot.reset();
            m_latestRealtimeTiming.reset();
        }
        m_timedSnapshotThread = std::thread([this] {
            const auto drainLatest = [&] {
                std::optional<ngc::ExecutionSnapshot> latest;
                ngc::ExecutionSnapshot backendSnapshot;
                while (m_runtime.endpoint().tryTakeSnapshot(backendSnapshot)) {
                    latest = backendSnapshot;
                }
                if (latest) {
                    std::scoped_lock lock(m_timedSnapshotMutex);
                    m_latestTimedBackendSnapshot = *latest;
                }

                std::optional<ngc::RealtimeTimingSummary> latestTiming;
                ngc::RealtimeTimingSummary timing;
                while (m_runtime.tryTakeRealtimeTiming(timing)) {
                    if (!latestTiming) {
                        latestTiming = timing;
                    } else {
                        ngc::mergeRealtimeTiming(
                            *latestTiming, timing);
                    }
                }
                if (latestTiming) {
                    std::scoped_lock lock(m_timedSnapshotMutex);
                    if (!m_latestRealtimeTiming) {
                        m_latestRealtimeTiming = *latestTiming;
                    } else {
                        ngc::mergeRealtimeTiming(
                            *m_latestRealtimeTiming,
                            *latestTiming);
                    }
                }
            };

            for (;;) {
                drainLatest();
                std::unique_lock lock(m_timedSnapshotMutex);
                if (m_stopTimedSnapshotService) {
                    break;
                }
                m_timedSnapshotCv.wait_for(lock, std::chrono::milliseconds(1), [&] {
                    return m_stopTimedSnapshotService;
                });
            }
            drainLatest();
        });
    }

    void stopTimedSnapshotService() {
        {
            std::scoped_lock lock(m_timedSnapshotMutex);
            m_stopTimedSnapshotService = true;
        }
        m_timedSnapshotCv.notify_all();
        if (m_timedSnapshotThread.joinable()) {
            m_timedSnapshotThread.join();
        }
        std::scoped_lock lock(m_mutex);
        applyLatestTimedBackendSnapshot();
    }

    [[nodiscard]] std::optional<ngc::ExecutionSnapshot> latestTimedBackendSnapshot() const {
        std::scoped_lock lock(m_timedSnapshotMutex);

        return m_latestTimedBackendSnapshot;
    }

    [[nodiscard]] std::optional<ngc::RealtimeTimingSummary>
    latestRealtimeTiming() const {
        std::scoped_lock lock(m_timedSnapshotMutex);

        return m_latestRealtimeTiming;
    }

    void copyRuntimeTimingSnapshot() {
        const auto runtime = m_runtime.snapshot();
        m_snapshot.servoPeriodSeconds = runtime.servoPeriodSeconds;
        m_snapshot.schedulerPeriodSeconds = runtime.schedulerPeriodSeconds;
        m_snapshot.servoTicksPerSchedulerPeriod =
            runtime.servoTicksPerSchedulerPeriod;
        m_snapshot.tickMultiplier = runtime.tickMultiplier;
        m_snapshot.servoTicks = runtime.servoTicks;
        m_snapshot.programElapsedSeconds = runtime.programElapsedSeconds;
        m_snapshot.executedPathJerk = runtime.executedPathJerk;
        m_snapshot.deadlineMisses = runtime.deadlineMisses;
        m_snapshot.lastWakeLatenessSeconds = runtime.lastWakeLatenessSeconds;
        m_snapshot.maximumWakeLatenessSeconds =
            runtime.maximumWakeLatenessSeconds;
        m_snapshot.maximumTickExecutionSeconds =
            runtime.maximumTickExecutionSeconds;
    }

    void copyRealtimeTimingSnapshot() {
        ngc::RealtimeTimingSummary timing;
        while (m_runtime.tryTakeRealtimeTiming(timing)) {
            if (!m_snapshot.realtimeTiming) {
                m_snapshot.realtimeTiming = timing;
            } else {
                ngc::mergeRealtimeTiming(
                    *m_snapshot.realtimeTiming, timing);
            }
        }
    }

    void clearPresentation() {
        m_machineSession.presentationTracker().clearTracking();
    }

    void observeCommand(const ngc::MachineCommand &command, const ngc::ExecutionItem &item,
                        const ngc::TrajectoryCommandPresentation &captured,
                        const ngc::ExecutionMarkerId) {
        if(std::holds_alternative<ngc::ProbeMove>(command)) {
            const auto &move = std::get<ngc::TriggeredMove>(item);
            const auto contact = move.target + captured.tool.offset
                - captured.activeToolOffset;
            (void)m_runtime.configureSyntheticInput(move.moveId, contact);
        }
    }

    void observeBackendEvent(const ngc::ExecutionEvent &event) {
        if (const auto *held = std::get_if<ngc::BackendHeld>(&event)) {
            if(held->reason == ngc::BackendHoldReason::FeedHold) {
                m_snapshot.status = ngc::SimulationStatus::Paused;
            } else if (held->reason == ngc::BackendHoldReason::ControlledStop) {
                m_snapshot.machinePosition = held->state.position;
                m_snapshot.trajectoryBackendState = ngc::BackendState::Held;
                m_snapshot.trajectoryBackendActiveNormalRemainingSeconds = 0.0;
                m_snapshot.trajectoryBackendQueuedNormalSeconds = 0.0;
                m_snapshot.trajectoryBackendCommittedNormalSeconds = 0.0;
                m_snapshot.trajectoryBackendStopBranchSeconds = 0.0;
                m_snapshot.trajectoryBackendQueuedExecutionItems = 0;
                m_snapshot.trajectoryBackendVelocity = 0.0;
                m_snapshot.trajectoryBackendAcceleration = 0.0;
                m_snapshot.hasActiveMotion = false;
            }
        }
    }

    void applyActivePresentation(
            const ngc::TrajectoryCommandPresentation &presentation) {
        m_machineSession.presentationTracker().setActivePresentation(presentation);
    }

    void applyBackendObservation(ngc::SimulationSnapshot &target,
                                 const ngc::ExecutionSnapshot &backend) const {
        target.trajectoryBackendState = backend.state;
        target.trajectoryBackendEpoch = backend.epoch;
        target.trajectoryBackendChunk = backend.activeChunk;
        target.trajectoryBackendSpan = backend.activeSpan;
        target.trajectoryBackendSpanProgress = backend.spanProgress;
        target.trajectoryBackendActiveNormalRemainingSeconds =
            backend.activeNormalMotionRemainingSeconds;
        target.trajectoryBackendQueuedNormalSeconds = backend.queuedNormalMotionSeconds;
        target.trajectoryBackendCommittedNormalSeconds = backend.committedNormalMotionSeconds;
        target.trajectoryBackendStopBranchSeconds = backend.stopBranchRemainingSeconds;
        target.trajectoryBackendQueuedExecutionItems = backend.queuedExecutionItems;
        target.trajectoryBackendLastBranch = backend.lastBranch;
        target.trajectoryBackendFaultCode = backend.faultCode;
        target.trajectoryBackendVelocity = backend.commanded.velocity.length();
        target.trajectoryBackendAcceleration = backend.commanded.acceleration.length();
        target.trajectoryBackendExecutionRate = backend.executionRate;
        target.trajectoryBackendExecutionRateAcceleration =
            backend.executionRateAcceleration;
        if(const auto detail = m_machineSession.presentationTracker().executionSpanDiagnostic(
               backend.epoch, backend.activeSpan)) {
            target.trajectoryBackendSpanDetail=std::format(
                "{} ordinal={} duration={:.9g}s distance={:.9g} "
                "velocity={:.9g}->{:.9g} acceleration={:.9g}->{:.9g}",
                detail->stopTail ? "stop-tail" : "normal", detail->ordinal,
                detail->duration, detail->distance, detail->startVelocity,
                detail->endVelocity, detail->startAcceleration, detail->endAcceleration);
        } else {
            target.trajectoryBackendSpanDetail.clear();
        }
        if(backend.state == ngc::BackendState::Faulted) {
            target.status = ngc::SimulationStatus::Error;
            target.error = "motion backend fault " + std::to_string(backend.faultCode);
        }
        target.machinePosition = backend.commanded.position;
        target.joints = backend.commandedJoints;
        target.commandProgress = backend.spanProgress;
        target.hasActiveMotion = (backend.state == ngc::BackendState::Running
                                  || backend.state == ngc::BackendState::Holding)
            && backend.activeSpan != 0;
    }

    void applyBackendSnapshot(const ngc::ExecutionSnapshot &backend) {
        if (m_machineSession.programExecution().state()
                == ngc::ProgramExecutionState::StopComplete
            && backend.state != ngc::BackendState::Held) {
            return;
        }
        applyBackendObservation(m_snapshot, backend);
    }

    void applyLatestTimedBackendSnapshot() {
        {
            std::scoped_lock lock(m_timedSnapshotMutex);
            if (m_latestRealtimeTiming) {
                m_snapshot.realtimeTiming =
                    *m_latestRealtimeTiming;
            }
        }
        if (const auto backend = latestTimedBackendSnapshot()) {
            applyBackendSnapshot(*backend);
        }
    }

    static double &axisComponent(ngc::position_t &position, const ngc::Machine::Axis axis) {
        switch(axis) {
            case ngc::Machine::Axis::X: return position.x;
            case ngc::Machine::Axis::Y: return position.y;
            case ngc::Machine::Axis::Z: return position.z;
            case ngc::Machine::Axis::A: return position.a;
            case ngc::Machine::Axis::B: return position.b;
            case ngc::Machine::Axis::C: return position.c;
        }
        return position.x;
    }

    static double axisComponent(const ngc::position_t &position, const ngc::Machine::Axis axis) {
        switch (axis) {
            case ngc::Machine::Axis::X: return position.x;
            case ngc::Machine::Axis::Y: return position.y;
            case ngc::Machine::Axis::Z: return position.z;
            case ngc::Machine::Axis::A: return position.a;
            case ngc::Machine::Axis::B: return position.b;
            case ngc::Machine::Axis::C: return position.c;
        }

        return position.x;
    }

    ngc::TrajectoryCommandPresentation sessionPresentation() const {
        return {
            .tool = m_machineSession.interpreter().machine().toolGeometry(),
            .activeToolOffset = m_machineSession.interpreter().machine().toolOffset(),
            .workCoordinateSystem = ngc::WorkCoordinateSystem {
                std::string(ngc::name(
                    *m_machineSession.interpreter().machine().state().modeCoordSys)),
                m_machineSession.interpreter().machine().workOffset(),
            },
            .modalGCodes = m_machineSession.interpreter().machine().activeModalGCodes(),
            .activeBlocks = {},
        };
    }

    void clearActiveTool() {
        m_machineSession.presentationTracker().clearActiveTool();
    }

    void runSessionHoming() {
        ngc::position_t startingPosition;
        {
            std::scoped_lock lock(m_mutex);
            startingPosition = m_snapshot.machinePosition;
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.activity = ngc::SimulationActivity::Homing;
            m_snapshot.error.clear();
            m_snapshot.servoTicks = 0;
            clearPresentation();
            clearActiveTool();
        }

        const auto result = m_machineSession.runHoming(startingPosition);

        std::scoped_lock lock(m_mutex);
        copyRealtimeTimingSnapshot();
        m_running = false;
        m_stop = false;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.hasActiveMotion = false;
        if (!result) {
            std::println(stderr, "MOTION ERROR: {}", result.error());
            m_snapshot.powerState = m_machineSession.coordinator().powerState();
            m_snapshot.machineActivity = m_machineSession.coordinator().activity();
            if (const auto observation = m_machineSession.homingObservation()) {
                m_snapshot.machinePosition = observation->machinePosition;
                m_snapshot.joints = observation->joints;
                m_snapshot.commandProgress = observation->commandProgress;
                m_snapshot.servoTicks = observation->servoTicks;
                m_snapshot.trajectoryBackendState = observation->backendState;
                m_snapshot.trajectoryBackendFaultCode =
                    observation->backendFaultCode;
            }
            m_snapshot.status = ngc::SimulationStatus::Error;
            m_snapshot.error = result.error();
            return;
        }

        m_snapshot.machinePosition = result->observation.machinePosition;
        m_snapshot.joints = result->observation.joints;
        m_snapshot.commandProgress = result->observation.commandProgress;
        m_snapshot.servoTicks = result->observation.servoTicks;
        if (result->outcome == ngc::HomingOutcome::Stopped) {
            m_snapshot.status = ngc::SimulationStatus::Stopped;
        } else {
            m_snapshot.homedJoints = m_machineSession.homedJoints();
            m_snapshot.status = ngc::SimulationStatus::Completed;
        }
        clearActiveTool();
    }

    void runJogging(const ngc::ControlRequest &firstRequest) {
        ngc::position_t startingPosition;
        {
            std::scoped_lock lock(m_mutex);
            startingPosition = m_snapshot.machinePosition;
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.activity = ngc::SimulationActivity::Jogging;
            m_snapshot.error.clear();
            m_snapshot.jogging = true;
            m_snapshot.homedJoints = m_machineSession.homedJoints();
            std::visit([&](const auto &request) {
                using T = std::decay_t<decltype(request)>;
                if constexpr (std::same_as<T, ngc::StartContinuousJogRequest>
                              || std::same_as<T, ngc::StartIncrementalJogRequest>) {
                    m_snapshot.activeJogTarget = request.target;
                }
            }, firstRequest);
            clearActiveTool();
        }

        const auto result = m_machineSession.runJogging(startingPosition, firstRequest);

        std::scoped_lock lock(m_mutex);
        m_running = false;
        m_stop = false;
        m_activeJog.reset();
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.jogging = false;
        m_snapshot.hasActiveMotion = false;
        m_snapshot.activeJogTarget.reset();
        if (!result) {
            if (const auto observation = m_machineSession.joggingObservation()) {
                m_snapshot.machinePosition = observation->machinePosition;
                m_snapshot.joints = observation->joints;
                m_snapshot.commandProgress = observation->commandProgress;
                m_snapshot.servoTicks = observation->servoTicks;
                m_snapshot.trajectoryBackendState = observation->backendState;
                m_snapshot.trajectoryBackendFaultCode =
                    observation->backendFaultCode;
            }
            m_snapshot.status = ngc::SimulationStatus::Error;
            m_snapshot.error = result.error();
            m_machineSession.coordinator().commands().clear();
            clearActiveTool();
            return;
        }

        m_snapshot.machinePosition = result->observation.machinePosition;
        m_snapshot.joints = result->observation.joints;
        m_snapshot.commandProgress = result->observation.commandProgress;
        m_snapshot.servoTicks = result->observation.servoTicks;
        m_snapshot.lastJogStopReason = result->stopReason;
        m_snapshot.status = result->outcome == ngc::JoggingOutcome::Stopped
            ? ngc::SimulationStatus::Stopped : ngc::SimulationStatus::Completed;
        clearActiveTool();
    }

    std::optional<std::string> joinGeometry(const ngc::ExecutionEpochOutcome outcome) {
        const auto active = m_machineSession.executionEpochActive();
        auto finished = m_machineSession.finishExecutionEpoch(outcome);
        if (active) {
            std::scoped_lock lock(m_mutex);
            refreshParameterSnapshot();
            m_snapshot.geometryStream = std::move(finished.diagnostics);
        }

        return std::move(finished.persistenceError);
    }

    void refreshParameterSnapshot() {
        m_parameterSnapshot = m_machineSession.parameterSnapshot();
    }

    void work() {
        using clock = std::chrono::steady_clock;
        for(;;) {
            std::unique_lock lock(m_mutex);
            static_cast<void>(m_cv.wait_for(lock, std::chrono::milliseconds(16), [&] {
                return m_join || m_pendingPowerOperation
                    || !m_machineSession.coordinator().commands().empty();
            }));
            const auto emergencyStop = m_runtime.emergencyStopStatus();
            if (emergencyStop.latchedSources != 0
                && m_machineSession.coordinator().powerState()
                    == ngc::MachinePowerState::On) {
                m_machineSession.coordinator().commands().clear();
                m_machineSession.coordinator().fault();
                m_snapshot.powerState = ngc::MachinePowerState::Faulted;
                m_snapshot.machineActivity = ngc::MachineActivity::Faulted;
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.error = "emergency stop is latched";
                m_snapshot.emergencyStopActiveSources =
                    emergencyStop.activeSources;
                m_snapshot.emergencyStopLatchedSources =
                    emergencyStop.latchedSources;
            }
            if (m_join) {
                return;
            }
            if (!m_pendingPowerOperation
                && m_machineSession.coordinator().commands().empty()) {
                continue;
            }
            if (m_pendingPowerOperation) {
                const auto operation = *m_pendingPowerOperation;
                m_pendingPowerOperation.reset();
                lock.unlock();
                const auto succeeded = operation == PowerOperation::On
                    ? m_machineSession.powerOn() : m_machineSession.powerOff();
                lock.lock();
                m_snapshot.powerState = m_machineSession.coordinator().powerState();
                m_powerResult = succeeded
                    ? SessionCommandResult {}
                    : SessionCommandResult { SessionCommandRejection::CommandUnavailable };
                m_powerResultReady = true;
                if (!succeeded) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.error = operation == PowerOperation::On
                        ? "Simulation session failed to power on"
                        : "Simulation session failed to power off";
                }
                m_cv.notify_all();
                continue;
            }
            auto dispatched = m_machineSession.dispatchNextOperation();
            if (!dispatched) {
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_snapshot.error = dispatched.error();
                m_running = false;
                m_programRunning = false;
                continue;
            }
            if (!*dispatched) {
                continue;
            }
            auto operation = std::move(**dispatched);
            if (std::holds_alternative<ngc::StartHoming>(operation)) {
                m_running = true;
                m_stop = false;
                lock.unlock();
                runSessionHoming();
                continue;
            }
            if (auto *jog = std::get_if<ngc::StartJog>(&operation)) {
                auto request = std::move(jog->request);
                m_running = true;
                m_stop = false;
                lock.unlock();
                runJogging(request);
                continue;
            }
            if (std::holds_alternative<ngc::Stop>(operation)) {
                m_snapshot.status = ngc::SimulationStatus::Stopped;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                continue;
            }
            if (!std::holds_alternative<ngc::StartProgram>(operation)) {
                continue;
            }
            auto start = std::get<ngc::StartProgram>(std::move(operation));
            auto programs = std::move(start.programs);
            const auto preserve = start.preserveState;
            const auto startingPosition = m_snapshot.machinePosition;
            const auto startingJoints = m_snapshot.joints;
            m_running = true; m_programRunning = true; m_stop = false;
            if (!preserve) {
                m_snapshot = {};
                m_snapshot.powerState = m_machineSession.coordinator().powerState();
                m_snapshot.machinePosition = startingPosition;
                m_snapshot.joints = startingJoints;
                m_snapshot.homedJoints = m_machineSession.homedJoints();
                m_machineSession.presentationTracker().reset();
            }
            m_runtime.setTickMultiplier(static_cast<int>(m_tickMultiplier));
            copyRuntimeTimingSnapshot();
            m_snapshot.servoTicks = 0;
            m_snapshot.programElapsedSeconds = 0.0;
            m_snapshot.executedPathJerk = 0.0;
            m_snapshot.deadlineMisses = 0;
            m_snapshot.lastWakeLatenessSeconds = 0.0;
            m_snapshot.maximumWakeLatenessSeconds = 0.0;
            m_snapshot.maximumTickExecutionSeconds = 0.0;
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.activity = ngc::SimulationActivity::Program;
            m_snapshot.programOperation = ngc::ProgramOperationPresentation::Running;
            lock.unlock();

            if(!preserve) {
                std::scoped_lock snapshotLock(m_mutex);
                applyActivePresentation(sessionPresentation());
            }
            start.programs = std::move(programs);
            const auto epochResult = m_machineSession.beginExecutionEpoch(
                std::move(start), startingPosition);
            if (!epochResult) {
                (void)joinGeometry(ngc::ExecutionEpochOutcome::Abandoned);
                lock.lock();
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_snapshot.programOperation = ngc::ProgramOperationPresentation::Failed;
                m_snapshot.error = epochResult.error();
                m_running = false;
                m_programRunning = false;
                lock.unlock();
                continue;
            }
            const auto epoch = *epochResult;
            if (!m_runtime.beginTimedExecution()) {
                (void)joinGeometry(ngc::ExecutionEpochOutcome::Abandoned);
                m_machineSession.interpreter().reportError(
                    "machine-session runtime failed to start timed execution");
                m_machineSession.interpreter().stop();
                lock.lock();
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_snapshot.programOperation = ngc::ProgramOperationPresentation::Failed;
                m_snapshot.error = "machine-session executor failed to start";
                m_running = false;
                m_programRunning = false;
                lock.unlock();
                continue;
            }
            startTimedSnapshotService();

            const auto copyTimingSnapshot = [&] {
                copyRuntimeTimingSnapshot();
                m_snapshot.trajectoryPlanningActivity=m_machineSession.driver().planningActivity();
                m_snapshot.trajectoryPlanningActivitySeconds=
                    m_machineSession.driver().planningActivitySeconds();
                m_snapshot.trajectoryDriverActivity=m_machineSession.driver().activity();
                m_snapshot.trajectoryContinuousPlanSummary=
                    m_machineSession.driver().lastContinuousPlanSummary();
                m_snapshot.trajectoryContinuousCorrectionHistory=
                    m_machineSession.driver().lastContinuousCorrectionHistory();
            };
            auto nextPlanningRefresh=clock::now();
            m_machineSession.driver().setPlanningProgressCallback([&] {
                const auto now=clock::now();
                if(now<nextPlanningRefresh) return;
                nextPlanningRefresh=now+std::chrono::milliseconds(16);
                std::unique_lock snapshotLock(m_mutex,std::try_to_lock);
                if(!snapshotLock.owns_lock()) return;
                applyLatestTimedBackendSnapshot();
                copyTimingSnapshot();
            });
            struct PlanningProgressReset {
                ngc::PreparedTrajectoryExecutionDriver &driver;
                ~PlanningProgressReset() { driver.setPlanningProgressCallback({}); }
            } planningProgressReset{m_machineSession.driver()};

            for(;;) {
                lock.lock();
                applyLatestTimedBackendSnapshot();
                auto backendObservation = latestTimedBackendSnapshot().value_or(
                    ngc::ExecutionSnapshot {});
                if (backendObservation.epoch == 0) {
                    backendObservation.epoch = epoch;
                    backendObservation.state = m_snapshot.trajectoryBackendState;
                    backendObservation.commanded.position = m_snapshot.machinePosition;
                }
                const auto joining = m_join;
                const auto backendWorkAllowed = !m_runtime.executorBatchActive();
                struct NrtRefillGuard {
                    SessionBackendRuntime &runtime;
                    bool enabled = false;
                    NrtRefillGuard(SessionBackendRuntime &value, const bool enable)
                        : runtime(value), enabled(enable) {
                        if (enabled) {
                            runtime.setNrtRefillActive(true);
                        }
                    }
                    void release() {
                        if (enabled) {
                            runtime.setNrtRefillActive(false);
                        }
                        enabled = false;
                    }
                    ~NrtRefillGuard() { release(); }
                } nrtRefillGuard{
                    m_runtime, backendWorkAllowed && m_runtime.tickMultiplier() > 1};
                lock.unlock();
                const auto operation = m_machineSession.serviceProgramOperation(
                    backendObservation, joining, 64,
                    [&] {
                        return !m_runtime.executorBatchActive();
                    },
                    [&](auto &&updatePresentation) {
                        std::scoped_lock presentationLock(m_mutex);
                        updatePresentation();
                    },
                    [&](const auto &event) {
                        observeBackendEvent(event);
                    },
                    [&](const auto &command, const auto &chunk,
                        const auto &presentation,
                        const ngc::ExecutionMarkerId activationMarker) {
                        observeCommand(command, chunk, presentation, activationMarker);
                    },
                    [&](const auto &status) {
                        m_snapshot.statusMessages.push_back(status);
                        if (status.kind == ngc::InterpreterStatusKind::Alert) {
                            m_snapshot.operatorAlert = status.text;
                        }
                    });
                lock.lock();
                m_snapshot.programOperation =
                    m_machineSession.programExecution().presentation();
                if (operation.state == ngc::ProgramOperationState::Error) {
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                    m_snapshot.programOperation =
                        ngc::ProgramOperationPresentation::Failed;
                    m_snapshot.error = operation.error.value_or(
                        "machine-session program execution failed");
                    m_running = false;
                    m_programRunning = false;
                }
                if (operation.state == ngc::ProgramOperationState::StopComplete) {
                    const auto joining = m_join;
                    const auto controlledStopPosition = operation.stoppedPosition;
                    m_stop = false;
                    m_running = false;
                    m_programRunning = false;
                    m_snapshot.status = ngc::SimulationStatus::Holding;
                    copyTimingSnapshot();
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    stopTimedSnapshotService();
                    const auto persistenceError =
                        joinGeometry(ngc::ExecutionEpochOutcome::Stopped);
                    if (controlledStopPosition) {
                        m_machineSession.interpreter().machine().synchronizePosition(
                            *controlledStopPosition);
                    }
                    m_machineSession.interpreter().stop();
                    {
                        std::scoped_lock statusLock(m_mutex);
                        m_snapshot.statusMessages = m_machineSession.interpreter().statusMessages();
                        m_snapshot.trajectoryBackendState =
                            ngc::BackendState::Held;
                        m_snapshot.trajectoryBackendActiveNormalRemainingSeconds =
                            0.0;
                        m_snapshot.trajectoryBackendQueuedNormalSeconds = 0.0;
                        m_snapshot.trajectoryBackendCommittedNormalSeconds = 0.0;
                        m_snapshot.trajectoryBackendStopBranchSeconds = 0.0;
                        m_snapshot.trajectoryBackendQueuedExecutionItems = 0;
                        m_snapshot.trajectoryBackendVelocity = 0.0;
                        m_snapshot.trajectoryBackendAcceleration = 0.0;
                        m_snapshot.hasActiveMotion = false;
                        if (persistenceError) {
                            m_snapshot.status = ngc::SimulationStatus::Error;
                            m_snapshot.programOperation =
                                ngc::ProgramOperationPresentation::Failed;
                            m_snapshot.error = *persistenceError;
                        } else {
                            m_snapshot.status = ngc::SimulationStatus::Stopped;
                            m_snapshot.activity = ngc::SimulationActivity::Idle;
                            m_snapshot.programOperation =
                                ngc::ProgramOperationPresentation::Stopped;
                        }
                    }
                    if (joining) {
                        return;
                    }
                    break;
                }
                if (operation.state == ngc::ProgramOperationState::Paused) {
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                    copyTimingSnapshot();
                    m_runtime.releaseRefillOpportunity();
                    m_cv.wait(lock, [&] {
                        return m_join
                            || !m_machineSession.coordinator().commands().empty();
                    });
                    lock.unlock();
                    continue;
                }
                if (m_snapshot.status != ngc::SimulationStatus::Error) {
                    m_snapshot.status =
                        operation.state == ngc::ProgramOperationState::Holding
                        ? ngc::SimulationStatus::Holding : ngc::SimulationStatus::Running;
                }
                if (operation.workDeferred) {
                    copyTimingSnapshot();
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                applyLatestTimedBackendSnapshot();
                copyTimingSnapshot();
                const auto pacingError = m_runtime.snapshot().pacingError;
                if(pacingError != 0) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                    m_snapshot.programOperation =
                        ngc::ProgramOperationPresentation::Failed;
                    m_snapshot.error = "Windows servo pacer failed with error " + std::to_string(pacingError);
                    m_running = false;
                    m_programRunning = false;
                } else if(m_snapshot.status == ngc::SimulationStatus::Error) {
                    m_running = false;
                    m_programRunning = false;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                } else if(operation.state == ngc::ProgramOperationState::Completed) {
                    m_machineSession.completeProgramPresentation();
                }
                if ((operation.state == ngc::ProgramOperationState::Completed
                     || operation.state == ngc::ProgramOperationState::Error)
                    || !m_running) {
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    stopTimedSnapshotService();
                    const auto epochOutcome =
                        operation.state == ngc::ProgramOperationState::Completed
                        ? ngc::ExecutionEpochOutcome::Completed
                        : ngc::ExecutionEpochOutcome::Failed;
                    const auto persistenceError = joinGeometry(epochOutcome);
                    if (operation.state == ngc::ProgramOperationState::Error
                        && operation.error) {
                        m_machineSession.interpreter().reportError(*operation.error);
                    }
                    m_machineSession.interpreter().stop();
                    if (operation.state == ngc::ProgramOperationState::Completed
                        || operation.state == ngc::ProgramOperationState::Error) {
                        std::scoped_lock sessionLock(m_mutex);
                        m_snapshot.statusMessages = m_machineSession.interpreter().statusMessages();
                        if (operation.state == ngc::ProgramOperationState::Completed) {
                            applyActivePresentation(sessionPresentation());
                            m_running = false;
                            m_programRunning = false;
                            m_snapshot.activity = ngc::SimulationActivity::Idle;
                            if (persistenceError) {
                                m_snapshot.status = ngc::SimulationStatus::Error;
                                m_snapshot.programOperation =
                                    ngc::ProgramOperationPresentation::Failed;
                                m_snapshot.error = *persistenceError;
                            } else {
                                m_snapshot.status = ngc::SimulationStatus::Completed;
                                m_snapshot.programOperation =
                                    ngc::ProgramOperationPresentation::Completed;
                            }
                        } else {
                            if (persistenceError && m_snapshot.error.empty()) {
                                m_snapshot.error = *persistenceError;
                            }
                            m_snapshot.status = ngc::SimulationStatus::Error;
                        }
                    }
                    break;
                }

                const auto filled = operation.pumped > 0;
                m_snapshot.trajectoryPlanning = m_machineSession.driver().planningDiagnostics();
                copyTimingSnapshot();
                m_runtime.releaseRefillOpportunity();
                m_runtime.setRollingSupplyActive(
                    m_runtime.tickMultiplier() > 1
                    && m_machineSession.driver().hasUnpublishedRollingContinuation()
                    && !m_machineSession.driver().hasPendingPublication());
                if(filled) {
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                nrtRefillGuard.release();
                m_cv.wait_for(lock,
                               std::chrono::duration<double>(m_runtime.servoPeriod()),
                               [&] {
                                   return m_join
                                       || m_machineSession.programExecution().paused()
                                       || !m_machineSession.coordinator().commands().empty();
                               });
                lock.unlock();
            }
        }
    }
};

}

struct MachineSessionManagerSnapshots {
    std::optional<ngc::SimulationSnapshot> simulation;
    std::optional<ngc::MachineSessionSnapshot> machine;
};

class MachineSessionManager {
public:
    struct InProcessDualSessionTestTag {};
    static constexpr InProcessDualSessionTestTag inProcessDualSessionForTesting {};

private:
    mutable std::mutex m_mutex;
    std::unique_ptr<detail::MachineSessionHost> m_simulation;
    std::unique_ptr<detail::MachineSessionHost> m_machine;
    MachineControlAuthority m_controlAuthority {
        .target = MachineControlTarget::Simulation,
        .generation = 1,
    };
    bool m_guiEmergencyStopLatched = false;

    [[nodiscard]] detail::MachineSessionHost *sessionLocked(
            const MachineControlTarget target) {
        return target == MachineControlTarget::Simulation ? m_simulation.get() : m_machine.get();
    }

    [[nodiscard]] const detail::MachineSessionHost *sessionLocked(
            const MachineControlTarget target) const {
        return target == MachineControlTarget::Simulation ? m_simulation.get() : m_machine.get();
    }

    [[nodiscard]] detail::MachineSessionHost *controlledSessionLocked(
            const MachineControlAuthority authority) {
        if (authority != m_controlAuthority) {
            return nullptr;
        }

        return sessionLocked(authority.target);
    }

    [[nodiscard]] const detail::MachineSessionHost *controlledSessionLocked(
            const MachineControlAuthority authority) const {
        if (authority != m_controlAuthority) {
            return nullptr;
        }

        return sessionLocked(authority.target);
    }

    [[nodiscard]] detail::MachineSessionHost &controlledSessionLocked() {
        return *sessionLocked(m_controlAuthority.target);
    }

    [[nodiscard]] const detail::MachineSessionHost &controlledSessionLocked() const {
        return *sessionLocked(m_controlAuthority.target);
    }

    static MachineControlAuthority localAuthority(
            const detail::MachineSessionHost &session) {
        return session.state().authority;
    }

    static std::expected<void, std::string> staleControlAuthority() {
        return std::unexpected(
            "control has transferred or the request targets another session");
    }

public:
    explicit MachineSessionManager(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                                   const ngc::TrajectoryLimits limits = {},
                                   const ngc::SimulationTiming timing = {})
        : m_simulation(std::make_unique<detail::MachineSessionHost>(
              unit, limits, timing)) {}

    explicit MachineSessionManager(const ngc::MachineConfiguration &configuration)
#ifdef __linux__
        : m_simulation(
              std::make_unique<detail::MachineSessionHost>(configuration)),
          m_machine(configuration.machineExecutor
              ? std::make_unique<detail::MachineSessionHost>(
                    configuration, MachineControlTarget::Machine, true)
              : nullptr)
#else
        : m_simulation(
              std::make_unique<detail::MachineSessionHost>(configuration))
#endif
          {}

    explicit MachineSessionManager(const InProcessDualSessionTestTag,
                                   const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                                   const ngc::TrajectoryLimits limits = {},
                                   const ngc::SimulationTiming timing = {})
        : m_simulation(std::make_unique<detail::MachineSessionHost>(
              unit, limits, timing, MachineControlTarget::Simulation)),
          m_machine(std::make_unique<detail::MachineSessionHost>(
              unit, limits, timing, MachineControlTarget::Machine)) {}

    explicit MachineSessionManager(const InProcessDualSessionTestTag,
                                   const ngc::MachineConfiguration &configuration)
        : m_simulation(std::make_unique<detail::MachineSessionHost>(
              configuration, MachineControlTarget::Simulation)),
          m_machine(std::make_unique<detail::MachineSessionHost>(
              configuration, MachineControlTarget::Machine)) {}

    ~MachineSessionManager() { join(); }
    MachineSessionManager(const MachineSessionManager &) = delete;
    MachineSessionManager &operator=(const MachineSessionManager &) = delete;

    MachineSessionManagerState state() const {
        std::scoped_lock lock(m_mutex);

        return {
            .authority = m_controlAuthority,
            .simulationAvailable = m_simulation != nullptr,
            .machineAvailable = m_machine != nullptr,
            .guiEmergencyStopLatched = m_guiEmergencyStopLatched,
        };
    }

    std::expected<MachineControlAuthority, std::string> selectControlTarget(
            const MachineControlTarget target) {
        std::scoped_lock lock(m_mutex);
        auto *targetSession = sessionLocked(target);
        if (!targetSession) {
            return std::unexpected(target == MachineControlTarget::Machine
                ? "the Machine session is not configured"
                : "the Simulation session is not configured");
        }
        if (target == m_controlAuthority.target) {
            return m_controlAuthority;
        }
        if (!controlledSessionLocked().controllerDataMutable()
            || !targetSession->controllerDataMutable()) {
            return std::unexpected(
                "control transfer requires both machine sessions to be stationary and idle");
        }

        m_controlAuthority.target = target;
        ++m_controlAuthority.generation;

        return m_controlAuthority;
    }

    std::expected<MachineControlAuthority, std::string> simulateFromMachine(
            const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        if (authority != m_controlAuthority
            || authority.target != MachineControlTarget::Machine) {
            return std::unexpected(
                "control has transferred or the request targets another session");
        }
        if (!m_machine) {
            return std::unexpected("the Machine session is not configured");
        }
        if (!m_simulation) {
            return std::unexpected("the Simulation session is not configured");
        }
        if (m_simulation->snapshot().powerState != ngc::MachinePowerState::Off
            || !m_simulation->controllerDataMutable()) {
            return std::unexpected(
                "Machine-to-Simulation import requires Simulation to be powered off and idle");
        }

        const auto checkpoint = m_machine->checkpoint(localAuthority(*m_machine));
        if (!checkpoint) {
            return std::unexpected(checkpoint.error());
        }
        if (const auto restored = m_simulation->restoreCheckpoint(*checkpoint);
            !restored) {
            return std::unexpected(restored.error());
        }
        if (const auto powered =
                m_simulation->powerOn(localAuthority(*m_simulation));
            !powered) {
            return std::unexpected(
                "Simulation failed to power on after checkpoint import");
        }

        m_controlAuthority.target = MachineControlTarget::Simulation;
        ++m_controlAuthority.generation;

        return m_controlAuthority;
    }

    bool hasControlAuthority(const MachineControlAuthority authority) const {
        std::scoped_lock lock(m_mutex);

        return authority == m_controlAuthority;
    }

    SessionCommandResult powerOn(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        if (m_guiEmergencyStopLatched) {
            return {SessionCommandRejection::CommandUnavailable};
        }
        auto *session = controlledSessionLocked(authority);

        return session ? session->powerOn(localAuthority(*session))
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    SessionCommandResult powerOff(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session ? session->powerOff(localAuthority(*session))
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    bool emergencyStop() {
        std::scoped_lock lock(m_mutex);
        m_guiEmergencyStopLatched = true;
        if (m_simulation) {
            static_cast<void>(m_simulation->emergencyStop());
        }
        if (m_machine) {
            static_cast<void>(m_machine->emergencyStop());
        }

        return true;
    }

    std::expected<void, std::string> resetEmergencyStop() {
        std::scoped_lock lock(m_mutex);
        for (auto *session : {m_simulation.get(), m_machine.get()}) {
            if (!session
                || session->snapshot().powerState
                    != ngc::MachinePowerState::Faulted) {
                continue;
            }
            if (auto reset = session->resetEmergencyStop(); !reset) {
                return reset;
            }
        }
        m_guiEmergencyStopLatched = false;

        return {};
    }

    SessionCommandResult start(const MachineControlAuthority authority,
                               const std::vector<std::tuple<std::string, std::string>> &programs,
                               const ngc::ToolTable &tools, const bool preserveState = false) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session
            ? session->start(localAuthority(*session), programs, tools, preserveState)
            : SessionCommandResult { SessionCommandRejection::StaleControlAuthority };
    }

    SessionCommandResult home(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session ? session->home(localAuthority(*session))
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    bool homingAvailable() const {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().homingAvailable();
    }

    SessionCommandResult startJog(const MachineControlAuthority authority,
                                  const ngc::ControlRequest &request) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session ? session->startJog(localAuthority(*session), request)
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    bool renewJog(const MachineControlAuthority authority, const ngc::RequestId request,
                  const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session && session->renewJog(localAuthority(*session), request, jog);
    }

    bool setJogVelocity(const MachineControlAuthority authority,
                        const ngc::SetContinuousJogVelocityRequest &request) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session && session->setJogVelocity(localAuthority(*session), request);
    }

    std::expected<void, std::string> setActiveWorkCoordinate(
            const MachineControlAuthority authority, const ngc::Machine::Axis axis,
            const double workPosition) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);
        if (!session) {
            return staleControlAuthority();
        }

        return session->setActiveWorkCoordinate(
            localAuthority(*session), axis, workPosition);
    }

    bool stopJog(const MachineControlAuthority authority, const ngc::RequestId request,
                 const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session && session->stopJog(localAuthority(*session), request, jog);
    }

    SessionCommandResult feedHold(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session ? session->feedHold(localAuthority(*session))
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    SessionCommandResult resume(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session ? session->resume(localAuthority(*session))
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    SessionCommandResult stop(const MachineControlAuthority authority) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session ? session->stop(localAuthority(*session))
                       : SessionCommandResult {
                           SessionCommandRejection::StaleControlAuthority };
    }

    void setTickMultiplier(const int multiplier) {
        std::scoped_lock lock(m_mutex);
        controlledSessionLocked().setTickMultiplier(multiplier);
    }

    bool setSplineFitSolver(const ngc::spline_detail::SplineFitSolver solver) {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().setSplineFitSolver(solver);
    }

    bool setContinuousPlanningEffort(const ngc::ContinuousPlanningEffort &effort) {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().setContinuousPlanningEffort(effort);
    }

    bool setContinuousDiagnosticCallback(std::function<void(
            const ngc::ContinuousTrajectoryPlan &,
            std::span<const ngc::TrajectoryPlannerInput>)> callback) {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().setContinuousDiagnosticCallback(
            std::move(callback));
    }

    void setRapidSpeed(const double speed) {
        std::scoped_lock lock(m_mutex);
        controlledSessionLocked().setRapidSpeed(speed);
    }

    ngc::SimulationSnapshot snapshot() const {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().snapshot();
    }

    std::optional<ngc::MachineSessionSnapshot> snapshot(
            const MachineControlTarget target) const {
        std::scoped_lock lock(m_mutex);
        const auto *session = sessionLocked(target);
        if (!session) {
            return std::nullopt;
        }

        return session->snapshot();
    }

    MachineSessionManagerSnapshots snapshots() const {
        std::scoped_lock lock(m_mutex);
        MachineSessionManagerSnapshots result;
        if (m_simulation) {
            result.simulation = m_simulation->snapshot();
        }
        if (m_machine) {
            result.machine = m_machine->snapshot();
        }

        return result;
    }

    bool setToolTable(const MachineControlAuthority authority,
                      const ngc::ToolTable &tools) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);

        return session && session->setToolTable(localAuthority(*session), tools);
    }

    ngc::ToolTable toolTable() const {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().toolTable();
    }

    std::pair<ngc::ToolTable, std::uint64_t> toolTableSnapshot() const {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().toolTableSnapshot();
    }

    ngc::ParameterSnapshot parameterSnapshot() const {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().parameterSnapshot();
    }

    bool controllerDataMutable() const {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().controllerDataMutable();
    }

    std::expected<void, std::string> setToolTableStorePath(
            const MachineControlAuthority authority, const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);
        if (!session) {
            return staleControlAuthority();
        }

        return session->setToolTableStorePath(localAuthority(*session), path);
    }

    std::expected<void, std::string> saveToolTable(
            const MachineControlAuthority authority,
            const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        const auto *session = controlledSessionLocked(authority);
        if (!session) {
            return staleControlAuthority();
        }

        return session->saveToolTable(localAuthority(*session), path);
    }

    std::expected<void, std::string> setPersistentParameterStorePath(
            const MachineControlAuthority authority, const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);
        if (!session) {
            return staleControlAuthority();
        }

        return session->setPersistentParameterStorePath(
            localAuthority(*session), path);
    }

    std::expected<void, std::string> loadPersistentParameters(
            const MachineControlAuthority authority, const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        auto *session = controlledSessionLocked(authority);
        if (!session) {
            return staleControlAuthority();
        }

        return session->loadPersistentParameters(localAuthority(*session), path);
    }

    std::expected<void, std::string> savePersistentParameters(
            const MachineControlAuthority authority,
            const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        const auto *session = controlledSessionLocked(authority);
        if (!session) {
            return staleControlAuthority();
        }

        return session->savePersistentParameters(localAuthority(*session), path);
    }

    std::vector<ngc::ExecutedJerkSample> takeExecutedJerkSamples() {
        std::scoped_lock lock(m_mutex);

        return controlledSessionLocked().takeExecutedJerkSamples();
    }

    void join() {
        std::scoped_lock lock(m_mutex);
        if (m_simulation) {
            m_simulation->join();
        }
        if (m_machine) {
            m_machine->join();
        }
    }
};

}
