#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <expected>
#include <mutex>
#include <ranges>
#include <thread>
#include <tuple>
#include <string>
#include <vector>

#include "machine/GeometryStreamProducer.h"
#include "machine/InProcessSimulationRuntime.h"
#include "machine/MachineConfiguration.h"
#include "machine/MachineSessionCommand.h"
#include "machine/MachineSession.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"
#include "machine/SimulationPresentation.h"
#include "machine/ToolTable.h"
#include "memory/ParameterStore.h"

namespace ngc {

enum class MachineControlTarget {
    Simulation,
    Real,
};

struct MachineControlAuthority {
    MachineControlTarget target = MachineControlTarget::Simulation;
    std::uint64_t generation = 0;

    bool operator==(const MachineControlAuthority &) const = default;
};

struct MachineSessionManagerState {
    MachineControlAuthority authority;
    bool simulationAvailable = true;
    bool realAvailable = false;
};

class MachineSessionManager {
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
    ngc::InProcessSimulationRuntime m_runtime;
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
    MachineControlAuthority m_controlAuthority {
        .target = MachineControlTarget::Simulation,
        .generation = 1,
    };

    [[nodiscard]] bool motionOwnedOrQueued() const noexcept {
        return m_running || !m_machineSession.coordinator().commands().empty()
            || m_machineSession.coordinator().activity() != ngc::MachineActivity::Idle
            || m_activeJog.has_value();
    }

public:
    explicit MachineSessionManager(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                              const ngc::TrajectoryLimits limits = {},
                              const ngc::SimulationTiming timing = {})
        : m_runtime(limits, timing),
          m_machineSession(unit, ngc::InterpretationMode::Simulation, m_runtime,
                           limits, geometryPolicy(limits)),
          m_limits(limits) {
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        refreshParameterSnapshot();
        (void)m_machineSession.powerOn();
        m_snapshot.powerState = m_machineSession.coordinator().powerState();
        m_thread = std::thread(&MachineSessionManager::work, this);
    }
    explicit MachineSessionManager(const ngc::MachineConfiguration &configuration)
        : m_runtime(configuration),
          m_machineSession(configuration.unit, ngc::InterpretationMode::Simulation,
                           m_runtime, configuration.trajectory,
                           geometryPolicy(configuration.trajectory)),
          m_limits(configuration.trajectory),
          m_axes(configuration.axes), m_joints(configuration.joints),
          m_tickMultiplier(1) {
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        m_machineSession.configureJogging(configuration.axes, configuration.joints);
        m_machineSession.configureHoming(
            configuration.axes, configuration.joints, configuration.homing);
        clearActiveTool();
        refreshParameterSnapshot();
        (void)m_machineSession.powerOn();
        m_snapshot.powerState = m_machineSession.coordinator().powerState();
        m_thread = std::thread(&MachineSessionManager::work, this);
    }
    ~MachineSessionManager() { join(); }
    MachineSessionManager(const MachineSessionManager &) = delete;
    MachineSessionManager &operator=(const MachineSessionManager &) = delete;

    MachineSessionManagerState state() const {
        std::scoped_lock lock(m_mutex);

        return {
            .authority = m_controlAuthority,
            .simulationAvailable = true,
            .realAvailable = false,
        };
    }

    std::expected<MachineControlAuthority, std::string> selectControlTarget(
        const MachineControlTarget target) {
        std::scoped_lock lock(m_mutex);
        if (target == MachineControlTarget::Real) {
            return std::unexpected("the Real machine session is not configured");
        }
        if (target != m_controlAuthority.target) {
            m_controlAuthority.target = target;
            ++m_controlAuthority.generation;
        }

        return m_controlAuthority;
    }

    bool hasControlAuthority(const MachineControlAuthority authority) const {
        std::scoped_lock lock(m_mutex);

        return authority == m_controlAuthority;
    }

    SessionCommandResult start(const std::vector<std::tuple<std::string, std::string>> &programs,
                               const ngc::ToolTable &tools, const bool preserveState = false) {
        std::scoped_lock lock(m_mutex);
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        if (motionOwnedOrQueued()) {
            return { SessionCommandRejection::MotionOwned };
        }
        if (programs.empty()) {
            return { SessionCommandRejection::EmptyProgram };
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

    SessionCommandResult resetSimulation() {
        std::scoped_lock lock(m_mutex);
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
        }
        if (motionOwnedOrQueued()) {
            return { SessionCommandRejection::MotionOwned };
        }
        m_machineSession.interpreter().machine().beginProgramRun();
        refreshParameterSnapshot();
        m_snapshot = {};
        m_snapshot.powerState = ngc::MachinePowerState::On;
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());

        return {};
    }

    SessionCommandResult home() {
        std::scoped_lock lock(m_mutex);
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
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

    SessionCommandResult startJog(const ngc::ControlRequest &request) {
        const auto jog = std::visit([](const auto &value) -> std::optional<ngc::JogId> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, ngc::StartContinuousJogRequest>
                         || std::same_as<T, ngc::StartIncrementalJogRequest>) return value.jog;
            return std::nullopt;
        }, request);
        std::scoped_lock lock(m_mutex);
        if (m_machineSession.coordinator().powerState() != ngc::MachinePowerState::On) {
            return { SessionCommandRejection::SessionNotPowered };
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

    bool renewJog(const ngc::RequestId request, const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        if(!m_activeJog || *m_activeJog != jog) return false;
        if(m_machineSession.coordinator().commands().anyOf([&](const auto &command) {
            const auto *renewal = std::get_if<ngc::RenewJog>(&command);
            return renewal && renewal->jog == jog;
        })) return true;
        if (!m_machineSession.coordinator().commands().tryPush(ngc::RenewJog { request, jog })) return false;
        m_cv.notify_all();
        return true;
    }

    bool setJogVelocity(const ngc::SetContinuousJogVelocityRequest &request) {
        std::scoped_lock lock(m_mutex);
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
    setActiveWorkCoordinate(const ngc::Machine::Axis axis, const double workPosition) {
        std::scoped_lock lock(m_mutex);
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

    bool stopJog(const ngc::RequestId request, const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
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

    SessionCommandResult feedHold() {
        std::scoped_lock lock(m_mutex);
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
    SessionCommandResult resume() {
        std::scoped_lock lock(m_mutex);
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
    SessionCommandResult stop() {
        std::scoped_lock lock(m_mutex);
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
            }
        } else if (result.activity == ngc::SimulationActivity::Jogging) {
            if (const auto observation = m_machineSession.joggingObservation()) {
                result.machinePosition = observation->machinePosition;
                result.joints = observation->joints;
                result.commandProgress = observation->commandProgress;
                result.hasActiveMotion = observation->hasActiveMotion;
                result.servoTicks = observation->servoTicks;
            }
        }
        if (result.status == ngc::SimulationStatus::Paused
            && !m_machineSession.programExecution().programPaused()
            && result.trajectoryBackendState != ngc::BackendState::Held) {
            result.status = ngc::SimulationStatus::Holding;
        }
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
        result.powerState = m_machineSession.coordinator().powerState();
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

    bool setToolTable(const ngc::ToolTable &tools) {
        std::scoped_lock lock(m_mutex);
        return !motionOwnedOrQueued() && m_machineSession.setToolTable(tools);
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

    std::expected<void, std::string> setToolTableStorePath(
        const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot configure the tool-table store while motion owns the machine");
        }
        return m_machineSession.setToolTableStorePath(path);
    }

    std::expected<void, std::string> saveToolTable(
        const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot save the tool table while motion owns the machine");
        }

        return m_machineSession.saveToolTable(path);
    }

    std::expected<void, std::string> setPersistentParameterStorePath(
        const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot configure persistent parameters while motion owns the machine");
        }
        return m_machineSession.setPersistentParameterStorePath(path);
    }
    std::expected<void, std::string> loadPersistentParameters(const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
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

    std::expected<void, std::string> savePersistentParameters(const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
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
            (void)m_machineSession.powerOff();
            m_snapshot.powerState = m_machineSession.coordinator().powerState();
            m_snapshot.machineActivity = ngc::MachineActivity::Idle;
        }
    }

private:
    void startTimedSnapshotService() {
        {
            std::scoped_lock lock(m_timedSnapshotMutex);
            m_stopTimedSnapshotService = false;
            m_latestTimedBackendSnapshot.reset();
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
            target.error = "mock motion backend fault " + std::to_string(backend.faultCode);
        }
        target.machinePosition = backend.commanded.position;
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
        m_running = false;
        m_stop = false;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.hasActiveMotion = false;
        if (!result) {
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
            m_cv.wait(lock, [&] {
                return m_join || !m_machineSession.coordinator().commands().empty();
            });
            if (m_join) {
                return;
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
            const auto startingPosition = preserve ? m_snapshot.machinePosition : ngc::position_t{};
            m_running = true; m_programRunning = true; m_stop = false;
            if(!preserve) {
                m_snapshot = {};
                m_snapshot.powerState = m_machineSession.coordinator().powerState();
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
                    "in-process Simulation runtime failed to start timed execution");
                m_machineSession.interpreter().stop();
                lock.lock();
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_snapshot.programOperation = ngc::ProgramOperationPresentation::Failed;
                m_snapshot.error = "Simulation servo scheduler failed to start";
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
                    ngc::InProcessSimulationRuntime &runtime;
                    bool enabled = false;
                    NrtRefillGuard(ngc::InProcessSimulationRuntime &value, const bool enable)
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
                    m_snapshot.status = ngc::SimulationStatus::Error;
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
                        } else if (persistenceError && m_snapshot.error.empty()) {
                            m_snapshot.error = *persistenceError;
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
