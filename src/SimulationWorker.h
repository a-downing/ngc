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
#include "machine/MachineSession.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"
#include "machine/SimulationPresentation.h"
#include "machine/ToolTable.h"
#include "memory/ParameterStore.h"

class SimulationWorker {
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
    ngc::Machine::Unit m_unit;
    ngc::InProcessSimulationRuntime m_runtime;
    ngc::MachineSession m_machineSession;
    ngc::TrajectoryLimits m_limits;
    ngc::SimulationSnapshot m_snapshot;
    ngc::ToolTable m_toolTable;
    std::optional<std::filesystem::path> m_parameterStorePath;
    std::optional<std::filesystem::path> m_toolTableStorePath;
    bool m_toolTableInitialized = false;
    std::uint64_t m_toolTableRevision = 0;
    bool m_join = false;
    bool m_stop = false;
    bool m_controlledStopInProgress = false;
    bool m_controlledStopCompleted = false;
    std::optional<ngc::position_t> m_controlledStopPosition;
    bool m_paused = false;
    bool m_programPaused = false;
    bool m_programResumeRequested = false;
    bool m_running = false;
    bool m_programRunning = false;
    bool m_feedHoldRequested = false;
    bool m_feedHoldInProgress = false;
    bool m_feedHoldHeld = false;
    bool m_feedResumeRequested = false;
    bool m_feedResumeInProgress = false;
    std::optional<ngc::RequestId> m_pendingFeedHoldRequest;
    std::optional<ngc::RequestId> m_pendingFeedResumeRequest;
    std::optional<ngc::RequestId> m_pendingControlledStopRequest;
    std::optional<ngc::JogId> m_activeJog;
    std::vector<ngc::AxisConfiguration> m_axes;
    std::vector<ngc::JointConfiguration> m_joints;
    std::uint32_t m_tickMultiplier = 1;
    ngc::RequestId m_nextFeedHoldRequest = ngc::RequestId { 1 } << 63;

    class ActivityCompletion {
    public:
        explicit ActivityCompletion(ngc::ExecutionCoordinator &coordinator)
            : m_coordinator(coordinator) { }

        ~ActivityCompletion() {
            m_coordinator.finishActivity();
        }

        ActivityCompletion(const ActivityCompletion &) = delete;
        ActivityCompletion &operator=(const ActivityCompletion &) = delete;

    private:
        ngc::ExecutionCoordinator &m_coordinator;
    };

    [[nodiscard]] bool motionOwnedOrQueued() const noexcept {
        return m_running || !m_machineSession.coordinator().commands().empty()
            || m_machineSession.coordinator().activity() != ngc::MachineActivity::Idle
            || m_activeJog.has_value();
    }

public:
    explicit SimulationWorker(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                              const ngc::TrajectoryLimits limits = {},
                              const ngc::SimulationTiming timing = {})
        : m_unit(unit),
          m_runtime(limits, timing),
          m_machineSession(unit, ngc::InterpretationMode::Simulation, m_runtime.endpoint(),
                           limits, geometryPolicy(limits)),
          m_limits(limits) {
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        m_runtime.start();
        (void)m_machineSession.powerOn();
        m_snapshot.powerState = m_machineSession.coordinator().powerState();
        m_thread = std::thread(&SimulationWorker::work, this);
    }
    explicit SimulationWorker(const ngc::MachineConfiguration &configuration)
        : m_unit(configuration.unit),
          m_runtime(configuration),
          m_machineSession(configuration.unit, ngc::InterpretationMode::Simulation,
                           m_runtime.endpoint(), configuration.trajectory,
                           geometryPolicy(configuration.trajectory)),
          m_limits(configuration.trajectory),
          m_axes(configuration.axes), m_joints(configuration.joints),
          m_tickMultiplier(1) {
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        m_machineSession.configureHoming(
            configuration.axes, configuration.joints, configuration.homing);
        m_snapshot.machinePosition = { 6.0, 6.0, -6.0, 0.0, 0.0, 0.0 };
        clearActiveTool();
        m_runtime.start();
        (void)m_machineSession.powerOn();
        m_snapshot.powerState = m_machineSession.coordinator().powerState();
        m_thread = std::thread(&SimulationWorker::work, this);
    }
    ~SimulationWorker() { join(); }
    SimulationWorker(const SimulationWorker &) = delete;
    SimulationWorker &operator=(const SimulationWorker &) = delete;

    bool start(const std::vector<std::tuple<std::string, std::string>> &programs, const ngc::ToolTable &tools,
               const bool preserveState = false) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued() || programs.empty()) {
            return false;
        }
        const auto activity = std::get<1>(programs.back()) == "<MDI>"
            ? ngc::MachineActivity::Mdi : ngc::MachineActivity::Program;
        if (!m_machineSession.coordinator().beginActivity(activity)) {
            return false;
        }
        if (!m_toolTableInitialized) {
            m_toolTable = tools;
            m_machineSession.interpreter().machine().toolTable() = tools;
            m_toolTableInitialized = true;
            ++m_toolTableRevision;
        }
        if (!m_machineSession.coordinator().commands().tryPush(ngc::StartProgram {
                .programs = programs,
                .preserveState = preserveState,
                .activity = activity,
            })) {
            m_machineSession.coordinator().finishActivity();
            return false;
        }
        m_stop = false;
        m_controlledStopInProgress = false;
        m_controlledStopCompleted = false;
        m_controlledStopPosition.reset();
        m_paused = false;
        m_programPaused = false;
        m_programResumeRequested = false;
        m_feedHoldRequested = false;
        m_feedHoldInProgress = false;
        m_feedHoldHeld = false;
        m_feedResumeRequested = false;
        m_feedResumeInProgress = false;
        m_pendingFeedHoldRequest.reset();
        m_pendingFeedResumeRequest.reset();
        m_pendingControlledStopRequest.reset();
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Program;
        m_snapshot.operatorAlert.reset();
        m_cv.notify_all();
        return true;
    }

    bool resetSimulation() {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return false;
        }
        m_machineSession.interpreter().machine().beginProgramRun();
        m_snapshot = {};
        m_snapshot.powerState = ngc::MachinePowerState::On;
        copyRuntimeTimingSnapshot();
        m_machineSession.presentationTracker().reset(sessionPresentation());
        return true;
    }

    bool home() {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued() || !m_machineSession.homingAvailable()) {
            return false;
        }
        if (!m_machineSession.coordinator().beginActivity(ngc::MachineActivity::Homing)) {
            return false;
        }
        if (!m_machineSession.coordinator().commands().tryPush(ngc::StartHoming {})) {
            m_machineSession.coordinator().finishActivity();
            return false;
        }
        m_stop = false;
        m_controlledStopInProgress = false;
        m_controlledStopCompleted = false;
        m_controlledStopPosition.reset();
        m_pendingControlledStopRequest.reset();
        m_paused = false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Homing;
        m_snapshot.error.clear();
        m_cv.notify_all();
        return true;
    }

    bool homingAvailable() const {
        std::scoped_lock lock(m_mutex);
        return m_machineSession.homingAvailable();
    }

    bool startJog(const ngc::ControlRequest &request) {
        const auto jog = std::visit([](const auto &value) -> std::optional<ngc::JogId> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, ngc::StartContinuousJogRequest>
                         || std::same_as<T, ngc::StartIncrementalJogRequest>) return value.jog;
            return std::nullopt;
        }, request);
        std::scoped_lock lock(m_mutex);
        if (!jog || *jog == 0 || motionOwnedOrQueued()) {
            return false;
        }
        if (!m_machineSession.coordinator().beginActivity(ngc::MachineActivity::Jogging)) {
            return false;
        }
        if (!m_machineSession.coordinator().commands().tryPush(ngc::StartJog { request })) {
            m_machineSession.coordinator().finishActivity();
            return false;
        }
        m_stop = false;
        m_controlledStopInProgress = false;
        m_controlledStopCompleted = false;
        m_controlledStopPosition.reset();
        m_pendingControlledStopRequest.reset();
        m_activeJog = *jog;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Jogging;
        m_snapshot.jogging = true;
        m_snapshot.lastJogStopReason.reset();
        m_cv.notify_all();
        return true;
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
        const auto saved = persistParametersAtBoundary();
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

    bool feedHold() {
        std::scoped_lock lock(m_mutex);
        if(!m_running || !m_programRunning || m_paused || m_feedHoldRequested
           || m_feedHoldInProgress || m_feedHoldHeld || m_feedResumeInProgress
           || m_machineSession.coordinator().commands().contains<ngc::FeedHold>()) return false;
        if (!m_machineSession.coordinator().commands().tryPush(ngc::FeedHold {})) return false;
        m_snapshot.status = ngc::SimulationStatus::Holding;
        m_cv.notify_all();
        return true;
    }
    bool resume() {
        std::scoped_lock lock(m_mutex);
        if (!m_running || !m_programRunning || !m_paused) {
            return false;
        }
        if (m_machineSession.coordinator().commands().contains<ngc::Resume>()) {
            return false;
        }
        if (m_programPaused) {
            if (!m_machineSession.coordinator().commands().tryPush(ngc::Resume {})) return false;
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.operatorAlert.reset();
            m_cv.notify_all();
            return true;
        }
        if (!m_feedHoldHeld || m_feedResumeRequested || m_feedResumeInProgress) {
            return false;
        }
        if (!m_machineSession.coordinator().commands().tryPush(ngc::Resume {})) return false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_cv.notify_all();
        return true;
    }
    void stop() {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            m_machineSession.coordinator().commands().clear();
            (void)m_machineSession.coordinator().commands().tryPush(ngc::Stop {});
            m_snapshot.operatorAlert.reset();
            if (m_running) {
                m_snapshot.status = ngc::SimulationStatus::Holding;
            }
            m_cv.notify_all();
        }
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
        }
        if (result.status == ngc::SimulationStatus::Paused && !m_programPaused
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
                       && !m_programPaused)) {
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
        if (motionOwnedOrQueued()) {
            return false;
        }
        m_toolTable = tools;
        m_machineSession.interpreter().machine().toolTable() = tools;
        m_toolTableInitialized = true;
        ++m_toolTableRevision;

        return true;
    }

    ngc::ToolTable toolTable() const {
        std::scoped_lock lock(m_mutex);

        return m_toolTable;
    }

    std::pair<ngc::ToolTable, std::uint64_t> toolTableSnapshot() const {
        std::scoped_lock lock(m_mutex);

        return {m_toolTable, m_toolTableRevision};
    }

    std::expected<void, std::string> setToolTableStorePath(
        const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot configure the tool-table store while motion owns the machine");
        }
        m_toolTableStorePath = path;

        return {};
    }

    std::expected<void, std::string> saveToolTable(
        const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot save the tool table while motion owns the machine");
        }

        return m_toolTable.save(path);
    }

    std::expected<void, std::string> setPersistentParameterStorePath(
        const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected(
                "cannot configure persistent parameters while motion owns the machine");
        }
        m_parameterStorePath = path;

        return {};
    }
    std::expected<void, std::string> loadPersistentParameters(const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected("cannot load persistent parameters while motion owns the machine");
        }

        auto loaded = ngc::loadPersistentParameters(path, m_unit, m_machineSession.interpreter().machine().memory());
        if (loaded) {
            m_parameterStorePath = path;
            m_machineSession.interpreter().machine().beginProgramRun();
            m_machineSession.presentationTracker().setActivePresentation(sessionPresentation());
        }

        return loaded;
    }

    std::expected<void, std::string> savePersistentParameters(const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (motionOwnedOrQueued()) {
            return std::unexpected("cannot save persistent parameters while motion owns the machine");
        }

        return ngc::savePersistentParameters(path, m_unit, m_machineSession.interpreter().machine().memory());
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
            m_cv.notify_all();
        }
        m_thread.join();
        m_runtime.stop();
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

    std::expected<void, std::string> persistParametersAtBoundary() const {
        if (!m_parameterStorePath) {
            return {};
        }

        return ngc::savePersistentParameters(
            *m_parameterStorePath, m_unit, m_machineSession.interpreter().machine().memory());
    }

    std::expected<void, std::string> persistToolTableAtBoundary() {
        const auto &updated = m_machineSession.interpreter().machine().toolTable();
        if (updated == m_toolTable) {
            return {};
        }
        m_toolTable = updated;
        m_toolTableInitialized = true;
        ++m_toolTableRevision;
        if (!m_toolTableStorePath) {
            return {};
        }

        return m_toolTable.save(*m_toolTableStorePath);
    }

    void clearPresentation() {
        m_machineSession.presentationTracker().clearTracking();
    }

    void observeLifecycle(const ngc::InterpreterBlockLifecycle &lifecycle) {
        m_machineSession.presentationTracker().observeLifecycle(lifecycle);
    }

    void observeCommand(const ngc::MachineCommand &command, const ngc::ExecutionItem &item,
                        const ngc::TrajectoryCommandPresentation &captured,
                        const ngc::ExecutionMarkerId activationMarker) {
        if(std::holds_alternative<ngc::ProbeMove>(command)) {
            const auto &move = std::get<ngc::TriggeredMove>(item);
            const auto contact = move.target + captured.tool.offset
                - captured.activeToolOffset;
            (void)m_runtime.configureSyntheticInput(move.moveId, contact);
        }
        m_machineSession.presentationTracker().observeCommand(
            command, item, captured, activationMarker);
    }

    void observeBackendEvent(const ngc::ExecutionEvent &event) {
        if(const auto *accepted = std::get_if<ngc::ChunkAccepted>(&event)) {
            m_machineSession.presentationTracker().observeChunkAccepted(*accepted);
        } else if (const auto *marker =
                       std::get_if<ngc::ExecutionMarkerReached>(&event)) {
            m_machineSession.presentationTracker().observeMarkerReached(*marker);
        } else if(std::holds_alternative<ngc::TriggeredMoveCompleted>(event)) {
            if(m_feedHoldInProgress) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = false;
                m_paused = false;
                m_snapshot.status = ngc::SimulationStatus::Running;
            }
        } else if(const auto *retired = std::get_if<ngc::ChunkRetired>(&event)) {
            m_machineSession.presentationTracker().observeChunkRetired(*retired);
        } else if(const auto *held = std::get_if<ngc::BackendHeld>(&event)) {
            if(held->reason == ngc::BackendHoldReason::FeedHold) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = true;
                m_paused = true;
                m_snapshot.status = ngc::SimulationStatus::Paused;
            } else if (held->reason == ngc::BackendHoldReason::ControlledStop) {
                m_pendingControlledStopRequest.reset();
                m_controlledStopInProgress = false;
                m_controlledStopCompleted = true;
                m_snapshot.machinePosition = held->state.position;
                m_controlledStopPosition = held->state.position;
                m_snapshot.trajectoryBackendVelocity = 0.0;
                m_snapshot.trajectoryBackendAcceleration = 0.0;
                m_snapshot.hasActiveMotion = false;
            }
        } else if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event)) {
            if(m_pendingFeedHoldRequest && completed->request == *m_pendingFeedHoldRequest
               && !completed->succeeded) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_snapshot.status = ngc::SimulationStatus::Running;
            } else if(m_pendingFeedResumeRequest
                      && completed->request == *m_pendingFeedResumeRequest) {
                m_pendingFeedResumeRequest.reset();
                if(completed->succeeded) {
                    m_feedHoldHeld = false;
                    m_snapshot.status = ngc::SimulationStatus::Running;
                } else {
                    m_feedResumeInProgress = false;
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.error = "motion backend rejected the feed-resume request";
                    m_running = false;
                    m_programRunning = false;
                }
            } else if(m_pendingControlledStopRequest
                      && completed->request == *m_pendingControlledStopRequest
                      && !completed->succeeded) {
                m_pendingControlledStopRequest.reset();
                m_controlledStopInProgress = false;
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.error =
                    "motion backend rejected the controlled-stop request";
                m_running = false;
                m_programRunning = false;
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
        if (m_controlledStopCompleted
            && backend.state != ngc::BackendState::Held) {
            return;
        }
        applyBackendObservation(m_snapshot, backend);
        if(m_feedResumeInProgress && backend.state == ngc::BackendState::Running
           && backend.executionRate >= 1.0 - 1e-10
           && std::abs(backend.executionRateAcceleration) <= 1e-10) {
            m_feedResumeInProgress = false;
        }
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

    const ngc::JointConfiguration *configuredJoint(const ngc::JointId id) const {
        const auto found = std::ranges::find(m_joints, id, &ngc::JointConfiguration::id);
        return found == m_joints.end() ? nullptr : &*found;
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

    void applyServiceBackendSnapshot(const ngc::ExecutionSnapshot &backend) {
        m_snapshot.joints = backend.commandedJoints;
        for(const auto &axis : m_axes) {
            double sum = 0.0;
            std::size_t count = 0;
            for(const auto id : axis.joints) {
                const auto *joint = configuredJoint(id);
                if(!joint || std::abs(joint->coordinateScale) <= 1e-12) continue;
                sum += backend.commandedJoints.position[id] / joint->coordinateScale;
                ++count;
            }
            if(count != 0) axisComponent(m_snapshot.machinePosition, axis.axis) = sum / count;
        }
        m_snapshot.commandProgress = backend.spanProgress;
        m_snapshot.hasActiveMotion = backend.state == ngc::BackendState::Running
            && backend.activeJoints != 0;
        clearActiveTool();
    }

    bool submitServiceControl(const ngc::ControlRequest &request) {
        if (m_runtime.endpoint().trySubmit(request) != ngc::SubmitResult::Submitted) {
            return false;
        }
        m_runtime.advanceImmediate(0.0);
        return true;
    }

    void advanceServiceMotionPeriod() {
        const auto ticks = m_runtime.advanceServiceMotionPeriod();

        ngc::ExecutionSnapshot backendSnapshot;
        while (m_runtime.endpoint().tryTakeSnapshot(backendSnapshot)) {
            std::scoped_lock lock(m_mutex);
            applyServiceBackendSnapshot(backendSnapshot);
        }
        std::scoped_lock lock(m_mutex);
        m_snapshot.servoTicks += ticks;
    }

    bool setServiceJointPositions(const ngc::EpochId epoch, const ngc::JointMask joints,
                                  const ngc::JointVector &positions, ngc::RequestId &requestId) {
        const auto id = requestId++;
        if(!submitServiceControl(ngc::SetJointPositionRequest { id, joints, positions })) return false;
        bool succeeded = false;
        ngc::ExecutionEvent event;
        while (m_runtime.endpoint().tryTakeEvent(event))
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == id) succeeded = completed->succeeded;
        ngc::ExecutionSnapshot backendSnapshot;
        while (m_runtime.endpoint().tryTakeSnapshot(backendSnapshot)) {
            std::scoped_lock lock(m_mutex);
            applyServiceBackendSnapshot(backendSnapshot);
        }
        (void)epoch;
        return succeeded;
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

        const ngc::HomingRuntimeCallbacks callbacks {
            .stopRequested = [&] {
                std::scoped_lock lock(m_mutex);
                while (auto command = m_machineSession.coordinator().commands().tryPop()) {
                    if (std::holds_alternative<ngc::Stop>(*command)) {
                        m_stop = true;
                    }
                }
                if (m_join) {
                    m_stop = true;
                }
                if (m_stop) {
                    m_snapshot.status = ngc::SimulationStatus::Holding;
                }

                return m_stop;
            },
            .prepareTriggeredMove = [&](const ngc::TriggeredJointMove &move) {
                for (const auto &trigger : move.triggers) {
                    const auto *joint = configuredJoint(trigger.joint);
                    if (!joint) {
                        return false;
                    }
                    const auto position =
                        joint->homing.switchPosition * joint->coordinateScale;
                    if (!m_runtime.configureSyntheticJointInput(
                            move.moveId, trigger.joint, position)) {
                        return false;
                    }
                }

                return true;
            },
            .serviceImmediate = [&] {
                m_runtime.advanceImmediate(0.0);
            },
            .advanceServiceMotionPeriod = [&] {
                return m_runtime.advanceServiceMotionPeriod();
            },
            .waitForServiceMotion = [&] {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(m_runtime.schedulerPeriod()));
            },
            .observe = [&](const ngc::HomingObservation &observation) {
                std::scoped_lock lock(m_mutex);
                m_snapshot.machinePosition = observation.machinePosition;
                m_snapshot.joints = observation.joints;
                m_snapshot.commandProgress = observation.commandProgress;
                m_snapshot.hasActiveMotion = observation.hasActiveMotion;
                m_snapshot.servoTicks = observation.servoTicks;
                clearActiveTool();
            },
        };
        const auto result =
            m_machineSession.runHoming(startingPosition, callbacks);

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

    void failJog(const std::string &message) {
        std::scoped_lock lock(m_mutex);
        m_snapshot.status = ngc::SimulationStatus::Error;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.error = message;
        m_snapshot.hasActiveMotion = false;
        m_snapshot.jogging = false;
        m_activeJog.reset();
        m_machineSession.coordinator().commands().clear();
        m_running = false;
    }

    void runJogging(const ngc::ControlRequest &firstRequest) {
        ActivityCompletion activityCompletion(m_machineSession.coordinator());
        const auto epoch = m_machineSession.nextEpoch();
        const auto firstRequestId = std::visit([](const auto &request) { return request.id; }, firstRequest);
        ngc::RequestId internalRequest = std::numeric_limits<ngc::RequestId>::max() - 16;
        ngc::JointMask allJoints = 0;
        ngc::JointVector initial;
        {
            std::scoped_lock lock(m_mutex);
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.activity = ngc::SimulationActivity::Jogging;
            m_snapshot.error.clear();
            m_snapshot.jogging = true;
            m_snapshot.homedJoints = m_machineSession.homedJoints();
            for(const auto &joint : m_joints) {
                allJoints |= ngc::JointMask { 1 } << joint.id;
                initial[joint.id] = axisComponent(m_snapshot.machinePosition, joint.axis)
                    * joint.coordinateScale;
            }
            std::visit([&](const auto &request) {
                using T = std::decay_t<decltype(request)>;
                if constexpr(std::same_as<T, ngc::StartContinuousJogRequest>
                             || std::same_as<T, ngc::StartIncrementalJogRequest>)
                    m_snapshot.activeJogTarget = request.target;
            }, firstRequest);
            clearActiveTool();
        }

        if(!submitServiceControl(ngc::ResetRequest { internalRequest--, epoch })
           || !submitServiceControl(ngc::EnableRequest { internalRequest-- })
           || !setServiceJointPositions(epoch, allJoints, initial, internalRequest)
           || m_runtime.endpoint().trySubmit(firstRequest) != ngc::SubmitResult::Submitted) {
            failJog("failed to initialize the mock backend for jogging");
            return;
        }
        m_runtime.advanceImmediate(0.0);

        bool finished = false;
        bool stopSubmitted = false;
        bool sessionStopped = false;
        while(!finished) {
            std::deque<ngc::ControlRequest> controls;
            bool joining = false;
            {
                std::scoped_lock lock(m_mutex);
                joining = m_join;
                while (auto command = m_machineSession.coordinator().commands().tryPop()) {
                    if (const auto *renew = std::get_if<ngc::RenewJog>(&*command)) {
                        controls.emplace_back(
                            ngc::RenewJogLeaseRequest { renew->request, renew->jog });
                    } else if (const auto *update =
                                   std::get_if<ngc::SetJogVelocity>(&*command)) {
                        controls.emplace_back(update->request);
                    } else if (const auto *stop = std::get_if<ngc::StopJog>(&*command)) {
                        controls.emplace_back(
                            ngc::StopJogRequest { stop->request, stop->jog });
                        stopSubmitted = true;
                    } else if (std::holds_alternative<ngc::Stop>(*command)
                               && !stopSubmitted) {
                        controls.emplace_back(
                            ngc::StopJogRequest { internalRequest--, *m_activeJog });
                        stopSubmitted = true;
                        sessionStopped = true;
                    }
                }
                if(joining && !stopSubmitted) {
                    controls.emplace_back(
                        ngc::StopJogRequest { internalRequest--, *m_activeJog });
                    stopSubmitted = true;
                    sessionStopped = true;
                }
            }
            for(const auto &control : controls) {
                if (m_runtime.endpoint().trySubmit(control)
                    != ngc::SubmitResult::Submitted) {
                    failJog("mock backend jog control channel is full");
                    return;
                }
            }

            advanceServiceMotionPeriod();

            ngc::ExecutionEvent event;
            while (m_runtime.endpoint().tryTakeEvent(event)) {
                if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event)) {
                    if(!completed->succeeded && completed->request == firstRequestId) {
                        failJog("mock backend rejected a jog control request");
                        return;
                    }
                } else if(const auto *stopped = std::get_if<ngc::JogStopped>(&event)) {
                    std::scoped_lock lock(m_mutex);
                    if(m_activeJog && stopped->jog == *m_activeJog) {
                        m_snapshot.lastJogStopReason = stopped->reason;
                        finished = true;
                    }
                } else if(const auto *fault = std::get_if<ngc::BackendFault>(&event)) {
                    failJog("mock jogging backend fault " + std::to_string(fault->code));
                    return;
                }
            }
            if (!finished) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(m_runtime.schedulerPeriod()));
            }
            if(joining && finished) break;
        }

        std::scoped_lock lock(m_mutex);
        m_running = false;
        m_activeJog.reset();
        m_snapshot.status = sessionStopped ? ngc::SimulationStatus::Stopped
                                           : ngc::SimulationStatus::Completed;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.jogging = false;
        m_snapshot.hasActiveMotion = false;
        m_snapshot.activeJogTarget.reset();
        m_machineSession.interpreter().machine().synchronizePosition(m_snapshot.machinePosition);
        clearActiveTool();
    }

    void joinGeometry() {
        const auto active = m_machineSession.executionEpochActive();
        const auto diagnostics = m_machineSession.finishExecutionEpoch();
        if (active) {
            std::scoped_lock lock(m_mutex);
            m_snapshot.geometryStream = diagnostics;
        }
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
            auto command = m_machineSession.coordinator().commands().tryPop();
            if (!command) {
                continue;
            }
            if (std::holds_alternative<ngc::StartHoming>(*command)) {
                m_running = true;
                m_stop = false;
                lock.unlock();
                runSessionHoming();
                continue;
            }
            if (auto *jog = std::get_if<ngc::StartJog>(&*command)) {
                auto request = std::move(jog->request);
                m_running = true;
                m_stop = false;
                lock.unlock();
                runJogging(request);
                continue;
            }
            if (std::holds_alternative<ngc::Stop>(*command)) {
                m_snapshot.status = ngc::SimulationStatus::Stopped;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_machineSession.coordinator().finishActivity();
                continue;
            }
            if (!std::holds_alternative<ngc::StartProgram>(*command)) {
                continue;
            }
            auto start = std::get<ngc::StartProgram>(std::move(*command));
            ActivityCompletion activityCompletion(m_machineSession.coordinator());
            auto programs = std::move(start.programs);
            auto tools = m_toolTable;
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
            lock.unlock();

            if(!preserve) {
                std::scoped_lock snapshotLock(m_mutex);
                applyActivePresentation(sessionPresentation());
            }
            start.programs = std::move(programs);
            const auto epochResult = m_machineSession.beginExecutionEpoch(
                std::move(start), std::move(tools), startingPosition);
            if (!epochResult) {
                lock.lock();
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_snapshot.error = epochResult.error();
                m_running = false;
                m_programRunning = false;
                lock.unlock();
                continue;
            }
            const auto epoch = *epochResult;
            if (!m_runtime.beginTimedExecution()) {
                joinGeometry();
                m_machineSession.interpreter().reportError(
                    "in-process Simulation runtime failed to start timed execution");
                m_machineSession.interpreter().stop();
                lock.lock();
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
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
                while (auto command = m_machineSession.coordinator().commands().tryPop()) {
                    if (std::holds_alternative<ngc::Stop>(*command)) {
                        m_stop = true;
                        m_feedHoldRequested = false;
                        m_feedResumeRequested = false;
                        m_programResumeRequested = false;
                        m_snapshot.status = ngc::SimulationStatus::Holding;
                    } else if (std::holds_alternative<ngc::FeedHold>(*command)) {
                        m_feedHoldRequested = true;
                        m_feedHoldInProgress = true;
                    } else if (std::holds_alternative<ngc::Resume>(*command)) {
                        if (m_programPaused) {
                            m_programResumeRequested = true;
                            m_paused = false;
                        } else if (m_feedHoldHeld) {
                            m_feedResumeRequested = true;
                            m_feedResumeInProgress = true;
                            m_paused = false;
                        }
                    }
                }
                if (m_join) {
                    m_stop = true;
                }
                if (m_stop && !m_controlledStopInProgress
                    && !m_controlledStopCompleted && !m_feedHoldInProgress) {
                    const auto alreadyStationary =
                        m_snapshot.trajectoryBackendState == ngc::BackendState::Held
                        && m_snapshot.trajectoryBackendVelocity <= 1e-10
                        && m_snapshot.trajectoryBackendAcceleration <= 1e-10;
                    if (m_programPaused || alreadyStationary) {
                        m_controlledStopCompleted = true;
                        m_controlledStopPosition = m_snapshot.machinePosition;
                    } else {
                        const auto request = m_nextFeedHoldRequest++;
                        if (m_runtime.endpoint().trySubmit(
                                ngc::ControlledStopRequest { request })
                                != ngc::SubmitResult::Submitted) {
                            m_snapshot.status = ngc::SimulationStatus::Error;
                            m_snapshot.error =
                                "motion backend control channel is full while requesting stop";
                            m_running = false;
                            m_programRunning = false;
                        } else {
                            m_pendingControlledStopRequest = request;
                            m_controlledStopInProgress = true;
                        }
                    }
                }
                if(m_controlledStopCompleted) {
                    const auto joining = m_join;
                    m_stop = false;
                    m_controlledStopCompleted = false;
                    m_running = false;
                    m_programRunning = false;
                    m_paused = false;
                    m_programPaused = false;
                    m_feedHoldInProgress = false;
                    m_feedHoldHeld = false;
                    m_feedResumeInProgress = false;
                    m_snapshot.status = ngc::SimulationStatus::Holding;
                    copyTimingSnapshot();
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    stopTimedSnapshotService();
                    joinGeometry();
                    if (m_controlledStopPosition) {
                        m_machineSession.interpreter().machine().synchronizePosition(
                            *m_controlledStopPosition);
                    }
                    m_machineSession.interpreter().stop();
                    {
                        std::scoped_lock statusLock(m_mutex);
                        m_snapshot.statusMessages = m_machineSession.interpreter().statusMessages();
                        if (auto saved = persistToolTableAtBoundary(); !saved) {
                            m_snapshot.status = ngc::SimulationStatus::Error;
                            m_snapshot.error = saved.error();
                        } else {
                            m_snapshot.status = ngc::SimulationStatus::Stopped;
                            m_snapshot.activity = ngc::SimulationActivity::Idle;
                        }
                    }
                    if(joining) return;
                    break;
                }
                if (m_programResumeRequested) {
                    m_programResumeRequested = false;
                    if (!m_machineSession.driver().resumeProgram()) {
                        m_snapshot.status = ngc::SimulationStatus::Error;
                        m_snapshot.error =
                            "prepared trajectory driver rejected the M0 program resume";
                        m_running = false;
                        m_programRunning = false;
                    } else {
                        m_programPaused = false;
                    }
                }
                if(m_feedHoldRequested) {
                    m_feedHoldRequested = false;
                    const auto request = m_nextFeedHoldRequest++;
                    if (m_runtime.endpoint().trySubmit(ngc::FeedHoldRequest { request })
                            != ngc::SubmitResult::Submitted) {
                        m_snapshot.status = ngc::SimulationStatus::Error;
                        m_snapshot.error = "motion backend control channel is full while requesting feed hold";
                        m_running = false;
                        m_programRunning = false;
                    } else m_pendingFeedHoldRequest = request;
                }
                if(m_feedResumeRequested) {
                    m_feedResumeRequested = false;
                    const auto request = m_nextFeedHoldRequest++;
                    if (m_runtime.endpoint().trySubmit(
                            ngc::ResumeRequest { request, epoch })
                            != ngc::SubmitResult::Submitted) {
                        m_snapshot.status = ngc::SimulationStatus::Error;
                        m_snapshot.error = "motion backend control channel is full while resuming feed";
                        m_running = false;
                        m_programRunning = false;
                    } else m_pendingFeedResumeRequest = request;
                }
                if(m_paused) {
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                    copyTimingSnapshot();
                    m_runtime.releaseRefillOpportunity();
                    m_cv.wait(lock, [&] {
                        return m_join || m_stop || !m_paused
                            || !m_machineSession.coordinator().commands().empty();
                    });
                    lock.unlock();
                    continue;
                }
                if(m_snapshot.status != ngc::SimulationStatus::Error)
                    m_snapshot.status = (m_feedHoldInProgress || m_stop)
                        ? ngc::SimulationStatus::Holding : ngc::SimulationStatus::Running;
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
                } nrtRefillGuard{m_runtime, m_runtime.tickMultiplier() > 1};
                if (m_runtime.executorBatchActive()) {
                    copyTimingSnapshot();
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                m_machineSession.driver().serviceBackend([&](const auto &event) { observeBackendEvent(event); });
                if (auto presentation = m_machineSession.driver().takePresentationUpdate()) {
                    applyActivePresentation(*presentation);
                }
                if (m_runtime.executorBatchActive()) {
                    copyTimingSnapshot();
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                applyLatestTimedBackendSnapshot();
                copyTimingSnapshot();
                auto state = m_machineSession.driver().state();
                if (state == ngc::PreparedDriverState::ProgramPaused) {
                    m_programPaused = true;
                    m_paused = true;
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                }
                const auto pacingError = m_runtime.snapshot().pacingError;
                if(pacingError != 0) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                    m_snapshot.error = "Windows servo pacer failed with error " + std::to_string(pacingError);
                    m_running = false;
                    m_programRunning = false;
                } else if(m_snapshot.status == ngc::SimulationStatus::Error) {
                    m_running = false;
                    m_programRunning = false;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                } else if(state == ngc::PreparedDriverState::Error) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                    m_snapshot.error = *m_machineSession.driver().error(); m_running = false; m_programRunning = false;
                } else if(state == ngc::PreparedDriverState::Completed) {
                    m_machineSession.presentationTracker().completeDeferredBlocks();
                }
                if ((state == ngc::PreparedDriverState::Completed
                     || state == ngc::PreparedDriverState::Error)
                    || !m_running) {
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    stopTimedSnapshotService();
                    joinGeometry();
                    if(state == ngc::PreparedDriverState::Error && m_machineSession.driver().error())
                        m_machineSession.interpreter().reportError(*m_machineSession.driver().error());
                    m_machineSession.interpreter().stop();
                    if(state == ngc::PreparedDriverState::Completed
                       ||state == ngc::PreparedDriverState::Error) {
                        std::scoped_lock sessionLock(m_mutex);
                        m_snapshot.statusMessages = m_machineSession.interpreter().statusMessages();
                        const auto toolsSaved = persistToolTableAtBoundary();
                        if(state == ngc::PreparedDriverState::Completed) {
                            applyActivePresentation(sessionPresentation());
                            m_running = false;
                            m_programRunning = false;
                            m_snapshot.activity = ngc::SimulationActivity::Idle;
                            const auto parametersSaved = persistParametersAtBoundary();
                            if (!toolsSaved) {
                                m_snapshot.status = ngc::SimulationStatus::Error;
                                m_snapshot.error = toolsSaved.error();
                            } else if (!parametersSaved) {
                                m_snapshot.status = ngc::SimulationStatus::Error;
                                m_snapshot.error = parametersSaved.error();
                            } else {
                                m_snapshot.status = ngc::SimulationStatus::Completed;
                            }
                        } else if (!toolsSaved && m_snapshot.error.empty()) {
                            m_snapshot.error = toolsSaved.error();
                        }
                    }
                    break;
                }

                bool filled = false;
                for(int fill = 0; fill < 64; ++fill) {
                    lock.unlock();
                    const auto pumped=m_machineSession.driver().pumpOne(
                        [&](const auto &command, const auto &chunk, const auto &,
                            const auto &presentation,
                            const ngc::ExecutionMarkerId activationMarker) {
                            std::scoped_lock presentationLock(m_mutex);
                            observeCommand(command, chunk, presentation,
                                activationMarker);
                        },
                        [&](const auto &lifecycle) {
                            std::scoped_lock presentationLock(m_mutex);
                            observeLifecycle(lifecycle);
                        },
                        [&](const auto &status) {
                            std::scoped_lock presentationLock(m_mutex);
                            m_snapshot.statusMessages.push_back(status);
                            if (status.kind == ngc::InterpreterStatusKind::Alert) {
                                m_snapshot.operatorAlert = status.text;
                            }
                        });
                    lock.lock();
                    if (auto presentation = m_machineSession.driver().takePresentationUpdate()) {
                        applyActivePresentation(*presentation);
                    }
                    if(!pumped) break;
                    filled = true;
                    if(m_join||m_stop||m_paused||!m_machineSession.coordinator().commands().empty()) break;
                }
                m_snapshot.trajectoryPlanning = m_machineSession.driver().planningDiagnostics();
                copyTimingSnapshot();
                m_runtime.releaseRefillOpportunity();
                m_runtime.setRollingSupplyActive(
                    m_runtime.tickMultiplier() > 1
                    && m_machineSession.driver().hasUnpublishedRollingContinuation()
                    && !m_machineSession.driver().hasPendingPublication());
                state = m_machineSession.driver().state();
                if(state == ngc::PreparedDriverState::Error) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                    m_snapshot.error = *m_machineSession.driver().error();
                    m_running = false;
                    m_programRunning = false;
                } else if (state == ngc::PreparedDriverState::ProgramPaused) {
                    m_programPaused = true;
                    m_paused = true;
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                }
                if(!m_running) {
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    stopTimedSnapshotService();
                    joinGeometry();
                    if(state == ngc::PreparedDriverState::Error && m_machineSession.driver().error())
                        m_machineSession.interpreter().reportError(*m_machineSession.driver().error());
                    m_machineSession.interpreter().stop();
                    {
                        std::scoped_lock statusLock(m_mutex);
                        m_snapshot.statusMessages = m_machineSession.interpreter().statusMessages();
                    }
                    break;
                }
                if(filled) {
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                nrtRefillGuard.release();
                m_cv.wait_for(lock,
                              std::chrono::duration<double>(m_runtime.servoPeriod()),
                              [&] {
                                  return m_join || m_stop || m_paused
                                      || !m_machineSession.coordinator().commands().empty();
                              });
                lock.unlock();
            }
        }
    }
};
