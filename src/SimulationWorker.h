#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <expected>
#include <mutex>
#include <memory>
#include <ranges>
#include <thread>
#include <tuple>
#include <string>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/MachineConfiguration.h"
#include "machine/InProcessSimulationRuntime.h"
#include "machine/PresentationTracker.h"
#include "machine/SimulationPresentation.h"
#include "machine/ToolTable.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"
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

    static constexpr std::size_t MAX_PENDING_JOG_CONTROLS = 16;
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;
    std::thread m_geometryThread;
    ngc::InterpreterSession m_session;
    ngc::Machine::Unit m_unit;
    ngc::GeometryStreamPolicy m_geometryPolicy;
    ngc::InProcessSimulationRuntime m_runtime;
    ngc::PreparedGeometryForwardChannel m_geometryForward;
    ngc::GeometryFeedbackChannel m_geometryFeedback;
    std::atomic<bool> m_geometryCancelled{false};
    std::unique_ptr<ngc::GeometryStreamProducer> m_geometryProducer;
    ngc::PreparedTrajectoryExecutionDriver m_driver;
    ngc::TrajectoryLimits m_limits;
    ngc::SimulationSnapshot m_snapshot;
    ngc::PresentationTracker m_presentationTracker;
    std::vector<std::tuple<std::string, std::string>> m_programs;
    ngc::ToolTable m_toolTable;
    std::optional<std::filesystem::path> m_parameterStorePath;
    std::optional<std::filesystem::path> m_toolTableStorePath;
    bool m_toolTableInitialized = false;
    std::uint64_t m_toolTableRevision = 0;
    bool m_join = false;
    bool m_start = false;
    bool m_stop = false;
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
    bool m_preserveState = false;
    bool m_home = false;
    std::deque<ngc::ControlRequest> m_jogControls;
    std::optional<ngc::JogId> m_activeJog;
    ngc::JointMask m_homedJoints = 0;
    std::vector<ngc::AxisConfiguration> m_axes;
    std::vector<ngc::JointConfiguration> m_joints;
    ngc::HomingConfiguration m_homing;
    std::uint32_t m_tickMultiplier = 1;
    ngc::EpochId m_nextEpoch = 1;
    ngc::RequestId m_nextFeedHoldRequest = ngc::RequestId { 1 } << 63;

public:
    explicit SimulationWorker(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                              const ngc::TrajectoryLimits limits = {},
                              const ngc::SimulationTiming timing = {})
        : m_session(unit, ngc::InterpretationMode::Simulation),
          m_unit(unit),
          m_geometryPolicy(geometryPolicy(limits)),
          m_runtime(limits, timing),
          m_driver(m_runtime.endpoint(), m_geometryForward, m_geometryFeedback,
                   m_geometryCancelled, limits),
          m_limits(limits) {
        copyRuntimeTimingSnapshot();
        m_presentationTracker.reset(sessionPresentation());
        m_runtime.start();
        m_thread = std::thread(&SimulationWorker::work, this);
    }
    explicit SimulationWorker(const ngc::MachineConfiguration &configuration)
        : m_session(configuration.unit, ngc::InterpretationMode::Simulation),
          m_unit(configuration.unit),
          m_geometryPolicy(geometryPolicy(configuration.trajectory)),
          m_runtime(configuration),
          m_driver(m_runtime.endpoint(), m_geometryForward, m_geometryFeedback,
                   m_geometryCancelled, configuration.trajectory),
          m_limits(configuration.trajectory),
          m_axes(configuration.axes), m_joints(configuration.joints), m_homing(configuration.homing),
          m_tickMultiplier(1) {
        copyRuntimeTimingSnapshot();
        m_presentationTracker.reset(sessionPresentation());
        m_snapshot.machinePosition = { 6.0, 6.0, -6.0, 0.0, 0.0, 0.0 };
        clearActiveTool();
        m_runtime.start();
        m_thread = std::thread(&SimulationWorker::work, this);
    }
    ~SimulationWorker() { join(); }
    SimulationWorker(const SimulationWorker &) = delete;
    SimulationWorker &operator=(const SimulationWorker &) = delete;

    bool start(const std::vector<std::tuple<std::string, std::string>> &programs, const ngc::ToolTable &tools,
               const bool preserveState = false) {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog || programs.empty()) return false;
        m_programs = programs;
        if (!m_toolTableInitialized) {
            m_toolTable = tools;
            m_session.machine().toolTable() = tools;
            m_toolTableInitialized = true;
            ++m_toolTableRevision;
        }
        m_preserveState = preserveState;
        m_start = true;
        m_stop = false;
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
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Program;
        m_snapshot.operatorAlert.reset();
        m_cv.notify_all();
        return true;
    }

    bool resetSimulation() {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog) return false;
        m_session.machine().beginProgramRun();
        m_snapshot = {};
        copyRuntimeTimingSnapshot();
        m_presentationTracker.reset(sessionPresentation());
        m_programs.clear();
        return true;
    }

    bool home() {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog || m_joints.empty() || m_homing.groups.empty()) return false;
        m_home = true;
        m_stop = false;
        m_paused = false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_snapshot.activity = ngc::SimulationActivity::Homing;
        m_snapshot.error.clear();
        m_cv.notify_all();
        return true;
    }

    bool homingAvailable() const {
        std::scoped_lock lock(m_mutex);
        return !m_joints.empty() && !m_homing.groups.empty();
    }

    bool startJog(const ngc::ControlRequest &request) {
        const auto jog = std::visit([](const auto &value) -> std::optional<ngc::JogId> {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, ngc::StartContinuousJogRequest>
                         || std::same_as<T, ngc::StartIncrementalJogRequest>) return value.jog;
            return std::nullopt;
        }, request);
        std::scoped_lock lock(m_mutex);
        if(!jog || *jog == 0 || m_running || m_start || m_home || m_activeJog) return false;
        m_activeJog = *jog;
        m_jogControls.push_back(request);
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
        if(std::ranges::any_of(m_jogControls, [&](const auto &control) {
            const auto *renewal = std::get_if<ngc::RenewJogLeaseRequest>(&control);
            return renewal && renewal->jog == jog;
        })) return true;
        if(m_jogControls.size() >= MAX_PENDING_JOG_CONTROLS) return false;
        m_jogControls.emplace_back(ngc::RenewJogLeaseRequest { request, jog });
        m_cv.notify_all();
        return true;
    }

    bool setJogVelocity(const ngc::SetContinuousJogVelocityRequest &request) {
        std::scoped_lock lock(m_mutex);
        if(!m_activeJog || *m_activeJog != request.jog) return false;
        std::erase_if(m_jogControls, [&](const auto &control) {
            if(const auto *renewal = std::get_if<ngc::RenewJogLeaseRequest>(&control))
                return renewal->jog == request.jog;
            if(const auto *update = std::get_if<ngc::SetContinuousJogVelocityRequest>(&control))
                return update->jog == request.jog;
            return false;
        });
        if(m_jogControls.size() >= MAX_PENDING_JOG_CONTROLS) return false;
        m_jogControls.emplace_back(request);
        m_cv.notify_all();
        return true;
    }

    std::expected<void, std::string>
    setActiveWorkCoordinate(const ngc::Machine::Axis axis, const double workPosition) {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog
           || m_snapshot.status == ngc::SimulationStatus::Error)
            return std::unexpected("cannot change a work offset while motion owns the machine");
        if(!std::isfinite(workPosition))
            return std::unexpected("requested work coordinate is not finite");
        const auto machinePosition = axisComponent(m_snapshot.machinePosition, axis);
        const auto toolOffset = axisComponent(
            m_presentationTracker.snapshot().activePresentation.activeToolOffset, axis);
        const auto offset = machinePosition - toolOffset - workPosition;
        m_session.machine().setActiveWorkOffset(axis, offset);
        const ngc::WorkCoordinateSystem updated {
            std::string(ngc::name(*m_session.machine().state().modeCoordSys)),
            m_session.machine().workOffset(),
        };
        m_presentationTracker.setActiveWorkCoordinateSystem(updated);
        const auto saved = persistParametersAtBoundary();
        if (!saved) {
            return std::unexpected(saved.error());
        }

        return {};
    }

    bool stopJog(const ngc::RequestId request, const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        if(!m_activeJog || *m_activeJog != jog) return false;
        std::erase_if(m_jogControls, [&](const auto &control) {
            const auto *renewal = std::get_if<ngc::RenewJogLeaseRequest>(&control);
            if(renewal) return renewal->jog == jog;
            const auto *update = std::get_if<ngc::SetContinuousJogVelocityRequest>(&control);
            return update && update->jog == jog;
        });
        if(std::ranges::any_of(m_jogControls, [&](const auto &control) {
            const auto *stop = std::get_if<ngc::StopJogRequest>(&control);
            return stop && stop->jog == jog;
        })) return true;
        if(m_jogControls.size() >= MAX_PENDING_JOG_CONTROLS) return false;
        m_jogControls.emplace_back(ngc::StopJogRequest { request, jog });
        m_cv.notify_all();
        return true;
    }

    bool feedHold() {
        std::scoped_lock lock(m_mutex);
        if(!m_running || !m_programRunning || m_paused || m_feedHoldRequested
           || m_feedHoldInProgress || m_feedHoldHeld || m_feedResumeInProgress) return false;
        m_feedHoldRequested = true;
        m_feedHoldInProgress = true;
        m_snapshot.status = ngc::SimulationStatus::Holding;
        m_cv.notify_all();
        return true;
    }
    bool resume() {
        std::scoped_lock lock(m_mutex);
        if (!m_running || !m_programRunning || !m_paused) {
            return false;
        }
        if (m_programPaused) {
            m_programResumeRequested = true;
            m_paused = false;
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.operatorAlert.reset();
            m_cv.notify_all();
            return true;
        }
        if (!m_feedHoldHeld || m_feedResumeRequested || m_feedResumeInProgress) {
            return false;
        }
        m_feedResumeRequested = true;
        m_feedResumeInProgress = true;
        m_paused = false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_cv.notify_all();
        return true;
    }
    void stop() {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog) {
            m_stop = true;
            m_start = false;
            m_home = false;
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
            m_snapshot.operatorAlert.reset();
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
        if(m_running || m_start || m_home || m_activeJog) return false;
        m_geometryPolicy.splineFitSolver = solver;
        return true;
    }
    bool setContinuousPlanningEffort(const ngc::ContinuousPlanningEffort &effort) {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog) return false;
        auto configuredEffort = effort;
        configuredEffort.quinticServoPeriod = m_runtime.servoPeriod();
        m_driver.setContinuousPlanningEffort(configuredEffort);
        return true;
    }
    bool setContinuousDiagnosticCallback(std::function<void(
            const ngc::ContinuousTrajectoryPlan &,
            std::span<const ngc::TrajectoryPlannerInput>)> callback) {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog) return false;
        m_driver.setContinuousDiagnosticCallback(std::move(callback));
        return true;
    }
    void setRapidSpeed(const double speed) {
        std::scoped_lock lock(m_mutex);
        m_limits.rapidSpeed = std::max(speed, 1e-6);
        if(!m_running&&!m_start&&!m_home&&!m_activeJog) m_driver.setLimits(m_limits);
    }
    ngc::SimulationSnapshot snapshot() const {
        std::scoped_lock lock(m_mutex);
        auto result = m_snapshot;
        const auto &presentation = m_presentationTracker.snapshot();
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
        if (m_running || m_start || m_home || m_activeJog) {
            return false;
        }
        m_toolTable = tools;
        m_session.machine().toolTable() = tools;
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
        if (m_running || m_start || m_home || m_activeJog) {
            return std::unexpected(
                "cannot configure the tool-table store while motion owns the machine");
        }
        m_toolTableStorePath = path;

        return {};
    }

    std::expected<void, std::string> saveToolTable(
        const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (m_running || m_start || m_home || m_activeJog) {
            return std::unexpected(
                "cannot save the tool table while motion owns the machine");
        }

        return m_toolTable.save(path);
    }

    std::expected<void, std::string> setPersistentParameterStorePath(
        const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (m_running || m_start || m_home || m_activeJog) {
            return std::unexpected(
                "cannot configure persistent parameters while motion owns the machine");
        }
        m_parameterStorePath = path;

        return {};
    }
    std::expected<void, std::string> loadPersistentParameters(const std::filesystem::path &path) {
        std::scoped_lock lock(m_mutex);
        if (m_running || m_start || m_home || m_activeJog) {
            return std::unexpected("cannot load persistent parameters while motion owns the machine");
        }

        auto loaded = ngc::loadPersistentParameters(path, m_unit, m_session.machine().memory());
        if (loaded) {
            m_parameterStorePath = path;
            m_session.machine().beginProgramRun();
            m_presentationTracker.setActivePresentation(sessionPresentation());
        }

        return loaded;
    }

    std::expected<void, std::string> savePersistentParameters(const std::filesystem::path &path) const {
        std::scoped_lock lock(m_mutex);
        if (m_running || m_start || m_home || m_activeJog) {
            return std::unexpected("cannot save persistent parameters while motion owns the machine");
        }

        return ngc::savePersistentParameters(path, m_unit, m_session.machine().memory());
    }
    std::vector<ngc::ExecutedJerkSample> takeExecutedJerkSamples() {
        auto samples = m_runtime.takeExecutedJerkSamples();
        std::scoped_lock lock(m_mutex);
        for(auto &sample:samples) {
            if (const auto toolOffset = m_presentationTracker.toolOffsetForChunk(
                    sample.epoch, sample.chunk)) {
                sample.position = sample.position - *toolOffset;
            }
        }
        return samples;
    }

    void join() {
        { std::scoped_lock lock(m_mutex); if(!m_thread.joinable()) return; m_join = true; m_cv.notify_all(); }
        m_thread.join();
    }

private:
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
            *m_parameterStorePath, m_unit, m_session.machine().memory());
    }

    std::expected<void, std::string> persistToolTableAtBoundary() {
        const auto &updated = m_session.machine().toolTable();
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
        m_presentationTracker.clearTracking();
    }

    void observeLifecycle(const ngc::InterpreterBlockLifecycle &lifecycle) {
        m_presentationTracker.observeLifecycle(lifecycle);
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
        m_presentationTracker.observeCommand(
            command, item, captured, activationMarker);
    }

    void observeBackendEvent(const ngc::ExecutionEvent &event) {
        if(const auto *accepted = std::get_if<ngc::ChunkAccepted>(&event)) {
            m_presentationTracker.observeChunkAccepted(*accepted);
        } else if (const auto *marker =
                       std::get_if<ngc::ExecutionMarkerReached>(&event)) {
            m_presentationTracker.observeMarkerReached(*marker);
        } else if(std::holds_alternative<ngc::TriggeredMoveCompleted>(event)) {
            if(m_feedHoldInProgress) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = false;
                m_paused = false;
                m_snapshot.status = ngc::SimulationStatus::Running;
            }
        } else if(const auto *retired = std::get_if<ngc::ChunkRetired>(&event)) {
            m_presentationTracker.observeChunkRetired(*retired);
        } else if(const auto *held = std::get_if<ngc::BackendHeld>(&event)) {
            if(held->reason == ngc::BackendHoldReason::FeedHold) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = true;
                m_paused = true;
                m_snapshot.status = ngc::SimulationStatus::Paused;
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
            }
        }
    }

    void applyActivePresentation(
            const ngc::TrajectoryCommandPresentation &presentation) {
        m_presentationTracker.setActivePresentation(presentation);
    }

    void applyBackendSnapshot(const ngc::ExecutionSnapshot &backend) {
        m_snapshot.trajectoryBackendState = backend.state;
        m_snapshot.trajectoryBackendEpoch = backend.epoch;
        m_snapshot.trajectoryBackendChunk = backend.activeChunk;
        m_snapshot.trajectoryBackendSpan = backend.activeSpan;
        m_snapshot.trajectoryBackendSpanProgress = backend.spanProgress;
        m_snapshot.trajectoryBackendActiveNormalRemainingSeconds =
            backend.activeNormalMotionRemainingSeconds;
        m_snapshot.trajectoryBackendQueuedNormalSeconds = backend.queuedNormalMotionSeconds;
        m_snapshot.trajectoryBackendCommittedNormalSeconds = backend.committedNormalMotionSeconds;
        m_snapshot.trajectoryBackendStopBranchSeconds = backend.stopBranchRemainingSeconds;
        m_snapshot.trajectoryBackendQueuedExecutionItems = backend.queuedExecutionItems;
        m_snapshot.trajectoryBackendLastBranch = backend.lastBranch;
        m_snapshot.trajectoryBackendFaultCode = backend.faultCode;
        m_snapshot.trajectoryBackendVelocity = backend.commanded.velocity.length();
        m_snapshot.trajectoryBackendAcceleration = backend.commanded.acceleration.length();
        m_snapshot.trajectoryBackendExecutionRate = backend.executionRate;
        m_snapshot.trajectoryBackendExecutionRateAcceleration =
            backend.executionRateAcceleration;
        if(m_feedResumeInProgress && backend.state == ngc::BackendState::Running
           && backend.executionRate >= 1.0 - 1e-10
           && std::abs(backend.executionRateAcceleration) <= 1e-10)
            m_feedResumeInProgress = false;
        if(const auto detail = m_presentationTracker.executionSpanDiagnostic(
               backend.epoch, backend.activeSpan)) {
            m_snapshot.trajectoryBackendSpanDetail=std::format(
                "{} ordinal={} duration={:.9g}s distance={:.9g} "
                "velocity={:.9g}->{:.9g} acceleration={:.9g}->{:.9g}",
                detail->stopTail ? "stop-tail" : "normal", detail->ordinal,
                detail->duration, detail->distance, detail->startVelocity,
                detail->endVelocity, detail->startAcceleration, detail->endAcceleration);
        } else {
            m_snapshot.trajectoryBackendSpanDetail.clear();
        }
        if(backend.state == ngc::BackendState::Faulted) {
            m_snapshot.status = ngc::SimulationStatus::Error;
            m_snapshot.error = "mock motion backend fault " + std::to_string(backend.faultCode);
        }
        m_snapshot.machinePosition = backend.commanded.position;
        m_snapshot.commandProgress = backend.spanProgress;
        m_snapshot.hasActiveMotion = (backend.state == ngc::BackendState::Running
                                      || backend.state == ngc::BackendState::Holding)
            && backend.activeSpan != 0;
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
            .tool = m_session.machine().toolGeometry(),
            .activeToolOffset = m_session.machine().toolOffset(),
            .workCoordinateSystem = ngc::WorkCoordinateSystem {
                std::string(ngc::name(
                    *m_session.machine().state().modeCoordSys)),
                m_session.machine().workOffset(),
            },
            .modalGCodes = m_session.machine().activeModalGCodes(),
            .activeBlocks = {},
        };
    }

    void clearActiveTool() {
        m_presentationTracker.clearActiveTool();
    }

    void applyHomingBackendSnapshot(const ngc::ExecutionSnapshot &backend) {
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

    bool homingMayContinue() {
        std::unique_lock lock(m_mutex);
        while(m_paused && !m_stop && !m_join) {
            m_snapshot.status = ngc::SimulationStatus::Paused;
            m_cv.wait(lock, [&] { return !m_paused || m_stop || m_join; });
        }
        if(m_stop || m_join) {
            m_stop = false;
            m_running = false;
            m_snapshot.status = ngc::SimulationStatus::Stopped;
            m_snapshot.activity = ngc::SimulationActivity::Idle;
            m_snapshot.hasActiveMotion = false;
            return false;
        }
        m_snapshot.status = ngc::SimulationStatus::Running;
        return true;
    }

    void failHoming(const std::string &message) {
        std::scoped_lock lock(m_mutex);
        m_snapshot.status = ngc::SimulationStatus::Error;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.error = message;
        m_snapshot.hasActiveMotion = false;
        m_running = false;
    }

    bool homingAlreadyEnded() const {
        std::scoped_lock lock(m_mutex);
        return !m_running;
    }

    bool submitHomingControl(const ngc::ControlRequest &request) {
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
            applyHomingBackendSnapshot(backendSnapshot);
        }
        std::scoped_lock lock(m_mutex);
        m_snapshot.servoTicks += ticks;
    }

    std::optional<ngc::TriggeredJointMoveCompleted> executeHomingMove(
        const ngc::TriggeredJointMove &move,
        const std::vector<std::pair<ngc::JointId, double>> &transitions,
        ngc::RequestId &requestId) {
        ngc::ExecutionEvent discarded;
        while (m_runtime.endpoint().tryTakeEvent(discarded)) { }
        for (const auto &[joint, position] : transitions) {
            if (!m_runtime.configureSyntheticJointInput(move.moveId, joint, position)) {
                return std::nullopt;
            }
        }
        if (m_runtime.endpoint().tryPublish(ngc::ExecutionItem { move })
            != ngc::PublishResult::Published) {
            return std::nullopt;
        }
        if(!submitHomingControl(ngc::ResumeRequest { requestId++, move.epoch })) return std::nullopt;

        for(std::size_t guard = 0; guard < 10000000; ++guard) {
            if(!homingMayContinue()) return std::nullopt;
            advanceServiceMotionPeriod();
            ngc::ExecutionEvent event;
            while (m_runtime.endpoint().tryTakeEvent(event)) {
                if(const auto *completed = std::get_if<ngc::TriggeredJointMoveCompleted>(&event))
                    if(completed->move == move.moveId) return *completed;
                if(const auto *fault = std::get_if<ngc::BackendFault>(&event)) {
                    failHoming("mock homing backend fault " + std::to_string(fault->code));
                    return std::nullopt;
                }
            }
            std::this_thread::sleep_for(
                std::chrono::duration<double>(m_runtime.schedulerPeriod()));
        }
        return std::nullopt;
    }

    bool setHomingJointPositions(const ngc::EpochId epoch, const ngc::JointMask joints,
                                 const ngc::JointVector &positions, ngc::RequestId &requestId) {
        const auto id = requestId++;
        if(!submitHomingControl(ngc::SetJointPositionRequest { id, joints, positions })) return false;
        bool succeeded = false;
        ngc::ExecutionEvent event;
        while (m_runtime.endpoint().tryTakeEvent(event))
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == id) succeeded = completed->succeeded;
        ngc::ExecutionSnapshot backendSnapshot;
        while (m_runtime.endpoint().tryTakeSnapshot(backendSnapshot)) {
            std::scoped_lock lock(m_mutex);
            applyHomingBackendSnapshot(backendSnapshot);
        }
        (void)epoch;
        return succeeded;
    }

    ngc::TriggeredJointMove makeHomingMove(const ngc::HomingGroupConfiguration &group,
                                           const ngc::EpochId epoch, ngc::ChunkId &chunk,
                                           ngc::BranchSequence &branch, ngc::TriggeredMoveId &moveId,
                                           const bool triggered, const bool slow, const bool backoff) const {
        ngc::TriggeredJointMove move;
        move.epoch = epoch;
        move.id = chunk++;
        move.predecessorBranch = branch;
        move.branch = ++branch;
        move.moveId = moveId++;
        move.targetMode = backoff || triggered ? ngc::JointTargetMode::Relative
                                               : ngc::JointTargetMode::Absolute;
        for(const auto id : group.joints) {
            const auto *joint = configuredJoint(id);
            if(!joint) continue;
            move.joints |= ngc::JointMask { 1 } << id;
            const auto scale = joint->coordinateScale;
            const auto searchDirection = std::copysign(1.0, joint->homing.searchVelocity * scale);
            const auto range = (joint->maximum - joint->minimum) * std::abs(scale);
            if(backoff)
                move.target[id] = -searchDirection * joint->homing.backoffDistance * std::abs(scale);
            else if(triggered)
                move.target[id] = searchDirection * (slow
                    ? std::max(2.0 * joint->homing.backoffDistance * std::abs(scale), 0.01)
                    : range + 2.0 * joint->homing.backoffDistance * std::abs(scale));
            else
                move.target[id] = joint->homing.homePosition * scale;
            const auto phaseVelocity = backoff ? std::abs(joint->homing.searchVelocity * scale)
                : slow ? std::abs(joint->homing.latchVelocity * scale)
                : triggered ? std::abs(joint->homing.searchVelocity * scale)
                : (joint->homing.finalVelocity == 0.0 ? joint->maxVelocity
                                                      : std::abs(joint->homing.finalVelocity * scale));
            move.limits.velocity[id] = std::min(joint->maxVelocity, phaseVelocity);
            move.limits.acceleration[id] = joint->maxAcceleration;
            move.limits.jerk[id] = joint->maxJerk;
            if(triggered)
                (void)move.triggers.push({ id, joint->homing.input, joint->homing.condition,
                                           joint->homing.debounce });
        }
        move.triggerRequired = triggered;
        return move;
    }

    void runHoming() {
        const auto epoch = m_nextEpoch++;
        ngc::RequestId requestId = 1;
        ngc::ChunkId chunk = 1;
        ngc::BranchSequence branch = 0;
        ngc::TriggeredMoveId moveId = 1;
        ngc::JointMask allJoints = 0;
        ngc::JointVector initial;
        {
            std::scoped_lock lock(m_mutex);
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.activity = ngc::SimulationActivity::Homing;
            m_snapshot.error.clear();
            m_snapshot.servoTicks = 0;
            clearPresentation();
            for(const auto &joint : m_joints) {
                allJoints |= ngc::JointMask { 1 } << joint.id;
                initial[joint.id] = axisComponent(m_snapshot.machinePosition, joint.axis)
                    * joint.coordinateScale;
            }
            clearActiveTool();
        }

        if(!submitHomingControl(ngc::ResetRequest { requestId++, epoch })
           || !submitHomingControl(ngc::EnableRequest { requestId++ })
           || !setHomingJointPositions(epoch, allJoints, initial, requestId)) {
            failHoming("failed to initialize the mock backend for homing");
            return;
        }

        for(const auto &group : m_homing.groups) {
            std::vector<std::pair<ngc::JointId, double>> transitions;
            for(const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if(joint) transitions.emplace_back(id, joint->homing.switchPosition * joint->coordinateScale);
            }

            const auto fast = makeHomingMove(group, epoch, chunk, branch, moveId, true, false, false);
            const auto fastResult = executeHomingMove(fast, transitions, requestId);
            if(!fastResult || fastResult->status != ngc::TriggeredMoveStatus::Triggered) {
                if(!homingAlreadyEnded())
                    failHoming("fast homing search reached its travel limit before the switch");
                return;
            }

            const auto backoff = makeHomingMove(group, epoch, chunk, branch, moveId, false, false, true);
            auto backoffToClearance = backoff;
            backoffToClearance.targetMode = ngc::JointTargetMode::Absolute;
            for(const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if(!joint) continue;
                const auto searchDirection = std::copysign(
                    1.0, joint->homing.searchVelocity * joint->coordinateScale);
                backoffToClearance.target[id] = fastResult->triggerState.position[id]
                    - searchDirection * joint->homing.backoffDistance * std::abs(joint->coordinateScale);
            }
            const auto backoffResult = executeHomingMove(backoffToClearance, {}, requestId);
            if(!backoffResult || backoffResult->status != ngc::TriggeredMoveStatus::ReachedTarget) {
                if(!homingAlreadyEnded()) failHoming("fixed homing backoff did not complete");
                return;
            }
            for(const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if(!joint) continue;
                const auto switchPosition = joint->homing.switchPosition * joint->coordinateScale;
                const auto searchDirection = std::copysign(
                    1.0, joint->homing.searchVelocity * joint->coordinateScale);
                if(searchDirection * (backoffResult->stoppedState.position[id] - switchPosition) >= 0.0) {
                    failHoming("configured homing backoff did not clear the switch");
                    return;
                }
            }

            const auto slow = makeHomingMove(group, epoch, chunk, branch, moveId, true, true, false);
            const auto slowResult = executeHomingMove(slow, transitions, requestId);
            if(!slowResult || slowResult->status != ngc::TriggeredMoveStatus::Triggered) {
                if(!homingAlreadyEnded())
                    failHoming("slow homing search reached its travel limit before the switch");
                return;
            }

            auto calibrated = slowResult->stoppedState.position;
            for(const auto id : group.joints) {
                const auto *joint = configuredJoint(id);
                if(!joint) continue;
                const auto desiredSwitch = joint->homing.switchPosition * joint->coordinateScale;
                calibrated[id] += desiredSwitch - slowResult->triggerState.position[id];
            }
            if(!setHomingJointPositions(epoch, slow.triggerRequired ? slow.joints : 0, calibrated, requestId)) {
                failHoming("failed to establish joint coordinates after slow homing search");
                return;
            }

            const auto finalMove = makeHomingMove(group, epoch, chunk, branch, moveId, false, false, false);
            const auto finalResult = executeHomingMove(finalMove, {}, requestId);
            if(!finalResult || finalResult->status != ngc::TriggeredMoveStatus::ReachedTarget) {
                if(!homingAlreadyEnded())
                    failHoming("final move to the configured home position did not complete");
                return;
            }
        }

        std::scoped_lock lock(m_mutex);
        m_running = false;
        m_snapshot.status = ngc::SimulationStatus::Completed;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.hasActiveMotion = false;
        m_homedJoints = allJoints;
        m_snapshot.homedJoints = m_homedJoints;
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
        m_jogControls.clear();
        m_running = false;
    }

    void runJogging(const ngc::ControlRequest &firstRequest) {
        const auto epoch = m_nextEpoch++;
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
            m_snapshot.homedJoints = m_homedJoints;
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

        if(!submitHomingControl(ngc::ResetRequest { internalRequest--, epoch })
           || !submitHomingControl(ngc::EnableRequest { internalRequest-- })
           || !setHomingJointPositions(epoch, allJoints, initial, internalRequest)
           || m_runtime.endpoint().trySubmit(firstRequest) != ngc::SubmitResult::Submitted) {
            failJog("failed to initialize the mock backend for jogging");
            return;
        }
        m_runtime.advanceImmediate(0.0);

        bool finished = false;
        bool abortSubmitted = false;
        while(!finished) {
            std::deque<ngc::ControlRequest> controls;
            bool joining = false;
            {
                std::scoped_lock lock(m_mutex);
                joining = m_join;
                if((m_stop || joining) && !abortSubmitted) {
                    controls.emplace_back(ngc::AbortRequest { internalRequest-- });
                    abortSubmitted = true;
                    m_stop = false;
                }
                controls.insert(controls.end(), m_jogControls.begin(), m_jogControls.end());
                m_jogControls.clear();
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
        m_snapshot.status = ngc::SimulationStatus::Completed;
        m_snapshot.activity = ngc::SimulationActivity::Idle;
        m_snapshot.jogging = false;
        m_snapshot.hasActiveMotion = false;
        m_snapshot.activeJogTarget.reset();
        clearActiveTool();
    }

    void joinGeometry(const bool cancel) {
        if(cancel) m_geometryCancelled.store(true, std::memory_order_release);
        m_session.requestStop();
        m_geometryForward.notifyAll();
        m_geometryFeedback.notifyAll();
        if(m_geometryThread.joinable()) m_geometryThread.join();
        if(m_geometryProducer) {
            std::scoped_lock lock(m_mutex);
            m_snapshot.geometryStream = m_geometryProducer->diagnostics();
        }
        m_geometryProducer.reset();
        ngc::PreparedForwardMessage forward;
        while(m_geometryForward.tryPop(forward)) { }
        ngc::PreparedFeedbackMessage feedback;
        while(m_geometryFeedback.tryPop(feedback)) { }
    }

    void work() {
        using clock = std::chrono::steady_clock;
        for(;;) {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [&] { return m_join || m_start || m_home || !m_jogControls.empty(); });
            if(m_join) return;
            if(!m_jogControls.empty()) {
                auto request = std::move(m_jogControls.front());
                m_jogControls.pop_front();
                m_running = true;
                m_stop = false;
                lock.unlock();
                runJogging(request);
                continue;
            }
            if(m_home) {
                m_home = false;
                m_running = true;
                m_stop = false;
                lock.unlock();
                runHoming();
                continue;
            }
            auto programs = m_programs;
            auto tools = m_toolTable;
            const auto preserve = m_preserveState;
            const auto startingPosition = preserve ? m_snapshot.machinePosition : ngc::position_t{};
            m_start = false; m_running = true; m_programRunning = true; m_stop = false;
            if(!preserve) {
                m_snapshot = {};
                m_presentationTracker.reset();
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
            m_driver.setLimits(m_limits);
            lock.unlock();

            m_session.setPrograms(programs);
            m_session.machine().toolTable() = std::move(tools);
            m_session.compile([](const auto &callback) { callback(); });
            if(preserve) m_session.beginContinuation(); else m_session.begin();
            if(!preserve) {
                std::scoped_lock snapshotLock(m_mutex);
                applyActivePresentation(sessionPresentation());
            }
            m_geometryCancelled.store(false, std::memory_order_release);
            const auto epoch = m_nextEpoch++;
            if(!m_driver.begin(epoch, startingPosition)) {
                m_session.reportError("simulation trajectory driver failed to initialize its backend control channels");
                m_session.stop();
                lock.lock(); m_snapshot.status = ngc::SimulationStatus::Error; m_snapshot.activity = ngc::SimulationActivity::Idle; m_snapshot.error = "motion backend control channel is full"; m_running = false; m_programRunning = false; lock.unlock();
                continue;
            }
            m_geometryProducer = std::make_unique<ngc::GeometryStreamProducer>(
                m_session, m_geometryForward, m_geometryFeedback, m_geometryCancelled,
                m_geometryPolicy);
            m_geometryThread = std::thread([this, epoch] {
                (void)m_geometryProducer->run(epoch);
            });
            if (!m_runtime.beginTimedExecution()) {
                joinGeometry(true);
                m_session.reportError(
                    "in-process Simulation runtime failed to start timed execution");
                m_session.stop();
                lock.lock();
                m_snapshot.status = ngc::SimulationStatus::Error;
                m_snapshot.activity = ngc::SimulationActivity::Idle;
                m_snapshot.error = "Simulation servo scheduler failed to start";
                m_running = false;
                m_programRunning = false;
                lock.unlock();
                continue;
            }

            const auto copyTimingSnapshot = [&] {
                copyRuntimeTimingSnapshot();
                m_snapshot.trajectoryPlanningActivity=m_driver.planningActivity();
                m_snapshot.trajectoryPlanningActivitySeconds=
                    m_driver.planningActivitySeconds();
                m_snapshot.trajectoryDriverActivity=m_driver.activity();
                m_snapshot.trajectoryContinuousPlanSummary=
                    m_driver.lastContinuousPlanSummary();
                m_snapshot.trajectoryContinuousCorrectionHistory=
                    m_driver.lastContinuousCorrectionHistory();
            };
            auto nextPlanningRefresh=clock::now();
            m_driver.setPlanningProgressCallback([&] {
                const auto now=clock::now();
                if(now<nextPlanningRefresh) return;
                nextPlanningRefresh=now+std::chrono::milliseconds(16);
                std::unique_lock snapshotLock(m_mutex,std::try_to_lock);
                if(!snapshotLock.owns_lock()) return;
                ngc::ExecutionSnapshot backendSnapshot;
                while (m_runtime.endpoint().tryTakeSnapshot(backendSnapshot))
                    applyBackendSnapshot(backendSnapshot);
                copyTimingSnapshot();
            });
            struct PlanningProgressReset {
                ngc::PreparedTrajectoryExecutionDriver &driver;
                ~PlanningProgressReset() { driver.setPlanningProgressCallback({}); }
            } planningProgressReset{m_driver};

            for(;;) {
                lock.lock();
                if(m_join || m_stop) {
                    const auto joining = m_join; m_stop = false; m_running = false; m_programRunning = false; m_snapshot.status = ngc::SimulationStatus::Stopped; m_snapshot.activity = ngc::SimulationActivity::Idle;
                    copyTimingSnapshot();
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    joinGeometry(true);
                    m_session.stop();
                    {
                        std::scoped_lock statusLock(m_mutex);
                        m_snapshot.statusMessages = m_session.statusMessages();
                        if (auto saved = persistToolTableAtBoundary(); !saved) {
                            m_snapshot.status = ngc::SimulationStatus::Error;
                            m_snapshot.error = saved.error();
                        }
                    }
                    if(joining) return;
                    break;
                }
                if (m_programResumeRequested) {
                    m_programResumeRequested = false;
                    if (!m_driver.resumeProgram()) {
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
                    m_cv.wait(lock, [&] { return m_join || m_stop || !m_paused; });
                    lock.unlock();
                    continue;
                }
                if(m_snapshot.status != ngc::SimulationStatus::Error)
                    m_snapshot.status = m_feedHoldInProgress
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
                m_driver.serviceBackend([&](const auto &event) { observeBackendEvent(event); });
                if (auto presentation = m_driver.takePresentationUpdate()) {
                    applyActivePresentation(*presentation);
                }
                if (m_runtime.executorBatchActive()) {
                    copyTimingSnapshot();
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                ngc::ExecutionSnapshot backendSnapshot;
                while (m_runtime.endpoint().tryTakeSnapshot(backendSnapshot)) {
                    applyBackendSnapshot(backendSnapshot);
                }
                copyTimingSnapshot();
                auto state = m_driver.state();
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
                    m_snapshot.error = *m_driver.error(); m_running = false; m_programRunning = false;
                } else if(state == ngc::PreparedDriverState::Completed) {
                    m_presentationTracker.completeDeferredBlocks();
                }
                if ((state == ngc::PreparedDriverState::Completed
                     || state == ngc::PreparedDriverState::Error)
                    || !m_running) {
                    lock.unlock();
                    m_runtime.endTimedExecution();
                    joinGeometry(state == ngc::PreparedDriverState::Error);
                    if(state == ngc::PreparedDriverState::Error && m_driver.error())
                        m_session.reportError(*m_driver.error());
                    m_session.stop();
                    if(state == ngc::PreparedDriverState::Completed
                       ||state == ngc::PreparedDriverState::Error) {
                        std::scoped_lock sessionLock(m_mutex);
                        m_snapshot.statusMessages = m_session.statusMessages();
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
                    const auto pumped=m_driver.pumpOne(
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
                    if (auto presentation = m_driver.takePresentationUpdate()) {
                        applyActivePresentation(*presentation);
                    }
                    if(!pumped) break;
                    filled = true;
                    if(m_join||m_stop||m_paused) break;
                }
                m_snapshot.trajectoryPlanning = m_driver.planningDiagnostics();
                copyTimingSnapshot();
                m_runtime.releaseRefillOpportunity();
                m_runtime.setRollingSupplyActive(
                    m_runtime.tickMultiplier() > 1
                    && m_driver.hasUnpublishedRollingContinuation()
                    && !m_driver.hasPendingPublication());
                state = m_driver.state();
                if(state == ngc::PreparedDriverState::Error) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.activity = ngc::SimulationActivity::Idle;
                    m_snapshot.error = *m_driver.error();
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
                    joinGeometry(true);
                    if(state == ngc::PreparedDriverState::Error && m_driver.error())
                        m_session.reportError(*m_driver.error());
                    m_session.stop();
                    {
                        std::scoped_lock statusLock(m_mutex);
                        m_snapshot.statusMessages = m_session.statusMessages();
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
                              [&] { return m_join || m_stop || m_paused; });
                lock.unlock();
            }
        }
    }
};
