#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <map>
#include <ranges>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <set>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/MachineConfiguration.h"
#include "machine/MockMotionBackend.h"
#include "machine/SimulationPresentation.h"
#include "machine/ToolTable.h"
#include "machine/TrajectoryExecutionDriver.h"
#include "WindowsServoPacer.h"

class SimulationWorker {
    static constexpr std::size_t MAX_PENDING_JOG_CONTROLS = 16;
    struct ChunkPresentation {
        ngc::ToolGeometry tool{};
        ngc::position_t activeToolOffset{};
        std::optional<ngc::WorkCoordinateSystem> workCoordinateSystem;
        std::vector<std::string> modalGCodes;
        std::vector<ngc::BlockExecution> activeBlocks;
        std::vector<ngc::BlockExecution> completedBlocks;
        std::optional<ngc::MachineCommand> command;
    };

    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;
    ngc::InterpreterSession m_session;
    ngc::MockMotionBackend m_backend;
    ngc::TrajectoryExecutionDriver m_driver;
    ngc::TrajectoryLimits m_limits;
    ngc::SimulationSnapshot m_snapshot;
    using ChunkKey = std::pair<ngc::EpochId, ngc::ChunkId>;
    using SpanKey = std::pair<ngc::EpochId, ngc::SpanId>;
    std::map<ChunkKey, ChunkPresentation> m_chunks;
    std::map<SpanKey, ChunkPresentation> m_spanPresentations;
    std::map<ChunkKey, ngc::SpanId> m_chunkFirstSpan;
    std::map<std::uint64_t, ChunkKey> m_blockChunks;
    std::map<std::uint64_t, ngc::BlockExecution> m_deferredCompletedBlocks;
    std::set<ChunkKey> m_retiredChunks;
    std::vector<ngc::BlockExecution> m_interpretedBlocks;
    std::optional<ChunkKey> m_lastChunk;
    std::vector<std::tuple<std::string, std::string>> m_programs;
    ngc::ToolTable m_toolTable;
    bool m_join = false;
    bool m_start = false;
    bool m_stop = false;
    bool m_paused = false;
    bool m_running = false;
    bool m_preserveState = false;
    bool m_home = false;
    std::deque<ngc::ControlRequest> m_jogControls;
    std::optional<ngc::JogId> m_activeJog;
    ngc::JointMask m_homedJoints = 0;
    std::vector<ngc::AxisConfiguration> m_axes;
    std::vector<ngc::JointConfiguration> m_joints;
    ngc::HomingConfiguration m_homing;
    double m_servoPeriod;
    double m_schedulerPeriod;
    std::uint32_t m_servoTicksPerSchedulerPeriod;
    std::uint32_t m_tickMultiplier = 1;
    std::atomic<std::uint32_t> m_executorTickMultiplier{1};
    std::atomic<bool> m_executorPaused{false};
    ngc::EpochId m_nextEpoch = 1;

public:
    explicit SimulationWorker(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                              const ngc::TrajectoryLimits limits = {},
                              const ngc::SimulationTiming timing = {})
        : m_session(unit, ngc::InterpretationMode::Simulation), m_driver(m_session, m_backend, limits),
          m_limits(limits), m_servoPeriod(timing.servoPeriod), m_schedulerPeriod(timing.schedulerPeriod),
          m_servoTicksPerSchedulerPeriod(static_cast<std::uint32_t>(
              std::max(1.0, std::round(timing.schedulerPeriod / timing.servoPeriod)))) {
        m_snapshot.servoPeriodSeconds = m_servoPeriod;
        m_snapshot.schedulerPeriodSeconds = m_schedulerPeriod;
        m_snapshot.servoTicksPerSchedulerPeriod = m_servoTicksPerSchedulerPeriod;
        m_thread = std::thread(&SimulationWorker::work, this);
    }
    explicit SimulationWorker(const ngc::MachineConfiguration &configuration)
        : m_session(configuration.unit, ngc::InterpretationMode::Simulation),
          m_driver(m_session, m_backend, configuration.trajectory), m_limits(configuration.trajectory),
          m_axes(configuration.axes), m_joints(configuration.joints), m_homing(configuration.homing),
          m_servoPeriod(configuration.simulation.servoPeriod),
          m_schedulerPeriod(configuration.simulation.schedulerPeriod),
          m_servoTicksPerSchedulerPeriod(static_cast<std::uint32_t>(std::max(
              1.0, std::round(configuration.simulation.schedulerPeriod / configuration.simulation.servoPeriod)))) {
        m_snapshot.servoPeriodSeconds = m_servoPeriod;
        m_snapshot.schedulerPeriodSeconds = m_schedulerPeriod;
        m_snapshot.servoTicksPerSchedulerPeriod = m_servoTicksPerSchedulerPeriod;
        m_snapshot.machinePosition = { 6.0, 6.0, -6.0, 0.0, 0.0, 0.0 };
        updateHomingToolPose();
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
        m_toolTable = tools;
        m_preserveState = preserveState;
        m_start = true;
        m_stop = false;
        m_paused = false;
        m_snapshot.status = ngc::SimulationStatus::Running;
        m_cv.notify_all();
        return true;
    }

    bool resetSimulation() {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog) return false;
        m_session.machine().beginProgramRun();
        m_snapshot = {};
        clearPresentation();
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
        m_snapshot.error.clear();
        m_executorPaused.store(false, std::memory_order_relaxed);
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

    bool stopJog(const ngc::RequestId request, const ngc::JogId jog) {
        std::scoped_lock lock(m_mutex);
        if(!m_activeJog || *m_activeJog != jog) return false;
        std::erase_if(m_jogControls, [&](const auto &control) {
            const auto *renewal = std::get_if<ngc::RenewJogLeaseRequest>(&control);
            return renewal && renewal->jog == jog;
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

    void pause() { std::scoped_lock lock(m_mutex); if(m_running) { m_paused = true; m_executorPaused = true; } }
    void resume() { std::scoped_lock lock(m_mutex); if(m_running) { m_paused = false; m_executorPaused = false; m_cv.notify_all(); } }
    void stop() {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || m_home || m_activeJog) {
            m_stop = true;
            m_start = false;
            m_home = false;
            m_paused = false;
            m_cv.notify_all();
        }
    }
    void setTickMultiplier(const int multiplier) {
        std::scoped_lock lock(m_mutex);
        m_tickMultiplier = static_cast<std::uint32_t>(std::clamp(multiplier, 1, 1000));
        m_executorTickMultiplier.store(m_tickMultiplier, std::memory_order_relaxed);
        m_snapshot.tickMultiplier = m_tickMultiplier;
    }
    void setRapidSpeed(const double speed) {
        std::scoped_lock lock(m_mutex);
        m_limits.rapidSpeed = std::max(speed, 1e-6);
        if(!m_running&&!m_start&&!m_home&&!m_activeJog) m_driver.setLimits(m_limits);
    }
    ngc::SimulationSnapshot snapshot() const { std::scoped_lock lock(m_mutex); return m_snapshot; }
    std::vector<ngc::ExecutedJerkSample> takeExecutedJerkSamples() {
        auto samples=m_backend.takeExecutedJerkSamples();
        std::scoped_lock lock(m_mutex);
        for(auto &sample:samples) {
            const ChunkPresentation *presentation=nullptr;
            auto span=m_spanPresentations.upper_bound({sample.epoch,sample.span});
            if(span!=m_spanPresentations.begin()) {
                --span;
                if(span->first.first==sample.epoch) presentation=&span->second;
            }
            if(!presentation) {
                const auto chunk=m_chunks.find({sample.epoch,sample.chunk});
                if(chunk!=m_chunks.end()) presentation=&chunk->second;
            }
            if(presentation) sample.position=sample.position-presentation->tool.offset;
        }
        return samples;
    }

    void join() {
        { std::scoped_lock lock(m_mutex); if(!m_thread.joinable()) return; m_join = true; m_cv.notify_all(); }
        m_thread.join();
    }

private:
    void clearPresentation() {
        m_chunks.clear();
        m_spanPresentations.clear();
        m_chunkFirstSpan.clear();
        m_blockChunks.clear();
        m_deferredCompletedBlocks.clear();
        m_retiredChunks.clear();
        m_interpretedBlocks.clear();
        m_lastChunk.reset();
    }

    void completeBlock(const ngc::BlockExecution &block) {
        m_snapshot.completedBlocks.push_back(block);
        auto &flags = m_snapshot.completedLineFlags[block.source];
        if(block.line >= static_cast<int>(flags.size())) flags.resize(static_cast<std::size_t>(block.line) + 1);
        if(block.line >= 0) flags[static_cast<std::size_t>(block.line)] = 1;
    }

    void observeLifecycle(const ngc::InterpreterBlockLifecycle &lifecycle) {
        if(lifecycle.phase == ngc::BlockLifecyclePhase::Entered) {
            m_interpretedBlocks.push_back(lifecycle.block);
            return;
        }
        const auto match = std::ranges::find_if(m_interpretedBlocks, [&](const auto &block) { return block.id == lifecycle.block.id; });
        if(match != m_interpretedBlocks.end()) m_interpretedBlocks.erase(match);
        if(const auto owner=m_blockChunks.find(lifecycle.block.id);owner!=m_blockChunks.end()) {
            if(m_retiredChunks.contains(owner->second)) completeBlock(lifecycle.block);
            else m_chunks[owner->second].completedBlocks.push_back(lifecycle.block);
        } else m_deferredCompletedBlocks.insert_or_assign(lifecycle.block.id,lifecycle.block);
    }

    void observeCommand(const ngc::MachineCommand &command, const ngc::ExecutionItem &item,
                        const ngc::TrajectoryCommandPresentation &captured,
                        const ngc::SpanId activationSpan) {
        ChunkPresentation presentation;
        presentation.tool = captured.tool;
        presentation.activeToolOffset = captured.activeToolOffset;
        presentation.workCoordinateSystem = captured.workCoordinateSystem;
        presentation.modalGCodes = captured.modalGCodes;
        presentation.activeBlocks = captured.activeBlocks;
        presentation.command = command;
        const auto key = std::visit([](const auto &value) { return ChunkKey { value.epoch, value.id }; }, item);
        for(const auto &block:captured.activeBlocks) {
            m_blockChunks.insert_or_assign(block.id,key);
            if(const auto completed=m_deferredCompletedBlocks.find(block.id);
               completed!=m_deferredCompletedBlocks.end()) {
                presentation.completedBlocks.push_back(completed->second);
                m_deferredCompletedBlocks.erase(completed);
            }
        }
        if(std::holds_alternative<ngc::ProbeMove>(command)) {
            const auto &move = std::get<ngc::TriggeredMove>(item);
            const auto contact = move.target + presentation.tool.offset - presentation.activeToolOffset;
            (void)m_backend.configureSyntheticInput(move.moveId, contact);
        }
        if(activationSpan!=0) {
            m_spanPresentations.insert_or_assign({key.first,activationSpan},presentation);
            m_chunkFirstSpan.try_emplace(key,activationSpan);
        }
        if(const auto existing=m_chunks.find(key);existing!=m_chunks.end())
            presentation.completedBlocks.insert(presentation.completedBlocks.end(),
                existing->second.completedBlocks.begin(),existing->second.completedBlocks.end());
        m_chunks.insert_or_assign(key, std::move(presentation));
        m_lastChunk = key;
    }

    void observeBackendEvent(const ngc::ExecutionEvent &event) {
        if(const auto *accepted = std::get_if<ngc::ChunkAccepted>(&event)) {
            const ChunkKey key{accepted->epoch,accepted->chunk};
            if(const auto first=m_chunkFirstSpan.find(key);first!=m_chunkFirstSpan.end()) {
                const auto found=m_spanPresentations.find({accepted->epoch,first->second});
                if(found!=m_spanPresentations.end()) applyPresentation(found->second);
            }
        } else if(const auto *retired = std::get_if<ngc::ChunkRetired>(&event)) {
            const ChunkKey key { retired->epoch, retired->chunk };
            const auto found = m_chunks.find(key);
            if(found != m_chunks.end()) for(const auto &block : found->second.completedBlocks) completeBlock(block);
            m_retiredChunks.insert(key);
        }
    }

    void applyPresentation(const ChunkPresentation &presentation) {
        m_snapshot.activeBlocks = presentation.activeBlocks;
        m_snapshot.activeModalGCodes = presentation.modalGCodes;
        m_snapshot.activeWorkCoordinateSystem = presentation.workCoordinateSystem;
        if(presentation.workCoordinateSystem) {
            const auto system = std::ranges::find_if(m_snapshot.usedWorkCoordinateSystems,
                [&](const auto &value) { return value.name == presentation.workCoordinateSystem->name; });
            if(system == m_snapshot.usedWorkCoordinateSystems.end()) m_snapshot.usedWorkCoordinateSystems.push_back(*presentation.workCoordinateSystem);
        }
        m_snapshot.toolPosition = m_snapshot.machinePosition - presentation.tool.offset;
        m_snapshot.toolPose = { presentation.tool, m_snapshot.machinePosition, m_snapshot.toolPosition };
        if(presentation.command) std::visit([&](const auto &command) {
            using T = std::decay_t<decltype(command)>;
            if constexpr(std::same_as<T, ngc::SpindleStart>) {
                m_snapshot.spindleRunning = true; m_snapshot.spindleSpeed = command.speed(); m_snapshot.spindleDirection = command.direction();
            } else if constexpr(std::same_as<T, ngc::SpindleStop>) {
                m_snapshot.spindleRunning = false; m_snapshot.spindleSpeed = 0.0;
            }
        }, *presentation.command);
    }

    void applyBackendSnapshot(const ngc::ExecutionSnapshot &backend) {
        if(backend.state == ngc::BackendState::Faulted) {
            m_snapshot.status = ngc::SimulationStatus::Error;
            m_snapshot.error = "mock motion backend fault " + std::to_string(backend.faultCode);
        }
        m_snapshot.machinePosition = backend.commanded.position;
        m_snapshot.commandProgress = backend.spanProgress;
        m_snapshot.hasActiveMotion = backend.state == ngc::BackendState::Running && backend.activeSpan != 0;
        if(backend.activeSpan!=0) {
            auto found=m_spanPresentations.upper_bound({backend.epoch,backend.activeSpan});
            if(found!=m_spanPresentations.begin()) {
                --found;
                if(found->first.first==backend.epoch) {
                    applyPresentation(found->second);
                    return;
                }
            }
        }
        const auto found=m_chunks.find({backend.epoch,backend.activeChunk});
        if(found!=m_chunks.end()) applyPresentation(found->second);
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

    const ngc::JointConfiguration *configuredJoint(const ngc::JointId id) const {
        const auto found = std::ranges::find(m_joints, id, &ngc::JointConfiguration::id);
        return found == m_joints.end() ? nullptr : &*found;
    }

    void updateHomingToolPose() {
        m_snapshot.toolPosition = m_snapshot.machinePosition;
        m_snapshot.toolPose = { {}, m_snapshot.machinePosition, m_snapshot.machinePosition };
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
        updateHomingToolPose();
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
            m_snapshot.hasActiveMotion = false;
            return false;
        }
        m_snapshot.status = ngc::SimulationStatus::Running;
        return true;
    }

    void failHoming(const std::string &message) {
        std::scoped_lock lock(m_mutex);
        m_snapshot.status = ngc::SimulationStatus::Error;
        m_snapshot.error = message;
        m_snapshot.hasActiveMotion = false;
        m_running = false;
    }

    bool homingAlreadyEnded() const {
        std::scoped_lock lock(m_mutex);
        return !m_running;
    }

    bool submitHomingControl(const ngc::ControlRequest &request) {
        if(m_backend.trySubmit(request) != ngc::SubmitResult::Submitted) return false;
        m_backend.advance(0.0);
        return true;
    }

    std::optional<ngc::TriggeredJointMoveCompleted> executeHomingMove(
        const ngc::TriggeredJointMove &move,
        const std::vector<std::pair<ngc::JointId, double>> &transitions,
        ngc::RequestId &requestId) {
        ngc::ExecutionEvent discarded;
        while(m_backend.tryTakeEvent(discarded)) { }
        for(const auto &[joint, position] : transitions)
            if(!m_backend.configureSyntheticJointInput(move.moveId, joint, position)) return std::nullopt;
        if(m_backend.tryPublish(ngc::ExecutionItem { move }) != ngc::PublishResult::Published)
            return std::nullopt;
        if(!submitHomingControl(ngc::ResumeRequest { requestId++, move.epoch })) return std::nullopt;

        for(std::size_t guard = 0; guard < 10000000; ++guard) {
            if(!homingMayContinue()) return std::nullopt;
            const auto multiplier = m_executorTickMultiplier.load(std::memory_order_relaxed);
            const auto ticks = static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod) * multiplier;
            for(std::uint64_t tick = 0; tick < ticks; ++tick)
                m_backend.advanceTick(m_servoPeriod, tick + 1 == ticks);

            ngc::ExecutionSnapshot backendSnapshot;
            while(m_backend.tryTakeSnapshot(backendSnapshot)) {
                std::scoped_lock lock(m_mutex);
                applyHomingBackendSnapshot(backendSnapshot);
            }
            {
                std::scoped_lock lock(m_mutex);
                m_snapshot.servoTicks += ticks;
            }
            ngc::ExecutionEvent event;
            while(m_backend.tryTakeEvent(event)) {
                if(const auto *completed = std::get_if<ngc::TriggeredJointMoveCompleted>(&event))
                    if(completed->move == move.moveId) return *completed;
                if(const auto *fault = std::get_if<ngc::BackendFault>(&event)) {
                    failHoming("mock homing backend fault " + std::to_string(fault->code));
                    return std::nullopt;
                }
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(m_schedulerPeriod));
        }
        return std::nullopt;
    }

    bool setHomingJointPositions(const ngc::EpochId epoch, const ngc::JointMask joints,
                                 const ngc::JointVector &positions, ngc::RequestId &requestId) {
        const auto id = requestId++;
        if(!submitHomingControl(ngc::SetJointPositionRequest { id, joints, positions })) return false;
        bool succeeded = false;
        ngc::ExecutionEvent event;
        while(m_backend.tryTakeEvent(event))
            if(const auto *completed = std::get_if<ngc::RequestCompleted>(&event))
                if(completed->request == id) succeeded = completed->succeeded;
        ngc::ExecutionSnapshot backendSnapshot;
        while(m_backend.tryTakeSnapshot(backendSnapshot)) {
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
            m_snapshot.error.clear();
            m_snapshot.servoTicks = 0;
            clearPresentation();
            for(const auto &joint : m_joints) {
                allJoints |= ngc::JointMask { 1 } << joint.id;
                initial[joint.id] = axisComponent(m_snapshot.machinePosition, joint.axis)
                    * joint.coordinateScale;
            }
            updateHomingToolPose();
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
        m_snapshot.hasActiveMotion = false;
        m_homedJoints = allJoints;
        m_snapshot.homedJoints = m_homedJoints;
        updateHomingToolPose();
    }

    void failJog(const std::string &message) {
        std::scoped_lock lock(m_mutex);
        m_snapshot.status = ngc::SimulationStatus::Error;
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
            updateHomingToolPose();
        }

        if(!submitHomingControl(ngc::ResetRequest { internalRequest--, epoch })
           || !submitHomingControl(ngc::EnableRequest { internalRequest-- })
           || !setHomingJointPositions(epoch, allJoints, initial, internalRequest)
           || m_backend.trySubmit(firstRequest) != ngc::SubmitResult::Submitted) {
            failJog("failed to initialize the mock backend for jogging");
            return;
        }
        m_backend.advance(0.0);

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
                if(m_backend.trySubmit(control) != ngc::SubmitResult::Submitted) {
                    failJog("mock backend jog control channel is full");
                    return;
                }
            }

            const auto multiplier = m_executorTickMultiplier.load(std::memory_order_relaxed);
            const auto ticks = static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod) * multiplier;
            for(std::uint64_t tick = 0; tick < ticks; ++tick)
                m_backend.advanceTick(m_servoPeriod, tick + 1 == ticks);

            ngc::ExecutionSnapshot backendSnapshot;
            while(m_backend.tryTakeSnapshot(backendSnapshot)) {
                std::scoped_lock lock(m_mutex);
                applyHomingBackendSnapshot(backendSnapshot);
            }
            {
                std::scoped_lock lock(m_mutex);
                m_snapshot.servoTicks += ticks;
            }

            ngc::ExecutionEvent event;
            while(m_backend.tryTakeEvent(event)) {
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
            if(!finished) std::this_thread::sleep_for(std::chrono::duration<double>(m_schedulerPeriod));
            if(joining && finished) break;
        }

        std::scoped_lock lock(m_mutex);
        m_running = false;
        m_activeJog.reset();
        m_snapshot.status = ngc::SimulationStatus::Completed;
        m_snapshot.jogging = false;
        m_snapshot.hasActiveMotion = false;
        m_snapshot.activeJogTarget.reset();
        updateHomingToolPose();
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
            m_start = false; m_running = true; m_stop = false;
            if(!preserve) { m_snapshot = {}; clearPresentation(); }
            m_snapshot.status = ngc::SimulationStatus::Running;
            m_snapshot.servoPeriodSeconds = m_servoPeriod;
            m_snapshot.schedulerPeriodSeconds = m_schedulerPeriod;
            m_snapshot.servoTicksPerSchedulerPeriod = m_servoTicksPerSchedulerPeriod;
            m_snapshot.tickMultiplier = m_tickMultiplier;
            m_snapshot.servoTicks = 0;
            m_snapshot.programElapsedSeconds = 0.0;
            m_snapshot.executedPathJerk = 0.0;
            m_snapshot.deadlineMisses = 0;
            m_snapshot.lastWakeLatenessSeconds = 0.0;
            m_snapshot.maximumWakeLatenessSeconds = 0.0;
            m_snapshot.maximumTickExecutionSeconds = 0.0;
            m_executorTickMultiplier.store(m_tickMultiplier, std::memory_order_relaxed);
            m_executorPaused.store(false, std::memory_order_relaxed);
            m_driver.setLimits(m_limits);
            lock.unlock();

            m_session.setPrograms(programs);
            m_session.machine().toolTable() = std::move(tools);
            m_session.compile([](const auto &callback) { callback(); });
            if(preserve) m_session.beginContinuation(); else m_session.begin();
            m_backend.clearTrajectoryDiagnostics();
            if(!m_driver.begin(m_nextEpoch++, startingPosition)) {
                m_session.reportError("simulation trajectory driver failed to initialize its backend control channels");
                m_session.stop();
                lock.lock(); m_snapshot.status = ngc::SimulationStatus::Error; m_snapshot.error = "motion backend control channel is full"; m_running = false; lock.unlock();
                continue;
            }

            std::atomic<bool> stopExecutor{false};
            std::atomic<std::uint64_t> servoTicks{0};
            std::atomic<double> programElapsedSeconds{0.0};
            std::atomic<std::uint64_t> deadlineMisses{0};
            std::atomic<double> lastWakeLateness{0.0};
            std::atomic<double> maximumWakeLateness{0.0};
            std::atomic<double> maximumTickExecution{0.0};
            std::atomic<std::uint32_t> executorError{0};
            std::atomic<bool> executorBatchActive{false};
            std::atomic<bool> executorRefillRequested{false};
            const auto updateMaximum = [](std::atomic<double> &target, const double value) {
                auto current = target.load(std::memory_order_relaxed);
                while(current < value && !target.compare_exchange_weak(
                    current, value, std::memory_order_relaxed, std::memory_order_relaxed)) { }
            };
            std::thread executor([&] {
                WindowsServoPacer pacer(m_schedulerPeriod);
                if(!pacer.valid()) {
                    executorError.store(pacer.errorCode(), std::memory_order_release);
                    return;
                }
                while(!stopExecutor.load(std::memory_order_acquire)) {
                    WindowsServoPacer::WaitResult timing;
                    if(!pacer.wait(timing)) {
                        executorError.store(pacer.errorCode(), std::memory_order_release);
                        return;
                    }
                    lastWakeLateness.store(timing.latenessSeconds, std::memory_order_relaxed);
                    updateMaximum(maximumWakeLateness, timing.latenessSeconds);
                    deadlineMisses.fetch_add(timing.missedPeriods, std::memory_order_relaxed);
                    if(m_executorPaused.load(std::memory_order_relaxed)) {
                        pacer.reset();
                        continue;
                    }

                    const auto multiplier = m_executorTickMultiplier.load(std::memory_order_relaxed);
                    const auto ticksThisPeriod = static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod)
                        * multiplier;
                    executorBatchActive.store(true, std::memory_order_release);
                    for(std::uint64_t tick = 0; tick < ticksThisPeriod
                        && !stopExecutor.load(std::memory_order_relaxed)
                        && !m_executorPaused.load(std::memory_order_relaxed); ++tick) {
                        const auto started = clock::now();
                        const auto crossedChunk=m_backend.advanceTick(
                            m_servoPeriod,tick+1==ticksThisPeriod);
                        programElapsedSeconds.fetch_add(
                            m_backend.lastAdvanceProgramSeconds(), std::memory_order_relaxed);
                        const auto duration = std::chrono::duration<double>(clock::now() - started).count();
                        updateMaximum(maximumTickExecution, duration);
                        servoTicks.fetch_add(1, std::memory_order_relaxed);
                        if(crossedChunk&&tick+1<ticksThisPeriod) {
                            // Accelerated mock playback can consume multiple RT
                            // packets inside one scheduler batch. Yield at each
                            // continuation so the sole NRT producer can replace
                            // the freed queue slot before the bounded horizon
                            // drains and legitimately selects a stop branch.
                            executorRefillRequested.store(true,std::memory_order_release);
                            executorBatchActive.store(false,std::memory_order_release);
                            while(executorRefillRequested.load(std::memory_order_acquire)
                                  &&!stopExecutor.load(std::memory_order_relaxed)
                                  &&!m_executorPaused.load(std::memory_order_relaxed))
                                std::this_thread::yield();
                            if(stopExecutor.load(std::memory_order_relaxed)
                               ||m_executorPaused.load(std::memory_order_relaxed)) break;
                            executorBatchActive.store(true,std::memory_order_release);
                        }
                    }
                    executorBatchActive.store(false, std::memory_order_release);
                }
            });

            const auto copyTimingSnapshot = [&] {
                m_snapshot.servoPeriodSeconds = m_servoPeriod;
                m_snapshot.schedulerPeriodSeconds = m_schedulerPeriod;
                m_snapshot.servoTicksPerSchedulerPeriod = m_servoTicksPerSchedulerPeriod;
                m_snapshot.tickMultiplier = m_executorTickMultiplier.load(std::memory_order_relaxed);
                m_snapshot.servoTicks = servoTicks.load(std::memory_order_relaxed);
                m_snapshot.programElapsedSeconds = programElapsedSeconds.load(std::memory_order_relaxed);
                m_snapshot.executedPathJerk=m_backend.currentProgramJerkMagnitude();
                m_snapshot.deadlineMisses = deadlineMisses.load(std::memory_order_relaxed);
                m_snapshot.lastWakeLatenessSeconds = lastWakeLateness.load(std::memory_order_relaxed);
                m_snapshot.maximumWakeLatenessSeconds = maximumWakeLateness.load(std::memory_order_relaxed);
                m_snapshot.maximumTickExecutionSeconds = maximumTickExecution.load(std::memory_order_relaxed);
            };
            auto nextPlanningRefresh=clock::now();
            m_driver.setPlanningProgressCallback([&] {
                const auto now=clock::now();
                if(now<nextPlanningRefresh) return;
                nextPlanningRefresh=now+std::chrono::milliseconds(16);
                std::unique_lock snapshotLock(m_mutex,std::try_to_lock);
                if(!snapshotLock.owns_lock()) return;
                ngc::ExecutionSnapshot backendSnapshot;
                while(m_backend.tryTakeSnapshot(backendSnapshot))
                    applyBackendSnapshot(backendSnapshot);
                copyTimingSnapshot();
            });
            struct PlanningProgressReset {
                ngc::TrajectoryExecutionDriver &driver;
                ~PlanningProgressReset() { driver.setPlanningProgressCallback({}); }
            } planningProgressReset{m_driver};

            for(;;) {
                lock.lock();
                if(m_join || m_stop) {
                    const auto joining = m_join; m_stop = false; m_running = false; m_snapshot.status = ngc::SimulationStatus::Stopped;
                    copyTimingSnapshot();
                    stopExecutor.store(true, std::memory_order_release);
                    executorRefillRequested.store(false,std::memory_order_release);
                    lock.unlock();
                    executor.join();
                    m_session.stop();
                    if(joining) return;
                    break;
                }
                if(m_paused) {
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                    copyTimingSnapshot();
                    executorRefillRequested.store(false,std::memory_order_release);
                    m_cv.wait(lock, [&] { return m_join || m_stop || !m_paused; });
                    lock.unlock();
                    continue;
                }
                m_snapshot.status = ngc::SimulationStatus::Running;
                if(executorBatchActive.load(std::memory_order_acquire)) {
                    copyTimingSnapshot();
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                m_driver.serviceBackend([&](const auto &event) { observeBackendEvent(event); });
                if(executorBatchActive.load(std::memory_order_acquire)) {
                    copyTimingSnapshot();
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                ngc::ExecutionSnapshot backendSnapshot;
                while(m_backend.tryTakeSnapshot(backendSnapshot)) applyBackendSnapshot(backendSnapshot);
                copyTimingSnapshot();
                m_snapshot.statusMessages=m_session.statusMessages();
                auto state = m_driver.state();
                const auto pacingError = executorError.load(std::memory_order_acquire);
                if(pacingError != 0) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.error = "Windows servo pacer failed with error " + std::to_string(pacingError);
                    m_running = false;
                } else if(m_snapshot.status == ngc::SimulationStatus::Error) {
                    m_running = false;
                } else if(state == ngc::TrajectoryDriverState::Error) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.error = *m_driver.error(); m_running = false;
                } else if(state == ngc::TrajectoryDriverState::Completed) {
                    for(const auto &[id,block]:m_deferredCompletedBlocks) {
                        (void)id;
                        completeBlock(block);
                    }
                    m_deferredCompletedBlocks.clear();
                    m_snapshot.status = ngc::SimulationStatus::Completed;
                    m_snapshot.activeModalGCodes = m_session.machine().activeModalGCodes();
                    const auto tool = m_session.machine().toolGeometry();
                    m_snapshot.toolPosition = m_snapshot.machinePosition - tool.offset;
                    m_snapshot.toolPose = { tool, m_snapshot.machinePosition, m_snapshot.toolPosition };
                    m_running = false;
                }
                if(state != ngc::TrajectoryDriverState::Running || !m_running) {
                    stopExecutor.store(true, std::memory_order_release);
                    executorRefillRequested.store(false,std::memory_order_release);
                    lock.unlock();
                    executor.join();
                    m_session.stop();
                    break;
                }

                bool filled = false;
                for(int fill = 0; fill < 64; ++fill) {
                    lock.unlock();
                    const auto pumped=m_driver.pumpOne([](const auto &callback) { callback(); },
                        [&](const auto &command, const auto &chunk, const auto &,
                            const auto &presentation, const ngc::SpanId activationSpan) {
                            std::scoped_lock presentationLock(m_mutex);
                            observeCommand(command,chunk,presentation,activationSpan);
                        },
                        [&](const auto &lifecycle) {
                            std::scoped_lock presentationLock(m_mutex);
                            observeLifecycle(lifecycle);
                        });
                    lock.lock();
                    if(!pumped) break;
                    filled = true;
                    if(m_join||m_stop||m_paused) break;
                }
                m_snapshot.statusMessages = m_session.statusMessages();
                m_snapshot.trajectoryPlanning = m_driver.planningDiagnostics();
                executorRefillRequested.store(false,std::memory_order_release);
                state = m_driver.state();
                if(state == ngc::TrajectoryDriverState::Error) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.error = *m_driver.error();
                    m_running = false;
                }
                if(!m_running) {
                    stopExecutor.store(true, std::memory_order_release);
                    executorRefillRequested.store(false,std::memory_order_release);
                    lock.unlock();
                    executor.join();
                    m_session.stop();
                    break;
                }
                if(filled) {
                    lock.unlock();
                    std::this_thread::yield();
                    continue;
                }
                m_cv.wait_for(lock, std::chrono::duration<double>(m_servoPeriod),
                              [&] { return m_join || m_stop || m_paused; });
                lock.unlock();
            }
        }
    }
};
