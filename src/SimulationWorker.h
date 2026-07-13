#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
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
    std::map<ChunkKey, ChunkPresentation> m_chunks;
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
    ~SimulationWorker() { join(); }
    SimulationWorker(const SimulationWorker &) = delete;
    SimulationWorker &operator=(const SimulationWorker &) = delete;

    bool start(const std::vector<std::tuple<std::string, std::string>> &programs, const ngc::ToolTable &tools,
               const bool preserveState = false) {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || programs.empty()) return false;
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
        if(m_running || m_start) return false;
        m_session.machine().beginProgramRun();
        m_snapshot = {};
        clearPresentation();
        m_programs.clear();
        return true;
    }

    void pause() { std::scoped_lock lock(m_mutex); if(m_running) { m_paused = true; m_executorPaused = true; } }
    void resume() { std::scoped_lock lock(m_mutex); if(m_running) { m_paused = false; m_executorPaused = false; m_cv.notify_all(); } }
    void stop() { std::scoped_lock lock(m_mutex); if(m_running || m_start) { m_stop = true; m_start = false; m_paused = false; m_cv.notify_all(); } }
    void setTickMultiplier(const int multiplier) {
        std::scoped_lock lock(m_mutex);
        m_tickMultiplier = static_cast<std::uint32_t>(std::clamp(multiplier, 1, 1000));
        m_executorTickMultiplier.store(m_tickMultiplier, std::memory_order_relaxed);
        m_snapshot.tickMultiplier = m_tickMultiplier;
    }
    void setRapidSpeed(const double speed) {
        std::scoped_lock lock(m_mutex);
        m_limits.rapidSpeed = std::max(speed, 1e-6);
        m_driver.setLimits(m_limits);
    }
    ngc::SimulationSnapshot snapshot() const { std::scoped_lock lock(m_mutex); return m_snapshot; }

    void join() {
        { std::scoped_lock lock(m_mutex); if(!m_thread.joinable()) return; m_join = true; m_cv.notify_all(); }
        m_thread.join();
    }

private:
    void clearPresentation() {
        m_chunks.clear();
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
        if(m_lastChunk && !m_retiredChunks.contains(*m_lastChunk))
            m_chunks[*m_lastChunk].completedBlocks.push_back(lifecycle.block);
        else
            completeBlock(lifecycle.block);
    }

    void observeCommand(const ngc::MachineCommand &command, const ngc::PlanChunk &chunk) {
        ChunkPresentation presentation;
        presentation.tool = m_session.machine().toolGeometry();
        presentation.activeToolOffset = m_session.machine().toolOffset();
        presentation.workCoordinateSystem = ngc::WorkCoordinateSystem {
            std::string(ngc::name(*m_session.machine().state().modeCoordSys)), m_session.machine().workOffset() };
        presentation.modalGCodes = m_session.machine().activeModalGCodes();
        presentation.activeBlocks = m_interpretedBlocks;
        presentation.command = command;
        if(const auto *probe = std::get_if<ngc::ProbeMove>(&command))
            (void)m_backend.configureSyntheticProbe(probe->id(), presentation.tool.offset, presentation.activeToolOffset);
        const ChunkKey key { chunk.epoch, chunk.id };
        m_chunks.insert_or_assign(key, std::move(presentation));
        m_lastChunk = key;
    }

    void observeBackendEvent(const ngc::ExecutionEvent &event) {
        if(const auto *accepted = std::get_if<ngc::ChunkAccepted>(&event)) {
            const auto found = m_chunks.find({ accepted->epoch, accepted->chunk });
            if(found != m_chunks.end()) applyPresentation(found->second);
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
        const auto found = m_chunks.find({ backend.epoch, backend.activeChunk });
        if(found == m_chunks.end()) return;
        const auto &presentation = found->second;
        applyPresentation(presentation);
    }

    void work() {
        using clock = std::chrono::steady_clock;
        for(;;) {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [&] { return m_join || m_start; });
            if(m_join) return;
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
            m_snapshot.deadlineMisses = 0;
            m_snapshot.lastWakeLatenessSeconds = 0.0;
            m_snapshot.maximumWakeLatenessSeconds = 0.0;
            m_snapshot.maximumTickExecutionSeconds = 0.0;
            m_executorTickMultiplier.store(m_tickMultiplier, std::memory_order_relaxed);
            m_executorPaused.store(false, std::memory_order_relaxed);
            lock.unlock();

            m_session.setPrograms(programs);
            m_session.machine().toolTable() = std::move(tools);
            m_session.compile([](const auto &callback) { callback(); });
            if(preserve) m_session.beginContinuation(); else m_session.begin();
            m_driver.setLimits(m_limits);
            if(!m_driver.begin(m_nextEpoch++, startingPosition)) {
                lock.lock(); m_snapshot.status = ngc::SimulationStatus::Error; m_snapshot.error = "motion backend control channel is full"; m_running = false; lock.unlock();
                continue;
            }

            std::atomic<bool> stopExecutor{false};
            std::atomic<std::uint64_t> servoTicks{0};
            std::atomic<std::uint64_t> deadlineMisses{0};
            std::atomic<double> lastWakeLateness{0.0};
            std::atomic<double> maximumWakeLateness{0.0};
            std::atomic<double> maximumTickExecution{0.0};
            std::atomic<std::uint32_t> executorError{0};
            std::atomic<bool> executorBatchActive{false};
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
                        m_backend.advanceTick(m_servoPeriod, tick + 1 == ticksThisPeriod);
                        const auto duration = std::chrono::duration<double>(clock::now() - started).count();
                        updateMaximum(maximumTickExecution, duration);
                        servoTicks.fetch_add(1, std::memory_order_relaxed);
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
                m_snapshot.deadlineMisses = deadlineMisses.load(std::memory_order_relaxed);
                m_snapshot.lastWakeLatenessSeconds = lastWakeLateness.load(std::memory_order_relaxed);
                m_snapshot.maximumWakeLatenessSeconds = maximumWakeLateness.load(std::memory_order_relaxed);
                m_snapshot.maximumTickExecutionSeconds = maximumTickExecution.load(std::memory_order_relaxed);
            };

            for(;;) {
                lock.lock();
                if(m_join || m_stop) {
                    const auto joining = m_join; m_stop = false; m_running = false; m_snapshot.status = ngc::SimulationStatus::Stopped;
                    copyTimingSnapshot();
                    stopExecutor.store(true, std::memory_order_release);
                    lock.unlock();
                    executor.join();
                    m_session.stop();
                    if(joining) return;
                    break;
                }
                if(m_paused) {
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                    copyTimingSnapshot();
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
                    m_snapshot.status = ngc::SimulationStatus::Completed;
                    m_snapshot.activeModalGCodes = m_session.machine().activeModalGCodes();
                    const auto tool = m_session.machine().toolGeometry();
                    m_snapshot.toolPosition = m_snapshot.machinePosition - tool.offset;
                    m_snapshot.toolPose = { tool, m_snapshot.machinePosition, m_snapshot.toolPosition };
                    m_running = false;
                }
                if(state != ngc::TrajectoryDriverState::Running || !m_running) {
                    stopExecutor.store(true, std::memory_order_release);
                    lock.unlock();
                    executor.join();
                    m_session.stop();
                    break;
                }

                bool filled = false;
                for(int fill = 0; fill < 64; ++fill) {
                    if(!m_driver.pumpOne([](const auto &callback) { callback(); },
                        [&](const auto &command, const auto &chunk) { observeCommand(command, chunk); },
                        [&](const auto &lifecycle) { observeLifecycle(lifecycle); })) break;
                    filled = true;
                }
                m_snapshot.statusMessages = m_session.statusMessages();
                state = m_driver.state();
                if(state == ngc::TrajectoryDriverState::Error) {
                    m_snapshot.status = ngc::SimulationStatus::Error;
                    m_snapshot.error = *m_driver.error();
                    m_running = false;
                }
                if(!m_running) {
                    stopExecutor.store(true, std::memory_order_release);
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
