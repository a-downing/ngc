#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <map>
#include <ranges>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <set>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/MockMotionBackend.h"
#include "machine/SimulationPresentation.h"
#include "machine/ToolTable.h"
#include "machine/TrajectoryExecutionDriver.h"

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
    double m_playbackRate = 1.0;
    ngc::EpochId m_nextEpoch = 1;

public:
    explicit SimulationWorker(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch,
                              const ngc::TrajectoryLimits limits = {})
        : m_session(unit, ngc::InterpretationMode::Simulation), m_driver(m_session, m_backend, limits),
          m_limits(limits) { m_thread = std::thread(&SimulationWorker::work, this); }
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

    void pause() { std::scoped_lock lock(m_mutex); if(m_running) m_paused = true; }
    void resume() { std::scoped_lock lock(m_mutex); if(m_running) { m_paused = false; m_cv.notify_all(); } }
    void stop() { std::scoped_lock lock(m_mutex); if(m_running || m_start) { m_stop = true; m_start = false; m_paused = false; m_cv.notify_all(); } }
    void setPlaybackRate(const double rate) { std::scoped_lock lock(m_mutex); m_playbackRate = std::clamp(rate, 0.01, 1000.0); }
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
            auto previous = clock::now();

            for(;;) {
                lock.lock();
                if(m_join || m_stop) {
                    const auto joining = m_join; m_stop = false; m_running = false; m_snapshot.status = ngc::SimulationStatus::Stopped;
                    lock.unlock(); m_session.stop(); if(joining) return; break;
                }
                if(m_paused) {
                    m_snapshot.status = ngc::SimulationStatus::Paused;
                    m_cv.wait(lock, [&] { return m_join || m_stop || !m_paused; });
                    previous = clock::now(); lock.unlock(); continue;
                }
                m_snapshot.status = ngc::SimulationStatus::Running;
                const auto rate = m_playbackRate;
                // Keep event headroom: every accepted chunk can produce acceptance,
                // branch, retirement, and probe/held records before the next drain.
                for(int fill = 0; fill < 4; ++fill) {
                    if(!m_driver.pumpOne([](const auto &callback) { callback(); },
                        [&](const auto &command, const auto &chunk) { observeCommand(command, chunk); },
                        [&](const auto &lifecycle) { observeLifecycle(lifecycle); })) break;
                }
                m_snapshot.statusMessages = m_session.statusMessages();
                lock.unlock();

                const auto now = clock::now();
                m_backend.advance(std::chrono::duration<double>(now - previous).count() * rate);
                previous = now;

                lock.lock();
                m_driver.serviceBackend([&](const auto &event) { observeBackendEvent(event); });
                ngc::ExecutionSnapshot backendSnapshot;
                while(m_backend.tryTakeSnapshot(backendSnapshot)) applyBackendSnapshot(backendSnapshot);
                const auto state = m_driver.state();
                if(m_snapshot.status == ngc::SimulationStatus::Error) {
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
                lock.unlock();
                if(state != ngc::TrajectoryDriverState::Running || !m_running) { m_session.stop(); break; }

                lock.lock();
                m_cv.wait_for(lock, std::chrono::milliseconds(8), [&] { return m_join || m_stop || m_paused; });
                lock.unlock();
            }
        }
    }
};
