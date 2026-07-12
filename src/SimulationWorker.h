#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/ExecutionDriver.h"
#include "machine/SimulationExecutor.h"
#include "machine/ToolTable.h"

class SimulationWorker {
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    ngc::InterpreterSession m_session{ ngc::Machine::Unit::Inch, ngc::InterpretationMode::Simulation };
    ngc::SimulationExecutor m_executor;
    ngc::ExecutionDriver m_driver{ m_session, m_executor };
    std::vector<std::tuple<std::string, std::string>> m_programs;
    ngc::ToolTable m_toolTable;
    std::vector<ngc::InterpreterStatusMessage> m_statusMessages;

    bool m_join = false;
    bool m_start = false;
    bool m_stop = false;
    bool m_paused = false;
    bool m_running = false;
    double m_playbackRate = 1.0;
    double m_rapidSpeed = 100.0;

public:
    SimulationWorker() : m_thread(&SimulationWorker::work, this) { }
    ~SimulationWorker() { join(); }

    SimulationWorker(const SimulationWorker &) = delete;
    SimulationWorker &operator=(const SimulationWorker &) = delete;

    bool start(const std::vector<std::tuple<std::string, std::string>> &programs, const ngc::ToolTable &toolTable) {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start || programs.empty()) return false;
        m_programs = programs;
        m_toolTable = toolTable;
        m_start = true;
        m_stop = false;
        m_paused = false;
        m_cv.notify_all();
        return true;
    }

    void pause() {
        std::scoped_lock lock(m_mutex);
        if(m_running) m_paused = true;
    }

    void resume() {
        std::scoped_lock lock(m_mutex);
        if(m_running) {
            m_paused = false;
            m_cv.notify_all();
        }
    }

    void stop() {
        std::scoped_lock lock(m_mutex);
        if(m_running || m_start) {
            m_stop = true;
            m_start = false;
            m_paused = false;
            m_cv.notify_all();
        }
    }

    void setPlaybackRate(const double rate) {
        std::scoped_lock lock(m_mutex);
        m_playbackRate = std::clamp(rate, 0.01, 1000.0);
    }

    void setRapidSpeed(const double speed) {
        std::scoped_lock lock(m_mutex);
        m_rapidSpeed = std::max(speed, 1e-6);
    }

    ngc::SimulationSnapshot snapshot() const {
        std::scoped_lock lock(m_mutex);
        auto result = m_executor.lightweightSnapshot();
        result.statusMessages = m_statusMessages;
        if(m_running && m_paused) result.status = ngc::SimulationStatus::Paused;
        return result;
    }

    void join() {
        {
            std::scoped_lock lock(m_mutex);
            if(!m_thread.joinable()) return;
            m_join = true;
            m_cv.notify_all();
        }
        m_thread.join();
    }

private:
    void work() {
        using clock = std::chrono::steady_clock;
        for(;;) {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [&] { return m_join || m_start; });
            if(m_join) return;

            auto programs = m_programs;
            auto toolTable = m_toolTable;
            m_start = false;
            m_running = true;
            m_executor.reset();
            m_driver.reset();
            m_statusMessages.clear();
            m_executor.setRapidSpeed(m_rapidSpeed);
            m_executor.setStatus(ngc::SimulationStatus::Running);
            lock.unlock();

            m_session.setPrograms(programs);
            m_session.machine().toolTable() = std::move(toolTable);
            m_session.compile([](const auto &callback) { callback(); });
            m_session.begin();

            auto previous = clock::now();

            for(;;) {
                lock.lock();
                if(m_join || m_stop) {
                    const auto joining = m_join;
                    m_stop = false;
                    m_running = false;
                    m_executor.reset();
                    lock.unlock();
                    m_session.stop();
                    if(joining) return;
                    break;
                }

                if(m_paused) {
                    m_cv.wait(lock, [&] { return m_join || m_stop || !m_paused; });
                    previous = clock::now();
                    lock.unlock();
                    continue;
                }

                const auto canAccept = m_executor.canAccept();
                const auto playbackRate = m_playbackRate;
                lock.unlock();

                if(canAccept) {
                    lock.lock();
                    while(m_executor.canAccept()
                          && m_driver.pumpOne(
                              [](const auto &callback) { callback(); },
                              [](const ngc::MachineCommand &, const ngc::position_t &, const ngc::ToolGeometry &,
                                 const ngc::WorkCoordinateSystem &) { })) {
                    }
                    m_statusMessages = m_session.statusMessages();
                    lock.unlock();
                }

                const auto now = clock::now();
                const auto elapsed = std::chrono::duration<double>(now - previous).count() * playbackRate;
                previous = now;

                lock.lock();
                m_executor.advance(elapsed);
                m_driver.deliverProbeResult();
                const auto driverState = m_driver.state();
                const auto motionComplete = driverState == ngc::ExecutionDriverState::Completed;
                const auto needsCommands = m_executor.empty()
                    && driverState == ngc::ExecutionDriverState::Running;
                if(driverState == ngc::ExecutionDriverState::Error) {
                    m_executor.setError(*m_driver.error());
                    m_running = false;
                    lock.unlock();
                    m_session.stop();
                    break;
                }
                if(motionComplete) {
                    m_executor.setStatus(ngc::SimulationStatus::Completed);
                    m_running = false;
                }
                lock.unlock();

                if(motionComplete) {
                    m_session.stop();
                    break;
                }

                if(needsCommands) continue;

                lock.lock();
                m_cv.wait_for(lock, std::chrono::milliseconds(8), [&] { return m_join || m_stop || m_paused; });
                lock.unlock();
            }
        }
    }
};
