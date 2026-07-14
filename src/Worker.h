#pragma once

#include <condition_variable>
#include <format>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

#include "evaluator/InterpreterSession.h"
#include "machine/ToolpathRecorder.h"
#include "memory/Vars.h"

class Worker {
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    ngc::InterpreterSession m_session;
    ngc::ToolpathRecorder m_toolpath;
    std::unordered_map<ngc::Var, double> m_parameterSnapshot;

    bool m_doJoin = false;
    bool m_doCompile = false;
    bool m_doExecute = false;
    bool m_busy = false;

public:
    explicit Worker(const ngc::Machine::Unit unit = ngc::Machine::Unit::Inch)
        : m_session(unit, ngc::InterpretationMode::Preview) {
        refreshParameterSnapshot();
        m_thread = std::thread(&Worker::work, this);
    }

    ~Worker() { join(); }

    Worker(const Worker &) = delete;
    Worker &operator=(const Worker &) = delete;

    double read(ngc::Var var) const {
        std::scoped_lock lock(m_mutex);
        return m_parameterSnapshot.at(var);
    }

    auto lock(const auto &callback) const {
        std::scoped_lock lock(m_mutex);
        return callback();
    }

    const ngc::Machine &machine() const { return m_session.machine(); }
    const ngc::ToolpathRecorder &toolpath() const { return m_toolpath; }

    bool compiled() const {
        std::scoped_lock lock(m_mutex);
        return m_session.compiled();
    }

    bool busy() const {
        std::scoped_lock lock(m_mutex);
        return m_busy;
    }

    bool setToolTable(const ngc::ToolTable &tools) {
        std::scoped_lock lock(m_mutex);

        if(m_busy) {
            return false;
        }

        m_session.machine().toolTable() = tools;
        return true;
    }

    std::vector<ngc::Parser::Error> parserErrors() const {
        std::scoped_lock lock(m_mutex);
        return m_session.parserErrors();
    }

    std::vector<std::string> blockMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_session.blockMessages();
    }

    bool compile(const std::vector<std::tuple<std::string, std::string>> &programs) {
        std::scoped_lock lock(m_mutex);

        if(m_busy) {
            return false;
        }

        m_session.setPrograms(programs);
        m_doCompile = true;
        m_cv.notify_one();
        return true;
    }

    void join() {
        std::unique_lock lock(m_mutex);
        if(!m_thread.joinable()) return;
        m_doJoin = true;
        m_session.requestStop();
        m_cv.notify_one();
        lock.unlock();
        m_thread.join();
    }

    bool execute() {
        std::scoped_lock lock(m_mutex);
        
        if(m_busy) {
            return false;
        }

        m_doExecute = true;
        m_cv.notify_one();
        return true;
    }

    std::vector<ngc::InterpreterStatusMessage> statusMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_session.statusMessages();
    }

    bool clearToolpath() {
        std::scoped_lock lock(m_mutex);
        if(m_busy) return false;
        m_toolpath.clear();
        return true;
    }

private:
    void work() {
        for(;;) {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [&] { return m_doJoin || m_doCompile || m_doExecute; });

            m_busy = true;

            if(m_doJoin) {
                m_doJoin = false;
                return;
            }

            if(m_doCompile) {
                m_doCompile = false;
                lock.unlock();
                doCompile();
                continue;
            }

            if(m_doExecute) {
                m_doExecute = false;
                lock.unlock();
                doExecute();
                continue;
            }
        }
    }

    void doCompile() {
        m_session.compile([&](const auto &callback) {
            std::scoped_lock lock(m_mutex);
            callback();
        });

        std::scoped_lock lock(m_mutex);
        m_busy = false;
    }

    void doExecute() {
        {
            std::scoped_lock lock(m_mutex);
            m_session.begin();
            m_toolpath.clear();
        }

        // Preview consumes canonical geometry directly. Timed polynomial
        // planning, constraint verification, packetization, and backend
        // execution belong to Simulation/RealRun and provide no display data.
        ngc::ToolpathRecorder preview;
        for(;;) {
            if(joinRequested()) {
                m_session.stop();
                break;
            }

            auto event=m_session.next([&](const auto &callback) {
                std::scoped_lock lock(m_mutex);
                callback();
            });
            if(auto *command=std::get_if<ngc::MachineCommand>(&event)) {
                ngc::position_t activeToolOffset;
                ngc::WorkCoordinateSystem workCoordinateSystem;
                bool g64Active=false;
                std::optional<double> g64Tolerance;
                {
                    std::scoped_lock lock(m_mutex);
                    activeToolOffset=m_session.machine().toolOffset();
                    workCoordinateSystem={
                        std::string(name(*m_session.machine().state().modeCoordSys)),
                        m_session.machine().workOffset()};
                    g64Active=m_session.machine().state().modePath==ngc::GCPath::G64;
                    g64Tolerance=m_session.machine().pathTolerance();
                }
                std::optional<ngc::ProbeMove> probe;
                if(const auto *value=std::get_if<ngc::ProbeMove>(command)) probe=*value;
                preview.consume(std::move(*command),activeToolOffset,std::move(workCoordinateSystem),
                    g64Active,g64Tolerance);
                if(probe) {
                    // Geometry preview assumes contact at the canonical probe
                    // target. Physical tool length/contact simulation belongs
                    // only to timed execution.
                    const auto result=ngc::ProbeResult{
                        probe->id(),ngc::ProbeStatus::Triggered,probe->target(),probe->target()};
                    std::scoped_lock lock(m_mutex);
                    m_session.provideProbeResult(result);
                }
            } else if(std::holds_alternative<ngc::InterpreterCompleted>(event)) {
                break;
            } else if(std::holds_alternative<ngc::InterpreterError>(event)) {
                break;
            } else if(const auto *waiting=std::get_if<ngc::InterpreterWaitingForProbe>(&event)) {
                {
                    std::scoped_lock lock(m_mutex);
                    m_session.reportError(std::format(
                        "preview reached probe barrier {} without its canonical probe command",
                        waiting->commandId));
                }
                m_session.stop();
                break;
            }
        }

        std::scoped_lock lock(m_mutex);
        m_toolpath.replace(std::move(preview));
        refreshParameterSnapshot();
        m_busy = false;
    }

    void refreshParameterSnapshot() {
        for(const auto &[var, _name, _address, _flags, _value] : ngc::gVars) {
            m_parameterSnapshot.insert_or_assign(var, m_session.machine().memory().read(var));
        }
    }

    bool joinRequested() const {
        std::scoped_lock lock(m_mutex);
        return m_doJoin;
    }
};
