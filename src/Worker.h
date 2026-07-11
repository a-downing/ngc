#pragma once

#include <condition_variable>
#include <format>
#include <mutex>
#include <thread>

#include "evaluator/InterpreterSession.h"
#include "machine/ExecutionDriver.h"
#include "machine/SimulationExecutor.h"
#include "machine/ToolpathRecorder.h"
#include "memory/Vars.h"

class Worker {
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    ngc::InterpreterSession m_session{ ngc::Machine::Unit::Inch, ngc::InterpretationMode::Preview };
    ngc::SimulationExecutor m_executor;
    ngc::ExecutionDriver m_driver{ m_session, m_executor };
    ngc::ToolpathRecorder m_toolpath;

    bool m_doJoin = false;
    bool m_doCompile = false;
    bool m_doExecute = false;
    bool m_busy = false;

public:
    Worker() {
        m_thread = std::thread(&Worker::work, this);
    }

    double read(ngc::Var var) {
        std::scoped_lock lock(m_mutex);
        return m_session.machine().memory().read(var);
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

    std::vector<std::string> printMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_session.printMessages();
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
        m_doJoin = true;
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
            }

            if(m_doExecute) {
                m_doExecute = false;
                lock.unlock();
                doExecute();
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
            m_executor.reset();
            m_driver.reset();
            m_toolpath.clear();
        }

        for(;;) {
            m_driver.pumpOne([&](const auto &callback) {
                std::scoped_lock lock(m_mutex);
                callback();
            }, [&](const ngc::MachineCommand &command, const ngc::position_t &toolOffset, const ngc::ToolGeometry &) {
                std::scoped_lock lock(m_mutex);
                m_toolpath.consume(command, toolOffset);
            });

            m_executor.completeQueued();
            {
                std::scoped_lock lock(m_mutex);
                m_driver.deliverProbeResult();
            }

            if(m_driver.state() == ngc::ExecutionDriverState::Completed) break;
            if(m_driver.state() == ngc::ExecutionDriverState::Error) {
                std::scoped_lock lock(m_mutex);
                m_session.printMessages().emplace_back(*m_driver.error());
                break;
            }
        }

        std::scoped_lock lock(m_mutex);
        m_busy = false;
    }
};
