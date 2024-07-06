#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <print>
#include <thread>
#include <unistd.h>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
#include "memory/Memory.h"
#include "memory/Vars.h"
#include "parser/Program.h"

class Worker {
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    ngc::Machine m_machine;

    bool m_doJoin = false;
    bool m_doCompile = false;
    bool m_doExecute = false;
    bool m_compiled = false;
    bool m_busy = false;

    std::vector<ngc::Program> m_programs;
    std::vector<ngc::Parser::Error> m_parserErrors;
    std::vector<std::string> m_printMessages;
    std::vector<std::string> m_blockMessages;

public:
    Worker() {
        m_thread = std::thread(&Worker::work, this);
    }

    double read(ngc::Var var) {
        std::scoped_lock lock(m_mutex);
        return m_machine.memory().read(var);
    }

    auto lock(const auto &callback) const {
        std::scoped_lock lock(m_mutex);
        return callback();
    }

    const ngc::Machine &machine() const { return m_machine; }

    bool compiled() const {
        std::scoped_lock lock(m_mutex);
        return m_compiled;
    }

    bool busy() const {
        std::scoped_lock lock(m_mutex);
        return m_busy;
    }

    std::vector<ngc::Parser::Error> parserErrors() const {
        std::scoped_lock lock(m_mutex);
        return m_parserErrors;
    }

    std::vector<std::string> printMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_printMessages;
    }

    std::vector<std::string> blockMessages() const {
        std::scoped_lock lock(m_mutex);
        return m_blockMessages;
    }

    bool compile(const std::vector<std::tuple<std::string, std::string>> &programs) {
        std::scoped_lock lock(m_mutex);

        if(m_busy) {
            return false;
        }

        m_programs.clear();
        m_parserErrors.clear();
        m_printMessages.clear();
        m_blockMessages.clear();
        
        for(auto &[source, name] : programs) {
            m_programs.emplace_back(source, name);
        }

        m_compiled = false;
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
        m_parserErrors.clear();

        for(auto &program : m_programs) {
            auto result = program.compile();

            if(!result) {
                std::scoped_lock lock(m_mutex);
                m_parserErrors.emplace_back(std::move(result.error()));
            }
        }

        std::scoped_lock lock(m_mutex);

        if(m_parserErrors.empty() && !m_programs.empty()) {
            m_compiled = true;
        }

        m_busy = false;
    }

    void doExecute() {
        std::function callback = [&] (std::unique_ptr<const ngc::EvaluatorMessage> msg, ngc::Evaluator &eval) {
            if(auto blockMsg = msg->as<ngc::BlockMessage>()) {
                ngc::GCodeState state;

                for(const auto &word : blockMsg->block().words()) {
                    state.affectState(word);
                }

                if(state.modeToolChange) {
                    eval.call("_tool_change", *state.T);
                }

                std::scoped_lock lock(m_mutex);
                m_blockMessages.emplace_back(blockMsg->block().statement()->text());

                m_machine.executeBlock(blockMsg->block());
            }

            if(auto printMsg = msg->as<ngc::PrintMessage>()) {
                std::scoped_lock lock(m_mutex);
                m_printMessages.emplace_back(printMsg->text());
            }
        };

        auto eval = ngc::Evaluator(m_machine.memory(), callback);

        auto preamble = ngc::Preamble(m_machine.memory());
        eval.executeFirstPass(preamble.statements());

        for(const auto &program : m_programs) {
            eval.executeFirstPass(program.statements());
        }

        for(const auto &program : m_programs) {
            eval.executeSecondPass(program.statements());
        }

        std::scoped_lock lock(m_mutex);
        m_busy = false;
    }
};