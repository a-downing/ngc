#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <print>
#include <thread>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "memory/Memory.h"
#include "parser/Program.h"

class Worker {
    ngc::Memory m_mem;

    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    bool m_doJoin = false;
    bool m_doCompile = false;
    bool m_doExecute = false;
    bool m_compiled = false;

    std::vector<ngc::Program> m_programs;
    std::vector<ngc::Parser::Error> m_parserErrors;
    std::vector<std::unique_ptr<const ngc::EvaluatorMessage>> m_evaluatorMessages;

public:
    Worker(ngc::Memory &mem) : m_mem(mem), m_thread(std::thread(&Worker::work, this)) { }

    std::span<const ngc::Program> programs() const {
        std::scoped_lock lock(m_mutex);
        return m_programs;
    }

    std::vector<std::unique_ptr<const ngc::EvaluatorMessage>> moveEvaluatorMessages() {
        std::scoped_lock lock(m_mutex);
        return std::move(m_evaluatorMessages);
    }

    void join() {
        std::unique_lock lock(m_mutex);
        m_doJoin = true;
        m_cv.notify_one();
        lock.unlock();
        m_thread.join();
    }

    void setPrograms(std::vector<ngc::Program> programs) {
        std::scoped_lock lock(m_mutex);
        m_programs = std::move(programs);
        m_compiled = false;
    }

    bool compiled() const {
        std::scoped_lock lock(m_mutex);
        return m_compiled;
    }

    void compile() {
        std::scoped_lock lock(m_mutex);
        m_compiled = false;
        m_doCompile = true;
        m_cv.notify_one();
    }

    void execute() {
        std::scoped_lock lock(m_mutex);
        m_doExecute = true;
        m_cv.notify_one();
    }

private:
    void work() {
        for(;;) {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [&] { return m_doJoin || m_doCompile || m_doExecute; });

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
                m_parserErrors.emplace_back(std::move(result.error()));
            }
        }

        std::scoped_lock lock(m_mutex);

        if(m_parserErrors.empty() && !m_programs.empty()) {
            m_compiled = true;
        }
    }

    void doExecute() {
        m_evaluatorMessages.clear();

        std::function callback = [&] (std::unique_ptr<const ngc::EvaluatorMessage> msg, ngc::Evaluator &eval) {
            if(auto blockMsg = msg->as<ngc::BlockMessage>()) {
                ngc::GCodeState state;

                for(const auto &word : blockMsg->block().words()) {
                    state.affectState(word);
                }

                if(state.modeToolChange()) {
                    eval.call("_tool_change", state.T());
                }

                std::println("BLOCK: {}", blockMsg->block().statement()->text());
            }

            if(auto printMsg = msg->as<ngc::PrintMessage>()) {
                std::println("PRINT: {}", printMsg->text());
            }

            m_evaluatorMessages.emplace_back(std::move(msg));
        };

        auto eval = ngc::Evaluator(m_mem, callback);

        std::println("first pass: preamble");
        auto preamble = ngc::Preamble(m_mem);
        eval.executeFirstPass(preamble.statements());

        for(const auto &program : m_programs) {
            std::println("first pass: {}", program.source().name());
            eval.executeFirstPass(program.statements());
        }

        for(const auto &program : m_programs) {
            std::println("executing: {}", program.source().name());
            eval.executeSecondPass(program.statements());
        }
    }
};