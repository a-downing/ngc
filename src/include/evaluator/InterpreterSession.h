#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
#include "parser/Program.h"

namespace ngc {
    struct InterpreterCompleted { };

    struct InterpreterError {
        std::string message;
    };

    struct InterpreterWaitingForProbe {
        std::uint64_t commandId;
    };

    using InterpreterEvent = std::variant<MachineCommand, InterpreterWaitingForProbe, InterpreterCompleted, InterpreterError>;

    class InterpreterSession {
        struct ExecutionStopped { };

        Machine m_machine;
        std::vector<Program> m_programs;
        std::vector<Parser::Error> m_parserErrors;
        std::vector<std::string> m_printMessages;
        std::vector<std::string> m_blockMessages;
        bool m_compiled = false;

        std::mutex m_executionMutex;
        std::condition_variable m_executionCv;
        std::thread m_executionThread;
        std::unique_ptr<const EvaluatorMessage> m_pendingMessage;
        std::deque<MachineCommand> m_pendingCommands;
        std::optional<std::uint64_t> m_pendingProbe;
        bool m_evaluatorPaused = false;
        bool m_resumeEvaluator = false;
        bool m_executionStarted = false;
        bool m_executionFinished = false;
        bool m_stopExecution = false;
        std::optional<std::string> m_executionError;

    public:
        explicit InterpreterSession(const Machine::Unit unit) : m_machine(unit) { }

        ~InterpreterSession() {
            stop();
        }

        InterpreterSession(const InterpreterSession &) = delete;
        InterpreterSession &operator=(const InterpreterSession &) = delete;

        template<typename Self> auto &machine(this Self &&self) { return std::forward<Self>(self).m_machine; }
        template<typename Self> auto &parserErrors(this Self &&self) { return std::forward<Self>(self).m_parserErrors; }
        template<typename Self> auto &printMessages(this Self &&self) { return std::forward<Self>(self).m_printMessages; }
        template<typename Self> auto &blockMessages(this Self &&self) { return std::forward<Self>(self).m_blockMessages; }

        bool compiled() const { return m_compiled; }

        void setPrograms(const std::vector<std::tuple<std::string, std::string>> &programs) {
            stop();
            m_programs.clear();
            m_parserErrors.clear();
            m_printMessages.clear();
            m_blockMessages.clear();

            for(const auto &[source, name] : programs) {
                m_programs.emplace_back(source, name);
            }

            m_compiled = false;
        }

        template<typename Synchronize>
        void compile(Synchronize &&synchronize) {
            stop();
            std::vector<Parser::Error> errors;

            for(auto &program : m_programs) {
                auto result = program.compile();
                if(!result) {
                    errors.emplace_back(std::move(result.error()));
                }
            }

            synchronize([&] {
                m_parserErrors = std::move(errors);
                m_compiled = m_parserErrors.empty() && !m_programs.empty();
            });
        }

        void begin() {
            stop();
            m_machine.beginProgramRun();
            m_printMessages.clear();
            m_blockMessages.clear();
            m_pendingCommands.clear();
            m_pendingProbe.reset();
            m_pendingMessage.reset();
            m_executionError.reset();
            m_evaluatorPaused = false;
            m_resumeEvaluator = false;
            m_executionFinished = false;
            m_stopExecution = false;
            m_executionStarted = true;

            if(!m_compiled) {
                m_executionError = "interpreter session has not compiled a program";
                m_executionFinished = true;
                return;
            }

            m_executionThread = std::thread(&InterpreterSession::evaluate, this);
        }

        InterpreterEvent next() {
            return next([](const auto &callback) { callback(); });
        }

        template<typename Synchronize>
        InterpreterEvent next(Synchronize &&synchronize) {
            if(!m_executionStarted) {
                begin();
            }

            for(;;) {
                if(m_pendingProbe) {
                    return InterpreterWaitingForProbe { *m_pendingProbe };
                }

                if(!m_pendingCommands.empty()) {
                    auto command = std::move(m_pendingCommands.front());
                    m_pendingCommands.pop_front();

                    if(const auto probe = std::get_if<ProbeMove>(&command)) {
                        m_pendingProbe = probe->id();
                    }

                    return command;
                }

                resumePausedEvaluator();

                std::unique_lock lock(m_executionMutex);
                m_executionCv.wait(lock, [&] {
                    return m_pendingMessage || m_executionFinished;
                });

                if(m_pendingMessage) {
                    auto message = std::move(m_pendingMessage);
                    lock.unlock();
                    synchronize([&] { processMessage(*message); });
                    continue;
                }

                const auto error = m_executionError;
                lock.unlock();
                finishExecutionThread();

                if(error) {
                    return InterpreterError { *error };
                }

                return InterpreterCompleted {};
            }
        }

        void provideProbeResult(const ProbeResult &result) {
            if(!m_pendingProbe || *m_pendingProbe != result.id) {
                throw std::logic_error("probe result does not match the pending probe command");
            }

            m_machine.acceptProbeResult(result);
            m_pendingProbe.reset();
        }

        void stop() {
            {
                std::scoped_lock lock(m_executionMutex);
                m_stopExecution = true;
                m_resumeEvaluator = true;
            }
            m_executionCv.notify_all();
            finishExecutionThread();
            m_executionStarted = false;
            m_evaluatorPaused = false;
            m_pendingMessage.reset();
            m_pendingCommands.clear();
            m_pendingProbe.reset();
        }

    private:
        void evaluate() {
            try {
                const std::function callback = [&] (std::unique_ptr<const EvaluatorMessage> message, Evaluator &evaluator) {
                    if(const auto blockMessage = message->as<BlockMessage>()) {
                        GCodeState state;

                        for(const auto &word : blockMessage->block().words()) {
                            state.affectState(word);
                        }

                        if(state.modeToolChange) {
                            evaluator.call("_tool_change", *state.T);
                        }
                    }

                    publishMessage(std::move(message));
                };

                Evaluator evaluator(m_machine.memory(), callback);
                Preamble preamble(m_machine.memory());
                evaluator.executeFirstPass(preamble.statements());

                for(const auto &program : m_programs) {
                    evaluator.executeFirstPass(program.statements());
                }

                for(const auto &program : m_programs) {
                    evaluator.executeSecondPass(program.statements());
                }
            } catch(const ExecutionStopped &) {
            } catch(const std::exception &error) {
                std::scoped_lock lock(m_executionMutex);
                m_executionError = error.what();
            } catch(...) {
                std::scoped_lock lock(m_executionMutex);
                m_executionError = "unknown interpreter error";
            }

            {
                std::scoped_lock lock(m_executionMutex);
                m_executionFinished = true;
            }
            m_executionCv.notify_all();
        }

        void publishMessage(std::unique_ptr<const EvaluatorMessage> message) {
            std::unique_lock lock(m_executionMutex);
            m_pendingMessage = std::move(message);
            m_evaluatorPaused = true;
            m_executionCv.notify_all();
            m_executionCv.wait(lock, [&] { return m_resumeEvaluator || m_stopExecution; });

            if(m_stopExecution) {
                throw ExecutionStopped {};
            }

            m_resumeEvaluator = false;
            m_evaluatorPaused = false;
        }

        void processMessage(const EvaluatorMessage &message) {
            if(const auto blockMessage = message.as<BlockMessage>()) {
                m_blockMessages.emplace_back(blockMessage->block().statement()->text());
                auto commands = m_machine.executeBlock(blockMessage->block());

                for(auto &command : commands) {
                    m_pendingCommands.emplace_back(std::move(command));
                }
            }

            if(const auto printMessage = message.as<PrintMessage>()) {
                m_printMessages.emplace_back(printMessage->text());
            }
        }

        void resumePausedEvaluator() {
            std::scoped_lock lock(m_executionMutex);
            if(m_evaluatorPaused) {
                m_resumeEvaluator = true;
                m_executionCv.notify_all();
            }
        }

        void finishExecutionThread() {
            if(m_executionThread.joinable()) {
                m_executionThread.join();
            }
        }
    };
}
