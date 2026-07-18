#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/InterpreterStatus.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
#include "parser/Program.h"

namespace ngc {
    enum class InterpretationMode {
        Preview,
        Simulation,
        RealRun,
    };

    constexpr double taskValue(const InterpretationMode mode) {
        switch(mode) {
            case InterpretationMode::Preview: return 0.0;
            case InterpretationMode::Simulation: return 1.0;
            case InterpretationMode::RealRun: return 2.0;
        }
        return 0.0;
    }

    struct InterpreterCompleted { };

    struct InterpreterError {
        std::string message;
    };

    struct InterpreterWaitingForProbe {
        std::uint64_t commandId;
    };

    struct InterpreterWaitingForSynchronization { };

    struct BlockExecution {
        std::uint64_t id;
        std::string text;
        std::string source;
        int line;
    };

    enum class BlockLifecyclePhase { Entered, Completed };

    struct InterpreterBlockLifecycle {
        BlockExecution block;
        BlockLifecyclePhase phase;
    };

    using InterpreterEvent = std::variant<MachineCommand, InterpreterBlockLifecycle,
        InterpreterWaitingForProbe, InterpreterWaitingForSynchronization,
        InterpreterCompleted, InterpreterError>;

    class InterpreterSession {
        struct ExecutionStopped { };

        Machine m_machine;
        InterpretationMode m_mode;
        std::vector<Program> m_programs;
        std::vector<Parser::Error> m_parserErrors;
        std::vector<InterpreterStatusMessage> m_statusMessages;
        std::vector<std::string> m_blockMessages;
        bool m_compiled = false;

        std::mutex m_executionMutex;
        std::condition_variable m_executionCv;
        std::thread m_executionThread;
        std::unique_ptr<const EvaluatorMessage> m_pendingMessage;
        std::optional<BlockExecution> m_pendingMessageBlock;
        std::deque<MachineCommand> m_pendingCommands;
        std::deque<InterpreterBlockLifecycle> m_pendingBlockLifecycle;
        std::optional<std::uint64_t> m_pendingProbe;
        bool m_pendingSynchronization = false;
        bool m_evaluatorPaused = false;
        bool m_resumeEvaluator = false;
        bool m_executionStarted = false;
        bool m_executionFinished = false;
        bool m_stopExecution = false;
        std::optional<std::string> m_executionError;
        std::uint64_t m_nextBlockExecutionId = 1;
        std::optional<InterpreterBlockLifecycle> m_publishedBlockLifecycle;

    public:
        InterpreterSession(const Machine::Unit unit, const InterpretationMode mode) : m_machine(unit), m_mode(mode) { }

        ~InterpreterSession() {
            stop();
        }

        InterpreterSession(const InterpreterSession &) = delete;
        InterpreterSession &operator=(const InterpreterSession &) = delete;

        template<typename Self> auto &machine(this Self &&self) { return std::forward<Self>(self).m_machine; }
        template<typename Self> auto &parserErrors(this Self &&self) { return std::forward<Self>(self).m_parserErrors; }
        template<typename Self> auto &statusMessages(this Self &&self) { return std::forward<Self>(self).m_statusMessages; }
        template<typename Self> auto &blockMessages(this Self &&self) { return std::forward<Self>(self).m_blockMessages; }

        void reportError(std::string text) {
            std::println(stderr, "ERROR: {}", text);
            m_statusMessages.push_back({ InterpreterStatusKind::Error, std::move(text) });
        }

        bool compiled() const { return m_compiled; }

        void setPrograms(const std::vector<std::tuple<std::string, std::string>> &programs) {
            stop();
            m_programs.clear();
            m_parserErrors.clear();
            m_statusMessages.clear();
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

            for(const auto &error:errors) std::println(stderr,"ERROR: {}",error.text());

            synchronize([&] {
                m_parserErrors = std::move(errors);
                m_compiled = m_parserErrors.empty() && !m_programs.empty();
            });
        }

        void begin() {
            beginImpl(true);
        }

        void beginContinuation() {
            beginImpl(false);
        }

    private:
        void beginImpl(const bool resetMachine) {
            stop();
            m_statusMessages.clear();
            m_blockMessages.clear();
            m_pendingCommands.clear();
            m_pendingBlockLifecycle.clear();
            m_pendingProbe.reset();
            m_pendingSynchronization = false;
            m_pendingMessage.reset();
            m_pendingMessageBlock.reset();
            m_executionError.reset();
            m_nextBlockExecutionId = 1;
            m_publishedBlockLifecycle.reset();
            m_evaluatorPaused = false;
            m_resumeEvaluator = false;
            m_executionFinished = false;
            m_stopExecution = false;
            m_executionStarted = true;

            try {
                if(resetMachine) m_machine.beginProgramRun();
                m_machine.memory().write(Var::TASK, taskValue(m_mode), true);
            } catch(const std::exception &error) {
                m_executionError = error.what();
                m_executionFinished = true;
                return;
            }

            if(!m_compiled) {
                m_executionError = "interpreter session has not compiled a program";
                m_executionFinished = true;
                return;
            }

            m_executionThread = std::thread(&InterpreterSession::evaluate, this);
        }

    public:

        InterpreterEvent next() {
            return next([](const auto &callback) { callback(); });
        }

        template<typename Synchronize>
        InterpreterEvent next(Synchronize &&synchronize) {
            for(;;) {
                auto event = nextImpl(std::forward<Synchronize>(synchronize), false);
                if(!std::holds_alternative<InterpreterWaitingForSynchronization>(event)) return event;
                provideSynchronization();
            }
        }

        template<typename Synchronize>
        InterpreterEvent nextWithBlocks(Synchronize &&synchronize) {
            return nextImpl(std::forward<Synchronize>(synchronize), true);
        }

    private:
        template<typename Synchronize>
        InterpreterEvent nextImpl(Synchronize &&synchronize, const bool includeBlocks) {
            if(!m_executionStarted) {
                begin();
            }

            for(;;) {
                if(m_pendingProbe) {
                    return InterpreterWaitingForProbe { *m_pendingProbe };
                }

                if(m_pendingSynchronization) return InterpreterWaitingForSynchronization {};

                if(!m_pendingBlockLifecycle.empty()) {
                    auto lifecycle = std::move(m_pendingBlockLifecycle.front());
                    m_pendingBlockLifecycle.pop_front();
                    if(includeBlocks) return lifecycle;
                }

                if(!m_pendingCommands.empty()) {
                    auto command = m_pendingCommands.front();
                    m_pendingCommands.pop_front();

                    if(const auto probe = std::get_if<ProbeMove>(&command)) {
                        m_pendingProbe = probe->id();
                    }

                    return command;
                }

                resumePausedEvaluator();

                std::unique_lock lock(m_executionMutex);
                m_executionCv.wait(lock, [&] {
                    return m_pendingMessage || m_publishedBlockLifecycle || m_executionFinished;
                });

                if(m_publishedBlockLifecycle) {
                    auto lifecycle = std::move(*m_publishedBlockLifecycle);
                    m_publishedBlockLifecycle.reset();
                    lock.unlock();
                    synchronize([&] { m_pendingBlockLifecycle.emplace_back(std::move(lifecycle)); });
                    continue;
                }

                if(m_pendingMessage) {
                    auto message = std::move(m_pendingMessage);
                    auto block = std::exchange(m_pendingMessageBlock, std::nullopt);
                    lock.unlock();
                    if(message->as<SynchronizationMessage>()) {
                        m_pendingSynchronization = true;
                        return InterpreterWaitingForSynchronization {};
                    }
                    try {
                        synchronize([&] { processMessage(*message, block); });
                    } catch(const std::exception &error) {
                        const auto text = statusErrorText(*message, error.what());
                        stop();
                        reportError(text);
                        return InterpreterError { text };
                    } catch(...) {
                        const auto text = statusErrorText(*message, "unknown interpreter error");
                        stop();
                        reportError(text);
                        return InterpreterError { text };
                    }
                    continue;
                }

                const auto error = m_executionError;
                lock.unlock();
                finishExecutionThread();

                if(error) {
                    reportError(*error);
                    return InterpreterError { *error };
                }

                return InterpreterCompleted {};
            }
        }

    public:

        void provideProbeResult(const ProbeResult &result) {
            if(!m_pendingProbe || *m_pendingProbe != result.id) {
                throw std::logic_error("probe result does not match the pending probe command");
            }

            m_machine.acceptProbeResult(result);
            m_pendingProbe.reset();
        }

        void provideSynchronization() {
            if(!m_pendingSynchronization)
                throw std::logic_error("interpreter is not waiting for synchronization");
            m_pendingSynchronization = false;
        }

        void requestStop() {
            {
                std::scoped_lock lock(m_executionMutex);
                m_stopExecution = true;
                m_resumeEvaluator = true;
            }
            m_executionCv.notify_all();
        }

        void stop() {
            requestStop();
            finishExecutionThread();
            m_executionStarted = false;
            m_evaluatorPaused = false;
            m_pendingMessage.reset();
            m_pendingMessageBlock.reset();
            m_publishedBlockLifecycle.reset();
            m_pendingCommands.clear();
            m_pendingBlockLifecycle.clear();
            m_pendingProbe.reset();
            m_pendingSynchronization = false;
        }

    private:
        static std::string statusErrorText(const EvaluatorMessage &message, const std::string_view error) {
            if(const auto blockMessage = message.as<BlockMessage>()) {
                const auto location = blockMessage->block().statement()->startToken().location();
                if(error.starts_with(location)) return std::string(error);
                return std::format("{}: {}", location, error);
            }
            return std::string(error);
        }

        void evaluate() {
            try {
                const std::function callback = [&] (std::unique_ptr<const EvaluatorMessage> message, Evaluator &evaluator) {
                    if(const auto blockMessage = message->as<BlockMessage>()) {
                        const auto &statement = *blockMessage->block().statement();
                        const auto &token = statement.startToken();
                        const BlockExecution execution {
                            .id = m_nextBlockExecutionId++,
                            .text = statement.text(),
                            .source = std::string(token.source()->name()),
                            .line = token.source()->line(),
                        };
                        GCodeState state;

                        try {
                            for(const auto &word : blockMessage->block().words()) {
                                state.affectState(word);
                            }
                        } catch(const std::exception &error) {
                            throw std::runtime_error(std::format("{}: {}", token.location(), error.what()));
                        }

                        if(state.modeToolChange) {
                            evaluator.synchronize();
                            m_machine.prepareToolChange(static_cast<int>(*state.T));
                            publishMessage(std::move(message), execution);
                            evaluator.call("_tool_change", *state.T);
                            publishBlockLifecycle({ execution, BlockLifecyclePhase::Completed });
                            return;
                        }

                        publishMessage(std::move(message), execution);
                        publishBlockLifecycle({ execution, BlockLifecyclePhase::Completed });
                        return;
                    }

                    publishMessage(std::move(message));
                };

                const auto interrupt = [&] {
                    std::scoped_lock lock(m_executionMutex);
                    if(m_stopExecution) throw ExecutionStopped {};
                };
                Evaluator evaluator(m_machine.memory(), callback, interrupt);
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

        void publishMessage(std::unique_ptr<const EvaluatorMessage> message,
                            std::optional<BlockExecution> block = std::nullopt) {
            std::unique_lock lock(m_executionMutex);
            m_pendingMessage = std::move(message);
            m_pendingMessageBlock = std::move(block);
            m_evaluatorPaused = true;
            m_executionCv.notify_all();
            m_executionCv.wait(lock, [&] { return m_resumeEvaluator || m_stopExecution; });

            if(m_stopExecution) {
                throw ExecutionStopped {};
            }

            m_resumeEvaluator = false;
            m_evaluatorPaused = false;
        }

        void publishBlockLifecycle(InterpreterBlockLifecycle lifecycle) {
            std::unique_lock lock(m_executionMutex);
            m_publishedBlockLifecycle = std::move(lifecycle);
            m_evaluatorPaused = true;
            m_executionCv.notify_all();
            m_executionCv.wait(lock, [&] { return m_resumeEvaluator || m_stopExecution; });

            if(m_stopExecution) throw ExecutionStopped {};
            m_resumeEvaluator = false;
            m_evaluatorPaused = false;
        }

        void processMessage(const EvaluatorMessage &message, const std::optional<BlockExecution> &block) {
            if(const auto blockMessage = message.as<BlockMessage>()) {
                auto text = blockMessage->block().statement()->text();
                m_blockMessages.emplace_back(text);
                if(block) m_pendingBlockLifecycle.push_back({ *block, BlockLifecyclePhase::Entered });
                auto commands = m_machine.executeBlock(blockMessage->block());

                for(auto &command : commands) {
                    m_pendingCommands.emplace_back(command);
                }
            }

            if(const auto printMessage = message.as<PrintMessage>()) {
                m_statusMessages.push_back({ InterpreterStatusKind::Print, printMessage->text() });
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
