#pragma once

#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "evaluator/Evaluator.h"
#include "evaluator/Preamble.h"
#include "machine/Machine.h"
#include "parser/Program.h"

namespace ngc {
    class InterpreterSession {
        Machine m_machine;
        std::vector<Program> m_programs;
        std::vector<Parser::Error> m_parserErrors;
        std::vector<std::string> m_printMessages;
        std::vector<std::string> m_blockMessages;
        bool m_compiled = false;

    public:
        explicit InterpreterSession(const Machine::Unit unit) : m_machine(unit) { }

        template<typename Self> auto &machine(this Self &&self) { return std::forward<Self>(self).m_machine; }
        template<typename Self> auto &parserErrors(this Self &&self) { return std::forward<Self>(self).m_parserErrors; }
        template<typename Self> auto &printMessages(this Self &&self) { return std::forward<Self>(self).m_printMessages; }
        template<typename Self> auto &blockMessages(this Self &&self) { return std::forward<Self>(self).m_blockMessages; }

        bool compiled() const { return m_compiled; }

        void setPrograms(const std::vector<std::tuple<std::string, std::string>> &programs) {
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

        template<typename Synchronize>
        void execute(Synchronize &&synchronize) {
            synchronize([&] {
                m_machine.beginProgramRun();
                m_printMessages.clear();
                m_blockMessages.clear();
            });

            const std::function callback = [&] (std::unique_ptr<const EvaluatorMessage> message, Evaluator &evaluator) {
                if(const auto blockMessage = message->as<BlockMessage>()) {
                    GCodeState state;

                    for(const auto &word : blockMessage->block().words()) {
                        state.affectState(word);
                    }

                    if(state.modeToolChange) {
                        evaluator.call("_tool_change", *state.T);
                    }

                    synchronize([&] {
                        m_blockMessages.emplace_back(blockMessage->block().statement()->text());
                        m_machine.executeBlock(blockMessage->block());
                    });
                }

                if(const auto printMessage = message->as<PrintMessage>()) {
                    synchronize([&] {
                        m_printMessages.emplace_back(printMessage->text());
                    });
                }
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
        }
    };
}
