#pragma once

#include <optional>
#include <string>
#include <utility>

#include "evaluator/InterpreterSession.h"
#include "machine/SimulationExecutor.h"

namespace ngc {
    enum class ExecutionDriverState {
        Running,
        Completed,
        Error,
    };

    class ExecutionDriver {
        InterpreterSession &m_session;
        SimulationExecutor &m_executor;
        bool m_interpretationComplete = false;
        bool m_probePending = false;
        std::optional<std::string> m_error;

    public:
        ExecutionDriver(InterpreterSession &session, SimulationExecutor &executor)
            : m_session(session), m_executor(executor) { }

        void reset() {
            m_interpretationComplete = false;
            m_probePending = false;
            m_error.reset();
        }

        template<typename Synchronize, typename Observe>
        bool pumpOne(Synchronize &&synchronize, Observe &&observe) {
            if(m_interpretationComplete || m_probePending || !m_executor.canAccept() || m_error) return false;

            auto event = m_session.next(std::forward<Synchronize>(synchronize));
            if(auto command = std::get_if<MachineCommand>(&event)) {
                const auto toolOffset = m_session.machine().toolOffset();
                const auto tool = m_session.machine().toolGeometry();
                m_probePending = std::holds_alternative<ProbeMove>(*command);
                observe(*command, toolOffset, tool);
                m_executor.consume({ std::move(*command), toolOffset, tool });
            } else if(std::holds_alternative<InterpreterCompleted>(event)) {
                m_interpretationComplete = true;
            } else if(const auto error = std::get_if<InterpreterError>(&event)) {
                m_error = error->message;
            }
            return true;
        }

        void deliverProbeResult() {
            if(auto result = m_executor.takeProbeResult()) {
                m_session.provideProbeResult(*result);
                m_probePending = false;
            }
        }

        ExecutionDriverState state() const {
            if(m_error) return ExecutionDriverState::Error;
            if(m_interpretationComplete && m_executor.empty()) return ExecutionDriverState::Completed;
            return ExecutionDriverState::Running;
        }

        const std::optional<std::string> &error() const { return m_error; }
    };
}
