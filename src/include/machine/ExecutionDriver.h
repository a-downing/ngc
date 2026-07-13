#pragma once

#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

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
        std::vector<BlockExecution> m_blockStack;

    public:
        ExecutionDriver(InterpreterSession &session, SimulationExecutor &executor)
            : m_session(session), m_executor(executor) { }

        void reset() {
            m_interpretationComplete = false;
            m_probePending = false;
            m_error.reset();
            m_blockStack.clear();
        }

        template<typename Synchronize, typename Observe>
        bool pumpOne(Synchronize &&synchronize, Observe &&observe) {
            if(m_interpretationComplete || m_probePending || !m_executor.canAccept() || m_error) return false;

            auto event = m_session.nextWithBlocks(std::forward<Synchronize>(synchronize));
            if(auto lifecycle = std::get_if<InterpreterBlockLifecycle>(&event)) {
                if(lifecycle->phase == BlockLifecyclePhase::Entered) {
                    m_blockStack.emplace_back(lifecycle->block);
                } else {
                    const auto match = std::ranges::find_if(m_blockStack, [&](const BlockExecution &block) {
                        return block.id == lifecycle->block.id;
                    });
                    if(match != m_blockStack.end()) m_blockStack.erase(match);
                }
                m_executor.consume(SimulatedCommand::lifecycleMarker(*lifecycle, m_session.machine().activeModalGCodes()));
            } else if(auto command = std::get_if<MachineCommand>(&event)) {
                const auto toolOffset = m_session.machine().toolOffset();
                const auto tool = m_session.machine().toolGeometry();
                const WorkCoordinateSystem workCoordinateSystem {
                    .name = std::string(name(*m_session.machine().state().modeCoordSys)),
                    .offset = m_session.machine().workOffset(),
                };
                m_probePending = std::holds_alternative<ProbeMove>(*command);
                observe(*command, toolOffset, tool, workCoordinateSystem);
                m_executor.consume({ std::move(*command), toolOffset, tool, workCoordinateSystem,
                                     m_session.machine().activeModalGCodes(),
                                     m_blockStack.empty() ? std::nullopt : std::optional { m_blockStack.back() } });
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
