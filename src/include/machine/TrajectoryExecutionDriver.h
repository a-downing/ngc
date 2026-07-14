#pragma once

#include <concepts>
#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "evaluator/InterpreterSession.h"
#include "machine/BoundedLookaheadTrajectoryPlanner.h"
#include "machine/MotionBackend.h"

namespace ngc {
    enum class TrajectoryDriverState { Running, Completed, Error };

    // NRT coordinator. It is the sole producer of plans/control requests and the
    // sole consumer of backend events/snapshots. The GUI must communicate with
    // the owner of this object, never with MotionBackend directly.
    class TrajectoryExecutionDriver {
        InterpreterSession &m_session;
        MotionBackend &m_backend;
        BoundedLookaheadTrajectoryPlanner m_planner;
        std::unique_ptr<PlannedExecution> m_pending;
        std::optional<std::string> m_error;
        std::size_t m_outstandingChunks = 0;
        bool m_interpretationComplete = false;
        bool m_probePending = false;
        bool m_synchronizationPending = false;
        bool m_waitingForHeld = false;
        EpochId m_epoch = 1;
        RequestId m_nextRequest = 1;
        RequestId m_startRequest = 0;
        bool m_backendReady = false;
        std::vector<BlockExecution> m_activeBlocks;
        std::optional<TrajectoryPlannerInput> m_deferredInput;

        template<typename Observe>
        void observePlanned(const PlannedExecution &planned, Observe &&observe) {
            for(std::size_t index=0; index<planned.inputs.size(); ++index) {
                const auto &input=planned.inputs[index];
                const auto activation=index<planned.activationSpans.size()
                    ? planned.activationSpans[index] : SpanId{};
                if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &,
                                             const TrajectoryPlanningMetadata &,
                                             const TrajectoryCommandPresentation &, SpanId>)
                    observe(input.command,planned.item,input.metadata,input.presentation,activation);
                else if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &,
                                                  const TrajectoryPlanningMetadata &>)
                    observe(input.command,planned.item,input.metadata);
                else if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &>)
                    observe(input.command,planned.item);
                else observe(input.command);
            }
        }

    public:
        TrajectoryExecutionDriver(InterpreterSession &session, MotionBackend &backend,
                                  TrajectoryLimits limits = {})
            : m_session(session), m_backend(backend), m_planner(limits) { }

        bool begin(const EpochId epoch = 1, const position_t &position = {}) {
            ExecutionEvent staleEvent;
            while(m_backend.tryTakeEvent(staleEvent)) { }
            m_pending.reset();
            m_error.reset();
            m_outstandingChunks = 0;
            m_interpretationComplete = false;
            m_probePending = false;
            m_synchronizationPending = false;
            m_waitingForHeld = false;
            m_backendReady = false;
            m_activeBlocks.clear();
            m_deferredInput.reset();
            m_epoch = epoch;
            m_planner.clearDiagnostics();
            m_planner.reset(epoch, position);
            if(m_backend.trySubmit(ResetRequest { m_nextRequest++, epoch }) != SubmitResult::Submitted) return false;
            m_startRequest = m_nextRequest++;
            return m_backend.trySubmit(StartRequest { m_startRequest, epoch }) == SubmitResult::Submitted;
        }

        void setLimits(const TrajectoryLimits &limits) { m_planner.setLimits(limits); }

        template<typename Synchronize, typename Observe, typename ObserveLifecycle = std::nullptr_t>
        bool pumpOne(Synchronize &&synchronize, Observe &&observe, ObserveLifecycle &&observeLifecycle = nullptr) {
            if(m_error || !m_backendReady || m_waitingForHeld) return false;
            if(m_pending) {
                const auto publication = m_backend.tryPublish(m_pending->item);
                if(publication == PublishResult::Full) return false;
                if(publication != PublishResult::Published) {
                    m_error = "trajectory backend rejected a pending planner-produced chunk";
                    return false;
                }
                observePlanned(*m_pending,std::forward<Observe>(observe));
                m_probePending = std::ranges::any_of(m_pending->inputs,[](const auto &input) {
                    return std::holds_alternative<ProbeMove>(input.command);
                });
                ++m_outstandingChunks;
                m_pending.reset();
                return true;
            }
            const auto publishPlanned=[&](std::unique_ptr<PlannedExecution> compiled) {
                const auto result=m_backend.tryPublish(compiled->item);
                if(result==PublishResult::Published) {
                    observePlanned(*compiled,std::forward<Observe>(observe));
                    m_probePending=std::ranges::any_of(compiled->inputs,[](const auto &input) {
                        return std::holds_alternative<ProbeMove>(input.command);
                    });
                    ++m_outstandingChunks;
                } else if(result==PublishResult::Full) m_pending=std::move(compiled);
                else m_error="trajectory backend rejected a planner-produced chunk";
            };
            const auto planWindow=[&]() -> bool {
                auto planned=m_planner.planWindow();
                if(!planned) { m_error=planned.error(); return false; }
                if(!*planned) { m_error="trajectory lookahead produced no planned execution"; return false; }
                publishPlanned(std::move(*planned));
                return true;
            };

            if((m_interpretationComplete || m_synchronizationPending)
               &&m_planner.windowSize()!=0) {
                (void)planWindow();
                return true;
            }
            if(m_interpretationComplete || m_synchronizationPending) return false;
            if(m_probePending) return false;

            if(m_deferredInput) {
                auto input=std::move(*m_deferredInput);
                m_deferredInput.reset();
                if(m_planner.windowSize()!=0 && !m_planner.canAppend(input)) {
                    m_deferredInput=std::move(input);
                    (void)planWindow();
                    return true;
                }
                const auto retain=BoundedLookaheadTrajectoryPlanner::eligibleForLookahead(input);
                if(!m_planner.enqueue(std::move(input))) {
                    m_error="bounded trajectory lookahead window is full";
                    return true;
                }
                if(retain) return true;
                (void)planWindow();
                return true;
            }

            auto event = m_session.nextWithBlocks(std::forward<Synchronize>(synchronize));
            if(const auto *lifecycle = std::get_if<InterpreterBlockLifecycle>(&event)) {
                if(lifecycle->phase==BlockLifecyclePhase::Entered) m_activeBlocks.push_back(lifecycle->block);
                else if(const auto found=std::ranges::find(m_activeBlocks,lifecycle->block.id,&BlockExecution::id);
                        found!=m_activeBlocks.end()) m_activeBlocks.erase(found);
                if constexpr(!std::same_as<std::remove_cvref_t<ObserveLifecycle>, std::nullptr_t>)
                    observeLifecycle(*lifecycle);
            } else if(auto command = std::get_if<MachineCommand>(&event)) {
                const TrajectoryPlanningMetadata metadata {
                    .pathMode = m_session.machine().state().modePath == GCPath::G64
                        ? ExecutablePathMode::Continuous : ExecutablePathMode::ExactStop,
                    .pathTolerance = m_session.machine().pathTolerance(),
                };
                const TrajectoryCommandPresentation presentation {
                    .tool=m_session.machine().toolGeometry(),
                    .activeToolOffset=m_session.machine().toolOffset(),
                    .workCoordinateSystem=WorkCoordinateSystem {
                        std::string(name(*m_session.machine().state().modeCoordSys)),
                        m_session.machine().workOffset()},
                    .modalGCodes=m_session.machine().activeModalGCodes(),
                    .activeBlocks=m_activeBlocks,
                };
                TrajectoryPlannerInput input{*command,metadata,presentation};
                if(m_planner.windowSize()!=0 && !m_planner.canAppend(input)) {
                    m_deferredInput=std::move(input);
                    (void)planWindow();
                } else {
                    const auto retain=BoundedLookaheadTrajectoryPlanner::eligibleForLookahead(input);
                    if(!m_planner.enqueue(std::move(input))) {
                        m_error="bounded trajectory lookahead window is full";
                        return true;
                    }
                    if(!retain || m_planner.full()) (void)planWindow();
                }
            } else if(std::holds_alternative<InterpreterCompleted>(event)) {
                if(m_planner.windowSize()!=0) (void)planWindow();
                m_interpretationComplete = true;
            } else if(std::holds_alternative<InterpreterWaitingForSynchronization>(event)) {
                if(m_planner.windowSize()!=0) {
                    (void)planWindow();
                    m_synchronizationPending=true;
                } else if(m_outstandingChunks == 0 && !m_pending) {
                    m_session.provideSynchronization();
                } else {
                    m_synchronizationPending = true;
                }
            } else if(const auto *error = std::get_if<InterpreterError>(&event)) {
                m_error = error->message;
            }
            return true;
        }

        void serviceBackend() {
            serviceBackend([](const ExecutionEvent &) { });
        }

        template<typename Observe>
        void serviceBackend(Observe &&observe) {
            ExecutionEvent event;
            while(m_backend.tryTakeEvent(event)) {
                observe(event);
                if(const auto *move = std::get_if<TriggeredMoveCompleted>(&event)) {
                    if(move->epoch == m_epoch && m_probePending) {
                        const auto status = [&] {
                            switch(move->status) {
                            case TriggeredMoveStatus::Triggered: return ProbeStatus::Triggered;
                            case TriggeredMoveStatus::ReachedTarget: return ProbeStatus::ReachedTarget;
                            case TriggeredMoveStatus::Aborted: return ProbeStatus::Aborted;
                            case TriggeredMoveStatus::Fault: return ProbeStatus::Fault;
                            }
                            return ProbeStatus::Fault;
                        }();
                        m_session.provideProbeResult({ move->move, status,
                            move->triggerState.position, move->stoppedState.position });
                        m_probePending = false;
                    }
                } else if(const auto *retired = std::get_if<ChunkRetired>(&event)) {
                    if(retired->epoch == m_epoch && m_outstandingChunks > 0) --m_outstandingChunks;
                } else if(const auto *fault = std::get_if<BackendFault>(&event)) {
                    m_error = "motion backend fault " + std::to_string(fault->code);
                } else if(const auto *rejected = std::get_if<ChunkRejected>(&event)) {
                    if(rejected->epoch == m_epoch)
                        m_error = "motion backend rejected chunk " + std::to_string(rejected->chunk);
                } else if(const auto *branch = std::get_if<BranchSelected>(&event)) {
                    if(branch->epoch == m_epoch && branch->choice == BranchChoice::Stop)
                        m_waitingForHeld = true;
                } else if(const auto *held = std::get_if<BackendHeld>(&event)) {
                    if(held->epoch == m_epoch) {
                        m_waitingForHeld = false;
                        if(m_synchronizationPending) {
                            m_session.provideSynchronization();
                            m_synchronizationPending = false;
                        }
                    }
                    if(held->epoch == m_epoch && !m_interpretationComplete && !m_probePending) {
                        ++m_epoch;
                        m_backendReady = false;
                        m_outstandingChunks = 0;
                        m_planner.reset(m_epoch, held->state.position);
                        if(m_pending) {
                            auto pending = std::move(m_pending);
                            for(auto &input:pending->inputs) {
                                if(!m_planner.enqueue(std::move(input))) {
                                    m_error = "bounded trajectory lookahead window is full during held-state recovery";
                                    break;
                                }
                            }
                            if(!m_error) {
                                auto replanned = m_planner.planWindow();
                                if(!replanned) {
                                    m_error = replanned.error();
                                    m_pending.reset();
                                } else if(!*replanned) {
                                    m_error = "trajectory lookahead lost a command during held-state recovery";
                                    m_pending.reset();
                                } else {
                                    m_pending = std::move(*replanned);
                                }
                            } else m_pending.reset();
                        } else {
                            m_pending.reset();
                        }
                        if(m_backend.trySubmit(ResetRequest { m_nextRequest++, m_epoch }) != SubmitResult::Submitted) {
                            m_error = "motion backend control channel is full during held-state recovery";
                        } else {
                            m_startRequest = m_nextRequest++;
                            if(m_backend.trySubmit(StartRequest { m_startRequest, m_epoch }) != SubmitResult::Submitted)
                                m_error = "motion backend control channel is full during held-state recovery";
                        }
                    }
                } else if(const auto *completed = std::get_if<RequestCompleted>(&event)) {
                    if(completed->request == m_startRequest) {
                        m_backendReady = completed->succeeded;
                        if(!completed->succeeded) m_error = "motion backend rejected start request";
                    }
                }
            }
        }

        bool canPublish() const { return !m_pending; }

        TrajectoryDriverState state() const {
            if(m_error) return TrajectoryDriverState::Error;
            if(m_interpretationComplete && !m_pending && !m_deferredInput
               && m_planner.windowSize()==0 && m_outstandingChunks == 0)
                return TrajectoryDriverState::Completed;
            return TrajectoryDriverState::Running;
        }

        const std::optional<std::string> &error() const { return m_error; }
        std::size_t outstandingChunks() const { return m_outstandingChunks; }
        bool interpretationComplete() const { return m_interpretationComplete; }
        bool probePending() const { return m_probePending; }
        bool synchronizationPending() const { return m_synchronizationPending; }
        EpochId epoch() const { return m_epoch; }
        bool waitingForHeld() const { return m_waitingForHeld; }
        const TrajectoryPlanningDiagnostics &planningDiagnostics() const { return m_planner.diagnostics(); }
        std::size_t lookaheadWindowSize() const { return m_planner.windowSize(); }
    };
}
