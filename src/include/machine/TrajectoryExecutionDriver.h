#pragma once

#include <concepts>
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
            if(m_error || !m_backendReady || m_interpretationComplete || m_waitingForHeld
               || m_synchronizationPending) return false;
            if(m_pending) {
                const auto publication = m_backend.tryPublish(m_pending->item);
                if(publication == PublishResult::Full) return false;
                if(publication != PublishResult::Published) {
                    m_error = "trajectory backend rejected a pending planner-produced chunk";
                    return false;
                }
                if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &,
                                             const TrajectoryPlanningMetadata &>)
                    observe(m_pending->command, m_pending->item, m_pending->metadata);
                else if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &>)
                    observe(m_pending->command, m_pending->item);
                else
                    observe(m_pending->command);
                m_probePending = std::holds_alternative<ProbeMove>(m_pending->command);
                ++m_outstandingChunks;
                m_pending.reset();
                return true;
            }
            if(m_probePending) return false;

            auto event = m_session.nextWithBlocks(std::forward<Synchronize>(synchronize));
            if(const auto *lifecycle = std::get_if<InterpreterBlockLifecycle>(&event)) {
                if constexpr(!std::same_as<std::remove_cvref_t<ObserveLifecycle>, std::nullptr_t>)
                    observeLifecycle(*lifecycle);
            } else if(auto command = std::get_if<MachineCommand>(&event)) {
                const TrajectoryPlanningMetadata metadata {
                    .pathMode = m_session.machine().state().modePath == GCPath::G64
                        ? ExecutablePathMode::Continuous : ExecutablePathMode::ExactStop,
                    .pathTolerance = m_session.machine().pathTolerance(),
                };
                if(!m_planner.enqueue({ *command, metadata })) {
                    m_error = "bounded trajectory lookahead window is full";
                    return true;
                }
                auto planned = m_planner.planOne();
                if(!planned) {
                    m_error = planned.error();
                    return true;
                }
                if(!*planned) {
                    m_error = "trajectory lookahead accepted a command without producing an exact-stop fallback";
                    return true;
                }
                auto compiled = std::move(*planned);
                const auto result = m_backend.tryPublish(compiled->item);
                if(result == PublishResult::Published) {
                    if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &,
                                                 const TrajectoryPlanningMetadata &>)
                        observe(compiled->command, compiled->item, compiled->metadata);
                    else if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &>)
                        observe(compiled->command, compiled->item);
                    else
                        observe(compiled->command);
                    m_probePending = std::holds_alternative<ProbeMove>(compiled->command);
                    ++m_outstandingChunks;
                } else if(result == PublishResult::Full) {
                    m_pending = std::move(compiled);
                }
                else m_error = "trajectory backend rejected a planner-produced chunk";
            } else if(std::holds_alternative<InterpreterCompleted>(event)) {
                m_interpretationComplete = true;
            } else if(std::holds_alternative<InterpreterWaitingForSynchronization>(event)) {
                if(m_outstandingChunks == 0 && !m_pending) {
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
                            if(!m_planner.enqueue({ pending->command, pending->metadata })) {
                                m_error = "bounded trajectory lookahead window is full during held-state recovery";
                                m_pending.reset();
                            } else {
                                auto replanned = m_planner.planOne();
                                if(!replanned) {
                                    m_error = replanned.error();
                                    m_pending.reset();
                                } else if(!*replanned) {
                                    m_error = "trajectory lookahead lost a command during held-state recovery";
                                    m_pending.reset();
                                } else {
                                    m_pending = std::move(*replanned);
                                }
                            }
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
            if(m_interpretationComplete && !m_pending && m_outstandingChunks == 0)
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
