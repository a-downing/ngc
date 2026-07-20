#pragma once

#include <algorithm>
#include <concepts>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "machine/TrajectoryPlanner.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/MotionBackend.h"

namespace ngc {
    enum class PreparedDriverState { Running, Completed, Error };

    // Planning owner for the split pipeline. It has no InterpreterSession
    // reference: barrier results travel back through GeometryFeedbackChannel.
    class PreparedTrajectoryExecutionDriver {
        MotionBackend &m_backend;
        PreparedGeometryForwardChannel &m_forward;
        GeometryFeedbackChannel &m_feedback;
        std::atomic<bool> &m_cancelled;
        TrajectoryPlanner m_planner;
        std::unique_ptr<PlannedExecution> m_pending;
        std::size_t m_pendingItem = 0;
        std::optional<PreparedStreamMessage> m_deferredMessage;
        std::optional<std::string> m_error;
        std::size_t m_outstandingChunks = 0;
        bool m_forwardComplete = false;
        bool m_probePending = false;
        std::optional<std::uint64_t> m_probeFence;
        std::optional<SynchronizationFenceId> m_synchronizationFence;
        bool m_waitingForHeld = false;
        GeometryEpoch m_epoch = 0;
        GeometrySequence m_nextSequence = 1;
        RequestId m_nextRequest = 1;
        RequestId m_startRequest = 0;
        bool m_backendReady = false;

        void fail(std::string message) {
            if(m_error) return;
            m_error = std::move(message);
            if(!m_feedback.tryPush(std::make_unique<const GeometryFeedback>(
                    AbortGeometryRun{m_epoch, *m_error}))) m_feedback.notifyAll();
            m_cancelled.store(true, std::memory_order_release);
            m_forward.notifyAll();
            m_feedback.notifyAll();
        }

        template<typename Observe>
        void observePlanned(const PlannedExecution &planned, const std::size_t itemIndex,
                            Observe &&observe) {
            auto activation=std::ranges::lower_bound(
                planned.activations,itemIndex,{},&TimedCommandActivation::chunk);
            for(;activation!=planned.activations.end()
                    &&activation->chunk==itemIndex;++activation) {
                const auto &input=planned.inputs[activation->input];
                const auto &item = planned.items[itemIndex];
                if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &,
                                             const TrajectoryPlanningMetadata &,
                                             const TrajectoryCommandPresentation &, SpanId>)
                    observe(input.command,item,input.metadata,input.presentation,activation->span);
                else if constexpr(std::invocable<Observe, const MachineCommand &, const ExecutionItem &,
                                                  const TrajectoryPlanningMetadata &>)
                    observe(input.command, item, input.metadata);
                else observe(input.command, item);
            }
        }

        static TrajectoryPlannerInput inputFrom(const PreparedCommandRecord &record) {
            return { record.command, record.metadata, record.presentation,
                     record.presentationActivation, record.continuousScaleOverride };
        }

        bool validateSequence(const PreparedStreamMessage &message) {
            const auto [epoch, sequence] = std::visit([](const auto &value) {
                return std::pair { value.epoch, value.sequence };
            }, message);
            if(epoch != m_epoch)
                return (fail(std::format("prepared geometry message has stale epoch {} expected {}",
                    epoch, m_epoch)), false);
            if(sequence != m_nextSequence)
                return (fail(std::format("prepared geometry sequence gap or duplicate: received {} expected {}",
                    sequence, m_nextSequence)), false);
            ++m_nextSequence;
            return true;
        }

        bool publishPlanned(std::unique_ptr<PlannedExecution> planned) {
            if(!planned || planned->items.empty()) {
                fail("prepared trajectory planner produced an empty execution packet batch");
                return false;
            }
            m_pending = std::move(planned);
            m_pendingItem = 0;
            return true;
        }

        bool planWindow(const bool allowTerminalStop = true) {
            auto planned = m_planner.planWindow(allowTerminalStop);
            if(!planned) {
                fail(planned.error());
                return false;
            }
            if(*planned) return publishPlanned(std::move(*planned));
            return true;
        }

        bool appendSlice(const PreparedGeometrySlice &slice) {
            if(!m_planner.enqueuePrepared(slice)) {
                fail("prepared trajectory planner rejected a geometry slice: "
                    +m_planner.lastPreparedEnqueueError());
                return false;
            }
            if(m_planner.shouldPlanRollingPrefix()) return planWindow(false);
            return true;
        }

        bool processMessage(PreparedStreamMessage message,
                            auto &&observeLifecycle, auto &&observeStatus,
                            const bool sequenceAlreadyValidated = false) {
            if(!sequenceAlreadyValidated && !validateSequence(message)) return false;
            return std::visit([&](auto &&value) -> bool {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, PreparedGeometrySlice>) {
                    if(m_planner.windowSize()!=0&&m_planner.preparedChainEnded()) {
                        m_deferredMessage=std::move(message);
                        return planWindow();
                    }
                    if(!appendSlice(value)) {
                        m_deferredMessage = std::move(message);
                        return planWindow();
                    }
                    return true;
                } else if constexpr(std::same_as<T, PreparedStandaloneCommand>) {
                    if(m_planner.windowSize() != 0) {
                        m_deferredMessage = std::move(message);
                        return planWindow();
                    }
                    auto input = inputFrom(value.command);
                    if(!m_planner.enqueue(std::move(input))) {
                        fail("bounded prepared lookahead rejected a standalone command");
                        return false;
                    }
                    return planWindow();
                } else if constexpr(std::same_as<T, PreparedContinuousEnd>) {
                    if(!m_planner.endPreparedChain(value.chain)) {
                        fail("prepared trajectory planner received an end for the wrong geometry chain");
                        return false;
                    }
                    return planWindow();
                } else if constexpr(std::same_as<T, PreparedBlockLifecycleMessage>) {
                    if constexpr(!std::same_as<std::remove_cvref_t<decltype(observeLifecycle)>,
                                               std::nullptr_t>)
                        observeLifecycle(value.lifecycle);
                    return true;
                } else if constexpr(std::same_as<T, PreparedSynchronizationFence>) {
                    if(m_planner.windowSize() != 0 && !planWindow()) return false;
                    m_synchronizationFence = value.fence;
                    return releaseSynchronizationIfHeld();
                } else if constexpr(std::same_as<T, PreparedProbeFence>) {
                    m_probeFence = value.commandId;
                    return true;
                } else if constexpr(std::same_as<T, PreparedStatusMessage>) {
                    if constexpr(!std::same_as<std::remove_cvref_t<decltype(observeStatus)>,
                                               std::nullptr_t>)
                        observeStatus(value.status);
                    return true;
                } else if constexpr(std::same_as<T, PreparedProgramEnd>) {
                    if(m_planner.windowSize() != 0 && !planWindow()) return false;
                    m_forwardComplete = true;
                    return true;
                } else if constexpr(std::same_as<T, PreparedFailure>) {
                    fail(value.error);
                    return false;
                }
            }, std::move(message));
        }

        bool releaseSynchronizationIfHeld() {
            if(!m_synchronizationFence || m_pending || m_outstandingChunks != 0
               || m_planner.windowSize() != 0) return true;
            if(!m_feedback.tryPush(std::make_unique<const GeometryFeedback>(
                    ReleaseSynchronization{m_epoch, *m_synchronizationFence}))) {
                fail("geometry feedback queue is full while releasing synchronization");
                return false;
            }
            m_synchronizationFence.reset();
            return true;
        }

    public:
        PreparedTrajectoryExecutionDriver(MotionBackend &backend,
                                          PreparedGeometryForwardChannel &forward,
                                          GeometryFeedbackChannel &feedback,
                                          std::atomic<bool> &cancelled,
                                          TrajectoryLimits limits = {})
            : m_backend(backend), m_forward(forward), m_feedback(feedback),
              m_cancelled(cancelled), m_planner(limits) { }

        bool begin(const GeometryEpoch epoch = 1, const position_t &position = {}) {
            ExecutionEvent stale;
            while(m_backend.tryTakeEvent(stale)) { }
            m_pending.reset();
            m_pendingItem = 0;
            m_deferredMessage.reset();
            m_error.reset();
            m_outstandingChunks = 0;
            m_forwardComplete = false;
            m_probePending = false;
            m_probeFence.reset();
            m_synchronizationFence.reset();
            m_waitingForHeld = false;
            m_backendReady = false;
            m_epoch = epoch;
            m_nextSequence = 1;
            m_planner.clearDiagnostics();
            m_planner.reset(epoch, position);
            if(m_backend.trySubmit(ResetRequest{m_nextRequest++, epoch}) != SubmitResult::Submitted)
                return false;
            m_startRequest = m_nextRequest++;
            return m_backend.trySubmit(StartRequest{m_startRequest, epoch}) == SubmitResult::Submitted;
        }

        void setLimits(const TrajectoryLimits &limits) { m_planner.setLimits(limits); }
        void setContinuousPlanningEffort(const ContinuousPlanningEffort &effort) {
            m_planner.setContinuousPlanningEffort(effort);
        }
        void setPlanningProgressCallback(std::function<void()> callback) {
            m_planner.setProgressCallback(std::move(callback));
        }

        template<typename Observe, typename ObserveLifecycle = std::nullptr_t,
                 typename ObserveStatus = std::nullptr_t>
        bool pumpOne(Observe &&observe, ObserveLifecycle &&observeLifecycle = nullptr,
                     ObserveStatus &&observeStatus = nullptr) {
            if(m_error || !m_backendReady || m_waitingForHeld) return false;
            if(m_pending) {
                if(m_pendingItem >= m_pending->items.size()) {
                    fail("prepared trajectory driver retained an invalid packet index");
                    return false;
                }
                const auto publication = m_backend.tryPublish(m_pending->items[m_pendingItem]);
                if(publication == PublishResult::Full) return false;
                if(publication != PublishResult::Published) {
                    fail("motion backend rejected a prepared planner-produced item");
                    return false;
                }
                observePlanned(*m_pending, m_pendingItem, std::forward<Observe>(observe));
                auto activation=std::ranges::lower_bound(
                    m_pending->activations,m_pendingItem,{},&TimedCommandActivation::chunk);
                for(;activation!=m_pending->activations.end()
                        &&activation->chunk==m_pendingItem;++activation)
                    if(std::holds_alternative<ProbeMove>(
                            m_pending->inputs[activation->input].command))
                        m_probePending = true;
                ++m_outstandingChunks;
                ++m_pendingItem;
                if(m_pendingItem == m_pending->items.size()) {
                    m_pending.reset();
                    m_pendingItem = 0;
                    (void)releaseSynchronizationIfHeld();
                }
                return true;
            }

            if(m_deferredMessage) {
                auto deferred = std::move(*m_deferredMessage);
                m_deferredMessage.reset();
                if(m_planner.windowSize() != 0) {
                    m_deferredMessage = std::move(deferred);
                    return planWindow();
                }
                if(!processMessage(std::move(deferred), std::forward<ObserveLifecycle>(observeLifecycle),
                                    std::forward<ObserveStatus>(observeStatus), true)) return false;
                return true;
            }

            if(m_planner.shouldPlanImmediately()) return planWindow();

            PreparedForwardMessage message;
            if(!m_forward.tryPop(message)) return false;
            if(!message) {
                fail("prepared geometry forward channel contained a null message");
                return false;
            }
            if(!processMessage(std::move(*message), std::forward<ObserveLifecycle>(observeLifecycle),
                               std::forward<ObserveStatus>(observeStatus))) return false;
            return true;
        }

        void serviceBackend() { serviceBackend([](const ExecutionEvent &) { }); }

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
                        if(!m_feedback.tryPush(std::make_unique<const GeometryFeedback>(
                                DeliverProbeResult{m_epoch, {move->move, status,
                                    move->triggerState.position, move->stoppedState.position}})))
                            fail("geometry feedback queue is full while delivering probe result");
                        m_probePending = false;
                    }
                } else if(const auto *retired = std::get_if<ChunkRetired>(&event)) {
                    if(retired->epoch == m_epoch && m_outstandingChunks > 0) --m_outstandingChunks;
                    (void)releaseSynchronizationIfHeld();
                } else if(const auto *fault = std::get_if<BackendFault>(&event)) {
                    fail("motion backend fault " + std::to_string(fault->code));
                } else if(const auto *rejected = std::get_if<ChunkRejected>(&event)) {
                    if(rejected->epoch == m_epoch)
                        fail("motion backend rejected chunk " + std::to_string(rejected->chunk));
                } else if(const auto *branch = std::get_if<BranchSelected>(&event)) {
                    if(branch->epoch == m_epoch && branch->choice == BranchChoice::Stop)
                        m_waitingForHeld = true;
                } else if(const auto *held = std::get_if<BackendHeld>(&event)) {
                    if(held->epoch == m_epoch) {
                        m_waitingForHeld = false;
                        if(held->reason == BackendHoldReason::FeedHold) continue;
                        (void)releaseSynchronizationIfHeld();
                        if(m_planner.hasRollingContinuation()) {
                            fail("motion stopped on a rolling-horizon packet branch with retained prepared geometry");
                        } else {
                            const auto request = m_nextRequest++;
                            if(m_backend.trySubmit(ResumeRequest{request, m_epoch})
                                    != SubmitResult::Submitted)
                                fail("motion backend control channel is full while resuming after an exact stop");
                        }
                    }
                } else if(const auto *completed = std::get_if<RequestCompleted>(&event)) {
                    if(completed->request == m_startRequest) {
                        m_backendReady = completed->succeeded;
                        if(!completed->succeeded) fail("motion backend rejected start request");
                    }
                }
            }
        }

        PreparedDriverState state() const {
            if(m_error) return PreparedDriverState::Error;
            if(m_forwardComplete && !m_pending && !m_deferredMessage
               &&m_planner.windowSize() == 0 && m_outstandingChunks == 0
               &&!m_probePending && !m_synchronizationFence)
                return PreparedDriverState::Completed;
            return PreparedDriverState::Running;
        }

        const std::optional<std::string> &error() const { return m_error; }
        const TrajectoryPlanningDiagnostics &planningDiagnostics() const { return m_planner.diagnostics(); }
        const std::string &planningActivity() const { return m_planner.planningActivity(); }
        double planningActivitySeconds() const { return m_planner.planningActivitySeconds(); }
        const std::string &lastContinuousPlanSummary() const {
            return m_planner.lastContinuousPlanSummary();
        }
        const std::string &lastContinuousCorrectionHistory() const {
            return m_planner.lastContinuousCorrectionHistory();
        }
        bool hasPendingPublication() const { return m_pending!=nullptr; }
        bool hasUnpublishedRollingContinuation() const {
            return m_planner.hasRollingContinuation();
        }
        std::string activity() const {
            if(m_error) return "error: "+*m_error;
            if(!m_backendReady) return "waiting for backend start acknowledgement";
            if(m_waitingForHeld) return std::format(
                "waiting for backend held event: outstanding={} retained_commands={} "
                "retained_pieces={} retained_nominal={:.3f}s rolling_continuation={}",
                m_outstandingChunks,m_planner.windowSize(),m_planner.preparedPieceCount(),
                m_planner.preparedNominalDuration(),m_planner.hasRollingContinuation());
            if(m_pending) return std::format(
                "publishing planned packet batch: packet={}/{} outstanding={} forward_queue={}",
                m_pendingItem+1,m_pending->items.size(),m_outstandingChunks,m_forward.size());
            if(m_planner.windowSize()!=0) return std::format(
                "retaining prepared work: commands={} pieces={} nominal={:.3f}s "
                "chain_ended={} rolling_ready={} rolling_continuation={} outstanding={} "
                "forward_queue={}",
                m_planner.windowSize(),m_planner.preparedPieceCount(),
                m_planner.preparedNominalDuration(),m_planner.preparedChainEnded(),
                m_planner.shouldPlanRollingPrefix(),m_planner.hasRollingContinuation(),
                m_outstandingChunks,m_forward.size());
            if(m_forwardComplete) return std::format(
                "forward stream complete; outstanding={} probe_pending={} synchronization={}",
                m_outstandingChunks,m_probePending,m_synchronizationFence.has_value());
            return std::format(
                "waiting for prepared stream: forward_queue={} outstanding={} deferred={} "
                "probe_pending={} synchronization={}",
                m_forward.size(),m_outstandingChunks,m_deferredMessage.has_value(),
                m_probePending,m_synchronizationFence.has_value());
        }
    };
}
