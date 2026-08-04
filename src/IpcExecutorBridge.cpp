#include "machine/IpcExecutorBridge.h"

#include <variant>

#include "ExecutionItemOperations.h"

namespace ngc {
    IpcExecutorBridge::IpcExecutorBridge(
        IpcSharedRegion &region,
        BackendRuntime &runtime,
        MotionBackend &backend,
        IpcExecutorPolicy *policy) noexcept
        : m_region(region),
          m_runtime(runtime),
          m_backend(backend),
          m_policy(policy) { }

    bool IpcExecutorBridge::service(const bool consume) noexcept {
        auto progressed = publishOutputs();
        if (consume) {
            progressed = submitInputs() || progressed;
        }

        return progressed;
    }

    std::uint64_t IpcExecutorBridge::completedControls() const noexcept {
        return m_completedControls;
    }

    bool IpcExecutorBridge::publishOutputs() noexcept {
        auto progressed = false;
        if (!m_pendingEvent.has_value()) {
            ExecutionEvent event;
            if (m_backend.tryTakeEvent(event)) {
                if (m_policy != nullptr) {
                    m_policy->observeEvent(event);
                }
                m_pendingEvent = event;
                progressed = true;
            }
        }
        if (m_pendingEvent.has_value()
            && ipcTryPush(m_region.events, *m_pendingEvent)) {
            if (std::holds_alternative<RequestCompleted>(
                    *m_pendingEvent)) {
                ++m_completedControls;
            }
            m_pendingEvent.reset();
            progressed = true;
        }

        if (!m_pendingSnapshot.has_value()) {
            ExecutionSnapshot snapshot;
            if (m_backend.tryTakeSnapshot(snapshot)) {
                if (m_policy != nullptr) {
                    m_policy->observeSnapshot(snapshot);
                }
                m_latestSnapshot = snapshot;
                m_pendingSnapshot = snapshot;
                progressed = true;
            }
        }
        if (m_pendingSnapshot.has_value()
            && ipcTryPush(m_region.snapshots, *m_pendingSnapshot)) {
            m_pendingSnapshot.reset();
            progressed = true;
        }

        if (!m_pendingTiming.has_value()) {
            RealtimeTimingSummary timing;
            if (m_runtime.tryTakeRealtimeTiming(timing)) {
                m_pendingTiming = timing;
                progressed = true;
            }
        }
        if (m_pendingTiming.has_value()
            && ipcTryPush(m_region.realtimeTiming, *m_pendingTiming)) {
            m_pendingTiming.reset();
            progressed = true;
        }

        return progressed;
    }

    bool IpcExecutorBridge::submitInputs() noexcept {
        // This bridge loop is the executor's sole ordinary-ingress producer.
        // Keep both publication and control submission on this thread so their
        // order is preserved by ProductionExecutorCore's shared SPSC queue.
        auto progressed = false;
        if (!m_pendingItem.has_value()) {
            ExecutionItem item;
            if (ipcTryPop(m_region.executionItems, item)) {
                m_pendingItem = item;
                progressed = true;
            }
        }
        if (m_pendingItem.has_value()) {
            if (!m_pendingItemPrepared) {
                if (m_policy != nullptr) {
                    m_policy->prepareExecutionItem(
                        *m_pendingItem, m_latestSnapshot);
                }
                m_pendingItemPrepared = true;
            }
            const auto result = m_backend.tryPublish(*m_pendingItem);
            if (result == PublishResult::Published) {
                if (m_policy != nullptr) {
                    m_policy->executionItemPublished();
                }
                m_pendingItem.reset();
                m_pendingItemPrepared = false;
                progressed = true;
            } else if (result == PublishResult::Invalid
                       && !m_pendingEvent.has_value()) {
                if (m_policy != nullptr) {
                    m_policy->executionItemRejected();
                }
                m_pendingEvent = ChunkRejected{
                    execution_item::epoch(*m_pendingItem),
                    execution_item::id(*m_pendingItem),
                };
                m_pendingItem.reset();
                m_pendingItemPrepared = false;
                progressed = true;
            }
        }

        if (!m_pendingControl.has_value()) {
            ControlRequest request;
            if (ipcTryPop(m_region.controls, request)) {
                m_pendingControl = request;
                progressed = true;
            }
        }
        if (m_pendingControl.has_value()
            && m_backend.trySubmit(*m_pendingControl)
                == SubmitResult::Submitted) {
            m_pendingControl.reset();
            progressed = true;
        }

        return progressed;
    }

    ExecutionSnapshot stopExecutorAfterFrontendLoss(BackendRuntime &runtime) {
        constexpr RequestId internalRequest = 0;
        auto &backend = runtime.endpoint();
        auto snapshot = ExecutionSnapshot{};
        auto stopPending = false;
        auto snapshotSeen = false;
        auto stopResult = std::optional<bool>{};
        const auto service = [&] {
            runtime.serviceImmediate();
            runtime.waitForServiceMotion();
        };

        ExecutionEvent staleEvent;
        while (backend.tryTakeEvent(staleEvent)) { }
        while (backend.tryTakeSnapshot(snapshot)) { }

        for (;;) {
            auto completedThisIteration = false;
            ExecutionEvent event;
            while (backend.tryTakeEvent(event)) {
                const auto *completed =
                    std::get_if<RequestCompleted>(&event);
                if (completed != nullptr
                    && completed->request == internalRequest) {
                    stopPending = false;
                    stopResult = completed->succeeded;
                    completedThisIteration = true;
                }
            }
            while (backend.tryTakeSnapshot(snapshot)) {
                snapshotSeen = true;
            }

            if (completedThisIteration) {
                snapshotSeen = false;
                service();
                continue;
            }
            if (stopResult.has_value() && snapshotSeen
                && (snapshot.state == BackendState::Held
                    || snapshot.state == BackendState::Faulted
                    || snapshot.state == BackendState::Disabled)) {
                break;
            }
            if (stopResult.has_value() && !*stopResult) {
                stopResult.reset();
            }
            if (!stopPending
                && !stopResult.has_value()
                && backend.trySubmit(
                    ControlledStopRequest{internalRequest})
                    == SubmitResult::Submitted) {
                stopPending = true;
            }

            service();
        }

        auto disablePending = false;
        while (snapshot.state != BackendState::Disabled) {
            ExecutionEvent event;
            while (backend.tryTakeEvent(event)) { }
            while (backend.tryTakeSnapshot(snapshot)) { }
            if (snapshot.state == BackendState::Disabled) {
                break;
            }
            if (!disablePending
                && backend.trySubmit(DisableRequest{internalRequest})
                    == SubmitResult::Submitted) {
                disablePending = true;
            }

            service();
        }

        // Disable is staged by applyOutputs() after its servo tick. One more
        // tick transmits that safe image to cyclic hardware before shutdown.
        service();

        return snapshot;
    }
}
