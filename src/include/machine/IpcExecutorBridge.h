#pragma once

#include <cstdint>
#include <optional>

#include "machine/BackendRuntime.h"
#include "machine/IpcProtocol.h"

namespace ngc {
    class IpcExecutorPolicy {
    public:
        virtual ~IpcExecutorPolicy() = default;

        virtual void prepareExecutionItem(
            ExecutionItem &, const ExecutionSnapshot &) noexcept { }
        virtual void executionItemPublished() noexcept { }
        virtual void executionItemRejected() noexcept { }
        virtual void observeEvent(const ExecutionEvent &) noexcept { }
        virtual void observeSnapshot(
            const ExecutionSnapshot &) noexcept { }
    };

    class IpcExecutorBridge {
    public:
        IpcExecutorBridge(
            IpcSharedRegion &region,
            BackendRuntime &runtime,
            MotionBackend &backend,
            IpcExecutorPolicy *policy = nullptr) noexcept;

        bool service(bool consume) noexcept;
        [[nodiscard]] std::uint64_t completedControls() const noexcept;

    private:
        bool publishOutputs() noexcept;
        bool submitInputs() noexcept;

        IpcSharedRegion &m_region;
        BackendRuntime &m_runtime;
        MotionBackend &m_backend;
        IpcExecutorPolicy *m_policy = nullptr;
        ExecutionSnapshot m_latestSnapshot;
        std::optional<ExecutionItem> m_pendingItem;
        std::optional<ControlRequest> m_pendingControl;
        std::optional<ExecutionEvent> m_pendingEvent;
        std::optional<ExecutionSnapshot> m_pendingSnapshot;
        std::optional<RealtimeTimingSummary> m_pendingTiming;
        std::uint64_t m_completedControls = 0;
        bool m_pendingItemPrepared = false;
    };
}
