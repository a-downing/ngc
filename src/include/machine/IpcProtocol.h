#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "machine/MotionBackend.h"
#include "machine/RealtimeTiming.h"

namespace ngc {
    inline constexpr std::uint64_t IPC_MAGIC = 0x4e47435f49504331ULL;
    inline constexpr std::uint32_t IPC_ABI_VERSION = 2;
    inline constexpr std::size_t IPC_EXECUTION_CAPACITY = 8;
    inline constexpr std::size_t IPC_CONTROL_CAPACITY = 16;
    inline constexpr std::size_t IPC_EVENT_CAPACITY = 64;
    inline constexpr std::size_t IPC_SNAPSHOT_CAPACITY = 8;
    inline constexpr std::size_t IPC_REALTIME_TIMING_CAPACITY = 64;

    enum class IpcConnectionState : std::uint32_t {
        Empty,
        FrontendReady,
        PeerReady,
        Running,
        StopRequested,
        PeerStopped,
        Rejected,
        PeerLost,
    };

    enum class IpcRejection : std::uint32_t {
        None,
        InvalidMagic,
        AbiVersion,
        RegionLayout,
        ConfigurationFingerprint,
        TopologyFingerprint,
        SessionGeneration,
        EpochGeneration,
        AuthorityGeneration,
    };

    struct IpcIdentity {
        std::uint64_t configurationFingerprint = 0;
        std::uint64_t topologyFingerprint = 0;
        std::uint64_t sessionGeneration = 0;
        std::uint64_t epochGeneration = 0;
        std::uint64_t authorityGeneration = 0;
    };

    template<typename T, std::size_t Capacity>
    struct IpcRingStorage {
        static_assert(Capacity > 0);
        static_assert(std::is_trivially_copyable_v<T>);

        alignas(64) std::uint32_t head = 0;
        alignas(64) std::uint32_t tail = 0;
        alignas(T) std::array<std::array<std::byte, sizeof(T)>, Capacity + 1> values{};
    };

    struct IpcSharedRegion {
        std::uint64_t magic = 0;
        std::uint32_t abiVersion = 0;
        std::uint32_t regionSize = 0;
        std::uint32_t executionItemSize = 0;
        std::uint32_t controlRequestSize = 0;
        std::uint32_t executionEventSize = 0;
        std::uint32_t executionSnapshotSize = 0;
        std::uint32_t realtimeTimingSummarySize = 0;
        IpcIdentity identity{};
        alignas(64) std::uint32_t connectionState = 0;
        std::uint32_t rejection = 0;
        std::uint32_t frontendProcessId = 0;
        std::uint32_t peerProcessId = 0;
        alignas(64) std::uint64_t frontendHeartbeat = 0;
        alignas(64) std::uint64_t peerHeartbeat = 0;
        IpcRingStorage<ExecutionItem, IPC_EXECUTION_CAPACITY> executionItems;
        IpcRingStorage<ControlRequest, IPC_CONTROL_CAPACITY> controls;
        IpcRingStorage<ExecutionEvent, IPC_EVENT_CAPACITY> events;
        IpcRingStorage<ExecutionSnapshot, IPC_SNAPSHOT_CAPACITY> snapshots;
        IpcRingStorage<
            RealtimeTimingSummary,
            IPC_REALTIME_TIMING_CAPACITY> realtimeTiming;
    };

    static_assert(std::is_standard_layout_v<IpcSharedRegion>);
    static_assert(std::atomic_ref<std::uint32_t>::is_always_lock_free);
    static_assert(std::atomic_ref<std::uint64_t>::is_always_lock_free);

    inline IpcConnectionState ipcConnectionState(IpcSharedRegion &region) noexcept {
        return static_cast<IpcConnectionState>(
            std::atomic_ref(region.connectionState).load(std::memory_order_acquire));
    }

    inline void setIpcConnectionState(IpcSharedRegion &region, const IpcConnectionState state) noexcept {
        std::atomic_ref(region.connectionState).store(
            static_cast<std::uint32_t>(state), std::memory_order_release);
    }

    inline IpcRejection ipcRejection(IpcSharedRegion &region) noexcept {
        return static_cast<IpcRejection>(
            std::atomic_ref(region.rejection).load(std::memory_order_acquire));
    }

    inline void setIpcRejection(IpcSharedRegion &region, const IpcRejection rejection) noexcept {
        std::atomic_ref(region.rejection).store(
            static_cast<std::uint32_t>(rejection), std::memory_order_release);
    }

    template<typename T, std::size_t Capacity>
    bool ipcTryPush(IpcRingStorage<T, Capacity> &ring, const T &value) noexcept {
        auto head = std::atomic_ref(ring.head);
        auto tail = std::atomic_ref(ring.tail);
        const auto current = head.load(std::memory_order_relaxed);
        const auto next = current + 1 == Capacity + 1 ? 0 : current + 1;
        if (next == tail.load(std::memory_order_acquire)) {
            return false;
        }

        std::memcpy(ring.values[current].data(), &value, sizeof(T));
        head.store(next, std::memory_order_release);

        return true;
    }

    template<typename T, std::size_t Capacity>
    bool ipcTryPop(IpcRingStorage<T, Capacity> &ring, T &value) noexcept {
        auto head = std::atomic_ref(ring.head);
        auto tail = std::atomic_ref(ring.tail);
        const auto current = tail.load(std::memory_order_relaxed);
        if (current == head.load(std::memory_order_acquire)) {
            return false;
        }

        std::memcpy(&value, ring.values[current].data(), sizeof(T));
        tail.store(current + 1 == Capacity + 1 ? 0 : current + 1,
                   std::memory_order_release);

        return true;
    }

    inline void initializeIpcSharedRegion(IpcSharedRegion &region, const IpcIdentity identity,
                                          const std::uint32_t frontendProcessId) noexcept {
        std::memset(&region, 0, sizeof(region));
        region.magic = IPC_MAGIC;
        region.abiVersion = IPC_ABI_VERSION;
        region.regionSize = sizeof(IpcSharedRegion);
        region.executionItemSize = sizeof(ExecutionItem);
        region.controlRequestSize = sizeof(ControlRequest);
        region.executionEventSize = sizeof(ExecutionEvent);
        region.executionSnapshotSize = sizeof(ExecutionSnapshot);
        region.realtimeTimingSummarySize =
            sizeof(RealtimeTimingSummary);
        region.identity = identity;
        region.frontendProcessId = frontendProcessId;
        setIpcConnectionState(region, IpcConnectionState::FrontendReady);
    }

    inline IpcRejection validateIpcSharedRegion(
        const IpcSharedRegion &region, const IpcIdentity &expected) noexcept {
        if (region.magic != IPC_MAGIC) {
            return IpcRejection::InvalidMagic;
        }
        if (region.abiVersion != IPC_ABI_VERSION) {
            return IpcRejection::AbiVersion;
        }
        if (region.regionSize != sizeof(IpcSharedRegion)
            || region.executionItemSize != sizeof(ExecutionItem)
            || region.controlRequestSize != sizeof(ControlRequest)
            || region.executionEventSize != sizeof(ExecutionEvent)
            || region.executionSnapshotSize != sizeof(ExecutionSnapshot)
            || region.realtimeTimingSummarySize
                != sizeof(RealtimeTimingSummary)) {
            return IpcRejection::RegionLayout;
        }
        if (region.identity.configurationFingerprint != expected.configurationFingerprint) {
            return IpcRejection::ConfigurationFingerprint;
        }
        if (region.identity.topologyFingerprint != expected.topologyFingerprint) {
            return IpcRejection::TopologyFingerprint;
        }
        if (region.identity.sessionGeneration != expected.sessionGeneration) {
            return IpcRejection::SessionGeneration;
        }
        if (region.identity.epochGeneration != expected.epochGeneration) {
            return IpcRejection::EpochGeneration;
        }
        if (region.identity.authorityGeneration != expected.authorityGeneration) {
            return IpcRejection::AuthorityGeneration;
        }

        return IpcRejection::None;
    }
}
