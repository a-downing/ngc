#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ngc {
    enum class EmergencyStopSource : std::uint32_t {
        Gui = 1U << 0,
        Pendant = 1U << 1,
        PhysicalExternalEnable = 1U << 2,
    };

    enum class EmergencyStopResetResult : std::uint32_t {
        None,
        Cleared,
        BlockedByActiveSource,
        NotLatched,
    };

    constexpr std::uint32_t emergencyStopSourceMask(
        const EmergencyStopSource source) noexcept {
        return static_cast<std::uint32_t>(source);
    }

    inline constexpr std::uint32_t EMERGENCY_STOP_FAULT = 0x4553'0001;

    struct EmergencyStopControlBlock {
        alignas(64) std::uint32_t requestedSources = 0;
        std::uint32_t requestReserved = 0;
        std::uint64_t resetGeneration = 0;
        std::array<std::byte, 48> requestPadding{};

        alignas(64) std::uint64_t statusSequence = 0;
        std::uint64_t acknowledgedResetGeneration = 0;
        std::uint32_t activeSources = 0;
        std::uint32_t latchedSources = 0;
        std::uint32_t resetResult = 0;
        std::uint32_t statusReserved = 0;
        std::array<std::byte, 32> statusPadding{};
    };
    static_assert(std::is_standard_layout_v<EmergencyStopControlBlock>);
    static_assert(std::is_trivially_copyable_v<EmergencyStopControlBlock>);
    static_assert(sizeof(EmergencyStopControlBlock) == 128);
    static_assert(std::atomic_ref<std::uint32_t>::is_always_lock_free);
    static_assert(std::atomic_ref<std::uint64_t>::is_always_lock_free);

    struct EmergencyStopStatus {
        std::uint64_t acknowledgedResetGeneration = 0;
        std::uint32_t activeSources = 0;
        std::uint32_t latchedSources = 0;
        EmergencyStopResetResult resetResult = EmergencyStopResetResult::None;
    };

    class EmergencyStopInterface {
    public:
        explicit EmergencyStopInterface(EmergencyStopControlBlock &control) noexcept
            : m_control(control) { }

        void request(EmergencyStopSource source) noexcept;
        void release(EmergencyStopSource source) noexcept;
        [[nodiscard]] std::uint64_t requestReset() noexcept;
        [[nodiscard]] std::uint32_t requestedSources() const noexcept;
        [[nodiscard]] std::uint64_t resetGeneration() const noexcept;
        [[nodiscard]] EmergencyStopStatus status() const noexcept;
        void publish(const EmergencyStopStatus &status) noexcept;

    private:
        EmergencyStopControlBlock &m_control;
    };

    class EmergencyStopState {
    public:
        explicit EmergencyStopState(EmergencyStopControlBlock &control) noexcept
            : m_interface(control) { }

        [[nodiscard]] bool update(std::uint32_t localActiveSources = 0) noexcept;
        [[nodiscard]] bool latched() const noexcept;
        [[nodiscard]] EmergencyStopStatus status() const noexcept;

    private:
        EmergencyStopInterface m_interface;
        EmergencyStopStatus m_status;
        std::uint64_t m_observedResetGeneration = 0;
    };
}
