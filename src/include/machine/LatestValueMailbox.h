#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ngc {
    // A bounded, wait-free, single-producer/single-consumer latest-value
    // channel. Slot ownership is transferred through one atomic exchange, so
    // the producer never writes the consumer's front slot and the consumer
    // never reads the producer's back slot.
    template<typename T>
    class LatestValueMailbox {
        static_assert(std::is_trivially_copyable_v<T>);
        static_assert(std::atomic<std::uint8_t>::is_always_lock_free);

    public:
        explicit LatestValueMailbox(const T &initial = {}) noexcept
            : m_slots{initial, initial, initial} { }

        LatestValueMailbox(const LatestValueMailbox &) = delete;
        LatestValueMailbox &operator=(const LatestValueMailbox &) = delete;

        // Called only by the single producer.
        void publish(const T &value) noexcept {
            m_slots[m_back] = value;
            const auto previousMiddle = m_middle.exchange(
                static_cast<std::uint8_t>(m_back | DIRTY),
                std::memory_order_acq_rel);
            m_back = slotIndex(previousMiddle);
        }

        // Called only by the single consumer. Returns false when no value has
        // been published since the preceding successful consumeLatest().
        bool consumeLatest(T &value) noexcept {
            auto middle = m_middle.load(std::memory_order_acquire);
            if ((middle & DIRTY) == 0) {
                return false;
            }

            middle = m_middle.exchange(
                static_cast<std::uint8_t>(m_front),
                std::memory_order_acq_rel);
            m_front = slotIndex(middle);
            value = m_slots[m_front];

            return true;
        }

        // Called only by the single consumer.
        [[nodiscard]] const T &current() const noexcept {
            return m_slots[m_front];
        }

    private:
        static constexpr std::uint8_t DIRTY = 0x80;
        static constexpr std::uint8_t INDEX_MASK = 0x03;

        static constexpr std::size_t slotIndex(
                const std::uint8_t value) noexcept {
            return value & INDEX_MASK;
        }

        std::array<T, 3> m_slots;
        std::atomic<std::uint8_t> m_middle{1};
        std::size_t m_front = 0;
        std::size_t m_back = 2;
    };
}
