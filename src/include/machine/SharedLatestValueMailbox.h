#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ngc {
    template<typename T>
    struct SharedLatestValueMailboxStorage {
        static_assert(std::is_trivially_copyable_v<T>);

        std::array<T, 3> slots{};
        alignas(64) std::uint32_t middle = 1;
    };

    template<typename T>
    void initializeSharedLatestValueMailbox(
            SharedLatestValueMailboxStorage<T> &storage) noexcept {
        storage.slots = {};
        storage.middle = 1;
    }

    template<typename T>
    class SharedLatestValueProducer {
    public:
        explicit SharedLatestValueProducer(
            SharedLatestValueMailboxStorage<T> &storage) noexcept
            : m_storage(storage) { }

        void publish(const T &value) noexcept {
            m_storage.slots[m_back] = value;
            auto middle = std::atomic_ref(m_storage.middle);
            const auto previousMiddle = middle.exchange(
                static_cast<std::uint32_t>(m_back | DIRTY),
                std::memory_order_acq_rel);
            m_back = slotIndex(previousMiddle);
        }

    private:
        static constexpr std::uint32_t DIRTY = 0x8000'0000;
        static constexpr std::uint32_t INDEX_MASK = 0x03;

        static constexpr std::size_t slotIndex(
                const std::uint32_t value) noexcept {
            return value & INDEX_MASK;
        }

        SharedLatestValueMailboxStorage<T> &m_storage;
        std::size_t m_back = 2;
    };

    template<typename T>
    class SharedLatestValueConsumer {
    public:
        explicit SharedLatestValueConsumer(
            SharedLatestValueMailboxStorage<T> &storage) noexcept
            : m_storage(storage) { }

        bool consumeLatest(T &value) noexcept {
            auto middle = std::atomic_ref(m_storage.middle);
            auto published = middle.load(std::memory_order_acquire);
            if ((published & DIRTY) == 0) {
                return false;
            }

            published = middle.exchange(
                static_cast<std::uint32_t>(m_front),
                std::memory_order_acq_rel);
            m_front = slotIndex(published);
            value = m_storage.slots[m_front];

            return true;
        }

    private:
        static constexpr std::uint32_t DIRTY = 0x8000'0000;
        static constexpr std::uint32_t INDEX_MASK = 0x03;

        static constexpr std::size_t slotIndex(
                const std::uint32_t value) noexcept {
            return value & INDEX_MASK;
        }

        SharedLatestValueMailboxStorage<T> &m_storage;
        std::size_t m_front = 0;
    };

    static_assert(std::atomic_ref<std::uint32_t>::is_always_lock_free);
}
