#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ngc {
    // Exactly one thread may call tryPush and exactly one other thread may call
    // tryPop. Capacity is the usable capacity (the storage has one sentinel slot).
    template<typename T, std::size_t Capacity>
    class SpscChannel {
        static_assert(Capacity > 0);
        static_assert(std::is_trivially_copyable_v<T>);
        static constexpr std::size_t STORAGE_SIZE = Capacity + 1;

        alignas(64) std::array<T, STORAGE_SIZE> m_values{};
        alignas(64) std::atomic<std::size_t> m_head{0};
        alignas(64) std::atomic<std::size_t> m_tail{0};

        static constexpr std::size_t next(const std::size_t value) noexcept {
            return value + 1 == STORAGE_SIZE ? 0 : value + 1;
        }

    public:
        bool tryPush(const T &value) noexcept {
            const auto head = m_head.load(std::memory_order_relaxed);
            const auto nextHead = next(head);
            if(nextHead == m_tail.load(std::memory_order_acquire)) return false;
            m_values[head] = value;
            m_head.store(nextHead, std::memory_order_release);
            return true;
        }

        bool tryPop(T &value) noexcept {
            const auto tail = m_tail.load(std::memory_order_relaxed);
            if(tail == m_head.load(std::memory_order_acquire)) return false;
            value = m_values[tail];
            m_tail.store(next(tail), std::memory_order_release);
            return true;
        }

        bool empty() const noexcept {
            return m_tail.load(std::memory_order_relaxed) == m_head.load(std::memory_order_acquire);
        }
    };

}
