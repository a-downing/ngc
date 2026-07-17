#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>

namespace ngc {
    // NRT-to-NRT SPSC queue. Capacity is the number of usable entries; unlike
    // the RT SpscChannel this queue owns the lifetime of its values and can
    // therefore transport move-only prepared geometry messages.
    template<typename T, std::size_t Capacity>
    class OwningSpscChannel {
        static_assert(Capacity > 0);

        std::array<std::optional<T>, Capacity> m_values;
        alignas(64) std::atomic<std::size_t> m_head{0};
        alignas(64) std::atomic<std::size_t> m_tail{0};
        mutable std::mutex m_waitMutex;
        std::condition_variable m_notEmpty;
        std::condition_variable m_notFull;
        std::atomic<bool> m_producerWaiting{false};
        std::atomic<bool> m_consumerWaiting{false};

        void notifyConsumer() {
            if(!m_consumerWaiting.load(std::memory_order_acquire)) return;
            // Pair waiter registration and notification with the same mutex.
            // This closes the gap between the wait predicate and actually
            // sleeping without putting a mutex on the normal SPSC fast path.
            std::lock_guard lock(m_waitMutex);
            m_notEmpty.notify_one();
        }

        void notifyProducer() {
            if(!m_producerWaiting.load(std::memory_order_acquire)) return;
            std::lock_guard lock(m_waitMutex);
            m_notFull.notify_one();
        }

    public:
        OwningSpscChannel() = default;
        OwningSpscChannel(const OwningSpscChannel &) = delete;
        OwningSpscChannel &operator=(const OwningSpscChannel &) = delete;

        ~OwningSpscChannel() = default;

        bool tryPush(T &&value) {
            const auto head = m_head.load(std::memory_order_relaxed);
            const auto tail = m_tail.load(std::memory_order_acquire);
            if(head - tail >= Capacity) return false;
            m_values[head % Capacity].emplace(std::move(value));
            m_head.store(head + 1, std::memory_order_release);
            notifyConsumer();
            return true;
        }

        bool tryPop(T &value) {
            const auto tail = m_tail.load(std::memory_order_relaxed);
            const auto head = m_head.load(std::memory_order_acquire);
            if(tail == head) return false;
            auto &slot = m_values[tail % Capacity];
            value = std::move(*slot);
            slot.reset();
            m_tail.store(tail + 1, std::memory_order_release);
            notifyProducer();
            return true;
        }

        bool empty() const noexcept {
            return m_tail.load(std::memory_order_relaxed)
                == m_head.load(std::memory_order_acquire);
        }

        bool full() const noexcept {
            return m_head.load(std::memory_order_relaxed)
                - m_tail.load(std::memory_order_acquire) >= Capacity;
        }

        std::size_t size() const noexcept {
            return m_head.load(std::memory_order_acquire)
                - m_tail.load(std::memory_order_acquire);
        }

        static constexpr std::size_t capacity() noexcept { return Capacity; }

        // These helpers are for NRT participants that should sleep rather than
        // spin. Cancellation is supplied by the owner of the run and is part
        // of every wait predicate, so shutdown cannot strand a participant.
        template<typename Cancelled>
        bool waitPush(T value, Cancelled &&cancelled) {
            while(!tryPush(std::move(value))) {
                std::unique_lock lock(m_waitMutex);
                m_producerWaiting.store(true, std::memory_order_release);
                m_notFull.wait(lock, [&] { return !full() || cancelled(); });
                m_producerWaiting.store(false, std::memory_order_release);
                if(cancelled()) return false;
            }
            return true;
        }

        template<typename Cancelled>
        bool waitPop(T &value, Cancelled &&cancelled) {
            while(!tryPop(value)) {
                std::unique_lock lock(m_waitMutex);
                m_consumerWaiting.store(true, std::memory_order_release);
                m_notEmpty.wait(lock, [&] { return !empty() || cancelled(); });
                m_consumerWaiting.store(false, std::memory_order_release);
                if(cancelled()) return false;
            }
            return true;
        }

        void notifyAll() noexcept {
            // Cancellation is changed by the owner before this call. Taking
            // the wait mutex ensures a waiter either observes cancellation in
            // its predicate or is already asleep when notified.
            std::lock_guard lock(m_waitMutex);
            m_notEmpty.notify_all();
            m_notFull.notify_all();
        }
    };
}
