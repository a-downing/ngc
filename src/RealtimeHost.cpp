#include "machine/RealtimeHost.h"

#include <array>
#include <cerrno>
#include <cstddef>
#include <stdexcept>
#include <system_error>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#endif

namespace ngc {
    namespace {
#ifdef __linux__
        constexpr std::size_t REALTIME_STACK_PREFAULT_BYTES = 128 * 1024;

        void validateCpu(const std::uint32_t cpu) {
            if (cpu >= CPU_SETSIZE) {
                throw std::runtime_error(
                    "real-time CPU exceeds CPU_SETSIZE");
            }
        }

        void prefaultCurrentThreadStack() noexcept {
            std::array<std::byte, REALTIME_STACK_PREFAULT_BYTES> stack{};
            volatile auto *pages = stack.data();
            constexpr auto pageSize = std::size_t{4096};
            for (auto offset = std::size_t{0};
                 offset < stack.size(); offset += pageSize) {
                pages[offset] = std::byte{0};
            }
        }
#endif
    }

    void excludeCpuFromCurrentThread(const std::uint32_t cpu) {
#ifdef __linux__
        validateCpu(cpu);

        cpu_set_t affinity;
        CPU_ZERO(&affinity);
        if (const auto error = pthread_getaffinity_np(
                pthread_self(), sizeof(affinity), &affinity);
            error != 0) {
            throw std::system_error(
                error, std::generic_category(),
                "failed to read host thread affinity");
        }

        CPU_CLR(cpu, &affinity);
        auto hasHousekeepingCpu = false;
        for (auto candidate = 0;
             candidate < CPU_SETSIZE; ++candidate) {
            hasHousekeepingCpu =
                hasHousekeepingCpu
                || CPU_ISSET(candidate, &affinity);
        }
        if (!hasHousekeepingCpu) {
            throw std::runtime_error(
                "host thread has no housekeeping CPU after isolation");
        }
        if (const auto error = pthread_setaffinity_np(
                pthread_self(), sizeof(affinity), &affinity);
            error != 0) {
            throw std::system_error(
                error, std::generic_category(),
                "failed to isolate host thread from the real-time CPU");
        }
#else
        static_cast<void>(cpu);

        throw std::runtime_error(
            "real-time CPU isolation is unavailable on this platform");
#endif
    }

    void lockProcessMemory() {
#ifdef __linux__
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            throw std::system_error(
                errno, std::generic_category(),
                "failed to lock process memory");
        }
#else
        throw std::runtime_error(
            "process memory locking is unavailable on this platform");
#endif
    }

    void configureCurrentRealtimeThread(
        const std::uint32_t cpu,
        const int priority) {
#ifdef __linux__
        validateCpu(cpu);
        if (priority < 1 || priority > 99) {
            throw std::invalid_argument(
                "real-time priority must be between 1 and 99");
        }

        cpu_set_t affinity;
        CPU_ZERO(&affinity);
        CPU_SET(cpu, &affinity);
        if (const auto error = pthread_setaffinity_np(
                pthread_self(), sizeof(affinity), &affinity);
            error != 0) {
            throw std::system_error(
                error, std::generic_category(),
                "failed to set real-time thread CPU affinity");
        }

        prefaultCurrentThreadStack();

        sched_param parameters{};
        parameters.sched_priority = priority;
        if (const auto error = pthread_setschedparam(
                pthread_self(), SCHED_FIFO, &parameters);
            error != 0) {
            throw std::system_error(
                error, std::generic_category(),
                "failed to set real-time thread SCHED_FIFO policy");
        }
#else
        static_cast<void>(cpu);
        static_cast<void>(priority);

        throw std::runtime_error(
            "real-time thread hosting is unavailable on this platform");
#endif
    }

    void sleepUntilMonotonic(
        const std::chrono::steady_clock::time_point deadline) noexcept {
#ifdef __linux__
        const auto nanoseconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                deadline.time_since_epoch()).count();
        const timespec target{
            .tv_sec = static_cast<time_t>(
                nanoseconds / 1'000'000'000),
            .tv_nsec = static_cast<long>(
                nanoseconds % 1'000'000'000),
        };

        while (clock_nanosleep(
                   CLOCK_MONOTONIC, TIMER_ABSTIME,
                   &target, nullptr)
               == EINTR) { }
#else
        std::this_thread::sleep_until(deadline);
#endif
    }
}
