#include "machine/RealtimeHost.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <stdexcept>
#include <system_error>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
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
#elif defined(_WIN32)
        constexpr auto WINDOWS_HIGH_RESOLUTION_TIMER = DWORD{0x00000002};
        constexpr auto WINDOWS_DEADLINE_SPIN =
            std::chrono::microseconds{600};
        constexpr std::size_t WINDOWS_STACK_PREFAULT_BYTES = 128 * 1024;

        void prefaultCurrentThreadStack() noexcept {
            std::array<std::byte, WINDOWS_STACK_PREFAULT_BYTES> stack{};
            volatile auto *pages = stack.data();
            constexpr auto pageSize = std::size_t{4096};
            for (auto offset = std::size_t{0};
                 offset < stack.size(); offset += pageSize) {
                pages[offset] = std::byte{0};
            }
        }

        class WindowsDeadlineTimer {
        public:
            WindowsDeadlineTimer() noexcept
                : m_handle(CreateWaitableTimerExW(
                      nullptr, nullptr, WINDOWS_HIGH_RESOLUTION_TIMER,
                      TIMER_MODIFY_STATE | SYNCHRONIZE)) {
                if (m_handle == nullptr) {
                    m_handle = CreateWaitableTimerExW(
                        nullptr, nullptr, 0,
                        TIMER_MODIFY_STATE | SYNCHRONIZE);
                }
            }

            ~WindowsDeadlineTimer() {
                if (m_handle != nullptr) {
                    CloseHandle(m_handle);
                }
            }

            WindowsDeadlineTimer(const WindowsDeadlineTimer &) = delete;
            WindowsDeadlineTimer &operator=(const WindowsDeadlineTimer &) = delete;

            bool waitUntil(const std::chrono::steady_clock::time_point deadline) noexcept {
                if (m_handle == nullptr) {
                    return false;
                }

                const auto timerDeadline =
                    deadline - WINDOWS_DEADLINE_SPIN;
                const auto remaining =
                    timerDeadline - std::chrono::steady_clock::now();
                if (remaining
                    > std::chrono::steady_clock::duration::zero()) {
                    const auto hundredNanoseconds =
                        std::max<std::int64_t>(
                            1, (std::chrono::duration_cast<
                                    std::chrono::nanoseconds>(
                                    remaining).count() + 99) / 100);
                    LARGE_INTEGER due{};
                    due.QuadPart = -hundredNanoseconds;
                    if (!SetWaitableTimerEx(
                            m_handle, &due, 0, nullptr, nullptr,
                            nullptr, 0)
                        || WaitForSingleObject(m_handle, INFINITE)
                            != WAIT_OBJECT_0) {
                        return false;
                    }
                }

                while (std::chrono::steady_clock::now() < deadline) {
                    YieldProcessor();
                }

                return true;
            }

        private:
            HANDLE m_handle = nullptr;
        };

        DWORD_PTR processorMask(const std::uint32_t cpu) noexcept {
            constexpr auto bits = sizeof(DWORD_PTR) * 8;
            if (cpu >= bits) {
                return 0;
            }

            return DWORD_PTR{1} << cpu;
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
#elif defined(_WIN32)
        DWORD_PTR processAffinity = 0;
        DWORD_PTR systemAffinity = 0;
        if (!GetProcessAffinityMask(
                GetCurrentProcess(), &processAffinity,
                &systemAffinity)) {
            return;
        }

        const auto selected = processorMask(cpu);
        const auto housekeeping = processAffinity & ~selected;
        if (selected == 0 || housekeeping == 0) {
            return;
        }
        static_cast<void>(SetThreadAffinityMask(
            GetCurrentThread(), housekeeping));
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
#elif defined(_WIN32)
        // Windows has no mlockall equivalent. This request is best effort and
        // must not prevent the physical or test peer from starting.
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
#elif defined(_WIN32)
        static_cast<void>(priority);

        prefaultCurrentThreadStack();
        const auto affinity = processorMask(cpu);
        if (affinity != 0) {
            static_cast<void>(SetThreadAffinityMask(
                GetCurrentThread(), affinity));
        }
        static_cast<void>(SetThreadPriorityBoost(
            GetCurrentThread(), TRUE));
        static_cast<void>(SetThreadPriority(
            GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL));
        auto powerThrottling = THREAD_POWER_THROTTLING_STATE{
            .Version = THREAD_POWER_THROTTLING_CURRENT_VERSION,
            .ControlMask =
                THREAD_POWER_THROTTLING_EXECUTION_SPEED,
            .StateMask = 0,
        };
        static_cast<void>(SetThreadInformation(
            GetCurrentThread(), ThreadPowerThrottling,
            &powerThrottling, sizeof(powerThrottling)));
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
#elif defined(_WIN32)
        thread_local WindowsDeadlineTimer timer;
        if (!timer.waitUntil(deadline)) {
            std::this_thread::sleep_until(deadline);
        }
#else
        std::this_thread::sleep_until(deadline);
#endif
    }
}
