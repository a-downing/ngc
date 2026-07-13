#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

class WindowsServoPacer {
public:
    struct WaitResult {
        double latenessSeconds = 0.0;
        std::uint64_t missedPeriods = 0;
    };

private:
    double m_periodSeconds;
#ifdef _WIN32
    HANDLE m_timer = nullptr;
    LONGLONG m_frequency = 0;
    LONGLONG m_periodTicks = 0;
    LONGLONG m_nextDeadline = 0;
    int m_previousPriority = THREAD_PRIORITY_ERROR_RETURN;
    DWORD m_error = ERROR_SUCCESS;
#else
    std::chrono::steady_clock::time_point m_nextDeadline;
#endif

public:
    explicit WindowsServoPacer(const double periodSeconds) : m_periodSeconds(periodSeconds) {
#ifdef _WIN32
        LARGE_INTEGER frequency;
        if(!QueryPerformanceFrequency(&frequency)) {
            m_error = GetLastError();
            return;
        }
        m_frequency = frequency.QuadPart;
        m_periodTicks = std::max<LONGLONG>(1, static_cast<LONGLONG>(m_periodSeconds*m_frequency + 0.5));
        m_timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                         TIMER_MODIFY_STATE | SYNCHRONIZE);
        if(!m_timer) {
            m_error = GetLastError();
            return;
        }
        m_previousPriority = GetThreadPriority(GetCurrentThread());
        if(!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST)) m_error = GetLastError();
#endif
        reset();
    }

    ~WindowsServoPacer() {
#ifdef _WIN32
        if(m_previousPriority != THREAD_PRIORITY_ERROR_RETURN)
            (void)SetThreadPriority(GetCurrentThread(), m_previousPriority);
        if(m_timer) CloseHandle(m_timer);
#endif
    }

    WindowsServoPacer(const WindowsServoPacer &) = delete;
    WindowsServoPacer &operator=(const WindowsServoPacer &) = delete;

    bool valid() const {
#ifdef _WIN32
        return m_timer != nullptr && m_frequency > 0 && m_error == ERROR_SUCCESS;
#else
        return true;
#endif
    }

    std::uint32_t errorCode() const {
#ifdef _WIN32
        return m_error;
#else
        return 0;
#endif
    }

    void reset() {
#ifdef _WIN32
        if(!m_timer || m_frequency == 0) return;
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        m_nextDeadline = now.QuadPart + m_periodTicks;
#else
        m_nextDeadline = std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(m_periodSeconds));
#endif
    }

    bool wait(WaitResult &result) {
#ifdef _WIN32
        if(!valid()) return false;
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        const auto remainingTicks = m_nextDeadline - now.QuadPart;
        if(remainingTicks > 0) {
            LARGE_INTEGER due;
            due.QuadPart = -std::max<LONGLONG>(1,
                (remainingTicks*10'000'000 + m_frequency - 1) / m_frequency);
            if(!SetWaitableTimerEx(m_timer, &due, 0, nullptr, nullptr, nullptr, 0)) {
                m_error = GetLastError();
                return false;
            }
            if(WaitForSingleObject(m_timer, INFINITE) != WAIT_OBJECT_0) {
                m_error = GetLastError();
                return false;
            }
        }
        QueryPerformanceCounter(&now);
        const auto lateTicks = std::max<LONGLONG>(0, now.QuadPart - m_nextDeadline);
        result.latenessSeconds = static_cast<double>(lateTicks) / static_cast<double>(m_frequency);
        result.missedPeriods = static_cast<std::uint64_t>(lateTicks / m_periodTicks);
        m_nextDeadline += static_cast<LONGLONG>(result.missedPeriods + 1) * m_periodTicks;
        return true;
#else
        std::this_thread::sleep_until(m_nextDeadline);
        const auto now = std::chrono::steady_clock::now();
        const auto lateness = std::max(now - m_nextDeadline, std::chrono::steady_clock::duration::zero());
        result.latenessSeconds = std::chrono::duration<double>(lateness).count();
        result.missedPeriods = static_cast<std::uint64_t>(result.latenessSeconds / m_periodSeconds);
        m_nextDeadline += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(m_periodSeconds * static_cast<double>(result.missedPeriods + 1)));
        return true;
#endif
    }
};
