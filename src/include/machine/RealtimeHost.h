#pragma once

#include <chrono>
#include <cstdint>

namespace ngc {
    void excludeCpuFromCurrentThread(std::uint32_t cpu);
    void lockProcessMemory();
    void configureCurrentRealtimeThread(std::uint32_t cpu, int priority);
    void sleepUntilMonotonic(
        std::chrono::steady_clock::time_point deadline) noexcept;
}
