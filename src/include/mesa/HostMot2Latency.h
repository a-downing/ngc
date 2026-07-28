#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>

#include "mesa/HostMot2Discovery.h"

namespace ngc::mesa {
    struct HostMot2LatencyConfiguration {
        std::size_t sampleCount = 10'000;
        std::chrono::nanoseconds period = std::chrono::milliseconds(1);
    };

    struct HostMot2LatencyDistribution {
        std::size_t sampleCount = 0;
        std::chrono::nanoseconds minimum{};
        std::chrono::nanoseconds mean{};
        std::chrono::nanoseconds percentile50{};
        std::chrono::nanoseconds percentile95{};
        std::chrono::nanoseconds percentile99{};
        std::chrono::nanoseconds maximum{};
    };

    struct HostMot2LatencyResult {
        std::size_t attemptedSamples = 0;
        std::size_t receivedResponses = 0;
        std::size_t readFailures = 0;
        std::size_t contentMismatches = 0;
        std::uint64_t missedPeriods = 0;
        HostMot2LatencyDistribution roundTrip;
        HostMot2LatencyDistribution wakeLateness;
        std::string lastReadError;
    };

    [[nodiscard]] std::expected<HostMot2LatencyResult, std::string>
    measureHostMot2ReadLatency(
        HostMot2RegisterReader &reader,
        const HostMot2LatencyConfiguration &configuration);
}
