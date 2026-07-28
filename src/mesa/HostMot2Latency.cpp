#include "mesa/HostMot2Latency.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

namespace ngc::mesa {
    namespace {
        using Clock = std::chrono::steady_clock;
        using Nanoseconds = std::chrono::nanoseconds;

        constexpr std::uint32_t HOSTMOT2_COOKIE_ADDRESS = 0x0100;
        constexpr std::uint32_t HOSTMOT2_COOKIE = 0x55AA'CAFE;

        std::uint32_t decodeLittleEndian32(
            const std::array<std::byte, 4> &bytes) noexcept {
            return std::to_integer<std::uint32_t>(bytes[0])
                | (std::to_integer<std::uint32_t>(bytes[1]) << 8)
                | (std::to_integer<std::uint32_t>(bytes[2]) << 16)
                | (std::to_integer<std::uint32_t>(bytes[3]) << 24);
        }

        Nanoseconds percentile(
            const std::vector<Nanoseconds> &sorted,
            const std::size_t numerator) {
            const auto rank =
                sorted.size() / 100 * numerator
                + (sorted.size() % 100 * numerator + 99) / 100;

            return sorted[std::max<std::size_t>(rank, 1) - 1];
        }

        HostMot2LatencyDistribution summarize(
            std::vector<Nanoseconds> samples) {
            HostMot2LatencyDistribution result{
                .sampleCount = samples.size(),
            };
            if (samples.empty()) {
                return result;
            }

            std::ranges::sort(samples);
            std::chrono::duration<long double, std::nano> total{};
            for (const auto sample : samples) {
                total += sample;
            }
            result.minimum = samples.front();
            result.mean = std::chrono::duration_cast<Nanoseconds>(
                total / static_cast<long double>(samples.size()));
            result.percentile50 = percentile(samples, 50);
            result.percentile95 = percentile(samples, 95);
            result.percentile99 = percentile(samples, 99);
            result.maximum = samples.back();

            return result;
        }
    }

    std::expected<HostMot2LatencyResult, std::string>
    measureHostMot2ReadLatency(
        HostMot2RegisterReader &reader,
        const HostMot2LatencyConfiguration &configuration) {
        if (configuration.sampleCount == 0) {
            return std::unexpected(
                "HostMot2 latency sample count must be positive");
        }
        if (configuration.period <= Nanoseconds::zero()) {
            return std::unexpected(
                "HostMot2 latency period must be positive");
        }
        if (configuration.sampleCount
            > static_cast<std::size_t>(
                std::numeric_limits<std::int64_t>::max())) {
            return std::unexpected(
                "HostMot2 latency sample count is too large");
        }

        HostMot2LatencyResult result;
        std::vector<Nanoseconds> roundTrips;
        std::vector<Nanoseconds> wakeLateness;
        roundTrips.reserve(configuration.sampleCount);
        wakeLateness.reserve(configuration.sampleCount);
        auto nextStart = Clock::now();

        for (std::size_t sample = 0;
             sample < configuration.sampleCount; ++sample) {
            std::this_thread::sleep_until(nextStart);
            const auto start = Clock::now();
            wakeLateness.push_back(std::chrono::duration_cast<Nanoseconds>(
                std::max(start - nextStart, Clock::duration::zero())));

            std::array<std::byte, 4> cookieBytes{};
            const auto read = reader.read(
                HOSTMOT2_COOKIE_ADDRESS, cookieBytes);
            const auto end = Clock::now();
            ++result.attemptedSamples;
            if (!read) {
                ++result.readFailures;
                result.lastReadError = read.error();
            } else {
                ++result.receivedResponses;
                roundTrips.push_back(
                    std::chrono::duration_cast<Nanoseconds>(end - start));
                if (decodeLittleEndian32(cookieBytes)
                    != HOSTMOT2_COOKIE) {
                    ++result.contentMismatches;
                }
            }

            nextStart += configuration.period;
            if (end > nextStart) {
                const auto overdue =
                    std::chrono::duration_cast<Nanoseconds>(
                        end - nextStart);
                const auto skipped =
                    static_cast<std::uint64_t>(
                        overdue.count() / configuration.period.count())
                    + 1;
                result.missedPeriods += skipped;
                nextStart += configuration.period * skipped;
            }
        }

        result.roundTrip = summarize(std::move(roundTrips));
        result.wakeLateness = summarize(std::move(wakeLateness));

        return result;
    }
}
