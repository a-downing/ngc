#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ngc {
    inline constexpr std::size_t REALTIME_TIMING_HISTOGRAM_BUCKETS = 32;

    // Histogram bucket zero contains zero-nanosecond samples. Positive bucket
    // b contains values from 2^(b-1) through 2^b-1 nanoseconds, clamped to the
    // final bucket.
    struct RealtimeTimingSummary {
        std::uint64_t firstTick = 0;
        std::uint64_t lastTick = 0;
        std::uint64_t sampleCount = 0;
        std::int64_t maximumWakeLatenessNanoseconds = 0;
        std::uint64_t maximumExecutionNanoseconds = 0;
        std::int64_t minimumDeadlineSlackNanoseconds = 0;
        std::uint64_t missedDeadlines = 0;
        std::uint64_t skippedPeriods = 0;
        std::uint64_t maximumConsecutiveMisses = 0;
        std::uint64_t worstTick = 0;
        std::uint64_t failedPublications = 0;
        std::uint32_t ioFaultCode = 0;
        std::uint32_t ioFaultJoint = 0;
        double ioFaultFollowingErrorSteps = 0.0;
        double ioFaultTargetPosition = 0.0;
        double ioFaultActualPosition = 0.0;
        std::int32_t ioFaultDpllPhaseErrorNanoseconds = 0;
        std::array<
            std::uint64_t,
            REALTIME_TIMING_HISTOGRAM_BUCKETS> wakeLatenessHistogram{};
        std::array<
            std::uint64_t,
            REALTIME_TIMING_HISTOGRAM_BUCKETS> executionHistogram{};
    };

    static_assert(std::is_trivially_copyable_v<RealtimeTimingSummary>);

    inline void mergeRealtimeTiming(
        RealtimeTimingSummary &aggregate,
        const RealtimeTimingSummary &summary) noexcept {
        if (summary.sampleCount == 0) {
            return;
        }
        if (aggregate.sampleCount == 0) {
            aggregate = summary;

            return;
        }

        aggregate.lastTick = summary.lastTick;
        aggregate.sampleCount += summary.sampleCount;
        aggregate.maximumWakeLatenessNanoseconds = std::max(
            aggregate.maximumWakeLatenessNanoseconds,
            summary.maximumWakeLatenessNanoseconds);
        aggregate.maximumExecutionNanoseconds = std::max(
            aggregate.maximumExecutionNanoseconds,
            summary.maximumExecutionNanoseconds);
        if (summary.minimumDeadlineSlackNanoseconds
            < aggregate.minimumDeadlineSlackNanoseconds) {
            aggregate.minimumDeadlineSlackNanoseconds =
                summary.minimumDeadlineSlackNanoseconds;
            aggregate.worstTick = summary.worstTick;
        }
        aggregate.missedDeadlines += summary.missedDeadlines;
        aggregate.skippedPeriods += summary.skippedPeriods;
        aggregate.maximumConsecutiveMisses = std::max(
            aggregate.maximumConsecutiveMisses,
            summary.maximumConsecutiveMisses);
        aggregate.failedPublications += summary.failedPublications;
        if (summary.ioFaultCode != 0) {
            aggregate.ioFaultCode = summary.ioFaultCode;
            aggregate.ioFaultJoint = summary.ioFaultJoint;
            aggregate.ioFaultFollowingErrorSteps =
                summary.ioFaultFollowingErrorSteps;
            aggregate.ioFaultTargetPosition =
                summary.ioFaultTargetPosition;
            aggregate.ioFaultActualPosition =
                summary.ioFaultActualPosition;
            aggregate.ioFaultDpllPhaseErrorNanoseconds =
                summary.ioFaultDpllPhaseErrorNanoseconds;
        }
        for (auto bucket = std::size_t{0};
             bucket < REALTIME_TIMING_HISTOGRAM_BUCKETS;
             ++bucket) {
            aggregate.wakeLatenessHistogram[bucket] +=
                summary.wakeLatenessHistogram[bucket];
            aggregate.executionHistogram[bucket] +=
                summary.executionHistogram[bucket];
        }
    }
}
