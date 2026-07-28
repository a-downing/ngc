#include <charconv>
#include <chrono>
#include <cstdint>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mesa/HostMot2Discovery.h"
#include "mesa/HostMot2Latency.h"
#include "mesa/Lbp16UdpTransport.h"

namespace {
    struct Options {
        std::string address;
        std::size_t samples = 10'000;
        std::chrono::microseconds period{1'000};
        std::chrono::milliseconds timeout{10};
    };

    std::uint64_t parseUnsigned(
        const std::string_view value,
        const std::string_view description) {
        std::uint64_t result = 0;
        const auto parsed = std::from_chars(
            value.data(), value.data() + value.size(), result);
        if (parsed.ec != std::errc{}
            || parsed.ptr != value.data() + value.size()) {
            throw std::runtime_error(std::format(
                "{} must be an unsigned integer", description));
        }

        return result;
    }

    Options parseOptions(const int argc, char **argv) {
        Options result;
        for (auto index = 1; index < argc; ++index) {
            const auto option = std::string_view(argv[index]);
            const auto value = [&]() -> std::string_view {
                if (++index == argc) {
                    throw std::runtime_error(std::format(
                        "{} requires a value", option));
                }

                return argv[index];
            };

            if (option == "--address") {
                result.address = value();
            } else if (option == "--samples") {
                const auto samples = parseUnsigned(value(), "--samples");
                if (samples == 0) {
                    throw std::runtime_error(
                        "--samples must be positive");
                }
                if (samples > std::numeric_limits<std::size_t>::max()) {
                    throw std::runtime_error(
                        "--samples is too large");
                }
                result.samples = static_cast<std::size_t>(samples);
            } else if (option == "--period-us") {
                const auto period = parseUnsigned(
                    value(), "--period-us");
                if (period == 0) {
                    throw std::runtime_error(
                        "--period-us must be positive");
                }
                if (period > static_cast<std::uint64_t>(
                        std::numeric_limits<std::int64_t>::max())) {
                    throw std::runtime_error(
                        "--period-us is too large");
                }
                result.period = std::chrono::microseconds(
                    static_cast<std::int64_t>(period));
            } else if (option == "--timeout-ms") {
                const auto timeout = parseUnsigned(
                    value(), "--timeout-ms");
                if (timeout == 0) {
                    throw std::runtime_error(
                        "--timeout-ms must be positive");
                }
                if (timeout > static_cast<std::uint64_t>(
                        std::numeric_limits<std::int64_t>::max())) {
                    throw std::runtime_error(
                        "--timeout-ms is too large");
                }
                result.timeout = std::chrono::milliseconds(
                    static_cast<std::int64_t>(timeout));
            } else {
                throw std::runtime_error(std::format(
                    "unknown option '{}'", option));
            }
        }
        if (result.address.empty()) {
            throw std::runtime_error("--address is required");
        }

        return result;
    }

    double microseconds(const std::chrono::nanoseconds value) {
        return std::chrono::duration<double, std::micro>(value).count();
    }

    void printDistribution(
        const std::string_view label,
        const ngc::mesa::HostMot2LatencyDistribution &distribution) {
        if (distribution.sampleCount == 0) {
            std::println("{}: no samples", label);

            return;
        }

        std::println(
            "{} (us): min={:.3f} mean={:.3f} p50={:.3f} "
            "p95={:.3f} p99={:.3f} max={:.3f}",
            label,
            microseconds(distribution.minimum),
            microseconds(distribution.mean),
            microseconds(distribution.percentile50),
            microseconds(distribution.percentile95),
            microseconds(distribution.percentile99),
            microseconds(distribution.maximum));
    }
}

int main(const int argc, char **argv) {
    try {
        const auto options = parseOptions(argc, argv);
        const auto transport = ngc::mesa::Lbp16UdpTransport::open({
            .address = options.address,
            .timeout = options.timeout,
        });
        if (!transport) {
            throw std::runtime_error(transport.error());
        }
        const auto inventory = ngc::mesa::discoverHostMot2(**transport);
        if (!inventory) {
            throw std::runtime_error(inventory.error());
        }

        std::println(
            "Measuring {} read-only HostMot2 cookie transactions "
            "on {} at {} us periods",
            options.samples, inventory->idrom.boardName,
            options.period.count());
        const auto result = ngc::mesa::measureHostMot2ReadLatency(
            **transport, {
                .sampleCount = options.samples,
                .period = options.period,
            });
        if (!result) {
            throw std::runtime_error(result.error());
        }

        std::println(
            "Transactions: attempted={} responses={} read_failures={} "
            "content_mismatches={} missed_periods={}",
            result->attemptedSamples, result->receivedResponses,
            result->readFailures, result->contentMismatches,
            result->missedPeriods);
        printDistribution("Round trip", result->roundTrip);
        printDistribution("Wake lateness", result->wakeLateness);
        if (!result->lastReadError.empty()) {
            std::println("Last read failure: {}", result->lastReadError);
        }

        return result->readFailures == 0
            && result->contentMismatches == 0 ? 0 : 2;
    } catch (const std::exception &error) {
        std::cerr << "Mesa latency measurement failed: "
                  << error.what() << '\n';

        return 1;
    }
}
