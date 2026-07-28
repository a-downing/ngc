#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <format>
#include <optional>
#include <print>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "physical/HuanyangSpindleHardware.h"
#include "physical/PhysicalBackendConfiguration.h"
#include "physical/SerialTransport.h"

namespace {
    using namespace std::chrono_literals;

    volatile std::sig_atomic_t interrupted = 0;

    struct Options {
        std::filesystem::path backendConfiguration =
            "physical_backend.toml";
        std::chrono::milliseconds transactionTimeout{500};
        std::chrono::milliseconds sampleInterval{100};
        std::chrono::milliseconds commandDuration{1000};
        std::chrono::milliseconds stopTimeout{5000};
        std::uint32_t samples = 10;
        std::optional<double> commandTestSpeed;
        bool validateConfigurationOnly = false;
        bool help = false;
    };

    void handleSignal(const int) {
        interrupted = 1;
    }

    std::uint32_t parseUnsigned(
        const std::string_view value,
        const std::string_view description) {
        auto result = std::uint32_t{0};
        const auto parsed = std::from_chars(
            value.data(), value.data() + value.size(), result);
        if (parsed.ec != std::errc{}
            || parsed.ptr != value.data() + value.size()
            || result == 0) {
            throw std::runtime_error(std::format(
                "{} must be a positive unsigned integer",
                description));
        }

        return result;
    }

    double parsePositive(
        const std::string_view value,
        const std::string_view description) {
        auto result = 0.0;
        const auto parsed = std::from_chars(
            value.data(), value.data() + value.size(), result);
        if (parsed.ec != std::errc{}
            || parsed.ptr != value.data() + value.size()
            || !std::isfinite(result) || result <= 0.0) {
            throw std::runtime_error(std::format(
                "{} must be a positive finite number",
                description));
        }

        return result;
    }

    Options parseOptions(const int argc, char **argv) {
        auto result = Options{};
        for (auto index = 1; index < argc; ++index) {
            const auto option = std::string_view(argv[index]);
            const auto value = [&]() -> std::string_view {
                if (++index == argc) {
                    throw std::runtime_error(std::format(
                        "{} requires a value", option));
                }

                return argv[index];
            };

            if (option == "--backend-configuration") {
                result.backendConfiguration = value();
            } else if (option == "--timeout-ms") {
                result.transactionTimeout =
                    std::chrono::milliseconds(parseUnsigned(
                        value(), "--timeout-ms"));
            } else if (option == "--samples") {
                result.samples =
                    parseUnsigned(value(), "--samples");
            } else if (option == "--sample-interval-ms") {
                result.sampleInterval =
                    std::chrono::milliseconds(parseUnsigned(
                        value(), "--sample-interval-ms"));
            } else if (option == "--command-test-speed") {
                result.commandTestSpeed = parsePositive(
                    value(), "--command-test-speed");
            } else if (option == "--command-duration-ms") {
                result.commandDuration =
                    std::chrono::milliseconds(parseUnsigned(
                        value(), "--command-duration-ms"));
            } else if (option == "--stop-timeout-ms") {
                result.stopTimeout =
                    std::chrono::milliseconds(parseUnsigned(
                        value(), "--stop-timeout-ms"));
            } else if (option == "--validate-config-only") {
                result.validateConfigurationOnly = true;
            } else if (option == "--help") {
                result.help = true;
            } else {
                throw std::runtime_error(std::format(
                    "unknown option '{}'", option));
            }
        }

        return result;
    }

    void printUsage() {
        std::println(
            "Usage: ngc_huanyang_spindle_diagnostic [options]\n"
            "  --backend-configuration PATH\n"
            "  --timeout-ms N\n"
            "  --samples N\n"
            "  --sample-interval-ms N\n"
            "  --validate-config-only\n"
            "\n"
            "Default operation establishes Stop, reads the configured "
            "PD values, and polls status without commanding Run.\n"
            "\n"
            "Motion-capable opt-in:\n"
            "  --command-test-speed RPM\n"
            "  --command-duration-ms N\n"
            "  --stop-timeout-ms N\n"
            "\n"
            "The command test executes CW, Stop-to-zero, CCW, and "
            "Stop-to-zero. Use it only after physical safety checks.");
    }

    bool sleepInterruptibly(
        const std::chrono::milliseconds duration) {
        const auto deadline =
            std::chrono::steady_clock::now() + duration;
        while (!interrupted
               && std::chrono::steady_clock::now() < deadline) {
            const auto remaining =
                deadline - std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::min(
                std::chrono::duration_cast<
                    std::chrono::milliseconds>(remaining),
                50ms));
        }

        return interrupted == 0;
    }

    ngc::SpindleHardwareStatus pollStatus(
        ngc::physical::HuanyangSpindleHardware &hardware,
        const std::string_view phase) {
        auto status = ngc::SpindleHardwareStatus{};
        if (!hardware.pollStatus(status)) {
            throw std::runtime_error(std::format(
                "Huanyang status polling failed during {}", phase));
        }
        std::println(
            "{}: speed={:.3f} rpm current={:.3f} A at_speed={}",
            phase, status.speed, status.current,
            status.atSpeed ? "yes" : "no");

        return status;
    }

    bool waitForStopped(
        ngc::physical::HuanyangSpindleHardware &hardware,
        const Options &options,
        const std::string_view phase) {
        const auto deadline =
            std::chrono::steady_clock::now() + options.stopTimeout;
        while (!interrupted
               && std::chrono::steady_clock::now() < deadline) {
            const auto status = pollStatus(hardware, phase);
            if (std::abs(status.speed) <= 1.0) {
                return true;
            }
            const auto remaining =
                std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                        deadline
                        - std::chrono::steady_clock::now());
            if (remaining <= 0ms
                || !sleepInterruptibly(std::min(
                    options.sampleInterval, remaining))) {
                break;
            }
        }

        return false;
    }

    bool sampleFor(
        ngc::physical::HuanyangSpindleHardware &hardware,
        const Options &options,
        const std::string_view phase,
        const std::chrono::milliseconds duration) {
        const auto deadline =
            std::chrono::steady_clock::now() + duration;
        while (!interrupted
               && std::chrono::steady_clock::now() < deadline) {
            static_cast<void>(pollStatus(hardware, phase));
            const auto remaining =
                std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                        deadline
                        - std::chrono::steady_clock::now());
            if (remaining <= 0ms
                || !sleepInterruptibly(std::min(
                    options.sampleInterval, remaining))) {
                break;
            }
        }

        return interrupted == 0;
    }

    void commandStop(
        ngc::physical::HuanyangSpindleHardware &hardware,
        const Options &options,
        const std::string_view phase) {
        if (!hardware.applyDesired({})) {
            throw std::runtime_error(std::format(
                "Huanyang Stop command failed after {}", phase));
        }
        if (!waitForStopped(hardware, options, "stopping")) {
            throw std::runtime_error(std::format(
                "Huanyang spindle did not report zero speed "
                "within {} ms after {}",
                options.stopTimeout.count(), phase));
        }
    }

    bool runDirection(
        ngc::physical::HuanyangSpindleHardware &hardware,
        const Options &options,
        const ngc::Direction direction,
        const std::string_view phase) {
        const auto desired = ngc::SpindleEvent{
            .enabled = true,
            .direction = direction,
            .speed = *options.commandTestSpeed,
        };
        if (!hardware.applyDesired(desired)) {
            throw std::runtime_error(std::format(
                "Huanyang {} command failed", phase));
        }
        const auto completed = sampleFor(
            hardware, options, phase, options.commandDuration);
        if (!completed) {
            hardware.safeStop();

            return false;
        }
        commandStop(hardware, options, phase);

        return true;
    }

    void printSetup(
        const ngc::physical::HuanyangSpindleSetup &setup) {
        std::println(
            "PD004 base frequency: {:.2f} Hz\n"
            "PD005 maximum frequency: {:.2f} Hz\n"
            "PD011 minimum frequency: {:.2f} Hz\n"
            "PD141 rated voltage: {:.1f} V\n"
            "PD142 rated current: {:.1f} A\n"
            "PD143 motor poles: {}\n"
            "PD144 rated speed at 50 Hz: {:.0f} rpm\n"
            "Derived rated speed at maximum frequency: {:.0f} rpm",
            setup.baseFrequency,
            setup.maximumFrequency,
            setup.minimumFrequency,
            setup.ratedVoltage,
            setup.ratedCurrent,
            setup.motorPoles,
            setup.ratedSpeedAt50Hz,
            setup.ratedMaximumSpeed);
    }

    int run(const Options &options) {
        if (options.help) {
            printUsage();

            return 0;
        }
        const auto physical =
            ngc::physical::loadPhysicalBackendConfiguration(
                options.backendConfiguration);
        if (!physical) {
            throw std::runtime_error(physical.error());
        }
        if (!physical->spindle.has_value()) {
            throw std::runtime_error(
                "physical backend configuration has no spindle role");
        }
        const auto &configuration = *physical->spindle;
        if (options.commandTestSpeed.has_value()
            && *options.commandTestSpeed
                > configuration.maximumSpeed) {
            throw std::runtime_error(std::format(
                "command-test speed exceeds configured maximum "
                "speed {:.3f} rpm",
                configuration.maximumSpeed));
        }
        if (options.validateConfigurationOnly) {
            return 0;
        }

        auto transport = ngc::physical::openSerialTransport(
            configuration, options.transactionTimeout);
        if (!transport) {
            throw std::runtime_error(transport.error());
        }
        auto hardware =
            ngc::physical::HuanyangSpindleHardware::create(
                configuration, std::move(*transport));
        if (!hardware) {
            throw std::runtime_error(hardware.error());
        }

        printSetup((*hardware)->setup());
        if (!waitForStopped(
                **hardware, options, "startup stop")) {
            throw std::runtime_error(
                "Huanyang spindle did not report zero speed "
                "after startup Stop");
        }
        if (!options.commandTestSpeed.has_value()) {
            for (auto sample = std::uint32_t{0};
                 sample < options.samples && !interrupted;
                 ++sample) {
                static_cast<void>(pollStatus(
                    **hardware, "stop-only"));
                if (sample + 1 < options.samples) {
                    static_cast<void>(sleepInterruptibly(
                        options.sampleInterval));
                }
            }

            return interrupted ? 130 : 0;
        }

        std::println(
            stderr,
            "WARNING: command test requested at {:.3f} rpm",
            *options.commandTestSpeed);
        if (!runDirection(
                **hardware, options,
                ngc::Direction::CW, "CW")) {
            return 130;
        }
        if (!runDirection(
                **hardware, options,
                ngc::Direction::CCW, "CCW")) {
            return 130;
        }
        commandStop(**hardware, options, "command test");
        std::println("Huanyang spindle diagnostic passed");

        return 0;
    }
}

int main(const int argc, char **argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception &error) {
        std::println(
            stderr, "Huanyang spindle diagnostic failed: {}",
            error.what());

        return 1;
    }
}
