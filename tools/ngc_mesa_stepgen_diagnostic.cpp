#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "config/BackendRuntimeConfiguration.h"
#include "machine/MachineConfiguration.h"
#include "machine/RealtimeHost.h"
#include "mesa/HostMot2CyclicIo.h"
#include "mesa/HostMot2Discovery.h"
#include "mesa/Lbp16UdpTransport.h"
#include "mesa/MesaBackendConfiguration.h"
#include "mesa/SevenI96Capabilities.h"
#include "mesa/SevenI96CyclicLayout.h"

namespace {
    volatile std::sig_atomic_t interrupted = 0;

    struct Options {
        std::string machineConfiguration = "machine.toml";
        std::string mesaConfiguration =
            "physical_backend.toml";
        std::chrono::milliseconds timeout{10};
        std::uint32_t cyclesPerDirection = 1'000;
        double rateStepsPerSecond = 800.0;
        bool validateConfigurationOnly = false;
        bool ordinaryScheduler = false;
    };

    void handleSignal(const int) {
        interrupted = 1;
    }

    std::uint32_t parseUnsigned(
        const std::string_view value,
        const std::string_view description) {
        std::uint32_t result = 0;
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
        double result = 0.0;
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

            if (option == "--machine-config") {
                result.machineConfiguration = value();
            } else if (option == "--mesa-config") {
                result.mesaConfiguration = value();
            } else if (option == "--timeout-ms") {
                result.timeout = std::chrono::milliseconds(
                    parseUnsigned(value(), "--timeout-ms"));
            } else if (option == "--cycles") {
                result.cyclesPerDirection =
                    parseUnsigned(value(), "--cycles");
            } else if (option == "--rate-steps-per-second") {
                result.rateStepsPerSecond = parsePositive(
                    value(), "--rate-steps-per-second");
            } else if (option == "--validate-config-only") {
                result.validateConfigurationOnly = true;
            } else if (option == "--ordinary-scheduler") {
                result.ordinaryScheduler = true;
            } else {
                throw std::runtime_error(std::format(
                    "unknown option '{}'", option));
            }
        }

        return result;
    }

    std::uint32_t servoPeriodNanoseconds(
        const ngc::MachineConfiguration &configuration) {
        if (!configuration.realBackend) {
            throw std::runtime_error(
                "machine configuration has no real_backend servo period");
        }
        const auto scaled =
            configuration.realBackend->servoPeriod
            * 1'000'000'000.0;
        if (!std::isfinite(scaled) || scaled < 1.0
            || scaled > std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error(
                "real_backend servo period is outside "
                "the diagnostic's nanosecond range");
        }

        return static_cast<std::uint32_t>(std::llround(scaled));
    }

    ngc::mesa::HostMot2CyclicLayout diagnosticLayout(
        const ngc::mesa::SevenI96Capabilities &capabilities,
        const ngc::mesa::MesaBackendConfiguration &configuration) {
        auto result = ngc::mesa::sevenI96CyclicLayout(
            capabilities, configuration.stepTiming);
        result.stepGeneratorCount =
            configuration.stepGenerators.size();
        for (std::size_t index = 0;
             index < result.stepGeneratorCount; ++index) {
            const auto &configured =
                configuration.stepGenerators[index];
            const auto &pins =
                capabilities.stepGenerators[configured.channel];
            result.stepGenerators[index] = {
                .channel = configured.channel,
                .stepPin = pins.stepPin,
                .directionPin = pins.directionPin,
                .invertDirection = configured.invertDirection,
                .timing = configuration.stepTiming,
            };
        }
        result.digitalOutputCount = 0;

        return result;
    }

    std::string_view faultName(
        ngc::mesa::HostMot2CyclicIoFault fault) noexcept;

    struct ScalarTimingDiagnostics {
        std::uint64_t samples = 0;
        long double total = 0.0;
        long double absoluteTotal = 0.0;
        std::int64_t minimum = std::numeric_limits<std::int64_t>::max();
        std::int64_t maximum = std::numeric_limits<std::int64_t>::min();
        std::uint64_t maximumAbsolute = 0;

        void observe(const std::int64_t value) noexcept {
            ++samples;
            total += static_cast<long double>(value);
            const auto magnitude = value < 0
                ? static_cast<std::uint64_t>(-(value + 1)) + 1
                : static_cast<std::uint64_t>(value);
            absoluteTotal += static_cast<long double>(magnitude);
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
            maximumAbsolute = std::max(maximumAbsolute, magnitude);
        }

        [[nodiscard]] double average() const noexcept {
            return samples == 0
                ? 0.0
                : static_cast<double>(total / samples);
        }

        [[nodiscard]] double averageAbsolute() const noexcept {
            return samples == 0
                ? 0.0
                : static_cast<double>(absoluteTotal / samples);
        }
    };

    struct CyclicTimingDiagnostics {
        ScalarTimingDiagnostics wakeLateness;
        ScalarTimingDiagnostics periodJitter;
        ScalarTimingDiagnostics exchangeDuration;
        ScalarTimingDiagnostics dpllPhaseOffset;
        std::uint64_t missedDeadlines = 0;
    };

    class SafeOutputGuard {
    public:
        explicit SafeOutputGuard(
            ngc::mesa::HostMot2CyclicIo &io) noexcept : m_io(io) { }

        ~SafeOutputGuard() {
            static_cast<void>(m_io.cycle({}));
        }

        SafeOutputGuard(const SafeOutputGuard &) = delete;
        SafeOutputGuard &operator=(const SafeOutputGuard &) = delete;

    private:
        ngc::mesa::HostMot2CyclicIo &m_io;
    };

    void disableOutputs(ngc::mesa::HostMot2CyclicIo &io) {
        const auto result = io.cycle({});
        if (result.fault
                != ngc::mesa::HostMot2CyclicIoFault::None
            && result.fault
                != ngc::mesa::HostMot2CyclicIoFault::WatchdogTripped) {
            throw std::runtime_error(std::format(
                "safe HostMot2 shutdown failed: {} ({})",
                faultName(result.fault),
                static_cast<std::uint32_t>(result.fault)));
        }
    }

    class ScheduledCyclicIo {
    public:
        ScheduledCyclicIo(
            ngc::mesa::HostMot2CyclicIo &io,
            const std::chrono::nanoseconds period)
            : m_io(io),
              m_period(period),
              m_deadline(std::chrono::steady_clock::now() + period) { }

        ~ScheduledCyclicIo() {
            if (!m_reported) {
                try {
                    reportDiagnostics();
                } catch (...) { }
            }
        }

        ScheduledCyclicIo(const ScheduledCyclicIo &) = delete;
        ScheduledCyclicIo &operator=(const ScheduledCyclicIo &) = delete;

        const ngc::mesa::HostMot2CyclicInputImage &cycle(
            const ngc::mesa::HostMot2CyclicOutputImage &outputs) {
            const auto deadline = m_deadline;
            ngc::sleepUntilMonotonic(deadline);
            const auto wake = std::chrono::steady_clock::now();
            m_deadline += m_period;
            m_diagnostics.wakeLateness.observe(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    wake - deadline).count());
            if (m_havePreviousWake) {
                m_diagnostics.periodJitter.observe(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        wake - m_previousWake - m_period).count());
            }
            m_previousWake = wake;
            m_havePreviousWake = true;

            const auto result = m_io.cycle(outputs);
            const auto finished = std::chrono::steady_clock::now();
            m_diagnostics.exchangeDuration.observe(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    finished - wake).count());
            if (finished > m_deadline) {
                ++m_diagnostics.missedDeadlines;
            }
            if (result.dpllPhaseErrorValid) {
                m_diagnostics.dpllPhaseOffset.observe(
                    result.dpllPhaseErrorNanoseconds);
            }
            if (result.fault
                    != ngc::mesa::HostMot2CyclicIoFault::None
                || !result.inputsValid) {
                const auto dpllDetail =
                    result.dpllPhaseErrorValid
                    ? std::format(
                        "; observed phase {} ns",
                        result.dpllPhaseErrorNanoseconds)
                    : std::string{};
                throw std::runtime_error(std::format(
                    "HostMot2 cyclic exchange failed: {} ({}){}",
                    faultName(result.fault),
                    static_cast<std::uint32_t>(result.fault),
                    dpllDetail));
            }
            const auto &inputs = m_io.inputImage();

            return inputs;
        }

        void resetDiagnostics() noexcept {
            m_diagnostics = {};
            m_previousWake = {};
            m_havePreviousWake = false;
            m_reported = false;
        }

        void reportDiagnostics() {
            constexpr auto nanosecondsPerMicrosecond = 1'000.0;
            const auto &wake = m_diagnostics.wakeLateness;
            const auto &jitter = m_diagnostics.periodJitter;
            const auto &exchange = m_diagnostics.exchangeDuration;
            const auto &phase = m_diagnostics.dpllPhaseOffset;

            std::println(
                "Cyclic timing over {} exchanges:",
                wake.samples);
            std::println(
                "  wake lateness: average={:.3f} us "
                "average_abs={:.3f} us worst_abs={:.3f} us "
                "range=[{:.3f}, {:.3f}] us",
                wake.average() / nanosecondsPerMicrosecond,
                wake.averageAbsolute() / nanosecondsPerMicrosecond,
                static_cast<double>(wake.maximumAbsolute)
                    / nanosecondsPerMicrosecond,
                static_cast<double>(
                    wake.samples == 0 ? 0 : wake.minimum)
                    / nanosecondsPerMicrosecond,
                static_cast<double>(
                    wake.samples == 0 ? 0 : wake.maximum)
                    / nanosecondsPerMicrosecond);
            std::println(
                "  period jitter: average={:.3f} us "
                "average_abs={:.3f} us worst_abs={:.3f} us "
                "range=[{:.3f}, {:.3f}] us",
                jitter.average() / nanosecondsPerMicrosecond,
                jitter.averageAbsolute() / nanosecondsPerMicrosecond,
                static_cast<double>(jitter.maximumAbsolute)
                    / nanosecondsPerMicrosecond,
                static_cast<double>(
                    jitter.samples == 0 ? 0 : jitter.minimum)
                    / nanosecondsPerMicrosecond,
                static_cast<double>(
                    jitter.samples == 0 ? 0 : jitter.maximum)
                    / nanosecondsPerMicrosecond);
            std::println(
                "  UDP exchange: average={:.3f} us worst={:.3f} us; "
                "missed next deadlines={}",
                exchange.average() / nanosecondsPerMicrosecond,
                static_cast<double>(exchange.maximum)
                    / nanosecondsPerMicrosecond,
                m_diagnostics.missedDeadlines);
            std::println(
                "  DPLL phase error: average={:.3f} us "
                "average_abs={:.3f} us worst_abs={:.3f} us "
                "range=[{:.3f}, {:.3f}] us over {} ready samples",
                phase.average() / nanosecondsPerMicrosecond,
                phase.averageAbsolute() / nanosecondsPerMicrosecond,
                static_cast<double>(phase.maximumAbsolute)
                    / nanosecondsPerMicrosecond,
                static_cast<double>(
                    phase.samples == 0 ? 0 : phase.minimum)
                    / nanosecondsPerMicrosecond,
                static_cast<double>(
                    phase.samples == 0 ? 0 : phase.maximum)
                    / nanosecondsPerMicrosecond,
                phase.samples);
            m_reported = true;
        }

    private:
        ngc::mesa::HostMot2CyclicIo &m_io;
        std::chrono::nanoseconds m_period;
        std::chrono::steady_clock::time_point m_deadline;
        std::chrono::steady_clock::time_point m_previousWake;
        CyclicTimingDiagnostics m_diagnostics;
        bool m_havePreviousWake = false;
        bool m_reported = false;
    };

    ngc::mesa::HostMot2CyclicOutputImage enabledZeroOutput(
        const std::size_t stepGeneratorCount) {
        auto result = ngc::mesa::HostMot2CyclicOutputImage{};
        result.watchdogEnabled = true;
        result.stepGeneratorsEnabled = true;
        for (std::size_t index = 0;
             index < stepGeneratorCount; ++index) {
            result.stepGenerators[index].enabled = true;
        }

        return result;
    }

    std::string_view faultName(
        const ngc::mesa::HostMot2CyclicIoFault fault) noexcept {
        using Fault = ngc::mesa::HostMot2CyclicIoFault;
        switch (fault) {
            case Fault::None: return "none";
            case Fault::NotInitialized: return "not initialized";
            case Fault::InvalidOutput: return "invalid output";
            case Fault::Transport: return "transport";
            case Fault::ReadSequenceMismatch:
                return "read sequence mismatch";
            case Fault::WriteSequenceMismatch:
                return "write sequence mismatch";
            case Fault::BoardProtocolError:
                return "board protocol error";
            case Fault::WatchdogTripped:
                return "watchdog tripped";
            case Fault::DpllPhaseError:
                return "DPLL phase error";
        }

        return "unknown";
    }
}

int main(const int argc, char **argv) {
    try {
        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);
        const auto options = parseOptions(argc, argv);
        const auto machine = ngc::loadMachineConfiguration(
            options.machineConfiguration);
        if (!machine) {
            throw std::runtime_error(machine.error());
        }
        const auto mesa = ngc::mesa::loadMesaBackendConfiguration(
            options.mesaConfiguration);
        if (!mesa) {
            throw std::runtime_error(mesa.error());
        }
        const auto host =
            ngc::loadBackendRuntimeHostConfiguration(
                options.mesaConfiguration);
        if (!host) {
            throw std::runtime_error(host.error());
        }
        const auto periodNanoseconds =
            servoPeriodNanoseconds(*machine);
        if (options.validateConfigurationOnly) {
            std::println(
                "Validated {} and {}: address={} servo_period={} ns "
                "stepgens={}",
                options.machineConfiguration,
                options.mesaConfiguration,
                mesa->address, periodNanoseconds,
                mesa->stepGenerators.size());

            return 0;
        }
        auto transport = ngc::mesa::Lbp16UdpTransport::open({
            .address = mesa->address,
            .timeout = options.timeout,
        });
        if (!transport) {
            throw std::runtime_error(transport.error());
        }
        const auto inventory =
            ngc::mesa::discoverHostMot2(**transport);
        if (!inventory) {
            throw std::runtime_error(inventory.error());
        }
        const auto capabilities =
            ngc::mesa::validateSevenI96Capabilities(*inventory);
        if (!capabilities) {
            throw std::runtime_error(capabilities.error());
        }
        auto cyclicConfiguration =
            ngc::mesa::HostMot2CyclicConfiguration{
                .watchdogTimeoutNanoseconds =
                    mesa->watchdogTimeoutNanoseconds,
                .dpll = mesa->dpll,
            };
        cyclicConfiguration.dpll.servoPeriodNanoseconds =
            periodNanoseconds;
        const auto layout = diagnosticLayout(
            *capabilities, *mesa);
        auto created = ngc::mesa::HostMot2CyclicIo::create(
            **transport, layout, cyclicConfiguration);
        if (!created) {
            throw std::runtime_error(created.error());
        }
        auto io = std::move(*created);
        const auto initialized = io->initializeSafe();
        if (initialized.fault
            != ngc::mesa::HostMot2CyclicIoFault::None) {
            throw std::runtime_error(std::format(
                "safe HostMot2 initialization failed with fault {}",
                static_cast<std::uint32_t>(initialized.fault)));
        }
        SafeOutputGuard safeOutputs(*io);
        ScheduledCyclicIo scheduled(
            *io, std::chrono::nanoseconds(periodNanoseconds));
        auto outputs = enabledZeroOutput(
            mesa->stepGenerators.size());

        std::println(
            "Connected to {} at {}; configured {} StepGens",
            inventory->idrom.boardName, mesa->address,
            mesa->stepGenerators.size());
        if (!options.ordinaryScheduler
            && host->realtimeEnabled) {
            if (host->lockMemory) {
                ngc::lockProcessMemory();
            }
            ngc::configureCurrentRealtimeThread(
                host->realtimeCpu,
                host->realtimePriority);
            std::println(
                "Using CPU {} with SCHED_FIFO priority {}{}",
                host->realtimeCpu,
                host->realtimePriority,
                host->lockMemory
                    ? " and locked memory"
                    : "");
        } else {
            std::println(
                "Using the ordinary scheduler{}",
                options.ordinaryScheduler
                    ? " by explicit request"
                    : " because no real-time host policy is configured");
        }
        auto convergenceCycles = std::uint32_t{0};
        const auto convergenceLimit =
            mesa->dpll.convergenceCycles * 10 + 1'000;
        while (!io->inputImage().dpll.ready) {
            if (interrupted != 0) {
                throw std::runtime_error("diagnostic interrupted");
            }
            static_cast<void>(scheduled.cycle(outputs));
            if (++convergenceCycles > convergenceLimit) {
                throw std::runtime_error(
                    "DPLL did not converge within the diagnostic limit");
            }
        }
        std::println(
            "DPLL converged after {} cycles; phase error {} ns",
            convergenceCycles,
            io->inputImage().dpll.phaseErrorNanoseconds);
        scheduled.resetDiagnostics();

        const auto periodSeconds =
            static_cast<double>(periodNanoseconds)
            / 1'000'000'000.0;
        for (std::size_t index = 0;
             index < mesa->stepGenerators.size(); ++index) {
            const auto &configured =
                mesa->stepGenerators[index];
            for (const auto direction : {1.0, -1.0}) {
                static_cast<void>(scheduled.cycle(outputs));
                const auto start =
                    io->inputImage()
                        .stepAccumulatorSubcounts[index];
                outputs.stepGenerators[index].stepsPerSecond =
                    direction * options.rateStepsPerSecond;
                for (std::uint32_t cycle = 0;
                     cycle < options.cyclesPerDirection; ++cycle) {
                    if (interrupted != 0) {
                        throw std::runtime_error(
                            "diagnostic interrupted");
                    }
                    static_cast<void>(scheduled.cycle(outputs));
                }
                outputs.stepGenerators[index].stepsPerSecond = 0.0;
                static_cast<void>(scheduled.cycle(outputs));
                const auto end =
                    io->inputImage()
                        .stepAccumulatorSubcounts[index];
                const auto actual =
                    static_cast<double>(end - start);
                const auto expected =
                    direction * options.rateStepsPerSecond
                    * options.cyclesPerDirection
                    * periodSeconds * 65'536.0
                    * (configured.invertDirection ? -1.0 : 1.0);
                const auto errorSteps =
                    std::abs(actual - expected) / 65'536.0;
                if (errorSteps
                    > configured.maximumGeneratedStepError) {
                    throw std::runtime_error(std::format(
                        "StepGen channel {} accumulator error "
                        "{:.6f} steps exceeds {:.6f}",
                        configured.channel, errorSteps,
                        configured.maximumGeneratedStepError));
                }
                std::println(
                    "channel {} {:>8.3f} steps/s: "
                    "delta={:.6f} steps expected={:.6f} "
                    "error={:.6f}",
                    configured.channel,
                    direction * options.rateStepsPerSecond,
                    actual / 65'536.0,
                    expected / 65'536.0,
                    errorSteps);
            }
        }

        scheduled.reportDiagnostics();
        disableOutputs(*io);
        std::println(
            "Mesa StepGen diagnostic passed; watchdog and rates disabled");

        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Mesa StepGen diagnostic failed: "
                  << error.what() << '\n';

        return 1;
    }
}
