#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream>
#include <numeric>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "machine/MachineConfiguration.h"
#include "mesa/MesaProductionExecutorIo.h"
#include "physical/PhysicalBackendConfiguration.h"

namespace {
    struct Options {
        std::filesystem::path machineConfiguration =
            "machine.toml";
        std::filesystem::path backendConfiguration =
            "physical_backend.toml";
        std::uint32_t batches = 1'000;
        std::uint32_t iterationsPerBatch = 1'000;
    };

    struct Timing {
        double minimumNanoseconds = 0.0;
        double meanNanoseconds = 0.0;
        double p50Nanoseconds = 0.0;
        double p95Nanoseconds = 0.0;
        double p99Nanoseconds = 0.0;
        double maximumNanoseconds = 0.0;
    };

    std::uint32_t parsePositiveUnsigned(
        const std::string_view text,
        const std::string_view option) {
        auto result = std::uint32_t{0};
        const auto parsed = std::from_chars(
            text.data(), text.data() + text.size(), result);
        if (parsed.ec != std::errc{}
            || parsed.ptr != text.data() + text.size()
            || result == 0) {
            throw std::runtime_error(std::format(
                "{} must be a positive unsigned integer",
                option));
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

            if (option == "--machine-config") {
                result.machineConfiguration = value();
            } else if (option == "--backend-config") {
                result.backendConfiguration = value();
            } else if (option == "--batches") {
                result.batches = parsePositiveUnsigned(
                    value(), option);
            } else if (option == "--iterations-per-batch") {
                result.iterationsPerBatch =
                    parsePositiveUnsigned(value(), option);
            } else {
                throw std::runtime_error(std::format(
                    "unknown option '{}'", option));
            }
        }

        return result;
    }

    Timing summarize(std::vector<double> samples) {
        std::ranges::sort(samples);
        const auto percentile =
            [&](const double value) {
                const auto index = static_cast<std::size_t>(
                    std::floor(
                        value
                        * static_cast<double>(
                            samples.size() - 1)));

                return samples[index];
            };

        return {
            .minimumNanoseconds = samples.front(),
            .meanNanoseconds =
                std::accumulate(
                    samples.begin(), samples.end(), 0.0)
                / static_cast<double>(samples.size()),
            .p50Nanoseconds = percentile(0.50),
            .p95Nanoseconds = percentile(0.95),
            .p99Nanoseconds = percentile(0.99),
            .maximumNanoseconds = samples.back(),
        };
    }

    template<class Operation>
    Timing measure(
        const Options &options,
        Operation &&operation,
        std::uint64_t &checksum) {
        using Clock = std::chrono::steady_clock;

        std::vector<double> samples;
        samples.reserve(options.batches);
        for (auto batch = std::uint32_t{0};
             batch < options.batches; ++batch) {
            const auto started = Clock::now();
            for (auto iteration = std::uint32_t{0};
                 iteration < options.iterationsPerBatch;
                 ++iteration) {
                operation();
            }
            const auto finished = Clock::now();
            const auto elapsed =
                std::chrono::duration<double, std::nano>(
                    finished - started).count();
            samples.push_back(
                elapsed
                / static_cast<double>(
                    options.iterationsPerBatch));
            checksum += batch + 1;
        }

        return summarize(std::move(samples));
    }

    void report(
        const std::string_view name,
        const Timing &timing,
        const double servoPeriodNanoseconds) {
        std::println(
            "{}: mean={:.3f} ns p50={:.3f} ns "
            "p95={:.3f} ns p99={:.3f} ns "
            "range=[{:.3f}, {:.3f}] ns "
            "mean_servo_budget={:.6f}%",
            name,
            timing.meanNanoseconds,
            timing.p50Nanoseconds,
            timing.p95Nanoseconds,
            timing.p99Nanoseconds,
            timing.minimumNanoseconds,
            timing.maximumNanoseconds,
            100.0 * timing.meanNanoseconds
                / servoPeriodNanoseconds);
    }
}

int main(const int argc, char **argv) {
    try {
        const auto options = parseOptions(argc, argv);
        const auto machine = ngc::loadMachineConfiguration(
            options.machineConfiguration);
        if (!machine) {
            throw std::runtime_error(machine.error());
        }
        const auto physical =
            ngc::physical::loadPhysicalBackendConfiguration(
                options.backendConfiguration);
        if (!physical) {
            throw std::runtime_error(physical.error());
        }
        auto program =
            ngc::mesa::compileMesaDigitalIoProgram(
                *machine, physical->motion);
        if (!program) {
            throw std::runtime_error(program.error());
        }

        auto fieldInputs = ngc::FieldDigitalInputImage{};
        auto logicalOutputs =
            ngc::LogicalDigitalOutputImage{};
        auto logicalInputs =
            ngc::LogicalDigitalInputImage{};
        auto fieldOutputs =
            ngc::FieldDigitalOutputImage{};
        auto motion = ngc::ProductionExecutorMotionContext{};
        motion.flags =
            ngc::PRODUCTION_EXECUTOR_MOTION_IS_PROBE
            | ngc::PRODUCTION_EXECUTOR_MOTION_IS_HOMING;
        motion.axisTarget.z = 1.0;
        motion.axisPosition.z = 0.75;
        motion.moveJoints = ngc::JointMask{0x0F};
        motion.triggerJoints = ngc::JointMask{0x0F};
        for (ngc::JointId joint = 0;
             joint < 4; ++joint) {
            motion.jointPosition[joint] = 0.25;
        }
        for (std::size_t index = 0;
             index < program->fieldInputCount(); ++index) {
            fieldInputs[index] = index % 2 == 0;
        }
        for (std::size_t index = 0;
             index < logicalOutputs.size(); ++index) {
            logicalOutputs[index] = index % 3 == 0;
        }

        for (auto iteration = 0; iteration < 10'000;
             ++iteration) {
            program->executeInputs(
                fieldInputs, logicalOutputs,
                motion, logicalInputs);
            program->executeOutputs(
                fieldInputs, logicalOutputs,
                motion, fieldOutputs);
        }

        auto checksum = std::uint64_t{0};
        const auto input = measure(
            options,
            [&] {
                program->executeInputs(
                    fieldInputs, logicalOutputs,
                    motion, logicalInputs);
            },
            checksum);
        checksum += logicalInputs.count();
        const auto output = measure(
            options,
            [&] {
                program->executeOutputs(
                    fieldInputs, logicalOutputs,
                    motion, fieldOutputs);
            },
            checksum);
        checksum += fieldOutputs.count();
        const auto combined = measure(
            options,
            [&] {
                program->executeInputs(
                    fieldInputs, logicalOutputs,
                    motion, logicalInputs);
                program->executeOutputs(
                    fieldInputs, logicalOutputs,
                    motion, fieldOutputs);
            },
            checksum);
        checksum += logicalInputs.count()
            + fieldOutputs.count();

        const auto servoPeriodNanoseconds =
            machine->realBackend->servoPeriod
            * 1'000'000'000.0;
        std::println(
            "Mesa digital I/O program: instructions={} "
            "field_inputs={} logical_inputs={} "
            "field_outputs={} logical_outputs={} "
            "batches={} iterations_per_batch={}",
            program->instructionCount(),
            program->fieldInputCount(),
            program->logicalInputCount(),
            program->fieldOutputCount(),
            program->logicalOutputCount(),
            options.batches,
            options.iterationsPerBatch);
        report("input pass", input, servoPeriodNanoseconds);
        report("output pass", output, servoPeriodNanoseconds);
        report(
            "combined servo pass", combined,
            servoPeriodNanoseconds);
        std::println(
            "checksum={} (batch timing amortizes clock "
            "overhead; no baseline was subtracted)",
            checksum);

        return 0;
    } catch (const std::exception &error) {
        std::cerr
            << "Mesa digital I/O benchmark failed: "
            << error.what() << '\n';

        return 1;
    }
}
