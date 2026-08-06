#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "config/ConfigurationFingerprint.h"
#include "machine/IpcExecutorPeer.h"
#include "machine/MachineConfiguration.h"
#include "machine/PhysicalExecutorIo.h"
#include "machine/HostedExecutorRuntime.h"
#include "mesa/HostMot2CyclicIo.h"
#include "mesa/HostMot2Discovery.h"
#include "mesa/Lbp16UdpTransport.h"
#include "mesa/MesaProductionExecutorIo.h"
#include "mesa/SevenI96Capabilities.h"
#include "mesa/SevenI96CyclicLayout.h"
#include "physical/HuanyangSpindleHardware.h"
#include "physical/PhysicalBackendConfiguration.h"

namespace {
    struct Options {
        ngc::IpcExecutorPeerOptions ipc;
        std::filesystem::path machineConfiguration = "machine.toml";
        std::filesystem::path backendConfiguration =
            "physical_backend.toml";
        std::chrono::milliseconds timeout{10};
    };

    Options parseOptions(const int argc, char **argv) {
        Options result;
        for (auto index = 1; index < argc; ++index) {
            const auto option = std::string_view(argv[index]);
            const auto value = [&]() -> std::string_view {
                if (++index == argc) {
                    throw std::runtime_error(
                        "missing option value for "
                        + std::string(option));
                }

                return argv[index];
            };

            if (ngc::parseIpcExecutorPeerOption(
                    index, argc, argv, result.ipc)) {
                continue;
            }
            if (option == "--machine-configuration") {
                result.machineConfiguration = value();
            } else if (option == "--backend-configuration") {
                result.backendConfiguration = value();
            } else if (option == "--timeout-ms") {
                result.timeout = std::chrono::milliseconds(
                    ngc::parseUnsignedCommandLineValue(value()));
            } else {
                throw std::runtime_error(
                    "unknown option: " + std::string(option));
            }
        }
        ngc::validateIpcExecutorPeerOptions(result.ipc);

        return result;
    }

    std::uint32_t servoPeriodNanoseconds(
        const ngc::MachineConfiguration &configuration) {
        if (!configuration.machineExecutor.has_value()) {
            throw std::runtime_error(
                "machine configuration has no machine_executor");
        }
        const auto scaled =
            configuration.machineExecutor->servoPeriod
            * 1'000'000'000.0;
        if (!std::isfinite(scaled) || scaled < 1.0
            || scaled > std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error(
                "machine_executor servo period is outside "
                "the supported nanosecond range");
        }

        return static_cast<std::uint32_t>(std::llround(scaled));
    }

    double physicalUnitsPerMachineUnit(
        const ngc::Machine::Unit machine,
        const ngc::mesa::MesaLinearUnit physical) noexcept {
        if (machine == ngc::Machine::Unit::Millimeter
            && physical == ngc::mesa::MesaLinearUnit::Inch) {
            return 1.0 / 25.4;
        }
        if (machine == ngc::Machine::Unit::Inch
            && physical == ngc::mesa::MesaLinearUnit::Millimeter) {
            return 25.4;
        }

        return 1.0;
    }

    bool linearAxis(const ngc::Machine::Axis axis) noexcept {
        return axis == ngc::Machine::Axis::X
            || axis == ngc::Machine::Axis::Y
            || axis == ngc::Machine::Axis::Z;
    }

    ngc::DigitalInputId resolveInput(
        const ngc::MachineConfiguration &machine,
        const std::string_view name) {
        const auto found = std::ranges::find(
            machine.digitalInputs, name,
            &ngc::DigitalInputConfiguration::name);
        if (found == machine.digitalInputs.end()) {
            throw std::runtime_error(std::format(
                "Mesa safety input '{}' is not declared "
                "by machine configuration",
                name));
        }

        return found->id;
    }

    ngc::mesa::HostMot2CyclicLayout physicalLayout(
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

    std::vector<ngc::mesa::MesaStepGeneratorMapping>
    stepGeneratorMappings(
        const ngc::MachineConfiguration &machine,
        const ngc::mesa::MesaBackendConfiguration &mesa) {
        if (mesa.stepGenerators.size() != machine.joints.size()) {
            throw std::runtime_error(
                "Mesa configuration must map every configured joint");
        }

        std::vector<ngc::mesa::MesaStepGeneratorMapping> result;
        result.reserve(mesa.stepGenerators.size());
        for (std::size_t index = 0;
             index < mesa.stepGenerators.size(); ++index) {
            const auto &configured = mesa.stepGenerators[index];
            const auto joint = std::ranges::find(
                machine.joints, configured.joint,
                &ngc::JointConfiguration::id);
            if (joint == machine.joints.end()) {
                throw std::runtime_error(std::format(
                    "Mesa configuration maps unknown joint {}",
                    configured.joint));
            }
            const auto unitScale = linearAxis(joint->axis)
                ? physicalUnitsPerMachineUnit(
                    machine.unit, mesa.linearUnit)
                : 1.0;
            result.push_back({
                .joint = configured.joint,
                .stepGenerator = index,
                .stepsPerMachineUnit =
                    configured.stepsPerUnit * unitScale,
                .positionGainPerSecond =
                    configured.positionGainPerSecond,
                .maximumCorrectionVelocity =
                    configured.maximumCorrectionVelocity / unitScale,
                .maximumGeneratedStepError =
                    configured.maximumGeneratedStepError,
            });
        }

        return result;
    }

    struct ResolvedMesaExecutorConfiguration {
        std::uint32_t servoPeriodNanoseconds;
        std::vector<ngc::mesa::MesaStepGeneratorMapping> mappings;
        ngc::mesa::MesaExecutorSafetyInput safetyInput;
        ngc::DigitalIoProgram ioProgram;
    };

    ResolvedMesaExecutorConfiguration resolveExecutorConfiguration(
        const ngc::MachineConfiguration &machine,
        const ngc::physical::PhysicalBackendConfiguration &physical) {
        const auto &mesa = physical.motion;
        if (!mesa.safety.has_value()) {
            throw std::runtime_error(
                "Mesa physical backend requires a configured "
                "motion.safety enable input");
        }
        if (!physical.runtime.realtimeEnabled) {
            throw std::runtime_error(
                "the Mesa physical backend requires configured "
                "real-time CPU and priority");
        }
        auto ioProgram = ngc::mesa::compileMesaDigitalIoProgram(
            machine, mesa);
        if (!ioProgram) {
            throw std::runtime_error(ioProgram.error());
        }

        return {
            .servoPeriodNanoseconds = servoPeriodNanoseconds(machine),
            .mappings = stepGeneratorMappings(machine, mesa),
            .safetyInput = {
                .input = resolveInput(
                    machine, mesa.safety->enableInput),
                .requiredLevel = mesa.safety->polarity
                    == ngc::mesa::MesaSafetyPolarity::ActiveHigh,
            },
            .ioProgram = std::move(*ioProgram),
        };
    }

    std::unique_ptr<ngc::HostedExecutorRuntime> makeRuntime(
        const ngc::MachineConfiguration &machine,
        const ngc::physical::PhysicalBackendConfiguration
            &physical,
        ngc::mesa::Lbp16UdpTransport &transport,
        ResolvedMesaExecutorConfiguration resolved) {
        const auto &mesa = physical.motion;
        const auto inventory =
            ngc::mesa::discoverHostMot2(transport);
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
                    mesa.watchdogTimeoutNanoseconds,
                .dpll = mesa.dpll,
            };
        cyclicConfiguration.dpll.servoPeriodNanoseconds =
            resolved.servoPeriodNanoseconds;
        auto cyclic = ngc::mesa::HostMot2CyclicIo::create(
            transport, physicalLayout(*capabilities, mesa),
            cyclicConfiguration);
        if (!cyclic) {
            throw std::runtime_error(cyclic.error());
        }

        auto motion = ngc::mesa::MesaProductionExecutorIo::create(
            std::move(*cyclic),
            std::move(resolved.ioProgram),
            resolved.mappings, resolved.safetyInput);
        if (!motion) {
            throw std::runtime_error(motion.error());
        }
        auto spindle = std::unique_ptr<ngc::SpindleHardware>{};
        if (physical.spindle.has_value()
            && physical.spindle->enabled) {
            auto hardware =
                ngc::physical::createHuanyangSpindleHardware(
                    *physical.spindle);
            if (!hardware) {
                throw std::runtime_error(hardware.error());
            }
            spindle = std::move(*hardware);
        }
        auto io =
            std::make_unique<ngc::PhysicalExecutorIo>(
                std::move(*motion), std::move(spindle));

        auto runtimeConfiguration =
            ngc::hostedExecutorRuntimeConfiguration(machine);
        runtimeConfiguration.realtime = physical.runtime;

        return std::make_unique<ngc::HostedExecutorRuntime>(
            std::move(runtimeConfiguration), std::move(io));
    }

    int run(const Options &options) {
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
        const auto &mesa = physical->motion;
        auto resolved = resolveExecutorConfiguration(
            *machine, *physical);
        if (options.ipc.validateConfigurationOnly) {
            return 0;
        }

        auto transport = std::unique_ptr<ngc::mesa::Lbp16UdpTransport>{};
        const auto fingerprint =
            ngc::toml_configuration::combinedFingerprint(
                machine->sourceFingerprint,
                physical->sourceFingerprint,
                machine->machineExecutor->servoPeriod);

        return ngc::runIpcExecutorPeer(
            options.ipc, fingerprint, [&] {
                auto openedTransport = ngc::mesa::Lbp16UdpTransport::open({
                    .address = mesa.address,
                    .timeout = options.timeout,
                });
                if (!openedTransport) {
                    throw std::runtime_error(openedTransport.error());
                }
                transport = std::move(*openedTransport);

                return ngc::IpcExecutorPeerRuntime{
                    .runtime = makeRuntime(
                        *machine, *physical, *transport,
                        std::move(resolved)),
                };
            });
    }
}

int main(const int argc, char **argv) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception &error) {
        std::cerr << "ngc_mesa_backend failed: "
                  << error.what() << '\n';

        return 1;
    }
}
