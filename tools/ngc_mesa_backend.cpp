#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "IpcPlatform.h"
#include "machine/DigitalIoProgram.h"
#include "machine/IpcExecutorBridge.h"
#include "machine/IpcProtocol.h"
#include "machine/MachineConfiguration.h"
#include "machine/PhysicalProductionExecutorIo.h"
#include "machine/ProductionExecutorRuntime.h"
#include "mesa/HostMot2CyclicIo.h"
#include "mesa/HostMot2Discovery.h"
#include "mesa/Lbp16UdpTransport.h"
#include "mesa/MesaProductionExecutorIo.h"
#include "mesa/SevenI96Capabilities.h"
#include "mesa/SevenI96CyclicLayout.h"
#include "physical/PhysicalBackendConfiguration.h"

namespace {
    struct Options {
        std::string mapping;
        ngc::IpcIdentity expected{};
        std::filesystem::path machineConfiguration = "machine.toml";
        std::filesystem::path backendConfiguration =
            "physical_backend.toml";
        std::chrono::milliseconds timeout{10};
        bool validateConfigurationOnly = false;
    };

    std::uint64_t parseUnsigned(const std::string_view value) {
        std::size_t consumed = 0;
        const auto result = std::stoull(std::string(value), &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("invalid unsigned integer");
        }

        return result;
    }

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

            if (option == "--mapping") {
                result.mapping = value();
            } else if (option == "--configuration") {
                result.expected.configurationFingerprint =
                    parseUnsigned(value());
            } else if (option == "--topology") {
                result.expected.topologyFingerprint =
                    parseUnsigned(value());
            } else if (option == "--session") {
                result.expected.sessionGeneration =
                    parseUnsigned(value());
            } else if (option == "--epoch") {
                result.expected.epochGeneration =
                    parseUnsigned(value());
            } else if (option == "--authority") {
                result.expected.authorityGeneration =
                    parseUnsigned(value());
            } else if (option == "--machine-configuration") {
                result.machineConfiguration = value();
            } else if (option == "--backend-configuration") {
                result.backendConfiguration = value();
            } else if (option == "--timeout-ms") {
                result.timeout = std::chrono::milliseconds(
                    parseUnsigned(value()));
            } else if (option == "--validate-config-only") {
                result.validateConfigurationOnly = true;
            } else {
                throw std::runtime_error(
                    "unknown option: " + std::string(option));
            }
        }
        if (!result.validateConfigurationOnly
            && result.mapping.empty()) {
            throw std::runtime_error("--mapping is required");
        }

        return result;
    }

    std::uint32_t servoPeriodNanoseconds(
        const ngc::MachineConfiguration &configuration) {
        if (!configuration.realBackend.has_value()) {
            throw std::runtime_error(
                "machine configuration has no real_backend");
        }
        const auto scaled =
            configuration.realBackend->servoPeriod
            * 1'000'000'000.0;
        if (!std::isfinite(scaled) || scaled < 1.0
            || scaled > std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error(
                "real_backend servo period is outside "
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

    ngc::DigitalIoProgram compileDigitalIoProgram(
        const ngc::MachineConfiguration &machine,
        const ngc::mesa::MesaBackendConfiguration &mesa) {
        std::vector<ngc::DigitalInputId> logicalInputs;
        std::vector<ngc::DigitalIoSymbol> symbols;
        logicalInputs.reserve(machine.digitalInputs.size());
        symbols.reserve(
            machine.digitalInputs.size()
            + mesa.fieldInputs.size());
        for (const auto &input : machine.digitalInputs) {
            logicalInputs.push_back(input.id);
            symbols.push_back({
                .name = input.name,
                .kind =
                    ngc::DigitalIoSymbolKind::LogicalInput,
                .id = input.id,
            });
        }
        for (const auto &input : mesa.fieldInputs) {
            symbols.push_back({
                .name = input.name,
                .kind = ngc::DigitalIoSymbolKind::FieldInput,
                .id = input.index,
            });
        }
        const auto logicalOutputs =
            std::span<const ngc::DigitalOutputId>{};
        auto program = ngc::DigitalIoProgram::compile(
            mesa.ioProgram,
            ngc::mesa::SEVEN_I96_ISOLATED_INPUT_COUNT,
            logicalInputs, 0, logicalOutputs,
            machine.realBackend->servoPeriod, symbols);
        if (!program) {
            throw std::runtime_error(program.error());
        }

        auto result = std::move(*program);

        return result;
    }

    std::unique_ptr<ngc::ProductionExecutorRuntime> makeRuntime(
        const ngc::MachineConfiguration &machine,
        const ngc::physical::PhysicalBackendConfiguration
            &physical,
        ngc::mesa::Lbp16UdpTransport &transport) {
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

        const auto periodNanoseconds =
            servoPeriodNanoseconds(machine);
        auto cyclicConfiguration =
            ngc::mesa::HostMot2CyclicConfiguration{
                .watchdogTimeoutNanoseconds =
                    mesa.watchdogTimeoutNanoseconds,
                .dpll = mesa.dpll,
            };
        cyclicConfiguration.dpll.servoPeriodNanoseconds =
            periodNanoseconds;
        auto cyclic = ngc::mesa::HostMot2CyclicIo::create(
            transport, physicalLayout(*capabilities, mesa),
            cyclicConfiguration);
        if (!cyclic) {
            throw std::runtime_error(cyclic.error());
        }

        const auto mappings =
            stepGeneratorMappings(machine, mesa);
        if (!mesa.safety.has_value()) {
            throw std::runtime_error(
                "Mesa physical backend requires a configured "
                "motion.safety enable input");
        }
        const auto safetyInput = ngc::mesa::MesaExecutorSafetyInput{
            .input = resolveInput(
                machine, mesa.safety->enableInput),
            .requiredLevel = mesa.safety->polarity
                == ngc::mesa::MesaSafetyPolarity::ActiveHigh,
        };
        auto motion = ngc::mesa::MesaProductionExecutorIo::create(
            std::move(*cyclic),
            compileDigitalIoProgram(machine, mesa),
            mappings, safetyInput);
        if (!motion) {
            throw std::runtime_error(motion.error());
        }
        auto spindle = std::unique_ptr<ngc::SpindleHardware>{};
        if (physical.spindle.has_value()
            && physical.spindle->enabled) {
            throw std::runtime_error(
                "enabled Huanyang spindle hardware is not "
                "implemented");
        }
        auto io =
            std::make_unique<ngc::PhysicalProductionExecutorIo>(
                std::move(*motion), std::move(spindle));

        return std::make_unique<ngc::ProductionExecutorRuntime>(
            ngc::productionExecutorRuntimeConfiguration(machine),
            std::move(io));
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
        static_cast<void>(servoPeriodNanoseconds(*machine));
        if (mesa.safety.has_value()) {
            static_cast<void>(resolveInput(
                *machine, mesa.safety->enableInput));
        }
        static_cast<void>(stepGeneratorMappings(*machine, mesa));
        static_cast<void>(compileDigitalIoProgram(
            *machine, mesa));
        if (options.validateConfigurationOnly) {
            return 0;
        }
        if (!mesa.safety.has_value()) {
            throw std::runtime_error(
                "Mesa physical backend requires a configured "
                "motion.safety enable input");
        }
#ifndef __linux__
        throw std::runtime_error(
            "the Mesa physical backend requires Linux");
#else
        if (!machine->realBackend->realtimeEnabled) {
            throw std::runtime_error(
                "the Mesa physical backend requires configured "
                "real-time CPU and priority");
        }
#endif

        auto memory = ngc::ipc_detail::SharedMemory::open(
            options.mapping, sizeof(ngc::IpcSharedRegion));
        auto &region =
            *static_cast<ngc::IpcSharedRegion *>(memory.data());
        const auto rejection = ngc::validateIpcSharedRegion(
            region, options.expected);
        if (rejection != ngc::IpcRejection::None) {
            ngc::setIpcRejection(region, rejection);
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::Rejected);

            return 2;
        }

        auto transport = ngc::mesa::Lbp16UdpTransport::open({
            .address = mesa.address,
            .timeout = options.timeout,
        });
        if (!transport) {
            throw std::runtime_error(transport.error());
        }
        auto runtime = makeRuntime(
            *machine, *physical, **transport);
        runtime->start();
        region.peerProcessId =
            ngc::ipc_detail::currentProcessId();
        ngc::setIpcConnectionState(
            region, ngc::IpcConnectionState::PeerReady);
        while (ngc::ipcConnectionState(region)
               == ngc::IpcConnectionState::PeerReady) {
            std::this_thread::yield();
        }
        if (ngc::ipcConnectionState(region)
            != ngc::IpcConnectionState::Running) {
            runtime->stop();
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::PeerStopped);

            return 0;
        }

        ngc::IpcExecutorBridge bridge(
            region, *runtime, runtime->endpoint());
        for (;;) {
            if (ngc::ipcConnectionState(region)
                == ngc::IpcConnectionState::StopRequested) {
                runtime->stop();
                ngc::setIpcConnectionState(
                    region, ngc::IpcConnectionState::PeerStopped);

                return 0;
            }

            std::atomic_ref(region.peerHeartbeat).fetch_add(
                1, std::memory_order_relaxed);
            if (!bridge.service(true)) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(100));
            }
        }
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
