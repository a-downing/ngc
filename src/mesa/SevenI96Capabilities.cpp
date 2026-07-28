#include "mesa/SevenI96Capabilities.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <string_view>

namespace ngc::mesa {
    namespace {
        constexpr std::uint8_t DPLL_TAG = 0x1A;
        constexpr std::uint8_t WATCHDOG_TAG = 0x02;
        constexpr std::uint8_t IO_PORT_TAG = 0x03;
        constexpr std::uint8_t STEP_GENERATOR_TAG = 0x05;
        constexpr std::uint8_t ENCODER_TAG = 0x04;
        constexpr std::uint8_t SSR_TAG = 0xC3;
        constexpr std::uint32_t HOSTMOT2_ADDRESS_SPACE_SIZE = 0x1'0000;
        constexpr std::uint32_t EXPECTED_CLOCK_HZ = 100'000'000;
        constexpr std::uint32_t EXPECTED_INSTANCE_STRIDE = 4;
        constexpr std::uint32_t EXPECTED_REGISTER_STRIDE = 256;

        struct ExpectedModule {
            std::uint8_t tag = 0;
            std::uint8_t version = 0;
            std::uint8_t instances = 0;
            std::uint8_t registers = 0;
            std::uint32_t multipleRegisterBitmap = 0;
            std::string_view name;
        };

        std::expected<const HostMot2Module *, std::string> findModule(
            const HostMot2Inventory &inventory,
            const ExpectedModule &expected) {
            const auto first = std::ranges::find(
                inventory.modules, expected.tag, &HostMot2Module::tag);
            if (first == inventory.modules.end()) {
                return std::unexpected(std::format(
                    "7I96 requires the HostMot2 {} module",
                    expected.name));
            }
            if (std::ranges::find(
                    std::next(first), inventory.modules.end(),
                    expected.tag, &HostMot2Module::tag)
                != inventory.modules.end()) {
                return std::unexpected(std::format(
                    "7I96 HostMot2 inventory contains duplicate {} modules",
                    expected.name));
            }

            return &*first;
        }

        std::expected<SevenI96ModuleLayout, std::string> validateModule(
            const HostMot2Inventory &inventory,
            const ExpectedModule &expected) {
            const auto found = findModule(inventory, expected);
            if (!found) {
                return std::unexpected(found.error());
            }
            const auto &module = **found;
            if (module.version != expected.version) {
                return std::unexpected(std::format(
                    "7I96 {} module version {} is incompatible; expected {}",
                    expected.name, module.version, expected.version));
            }
            if (module.instances != expected.instances) {
                return std::unexpected(std::format(
                    "7I96 {} module exposes {} instances; expected {}",
                    expected.name, module.instances, expected.instances));
            }
            if (module.registers != expected.registers) {
                return std::unexpected(std::format(
                    "7I96 {} module exposes {} registers; expected {}",
                    expected.name, module.registers, expected.registers));
            }
            if (module.multipleRegisterBitmap
                != expected.multipleRegisterBitmap) {
                return std::unexpected(std::format(
                    "7I96 {} module multiple-register bitmap 0x{:08X} "
                    "is incompatible; expected 0x{:08X}",
                    expected.name, module.multipleRegisterBitmap,
                    expected.multipleRegisterBitmap));
            }
            if ((module.baseAddress & 0x3) != 0) {
                return std::unexpected(std::format(
                    "7I96 {} module base address 0x{:04X} is not aligned",
                    expected.name, module.baseAddress));
            }

            const auto clock = hostMot2ModuleClockHz(inventory, module);
            const auto instanceStride =
                hostMot2ModuleInstanceStride(inventory, module);
            const auto registerStride =
                hostMot2ModuleRegisterStride(inventory, module);
            if (clock != EXPECTED_CLOCK_HZ) {
                return std::unexpected(std::format(
                    "7I96 {} module clock {} Hz is incompatible; expected {} Hz",
                    expected.name, clock, EXPECTED_CLOCK_HZ));
            }
            if (instanceStride != EXPECTED_INSTANCE_STRIDE
                || registerStride != EXPECTED_REGISTER_STRIDE) {
                return std::unexpected(std::format(
                    "7I96 {} module strides {}/{} are incompatible; "
                    "expected {}/{}",
                    expected.name, instanceStride, registerStride,
                    EXPECTED_INSTANCE_STRIDE, EXPECTED_REGISTER_STRIDE));
            }

            const auto finalAddress =
                static_cast<std::uint64_t>(module.baseAddress)
                + static_cast<std::uint64_t>(module.registers - 1)
                    * registerStride
                + static_cast<std::uint64_t>(module.instances - 1)
                    * instanceStride;
            if (finalAddress + sizeof(std::uint32_t)
                > HOSTMOT2_ADDRESS_SPACE_SIZE) {
                return std::unexpected(std::format(
                    "7I96 {} module register layout exceeds "
                    "the HostMot2 address space",
                    expected.name));
            }

            return SevenI96ModuleLayout{
                .descriptor = module,
                .clockHz = clock,
                .instanceStride = instanceStride,
                .registerStride = registerStride,
            };
        }

        std::expected<void, std::string> validatePin(
            const HostMot2Inventory &inventory,
            const std::size_t pinIndex,
            const std::uint8_t secondaryTag,
            const std::uint8_t secondaryChannel,
            const std::uint8_t secondaryPin,
            const bool output,
            const std::string_view description) {
            if (pinIndex >= inventory.pins.size()) {
                return std::unexpected(std::format(
                    "7I96 {} references missing HostMot2 pin {}",
                    description, pinIndex));
            }
            const auto &pin = inventory.pins[pinIndex];
            if (pin.primaryTag != IO_PORT_TAG) {
                return std::unexpected(std::format(
                    "7I96 {} pin {} has primary function {}; expected IOPort",
                    description, pinIndex,
                    hostMot2ModuleName(pin.primaryTag)));
            }
            if (pin.secondaryTag != secondaryTag
                || pin.secondaryChannelNumber() != secondaryChannel
                || pin.secondaryPinNumber() != secondaryPin
                || pin.secondaryOutput() != output) {
                return std::unexpected(std::format(
                    "7I96 {} pin {} has incompatible HostMot2 assignment",
                    description, pinIndex));
            }

            return {};
        }

        template<std::size_t Capacity>
        std::expected<void, std::string> validateSelections(
            const std::vector<std::uint8_t> &selections,
            const std::string_view description) {
            std::array<bool, Capacity> selected{};
            for (const auto selection : selections) {
                if (selection >= Capacity) {
                    return std::unexpected(std::format(
                        "configured 7I96 {} {} is outside the supported range 0..{}",
                        description, selection, Capacity - 1));
                }
                if (selected[selection]) {
                    return std::unexpected(std::format(
                        "configured 7I96 {} {} is duplicated",
                        description, selection));
                }
                selected[selection] = true;
            }

            return {};
        }
    }

    std::expected<SevenI96Capabilities, std::string>
    validateSevenI96Capabilities(
        const HostMot2Inventory &inventory,
        const SevenI96CapabilityRequirements &requirements) {
        if (inventory.idrom.boardName != "MESA7I96") {
            return std::unexpected(std::format(
                "expected HostMot2 board MESA7I96, discovered '{}'",
                inventory.idrom.boardName));
        }
        if (inventory.idrom.type != 3) {
            return std::unexpected(std::format(
                "7I96 HostMot2 IDROM type {} is incompatible; expected 3",
                inventory.idrom.type));
        }
        if (inventory.idrom.ioPorts != 3
            || inventory.idrom.portWidth != 17
            || inventory.idrom.ioWidth != 51
            || inventory.pins.size() != 51) {
            return std::unexpected(std::format(
                "7I96 I/O topology is incompatible: ports={} width={} "
                "port_width={} discovered_pins={}; expected 3/51/17/51",
                inventory.idrom.ioPorts, inventory.idrom.ioWidth,
                inventory.idrom.portWidth, inventory.pins.size()));
        }

        SevenI96Capabilities result;
        const auto dpll = validateModule(inventory, {
            .tag = DPLL_TAG,
            .version = 0,
            .instances = 1,
            .registers = 7,
            .multipleRegisterBitmap = 0,
            .name = "DPLL",
        });
        const auto watchdog = validateModule(inventory, {
            .tag = WATCHDOG_TAG,
            .version = 0,
            .instances = 1,
            .registers = 3,
            .multipleRegisterBitmap = 0,
            .name = "Watchdog",
        });
        const auto ioPort = validateModule(inventory, {
            .tag = IO_PORT_TAG,
            .version = 0,
            .instances = 3,
            .registers = 5,
            .multipleRegisterBitmap = 0x1F,
            .name = "IOPort",
        });
        const auto stepGenerator = validateModule(inventory, {
            .tag = STEP_GENERATOR_TAG,
            .version = 2,
            .instances = 5,
            .registers = 10,
            .multipleRegisterBitmap = 0x1FF,
            .name = "StepGen",
        });
        const auto encoder = validateModule(inventory, {
            .tag = ENCODER_TAG,
            .version = 2,
            .instances = 1,
            .registers = 5,
            .multipleRegisterBitmap = 0x03,
            .name = "Encoder",
        });
        const auto ssr = validateModule(inventory, {
            .tag = SSR_TAG,
            .version = 0,
            .instances = 1,
            .registers = 2,
            .multipleRegisterBitmap = 0x03,
            .name = "SSR",
        });
        const std::array moduleResults{
            &dpll, &watchdog, &ioPort,
            &stepGenerator, &encoder, &ssr,
        };
        for (const auto *module : moduleResults) {
            if (!*module) {
                return std::unexpected(module->error());
            }
        }
        result.dpll = *dpll;
        result.watchdog = *watchdog;
        result.ioPort = *ioPort;
        result.stepGenerator = *stepGenerator;
        result.encoder = *encoder;
        result.ssr = *ssr;

        for (std::size_t pin = 0;
             pin < SEVEN_I96_ISOLATED_INPUT_COUNT; ++pin) {
            const auto valid = validatePin(
                inventory, pin, 0, 0, 0, false,
                "isolated input");
            if (!valid) {
                return std::unexpected(valid.error());
            }
            result.isolatedInputPins[pin] =
                static_cast<std::uint8_t>(pin);
        }
        for (std::size_t output = 0;
             output < SEVEN_I96_ISOLATED_OUTPUT_COUNT; ++output) {
            const auto pin =
                SEVEN_I96_ISOLATED_INPUT_COUNT + output;
            const auto valid = validatePin(
                inventory, pin, SSR_TAG, 0,
                static_cast<std::uint8_t>(output + 1), true,
                "isolated output");
            if (!valid) {
                return std::unexpected(valid.error());
            }
            result.isolatedOutputPins[output] =
                static_cast<std::uint8_t>(pin);
        }
        for (std::size_t channel = 0;
             channel < SEVEN_I96_STEP_GENERATOR_COUNT; ++channel) {
            const auto stepPin =
                SEVEN_I96_ISOLATED_INPUT_COUNT
                + SEVEN_I96_ISOLATED_OUTPUT_COUNT
                + channel * 2;
            const auto directionPin = stepPin + 1;
            const auto validStep = validatePin(
                inventory, stepPin, STEP_GENERATOR_TAG,
                static_cast<std::uint8_t>(channel), 1, true,
                "step-generator step");
            if (!validStep) {
                return std::unexpected(validStep.error());
            }
            const auto validDirection = validatePin(
                inventory, directionPin, STEP_GENERATOR_TAG,
                static_cast<std::uint8_t>(channel), 2, true,
                "step-generator direction");
            if (!validDirection) {
                return std::unexpected(validDirection.error());
            }
            result.stepGenerators[channel] = {
                .channel = static_cast<std::uint8_t>(channel),
                .stepPin = static_cast<std::uint8_t>(stepPin),
                .directionPin =
                    static_cast<std::uint8_t>(directionPin),
            };
        }

        constexpr std::size_t ENCODER_PIN = 27;
        if (const auto valid = validatePin(
                inventory, ENCODER_PIN, ENCODER_TAG, 0, 1, false,
                "encoder A"); !valid) {
            return std::unexpected(valid.error());
        }
        if (const auto valid = validatePin(
                inventory, ENCODER_PIN + 1, ENCODER_TAG, 0, 2, false,
                "encoder B"); !valid) {
            return std::unexpected(valid.error());
        }
        if (const auto valid = validatePin(
                inventory, ENCODER_PIN + 2, ENCODER_TAG, 0, 3, false,
                "encoder index"); !valid) {
            return std::unexpected(valid.error());
        }
        result.encoderPins = {
            .channel = 0,
            .pinA = ENCODER_PIN,
            .pinB = ENCODER_PIN + 1,
            .indexPin = ENCODER_PIN + 2,
        };

        if (const auto valid = validateSelections<
                SEVEN_I96_STEP_GENERATOR_COUNT>(
                requirements.stepGeneratorChannels,
                "step-generator channel"); !valid) {
            return std::unexpected(valid.error());
        }
        if (const auto valid = validateSelections<
                SEVEN_I96_ISOLATED_INPUT_COUNT>(
                requirements.isolatedInputPins,
                "isolated input pin"); !valid) {
            return std::unexpected(valid.error());
        }
        if (const auto valid = validateSelections<
                SEVEN_I96_ISOLATED_OUTPUT_COUNT>(
                requirements.isolatedOutputPins,
                "isolated output pin"); !valid) {
            return std::unexpected(valid.error());
        }
        if (requirements.encoderChannel
            && *requirements.encoderChannel != 0) {
            return std::unexpected(std::format(
                "configured 7I96 encoder channel {} is outside "
                "the supported range 0..0",
                *requirements.encoderChannel));
        }

        return result;
    }
}
