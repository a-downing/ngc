#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <vector>

#include "mesa/HostMot2Discovery.h"

namespace ngc::mesa {
    inline constexpr std::size_t SEVEN_I96_STEP_GENERATOR_COUNT = 5;
    inline constexpr std::size_t SEVEN_I96_ISOLATED_INPUT_COUNT = 11;
    inline constexpr std::size_t SEVEN_I96_ISOLATED_OUTPUT_COUNT = 6;

    struct SevenI96CapabilityRequirements {
        std::vector<std::uint8_t> stepGeneratorChannels;
        std::vector<std::uint8_t> isolatedInputPins;
        std::vector<std::uint8_t> isolatedOutputPins;
        std::optional<std::uint8_t> encoderChannel;
    };

    struct SevenI96ModuleLayout {
        HostMot2Module descriptor;
        std::uint32_t clockHz = 0;
        std::uint32_t instanceStride = 0;
        std::uint32_t registerStride = 0;
    };

    struct SevenI96StepGeneratorPins {
        std::uint8_t channel = 0;
        std::uint8_t stepPin = 0;
        std::uint8_t directionPin = 0;
    };

    struct SevenI96EncoderPins {
        std::uint8_t channel = 0;
        std::uint8_t pinA = 0;
        std::uint8_t pinB = 0;
        std::uint8_t indexPin = 0;
    };

    struct SevenI96Capabilities {
        SevenI96ModuleLayout dpll;
        SevenI96ModuleLayout watchdog;
        SevenI96ModuleLayout ioPort;
        SevenI96ModuleLayout stepGenerator;
        SevenI96ModuleLayout encoder;
        SevenI96ModuleLayout ssr;
        std::array<
            SevenI96StepGeneratorPins,
            SEVEN_I96_STEP_GENERATOR_COUNT> stepGenerators;
        SevenI96EncoderPins encoderPins;
        std::array<
            std::uint8_t,
            SEVEN_I96_ISOLATED_INPUT_COUNT> isolatedInputPins;
        std::array<
            std::uint8_t,
            SEVEN_I96_ISOLATED_OUTPUT_COUNT> isolatedOutputPins;
    };

    [[nodiscard]] std::expected<SevenI96Capabilities, std::string>
    validateSevenI96Capabilities(
        const HostMot2Inventory &inventory,
        const SevenI96CapabilityRequirements &requirements = {});
}
