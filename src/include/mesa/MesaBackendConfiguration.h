#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "mesa/HostMot2CyclicIo.h"

namespace ngc::mesa {
    enum class MesaLinearUnit : std::uint8_t {
        Millimeter,
        Inch,
    };

    struct MesaConfiguredStepGenerator {
        std::uint8_t joint = 0;
        std::uint8_t channel = 0;
        double stepsPerUnit = 0.0;
        bool invertDirection = false;
        double positionGainPerSecond = 0.0;
        double maximumCorrectionVelocity = 0.0;
        double maximumGeneratedStepError = 0.0;
    };

    struct MesaNamedFieldInput {
        std::string name;
        std::uint16_t index = 0;
    };

    enum class MesaSafetyPolarity : std::uint8_t {
        ActiveHigh,
        ActiveLow,
    };

    struct MesaSafetyConfiguration {
        std::string enableInput;
        MesaSafetyPolarity polarity =
            MesaSafetyPolarity::ActiveHigh;
    };

    struct MesaBackendConfiguration {
        std::string address;
        std::string expectedBoard;
        std::string ioProgram;
        MesaLinearUnit linearUnit = MesaLinearUnit::Millimeter;
        HostMot2StepTiming stepTiming;
        std::uint32_t watchdogTimeoutNanoseconds = 0;
        HostMot2DpllConfiguration dpll;
        std::optional<MesaSafetyConfiguration> safety;
        std::vector<MesaNamedFieldInput> fieldInputs;
        std::vector<MesaConfiguredStepGenerator> stepGenerators;
    };

    [[nodiscard]] std::expected<
        MesaBackendConfiguration, std::string>
    loadMesaBackendConfiguration(
        const std::filesystem::path &path);
}
