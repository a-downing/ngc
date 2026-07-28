#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>

#include "mesa/MesaBackendConfiguration.h"

namespace ngc::physical {
    enum class SerialParity : std::uint8_t {
        None,
        Even,
        Odd,
    };

    struct HuanyangSpindleConfiguration {
        bool enabled = false;
        std::string device;
        std::uint32_t baud = 0;
        std::uint8_t dataBits = 0;
        SerialParity parity = SerialParity::None;
        std::uint8_t stopBits = 0;
        std::uint8_t slaveAddress = 0;
        double maximumSpeed = 0.0;
        double atSpeedTolerance = 0.0;
    };

    struct PhysicalBackendConfiguration {
        mesa::MesaBackendConfiguration motion;
        std::optional<HuanyangSpindleConfiguration> spindle;
    };

    [[nodiscard]] std::expected<
        PhysicalBackendConfiguration, std::string>
    loadPhysicalBackendConfiguration(
        const std::filesystem::path &path);
}
