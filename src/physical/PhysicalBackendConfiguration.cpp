#include "physical/PhysicalBackendConfiguration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string_view>

#include <toml++/toml.hpp>

#include "config/TomlConfiguration.h"

namespace ngc::physical {
    namespace {
        std::expected<std::uint8_t, std::string> byteInteger(
            const toml::table &table,
            const std::string_view name,
            const std::filesystem::path &path) {
            const auto value = toml_configuration::integer(
                table, name, path);
            if (!value) {
                return std::unexpected(value.error());
            }
            if (*value < 0
                || *value
                    > std::numeric_limits<std::uint8_t>::max()) {
                return std::unexpected(toml_configuration::error(
                    path, name, "is outside the supported byte range",
                    table.get(name)));
            }

            return static_cast<std::uint8_t>(*value);
        }

        std::optional<std::string> unknownSpindleField(
            const toml::table &table) {
            constexpr std::array<std::string_view, 10> fields{
                "enabled", "driver", "device", "baud",
                "data_bits", "parity", "stop_bits",
                "slave_address", "maximum_speed",
                "at_speed_tolerance",
            };
            for (const auto &[key, value] : table) {
                static_cast<void>(value);
                if (std::ranges::find(
                        fields, key.str()) == fields.end()) {
                    return std::string(key.str());
                }
            }

            return std::nullopt;
        }
    }

    std::expected<PhysicalBackendConfiguration, std::string>
    loadPhysicalBackendConfiguration(
        const std::filesystem::path &path) {
        auto runtime =
            loadBackendRuntimeHostConfiguration(path);
        if (!runtime) {
            return std::unexpected(runtime.error());
        }
        auto motion = mesa::loadMesaBackendConfiguration(path);
        if (!motion) {
            return std::unexpected(motion.error());
        }

        try {
            const auto document = toml::parse_file(path.string());
            auto result = PhysicalBackendConfiguration{
                .runtime = *runtime,
                .motion = std::move(*motion),
                .spindle = std::nullopt,
            };
            const auto *spindleNode = document.get("spindle");
            if (spindleNode == nullptr) {
                return result;
            }
            const auto *spindle = spindleNode->as_table();
            if (spindle == nullptr) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle", "must be a table",
                    spindleNode));
            }
            if (const auto unknown =
                    unknownSpindleField(*spindle)) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle." + *unknown,
                    "is not a supported spindle field",
                    spindle->get(*unknown)));
            }

            const auto enabled =
                toml_configuration::requiredBool(
                    *spindle, "enabled", path);
            const auto driver =
                toml_configuration::requiredString(
                    *spindle, "driver", path);
            const auto device =
                toml_configuration::requiredString(
                    *spindle, "device", path);
            const auto baud =
                toml_configuration::integer(
                    *spindle, "baud", path);
            const auto dataBits =
                byteInteger(*spindle, "data_bits", path);
            const auto parity =
                toml_configuration::requiredString(
                    *spindle, "parity", path);
            const auto stopBits =
                byteInteger(*spindle, "stop_bits", path);
            const auto slaveAddress =
                byteInteger(*spindle, "slave_address", path);
            const auto maximumSpeed =
                toml_configuration::positiveNumber(
                    *spindle, "maximum_speed", path);
            const auto atSpeedTolerance =
                toml_configuration::positiveNumber(
                    *spindle, "at_speed_tolerance", path);
            const std::array required{
                enabled.has_value(), driver.has_value(),
                device.has_value(), baud.has_value(),
                dataBits.has_value(), parity.has_value(),
                stopBits.has_value(),
                slaveAddress.has_value(),
                maximumSpeed.has_value(),
                atSpeedTolerance.has_value(),
            };
            if (!std::ranges::all_of(
                    required, [](const bool value) {
                        return value;
                    })) {
                const std::array errors{
                    enabled.error_or({}), driver.error_or({}),
                    device.error_or({}), baud.error_or({}),
                    dataBits.error_or({}), parity.error_or({}),
                    stopBits.error_or({}),
                    slaveAddress.error_or({}),
                    maximumSpeed.error_or({}),
                    atSpeedTolerance.error_or({}),
                };
                const auto found = std::ranges::find_if(
                    errors, [](const std::string &value) {
                        return !value.empty();
                    });

                return std::unexpected(*found);
            }

            constexpr std::array supportedBaud{
                110, 300, 600, 1200, 2400, 4800,
                9600, 19200, 38400, 57600, 115200,
            };
            if (*driver != "huanyang_hycomm") {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.driver",
                    "must be huanyang_hycomm",
                    spindle->get("driver")));
            }
            if (device->empty()) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.device", "must not be empty",
                    spindle->get("device")));
            }
            if (*baud < 0
                || !std::ranges::contains(
                    supportedBaud,
                    static_cast<int>(*baud))) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.baud",
                    "is not a supported serial baud rate",
                    spindle->get("baud")));
            }
            if (*dataBits < 5 || *dataBits > 8) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.data_bits",
                    "must be between 5 and 8",
                    spindle->get("data_bits")));
            }
            if (*parity != "none"
                && *parity != "even"
                && *parity != "odd") {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.parity",
                    "must be none, even, or odd",
                    spindle->get("parity")));
            }
            if (*stopBits != 1 && *stopBits != 2) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.stop_bits",
                    "must be 1 or 2",
                    spindle->get("stop_bits")));
            }
            if (*slaveAddress == 0 || *slaveAddress > 247) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle.slave_address",
                    "must be between 1 and 247",
                    spindle->get("slave_address")));
            }
            if (!std::isfinite(*maximumSpeed)
                || !std::isfinite(*atSpeedTolerance)
                || *atSpeedTolerance > 1.0) {
                return std::unexpected(toml_configuration::error(
                    path, "spindle",
                    "contains an invalid speed or tolerance",
                    spindleNode));
            }

            result.spindle = {
                .enabled = *enabled,
                .device = *device,
                .baud = static_cast<std::uint32_t>(*baud),
                .dataBits = *dataBits,
                .parity = *parity == "none"
                    ? SerialParity::None
                    : (*parity == "even"
                        ? SerialParity::Even
                        : SerialParity::Odd),
                .stopBits = *stopBits,
                .slaveAddress = *slaveAddress,
                .maximumSpeed = *maximumSpeed,
                .atSpeedTolerance = *atSpeedTolerance,
            };

            return result;
        } catch (const toml::parse_error &error) {
            return std::unexpected(std::string(error.description()));
        }
    }
}
