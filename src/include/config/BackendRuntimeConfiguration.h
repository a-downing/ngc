#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <string>

#include <toml++/toml.hpp>

namespace ngc {
    struct BackendRuntimeHostConfiguration {
        bool realtimeEnabled = false;
        std::uint32_t realtimeCpu = 0;
        int realtimePriority = 0;
        bool lockMemory = false;
    };

    [[nodiscard]] std::expected<
        BackendRuntimeHostConfiguration, std::string>
    loadBackendRuntimeHostConfiguration(
        const std::filesystem::path &path);

    [[nodiscard]] std::expected<
        BackendRuntimeHostConfiguration, std::string>
    loadBackendRuntimeHostConfiguration(
        const toml::table &document,
        const std::filesystem::path &path);
}
