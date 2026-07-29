#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <string>

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
}
