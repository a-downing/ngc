#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>

namespace ngc::toml_configuration {
    [[nodiscard]] std::expected<std::uint64_t, std::string>
    fileFingerprint(const std::filesystem::path &path);

    [[nodiscard]] std::uint64_t combinedFingerprint(
        std::uint64_t machineFingerprint,
        std::optional<std::uint64_t> backendFingerprint,
        double executorServoPeriod) noexcept;
}
