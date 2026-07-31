#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <string>
#include <string_view>

#include <toml++/toml.hpp>

namespace ngc::toml_configuration {
    struct Document {
        toml::table table;
        std::uint64_t fingerprint = 0;
    };

    [[nodiscard]] std::expected<Document, std::string> loadDocument(
        const std::filesystem::path &path);

    [[nodiscard]] std::string error(
        const std::filesystem::path &path,
        std::string_view field,
        std::string_view message,
        const toml::node *node = nullptr);

    [[nodiscard]] std::expected<double, std::string> number(
        const toml::table &table,
        std::string_view name,
        const std::filesystem::path &path);
    [[nodiscard]] std::expected<double, std::string> positiveNumber(
        const toml::table &table,
        std::string_view name,
        const std::filesystem::path &path);
    [[nodiscard]] std::expected<std::string, std::string> requiredString(
        const toml::table &table,
        std::string_view name,
        const std::filesystem::path &path);
    [[nodiscard]] std::expected<bool, std::string> requiredBool(
        const toml::table &table,
        std::string_view name,
        const std::filesystem::path &path);
    [[nodiscard]] std::expected<std::int64_t, std::string> integer(
        const toml::table &table,
        std::string_view name,
        const std::filesystem::path &path);
}
