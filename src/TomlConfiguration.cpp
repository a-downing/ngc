#include "config/TomlConfiguration.h"

#include <cmath>
#include <format>
#include <optional>

namespace ngc::toml_configuration {
    std::string error(
        const std::filesystem::path &path,
        const std::string_view field,
        const std::string_view message,
        const toml::node *node) {
        if (node != nullptr) {
            const auto source = node->source();

            return std::format(
                "{}:{}:{}: {}: {}", path.string(),
                source.begin.line, source.begin.column,
                field, message);
        }

        return std::format(
            "{}: {}: {}", path.string(), field, message);
    }

    std::expected<double, std::string> number(
        const toml::table &table,
        const std::string_view name,
        const std::filesystem::path &path) {
        const auto *node = table.get(name);
        const auto value = node != nullptr
            ? node->value<double>()
            : std::optional<double>{};
        if (!value || !std::isfinite(*value)) {
            return std::unexpected(error(
                path, name, "must be a finite number", node));
        }

        return *value;
    }

    std::expected<double, std::string> positiveNumber(
        const toml::table &table,
        const std::string_view name,
        const std::filesystem::path &path) {
        const auto value = number(table, name, path);
        if (!value) {
            return value;
        }
        if (*value <= 0.0) {
            return std::unexpected(error(
                path, name, "must be greater than zero",
                table.get(name)));
        }

        return value;
    }

    std::expected<std::string, std::string> requiredString(
        const toml::table &table,
        const std::string_view name,
        const std::filesystem::path &path) {
        const auto *node = table.get(name);
        const auto value = node != nullptr
            ? node->value<std::string>()
            : std::optional<std::string>{};
        if (!value || value->empty()) {
            return std::unexpected(error(
                path, name, "must be a non-empty string", node));
        }

        return *value;
    }

    std::expected<bool, std::string> requiredBool(
        const toml::table &table,
        const std::string_view name,
        const std::filesystem::path &path) {
        const auto *node = table.get(name);
        const auto value = node != nullptr
            ? node->value<bool>()
            : std::optional<bool>{};
        if (!value) {
            return std::unexpected(error(
                path, name, "must be a boolean", node));
        }

        return *value;
    }

    std::expected<std::int64_t, std::string> integer(
        const toml::table &table,
        const std::string_view name,
        const std::filesystem::path &path) {
        const auto *node = table.get(name);
        const auto value = node != nullptr
            ? node->value<std::int64_t>()
            : std::optional<std::int64_t>{};
        if (!value) {
            return std::unexpected(error(
                path, name, "must be an integer", node));
        }

        return *value;
    }
}
