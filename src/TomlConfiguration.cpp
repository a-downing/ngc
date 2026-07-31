#include "config/TomlConfiguration.h"

#include <bit>
#include <cmath>
#include <cstddef>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>

#include "config/ConfigurationFingerprint.h"

namespace ngc::toml_configuration {
    namespace {
        constexpr std::uint64_t FNV_OFFSET_BASIS =
            14695981039346656037ULL;
        constexpr std::uint64_t FNV_PRIME = 1099511628211ULL;

        void appendByte(
            std::uint64_t &fingerprint,
            const std::uint8_t value) noexcept {
            fingerprint ^= value;
            fingerprint *= FNV_PRIME;
        }

        void appendBytes(
            std::uint64_t &fingerprint,
            const std::string_view value) noexcept {
            for (const auto byte : value) {
                appendByte(
                    fingerprint,
                    static_cast<std::uint8_t>(
                        static_cast<unsigned char>(byte)));
            }
        }

        void appendUnsigned(
            std::uint64_t &fingerprint,
            const std::uint64_t value) noexcept {
            for (auto shift = 0U; shift < 64U; shift += 8U) {
                appendByte(
                    fingerprint,
                    static_cast<std::uint8_t>(value >> shift));
            }
        }

        std::uint64_t sourceFingerprint(
            const std::string_view source) noexcept {
            auto result = FNV_OFFSET_BASIS;
            appendBytes(result, "ngc-toml-source-v1");
            appendUnsigned(result, source.size());
            appendBytes(result, source);

            return result;
        }
    }

    std::expected<Document, std::string> loadDocument(
        const std::filesystem::path &path) {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            return std::unexpected(std::format(
                "{}: could not open configuration file",
                path.string()));
        }
        const std::string source{
            std::istreambuf_iterator<char>(file),
            std::istreambuf_iterator<char>()};
        if (file.bad()) {
            return std::unexpected(std::format(
                "{}: could not read complete configuration file",
                path.string()));
        }

        try {
            return Document{
                .table = toml::parse(source, path.string()),
                .fingerprint = sourceFingerprint(source),
            };
        } catch (const toml::parse_error &parseError) {
            return std::unexpected(std::format(
                "{}:{}:{}: {}", path.string(),
                parseError.source().begin.line,
                parseError.source().begin.column,
                parseError.description()));
        }
    }

    std::expected<std::uint64_t, std::string>
    fileFingerprint(const std::filesystem::path &path) {
        const auto document = loadDocument(path);
        if (!document) {
            return std::unexpected(document.error());
        }

        return document->fingerprint;
    }

    std::uint64_t combinedFingerprint(
        const std::uint64_t machineFingerprint,
        const std::optional<std::uint64_t> backendFingerprint,
        const double executorServoPeriod) noexcept {
        auto result = FNV_OFFSET_BASIS;
        appendBytes(result, "ngc-ipc-configuration-v1");
        appendUnsigned(result, machineFingerprint);
        appendByte(
            result,
            backendFingerprint.has_value() ? 1U : 0U);
        if (backendFingerprint.has_value()) {
            appendUnsigned(result, *backendFingerprint);
        }
        appendUnsigned(
            result,
            std::bit_cast<std::uint64_t>(
                executorServoPeriod));

        return result;
    }

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
