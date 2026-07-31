#include "config/BackendRuntimeConfiguration.h"

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <string_view>

#include <toml++/toml.hpp>

#include "config/TomlConfiguration.h"

namespace ngc {
    namespace {
        std::optional<std::string> unknownRuntimeField(
            const toml::table &table) {
            constexpr std::array<std::string_view, 3> fields{
                "realtime_cpu",
                "realtime_priority",
                "lock_memory",
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

    std::expected<BackendRuntimeHostConfiguration, std::string>
    loadBackendRuntimeHostConfiguration(
        const std::filesystem::path &path) {
        const auto document =
            toml_configuration::loadDocument(path);
        if (!document) {
            return std::unexpected(document.error());
        }

        return loadBackendRuntimeHostConfiguration(
            document->table, path);
    }

    std::expected<BackendRuntimeHostConfiguration, std::string>
    loadBackendRuntimeHostConfiguration(
        const toml::table &document,
        const std::filesystem::path &path) {
        const auto *runtimeNode = document.get("runtime");
        const auto *runtime =
            runtimeNode != nullptr
                ? runtimeNode->as_table()
                : nullptr;
        if (runtime == nullptr) {
            return std::unexpected(toml_configuration::error(
                path, "runtime", "must be a table",
                runtimeNode));
        }
        if (const auto unknown =
                unknownRuntimeField(*runtime)) {
            return std::unexpected(toml_configuration::error(
                path, "runtime." + *unknown,
                "is not a supported runtime field",
                runtime->get(*unknown)));
        }

        const auto *cpuNode =
            runtime->get("realtime_cpu");
        const auto *priorityNode =
            runtime->get("realtime_priority");
        if ((cpuNode == nullptr)
            != (priorityNode == nullptr)) {
            return std::unexpected(toml_configuration::error(
                path, "runtime",
                "realtime_cpu and realtime_priority "
                "must be configured together",
                runtimeNode));
        }

        auto result = BackendRuntimeHostConfiguration{};
        if (cpuNode != nullptr) {
            const auto cpu =
                toml_configuration::integer(
                    *runtime, "realtime_cpu", path);
            const auto priority =
                toml_configuration::integer(
                    *runtime, "realtime_priority", path);
            if (!cpu) {
                return std::unexpected(cpu.error());
            }
            if (!priority) {
                return std::unexpected(priority.error());
            }
            if (*cpu < 0
                || *cpu
                    > std::numeric_limits<
                        std::uint32_t>::max()) {
                return std::unexpected(
                    toml_configuration::error(
                        path, "runtime.realtime_cpu",
                        "must be a non-negative 32-bit "
                        "CPU index",
                        cpuNode));
            }
            if (*priority < 1 || *priority > 99) {
                return std::unexpected(
                    toml_configuration::error(
                        path,
                        "runtime.realtime_priority",
                        "must be between 1 and 99",
                        priorityNode));
            }
            result.realtimeEnabled = true;
            result.realtimeCpu =
                static_cast<std::uint32_t>(*cpu);
            result.realtimePriority =
                static_cast<int>(*priority);
        }

        if (runtime->contains("lock_memory")) {
            const auto lockMemory =
                toml_configuration::requiredBool(
                    *runtime, "lock_memory", path);
            if (!lockMemory) {
                return std::unexpected(
                    lockMemory.error());
            }
            result.lockMemory = *lockMemory;
        }
        if (result.lockMemory
            && !result.realtimeEnabled) {
            return std::unexpected(
                toml_configuration::error(
                    path, "runtime.lock_memory",
                    "requires realtime_cpu and "
                    "realtime_priority",
                    runtime->get("lock_memory")));
        }

        return result;
    }
}
