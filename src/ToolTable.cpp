#include "machine/ToolTable.h"

#include <cmath>
#include <format>
#include <locale>
#include <sstream>
#include <utility>

#include "utils.h"

namespace ngc {
    std::string ToolTable::tool_entry_t::text() const {
        return std::format("T{} X{} Y{} Z{} A{} B{} C{} DIAMETER: {} COMMENT: \"{}\"",
                           number, x, y, z, a, b, c, diameter, comment);
    }

    ToolTable::const_iterator ToolTable::begin() const { return m_tools.begin(); }
    ToolTable::const_iterator ToolTable::end() const { return m_tools.end(); }

    std::optional<ToolTable::tool_entry_t> ToolTable::get(const int num) const {
        const auto match = m_tools.find(num);
        return match == m_tools.end() ? std::nullopt : std::optional { match->second };
    }

    void ToolTable::set(const int num, const tool_entry_t &tool) { m_tools.insert_or_assign(num, tool); }

    std::expected<void, std::string> ToolTable::load(const std::filesystem::path &path) {
        const auto result = readFile(path);
        if (!result) {
            return std::unexpected(result.error().what());
        }

        std::istringstream input(*result);
        input.imbue(std::locale::classic());
        std::string line;
        std::size_t row = 0;
        std::map<int, tool_entry_t> tools;
        while (std::getline(input, line)) {
            ++row;
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (line.find_first_not_of(" \t\r") == std::string::npos) {
                continue;
            }

            tool_entry_t tool{};
            std::istringstream fields(line);
            fields.imbue(std::locale::classic());
            if (!(fields >> tool.number >> tool.x >> tool.y >> tool.z
                  >> tool.a >> tool.b >> tool.c >> tool.diameter)) {
                return std::unexpected(std::format("{}: row:{} expected 8 numeric values", path.string(), row));
            }
            if (tool.number <= 0) {
                return std::unexpected(std::format(
                    "{}: row:{} tool number must be positive", path.string(), row));
            }
            if (!std::isfinite(tool.x) || !std::isfinite(tool.y) || !std::isfinite(tool.z)
                || !std::isfinite(tool.a) || !std::isfinite(tool.b) || !std::isfinite(tool.c)
                || !std::isfinite(tool.diameter) || tool.diameter < 0.0) {
                return std::unexpected(std::format(
                    "{}: row:{} tool offsets must be finite and diameter must be finite and non-negative",
                    path.string(), row));
            }

            std::getline(fields >> std::ws, tool.comment);
            const auto toolNumber = tool.number;
            const auto [_, inserted] = tools.emplace(toolNumber, std::move(tool));
            if (!inserted) {
                return std::unexpected(std::format("{}: row:{} duplicate tool number {}",
                                                   path.string(), row, toolNumber));
            }
        }

        m_tools = std::move(tools);
        return {};
    }

    std::expected<void, std::string> ToolTable::save(const std::filesystem::path &path) const {
        std::string text;
        for (const auto &[number, tool] : m_tools) {
            if (number != tool.number || tool.number <= 0
                || !std::isfinite(tool.x) || !std::isfinite(tool.y)
                || !std::isfinite(tool.z) || !std::isfinite(tool.a) || !std::isfinite(tool.b)
                || !std::isfinite(tool.c) || !std::isfinite(tool.diameter)
                || tool.diameter < 0.0
                || tool.comment.find_first_of("\r\n") != std::string::npos) {
                return std::unexpected(std::format(
                    "cannot save invalid tool-table entry {}", tool.number));
            }
            text += std::format("{} {} {} {} {} {} {} {}", tool.number, tool.x, tool.y, tool.z,
                                tool.a, tool.b, tool.c, tool.diameter);
            if (!tool.comment.empty()) {
                text += " " + tool.comment;
            }
            text += '\n';
        }

        if (auto result = writeFile(path, text); !result) {
            return std::unexpected(result.error().what());
        }

        return {};
    }

    std::expected<void, std::string> migrateLegacyToolTables(const ToolTableStorePaths &paths) {
        std::error_code realError;
        std::error_code simulationError;
        std::error_code legacyError;
        const auto realExists = std::filesystem::exists(paths.real, realError);
        const auto simulationExists = std::filesystem::exists(paths.simulation, simulationError);
        const auto legacyExists = std::filesystem::exists(paths.legacy, legacyError);
        if (realError || simulationError || legacyError) {
            const auto message = realError ? realError.message()
                : simulationError ? simulationError.message() : legacyError.message();
            return std::unexpected("failed to inspect tool-table stores: " + message);
        }
        if (realExists != simulationExists) {
            return std::unexpected(
                "only one isolated tool-table store exists; refusing ambiguous legacy migration");
        }
        if (realExists) {
            return {};
        }
        if (!legacyExists) {
            return std::unexpected(std::format(
                "neither isolated tool tables nor legacy tool table '{}' exist",
                paths.legacy.string()));
        }

        ToolTable legacy;
        if (auto loaded = legacy.load(paths.legacy); !loaded) {
            return std::unexpected(loaded.error());
        }
        if (auto saved = legacy.save(paths.real); !saved) {
            return std::unexpected(std::format(
                "failed to seed Real tool table: {}", saved.error()));
        }
        if (auto saved = legacy.save(paths.simulation); !saved) {
            return std::unexpected(std::format(
                "failed to seed Simulation tool table after Real was seeded: {}", saved.error()));
        }

        return {};
    }
}
