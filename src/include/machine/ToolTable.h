#pragma once

#include <compare>
#include <expected>
#include <filesystem>
#include <map>
#include <optional>
#include <string>

namespace ngc {
    struct ToolTableStorePaths {
        std::filesystem::path legacy = "tool_table.txt";
        std::filesystem::path real = "real_tool_table.txt";
        std::filesystem::path simulation = "simulation_tool_table.txt";
    };

    class ToolTable {
    public:
        static constexpr std::string_view FILENAME = "tool_table.txt";

        struct tool_entry_t {
            int number;
            double x, y, z, a, b, c;
            double diameter;
            std::string comment;

            std::string text() const;
            auto operator<=>(const tool_entry_t &) const = default;
        };

        using const_iterator = std::map<int, tool_entry_t>::const_iterator;

        const_iterator begin() const;
        const_iterator end() const;
        std::optional<tool_entry_t> get(int num) const;
        void set(int num, const tool_entry_t &tool);
        std::expected<void, std::string> load(const std::filesystem::path &path = FILENAME);
        std::expected<void, std::string> save(const std::filesystem::path &path = FILENAME) const;
        bool operator==(const ToolTable &) const = default;

    private:
        std::map<int, tool_entry_t> m_tools;
    };

    std::expected<void, std::string> migrateLegacyToolTables(const ToolTableStorePaths &paths);
}
