#pragma once

#include <expected>
#include <filesystem>
#include <map>
#include <optional>
#include <string>

namespace ngc {
    class ToolTable {
    public:
        static constexpr std::string_view FILENAME = "tool_table.txt";

        struct tool_entry_t {
            int number;
            double x, y, z, a, b, c;
            double diameter;
            std::string comment;

            std::string text() const;
        };

        using const_iterator = std::map<int, tool_entry_t>::const_iterator;

        const_iterator begin() const;
        const_iterator end() const;
        std::optional<tool_entry_t> get(int num) const;
        void set(int num, const tool_entry_t &tool);
        std::expected<void, std::string> load(const std::filesystem::path &path = FILENAME);
        std::expected<void, std::string> save(const std::filesystem::path &path = FILENAME) const;

    private:
        std::map<int, tool_entry_t> m_tools;
    };
}
