#pragma once

#include <optional>
#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <print>
#include <format>
#include <map>
#include <sstream>
#include <utility>
#include <cctype>

#include "utils.h"

namespace ngc {
    class ToolTable {
    public:
        static constexpr std::string_view FILENAME = "tool_table.txt";

        struct tool_entry_t {
            int number;
            double x, y, z, a, b, c;
            double diameter;
            std::string comment;

            std::string text() const { return std::format("T{} X{} Y{} Z{} A{} B{} C{} DIAMETER: {} COMMENT: \"{}\"", number, x, y, z, a, b, c, diameter, comment); }
        };

    private:
        std::map<int, tool_entry_t> m_tools;

    public:
        auto begin() const { return m_tools.begin(); }
        auto end() const { return m_tools.end(); }

        std::optional<tool_entry_t> get(int num) const {
            if(!m_tools.contains(num)) {
                return std::nullopt;
            }

            return m_tools.at(num);
        }

        void set(int num, const tool_entry_t &tool) {
            m_tools.insert_or_assign(num, tool);
        }

        std::expected<void, std::string> load(const std::filesystem::path &path = FILENAME) {
            m_tools.clear();
            const auto result = readFile(path);

            if(!result) {
                return std::unexpected(result.error().what());
            }

            std::istringstream input(*result);
            std::string line;
            std::size_t row = 0;
            while(std::getline(input, line)) {
                ++row;
                if(line.find_first_not_of(" \t\r") == std::string::npos) continue;

                tool_entry_t tool{};
                std::istringstream fields(line);
                if(!(fields >> tool.number >> tool.x >> tool.y >> tool.z
                     >> tool.a >> tool.b >> tool.c >> tool.diameter)) {
                    return std::unexpected(std::format("{}: row:{} expected 8 numeric values", path.string(), row));
                }

                std::getline(fields >> std::ws, tool.comment);
                const auto toolNumber = tool.number;
                const auto [_, inserted] = m_tools.emplace(toolNumber, std::move(tool));
                if(!inserted) {
                    return std::unexpected(std::format("{}: row:{} duplicate tool number {}",
                                                       path.string(), row, toolNumber));
                }
            }

            return {};
        }

        std::expected<void, std::string> save(const std::filesystem::path &path = FILENAME) const {
            std::string text;

            for(const auto &[_, tool] : m_tools) {
                text += std::format("{} {} {} {} {} {} {} {} {}\n", tool.number, tool.x, tool.y, tool.z, tool.a, tool.b, tool.c, tool.diameter, tool.comment);
            }

            auto result = writeFile(path, text);

            if(!result) {
                return std::unexpected(result.error().what());
            }

            return {};
        }
    };
}
