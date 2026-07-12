#include "machine/ToolTable.h"

#include <format>
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
        if(!result) return std::unexpected(result.error().what());

        std::istringstream input(*result);
        std::string line;
        std::size_t row = 0;
        std::map<int, tool_entry_t> tools;
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
            const auto [_, inserted] = tools.emplace(toolNumber, std::move(tool));
            if(!inserted) {
                return std::unexpected(std::format("{}: row:{} duplicate tool number {}",
                                                   path.string(), row, toolNumber));
            }
        }

        m_tools = std::move(tools);
        return {};
    }

    std::expected<void, std::string> ToolTable::save(const std::filesystem::path &path) const {
        std::string text;
        for(const auto &[_, tool] : m_tools) {
            text += std::format("{} {} {} {} {} {} {} {} {}\n", tool.number, tool.x, tool.y, tool.z,
                                tool.a, tool.b, tool.c, tool.diameter, tool.comment);
        }

        if(auto result = writeFile(path, text); !result) return std::unexpected(result.error().what());
        return {};
    }
}
