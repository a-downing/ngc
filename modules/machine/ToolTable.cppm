module;

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
#include <utility>
#include <cctype>

export module machine:ToolTable;
import utils;

export namespace ngc {
    class ToolTable {
    public:
        static constexpr std::string_view FILENAME = "tool_table.txt";

        struct tool_entry_t {
            int number;
            double x, y, z, a, b, c;
            double diameter;
            std::string comment;

            std::string text() { return std::format("T{} X{} Y{} Z{} A{} B{} C{} DIAMETER: {} COMMENT: \"{}\"", number, x, y, z, a, b, c, diameter, comment); }
        };

    private:
        std::map<int, const tool_entry_t> m_tools;

    public:
        std::expected<void, std::string> load() {
            m_tools.clear();
            const auto result = readFile(FILENAME);

            if(!result) {
                return std::unexpected(result.error().what());
            }

            const std::string &text = *result;
            std::string token;
            int col = 0;
            tool_entry_t tool;

            for(size_t i = 0; i < text.size(); i++) {
                const char c = text[i];

                if((!std::isspace(c) || col >= 8) && c != '\n' && i < text.size() - 1) {
                    token += c;
                    continue;
                }

                if((std::isspace(c) || i == text.size() - 1) && col < 8 && !token.empty()) {
                    auto value = fromChars(token);

                    if(!value) {
                        return std::unexpected(std::format("{}: row:{} col:{} failed to convert '{}' to number", FILENAME, m_tools.size()+1, col+1, token));
                    }

                    switch (col) {
                        case 0: tool.number = static_cast<int>(*value); break;
                        case 1: tool.x = *value; break;
                        case 2: tool.y = *value; break;
                        case 3: tool.z = *value; break;
                        case 4: tool.a = *value; break;
                        case 5: tool.b = *value; break;
                        case 6: tool.c = *value; break;
                        case 7: tool.diameter = *value; break;
                        default: std::println(stderr, "tool_table: {}", token);
                    }

                    token.clear();
                    col++;
                }

                if(c == '\n' || i == text.size() - 1) {
                    if(col == 0) {
                        continue;
                    }

                    if(col < 8) {
                        return std::unexpected(std::format("{}: row:{} expected 8 values", FILENAME, m_tools.size()+1));
                    }

                    tool.comment = std::move(token);

                    m_tools.emplace(tool.number, tool);
                    col = 0;
                    token.clear();
                }
            }

            return {};
        }

        std::expected<void, std::string> save() const {
            std::string text;

            for(const auto &[_, tool] : m_tools) {
                text += std::format("{} {} {} {} {} {} {} {} {}\n", tool.number, tool.x, tool.y, tool.z, tool.a, tool.b, tool.c, tool.diameter, tool.comment);
            }

            auto result = writeFile(FILENAME, text);

            if(!result) {
                return std::unexpected(result.error().what());
            }

            return {};
        }
    };
}