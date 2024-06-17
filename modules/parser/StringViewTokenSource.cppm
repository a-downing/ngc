module;

#include <memory>
#include <string_view>

export module parser:StringViewTokenSource;
import :TokenSource;

namespace ngc {
    class StringViewTokenSource final : public TokenSource {
        std::string_view m_text;
        std::string_view m_name;
        size_t m_start;
        size_t m_end;
        int m_line;
        int m_col;

    public:
        StringViewTokenSource(const std::string_view text, const std::string_view name, const size_t start, const size_t end, const int line, const int col) : m_text(text), m_name(name), m_start(start), m_end(end), m_line(line), m_col(col) { }
        [[nodiscard]] std::unique_ptr<TokenSource> clone() const override { return std::make_unique<StringViewTokenSource>(*this); }
        [[nodiscard]] std::string_view text() const override { return m_text.substr(m_start, m_end - m_start); }
        [[nodiscard]] std::string_view name() const override { return m_name; }
        [[nodiscard]] int line() const override { return m_line; }
        [[nodiscard]] int col() const override { return m_col; }

        [[nodiscard]] std::string location() const override {
            return std::format("{}:{}:{}", name(), line(), col());
        }
    };
}
