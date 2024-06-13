#ifndef STRINGTOKENSOURCE_H
#define STRINGTOKENSOURCE_H

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include <TokenSource.h>

namespace ngc {
    class StringTokenSource final : public TokenSource {
        std::string m_text;
        std::string m_name;

    public:
        StringTokenSource(std::string text, std::string name) : m_text(std::move(text)), m_name(std::move(name)) { }
        [[nodiscard]] std::string_view text() const override { return m_text; }
        [[nodiscard]] std::string_view name() const override { return m_name; }
        [[nodiscard]] int line() const override { return 1; }
        [[nodiscard]] int col() const override { return 1; }
        [[nodiscard]] std::unique_ptr<TokenSource> clone() const override { return std::make_unique<StringTokenSource>(m_text, m_name); }
    };
}

#endif //STRINGTOKENSOURCE_H