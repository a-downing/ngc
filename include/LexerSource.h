#ifndef LEXERSOURCE_H
#define LEXERSOURCE_H

#include <string>
#include <string_view>
#include <utility>

namespace ngc
{
    class LexerSource {
        std::string m_text;
        std::string m_name;
        size_t m_index = 0;
        int m_line = 1;
        int m_col = 1;

    public:
        LexerSource(const LexerSource &) = delete;
        LexerSource(LexerSource &&) = default;
        LexerSource &operator=(const LexerSource &) = delete;
        LexerSource &operator=(LexerSource &&) = default;

        LexerSource(std::string text, std::string name) : m_text(std::move(text)), m_name(std::move(name)) { }

        void visit(const char c) {
            if(c == '\n') {
                m_line++;
                m_col = 0;
            }

            m_col++;
        }

        [[nodiscard]] const std::string &name() const {
            return m_name;
        }

        [[nodiscard]] size_t index() const {
            return m_index;
        }

        [[nodiscard]] int line() const {
            return m_line;
        }

        [[nodiscard]] int col() const {
            return m_col;
        }

        [[nodiscard]] char peek() const {
            return m_index < m_text.size() ? m_text[m_index] : '\0';
        }

        [[nodiscard]] char next() {
            if(m_index < m_text.size()) {
                const char c  = m_text[m_index++];
                visit(c);
                return c;
            }

            return '\0';
        }

        [[nodiscard]] char prev() const {
            return m_index > 0 ? m_text[m_index - 1] : '\0';
        }

        void advance() {
            if(m_index < m_text.size()) {
                visit(m_text[m_index++]);
            }
        }

        [[nodiscard]] bool end() const {
            return m_index >= m_text.size();
        }

        [[nodiscard]] std::string_view text() const {
            return m_text;
        }

        [[nodiscard]] std::string_view text(const size_t start, const size_t end) const {
            return std::string_view(m_text).substr(start, end - start);
        }
    };
}

#endif //LEXERSOURCE_H
