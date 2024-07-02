#pragma once

#include <string>
#include <string_view>
#include <utility>
#include <stack>
#include <print>

namespace ngc
{
    class LexerSource {
        struct state_t {
            size_t index;
            int line;
            int col;
        };

        std::string m_text;
        std::string m_name;
        std::stack<state_t> m_state;

        [[nodiscard]] const state_t &state() const { return m_state.top(); }
        state_t &state() { return m_state.top(); }

    public:
        LexerSource(const LexerSource &) = delete;
        LexerSource(LexerSource &&)  noexcept = default;
        LexerSource &operator=(const LexerSource &) = delete;
        LexerSource &operator=(LexerSource &&) = default;

        LexerSource(std::string text, std::string name) : m_text(std::move(text)), m_name(std::move(name)), m_state(std::initializer_list<state_t> {{ 0, 1, 1 }}) { }

        void reset() {
            while(!m_state.empty()) {
                m_state.pop();
            }

            m_state.emplace(state_t { 0, 1, 1 });
        }

        void pushState() {
            m_state.push(m_state.top());
        }

        void popState() {
            m_state.pop();
        }

        [[nodiscard]] const std::string &name() const {
            return m_name;
        }

        [[nodiscard]] size_t index() const {
            return state().index;
        }

        [[nodiscard]] int line() const {
            return state().line;
        }

        [[nodiscard]] int col() const {
            return state().col;
        }

        [[nodiscard]] char peek() const {
            return state().index < m_text.size() ? m_text[state().index] : '\0';
        }

        [[nodiscard]] char next() {
            if(state().index < m_text.size()) {
                const char c  = m_text[state().index++];
                visit(c);
                return c;
            }

            return '\0';
        }

        [[nodiscard]] char prev() const {
            return state().index > 0 ? m_text[state().index - 1] : '\0';
        }

        void advance() {
            if(state().index < m_text.size()) {
                visit(m_text[state().index++]);
            }
        }

        [[nodiscard]] bool end() const {
            return state().index >= m_text.size();
        }

        [[nodiscard]] std::string_view text() const {
            return m_text;
        }

        [[nodiscard]] std::string_view text(const size_t start, const size_t end) const {
            return std::string_view(m_text).substr(start, end - start);
        }

    private:
        void visit(const char c) {
            if(c == '\n') {
                state().line++;
                state().col = 0;
            }

            state().col++;
        }
    };
}
