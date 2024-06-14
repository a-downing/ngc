#ifndef LEXER_H
#define LEXER_H

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <cctype>
#include <format>
#include <utility>

#include <LexerSource.h>
#include <Token.h>
#include <StringViewTokenSource.h>

namespace ngc
{
    class Lexer final {
        enum class State {
            BEGIN,
            PARSING,
            END
        };

        State m_state = State::BEGIN;
        LexerSource &m_source;
        size_t m_index = 0;
        int m_line = 1;
        int m_col = 1;

    public:
        class Error final {
            const std::string m_message;
            const std::string m_sourceName;
            const int m_line;
            const int m_col;

        public:
            Error(std::string message, std::string sourceName, const int line, const int col): m_message(std::move(message)), m_sourceName(std::move(sourceName)), m_line(line), m_col(col) { }
            [[nodiscard]] const std::string &message() const { return m_message; }
            [[nodiscard]] const std::string &sourceName() const { return m_sourceName; }
            [[nodiscard]] int line() const { return m_line; }
            [[nodiscard]] int col() const { return m_col; }

            [[nodiscard]] std::string location() const {
                return std::format("{}:{}:{}", m_sourceName, m_line, m_col);
            }
        };

        explicit Lexer(LexerSource &source): m_source(source) { }

        void pushState() {
            m_source.pushState();
        }

        void popState() {
            m_source.popState();
        }

        [[nodiscard]] std::expected<Token, Error> nextToken() {
            if(m_state == State::BEGIN) {
                while(!end()) {
                    if(!std::isspace(peek())) {
                        break;
                    }

                    advance();
                }

                if(end()) {
                    m_state = State::END;
                } else {
                    m_state = State::PARSING;
                }
            }

            while(!end()) {
                if(!std::isspace(peek()) || check('\n')) {
                    break;
                }

                advance();
            }

            m_index = m_source.index();
            m_line = m_source.line();
            m_col = m_source.col();

            if (end() || m_state == State::END) {
                return makeToken(Token::Kind::NONE);
            }

            const char c = next();

            switch (c) {
            case 'A': case 'a': return letter(Token::Kind::A);
            case 'B': case 'b': return letter(Token::Kind::B);
            case 'C': case 'c': return letter(Token::Kind::C);
            case 'D': case 'd': return letter(Token::Kind::D);
            case 'F': case 'f': return letter(Token::Kind::F);
            case 'G': case 'g': return letter(Token::Kind::G);
            case 'H': case 'h': return letter(Token::Kind::H);
            case 'I': case 'i': return letter(Token::Kind::I);
            case 'J': case 'j': return letter(Token::Kind::J);
            case 'K': case 'k': return letter(Token::Kind::K);
            case 'L': case 'l': return letter(Token::Kind::L);
            case 'M': case 'm': return letter(Token::Kind::M);
            case 'N': case 'n': return letter(Token::Kind::N);
            case 'O': case 'o': return letter(Token::Kind::O);
            case 'P': case 'p': return letter(Token::Kind::P);
            case 'Q': case 'q': return letter(Token::Kind::Q);
            case 'R': case 'r': return letter(Token::Kind::R);
            case 'S': case 's': return letter(Token::Kind::S);
            case 'T': case 't': return letter(Token::Kind::T);
            case 'X': case 'x': return letter(Token::Kind::X);
            case 'Y': case 'y': return letter(Token::Kind::Y);
            case 'Z': case 'z': return letter(Token::Kind::Z);
            case '%': return makeToken(Token::Kind::PERCENT);
            case '\n': return makeToken(Token::Kind::NEWLINE);
            case '[': return makeToken(Token::Kind::LBRACKET);
            case ']': return makeToken(Token::Kind::RBRACKET);
            case '{': return makeToken(Token::Kind::LBRACE);
            case '}': return makeToken(Token::Kind::RBRACE);
            case '(': return comment();
            case '+': return makeToken(Token::Kind::PLUS);
            case '-': return makeToken(Token::Kind::MINUS);
            case '/': return makeToken(Token::Kind::SLASH);
            case ',': return makeToken(Token::Kind::COMMA);
            case '&': return makeToken(Token::Kind::AMPERSAND);
            case '"': return string();
            case '=':
                if(match('=')) {
                    return makeToken(Token::Kind::EQ);
                }

                return makeToken(Token::Kind::ASSIGN);
            case '!':
                if(match('=')) {
                    return makeToken(Token::Kind::NE);
                }
            case '<':
                if(match('=')) {
                    return makeToken(Token::Kind::LE);
                }

                return makeToken(Token::Kind::LT);
            case '>':
                if(match('=')) {
                    return makeToken(Token::Kind::GE);
                }

                return makeToken(Token::Kind::GT);
            case '*':
                if(match('*')) {
                    return makeToken(Token::Kind::POW);
                }

                return makeToken(Token::Kind::MUL);
            case '.':
                if(std::isdigit(peek())) {
                    return number(c);
                }
            case '#':
                if(std::isalpha(peek()) || check('_')) {
                    return identifier(Token::Kind::NAMED_VARIABLE);
                }

                return makeToken(Token::Kind::POUND);
            }

            if(std::isdigit(c)) {
                return number(c);
            }

            if(std::isalpha(c) || c == '_') {
                return identifier(Token::Kind::IDENTIFIER);
            }

            return std::unexpected(Error(std::format("unhandled character: '{}'", c), m_source.name(), m_line, m_col));
        }

    private:
        [[nodiscard]] std::expected<Token, Error> string() {
            while(!match('"') && !end()) {
                if(match('\'')) {
                    advance();
                }

                advance();
            }

            if(prev() != '"') {
                return std::unexpected(Error("unterminated string", m_source.name(), line(), col()));
            }


            return makeToken(Token::Kind::STRING);
        }

        [[nodiscard]] std::expected<Token, Error> name(const Token::Kind kind) {
            while(!match('>') && !end()) {
                advance();
            }

            if(prev() != '>') {
                return std::unexpected(Error("unterminated name", m_source.name(), line(), col()));
            }

            return makeToken(kind);
        }

        [[nodiscard]] std::expected<Token, Error> identifier(const Token::Kind kind) {
            while(std::isalnum(peek()) || check('_')) {
                advance();
            }

            if(kind == Token::Kind::IDENTIFIER) {
                return maybeMakeKeyword();
            }

            return makeToken(kind);
        }

        [[nodiscard]] std::expected<Token, Error> comment() {
            while(!match(')') && !end()) {
                advance();
            }

            if(prev() != ')') {
                return std::unexpected(Error("unterminated comment", m_source.name(), line(), col()));
            }

            return makeToken(Token::Kind::COMMENT);
        }

        [[nodiscard]] Token number(const char c) {
            consume_number();

            if(c != '.' && match('.')) {
                consume_number();
            }

            return makeToken(Token::Kind::NUMBER);
        }

        void consume_number() {
            while(std::isdigit(peek())) {
                advance();
            }
        }

        [[nodiscard]] std::expected<Token, Error> letter(const Token::Kind kind) {
            if(std::isalpha(peek())) {
                return identifier(Token::Kind::IDENTIFIER);
            }

            return makeToken(kind);
        }

        [[nodiscard]] Token makeToken(const Token::Kind kind) const {
            return { kind, std::make_unique<StringViewTokenSource>(m_source.text(), m_source.name(), m_index, index(), m_line, m_col) };
        }

        [[nodiscard]] bool match(const char c) {
            if(check(c)) {
                advance();
                return true;
            }

            return false;
        }

        [[nodiscard]] bool check(const char c) const {
            return peek() == c;
        }

        [[nodiscard]] char peek() const {
            return m_source.peek();
        }

        [[nodiscard]] char next() {
            return m_source.next();
        }

        [[nodiscard]] char prev() const {
            return m_source.prev();
        }

        [[nodiscard]] size_t index() const {
            return m_source.index();
        }

        [[nodiscard]] int line() const {
            return m_source.line();
        }

        [[nodiscard]] int col() const {
            return m_source.col();
        }

        void advance() {
            return m_source.advance();
        }

        [[nodiscard]] bool end() const {
            return m_source.end();
        }

        [[nodiscard]] std::expected<Token, Error> maybeMakeKeyword() const {
            const auto name = m_source.text(m_index, index());

            if(iequal(name, "and")) {
                return makeToken(Token::Kind::AND);
            }

            if(iequal(name, "or")) {
                return makeToken(Token::Kind::OR);
            }

            if(iequal(name, "xor")) {
                return makeToken(Token::Kind::XOR);
            }

            if(iequal(name, "mod")) {
                return makeToken(Token::Kind::MOD);
            }

            if(iequal(name, "sub")) {
                return makeToken(Token::Kind::SUB);
            }

            if(iequal(name, "return")) {
                return makeToken(Token::Kind::RETURN);
            }

            if(iequal(name, "if")) {
                return makeToken(Token::Kind::IF);
            }

            if(iequal(name, "else")) {
                return makeToken(Token::Kind::ELSE);
            }

            if(iequal(name, "while")) {
                return makeToken(Token::Kind::WHILE);
            }

            if(iequal(name, "continue")) {
                return makeToken(Token::Kind::CONTINUE);
            }

            if(iequal(name, "break")) {
                return makeToken(Token::Kind::BREAK);
            }

            if(iequal(name, "alias")) {
                return makeToken(Token::Kind::ALIAS);
            }

            if(iequal(name, "let")) {
                return makeToken(Token::Kind::LET);
            }

            return makeToken(Token::Kind::IDENTIFIER);
        }

        [[nodiscard]] static bool iequal(const std::string_view a, const std::string_view b) {
            return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](const char _a, const char _b) {
                return std::tolower(_a) == std::tolower(_b);
            });
        }
    };
}

#endif //LEXER_H
