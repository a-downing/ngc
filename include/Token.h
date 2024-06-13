#ifndef TOKEN_H
#define TOKEN_H

#include <memory>
#include <cmath>
#include <cassert>
#include <charconv>
#include <utility>
#include <stdexcept>

#include <TokenSource.h>
#include <StringTokenSource.h>

namespace ngc {
    class Token {
    public:
        enum class Kind {
            NONE,
            A, B, C, D, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, X, Y, Z,
            IDENTIFIER,
            COMMA,
            NAMED_VARIABLE,
            NUMBER,
            STRING,
            POUND,
            AMPERSAND,
            PERCENT,
            COMMENT,
            NEWLINE,
            LBRACKET, RBRACKET,
            ASSIGN,
            EQ, NE, LT, LE, GT, GE,
            PLUS, MINUS,
            MUL, SLASH, MOD, POW,
            AND, OR, XOR,
            SUB, ENDSUB, RETURN,
            IF, ELSE, ENDIF,
            WHILE, CONTINUE, BREAK, ENDWHILE,
            ALIAS, LET
        };

    private:
        Kind m_kind;
        std::unique_ptr<TokenSource> m_source;

    public:
        Token(): m_kind(Kind::NONE), m_source(std::make_unique<StringTokenSource>("none", "none")) { }
        Token(const Kind kind, std::unique_ptr<TokenSource> source): m_kind(kind), m_source(std::move(source)) { }
        Token(const Token &t) : m_kind(t.m_kind), m_source(t.m_source->clone()) { }
        Token(Token &&) = default;

        Token &operator=(const Token &t) {
            m_kind = t.m_kind;
            m_source = t.m_source->clone();
            return *this;
        }

        [[nodiscard]] Kind kind() const { return m_kind; }
        [[nodiscard]] const TokenSource *source() const { return m_source.get(); }
        [[nodiscard]] std::string_view text() const { return m_source->text(); }
        [[nodiscard]] bool number() const { return m_kind == Kind::NUMBER; }
        [[nodiscard]] bool is(const Kind kind) const { return m_kind == kind; }

        [[nodiscard]] std::string location() const {
            return std::format("{}:{}:{}", m_source->name(), m_source->line(), m_source->col());
        }

        [[nodiscard]] std::string_view value() const {
            switch (m_kind) {
                case Kind::NUMBER:
                case Kind::IDENTIFIER: return text();
                case Kind::NAMED_VARIABLE: return text().substr(1, text().size() - 1);
                case Kind::COMMENT:
                case Kind::STRING: return text().substr(1, text().size() - 2);
                default: throw std::logic_error(std::format("Token::value() called on non-value type Token {} '{}'", name(), text()));
            }
        }

        [[nodiscard]] bool integer() const {
            if(!number()) {
                return false;
            }

            auto d = as_double();
            return std::abs(std::round(d) - d) <= 0.0001;
        }

        [[nodiscard]] double as_double() const {
            if(m_kind != Kind::NUMBER) {
                throw std::logic_error(std::format("Token::as_double() called on Token {} '{}'", name(), text()));
            }

            double d;
            auto [ptr, ec] = std::from_chars(text().begin(), text().end(), d);

            if(ec != std::errc()) {
                throw std::logic_error(std::format("Token::as_double(): std::from_chars() failed on '{}'", text()));
            }

            return d;
        }

        [[nodiscard]] int as_integer() const {
            if(!integer()) {
                throw std::logic_error(std::format("Token::as_integer() called on Token {} '{}'", name(), text()));
            }

            return static_cast<int>(as_double());
        }

        [[nodiscard]] const char *name() const {
            switch (m_kind) {
            case Kind::NONE: return "NONE";
            case Kind::A: return "A";
            case Kind::B: return "B";
            case Kind::C: return "C";
            case Kind::D: return "D";
            case Kind::F: return "F";
            case Kind::G: return "G";
            case Kind::H: return "H";
            case Kind::I: return "I";
            case Kind::J: return "J";
            case Kind::K: return "K";
            case Kind::L: return "L";
            case Kind::M: return "M";
            case Kind::N: return "N";
            case Kind::O: return "O";
            case Kind::P: return "P";
            case Kind::Q: return "Q";
            case Kind::R: return "R";
            case Kind::S: return "S";
            case Kind::T: return "T";
            case Kind::X: return "X";
            case Kind::Y: return "Y";
            case Kind::Z: return "Z";
            case Kind::IDENTIFIER: return "IDENTIFIER";
            case Kind::COMMA: return "COMMA";
            case Kind::NAMED_VARIABLE: return "NAMED_VARIABLE";
            case Kind::NUMBER: return "NUMBER";
            case Kind::STRING: return "STRING";
            case Kind::POUND: return "POUND";
            case Kind::AMPERSAND: return "AMPERSAND";
            case Kind::PERCENT: return "PERCENT";
            case Kind::COMMENT: return "COMMENT";
            case Kind::NEWLINE: return "NEWLINE";
            case Kind::LBRACKET: return "LBRACKET";
            case Kind::RBRACKET: return "RBRACKET";
            case Kind::ASSIGN: return "ASSIGN";
            case Kind::EQ: return "EQ";
            case Kind::NE: return "NE";
            case Kind::LT: return "LT";
            case Kind::LE: return "LE";
            case Kind::GT: return "GT";
            case Kind::GE: return "GE";
            case Kind::PLUS: return "PLUS";
            case Kind::MINUS: return "MINUS";
            case Kind::MUL: return "MUL";
            case Kind::SLASH: return "SLASH";
            case Kind::MOD: return "MOD";
            case Kind::POW: return "POW";
            case Kind::AND: return "AND";
            case Kind::OR: return "OR";
            case Kind::XOR: return "XOR";
            case Kind::SUB: return "SUB";
            case Kind::ENDSUB: return "ENDSUB";
            case Kind::RETURN: return "RETURN";
            case Kind::IF: return "IF";
            case Kind::ELSE: return "ELSE";
            case Kind::ENDIF: return "ENDIF";
            case Kind::WHILE: return "WHILE";
            case Kind::CONTINUE: return "CONTINUE";
            case Kind::BREAK: return "BREAK";
            case Kind::ENDWHILE: return "ENDWHILE";
            case Kind::ALIAS: return "ALIAS";
            case Kind::LET: return "LET";
            default: throw std::logic_error(std::format("Token::name() missing case statement for {} '{}'", std::to_underlying(m_kind), text()));
            }
        }
    };
}

#endif //TOKEN_H
