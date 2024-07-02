#pragma once

#include <memory>
#include <cmath>
#include <cassert>
#include <utility>
#include <stdexcept>

#include "parser/TokenSource.h"
#include "parser/StringTokenSource.h"

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
            LBRACE, RBRACE,
            ASSIGN,
            EQ, NE, LT, LE, GT, GE,
            PLUS, MINUS,
            MUL, SLASH, MOD, POW,
            AND, OR, XOR,
            SUB, RETURN, IF, THEN, ELSE, WHILE, CONTINUE, BREAK,
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

        static Token fromDouble(const double d, std::string name = "double") { return { Kind::NUMBER, std::make_unique<StringTokenSource>(d, std::move(name)) }; }
        static Token fromString(const Kind kind, std::string text, std::string name = "string") { return { kind, std::make_unique<StringTokenSource>(std::move(text), std::move(name)) }; }

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

        [[nodiscard]] bool isLetter() const {
            switch(m_kind) {
                using enum Kind;
                case A: case B: case C: case D: case F: case G: case H: case I: case J: case K: case L: case M: case N: case O: case P: case Q: case R: case S: case T: case X: case Y: case Z:
                    return true;
                default:
                    return false;
            }
        }

        [[nodiscard]] std::string location() const {
            return m_source->location();
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

            auto result = fromChars(text());

            if(!result) {
                throw std::logic_error(std::format("StringTokenSource::{}(): std::from_chars() failed on '{}'", __func__, text()));
            }

            return *result;
        }

        [[nodiscard]] int as_integer() const {
            if(!integer()) {
                throw std::logic_error(std::format("Token::as_integer() called on Token {} '{}'", name(), text()));
            }

            return static_cast<int>(as_double());
        }

        [[nodiscard]] const char *name() const;
    };

    inline const char *name(const Token::Kind kind) {
        switch (kind) {
            case Token::Kind::NONE: return "NONE";
            case Token::Kind::A: return "A";
            case Token::Kind::B: return "B";
            case Token::Kind::C: return "C";
            case Token::Kind::D: return "D";
            case Token::Kind::F: return "F";
            case Token::Kind::G: return "G";
            case Token::Kind::H: return "H";
            case Token::Kind::I: return "I";
            case Token::Kind::J: return "J";
            case Token::Kind::K: return "K";
            case Token::Kind::L: return "L";
            case Token::Kind::M: return "M";
            case Token::Kind::N: return "N";
            case Token::Kind::O: return "O";
            case Token::Kind::P: return "P";
            case Token::Kind::Q: return "Q";
            case Token::Kind::R: return "R";
            case Token::Kind::S: return "S";
            case Token::Kind::T: return "T";
            case Token::Kind::X: return "X";
            case Token::Kind::Y: return "Y";
            case Token::Kind::Z: return "Z";
            case Token::Kind::IDENTIFIER: return "IDENTIFIER";
            case Token::Kind::COMMA: return "COMMA";
            case Token::Kind::NAMED_VARIABLE: return "NAMED_VARIABLE";
            case Token::Kind::NUMBER: return "NUMBER";
            case Token::Kind::STRING: return "STRING";
            case Token::Kind::POUND: return "POUND";
            case Token::Kind::AMPERSAND: return "AMPERSAND";
            case Token::Kind::PERCENT: return "PERCENT";
            case Token::Kind::COMMENT: return "COMMENT";
            case Token::Kind::NEWLINE: return "NEWLINE";
            case Token::Kind::LBRACKET: return "LBRACKET";
            case Token::Kind::RBRACKET: return "RBRACKET";
            case Token::Kind::LBRACE: return "LBRACE";
            case Token::Kind::RBRACE: return "RBRACE";
            case Token::Kind::ASSIGN: return "ASSIGN";
            case Token::Kind::EQ: return "EQ";
            case Token::Kind::NE: return "NE";
            case Token::Kind::LT: return "LT";
            case Token::Kind::LE: return "LE";
            case Token::Kind::GT: return "GT";
            case Token::Kind::GE: return "GE";
            case Token::Kind::PLUS: return "PLUS";
            case Token::Kind::MINUS: return "MINUS";
            case Token::Kind::MUL: return "MUL";
            case Token::Kind::SLASH: return "SLASH";
            case Token::Kind::MOD: return "MOD";
            case Token::Kind::POW: return "POW";
            case Token::Kind::AND: return "AND";
            case Token::Kind::OR: return "OR";
            case Token::Kind::XOR: return "XOR";
            case Token::Kind::SUB: return "SUB";
            case Token::Kind::RETURN: return "RETURN";
            case Token::Kind::IF: return "IF";
            case Token::Kind::ELSE: return "ELSE";
            case Token::Kind::WHILE: return "WHILE";
            case Token::Kind::CONTINUE: return "CONTINUE";
            case Token::Kind::BREAK: return "BREAK";
            case Token::Kind::ALIAS: return "ALIAS";
            case Token::Kind::LET: return "LET";
            default: throw std::logic_error(std::format("Token::name() missing case statement for {}", std::to_underlying(kind)));
        }
    }

    inline const char *Token::name() const {
        return ngc::name(m_kind);
    }
}
