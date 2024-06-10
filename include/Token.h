#ifndef TOKEN_H
#define TOKEN_H

#include <cstddef>
#include <cassert>
#include <charconv>

#include <CharacterSource.h>
#include <cmath>

namespace ngc {
    class Token {
    public:
        enum class Kind {
            NONE,
            A,
            B,
            C,
            D,
            F,
            G,
            H,
            I,
            J,
            K,
            L,
            M,
            N,
            O,
            P,
            Q,
            R,
            S,
            T,
            X,
            Y,
            Z,
            IDENTIFIER,
            COMMA,
            NAMED_VARIABLE,
            NUMBER,
            POUND,
            AMPERSAND,
            PERCENT,
            COMMENT,
            NEWLINE,
            LBRACKET,
            RBRACKET,
            EQUAL,
            EQUAL_EQUAL,
            NOT_EQUAL,
            LESS,
            LESS_EQUAL,
            GREATER,
            GREATER_EQUAL,
            PLUS,
            MINUS,
            MUL,
            SLASH,
            MOD,
            POW,
            AND,
            OR,
            XOR,
            SUB,
            ENDSUB,
            CALL,
            RETURN,
            IF,
            THEN,
            ELSE,
            ENDIF,
            DO,
            WHILE,
            CONTINUE,
            BREAK,
            ENDWHILE
        };

    private:
        Kind m_kind;
        const CharacterSource *m_source;
        size_t m_start;
        size_t m_end;
        int m_line;
        int m_col;

    public:
        Token(): m_kind(Kind::NONE), m_source(nullptr), m_start(0), m_end(0), m_line(0), m_col(0) { }
        Token(const Kind kind, const CharacterSource &source, const size_t start, const size_t end, const int line, const int col): m_kind(kind), m_source(&source), m_start(start), m_end(end), m_line(line), m_col(col) { }
        Token &operator=(const Token &token) = default;

        [[nodiscard]] Kind kind() const {
            return m_kind;
        }

        [[nodiscard]] const CharacterSource *source() const {
            return m_source;
        }

        [[nodiscard]] bool is(const Kind kind) const {
            return m_kind == kind;
        }

        [[nodiscard]] size_t start() const {
            return m_start;
        }

        [[nodiscard]] size_t end() const {
            return m_end;
        }

        [[nodiscard]] std::string sourceName() const {
            assert(m_source);
            return m_source ? m_source->name() : "";
        }

        [[nodiscard]] int line() const {
            return m_line;
        }

        [[nodiscard]] int col() const {
            return m_col;
        }

        [[nodiscard]] std::string location() const {
            return std::format("{}:{}:{}", sourceName(), line(), col());
        }

        [[nodiscard]] std::string_view text() const {
            return m_start == m_end ? "" : m_source->text(m_start, m_end);
        }

        [[nodiscard]] bool number() const {
            return m_kind == Kind::NUMBER;
        }

        [[nodiscard]] bool integer() const {
            if(!number()) {
                return false;
            }

            auto d = as_double();
            return std::abs(std::round(d) - d) <= 0.0001;
        }

        [[nodiscard]] double as_double() const {
            assert(m_kind == Kind::NUMBER);
            double d;
            auto [ptr, ec] = std::from_chars(text().begin(), text().end(), d);
            assert(ec == std::errc());
            return d;
        }

        [[nodiscard]] int as_integer() const {
            assert(integer());
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
            case Kind::POUND: return "POUND";
            case Kind::AMPERSAND: return "AMPERSAND";
            case Kind::PERCENT: return "PERCENT";

            case Kind::COMMENT: return "COMMENT";
            case Kind::NEWLINE: return "NEWLINE";

            case Kind::LBRACKET: return "LBRACKET";
            case Kind::RBRACKET: return "RBRACKET";

            case Kind::EQUAL: return "EQUAL";

            case Kind::EQUAL_EQUAL: return "EQUAL_EQUAL";
            case Kind::NOT_EQUAL: return "NOT_EQUAL";
            case Kind::LESS: return "LESS";
            case Kind::LESS_EQUAL: return "LESS_EQUAL";
            case Kind::GREATER: return "GREATER";
            case Kind::GREATER_EQUAL: return "GREATER_EQUAL";

            case Kind::PLUS: return "PLUS";
            case Kind::MINUS: return "MINUS";
            case Kind::MUL: return "MUL";
            case Kind::SLASH: return "DIV";
            case Kind::MOD: return "MOD";
            case Kind::POW: return "POW";

            case Kind::AND: return "AND";
            case Kind::OR: return "OR";
            case Kind::XOR: return "XOR";

            case Kind::SUB: return "SUB";
            case Kind::ENDSUB: return "ENDSUB";
            case Kind::CALL: return "CALL";
            case Kind::RETURN: return "RETURN";

            case Kind::IF: return "IF";
            case Kind::THEN: return "THEN";
            case Kind::ELSE: return "ELSE";
            case Kind::ENDIF: return "ENDIF";

            case Kind::DO: return "DO";
            case Kind::WHILE: return "WHILE";
            case Kind::CONTINUE: return "CONTINUE";
            case Kind::BREAK: return "BREAK";
            case Kind::ENDWHILE: return "ENDWHILE";
            }

            assert(!"unhandled token name");
        }

        [[nodiscard]] std::string toString() const {
            return std::format("{}({})", name(), text());
        }
    };
}

#endif //TOKEN_H
