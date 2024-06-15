#ifndef GCODE_H
#define GCODE_H

#include <optional>
#include <stdexcept>
#include <format>

#include <Token.h>
#include <Expression.h>
#include <Statement.h>

namespace ngc {
    enum class GCode {
        G0,
        G1,
        G17,
        G18,
        G19,
        G2,
        G20,
        G21,
        G3,
        G43,
        G49,
        G53,
        G54,
        G55,
        G56,
        G57,
        G58,
        G59,
        G59_1,
        G59_2,
        G59_3,
        G61_1,
        G90,
        G91,
        G93,
        G94,
    };

    inline const char *name(const GCode code) {
        switch(code) {
            case GCode::G0: return "G0";
            case GCode::G1: return "G1";
            case GCode::G17: return "G17";
            case GCode::G18: return "G18";
            case GCode::G19: return "G19";
            case GCode::G2: return "G2";
            case GCode::G20: return "G20";
            case GCode::G21: return "G21";
            case GCode::G3: return "G3";
            case GCode::G43: return "G43";
            case GCode::G49: return "G49";
            case GCode::G53: return "G53";
            case GCode::G54: return "G54";
            case GCode::G55: return "G55";
            case GCode::G56: return "G56";
            case GCode::G57: return "G57";
            case GCode::G58: return "G58";
            case GCode::G59: return "G59";
            case GCode::G59_1: return "G59.1";
            case GCode::G59_2: return "G59.2";
            case GCode::G59_3: return "G59.3";
            case GCode::G61_1: return "G61.1";
            case GCode::G90: return "G90";
            case GCode::G91: return "G91";
            case GCode::G93: return "G93";
            case GCode::G94: return "G94";
        }
    }

    enum class MCode {
        M0,
        M1,
        M2,
        M3,
        M30,
        M4,
        M5,
        M6,
    };

    inline const char *name(const MCode code) {
        switch(code) {
            case MCode::M0: return "M0";
            case MCode::M1: return "M1";
            case MCode::M2: return "M2";
            case MCode::M3: return "M3";
            case MCode::M4: return "M4";
            case MCode::M30: return "M30";
            case MCode::M5: return "M5";
            case MCode::M6: return "M6";
        }
    }

    enum class Letter {
        A, B, C, D, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, X, Y, Z
    };

    inline const char *name(const Letter letter) {
        switch(letter) {
            case Letter::A: return "A";
            case Letter::B: return "B";
            case Letter::C: return "C";
            case Letter::D: return "D";
            case Letter::F: return "F";
            case Letter::G: return "G";
            case Letter::H: return "H";
            case Letter::I: return "I";
            case Letter::J: return "J";
            case Letter::K: return "K";
            case Letter::L: return "L";
            case Letter::M: return "M";
            case Letter::N: return "N";
            case Letter::O: return "O";
            case Letter::P: return "P";
            case Letter::Q: return "Q";
            case Letter::R: return "R";
            case Letter::S: return "S";
            case Letter::T: return "T";
            case Letter::X: return "X";
            case Letter::Y: return "Y";
            case Letter::Z: return "Z";
        }
    }

    class Word {
        const WordExpression *m_expression;
        Letter m_letter;
        double m_real;

    public:
        Word(const WordExpression *expression, const Letter letter, const double real) : m_expression(expression), m_letter(letter), m_real(real) { }
        const WordExpression *expression() const { return m_expression; }
        Letter letter() const { return m_letter; }
        double real() const { return m_real; }
        std::string text() const { return std::format("{}{}", name(m_letter), m_real); }
    };

    class Block {
        const BlockStatement *m_statement;
        std::vector<Word> m_words;
    public:
        Block(const BlockStatement *statement, std::vector<Word> words) : m_statement(statement), m_words(std::move(words)) { }
        const BlockStatement *statement() { return m_statement; };
        const std::vector<Word> &words() { return m_words; }
        bool blockDelete() const { return m_statement->blockDelete() != std::nullopt; }
    };

    constexpr GCode convertGCode(double real) {
        int whole = static_cast<int>(real);
        int fract = static_cast<int>((real - whole) * 10 + 0.5);

        switch(whole) {
            case 0: return GCode::G0;
            case 1: return GCode::G1;
            case 17: return GCode::G17;
            case 18: return GCode::G18;
            case 19: return GCode::G19;
            case 2: return GCode::G2;
            case 20: return GCode::G20;
            case 21: return GCode::G21;
            case 3: return GCode::G3;
            case 43: return GCode::G43;
            case 49: return GCode::G49;
            case 53: return GCode::G53;
            case 54: return GCode::G54;
            case 55: return GCode::G55;
            case 56: return GCode::G56;
            case 57: return GCode::G57;
            case 58: return GCode::G58;
            case 59:
                switch(fract) {
                    case 0: return GCode::G59;
                    case 1: return GCode::G59_1;
                    case 2: return GCode::G59_2;
                    case 3: return GCode::G59_3;
                    default: throw std::logic_error(std::format("invalid gcode G{}", real));
                }
            case 61:
                switch(fract) {
                    case 1: return GCode::G61_1;
                    default: throw std::logic_error(std::format("invalid gcode G{}", real));
                }
            case 90: return GCode::G90;
            case 91: return GCode::G91;
            case 93: return GCode::G93;
            case 94: return GCode::G94;
            default: throw std::logic_error(std::format("invalid gcode G{}", real));
        }
    }

    constexpr MCode convertMCode(double real) {
        int whole = static_cast<int>(real);
        int fract = static_cast<int>((real - whole) * 10 + 0.5);

        switch(whole) {
            case 0: return MCode::M0;
            case 1: return MCode::M1;
            case 2: return MCode::M2;
            case 3: return MCode::M3;
            case 30: return MCode::M30;
            case 4: return MCode::M4;
            case 5: return MCode::M5;
            case 6: return MCode::M6;
            default: throw std::logic_error(std::format("invalid m-code M{}", real));
        }
    }

    constexpr Letter convertLetter(Token::Kind kind) {
        switch(kind) {
            case Token::Kind::A: return Letter::A;
            case Token::Kind::B: return Letter::B;
            case Token::Kind::C: return Letter::C;
            case Token::Kind::D: return Letter::D;
            case Token::Kind::F: return Letter::F;
            case Token::Kind::G: return Letter::G;
            case Token::Kind::H: return Letter::H;
            case Token::Kind::I: return Letter::I;
            case Token::Kind::J: return Letter::J;
            case Token::Kind::K: return Letter::K;
            case Token::Kind::L: return Letter::L;
            case Token::Kind::M: return Letter::M;
            case Token::Kind::N: return Letter::N;
            case Token::Kind::O: return Letter::O;
            case Token::Kind::P: return Letter::P;
            case Token::Kind::Q: return Letter::Q;
            case Token::Kind::R: return Letter::R;
            case Token::Kind::S: return Letter::S;
            case Token::Kind::T: return Letter::T;
            case Token::Kind::X: return Letter::X;
            case Token::Kind::Y: return Letter::Y;
            case Token::Kind::Z: return Letter::Z;
            default: throw std::logic_error(std::format("can't convert token {} to Letter", name(kind)));
        }
    }

    class MachineState {
        GCode m_modeMotion;
        GCode m_modePlane;
        GCode m_modeDistance;
        GCode m_modeFeedrate;
        GCode m_modeUnits;
        GCode m_modeToolLengthOffset;
        GCode m_modeCoordSys;
        GCode m_modePath;

        MCode m_modeStop;
        MCode m_modeToolChange;
        MCode m_modeSpindle;

        double m_A, m_B, m_C, m_D, m_F, m_H, m_I, m_J, m_K, m_L, m_M, m_N, m_O, m_P, m_Q, m_R, m_S, m_T, m_X, m_Y, m_Z;

        std::optional<GCode> m_nonModal;

    public:
        GCode modeMotion() const { return m_modeMotion; }
        GCode modePlane() const { return m_modePlane; }
        GCode modeDistance() const { return m_modeDistance; }
        GCode modeFeedrate() const { return m_modeFeedrate; }
        GCode modeUnits() const { return m_modeUnits; }
        GCode modeToolLengthOffset() const { return m_modeToolLengthOffset; }
        GCode modeCoordSys() const { return m_modeCoordSys; }
        GCode modePath() const { return m_modePath; }

        MCode modeStop() const { return m_modeStop; }
        MCode modeSpindle() const { return m_modeSpindle; }

        std::optional<GCode> nonModal() const { return m_nonModal; }

        void affectState(const Word &word) {
            if(word.letter() == Letter::G) {
                return affectState(convertGCode(word.real()));
            }

            if(word.letter() == Letter::M) {
                return affectState(convertMCode(word.real()));
            }

            switch (word.letter()) {
                case Letter::A: m_A = word.real(); break;
                case Letter::B: m_B = word.real(); break;
                case Letter::C: m_C = word.real(); break;
                case Letter::D: m_D = word.real(); break;
                case Letter::F: m_F = word.real(); break;
                case Letter::H: m_H = word.real(); break;
                case Letter::I: m_I = word.real(); break;
                case Letter::J: m_J = word.real(); break;
                case Letter::K: m_K = word.real(); break;
                case Letter::L: m_L = word.real(); break;
                case Letter::N: m_N = word.real(); break;
                case Letter::O: m_O = word.real(); break;
                case Letter::P: m_P = word.real(); break;
                case Letter::Q: m_Q = word.real(); break;
                case Letter::R: m_R = word.real(); break;
                case Letter::S: m_S = word.real(); break;
                case Letter::T: m_T = word.real(); break;
                case Letter::X: m_X = word.real(); break;
                case Letter::Y: m_Y = word.real(); break;
                case Letter::Z: m_Z = word.real(); break;
                default: std::logic_error(std::format("invalid word {}", word.text()));
            }
        }

        void affectState(const GCode code) {
            switch(code) {
                case GCode::G0:
                case GCode::G1:
                case GCode::G2:
                case GCode::G3:
                    m_modeMotion = code;
                    break;
                case GCode::G17:
                case GCode::G18:
                case GCode::G19:
                    m_modePlane = code;
                    break;
                case GCode::G90:
                case GCode::G91:
                    m_modeDistance = code;
                    break;
                case GCode::G93:
                case GCode::G94:
                    m_modeFeedrate = code;
                    break;
                case GCode::G20:
                case GCode::G21:
                    m_modeUnits = code;
                    break;
                case GCode::G43:
                case GCode::G49:
                    m_modeToolLengthOffset = code;
                    break;
                case GCode::G54:
                case GCode::G55:
                case GCode::G56:
                case GCode::G57:
                case GCode::G58:
                case GCode::G59:
                case GCode::G59_1:
                case GCode::G59_2:
                case GCode::G59_3:
                    m_modeCoordSys = code;
                    break;
                case GCode::G61_1:
                    m_modePath = code;
                    break;
                case GCode::G53:
                    m_nonModal = code;
                    break;
                default: throw std::logic_error(std::format("invalid g-code {}", name(code)));
            }
        }

        void affectState(const MCode code) {
            switch(code) {
                case MCode::M0:
                case MCode::M1:
                case MCode::M2:
                case MCode::M30:
                    m_modeStop = code;
                    break;
                case MCode::M3:
                case MCode::M4:
                case MCode::M5:
                    m_modeSpindle = code;
                    break;
                case MCode::M6:
                    m_modeToolChange = code;
                    break;
                default: throw std::logic_error(std::format("invalid m-code {}", name(code)));
            }
        }
    };
}

#endif //GCODE_H
