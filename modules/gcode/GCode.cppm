module;

#include <utility>
#include <vector>
#include <optional>
#include <stdexcept>
#include <format>
#include <print>

export module gcode;
import parser;

export namespace ngc {
#include "gcode.gen.h"
#include "mcode.gen.h"
}

export namespace ngc {
    inline GCode coordsys(const int i) {
        switch(i) {
        case 1: return GCode::G54;
        case 2: return GCode::G55;
        case 3: return GCode::G56;
        case 4: return GCode::G57;
        case 5: return GCode::G58;
        case 6: return GCode::G59;
        case 7: return GCode::G59_1;
        case 8: return GCode::G59_2;
        case 9: return GCode::G59_3;
        default: throw std::logic_error(std::format("invalid coordinate system number: {}", i));
        }
    }

    inline int coordsys(const GCode code) {
        switch(code) {
        case GCode::G54: return 1;
        case GCode::G55: return 2;
        case GCode::G56: return 3;
        case GCode::G57: return 4;
        case GCode::G58: return 5;
        case GCode::G59: return 6;
        case GCode::G59_1: return 7;
        case GCode::G59_2: return 8;
        case GCode::G59_3: return 9;
        default: throw std::logic_error(std::format("invalid coordinate system: {}", name(code)));
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
            default: throw std::runtime_error(std::format("{}() missing case statement for Letter::{}", __func__, std::to_underlying(letter)));
        }
    }

    class Word {
        const WordExpression *m_expression;
        Letter m_letter;
        double m_real;

    public:
        Word(const WordExpression *expression, const Letter letter, const double real) : m_expression(expression), m_letter(letter), m_real(real) { }
        [[nodiscard]] const WordExpression *expression() const { return m_expression; }
        [[nodiscard]] Letter letter() const { return m_letter; }
        [[nodiscard]] double real() const { return m_real; }
        [[nodiscard]] std::string text() const { return std::format("{}{}", name(m_letter), m_real); }
    };

    class Block {
        const BlockStatement *m_statement;
        std::vector<Word> m_words;
    public:
        Block(const BlockStatement *statement, std::vector<Word> words) : m_statement(statement), m_words(std::move(words)) { }
        [[nodiscard]] const BlockStatement *statement() const { return m_statement; };
        [[nodiscard]] const std::vector<Word> &words() const { return m_words; }
        [[nodiscard]] bool blockDelete() const { return m_statement->blockDelete() != std::nullopt; }
    };

    constexpr GCode convertGCode(double real) {
        const int whole = static_cast<int>(real);
        const int fract = static_cast<int>((real - whole) * 10 + 0.5);

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

    constexpr MCode convertMCode(const double real) {
        switch(static_cast<int>(real)) {
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

    constexpr Letter convertLetter(const Token::Kind kind) {
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

    struct GCodeStateDifference {
        std::optional<GCMotion> modeMotion;
        std::optional<GCPlane> modePlane;
        std::optional<GCDist> modeDistance;
        std::optional<GCFeed> modeFeedrate;
        std::optional<GCUnits> modeUnits;
        std::optional<GCTLen> modeToolLengthOffset;
        std::optional<GCCoord> modeCoordSys;
        std::optional<GCPath> modePath;

        std::optional<GCNonModal> nonModal;

        std::optional<MCSpindle> modeSpindle;
        std::optional<MCStop> modeStop;
        std::optional<MCToolChange> modeToolChange;

        std::optional<double> A, B, C, D, F, H, I, J, K, L, P, Q, R, S, T, X, Y, Z;
    };

    class GCodeState {
        GCMotion m_modeMotion{};
        GCPlane m_modePlane{};
        GCDist m_modeDistance{};
        GCFeed m_modeFeedrate{};
        GCUnits m_modeUnits{};
        GCTLen m_modeToolLengthOffset{};
        GCCoord m_modeCoordSys{};
        GCPath m_modePath{};

        std::optional<GCNonModal> m_nonModal{};

        MCSpindle m_modeSpindle{};
        std::optional<MCStop> m_modeStop{};
        std::optional<MCToolChange> m_modeToolChange{};

        double m_F = 0;
        double m_T = 0;
        double m_S = 0;
        std::optional<double> m_A, m_B, m_C, m_D, m_H, m_I, m_J, m_K, m_L, m_P, m_Q, m_R, m_X, m_Y, m_Z;

    public:
        GCodeState() {
            affectState(GCode::G0);
            affectState(GCode::G17);
            affectState(GCode::G90);
            affectState(GCode::G94);
            affectState(GCode::G20);
            affectState(GCode::G49);
            affectState(GCode::G54);
            affectState(GCode::G61_1);
            affectState(MCode::M5);
        }

        GCodeStateDifference calculateDifference(const GCodeState &s) const {
            GCodeStateDifference diff = {};

            if(s.m_modeMotion != m_modeMotion) { diff.modeMotion = s.m_modeMotion; }
            if(s.m_modePlane != m_modePlane) { diff.modePlane = s.m_modePlane; }
            if(s.m_modeDistance != m_modeDistance) { diff.modeDistance = s.m_modeDistance; }
            if(s.m_modeFeedrate != m_modeFeedrate) { diff.modeFeedrate = s.m_modeFeedrate; }
            if(s.m_modeUnits != m_modeUnits) { diff.modeUnits = s.m_modeUnits; }
            if(s.m_modeToolLengthOffset != m_modeToolLengthOffset) { diff.modeToolLengthOffset = s.m_modeToolLengthOffset; }
            if(s.m_modeCoordSys != m_modeCoordSys) { diff.modeCoordSys = s.m_modeCoordSys; }
            if(s.m_modePath != m_modePath) { diff.modePath = s.m_modePath; }

            if(s.m_nonModal != m_nonModal) { diff.nonModal = s.m_nonModal; }

            if(s.m_modeSpindle != m_modeSpindle) { diff.modeSpindle = s.m_modeSpindle; }
            if(s.m_modeStop != m_modeStop) { diff.modeStop = s.m_modeStop; }
            if(s.m_modeToolChange != m_modeToolChange) { diff.modeToolChange = s.m_modeToolChange; }

            if(s.m_A != m_A) { diff.A = s.m_A; }
            if(s.m_B != m_B) { diff.B = s.m_B; }
            if(s.m_C != m_C) { diff.C = s.m_C; }
            if(s.m_D != m_D) { diff.D = s.m_D; }
            if(s.m_F != m_F) { diff.F = s.m_F; }
            if(s.m_H != m_H) { diff.H = s.m_H; }
            if(s.m_I != m_I) { diff.I = s.m_I; }
            if(s.m_J != m_J) { diff.J = s.m_J; }
            if(s.m_K != m_K) { diff.K = s.m_K; }
            if(s.m_L != m_L) { diff.L = s.m_L; }
            if(s.m_P != m_P) { diff.P = s.m_P; }
            if(s.m_Q != m_Q) { diff.Q = s.m_Q; }
            if(s.m_R != m_R) { diff.R = s.m_R; }
            if(s.m_S != m_S) { diff.S = s.m_S; }
            if(s.m_T != m_T) { diff.T = s.m_T; }
            if(s.m_X != m_X) { diff.X = s.m_X; }
            if(s.m_Y != m_Y) { diff.Y = s.m_Y; }
            if(s.m_Z != m_Z) { diff.Z = s.m_Z; }

            return diff;
        }

        void resetModal() {
            m_nonModal = std::nullopt;
            m_modeStop = std::nullopt;
            m_modeToolChange = std::nullopt;
            m_A = m_B = m_C = m_D = m_H = m_I = m_J = m_K = m_L = m_P = m_Q = m_R = m_X = m_Y = m_Z = std::nullopt;
        }

        [[nodiscard]] GCMotion modeMotion() const { return m_modeMotion; }
        [[nodiscard]] GCPlane modePlane() const { return m_modePlane; }
        [[nodiscard]] GCDist modeDistance() const { return m_modeDistance; }
        [[nodiscard]] GCFeed modeFeedrate() const { return m_modeFeedrate; }
        [[nodiscard]] GCUnits modeUnits() const { return m_modeUnits; }
        [[nodiscard]] GCTLen modeToolLengthOffset() const { return m_modeToolLengthOffset; }
        [[nodiscard]] GCCoord modeCoordSys() const { return m_modeCoordSys; }
        [[nodiscard]] GCPath modePath() const { return m_modePath; }

        [[nodiscard]] std::optional<double> A() const { return m_A; }
        [[nodiscard]] std::optional<double> B() const { return m_B; }
        [[nodiscard]] std::optional<double> C() const { return m_C; }
        [[nodiscard]] std::optional<double> D() const { return m_D; }
        [[nodiscard]] double F() const { return m_F; }
        [[nodiscard]] std::optional<double> H() const { return m_H; }
        [[nodiscard]] std::optional<double> I() const { return m_I; }
        [[nodiscard]] std::optional<double> J() const { return m_J; }
        [[nodiscard]] std::optional<double> K() const { return m_K; }
        [[nodiscard]] std::optional<double> L() const { return m_L; }
        [[nodiscard]] std::optional<double> P() const { return m_P; }
        [[nodiscard]] std::optional<double> Q() const { return m_Q; }
        [[nodiscard]] std::optional<double> R() const { return m_R; }
        [[nodiscard]] double S() const { return m_S; }
        [[nodiscard]] double T() const { return m_T; }
        [[nodiscard]] std::optional<double> X() const { return m_X; }
        [[nodiscard]] std::optional<double> Y() const { return m_Y; }
        [[nodiscard]] std::optional<double> Z() const { return m_Z; }

        [[nodiscard]] MCSpindle modeSpindle() const { return m_modeSpindle; }
        [[nodiscard]] std::optional<MCStop> modeStop() const { return m_modeStop; }
        [[nodiscard]] std::optional<MCToolChange> modeToolChange() const { return m_modeToolChange; }

        [[nodiscard]] std::optional<GCNonModal> nonModal() const { return m_nonModal; }

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
                case Letter::N: break;
                case Letter::P: m_P = word.real(); break;
                case Letter::Q: m_Q = word.real(); break;
                case Letter::R: m_R = word.real(); break;
                case Letter::S: m_S = word.real(); break;
                case Letter::T: m_T = word.real(); break;
                case Letter::X: m_X = word.real(); break;
                case Letter::Y: m_Y = word.real(); break;
                case Letter::Z: m_Z = word.real(); break;
                default: throw std::logic_error(std::format("invalid word {}", word.text()));
            }
        }

        void affectState(const GCode code) {
            switch(code) {
                case GCode::G0:
                case GCode::G1:
                case GCode::G2:
                case GCode::G3:
                    m_modeMotion = static_cast<GCMotion>(code);
                    break;
                case GCode::G17:
                case GCode::G18:
                case GCode::G19:
                    m_modePlane = static_cast<GCPlane>(code);
                    break;
                case GCode::G90:
                case GCode::G91:
                    m_modeDistance = static_cast<GCDist>(code);
                    break;
                case GCode::G93:
                case GCode::G94:
                    m_modeFeedrate = static_cast<GCFeed>(code);
                    break;
                case GCode::G20:
                case GCode::G21:
                    m_modeUnits = static_cast<GCUnits>(code);
                    break;
                case GCode::G43:
                case GCode::G49:
                    m_modeToolLengthOffset = static_cast<GCTLen>(code);
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
                    m_modeCoordSys = static_cast<GCCoord>(code);
                    break;
                case GCode::G61_1:
                    m_modePath = static_cast<GCPath>(code);
                    break;
                case GCode::G53:
                    m_nonModal = static_cast<GCNonModal>(code);
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
                    m_modeStop = static_cast<MCStop>(code);
                    break;
                case MCode::M3:
                case MCode::M4:
                case MCode::M5:
                    m_modeSpindle = static_cast<MCSpindle>(code);
                    break;
                case MCode::M6:
                    m_modeToolChange = static_cast<MCToolChange>(code);
                    break;
                default: throw std::logic_error(std::format("invalid m-code {}", name(code)));
            }
        }
    };
}
