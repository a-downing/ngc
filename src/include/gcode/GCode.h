#pragma once

#include <utility>
#include <vector>
#include <optional>
#include <format>
#include <print>

// #define GC_MOTION(DO) \
//     DO(G0, GCMotion) \
//     DO(G1, GCMotion) \
//     DO(G2, GCMotion) \
//     DO(G3, GCMotion)

// #define GC_PLANE(DO) \
//     DO(G17, GCPlane) \
//     DO(G18, GCPlane) \
//     DO(G19, GCPlane)

// #define GC_DISTANCE(DO) \
//     DO(G90, GCDistance) \
//     DO(G91, GCDistance)

// #define GC_FEED(DO) \
//     DO(G93, GCFeed) \
//     DO(G94, GCFeed)

// #define GC_UNITS(DO) \
//     DO(G20, GCUnits) \
//     DO(G21, GCUnits)

// #define GC_TOOL_LEN(DO) \
//     DO(G43, GCToolLen) \
//     DO(G49, GCToolLen)

// #define GC_COORD_SYS(DO) \
//     DO(G54, GCCoordSys) \
//     DO(G55, GCCoordSys) \
//     DO(G56, GCCoordSys) \
//     DO(G57, GCCoordSys) \
//     DO(G58, GCCoordSys) \
//     DO(G59, GCCoordSys) \
//     DO(G59_1, GCCoordSys) \
//     DO(G59_2, GCCoordSys) \
//     DO(G59_3, GCCoordSys)

// #define GC_PATH(DO) \
//     DO(G61_1, GCPath)

// #define GC_NON_MODAL(DO) \
//     DO(G53, GCNonModal)

// #define GCODES(DO) \
//     GC_MOTION(DO) \
//     GC_PLANE(DO) \
//     GC_DISTANCE(DO) \
//     GC_FEED(DO) \
//     GC_UNITS(DO) \
//     GC_TOOL_LEN(DO) \
//     GC_COORD_SYS(DO) \
//     GC_PATH(DO) \
//     GC_NON_MODAL(DO)

#include "parser/Token.h"
#include "parser/Expression.h"
#include "parser/Statement.h"
#include "utils.h"

namespace ngc {
#include "gcode/gcode.gen.h"
#include "gcode/mcode.gen.h"
}

namespace ngc {

    // enum class GCode {
    //     #define ENUM_VALUE(name, ...) name,
    //     GCODES(ENUM_VALUE)
    //     #undef ENUM_VALUE
    // };

    // inline constexpr std::string_view name(const GCode code) {
    //     switch(code) {
    //         #define CASE(name, ...) case GCode::name: return #name;
    //         GCODES(CASE)
    //         #undef CASE

    //         default: UNREACHABLE();
    //     }
    // }

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
        }

        PANIC("invalid coordinate system number: {}", i);
    }

    inline int coordsys(const GCCoord code) {
        switch(code) {
            case GCCoord::G54: return 1;
            case GCCoord::G55: return 2;
            case GCCoord::G56: return 3;
            case GCCoord::G57: return 4;
            case GCCoord::G58: return 5;
            case GCCoord::G59: return 6;
            case GCCoord::G59_1: return 7;
            case GCCoord::G59_2: return 8;
            case GCCoord::G59_3: return 9;
        }

        PANIC("invalid coordinate system: {}", name(code));
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

        PANIC("{}() missing case statement for Letter::{}", __func__, std::to_underlying(letter));
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
                }

                PANIC("invalid gcode G{}", real);
            case 61:
                switch(fract) {
                    case 1: return GCode::G61_1;
                }

                PANIC("invalid gcode G{}", real);
            case 90: return GCode::G90;
            case 91: return GCode::G91;
            case 93: return GCode::G93;
            case 94: return GCode::G94;
        }

        PANIC("invalid gcode G{}", real);
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
        }

        PANIC("invalid m-code M{}", real);
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
        }

        PANIC("can't convert token {} to Letter", name(kind));
    }

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

        void resetModal() {
            m_nonModal = std::nullopt;
            m_modeStop = std::nullopt;
            m_modeToolChange = std::nullopt;
            m_A = m_B = m_C = m_D = m_H = m_I = m_J = m_K = m_L = m_P = m_Q = m_R = m_X = m_Y = m_Z = std::nullopt;
        }

        GCMotion modeMotion() const { return m_modeMotion; }
        GCPlane modePlane() const { return m_modePlane; }
        GCDist modeDistance() const { return m_modeDistance; }
        GCFeed modeFeedrate() const { return m_modeFeedrate; }
        GCUnits modeUnits() const { return m_modeUnits; }
        GCTLen modeToolLengthOffset() const { return m_modeToolLengthOffset; }
        GCCoord modeCoordSys() const { return m_modeCoordSys; }
        GCPath modePath() const { return m_modePath; }
        MCSpindle modeSpindle() const { return m_modeSpindle; }
        std::optional<MCStop> modeStop() const { return m_modeStop; }
        std::optional<MCToolChange> modeToolChange() const { return m_modeToolChange; }
        std::optional<GCNonModal> nonModal() const { return m_nonModal; }

        void modeMotion(const GCMotion x) { m_modeMotion = x; }
        void modePlane(const GCPlane x) { m_modePlane = x; }
        void modeDistance(const GCDist x) { m_modeDistance = x; }
        void modeFeedrate(const GCFeed x) { m_modeFeedrate = x; }
        void modeUnits(const GCUnits x) { m_modeUnits = x; }
        void modeToolLengthOffset(const GCTLen x) { m_modeToolLengthOffset = x; }
        void modeCoordSys(const GCCoord x) { m_modeCoordSys = x; }
        void modePath(const GCPath x) { m_modePath = x; }
        void modeSpindle(const MCSpindle x) { m_modeSpindle = x; }
        void modeStop(const MCStop x) { m_modeStop = x; }
        void modeToolChange(const MCToolChange x) { m_modeToolChange = x; }
        void nonModal(const GCNonModal x) { m_nonModal = x; }

        std::optional<double> A() const { return m_A; }
        std::optional<double> B() const { return m_B; }
        std::optional<double> C() const { return m_C; }
        std::optional<double> D() const { return m_D; }
        double F() const { return m_F; }
        std::optional<double> H() const { return m_H; }
        std::optional<double> I() const { return m_I; }
        std::optional<double> J() const { return m_J; }
        std::optional<double> K() const { return m_K; }
        std::optional<double> L() const { return m_L; }
        std::optional<double> P() const { return m_P; }
        std::optional<double> Q() const { return m_Q; }
        std::optional<double> R() const { return m_R; }
        double S() const { return m_S; }
        double T() const { return m_T; }
        std::optional<double> X() const { return m_X; }
        std::optional<double> Y() const { return m_Y; }
        std::optional<double> Z() const { return m_Z; }

        void A(const double x) { m_A = x; }
        void B(const double x) { m_B = x; }
        void C(const double x) { m_C = x; }
        void D(const double x) { m_D = x; }
        void F(const double x) { m_F = x; }
        void H(const double x) { m_H = x; }
        void I(const double x) { m_I = x; }
        void J(const double x) { m_J = x; }
        void K(const double x) { m_K = x; }
        void L(const double x) { m_L = x; }
        void P(const double x) { m_P = x; }
        void Q(const double x) { m_Q = x; }
        void R(const double x) { m_R = x; }
        void S(const double x) { m_S = x; }
        void T(const double x) { m_T = x; }
        void X(const double x) { m_X = x; }
        void Y(const double x) { m_Y = x; }
        void Z(const double x) { m_Z = x; }

        void affectState(const Word &word) {
            switch (word.letter()) {
                case Letter::A: m_A = word.real(); return;
                case Letter::B: m_B = word.real(); return;
                case Letter::C: m_C = word.real(); return;
                case Letter::D: m_D = word.real(); return;
                case Letter::F: m_F = word.real(); return;
                case Letter::G: return affectState(convertGCode(word.real()));
                case Letter::H: m_H = word.real(); return;
                case Letter::I: m_I = word.real(); return;
                case Letter::J: m_J = word.real(); return;
                case Letter::K: m_K = word.real(); return;
                case Letter::L: m_L = word.real(); return;
                case Letter::M: return affectState(convertMCode(word.real()));
                case Letter::N: return;
                case Letter::P: m_P = word.real(); return;
                case Letter::Q: m_Q = word.real(); return;
                case Letter::R: m_R = word.real(); return;
                case Letter::S: m_S = word.real(); return;
                case Letter::T: m_T = word.real(); return;
                case Letter::X: m_X = word.real(); return;
                case Letter::Y: m_Y = word.real(); return;
                case Letter::Z: m_Z = word.real(); return;
            }

            PANIC("invalid word {}", word.text());
        }

        void affectState(const GCode code) {
            switch(code) {
                case GCode::G0:
                case GCode::G1:
                case GCode::G2:
                case GCode::G3:
                    m_modeMotion = static_cast<GCMotion>(code);
                    return;
                case GCode::G17:
                case GCode::G18:
                case GCode::G19:
                    m_modePlane = static_cast<GCPlane>(code);
                    return;
                case GCode::G90:
                case GCode::G91:
                    m_modeDistance = static_cast<GCDist>(code);
                    return;
                case GCode::G93:
                case GCode::G94:
                    m_modeFeedrate = static_cast<GCFeed>(code);
                    return;
                case GCode::G20:
                case GCode::G21:
                    m_modeUnits = static_cast<GCUnits>(code);
                    return;
                case GCode::G43:
                case GCode::G49:
                    m_modeToolLengthOffset = static_cast<GCTLen>(code);
                    return;
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
                    return;
                case GCode::G61_1:
                    m_modePath = static_cast<GCPath>(code);
                    return;
                case GCode::G53:
                    m_nonModal = static_cast<GCNonModal>(code);
                    return;
            }

            PANIC("invalid g-code {}", name(code));
        }

        void affectState(const MCode code) {
            switch(code) {
                case MCode::M0:
                case MCode::M1:
                case MCode::M2:
                case MCode::M30:
                    m_modeStop = static_cast<MCStop>(code);
                    return;
                case MCode::M3:
                case MCode::M4:
                case MCode::M5:
                    m_modeSpindle = static_cast<MCSpindle>(code);
                    return;
                case MCode::M6:
                    m_modeToolChange = static_cast<MCToolChange>(code);
                    return;
            }

            PANIC("invalid m-code {}", name(code));
        }
    };
}

