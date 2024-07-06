#pragma once

#include <utility>
#include <vector>
#include <optional>
#include <format>
#include <print>

#include "parser/Token.h"
#include "parser/Expression.h"
#include "parser/Statement.h"
#include "utils.h"

namespace ngc {
#include "gcode/gcode.gen.h"
#include "gcode/mcode.gen.h"
}

namespace ngc {
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
            case 0:
                switch(fract) {
                    case 0: return GCode::G0;
                }

                break;
            case 1:
                switch(fract) {
                    case 0: return GCode::G1;
                }

                break;
            case 10:
                switch(fract) {
                    case 0: return GCode::G10;
                }

                break;
            case 17:
                switch(fract) {
                    case 0: return GCode::G17;
                }

                break;
            case 18:
                switch(fract) {
                    case 0: return GCode::G18;
                }

                break;
            case 19:
                switch(fract) {
                    case 0: return GCode::G19;
                }

                break;
            case 2:
                switch(fract) {
                    case 0: return GCode::G2;
                }

                break;
            case 20:
                switch(fract) {
                    case 0: return GCode::G20;
                }

                break;
            case 21:
                switch(fract) {
                    case 0: return GCode::G21;
                }

                break;
            case 3:
                switch(fract) {
                    case 0: return GCode::G3;
                }

                break;
            case 43:
                switch(fract) {
                    case 0: return GCode::G43;
                }

                break;
            case 49:
                switch(fract) {
                    case 0: return GCode::G49;
                }

                break;
            case 53:
                switch(fract) {
                    case 0: return GCode::G53;
                }

                break;
            case 54:
                switch(fract) {
                    case 0: return GCode::G54;
                }

                break;
            case 55:
                switch(fract) {
                    case 0: return GCode::G55;
                }

                break;
            case 56:
                switch(fract) {
                    case 0: return GCode::G56;
                }

                break;
            case 57:
                switch(fract) {
                    case 0: return GCode::G57;
                }

                break;
            case 58:
                switch(fract) {
                    case 0: return GCode::G58;
                }

                break;
            case 59:
                switch(fract) {
                    case 0: return GCode::G59;
                    case 1: return GCode::G59_1;
                    case 2: return GCode::G59_2;
                    case 3: return GCode::G59_3;
                }

                break;
            case 61:
                switch(fract) {
                    case 1: return GCode::G61_1;
                }

                break;
            case 90:
                switch(fract) {
                    case 0: return GCode::G90;
                }

                break;
            case 91:
                switch(fract) {
                    case 0: return GCode::G91;
                }

                break;
            case 93:
                switch(fract) {
                    case 0: return GCode::G93;
                }

                break;
            case 94:
                switch(fract) {
                    case 0: return GCode::G94;
                }

                break;
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
    public:
        std::optional<GCMotion> modeMotion{};
        std::optional<GCPlane> modePlane{};
        std::optional<GCDist> modeDistance{};
        std::optional<GCFeed> modeFeedrate{};
        std::optional<GCUnits> modeUnits{};
        std::optional<GCTLen> modeToolOffset{};
        std::optional<GCCoord> modeCoordSys{};
        std::optional<GCPath> modePath{};
        std::optional<MCSpindle> modeSpindle{};

        std::optional<GCNonModal> nonModal{};
        std::optional<MCStop> modeStop{};
        std::optional<MCToolChange> modeToolChange{};

        std::optional<double> A, B, C, D, F, H, I, J, K, L, P, Q, R, S, T, X, Y, Z;

    public:
        static GCodeState makeDefault() {
            GCodeState state;
            state.affectState(GCode::G0);
            state.affectState(GCode::G17);
            state.affectState(GCode::G90);
            state.affectState(GCode::G94);
            state.affectState(GCode::G20);
            state.affectState(GCode::G49);
            state.affectState(GCode::G54);
            state.affectState(GCode::G61_1);
            state.affectState(MCode::M5);
            return state;
        }

        bool empty() const {
            if(modeMotion) { return false; }
            if(modePlane) { return false; }
            if(modeDistance) { return false; }
            if(modeFeedrate) { return false; }
            if(modeUnits) { return false; }
            if(modeToolOffset) { return false; }
            if(modeCoordSys) { return false; }
            if(modePath) { return false; }
            if(modeSpindle) { return false; }

            if(nonModal) { return false; }
            if(modeStop) { return false; }
            if(modeToolChange) { return false; }

            if(A || B || C || D || F || H || I || J || K || L || P || Q || R || S || T || X || Y || Z) { return false; }

            return true;
        }
        
        std::expected<void, std::string_view> valid() const {
            if(!modeMotion) { return std::unexpected("missing motion mode"); }
            if(!modePlane) { return std::unexpected("missing plane mode"); }
            if(!modeDistance) { return std::unexpected("missing distance mode"); }
            if(!modeFeedrate) { return std::unexpected("missing feedrate mode"); }
            if(!modeUnits) { return std::unexpected("missing units mode"); }
            if(!modeToolOffset) { return std::unexpected("missing tool offset mode"); }
            if(!modeCoordSys) { return std::unexpected("missing coordinate system"); }
            if(!modePath) { return std::unexpected("missing path mode"); }
            if(!modeSpindle) { return std::unexpected("missing spindle mode"); }

            return {};
        }

        void resetModal() {
            nonModal = std::nullopt;
            modeStop = std::nullopt;
            modeToolChange = std::nullopt;
            A = B = C = D = H = I = J = K = L = P = Q = R = X = Y = Z = std::nullopt;
        }

        void affectState(const Word &word) {
            switch (word.letter()) {
                case Letter::A: A = word.real(); return;
                case Letter::B: B = word.real(); return;
                case Letter::C: C = word.real(); return;
                case Letter::D: D = word.real(); return;
                case Letter::F: F = word.real(); return;
                case Letter::G: return affectState(convertGCode(word.real()));
                case Letter::H: H = word.real(); return;
                case Letter::I: I = word.real(); return;
                case Letter::J: J = word.real(); return;
                case Letter::K: K = word.real(); return;
                case Letter::L: L = word.real(); return;
                case Letter::M: return affectState(convertMCode(word.real()));
                case Letter::N: return;
                case Letter::P: P = word.real(); return;
                case Letter::Q: Q = word.real(); return;
                case Letter::R: R = word.real(); return;
                case Letter::S: S = word.real(); return;
                case Letter::T: T = word.real(); return;
                case Letter::X: X = word.real(); return;
                case Letter::Y: Y = word.real(); return;
                case Letter::Z: Z = word.real(); return;
            }

            PANIC("invalid word {}", word.text());
        }

        void affectState(const GCode code) {
            switch(code) {
                case GCode::G0:
                case GCode::G1:
                case GCode::G2:
                case GCode::G3:
                    modeMotion = static_cast<GCMotion>(code);
                    return;
                case GCode::G17:
                case GCode::G18:
                case GCode::G19:
                    modePlane = static_cast<GCPlane>(code);
                    return;
                case GCode::G90:
                case GCode::G91:
                    modeDistance = static_cast<GCDist>(code);
                    return;
                case GCode::G93:
                case GCode::G94:
                    modeFeedrate = static_cast<GCFeed>(code);
                    return;
                case GCode::G20:
                case GCode::G21:
                    modeUnits = static_cast<GCUnits>(code);
                    return;
                case GCode::G43:
                case GCode::G49:
                    modeToolOffset = static_cast<GCTLen>(code);
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
                    modeCoordSys = static_cast<GCCoord>(code);
                    return;
                case GCode::G61_1:
                    modePath = static_cast<GCPath>(code);
                    return;
                case GCode::G53:
                case GCode::G10:
                    nonModal = static_cast<GCNonModal>(code);
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
                    modeStop = static_cast<MCStop>(code);
                    return;
                case MCode::M3:
                case MCode::M4:
                case MCode::M5:
                    modeSpindle = static_cast<MCSpindle>(code);
                    return;
                case MCode::M6:
                    modeToolChange = static_cast<MCToolChange>(code);
                    return;
            }

            PANIC("invalid m-code {}", name(code));
        }
    };
}

