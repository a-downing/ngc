#ifndef MCODE_GEN_H
#define MCODE_GEN_H

enum class MCode {
    M0 = 0,
    M1 = 1,
    M2 = 2,
    M30 = 3,
    M6 = 4,
    M3 = 5,
    M4 = 6,
    M5 = 7,
};

inline const char *name(const MCode code) {
    switch(code) {
        case MCode::M0: return "M0";
        case MCode::M1: return "M1";
        case MCode::M2: return "M2";
        case MCode::M30: return "M30";
        case MCode::M6: return "M6";
        case MCode::M3: return "M3";
        case MCode::M4: return "M4";
        case MCode::M5: return "M5";
        default: throw std::runtime_error(std::format("{}() invalid code MCode::{}", __func__, std::to_underlying(code)));
    }
}

enum class MCStop {
    M0 = 0,
    M1 = 1,
    M2 = 2,
    M30 = 3,
};

inline const char *name(const MCStop code) {
    switch(code) {
        case MCStop::M0: return "M0";
        case MCStop::M1: return "M1";
        case MCStop::M2: return "M2";
        case MCStop::M30: return "M30";
        default: throw std::runtime_error(std::format("{}() invalid code MCStop::{}", __func__, std::to_underlying(code)));
    }
}

enum class MCToolChange {
    M6 = 4,
};

inline const char *name(const MCToolChange code) {
    switch(code) {
        case MCToolChange::M6: return "M6";
        default: throw std::runtime_error(std::format("{}() invalid code MCToolChange::{}", __func__, std::to_underlying(code)));
    }
}

enum class MCSpindle {
    M3 = 5,
    M4 = 6,
    M5 = 7,
};

inline const char *name(const MCSpindle code) {
    switch(code) {
        case MCSpindle::M3: return "M3";
        case MCSpindle::M4: return "M4";
        case MCSpindle::M5: return "M5";
        default: throw std::runtime_error(std::format("{}() invalid code MCSpindle::{}", __func__, std::to_underlying(code)));
    }
}

#endif
