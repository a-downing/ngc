#pragma once

#include <cstdint>
#include <utility>
#include <string_view>

#include "utils.h"

enum class GCode : std::uint8_t {
    G0 = 0,
    G1 = 1,
    G2 = 2,
    G3 = 3,
    G17 = 4,
    G18 = 5,
    G19 = 6,
    G90 = 7,
    G91 = 8,
    G93 = 9,
    G94 = 10,
    G20 = 11,
    G21 = 12,
    G43 = 13,
    G49 = 14,
    G54 = 15,
    G55 = 16,
    G56 = 17,
    G57 = 18,
    G58 = 19,
    G59 = 20,
    G59_1 = 21,
    G59_2 = 22,
    G59_3 = 23,
    G61_1 = 24,
    G53 = 25,
    G10 = 26,
};

inline std::string_view name(const GCode code) {
    switch(code) {
        case GCode::G0: return "G0";
        case GCode::G1: return "G1";
        case GCode::G2: return "G2";
        case GCode::G3: return "G3";
        case GCode::G17: return "G17";
        case GCode::G18: return "G18";
        case GCode::G19: return "G19";
        case GCode::G90: return "G90";
        case GCode::G91: return "G91";
        case GCode::G93: return "G93";
        case GCode::G94: return "G94";
        case GCode::G20: return "G20";
        case GCode::G21: return "G21";
        case GCode::G43: return "G43";
        case GCode::G49: return "G49";
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
        case GCode::G53: return "G53";
        case GCode::G10: return "G10";
    }

    PANIC("{}() invalid code GCode::{}", __func__, std::to_underlying(code));
}

enum class GCMotion : std::uint8_t {
    G0 = 0,
    G1 = 1,
    G2 = 2,
    G3 = 3,
};

inline std::string_view name(const GCMotion code) {
    switch(code) {
        case GCMotion::G0: return "G0";
        case GCMotion::G1: return "G1";
        case GCMotion::G2: return "G2";
        case GCMotion::G3: return "G3";
    }

    PANIC("{}() invalid code GCMotion::{}", __func__, std::to_underlying(code));
}

enum class GCPlane : std::uint8_t {
    G17 = 4,
    G18 = 5,
    G19 = 6,
};

inline std::string_view name(const GCPlane code) {
    switch(code) {
        case GCPlane::G17: return "G17";
        case GCPlane::G18: return "G18";
        case GCPlane::G19: return "G19";
    }

    PANIC("{}() invalid code GCPlane::{}", __func__, std::to_underlying(code));
}

enum class GCDist : std::uint8_t {
    G90 = 7,
    G91 = 8,
};

inline std::string_view name(const GCDist code) {
    switch(code) {
        case GCDist::G90: return "G90";
        case GCDist::G91: return "G91";
    }

    PANIC("{}() invalid code GCDist::{}", __func__, std::to_underlying(code));
}

enum class GCFeed : std::uint8_t {
    G93 = 9,
    G94 = 10,
};

inline std::string_view name(const GCFeed code) {
    switch(code) {
        case GCFeed::G93: return "G93";
        case GCFeed::G94: return "G94";
    }

    PANIC("{}() invalid code GCFeed::{}", __func__, std::to_underlying(code));
}

enum class GCUnits : std::uint8_t {
    G20 = 11,
    G21 = 12,
};

inline std::string_view name(const GCUnits code) {
    switch(code) {
        case GCUnits::G20: return "G20";
        case GCUnits::G21: return "G21";
    }

    PANIC("{}() invalid code GCUnits::{}", __func__, std::to_underlying(code));
}

enum class GCTLen : std::uint8_t {
    G43 = 13,
    G49 = 14,
};

inline std::string_view name(const GCTLen code) {
    switch(code) {
        case GCTLen::G43: return "G43";
        case GCTLen::G49: return "G49";
    }

    PANIC("{}() invalid code GCTLen::{}", __func__, std::to_underlying(code));
}

enum class GCCoord : std::uint8_t {
    G54 = 15,
    G55 = 16,
    G56 = 17,
    G57 = 18,
    G58 = 19,
    G59 = 20,
    G59_1 = 21,
    G59_2 = 22,
    G59_3 = 23,
};

inline std::string_view name(const GCCoord code) {
    switch(code) {
        case GCCoord::G54: return "G54";
        case GCCoord::G55: return "G55";
        case GCCoord::G56: return "G56";
        case GCCoord::G57: return "G57";
        case GCCoord::G58: return "G58";
        case GCCoord::G59: return "G59";
        case GCCoord::G59_1: return "G59.1";
        case GCCoord::G59_2: return "G59.2";
        case GCCoord::G59_3: return "G59.3";
    }

    PANIC("{}() invalid code GCCoord::{}", __func__, std::to_underlying(code));
}

enum class GCPath : std::uint8_t {
    G61_1 = 24,
};

inline std::string_view name(const GCPath code) {
    switch(code) {
        case GCPath::G61_1: return "G61.1";
    }

    PANIC("{}() invalid code GCPath::{}", __func__, std::to_underlying(code));
}

enum class GCNonModal : std::uint8_t {
    G53 = 25,
    G10 = 26,
};

inline std::string_view name(const GCNonModal code) {
    switch(code) {
        case GCNonModal::G53: return "G53";
        case GCNonModal::G10: return "G10";
    }

    PANIC("{}() invalid code GCNonModal::{}", __func__, std::to_underlying(code));
}

