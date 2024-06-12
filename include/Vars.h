#ifndef VARS_H
#define VARS_H

#include <tuple>
#include <string_view>

#include <MemoryCell.h>

namespace ngc {
    enum class Var {
        TIME,
        G54_X,
        G54_Y,
        G54_Z,
    };

    static constexpr std::initializer_list<std::tuple<Var, std::string_view, MemoryCell::Flags>> VARS = {
        { Var::TIME, "_time", MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE },
        { Var::G54_X, "_g54_x", MemoryCell::Flags::READ },
        { Var::G54_Y, "_g54_y", MemoryCell::Flags::READ },
        { Var::G54_Z, "_g54_z", MemoryCell::Flags::READ }
    };
}

#endif
