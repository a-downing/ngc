#ifndef VARS_H
#define VARS_H

#include <tuple>
#include <string_view>

#include <MemoryCell.h>

namespace ngc {
    enum class Var {
        G28_X,
        G28_Y,
        G28_Z,
        G28_A,
        G28_B,
        G28_C,

        G30_X,
        G30_Y,
        G30_Z,
        G30_A,
        G30_B,
        G30_C,

        TIME,
    };

    static constexpr std::initializer_list<std::tuple<Var, std::string_view, size_t, MemoryCell::Flags>> VARS = {
        { Var::G28_X, "_g28_x", 5161, MemoryCell::Flags::READ },
        { Var::G28_Y, "_g28_y", 0, MemoryCell::Flags::READ },
        { Var::G28_Z, "_g28_z", 0, MemoryCell::Flags::READ },
        { Var::G28_A, "_g28_a", 0, MemoryCell::Flags::READ },
        { Var::G28_B, "_g28_b", 0, MemoryCell::Flags::READ },
        { Var::G28_C, "_g28_c", 0, MemoryCell::Flags::READ },

        { Var::G30_X, "_g30_x", 5181, MemoryCell::Flags::READ },
        { Var::G30_Y, "_g30_y", 0, MemoryCell::Flags::READ },
        { Var::G30_Z, "_g30_z", 0, MemoryCell::Flags::READ },
        { Var::G30_A, "_g30_a", 0, MemoryCell::Flags::READ },
        { Var::G30_B, "_g30_b", 0, MemoryCell::Flags::READ },
        { Var::G30_C, "_g30_c", 0, MemoryCell::Flags::READ },

        { Var::TIME, "_time", 0, MemoryCell::Flags::READ },
    };
}

#endif
