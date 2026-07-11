#pragma once

#include <array>
#include <string_view>
#include <tuple>

#include "utils.h"
#include "memory/MemoryCell.h"

#define VARS(DO) \
    DO(PROBE_X, _probe_x, 5061, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_Y, _probe_y, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_Z, _probe_z, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_A, _probe_a, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_B, _probe_b, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_C, _probe_c, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_U, _probe_u, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_V, _probe_v, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_W, _probe_w, 0, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(PROBE_SUCCESS, _probe_success, 5070, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0) \
    DO(G28_X, _g28_x, 5161, MemoryCell::Flags::READ, 0) \
    DO(G28_Y, _g28_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G28_Z, _g28_z, 0, MemoryCell::Flags::READ, -0.100) \
    DO(G28_A, _g28_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G28_B, _g28_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G28_C, _g28_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G30_X, _g30_x, 5181, MemoryCell::Flags::READ, 0) \
    DO(G30_Y, _g30_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G30_Z, _g30_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G30_A, _g30_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G30_B, _g30_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G30_C, _g30_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G92_X, _g92_x, 5211, MemoryCell::Flags::READ, 0) \
    DO(G92_Y, _g92_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G92_Z, _g92_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G92_A, _g92_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G92_B, _g92_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G92_C, _g92_c, 0, MemoryCell::Flags::READ, 0) \
    DO(COORDSYS, _coordsys, 5220, MemoryCell::Flags::READ, 1) \
    DO(G54_X, _g54_x, 5221, MemoryCell::Flags::READ, 0) \
    DO(G54_Y, _g54_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G54_Z, _g54_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G54_A, _g54_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G54_B, _g54_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G54_C, _g54_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G55_X, _g55_x, 5241, MemoryCell::Flags::READ, 0) \
    DO(G55_Y, _g55_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G55_Z, _g55_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G55_A, _g55_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G55_B, _g55_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G55_C, _g55_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G56_X, _g56_x, 5261, MemoryCell::Flags::READ, 0) \
    DO(G56_Y, _g56_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G56_Z, _g56_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G56_A, _g56_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G56_B, _g56_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G56_C, _g56_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G57_X, _g57_x, 5281, MemoryCell::Flags::READ, 0) \
    DO(G57_Y, _g57_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G57_Z, _g57_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G57_A, _g57_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G57_B, _g57_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G57_C, _g57_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G58_X, _g58_x, 5301, MemoryCell::Flags::READ, 0) \
    DO(G58_Y, _g58_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G58_Z, _g58_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G58_A, _g58_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G58_B, _g58_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G58_C, _g58_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_X, _g59_x, 5321, MemoryCell::Flags::READ, 0) \
    DO(G59_Y, _g59_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_Z, _g59_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_A, _g59_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_B, _g59_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_C, _g59_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_1_X, _g59_1_x, 5341, MemoryCell::Flags::READ, 0) \
    DO(G59_1_Y, _g59_1_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_1_Z, _g59_1_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_1_A, _g59_1_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_1_B, _g59_1_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_1_C, _g59_1_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_2_X, _g59_2_x, 5361, MemoryCell::Flags::READ, 0) \
    DO(G59_2_Y, _g59_2_y, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_2_Z, _g59_2_z, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_2_A, _g59_2_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_2_B, _g59_2_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_2_C, _g59_2_c, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_3_X, _g59_3_x, 5381, MemoryCell::Flags::READ, -13.940) \
    DO(G59_3_Y, _g59_3_y, 0, MemoryCell::Flags::READ, 2.895) \
    DO(G59_3_Z, _g59_3_z, 0, MemoryCell::Flags::READ, -6.560) \
    DO(G59_3_A, _g59_3_a, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_3_B, _g59_3_b, 0, MemoryCell::Flags::READ, 0) \
    DO(G59_3_C, _g59_3_c, 0, MemoryCell::Flags::READ, 0) \
    /* System Vars */ \
    DO(TASK, _task, 6000, MemoryCell::Flags::READ | MemoryCell::Flags::VOLATILE, 0)

namespace ngc {
    enum class Var {
        #define ENUM_VALUE(name, ...) name,
        VARS(ENUM_VALUE)
        #undef ENUM_VALUE
    };

    using vars_t = std::tuple<Var, std::string_view, size_t, MemoryCell::Flags, double>;

    constexpr std::array gVars = {
        #define ITEM(var, name, addr, flags, value) vars_t { Var::var, #name, addr, flags, value },
        VARS(ITEM)
        #undef ITEM
    };

    inline constexpr std::string_view name(const Var var) {
        switch(var) {
            #define CASE(var, name, addr, flags, value) case Var::var: return #var;
            VARS(CASE)
            #undef CASE

            default: UNREACHABLE();
        }
    }
}
