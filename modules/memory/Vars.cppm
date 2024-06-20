module;

#include <tuple>
#include <string_view>

export module memory:Vars;
import :MemoryCell;

export namespace ngc {
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

        G92_X,
        G92_Y,
        G92_Z,
        G92_A,
        G92_B,
        G92_C,

        COORDSYS,

        G54_X,
        G54_Y,
        G54_Z,
        G54_A,
        G54_B,
        G54_C,

        G55_X,
        G55_Y,
        G55_Z,
        G55_A,
        G55_B,
        G55_C,

        G56_X,
        G56_Y,
        G56_Z,
        G56_A,
        G56_B,
        G56_C,

        G57_X,
        G57_Y,
        G57_Z,
        G57_A,
        G57_B,
        G57_C,

        G58_X,
        G58_Y,
        G58_Z,
        G58_A,
        G58_B,
        G58_C,

        G59_X,
        G59_Y,
        G59_Z,
        G59_A,
        G59_B,
        G59_C,

        G59_1_X,
        G59_1_Y,
        G59_1_Z,
        G59_1_A,
        G59_1_B,
        G59_1_C,

        G59_2_X,
        G59_2_Y,
        G59_2_Z,
        G59_2_A,
        G59_2_B,
        G59_2_C,

        G59_3_X,
        G59_3_Y,
        G59_3_Z,
        G59_3_A,
        G59_3_B,
        G59_3_C,
    };

    using vars_t = std::tuple<Var, std::string_view, size_t, MemoryCell::Flags, double>;

    // const and constexpr result in linker error with -O2, not sure if bug or I'm overlooking something
    constinit std::initializer_list<vars_t> VARS = {
        { Var::G28_X, "_g28_x", 5161, MemoryCell::Flags::READ, 0 },
        { Var::G28_Y, "_g28_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G28_Z, "_g28_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G28_A, "_g28_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G28_B, "_g28_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G28_C, "_g28_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G30_X, "_g30_x", 5181, MemoryCell::Flags::READ, 0 },
        { Var::G30_Y, "_g30_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G30_Z, "_g30_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G30_A, "_g30_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G30_B, "_g30_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G30_C, "_g30_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G92_X, "_g92_x", 5211, MemoryCell::Flags::READ, 0 },
        { Var::G92_Y, "_g92_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G92_Z, "_g92_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G92_A, "_g92_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G92_B, "_g92_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G92_C, "_g92_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::COORDSYS, "_coordsys", 5220, MemoryCell::Flags::READ, 1 },

        { Var::G54_X, "_g54_x", 5221, MemoryCell::Flags::READ, 0 },
        { Var::G54_Y, "_g54_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G54_Z, "_g54_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G54_A, "_g54_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G54_B, "_g54_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G54_C, "_g54_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G55_X, "_g55_x", 5241, MemoryCell::Flags::READ, 0 },
        { Var::G55_Y, "_g55_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G55_Z, "_g55_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G55_A, "_g55_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G55_B, "_g55_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G55_C, "_g55_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G56_X, "_g56_x", 5261, MemoryCell::Flags::READ, 0 },
        { Var::G56_Y, "_g56_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G56_Z, "_g56_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G56_A, "_g56_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G56_B, "_g56_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G56_C, "_g56_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G57_X, "_g57_x", 5281, MemoryCell::Flags::READ, 0 },
        { Var::G57_Y, "_g57_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G57_Z, "_g57_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G57_A, "_g57_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G57_B, "_g57_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G57_C, "_g57_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G58_X, "_g58_x", 5301, MemoryCell::Flags::READ, 0 },
        { Var::G58_Y, "_g58_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G58_Z, "_g58_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G58_A, "_g58_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G58_B, "_g58_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G58_C, "_g58_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G59_X, "_g59_x", 5321, MemoryCell::Flags::READ, 0 },
        { Var::G59_Y, "_g59_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_Z, "_g59_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_A, "_g59_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_B, "_g59_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_C, "_g59_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G59_1_X, "_g59_1_x", 5341, MemoryCell::Flags::READ, 0 },
        { Var::G59_1_Y, "_g59_1_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_1_Z, "_g59_1_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_1_A, "_g59_1_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_1_B, "_g59_1_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_1_C, "_g59_1_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G59_2_X, "_g59_2_x", 5361, MemoryCell::Flags::READ, 0 },
        { Var::G59_2_Y, "_g59_2_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_2_Z, "_g59_2_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_2_A, "_g59_2_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_2_B, "_g59_2_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_2_C, "_g59_2_c", 0, MemoryCell::Flags::READ, 0 },

        { Var::G59_3_X, "_g59_3_x", 5381, MemoryCell::Flags::READ, 0 },
        { Var::G59_3_Y, "_g59_3_y", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_3_Z, "_g59_3_z", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_3_A, "_g59_3_a", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_3_B, "_g59_3_b", 0, MemoryCell::Flags::READ, 0 },
        { Var::G59_3_C, "_g59_3_c", 0, MemoryCell::Flags::READ, 0 },
    };
}
