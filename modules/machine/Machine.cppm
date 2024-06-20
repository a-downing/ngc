module;

#include <print>
#include <vector>
#include <format>
#include <memory>
#include <stdexcept>
#include <utility>

export module machine;
export import :ToolTable;
export import :MachineCommand;
import utils;
import memory;
import gcode;

export namespace ngc {
    struct position_t {
        double x{}, y{}, z{}, a{}, b{}, c{};
    };

    position_t operator+(const position_t &a, const position_t &b) {
        return { a.x+b.x, a.y+b.y, a.z+b.z, a.a+b.a, a.b+b.b, a.c+b.c };
    }

    class Machine {
        position_t m_pos = {};
        Memory &m_mem;
        GCodeState m_state{};
        std::vector<std::unique_ptr<MachineCommand>> m_commands;

    public:
        explicit Machine(Memory &mem) : m_mem(mem) {
            const auto num = static_cast<int>(mem.read(Var::COORDSYS));
            m_state.affectState(coordsys(num));
        }

        const GCodeState &state() const { return m_state; }
        const std::vector<std::unique_ptr<MachineCommand>> &commands() const { return m_commands; }

        void executeBlock(const Block &block) {
            auto state = m_state;
            state.resetModal();

            if(block.blockDelete()) {
                return;
            }

            for(const auto &word : block.words()) {
                state.affectState(word);
            }

            const auto diff = m_state.calculateDifference(state);

            if(diff.modeSpindle) {
                if(*diff.modeSpindle == MCSpindle::M5) {
                    m_commands.emplace_back(std::make_unique<SpindleStop>());
                } else if(state.S() > 0 && (*diff.modeSpindle == MCSpindle::M3 || *diff.modeSpindle == MCSpindle::M4)) {
                    const auto dir = *diff.modeSpindle == MCSpindle::M3 ? SpindleStart::Dir::CW : SpindleStart::Dir::CCW;
                    m_commands.emplace_back(std::make_unique<SpindleStart>(dir, state.S()));
                }
            }

            m_state = state;
        }

    private:
        [[nodiscard]] position_t position(const GCodeState &state) {
            switch(state.modeDistance()) {
                case GCDist::G90: return positionAbsolute(state);
                case GCDist::G91: return positionRelative(state);
                default: throw std::runtime_error(std::format("invalid code GCodeState::{}", std::to_underlying(state.modeDistance())));
            }
        }

        [[nodiscard]] position_t positionAbsolute(const GCodeState &state) const {
            auto pos = position_t();

            if(state.X()) {
                pos.x = *state.X();
            }

            if(state.Y()) {
                pos.y = *state.Y();
            }

            if(state.Z()) {
                pos.z = *state.Z();
            }

            return pos;
        }

        [[nodiscard]] position_t positionRelative(const GCodeState &state) const {
            auto pos = m_pos;

            if(state.X()) {
                pos.x = *state.X();
            }

            if(state.Y()) {
                pos.y = *state.Y();
            }

            if(state.Z()) {
                pos.z = *state.Z();
            }

            return pos;
        }

        [[nodiscard]] position_t offset(const GCCoord code) {
            switch(code) {
                case GCCoord::G54: return offset(Var::G54_X, Var::G54_Y, Var::G54_Z, Var::G54_A, Var::G54_B, Var::G54_C);
                case GCCoord::G55: return offset(Var::G55_X, Var::G55_Y, Var::G55_Z, Var::G55_A, Var::G55_B, Var::G55_C);
                case GCCoord::G56: return offset(Var::G56_X, Var::G56_Y, Var::G56_Z, Var::G56_A, Var::G56_B, Var::G56_C);
                case GCCoord::G57: return offset(Var::G57_X, Var::G57_Y, Var::G57_Z, Var::G57_A, Var::G57_B, Var::G57_C);
                case GCCoord::G58: return offset(Var::G58_X, Var::G58_Y, Var::G58_Z, Var::G58_A, Var::G58_B, Var::G58_C);
                case GCCoord::G59: return offset(Var::G59_X, Var::G59_Y, Var::G59_Z, Var::G59_A, Var::G59_B, Var::G59_C);
                case GCCoord::G59_1: return offset(Var::G59_1_X, Var::G59_1_Y, Var::G59_1_Z, Var::G59_1_A, Var::G59_1_B, Var::G59_1_C);
                case GCCoord::G59_2: return offset(Var::G59_2_X, Var::G59_2_Y, Var::G59_2_Z, Var::G59_2_A, Var::G59_2_B, Var::G59_2_C);
                case GCCoord::G59_3: return offset(Var::G59_3_X, Var::G59_3_Y, Var::G59_3_Z, Var::G59_3_A, Var::G59_3_B, Var::G59_3_C);
                default: throw std::runtime_error(std::format("invalid offset GCode::{}", std::to_underlying(code)));
            }
        }

        [[nodiscard]] position_t offset(const Var x, const Var y, const Var z, const Var a, const Var b, const Var c) const {
            return { m_mem.read(x), m_mem.read(y), m_mem.read(z), m_mem.read(a), m_mem.read(b), m_mem.read(c) };
        }
    };
}