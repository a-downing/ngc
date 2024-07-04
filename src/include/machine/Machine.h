#pragma once

#include <print>
#include <tuple>
#include <vector>
#include <format>
#include <memory>
#include <stdexcept>
#include <utility>

#include "gcode/GCode.h"
#include "gcode/GCodeStateDifference.h"
#include "gcode/gcode.gen.h"
#include "machine/ToolTable.h"
#include "memory/Memory.h"
#include "machine/MachineCommand.h"
#include "memory/Vars.h"


namespace ngc {
    struct position_t {
        double x{}, y{}, z{}, a{}, b{}, c{};
    };

    inline position_t operator+(const position_t &a, const position_t &b) {
        return { a.x+b.x, a.y+b.y, a.z+b.z, a.a+b.a, a.b+b.b, a.c+b.c };
    }

    class Machine {
        position_t m_pos = {};
        position_t m_toolOffset = {};
        Memory m_memory;
        ToolTable m_toolTable;
        GCodeState m_state{};
        std::vector<std::unique_ptr<MachineCommand>> m_commands;

    public:
        Machine() {
            m_memory.init(gVars);
            auto result = m_toolTable.load();

            if(!result) {
                PANIC("{}", result.error());
            }

            const auto num = static_cast<int>(m_memory.read(Var::COORDSYS));
            m_state.affectState(coordsys(num));
        }

        template<typename Self> auto &memory(this Self &&self) { return std::forward<Self>(self).m_memory; }
        template<typename Self> auto &toolTable(this Self &&self) { return std::forward<Self>(self).m_toolTable; }
        template<typename Self> auto &state(this Self &&self) { return std::forward<Self>(self).m_state; }
        template<typename Self> auto &commands(this Self &&self) { return std::forward<Self>(self).m_commands; }

        void executeBlock(const Block &block) {
            m_state.resetModal();
            auto state = m_state;

            if(block.blockDelete()) {
                return;
            }

            for(const auto &word : block.words()) {
                state.affectState(word);
            }

            auto diff = GCodeStateDifference(m_state, state);

            if(auto modeMotion = diff.takeModeMotion()) {
                m_state.modeMotion(*modeMotion);
            }

            if(auto modeDistance = diff.takeModeDistance()) {
                m_state.modeDistance(*modeDistance);
            }

            if(auto modeCoordSys = diff.takeModeCoordSys()) {
                m_state.modeCoordSys(*modeCoordSys);
            }

            if(auto nonModal = diff.monModal()) {
                if(m_state.nonModal()) {
                    PANIC("already have non-modal: {}", name(*m_state.nonModal()));
                }

                if(*nonModal == GCNonModal::G53) {
                    m_state.nonModal(*diff.takeNonModal());
                }
            }

            if(auto tool = diff.takeT()) {
                m_state.T(*tool);
            }

            if(auto speed = diff.takeS()) {
                m_state.S(*speed);
            }

            if(auto feed = diff.takeF()) {
                m_state.F(*feed);
            }

            if(auto modeToolChange = diff.takeModeToolChange()) {
                m_state.modeToolChange(*modeToolChange);
            }

            if(auto modeSpindle = diff.takeModeSpindle()) {
                m_state.modeSpindle(*modeSpindle);

                if(modeSpindle == MCSpindle::M5) {
                    m_commands.emplace_back(std::make_unique<SpindleStop>());
                } else if(m_state.S() > 0 && (modeSpindle == MCSpindle::M3 || modeSpindle == MCSpindle::M4)) {
                    const auto dir = modeSpindle == MCSpindle::M3 ? SpindleStart::Dir::CW : SpindleStart::Dir::CCW;
                    m_commands.emplace_back(std::make_unique<SpindleStart>(dir, m_state.S()));
                }
            }

            if(auto modeToolLengthOffset = diff.takeModeToolLengthOffset()) {
                int toolNumber = static_cast<int>(m_state.T());

                std::ignore = diff.takeMovement();
                m_state.modeToolLengthOffset(*modeToolLengthOffset);

                if(auto h = diff.takeH()) {
                    toolNumber = static_cast<int>(*h);
                }

                if(auto tool = m_toolTable.get(toolNumber)) {
                    m_toolOffset = { tool->x, tool->y, tool->z, tool->a, tool->b, tool->c };
                } else {
                    PANIC("{}: tool not found: {}", block.statement()->startToken().location(), toolNumber);
                }
            }

            if(auto modePlane = diff.takeModePlane()) {
                m_state.modePlane(*modePlane);
            }

            if(auto modeStop = diff.takeModeStop()) {
                m_state.modeStop(*modeStop);
            }

            if(auto movement = diff.takeMovement(); !movement.empty()) {
                if(movement.X) { m_state.X(*movement.X); }
                if(movement.Y) { m_state.Y(*movement.Y); }
                if(movement.Z) { m_state.Z(*movement.Z); }
                if(movement.A) { m_state.A(*movement.A); }
                if(movement.B) { m_state.B(*movement.B); }
                if(movement.C) { m_state.C(*movement.C); }
                if(movement.I) { m_state.I(*movement.I); }
                if(movement.J) { m_state.J(*movement.J); }
                if(movement.K) { m_state.K(*movement.K); }

                handleMotion(block);
            }

            if(!diff.empty()) {
                PANIC("{}: unhandled gcode state change: {}", block.statement()->startToken().location(), diff.text());
            }
        }

    private:
        void handleMotion(const Block &block) {
            std::println("handleMotion: {}", block.statement()->text());
            // TODO: handle motion
        }

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
            return { m_memory.read(x), m_memory.read(y), m_memory.read(z), m_memory.read(a), m_memory.read(b), m_memory.read(c) };
        }
    };
}