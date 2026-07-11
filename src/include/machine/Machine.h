#pragma once

#ifdef __clang__
    #pragma push_macro("__cpp_concepts")
    #define __cpp_concepts 202002L
    #include <expected>
    #pragma pop_macro("__cpp_concepts")
#else
    #include <expected>
#endif

#include <expected>
#include <optional>
#include <print>
#include <vector>
#include <format>
#include <utility>

#include "gcode/GCode.h"
#include "gcode/gcode.gen.h"
#include "machine/ToolTable.h"
#include "memory/Memory.h"
#include "machine/MachineCommand.h"
#include "memory/Vars.h"
#include "utils.h"

namespace ngc {
    class Machine {
    public:
        enum class Axis {
            X, Y, Z, A, B, C
        };

        enum class Unit {
            Millimeter,
            Inch
        };

    private:
        Unit m_unit;
        position_t m_pos = {};
        position_t m_workOffset = {};
        position_t m_toolOffset = {};
        Memory m_memory;
        ToolTable m_toolTable;
        GCodeState m_state = GCodeState::makeDefault();
        std::uint64_t m_nextCommandId = 1;

        struct PendingProbe {
            std::uint64_t id;
            position_t workOffset;
            position_t toolOffset;
            GCUnits programUnit;
        };

        std::optional<PendingProbe> m_pendingProbe;

    public:
        Machine(Unit unit) {
            m_unit = unit;
            m_memory.init(gVars);
            auto result = m_toolTable.load();
            if(!result) {
                PANIC("{}", result.error());
            }
            beginProgramRun();
        }

        void beginProgramRun() {
            m_pos = {};
            m_workOffset = {};
            m_toolOffset = {};
            m_pendingProbe.reset();
            m_state = GCodeState::makeDefault();
            const auto num = static_cast<int>(m_memory.read(Var::COORDSYS));
            m_state.affectState(coordsys(num));
        }

        template<typename Self> auto &workOffset(this Self &&self) { return std::forward<Self>(self).m_workOffset; }
        template<typename Self> auto &memory(this Self &&self) { return std::forward<Self>(self).m_memory; }
        template<typename Self> auto &toolTable(this Self &&self) { return std::forward<Self>(self).m_toolTable; }
        template<typename Self> auto &state(this Self &&self) { return std::forward<Self>(self).m_state; }

        double arcTolerance() const {
            constexpr double INCH_TOLERANCE = 0.0005;
            return m_unit == Unit::Inch ? INCH_TOLERANCE : INCH_TOLERANCE * 25.4;
        }

        std::expected<void, std::string> validateArc(const position_t &from, const position_t &to, const vec3_t &center) const {
            double startRadius = 0.0;
            double endRadius = 0.0;

            switch(*m_state.modePlane) {
                case GCPlane::G17:
                    startRadius = std::hypot(from.x - center.x, from.y - center.y);
                    endRadius = std::hypot(to.x - center.x, to.y - center.y);
                    break;
                case GCPlane::G18:
                    startRadius = std::hypot(from.x - center.x, from.z - center.z);
                    endRadius = std::hypot(to.x - center.x, to.z - center.z);
                    break;
                case GCPlane::G19:
                    startRadius = std::hypot(from.y - center.y, from.z - center.z);
                    endRadius = std::hypot(to.y - center.y, to.z - center.z);
                    break;
            }

            if(startRadius == 0.0 || endRadius == 0.0) {
                return std::unexpected("arc radius must be greater than zero");
            }

            if(std::abs(startRadius - endRadius) > arcTolerance()) {
                return std::unexpected(std::format("arc radius mismatch: start {} end {} tolerance {}", startRadius, endRadius, arcTolerance()));
            }

            return {};
        }

        void acceptProbeResult(const ProbeResult &result) {
            if(!m_pendingProbe || m_pendingProbe->id != result.id) {
                PANIC("unexpected probe result id {}", result.id);
            }

            const auto workPosition = result.triggerPosition - m_pendingProbe->workOffset - m_pendingProbe->toolOffset;
            const auto scale = linearScale(m_pendingProbe->programUnit);
            m_memory.write(Var::PROBE_X, workPosition.x / scale, true);
            m_memory.write(Var::PROBE_Y, workPosition.y / scale, true);
            m_memory.write(Var::PROBE_Z, workPosition.z / scale, true);
            m_memory.write(Var::PROBE_A, workPosition.a, true);
            m_memory.write(Var::PROBE_B, workPosition.b, true);
            m_memory.write(Var::PROBE_C, workPosition.c, true);
            m_memory.write(Var::PROBE_U, 0.0, true);
            m_memory.write(Var::PROBE_V, 0.0, true);
            m_memory.write(Var::PROBE_W, 0.0, true);
            m_memory.write(Var::PROBE_SUCCESS, result.status == ProbeStatus::Triggered ? 1.0 : 0.0, true);
            m_pos = result.stoppedPosition;
            m_pendingProbe.reset();
        }

        std::vector<MachineCommand> executeBlock(const Block &block) {
            std::vector<MachineCommand> commands;
            m_workOffset = offset(*m_state.modeCoordSys);
            m_state.resetModal();
            auto valid = m_state.valid();

            if(!valid) {
                PANIC("invalid gcode state: {}", valid.error());
            }

            GCodeState state;

            if(block.blockDelete()) {
                return commands;
            }

            for(const auto &word : block.words()) {
                state.affectState(word);
            }

            if(state.modeCoordSys) {
                m_state.modeCoordSys = std::exchange(state.modeCoordSys, std::nullopt);
                m_workOffset = offset(*m_state.modeCoordSys);
            }

            if(state.modeUnits) {
                m_state.modeUnits = std::exchange(state.modeUnits, std::nullopt);
            }

            if(state.nonModal) {
                if(m_state.nonModal) {
                    PANIC("already have non-modal: {}", name(*m_state.nonModal));
                }

                switch(*state.nonModal) {
                    case GCNonModal::G53:
                        m_state.nonModal = std::exchange(state.nonModal, std::nullopt);
                        break;
                    case GCNonModal::G10:
                        handleG10(block, state);
                        break;
                }
            }

            if(state.modePath) {
                m_state.modePath = std::exchange(state.modePath, std::nullopt);
            }

            if(state.modeMotion) {
                m_state.modeMotion = std::exchange(state.modeMotion, std::nullopt);
            }

            if(state.modeDistance) {
                m_state.modeDistance = std::exchange(state.modeDistance, std::nullopt);
            }

            if(state.modeFeedrate) {
                if(state.modeFeedrate != GCFeed::G94) {
                    PANIC("{}", name(*state.modeFeedrate));
                }

                m_state.modeFeedrate = std::exchange(state.modeFeedrate, std::nullopt);
            }

            if(state.F) {
                m_state.F = *std::exchange(state.F, std::nullopt) * linearScale();
            }

            if(state.S) {
                m_state.S = std::exchange(state.S, std::nullopt);
            }

            if(state.T) {
                m_state.T = std::exchange(state.T, std::nullopt);
            }

            if(state.modeToolChange) {
                m_state.modeToolChange = std::exchange(state.modeToolChange, std::nullopt);
            }

            if(state.modeSpindle) {
                m_state.modeSpindle = std::exchange(state.modeSpindle, std::nullopt);

                if(m_state.modeSpindle == MCSpindle::M5) {
                    commands.emplace_back(SpindleStop{});
                } else if(m_state.S > 0 && (m_state.modeSpindle == MCSpindle::M3 || m_state.modeSpindle == MCSpindle::M4)) {
                    const auto dir = m_state.modeSpindle == MCSpindle::M3 ? Direction::CW : Direction::CCW;
                    commands.emplace_back(SpindleStart{dir, *m_state.S});
                }
            }

            if(state.modeToolOffset) {
                m_state.modeToolOffset = std::exchange(state.modeToolOffset, std::nullopt);
                
                if(m_state.modeToolOffset == GCTLen::G43) {
                    int toolNumber = 0;

                    if(state.H) {
                        m_state.H = std::exchange(state.H, std::nullopt);
                        toolNumber = static_cast<int>(*m_state.H);
                    }

                    if(toolNumber == 0) {
                        if(!m_state.T || static_cast<int>(*m_state.T) == 0) {
                            PANIC("{}: no tool specified for tool change and no T currently programmed: {}", block.statement()->startToken().location(), block.statement()->text());
                        }

                        toolNumber = static_cast<int>(*m_state.T);
                    }

                    if(auto tool = m_toolTable.get(toolNumber)) {
                        m_toolOffset = { tool->x, tool->y, tool->z, tool->a, tool->b, tool->c };
                    } else {
                        PANIC("{}: tool not found: {}", block.statement()->startToken().location(), toolNumber);
                    }
                } else if(m_state.modeToolOffset == GCTLen::G49) {
                    m_toolOffset = {};
                }
            }

            if(state.modePlane) {
                m_state.modePlane = std::exchange(state.modePlane, std::nullopt);
            }

            if(state.modeStop) {
                m_state.modeStop = std::exchange(state.modeStop, std::nullopt);
            }

            bool hasMovement = false;

            if(state.X) { m_state.X = *std::exchange(state.X, std::nullopt) * linearScale(); hasMovement = true; }
            if(state.Y) { m_state.Y = *std::exchange(state.Y, std::nullopt) * linearScale(); hasMovement = true; }
            if(state.Z) { m_state.Z = *std::exchange(state.Z, std::nullopt) * linearScale(); hasMovement = true; }
            if(state.A) { m_state.A = std::exchange(state.A, std::nullopt); hasMovement = true; }
            if(state.B) { m_state.B = std::exchange(state.B, std::nullopt); hasMovement = true; }
            if(state.C) { m_state.C = std::exchange(state.C, std::nullopt); hasMovement = true; }
            if(state.I) { m_state.I = *std::exchange(state.I, std::nullopt) * linearScale(); hasMovement = true; }
            if(state.J) { m_state.J = *std::exchange(state.J, std::nullopt) * linearScale(); hasMovement = true; }
            if(state.K) { m_state.K = *std::exchange(state.K, std::nullopt) * linearScale(); hasMovement = true; }

            if(hasMovement) {
                commands.emplace_back(handleMotion(block));
            }

            if(!state.empty()) {
                PANIC("{}: unhandled gcode state change", block.statement()->startToken().location());
            }

            return commands;
        }

    private:
        void handleG10(const Block &block, GCodeState &state) {
            if(!state.L || !state.P) {
                PANIC("{}: G10 missing L or P: {}", block.statement()->startToken().location(), block.statement()->text());
            }

            m_state.L = std::exchange(state.L, std::nullopt);
            m_state.P = std::exchange(state.P, std::nullopt);

            if(*m_state.L == 2.0) {
                m_state.nonModal = std::exchange(state.nonModal, std::nullopt);
                auto code = coordsys(static_cast<int>(*m_state.P));
                auto startAddr = offsetStartAddress(static_cast<GCCoord>(code));

                if(state.X) {
                    m_state.X = *std::exchange(state.X, std::nullopt) * linearScale();
                    auto result = m_memory.write(startAddr + 0, *m_state.X, true);
                    if(!result) { PANIC(); }
                }

                if(state.Y) {
                    m_state.Y = *std::exchange(state.Y, std::nullopt) * linearScale();
                    auto result = m_memory.write(startAddr + 1, *m_state.Y, true);
                    if(!result) { PANIC(); }
                }

                if(state.Z) {
                    m_state.Z = *std::exchange(state.Z, std::nullopt) * linearScale();
                    auto result = m_memory.write(startAddr + 2, *m_state.Z, true);
                    if(!result) { PANIC(); }
                }

                if(state.A) {
                    m_state.A = std::exchange(state.A, std::nullopt);
                    auto result = m_memory.write(startAddr + 3, *m_state.A, true);
                    if(!result) { PANIC(); }
                }

                if(state.B) {
                    m_state.B = std::exchange(state.B, std::nullopt);
                    auto result = m_memory.write(startAddr + 4, *m_state.B, true);
                    if(!result) { PANIC(); }
                }

                if(state.C) {
                    m_state.C = std::exchange(state.C, std::nullopt);
                    auto result = m_memory.write(startAddr + 5, *m_state.C, true);
                    if(!result) { PANIC(); }
                }

                m_workOffset = offset(*m_state.modeCoordSys);
                return;
            }
        }

        MachineCommand handleMotion(const Block &block) {
            std::println("block: {}", block.statement()->text());
            
            if(m_state.modeMotion == GCMotion::G0 || m_state.modeMotion == GCMotion::G1) {
                auto pos = position();
                std::println("pos: {}", pos.text());
                auto speed = m_state.modeMotion == GCMotion::G1 ? *m_state.F : -1;
                auto command = MoveLine{m_pos, pos, speed};
                m_pos = pos;
                return command;
            }

            if(m_state.modeMotion == GCMotion::G2 || m_state.modeMotion == GCMotion::G3) {
                auto pos = position();
                auto flip = m_state.modeMotion == GCMotion::G3 ? 1.0 : -1.0;
                std::optional<vec3_t> center;
                std::optional<vec3_t> axis;

                if(m_state.modePlane == GCPlane::G17) {
                    if(!m_state.I && !m_state.J) {
                        PANIC("missing I and J for G17 arc: {}", block.statement()->text());
                    }

                    center = vec3_t(m_pos.x + m_state.I.value_or(0.0), m_pos.y + m_state.J.value_or(0.0), m_pos.z);
                    axis = vec3_t(0.0, 0.0, 1.0) * flip;
                }

                if(m_state.modePlane == GCPlane::G18) {
                    if(!m_state.I && !m_state.K) {
                        PANIC("missing I and K for G18 arc: {}", block.statement()->text());
                    }

                    center = vec3_t(m_pos.x + m_state.I.value_or(0.0), m_pos.y, m_pos.z + m_state.K.value_or(0.0));
                    axis = vec3_t(0.0, 1.0, 0.0) * flip;
                }

                if(m_state.modePlane == GCPlane::G19) {
                    if(!m_state.J && !m_state.K) {
                        PANIC("missing J and K for G19 arc: {}", block.statement()->text());
                    }

                    center = vec3_t(m_pos.x, m_pos.y + m_state.J.value_or(0.0), m_pos.z + m_state.K.value_or(0.0));
                    axis = vec3_t(1.0, 0.0, 0.0) * flip;
                }

                if(!center || !axis) {
                    PANIC();
                }

                if(auto valid = validateArc(m_pos, pos, *center); !valid) {
                    PANIC("{}: {}: {}", block.statement()->startToken().location(), valid.error(), block.statement()->text());
                }

                auto command = MoveArc{m_pos, pos, *center, *axis, *m_state.F};
                m_pos = pos;
                return command;
            }

            if(m_state.modeMotion == GCMotion::G38_3) {
                if(!m_state.F || *m_state.F <= 0.0) {
                    PANIC("G38.3 requires a positive feedrate: {}", block.statement()->text());
                }

                const auto target = position();
                const auto id = m_nextCommandId++;
                m_pendingProbe = PendingProbe { id, m_workOffset, m_toolOffset, *m_state.modeUnits };
                return ProbeMove { id, m_pos, target, *m_state.F, true, false };
            }

            PANIC("unhandled motion: {}", block.statement()->text());
        }

        double offsets(const Axis a) const {
            auto workOffset = m_state.nonModal == GCNonModal::G53 ? position_t() : m_workOffset;

            switch(a) {
                case Axis::X: return workOffset.x + m_toolOffset.x;
                case Axis::Y: return workOffset.y + m_toolOffset.y;
                case Axis::Z: return workOffset.z + m_toolOffset.z;
                case Axis::A: return workOffset.a + m_toolOffset.a;
                case Axis::B: return workOffset.b + m_toolOffset.b;
                case Axis::C: return workOffset.c + m_toolOffset.c;
            }

            PANIC();
        }

        double axis(const Axis a) const {
            if(m_state.modeDistance == GCDist::G91) {
                switch(a) {
                    case Axis::X: return m_state.X ? *m_state.X + m_pos.x : m_pos.x;
                    case Axis::Y: return m_state.Y ? *m_state.Y + m_pos.y : m_pos.y;
                    case Axis::Z: return m_state.Z ? *m_state.Z + m_pos.z : m_pos.z;
                    case Axis::A: return m_state.A ? *m_state.A + m_pos.a : m_pos.a;
                    case Axis::B: return m_state.B ? *m_state.B + m_pos.b : m_pos.b;
                    case Axis::C: return m_state.C ? *m_state.C + m_pos.c : m_pos.c;
                }

                PANIC();
            }

            if(m_state.modeDistance != GCDist::G90) {
                PANIC();
            }

            switch(a) {
                case Axis::X: return m_state.X ? *m_state.X + offsets(a) : m_pos.x;
                case Axis::Y: return m_state.Y ? *m_state.Y + offsets(a) : m_pos.y;
                case Axis::Z: return m_state.Z ? *m_state.Z + offsets(a) : m_pos.z;
                case Axis::A: return m_state.A ? *m_state.A + offsets(a) : m_pos.a;
                case Axis::B: return m_state.B ? *m_state.B + offsets(a) : m_pos.b;
                case Axis::C: return m_state.C ? *m_state.C + offsets(a) : m_pos.c;
            }

            PANIC();
        }

        position_t position() const {
            return { axis(Axis::X), axis(Axis::Y), axis(Axis::Z), axis(Axis::A), axis(Axis::B), axis(Axis::C) };
        }

        position_t offset(const GCCoord code) {
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
            }

            PANIC("invalid offset GCode::{}", std::to_underlying(code));
        }

        position_t offset(const Var x, const Var y, const Var z, const Var a, const Var b, const Var c) const {
            return { m_memory.read(x), m_memory.read(y), m_memory.read(z), m_memory.read(a), m_memory.read(b), m_memory.read(c) };
        }

        uint32_t offsetStartAddress(const GCCoord code) {
            switch(code) {
                case GCCoord::G54: return m_memory.deref(Var::G54_X);
                case GCCoord::G55: return m_memory.deref(Var::G55_X);
                case GCCoord::G56: return m_memory.deref(Var::G56_X);
                case GCCoord::G57: return m_memory.deref(Var::G57_X);
                case GCCoord::G58: return m_memory.deref(Var::G58_X);
                case GCCoord::G59: return m_memory.deref(Var::G59_X);
                case GCCoord::G59_1: return m_memory.deref(Var::G59_1_X);
                case GCCoord::G59_2: return m_memory.deref(Var::G59_2_X);
                case GCCoord::G59_3: return m_memory.deref(Var::G59_3_X);
            }

            PANIC("invalid offset GCode::{}", std::to_underlying(code));
        }

        double linearScale() const {
            return linearScale(*m_state.modeUnits);
        }

        double linearScale(const GCUnits programUnit) const {
            if(m_unit == Unit::Inch) {
                return programUnit == GCUnits::G20 ? 1.0 : 1.0 / 25.4;
            }

            return programUnit == GCUnits::G20 ? 25.4 : 1.0;
        }
    };
}
