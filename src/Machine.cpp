#include "machine/Machine.h"

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
    class Machine::Impl {
    public:
        using Axis = Machine::Axis;
        using Unit = Machine::Unit;

    private:
        Unit m_unit;
        position_t m_pos = {};
        position_t m_workOffset = {};
        position_t m_toolOffset = {};
        Memory m_memory;
        ToolTable m_toolTable;
        GCodeState m_state = GCodeState::makeDefault();
        int m_physicalToolNumber = 0;
        std::uint64_t m_nextCommandId = 1;

        struct PendingProbe {
            std::uint64_t id;
            position_t workOffset;
            position_t toolOffset;
            GCUnits programUnit;
        };

        std::optional<PendingProbe> m_pendingProbe;

    public:
        explicit Impl(Unit unit) {
            m_unit = unit;
            m_memory.init(gVars);
            beginProgramRun();
        }

        void beginProgramRun() {
            m_memory.resetProgramStorage();
            m_pos = {};
            m_workOffset = {};
            m_toolOffset = {};
            m_physicalToolNumber = 0;
            m_pendingProbe.reset();
            m_state = GCodeState::makeDefault();
            const auto num = static_cast<int>(m_memory.read(Var::COORDSYS));
            m_state.affectState(coordsys(num));
        }

        template<typename Self> auto &workOffset(this Self &&self) { return std::forward<Self>(self).m_workOffset; }
        template<typename Self> auto &toolOffset(this Self &&self) { return std::forward<Self>(self).m_toolOffset; }
        ToolGeometry toolGeometry() const {
            const auto tool = m_toolTable.get(m_physicalToolNumber);
            if(!tool) return {};
            return {
                .number = m_physicalToolNumber,
                .offset = { tool->x, tool->y, tool->z, tool->a, tool->b, tool->c },
                .diameter = tool->diameter,
            };
        }
        position_t physicalToolOffset() const { return toolGeometry().offset; }
        void prepareToolChange(const int toolNumber) { m_physicalToolNumber = toolNumber; }
        template<typename Self> auto &memory(this Self &&self) { return std::forward<Self>(self).m_memory; }

        void setActiveWorkOffset(const Axis axis, const double value) {
            const auto component = [&] {
                switch(axis) {
                    case Axis::X: return 0U;
                    case Axis::Y: return 1U;
                    case Axis::Z: return 2U;
                    case Axis::A: return 3U;
                    case Axis::B: return 4U;
                    case Axis::C: return 5U;
                }
                PANIC("invalid machine axis");
            }();
            const auto result = m_memory.write(
                offsetStartAddress(*m_state.modeCoordSys) + component, value, true);
            if(!result) PANIC("active work offset address must be writable internally");
            m_workOffset = offset(*m_state.modeCoordSys);
        }
        template<typename Self> auto &toolTable(this Self &&self) { return std::forward<Self>(self).m_toolTable; }
        template<typename Self> auto &state(this Self &&self) { return std::forward<Self>(self).m_state; }

        std::vector<std::string> activeModalGCodes() const {
            std::vector<std::string> codes;
            const auto append = [&](const auto &code) {
                if(code) codes.emplace_back(name(*code));
            };
            append(m_state.modeMotion);
            append(m_state.modePlane);
            append(m_state.modeDistance);
            append(m_state.modeArcDistance);
            append(m_state.modeFeedrate);
            append(m_state.modeUnits);
            append(m_state.modeToolOffset);
            append(m_state.modeCoordSys);
            append(m_state.modePath);
            return codes;
        }

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
            GCodeState blockState;
            for(const auto &word : block.words()) blockState.affectState(word);

            const auto positionCheckpoint = m_pos;
            const auto workOffsetCheckpoint = m_workOffset;
            const auto toolOffsetCheckpoint = m_toolOffset;
            const auto stateCheckpoint = m_state;
            const auto physicalToolCheckpoint = m_physicalToolNumber;
            const auto nextCommandIdCheckpoint = m_nextCommandId;
            const auto pendingProbeCheckpoint = m_pendingProbe;
            std::optional<Memory> memoryCheckpoint;
            if(blockState.nonModal == GCNonModal::G10) memoryCheckpoint = m_memory;

            try {
                return executeBlockImpl(block);
            } catch(...) {
                m_pos = positionCheckpoint;
                m_workOffset = workOffsetCheckpoint;
                m_toolOffset = toolOffsetCheckpoint;
                m_state = stateCheckpoint;
                m_physicalToolNumber = physicalToolCheckpoint;
                m_nextCommandId = nextCommandIdCheckpoint;
                m_pendingProbe = pendingProbeCheckpoint;
                if(memoryCheckpoint) m_memory = std::move(*memoryCheckpoint);
                throw;
            }
        }

        std::optional<double> pathTolerance() const { return m_state.pathTolerance; }

    private:
        std::vector<MachineCommand> executeBlockImpl(const Block &block) {
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
                    throw std::runtime_error(std::format("multiple non-modal G-codes in block: {}", block.statement()->text()));
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
                if(m_state.modePath == GCPath::G64) {
                    if(state.P) {
                        if(*state.P < 0.0) {
                            throw std::runtime_error(std::format("G64 P tolerance must be non-negative: {}",
                                                                 block.statement()->text()));
                        }
                        m_state.pathTolerance = *std::exchange(state.P, std::nullopt) * linearScale();
                    }
                } else {
                    m_state.pathTolerance.reset();
                }
            }

            if(state.modeMotion) {
                m_state.modeMotion = std::exchange(state.modeMotion, std::nullopt);
            }

            if(state.modeDistance) {
                m_state.modeDistance = std::exchange(state.modeDistance, std::nullopt);
            }

            if(state.modeArcDistance) {
                m_state.modeArcDistance = std::exchange(state.modeArcDistance, std::nullopt);
            }

            if(state.modeFeedrate) {
                if(state.modeFeedrate != GCFeed::G94) {
                    throw std::runtime_error(std::format("unsupported feed mode {}: {}",
                                                         name(*state.modeFeedrate), block.statement()->text()));
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
                            throw std::runtime_error(std::format("{}: G43 requires H or a programmed tool: {}",
                                                                 block.statement()->startToken().location(), block.statement()->text()));
                        }

                        toolNumber = static_cast<int>(*m_state.T);
                    }

                    if(auto tool = m_toolTable.get(toolNumber)) {
                        m_toolOffset = { tool->x, tool->y, tool->z, tool->a, tool->b, tool->c };
                    } else {
                        throw std::runtime_error(std::format("{}: tool {} not found: {}",
                                                             block.statement()->startToken().location(), toolNumber,
                                                             block.statement()->text()));
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
                throw std::runtime_error(std::format("{}: unsupported word or code in block: {}",
                                                     block.statement()->startToken().location(), block.statement()->text()));
            }

            return commands;
        }

        void handleG10(const Block &block, GCodeState &state) {
            if(!state.L || !state.P) {
                throw std::runtime_error(std::format("{}: G10 requires L and P: {}",
                                                     block.statement()->startToken().location(), block.statement()->text()));
            }

            m_state.L = std::exchange(state.L, std::nullopt);
            m_state.P = std::exchange(state.P, std::nullopt);

            if(*m_state.L == 2.0) {
                m_state.nonModal = std::exchange(state.nonModal, std::nullopt);
                const auto coordinateSystem = static_cast<int>(*m_state.P);
                if(*m_state.P != coordinateSystem || coordinateSystem < 1 || coordinateSystem > 9) {
                    throw std::runtime_error(std::format("{}: G10 has invalid coordinate system P{}: {}",
                                                         block.statement()->startToken().location(), *m_state.P,
                                                         block.statement()->text()));
                }
                auto code = coordsys(coordinateSystem);
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

            throw std::runtime_error(std::format("{}: unsupported G10 L{} operation: {}",
                                                 block.statement()->startToken().location(), *m_state.L,
                                                 block.statement()->text()));
        }

        MachineCommand handleMotion(const Block &block) {
            if(m_state.modeMotion == GCMotion::G0 || m_state.modeMotion == GCMotion::G1) {
                if(m_state.modeMotion == GCMotion::G1 && (!m_state.F || *m_state.F <= 0.0)) {
                    throw std::runtime_error(std::format("G1 requires a positive feedrate: {}", block.statement()->text()));
                }
                auto pos = position();
                auto speed = m_state.modeMotion == GCMotion::G1 ? *m_state.F : -1;
                auto command = MoveLine{m_pos, pos, speed, m_state.nonModal == GCNonModal::G53};
                m_pos = pos;
                return command;
            }

            if(m_state.modeMotion == GCMotion::G2 || m_state.modeMotion == GCMotion::G3) {
                if(!m_state.F || *m_state.F <= 0.0) {
                    throw std::runtime_error(std::format("{} requires a positive feedrate: {}",
                                                         name(*m_state.modeMotion), block.statement()->text()));
                }
                auto pos = position();
                auto flip = m_state.modeMotion == GCMotion::G3 ? 1.0 : -1.0;
                std::optional<vec3_t> center;
                std::optional<vec3_t> axis;

                const auto centerCoordinate = [&](const Axis coordinateAxis, const std::optional<double> word) {
                    if(m_state.modeArcDistance == GCArcDist::G91_1) {
                        switch(coordinateAxis) {
                            case Axis::X: return m_pos.x + word.value_or(0.0);
                            case Axis::Y: return m_pos.y + word.value_or(0.0);
                            case Axis::Z: return m_pos.z + word.value_or(0.0);
                            case Axis::A:
                            case Axis::B:
                            case Axis::C: PANIC("rotary axis cannot be an arc center coordinate");
                        }
                    }

                    if(m_state.modeArcDistance == GCArcDist::G90_1) {
                        return word.value_or(0.0) + offsets(coordinateAxis);
                    }

                    PANIC("invalid arc distance mode");
                };

                if(m_state.modePlane == GCPlane::G17) {
                    if(!m_state.I && !m_state.J) {
                        throw std::runtime_error(std::format("G17 arc requires I or J: {}", block.statement()->text()));
                    }

                    center = vec3_t(centerCoordinate(Axis::X, m_state.I), centerCoordinate(Axis::Y, m_state.J), m_pos.z);
                    axis = vec3_t(0.0, 0.0, 1.0) * flip;
                }

                if(m_state.modePlane == GCPlane::G18) {
                    if(!m_state.I && !m_state.K) {
                        throw std::runtime_error(std::format("G18 arc requires I or K: {}", block.statement()->text()));
                    }

                    center = vec3_t(centerCoordinate(Axis::X, m_state.I), m_pos.y, centerCoordinate(Axis::Z, m_state.K));
                    axis = vec3_t(0.0, 1.0, 0.0) * flip;
                }

                if(m_state.modePlane == GCPlane::G19) {
                    if(!m_state.J && !m_state.K) {
                        throw std::runtime_error(std::format("G19 arc requires J or K: {}", block.statement()->text()));
                    }

                    center = vec3_t(m_pos.x, centerCoordinate(Axis::Y, m_state.J), centerCoordinate(Axis::Z, m_state.K));
                    axis = vec3_t(1.0, 0.0, 0.0) * flip;
                }

                if(!center || !axis) {
                    PANIC();
                }

                if(auto valid = validateArc(m_pos, pos, *center); !valid) {
                    throw std::runtime_error(std::format("{}: {}: {}", block.statement()->startToken().location(),
                                                         valid.error(), block.statement()->text()));
                }

                auto command = MoveArc{m_pos, pos, *center, *axis, *m_state.F};
                m_pos = pos;
                return command;
            }

            if(m_state.modeMotion == GCMotion::G38_3) {
                if(!m_state.F || *m_state.F <= 0.0) {
                    throw std::runtime_error(std::format("G38.3 requires a positive feedrate: {}", block.statement()->text()));
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

    Machine::Machine(const Unit unit) : m_impl(std::make_unique<Impl>(unit)) { }
    Machine::~Machine() = default;
    Machine::Machine(Machine &&) noexcept = default;
    Machine &Machine::operator=(Machine &&) noexcept = default;

    void Machine::beginProgramRun() { m_impl->beginProgramRun(); }
    const position_t &Machine::workOffset() const { return m_impl->workOffset(); }
    void Machine::setActiveWorkOffset(const Axis axis, const double value) {
        m_impl->setActiveWorkOffset(axis, value);
    }
    const position_t &Machine::toolOffset() const { return m_impl->toolOffset(); }
    ToolGeometry Machine::toolGeometry() const { return m_impl->toolGeometry(); }
    position_t Machine::physicalToolOffset() const { return m_impl->physicalToolOffset(); }
    void Machine::prepareToolChange(const int toolNumber) { m_impl->prepareToolChange(toolNumber); }
    Memory &Machine::memory() { return m_impl->memory(); }
    const Memory &Machine::memory() const { return m_impl->memory(); }
    ToolTable &Machine::toolTable() { return m_impl->toolTable(); }
    const ToolTable &Machine::toolTable() const { return m_impl->toolTable(); }
    GCodeState &Machine::state() { return m_impl->state(); }
    const GCodeState &Machine::state() const { return m_impl->state(); }
    std::vector<std::string> Machine::activeModalGCodes() const { return m_impl->activeModalGCodes(); }
    double Machine::arcTolerance() const { return m_impl->arcTolerance(); }

    std::optional<double> Machine::pathTolerance() const { return m_impl->pathTolerance(); }
    std::expected<void, std::string> Machine::validateArc(const position_t &from, const position_t &to,
                                                          const vec3_t &center) const {
        return m_impl->validateArc(from, to, center);
    }
    void Machine::acceptProbeResult(const ProbeResult &result) { m_impl->acceptProbeResult(result); }
    std::vector<MachineCommand> Machine::executeBlock(const Block &block) { return m_impl->executeBlock(block); }
}
