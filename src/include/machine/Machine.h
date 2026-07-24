#pragma once

#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gcode/GCode.h"
#include "machine/MachineCommand.h"
#include "machine/ToolTable.h"
#include "memory/Memory.h"

namespace ngc {
    class Machine {
    public:
        enum class Axis { X, Y, Z, A, B, C };
        enum class Unit { Millimeter, Inch };

        struct ToolChangeModalCheckpoint {
            GCMotion motion;
            GCPlane plane;
            GCDist distance;
            GCArcDist arcDistance;
            GCFeed feedMode;
            GCUnits units;
            GCTLen toolOffsetMode;
            GCCoord coordinateSystem;
            GCPath pathMode;
            std::optional<double> pathTolerance;
            std::optional<double> feedrate;
            std::optional<double> spindleSpeed;
            std::optional<double> selectedTool;
            position_t appliedToolOffset;
        };

        explicit Machine(Unit unit);
        ~Machine();

        Machine(const Machine &) = delete;
        Machine &operator=(const Machine &) = delete;
        Machine(Machine &&) noexcept;
        Machine &operator=(Machine &&) noexcept;

        void beginProgramRun();
        const position_t &workOffset() const;
        void setActiveWorkOffset(Axis axis, double value);
        const position_t &toolOffset() const;
        ToolGeometry toolGeometry() const;
        position_t physicalToolOffset() const;
        void prepareToolChange(int toolNumber);
        ToolChangeModalCheckpoint captureToolChangeModalCheckpoint() const;
        void restoreToolChangeModalCheckpoint(const ToolChangeModalCheckpoint &checkpoint);
        Memory &memory();
        const Memory &memory() const;
        ToolTable &toolTable();
        const ToolTable &toolTable() const;
        GCodeState &state();
        const GCodeState &state() const;
        std::vector<std::string> activeModalGCodes() const;
        double arcTolerance() const;
        std::optional<double> pathTolerance() const;
        std::expected<void, std::string> validateArc(const position_t &from, const position_t &to,
                                                     const vec3_t &center) const;
        void acceptProbeResult(const ProbeResult &result);
        std::vector<MachineCommand> executeBlock(const Block &block);

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
