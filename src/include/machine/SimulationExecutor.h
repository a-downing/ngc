#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "evaluator/InterpreterStatus.h"
#include "evaluator/InterpreterSession.h"
#include "machine/MachineCommand.h"

namespace ngc {
    enum class SimulationStatus { Stopped, Running, Paused, Completed, Error };

    struct SimulatedCommand {
        std::optional<MachineCommand> command;
        position_t toolOffset{};
        ToolGeometry tool{};
        std::optional<WorkCoordinateSystem> workCoordinateSystem;
        std::vector<std::string> modalGCodes;
        std::optional<BlockExecution> block;
        std::optional<InterpreterBlockLifecycle> lifecycle;

        SimulatedCommand(MachineCommand commandValue, position_t activeToolOffset = {}, ToolGeometry toolGeometry = {},
                         std::optional<WorkCoordinateSystem> activeWorkCoordinateSystem = std::nullopt,
                         std::vector<std::string> activeModalGCodes = {},
                         std::optional<BlockExecution> blockExecution = std::nullopt)
            : command(std::move(commandValue)), toolOffset(activeToolOffset), tool(std::move(toolGeometry)),
              workCoordinateSystem(std::move(activeWorkCoordinateSystem)), modalGCodes(std::move(activeModalGCodes)),
              block(std::move(blockExecution)) { }

        static SimulatedCommand lifecycleMarker(InterpreterBlockLifecycle blockLifecycle, std::vector<std::string> activeModalGCodes = {}) {
            SimulatedCommand marker { SpindleStop {} };
            marker.command.reset();
            marker.lifecycle = std::move(blockLifecycle);
            marker.modalGCodes = std::move(activeModalGCodes);
            return marker;
        }
    };

    struct SimulationSnapshot {
        SimulationStatus status = SimulationStatus::Stopped;
        position_t machinePosition{};
        position_t toolPosition{};
        ToolPose toolPose{};
        double commandProgress = 0.0;
        bool hasActiveMotion = false;
        bool spindleRunning = false;
        double spindleSpeed = 0.0;
        Direction spindleDirection = Direction::CW;
        std::string error;
        std::vector<InterpreterStatusMessage> statusMessages;
        std::optional<WorkCoordinateSystem> activeWorkCoordinateSystem;
        std::vector<std::string> activeModalGCodes;
        std::vector<WorkCoordinateSystem> usedWorkCoordinateSystems;
        std::vector<BlockExecution> activeBlocks;
        std::vector<BlockExecution> completedBlocks;
        std::unordered_map<std::string, std::vector<std::uint8_t>> completedLineFlags;
    };

    namespace simulation_detail {
        struct ArcGeometry {
            vec3_t center;
            vec3_t axisUnit;
            vec3_t startArm;
            vec3_t endArm;
            vec3_t axial;
            double sweep;
        };

        position_t mix(const position_t &from, const position_t &to, double t);
        double linearDistance(const position_t &from, const position_t &to);
        std::optional<ArcGeometry> arcGeometry(const MoveArc &arc);
        position_t interpolate(const MoveArc &arc, double t);
        double pathLength(const MoveArc &arc);
    }

    class SimulationExecutor {
    public:
        SimulationExecutor();
        ~SimulationExecutor();
        SimulationExecutor(const SimulationExecutor &) = delete;
        SimulationExecutor &operator=(const SimulationExecutor &) = delete;
        SimulationExecutor(SimulationExecutor &&) noexcept;
        SimulationExecutor &operator=(SimulationExecutor &&) noexcept;

        void reset();
        void prepareContinuation();
        bool canAccept() const;
        bool empty() const;
        void setRapidSpeed(double speed);
        void setStatus(SimulationStatus status);
        void setError(std::string error);
        void consume(SimulatedCommand command);
        void advance(double seconds);
        void completeQueued();
        SimulationSnapshot snapshot() const;
        SimulationSnapshot lightweightSnapshot() const;
        std::optional<ProbeResult> takeProbeResult();

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
