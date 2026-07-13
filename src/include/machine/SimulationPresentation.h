#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "evaluator/InterpreterStatus.h"
#include "machine/MachineCommand.h"

namespace ngc {
    enum class SimulationStatus { Stopped, Running, Paused, Completed, Error };

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
}
