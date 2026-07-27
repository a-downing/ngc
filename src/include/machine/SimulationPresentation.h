#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "evaluator/InterpreterStatus.h"
#include "machine/TrajectoryPlanner.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/MachineCommand.h"
#include "machine/MachineSession.h"
#include "machine/MotionBackend.h"
#include "machine/RealtimeTiming.h"

namespace ngc {
    enum class SimulationStatus { Stopped, Running, Holding, Paused, Completed, Error };
    enum class SimulationActivity { Idle, Program, Homing, Jogging };

    struct SimulationDiagnostics {
        double servoPeriodSeconds = 0.001;
        double schedulerPeriodSeconds = 0.01;
        std::uint32_t servoTicksPerSchedulerPeriod = 10;
        std::uint32_t tickMultiplier = 1;
        std::uint64_t servoTicks = 0;
        double programElapsedSeconds = 0.0;
        double executedPathJerk = 0.0;
        std::uint64_t deadlineMisses = 0;
        double lastWakeLatenessSeconds = 0.0;
        double maximumWakeLatenessSeconds = 0.0;
        double maximumTickExecutionSeconds = 0.0;
    };

    struct MachineSessionSnapshot {
        MachinePowerState powerState = MachinePowerState::Off;
        MachineActivity machineActivity = MachineActivity::Idle;
        ProgramOperationPresentation programOperation =
            ProgramOperationPresentation::Inactive;
        std::optional<SimulationDiagnostics> simulationDiagnostics;
        std::optional<RealtimeTimingSummary> realtimeTiming;
        position_t machinePosition{};
        TrajectoryCommandPresentation activePresentation{};
        double commandProgress = 0.0;
        TrajectoryPlanningDiagnostics trajectoryPlanning;
        std::string trajectoryPlanningActivity;
        double trajectoryPlanningActivitySeconds = 0.0;
        std::string trajectoryDriverActivity;
        std::string trajectoryContinuousPlanSummary;
        std::string trajectoryContinuousCorrectionHistory;
        BackendState trajectoryBackendState = BackendState::Disabled;
        EpochId trajectoryBackendEpoch = 0;
        ChunkId trajectoryBackendChunk = 0;
        SpanId trajectoryBackendSpan = 0;
        double trajectoryBackendSpanProgress = 0.0;
        double trajectoryBackendActiveNormalRemainingSeconds = 0.0;
        double trajectoryBackendQueuedNormalSeconds = 0.0;
        double trajectoryBackendCommittedNormalSeconds = 0.0;
        double trajectoryBackendStopBranchSeconds = 0.0;
        std::uint32_t trajectoryBackendQueuedExecutionItems = 0;
        BranchSequence trajectoryBackendLastBranch = 0;
        std::uint32_t trajectoryBackendFaultCode = 0;
        double trajectoryBackendVelocity = 0.0;
        double trajectoryBackendAcceleration = 0.0;
        double trajectoryBackendExecutionRate = 1.0;
        double trajectoryBackendExecutionRateAcceleration = 0.0;
        std::string trajectoryBackendSpanDetail;
        GeometryStreamDiagnostics geometryStream;
        bool hasActiveMotion = false;
        bool jogging = false;
        JointMask homedJoints = 0;
        JointMotionState joints{};
        std::optional<JogTarget> activeJogTarget;
        std::optional<JogStopReason> lastJogStopReason;
        bool spindleRunning = false;
        double spindleSpeed = 0.0;
        Direction spindleDirection = Direction::CW;
        std::optional<std::string> operatorAlert;
        std::string error;
        std::vector<InterpreterStatusMessage> statusMessages;
        std::vector<WorkCoordinateSystem> usedWorkCoordinateSystems;
        std::vector<BlockExecution> completedBlocks;
        std::unordered_map<std::string, std::vector<std::uint8_t>> completedLineFlags;
    };

    struct SimulationSnapshot : MachineSessionSnapshot {
        SimulationStatus status = SimulationStatus::Stopped;
        SimulationActivity activity = SimulationActivity::Idle;
        double servoPeriodSeconds = 0.001;
        double schedulerPeriodSeconds = 0.01;
        std::uint32_t servoTicksPerSchedulerPeriod = 10;
        std::uint32_t tickMultiplier = 1;
        std::uint64_t servoTicks = 0;
        double programElapsedSeconds = 0.0;
        double executedPathJerk = 0.0;
        std::uint64_t deadlineMisses = 0;
        double lastWakeLatenessSeconds = 0.0;
        double maximumWakeLatenessSeconds = 0.0;
        double maximumTickExecutionSeconds = 0.0;
    };

    inline ToolPose machineSessionToolPose(const MachineSessionSnapshot &snapshot) {
        const auto tipPosition =
            snapshot.machinePosition - snapshot.activePresentation.tool.offset;

        return {
            snapshot.activePresentation.tool,
            snapshot.machinePosition,
            tipPosition,
        };
    }

    inline ToolPose simulationToolPose(const SimulationSnapshot &snapshot) {
        return machineSessionToolPose(snapshot);
    }
}
