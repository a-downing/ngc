#pragma once

#include <string_view>

#include "machine/SimulationPresentation.h"

namespace ngc::gui {
    struct MachineSessionControls {
        bool powered = false;
        bool idle = false;
        bool canStart = false;
        bool canHome = false;
        bool canJog = false;
        bool canFeedHold = false;
        bool canResume = false;
        bool canStop = false;
        bool canReset = false;
    };

    constexpr std::string_view powerStateName(const MachinePowerState state) noexcept {
        switch (state) {
            case MachinePowerState::Off: return "Off";
            case MachinePowerState::Starting: return "Starting";
            case MachinePowerState::On: return "On";
            case MachinePowerState::Stopping: return "Stopping";
            case MachinePowerState::Faulted: return "Faulted";
        }

        return "Unknown";
    }

    constexpr std::string_view machineActivityName(const MachineActivity activity) noexcept {
        switch (activity) {
            case MachineActivity::Idle: return "Idle";
            case MachineActivity::Program: return "Program";
            case MachineActivity::Mdi: return "MDI";
            case MachineActivity::Homing: return "Homing";
            case MachineActivity::Jogging: return "Jogging";
            case MachineActivity::Holding: return "Holding";
            case MachineActivity::Stopping: return "Stopping";
            case MachineActivity::Faulted: return "Faulted";
        }

        return "Unknown";
    }

    constexpr std::string_view simulationStatusName(const SimulationStatus status) noexcept {
        switch (status) {
            case SimulationStatus::Stopped: return "Stopped";
            case SimulationStatus::Running: return "Running";
            case SimulationStatus::Holding: return "Holding";
            case SimulationStatus::Paused: return "Paused";
            case SimulationStatus::Completed: return "Completed";
            case SimulationStatus::Error: return "Error";
        }

        return "Unknown";
    }

    inline MachineSessionControls machineSessionControls(
            const SimulationSnapshot &snapshot, const bool homingAvailable) noexcept {
        const auto powered = snapshot.powerState == MachinePowerState::On;
        const auto operationActive = snapshot.status == SimulationStatus::Running
            || snapshot.status == SimulationStatus::Holding
            || snapshot.status == SimulationStatus::Paused;
        const auto idle = snapshot.machineActivity == MachineActivity::Idle && !operationActive;
        const auto motionOwned = operationActive
            || snapshot.machineActivity == MachineActivity::Program
            || snapshot.machineActivity == MachineActivity::Mdi
            || snapshot.machineActivity == MachineActivity::Homing
            || snapshot.machineActivity == MachineActivity::Jogging
            || snapshot.machineActivity == MachineActivity::Holding
            || snapshot.machineActivity == MachineActivity::Stopping;
        const auto programActivity = snapshot.machineActivity == MachineActivity::Program
            || snapshot.machineActivity == MachineActivity::Mdi
            || snapshot.machineActivity == MachineActivity::Holding
            || snapshot.activity == SimulationActivity::Program;

        return {
            .powered = powered,
            .idle = idle,
            .canStart = powered && idle,
            .canHome = powered && idle && homingAvailable,
            .canJog = powered && idle,
            .canFeedHold = powered && programActivity
                && snapshot.status == SimulationStatus::Running
                && snapshot.trajectoryBackendExecutionRate >= 1.0 - 1e-10,
            .canResume = powered && programActivity
                && snapshot.status == SimulationStatus::Paused,
            .canStop = powered && motionOwned,
            .canReset = powered && !operationActive
                && (snapshot.machineActivity == MachineActivity::Idle
                    || snapshot.machineActivity == MachineActivity::Faulted),
        };
    }

    inline std::string_view unavailableMotionReason(
            const SimulationSnapshot &snapshot) noexcept {
        if (snapshot.powerState != MachinePowerState::On) {
            return "The Simulation session is not powered on.";
        }
        if (snapshot.machineActivity == MachineActivity::Faulted
            || snapshot.status == SimulationStatus::Error) {
            return "The Simulation session is faulted.";
        }
        if (snapshot.machineActivity != MachineActivity::Idle) {
            return "Another operation currently owns the Simulation session.";
        }

        return "The Simulation session is completing a queued operation.";
    }
}
