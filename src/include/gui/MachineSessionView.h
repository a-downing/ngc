#pragma once

#include <string>
#include <string_view>

#include "machine/MachineSessionCommand.h"
#include "machine/SimulationPresentation.h"

namespace ngc::gui {
    struct OperatorDro {
        position_t machinePosition{};
        position_t workPosition{};
        position_t toolTipPosition{};
        position_t activeToolOffset{};
        std::string_view workCoordinateSystem = "--";
        int tool = 0;
        bool spindleRunning = false;
        double spindleSpeed = 0.0;
        Direction spindleDirection = Direction::CW;
        MachineActivity motionOwner = MachineActivity::Idle;
    };

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

    inline OperatorDro operatorDro(const MachineSessionSnapshot &snapshot) noexcept {
        const auto workOffset = snapshot.activePresentation.workCoordinateSystem
            ? snapshot.activePresentation.workCoordinateSystem->offset
            : position_t {};

        return {
            .machinePosition = snapshot.machinePosition,
            .workPosition =
                snapshot.machinePosition - workOffset - snapshot.activePresentation.activeToolOffset,
            .toolTipPosition =
                snapshot.machinePosition - snapshot.activePresentation.tool.offset,
            .activeToolOffset = snapshot.activePresentation.activeToolOffset,
            .workCoordinateSystem = snapshot.activePresentation.workCoordinateSystem
                ? std::string_view(snapshot.activePresentation.workCoordinateSystem->name)
                : std::string_view("--"),
            .tool = snapshot.activePresentation.tool.number,
            .spindleRunning = snapshot.spindleRunning,
            .spindleSpeed = snapshot.spindleSpeed,
            .spindleDirection = snapshot.spindleDirection,
            .motionOwner = snapshot.machineActivity,
        };
    }

    inline std::string machineModalText(const MachineSessionSnapshot &snapshot) {
        std::string result;
        for (const auto &code : snapshot.activePresentation.modalGCodes) {
            if (!result.empty()) {
                result += ' ';
            }
            result += code;
        }

        return result;
    }

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

    constexpr std::string_view sessionCommandRejectionReason(
            const SessionCommandRejection rejection) noexcept {
        switch (rejection) {
            case SessionCommandRejection::None: return "the command was accepted";
            case SessionCommandRejection::SessionNotPowered:
                return "the controlled session is not powered on";
            case SessionCommandRejection::MotionOwned:
                return "another operation owns the controlled session";
            case SessionCommandRejection::NoMotionOwner:
                return "no operation currently owns motion";
            case SessionCommandRejection::EmptyProgram:
                return "the submitted program is empty";
            case SessionCommandRejection::ToolTableUnavailable:
                return "the session tool table could not be initialized";
            case SessionCommandRejection::HomingUnavailable:
                return "homing is not configured for the controlled session";
            case SessionCommandRejection::InvalidJogRequest:
                return "the jog request is invalid";
            case SessionCommandRejection::ProgramNotRunning:
                return "no program or MDI operation is running";
            case SessionCommandRejection::ProgramAlreadyPaused:
                return "the program is already paused";
            case SessionCommandRejection::ProgramNotPaused:
                return "the program is not paused";
            case SessionCommandRejection::FeedHoldInProgress:
                return "feed-hold acknowledgement is still pending";
            case SessionCommandRejection::FeedResumeInProgress:
                return "feed-resume acknowledgement is still pending";
            case SessionCommandRejection::CommandAlreadyQueued:
                return "the command is already queued";
            case SessionCommandRejection::CommandQueueFull:
                return "the session command queue is full";
            case SessionCommandRejection::CommandUnavailable:
                return "the controlled session cannot accept the command in its current state";
        }

        return "the command was rejected for an unknown reason";
    }

    constexpr std::string_view programOperationName(
            const ProgramOperationPresentation operation) noexcept {
        switch (operation) {
            case ProgramOperationPresentation::Inactive: return "Inactive";
            case ProgramOperationPresentation::Running: return "Running";
            case ProgramOperationPresentation::FeedHoldPending: return "Feed hold pending";
            case ProgramOperationPresentation::Held: return "Held";
            case ProgramOperationPresentation::Resuming: return "Resuming";
            case ProgramOperationPresentation::ProgramPaused: return "M0 pause";
            case ProgramOperationPresentation::Stopping: return "Stopping";
            case ProgramOperationPresentation::Completed: return "Completed";
            case ProgramOperationPresentation::Stopped: return "Stopped";
            case ProgramOperationPresentation::Failed: return "Failed";
        }

        return "Unknown";
    }

    inline MachineSessionControls machineSessionControls(
            const SimulationSnapshot &snapshot, const bool homingAvailable) noexcept {
        const auto powered = snapshot.powerState == MachinePowerState::On;
        const auto operationActive =
            snapshot.programOperation == ProgramOperationPresentation::Running
            || snapshot.programOperation == ProgramOperationPresentation::FeedHoldPending
            || snapshot.programOperation == ProgramOperationPresentation::Held
            || snapshot.programOperation == ProgramOperationPresentation::Resuming
            || snapshot.programOperation == ProgramOperationPresentation::ProgramPaused
            || snapshot.programOperation == ProgramOperationPresentation::Stopping;
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
                && snapshot.programOperation == ProgramOperationPresentation::Running,
            .canResume = powered && programActivity
                && (snapshot.programOperation == ProgramOperationPresentation::Held
                    || snapshot.programOperation
                        == ProgramOperationPresentation::ProgramPaused),
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

    inline std::string_view controllerDataUnavailableReason(
            const SimulationSnapshot &snapshot) noexcept {
        switch (snapshot.machineActivity) {
            case MachineActivity::Program:
                return "Program currently owns the Simulation session.";
            case MachineActivity::Mdi:
                return "MDI currently owns the Simulation session.";
            case MachineActivity::Homing:
                return "Homing currently owns the Simulation session.";
            case MachineActivity::Jogging:
                return "Jogging currently owns the Simulation session.";
            case MachineActivity::Holding:
                return "The Simulation session is completing a hold transition.";
            case MachineActivity::Stopping:
                return "The Simulation session is stopping.";
            case MachineActivity::Faulted:
                return "The Simulation session is faulted.";
            case MachineActivity::Idle:
                return "The Simulation session is completing a queued operation.";
        }

        return "The Simulation session cannot accept controller-data edits.";
    }
}
