#pragma once

#include <expected>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "machine/TrajectoryCompiler.h"
#include "machine/Machine.h"

namespace ngc {
    struct SimulationTiming {
        // Seconds per simulated servo tick. Windows pacing is best-effort, but
        // every mock executor update advances by exactly this duration.
        double servoPeriod = 0.001;
        // Seconds between Windows scheduler wakes. Each wake executes an integer
        // batch of servo ticks, independent of Windows wake jitter.
        double schedulerPeriod = 0.01;
    };

    struct JoggingConfiguration {
        double acceleration = 0.0;
        double jerk = 0.0;
    };

    struct FeedHoldConfiguration {
        double tangentialAcceleration = 5.0;
        double tangentialJerk = 25.0;
    };

    enum class PendantDriver { VistaCncP2s };

    struct PendantStepConfiguration {
        double fineDistance = 0.0;
        double coarseDistance = 0.0;
    };

    struct PendantVelocityConfiguration {
        double maxVelocityScale = 0.0;
        double fullScaleCountsPerSecond = 0.0;
        double leaseDuration = 0.0;
    };

    struct PendantConfiguration {
        bool enabled = false;
        PendantDriver driver = PendantDriver::VistaCncP2s;
        PendantStepConfiguration step;
        PendantVelocityConfiguration velocity;
    };

    struct AxisConfiguration {
        Machine::Axis axis = Machine::Axis::X;
        std::vector<JointId> joints;
        double minimum = 0.0;
        double maximum = 0.0;
        double maxVelocity = 0.0;
        double maxAcceleration = 0.0;
        double maxJerk = 0.0;
    };

    struct DigitalInputConfiguration {
        std::string name;
        DigitalInputId id = 0;
    };

    struct ProbingConfiguration {
        DigitalInputId input = 0;
        InputCondition condition = InputCondition::Active;
        double debounce = 0.0;
    };

    struct JointHomingConfiguration {
        DigitalInputId input = 0;
        InputCondition condition = InputCondition::Active;
        double homePosition = 0.0;
        double switchPosition = 0.0;
        double searchVelocity = 0.0;
        double latchVelocity = 0.0;
        double backoffDistance = 0.0;
        double debounce = 0.0;
        double finalVelocity = 0.0;
        bool useIndex = false;
    };

    struct JointConfiguration {
        JointId id = 0;
        std::string name;
        Machine::Axis axis = Machine::Axis::X;
        double coordinateScale = 1.0;
        double minimum = 0.0;
        double maximum = 0.0;
        double maxVelocity = 0.0;
        double maxAcceleration = 0.0;
        double maxJerk = 0.0;
        JointHomingConfiguration homing;
    };

    struct HomingGroupConfiguration {
        std::string name;
        std::uint32_t sequence = 0;
        std::vector<JointId> joints;
        bool startTogether = false;
        bool stopEachJointOnTrigger = false;
        bool finalMoveTogether = false;
    };

    struct HomingConfiguration {
        bool requireBeforeMotion = false;
        std::vector<HomingGroupConfiguration> groups;
    };

    struct MachineConfiguration {
        Machine::Unit unit = Machine::Unit::Inch;
        std::vector<Machine::Axis> coordinates;
        TrajectoryLimits trajectory;
        SimulationTiming simulation;
        FeedHoldConfiguration feedHold;
        JoggingConfiguration jogging;
        PendantConfiguration pendant;
        std::vector<AxisConfiguration> axes;
        std::vector<DigitalInputConfiguration> digitalInputs;
        ProbingConfiguration probing;
        std::vector<JointConfiguration> joints;
        HomingConfiguration homing;
    };

    std::expected<MachineConfiguration, std::string>
    loadMachineConfiguration(const std::filesystem::path &path);
}
