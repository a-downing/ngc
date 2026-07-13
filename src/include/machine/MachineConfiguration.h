#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include "machine/ExactStopTrajectoryPlanner.h"
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

    struct MachineConfiguration {
        Machine::Unit unit = Machine::Unit::Inch;
        TrajectoryLimits trajectory;
        SimulationTiming simulation;
    };

    std::expected<MachineConfiguration, std::string>
    loadMachineConfiguration(const std::filesystem::path &path);
}
