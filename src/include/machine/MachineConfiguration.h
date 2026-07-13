#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include "machine/ExactStopTrajectoryPlanner.h"
#include "machine/Machine.h"

namespace ngc {
    struct MachineConfiguration {
        Machine::Unit unit = Machine::Unit::Inch;
        TrajectoryLimits trajectory;
    };

    std::expected<MachineConfiguration, std::string>
    loadMachineConfiguration(const std::filesystem::path &path);
}
