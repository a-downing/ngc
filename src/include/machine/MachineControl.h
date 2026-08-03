#pragma once

#include <cstdint>

namespace ngc {
    enum class MachineControlTarget {
        Simulation,
        Machine,
    };

    struct MachineControlAuthority {
        MachineControlTarget target = MachineControlTarget::Simulation;
        std::uint64_t generation = 0;

        bool operator==(const MachineControlAuthority &) const = default;
    };

    struct MachineSessionManagerState {
        MachineControlAuthority authority;
        bool simulationAvailable = true;
        bool machineAvailable = false;
        bool guiEmergencyStopLatched = false;
    };
}
