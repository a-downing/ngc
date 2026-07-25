#pragma once

#include <cstdint>

namespace ngc {
    enum class MachineControlTarget {
        Simulation,
        Real,
    };

    struct MachineControlAuthority {
        MachineControlTarget target = MachineControlTarget::Simulation;
        std::uint64_t generation = 0;

        bool operator==(const MachineControlAuthority &) const = default;
    };

    struct MachineSessionManagerState {
        MachineControlAuthority authority;
        bool simulationAvailable = true;
        bool realAvailable = false;
    };
}
