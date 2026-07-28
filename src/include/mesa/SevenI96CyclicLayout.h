#pragma once

#include "mesa/HostMot2CyclicIo.h"
#include "mesa/SevenI96Capabilities.h"

namespace ngc::mesa {
    [[nodiscard]] HostMot2CyclicLayout sevenI96CyclicLayout(
        const SevenI96Capabilities &capabilities,
        const HostMot2StepTiming &stepTiming);
}
