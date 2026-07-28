#include "mesa/SevenI96CyclicLayout.h"

namespace ngc::mesa {
    HostMot2CyclicLayout sevenI96CyclicLayout(
        const SevenI96Capabilities &capabilities,
        const HostMot2StepTiming &stepTiming) {
        auto result = HostMot2CyclicLayout{
            .dpll = capabilities.dpll,
            .watchdog = capabilities.watchdog,
            .ioPort = capabilities.ioPort,
            .stepGenerator = capabilities.stepGenerator,
            .ssr = capabilities.ssr,
            .ioPortWidth = 17,
        };

        result.stepGeneratorCount = capabilities.stepGenerators.size();
        for (std::size_t index = 0;
             index < result.stepGeneratorCount; ++index) {
            const auto &stepGenerator = capabilities.stepGenerators[index];
            result.stepGenerators[index] = {
                .channel = stepGenerator.channel,
                .stepPin = stepGenerator.stepPin,
                .directionPin = stepGenerator.directionPin,
                .invertDirection = false,
                .timing = stepTiming,
            };
        }

        result.digitalInputCount = capabilities.isolatedInputPins.size();
        for (std::size_t index = 0;
             index < result.digitalInputCount; ++index) {
            result.digitalInputs[index] = {
                .pin = capabilities.isolatedInputPins[index],
            };
        }

        result.digitalOutputCount = capabilities.isolatedOutputPins.size();
        for (std::size_t index = 0;
             index < result.digitalOutputCount; ++index) {
            result.digitalOutputs[index] = {
                .instance = 0,
                .output = static_cast<std::uint8_t>(index),
                .pin = capabilities.isolatedOutputPins[index],
                .activeLow = false,
                .safeState = false,
            };
        }

        return result;
    }
}
