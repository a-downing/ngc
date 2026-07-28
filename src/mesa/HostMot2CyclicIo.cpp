#include "mesa/HostMot2CyclicIo.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string_view>

namespace ngc::mesa {
    namespace {
        constexpr std::uint64_t NANOSECONDS_PER_SECOND = 1'000'000'000;
        constexpr std::uint32_t WATCHDOG_DISABLED = 0x8000'0000;
        constexpr std::uint32_t WATCHDOG_PET = 0x5A00'0000;
        constexpr std::uint32_t WATCHDOG_TRIPPED = 0x0000'0001;
        constexpr std::uint32_t STEPGEN_MASTER_DDS = 0xFFFF'FFFF;
        constexpr std::uint32_t STEPGEN_DPLL_ENABLE = 0x0000'8000;
        constexpr std::uint32_t DPLL_PHASE_LIMIT = 0x0040'0000;
        constexpr std::uint32_t DPLL_TIME_CONSTANT = 2'000;
        constexpr long double DPLL_PHASE_SCALE = 4'294'967'296.0L;
        constexpr std::uint32_t SSR_ENABLE = 0x0000'1000;
        constexpr std::uint32_t SSR_DIVISOR_MASK = 0x0000'0FFF;

        std::uint32_t registerAddress(
            const HostMot2ModuleLayout &module,
            const std::uint32_t registerIndex) noexcept {
            return module.descriptor.baseAddress
                + registerIndex * module.registerStride;
        }

        std::uint32_t littleEndian32(
            const std::span<const std::byte> bytes) noexcept {
            return std::to_integer<std::uint32_t>(bytes[0])
                | (std::to_integer<std::uint32_t>(bytes[1]) << 8)
                | (std::to_integer<std::uint32_t>(bytes[2]) << 16)
                | (std::to_integer<std::uint32_t>(bytes[3]) << 24);
        }

        void putLittleEndian32(
            const std::span<std::byte> bytes,
            const std::uint32_t value) noexcept {
            bytes[0] = static_cast<std::byte>(value);
            bytes[1] = static_cast<std::byte>(value >> 8);
            bytes[2] = static_cast<std::byte>(value >> 16);
            bytes[3] = static_cast<std::byte>(value >> 24);
        }

        void putWords(
            const std::span<std::byte> destination,
            const std::span<const std::uint32_t> words) noexcept {
            for (std::size_t index = 0; index < words.size(); ++index) {
                putLittleEndian32(
                    destination.subspan(index * sizeof(std::uint32_t)),
                    words[index]);
            }
        }

        std::expected<std::uint32_t, std::string> clockTicks(
            const std::uint32_t nanoseconds,
            const std::uint32_t clockHz,
            const std::string_view description) {
            if (nanoseconds == 0 || clockHz == 0) {
                return std::unexpected(std::format(
                    "HostMot2 {} must be positive", description));
            }
            const auto numerator =
                static_cast<std::uint64_t>(nanoseconds) * clockHz;
            const auto ticks =
                (numerator + NANOSECONDS_PER_SECOND - 1)
                / NANOSECONDS_PER_SECOND;
            if (ticks == 0 || ticks > 0x3FFF) {
                return std::unexpected(std::format(
                    "HostMot2 {} requires {} clock ticks; "
                    "the supported range is 1..16383",
                    description, ticks));
            }

            return static_cast<std::uint32_t>(ticks);
        }

        std::expected<std::uint32_t, std::string> watchdogTimer(
            const std::uint32_t nanoseconds,
            const std::uint32_t clockHz) {
            if (nanoseconds == 0 || clockHz == 0) {
                return std::unexpected(
                    "HostMot2 watchdog timeout must be positive");
            }
            const auto numerator =
                static_cast<std::uint64_t>(nanoseconds) * clockHz;
            const auto ticks =
                (numerator + NANOSECONDS_PER_SECOND - 1)
                / NANOSECONDS_PER_SECOND;
            if (ticks == 0 || ticks > 0x8000'0000ULL) {
                return std::unexpected(std::format(
                    "HostMot2 watchdog timeout requires {} clock ticks; "
                    "the supported range is 1..2147483648",
                    ticks));
            }

            return static_cast<std::uint32_t>(ticks - 1);
        }

        std::expected<std::uint32_t, std::string> ssrRate(
            const std::uint32_t frequencyHz,
            const std::uint32_t clockHz,
            const std::size_t instance) {
            if (frequencyHz == 0 || clockHz == 0) {
                return std::unexpected(std::format(
                    "HostMot2 SSR instance {} frequency must be positive",
                    instance));
            }
            const auto denominator =
                static_cast<std::uint64_t>(frequencyHz) * 2;
            if (denominator > clockHz) {
                return std::unexpected(std::format(
                    "HostMot2 SSR instance {} frequency {} Hz "
                    "exceeds the module clock",
                    instance, frequencyHz));
            }
            const auto quotient = clockHz / denominator;
            if (quotient < 2 || quotient - 2 > SSR_DIVISOR_MASK) {
                return std::unexpected(std::format(
                    "HostMot2 SSR instance {} frequency {} Hz "
                    "is outside the supported divisor range",
                    instance, frequencyHz));
            }

            return static_cast<std::uint32_t>(quotient - 2) | SSR_ENABLE;
        }

        HostMot2CyclicIoFault cyclicFault(
            const Lbp16CyclicFault fault) noexcept {
            switch (fault) {
                case Lbp16CyclicFault::None:
                    return HostMot2CyclicIoFault::None;
                case Lbp16CyclicFault::ReadSequenceMismatch:
                    return HostMot2CyclicIoFault::ReadSequenceMismatch;
                case Lbp16CyclicFault::WriteSequenceMismatch:
                    return HostMot2CyclicIoFault::WriteSequenceMismatch;
                case Lbp16CyclicFault::BoardProtocolError:
                    return HostMot2CyclicIoFault::BoardProtocolError;
                case Lbp16CyclicFault::NotFinalized:
                case Lbp16CyclicFault::Transport:
                    return HostMot2CyclicIoFault::Transport;
            }

            return HostMot2CyclicIoFault::Transport;
        }

        std::expected<void, std::string> validateModule(
            const HostMot2ModuleLayout &module,
            const std::uint8_t minimumRegisters,
            const std::size_t instanceCapacity,
            const std::string_view description) {
            if (module.descriptor.instances == 0
                || module.descriptor.instances > instanceCapacity) {
                return std::unexpected(std::format(
                    "HostMot2 {} instance count {} is outside "
                    "the cyclic capacity 1..{}",
                    description, module.descriptor.instances,
                    instanceCapacity));
            }
            if (module.descriptor.registers < minimumRegisters
                || module.instanceStride == 0
                || module.registerStride == 0
                || module.clockHz == 0) {
                return std::unexpected(std::format(
                    "HostMot2 {} layout is incomplete or incompatible",
                    description));
            }

            return {};
        }

        std::expected<void, std::string> validateLayout(
            const HostMot2CyclicLayout &layout) {
            if (layout.ioPortWidth == 0 || layout.ioPortWidth > 32) {
                return std::unexpected(
                    "HostMot2 I/O port width must be in the range 1..32");
            }
            const std::array moduleChecks{
                validateModule(
                    layout.watchdog, 3, 1, "watchdog"),
                validateModule(
                    layout.ioPort, 5,
                    HOSTMOT2_CYCLIC_IO_PORT_CAPACITY, "I/O port"),
                validateModule(
                    layout.stepGenerator, 10,
                    HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY,
                    "step generator"),
            };
            for (const auto &check : moduleChecks) {
                if (!check) {
                    return check;
                }
            }
            if (layout.ssr.descriptor.instances == 0) {
                if (layout.digitalOutputCount != 0) {
                    return std::unexpected(
                        "HostMot2 digital outputs require an SSR module");
                }
            } else if (const auto valid = validateModule(
                    layout.ssr, 2,
                    HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY, "SSR");
                !valid) {
                return valid;
            }
            if (layout.stepGeneratorCount
                    > HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY
                || layout.digitalInputCount
                    > HOSTMOT2_CYCLIC_DIGITAL_INPUT_CAPACITY
                || layout.digitalOutputCount
                    > HOSTMOT2_CYCLIC_DIGITAL_OUTPUT_CAPACITY) {
                return std::unexpected(
                    "HostMot2 cyclic binding count exceeds its capacity");
            }

            const auto pinCount =
                static_cast<std::size_t>(layout.ioPort.descriptor.instances)
                * layout.ioPortWidth;
            std::array<
                bool,
                HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY> selectedChannels{};
            std::array<
                bool,
                HOSTMOT2_CYCLIC_PIN_CAPACITY> selectedPins{};
            for (std::size_t index = 0;
                 index < layout.stepGeneratorCount; ++index) {
                const auto &binding = layout.stepGenerators[index];
                if (binding.channel
                        >= layout.stepGenerator.descriptor.instances
                    || binding.stepPin >= pinCount
                    || binding.directionPin >= pinCount
                    || binding.stepPin >= selectedPins.size()
                    || binding.directionPin >= selectedPins.size()
                    || selectedChannels[binding.channel]
                    || selectedPins[binding.stepPin]
                    || selectedPins[binding.directionPin]) {
                    return std::unexpected(std::format(
                        "HostMot2 step-generator binding {} is invalid "
                        "or duplicated",
                        index));
                }
                selectedChannels[binding.channel] = true;
                selectedPins[binding.stepPin] = true;
                selectedPins[binding.directionPin] = true;
            }

            for (std::size_t index = 0;
                 index < layout.digitalInputCount; ++index) {
                const auto pin = layout.digitalInputs[index].pin;
                if (pin >= pinCount
                    || pin >= selectedPins.size()
                    || selectedPins[pin]) {
                    return std::unexpected(std::format(
                        "HostMot2 digital-input binding {} is invalid "
                        "or duplicated",
                        index));
                }
                selectedPins[pin] = true;
            }

            std::array<
                std::uint32_t,
                HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY> selectedOutputs{};
            for (std::size_t index = 0;
                 index < layout.digitalOutputCount; ++index) {
                const auto &binding = layout.digitalOutputs[index];
                if (binding.instance >= layout.ssr.descriptor.instances
                    || binding.output >= 32
                    || binding.pin >= pinCount
                    || binding.pin >= selectedPins.size()
                    || selectedPins[binding.pin]
                    || (selectedOutputs[binding.instance]
                        & (std::uint32_t{1} << binding.output)) != 0) {
                    return std::unexpected(std::format(
                        "HostMot2 digital-output binding {} is invalid "
                        "or duplicated",
                        index));
                }
                const auto safePhysicalLevel =
                    binding.safeState != binding.activeLow;
                if (safePhysicalLevel) {
                    return std::unexpected(std::format(
                        "HostMot2 digital-output binding {} cannot establish "
                        "its configured safe state while the SSR is disabled",
                        index));
                }
                selectedPins[binding.pin] = true;
                selectedOutputs[binding.instance] |=
                    std::uint32_t{1} << binding.output;
            }

            return {};
        }

        std::expected<void, std::string> validateDpll(
            const HostMot2CyclicLayout &layout,
            const HostMot2DpllConfiguration &configuration) {
            if (!configuration.enabled) {
                return {};
            }
            if (const auto valid = validateModule(
                    layout.dpll, 7, 1, "DPLL"); !valid) {
                return valid;
            }
            if (configuration.stepGeneratorTimer < 1
                || configuration.stepGeneratorTimer > 4) {
                return std::unexpected(
                    "HostMot2 DPLL StepGen timer must be in the range 1..4");
            }
            if (configuration.servoPeriodNanoseconds == 0
                || configuration.stepGeneratorSampleOffsetNanoseconds >= 0
                || static_cast<std::uint64_t>(
                    -static_cast<std::int64_t>(
                        configuration.stepGeneratorSampleOffsetNanoseconds))
                    >= configuration.servoPeriodNanoseconds) {
                return std::unexpected(
                    "HostMot2 DPLL StepGen sample offset must be negative "
                    "and shorter than the servo period");
            }
            if (configuration.maximumPhaseErrorNanoseconds == 0
                || configuration.maximumPhaseErrorNanoseconds
                    >= static_cast<std::uint64_t>(
                        -static_cast<std::int64_t>(
                            configuration
                                .stepGeneratorSampleOffsetNanoseconds))) {
                return std::unexpected(
                    "HostMot2 DPLL maximum phase error must be positive "
                    "and smaller than the StepGen sample lead");
            }
            if (configuration.convergenceCycles == 0) {
                return std::unexpected(
                    "HostMot2 DPLL convergence cycle count must be positive");
            }

            return {};
        }

        std::uint16_t dpllTimerValue(
            const HostMot2DpllConfiguration &configuration) noexcept {
            const auto lead = -static_cast<std::int64_t>(
                configuration.stepGeneratorSampleOffsetNanoseconds);
            const auto scaled =
                lead * 0x1'0000LL
                / configuration.servoPeriodNanoseconds;

            return static_cast<std::uint16_t>(scaled);
        }
    }

    class HostMot2CyclicIo::Impl {
    public:
        Impl(
            Lbp16DatagramTransport &transport,
            const HostMot2CyclicLayout &layout,
            const HostMot2CyclicConfiguration &configuration) noexcept
            : dpllProbeTransaction(transport),
              setupTransaction(transport),
              cyclicTransaction(transport),
              layout(layout),
              configuration(configuration) { }

        std::expected<void, std::string> build() {
            if (const auto valid = validateLayout(layout); !valid) {
                return valid;
            }
            if (const auto valid = validateDpll(
                    layout, configuration.dpll); !valid) {
                return valid;
            }
            if (const auto timer = watchdogTimer(
                    configuration.watchdogTimeoutNanoseconds,
                    layout.watchdog.clockHz); timer) {
                watchdogTimerRegister = *timer;
            } else {
                return std::unexpected(timer.error());
            }

            const auto stepGeneratorInstances =
                layout.stepGenerator.descriptor.instances;
            for (std::size_t bindingIndex = 0;
                 bindingIndex < layout.stepGeneratorCount; ++bindingIndex) {
                const auto &binding = layout.stepGenerators[bindingIndex];
                const std::array timingResults{
                    clockTicks(
                        binding.timing.stepLengthNanoseconds,
                        layout.stepGenerator.clockHz, "step length"),
                    clockTicks(
                        binding.timing.stepSpaceNanoseconds,
                        layout.stepGenerator.clockHz, "step space"),
                    clockTicks(
                        binding.timing.directionSetupNanoseconds,
                        layout.stepGenerator.clockHz,
                        "direction setup time"),
                    clockTicks(
                        binding.timing.directionHoldNanoseconds,
                        layout.stepGenerator.clockHz,
                        "direction hold time"),
                };
                for (const auto &timing : timingResults) {
                    if (!timing) {
                        return std::unexpected(timing.error());
                    }
                }
                const auto channel = binding.channel;
                stepLengthRegisters[channel] = *timingResults[0];
                stepSpaceRegisters[channel] = *timingResults[1];
                directionSetupRegisters[channel] = *timingResults[2];
                directionHoldRegisters[channel] = *timingResults[3];
            }

            const auto ssrInstances = layout.ssr.descriptor.instances;
            for (std::size_t instance = 0;
                 instance < ssrInstances; ++instance) {
                const auto used = std::ranges::any_of(
                    std::span(layout.digitalOutputs).first(
                        layout.digitalOutputCount),
                    [instance](const HostMot2DigitalOutputBinding &binding) {
                        return binding.instance == instance;
                    });
                if (!used) {
                    continue;
                }
                const auto rate = ssrRate(
                    configuration.isolatedOutputFrequencyHz[instance],
                    layout.ssr.clockHz, instance);
                if (!rate) {
                    return std::unexpected(rate.error());
                }
                ssrRateRegisters[instance] = *rate;
            }

            const auto ioPortInstances = layout.ioPort.descriptor.instances;
            for (std::size_t index = 0;
                 index < layout.stepGeneratorCount; ++index) {
                setOutputPin(layout.stepGenerators[index].stepPin);
                setOutputPin(layout.stepGenerators[index].directionPin);
            }
            for (std::size_t index = 0;
                 index < layout.digitalOutputCount; ++index) {
                setOutputPin(layout.digitalOutputs[index].pin);
            }

            if (configuration.dpll.enabled) {
                const auto dpllControl = dpllProbeTransaction.addHostMot2Read(
                    registerAddress(layout.dpll, 3),
                    sizeof(std::uint32_t));
                if (!dpllControl) {
                    return std::unexpected(
                        "could not construct HostMot2 DPLL probe");
                }
                dpllProbeControl = *dpllControl;
                if (const auto finalized =
                        dpllProbeTransaction.finalize();
                    !finalized) {
                    return finalized;
                }
                if (const auto result = addSetupWrite(
                        setupDpllPhaseError, layout.dpll, 1, 1);
                    !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupDpllBaseRate, layout.dpll, 0, 1);
                    !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupDpllControl0, layout.dpll, 2, 1);
                    !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupDpllControl1, layout.dpll, 3, 1);
                    !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupDpllTimer12, layout.dpll, 4, 1);
                    !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupDpllTimer34, layout.dpll, 5, 1);
                    !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupStepGeneratorDpllTimer,
                        layout.stepGenerator, 10, 1);
                    !result) {
                    return result;
                }
            }

            if (const auto result = addSetupWrite(
                    setupStepRates, layout.stepGenerator, 0,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupSafeDataDirection, layout.ioPort, 1,
                    ioPortInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupStepModes, layout.stepGenerator, 2,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupDirectionSetup, layout.stepGenerator, 3,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupDirectionHold, layout.stepGenerator, 4,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupStepLength, layout.stepGenerator, 5,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupStepSpace, layout.stepGenerator, 6,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupMasterDds, layout.stepGenerator, 9, 1); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupIoData, layout.ioPort, 0,
                    ioPortInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupOutputInvert, layout.ioPort, 4,
                    ioPortInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupOpenDrain, layout.ioPort, 3,
                    ioPortInstances); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupAlternateSource, layout.ioPort, 2,
                    ioPortInstances); !result) {
                return result;
            }
            if (ssrInstances != 0) {
                if (const auto result = addSetupWrite(
                        setupSsrData, layout.ssr, 0,
                        ssrInstances); !result) {
                    return result;
                }
                if (const auto result = addSetupWrite(
                        setupSsrRate, layout.ssr, 1,
                        ssrInstances); !result) {
                    return result;
                }
            }
            if (const auto result = addSetupWrite(
                    setupWatchdogTimer, layout.watchdog, 0, 1); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupWatchdogStatus, layout.watchdog, 1, 1); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupWatchdogReset, layout.watchdog, 2, 1); !result) {
                return result;
            }
            if (const auto result = addSetupWrite(
                    setupDataDirection, layout.ioPort, 1,
                    ioPortInstances); !result) {
                return result;
            }
            if (const auto finalized = setupTransaction.finalize();
                !finalized) {
                return finalized;
            }

            putWords(
                setupTransaction.writeData(setupDirectionSetup),
                std::span(directionSetupRegisters).first(
                    stepGeneratorInstances));
            putWords(
                setupTransaction.writeData(setupDirectionHold),
                std::span(directionHoldRegisters).first(
                    stepGeneratorInstances));
            putWords(
                setupTransaction.writeData(setupStepLength),
                std::span(stepLengthRegisters).first(
                    stepGeneratorInstances));
            putWords(
                setupTransaction.writeData(setupStepSpace),
                std::span(stepSpaceRegisters).first(
                    stepGeneratorInstances));
            putLittleEndian32(
                setupTransaction.writeData(setupMasterDds),
                STEPGEN_MASTER_DDS);
            putWords(
                setupTransaction.writeData(setupAlternateSource),
                std::span(alternateSourceRegisters).first(
                    ioPortInstances));
            putLittleEndian32(
                setupTransaction.writeData(setupWatchdogTimer),
                WATCHDOG_DISABLED);
            putLittleEndian32(
                setupTransaction.writeData(setupWatchdogReset),
                WATCHDOG_PET);
            putWords(
                setupTransaction.writeData(setupDataDirection),
                std::span(dataDirectionRegisters).first(
                    ioPortInstances));
            if (configuration.dpll.enabled) {
                const auto timerValue = dpllTimerValue(
                    configuration.dpll);
                auto timer12 = std::uint32_t{};
                auto timer34 = std::uint32_t{};
                const auto timerShift =
                    ((configuration.dpll.stepGeneratorTimer - 1) % 2) * 16;
                if (configuration.dpll.stepGeneratorTimer <= 2) {
                    timer12 = static_cast<std::uint32_t>(
                        timerValue) << timerShift;
                } else {
                    timer34 = static_cast<std::uint32_t>(
                        timerValue) << timerShift;
                }
                putLittleEndian32(
                    setupTransaction.writeData(setupDpllPhaseError), 0);
                putLittleEndian32(
                    setupTransaction.writeData(setupDpllControl1),
                    DPLL_TIME_CONSTANT << 16);
                putLittleEndian32(
                    setupTransaction.writeData(setupDpllTimer12),
                    timer12);
                putLittleEndian32(
                    setupTransaction.writeData(setupDpllTimer34),
                    timer34);
                putLittleEndian32(
                    setupTransaction.writeData(
                        setupStepGeneratorDpllTimer),
                    STEPGEN_DPLL_ENABLE
                        | (static_cast<std::uint32_t>(
                            configuration.dpll.stepGeneratorTimer) << 12));
            }

            if (const auto result = addCyclicWrite(
                    cyclicStepRates, layout.stepGenerator, 0,
                    stepGeneratorInstances); !result) {
                return result;
            }
            if (ssrInstances != 0) {
                if (const auto result = addCyclicWrite(
                        cyclicSsrData, layout.ssr, 0,
                        ssrInstances); !result) {
                    return result;
                }
                if (const auto result = addCyclicWrite(
                        cyclicSsrRate, layout.ssr, 1,
                        ssrInstances); !result) {
                    return result;
                }
            }
            if (const auto result = addCyclicWrite(
                    cyclicWatchdogTimer, layout.watchdog, 0, 1); !result) {
                return result;
            }
            if (const auto result = addCyclicWrite(
                    cyclicWatchdogReset, layout.watchdog, 2, 1); !result) {
                return result;
            }
            const auto ioData = cyclicTransaction.addHostMot2Read(
                registerAddress(layout.ioPort, 0),
                ioPortInstances * sizeof(std::uint32_t));
            const auto accumulators = cyclicTransaction.addHostMot2Read(
                registerAddress(layout.stepGenerator, 1),
                stepGeneratorInstances * sizeof(std::uint32_t));
            const auto watchdogStatus = cyclicTransaction.addHostMot2Read(
                registerAddress(layout.watchdog, 1),
                sizeof(std::uint32_t));
            auto dpllSync = std::expected<
                Lbp16CyclicRead, std::string>{
                Lbp16CyclicRead{},
            };
            if (configuration.dpll.enabled) {
                dpllSync = cyclicTransaction.addHostMot2Read(
                    registerAddress(layout.dpll, 6),
                    sizeof(std::uint32_t));
            }
            if (!ioData || !accumulators || !watchdogStatus || !dpllSync) {
                return std::unexpected(
                    "could not construct HostMot2 cyclic input transaction");
            }
            cyclicIoData = *ioData;
            cyclicStepAccumulators = *accumulators;
            cyclicWatchdogStatus = *watchdogStatus;
            if (configuration.dpll.enabled) {
                cyclicDpllSync = *dpllSync;
            }
            if (const auto finalized = cyclicTransaction.finalize();
                !finalized) {
                return finalized;
            }

            return {};
        }

        HostMot2CyclicIoResult initializeSafe() noexcept {
            if (latchedResult.fault != HostMot2CyclicIoFault::None) {
                return latchedResult;
            }
            if (isInitialized) {
                return {};
            }

            auto result = HostMot2CyclicIoResult{};
            if (configuration.dpll.enabled) {
                result.transaction = dpllProbeTransaction.exchange(
                    nextReadSequence, nextWriteSequence);
                advanceSequences();
                result.fault = cyclicFault(result.transaction.fault);
                if (result.fault != HostMot2CyclicIoFault::None) {
                    return latch(result);
                }
                const auto control =
                    dpllProbeTransaction.readData(dpllProbeControl);
                if (control.size() != sizeof(std::uint32_t)
                    || !configureDpll(
                        littleEndian32(control) & 0xFF)) {
                    result.fault =
                        HostMot2CyclicIoFault::BoardProtocolError;

                    return latch(result);
                }
            }

            result.transaction = setupTransaction.exchange(
                nextReadSequence, nextWriteSequence);
            advanceSequences();
            result.fault = cyclicFault(result.transaction.fault);
            if (result.fault != HostMot2CyclicIoFault::None) {
                return latch(result);
            }

            isInitialized = true;

            return result;
        }

        HostMot2CyclicIoResult cycle(
            const HostMot2CyclicOutputImage &outputs) noexcept {
            if (latchedResult.fault != HostMot2CyclicIoFault::None) {
                return latchedResult;
            }
            if (!isInitialized) {
                auto result = HostMot2CyclicIoResult{};
                result.fault = HostMot2CyclicIoFault::NotInitialized;

                return result;
            }
            if ((outputs.stepGeneratorsEnabled
                    || outputs.digitalOutputsEnabled)
                && !outputs.watchdogEnabled) {
                auto result = HostMot2CyclicIoResult{};
                result.fault = HostMot2CyclicIoFault::InvalidOutput;

                return latch(result);
            }
            if (outputs.digitalOutputsEnabled
                && layout.digitalOutputCount == 0) {
                auto result = HostMot2CyclicIoResult{};
                result.fault = HostMot2CyclicIoFault::InvalidOutput;

                return latch(result);
            }

            std::array<
                std::uint32_t,
                HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY> stepRates{};
            for (std::size_t index = 0;
                 index < layout.stepGeneratorCount; ++index) {
                const auto &command = outputs.stepGenerators[index];
                if (!outputs.stepGeneratorsEnabled || !command.enabled) {
                    continue;
                }
                auto stepsPerSecond = command.stepsPerSecond;
                if (layout.stepGenerators[index].invertDirection) {
                    stepsPerSecond = -stepsPerSecond;
                }
                const auto &timing =
                    layout.stepGenerators[index].timing;
                const auto minimumStepPeriod =
                    static_cast<std::uint64_t>(
                        timing.stepLengthNanoseconds)
                    + timing.stepSpaceNanoseconds;
                const auto maximumStepsPerSecond =
                    static_cast<double>(NANOSECONDS_PER_SECOND)
                    / minimumStepPeriod;
                const auto scaled =
                    stepsPerSecond * 4'294'967'296.0
                    / layout.stepGenerator.clockHz;
                if (!std::isfinite(scaled)
                    || std::abs(stepsPerSecond)
                        > maximumStepsPerSecond
                    || scaled
                        < std::numeric_limits<std::int32_t>::min()
                    || scaled
                        > std::numeric_limits<std::int32_t>::max()) {
                    auto result = HostMot2CyclicIoResult{};
                    result.fault = HostMot2CyclicIoFault::InvalidOutput;

                    return latch(result);
                }
                stepRates[layout.stepGenerators[index].channel] =
                    configuration.dpll.enabled && !dpllReady
                    ? 0
                    : std::bit_cast<std::uint32_t>(
                        static_cast<std::int32_t>(scaled));
            }

            std::array<
                std::uint32_t,
                HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY> ssrData{};
            std::array<
                std::uint32_t,
                HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY> enabledSsrRates{};
            if (outputs.digitalOutputsEnabled) {
                for (std::size_t index = 0;
                     index < layout.digitalOutputCount; ++index) {
                    const auto &binding = layout.digitalOutputs[index];
                    const auto physicalLevel =
                        outputs.digitalOutputs[index]
                        != binding.activeLow;
                    if (physicalLevel) {
                        ssrData[binding.instance] |=
                            std::uint32_t{1} << binding.output;
                    }
                    enabledSsrRates[binding.instance] =
                        ssrRateRegisters[binding.instance];
                }
            }

            putWords(
                cyclicTransaction.writeData(cyclicStepRates),
                std::span(stepRates).first(
                    layout.stepGenerator.descriptor.instances));
            if (layout.ssr.descriptor.instances != 0) {
                putWords(
                    cyclicTransaction.writeData(cyclicSsrData),
                    std::span(ssrData).first(
                        layout.ssr.descriptor.instances));
                putWords(
                    cyclicTransaction.writeData(cyclicSsrRate),
                    std::span(enabledSsrRates).first(
                        layout.ssr.descriptor.instances));
            }
            putLittleEndian32(
                cyclicTransaction.writeData(cyclicWatchdogTimer),
                outputs.watchdogEnabled
                    ? watchdogTimerRegister
                    : WATCHDOG_DISABLED);
            putLittleEndian32(
                cyclicTransaction.writeData(cyclicWatchdogReset),
                WATCHDOG_PET);

            auto result = HostMot2CyclicIoResult{};
            result.transaction = cyclicTransaction.exchange(
                nextReadSequence, nextWriteSequence);
            advanceSequences();
            result.fault = cyclicFault(result.transaction.fault);
            if (result.fault != HostMot2CyclicIoFault::None) {
                return latch(result);
            }

            const auto watchdogStatus =
                cyclicTransaction.readData(cyclicWatchdogStatus);
            if (watchdogStatus.size() != sizeof(std::uint32_t)
                || (littleEndian32(watchdogStatus)
                    & WATCHDOG_TRIPPED) != 0) {
                result.fault = HostMot2CyclicIoFault::WatchdogTripped;

                return latch(result);
            }

            auto nextInput = HostMot2CyclicInputImage{};
            if (configuration.dpll.enabled) {
                const auto sync =
                    cyclicTransaction.readData(cyclicDpllSync);
                if (sync.size() != sizeof(std::uint32_t)) {
                    result.fault =
                        HostMot2CyclicIoFault::BoardProtocolError;

                    return latch(result);
                }
                const auto rawPhase = std::bit_cast<std::int32_t>(
                    littleEndian32(sync));
                const auto phaseNanoseconds = std::llround(
                    static_cast<long double>(rawPhase)
                    * configuration.dpll.servoPeriodNanoseconds
                    / DPLL_PHASE_SCALE);
                if (phaseNanoseconds
                        < std::numeric_limits<std::int32_t>::min()
                    || phaseNanoseconds
                        > std::numeric_limits<std::int32_t>::max()) {
                    result.fault =
                        HostMot2CyclicIoFault::BoardProtocolError;

                    return latch(result);
                }
                result.dpllPhaseErrorNanoseconds =
                    static_cast<std::int32_t>(phaseNanoseconds);
                result.dpllPhaseErrorValid = true;
                const auto phaseMagnitude =
                    static_cast<std::uint64_t>(std::abs(
                        phaseNanoseconds));
                if (phaseMagnitude
                    > configuration.dpll
                        .maximumPhaseErrorNanoseconds) {
                    dpllConvergenceCycles = 0;
                    if (dpllReady) {
                        result.fault =
                            HostMot2CyclicIoFault::DpllPhaseError;

                        return latch(result);
                    }
                } else if (!dpllReady) {
                    ++dpllConvergenceCycles;
                    dpllReady = dpllConvergenceCycles
                        >= configuration.dpll.convergenceCycles;
                }
                nextInput.dpll = {
                    .phaseErrorNanoseconds =
                        static_cast<std::int32_t>(phaseNanoseconds),
                    .enabled = true,
                    .ready = dpllReady,
                };
            }
            const auto ioData =
                cyclicTransaction.readData(cyclicIoData);
            for (std::size_t index = 0;
                 index < layout.digitalInputCount; ++index) {
                const auto &binding = layout.digitalInputs[index];
                const auto port = binding.pin / layout.ioPortWidth;
                const auto portPin = binding.pin % layout.ioPortWidth;
                const auto word = littleEndian32(
                    ioData.subspan(port * sizeof(std::uint32_t)));
                const auto physicalLevel =
                    (word & (std::uint32_t{1} << portPin)) != 0;
                nextInput.fieldDigitalInputs[index] = physicalLevel;
            }

            auto nextExtended = extendedStepAccumulator;
            auto nextPrevious = previousRawStepAccumulator;
            const auto accumulators =
                cyclicTransaction.readData(cyclicStepAccumulators);
            for (std::size_t index = 0;
                 index < layout.stepGeneratorCount; ++index) {
                const auto channel = layout.stepGenerators[index].channel;
                const auto raw = littleEndian32(
                    accumulators.subspan(
                        channel * sizeof(std::uint32_t)));
                const auto delta = std::bit_cast<std::int32_t>(
                    raw - previousRawStepAccumulator[channel]);
                nextExtended[channel] += delta;
                nextPrevious[channel] = raw;
                nextInput.rawStepAccumulator[index] = raw;
                nextInput.stepAccumulatorSubcounts[index] =
                    nextExtended[channel];
            }
            extendedStepAccumulator = nextExtended;
            previousRawStepAccumulator = nextPrevious;
            input = nextInput;
            result.inputsValid = true;

            return result;
        }

        bool configureDpll(
            const std::uint32_t accumulatorBits) noexcept {
            if (accumulatorBits == 0 || accumulatorBits > 62) {
                return false;
            }
            const auto accumulatorScale =
                static_cast<long double>(
                    std::uint64_t{1} << accumulatorBits);
            const auto baseFrequency =
                static_cast<long double>(NANOSECONDS_PER_SECOND)
                / configuration.dpll.servoPeriodNanoseconds;
            const auto prescale = std::max(
                std::uint64_t{1},
                static_cast<std::uint64_t>(
                    (static_cast<long double>(
                        std::uint64_t{1} << 30)
                        * layout.dpll.clockHz)
                    / (accumulatorScale * baseFrequency)));
            if (prescale > 0xFF) {
                return false;
            }
            const auto baseRate = static_cast<std::uint64_t>(
                baseFrequency * accumulatorScale * prescale
                / layout.dpll.clockHz);
            if (baseRate == 0
                || baseRate > std::numeric_limits<std::uint32_t>::max()) {
                return false;
            }

            putLittleEndian32(
                setupTransaction.writeData(setupDpllBaseRate),
                static_cast<std::uint32_t>(baseRate));
            putLittleEndian32(
                setupTransaction.writeData(setupDpllControl0),
                static_cast<std::uint32_t>(prescale << 24)
                    | DPLL_PHASE_LIMIT);

            return true;
        }

        std::expected<void, std::string> addSetupWrite(
            Lbp16CyclicWrite &handle,
            const HostMot2ModuleLayout &module,
            const std::uint32_t registerIndex,
            const std::size_t wordCount) {
            const auto added = setupTransaction.addHostMot2Write(
                registerAddress(module, registerIndex),
                wordCount * sizeof(std::uint32_t));
            if (!added) {
                return std::unexpected(added.error());
            }
            handle = *added;

            return {};
        }

        std::expected<void, std::string> addCyclicWrite(
            Lbp16CyclicWrite &handle,
            const HostMot2ModuleLayout &module,
            const std::uint32_t registerIndex,
            const std::size_t wordCount) {
            const auto added = cyclicTransaction.addHostMot2Write(
                registerAddress(module, registerIndex),
                wordCount * sizeof(std::uint32_t));
            if (!added) {
                return std::unexpected(added.error());
            }
            handle = *added;

            return {};
        }

        void setOutputPin(const std::uint16_t pin) noexcept {
            const auto port = pin / layout.ioPortWidth;
            const auto portPin = pin % layout.ioPortWidth;
            const auto mask = std::uint32_t{1} << portPin;
            alternateSourceRegisters[port] |= mask;
            dataDirectionRegisters[port] |= mask;
        }

        void advanceSequences() noexcept {
            ++nextReadSequence;
            ++nextWriteSequence;
        }

        HostMot2CyclicIoResult latch(
            const HostMot2CyclicIoResult &result) noexcept {
            input = {};
            latchedResult = result;

            return latchedResult;
        }

        Lbp16CyclicTransaction dpllProbeTransaction;
        Lbp16CyclicTransaction setupTransaction;
        Lbp16CyclicTransaction cyclicTransaction;
        HostMot2CyclicLayout layout;
        HostMot2CyclicConfiguration configuration;

        Lbp16CyclicRead dpllProbeControl;
        Lbp16CyclicWrite setupDpllPhaseError;
        Lbp16CyclicWrite setupDpllBaseRate;
        Lbp16CyclicWrite setupDpllControl0;
        Lbp16CyclicWrite setupDpllControl1;
        Lbp16CyclicWrite setupDpllTimer12;
        Lbp16CyclicWrite setupDpllTimer34;
        Lbp16CyclicWrite setupStepGeneratorDpllTimer;
        Lbp16CyclicWrite setupStepRates;
        Lbp16CyclicWrite setupSafeDataDirection;
        Lbp16CyclicWrite setupStepModes;
        Lbp16CyclicWrite setupDirectionSetup;
        Lbp16CyclicWrite setupDirectionHold;
        Lbp16CyclicWrite setupStepLength;
        Lbp16CyclicWrite setupStepSpace;
        Lbp16CyclicWrite setupMasterDds;
        Lbp16CyclicWrite setupIoData;
        Lbp16CyclicWrite setupOutputInvert;
        Lbp16CyclicWrite setupOpenDrain;
        Lbp16CyclicWrite setupAlternateSource;
        Lbp16CyclicWrite setupSsrData;
        Lbp16CyclicWrite setupSsrRate;
        Lbp16CyclicWrite setupWatchdogTimer;
        Lbp16CyclicWrite setupWatchdogStatus;
        Lbp16CyclicWrite setupWatchdogReset;
        Lbp16CyclicWrite setupDataDirection;

        Lbp16CyclicWrite cyclicStepRates;
        Lbp16CyclicWrite cyclicSsrData;
        Lbp16CyclicWrite cyclicSsrRate;
        Lbp16CyclicWrite cyclicWatchdogTimer;
        Lbp16CyclicWrite cyclicWatchdogReset;
        Lbp16CyclicRead cyclicIoData;
        Lbp16CyclicRead cyclicStepAccumulators;
        Lbp16CyclicRead cyclicWatchdogStatus;
        Lbp16CyclicRead cyclicDpllSync;

        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            directionSetupRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            directionHoldRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            stepLengthRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            stepSpaceRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_IO_PORT_CAPACITY>
            alternateSourceRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_IO_PORT_CAPACITY>
            dataDirectionRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY>
            ssrRateRegisters{};
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            previousRawStepAccumulator{};
        std::array<
            std::int64_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            extendedStepAccumulator{};

        HostMot2CyclicInputImage input;
        HostMot2CyclicIoResult latchedResult;
        std::uint32_t watchdogTimerRegister = 0;
        std::uint32_t nextReadSequence = 1;
        std::uint32_t nextWriteSequence = 1;
        std::uint32_t dpllConvergenceCycles = 0;
        bool isInitialized = false;
        bool dpllReady = false;
    };

    std::expected<std::unique_ptr<HostMot2CyclicIo>, std::string>
    HostMot2CyclicIo::create(
        Lbp16DatagramTransport &transport,
        const HostMot2CyclicLayout &layout,
        const HostMot2CyclicConfiguration &configuration) {
        auto impl = std::make_unique<Impl>(
            transport, layout, configuration);
        if (const auto built = impl->build(); !built) {
            return std::unexpected(built.error());
        }

        return std::unique_ptr<HostMot2CyclicIo>(
            new HostMot2CyclicIo(std::move(impl)));
    }

    HostMot2CyclicIo::HostMot2CyclicIo(
        std::unique_ptr<Impl> impl) noexcept : m_impl(std::move(impl)) { }

    HostMot2CyclicIo::~HostMot2CyclicIo() = default;

    HostMot2CyclicIoResult
    HostMot2CyclicIo::initializeSafe() noexcept {
        return m_impl->initializeSafe();
    }

    HostMot2CyclicIoResult HostMot2CyclicIo::cycle(
        const HostMot2CyclicOutputImage &outputs) noexcept {
        return m_impl->cycle(outputs);
    }

    const HostMot2CyclicInputImage &
    HostMot2CyclicIo::inputImage() const noexcept {
        return m_impl->input;
    }

    HostMot2CyclicIoFault HostMot2CyclicIo::fault() const noexcept {
        return m_impl->latchedResult.fault;
    }

    bool HostMot2CyclicIo::initialized() const noexcept {
        return m_impl->isInitialized;
    }

    std::size_t HostMot2CyclicIo::stepGeneratorCount() const noexcept {
        return m_impl->layout.stepGeneratorCount;
    }

    std::size_t HostMot2CyclicIo::digitalInputCount() const noexcept {
        return m_impl->layout.digitalInputCount;
    }

    std::size_t HostMot2CyclicIo::digitalOutputCount() const noexcept {
        return m_impl->layout.digitalOutputCount;
    }

    const HostMot2DpllConfiguration &
    HostMot2CyclicIo::dpllConfiguration() const noexcept {
        return m_impl->configuration.dpll;
    }
}
