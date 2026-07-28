#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "mesa/HostMot2Discovery.h"
#include "mesa/Lbp16CyclicTransaction.h"

namespace ngc::mesa {
    inline constexpr std::size_t HOSTMOT2_CYCLIC_IO_PORT_CAPACITY = 32;
    inline constexpr std::size_t HOSTMOT2_CYCLIC_PIN_CAPACITY =
        HOSTMOT2_CYCLIC_IO_PORT_CAPACITY * 32;
    inline constexpr std::size_t
        HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY = 32;
    inline constexpr std::size_t
        HOSTMOT2_CYCLIC_DIGITAL_INPUT_CAPACITY = 128;
    inline constexpr std::size_t
        HOSTMOT2_CYCLIC_DIGITAL_OUTPUT_CAPACITY = 128;
    inline constexpr std::size_t
        HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY = 8;

    struct HostMot2StepTiming {
        std::uint32_t stepLengthNanoseconds = 0;
        std::uint32_t stepSpaceNanoseconds = 0;
        std::uint32_t directionSetupNanoseconds = 0;
        std::uint32_t directionHoldNanoseconds = 0;
    };

    struct HostMot2StepGeneratorBinding {
        std::uint8_t channel = 0;
        std::uint16_t stepPin = 0;
        std::uint16_t directionPin = 0;
        bool invertDirection = false;
        HostMot2StepTiming timing;
    };

    struct HostMot2DigitalInputBinding {
        std::uint16_t pin = 0;
    };

    struct HostMot2DigitalOutputBinding {
        std::uint8_t instance = 0;
        std::uint8_t output = 0;
        std::uint16_t pin = 0;
        bool activeLow = false;
        bool safeState = false;
    };

    struct HostMot2CyclicLayout {
        HostMot2ModuleLayout dpll;
        HostMot2ModuleLayout watchdog;
        HostMot2ModuleLayout ioPort;
        HostMot2ModuleLayout stepGenerator;
        HostMot2ModuleLayout ssr;
        std::uint16_t ioPortWidth = 0;
        std::array<
            HostMot2StepGeneratorBinding,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY> stepGenerators{};
        std::size_t stepGeneratorCount = 0;
        std::array<
            HostMot2DigitalInputBinding,
            HOSTMOT2_CYCLIC_DIGITAL_INPUT_CAPACITY> digitalInputs{};
        std::size_t digitalInputCount = 0;
        std::array<
            HostMot2DigitalOutputBinding,
            HOSTMOT2_CYCLIC_DIGITAL_OUTPUT_CAPACITY> digitalOutputs{};
        std::size_t digitalOutputCount = 0;
    };

    struct HostMot2DpllConfiguration {
        bool enabled = false;
        std::uint8_t stepGeneratorTimer = 0;
        std::int32_t stepGeneratorSampleOffsetNanoseconds = 0;
        std::uint32_t servoPeriodNanoseconds = 0;
        std::uint32_t maximumPhaseErrorNanoseconds = 0;
        std::uint32_t convergenceCycles = 0;
    };

    struct HostMot2CyclicConfiguration {
        std::uint32_t watchdogTimeoutNanoseconds = 0;
        HostMot2DpllConfiguration dpll;
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_SSR_INSTANCE_CAPACITY>
            isolatedOutputFrequencyHz{};
    };

    struct HostMot2StepGeneratorCommand {
        double stepsPerSecond = 0.0;
        bool enabled = false;
    };

    struct HostMot2CyclicOutputImage {
        std::array<
            HostMot2StepGeneratorCommand,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY> stepGenerators{};
        std::bitset<
            HOSTMOT2_CYCLIC_DIGITAL_OUTPUT_CAPACITY> digitalOutputs;
        bool stepGeneratorsEnabled = false;
        bool digitalOutputsEnabled = false;
        bool watchdogEnabled = false;
    };

    struct HostMot2DpllObservation {
        std::int32_t phaseErrorNanoseconds = 0;
        bool enabled = false;
        bool ready = false;
    };

    struct HostMot2CyclicInputImage {
        std::array<
            std::uint32_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY> rawStepAccumulator{};
        std::array<
            std::int64_t,
            HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>
            stepAccumulatorSubcounts{};
        std::bitset<
            HOSTMOT2_CYCLIC_DIGITAL_INPUT_CAPACITY> fieldDigitalInputs;
        HostMot2DpllObservation dpll;
    };

    enum class HostMot2CyclicIoFault : std::uint8_t {
        None,
        NotInitialized,
        InvalidOutput,
        Transport,
        ReadSequenceMismatch,
        WriteSequenceMismatch,
        BoardProtocolError,
        WatchdogTripped,
        DpllPhaseError,
    };

    struct HostMot2CyclicIoResult {
        HostMot2CyclicIoFault fault = HostMot2CyclicIoFault::None;
        Lbp16CyclicResult transaction;
        bool inputsValid = false;
    };

    class HostMot2CyclicIo {
    public:
        [[nodiscard]] static std::expected<
            std::unique_ptr<HostMot2CyclicIo>, std::string>
        create(
            Lbp16DatagramTransport &transport,
            const HostMot2CyclicLayout &layout,
            const HostMot2CyclicConfiguration &configuration);

        ~HostMot2CyclicIo();
        HostMot2CyclicIo(const HostMot2CyclicIo &) = delete;
        HostMot2CyclicIo &operator=(const HostMot2CyclicIo &) = delete;

        [[nodiscard]] HostMot2CyclicIoResult initializeSafe() noexcept;
        [[nodiscard]] HostMot2CyclicIoResult cycle(
            const HostMot2CyclicOutputImage &outputs) noexcept;

        [[nodiscard]] const HostMot2CyclicInputImage &
        inputImage() const noexcept;
        [[nodiscard]] HostMot2CyclicIoFault fault() const noexcept;
        [[nodiscard]] bool initialized() const noexcept;
        [[nodiscard]] std::size_t stepGeneratorCount() const noexcept;
        [[nodiscard]] std::size_t digitalInputCount() const noexcept;
        [[nodiscard]] std::size_t digitalOutputCount() const noexcept;
        [[nodiscard]] const HostMot2DpllConfiguration &
        dpllConfiguration() const noexcept;

    private:
        class Impl;

        explicit HostMot2CyclicIo(std::unique_ptr<Impl> impl) noexcept;

        std::unique_ptr<Impl> m_impl;
    };
}
