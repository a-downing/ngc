#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <span>
#include <string>

#include "machine/DigitalIoProgram.h"
#include "machine/MachineConfiguration.h"
#include "machine/HostedExecutorRuntime.h"
#include "mesa/HostMot2CyclicIo.h"
#include "mesa/MesaBackendConfiguration.h"

namespace ngc::mesa {
    inline constexpr std::uint32_t
        MESA_PRODUCTION_EXECUTOR_IO_FAULT_BASE = 0x4D530000;

    struct MesaStepGeneratorMapping {
        JointId joint = 0;
        std::size_t stepGenerator = 0;
        double stepsPerMachineUnit = 0.0;
        double positionGainPerSecond = 0.0;
        double maximumCorrectionVelocity = 0.0;
        double maximumGeneratedStepError = 0.0;
    };

    struct MesaExecutorSafetyInput {
        DigitalInputId input = 0;
        bool requiredLevel = true;
    };

    inline constexpr std::uint32_t
        MESA_STEPGEN_ALIGNMENT_FAULT = 0x4D53'0100;
    inline constexpr std::uint32_t
        MESA_STEPGEN_FOLLOWING_ERROR_FAULT = 0x4D53'0101;
    inline constexpr std::uint32_t
        MESA_EXTERNAL_ENABLE_FAULT = 0x4D53'0102;

    [[nodiscard]] std::expected<DigitalIoProgram, std::string>
    compileMesaDigitalIoProgram(
        const MachineConfiguration &machine,
        const MesaBackendConfiguration &mesa);

    class MesaProductionExecutorIo final : public ProductionExecutorIo {
    public:
        [[nodiscard]] static std::expected<
            std::unique_ptr<MesaProductionExecutorIo>, std::string>
        create(
            std::unique_ptr<HostMot2CyclicIo> io,
            DigitalIoProgram ioProgram,
            std::span<const MesaStepGeneratorMapping>
                stepGenerators = {},
            std::optional<MesaExecutorSafetyInput>
                safetyInput = std::nullopt);

        void sampleDigitalInputs(
            const ProductionExecutorMotionContext &motion,
            ProductionExecutorDigitalInputs &inputs) noexcept override;
        void applyOutputs(
            const ProductionExecutorOutputState &outputs) noexcept override;
        [[nodiscard]] std::uint32_t faultCode() const noexcept override;
        [[nodiscard]] ProductionExecutorIoFaultDiagnostic
        faultDiagnostic() const noexcept override;
        [[nodiscard]] std::uint32_t emergencyStopSources() const noexcept override;
        [[nodiscard]] std::uint32_t emergencyStopFaultCode() const noexcept override;

        [[nodiscard]] const HostMot2CyclicOutputImage &
        pendingOutputs() const noexcept;

    private:
        MesaProductionExecutorIo(
            std::unique_ptr<HostMot2CyclicIo> io,
            DigitalIoProgram ioProgram,
            std::span<const MesaStepGeneratorMapping>
                stepGenerators,
            std::optional<MesaExecutorSafetyInput>
                safetyInput) noexcept;

        std::unique_ptr<HostMot2CyclicIo> m_io;
        DigitalIoProgram m_ioProgram;
        std::array<
            MesaStepGeneratorMapping,
            MAX_JOINTS> m_stepGenerators{};
        std::size_t m_stepGeneratorCount = 0;
        std::optional<MesaExecutorSafetyInput> m_safetyInput;
        HostMot2CyclicOutputImage m_pendingOutputs;
        JointMotionState m_lastCommandedJoints;
        JointMotionState m_pendingCommandedJoints;
        JointMotionState m_activeCommandedJoints;
        JointMotionState m_sampledFeedbackTargetJoints;
        std::array<std::int64_t, MAX_JOINTS>
            m_sampledAccumulatorSubcounts{};
        std::array<double, MAX_JOINTS>
            m_accumulatorOriginSubcounts{};
        FieldDigitalInputImage m_fieldInputs;
        LogicalDigitalOutputImage m_logicalOutputs;
        std::uint32_t m_faultCode = 0;
        ProductionExecutorIoFaultDiagnostic m_faultDiagnostic;
        bool m_lastExecutorEnabled = false;
        bool m_accumulatorFeedbackAvailable = false;
        bool m_accumulatorFeedbackAligned = false;
        bool m_externalEnableActive = false;
    };
}
