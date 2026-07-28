#include "mesa/MesaProductionExecutorIo.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <format>
#include <utility>

namespace ngc::mesa {
    namespace {
        std::uint32_t executorFaultCode(
            const HostMot2CyclicIoFault fault) noexcept {
            return fault == HostMot2CyclicIoFault::None
                ? 0
                : MESA_PRODUCTION_EXECUTOR_IO_FAULT_BASE
                    | static_cast<std::uint32_t>(fault);
        }
    }

    std::expected<
        std::unique_ptr<MesaProductionExecutorIo>, std::string>
    MesaProductionExecutorIo::create(
        std::unique_ptr<HostMot2CyclicIo> io,
        DigitalIoProgram ioProgram,
        const std::span<const MesaStepGeneratorMapping>
            stepGenerators) {
        if (!io) {
            return std::unexpected(
                "Mesa executor I/O requires HostMot2 cyclic I/O");
        }
        if (ioProgram.fieldInputCount()
            > io->digitalInputCount()) {
            return std::unexpected(
                "input program references unavailable Mesa field inputs");
        }
        if (ioProgram.fieldOutputCount()
            > io->digitalOutputCount()) {
            return std::unexpected(
                "digital I/O program references unavailable Mesa field outputs");
        }
        if (stepGenerators.size() > MAX_JOINTS) {
            return std::unexpected(
                "Mesa joint-to-StepGen mapping exceeds joint capacity");
        }

        auto mappedJoints = JointMask{0};
        auto mappedStepGenerators =
            std::bitset<HOSTMOT2_CYCLIC_STEP_GENERATOR_CAPACITY>{};
        for (const auto &mapping : stepGenerators) {
            if (mapping.joint >= MAX_JOINTS) {
                return std::unexpected(
                    "Mesa StepGen mapping references an unknown joint");
            }
            if (mapping.stepGenerator
                >= io->stepGeneratorCount()) {
                return std::unexpected(
                    "Mesa StepGen mapping references an unavailable generator");
            }
            if (!std::isfinite(mapping.stepsPerMachineUnit)
                || mapping.stepsPerMachineUnit == 0.0) {
                return std::unexpected(
                    "Mesa StepGen mapping requires finite nonzero steps per machine unit");
            }

            const auto joint =
                static_cast<JointMask>(JointMask{1} << mapping.joint);
            if ((mappedJoints & joint) != 0) {
                return std::unexpected(std::format(
                    "Mesa StepGen mapping duplicates joint {}",
                    mapping.joint));
            }
            if (mappedStepGenerators[mapping.stepGenerator]) {
                return std::unexpected(std::format(
                    "Mesa StepGen mapping duplicates generator {}",
                    mapping.stepGenerator));
            }
            mappedJoints |= joint;
            mappedStepGenerators[mapping.stepGenerator] = true;
        }

        const auto initialized = io->initializeSafe();
        if (initialized.fault
            != HostMot2CyclicIoFault::None) {
            return std::unexpected(std::format(
                "Mesa safe initialization failed with fault {}",
                static_cast<std::uint32_t>(initialized.fault)));
        }

        return std::unique_ptr<MesaProductionExecutorIo>(
            new MesaProductionExecutorIo(
                std::move(io), std::move(ioProgram),
                stepGenerators));
    }

    MesaProductionExecutorIo::MesaProductionExecutorIo(
        std::unique_ptr<HostMot2CyclicIo> io,
        DigitalIoProgram ioProgram,
        const std::span<const MesaStepGeneratorMapping>
            stepGenerators) noexcept
        : m_io(std::move(io)),
          m_ioProgram(std::move(ioProgram)),
          m_stepGeneratorCount(stepGenerators.size()) {
        std::ranges::copy(
            stepGenerators, m_stepGenerators.begin());
    }

    void MesaProductionExecutorIo::sampleDigitalInputs(
        ProductionExecutorDigitalInputs &inputs) noexcept {
        inputs.reset();
        if (m_faultCode != 0) {
            return;
        }

        const auto result = m_io->cycle(m_pendingOutputs);
        if (result.fault != HostMot2CyclicIoFault::None
            || !result.inputsValid) {
            m_faultCode = executorFaultCode(result.fault);
            if (m_faultCode == 0) {
                m_faultCode = executorFaultCode(
                    HostMot2CyclicIoFault::Transport);
            }

            return;
        }

        const auto &mesaInputs =
            m_io->inputImage().fieldDigitalInputs;
        for (std::size_t index = 0;
             index < m_ioProgram.fieldInputCount(); ++index) {
            m_fieldInputs[index] = mesaInputs[index];
        }
        m_ioProgram.executeInputs(
            m_fieldInputs, m_logicalOutputs, inputs);
    }

    void MesaProductionExecutorIo::applyOutputs(
        const ProductionExecutorOutputState &outputs) noexcept {
        auto next = HostMot2CyclicOutputImage{};
        if (m_faultCode != 0 || !outputs.executorEnabled) {
            m_logicalOutputs.reset();
            m_pendingOutputs = next;

            return;
        }

        m_logicalOutputs = outputs.digitalOutputs;
        next.watchdogEnabled = true;
        next.stepGeneratorsEnabled =
            m_stepGeneratorCount != 0;
        for (std::size_t index = 0;
             index < m_stepGeneratorCount; ++index) {
            const auto &mapping = m_stepGenerators[index];
            next.stepGenerators[mapping.stepGenerator] = {
                .stepsPerSecond =
                    outputs.commandedJoints.velocity[mapping.joint]
                    * mapping.stepsPerMachineUnit,
                .enabled = true,
            };
        }
        auto fieldOutputs = FieldDigitalOutputImage{};
        m_ioProgram.executeOutputs(
            m_fieldInputs, m_logicalOutputs, fieldOutputs);
        next.digitalOutputsEnabled =
            m_ioProgram.fieldOutputCount() != 0;
        for (std::size_t index = 0;
             index < m_ioProgram.fieldOutputCount(); ++index) {
            next.digitalOutputs[index] = fieldOutputs[index];
        }

        m_pendingOutputs = next;
    }

    std::uint32_t MesaProductionExecutorIo::faultCode() const noexcept {
        return m_faultCode;
    }

    const HostMot2CyclicOutputImage &
    MesaProductionExecutorIo::pendingOutputs() const noexcept {
        return m_pendingOutputs;
    }
}
