#include "mesa/MesaProductionExecutorIo.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <format>
#include <utility>

namespace ngc::mesa {
    namespace {
        constexpr double STEPGEN_SUBCOUNTS_PER_STEP = 65'536.0;
        constexpr double STATIONARY_TOLERANCE = 1e-12;

        std::uint32_t executorFaultCode(
            const HostMot2CyclicIoFault fault) noexcept {
            return fault == HostMot2CyclicIoFault::None
                ? 0
                : MESA_PRODUCTION_EXECUTOR_IO_FAULT_BASE
                    | static_cast<std::uint32_t>(fault);
        }

        double jointPositionAtOffset(
            const JointMotionState &state,
            const JointId joint,
            const double offsetSeconds) noexcept {
            return state.position[joint]
                + state.velocity[joint] * offsetSeconds
                + 0.5 * state.acceleration[joint]
                    * offsetSeconds * offsetSeconds;
        }

        bool jointIsStationary(
            const JointMotionState &state,
            const JointId joint) noexcept {
            return std::abs(state.velocity[joint])
                    <= STATIONARY_TOLERANCE
                && std::abs(state.acceleration[joint])
                    <= STATIONARY_TOLERANCE;
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
        if (!stepGenerators.empty()
            && !io->dpllConfiguration().enabled) {
            return std::unexpected(
                "Mesa StepGen position feedback requires the HostMot2 DPLL");
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
                || mapping.stepsPerMachineUnit <= 0.0) {
                return std::unexpected(
                    "Mesa StepGen mapping requires finite positive steps per machine unit");
            }
            if (!std::isfinite(mapping.positionGainPerSecond)
                || mapping.positionGainPerSecond <= 0.0
                || !std::isfinite(mapping.maximumCorrectionVelocity)
                || mapping.maximumCorrectionVelocity <= 0.0
                || !std::isfinite(mapping.maximumGeneratedStepError)
                || mapping.maximumGeneratedStepError <= 0.0) {
                return std::unexpected(
                    "Mesa StepGen mapping requires positive bounded "
                    "position-feedback parameters");
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
            m_io->inputImage();
        if (m_stepGeneratorCount != 0) {
            m_accumulatorFeedbackAvailable =
                mesaInputs.dpll.enabled && mesaInputs.dpll.ready;
            if (m_accumulatorFeedbackAvailable) {
                for (std::size_t index = 0;
                     index < m_stepGeneratorCount; ++index) {
                    const auto &mapping = m_stepGenerators[index];
                    m_sampledAccumulatorSubcounts[index] =
                        mesaInputs.stepAccumulatorSubcounts[
                            mapping.stepGenerator];
                }
                if (!m_accumulatorFeedbackAligned
                    && m_lastExecutorEnabled) {
                    for (std::size_t index = 0;
                         index < m_stepGeneratorCount; ++index) {
                        const auto &mapping = m_stepGenerators[index];
                        if (!jointIsStationary(
                                m_lastCommandedJoints,
                                mapping.joint)) {
                            m_faultCode =
                                MESA_STEPGEN_ALIGNMENT_FAULT;

                            return;
                        }
                    }

                    const auto &dpll =
                        m_io->dpllConfiguration();
                    const auto latchOffset =
                        (static_cast<double>(
                            dpll.servoPeriodNanoseconds)
                            + dpll
                                .stepGeneratorSampleOffsetNanoseconds)
                        / 1'000'000'000.0;
                    for (std::size_t index = 0;
                         index < m_stepGeneratorCount; ++index) {
                        const auto &mapping = m_stepGenerators[index];
                        const auto subcountsPerMachineUnit =
                            mapping.stepsPerMachineUnit
                            * STEPGEN_SUBCOUNTS_PER_STEP;
                        const auto expectedPosition =
                            jointPositionAtOffset(
                                m_lastCommandedJoints,
                                mapping.joint, latchOffset);
                        m_accumulatorOriginSubcounts[index] =
                            static_cast<double>(
                                m_sampledAccumulatorSubcounts[index])
                            - expectedPosition
                                * subcountsPerMachineUnit;
                    }
                    m_accumulatorFeedbackAligned = true;
                }
            }
        }
        const auto &fieldDigitalInputs =
            mesaInputs.fieldDigitalInputs;
        for (std::size_t index = 0;
             index < m_ioProgram.fieldInputCount(); ++index) {
            m_fieldInputs[index] = fieldDigitalInputs[index];
        }
        m_ioProgram.executeInputs(
            m_fieldInputs, m_logicalOutputs, inputs);
    }

    void MesaProductionExecutorIo::applyOutputs(
        const ProductionExecutorOutputState &outputs) noexcept {
        auto next = HostMot2CyclicOutputImage{};
        if (m_faultCode != 0 || !outputs.executorEnabled) {
            m_logicalOutputs.reset();
            m_lastCommandedJoints = outputs.commandedJoints;
            m_lastExecutorEnabled = false;
            m_accumulatorFeedbackAligned = false;
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
            auto commandVelocity = 0.0;
            if (m_accumulatorFeedbackAligned
                && m_accumulatorFeedbackAvailable) {
                const auto &dpll = m_io->dpllConfiguration();
                const auto latchOffset =
                    static_cast<double>(
                        dpll.stepGeneratorSampleOffsetNanoseconds)
                    / 1'000'000'000.0;
                const auto desiredPosition = jointPositionAtOffset(
                    outputs.commandedJoints,
                    mapping.joint, latchOffset);
                const auto subcountsPerMachineUnit =
                    mapping.stepsPerMachineUnit
                    * STEPGEN_SUBCOUNTS_PER_STEP;
                const auto stationaryCoordinateChange =
                    jointIsStationary(
                        outputs.commandedJoints, mapping.joint)
                    && jointIsStationary(
                        m_lastCommandedJoints, mapping.joint)
                    && std::abs(
                        outputs.commandedJoints.position[
                            mapping.joint]
                        - m_lastCommandedJoints.position[
                            mapping.joint])
                        > STATIONARY_TOLERANCE;
                if (stationaryCoordinateChange) {
                    m_accumulatorOriginSubcounts[index] =
                        static_cast<double>(
                            m_sampledAccumulatorSubcounts[index])
                        - desiredPosition
                            * subcountsPerMachineUnit;
                }
                const auto actualPosition =
                    (static_cast<double>(
                        m_sampledAccumulatorSubcounts[index])
                        - m_accumulatorOriginSubcounts[index])
                    / subcountsPerMachineUnit;
                const auto followingError =
                    desiredPosition - actualPosition;
                if (!std::isfinite(followingError)
                    || std::abs(
                        followingError
                            * mapping.stepsPerMachineUnit)
                        > mapping.maximumGeneratedStepError) {
                    m_faultCode =
                        MESA_STEPGEN_FOLLOWING_ERROR_FAULT;
                    m_pendingOutputs = {};

                    return;
                }
                const auto correction = std::clamp(
                    followingError
                        * mapping.positionGainPerSecond,
                    -mapping.maximumCorrectionVelocity,
                    mapping.maximumCorrectionVelocity);
                commandVelocity =
                    outputs.commandedJoints.velocity[
                        mapping.joint] + correction;
            } else if (m_accumulatorFeedbackAvailable
                && !jointIsStationary(
                    outputs.commandedJoints,
                    mapping.joint)) {
                m_faultCode = MESA_STEPGEN_ALIGNMENT_FAULT;
                m_pendingOutputs = {};

                return;
            }
            next.stepGenerators[mapping.stepGenerator] = {
                .stepsPerSecond =
                    commandVelocity * mapping.stepsPerMachineUnit,
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

        m_lastCommandedJoints = outputs.commandedJoints;
        m_lastExecutorEnabled = true;
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
