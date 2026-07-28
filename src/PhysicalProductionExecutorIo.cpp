#include "machine/PhysicalProductionExecutorIo.h"

#include <stdexcept>
#include <utility>

namespace ngc {
    PhysicalProductionExecutorIo::PhysicalProductionExecutorIo(
        std::unique_ptr<ProductionExecutorIo> motion,
        std::unique_ptr<SpindleHardware> spindle)
        : m_motion(std::move(motion)) {
        if (!m_motion) {
            throw std::invalid_argument(
                "physical executor I/O requires motion hardware");
        }
        if (spindle) {
            m_spindle = std::make_unique<SpindleWorker>(
                std::move(spindle));
            m_spindle->start();
        }
    }

    PhysicalProductionExecutorIo::~PhysicalProductionExecutorIo() {
        if (m_spindle) {
            m_spindle->stop();
        }
    }

    void PhysicalProductionExecutorIo::sampleDigitalInputs(
        ProductionExecutorDigitalInputs &inputs) noexcept {
        m_motion->sampleDigitalInputs(inputs);
    }

    void PhysicalProductionExecutorIo::applyOutputs(
        const ProductionExecutorOutputState &outputs) noexcept {
        m_motion->applyOutputs(outputs);
        if (!m_spindle
            || (m_lastSpindle.has_value()
                && sameSpindle(
                    *m_lastSpindle, outputs.spindle))) {
            return;
        }
        if (m_spindle->tryCommand(outputs.spindle)) {
            m_lastSpindle = outputs.spindle;
        }
    }

    std::uint32_t
    PhysicalProductionExecutorIo::faultCode() const noexcept {
        const auto motionFault = m_motion->faultCode();
        if (motionFault != 0) {
            return motionFault;
        }

        return m_spindle ? m_spindle->faultCode() : 0;
    }

    bool PhysicalProductionExecutorIo::prepareTriggeredJointMove(
        const TriggeredJointMove &move) noexcept {
        return m_motion->prepareTriggeredJointMove(move);
    }

    bool PhysicalProductionExecutorIo::sameSpindle(
        const SpindleEvent &left,
        const SpindleEvent &right) noexcept {
        return left.enabled == right.enabled
            && left.direction == right.direction
            && left.speed == right.speed;
    }
}
