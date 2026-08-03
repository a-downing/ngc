#include "machine/PhysicalExecutorIo.h"

#include <stdexcept>
#include <utility>

namespace ngc {
    PhysicalExecutorIo::PhysicalExecutorIo(
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

    PhysicalExecutorIo::~PhysicalExecutorIo() {
        if (m_spindle) {
            m_spindle->stop();
        }
    }

    void PhysicalExecutorIo::sampleDigitalInputs(
        const ProductionExecutorMotionContext &motion,
        ProductionExecutorDigitalInputs &inputs) noexcept {
        m_motion->sampleDigitalInputs(motion, inputs);
    }

    void PhysicalExecutorIo::applyOutputs(
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
    PhysicalExecutorIo::faultCode() const noexcept {
        const auto motionFault = m_motion->faultCode();
        if (motionFault != 0) {
            return motionFault;
        }

        return m_spindle ? m_spindle->faultCode() : 0;
    }

    ProductionExecutorIoFaultDiagnostic
    PhysicalExecutorIo::faultDiagnostic() const noexcept {
        return m_motion->faultDiagnostic();
    }

    bool PhysicalExecutorIo::prepareTriggeredJointMove(
        const TriggeredJointMove &move) noexcept {
        return m_motion->prepareTriggeredJointMove(move);
    }

    bool PhysicalExecutorIo::sameSpindle(
        const SpindleEvent &left,
        const SpindleEvent &right) noexcept {
        return left.enabled == right.enabled
            && left.direction == right.direction
            && left.speed == right.speed;
    }
}
