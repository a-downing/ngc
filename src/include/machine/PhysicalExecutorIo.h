#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "machine/HostedExecutorRuntime.h"
#include "machine/SpindleHardware.h"

namespace ngc {
    class PhysicalExecutorIo final
        : public ProductionExecutorIo {
    public:
        PhysicalExecutorIo(
            std::unique_ptr<ProductionExecutorIo> motion,
            std::unique_ptr<SpindleHardware> spindle = {});
        ~PhysicalExecutorIo() override;

        void sampleDigitalInputs(
            const ProductionExecutorMotionContext &motion,
            ProductionExecutorDigitalInputs &inputs) noexcept override;
        void applyOutputs(
            const ProductionExecutorOutputState &outputs) noexcept override;
        [[nodiscard]] std::uint32_t faultCode() const noexcept override;
        [[nodiscard]] ProductionExecutorIoFaultDiagnostic
        faultDiagnostic() const noexcept override;
        [[nodiscard]] bool prepareTriggeredJointMove(
            const TriggeredJointMove &move) noexcept override;

    private:
        static bool sameSpindle(
            const SpindleEvent &left,
            const SpindleEvent &right) noexcept;

        std::unique_ptr<ProductionExecutorIo> m_motion;
        std::unique_ptr<SpindleWorker> m_spindle;
        std::optional<SpindleEvent> m_lastSpindle;
    };
}
