#pragma once

#include <bitset>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "machine/BackendRuntime.h"
#include "machine/MachineConfiguration.h"
#include "machine/ProductionExecutorCore.h"

namespace ngc {
    using ProductionExecutorDigitalInputs =
        std::bitset<ProductionExecutorCore::DIGITAL_INPUT_CAPACITY>;

    class ProductionExecutorIo {
    public:
        virtual ~ProductionExecutorIo() = default;

        virtual void sampleDigitalInputs(
            ProductionExecutorDigitalInputs &inputs) noexcept = 0;
        virtual void applyOutputs(
            const ProductionExecutorOutputState &outputs) noexcept = 0;
        [[nodiscard]] virtual bool prepareTriggeredJointMove(
            const TriggeredJointMove &) noexcept {
            return true;
        }
    };

    struct ProductionExecutorRuntimeConfiguration {
        double servoPeriod = 0.001;
        std::uint32_t serviceTicksPerPeriod = 1;
        ProductionExecutorConfiguration executor;
    };

    [[nodiscard]] ProductionExecutorRuntimeConfiguration
    productionExecutorRuntimeConfiguration(
        const MachineConfiguration &configuration);

    class ProductionExecutorRuntime final : public BackendRuntime {
    public:
        explicit ProductionExecutorRuntime(
            ProductionExecutorRuntimeConfiguration configuration,
            std::unique_ptr<ProductionExecutorIo> io = {});
        explicit ProductionExecutorRuntime(
            const MachineConfiguration &configuration,
            std::unique_ptr<ProductionExecutorIo> io = {});
        ~ProductionExecutorRuntime() override;
        ProductionExecutorRuntime(const ProductionExecutorRuntime &) = delete;
        ProductionExecutorRuntime &operator=(
            const ProductionExecutorRuntime &) = delete;

        MotionBackend &endpoint() noexcept override;
        void start() override;
        void stop() override;
        [[nodiscard]] BackendCapabilities capabilities() const noexcept override;
        [[nodiscard]] bool restoreStationaryState(
            const StationaryBackendState &state) noexcept override;
        [[nodiscard]] bool prepareTriggeredJointMove(
            const TriggeredJointMove &move) noexcept override;
        void serviceImmediate() override;
        [[nodiscard]] std::uint64_t advanceServiceMotionPeriod() override;
        void waitForServiceMotion() override;

        [[nodiscard]] bool started() const noexcept;
        [[nodiscard]] std::uint64_t servoTicks() const noexcept;

    private:
        void runServoLoop();
        void tick(bool publishSnapshot) noexcept;
        [[nodiscard]] bool running() const noexcept;
        [[nodiscard]] std::uint64_t waitForNextTick(
            std::uint64_t previousTicks);

        std::unique_ptr<ProductionExecutorCore> m_core;
        std::unique_ptr<ProductionExecutorIo> m_io;
        ProductionExecutorDigitalInputs m_inputs;
        double m_servoPeriod;
        std::uint32_t m_serviceTicksPerPeriod;
        mutable std::mutex m_lifecycleMutex;
        std::condition_variable m_lifecycleCv;
        std::thread m_servoThread;
        bool m_started = false;
        bool m_stopping = false;
        std::uint64_t m_servoTicks = 0;
    };
}
