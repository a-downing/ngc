#pragma once

#include <atomic>
#include <bitset>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

#include "config/BackendRuntimeConfiguration.h"
#include "machine/BackendRuntime.h"
#include "machine/MachineConfiguration.h"
#include "machine/ProductionExecutorCore.h"
#include "machine/RealtimeTiming.h"
#include "machine/SpscChannel.h"

namespace ngc {
    inline constexpr std::uint32_t
        PRODUCTION_EXECUTOR_DEADLINE_MISS_FAULT = 0x52540001;

    using ProductionExecutorDigitalInputs = LogicalDigitalInputImage;

    struct ProductionExecutorIoFaultDiagnostic {
        std::uint32_t code = 0;
        std::uint32_t joint = 0;
        double followingErrorSteps = 0.0;
        double targetPosition = 0.0;
        double actualPosition = 0.0;
        std::int32_t dpllPhaseErrorNanoseconds = 0;
    };
    static_assert(
        std::is_trivially_copyable_v<
            ProductionExecutorIoFaultDiagnostic>);

    class ProductionExecutorIo {
    public:
        virtual ~ProductionExecutorIo() = default;

        virtual void sampleDigitalInputs(
            const ProductionExecutorMotionContext &motion,
            ProductionExecutorDigitalInputs &inputs) noexcept = 0;
        virtual void applyOutputs(
            const ProductionExecutorOutputState &outputs) noexcept = 0;
        [[nodiscard]] virtual std::uint32_t faultCode() const noexcept {
            return 0;
        }
        [[nodiscard]] virtual ProductionExecutorIoFaultDiagnostic
        faultDiagnostic() const noexcept {
            return {};
        }
        [[nodiscard]] virtual bool prepareTriggeredJointMove(
            const TriggeredJointMove &) noexcept {
            return true;
        }
    };

    struct HostedExecutorRuntimeConfiguration {
        double servoPeriod = 0.001;
        std::uint32_t serviceTicksPerPeriod = 1;
        std::uint32_t timingPublicationTicks = 100;
        ProductionExecutorConfiguration executor;
        BackendRuntimeHostConfiguration realtime;
    };

    [[nodiscard]] HostedExecutorRuntimeConfiguration
    hostedExecutorRuntimeConfiguration(
        const MachineConfiguration &configuration);

    class HostedExecutorRuntime final : public BackendRuntime {
    public:
        explicit HostedExecutorRuntime(
            HostedExecutorRuntimeConfiguration configuration,
            std::unique_ptr<ProductionExecutorIo> io = {});
        explicit HostedExecutorRuntime(
            const MachineConfiguration &configuration,
            std::unique_ptr<ProductionExecutorIo> io = {});
        ~HostedExecutorRuntime() override;
        HostedExecutorRuntime(const HostedExecutorRuntime &) = delete;
        HostedExecutorRuntime &operator=(
            const HostedExecutorRuntime &) = delete;

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
        bool tryTakeRealtimeTiming(
            RealtimeTimingSummary &summary) noexcept override;

    private:
        struct TimingAccumulator;

        void configureServoThread();
        void runServoLoop();
        void tick(bool publishSnapshot) noexcept;
        void observeTiming(
            std::uint64_t tick, std::int64_t wakeLatenessNanoseconds,
            std::uint64_t executionNanoseconds,
            std::int64_t deadlineSlackNanoseconds,
            std::uint64_t skippedPeriods) noexcept;
        [[nodiscard]] bool running() const noexcept;
        [[nodiscard]] std::uint64_t waitForNextTick(
            std::uint64_t previousTicks);

        static constexpr std::size_t TIMING_CAPACITY = 64;

        std::unique_ptr<ProductionExecutorCore> m_core;
        std::unique_ptr<ProductionExecutorIo> m_io;
        std::unique_ptr<TimingAccumulator> m_timingAccumulator;
        ProductionExecutorDigitalInputs m_inputs;
        double m_servoPeriod;
        std::uint32_t m_serviceTicksPerPeriod;
        std::uint32_t m_timingPublicationTicks;
        BackendRuntimeHostConfiguration m_realtime;
        SpscChannel<RealtimeTimingSummary, TIMING_CAPACITY> m_timing;
        mutable std::mutex m_lifecycleMutex;
        std::condition_variable m_lifecycleCv;
        std::thread m_servoThread;
        bool m_started = false;
        bool m_startupComplete = false;
        bool m_ioFaultTimingPublished = false;
        std::string m_startupError;
        std::atomic<bool> m_stopping{false};
        std::atomic<std::uint64_t> m_servoTicks{0};
    };
}
