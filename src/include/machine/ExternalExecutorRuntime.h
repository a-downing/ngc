#pragma once

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "machine/BackendRuntime.h"
#include "machine/IpcProtocol.h"

namespace ngc {
    struct ExternalExecutorRuntimeConfiguration {
        std::filesystem::path peerExecutable;
        IpcIdentity identity{};
        IpcIdentity peerExpectedIdentity{};
        std::chrono::milliseconds handshakeTimeout{2000};
        std::chrono::milliseconds shutdownTimeout{2000};
        std::vector<std::string> peerArguments;
    };

    class ExternalExecutorRuntime final : public BackendRuntime {
    public:
        explicit ExternalExecutorRuntime(ExternalExecutorRuntimeConfiguration configuration);
        ~ExternalExecutorRuntime() override;
        ExternalExecutorRuntime(const ExternalExecutorRuntime &) = delete;
        ExternalExecutorRuntime &operator=(const ExternalExecutorRuntime &) = delete;

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

        [[nodiscard]] bool connected() const noexcept;
        [[nodiscard]] IpcRejection lastRejection() const noexcept;
        bool tryTakeRealtimeTiming(
            RealtimeTimingSummary &summary) noexcept override;
        void requestEmergencyStop(EmergencyStopSource source) noexcept override;
        void releaseEmergencyStop(EmergencyStopSource source) noexcept override;
        [[nodiscard]] std::uint64_t requestEmergencyStopReset() noexcept override;
        [[nodiscard]] EmergencyStopStatus emergencyStopStatus() const noexcept override;

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
