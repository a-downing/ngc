#pragma once

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "machine/BackendRuntime.h"
#include "machine/IpcProtocol.h"

namespace ngc {
    struct ExternalRealtimeRuntimeConfiguration {
        std::filesystem::path peerExecutable;
        IpcIdentity identity{};
        IpcIdentity peerExpectedIdentity{};
        std::chrono::milliseconds handshakeTimeout{2000};
        std::chrono::milliseconds shutdownTimeout{2000};
        std::vector<std::string> peerArguments;
    };

    class ExternalRealtimeRuntime final : public BackendRuntime {
    public:
        explicit ExternalRealtimeRuntime(ExternalRealtimeRuntimeConfiguration configuration);
        ~ExternalRealtimeRuntime() override;
        ExternalRealtimeRuntime(const ExternalRealtimeRuntime &) = delete;
        ExternalRealtimeRuntime &operator=(const ExternalRealtimeRuntime &) = delete;

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

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
