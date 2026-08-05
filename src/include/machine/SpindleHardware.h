#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "machine/MotionBackend.h"
#include "machine/SpscChannel.h"

namespace ngc {
    inline constexpr std::uint32_t
        SPINDLE_COMMAND_BACKPRESSURE_FAULT = 0x53500001;
    inline constexpr std::uint32_t
        SPINDLE_COMMUNICATION_FAULT = 0x53500002;

    struct SpindleHardwareStatus {
        bool communicationHealthy = false;
        bool atSpeed = false;
        double speed = 0.0;
        double current = 0.0;
        std::uint32_t faultCode = 0;
    };

    enum class SpindleSafetyState : std::uint8_t {
        Armed,
        StopRequested,
        SafeStopped,
    };

    class SpindleHardware {
    public:
        virtual ~SpindleHardware() = default;

        [[nodiscard]] virtual bool applyDesired(
            const SpindleEvent &desired,
            const std::atomic<SpindleSafetyState> &safety) noexcept = 0;
        [[nodiscard]] virtual bool pollStatus(
            SpindleHardwareStatus &status) noexcept = 0;
        virtual void safeStop() noexcept = 0;
    };

    class SpindleWorker {
    public:
        explicit SpindleWorker(
            std::unique_ptr<SpindleHardware> hardware,
            std::chrono::milliseconds pollingPeriod =
                std::chrono::milliseconds(10));
        ~SpindleWorker();
        SpindleWorker(const SpindleWorker &) = delete;
        SpindleWorker &operator=(const SpindleWorker &) = delete;

        void start();
        void stop() noexcept;
        void establishSafeStop() noexcept;
        [[nodiscard]] bool tryRearmAndCommand(
            const SpindleEvent &desired) noexcept;
        [[nodiscard]] std::uint32_t faultCode() const noexcept;
        [[nodiscard]] SpindleHardwareStatus status() const noexcept;

    private:
        void run() noexcept;
        void latchFault(std::uint32_t fault) noexcept;

        static constexpr std::size_t COMMAND_CAPACITY = 16;

        std::unique_ptr<SpindleHardware> m_hardware;
        std::chrono::milliseconds m_pollingPeriod;
        SpscChannel<SpindleEvent, COMMAND_CAPACITY> m_commands;
        std::atomic<bool> m_stopping{false};
        std::atomic<bool> m_started{false};
        std::atomic<SpindleSafetyState> m_safety{
            SpindleSafetyState::Armed};
        std::atomic<std::uint32_t> m_faultCode{0};
        std::atomic<bool> m_communicationHealthy{false};
        std::atomic<bool> m_atSpeed{false};
        std::atomic<double> m_speed{0.0};
        std::atomic<double> m_current{0.0};
        std::thread m_thread;
    };
}
