#include "machine/SpindleHardware.h"

#include <stdexcept>
#include <utility>

namespace ngc {
    SpindleWorker::SpindleWorker(
        std::unique_ptr<SpindleHardware> hardware,
        const std::chrono::milliseconds pollingPeriod)
        : m_hardware(std::move(hardware)),
          m_pollingPeriod(pollingPeriod) {
        if (!m_hardware) {
            throw std::invalid_argument(
                "spindle worker requires hardware");
        }
        if (m_pollingPeriod <= std::chrono::milliseconds::zero()) {
            throw std::invalid_argument(
                "spindle worker polling period must be positive");
        }
    }

    SpindleWorker::~SpindleWorker() {
        stop();
    }

    void SpindleWorker::start() {
        auto expected = false;
        if (!m_started.compare_exchange_strong(
                expected, true,
                std::memory_order_acq_rel)) {
            return;
        }
        m_stopping.store(false, std::memory_order_release);
        m_thread = std::thread([this] {
            run();
        });
    }

    void SpindleWorker::stop() noexcept {
        if (!m_started.exchange(
                false, std::memory_order_acq_rel)) {
            return;
        }
        establishSafeStop();
        m_stopping.store(true, std::memory_order_release);
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }

    void SpindleWorker::establishSafeStop() noexcept {
        auto expected = SpindleSafetyState::Armed;
        static_cast<void>(m_safety.compare_exchange_strong(
            expected, SpindleSafetyState::StopRequested,
            std::memory_order_acq_rel));
    }

    bool SpindleWorker::tryRearmAndCommand(
        const SpindleEvent &desired) noexcept {
        if (m_faultCode.load(std::memory_order_acquire) != 0) {
            return false;
        }
        auto safety = SpindleSafetyState::SafeStopped;
        if (!m_safety.compare_exchange_strong(
                safety, SpindleSafetyState::Armed,
                std::memory_order_acq_rel)
            && safety != SpindleSafetyState::Armed) {
            return false;
        }
        if (!m_commands.tryPush(desired)) {
            latchFault(SPINDLE_COMMAND_BACKPRESSURE_FAULT);
            establishSafeStop();

            return false;
        }

        return true;
    }

    std::uint32_t SpindleWorker::faultCode() const noexcept {
        return m_faultCode.load(std::memory_order_acquire);
    }

    SpindleHardwareStatus SpindleWorker::status() const noexcept {
        return {
            .communicationHealthy =
                m_communicationHealthy.load(
                    std::memory_order_acquire),
            .atSpeed =
                m_atSpeed.load(std::memory_order_acquire),
            .speed =
                m_speed.load(std::memory_order_acquire),
            .current =
                m_current.load(std::memory_order_acquire),
            .faultCode = faultCode(),
        };
    }

    void SpindleWorker::run() noexcept {
        while (!m_stopping.load(std::memory_order_acquire)) {
            auto desired = SpindleEvent{};
            if (m_safety.load(std::memory_order_acquire)
                == SpindleSafetyState::StopRequested) {
                while (m_commands.tryPop(desired)) { }
                m_hardware->safeStop();
                auto expected = SpindleSafetyState::StopRequested;
                static_cast<void>(m_safety.compare_exchange_strong(
                    expected, SpindleSafetyState::SafeStopped,
                    std::memory_order_acq_rel));
            }
            if (faultCode() != 0) {
                break;
            }
            if (m_safety.load(std::memory_order_acquire)
                != SpindleSafetyState::Armed) {
                std::this_thread::sleep_for(m_pollingPeriod);

                continue;
            }
            while (m_commands.tryPop(desired)) {
                if (m_safety.load(std::memory_order_acquire)
                    != SpindleSafetyState::Armed) {
                    break;
                }
                if (!m_hardware->applyDesired(
                        desired, m_safety)) {
                    latchFault(SPINDLE_COMMUNICATION_FAULT);
                    establishSafeStop();
                    break;
                }
            }

            auto status = SpindleHardwareStatus{};
            if (!m_hardware->pollStatus(status)) {
                latchFault(SPINDLE_COMMUNICATION_FAULT);
            } else {
                m_communicationHealthy.store(
                    status.communicationHealthy,
                    std::memory_order_release);
                m_atSpeed.store(
                    status.atSpeed,
                    std::memory_order_release);
                m_speed.store(
                    status.speed,
                    std::memory_order_release);
                m_current.store(
                    status.current,
                    std::memory_order_release);
                if (status.faultCode != 0) {
                    latchFault(status.faultCode);
                }
            }

            if (faultCode() != 0) {
                establishSafeStop();
                continue;
            }
            if (m_safety.load(std::memory_order_acquire)
                != SpindleSafetyState::Armed) {
                continue;
            }
            std::this_thread::sleep_for(m_pollingPeriod);
        }

        m_hardware->safeStop();
        m_communicationHealthy.store(
            false, std::memory_order_release);
        m_atSpeed.store(false, std::memory_order_release);
    }

    void SpindleWorker::latchFault(
        const std::uint32_t fault) noexcept {
        auto expected = std::uint32_t{0};
        static_cast<void>(m_faultCode.compare_exchange_strong(
            expected, fault, std::memory_order_acq_rel));
    }
}
