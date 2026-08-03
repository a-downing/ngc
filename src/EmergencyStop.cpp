#include "machine/EmergencyStop.h"

namespace ngc {
    void EmergencyStopInterface::request(const EmergencyStopSource source) noexcept {
        std::atomic_ref(m_control.requestedSources).fetch_or(
            emergencyStopSourceMask(source), std::memory_order_release);
    }

    void EmergencyStopInterface::release(const EmergencyStopSource source) noexcept {
        std::atomic_ref(m_control.requestedSources).fetch_and(
            ~emergencyStopSourceMask(source), std::memory_order_release);
    }

    std::uint64_t EmergencyStopInterface::requestReset() noexcept {
        return std::atomic_ref(m_control.resetGeneration).fetch_add(
            1, std::memory_order_acq_rel) + 1;
    }

    std::uint32_t EmergencyStopInterface::requestedSources() const noexcept {
        return std::atomic_ref(m_control.requestedSources).load(
            std::memory_order_acquire);
    }

    std::uint64_t EmergencyStopInterface::resetGeneration() const noexcept {
        return std::atomic_ref(m_control.resetGeneration).load(
            std::memory_order_acquire);
    }

    EmergencyStopStatus EmergencyStopInterface::status() const noexcept {
        for (;;) {
            const auto before = std::atomic_ref(m_control.statusSequence).load(
                std::memory_order_acquire);
            if ((before & 1U) != 0) {
                continue;
            }
            const EmergencyStopStatus result{
                .acknowledgedResetGeneration = std::atomic_ref(
                    m_control.acknowledgedResetGeneration).load(
                        std::memory_order_relaxed),
                .activeSources = std::atomic_ref(m_control.activeSources).load(
                    std::memory_order_relaxed),
                .latchedSources = std::atomic_ref(m_control.latchedSources).load(
                    std::memory_order_relaxed),
                .resetResult = static_cast<EmergencyStopResetResult>(
                    std::atomic_ref(m_control.resetResult).load(
                        std::memory_order_relaxed)),
            };
            const auto after = std::atomic_ref(m_control.statusSequence).load(
                std::memory_order_acquire);
            if (before == after) {
                return result;
            }
        }
    }

    void EmergencyStopInterface::publish(const EmergencyStopStatus &status) noexcept {
        auto sequence = std::atomic_ref(m_control.statusSequence);
        sequence.fetch_add(1, std::memory_order_acq_rel);
        std::atomic_ref(m_control.acknowledgedResetGeneration).store(
            status.acknowledgedResetGeneration, std::memory_order_relaxed);
        std::atomic_ref(m_control.activeSources).store(
            status.activeSources, std::memory_order_relaxed);
        std::atomic_ref(m_control.latchedSources).store(
            status.latchedSources, std::memory_order_relaxed);
        std::atomic_ref(m_control.resetResult).store(
            static_cast<std::uint32_t>(status.resetResult),
            std::memory_order_relaxed);
        sequence.fetch_add(1, std::memory_order_release);
    }

    bool EmergencyStopState::update(const std::uint32_t localActiveSources) noexcept {
        m_status.activeSources = m_interface.requestedSources() | localActiveSources;
        m_status.latchedSources |= m_status.activeSources;

        const auto resetGeneration = m_interface.resetGeneration();
        if (resetGeneration != m_observedResetGeneration) {
            m_observedResetGeneration = resetGeneration;
            m_status.acknowledgedResetGeneration = resetGeneration;
            if (m_status.activeSources != 0) {
                m_status.resetResult =
                    EmergencyStopResetResult::BlockedByActiveSource;
            } else if (m_status.latchedSources == 0) {
                m_status.resetResult = EmergencyStopResetResult::NotLatched;
            } else {
                m_status.latchedSources = 0;
                m_status.resetResult = EmergencyStopResetResult::Cleared;
            }
        }
        m_interface.publish(m_status);

        return latched();
    }

    bool EmergencyStopState::latched() const noexcept {
        return m_status.latchedSources != 0;
    }

    EmergencyStopStatus EmergencyStopState::status() const noexcept {
        return m_status;
    }
}
