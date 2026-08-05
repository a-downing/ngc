#pragma once

#include "machine/MotionBackend.h"

namespace ngc {
    // The sole NRT owner of executor lifecycle intent. Callers name the desired
    // state; this controller supplies monotonic generations and prevents them
    // from constructing or replaying raw demand records.
    class ExecutorDemandController {
    public:
        explicit ExecutorDemandController(MotionBackend &backend) noexcept
            : m_backend(backend) { }

        [[nodiscard]] bool request(const EpochId epoch,
                                   const ExecutorDemandMode mode) noexcept {
            const ExecutorDemand demand{
                .generation = m_nextGeneration,
                .epoch = epoch,
                .mode = mode,
            };
            if (m_backend.publishDemand(demand)
                != DemandPublishResult::Published) {
                return false;
            }

            ++m_nextGeneration;
            m_lastDemand = demand;

            return true;
        }

        [[nodiscard]] DemandGeneration lastGeneration() const noexcept {
            return m_lastDemand.generation;
        }

        [[nodiscard]] ExecutorDemandMode mode() const noexcept {
            return m_lastDemand.mode;
        }

        [[nodiscard]] EpochId epoch() const noexcept {
            return m_lastDemand.epoch;
        }

    private:
        MotionBackend &m_backend;
        ExecutorDemand m_lastDemand;
        DemandGeneration m_nextGeneration = 1;
    };
}
