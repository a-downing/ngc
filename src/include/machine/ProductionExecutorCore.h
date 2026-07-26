#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "machine/MotionBackend.h"
#include "machine/SpscChannel.h"

namespace ngc {
    class ProductionExecutorCore final : public MotionBackend {
    public:
        static constexpr std::size_t PLAN_CAPACITY = 8;
        static constexpr std::size_t CONTROL_CAPACITY = 16;
        static constexpr std::size_t EVENT_CAPACITY =
            PLAN_CAPACITY * (MAX_EXECUTION_MARKERS_PER_CHUNK + 4)
            + CONTROL_CAPACITY;
        static constexpr std::size_t SNAPSHOT_CAPACITY = 4;

        explicit ProductionExecutorCore(double servoPeriod);

        ProductionExecutorCore(const ProductionExecutorCore &) = delete;
        ProductionExecutorCore &operator=(const ProductionExecutorCore &) = delete;

        PublishResult tryPublish(const ExecutionItem &item) noexcept override;
        SubmitResult trySubmit(const ControlRequest &request) noexcept override;
        bool tryTakeEvent(ExecutionEvent &event) noexcept override;
        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override;

        void restoreStationaryState(const MotionState &commanded,
                                    const MotionState &feedback = {}) noexcept;
        void servoTick(bool publishSnapshot = true) noexcept;
        [[nodiscard]] double servoPeriod() const noexcept;

    private:
        struct PlanSlot {
            std::atomic<bool> occupied{false};
            PlanChunk chunk{};
            std::uint64_t normalMotionNanoseconds = 0;
        };

        static bool validPlanChunk(const PlanChunk &chunk) noexcept;
        static std::uint64_t normalMotionNanoseconds(const PlanChunk &chunk) noexcept;
        static std::uint64_t secondsToNanoseconds(double seconds) noexcept;

        void serviceControls() noexcept;
        void activateNext() noexcept;
        void advanceActive(double seconds) noexcept;
        void completeSpan() noexcept;
        void selectContinuationOrStop() noexcept;
        void emitExecutionMarkersThrough(double parameter) noexcept;
        void emit(const ExecutionEvent &event) noexcept;
        void fault(std::uint32_t code) noexcept;
        void discardExecution() noexcept;
        void release(std::uint8_t index) noexcept;
        void accountForDequeued(std::uint8_t index) noexcept;
        void publishSnapshot() noexcept;
        void refreshSnapshot() noexcept;

        [[nodiscard]] PlanChunk &activeChunk() noexcept;
        [[nodiscard]] const PlanChunk &activeChunk() const noexcept;
        [[nodiscard]] const AxisPolynomialSpan &currentSpan() const noexcept;

        template<typename Spans>
        static double remainingDuration(const Spans &spans, std::uint32_t current,
                                        double elapsed) noexcept {
            if (current >= spans.size) {
                return 0.0;
            }

            auto result = std::max(spans[current].duration - elapsed, 0.0);
            for (auto index = current + 1; index < spans.size; ++index) {
                result += std::max(spans[index].duration, 0.0);
            }

            return result;
        }

        std::array<PlanSlot, PLAN_CAPACITY> m_planSlots;
        SpscChannel<std::uint8_t, PLAN_CAPACITY> m_plans;
        SpscChannel<ControlRequest, CONTROL_CAPACITY> m_controls;
        SpscChannel<ExecutionEvent, EVENT_CAPACITY> m_events;
        SpscChannel<ExecutionSnapshot, SNAPSHOT_CAPACITY> m_snapshots;
        std::atomic<std::uint64_t> m_queuedNormalMotionNanoseconds{0};
        std::atomic<std::uint32_t> m_queuedExecutionItems{0};
        ExecutionSnapshot m_snapshot;
        std::optional<std::uint8_t> m_active;
        double m_servoPeriod;
        double m_spanElapsed = 0.0;
        std::uint32_t m_span = 0;
        std::uint32_t m_nextMarker = 0;
        bool m_stopping = false;
        bool m_faultEventEmitted = false;
    };
}
