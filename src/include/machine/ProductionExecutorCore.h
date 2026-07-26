#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include <ruckig/ruckig.hpp>

#include "machine/MotionBackend.h"
#include "machine/SpscChannel.h"

namespace ngc {
    class ProductionExecutorCore final : public MotionBackend {
    public:
        static constexpr std::size_t PLAN_CAPACITY = 8;
        static constexpr std::size_t CONTROL_CAPACITY = 16;
        static constexpr std::size_t EVENT_CAPACITY =
            PLAN_CAPACITY * (MAX_EXECUTION_MARKERS_PER_CHUNK + 5)
            + CONTROL_CAPACITY;
        static constexpr std::size_t SNAPSHOT_CAPACITY = 4;
        static constexpr std::size_t DIGITAL_INPUT_CAPACITY =
            std::numeric_limits<DigitalInputId>::max() + std::size_t{1};

        explicit ProductionExecutorCore(double servoPeriod);

        ProductionExecutorCore(const ProductionExecutorCore &) = delete;
        ProductionExecutorCore &operator=(const ProductionExecutorCore &) = delete;

        PublishResult tryPublish(const ExecutionItem &item) noexcept override;
        SubmitResult trySubmit(const ControlRequest &request) noexcept override;
        bool tryTakeEvent(ExecutionEvent &event) noexcept override;
        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override;

        void restoreStationaryState(const MotionState &commanded,
                                    const MotionState &feedback = {}) noexcept;
        // The hosting servo thread updates sampled input levels before
        // servoTick(). Edge conditions compare the current sample with the
        // preceding tick; hardware acquisition remains outside this core.
        void setDigitalInputSample(DigitalInputId input, bool active) noexcept;
        void servoTick(bool publishSnapshot = true) noexcept;
        [[nodiscard]] double servoPeriod() const noexcept;

    private:
        struct PlanSlot {
            std::atomic<bool> occupied{false};
            ExecutionItem item{};
            std::uint64_t normalMotionNanoseconds = 0;
        };

        struct TriggeredRuntime {
            position_t start{};
            position_t direction{};
            double length = 0.0;
            double elapsed = 0.0;
            bool stopping = false;
            TriggeredMoveStatus completionStatus =
                TriggeredMoveStatus::ReachedTarget;
            MotionState stopOrigin{};
            MotionState triggerState{};
            ruckig::Trajectory<1> trajectory;
        };

        struct TriggeredJointRuntime {
            double target = 0.0;
            double elapsed = 0.0;
            double debounceElapsed = 0.0;
            double triggerPosition = 0.0;
            double triggerVelocity = 0.0;
            double triggerAcceleration = 0.0;
            bool stopping = false;
            bool finished = false;
            bool triggerPending = false;
            ruckig::Trajectory<1> trajectory;
        };

        static bool validExecutionItem(const ExecutionItem &item) noexcept;
        static bool validPlanChunk(const PlanChunk &chunk) noexcept;
        static bool validTriggeredMove(const TriggeredMove &move) noexcept;
        static bool validTriggeredJointMove(const TriggeredJointMove &move) noexcept;
        static std::uint64_t normalMotionNanoseconds(const ExecutionItem &item) noexcept;
        static std::uint64_t secondsToNanoseconds(double seconds) noexcept;
        static EpochId itemEpoch(const ExecutionItem &item) noexcept;
        static ChunkId itemId(const ExecutionItem &item) noexcept;
        static BranchSequence itemPredecessor(const ExecutionItem &item) noexcept;

        void serviceControls() noexcept;
        void activateNext() noexcept;
        void advanceActive(double seconds) noexcept;
        void advancePlan(double &seconds) noexcept;
        bool initializeTriggered() noexcept;
        bool beginTriggeredStop(TriggeredMoveStatus status) noexcept;
        void advanceTriggered(double &seconds) noexcept;
        void completeTriggered(TriggeredMoveStatus status) noexcept;
        void faultTriggered() noexcept;
        [[nodiscard]] MotionState triggeredStateAt(
            double elapsed, const position_t &origin) const noexcept;
        [[nodiscard]] bool triggeredInputConditionMet(
            const TriggeredMove &move) const noexcept;
        bool initializeTriggeredJoints() noexcept;
        bool initializeTriggeredJoint(const TriggeredJointMove &move,
                                      JointId joint) noexcept;
        bool beginTriggeredJointStop(const TriggeredJointMove &move,
                                     JointId joint) noexcept;
        bool triggeredJointInputQualified(const JointTrigger &trigger,
                                          TriggeredJointRuntime &runtime) noexcept;
        void advanceTriggeredJoints(double &seconds) noexcept;
        void completeTriggeredJoints() noexcept;
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
        [[nodiscard]] TriggeredMove &activeTriggeredMove() noexcept;
        [[nodiscard]] const TriggeredMove &activeTriggeredMove() const noexcept;
        [[nodiscard]] TriggeredJointMove &activeTriggeredJointMove() noexcept;
        [[nodiscard]] const TriggeredJointMove &
        activeTriggeredJointMove() const noexcept;
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
        TriggeredRuntime m_triggered;
        std::array<TriggeredJointRuntime, MAX_JOINTS> m_triggeredJoints;
        JointMask m_triggeredJointMask = 0;
        std::bitset<DIGITAL_INPUT_CAPACITY> m_digitalInputs;
        std::bitset<DIGITAL_INPUT_CAPACITY> m_previousDigitalInputs;
        bool m_stopping = false;
        bool m_faultEventEmitted = false;
    };
}
