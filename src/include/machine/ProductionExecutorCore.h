#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

#include <ruckig/ruckig.hpp>

#include "machine/MotionBackend.h"
#include "machine/SpscChannel.h"

namespace ngc {
    struct ProductionExecutorAxisMapping {
        JointMask joints = 0;
        JointVector coordinateScale{};
    };

    struct ProductionExecutorFeedHoldConfiguration {
        // Both zero leave ordinary-plan feed hold unavailable. Positive values
        // bound the requested on-path braking and resume profile.
        double tangentialAcceleration = 0.0;
        double tangentialJerk = 0.0;
        // Aggregate and per-axis acceleration remain hard limits. The
        // tangential jerk shapes the scalar rate profile; full coupled and
        // per-axis jerk remain diagnostic rather than feasibility limits.
        double pathAcceleration = std::numeric_limits<double>::infinity();
        position_t axisAcceleration = {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
        };
    };

    struct ProductionExecutorConfiguration {
        std::array<
            ProductionExecutorAxisMapping,
            static_cast<std::size_t>(AxisId::C) + 1> axes{};
        ProductionExecutorFeedHoldConfiguration feedHold{};
        // Physical axis-space authority reserved for cancelling ordinary
        // PlanChunk motion. Zero limits leave that operation unavailable.
        AxisMotionLimits controlledStopLimits{};
        std::uint32_t maximumJogLeaseTicks =
            std::numeric_limits<std::uint32_t>::max();
    };
    static_assert(
        std::is_trivially_copyable_v<ProductionExecutorConfiguration>);

    struct ProductionExecutorOutputState {
        SpindleEvent spindle{};
    };
    static_assert(std::is_trivially_copyable_v<ProductionExecutorOutputState>);

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

        explicit ProductionExecutorCore(
            double servoPeriod,
            ProductionExecutorConfiguration configuration = {});

        ProductionExecutorCore(const ProductionExecutorCore &) = delete;
        ProductionExecutorCore &operator=(const ProductionExecutorCore &) = delete;

        PublishResult tryPublish(const ExecutionItem &item) noexcept override;
        SubmitResult trySubmit(const ControlRequest &request) noexcept override;
        bool tryTakeEvent(ExecutionEvent &event) noexcept override;
        bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override;

        void restoreStationaryState(const MotionState &commanded,
                                    const MotionState &feedback = {},
                                    const JointMotionState &commandedJoints = {},
                                    const JointMotionState &feedbackJoints = {}) noexcept;
        void serviceImmediate() noexcept;
        // The hosting servo thread updates sampled input levels before
        // servoTick(). Edge conditions compare the current sample with the
        // preceding tick; hardware acquisition remains outside this core.
        void setDigitalInputSample(DigitalInputId input, bool active) noexcept;
        void setDigitalInputSamples(
            const std::bitset<DIGITAL_INPUT_CAPACITY> &inputs) noexcept;
        void servoTick(bool publishSnapshot = true) noexcept;
        [[nodiscard]] double servoPeriod() const noexcept;
        // The hosting servo thread reads this after servoTick() and maps it to
        // physical outputs. It is not an NRT communication endpoint.
        [[nodiscard]] ProductionExecutorOutputState outputState() const noexcept;

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
            bool feedHoldStopping = false;
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

        struct JogRuntime {
            JogId id = 0;
            JogTarget target{};
            JogMotionLimits limits{};
            JogMotionLimits stopLimits{};
            JogTravelRange travel{};
            JointVector jointOrigin{};
            double axisOrigin = 0.0;
            double position = 0.0;
            double velocity = 0.0;
            double acceleration = 0.0;
            double elapsed = 0.0;
            double cruiseVelocity = 0.0;
            std::uint32_t leaseTicks = 0;
            std::uint32_t leasePeriod = 0;
            bool continuous = false;
            bool cruising = false;
            bool stopping = false;
            JogStopReason stopReason = JogStopReason::RequestedStop;
            ruckig::Trajectory<1> trajectory;
        };

        struct PlanStopRuntime {
            MotionState origin{};
            double elapsed = 0.0;
            bool stationary = false;
            ruckig::Trajectory<6> trajectory;
        };

        struct FeedRetimingRuntime {
            double rate = 1.0;
            double acceleration = 0.0;
            double jerk = 0.0;
            bool holding = false;
            bool held = false;
            bool resuming = false;
        };

        void serviceControls() noexcept;
        void activateNext() noexcept;
        void advanceActive(double seconds) noexcept;
        void advancePlan(double &seconds) noexcept;
        [[nodiscard]] bool feedHoldAvailable() const noexcept;
        bool feedRetimingAccelerationInterval(
            const position_t &referenceVelocity,
            const position_t &referenceAcceleration,
            double &lower, double &upper) const noexcept;
        double feedRetimingReferenceAdvance(double physicalSeconds) noexcept;
        [[nodiscard]] MotionState retimedState(
            const ExecutionPolynomialEvaluation &reference) const noexcept;
        void finishFeedHold() noexcept;
        void resetFeedRetiming() noexcept;
        void faultFeedRetimingAtStopBranch() noexcept;
        bool beginPlanStop() noexcept;
        void advancePlanStop(double &seconds) noexcept;
        void completePlanStop() noexcept;
        bool initializeTriggered() noexcept;
        bool beginTriggeredStop(TriggeredMoveStatus status,
                                bool feedHold = false) noexcept;
        void advanceTriggered(double &seconds) noexcept;
        void finishTriggeredFeedHold() noexcept;
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
                                     JointId joint, bool triggered) noexcept;
        bool triggeredJointInputQualified(const JointTrigger &trigger,
                                          TriggeredJointRuntime &runtime) noexcept;
        void advanceTriggeredJoints(double &seconds) noexcept;
        void completeTriggeredJoints() noexcept;
        [[nodiscard]] bool validJogTarget(const JogTarget &target) const noexcept;
        static bool validJogLimits(const JogMotionLimits &limits) noexcept;
        static bool validJogTravel(const JogTravelRange &travel) noexcept;
        [[nodiscard]] JointMask axisJoints(AxisId axis) const noexcept;
        [[nodiscard]] double jogCoordinate(const JogTarget &target) const noexcept;
        [[nodiscard]] double jogVelocity(const JogTarget &target) const noexcept;
        [[nodiscard]] double jogAcceleration(const JogTarget &target) const noexcept;
        void applyJogState() noexcept;
        bool calculateJogPosition(double distance, double velocity) noexcept;
        bool calculateJogVelocity(double targetVelocity) noexcept;
        bool initializeJog(const StartContinuousJogRequest &request) noexcept;
        bool initializeJog(const StartIncrementalJogRequest &request) noexcept;
        bool setContinuousJogVelocity(double signedVelocity) noexcept;
        bool beginJogStop(JogStopReason reason) noexcept;
        void advanceJog(double seconds) noexcept;
        void completeJog() noexcept;
        void abandonJog(JogStopReason reason) noexcept;
        void faultJog() noexcept;
        void completeSpan() noexcept;
        void selectContinuationOrStop() noexcept;
        void applyScheduledEventsForCurrentSpan() noexcept;
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
        std::optional<JogRuntime> m_jog;
        std::optional<PlanStopRuntime> m_planStop;
        FeedRetimingRuntime m_feedRetiming;
        ProductionExecutorConfiguration m_configuration;
        JointMask m_configuredJoints = 0;
        double m_servoPeriod;
        double m_spanElapsed = 0.0;
        std::uint32_t m_span = 0;
        std::uint32_t m_nextScheduledEvent = 0;
        std::uint32_t m_nextMarker = 0;
        ProductionExecutorOutputState m_outputState;
        TriggeredRuntime m_triggered;
        std::array<TriggeredJointRuntime, MAX_JOINTS> m_triggeredJoints;
        JointMask m_triggeredJointMask = 0;
        TriggeredMoveStatus m_triggeredJointCompletionStatus =
            TriggeredMoveStatus::ReachedTarget;
        EpochId m_controlledStoppedEpoch = 0;
        std::bitset<DIGITAL_INPUT_CAPACITY> m_digitalInputs;
        std::bitset<DIGITAL_INPUT_CAPACITY> m_previousDigitalInputs;
        bool m_stopping = false;
        bool m_faultEventEmitted = false;
    };
}
