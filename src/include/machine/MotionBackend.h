#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <variant>

#include "machine/MachineCommand.h"

namespace ngc {
    using EpochId = std::uint64_t;
    using ChunkId = std::uint64_t;
    using SpanId = std::uint64_t;
    using ExecutionMarkerId = std::uint64_t;
    using RequestId = std::uint64_t;
    using JogId = std::uint64_t;
    using BranchSequence = std::uint64_t;

    struct MotionState {
        position_t position{};
        position_t velocity{};
        position_t acceleration{};
    };

    enum class ExecutionPolynomialDegree : std::uint8_t {
        Cubic = 3,
        Quintic = 5,
    };

    // q(u) = origin + c1*u + c2*u^2 + ... + c5*u^5, u in [0, 1].
    // Cubics use zero c4 and c5 coefficients. Timing and all axis-limit
    // validation are completed by NRT before publication.
    struct AxisPolynomialSpan {
        SpanId id = 0;
        ExecutionPolynomialDegree degree = ExecutionPolynomialDegree::Cubic;
        double duration = 0.0;
        double inverseDuration = 0.0;
        double inverseDurationSquared = 0.0;
        double inverseDurationCubed = 0.0;
        position_t origin{};
        std::array<position_t, 5> coefficients{};
    };

    struct ExecutionPolynomialEvaluation {
        MotionState state{};
        position_t jerk{};
    };

    inline ExecutionPolynomialEvaluation evaluateExecutionPolynomial(
            const AxisPolynomialSpan &span, const double u) noexcept {
        const auto component = [&](const double position_t::*member) {
            const auto c1 = span.coefficients[0].*member;
            const auto c2 = span.coefficients[1].*member;
            const auto c3 = span.coefficients[2].*member;
            const auto c4 = span.coefficients[3].*member;
            const auto c5 = span.coefficients[4].*member;
            return std::array{
                span.origin.*member
                    + ((((c5 * u + c4) * u + c3) * u + c2) * u + c1) * u,
                ((((5.0 * c5 * u + 4.0 * c4) * u + 3.0 * c3) * u
                    + 2.0 * c2) * u + c1) * span.inverseDuration,
                (((20.0 * c5 * u + 12.0 * c4) * u + 6.0 * c3) * u
                    + 2.0 * c2) * span.inverseDurationSquared,
                ((60.0 * c5 * u + 24.0 * c4) * u + 6.0 * c3)
                    * span.inverseDurationCubed,
            };
        };
        const auto x = component(&position_t::x);
        const auto y = component(&position_t::y);
        const auto z = component(&position_t::z);
        const auto a = component(&position_t::a);
        const auto b = component(&position_t::b);
        const auto c = component(&position_t::c);

        return {
            .state = {
                .position = {x[0], y[0], z[0], a[0], b[0], c[0]},
                .velocity = {x[1], y[1], z[1], a[1], b[1], c[1]},
                .acceleration = {x[2], y[2], z[2], a[2], b[2], c[2]},
            },
            .jerk = {x[3], y[3], z[3], a[3], b[3], c[3]},
        };
    }

    inline MotionState executionSpanStart(
            const AxisPolynomialSpan &span) noexcept {
        return evaluateExecutionPolynomial(span, 0.0).state;
    }

    inline MotionState executionSpanEnd(
            const AxisPolynomialSpan &span) noexcept {
        return evaluateExecutionPolynomial(span, 1.0).state;
    }

    struct SpindleEvent {
        bool enabled = false;
        Direction direction = Direction::CW;
        double speed = 0.0;
    };

    using TrajectoryEvent = std::variant<SpindleEvent>;

    struct ScheduledEvent {
        // The backend activates this event before evaluating the indexed span.
        std::uint32_t span = 0;
        TrajectoryEvent value;
    };

    inline constexpr std::size_t MAX_NORMAL_SPANS_PER_CHUNK = 256;
    inline constexpr std::size_t MAX_STOP_SPANS_PER_CHUNK = 16;
    inline constexpr std::size_t MAX_EVENTS_PER_CHUNK = 16;
    inline constexpr std::size_t MAX_EXECUTION_MARKERS_PER_CHUNK = 256;

    struct ExecutionMarker {
        ExecutionMarkerId id = 0;
        std::uint32_t span = 0;
        double parameter = 0.0;
    };

    template<typename T, std::size_t Capacity>
    struct FixedArray {
        std::array<T, Capacity> values{};
        std::uint32_t size = 0;

        constexpr bool push(const T &value) noexcept {
            if(size == Capacity) return false;
            values[size++] = value;
            return true;
        }

        constexpr T &operator[](const std::size_t index) noexcept { return values[index]; }
        constexpr const T &operator[](const std::size_t index) const noexcept { return values[index]; }
        constexpr auto begin() noexcept { return values.begin(); }
        constexpr auto begin() const noexcept { return values.begin(); }
        constexpr auto end() noexcept { return values.begin() + size; }
        constexpr auto end() const noexcept { return values.begin() + size; }
    };

    struct PlanChunk {
        EpochId epoch = 0;
        ChunkId id = 0;
        BranchSequence predecessorBranch = 0;
        BranchSequence branch = 0;
        FixedArray<AxisPolynomialSpan, MAX_NORMAL_SPANS_PER_CHUNK> normalMotion;
        FixedArray<AxisPolynomialSpan, MAX_STOP_SPANS_PER_CHUNK> stopTail;
        FixedArray<ScheduledEvent, MAX_EVENTS_PER_CHUNK> events;
        FixedArray<ExecutionMarker, MAX_EXECUTION_MARKERS_PER_CHUNK> markers;
        MotionState branchState{};
        MotionState stopState{};
    };

    static_assert(std::is_trivially_copyable_v<PlanChunk>);

    using DigitalInputId = std::uint16_t;
    using TriggeredMoveId = std::uint64_t;
    using JointId = std::uint8_t;
    using JointMask = std::uint16_t;
    inline constexpr std::size_t MAX_JOINTS = 12;

    enum class InputCondition : std::uint8_t { Active, Inactive, RisingEdge, FallingEdge };

    struct AxisMotionLimits {
        position_t velocity{};
        position_t acceleration{};
        position_t jerk{};
    };

    struct JointVector {
        std::array<double, MAX_JOINTS> values{};

        constexpr double &operator[](const JointId joint) noexcept { return values[joint]; }
        constexpr double operator[](const JointId joint) const noexcept { return values[joint]; }
    };

    struct JointMotionState {
        JointVector position{};
        JointVector velocity{};
        JointVector acceleration{};
    };

    struct JointMotionLimits {
        JointVector velocity{};
        JointVector acceleration{};
        JointVector jerk{};
    };

    struct JointTrigger {
        JointId joint = 0;
        DigitalInputId input = 0;
        InputCondition condition = InputCondition::Active;
        double debounce = 0.0;
    };

    enum class JointTargetMode : std::uint8_t { Absolute, Relative };

    // A bounded executor-owned point-to-point move terminated by a sampled
    // digital-input condition. The executor generates both the approach and the
    // constrained stop. This primitive is shared by probing and homing phases.
    struct TriggeredMove {
        EpochId epoch = 0;
        ChunkId id = 0;
        BranchSequence predecessorBranch = 0;
        BranchSequence branch = 0;
        TriggeredMoveId moveId = 0;
        position_t target{};
        AxisMotionLimits limits{};
        DigitalInputId input = 0;
        InputCondition condition = InputCondition::Active;
        bool triggerRequired = false;
    };

    // Joint-space counterpart used for gantry squaring and other service
    // operations. Each joint stops independently on its own input transition;
    // the item completes after every selected joint has stopped or reached target.
    struct TriggeredJointMove {
        EpochId epoch = 0;
        ChunkId id = 0;
        BranchSequence predecessorBranch = 0;
        BranchSequence branch = 0;
        TriggeredMoveId moveId = 0;
        JointMask joints = 0;
        JointTargetMode targetMode = JointTargetMode::Absolute;
        JointVector target{};
        JointMotionLimits limits{};
        FixedArray<JointTrigger, MAX_JOINTS> triggers;
        bool triggerRequired = false;
    };

    using ExecutionItem = std::variant<PlanChunk, TriggeredMove, TriggeredJointMove>;
    static_assert(std::is_trivially_copyable_v<ExecutionItem>);

    enum class BackendState : std::uint8_t { Disabled, Held, Running, Holding, Faulted };
    enum class BranchChoice : std::uint8_t { Continue, Stop };
    enum class PublishResult : std::uint8_t { Published, Full, Invalid };
    enum class SubmitResult : std::uint8_t { Submitted, Full };

    enum class AxisId : std::uint8_t { X, Y, Z, A, B, C };
    enum class JogTargetType : std::uint8_t { Axis, JointGroup, Joint };

    struct JogTarget {
        JogTargetType type = JogTargetType::Axis;
        AxisId axis = AxisId::X;
        // Zero for Axis. JointGroup selects all coupled joints atomically; Joint
        // must contain exactly one bit. The backend validates the mask against
        // its typed machine configuration. JointGroup and Joint targets remain
        // usable before homing; Axis targets require a valid machine coordinate.
        JointMask joints = 0;
    };

    struct JogMotionLimits {
        double velocity = 0.0;
        double acceleration = 0.0;
        double jerk = 0.0;
    };

    struct JogTravelRange {
        double minimum = 0.0;
        double maximum = 0.0;
        bool enabled = false;
    };

    // Continuous jogging is a renewable dead-man lease measured in fixed servo
    // ticks. The backend clamps leaseTicks to its configured maximum and begins
    // a constrained stop if the matching renewal is not received in time.
    struct StartContinuousJogRequest {
        RequestId id = 0;
        JogId jog = 0;
        JogTarget target{};
        double signedVelocity = 0.0;
        // Reduced limits for entering the jog; stopLimits retain the selected
        // axis/joint's physical stopping authority.
        JogMotionLimits limits{};
        JogMotionLimits stopLimits{};
        JogTravelRange travel{};
        std::uint32_t leaseTicks = 0;
    };

    struct StartIncrementalJogRequest {
        RequestId id = 0;
        JogId jog = 0;
        JogTarget target{};
        double distance = 0.0;
        double velocity = 0.0;
        // Reduced jog profile limits and separate physical stop limits.
        JogMotionLimits limits{};
        JogMotionLimits stopLimits{};
        JogTravelRange travel{};
    };

    // Renewals and stops apply only to the exactly matching active jog token;
    // delayed requests for an old token cannot revive or stop a newer jog.
    struct RenewJogLeaseRequest { RequestId id = 0; JogId jog = 0; };
    struct SetContinuousJogVelocityRequest {
        RequestId id = 0;
        JogId jog = 0;
        double signedVelocity = 0.0;
    };
    struct StopJogRequest { RequestId id = 0; JogId jog = 0; };

    struct EnableRequest { RequestId id = 0; };
    struct DisableRequest { RequestId id = 0; };
    struct StartRequest { RequestId id = 0; EpochId epoch = 0; };
    struct FeedHoldRequest { RequestId id = 0; };
    struct ResumeRequest { RequestId id = 0; EpochId epoch = 0; };
    struct AbortRequest { RequestId id = 0; };
    struct ResetRequest { RequestId id = 0; EpochId nextEpoch = 0; };
    struct SetJointPositionRequest { RequestId id = 0; JointMask joints = 0; JointVector position{}; };
    using ControlRequest = std::variant<EnableRequest, DisableRequest, StartRequest,
                                        FeedHoldRequest, ResumeRequest, AbortRequest, ResetRequest,
                                        SetJointPositionRequest, StartContinuousJogRequest,
                                        StartIncrementalJogRequest, RenewJogLeaseRequest,
                                        SetContinuousJogVelocityRequest, StopJogRequest>;
    static_assert(std::is_trivially_copyable_v<ControlRequest>);

    struct ChunkAccepted { EpochId epoch; ChunkId chunk; };
    struct ChunkRejected { EpochId epoch; ChunkId chunk; };
    struct ChunkRetired { EpochId epoch; ChunkId chunk; };
    struct BranchSelected { EpochId epoch; BranchSequence branch; BranchChoice choice; ChunkId continuation; };
    enum class TriggeredMoveStatus : std::uint8_t { Triggered, ReachedTarget, Aborted, Fault };
    struct TriggeredMoveCompleted {
        EpochId epoch;
        TriggeredMoveId move;
        TriggeredMoveStatus status;
        MotionState triggerState;
        MotionState stoppedState;
    };
    struct TriggeredJointMoveCompleted {
        EpochId epoch;
        TriggeredMoveId move;
        TriggeredMoveStatus status;
        JointMask triggeredJoints;
        JointMotionState triggerState;
        JointMotionState stoppedState;
    };
    enum class JogStopReason : std::uint8_t {
        TargetReached,
        RequestedStop,
        LeaseExpired,
        LimitReached,
        Disabled,
        Aborted,
        Fault,
        Superseded,
    };
    struct JogStopped {
        JogId jog;
        JogTarget target;
        JogStopReason reason;
        MotionState axisState;
        JointMotionState jointState;
    };
    struct RequestCompleted { RequestId request; bool succeeded; };
    struct ExecutionMarkerReached {
        EpochId epoch;
        ChunkId chunk;
        ExecutionMarkerId marker;
        SpanId span;
        double parameter;
    };
    enum class BackendHoldReason : std::uint8_t { StopBranch, FeedHold };
    struct BackendHeld { EpochId epoch; MotionState state; BackendHoldReason reason; };
    struct BackendFault { std::uint32_t code; };
    using ExecutionEvent = std::variant<ChunkAccepted, ChunkRejected, ChunkRetired, BranchSelected,
                                        TriggeredMoveCompleted, TriggeredJointMoveCompleted,
                                        JogStopped, RequestCompleted, ExecutionMarkerReached,
                                        BackendHeld, BackendFault>;
    static_assert(std::is_trivially_copyable_v<ExecutionEvent>);

    struct ExecutionSnapshot {
        BackendState state = BackendState::Disabled;
        EpochId epoch = 0;
        ChunkId activeChunk = 0;
        SpanId activeSpan = 0;
        double spanProgress = 0.0;
        double activeNormalMotionRemainingSeconds = 0.0;
        double queuedNormalMotionSeconds = 0.0;
        double committedNormalMotionSeconds = 0.0;
        double stopBranchRemainingSeconds = 0.0;
        double executionRate = 1.0;
        double executionRateAcceleration = 0.0;
        std::uint32_t queuedExecutionItems = 0;
        BranchSequence lastBranch = 0;
        MotionState commanded{};
        MotionState feedback{};
        JointMask activeJoints = 0;
        JointMotionState commandedJoints{};
        JointMotionState feedbackJoints{};
        std::uint32_t faultCode = 0;
    };

    // NRT-facing endpoint. Calls only access bounded communication storage; they
    // never invoke the servo loop, allocate, wait, or acquire an RT-owned mutex.
    class MotionBackend {
    public:
        virtual ~MotionBackend() = default;
        virtual PublishResult tryPublish(const ExecutionItem &item) noexcept = 0;
        virtual SubmitResult trySubmit(const ControlRequest &request) noexcept = 0;
        virtual bool tryTakeEvent(ExecutionEvent &event) noexcept = 0;
        virtual bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept = 0;
    };

    // Simulation clock control is deliberately not part of the physical backend contract.
    class SimulatedClockControl {
    public:
        virtual ~SimulatedClockControl() = default;
        virtual void advance(double seconds) = 0;
        virtual void runUntilIdle() = 0;
    };
}
