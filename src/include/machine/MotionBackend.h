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
    using RequestId = std::uint64_t;
    using JogId = std::uint64_t;
    using BranchSequence = std::uint64_t;

    struct MotionState {
        position_t position{};
        position_t velocity{};
        position_t acceleration{};
    };

    // q(u) = ((a*u + b)*u + c)*u + d, u in [0, 1]. Timing and
    // all axis-limit validation are completed by NRT before publication.
    struct AxisPolynomialSpan {
        SpanId id = 0;
        double duration = 0.0;
        double inverseDuration = 0.0;
        double inverseDurationSquared = 0.0;
        double inverseDurationCubed = 0.0;
        position_t a{};
        position_t b{};
        position_t c{};
        position_t d{};
        MotionState end{};
    };

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

    enum class BackendState : std::uint8_t { Disabled, Held, Running, Faulted };
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
                                        StopJogRequest>;
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
    struct BackendHeld { EpochId epoch; MotionState state; };
    struct BackendFault { std::uint32_t code; };
    using ExecutionEvent = std::variant<ChunkAccepted, ChunkRejected, ChunkRetired, BranchSelected,
                                        TriggeredMoveCompleted, TriggeredJointMoveCompleted,
                                        JogStopped, RequestCompleted, BackendHeld, BackendFault>;
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
