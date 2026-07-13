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
    using BranchSequence = std::uint64_t;

    struct MotionState {
        position_t position{};
        position_t velocity{};
        position_t acceleration{};
    };

    struct ProgramCursor {
        std::uint64_t block = 0;
        double pathParameter = 0.0;
        std::uint32_t event = 0;
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
        ProgramCursor branchCursor{};
        ProgramCursor stopCursor{};
    };

    static_assert(std::is_trivially_copyable_v<PlanChunk>);

    using DigitalInputId = std::uint16_t;
    using TriggeredMoveId = std::uint64_t;

    enum class InputCondition : std::uint8_t { Active, Inactive, RisingEdge, FallingEdge };

    struct AxisMotionLimits {
        position_t velocity{};
        position_t acceleration{};
        position_t jerk{};
    };

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
        ProgramCursor cursor{};
    };

    using ExecutionItem = std::variant<PlanChunk, TriggeredMove>;
    static_assert(std::is_trivially_copyable_v<ExecutionItem>);

    enum class BackendState : std::uint8_t { Disabled, Held, Running, Faulted };
    enum class BranchChoice : std::uint8_t { Continue, Stop };
    enum class PublishResult : std::uint8_t { Published, Full, Invalid };
    enum class SubmitResult : std::uint8_t { Submitted, Full };

    struct EnableRequest { RequestId id = 0; };
    struct DisableRequest { RequestId id = 0; };
    struct StartRequest { RequestId id = 0; EpochId epoch = 0; };
    struct FeedHoldRequest { RequestId id = 0; };
    struct ResumeRequest { RequestId id = 0; EpochId epoch = 0; };
    struct AbortRequest { RequestId id = 0; };
    struct ResetRequest { RequestId id = 0; EpochId nextEpoch = 0; };
    using ControlRequest = std::variant<EnableRequest, DisableRequest, StartRequest,
                                        FeedHoldRequest, ResumeRequest, AbortRequest, ResetRequest>;

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
    struct RequestCompleted { RequestId request; bool succeeded; };
    struct BackendHeld { EpochId epoch; MotionState state; ProgramCursor cursor; };
    struct BackendFault { std::uint32_t code; };
    using ExecutionEvent = std::variant<ChunkAccepted, ChunkRejected, ChunkRetired, BranchSelected,
                                        TriggeredMoveCompleted, RequestCompleted, BackendHeld, BackendFault>;

    struct ExecutionSnapshot {
        BackendState state = BackendState::Disabled;
        EpochId epoch = 0;
        ChunkId activeChunk = 0;
        SpanId activeSpan = 0;
        double spanProgress = 0.0;
        BranchSequence lastBranch = 0;
        MotionState commanded{};
        MotionState feedback{};
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
