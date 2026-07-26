#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "machine/ProductionExecutorCore.h"

namespace {
    void require(const bool condition, const std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    void requireNear(const double actual, const double expected,
                     const std::string_view message) {
        if (std::abs(actual - expected) > 1e-9) {
            throw std::runtime_error(std::string(message));
        }
    }

    ngc::AxisPolynomialSpan linearSpan(const ngc::SpanId id,
                                       const double from, const double to,
                                       const double duration) {
        ngc::AxisPolynomialSpan span;
        span.id = id;
        span.duration = duration;
        span.inverseDuration = 1.0 / duration;
        span.inverseDurationSquared =
            span.inverseDuration * span.inverseDuration;
        span.inverseDurationCubed =
            span.inverseDurationSquared * span.inverseDuration;
        span.origin.x = from;
        span.coefficients[0].x = to - from;

        return span;
    }

    ngc::PlanChunk linearChunk(const ngc::EpochId epoch,
                               const ngc::ChunkId id,
                               const ngc::BranchSequence predecessor,
                               const ngc::BranchSequence branch,
                               const ngc::SpanId span, const double from,
                               const double to, const double duration) {
        ngc::PlanChunk chunk;
        chunk.epoch = epoch;
        chunk.id = id;
        chunk.predecessorBranch = predecessor;
        chunk.branch = branch;
        require(chunk.normalMotion.push(
                    linearSpan(span, from, to, duration)),
                "normal span did not fit");
        require(chunk.stopTail.push(
                    linearSpan(span + 1, to, to, 0.25)),
                "stop span did not fit");
        chunk.branchState = ngc::executionSpanEnd(chunk.normalMotion[0]);
        chunk.stopState.position.x = to;

        return chunk;
    }

    ngc::TriggeredMove triggeredMove(
        const ngc::EpochId epoch, const ngc::ChunkId id,
        const ngc::BranchSequence predecessor,
        const ngc::BranchSequence branch,
        const ngc::TriggeredMoveId moveId, const double target,
        const ngc::DigitalInputId input = 0,
        const ngc::InputCondition condition =
            ngc::InputCondition::RisingEdge) {
        ngc::TriggeredMove move;
        move.epoch = epoch;
        move.id = id;
        move.predecessorBranch = predecessor;
        move.branch = branch;
        move.moveId = moveId;
        move.target.x = target;
        move.limits.velocity.x = 0.5;
        move.limits.acceleration.x = 1.0;
        move.limits.jerk.x = 4.0;
        move.input = input;
        move.condition = condition;

        return move;
    }

    ngc::TriggeredJointMove triggeredJointMove(
        const ngc::EpochId epoch, const ngc::ChunkId id,
        const ngc::BranchSequence predecessor,
        const ngc::BranchSequence branch,
        const ngc::TriggeredMoveId moveId) {
        ngc::TriggeredJointMove move;
        move.epoch = epoch;
        move.id = id;
        move.predecessorBranch = predecessor;
        move.branch = branch;
        move.moveId = moveId;
        move.joints = ngc::JointMask{1} | ngc::JointMask{1} << 1;
        move.target[0] = 0.5;
        move.target[1] = 0.75;
        for (ngc::JointId joint = 0; joint < 2; ++joint) {
            move.limits.velocity[joint] = 0.5;
            move.limits.acceleration[joint] = 1.0;
            move.limits.jerk[joint] = 4.0;
        }

        return move;
    }

    std::vector<ngc::ExecutionEvent> takeEvents(
        ngc::ProductionExecutorCore &core) {
        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionEvent event;
        while (core.tryTakeEvent(event)) {
            events.push_back(event);
        }

        return events;
    }

    ngc::ExecutionSnapshot latestSnapshot(
        ngc::ProductionExecutorCore &core) {
        ngc::ExecutionSnapshot snapshot;
        auto found = false;
        while (core.tryTakeSnapshot(snapshot)) {
            found = true;
        }
        require(found, "executor did not publish a snapshot");

        return snapshot;
    }

    void initialize(ngc::ProductionExecutorCore &core,
                    const ngc::EpochId epoch) {
        require(core.trySubmit(ngc::ResetRequest{1, epoch})
                    == ngc::SubmitResult::Submitted,
                "reset did not fit");
        core.servoTick();
        takeEvents(core);
        latestSnapshot(core);

        require(core.trySubmit(ngc::EnableRequest{2})
                    == ngc::SubmitResult::Submitted,
                "enable did not fit");
        core.servoTick();
        const auto events = takeEvents(core);
        require(std::ranges::any_of(events, [](const auto &event) {
            const auto *completed =
                std::get_if<ngc::RequestCompleted>(&event);
            return completed != nullptr && completed->succeeded;
        }), "enable was not acknowledged");
        require(latestSnapshot(core).state == ngc::BackendState::Held,
                "enable did not establish held state");
    }

    template<typename Event>
    std::vector<Event> selectEvents(
        const std::vector<ngc::ExecutionEvent> &events) {
        std::vector<Event> selected;
        for (const auto &event : events) {
            if (const auto *value = std::get_if<Event>(&event)) {
                selected.push_back(*value);
            }
        }

        return selected;
    }

    void testFixedTickExecutionAndAccounting() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.25);
        initialize(*core, 10);

        const auto chunk =
            linearChunk(10, 101, 0, 201, 301, 0.0, 1.0, 1.0);
        require(core->tryPublish(chunk) == ngc::PublishResult::Published,
                "valid chunk was not published");
        require(core->trySubmit(ngc::StartRequest{3, 10})
                    == ngc::SubmitResult::Submitted,
                "start did not fit");

        core->servoTick();
        const auto snapshot = latestSnapshot(*core);
        require(snapshot.state == ngc::BackendState::Running,
                "start did not establish running state");
        requireNear(snapshot.commanded.position.x, 0.25,
                    "one fixed tick did not evaluate the polynomial");
        requireNear(snapshot.activeNormalMotionRemainingSeconds, 0.75,
                    "active normal duration was not reported");
        requireNear(snapshot.stopBranchRemainingSeconds, 0.25,
                    "stop-tail duration was not reported separately");
        requireNear(snapshot.committedNormalMotionSeconds, 0.75,
                    "committed normal duration was not reported");
        require(snapshot.queuedExecutionItems == 0,
                "activated chunk remained in the queued count");
    }

    void testContinuationMarkersAndTerminalStop() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.25);
        initialize(*core, 20);

        auto first =
            linearChunk(20, 201, 0, 301, 401, 0.0, 1.0, 1.0);
        auto second =
            linearChunk(20, 202, 301, 302, 403, 1.0, 2.0, 0.5);
        require(first.markers.push({501, 0, 0.0})
                    && first.markers.push({502, 0, 0.5})
                    && first.markers.push({503, 0, 1.0})
                    && second.markers.push({504, 0, 0.0})
                    && second.markers.push({505, 0, 1.0}),
                "marker fixtures did not fit");
        require(core->tryPublish(first) == ngc::PublishResult::Published
                    && core->tryPublish(second)
                        == ngc::PublishResult::Published,
                "dependent chunks were not published");
        require(core->trySubmit(ngc::StartRequest{3, 20})
                    == ngc::SubmitResult::Submitted,
                "start did not fit");

        std::vector<ngc::ExecutionEvent> events;
        for (auto tick = 0; tick < 7; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            latestSnapshot(*core);
        }

        const auto markers =
            selectEvents<ngc::ExecutionMarkerReached>(events);
        require(markers.size() == 5
                    && markers[0].marker == 501
                    && markers[1].marker == 502
                    && markers[2].marker == 503
                    && markers[3].marker == 504
                    && markers[4].marker == 505,
                "markers were not emitted exactly once in trajectory order");

        const auto branches = selectEvents<ngc::BranchSelected>(events);
        require(branches.size() == 2
                    && branches[0].choice == ngc::BranchChoice::Continue
                    && branches[0].continuation == second.id
                    && branches[1].choice == ngc::BranchChoice::Stop,
                "executor did not continue and then select the stop branch");

        const auto retired = selectEvents<ngc::ChunkRetired>(events);
        require(retired.size() == 2
                    && retired[0].chunk == first.id
                    && retired[1].chunk == second.id,
                "chunks were not retired exactly once in execution order");
        const auto held = selectEvents<ngc::BackendHeld>(events);
        require(held.size() == 1
                    && held[0].reason == ngc::BackendHoldReason::StopBranch,
                "executor did not report the terminal stop hold");

        core->servoTick();
        const auto completed = latestSnapshot(*core);
        require(completed.state == ngc::BackendState::Held,
                "terminal stop did not leave the executor held");
        requireNear(completed.commanded.position.x, 2.0,
                    "terminal stop did not reach its declared state");
    }

    void testMismatchedContinuationStopsSafely() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.25);
        initialize(*core, 30);

        const auto first =
            linearChunk(30, 301, 0, 401, 501, 0.0, 1.0, 0.5);
        const auto stale =
            linearChunk(30, 302, 999, 402, 503, 1.0, 2.0, 0.5);
        require(core->tryPublish(first) == ngc::PublishResult::Published
                    && core->tryPublish(stale)
                        == ngc::PublishResult::Published,
                "mismatched-continuation fixture did not publish");
        require(core->trySubmit(ngc::StartRequest{3, 30})
                    == ngc::SubmitResult::Submitted,
                "start did not fit");

        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot completed;
        for (auto tick = 0; tick < 3; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            completed = latestSnapshot(*core);
        }

        const auto rejected = selectEvents<ngc::ChunkRejected>(events);
        const auto branches = selectEvents<ngc::BranchSelected>(events);
        require(rejected.size() == 1 && rejected[0].chunk == stale.id,
                "mismatched continuation was not rejected");
        require(branches.size() == 1
                    && branches[0].choice == ngc::BranchChoice::Stop,
                "mismatched continuation did not select the proved stop tail");
        require(completed.state == ngc::BackendState::Held,
                "proved stop tail did not finish held");
    }

    void testTriggeredMoveReachesTargetAtRest() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 40);

        const auto move = triggeredMove(40, 401, 0, 501, 601, 0.2);
        require(core->tryPublish(ngc::ExecutionItem{move})
                    == ngc::PublishResult::Published,
                "triggered move was not published");
        require(core->trySubmit(ngc::StartRequest{3, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "triggered move did not accept start");

        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot completed;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            completed = latestSnapshot(*core);
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto moves =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(moves.size() == 1 && moves[0].move == move.moveId
                    && moves[0].status
                        == ngc::TriggeredMoveStatus::ReachedTarget,
                "untriggered move did not report target completion");
        requireNear(moves[0].stoppedState.position.x, move.target.x,
                    "triggered move did not reach its target");
        requireNear(moves[0].stoppedState.velocity.x, 0.0,
                    "target completion retained velocity");
        requireNear(moves[0].stoppedState.acceleration.x, 0.0,
                    "target completion retained acceleration");
        require(completed.state == ngc::BackendState::Held
                    && completed.lastBranch == move.branch,
                "target completion did not establish the terminal hold");
    }

    void testSampledTriggerGeneratesConstrainedStop() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 50);

        constexpr ngc::DigitalInputId input = 7;
        const auto move = triggeredMove(
            50, 501, 0, 601, 701, 1.0, input,
            ngc::InputCondition::RisingEdge);
        require(core->tryPublish(ngc::ExecutionItem{move})
                    == ngc::PublishResult::Published,
                "sampled-trigger move was not published");
        require(core->trySubmit(ngc::StartRequest{3, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "sampled-trigger move did not accept start");

        ngc::ExecutionSnapshot sampled;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            sampled = latestSnapshot(*core);
            if (sampled.commanded.position.x >= 0.1) {
                break;
            }
        }
        require(sampled.commanded.position.x >= 0.1
                    && sampled.commanded.velocity.x > 0.0,
                "trigger fixture did not establish moving approach state");

        core->setDigitalInputSample(input, true);
        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot completed;
        auto previousAcceleration = sampled.commanded.acceleration.x;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            completed = latestSnapshot(*core);
            require(std::abs(completed.commanded.velocity.x)
                        <= move.limits.velocity.x + 1e-9,
                    "triggered stop exceeded its velocity limit");
            require(std::abs(completed.commanded.acceleration.x)
                        <= move.limits.acceleration.x + 1e-9,
                    "triggered stop exceeded its acceleration limit");
            require(std::abs(completed.commanded.acceleration.x
                             - previousAcceleration)
                        <= move.limits.jerk.x * core->servoPeriod() + 1e-9,
                    "triggered stop exceeded its per-tick jerk limit");
            previousAcceleration = completed.commanded.acceleration.x;
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto moves =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(moves.size() == 1
                    && moves[0].status == ngc::TriggeredMoveStatus::Triggered,
                "sampled input edge did not complete the move as triggered");
        requireNear(moves[0].triggerState.position.x,
                    sampled.commanded.position.x,
                    "trigger state did not match the servo-sampled position");
        require(moves[0].triggerState.velocity.x > 0.0
                    && moves[0].stoppedState.position.x
                        > moves[0].triggerState.position.x
                    && moves[0].stoppedState.position.x < move.target.x,
                "triggered move did not retain contact and stopping states");
        requireNear(moves[0].stoppedState.velocity.x, 0.0,
                    "triggered stop retained velocity");
        requireNear(moves[0].stoppedState.acceleration.x, 0.0,
                    "triggered stop retained acceleration");
    }

    void testPlanContinuesIntoTriggeredMove() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 60);

        const auto plan =
            linearChunk(60, 601, 0, 701, 801, 0.0, 0.1, 0.1);
        const auto move =
            triggeredMove(60, 602, plan.branch, 702, 802, 0.3);
        require(core->tryPublish(ngc::ExecutionItem{plan})
                    == ngc::PublishResult::Published
                    && core->tryPublish(ngc::ExecutionItem{move})
                        == ngc::PublishResult::Published,
                "plan-to-triggered horizon was not published");
        require(core->trySubmit(ngc::StartRequest{3, plan.epoch})
                    == ngc::SubmitResult::Submitted,
                "plan-to-triggered horizon did not accept start");

        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot completed;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            completed = latestSnapshot(*core);
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto branches = selectEvents<ngc::BranchSelected>(events);
        require(branches.size() == 2
                    && branches[0].branch == plan.branch
                    && branches[0].choice == ngc::BranchChoice::Continue
                    && branches[0].continuation == move.id
                    && branches[1].branch == move.branch
                    && branches[1].choice == ngc::BranchChoice::Stop,
                "plan did not continue into the triggered item");
        const auto retired = selectEvents<ngc::ChunkRetired>(events);
        require(retired.size() == 2 && retired[0].chunk == plan.id
                    && retired[1].chunk == move.id,
                "plan-to-triggered items were not retired in order");
        const auto moves =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(moves.size() == 1
                    && moves[0].status
                        == ngc::TriggeredMoveStatus::ReachedTarget,
                "continued triggered move did not reach its target");
        requireNear(completed.commanded.position.x, move.target.x,
                    "continued triggered move stopped at the wrong position");
    }

    void testTriggeredJointsStopIndependentlyAndRetainState() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 70);

        constexpr ngc::DigitalInputId firstInput = 10;
        constexpr ngc::DigitalInputId secondInput = 11;
        auto move = triggeredJointMove(70, 701, 0, 801, 901);
        move.triggerRequired = true;
        require(move.triggers.push({
                    0, firstInput, ngc::InputCondition::Active, 0.02,
                }) && move.triggers.push({
                    1, secondInput, ngc::InputCondition::RisingEdge, 0.0,
                }),
                "joint-trigger fixtures did not fit");
        require(core->tryPublish(ngc::ExecutionItem{move})
                    == ngc::PublishResult::Published,
                "triggered joint move was not published");
        require(core->trySubmit(ngc::StartRequest{3, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "triggered joint move did not accept start");

        ngc::ExecutionSnapshot approaching;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            approaching = latestSnapshot(*core);
            if (approaching.commandedJoints.position[0] >= 0.05) {
                break;
            }
        }
        require(approaching.state == ngc::BackendState::Running
                    && approaching.activeJoints == move.joints
                    && approaching.commandedJoints.velocity[0] > 0.0
                    && approaching.commandedJoints.velocity[1] > 0.0,
                "joint trigger fixture did not establish moving joint state");

        core->setDigitalInputSample(firstInput, true);
        core->setDigitalInputSample(secondInput, true);
        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot completed;
        auto previousAcceleration = std::array{
            approaching.commandedJoints.acceleration[0],
            approaching.commandedJoints.acceleration[1],
        };
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            completed = latestSnapshot(*core);
            for (ngc::JointId joint = 0; joint < 2; ++joint) {
                require(std::abs(completed.commandedJoints.velocity[joint])
                            <= move.limits.velocity[joint] + 1e-9,
                        "triggered joint exceeded its velocity limit");
                require(std::abs(completed.commandedJoints.acceleration[joint])
                            <= move.limits.acceleration[joint] + 1e-9,
                        "triggered joint exceeded its acceleration limit");
                require(std::abs(
                            completed.commandedJoints.acceleration[joint]
                            - previousAcceleration[joint])
                            <= move.limits.jerk[joint]
                                * core->servoPeriod() + 1e-9,
                        "triggered joint exceeded its per-tick jerk limit");
                previousAcceleration[joint] =
                    completed.commandedJoints.acceleration[joint];
            }
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto completions =
            selectEvents<ngc::TriggeredJointMoveCompleted>(events);
        require(completions.size() == 1
                    && completions[0].move == move.moveId
                    && completions[0].status
                        == ngc::TriggeredMoveStatus::Triggered
                    && completions[0].triggeredJoints == move.joints,
                "joint inputs did not complete every selected joint as triggered");
        require(completions[0].triggerState.position[0]
                    > completions[0].triggerState.position[1],
                "debounced joint did not continue independently after the edge-triggered joint stopped");
        for (ngc::JointId joint = 0; joint < 2; ++joint) {
            requireNear(completions[0].stoppedState.velocity[joint], 0.0,
                        "triggered joint retained velocity");
            requireNear(completions[0].stoppedState.acceleration[joint], 0.0,
                        "triggered joint retained acceleration");
            require(completions[0].stoppedState.position[joint]
                        > completions[0].triggerState.position[joint],
                    "triggered joint did not execute its constrained stop");
        }
        require(completed.state == ngc::BackendState::Held
                    && completed.activeJoints == 0
                    && completed.lastBranch == move.branch,
                "triggered joint completion did not establish a terminal hold");

        const auto retainedPosition =
            completed.commandedJoints.position[0];
        initialize(*core, 71);
        auto relative = triggeredJointMove(71, 702, 0, 802, 902);
        relative.joints = ngc::JointMask{1};
        relative.targetMode = ngc::JointTargetMode::Relative;
        relative.target[0] = 0.1;
        require(core->tryPublish(ngc::ExecutionItem{relative})
                    == ngc::PublishResult::Published,
                "relative triggered joint move was not published");
        require(core->trySubmit(ngc::StartRequest{3, relative.epoch})
                    == ngc::SubmitResult::Submitted,
                "relative triggered joint move did not accept start");

        events.clear();
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            completed = latestSnapshot(*core);
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }
        const auto relativeCompletions =
            selectEvents<ngc::TriggeredJointMoveCompleted>(events);
        require(relativeCompletions.size() == 1
                    && relativeCompletions[0].status
                        == ngc::TriggeredMoveStatus::ReachedTarget,
                "untriggered relative joint move did not reach its target");
        requireNear(completed.commandedJoints.position[0],
                    retainedPosition + relative.target[0],
                    "relative joint target did not use retained joint state");
    }

    void testBoundedAndExplicitlyUnsupportedInputs() {
        auto rejectedPeriod = false;
        try {
            const auto invalid =
                std::make_unique<ngc::ProductionExecutorCore>(0.0);
            static_cast<void>(invalid);
        } catch (const std::invalid_argument &) {
            rejectedPeriod = true;
        }
        require(rejectedPeriod,
                "executor accepted a non-positive fixed servo period");

        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.001);

        ngc::TriggeredMove triggered;
        triggered.epoch = 1;
        triggered.id = 1;
        triggered.moveId = 1;
        require(core->tryPublish(ngc::ExecutionItem{triggered})
                    == ngc::PublishResult::Invalid,
                "executor accepted a triggered move with invalid limits");

        auto triggeredJoint =
            triggeredJointMove(1, 1, 0, 1, 1);
        triggeredJoint.triggerRequired = true;
        require(triggeredJoint.triggers.push({
                    0, 1, ngc::InputCondition::Active, 0.0,
                }),
                "required joint trigger fixture did not fit");
        require(core->tryPublish(ngc::ExecutionItem{triggeredJoint})
                    == ngc::PublishResult::Invalid,
                "executor accepted a required joint move with a missing trigger");

        triggeredJoint.triggerRequired = false;
        require(triggeredJoint.triggers.push({
                    0, 2, ngc::InputCondition::Active, 0.0,
                }),
                "duplicate joint trigger fixture did not fit");
        require(core->tryPublish(ngc::ExecutionItem{triggeredJoint})
                    == ngc::PublishResult::Invalid,
                "executor accepted duplicate triggers for one joint");

        auto scheduled =
            linearChunk(1, 1, 0, 1, 1, 0.0, 1.0, 1.0);
        require(scheduled.events.push({0, ngc::SpindleEvent{}}),
                "scheduled event fixture did not fit");
        require(core->tryPublish(scheduled) == ngc::PublishResult::Invalid,
                "initial production core silently accepted a hardware event");

        auto invalidCoefficient =
            linearChunk(1, 2, 0, 2, 3, 0.0, 1.0, 1.0);
        invalidCoefficient.normalMotion[0].coefficients[0].x =
            std::numeric_limits<double>::quiet_NaN();
        require(core->tryPublish(invalidCoefficient)
                    == ngc::PublishResult::Invalid,
                "executor accepted a non-finite polynomial coefficient");

        auto invalidInverse =
            linearChunk(1, 3, 0, 3, 5, 0.0, 1.0, 1.0);
        invalidInverse.normalMotion[0].inverseDuration = 2.0;
        require(core->tryPublish(invalidInverse)
                    == ngc::PublishResult::Invalid,
                "executor accepted inconsistent polynomial timing fields");

        auto sawPlanFull = false;
        for (std::size_t index = 0;
             index < ngc::ProductionExecutorCore::PLAN_CAPACITY + 1;
             ++index) {
            const auto chunk = linearChunk(
                1, index + 1, 0, index + 1,
                index * 2 + 1, 0.0, 1.0, 1.0);
            const auto result = core->tryPublish(chunk);
            if (result == ngc::PublishResult::Full) {
                sawPlanFull = true;
                break;
            }
        }
        require(sawPlanFull, "plan publication did not expose bounded capacity");

        auto sawControlFull = false;
        for (std::size_t index = 0;
             index < ngc::ProductionExecutorCore::CONTROL_CAPACITY + 1;
             ++index) {
            if (core->trySubmit(ngc::EnableRequest{index + 1})
                == ngc::SubmitResult::Full) {
                sawControlFull = true;
                break;
            }
        }
        require(sawControlFull,
                "control submission did not expose bounded capacity");
    }
}

int main() {
    static_assert(noexcept(
        std::declval<ngc::ProductionExecutorCore &>().servoTick()));

    try {
        testFixedTickExecutionAndAccounting();
        testContinuationMarkersAndTerminalStop();
        testMismatchedContinuationStopsSafely();
        testTriggeredMoveReachesTargetAtRest();
        testSampledTriggerGeneratesConstrainedStop();
        testPlanContinuesIntoTriggeredMove();
        testTriggeredJointsStopIndependentlyAndRetainState();
        testBoundedAndExplicitlyUnsupportedInputs();
    } catch (const std::exception &error) {
        std::cerr << "Production executor core test failure: "
                  << error.what() << '\n';

        return 1;
    }

    std::cout << "Production executor core tests passed\n";

    return 0;
}
