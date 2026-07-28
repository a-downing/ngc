#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
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

    void appendLinearSpan(ngc::PlanChunk &chunk, const ngc::SpanId span,
                          const double from, const double to,
                          const double duration) {
        require(chunk.normalMotion.push(
                    linearSpan(span, from, to, duration)),
                "additional normal span did not fit");
        chunk.stopTail[0] =
            linearSpan(span + 1, to, to, 0.25);
        chunk.branchState =
            ngc::executionSpanEnd(chunk.normalMotion[
                chunk.normalMotion.size - 1]);
        chunk.stopState = chunk.branchState;
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

    struct JointMoveRun {
        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot snapshot;
    };

    struct JogRun {
        std::vector<ngc::ExecutionEvent> events;
        std::vector<ngc::ExecutionSnapshot> samples;
        ngc::ExecutionSnapshot snapshot;
    };

    ngc::JogMotionLimits jogLimits() {
        return {
            .velocity = 0.5,
            .acceleration = 1.0,
            .jerk = 4.0,
        };
    }

    ngc::ProductionExecutorConfiguration controlledStopConfiguration() {
        ngc::ProductionExecutorConfiguration configuration;
        configuration.controlledStopLimits.velocity.x = 2.0;
        configuration.controlledStopLimits.acceleration.x = 2.0;
        configuration.controlledStopLimits.jerk.x = 8.0;

        return configuration;
    }

    ngc::ProductionExecutorConfiguration feedHoldConfiguration(
        const double acceleration = 1.0,
        const double jerk = 4.0) {
        ngc::ProductionExecutorConfiguration configuration;
        configuration.feedHold.tangentialAcceleration = acceleration;
        configuration.feedHold.tangentialJerk = jerk;
        configuration.feedHold.pathAcceleration = acceleration;
        configuration.feedHold.axisAcceleration.x = acceleration;

        return configuration;
    }

    JogRun runJogUntilHeld(ngc::ProductionExecutorCore &core,
                           const std::size_t maximumTicks = 4'000) {
        JogRun result;
        for (std::size_t tick = 0; tick < maximumTicks; ++tick) {
            core.servoTick();
            auto current = takeEvents(core);
            result.events.insert(
                result.events.end(), current.begin(), current.end());
            result.snapshot = latestSnapshot(core);
            result.samples.push_back(result.snapshot);
            if (result.snapshot.state == ngc::BackendState::Held) {
                return result;
            }
        }

        throw std::runtime_error("jog did not return to held");
    }

    JointMoveRun runResumedJointMove(
        ngc::ProductionExecutorCore &core,
        const ngc::TriggeredJointMove &move,
        const ngc::RequestId request,
        const std::optional<std::pair<ngc::DigitalInputId, double>>
            trigger = std::nullopt) {
        if (trigger) {
            core.setDigitalInputSample(trigger->first, false);
        }
        require(core.tryPublish(ngc::ExecutionItem{move})
                    == ngc::PublishResult::Published,
                "homing phase was not published");
        require(core.trySubmit(ngc::ResumeRequest{request, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "homing phase resume did not fit");

        JointMoveRun result;
        auto triggerApplied = false;
        for (auto tick = 0; tick < 2'000; ++tick) {
            if (trigger && !triggerApplied
                && result.snapshot.commandedJoints.position[0]
                    >= trigger->second) {
                core.setDigitalInputSample(trigger->first, true);
                triggerApplied = true;
            }
            core.servoTick();
            auto current = takeEvents(core);
            result.events.insert(
                result.events.end(), current.begin(), current.end());
            result.snapshot = latestSnapshot(core);
            if (result.snapshot.state == ngc::BackendState::Held) {
                break;
            }
        }

        require(result.snapshot.state == ngc::BackendState::Held,
                "homing phase did not return to held");
        const auto requests =
            selectEvents<ngc::RequestCompleted>(result.events);
        require(std::ranges::any_of(
                    requests, [request](const auto &completed) {
                        return completed.request == request
                            && completed.succeeded;
                    }),
                "homing phase resume was not acknowledged");

        return result;
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

    void testEnabledResetRetainsPoweredHeldState() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 10);

        require(core->trySubmit(ngc::ResetRequest{3, 11})
                    == ngc::SubmitResult::Submitted,
                "enabled reset did not fit");
        core->servoTick();
        const auto events = takeEvents(*core);
        const auto snapshot = latestSnapshot(*core);
        const auto requests =
            selectEvents<ngc::RequestCompleted>(events);
        require(requests.size() == 1 && requests[0].request == 3
                    && requests[0].succeeded,
                "enabled reset was not acknowledged");
        require(snapshot.state == ngc::BackendState::Held
                    && snapshot.epoch == 11,
                "epoch reset should retain the executor's enabled held state");
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

    void testScheduledSpindleEventsFollowExecutionCursor() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.25);
        initialize(*core, 21);

        auto first =
            linearChunk(21, 211, 0, 311, 411, 0.0, 1.0, 0.5);
        appendLinearSpan(first, 412, 1.0, 2.0, 0.5);
        auto second =
            linearChunk(21, 212, first.branch, 312, 414, 2.0, 3.0, 0.5);
        require(first.events.push({
                    0, ngc::SpindleEvent{
                        true, ngc::Direction::CW, 1'200.0,
                    },
                })
                    && first.events.push({
                        1, ngc::SpindleEvent{
                            true, ngc::Direction::CCW, 800.0,
                        },
                    })
                    && second.events.push({
                        0, ngc::SpindleEvent{},
                    }),
                "scheduled spindle fixtures did not fit");
        require(core->tryPublish(first)
                    == ngc::PublishResult::Published
                    && core->tryPublish(second)
                        == ngc::PublishResult::Published,
                "scheduled spindle chunks were not published");
        require(core->trySubmit(ngc::StartRequest{3, first.epoch})
                    == ngc::SubmitResult::Submitted,
                "scheduled spindle start did not fit");

        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        auto output = core->outputState().spindle;
        require(output.enabled && output.direction == ngc::Direction::CW
                    && output.speed == 1'200.0,
                "span-zero spindle event was not applied before motion");

        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        output = core->outputState().spindle;
        require(output.enabled && output.direction == ngc::Direction::CCW
                    && output.speed == 800.0,
                "next-span spindle event was not applied at its boundary");

        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        require(core->outputState().spindle.enabled,
                "future continuation event ran before its chunk");

        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        require(!core->outputState().spindle.enabled,
                "continuation spindle event was not applied at activation");
    }

    void testAbortSuppressesFutureScheduledEvents() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.25);
        initialize(*core, 22);

        auto chunk =
            linearChunk(22, 221, 0, 321, 421, 0.0, 1.0, 0.5);
        appendLinearSpan(chunk, 422, 1.0, 2.0, 0.5);
        require(chunk.events.push({
                    0, ngc::SpindleEvent{
                        true, ngc::Direction::CW, 1'000.0,
                    },
                })
                    && chunk.events.push({
                        1, ngc::SpindleEvent{
                            true, ngc::Direction::CCW, 500.0,
                        },
                    }),
                "abort spindle fixtures did not fit");
        require(core->tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "abort spindle chunk was not published");
        require(core->trySubmit(ngc::StartRequest{3, chunk.epoch})
                    == ngc::SubmitResult::Submitted,
                "abort spindle start did not fit");

        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        require(core->outputState().spindle.enabled,
                "abort fixture did not apply its initial spindle event");
        require(core->trySubmit(ngc::AbortRequest{4})
                    == ngc::SubmitResult::Submitted,
                "scheduled-event abort did not fit");

        core->servoTick();
        const auto events = takeEvents(*core);
        const auto snapshot = latestSnapshot(*core);
        require(snapshot.state == ngc::BackendState::Held,
                "scheduled-event abort did not hold the executor");
        require(!core->outputState().spindle.enabled,
                "abort did not establish a safe spindle output");
        require(std::ranges::any_of(
                    selectEvents<ngc::RequestCompleted>(events),
                    [](const auto &completed) {
                        return completed.request == 4
                            && completed.succeeded;
                    }),
                "scheduled-event abort was not acknowledged");

        require(core->trySubmit(ngc::DisableRequest{5})
                    == ngc::SubmitResult::Submitted,
                "scheduled-event disable did not fit");
        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        require(!core->outputState().spindle.enabled,
                "disable did not establish a safe spindle output");
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
                    0, firstInput, ngc::InputCondition::Active,
                }) && move.triggers.push({
                    1, secondInput, ngc::InputCondition::RisingEdge,
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

        core->setDigitalInputSample(secondInput, true);
        std::vector<ngc::ExecutionEvent> events;
        core->servoTick();
        auto firstEvents = takeEvents(*core);
        events.insert(events.end(), firstEvents.begin(), firstEvents.end());
        auto afterSecondTrigger = latestSnapshot(*core);
        core->setDigitalInputSample(firstInput, true);
        ngc::ExecutionSnapshot completed;
        auto previousAcceleration = std::array{
            afterSecondTrigger.commandedJoints.acceleration[0],
            afterSecondTrigger.commandedJoints.acceleration[1],
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
                "first joint did not continue independently after the second joint triggered");
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

    void testHomingControlSequence() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 80);

        ngc::JointVector initial;
        initial[0] = -0.1;
        initial[1] = 0.25;
        constexpr auto homingJoint = ngc::JointMask{1};
        require(core->trySubmit(
                    ngc::SetJointPositionRequest{
                        10, homingJoint | ngc::JointMask{1} << 1, initial,
                    }) == ngc::SubmitResult::Submitted,
                "initial homing coordinates did not fit");
        core->servoTick();
        auto events = takeEvents(*core);
        auto snapshot = latestSnapshot(*core);
        const auto coordinateRequests =
            selectEvents<ngc::RequestCompleted>(events);
        require(coordinateRequests.size() == 1
                    && coordinateRequests[0].request == 10
                    && coordinateRequests[0].succeeded,
                "stationary joint coordinate assignment was rejected");
        requireNear(snapshot.commandedJoints.position[0], initial[0],
                    "first joint coordinate was not assigned");
        requireNear(snapshot.commandedJoints.position[1], initial[1],
                    "second joint coordinate was not assigned");

        constexpr ngc::DigitalInputId input = 20;
        auto fast = triggeredJointMove(80, 801, 0, 901, 1'001);
        fast.joints = homingJoint;
        fast.targetMode = ngc::JointTargetMode::Relative;
        fast.target[0] = 0.5;
        fast.triggerRequired = true;
        require(fast.triggers.push({
                    0, input, ngc::InputCondition::Active,
                }),
                "fast-search trigger did not fit");
        const auto fastRun =
            runResumedJointMove(*core, fast, 11, {{input, 0.05}});
        const auto fastCompleted =
            selectEvents<ngc::TriggeredJointMoveCompleted>(
                fastRun.events);
        require(fastCompleted.size() == 1
                    && fastCompleted[0].status
                        == ngc::TriggeredMoveStatus::Triggered,
                "fast homing search did not stop on its input");

        auto backoff = triggeredJointMove(
            80, 802, fast.branch, 902, 1'002);
        backoff.joints = homingJoint;
        backoff.targetMode = ngc::JointTargetMode::Absolute;
        backoff.target[0] =
            fastCompleted[0].triggerState.position[0] - 0.05;
        const auto backoffRun =
            runResumedJointMove(*core, backoff, 12);
        const auto backoffCompleted =
            selectEvents<ngc::TriggeredJointMoveCompleted>(
                backoffRun.events);
        require(backoffCompleted.size() == 1
                    && backoffCompleted[0].status
                        == ngc::TriggeredMoveStatus::ReachedTarget,
                "fixed homing backoff did not reach its target");

        auto slow = triggeredJointMove(
            80, 803, backoff.branch, 903, 1'003);
        slow.joints = homingJoint;
        slow.targetMode = ngc::JointTargetMode::Relative;
        slow.target[0] = 0.15;
        slow.limits.velocity[0] = 0.1;
        slow.triggerRequired = true;
        require(slow.triggers.push({
                    0, input, ngc::InputCondition::Active,
                }),
                "slow-latch trigger did not fit");
        const auto slowTrigger =
            backoffCompleted[0].stoppedState.position[0] + 0.04;
        const auto slowRun =
            runResumedJointMove(*core, slow, 13, {{input, slowTrigger}});
        const auto slowCompleted =
            selectEvents<ngc::TriggeredJointMoveCompleted>(
                slowRun.events);
        require(slowCompleted.size() == 1
                    && slowCompleted[0].status
                        == ngc::TriggeredMoveStatus::Triggered,
                "slow homing latch did not stop on its input");

        auto calibrated = slowCompleted[0].stoppedState.position;
        calibrated[0] +=
            -0.25 - slowCompleted[0].triggerState.position[0];
        require(core->trySubmit(
                    ngc::SetJointPositionRequest{
                        14, homingJoint, calibrated,
                    }) == ngc::SubmitResult::Submitted,
                "latched coordinate assignment did not fit");
        core->servoTick();
        events = takeEvents(*core);
        snapshot = latestSnapshot(*core);
        const auto calibratedRequests =
            selectEvents<ngc::RequestCompleted>(events);
        require(calibratedRequests.size() == 1
                    && calibratedRequests[0].request == 14
                    && calibratedRequests[0].succeeded,
                "latched coordinate assignment was rejected");
        requireNear(snapshot.commandedJoints.position[0],
                    calibrated[0],
                    "latched coordinate assignment used the wrong position");
        requireNear(snapshot.commandedJoints.position[1], initial[1],
                    "masked coordinate assignment changed another joint");

        auto finalMove = triggeredJointMove(
            80, 804, slow.branch, 904, 1'004);
        finalMove.joints = homingJoint;
        finalMove.targetMode = ngc::JointTargetMode::Absolute;
        finalMove.target[0] = 0.1;
        const auto finalRun =
            runResumedJointMove(*core, finalMove, 15);
        const auto finalCompleted =
            selectEvents<ngc::TriggeredJointMoveCompleted>(
                finalRun.events);
        require(finalCompleted.size() == 1
                    && finalCompleted[0].status
                        == ngc::TriggeredMoveStatus::ReachedTarget,
                "final homing move did not reach its target");
        requireNear(finalRun.snapshot.commandedJoints.position[0],
                    finalMove.target[0],
                    "final homing move stopped at the wrong coordinate");
        requireNear(finalRun.snapshot.commandedJoints.position[1],
                    initial[1],
                    "homing sequence changed an unselected joint");
    }

    void testControlledStopAbortsTriggeredJointMove() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 90);

        auto move = triggeredJointMove(90, 901, 0, 1'001, 1'101);
        require(core->tryPublish(ngc::ExecutionItem{move})
                    == ngc::PublishResult::Published,
                "controlled-stop fixture was not published");
        require(core->trySubmit(ngc::ResumeRequest{10, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "controlled-stop fixture resume did not fit");

        ngc::ExecutionSnapshot moving;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            moving = latestSnapshot(*core);
            if (moving.commandedJoints.position[0] >= 0.05) {
                break;
            }
        }
        require(moving.state == ngc::BackendState::Running
                    && moving.commandedJoints.velocity[0] > 0.0,
                "controlled-stop fixture did not establish motion");

        require(core->trySubmit(
                    ngc::SetJointPositionRequest{
                        11, ngc::JointMask{1}, {},
                    }) == ngc::SubmitResult::Submitted,
                "moving coordinate-assignment rejection did not fit");
        core->servoTick();
        auto events = takeEvents(*core);
        auto snapshot = latestSnapshot(*core);
        const auto rejectedAssignments =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    rejectedAssignments, [](const auto &completed) {
                        return completed.request == 11
                            && !completed.succeeded;
                    }),
                "moving joint coordinate assignment was not rejected");

        require(core->trySubmit(ngc::ControlledStopRequest{12})
                    == ngc::SubmitResult::Submitted,
                "controlled stop did not fit");
        auto sawHolding = false;
        auto previousAcceleration =
            snapshot.commandedJoints.acceleration[0];
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            snapshot = latestSnapshot(*core);
            sawHolding = sawHolding
                || snapshot.state == ngc::BackendState::Holding;
            require(std::abs(
                        snapshot.commandedJoints.acceleration[0]
                        - previousAcceleration)
                        <= move.limits.jerk[0]
                            * core->servoPeriod() + 1e-9,
                    "controlled joint stop exceeded its per-tick jerk limit");
            previousAcceleration =
                snapshot.commandedJoints.acceleration[0];
            if (snapshot.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto requests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    requests, [](const auto &completed) {
                        return completed.request == 12
                            && completed.succeeded;
                    }),
                "controlled joint stop was not acknowledged");
        const auto completions =
            selectEvents<ngc::TriggeredJointMoveCompleted>(events);
        require(completions.size() == 1
                    && completions[0].status
                        == ngc::TriggeredMoveStatus::Aborted
                    && completions[0].triggeredJoints == 0,
                "controlled joint stop did not abort without fabricating triggers");
        const auto held = selectEvents<ngc::BackendHeld>(events);
        require(sawHolding
                    && std::ranges::any_of(
                        held, [](const auto &event) {
                            return event.reason
                                == ngc::BackendHoldReason::ControlledStop;
                        }),
                "controlled joint stop did not report its holding lifecycle");
        require(snapshot.state == ngc::BackendState::Held
                    && snapshot.commandedJoints.position[0]
                        > moving.commandedJoints.position[0]
                    && snapshot.commandedJoints.position[0]
                        < move.target[0],
                "controlled joint stop did not brake before the target");
        requireNear(snapshot.commandedJoints.velocity[0], 0.0,
                    "controlled joint stop retained velocity");
        requireNear(snapshot.commandedJoints.acceleration[0], 0.0,
                    "controlled joint stop retained acceleration");
    }

    void testControlledStopAbortsAxisTriggeredMove() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 91);

        const auto move =
            triggeredMove(91, 911, 0, 1'011, 1'111, 1.0);
        require(core->tryPublish(ngc::ExecutionItem{move})
                    == ngc::PublishResult::Published,
                "axis controlled-stop fixture was not published");
        require(core->trySubmit(ngc::ResumeRequest{10, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "axis controlled-stop fixture resume did not fit");

        ngc::ExecutionSnapshot moving;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            moving = latestSnapshot(*core);
            if (moving.commanded.position.x >= 0.05) {
                break;
            }
        }
        require(moving.state == ngc::BackendState::Running
                    && moving.commanded.velocity.x > 0.0,
                "axis controlled-stop fixture did not establish motion");
        require(core->trySubmit(ngc::ControlledStopRequest{11})
                    == ngc::SubmitResult::Submitted,
                "axis controlled stop did not fit");

        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot snapshot;
        auto sawHolding = false;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            snapshot = latestSnapshot(*core);
            sawHolding = sawHolding
                || snapshot.state == ngc::BackendState::Holding;
            if (snapshot.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto completions =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(completions.size() == 1
                    && completions[0].status
                        == ngc::TriggeredMoveStatus::Aborted,
                "axis controlled stop did not abort the triggered move");
        const auto held = selectEvents<ngc::BackendHeld>(events);
        require(sawHolding
                    && std::ranges::any_of(
                        held, [](const auto &event) {
                            return event.reason
                                == ngc::BackendHoldReason::ControlledStop;
                        }),
                "axis controlled stop did not report its holding lifecycle");
        require(snapshot.commanded.position.x
                    > moving.commanded.position.x
                    && snapshot.commanded.position.x < move.target.x,
                "axis controlled stop did not brake before the target");
        requireNear(snapshot.commanded.velocity.x, 0.0,
                    "axis controlled stop retained velocity");
        requireNear(snapshot.commanded.acceleration.x, 0.0,
                    "axis controlled stop retained acceleration");
    }

    void testControlledStopCancelsOrdinaryPlan() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(
            0.01, controlledStopConfiguration());
        initialize(*core, 92);

        auto active =
            linearChunk(92, 921, 0, 1'021, 1'121, 0.0, 1.0, 1.0);
        auto queued =
            linearChunk(92, 922, active.branch, 1'022, 1'122, 1.0, 2.0, 1.0);
        require(active.markers.push({1'201, 0, 0.1})
                    && active.markers.push({1'202, 0, 0.9})
                    && queued.markers.push({1'203, 0, 0.0})
                    && active.events.push({
                        0, ngc::SpindleEvent{
                            true, ngc::Direction::CW, 1'000.0,
                        },
                    })
                    && queued.events.push({
                        0, ngc::SpindleEvent{},
                    }),
                "controlled-stop markers did not fit");
        require(core->tryPublish(active)
                    == ngc::PublishResult::Published
                    && core->tryPublish(queued)
                        == ngc::PublishResult::Published,
                "controlled-stop plan horizon was not published");
        require(core->trySubmit(ngc::StartRequest{10, active.epoch})
                    == ngc::SubmitResult::Submitted,
                "controlled-stop plan start did not fit");

        std::vector<ngc::ExecutionEvent> events;
        ngc::ExecutionSnapshot moving;
        for (auto tick = 0; tick < 25; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            moving = latestSnapshot(*core);
        }
        require(moving.state == ngc::BackendState::Running
                    && moving.commanded.position.x > 0.2
                    && moving.commanded.velocity.x > 0.0,
                "ordinary controlled-stop fixture did not establish motion");
        require(core->outputState().spindle.enabled,
                "active controlled-stop spindle event was not applied");
        require(core->trySubmit(ngc::ControlledStopRequest{11})
                    == ngc::SubmitResult::Submitted,
                "ordinary controlled stop did not fit");

        auto previousAcceleration = moving.commanded.acceleration.x;
        auto sawHolding = false;
        ngc::ExecutionSnapshot stopped;
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            stopped = latestSnapshot(*core);
            sawHolding = sawHolding
                || stopped.state == ngc::BackendState::Holding;
            require(std::abs(stopped.commanded.acceleration.x)
                        <= 2.0 + 1e-9,
                    "ordinary controlled stop exceeded its acceleration limit");
            require(std::abs(
                        stopped.commanded.acceleration.x
                        - previousAcceleration)
                        <= 8.0 * core->servoPeriod() + 1e-8,
                    "ordinary controlled stop exceeded its per-tick jerk limit");
            previousAcceleration = stopped.commanded.acceleration.x;
            if (stopped.state == ngc::BackendState::Held) {
                break;
            }
        }

        require(sawHolding && stopped.state == ngc::BackendState::Held,
                "ordinary controlled stop did not complete its holding lifecycle");
        require(stopped.commanded.position.x
                    > moving.commanded.position.x
                    && stopped.commanded.position.x < 1.0,
                "ordinary controlled stop did not brake before the plan endpoint");
        requireNear(stopped.commanded.velocity.x, 0.0,
                    "ordinary controlled stop retained velocity");
        requireNear(stopped.commanded.acceleration.x, 0.0,
                    "ordinary controlled stop retained acceleration");
        require(stopped.queuedExecutionItems == 0,
                "ordinary controlled stop retained its queued horizon");

        const auto requests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    requests, [](const auto &completed) {
                        return completed.request == 11
                            && completed.succeeded;
                    }),
                "ordinary controlled stop was not acknowledged");
        const auto retired = selectEvents<ngc::ChunkRetired>(events);
        require(retired.size() == 2
                    && retired[0].chunk == active.id
                    && retired[1].chunk == queued.id,
                "ordinary controlled stop did not retire its horizon in order");
        const auto held = selectEvents<ngc::BackendHeld>(events);
        require(held.size() == 1
                    && held[0].reason
                        == ngc::BackendHoldReason::ControlledStop,
                "ordinary controlled stop did not report its terminal reason");
        const auto markers =
            selectEvents<ngc::ExecutionMarkerReached>(events);
        require(markers.size() == 1 && markers[0].marker == 1'201,
                "ordinary controlled stop emitted a marker beyond its cursor");
        require(core->outputState().spindle.enabled,
                "ordinary controlled stop applied an abandoned spindle event");

        require(core->tryPublish(
                    linearChunk(92, 923, 0, 1'023, 1'123, 0.0, 1.0, 1.0))
                    == ngc::PublishResult::Published,
                "same-epoch resume rejection fixture was not published");
        require(core->trySubmit(ngc::ResumeRequest{12, 92})
                    == ngc::SubmitResult::Submitted,
                "same-epoch resume rejection did not fit");
        core->servoTick();
        const auto resumeEvents = takeEvents(*core);
        latestSnapshot(*core);
        require(std::ranges::any_of(
                    selectEvents<ngc::RequestCompleted>(resumeEvents),
                    [](const auto &completed) {
                        return completed.request == 12
                            && !completed.succeeded;
                    }),
                "ordinary controlled stop allowed its abandoned epoch to resume");
    }

    void testFeedHoldPreservesAndResumesAxisTriggeredMove() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 93);

        const auto move =
            triggeredMove(93, 930, 0, 931, 932, 1.0);
        require(core->tryPublish(move)
                    == ngc::PublishResult::Published,
                "triggered feed-hold move was not published");
        require(core->trySubmit(ngc::StartRequest{3, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "triggered feed-hold move start did not fit");

        auto moving = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            moving = latestSnapshot(*core);
            if (moving.commanded.position.x >= 0.1) {
                break;
            }
        }
        require(moving.state == ngc::BackendState::Running
                    && moving.commanded.velocity.x > 0.0,
                "triggered feed-hold fixture did not establish motion");
        require(core->trySubmit(ngc::FeedHoldRequest{4})
                    == ngc::SubmitResult::Submitted,
                "triggered feed hold did not fit");

        core->servoTick();
        auto events = takeEvents(*core);
        auto previous = latestSnapshot(*core);
        require(previous.state == ngc::BackendState::Holding,
                "triggered feed hold did not expose its braking state");
        require(core->trySubmit(ngc::FeedHoldRequest{5})
                    == ngc::SubmitResult::Submitted,
                "duplicate triggered feed hold did not fit");

        auto held = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto currentEvents = takeEvents(*core);
            events.insert(
                events.end(), currentEvents.begin(), currentEvents.end());
            const auto current = latestSnapshot(*core);
            require(current.commanded.position.x
                        + 1e-12 >= previous.commanded.position.x,
                    "triggered feed hold moved backward");
            require(std::abs(current.commanded.velocity.x)
                        <= move.limits.velocity.x + 1e-9,
                    "triggered feed hold exceeded its velocity limit");
            require(std::abs(current.commanded.acceleration.x)
                        <= move.limits.acceleration.x + 1e-9,
                    "triggered feed hold exceeded its acceleration limit");
            require(std::abs(
                        current.commanded.acceleration.x
                        - previous.commanded.acceleration.x)
                        <= move.limits.jerk.x * core->servoPeriod() + 1e-9,
                    "triggered feed hold exceeded its per-tick jerk limit");
            previous = current;
            if (current.state == ngc::BackendState::Held) {
                held = current;
                break;
            }
        }

        require(held.state == ngc::BackendState::Held
                    && held.commanded.position.x > moving.commanded.position.x
                    && held.commanded.position.x < move.target.x,
                "triggered feed hold did not retain an interior approach");
        requireNear(held.commanded.velocity.x, 0.0,
                    "triggered feed hold retained velocity");
        requireNear(held.commanded.acceleration.x, 0.0,
                    "triggered feed hold retained acceleration");
        require(selectEvents<ngc::TriggeredMoveCompleted>(events).empty()
                    && selectEvents<ngc::BranchSelected>(events).empty()
                    && selectEvents<ngc::ChunkRetired>(events).empty(),
                "triggered feed hold completed or retired the active move");
        require(std::ranges::any_of(
                    selectEvents<ngc::BackendHeld>(events),
                    [](const auto &event) {
                        return event.reason
                            == ngc::BackendHoldReason::FeedHold;
                    }),
                "triggered feed hold did not report its hold reason");
        const auto holdRequests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    holdRequests, [](const auto &completed) {
                        return completed.request == 4
                            && completed.succeeded;
                    })
                    && std::ranges::any_of(
                        holdRequests, [](const auto &completed) {
                            return completed.request == 5
                                && !completed.succeeded;
                        }),
                "triggered feed hold request results were incorrect");

        for (auto tick = 0; tick < 5; ++tick) {
            core->servoTick();
            takeEvents(*core);
            const auto stationary = latestSnapshot(*core);
            requireNear(
                stationary.commanded.position.x,
                held.commanded.position.x,
                "held triggered move advanced without Resume");
        }

        require(core->trySubmit(ngc::ResumeRequest{6, move.epoch + 1})
                    == ngc::SubmitResult::Submitted,
                "stale triggered Resume did not fit");
        core->servoTick();
        const auto staleEvents = takeEvents(*core);
        latestSnapshot(*core);
        require(std::ranges::any_of(
                    selectEvents<ngc::RequestCompleted>(staleEvents),
                    [](const auto &completed) {
                        return completed.request == 6
                            && !completed.succeeded;
                    }),
                "triggered feed hold accepted a stale Resume");

        require(core->trySubmit(ngc::ResumeRequest{7, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "triggered feed-hold Resume did not fit");
        events.clear();
        auto completed = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto currentEvents = takeEvents(*core);
            events.insert(
                events.end(), currentEvents.begin(), currentEvents.end());
            completed = latestSnapshot(*core);
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto completions =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(completions.size() == 1
                    && completions[0].status
                        == ngc::TriggeredMoveStatus::ReachedTarget,
                "resumed triggered move did not reach its target");
        requireNear(completed.commanded.position.x, move.target.x,
                    "resumed triggered move lost its original target");
        require(std::ranges::any_of(
                    selectEvents<ngc::RequestCompleted>(events),
                    [](const auto &request) {
                        return request.request == 7
                            && request.succeeded;
                    }),
                "triggered feed-hold Resume was not acknowledged");
    }

    void testAxisTriggerDuringFeedHoldBrakingCompletesNormally() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 94);

        constexpr ngc::DigitalInputId input = 8;
        const auto move = triggeredMove(
            94, 940, 0, 941, 942, 1.0, input,
            ngc::InputCondition::RisingEdge);
        core->setDigitalInputSample(input, false);
        require(core->tryPublish(move)
                    == ngc::PublishResult::Published,
                "feed-hold trigger fixture was not published");
        require(core->trySubmit(ngc::StartRequest{3, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "feed-hold trigger fixture start did not fit");

        auto moving = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            moving = latestSnapshot(*core);
            if (moving.commanded.position.x >= 0.1) {
                break;
            }
        }
        require(moving.commanded.velocity.x > 0.0,
                "feed-hold trigger fixture did not establish motion");
        require(core->trySubmit(ngc::FeedHoldRequest{4})
                    == ngc::SubmitResult::Submitted,
                "feed-hold trigger request did not fit");
        core->servoTick();
        auto events = takeEvents(*core);
        const auto braking = latestSnapshot(*core);
        require(braking.state == ngc::BackendState::Holding
                    && braking.commanded.velocity.x > 0.0,
                "feed-hold trigger fixture did not begin braking");

        core->setDigitalInputSample(input, true);
        const auto triggerPosition = braking.commanded.position.x;
        auto completed = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            auto currentEvents = takeEvents(*core);
            events.insert(
                events.end(), currentEvents.begin(), currentEvents.end());
            completed = latestSnapshot(*core);
            if (completed.state == ngc::BackendState::Held) {
                break;
            }
        }

        const auto completions =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(completions.size() == 1
                    && completions[0].status
                        == ngc::TriggeredMoveStatus::Triggered,
                "input sampled during feed-hold braking did not trigger");
        requireNear(completions[0].triggerState.position.x,
                    triggerPosition,
                    "feed-hold braking did not latch the sampled trigger state");
        require(completions[0].stoppedState.position.x
                    > completions[0].triggerState.position.x
                    && completions[0].stoppedState.position.x
                        < move.target.x,
                "trigger during feed-hold braking lost its constrained stop");
        require(std::ranges::none_of(
                    selectEvents<ngc::BackendHeld>(events),
                    [](const auto &event) {
                        return event.reason
                            == ngc::BackendHoldReason::FeedHold;
                    }),
                "trigger during braking entered a transient feed-held state");
        require(std::ranges::any_of(
                    selectEvents<ngc::BackendHeld>(events),
                    [](const auto &event) {
                        return event.reason
                            == ngc::BackendHoldReason::StopBranch;
                    }),
                "trigger during braking did not complete at the stop branch");
    }

    void testControlledStopAbortsFeedHeldAxisTriggeredMove() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 95);

        const auto move =
            triggeredMove(95, 950, 0, 951, 952, 1.0);
        require(core->tryPublish(move)
                    == ngc::PublishResult::Published,
                "feed-held controlled-stop fixture was not published");
        require(core->trySubmit(ngc::StartRequest{3, move.epoch})
                    == ngc::SubmitResult::Submitted,
                "feed-held controlled-stop fixture start did not fit");

        auto moving = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            moving = latestSnapshot(*core);
            if (moving.commanded.position.x >= 0.1) {
                break;
            }
        }
        require(moving.commanded.velocity.x > 0.0,
                "feed-held controlled-stop fixture did not establish motion");
        require(core->trySubmit(ngc::FeedHoldRequest{4})
                    == ngc::SubmitResult::Submitted,
                "feed-held controlled-stop hold did not fit");

        auto held = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 1'000; ++tick) {
            core->servoTick();
            takeEvents(*core);
            held = latestSnapshot(*core);
            if (held.state == ngc::BackendState::Held) {
                break;
            }
        }
        require(held.state == ngc::BackendState::Held
                    && held.commanded.position.x < move.target.x,
                "controlled-stop fixture did not reach a feed-held state");

        require(core->trySubmit(ngc::ControlledStopRequest{5})
                    == ngc::SubmitResult::Submitted,
                "feed-held controlled stop did not fit");
        core->servoTick();
        const auto events = takeEvents(*core);
        const auto stopped = latestSnapshot(*core);

        const auto completions =
            selectEvents<ngc::TriggeredMoveCompleted>(events);
        require(completions.size() == 1
                    && completions[0].status
                        == ngc::TriggeredMoveStatus::Aborted,
                "controlled stop did not abort the feed-held move");
        requireNear(stopped.commanded.position.x,
                    held.commanded.position.x,
                    "controlled stop moved the feed-held position");
        require(std::ranges::any_of(
                    selectEvents<ngc::BackendHeld>(events),
                    [](const auto &event) {
                        return event.reason
                            == ngc::BackendHoldReason::ControlledStop;
                    }),
                "feed-held controlled stop did not report its hold reason");
        require(std::ranges::any_of(
                    selectEvents<ngc::RequestCompleted>(events),
                    [](const auto &completed) {
                        return completed.request == 5
                            && completed.succeeded;
                    }),
                "feed-held controlled stop was not acknowledged");
    }

    void testFeedHoldPreservesAndResumesOrdinaryPlan() {
        constexpr auto servoPeriod = 0.01;
        constexpr auto accelerationLimit = 1.0;
        constexpr auto jerkLimit = 4.0;
        auto core = std::make_unique<ngc::ProductionExecutorCore>(
            servoPeriod,
            feedHoldConfiguration(accelerationLimit, jerkLimit));
        initialize(*core, 93);

        auto chunk =
            linearChunk(93, 930, 0, 931, 932, 0.0, 10.0, 10.0);
        appendLinearSpan(chunk, 933, 10.0, 11.0, 1.0);
        require(chunk.markers.push({933, 0, 0.05})
                    && chunk.markers.push({934, 0, 0.2})
                    && chunk.events.push({
                        0, ngc::SpindleEvent{
                            true, ngc::Direction::CW, 1'500.0,
                        },
                    })
                    && chunk.events.push({
                        1, ngc::SpindleEvent{},
                    }),
                "feed-hold marker fixtures did not fit");
        require(core->tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "feed-hold plan was not published");
        require(core->trySubmit(ngc::StartRequest{3, 93})
                    == ngc::SubmitResult::Submitted,
                "feed-hold plan start did not fit");

        auto previous = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 20; ++tick) {
            core->servoTick();
            takeEvents(*core);
            previous = latestSnapshot(*core);
        }
        require(core->trySubmit(ngc::FeedHoldRequest{4})
                    == ngc::SubmitResult::Submitted,
                "feed hold did not fit");
        require(core->outputState().spindle.enabled,
                "feed-hold fixture did not apply its initial spindle event");

        std::vector<ngc::ExecutionEvent> events;
        auto sawHolding = false;
        auto held = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 500; ++tick) {
            core->servoTick();
            auto currentEvents = takeEvents(*core);
            events.insert(
                events.end(), currentEvents.begin(), currentEvents.end());
            const auto current = latestSnapshot(*core);
            sawHolding = sawHolding
                || current.state == ngc::BackendState::Holding;
            require(current.commanded.position.x
                        + 1e-12 >= previous.commanded.position.x,
                    "feed hold moved backward on the retained path");
            require(current.commanded.velocity.x >= -1e-12
                        && current.commanded.velocity.x <= 1.0 + 1e-9,
                    "feed hold produced an invalid path velocity");
            require(std::abs(current.commanded.acceleration.x)
                        <= accelerationLimit * 1.01 + 1e-9,
                    "feed hold exceeded its acceleration limit");
            previous = current;
            if (current.state == ngc::BackendState::Held) {
                held = current;
                break;
            }
        }

        require(sawHolding,
                "feed hold did not expose its braking state");
        require(held.state == ngc::BackendState::Held,
                "feed hold did not reach a stationary held state");
        requireNear(held.executionRate, 0.0,
                    "feed hold retained a nonzero execution rate");
        requireNear(held.commanded.velocity.x, 0.0,
                    "feed hold retained commanded velocity");
        requireNear(held.commanded.acceleration.x, 0.0,
                    "feed hold retained commanded acceleration");
        require(held.spanProgress > 0.02 && held.spanProgress < 0.2,
                "feed hold did not preserve an interior plan cursor");
        require(core->outputState().spindle.enabled,
                "feed hold applied a future spindle event");

        const auto holdRequests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    holdRequests, [](const auto &completed) {
                        return completed.request == 4
                            && completed.succeeded;
                    }),
                "feed hold was not acknowledged");
        const auto heldEvents =
            selectEvents<ngc::BackendHeld>(events);
        require(std::ranges::any_of(
                    heldEvents, [](const auto &event) {
                        return event.reason
                            == ngc::BackendHoldReason::FeedHold;
                    }),
                "feed hold did not report its hold reason");
        const auto markersBeforeResume =
            selectEvents<ngc::ExecutionMarkerReached>(events);
        require(std::ranges::none_of(
                    markersBeforeResume, [](const auto &marker) {
                        return marker.marker == 934;
                    }),
                "feed hold emitted a marker beyond its held cursor");

        for (auto tick = 0; tick < 5; ++tick) {
            core->servoTick();
            takeEvents(*core);
            const auto stationary = latestSnapshot(*core);
            requireNear(
                stationary.commanded.position.x,
                held.commanded.position.x,
                "held plan cursor advanced without Resume");
            requireNear(
                stationary.spanProgress, held.spanProgress,
                "held span progress advanced without Resume");
            require(core->outputState().spindle.enabled,
                    "held plan advanced its scheduled-event cursor");
        }

        require(core->trySubmit(ngc::ResumeRequest{5, 93})
                    == ngc::SubmitResult::Submitted,
                "feed resume did not fit");
        auto resumedRate = false;
        auto sawFutureMarker = false;
        auto markerCounts = std::array<int, 2>{};
        for (const auto &marker : markersBeforeResume) {
            if (marker.marker == 933) {
                ++markerCounts[0];
            }
        }
        previous = held;
        for (auto tick = 0; tick < 1'500; ++tick) {
            core->servoTick();
            auto currentEvents = takeEvents(*core);
            const auto currentMarkers =
                selectEvents<ngc::ExecutionMarkerReached>(
                    currentEvents);
            for (const auto &marker : currentMarkers) {
                if (marker.marker == 933) {
                    ++markerCounts[0];
                } else if (marker.marker == 934) {
                    ++markerCounts[1];
                    sawFutureMarker = true;
                }
            }
            events.insert(
                events.end(), currentEvents.begin(), currentEvents.end());
            const auto current = latestSnapshot(*core);
            require(current.commanded.position.x
                        + 1e-12 >= previous.commanded.position.x,
                    "feed resume moved backward on the retained path");
            require(std::abs(current.commanded.acceleration.x)
                        <= accelerationLimit * 1.01 + 1e-9,
                    "feed resume exceeded its acceleration limit");
            resumedRate = resumedRate
                || current.executionRate >= 1.0 - 1e-10;
            previous = current;
            if (resumedRate && sawFutureMarker
                && !core->outputState().spindle.enabled) {
                break;
            }
        }

        require(resumedRate,
                "feed resume did not restore the full execution rate");
        require(sawFutureMarker,
                "feed resume did not continue through a future marker");
        require(!core->outputState().spindle.enabled,
                "feed resume did not apply the future spindle event");
        require(markerCounts[0] == 1 && markerCounts[1] == 1,
                "feed hold/resume repeated or lost an execution marker");
        const auto resumeRequests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    resumeRequests, [](const auto &completed) {
                        return completed.request == 5
                            && completed.succeeded;
                    }),
                "feed resume was not acknowledged");
    }

    void testFeedRetimingFaultsAtStopBranch() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(
            0.01, feedHoldConfiguration(0.2, 0.4));
        initialize(*core, 94);

        auto chunk =
            linearChunk(94, 940, 0, 941, 942, 0.0, 0.2, 0.2);
        require(chunk.events.push({
                    0, ngc::SpindleEvent{
                        true, ngc::Direction::CW, 900.0,
                    },
                }),
                "feed-retiming fault spindle fixture did not fit");
        require(core->tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "short feed-hold plan was not published");
        require(core->trySubmit(ngc::StartRequest{3, 94})
                    == ngc::SubmitResult::Submitted,
                "short feed-hold plan start did not fit");
        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);
        require(core->trySubmit(ngc::FeedHoldRequest{4})
                    == ngc::SubmitResult::Submitted,
                "short-plan feed hold did not fit");

        std::vector<ngc::ExecutionEvent> events;
        auto snapshot = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 100; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            snapshot = latestSnapshot(*core);
            if (snapshot.state == ngc::BackendState::Faulted) {
                break;
            }
        }

        require(snapshot.state == ngc::BackendState::Faulted,
                "feed retiming entered a stop branch");
        const auto faults = selectEvents<ngc::BackendFault>(events);
        require(faults.size() == 1 && faults[0].code == 5,
                "stop-branch feed-retiming fault was not reported");
        require(std::ranges::none_of(
                    selectEvents<ngc::BranchSelected>(events),
                    [](const auto &branch) {
                        return branch.choice == ngc::BranchChoice::Stop;
                    }),
                "feed retiming selected a stop branch before faulting");
        require(!core->outputState().spindle.enabled,
                "feed-retiming fault did not establish a safe spindle output");
    }

    void testFeedHoldContinuesAcrossDependentChunks() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(
            0.01, feedHoldConfiguration());
        initialize(*core, 95);

        const auto first =
            linearChunk(95, 950, 0, 951, 952, 0.0, 0.2, 0.2);
        const auto second =
            linearChunk(95, 953, 951, 954, 955, 0.2, 2.2, 2.0);
        require(core->tryPublish(first)
                    == ngc::PublishResult::Published
                    && core->tryPublish(second)
                        == ngc::PublishResult::Published,
                "dependent feed-hold chunks were not published");
        require(core->trySubmit(ngc::StartRequest{3, 95})
                    == ngc::SubmitResult::Submitted,
                "dependent feed-hold plan start did not fit");
        for (auto tick = 0; tick < 5; ++tick) {
            core->servoTick();
            takeEvents(*core);
            latestSnapshot(*core);
        }
        require(core->trySubmit(ngc::FeedHoldRequest{4})
                    == ngc::SubmitResult::Submitted,
                "dependent-plan feed hold did not fit");

        std::vector<ngc::ExecutionEvent> events;
        auto snapshot = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 500; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            snapshot = latestSnapshot(*core);
            if (snapshot.state == ngc::BackendState::Held
                || snapshot.state == ngc::BackendState::Faulted) {
                break;
            }
        }

        require(snapshot.state == ngc::BackendState::Held,
                "feed hold failed while crossing a dependent chunk");
        require(snapshot.activeChunk == second.id,
                "feed hold did not retain the continuation chunk");
        require(snapshot.commanded.position.x > first.branchState.position.x,
                "feed hold did not advance across the chunk boundary");
        const auto firstBranch = first.branch;
        const auto secondId = second.id;
        require(std::ranges::any_of(
                    selectEvents<ngc::BranchSelected>(events),
                    [firstBranch, secondId](const auto &branch) {
                        return branch.branch == firstBranch
                            && branch.choice
                                == ngc::BranchChoice::Continue
                            && branch.continuation == secondId;
                    }),
                "feed hold did not select the dependent continuation");
        require(selectEvents<ngc::BackendFault>(events).empty(),
                "feed hold faulted at a valid continuation boundary");
    }

    void testIncrementalJointGroupJogReachesTarget() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 50);

        const ngc::StartIncrementalJogRequest request{
            .id = 10,
            .jog = 101,
            .target = {
                .type = ngc::JogTargetType::JointGroup,
                .joints = ngc::JointMask{1}
                    | ngc::JointMask{1} << 1,
            },
            .distance = 0.25,
            .velocity = 0.5,
            .limits = jogLimits(),
            .stopLimits = jogLimits(),
        };
        require(core->trySubmit(request) == ngc::SubmitResult::Submitted,
                "incremental joint-group jog did not fit");

        const auto run = runJogUntilHeld(*core);
        const auto requests =
            selectEvents<ngc::RequestCompleted>(run.events);
        require(std::ranges::any_of(
                    requests, [&](const auto &completed) {
                        return completed.request == request.id
                            && completed.succeeded;
                    }),
                "incremental joint-group jog was not acknowledged");
        const auto stopped = selectEvents<ngc::JogStopped>(run.events);
        require(stopped.size() == 1
                    && stopped[0].jog == request.jog
                    && stopped[0].reason
                        == ngc::JogStopReason::TargetReached,
                "incremental joint-group jog did not report target completion");
        requireNear(run.snapshot.commandedJoints.position[0], 0.25,
                    "incremental jog missed the first joint target");
        requireNear(run.snapshot.commandedJoints.position[1], 0.25,
                    "incremental jog missed the coupled joint target");
        requireNear(run.snapshot.commandedJoints.velocity[0], 0.0,
                    "incremental jog retained joint velocity");
        requireNear(run.snapshot.commandedJoints.acceleration[1], 0.0,
                    "incremental jog retained joint acceleration");
        require(run.snapshot.activeJoints == 0,
                "completed incremental jog retained active joints");

        auto previousAcceleration = 0.0;
        for (const auto &sample : run.samples) {
            const auto velocity =
                sample.commandedJoints.velocity[0];
            const auto acceleration =
                sample.commandedJoints.acceleration[0];
            const auto jerk =
                (acceleration - previousAcceleration) / 0.01;
            require(std::abs(velocity) <= 0.5 + 1e-9,
                    "incremental jog exceeded its velocity limit");
            require(std::abs(acceleration) <= 1.0 + 1e-9,
                    "incremental jog exceeded its acceleration limit");
            require(std::abs(jerk) <= 4.0 + 1e-7,
                    "incremental jog exceeded its jerk limit");
            previousAcceleration = acceleration;
        }
    }

    void testContinuousJogLeaseRenewalAndExpiry() {
        ngc::ProductionExecutorConfiguration configuration;
        configuration.maximumJogLeaseTicks = 8;
        auto core = std::make_unique<ngc::ProductionExecutorCore>(
            0.01, configuration);
        initialize(*core, 51);

        const ngc::StartContinuousJogRequest request{
            .id = 20,
            .jog = 102,
            .target = {
                .type = ngc::JogTargetType::Joint,
                .joints = ngc::JointMask{1},
            },
            .signedVelocity = 0.5,
            .limits = jogLimits(),
            .stopLimits = jogLimits(),
            .leaseTicks = 100,
        };
        require(core->trySubmit(request) == ngc::SubmitResult::Submitted,
                "continuous jog did not fit");

        std::vector<ngc::ExecutionEvent> events;
        for (auto tick = 0; tick < 5; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            latestSnapshot(*core);
        }
        require(core->trySubmit(
                    ngc::RenewJogLeaseRequest{21, request.jog})
                    == ngc::SubmitResult::Submitted,
                "jog lease renewal did not fit");
        for (auto tick = 0; tick < 5; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            const auto snapshot = latestSnapshot(*core);
            require(snapshot.state == ngc::BackendState::Running,
                    "renewed jog lease expired at its original deadline");
        }

        auto run = runJogUntilHeld(*core);
        events.insert(
            events.end(), run.events.begin(), run.events.end());
        const auto requests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    requests, [](const auto &completed) {
                        return completed.request == 21
                            && completed.succeeded;
                    }),
                "matching jog lease renewal was not acknowledged");
        const auto stopped = selectEvents<ngc::JogStopped>(events);
        require(stopped.size() == 1
                    && stopped[0].reason
                        == ngc::JogStopReason::LeaseExpired,
                "unrenewed continuous jog did not stop on lease expiry");
        require(run.snapshot.commandedJoints.position[0] > 0.0,
                "lease-expired jog did not advance the selected joint");
        requireNear(run.snapshot.commandedJoints.velocity[0], 0.0,
                    "lease-expired jog retained velocity");
        requireNear(run.snapshot.commandedJoints.acceleration[0], 0.0,
                    "lease-expired jog retained acceleration");
    }

    void testContinuousJogVelocityUpdateAndTokenMatchedStop() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 52);

        const ngc::StartContinuousJogRequest request{
            .id = 30,
            .jog = 103,
            .target = {
                .type = ngc::JogTargetType::Joint,
                .joints = ngc::JointMask{1},
            },
            .signedVelocity = 0.5,
            .limits = jogLimits(),
            .stopLimits = jogLimits(),
            .leaseTicks = 2'000,
        };
        require(core->trySubmit(request) == ngc::SubmitResult::Submitted,
                "velocity jog did not fit");

        std::vector<ngc::ExecutionEvent> events;
        auto snapshot = ngc::ExecutionSnapshot{};
        for (auto tick = 0; tick < 100; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            snapshot = latestSnapshot(*core);
        }
        require(snapshot.commandedJoints.velocity[0] > 0.0,
                "continuous jog did not establish positive motion");
        require(core->trySubmit(
                    ngc::SetContinuousJogVelocityRequest{
                        31, request.jog, -0.25})
                    == ngc::SubmitResult::Submitted,
                "continuous jog velocity update did not fit");

        auto reversed = false;
        for (auto tick = 0; tick < 300; ++tick) {
            core->servoTick();
            auto current = takeEvents(*core);
            events.insert(events.end(), current.begin(), current.end());
            snapshot = latestSnapshot(*core);
            if (snapshot.commandedJoints.velocity[0] < -0.1) {
                reversed = true;
                break;
            }
        }
        require(reversed,
                "continuous jog did not recompute through a reversal");
        require(core->trySubmit(
                    ngc::StopJogRequest{32, request.jog + 1})
                    == ngc::SubmitResult::Submitted,
                "stale jog stop did not fit");
        require(core->trySubmit(
                    ngc::StopJogRequest{33, request.jog})
                    == ngc::SubmitResult::Submitted,
                "matching jog stop did not fit");

        auto run = runJogUntilHeld(*core);
        events.insert(
            events.end(), run.events.begin(), run.events.end());
        const auto requests =
            selectEvents<ngc::RequestCompleted>(events);
        require(std::ranges::any_of(
                    requests, [](const auto &completed) {
                        return completed.request == 31
                            && completed.succeeded;
                    }),
                "velocity update was not acknowledged");
        require(std::ranges::any_of(
                    requests, [](const auto &completed) {
                        return completed.request == 32
                            && !completed.succeeded;
                    }),
                "stale jog token stopped the active jog");
        require(std::ranges::any_of(
                    requests, [](const auto &completed) {
                        return completed.request == 33
                            && completed.succeeded;
                    }),
                "matching jog stop was not acknowledged");
        const auto stopped = selectEvents<ngc::JogStopped>(events);
        require(stopped.size() == 1
                    && stopped[0].jog == request.jog
                    && stopped[0].reason
                        == ngc::JogStopReason::RequestedStop,
                "token-matched stop did not complete exactly one jog");
        requireNear(run.snapshot.commandedJoints.velocity[0], 0.0,
                    "requested jog stop retained velocity");
        requireNear(run.snapshot.commandedJoints.acceleration[0], 0.0,
                    "requested jog stop retained acceleration");
    }

    void testLogicalAxisJogUpdatesMappedJoints() {
        ngc::ProductionExecutorConfiguration configuration;
        auto &x = configuration.axes[
            static_cast<std::size_t>(ngc::AxisId::X)];
        x.joints = ngc::JointMask{1} | ngc::JointMask{1} << 1;
        x.coordinateScale[0] = 1.0;
        x.coordinateScale[1] = -1.0;
        auto core = std::make_unique<ngc::ProductionExecutorCore>(
            0.01, configuration);
        initialize(*core, 53);
        ngc::JointVector jointPosition;
        jointPosition[0] = 1.0;
        jointPosition[1] = -1.0;
        require(core->trySubmit(ngc::SetJointPositionRequest{
                    39, x.joints, jointPosition})
                    == ngc::SubmitResult::Submitted,
                "logical-axis joint origin assignment did not fit");
        core->servoTick();
        takeEvents(*core);
        latestSnapshot(*core);

        const ngc::StartIncrementalJogRequest request{
            .id = 40,
            .jog = 104,
            .target = {
                .type = ngc::JogTargetType::Axis,
                .axis = ngc::AxisId::X,
            },
            .distance = 0.2,
            .velocity = 0.5,
            .limits = jogLimits(),
            .stopLimits = jogLimits(),
        };
        require(core->trySubmit(request) == ngc::SubmitResult::Submitted,
                "logical-axis jog did not fit");

        const auto run = runJogUntilHeld(*core);
        const auto stopped =
            selectEvents<ngc::JogStopped>(run.events);
        require(stopped.size() == 1
                    && stopped[0].reason
                        == ngc::JogStopReason::TargetReached,
                "logical-axis jog did not complete");
        requireNear(run.snapshot.commanded.position.x, 0.2,
                    "logical-axis jog missed its coordinate target");
        requireNear(run.snapshot.commandedJoints.position[0], 1.2,
                    "logical-axis jog did not preserve the first joint offset");
        requireNear(run.snapshot.commandedJoints.position[1], -1.2,
                    "logical-axis jog did not preserve the second joint offset");
        requireNear(run.snapshot.commandedJoints.velocity[0], 0.0,
                    "logical-axis jog retained mapped joint velocity");
        requireNear(run.snapshot.commandedJoints.acceleration[1], 0.0,
                    "logical-axis jog retained mapped joint acceleration");
    }

    void testContinuousJogStopsAtTravelLimit() {
        auto core = std::make_unique<ngc::ProductionExecutorCore>(0.01);
        initialize(*core, 54);

        const ngc::StartContinuousJogRequest request{
            .id = 50,
            .jog = 105,
            .target = {
                .type = ngc::JogTargetType::Joint,
                .joints = ngc::JointMask{1},
            },
            .signedVelocity = 0.5,
            .limits = jogLimits(),
            .stopLimits = jogLimits(),
            .travel = {
                .minimum = -0.1,
                .maximum = 0.1,
                .enabled = true,
            },
            .leaseTicks = 1'000,
        };
        require(core->trySubmit(request) == ngc::SubmitResult::Submitted,
                "travel-limited jog did not fit");

        const auto run = runJogUntilHeld(*core);
        const auto stopped =
            selectEvents<ngc::JogStopped>(run.events);
        require(stopped.size() == 1
                    && stopped[0].reason
                        == ngc::JogStopReason::LimitReached,
                "continuous jog did not report its travel limit");
        requireNear(run.snapshot.commandedJoints.position[0], 0.1,
                    "continuous jog crossed or missed its travel limit");
        requireNear(run.snapshot.commandedJoints.velocity[0], 0.0,
                    "travel-limited jog retained velocity");
        requireNear(run.snapshot.commandedJoints.acceleration[0], 0.0,
                    "travel-limited jog retained acceleration");
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

        auto rejectedMapping = false;
        try {
            ngc::ProductionExecutorConfiguration configuration;
            configuration.axes[0].joints = ngc::JointMask{1};
            const auto invalid =
                std::make_unique<ngc::ProductionExecutorCore>(
                    0.001, configuration);
            static_cast<void>(invalid);
        } catch (const std::invalid_argument &) {
            rejectedMapping = true;
        }
        require(rejectedMapping,
                "executor accepted an axis mapping with a zero joint scale");

        auto rejectedFeedHold = false;
        try {
            ngc::ProductionExecutorConfiguration configuration;
            configuration.feedHold.tangentialAcceleration = 1.0;
            const auto invalid =
                std::make_unique<ngc::ProductionExecutorCore>(
                    0.001, configuration);
            static_cast<void>(invalid);
        } catch (const std::invalid_argument &) {
            rejectedFeedHold = true;
        }
        require(rejectedFeedHold,
                "executor accepted an incomplete feed-hold configuration");

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
                    0, 1, ngc::InputCondition::Active,
                }),
                "required joint trigger fixture did not fit");
        require(core->tryPublish(ngc::ExecutionItem{triggeredJoint})
                    == ngc::PublishResult::Invalid,
                "executor accepted a required joint move with a missing trigger");

        triggeredJoint.triggerRequired = false;
        require(triggeredJoint.triggers.push({
                    0, 2, ngc::InputCondition::Active,
                }),
                "duplicate joint trigger fixture did not fit");
        require(core->tryPublish(ngc::ExecutionItem{triggeredJoint})
                    == ngc::PublishResult::Invalid,
                "executor accepted duplicate triggers for one joint");

        auto scheduled =
            linearChunk(1, 1, 0, 1, 1, 0.0, 1.0, 1.0);
        require(scheduled.events.push({1, ngc::SpindleEvent{}}),
                "scheduled event fixture did not fit");
        require(core->tryPublish(scheduled) == ngc::PublishResult::Invalid,
                "executor accepted a scheduled event outside normal motion");

        auto invalidSpindle =
            linearChunk(1, 1, 0, 1, 1, 0.0, 1.0, 1.0);
        require(invalidSpindle.events.push({
                    0, ngc::SpindleEvent{
                        true, ngc::Direction::CW,
                        std::numeric_limits<double>::quiet_NaN(),
                    },
                }),
                "invalid spindle event fixture did not fit");
        require(core->tryPublish(invalidSpindle)
                    == ngc::PublishResult::Invalid,
                "executor accepted a non-finite spindle command");

        auto unorderedEvents =
            linearChunk(1, 1, 0, 1, 1, 0.0, 1.0, 1.0);
        appendLinearSpan(unorderedEvents, 3, 1.0, 2.0, 1.0);
        require(unorderedEvents.events.push({
                    1, ngc::SpindleEvent{},
                })
                    && unorderedEvents.events.push({
                        0, ngc::SpindleEvent{},
                    }),
                "unordered scheduled-event fixture did not fit");
        require(core->tryPublish(unorderedEvents)
                    == ngc::PublishResult::Invalid,
                "executor accepted out-of-order scheduled events");

        auto invalidDirection =
            linearChunk(1, 1, 0, 1, 1, 0.0, 1.0, 1.0);
        require(invalidDirection.events.push({
                    0, ngc::SpindleEvent{
                        true, static_cast<ngc::Direction>(2), 1'000.0,
                    },
                }),
                "invalid spindle direction fixture did not fit");
        require(core->tryPublish(invalidDirection)
                    == ngc::PublishResult::Invalid,
                "executor accepted an invalid spindle direction");

        initialize(*core, 1);
        const ngc::StartIncrementalJogRequest unsupportedAxis{
            .id = 100,
            .jog = 1,
            .target = {
                .type = ngc::JogTargetType::Axis,
                .axis = ngc::AxisId::X,
            },
            .distance = 0.1,
            .velocity = 0.1,
            .limits = jogLimits(),
            .stopLimits = jogLimits(),
        };
        require(core->trySubmit(unsupportedAxis)
                    == ngc::SubmitResult::Submitted,
                "unconfigured axis jog did not fit for rejection");
        core->servoTick();
        const auto unsupportedEvents = takeEvents(*core);
        latestSnapshot(*core);
        const auto unsupportedRequests =
            selectEvents<ngc::RequestCompleted>(unsupportedEvents);
        require(unsupportedRequests.size() == 1
                    && unsupportedRequests[0].request
                        == unsupportedAxis.id
                    && !unsupportedRequests[0].succeeded,
                "executor accepted an unconfigured logical-axis jog");

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
        testEnabledResetRetainsPoweredHeldState();
        testContinuationMarkersAndTerminalStop();
        testScheduledSpindleEventsFollowExecutionCursor();
        testAbortSuppressesFutureScheduledEvents();
        testMismatchedContinuationStopsSafely();
        testTriggeredMoveReachesTargetAtRest();
        testSampledTriggerGeneratesConstrainedStop();
        testPlanContinuesIntoTriggeredMove();
        testTriggeredJointsStopIndependentlyAndRetainState();
        testHomingControlSequence();
        testControlledStopAbortsTriggeredJointMove();
        testControlledStopAbortsAxisTriggeredMove();
        testControlledStopCancelsOrdinaryPlan();
        testFeedHoldPreservesAndResumesAxisTriggeredMove();
        testAxisTriggerDuringFeedHoldBrakingCompletesNormally();
        testControlledStopAbortsFeedHeldAxisTriggeredMove();
        testFeedHoldPreservesAndResumesOrdinaryPlan();
        testFeedRetimingFaultsAtStopBranch();
        testFeedHoldContinuesAcrossDependentChunks();
        testIncrementalJointGroupJogReachesTarget();
        testContinuousJogLeaseRenewalAndExpiry();
        testContinuousJogVelocityUpdateAndTokenMatchedStop();
        testLogicalAxisJogUpdatesMappedJoints();
        testContinuousJogStopsAtTravelLimit();
        testBoundedAndExplicitlyUnsupportedInputs();
    } catch (const std::exception &error) {
        std::cerr << "Production executor core test failure: "
                  << error.what() << '\n';

        return 1;
    }

    std::cout << "Production executor core tests passed\n";

    return 0;
}
