#include <algorithm>
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
                "initial production core accepted an unsupported triggered move");

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
        testBoundedAndExplicitlyUnsupportedInputs();
    } catch (const std::exception &error) {
        std::cerr << "Production executor core test failure: "
                  << error.what() << '\n';

        return 1;
    }

    std::cout << "Production executor core tests passed\n";

    return 0;
}
