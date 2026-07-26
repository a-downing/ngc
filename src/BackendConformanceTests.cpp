#include "test/BackendConformance.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace ngc::test {
    namespace {
        constexpr std::size_t MAX_SERVICE_STEPS = 10'000;

        [[noreturn]] void fail(const BackendConformanceTarget &target, const std::string_view message) {
            throw std::runtime_error(std::format("{} backend conformance: {}", target.name, message));
        }

        void require(const BackendConformanceTarget &target, const bool condition, const std::string_view message) {
            if (!condition) {
                fail(target, message);
            }
        }

        AxisPolynomialSpan linearSpan(const SpanId id, const double from, const double to, const double duration) {
            AxisPolynomialSpan span;
            span.id = id;
            span.duration = duration;
            span.inverseDuration = 1.0 / duration;
            span.inverseDurationSquared = span.inverseDuration * span.inverseDuration;
            span.inverseDurationCubed = span.inverseDurationSquared * span.inverseDuration;
            span.origin.x = from;
            span.coefficients[0].x = to - from;

            return span;
        }

        PlanChunk linearChunk(const EpochId epoch, const ChunkId id, const BranchSequence predecessor,
                              const BranchSequence branch, const SpanId span, const double from,
                              const double to, const double duration) {
            PlanChunk chunk;
            chunk.epoch = epoch;
            chunk.id = id;
            chunk.predecessorBranch = predecessor;
            chunk.branch = branch;
            if (!chunk.normalMotion.push(linearSpan(span, from, to, duration))) {
                throw std::logic_error("backend conformance normal span did not fit");
            }
            if (!chunk.stopTail.push(linearSpan(span + 1, to, to, 0.001))) {
                throw std::logic_error("backend conformance stop span did not fit");
            }
            chunk.branchState = executionSpanEnd(chunk.normalMotion[0]);
            chunk.stopState.position.x = to;

            return chunk;
        }

        class RuntimeFixture {
        public:
            RuntimeFixture(const BackendConformanceTarget &target,
                           std::unique_ptr<BackendRuntime> runtime)
                : m_target(target), m_runtime(std::move(runtime)),
                  m_backend(endpoint(target, m_runtime)) { }

            BackendRuntime &runtime() noexcept {
                return *m_runtime;
            }

            MotionBackend &backend() noexcept {
                return m_backend;
            }

            void serviceImmediate() {
                m_runtime->serviceImmediate();
            }

            void servicePeriod() {
                static_cast<void>(m_runtime->advanceServiceMotionPeriod());
            }

            std::vector<ExecutionEvent> takeEvents() {
                std::vector<ExecutionEvent> events;
                ExecutionEvent event;
                while (m_backend.tryTakeEvent(event)) {
                    events.push_back(event);
                }

                return events;
            }

            ExecutionSnapshot latestSnapshot() {
                ExecutionSnapshot snapshot;
                auto found = false;
                while (m_backend.tryTakeSnapshot(snapshot)) {
                    found = true;
                }
                if (!found) {
                    serviceImmediate();
                    while (m_backend.tryTakeSnapshot(snapshot)) {
                        found = true;
                    }
                }
                require(m_target, found, "the runtime did not publish an execution snapshot");

                return snapshot;
            }

            void initialize(const EpochId epoch) {
                require(m_target,
                        m_backend.trySubmit(ResetRequest{nextRequest(), epoch})
                            == SubmitResult::Submitted,
                        "reset did not fit in the control channel");
                serviceImmediate();
                takeEvents();

                require(m_target,
                        m_backend.trySubmit(EnableRequest{nextRequest()})
                            == SubmitResult::Submitted,
                        "enable did not fit in the control channel");
                serviceImmediate();
                const auto events = takeEvents();
                require(m_target,
                        std::ranges::any_of(events, [](const ExecutionEvent &event) {
                            const auto *completed = std::get_if<RequestCompleted>(&event);
                            return completed && completed->succeeded;
                        }),
                        "enable was not acknowledged");
                require(m_target, latestSnapshot().state == BackendState::Held,
                        "enable did not establish the held state");
            }

            RequestId nextRequest() noexcept {
                return m_nextRequest++;
            }

        private:
            static MotionBackend &endpoint(const BackendConformanceTarget &target,
                                           const std::unique_ptr<BackendRuntime> &runtime) {
                require(target, static_cast<bool>(runtime),
                        "the target factory returned no runtime");

                return runtime->endpoint();
            }

            const BackendConformanceTarget &m_target;
            std::unique_ptr<BackendRuntime> m_runtime;
            MotionBackend &m_backend;
            RequestId m_nextRequest = 1;
        };

        template<typename Event>
        std::vector<Event> selectEvents(const std::vector<ExecutionEvent> &events) {
            std::vector<Event> selected;
            for (const auto &event : events) {
                if (const auto *value = std::get_if<Event>(&event)) {
                    selected.push_back(*value);
                }
            }

            return selected;
        }

        std::vector<ExecutionEvent> serviceUntilStationary(
            const BackendConformanceTarget &target, RuntimeFixture &fixture) {
            std::vector<ExecutionEvent> events;
            for (std::size_t step = 0; step < MAX_SERVICE_STEPS; ++step) {
                fixture.servicePeriod();
                auto current = fixture.takeEvents();
                events.insert(events.end(), current.begin(), current.end());
                const auto state = fixture.latestSnapshot().state;
                if (state == BackendState::Held || state == BackendState::Disabled
                    || state == BackendState::Faulted) {
                    return events;
                }
            }
            fail(target, "execution did not reach a stationary terminal state");
        }

        void verifyRuntimeLifecycle(const BackendConformanceTarget &target) {
            auto runtime = target.createRuntime();
            require(target, static_cast<bool>(runtime), "the target factory returned no runtime");

            StationaryBackendState restored;
            restored.commanded.position.x = 0.125;
            restored.feedback = restored.commanded;
            require(target, runtime->restoreStationaryState(restored),
                    "a stopped runtime rejected stationary-state restoration");

            runtime->start();
            runtime->start();
            require(target, !runtime->restoreStationaryState(restored),
                    "a started runtime accepted stationary-state restoration");
            runtime->stop();
            runtime->stop();
            require(target, runtime->restoreStationaryState(restored),
                    "a stopped runtime did not accept a second stationary-state restoration");

            RuntimeFixture fixture(target, std::move(runtime));
            fixture.initialize(1);
            require(target,
                    fixture.backend().trySubmit(DisableRequest{fixture.nextRequest()})
                        == SubmitResult::Submitted,
                    "disable did not fit in the control channel");
            fixture.serviceImmediate();
            require(target, fixture.latestSnapshot().state == BackendState::Disabled,
                    "disable did not establish the disabled state");
        }

        void verifyExecutionItemValidation(const BackendConformanceTarget &target) {
            RuntimeFixture fixture(target, target.createRuntime());

            auto missingBranch =
                linearChunk(1, 1, 0, 1, 1, 0.0, 0.1, 0.1);
            missingBranch.branch = 0;
            require(target,
                    fixture.backend().tryPublish(missingBranch)
                        == PublishResult::Invalid,
                    "an execution item without a branch was accepted");

            auto invalidEvent =
                linearChunk(1, 2, 0, 2, 3, 0.0, 0.1, 0.1);
            require(target,
                    invalidEvent.events.push({
                        1, SpindleEvent{},
                    }),
                    "the invalid scheduled-event fixture did not fit");
            require(target,
                    fixture.backend().tryPublish(invalidEvent)
                        == PublishResult::Invalid,
                    "a scheduled event outside normal motion was accepted");

            auto invalidTrigger = target.makeTriggeredJointMove();
            invalidTrigger.epoch = 1;
            invalidTrigger.branch = 1;
            invalidTrigger.triggerRequired = true;
            invalidTrigger.triggers.size = 0;
            require(target,
                    fixture.backend().tryPublish(invalidTrigger)
                        == PublishResult::Invalid,
                    "a required triggered-joint move without triggers was accepted");
        }

        void verifyPublicationMarkersAndRepeatedEpochs(
            const BackendConformanceTarget &target) {
            RuntimeFixture fixture(target, target.createRuntime());
            fixture.initialize(10);

            auto first = linearChunk(10, 101, 0, 201, 301, 0.0, 0.05, 0.05);
            auto second = linearChunk(10, 102, first.branch, 202, 303, 0.05, 0.1, 0.05);
            require(target,
                    first.markers.push({401, 0, 0.0})
                        && first.markers.push({402, 0, 0.5})
                        && second.markers.push({403, 0, 0.0})
                        && second.markers.push({404, 0, 1.0}),
                    "marker fixtures did not fit");
            require(target,
                    fixture.backend().tryPublish(first) == PublishResult::Published
                        && fixture.backend().tryPublish(second)
                            == PublishResult::Published,
                    "a valid dependent execution horizon was not published");
            require(target,
                    fixture.backend().trySubmit(StartRequest{
                        fixture.nextRequest(), first.epoch}) == SubmitResult::Submitted,
                    "start did not fit in the control channel");

            const auto events = serviceUntilStationary(target, fixture);
            const auto accepted = selectEvents<ChunkAccepted>(events);
            const auto retired = selectEvents<ChunkRetired>(events);
            const auto branches = selectEvents<BranchSelected>(events);
            const auto markers = selectEvents<ExecutionMarkerReached>(events);
            require(target, accepted.size() == 2 && accepted[0].chunk == first.id
                    && accepted[1].chunk == second.id,
                    "dependent chunks were not accepted in publication order");
            require(target, retired.size() == 2 && retired[0].chunk == first.id
                    && retired[1].chunk == second.id,
                    "accepted chunks were not retired exactly once in execution order");
            require(target, branches.size() == 2
                    && branches[0].choice == BranchChoice::Continue
                    && branches[0].continuation == second.id
                    && branches[1].choice == BranchChoice::Stop,
                    "the runtime did not continue and then select the terminal stop branch");
            require(target, markers.size() == 4
                    && markers[0].marker == 401 && markers[1].marker == 402
                    && markers[2].marker == 403 && markers[3].marker == 404,
                    "execution markers were not emitted exactly once in trajectory order");
            const auto completed = fixture.latestSnapshot();
            require(target, completed.state == BackendState::Held
                    && std::abs(completed.commanded.position.x - 0.1) <= 1e-9,
                    "the dependent horizon did not stop at its declared terminal state");

            fixture.initialize(11);
            auto repeated = linearChunk(11, 103, 0, 203, 305, 0.0, 0.025, 0.025);
            require(target, repeated.markers.push({405, 0, 0.5}),
                    "repeated-epoch marker did not fit");
            require(target,
                    fixture.backend().tryPublish(repeated) == PublishResult::Published,
                    "the repeated epoch did not accept a new chunk");
            require(target,
                    fixture.backend().trySubmit(StartRequest{
                        fixture.nextRequest(), repeated.epoch}) == SubmitResult::Submitted,
                    "the repeated epoch did not accept start");
            const auto repeatedEvents = serviceUntilStationary(target, fixture);
            const auto repeatedMarkers =
                selectEvents<ExecutionMarkerReached>(repeatedEvents);
            require(target, repeatedMarkers.size() == 1
                    && repeatedMarkers[0].epoch == repeated.epoch
                    && repeatedMarkers[0].marker == 405,
                    "a repeated epoch did not isolate its marker stream");
        }

        void verifyTriggeredJointMotion(const BackendConformanceTarget &target) {
            RuntimeFixture fixture(target, target.createRuntime());
            fixture.initialize(20);

            auto move = target.makeTriggeredJointMove();
            move.epoch = 20;
            require(target, move.id != 0 && move.moveId != 0 && move.joints != 0
                    && move.triggers.size != 0,
                    "the target supplied an incomplete triggered-joint fixture");
            require(target, fixture.runtime().prepareTriggeredJointMove(move),
                    "the runtime could not prepare its triggered-joint test inputs");
            require(target,
                    fixture.backend().tryPublish(ExecutionItem{move})
                        == PublishResult::Published,
                    "the triggered-joint move was not published");
            require(target,
                    fixture.backend().trySubmit(StartRequest{
                        fixture.nextRequest(), move.epoch}) == SubmitResult::Submitted,
                    "the triggered-joint epoch did not accept start");

            const auto events = serviceUntilStationary(target, fixture);
            const auto completed =
                selectEvents<TriggeredJointMoveCompleted>(events);
            require(target, completed.size() == 1
                    && completed[0].move == move.moveId
                    && completed[0].status == TriggeredMoveStatus::Triggered
                    && completed[0].triggeredJoints == move.joints,
                    "the triggered-joint move did not report every configured trigger");
            for (JointId joint = 0; joint < MAX_JOINTS; ++joint) {
                if ((move.joints & (JointMask{1} << joint)) == 0) {
                    continue;
                }
                require(target,
                        std::abs(completed[0].stoppedState.velocity[joint]) <= 1e-9
                            && std::abs(completed[0].stoppedState.acceleration[joint])
                                <= 1e-9,
                        "triggered joint motion completed before reaching rest");
            }
        }

        void verifyJogLease(const BackendConformanceTarget &target) {
            RuntimeFixture fixture(target, target.createRuntime());
            fixture.initialize(30);

            const StartContinuousJogRequest request{
                .id = fixture.nextRequest(),
                .jog = 501,
                .target = {
                    .type = JogTargetType::Joint,
                    .joints = JointMask{1},
                },
                .signedVelocity = 0.5,
                .limits = {1.0, 2.0, 10.0},
                .stopLimits = {1.0, 2.0, 10.0},
                .leaseTicks = 20,
            };
            require(target,
                    fixture.backend().trySubmit(ControlRequest{request})
                        == SubmitResult::Submitted,
                    "continuous jog did not fit in the control channel");

            std::vector<ExecutionEvent> events;
            for (std::size_t step = 0; step < MAX_SERVICE_STEPS; ++step) {
                fixture.servicePeriod();
                auto current = fixture.takeEvents();
                events.insert(events.end(), current.begin(), current.end());
                if (!selectEvents<JogStopped>(events).empty()) {
                    break;
                }
            }
            const auto stopped = selectEvents<JogStopped>(events);
            require(target, stopped.size() == 1 && stopped[0].jog == request.jog
                    && stopped[0].reason == JogStopReason::LeaseExpired,
                    "an unrenewed continuous jog did not stop on lease expiry");
            require(target, stopped[0].jointState.position[0] > 0.0
                    && std::abs(stopped[0].jointState.velocity[0]) <= 1e-9
                    && std::abs(stopped[0].jointState.acceleration[0]) <= 1e-9,
                    "lease expiry did not return the moving joint to rest");
        }

        void verifyControlledStopAndAbort(const BackendConformanceTarget &target) {
            RuntimeFixture fixture(target, target.createRuntime());
            fixture.initialize(40);

            const auto chunk =
                linearChunk(40, 601, 0, 701, 801, 0.0, 1.0, 1.0);
            require(target,
                    fixture.backend().tryPublish(chunk) == PublishResult::Published,
                    "the controlled-stop trajectory was not published");
            require(target,
                    fixture.backend().trySubmit(StartRequest{
                        fixture.nextRequest(), chunk.epoch}) == SubmitResult::Submitted,
                    "the controlled-stop trajectory did not accept start");
            fixture.servicePeriod();
            fixture.takeEvents();
            const auto stopBeginning = fixture.latestSnapshot().commanded.position.x;
            const auto stopRequest = fixture.nextRequest();
            require(target,
                    fixture.backend().trySubmit(ControlledStopRequest{stopRequest})
                        == SubmitResult::Submitted,
                    "controlled stop did not fit in the control channel");
            const auto stoppedEvents = serviceUntilStationary(target, fixture);
            const auto held = selectEvents<BackendHeld>(stoppedEvents);
            const auto stopped = fixture.latestSnapshot();
            require(target, stopped.state == BackendState::Held
                    && stopped.commanded.position.x > stopBeginning
                    && stopped.commanded.position.x < 1.0
                    && std::abs(stopped.commanded.velocity.x) <= 1e-9
                    && std::abs(stopped.commanded.acceleration.x) <= 1e-9,
                    "controlled stop did not brake the active trajectory to rest");
            require(target,
                    std::ranges::any_of(held, [](const BackendHeld &event) {
                        return event.reason == BackendHoldReason::ControlledStop;
                    }),
                    "controlled stop did not identify its terminal hold reason");

            const auto resumeRequest = fixture.nextRequest();
            require(target,
                    fixture.backend().trySubmit(
                        ResumeRequest{resumeRequest, chunk.epoch})
                        == SubmitResult::Submitted,
                    "post-stop resume did not fit for rejection testing");
            fixture.serviceImmediate();
            const auto resumeEvents = fixture.takeEvents();
            require(target,
                    std::ranges::any_of(resumeEvents,
                        [resumeRequest](const ExecutionEvent &event) {
                            const auto *completed =
                                std::get_if<RequestCompleted>(&event);
                            return completed
                                && completed->request == resumeRequest
                                && !completed->succeeded;
                        }),
                    "controlled stop did not permanently reject epoch resume");

            fixture.initialize(41);
            const auto abortedChunk =
                linearChunk(41, 602, 0, 702, 803, 0.0, 1.0, 1.0);
            require(target,
                    fixture.backend().tryPublish(abortedChunk)
                        == PublishResult::Published,
                    "the abort trajectory was not published");
            require(target,
                    fixture.backend().trySubmit(StartRequest{
                        fixture.nextRequest(), abortedChunk.epoch})
                        == SubmitResult::Submitted,
                    "the abort trajectory did not accept start");
            fixture.servicePeriod();
            fixture.takeEvents();
            require(target,
                    fixture.backend().trySubmit(AbortRequest{fixture.nextRequest()})
                        == SubmitResult::Submitted,
                    "abort did not fit in the control channel");
            fixture.serviceImmediate();
            const auto aborted = fixture.latestSnapshot();
            require(target, aborted.state == BackendState::Held
                    && aborted.queuedExecutionItems == 0,
                    "abort did not abandon active and queued execution");
        }

        void verifyBoundedChannels(const BackendConformanceTarget &target) {
            RuntimeFixture planFixture(target, target.createRuntime());
            auto sawPlanFull = false;
            for (std::size_t index = 0; index < 1'024; ++index) {
                auto chunk = linearChunk(50, index + 1, 0, index + 1,
                                         index * 2 + 1, 0.0, 0.01, 0.01);
                const auto result = planFixture.backend().tryPublish(chunk);
                require(target, result != PublishResult::Invalid,
                        "a valid chunk was rejected while probing plan capacity");
                if (result == PublishResult::Full) {
                    sawPlanFull = true;
                    break;
                }
            }
            require(target, sawPlanFull,
                    "the execution publication channel did not expose bounded capacity");

            RuntimeFixture controlFixture(target, target.createRuntime());
            auto sawControlFull = false;
            for (std::size_t index = 0; index < 1'024; ++index) {
                const auto result = controlFixture.backend().trySubmit(
                    EnableRequest{index + 1});
                if (result == SubmitResult::Full) {
                    sawControlFull = true;
                    break;
                }
            }
            require(target, sawControlFull,
                    "the control channel did not expose bounded capacity");
        }

        void verifyFaultTransition(const BackendConformanceTarget &target) {
            RuntimeFixture fixture(target, target.createRuntime());
            fixture.initialize(60);

            const auto chunk =
                linearChunk(60, 901, 0, 1'001, 1'101, 0.0, 0.1, 0.1);
            require(target,
                    fixture.backend().tryPublish(chunk) == PublishResult::Published,
                    "the fault trajectory was not published");
            require(target,
                    fixture.backend().trySubmit(StartRequest{
                        fixture.nextRequest(), chunk.epoch}) == SubmitResult::Submitted,
                    "the fault trajectory did not accept start");
            for (auto step = 0; step < 9; ++step) {
                fixture.servicePeriod();
                fixture.takeEvents();
            }
            require(target,
                    fixture.backend().trySubmit(
                        FeedHoldRequest{fixture.nextRequest()})
                        == SubmitResult::Submitted,
                    "late feed hold did not fit in the control channel");

            const auto events = serviceUntilStationary(target, fixture);
            const auto faults = selectEvents<BackendFault>(events);
            require(target, fixture.latestSnapshot().state == BackendState::Faulted
                    && !faults.empty(),
                    "an infeasible late hold did not produce a visible backend fault");
        }
    }

    void runBackendConformanceSuite(const BackendConformanceTarget &target) {
        if (!target.createRuntime) {
            fail(target, "no runtime factory was supplied");
        }
        if (!target.makeTriggeredJointMove) {
            fail(target, "no triggered-joint fixture was supplied");
        }

        verifyRuntimeLifecycle(target);
        verifyExecutionItemValidation(target);
        verifyPublicationMarkersAndRepeatedEpochs(target);
        verifyTriggeredJointMotion(target);
        verifyJogLease(target);
        verifyControlledStopAndAbort(target);
        verifyBoundedChannels(target);
        verifyFaultTransition(target);
    }
}
