#include "machine/MockMotionBackend.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <ruckig/ruckig.hpp>

#include "machine/SpscChannel.h"

namespace ngc {
    namespace {
        constexpr std::size_t PLAN_CAPACITY = 8;
        constexpr std::size_t CONTROL_CAPACITY = 16;
        // One drain interval can retire the complete plan horizon. Each chunk can
        // produce accepted, branch, retired, and probe/held records, in addition
        // to control acknowledgements. Keep enough return capacity for that
        // worst-case burst so the bounded forward horizon cannot deadlock itself.
        constexpr std::size_t EVENT_CAPACITY = 64;
        constexpr std::size_t SNAPSHOT_CAPACITY = 4;
        constexpr std::size_t SYNTHETIC_INPUT_CAPACITY = 16;

        struct SyntheticInputTransition {
            TriggeredMoveId move = 0;
            position_t position{};
        };

        double magnitude(const position_t &value) {
            return std::sqrt(value.x*value.x + value.y*value.y + value.z*value.z
                + value.a*value.a + value.b*value.b + value.c*value.c);
        }

        double dot(const position_t &left, const position_t &right) {
            return left.x*right.x + left.y*right.y + left.z*right.z
                + left.a*right.a + left.b*right.b + left.c*right.c;
        }

        position_t scaled(const position_t &value, const double scale) {
            return { value.x*scale, value.y*scale, value.z*scale,
                     value.a*scale, value.b*scale, value.c*scale };
        }

        position_t evaluate(const AxisPolynomialSpan &span, const double u) {
            const auto component = [&](const double position_t::*member) {
                return ((span.a.*member * u + span.b.*member) * u + span.c.*member) * u + span.d.*member;
            };
            return { component(&position_t::x), component(&position_t::y), component(&position_t::z),
                     component(&position_t::a), component(&position_t::b), component(&position_t::c) };
        }

        position_t derivative(const AxisPolynomialSpan &span, const double u) {
            const auto component = [&](const double position_t::*member) {
                return (3.0 * span.a.*member * u * u + 2.0 * span.b.*member * u + span.c.*member)
                    * span.inverseDuration;
            };
            return { component(&position_t::x), component(&position_t::y), component(&position_t::z),
                     component(&position_t::a), component(&position_t::b), component(&position_t::c) };
        }

        position_t secondDerivative(const AxisPolynomialSpan &span, const double u) {
            const auto component = [&](const double position_t::*member) {
                return (6.0 * span.a.*member * u + 2.0 * span.b.*member) * span.inverseDurationSquared;
            };
            return { component(&position_t::x), component(&position_t::y), component(&position_t::z),
                     component(&position_t::a), component(&position_t::b), component(&position_t::c) };
        }
    }

    class MockMotionBackend::Impl {
        struct PlanSlot {
            std::atomic<bool> occupied{false};
            ExecutionItem item{};
        };
        std::array<PlanSlot, PLAN_CAPACITY> m_planSlots;
        SpscChannel<std::uint8_t, PLAN_CAPACITY> m_plans;
        SpscChannel<ControlRequest, CONTROL_CAPACITY> m_controls;
        SpscChannel<ExecutionEvent, EVENT_CAPACITY> m_events;
        SpscChannel<ExecutionSnapshot, SNAPSHOT_CAPACITY> m_snapshots;
        SpscChannel<SyntheticInputTransition, SYNTHETIC_INPUT_CAPACITY> m_syntheticInputs;
        FixedArray<SyntheticInputTransition, SYNTHETIC_INPUT_CAPACITY> m_pendingSyntheticInputs;
        ExecutionSnapshot m_snapshot;
        std::optional<std::uint8_t> m_active;
        bool m_stopping = false;
        std::uint32_t m_span = 0;
        std::uint32_t m_nextEvent = 0;
        double m_spanElapsed = 0.0;
        MockTrajectorySnapshot m_trajectoryDiagnostics;
        struct TriggeredRuntime {
            position_t start{};
            position_t direction{};
            double length = 0.0;
            double elapsed = 0.0;
            bool stopping = false;
            TriggeredMoveStatus completionStatus = TriggeredMoveStatus::ReachedTarget;
            MotionState triggerState{};
            ruckig::Trajectory<1> trajectory;
        } m_triggered;

    public:
        PublishResult publish(const ExecutionItem &item) noexcept {
            const auto valid = std::visit([](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::same_as<T, PlanChunk>)
                    return value.normalMotion.size != 0 && value.stopTail.size != 0
                        && value.epoch != 0 && value.id != 0;
                else
                    return value.epoch != 0 && value.id != 0 && value.moveId != 0
                        && magnitude(value.target) < std::numeric_limits<double>::infinity()
                        && magnitude(value.limits.velocity) > 0.0
                        && magnitude(value.limits.acceleration) > 0.0
                        && magnitude(value.limits.jerk) > 0.0;
            }, item);
            if(!valid) return PublishResult::Invalid;
            for(std::uint8_t index = 0; index < m_planSlots.size(); ++index) {
                bool expected = false;
                if(!m_planSlots[index].occupied.compare_exchange_strong(expected, true, std::memory_order_acquire)) continue;
                m_planSlots[index].item = item;
                if(m_plans.tryPush(index)) return PublishResult::Published;
                m_planSlots[index].occupied.store(false, std::memory_order_release);
                return PublishResult::Full;
            }
            return PublishResult::Full;
        }

        SubmitResult submit(const ControlRequest &request) noexcept {
            return m_controls.tryPush(request) ? SubmitResult::Submitted : SubmitResult::Full;
        }

        bool takeEvent(ExecutionEvent &event) noexcept { return m_events.tryPop(event); }
        bool takeSnapshot(ExecutionSnapshot &snapshot) noexcept { return m_snapshots.tryPop(snapshot); }
        bool configureSyntheticInput(const SyntheticInputTransition &input) noexcept {
            return m_syntheticInputs.tryPush(input);
        }
        void clearDiagnostics() { m_trajectoryDiagnostics = {}; }
        MockTrajectorySnapshot diagnostics() const { return m_trajectoryDiagnostics; }
        void advance(double seconds, const bool shouldPublishSnapshot = true) {
            serviceControls();
            SyntheticInputTransition input;
            while(m_syntheticInputs.tryPop(input)) {
                bool replaced = false;
                for(auto &pending : m_pendingSyntheticInputs) if(pending.move == input.move) {
                    pending = input; replaced = true; break;
                }
                if(!replaced) (void)m_pendingSyntheticInputs.push(input);
            }
            seconds = std::max(seconds, 0.0);
            if(m_snapshot.state != BackendState::Running) {
                if(shouldPublishSnapshot) publishSnapshot();
                return;
            }
            if(!m_active) activateNext();
            const auto executedActiveMotion = m_active.has_value();
            while(m_active && seconds > 0.0) {
                if(std::holds_alternative<TriggeredMove>(activeItem())) {
                    advanceTriggered(seconds);
                    continue;
                }
                const auto &span = currentSpan();
                const auto remaining = std::max(span.duration - m_spanElapsed, 0.0);
                const auto consumed = std::min(seconds, remaining);
                seconds -= consumed;
                m_spanElapsed += consumed;
                const auto u = span.duration > 0.0 ? std::clamp(m_spanElapsed * span.inverseDuration, 0.0, 1.0) : 1.0;
                m_snapshot.activeSpan = span.id;
                m_snapshot.spanProgress = u;
                m_snapshot.commanded = { evaluate(span, u), derivative(span, u), secondDerivative(span, u) };
                m_snapshot.feedback = m_snapshot.commanded;
                recordCalculatedPosition();
                if(m_spanElapsed + 1e-12 < span.duration) break;
                completeSpan();
            }
            // Always publish terminal/held state even inside a decimated batch so
            // NRT cannot observe completion before its matching final snapshot.
            if(shouldPublishSnapshot || (executedActiveMotion && !m_active)
               || m_snapshot.state != BackendState::Running)
                publishSnapshot();
        }

        void runUntilIdle() {
            runUntilIdle(3600.0);
        }

        void runUntilIdle(const double tickSeconds) {
            const auto step = tickSeconds > 0.0 ? tickSeconds : 3600.0;
            for(std::size_t guard = 0; guard < 100000000 && m_snapshot.state != BackendState::Faulted
                && (m_active || !m_plans.empty() || !m_controls.empty()); ++guard) {
                advance(step, false);
                // STOP is irrevocable. Queued descendants are now stale and may
                // remain in the forward channel until NRT observes HELD and sends
                // the recovery reset. Spinning on them cannot make progress.
                if(m_snapshot.state == BackendState::Held && m_controls.empty()) break;
            }
        }

    private:
        ExecutionItem &activeItem() { return m_planSlots[*m_active].item; }
        const ExecutionItem &activeItem() const { return m_planSlots[*m_active].item; }
        PlanChunk &activeChunk() { return std::get<PlanChunk>(activeItem()); }
        const PlanChunk &activeChunk() const { return std::get<PlanChunk>(activeItem()); }
        static EpochId itemEpoch(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.epoch; }, item);
        }
        static ChunkId itemId(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.id; }, item);
        }
        static BranchSequence itemPredecessor(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.predecessorBranch; }, item);
        }
        static BranchSequence itemBranch(const ExecutionItem &item) {
            return std::visit([](const auto &value) { return value.branch; }, item);
        }

        const AxisPolynomialSpan &currentSpan() const {
            const auto &chunk = activeChunk();
            return m_stopping ? chunk.stopTail[m_span] : chunk.normalMotion[m_span];
        }
        void release(const std::uint8_t index) { m_planSlots[index].occupied.store(false, std::memory_order_release); }

        void publishSnapshot() { (void)m_snapshots.tryPush(m_snapshot); }

        MotionState triggeredStateAt(const double elapsed, const position_t &origin) const {
            double position = 0.0, velocity = 0.0, acceleration = 0.0;
            m_triggered.trajectory.at_time(elapsed, position, velocity, acceleration);
            return { origin + scaled(m_triggered.direction, position),
                     scaled(m_triggered.direction, velocity),
                     scaled(m_triggered.direction, acceleration) };
        }

        bool initializeTriggered() {
            const auto &move = std::get<TriggeredMove>(activeItem());
            m_triggered = {};
            m_triggered.start = m_snapshot.commanded.position;
            const auto delta = move.target - m_triggered.start;
            m_triggered.length = magnitude(delta);
            if(m_triggered.length <= 1e-12) {
                m_triggered.direction = {};
                return true;
            }
            m_triggered.direction = scaled(delta, 1.0 / m_triggered.length);
            const auto velocityLimit = magnitude(move.limits.velocity);
            const auto accelerationLimit = magnitude(move.limits.acceleration);
            const auto jerkLimit = magnitude(move.limits.jerk);
            ruckig::InputParameter<1> input;
            input.current_position = {0.0};
            input.current_velocity = {dot(m_snapshot.commanded.velocity, m_triggered.direction)};
            input.current_acceleration = {dot(m_snapshot.commanded.acceleration, m_triggered.direction)};
            input.target_position = {m_triggered.length};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {velocityLimit};
            input.max_acceleration = {accelerationLimit};
            input.max_jerk = {jerkLimit};
            ruckig::Ruckig<1> generator;
            if(generator.calculate(input, m_triggered.trajectory) != ruckig::Result::Working) return false;
            return true;
        }

        std::optional<double> syntheticTransitionDistance(const TriggeredMoveId move) const {
            for(const auto &input : m_pendingSyntheticInputs) {
                if(input.move == move)
                    return dot(input.position - m_triggered.start, m_triggered.direction);
            }
            return std::nullopt;
        }

        void removeSyntheticInput(const TriggeredMoveId move) {
            for(std::uint32_t index = 0; index < m_pendingSyntheticInputs.size; ++index) {
                if(m_pendingSyntheticInputs[index].move != move) continue;
                m_pendingSyntheticInputs[index] = m_pendingSyntheticInputs[m_pendingSyntheticInputs.size - 1];
                --m_pendingSyntheticInputs.size;
                return;
            }
        }

        bool beginTriggeredStop(const TriggeredMoveStatus status) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            m_triggered.triggerState = m_snapshot.commanded;
            m_triggered.completionStatus = status;
            m_triggered.stopping = true;
            m_triggered.elapsed = 0.0;
            const auto scalarVelocity = dot(m_snapshot.commanded.velocity, m_triggered.direction);
            const auto scalarAcceleration = dot(m_snapshot.commanded.acceleration, m_triggered.direction);
            if(std::abs(scalarVelocity) <= 1e-12 && std::abs(scalarAcceleration) <= 1e-12) return true;
            ruckig::InputParameter<1> input;
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.current_position = {0.0};
            input.current_velocity = {scalarVelocity};
            input.current_acceleration = {scalarAcceleration};
            input.target_position = {0.0};
            input.target_velocity = {0.0};
            input.target_acceleration = {0.0};
            input.max_velocity = {std::max(magnitude(move.limits.velocity), std::abs(scalarVelocity))};
            input.max_acceleration = {magnitude(move.limits.acceleration)};
            input.max_jerk = {magnitude(move.limits.jerk)};
            ruckig::Ruckig<1> generator;
            return generator.calculate(input, m_triggered.trajectory) == ruckig::Result::Working;
        }

        void completeTriggered(const TriggeredMoveStatus status) {
            const auto move = std::get<TriggeredMove>(activeItem());
            const auto stopped = m_snapshot.commanded;
            const auto trigger = status == TriggeredMoveStatus::Triggered
                ? m_triggered.triggerState : stopped;
            emit(TriggeredMoveCompleted { move.epoch, move.moveId, status, trigger, stopped });
            emit(BranchSelected { move.epoch, move.branch, BranchChoice::Stop, 0 });
            emit(ChunkRetired { move.epoch, move.id });
            m_snapshot.state = BackendState::Held;
            m_snapshot.lastBranch = move.branch;
            emit(BackendHeld { move.epoch, stopped, move.cursor });
            removeSyntheticInput(move.moveId);
            release(*m_active);
            m_active.reset();
        }

        void faultTriggered() {
            m_snapshot.state = BackendState::Faulted;
            m_snapshot.faultCode = 2;
            emit(BackendFault { 2 });
        }

        void recordTriggeredPosition(const bool stopTail) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            if(m_trajectoryDiagnostics.spans.empty()
               || m_trajectoryDiagnostics.spans.back().epoch != move.epoch
               || m_trajectoryDiagnostics.spans.back().chunk != move.id
               || m_trajectoryDiagnostics.spans.back().stopTail != stopTail) {
                ExecutedTrajectorySpan executed { .epoch = move.epoch, .chunk = move.id,
                    .span = move.id, .stopTail = stopTail, .positions = {} };
                executed.positions.push_back(stopTail
                    ? m_triggered.triggerState.position : m_triggered.start);
                m_trajectoryDiagnostics.spans.push_back(std::move(executed));
            }
            m_trajectoryDiagnostics.spans.back().positions.push_back(m_snapshot.commanded.position);
            ++m_trajectoryDiagnostics.revision;
        }

        void advanceTriggered(double &seconds) {
            const auto &move = std::get<TriggeredMove>(activeItem());
            if(m_triggered.length <= 1e-12 && !m_triggered.stopping) {
                m_snapshot.commanded.position = move.target;
                m_snapshot.commanded.velocity = {};
                m_snapshot.commanded.acceleration = {};
                m_snapshot.feedback = m_snapshot.commanded;
                completeTriggered(TriggeredMoveStatus::ReachedTarget);
                return;
            }
            const auto duration = m_triggered.trajectory.get_duration();
            const auto consumed = std::min(seconds, std::max(duration - m_triggered.elapsed, 0.0));
            m_triggered.elapsed += consumed;
            const auto origin = m_triggered.stopping
                ? m_triggered.triggerState.position : m_triggered.start;
            m_snapshot.commanded = triggeredStateAt(m_triggered.elapsed, origin);
            m_snapshot.feedback = m_snapshot.commanded;
            m_snapshot.spanProgress = duration > 0.0
                ? std::clamp(m_triggered.elapsed / duration, 0.0, 1.0) : 1.0;
            recordTriggeredPosition(m_triggered.stopping);
            seconds -= consumed;
            if(!m_triggered.stopping) {
                const auto transition = syntheticTransitionDistance(move.moveId);
                const auto progress = dot(m_snapshot.commanded.position - m_triggered.start,
                                          m_triggered.direction);
                if(transition && progress + 1e-12 >= *transition) {
                    if(!beginTriggeredStop(TriggeredMoveStatus::Triggered)) { faultTriggered(); return; }
                    if(m_triggered.trajectory.get_duration() <= 1e-12)
                        completeTriggered(TriggeredMoveStatus::Triggered);
                    return;
                }
                if(m_triggered.elapsed + 1e-12 >= duration) {
                    m_snapshot.commanded.position = move.target;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    completeTriggered(TriggeredMoveStatus::ReachedTarget);
                }
            } else if(m_triggered.elapsed + 1e-12 >= duration) {
                m_snapshot.commanded.velocity = {};
                m_snapshot.commanded.acceleration = {};
                m_snapshot.feedback = m_snapshot.commanded;
                completeTriggered(m_triggered.completionStatus);
            }
        }

        void emit(const ExecutionEvent &event) {
            if(!m_events.tryPush(event)) {
                m_snapshot.state = BackendState::Faulted;
                m_snapshot.faultCode = 1; // Backend-to-NRT event overflow.
            }
        }

        void serviceControls() {
            ControlRequest request;
            while(m_controls.tryPop(request)) {
                std::visit([&](const auto &value) {
                    using T = std::decay_t<decltype(value)>;
                    bool success = true;
                    if constexpr(std::same_as<T, EnableRequest>) m_snapshot.state = BackendState::Held;
                    else if constexpr(std::same_as<T, DisableRequest>) {
                        if(m_active) release(*m_active);
                        m_active.reset(); m_snapshot.state = BackendState::Disabled;
                    } else if constexpr(std::same_as<T, StartRequest> || std::same_as<T, ResumeRequest>) {
                        m_snapshot.epoch = value.epoch; m_snapshot.state = BackendState::Running;
                    } else if constexpr(std::same_as<T, FeedHoldRequest>) {
                        if(m_active) { m_stopping = true; m_span = 0; m_spanElapsed = 0.0; }
                        else m_snapshot.state = BackendState::Held;
                    } else if constexpr(std::same_as<T, AbortRequest>) {
                        if(m_active) release(*m_active);
                        m_active.reset(); m_snapshot.state = BackendState::Held;
                    } else if constexpr(std::same_as<T, ResetRequest>) {
                        if(m_active) release(*m_active);
                        m_active.reset();
                        std::uint8_t discarded;
                        while(m_plans.tryPop(discarded)) release(discarded);
                        m_stopping = false; m_span = 0; m_nextEvent = 0; m_spanElapsed = 0.0;
                        m_snapshot = {}; m_snapshot.epoch = value.nextEpoch;
                    }
                    emit(RequestCompleted { value.id, success });
                }, request);
            }
        }

        void activateNext() {
            std::uint8_t index;
            if(!m_plans.tryPop(index)) return;
            const auto &item = m_planSlots[index].item;
            if(itemEpoch(item) != m_snapshot.epoch) {
                emit(ChunkRejected { itemEpoch(item), itemId(item) });
                release(index);
                return;
            }
            m_active = index;
            m_stopping = false;
            m_span = 0;
            m_nextEvent = 0;
            m_spanElapsed = 0.0;
            m_snapshot.activeChunk = itemId(item);
            m_snapshot.activeSpan = 0;
            emit(ChunkAccepted { itemEpoch(item), itemId(item) });
            if(std::holds_alternative<TriggeredMove>(item) && !initializeTriggered()) faultTriggered();
        }

        void completeSpan() {
            m_spanElapsed = 0.0;
            ++m_span;
            const auto count = m_stopping ? activeChunk().stopTail.size : activeChunk().normalMotion.size;
            if(m_span < count) return;
            if(m_stopping) {
                auto &chunk = activeChunk();
                emit(ChunkRetired { chunk.epoch, chunk.id });
                m_snapshot.state = BackendState::Held;
                m_snapshot.lastBranch = chunk.branch;
                emit(BackendHeld { chunk.epoch, chunk.stopState, chunk.stopCursor });
                release(*m_active);
                m_active.reset();
                return;
            }

            std::uint8_t continuationIndex = 0;
            const auto hasContinuation = m_plans.tryPop(continuationIndex);
            const auto &current = activeChunk();
            if(hasContinuation
               && itemEpoch(m_planSlots[continuationIndex].item) == current.epoch
               && itemPredecessor(m_planSlots[continuationIndex].item) == current.branch) {
                const auto oldIndex = *m_active;
                const auto &continuation = m_planSlots[continuationIndex].item;
                emit(BranchSelected { current.epoch, current.branch, BranchChoice::Continue, itemId(continuation) });
                emit(ChunkRetired { current.epoch, current.id });
                m_active = continuationIndex;
                release(oldIndex);
                m_span = 0;
                m_nextEvent = 0;
                m_spanElapsed = 0.0;
                m_snapshot.activeChunk = itemId(continuation);
                m_snapshot.activeSpan = 0;
                emit(ChunkAccepted { itemEpoch(continuation), itemId(continuation) });
                if(std::holds_alternative<TriggeredMove>(continuation) && !initializeTriggered()) faultTriggered();
            } else {
                if(hasContinuation) {
                    const auto &continuation = m_planSlots[continuationIndex].item;
                    emit(ChunkRejected { itemEpoch(continuation), itemId(continuation) });
                    release(continuationIndex);
                }
                emit(BranchSelected { current.epoch, current.branch, BranchChoice::Stop, 0 });
                m_stopping = true;
                m_span = 0;
            }
        }

        void recordCalculatedPosition() {
            const auto &chunk = activeChunk();
            const auto stopTail = m_stopping;
            const auto spanId = currentSpan().id;
            if(m_trajectoryDiagnostics.spans.empty()
               || m_trajectoryDiagnostics.spans.back().epoch != chunk.epoch
               || m_trajectoryDiagnostics.spans.back().chunk != chunk.id
               || m_trajectoryDiagnostics.spans.back().span != spanId
               || m_trajectoryDiagnostics.spans.back().stopTail != stopTail) {
                ExecutedTrajectorySpan executed {
                    .epoch = chunk.epoch,
                    .chunk = chunk.id,
                    .span = spanId,
                    .stopTail = stopTail,
                    .positions = {},
                };
                executed.positions.push_back(evaluate(currentSpan(), 0.0));
                m_trajectoryDiagnostics.spans.push_back(std::move(executed));
            }
            m_trajectoryDiagnostics.spans.back().positions.push_back(m_snapshot.commanded.position);
            ++m_trajectoryDiagnostics.revision;
        }
    };

    MockMotionBackend::MockMotionBackend() : m_impl(std::make_unique<Impl>()) { }
    MockMotionBackend::~MockMotionBackend() = default;
    PublishResult MockMotionBackend::tryPublish(const ExecutionItem &item) noexcept { return m_impl->publish(item); }
    SubmitResult MockMotionBackend::trySubmit(const ControlRequest &request) noexcept { return m_impl->submit(request); }
    bool MockMotionBackend::tryTakeEvent(ExecutionEvent &event) noexcept { return m_impl->takeEvent(event); }
    bool MockMotionBackend::tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept { return m_impl->takeSnapshot(snapshot); }
    void MockMotionBackend::advance(const double seconds) { m_impl->advance(seconds); }
    void MockMotionBackend::advanceTick(const double seconds, const bool publishSnapshot) {
        m_impl->advance(seconds, publishSnapshot);
    }
    void MockMotionBackend::runUntilIdle() { m_impl->runUntilIdle(); }
    void MockMotionBackend::runUntilIdle(const double tickSeconds) { m_impl->runUntilIdle(tickSeconds); }
    bool MockMotionBackend::configureSyntheticInput(const TriggeredMoveId move,
                                                    const position_t &transitionPosition) noexcept {
        return m_impl->configureSyntheticInput({ move, transitionPosition });
    }
    void MockMotionBackend::clearTrajectoryDiagnostics() { m_impl->clearDiagnostics(); }
    MockTrajectorySnapshot MockMotionBackend::trajectorySnapshot() const { return m_impl->diagnostics(); }
}
