#include "machine/MockMotionBackend.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <type_traits>

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
        constexpr std::size_t PROBE_CONFIG_CAPACITY = 16;

        struct SyntheticProbeConfig {
            std::uint64_t probeId = 0;
            position_t physicalToolOffset{};
            position_t activeToolOffset{};
        };

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
            PlanChunk chunk{};
        };
        std::array<PlanSlot, PLAN_CAPACITY> m_planSlots;
        SpscChannel<std::uint8_t, PLAN_CAPACITY> m_plans;
        SpscChannel<ControlRequest, CONTROL_CAPACITY> m_controls;
        SpscChannel<ExecutionEvent, EVENT_CAPACITY> m_events;
        SpscChannel<ExecutionSnapshot, SNAPSHOT_CAPACITY> m_snapshots;
        SpscChannel<SyntheticProbeConfig, PROBE_CONFIG_CAPACITY> m_probeConfigs;
        FixedArray<SyntheticProbeConfig, PROBE_CONFIG_CAPACITY> m_pendingProbeConfigs;
        ExecutionSnapshot m_snapshot;
        std::optional<std::uint8_t> m_active;
        bool m_stopping = false;
        std::uint32_t m_span = 0;
        std::uint32_t m_nextEvent = 0;
        double m_spanElapsed = 0.0;
        double m_playbackRate = 1.0;
        MockTrajectorySnapshot m_trajectoryDiagnostics;

    public:
        PublishResult publish(const PlanChunk &chunk) noexcept {
            if(chunk.normalMotion.size == 0 || chunk.stopTail.size == 0 || chunk.epoch == 0 || chunk.id == 0)
                return PublishResult::Invalid;
            for(std::uint8_t index = 0; index < m_planSlots.size(); ++index) {
                bool expected = false;
                if(!m_planSlots[index].occupied.compare_exchange_strong(expected, true, std::memory_order_acquire)) continue;
                m_planSlots[index].chunk = chunk;
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
        bool configureProbe(const SyntheticProbeConfig &config) noexcept { return m_probeConfigs.tryPush(config); }
        void clearDiagnostics() { m_trajectoryDiagnostics = {}; }
        MockTrajectorySnapshot diagnostics() const { return m_trajectoryDiagnostics; }
        void setPlaybackRate(const double rate) { m_playbackRate = std::clamp(rate, 0.0, 1000.0); }

        void advance(double seconds) {
            serviceControls();
            SyntheticProbeConfig config;
            while(m_probeConfigs.tryPop(config)) {
                bool replaced = false;
                for(auto &pending : m_pendingProbeConfigs) if(pending.probeId == config.probeId) { pending = config; replaced = true; break; }
                if(!replaced) (void)m_pendingProbeConfigs.push(config);
            }
            seconds = std::max(seconds, 0.0) * m_playbackRate;
            if(m_snapshot.state != BackendState::Running) {
                publishSnapshot();
                return;
            }
            if(!m_active) activateNext();
            while(m_active && (seconds > 0.0 || currentSpan().duration == 0.0)) {
                const auto &span = currentSpan();
                const auto probe = activeProbeContact();
                const auto completionElapsed = probe ? span.duration * probe->parameter : span.duration;
                const auto remaining = std::max(completionElapsed - m_spanElapsed, 0.0);
                const auto consumed = std::min(seconds, remaining);
                seconds -= consumed;
                m_spanElapsed += consumed;
                const auto u = span.duration > 0.0 ? std::clamp(m_spanElapsed * span.inverseDuration, 0.0, 1.0) : 1.0;
                m_snapshot.activeSpan = span.id;
                m_snapshot.spanProgress = u;
                m_snapshot.commanded = { evaluate(span, u), derivative(span, u), secondDerivative(span, u) };
                m_snapshot.feedback = m_snapshot.commanded;
                if(m_spanElapsed + 1e-12 < completionElapsed) break;
                completeSpan();
            }
            publishSnapshot();
        }

        void runUntilIdle() {
            for(std::size_t guard = 0; guard < 100000 && m_snapshot.state != BackendState::Faulted
                && (m_active || !m_plans.empty() || !m_controls.empty()); ++guard) {
                advance(3600.0);
                // STOP is irrevocable. Queued descendants are now stale and may
                // remain in the forward channel until NRT observes HELD and sends
                // the recovery reset. Spinning on them cannot make progress.
                if(m_snapshot.state == BackendState::Held && m_controls.empty()) break;
            }
        }

    private:
        struct ActiveProbeContact {
            std::uint64_t probeId = 0;
            position_t position{};
            double parameter = 1.0;
        };

        const AxisPolynomialSpan &currentSpan() const {
            const auto &chunk = m_planSlots[*m_active].chunk;
            return m_stopping ? chunk.stopTail[m_span] : chunk.normalMotion[m_span];
        }

        PlanChunk &activeChunk() { return m_planSlots[*m_active].chunk; }
        const PlanChunk &activeChunk() const { return m_planSlots[*m_active].chunk; }
        void release(const std::uint8_t index) { m_planSlots[index].occupied.store(false, std::memory_order_release); }

        void publishSnapshot() { (void)m_snapshots.tryPush(m_snapshot); }

        std::optional<ActiveProbeContact> activeProbeContact() const {
            if(m_stopping || !m_active) return std::nullopt;
            const auto &chunk = activeChunk();
            for(std::uint32_t event = 0; event < chunk.events.size; ++event) {
                if(chunk.events[event].span > m_span) continue;
                const auto *probe = std::get_if<ProbeEvent>(&chunk.events[event].value);
                if(!probe) continue;
                auto contact = evaluate(chunk.normalMotion[chunk.normalMotion.size - 1], 1.0);
                for(const auto &config : m_pendingProbeConfigs) {
                    if(config.probeId != probe->probeId) continue;
                    contact = contact + config.physicalToolOffset - config.activeToolOffset;
                    break;
                }
                const auto parameter = closestParameter(currentSpan(), contact);
                const auto nearest = evaluate(currentSpan(), parameter);
                const auto delta = nearest - contact;
                const auto distanceSquared = delta.x*delta.x + delta.y*delta.y + delta.z*delta.z
                    + delta.a*delta.a + delta.b*delta.b + delta.c*delta.c;
                if(distanceSquared <= 1e-12)
                    return ActiveProbeContact { probe->probeId, contact, parameter };
            }
            return std::nullopt;
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
            const auto &chunk = m_planSlots[index].chunk;
            if(chunk.epoch != m_snapshot.epoch) {
                emit(ChunkRejected { chunk.epoch, chunk.id });
                release(index);
                return;
            }
            m_active = index;
            m_stopping = false;
            m_span = 0;
            m_nextEvent = 0;
            m_spanElapsed = 0.0;
            m_snapshot.activeChunk = chunk.id;
            emit(ChunkAccepted { chunk.epoch, chunk.id });
        }

        void completeSpan() {
            m_spanElapsed = 0.0;
            bool probeStopped = false;
            double executedUntil = 1.0;
            auto terminalPosition = m_snapshot.commanded.position;
            const auto triggeredProbe = activeProbeContact();
            if(!m_stopping) {
                auto &chunk = activeChunk();
                while(m_nextEvent < chunk.events.size && chunk.events[m_nextEvent].span <= m_span) {
                    if(const auto probe = std::get_if<ProbeEvent>(&chunk.events[m_nextEvent].value)) {
                        if(!triggeredProbe || triggeredProbe->probeId != probe->probeId) break;
                        auto contact = triggeredProbe && triggeredProbe->probeId == probe->probeId
                            ? triggeredProbe->position : m_snapshot.commanded.position;
                        for(std::uint32_t index = 0; index < m_pendingProbeConfigs.size; ++index) {
                            if(m_pendingProbeConfigs[index].probeId != probe->probeId) continue;
                            m_pendingProbeConfigs[index] = m_pendingProbeConfigs[m_pendingProbeConfigs.size - 1];
                            --m_pendingProbeConfigs.size;
                            break;
                        }
                        m_snapshot.commanded.position = contact;
                        m_snapshot.commanded.velocity = {};
                        m_snapshot.commanded.acceleration = {};
                        m_snapshot.feedback = m_snapshot.commanded;
                        executedUntil = triggeredProbe ? triggeredProbe->parameter : closestParameter(currentSpan(), contact);
                        terminalPosition = contact;
                        emit(ProbeCompleted { chunk.epoch, ProbeResult { probe->probeId, ProbeStatus::Triggered,
                                                                           contact, contact } });
                        probeStopped = true;
                    }
                    ++m_nextEvent;
                }
            }
            recordCurrentSpan(executedUntil, terminalPosition);
            if(probeStopped) {
                auto &chunk = activeChunk();
                emit(BranchSelected { chunk.epoch, chunk.branch, BranchChoice::Stop, 0 });
                emit(ChunkRetired { chunk.epoch, chunk.id });
                m_snapshot.state = BackendState::Held;
                m_snapshot.lastBranch = chunk.branch;
                auto held = chunk.stopState;
                held.position = m_snapshot.commanded.position;
                held.velocity = {};
                held.acceleration = {};
                emit(BackendHeld { chunk.epoch, held, chunk.stopCursor });
                release(*m_active);
                m_active.reset();
                return;
            }
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
               && m_planSlots[continuationIndex].chunk.epoch == current.epoch
               && m_planSlots[continuationIndex].chunk.predecessorBranch == current.branch) {
                const auto oldIndex = *m_active;
                const auto &continuation = m_planSlots[continuationIndex].chunk;
                emit(BranchSelected { current.epoch, current.branch, BranchChoice::Continue, continuation.id });
                emit(ChunkRetired { current.epoch, current.id });
                m_active = continuationIndex;
                release(oldIndex);
                m_span = 0;
                m_nextEvent = 0;
                m_snapshot.activeChunk = continuation.id;
                emit(ChunkAccepted { continuation.epoch, continuation.id });
            } else {
                if(hasContinuation) {
                    const auto &continuation = m_planSlots[continuationIndex].chunk;
                    emit(ChunkRejected { continuation.epoch, continuation.id });
                    release(continuationIndex);
                }
                emit(BranchSelected { current.epoch, current.branch, BranchChoice::Stop, 0 });
                m_stopping = true;
                m_span = 0;
            }
        }

        static double closestParameter(const AxisPolynomialSpan &span, const position_t &target) {
            const auto distanceSquared = [&](const double u) {
                const auto delta = evaluate(span, u) - target;
                return delta.x*delta.x + delta.y*delta.y + delta.z*delta.z
                    + delta.a*delta.a + delta.b*delta.b + delta.c*delta.c;
            };
            double best = 0.0;
            double bestDistance = distanceSquared(0.0);
            constexpr int SAMPLES = 64;
            for(int sample = 1; sample <= SAMPLES; ++sample) {
                const auto u = static_cast<double>(sample) / SAMPLES;
                const auto distance = distanceSquared(u);
                if(distance < bestDistance) { best = u; bestDistance = distance; }
            }
            auto low = std::max(0.0, best - 1.0/SAMPLES);
            auto high = std::min(1.0, best + 1.0/SAMPLES);
            for(int iteration = 0; iteration < 24; ++iteration) {
                const auto left = std::lerp(low, high, 1.0/3.0);
                const auto right = std::lerp(low, high, 2.0/3.0);
                if(distanceSquared(left) <= distanceSquared(right)) high = right;
                else low = left;
            }
            return 0.5*(low+high);
        }

        void recordCurrentSpan(const double executedUntil, const position_t &terminalPosition) {
            const auto &chunk = activeChunk();
            m_trajectoryDiagnostics.spans.push_back({
                .epoch = chunk.epoch,
                .chunk = chunk.id,
                .span = currentSpan().id,
                .stopTail = m_stopping,
                .executedUntil = executedUntil,
                .polynomial = currentSpan(),
                .terminalPosition = terminalPosition,
            });
            ++m_trajectoryDiagnostics.revision;
        }
    };

    MockMotionBackend::MockMotionBackend() : m_impl(std::make_unique<Impl>()) { }
    MockMotionBackend::~MockMotionBackend() = default;
    PublishResult MockMotionBackend::tryPublish(const PlanChunk &chunk) noexcept { return m_impl->publish(chunk); }
    SubmitResult MockMotionBackend::trySubmit(const ControlRequest &request) noexcept { return m_impl->submit(request); }
    bool MockMotionBackend::tryTakeEvent(ExecutionEvent &event) noexcept { return m_impl->takeEvent(event); }
    bool MockMotionBackend::tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept { return m_impl->takeSnapshot(snapshot); }
    void MockMotionBackend::advance(const double seconds) { m_impl->advance(seconds); }
    void MockMotionBackend::runUntilIdle() { m_impl->runUntilIdle(); }
    void MockMotionBackend::setPlaybackRate(const double rate) { m_impl->setPlaybackRate(rate); }
    bool MockMotionBackend::configureSyntheticProbe(const std::uint64_t probeId,
                                                     const position_t &physicalToolOffset,
                                                     const position_t &activeToolOffset) noexcept {
        return m_impl->configureProbe({ probeId, physicalToolOffset, activeToolOffset });
    }
    void MockMotionBackend::clearTrajectoryDiagnostics() { m_impl->clearDiagnostics(); }
    MockTrajectorySnapshot MockMotionBackend::trajectorySnapshot() const { return m_impl->diagnostics(); }
}
