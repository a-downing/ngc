#include "machine/ProductionExecutorCore.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

namespace ngc {
    namespace {
        constexpr std::uint32_t EVENT_OVERFLOW_FAULT = 1;
        constexpr std::uint32_t INVALID_EXECUTION_STATE_FAULT = 2;

        bool zeroHighCubicControls(const AxisPolynomialSpan &span) noexcept {
            return span.coefficients[3].length() == 0.0
                && span.coefficients[4].length() == 0.0;
        }

        bool finitePosition(const position_t &position) noexcept {
            return std::isfinite(position.x) && std::isfinite(position.y)
                && std::isfinite(position.z) && std::isfinite(position.a)
                && std::isfinite(position.b) && std::isfinite(position.c);
        }

        bool finiteMotionState(const MotionState &state) noexcept {
            return finitePosition(state.position)
                && finitePosition(state.velocity)
                && finitePosition(state.acceleration);
        }

        bool approximatelyEqual(const double actual,
                                const double expected) noexcept {
            return std::abs(actual - expected)
                <= 1e-12 * std::max(1.0, std::abs(expected));
        }
    }

    ProductionExecutorCore::ProductionExecutorCore(const double servoPeriod)
        : m_servoPeriod(servoPeriod) {
        if (!std::isfinite(servoPeriod) || servoPeriod <= 0.0) {
            throw std::invalid_argument(
                "production executor servo period must be finite and positive");
        }
    }

    PublishResult ProductionExecutorCore::tryPublish(
        const ExecutionItem &item) noexcept {
        const auto *chunk = std::get_if<PlanChunk>(&item);
        if (chunk == nullptr || !validPlanChunk(*chunk)) {
            return PublishResult::Invalid;
        }

        for (std::uint8_t index = 0; index < m_planSlots.size(); ++index) {
            auto expected = false;
            if (!m_planSlots[index].occupied.compare_exchange_strong(
                    expected, true, std::memory_order_acquire)) {
                continue;
            }

            auto &slot = m_planSlots[index];
            slot.chunk = *chunk;
            slot.normalMotionNanoseconds = normalMotionNanoseconds(*chunk);
            m_queuedNormalMotionNanoseconds.fetch_add(
                slot.normalMotionNanoseconds, std::memory_order_release);
            m_queuedExecutionItems.fetch_add(1, std::memory_order_release);
            if (m_plans.tryPush(index)) {
                return PublishResult::Published;
            }

            m_queuedNormalMotionNanoseconds.fetch_sub(
                slot.normalMotionNanoseconds, std::memory_order_acq_rel);
            m_queuedExecutionItems.fetch_sub(1, std::memory_order_acq_rel);
            slot.occupied.store(false, std::memory_order_release);

            return PublishResult::Full;
        }

        return PublishResult::Full;
    }

    SubmitResult ProductionExecutorCore::trySubmit(
        const ControlRequest &request) noexcept {
        return m_controls.tryPush(request)
            ? SubmitResult::Submitted : SubmitResult::Full;
    }

    bool ProductionExecutorCore::tryTakeEvent(ExecutionEvent &event) noexcept {
        return m_events.tryPop(event);
    }

    bool ProductionExecutorCore::tryTakeSnapshot(
        ExecutionSnapshot &snapshot) noexcept {
        return m_snapshots.tryPop(snapshot);
    }

    void ProductionExecutorCore::restoreStationaryState(
        const MotionState &commanded, const MotionState &feedback) noexcept {
        discardExecution();

        ControlRequest control;
        while (m_controls.tryPop(control)) { }
        ExecutionEvent event;
        while (m_events.tryPop(event)) { }
        ExecutionSnapshot snapshot;
        while (m_snapshots.tryPop(snapshot)) { }

        m_snapshot = {};
        m_snapshot.state = BackendState::Disabled;
        m_snapshot.commanded = commanded;
        m_snapshot.feedback = feedback;
        m_faultEventEmitted = false;
    }

    void ProductionExecutorCore::servoTick(const bool shouldPublishSnapshot) noexcept {
        serviceControls();

        if (m_snapshot.state == BackendState::Running) {
            if (!m_active.has_value()) {
                activateNext();
            }
            if (m_active.has_value() && m_snapshot.state == BackendState::Running) {
                advanceActive(m_servoPeriod);
            }
        }

        if (shouldPublishSnapshot || m_snapshot.state == BackendState::Held
            || m_snapshot.state == BackendState::Faulted
            || m_snapshot.state == BackendState::Disabled) {
            publishSnapshot();
        }
    }

    double ProductionExecutorCore::servoPeriod() const noexcept {
        return m_servoPeriod;
    }

    bool ProductionExecutorCore::validPlanChunk(
        const PlanChunk &chunk) noexcept {
        if (chunk.epoch == 0 || chunk.id == 0 || chunk.branch == 0
            || chunk.normalMotion.size == 0 || chunk.stopTail.size == 0
            || chunk.events.size != 0
            || !finiteMotionState(chunk.branchState)
            || !finiteMotionState(chunk.stopState)) {
            return false;
        }

        const auto validSpan = [](const AxisPolynomialSpan &span) {
            if (span.id == 0
                || (span.degree != ExecutionPolynomialDegree::Cubic
                    && span.degree != ExecutionPolynomialDegree::Quintic)
                || !std::isfinite(span.duration) || span.duration <= 0.0
                || !finitePosition(span.origin)) {
                return false;
            }
            for (const auto &coefficient : span.coefficients) {
                if (!finitePosition(coefficient)) {
                    return false;
                }
            }

            const auto inverseDuration = 1.0 / span.duration;
            return std::isfinite(span.inverseDuration)
                && approximatelyEqual(
                    span.inverseDuration, inverseDuration)
                && std::isfinite(span.inverseDurationSquared)
                && approximatelyEqual(
                    span.inverseDurationSquared,
                    inverseDuration * inverseDuration)
                && std::isfinite(span.inverseDurationCubed)
                && approximatelyEqual(
                    span.inverseDurationCubed,
                    inverseDuration * inverseDuration * inverseDuration)
                && (span.degree != ExecutionPolynomialDegree::Cubic
                    || zeroHighCubicControls(span));
        };
        if (!std::ranges::all_of(chunk.normalMotion, validSpan)
            || !std::ranges::all_of(chunk.stopTail, validSpan)) {
            return false;
        }

        std::optional<std::pair<std::uint32_t, double>> previous;
        for (const auto &marker : chunk.markers) {
            const auto location = std::pair{marker.span, marker.parameter};
            if (marker.id == 0 || marker.span >= chunk.normalMotion.size
                || !std::isfinite(marker.parameter)
                || marker.parameter < 0.0 || marker.parameter > 1.0
                || (previous.has_value() && location < *previous)) {
                return false;
            }
            previous = location;
        }

        return true;
    }

    std::uint64_t ProductionExecutorCore::normalMotionNanoseconds(
        const PlanChunk &chunk) noexcept {
        auto seconds = 0.0;
        for (const auto &span : chunk.normalMotion) {
            seconds += std::max(span.duration, 0.0);
        }

        return secondsToNanoseconds(seconds);
    }

    std::uint64_t ProductionExecutorCore::secondsToNanoseconds(
        const double seconds) noexcept {
        constexpr auto scale = 1.0e9;
        if (!std::isfinite(seconds) || seconds <= 0.0) {
            return 0;
        }

        const auto maximum =
            static_cast<double>(std::numeric_limits<std::uint64_t>::max());
        if (seconds >= maximum / scale) {
            return std::numeric_limits<std::uint64_t>::max();
        }

        return static_cast<std::uint64_t>(std::llround(seconds * scale));
    }

    void ProductionExecutorCore::serviceControls() noexcept {
        ControlRequest request;
        while (m_controls.tryPop(request)) {
            std::visit([&](const auto &value) {
                using T = std::decay_t<decltype(value)>;
                auto success = false;
                if constexpr (std::same_as<T, EnableRequest>) {
                    success = m_snapshot.state == BackendState::Disabled
                        || m_snapshot.state == BackendState::Held;
                    if (success) {
                        m_snapshot.state = BackendState::Held;
                        m_snapshot.faultCode = 0;
                        m_faultEventEmitted = false;
                    }
                } else if constexpr (std::same_as<T, DisableRequest>) {
                    discardExecution();
                    m_snapshot.state = BackendState::Disabled;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    success = true;
                } else if constexpr (std::same_as<T, StartRequest>) {
                    success = m_snapshot.state == BackendState::Held
                        && value.epoch != 0 && value.epoch == m_snapshot.epoch;
                    if (success) {
                        m_snapshot.state = BackendState::Running;
                    }
                } else if constexpr (std::same_as<T, AbortRequest>) {
                    discardExecution();
                    m_snapshot.state = BackendState::Held;
                    m_snapshot.commanded.velocity = {};
                    m_snapshot.commanded.acceleration = {};
                    m_snapshot.feedback = m_snapshot.commanded;
                    success = true;
                } else if constexpr (std::same_as<T, ResetRequest>) {
                    discardExecution();
                    const auto commanded = m_snapshot.commanded;
                    const auto feedback = m_snapshot.feedback;
                    const auto commandedJoints = m_snapshot.commandedJoints;
                    const auto feedbackJoints = m_snapshot.feedbackJoints;
                    m_snapshot = {};
                    m_snapshot.epoch = value.nextEpoch;
                    m_snapshot.commanded = commanded;
                    m_snapshot.feedback = feedback;
                    m_snapshot.commandedJoints = commandedJoints;
                    m_snapshot.feedbackJoints = feedbackJoints;
                    m_faultEventEmitted = false;
                    success = value.nextEpoch != 0;
                }

                emit(RequestCompleted{value.id, success});
            }, request);
        }
    }

    void ProductionExecutorCore::activateNext() noexcept {
        std::uint8_t index = 0;
        if (!m_plans.tryPop(index)) {
            return;
        }
        accountForDequeued(index);

        const auto &chunk = m_planSlots[index].chunk;
        if (chunk.epoch != m_snapshot.epoch) {
            emit(ChunkRejected{chunk.epoch, chunk.id});
            release(index);

            return;
        }

        m_active = index;
        m_stopping = false;
        m_span = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
        m_snapshot.activeChunk = chunk.id;
        m_snapshot.activeSpan = 0;
        emit(ChunkAccepted{chunk.epoch, chunk.id});
        emitExecutionMarkersThrough(0.0);
    }

    void ProductionExecutorCore::advanceActive(double seconds) noexcept {
        while (m_active.has_value() && seconds > 0.0
               && m_snapshot.state == BackendState::Running) {
            const auto &span = currentSpan();
            const auto remaining =
                std::max(span.duration - m_spanElapsed, 0.0);
            const auto consumed = std::min(seconds, remaining);
            seconds -= consumed;
            m_spanElapsed += consumed;

            const auto parameter = std::clamp(
                m_spanElapsed * span.inverseDuration, 0.0, 1.0);
            m_snapshot.activeSpan = span.id;
            m_snapshot.spanProgress = parameter;
            m_snapshot.commanded =
                evaluateExecutionPolynomial(span, parameter).state;
            m_snapshot.feedback = m_snapshot.commanded;
            emitExecutionMarkersThrough(parameter);
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }

            if (m_spanElapsed + 1e-12 < span.duration) {
                return;
            }
            completeSpan();
        }
    }

    void ProductionExecutorCore::completeSpan() noexcept {
        m_spanElapsed = 0.0;
        ++m_span;
        const auto count = m_stopping
            ? activeChunk().stopTail.size : activeChunk().normalMotion.size;
        if (m_span < count) {
            emitExecutionMarkersThrough(0.0);

            return;
        }

        if (!m_stopping) {
            selectContinuationOrStop();

            return;
        }

        const auto chunk = activeChunk();
        m_snapshot.commanded = chunk.stopState;
        m_snapshot.feedback = m_snapshot.commanded;
        m_snapshot.lastBranch = chunk.branch;
        m_snapshot.state = BackendState::Held;
        emit(ChunkRetired{chunk.epoch, chunk.id});
        emit(BackendHeld{
            chunk.epoch, chunk.stopState, BackendHoldReason::StopBranch,
        });
        release(*m_active);
        m_active.reset();
    }

    void ProductionExecutorCore::selectContinuationOrStop() noexcept {
        std::uint8_t continuationIndex = 0;
        const auto hasContinuation = m_plans.tryPop(continuationIndex);
        if (hasContinuation) {
            accountForDequeued(continuationIndex);
        }

        const auto currentIndex = *m_active;
        const auto current = m_planSlots[currentIndex].chunk;
        if (hasContinuation) {
            const auto &continuation = m_planSlots[continuationIndex].chunk;
            if (continuation.epoch == current.epoch
                && continuation.predecessorBranch == current.branch) {
                emit(BranchSelected{
                    current.epoch, current.branch, BranchChoice::Continue,
                    continuation.id,
                });
                emit(ChunkRetired{current.epoch, current.id});
                m_active = continuationIndex;
                release(currentIndex);
                m_stopping = false;
                m_span = 0;
                m_nextMarker = 0;
                m_spanElapsed = 0.0;
                m_snapshot.activeChunk = continuation.id;
                m_snapshot.activeSpan = 0;
                emit(ChunkAccepted{continuation.epoch, continuation.id});
                emitExecutionMarkersThrough(0.0);

                return;
            }

            emit(ChunkRejected{continuation.epoch, continuation.id});
            release(continuationIndex);
        }

        emit(BranchSelected{
            current.epoch, current.branch, BranchChoice::Stop, 0,
        });
        m_stopping = true;
        m_span = 0;
        m_spanElapsed = 0.0;
    }

    void ProductionExecutorCore::emitExecutionMarkersThrough(
        const double parameter) noexcept {
        if (m_stopping || !m_active.has_value()) {
            return;
        }

        const auto &chunk = activeChunk();
        while (m_nextMarker < chunk.markers.size) {
            const auto &marker = chunk.markers[m_nextMarker];
            if (marker.span > m_span
                || (marker.span == m_span
                    && marker.parameter > parameter)) {
                return;
            }
            if (marker.span < m_span
                || marker.span >= chunk.normalMotion.size) {
                fault(INVALID_EXECUTION_STATE_FAULT);

                return;
            }

            emit(ExecutionMarkerReached{
                .epoch = chunk.epoch,
                .chunk = chunk.id,
                .marker = marker.id,
                .span = chunk.normalMotion[m_span].id,
                .parameter = marker.parameter,
            });
            ++m_nextMarker;
            if (m_snapshot.state == BackendState::Faulted) {
                return;
            }
        }
    }

    void ProductionExecutorCore::emit(
        const ExecutionEvent &event) noexcept {
        if (!m_events.tryPush(event)) {
            fault(EVENT_OVERFLOW_FAULT);
        }
    }

    void ProductionExecutorCore::fault(const std::uint32_t code) noexcept {
        m_snapshot.state = BackendState::Faulted;
        m_snapshot.faultCode = code;
        if (!m_faultEventEmitted) {
            m_faultEventEmitted = m_events.tryPush(BackendFault{code});
        }
    }

    void ProductionExecutorCore::discardExecution() noexcept {
        if (m_active.has_value()) {
            release(*m_active);
            m_active.reset();
        }

        std::uint8_t index = 0;
        while (m_plans.tryPop(index)) {
            accountForDequeued(index);
            release(index);
        }
        m_stopping = false;
        m_span = 0;
        m_nextMarker = 0;
        m_spanElapsed = 0.0;
    }

    void ProductionExecutorCore::release(const std::uint8_t index) noexcept {
        m_planSlots[index].occupied.store(false, std::memory_order_release);
    }

    void ProductionExecutorCore::accountForDequeued(
        const std::uint8_t index) noexcept {
        m_queuedNormalMotionNanoseconds.fetch_sub(
            m_planSlots[index].normalMotionNanoseconds,
            std::memory_order_acq_rel);
        m_queuedExecutionItems.fetch_sub(1, std::memory_order_acq_rel);
    }

    void ProductionExecutorCore::publishSnapshot() noexcept {
        refreshSnapshot();
        static_cast<void>(m_snapshots.tryPush(m_snapshot));
    }

    void ProductionExecutorCore::refreshSnapshot() noexcept {
        constexpr auto nanosecondsToSeconds = 1.0e-9;
        m_snapshot.activeNormalMotionRemainingSeconds = 0.0;
        m_snapshot.stopBranchRemainingSeconds = 0.0;
        m_snapshot.queuedNormalMotionSeconds =
            nanosecondsToSeconds * static_cast<double>(
                m_queuedNormalMotionNanoseconds.load(
                    std::memory_order_acquire));
        m_snapshot.queuedExecutionItems =
            m_queuedExecutionItems.load(std::memory_order_acquire);
        if (m_active.has_value()) {
            const auto &chunk = activeChunk();
            if (m_stopping) {
                m_snapshot.stopBranchRemainingSeconds =
                    remainingDuration(chunk.stopTail, m_span, m_spanElapsed);
            } else {
                m_snapshot.activeNormalMotionRemainingSeconds =
                    remainingDuration(
                        chunk.normalMotion, m_span, m_spanElapsed);
                m_snapshot.stopBranchRemainingSeconds =
                    remainingDuration(chunk.stopTail, 0, 0.0);
            }
        }
        m_snapshot.committedNormalMotionSeconds =
            m_snapshot.state == BackendState::Running && !m_stopping
            ? m_snapshot.activeNormalMotionRemainingSeconds
                + m_snapshot.queuedNormalMotionSeconds
            : 0.0;
    }

    PlanChunk &ProductionExecutorCore::activeChunk() noexcept {
        return m_planSlots[*m_active].chunk;
    }

    const PlanChunk &ProductionExecutorCore::activeChunk() const noexcept {
        return m_planSlots[*m_active].chunk;
    }

    const AxisPolynomialSpan &
    ProductionExecutorCore::currentSpan() const noexcept {
        const auto &chunk = activeChunk();

        return m_stopping
            ? chunk.stopTail[m_span] : chunk.normalMotion[m_span];
    }
}
