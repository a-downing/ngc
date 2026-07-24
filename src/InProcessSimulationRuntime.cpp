#include "machine/InProcessSimulationRuntime.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "WindowsServoPacer.h"

namespace ngc {
    InProcessSimulationRuntime::InProcessSimulationRuntime(const TrajectoryLimits &limits,
                                                           const SimulationTiming &timing)
        : m_backend({}, limits), m_servoPeriod(timing.servoPeriod),
          m_schedulerPeriod(timing.schedulerPeriod),
          m_servoTicksPerSchedulerPeriod(static_cast<std::uint32_t>(
              std::max(1.0, std::round(timing.schedulerPeriod / timing.servoPeriod)))) { }

    InProcessSimulationRuntime::InProcessSimulationRuntime(const MachineConfiguration &configuration)
        : m_backend(configuration.feedHold, configuration.trajectory,
                    configuration.axes, configuration.joints),
          m_servoPeriod(configuration.simulation.servoPeriod),
          m_schedulerPeriod(configuration.simulation.schedulerPeriod),
          m_servoTicksPerSchedulerPeriod(static_cast<std::uint32_t>(std::max(
              1.0, std::round(configuration.simulation.schedulerPeriod
                              / configuration.simulation.servoPeriod)))) { }

    InProcessSimulationRuntime::~InProcessSimulationRuntime() {
        stop();
    }

    MotionBackend &InProcessSimulationRuntime::endpoint() noexcept {
        return m_backend;
    }

    void InProcessSimulationRuntime::start() {
        std::scoped_lock lock(m_schedulerMutex);
        if (m_started) {
            return;
        }

        m_stopping.store(false, std::memory_order_release);
        m_started = true;
        m_schedulerThread = std::thread(&InProcessSimulationRuntime::runScheduler, this);
    }

    void InProcessSimulationRuntime::stop() {
        {
            std::scoped_lock lock(m_schedulerMutex);
            if (!m_started) {
                return;
            }
            m_stopping.store(true, std::memory_order_release);
            m_timedExecutionActive.store(false, std::memory_order_release);
            m_executorRefillRequested.store(false, std::memory_order_release);
            m_nrtRefillActive.store(false, std::memory_order_release);
            m_rollingSupplyActive.store(false, std::memory_order_release);
        }
        m_schedulerCv.notify_all();
        m_schedulerThread.join();

        std::scoped_lock lock(m_schedulerMutex);
        m_started = false;
        m_executorBatchActive.store(false, std::memory_order_release);
    }

    bool InProcessSimulationRuntime::beginTimedExecution() {
        {
            std::scoped_lock lock(m_schedulerMutex);
            if (!m_started || m_stopping.load(std::memory_order_acquire)
                || m_timedExecutionActive.load(std::memory_order_acquire)
                || m_pacingError.load(std::memory_order_acquire) != 0) {
                return false;
            }

            resetTimedDiagnostics();
            m_backend.clearTrajectoryDiagnostics();
            m_timedExecutionActive.store(true, std::memory_order_release);
        }
        m_schedulerCv.notify_all();

        return true;
    }

    void InProcessSimulationRuntime::endTimedExecution() {
        m_timedExecutionActive.store(false, std::memory_order_release);
        m_executorRefillRequested.store(false, std::memory_order_release);
        m_nrtRefillActive.store(false, std::memory_order_release);
        m_rollingSupplyActive.store(false, std::memory_order_release);
        m_schedulerCv.notify_all();

        while (m_executorBatchActive.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }

    bool InProcessSimulationRuntime::timedExecutionActive() const noexcept {
        return m_timedExecutionActive.load(std::memory_order_acquire);
    }

    void InProcessSimulationRuntime::setTickMultiplier(const int multiplier) noexcept {
        m_tickMultiplier.store(
            static_cast<std::uint32_t>(std::clamp(multiplier, 1, 1000)),
            std::memory_order_relaxed);
    }

    std::uint32_t InProcessSimulationRuntime::tickMultiplier() const noexcept {
        return m_tickMultiplier.load(std::memory_order_relaxed);
    }

    double InProcessSimulationRuntime::servoPeriod() const noexcept {
        return m_servoPeriod;
    }

    double InProcessSimulationRuntime::schedulerPeriod() const noexcept {
        return m_schedulerPeriod;
    }

    std::uint32_t InProcessSimulationRuntime::servoTicksPerSchedulerPeriod() const noexcept {
        return m_servoTicksPerSchedulerPeriod;
    }

    SimulationRuntimeSnapshot InProcessSimulationRuntime::snapshot() const noexcept {
        return {
            .servoPeriodSeconds = m_servoPeriod,
            .schedulerPeriodSeconds = m_schedulerPeriod,
            .servoTicksPerSchedulerPeriod = m_servoTicksPerSchedulerPeriod,
            .tickMultiplier = m_tickMultiplier.load(std::memory_order_relaxed),
            .servoTicks = m_servoTicks.load(std::memory_order_relaxed),
            .programElapsedSeconds = m_programElapsedSeconds.load(std::memory_order_relaxed),
            .executedPathJerk = m_backend.currentProgramJerkMagnitude(),
            .deadlineMisses = m_deadlineMisses.load(std::memory_order_relaxed),
            .lastWakeLatenessSeconds = m_lastWakeLateness.load(std::memory_order_relaxed),
            .maximumWakeLatenessSeconds =
                m_maximumWakeLateness.load(std::memory_order_relaxed),
            .maximumTickExecutionSeconds =
                m_maximumTickExecution.load(std::memory_order_relaxed),
            .pacingError = m_pacingError.load(std::memory_order_acquire),
        };
    }

    void InProcessSimulationRuntime::setNrtRefillActive(const bool active) noexcept {
        m_nrtRefillActive.store(active, std::memory_order_release);
    }

    bool InProcessSimulationRuntime::executorBatchActive() const noexcept {
        return m_executorBatchActive.load(std::memory_order_acquire);
    }

    void InProcessSimulationRuntime::releaseRefillOpportunity() noexcept {
        m_executorRefillRequested.store(false, std::memory_order_release);
    }

    void InProcessSimulationRuntime::setRollingSupplyActive(const bool active) noexcept {
        m_rollingSupplyActive.store(active, std::memory_order_release);
    }

    void InProcessSimulationRuntime::advanceImmediate(const double seconds) {
        m_backend.advance(seconds);
    }

    std::uint64_t InProcessSimulationRuntime::advanceServiceMotionPeriod() {
        const auto ticks = static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod)
            * m_tickMultiplier.load(std::memory_order_relaxed);
        for (std::uint64_t tick = 0; tick < ticks; ++tick) {
            m_backend.advanceTick(m_servoPeriod, tick + 1 == ticks);
        }

        return ticks;
    }

    bool InProcessSimulationRuntime::configureSyntheticInput(
        const TriggeredMoveId move, const position_t &transitionPosition) noexcept {
        return m_backend.configureSyntheticInput(move, transitionPosition);
    }

    bool InProcessSimulationRuntime::configureSyntheticJointInput(
        const TriggeredMoveId move, const JointId joint,
        const double transitionPosition) noexcept {
        return m_backend.configureSyntheticJointInput(move, joint, transitionPosition);
    }

    void InProcessSimulationRuntime::clearTrajectoryDiagnostics() {
        m_backend.clearTrajectoryDiagnostics();
    }

    std::vector<ExecutedJerkSample> InProcessSimulationRuntime::takeExecutedJerkSamples() {
        return m_backend.takeExecutedJerkSamples();
    }

    void InProcessSimulationRuntime::runScheduler() {
        using clock = std::chrono::steady_clock;

        WindowsServoPacer pacer(m_schedulerPeriod);
        if (!pacer.valid()) {
            m_pacingError.store(pacer.errorCode(), std::memory_order_release);
            return;
        }

        for (;;) {
            {
                std::unique_lock lock(m_schedulerMutex);
                m_schedulerCv.wait(lock, [&] {
                    return m_stopping.load(std::memory_order_acquire)
                        || m_timedExecutionActive.load(std::memory_order_acquire);
                });
                if (m_stopping.load(std::memory_order_acquire)) {
                    return;
                }
                pacer.reset();
            }

            while (m_timedExecutionActive.load(std::memory_order_acquire)
                   && !m_stopping.load(std::memory_order_acquire)) {
                WindowsServoPacer::WaitResult timing;
                if (!pacer.wait(timing)) {
                    m_pacingError.store(pacer.errorCode(), std::memory_order_release);
                    m_timedExecutionActive.store(false, std::memory_order_release);
                    break;
                }
                if (!m_timedExecutionActive.load(std::memory_order_acquire)) {
                    continue;
                }

                m_lastWakeLateness.store(timing.latenessSeconds, std::memory_order_relaxed);
                updateMaximum(m_maximumWakeLateness, timing.latenessSeconds);
                m_deadlineMisses.fetch_add(timing.missedPeriods, std::memory_order_relaxed);
                const auto multiplier = m_tickMultiplier.load(std::memory_order_relaxed);
                const auto ticksThisPeriod =
                    static_cast<std::uint64_t>(m_servoTicksPerSchedulerPeriod) * multiplier;

                if (multiplier > 1) {
                    for (;;) {
                        while ((m_nrtRefillActive.load(std::memory_order_acquire)
                                || m_rollingSupplyActive.load(std::memory_order_acquire))
                               && m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            std::this_thread::yield();
                        }
                        if (!m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            break;
                        }
                        m_executorBatchActive.store(true, std::memory_order_release);
                        if (!m_nrtRefillActive.load(std::memory_order_acquire)
                            && !m_rollingSupplyActive.load(std::memory_order_acquire)) {
                            break;
                        }
                        m_executorBatchActive.store(false, std::memory_order_release);
                    }
                    if (!m_timedExecutionActive.load(std::memory_order_relaxed)) {
                        m_executorBatchActive.store(false, std::memory_order_release);
                        continue;
                    }
                } else {
                    m_executorBatchActive.store(true, std::memory_order_release);
                }

                for (std::uint64_t tick = 0;
                     tick < ticksThisPeriod
                     && m_timedExecutionActive.load(std::memory_order_relaxed);
                     ++tick) {
                    const auto started = clock::now();
                    const auto crossedChunk =
                        m_backend.advanceTick(m_servoPeriod, tick + 1 == ticksThisPeriod);
                    m_programElapsedSeconds.fetch_add(
                        m_backend.lastAdvanceProgramSeconds(), std::memory_order_relaxed);
                    const auto duration =
                        std::chrono::duration<double>(clock::now() - started).count();
                    updateMaximum(m_maximumTickExecution, duration);
                    m_servoTicks.fetch_add(1, std::memory_order_relaxed);
                    if (crossedChunk && tick + 1 < ticksThisPeriod) {
                        m_executorRefillRequested.store(true, std::memory_order_release);
                        m_executorBatchActive.store(false, std::memory_order_release);
                        while (m_executorRefillRequested.load(std::memory_order_acquire)
                               && m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            std::this_thread::yield();
                        }
                        if (!m_timedExecutionActive.load(std::memory_order_relaxed)) {
                            break;
                        }
                        m_executorBatchActive.store(true, std::memory_order_release);
                    }
                }
                m_executorBatchActive.store(false, std::memory_order_release);
            }
        }
    }

    void InProcessSimulationRuntime::resetTimedDiagnostics() noexcept {
        m_executorBatchActive.store(false, std::memory_order_relaxed);
        m_executorRefillRequested.store(false, std::memory_order_relaxed);
        m_nrtRefillActive.store(false, std::memory_order_relaxed);
        m_rollingSupplyActive.store(false, std::memory_order_relaxed);
        m_servoTicks.store(0, std::memory_order_relaxed);
        m_programElapsedSeconds.store(0.0, std::memory_order_relaxed);
        m_deadlineMisses.store(0, std::memory_order_relaxed);
        m_lastWakeLateness.store(0.0, std::memory_order_relaxed);
        m_maximumWakeLateness.store(0.0, std::memory_order_relaxed);
        m_maximumTickExecution.store(0.0, std::memory_order_relaxed);
    }

    void InProcessSimulationRuntime::updateMaximum(
        std::atomic<double> &target, const double value) noexcept {
        auto current = target.load(std::memory_order_relaxed);
        while (current < value && !target.compare_exchange_weak(
                   current, value, std::memory_order_relaxed,
                   std::memory_order_relaxed)) { }
    }
}
