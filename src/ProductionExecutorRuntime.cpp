#include "machine/ProductionExecutorRuntime.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "machine/RealtimeHost.h"

namespace ngc {
    namespace {
        class NullProductionExecutorIo final : public ProductionExecutorIo {
        public:
            void sampleDigitalInputs(
                ProductionExecutorDigitalInputs &inputs) noexcept override {
                inputs.reset();
            }

            void applyOutputs(
                const ProductionExecutorOutputState &) noexcept override { }
        };

        std::size_t axisIndex(const Machine::Axis axis) noexcept {
            switch (axis) {
                case Machine::Axis::X: return static_cast<std::size_t>(AxisId::X);
                case Machine::Axis::Y: return static_cast<std::size_t>(AxisId::Y);
                case Machine::Axis::Z: return static_cast<std::size_t>(AxisId::Z);
                case Machine::Axis::A: return static_cast<std::size_t>(AxisId::A);
                case Machine::Axis::B: return static_cast<std::size_t>(AxisId::B);
                case Machine::Axis::C: return static_cast<std::size_t>(AxisId::C);
            }

            return 0;
        }

        double &component(position_t &value, const Machine::Axis axis) noexcept {
            switch (axis) {
                case Machine::Axis::X: return value.x;
                case Machine::Axis::Y: return value.y;
                case Machine::Axis::Z: return value.z;
                case Machine::Axis::A: return value.a;
                case Machine::Axis::B: return value.b;
                case Machine::Axis::C: return value.c;
            }

            return value.x;
        }

        std::size_t histogramBucket(const std::uint64_t nanoseconds) noexcept {
            if (nanoseconds == 0) {
                return 0;
            }

            return std::min<std::size_t>(
                std::bit_width(nanoseconds),
                REALTIME_TIMING_HISTOGRAM_BUCKETS - 1);
        }

    }

    struct ProductionExecutorRuntime::TimingAccumulator {
        RealtimeTimingSummary summary;
        std::uint64_t consecutiveMisses = 0;
        std::uint32_t ticksSincePublicationAttempt = 0;
    };

    ProductionExecutorRuntimeConfiguration productionExecutorRuntimeConfiguration(
        const MachineConfiguration &configuration) {
        ProductionExecutorRuntimeConfiguration result;
        if (configuration.realBackend.has_value()) {
            result.servoPeriod = configuration.realBackend->servoPeriod;
            result.serviceTicksPerPeriod = 1;
            result.realtime = {
                .enabled = configuration.realBackend->realtimeEnabled,
                .cpu = configuration.realBackend->realtimeCpu,
                .priority = configuration.realBackend->realtimePriority,
                .lockMemory = configuration.realBackend->lockMemory,
            };
        } else {
            result.servoPeriod = configuration.simulation.servoPeriod;
            result.serviceTicksPerPeriod = static_cast<std::uint32_t>(std::max(
                1.0, std::round(configuration.simulation.schedulerPeriod
                                / configuration.simulation.servoPeriod)));
        }
        result.executor.feedHold.tangentialAcceleration =
            configuration.feedHold.tangentialAcceleration;
        result.executor.feedHold.tangentialJerk =
            configuration.feedHold.tangentialJerk;
        result.executor.feedHold.pathAcceleration =
            configuration.trajectory.pathAcceleration;

        for (const auto &axis : configuration.axes) {
            auto &mapping = result.executor.axes[axisIndex(axis.axis)];
            auto &stopVelocity =
                component(result.executor.controlledStopLimits.velocity, axis.axis);
            auto &stopAcceleration =
                component(result.executor.controlledStopLimits.acceleration, axis.axis);
            auto &stopJerk =
                component(result.executor.controlledStopLimits.jerk, axis.axis);
            auto &holdAcceleration =
                component(result.executor.feedHold.axisAcceleration, axis.axis);

            stopVelocity = axis.maxVelocity;
            stopAcceleration = axis.maxAcceleration;
            stopJerk = axis.maxJerk;
            holdAcceleration = axis.maxAcceleration;
            for (const auto id : axis.joints) {
                if (id >= MAX_JOINTS) {
                    throw std::invalid_argument(
                        "production executor axis mapping contains an invalid joint");
                }
                const auto joint = std::ranges::find(
                    configuration.joints, id, &JointConfiguration::id);
                if (joint == configuration.joints.end()) {
                    throw std::invalid_argument(
                        "production executor axis mapping contains an unknown joint");
                }

                mapping.joints |= JointMask{1} << id;
                mapping.coordinateScale[id] = joint->coordinateScale;
            }
        }

        if (configuration.pendant.velocity.leaseDuration > 0.0) {
            const auto ticks = std::ceil(
                configuration.pendant.velocity.leaseDuration
                / result.servoPeriod);
            result.executor.maximumJogLeaseTicks = static_cast<std::uint32_t>(
                std::clamp(ticks, 1.0,
                           static_cast<double>(
                               std::numeric_limits<std::uint32_t>::max())));
        }

        return result;
    }

    ProductionExecutorRuntime::ProductionExecutorRuntime(
        ProductionExecutorRuntimeConfiguration configuration,
        std::unique_ptr<ProductionExecutorIo> io)
        : m_core(std::make_unique<ProductionExecutorCore>(
              configuration.servoPeriod, std::move(configuration.executor))),
          m_io(io ? std::move(io)
                  : std::make_unique<NullProductionExecutorIo>()),
          m_timingAccumulator(std::make_unique<TimingAccumulator>()),
          m_servoPeriod(configuration.servoPeriod),
          m_serviceTicksPerPeriod(configuration.serviceTicksPerPeriod),
          m_timingPublicationTicks(configuration.timingPublicationTicks),
          m_realtime(configuration.realtime) {
        if (m_serviceTicksPerPeriod == 0) {
            throw std::invalid_argument(
                "production executor service period must contain at least one tick");
        }
        if (m_timingPublicationTicks == 0) {
            throw std::invalid_argument(
                "production executor timing publication period must contain at least one tick");
        }
        if (m_realtime.enabled
            && (m_realtime.priority < 1 || m_realtime.priority > 99)) {
            throw std::invalid_argument(
                "production executor real-time priority must be between 1 and 99");
        }
    }

    ProductionExecutorRuntime::ProductionExecutorRuntime(
        const MachineConfiguration &configuration,
        std::unique_ptr<ProductionExecutorIo> io)
        : ProductionExecutorRuntime(
              productionExecutorRuntimeConfiguration(configuration),
              std::move(io)) { }

    ProductionExecutorRuntime::~ProductionExecutorRuntime() {
        stop();
    }

    MotionBackend &ProductionExecutorRuntime::endpoint() noexcept {
        return *m_core;
    }

    void ProductionExecutorRuntime::start() {
        std::unique_lock lock(m_lifecycleMutex);
        if (m_started) {
            return;
        }

        if (m_realtime.enabled) {
            excludeCpuFromCurrentThread(m_realtime.cpu);
            if (m_realtime.lockMemory) {
                lockProcessMemory();
            }
        }

        m_stopping.store(false, std::memory_order_release);
        m_startupComplete = false;
        m_startupError.clear();
        *m_timingAccumulator = {};
        RealtimeTimingSummary staleTiming;
        while (m_timing.tryPop(staleTiming)) { }
        m_started = true;
        try {
            m_servoThread = std::thread(
                &ProductionExecutorRuntime::runServoLoop, this);
        } catch (...) {
            m_started = false;
            throw;
        }

        m_lifecycleCv.wait(lock, [&] {
            return m_startupComplete;
        });
        if (!m_startupError.empty()) {
            const auto error = m_startupError;
            lock.unlock();
            m_servoThread.join();
            lock.lock();
            m_started = false;

            throw std::runtime_error(error);
        }
    }

    void ProductionExecutorRuntime::stop() {
        {
            std::scoped_lock lock(m_lifecycleMutex);
            if (!m_started) {
                return;
            }
            m_stopping.store(true, std::memory_order_release);
        }
        m_servoThread.join();

        std::scoped_lock lock(m_lifecycleMutex);
        m_started = false;
        m_stopping.store(false, std::memory_order_release);
    }

    BackendCapabilities ProductionExecutorRuntime::capabilities() const noexcept {
        return {};
    }

    bool ProductionExecutorRuntime::restoreStationaryState(
        const StationaryBackendState &state) noexcept {
        std::scoped_lock lock(m_lifecycleMutex);
        if (m_started) {
            return false;
        }

        m_core->restoreStationaryState(
            state.commanded, state.feedback,
            state.commandedJoints, state.feedbackJoints);

        return true;
    }

    bool ProductionExecutorRuntime::prepareTriggeredJointMove(
        const TriggeredJointMove &move) noexcept {
        return m_io->prepareTriggeredJointMove(move);
    }

    void ProductionExecutorRuntime::serviceImmediate() {
        if (running()) {
            static_cast<void>(waitForNextTick(servoTicks()));

            return;
        }

        m_core->serviceImmediate();
        m_io->applyOutputs(m_core->outputState());
    }

    std::uint64_t ProductionExecutorRuntime::advanceServiceMotionPeriod() {
        if (running()) {
            const auto before = servoTicks();
            auto advanced = std::uint64_t{0};
            while (advanced < m_serviceTicksPerPeriod) {
                const auto ticks = waitForNextTick(before + advanced);
                if (ticks == 0) {
                    break;
                }
                advanced += ticks;
            }

            return advanced;
        }

        for (std::uint32_t tickIndex = 0;
             tickIndex < m_serviceTicksPerPeriod; ++tickIndex) {
            tick(tickIndex + 1 == m_serviceTicksPerPeriod);
        }

        return m_serviceTicksPerPeriod;
    }

    void ProductionExecutorRuntime::waitForServiceMotion() {
        // advanceServiceMotionPeriod() either performs or waits for the fixed
        // servo work. No additional host-side delay is required here.
    }

    bool ProductionExecutorRuntime::started() const noexcept {
        std::scoped_lock lock(m_lifecycleMutex);

        return m_started;
    }

    std::uint64_t ProductionExecutorRuntime::servoTicks() const noexcept {
        return m_servoTicks.load(std::memory_order_relaxed);
    }

    bool ProductionExecutorRuntime::tryTakeRealtimeTiming(
        RealtimeTimingSummary &summary) noexcept {
        return m_timing.tryPop(summary);
    }

    void ProductionExecutorRuntime::configureServoThread() {
        if (!m_realtime.enabled) {
            return;
        }

        configureCurrentRealtimeThread(
            m_realtime.cpu, m_realtime.priority);
    }

    void ProductionExecutorRuntime::runServoLoop() {
        using clock = std::chrono::steady_clock;

        try {
            configureServoThread();
        } catch (const std::exception &error) {
            {
                std::scoped_lock lock(m_lifecycleMutex);
                m_startupError = error.what();
                m_startupComplete = true;
            }
            m_lifecycleCv.notify_all();

            return;
        }

        {
            std::scoped_lock lock(m_lifecycleMutex);
            m_startupComplete = true;
        }
        m_lifecycleCv.notify_all();

        const auto period = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(m_servoPeriod));
        auto deadline = clock::now();
        for (;;) {
            deadline += period;
            if (m_realtime.enabled) {
                sleepUntilMonotonic(deadline);
            } else {
                std::this_thread::sleep_until(deadline);
            }
            if (m_stopping.load(std::memory_order_acquire)) {
                break;
            }

            const auto wake = clock::now();
            tick(true);
            const auto finished = clock::now();
            const auto nextDeadline = deadline + period;
            const auto wakeLateness = std::chrono::duration_cast<
                std::chrono::nanoseconds>(wake - deadline).count();
            const auto execution = std::chrono::duration_cast<
                std::chrono::nanoseconds>(finished - wake).count();
            const auto slack = std::chrono::duration_cast<
                std::chrono::nanoseconds>(nextDeadline - finished).count();
            auto skippedPeriods = std::uint64_t{0};
            if (finished >= nextDeadline) {
                skippedPeriods = static_cast<std::uint64_t>(
                    (finished - deadline) / period);
                deadline += period * skippedPeriods;
            }

            observeTiming(
                m_servoTicks.load(std::memory_order_relaxed),
                wakeLateness, static_cast<std::uint64_t>(execution),
                slack, skippedPeriods);
            if (m_realtime.enabled && slack < 0) {
                m_core->reportHostFault(
                    PRODUCTION_EXECUTOR_DEADLINE_MISS_FAULT);
                m_io->applyOutputs(m_core->outputState());
            }
        }

        if (m_timingAccumulator->summary.sampleCount != 0) {
            static_cast<void>(m_timing.tryPush(
                m_timingAccumulator->summary));
        }
    }

    void ProductionExecutorRuntime::tick(
        const bool publishSnapshot) noexcept {
        m_io->sampleDigitalInputs(m_inputs);
        if (const auto fault = m_io->faultCode(); fault != 0) {
            m_core->reportHostFault(fault);
        } else {
            m_core->setDigitalInputSamples(m_inputs);
            m_core->servoTick(publishSnapshot);
        }
        m_io->applyOutputs(m_core->outputState());

        m_servoTicks.fetch_add(1, std::memory_order_relaxed);
    }

    void ProductionExecutorRuntime::observeTiming(
        const std::uint64_t tick,
        const std::int64_t wakeLatenessNanoseconds,
        const std::uint64_t executionNanoseconds,
        const std::int64_t deadlineSlackNanoseconds,
        const std::uint64_t skippedPeriods) noexcept {
        auto &accumulator = *m_timingAccumulator;
        auto &summary = accumulator.summary;
        if (summary.sampleCount == 0) {
            summary.firstTick = tick;
            summary.minimumDeadlineSlackNanoseconds =
                deadlineSlackNanoseconds;
            summary.worstTick = tick;
        }

        summary.lastTick = tick;
        ++summary.sampleCount;
        summary.maximumWakeLatenessNanoseconds = std::max(
            summary.maximumWakeLatenessNanoseconds,
            wakeLatenessNanoseconds);
        summary.maximumExecutionNanoseconds = std::max(
            summary.maximumExecutionNanoseconds,
            executionNanoseconds);
        if (deadlineSlackNanoseconds
            < summary.minimumDeadlineSlackNanoseconds) {
            summary.minimumDeadlineSlackNanoseconds =
                deadlineSlackNanoseconds;
            summary.worstTick = tick;
        }
        summary.skippedPeriods += skippedPeriods;

        if (deadlineSlackNanoseconds < 0) {
            ++summary.missedDeadlines;
            ++accumulator.consecutiveMisses;
            summary.maximumConsecutiveMisses = std::max(
                summary.maximumConsecutiveMisses,
                accumulator.consecutiveMisses);
        } else {
            accumulator.consecutiveMisses = 0;
        }

        const auto nonnegativeWake = static_cast<std::uint64_t>(
            std::max<std::int64_t>(0, wakeLatenessNanoseconds));
        ++summary.wakeLatenessHistogram[
            histogramBucket(nonnegativeWake)];
        ++summary.executionHistogram[
            histogramBucket(executionNanoseconds)];

        ++accumulator.ticksSincePublicationAttempt;
        if (accumulator.ticksSincePublicationAttempt
            < m_timingPublicationTicks) {
            return;
        }

        accumulator.ticksSincePublicationAttempt = 0;
        if (m_timing.tryPush(summary)) {
            const auto consecutiveMisses =
                accumulator.consecutiveMisses;
            accumulator = {};
            accumulator.consecutiveMisses = consecutiveMisses;
        } else {
            ++summary.failedPublications;
        }
    }

    bool ProductionExecutorRuntime::running() const noexcept {
        std::scoped_lock lock(m_lifecycleMutex);

        return m_started
            && !m_stopping.load(std::memory_order_acquire);
    }

    std::uint64_t ProductionExecutorRuntime::waitForNextTick(
        const std::uint64_t previousTicks) {
        auto current = m_servoTicks.load(std::memory_order_relaxed);
        while (current == previousTicks && running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            current = m_servoTicks.load(std::memory_order_relaxed);
        }

        return current - previousTicks;
    }
}
