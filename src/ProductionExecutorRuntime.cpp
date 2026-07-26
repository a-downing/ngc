#include "machine/ProductionExecutorRuntime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <utility>

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
    }

    ProductionExecutorRuntimeConfiguration productionExecutorRuntimeConfiguration(
        const MachineConfiguration &configuration) {
        ProductionExecutorRuntimeConfiguration result;
        result.servoPeriod = configuration.simulation.servoPeriod;
        result.serviceTicksPerPeriod = static_cast<std::uint32_t>(std::max(
            1.0, std::round(configuration.simulation.schedulerPeriod
                            / configuration.simulation.servoPeriod)));
        result.executor.feedHold.tangentialAcceleration =
            configuration.feedHold.tangentialAcceleration;
        result.executor.feedHold.tangentialJerk =
            configuration.feedHold.tangentialJerk;
        result.executor.feedHold.pathAcceleration =
            configuration.trajectory.pathAcceleration;
        result.executor.feedHold.pathJerk =
            configuration.trajectory.pathJerk;

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
            auto &holdJerk =
                component(result.executor.feedHold.axisJerk, axis.axis);

            stopVelocity = axis.maxVelocity;
            stopAcceleration = axis.maxAcceleration;
            stopJerk = axis.maxJerk;
            holdAcceleration = axis.maxAcceleration;
            holdJerk = axis.maxJerk;
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
          m_servoPeriod(configuration.servoPeriod),
          m_serviceTicksPerPeriod(configuration.serviceTicksPerPeriod) {
        if (m_serviceTicksPerPeriod == 0) {
            throw std::invalid_argument(
                "production executor service period must contain at least one tick");
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
        std::scoped_lock lock(m_lifecycleMutex);
        if (m_started) {
            return;
        }

        m_stopping = false;
        m_started = true;
        try {
            m_servoThread = std::thread(
                &ProductionExecutorRuntime::runServoLoop, this);
        } catch (...) {
            m_started = false;
            throw;
        }
    }

    void ProductionExecutorRuntime::stop() {
        {
            std::scoped_lock lock(m_lifecycleMutex);
            if (!m_started) {
                return;
            }
            m_stopping = true;
        }
        m_lifecycleCv.notify_all();
        m_servoThread.join();

        std::scoped_lock lock(m_lifecycleMutex);
        m_started = false;
        m_stopping = false;
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
        std::scoped_lock lock(m_lifecycleMutex);

        return m_servoTicks;
    }

    void ProductionExecutorRuntime::runServoLoop() {
        using clock = std::chrono::steady_clock;

        auto deadline = clock::now();
        for (;;) {
            deadline += std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(m_servoPeriod));
            {
                std::unique_lock lock(m_lifecycleMutex);
                if (m_lifecycleCv.wait_until(lock, deadline, [&] {
                        return m_stopping;
                    })) {
                    return;
                }
            }

            tick(true);
        }
    }

    void ProductionExecutorRuntime::tick(
        const bool publishSnapshot) noexcept {
        m_io->sampleDigitalInputs(m_inputs);
        m_core->setDigitalInputSamples(m_inputs);
        m_core->servoTick(publishSnapshot);
        m_io->applyOutputs(m_core->outputState());

        {
            std::scoped_lock lock(m_lifecycleMutex);
            ++m_servoTicks;
        }
        m_lifecycleCv.notify_all();
    }

    bool ProductionExecutorRuntime::running() const noexcept {
        std::scoped_lock lock(m_lifecycleMutex);

        return m_started && !m_stopping;
    }

    std::uint64_t ProductionExecutorRuntime::waitForNextTick(
        const std::uint64_t previousTicks) {
        std::unique_lock lock(m_lifecycleMutex);
        m_lifecycleCv.wait(lock, [&] {
            return m_servoTicks != previousTicks || !m_started || m_stopping;
        });

        return m_servoTicks - previousTicks;
    }
}
