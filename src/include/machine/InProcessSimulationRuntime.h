#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "machine/MachineConfiguration.h"
#include "machine/MockMotionBackend.h"

namespace ngc {
    struct SimulationRuntimeSnapshot {
        double servoPeriodSeconds = 0.001;
        double schedulerPeriodSeconds = 0.01;
        std::uint32_t servoTicksPerSchedulerPeriod = 10;
        std::uint32_t tickMultiplier = 1;
        std::uint64_t servoTicks = 0;
        double programElapsedSeconds = 0.0;
        double executedPathJerk = 0.0;
        std::uint64_t deadlineMisses = 0;
        double lastWakeLatenessSeconds = 0.0;
        double maximumWakeLatenessSeconds = 0.0;
        double maximumTickExecutionSeconds = 0.0;
        std::uint32_t pacingError = 0;
    };

    class InProcessSimulationRuntime {
    public:
        explicit InProcessSimulationRuntime(const TrajectoryLimits &limits = {},
                                            const SimulationTiming &timing = {});
        explicit InProcessSimulationRuntime(const MachineConfiguration &configuration);
        ~InProcessSimulationRuntime();
        InProcessSimulationRuntime(const InProcessSimulationRuntime &) = delete;
        InProcessSimulationRuntime &operator=(const InProcessSimulationRuntime &) = delete;

        MotionBackend &endpoint() noexcept;

        void start();
        void stop();
        bool beginTimedExecution();
        void endTimedExecution();
        [[nodiscard]] bool timedExecutionActive() const noexcept;

        void setTickMultiplier(int multiplier) noexcept;
        [[nodiscard]] std::uint32_t tickMultiplier() const noexcept;
        [[nodiscard]] double servoPeriod() const noexcept;
        [[nodiscard]] double schedulerPeriod() const noexcept;
        [[nodiscard]] std::uint32_t servoTicksPerSchedulerPeriod() const noexcept;
        [[nodiscard]] SimulationRuntimeSnapshot snapshot() const noexcept;

        void setNrtRefillActive(bool active) noexcept;
        [[nodiscard]] bool executorBatchActive() const noexcept;
        void releaseRefillOpportunity() noexcept;
        void setRollingSupplyActive(bool active) noexcept;

        void advanceImmediate(double seconds);
        std::uint64_t advanceServiceMotionPeriod();
        bool configureSyntheticInput(TriggeredMoveId move, const position_t &transitionPosition) noexcept;
        bool configureSyntheticJointInput(TriggeredMoveId move, JointId joint,
                                          double transitionPosition) noexcept;
        void clearTrajectoryDiagnostics();
        std::vector<ExecutedJerkSample> takeExecutedJerkSamples();

    private:
        void runScheduler();
        void resetTimedDiagnostics() noexcept;
        static void updateMaximum(std::atomic<double> &target, double value) noexcept;

        MockMotionBackend m_backend;
        double m_servoPeriod;
        double m_schedulerPeriod;
        std::uint32_t m_servoTicksPerSchedulerPeriod;
        std::atomic<std::uint32_t> m_tickMultiplier{1};
        std::mutex m_schedulerMutex;
        std::condition_variable m_schedulerCv;
        std::thread m_schedulerThread;
        bool m_started = false;
        std::atomic<bool> m_stopping{false};
        std::atomic<bool> m_timedExecutionActive{false};
        std::atomic<bool> m_executorBatchActive{false};
        std::atomic<bool> m_executorRefillRequested{false};
        std::atomic<bool> m_nrtRefillActive{false};
        std::atomic<bool> m_rollingSupplyActive{false};
        std::atomic<std::uint64_t> m_servoTicks{0};
        std::atomic<double> m_programElapsedSeconds{0.0};
        std::atomic<std::uint64_t> m_deadlineMisses{0};
        std::atomic<double> m_lastWakeLateness{0.0};
        std::atomic<double> m_maximumWakeLateness{0.0};
        std::atomic<double> m_maximumTickExecution{0.0};
        std::atomic<std::uint32_t> m_pacingError{0};
    };
}
