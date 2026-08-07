#pragma once

#include <cstddef>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "machine/ExecutorDemandController.h"
#include "machine/MotionBackend.h"

namespace ngc {
    struct ServicedMotionRuntimeCallbacks {
        std::function<void()> serviceImmediate;
        std::function<std::uint64_t()> advanceServiceMotionPeriod;
        std::function<void()> waitForServiceMotion;
        std::function<void(const ExecutionSnapshot &, std::uint64_t)> observeSnapshot;
        std::function<void()> stopRuntime;
        std::function<void()> faultSession;
    };

    class ServicedMotionOperation {
    public:
        static constexpr std::size_t SERVICE_ITERATION_LIMIT = 10000000;

        ServicedMotionOperation(MotionBackend &backend,
                                ExecutorDemandController &demand,
                                ServicedMotionRuntimeCallbacks callbacks);

        [[nodiscard]] std::expected<void, std::string> begin(
            EpochId epoch, ExecutorDemandMode mode);
        [[nodiscard]] std::expected<void, std::string> requestDemand(
            ExecutorDemandMode mode);
        [[nodiscard]] bool submit(const ControlRequest &request);
        [[nodiscard]] bool publish(const ExecutionItem &item);
        void discardPendingEvents() noexcept;
        void motionMayBeActive() noexcept;
        void observeTerminalJoints(
            const JointMotionState &joints,
            BackendState state = BackendState::Held);

        template<typename Predicate, typename BeforeService>
        [[nodiscard]] std::expected<ExecutionEvent, std::string> serviceUntil(
                Predicate &&predicate, BeforeService &&beforeService,
                const std::string &limitError) {
            for (std::size_t guard = 0; guard < SERVICE_ITERATION_LIMIT; ++guard) {
                if (const auto error = beforeService()) {
                    return std::unexpected(*error);
                }

                drainSnapshots();
                if (const auto error = observedTerminalFailure()) {
                    return std::unexpected(*error);
                }
                ExecutionEvent event;
                while (m_backend.tryTakeEvent(event)) {
                    if (predicate(event)) {
                        return event;
                    }
                    if (const auto *fault = std::get_if<BackendFault>(&event)) {
                        observeFault(fault->code);

                        return std::unexpected(
                            "motion backend fault " + std::to_string(fault->code));
                    }
                }

                m_servoTicks += m_callbacks.advanceServiceMotionPeriod();
                drainSnapshots();
                if (const auto error = observedTerminalFailure()) {
                    return std::unexpected(*error);
                }
                m_callbacks.waitForServiceMotion();
            }

            return std::unexpected(limitError);
        }

        template<typename Predicate>
        [[nodiscard]] std::expected<ExecutionEvent, std::string> serviceUntil(
                Predicate &&predicate, const std::string &limitError) {
            return serviceUntil(
                std::forward<Predicate>(predicate),
                []() -> std::optional<std::string> { return std::nullopt; },
                limitError);
        }

        template<typename Result>
        [[nodiscard]] std::expected<Result, std::string> finish(
                std::expected<Result, std::string> result) {
            if (!m_motionMayBeActive || m_terminalObserved) {
                return result;
            }

            const auto cleanup = stopAndQuiesce();
            if (cleanup) {
                return result;
            }
            if (result) {
                return std::unexpected(cleanup.error());
            }

            return std::unexpected(
                result.error() + "; cleanup failed: " + cleanup.error());
        }

    private:
        void observeTerminalSnapshot(const ExecutionSnapshot &snapshot);
        void observeFault(std::uint32_t code);
        void drainSnapshots();
        [[nodiscard]] std::optional<std::string> observedTerminalFailure() const;
        [[nodiscard]] bool stationaryAfterStop() const noexcept;
        [[nodiscard]] std::expected<void, std::string> stopAndQuiesce();
        [[nodiscard]] std::string stopRuntimeFallback(const std::string &error);
        void reportFault();

        MotionBackend &m_backend;
        ExecutorDemandController &m_demand;
        ServicedMotionRuntimeCallbacks m_callbacks;
        std::optional<ExecutionSnapshot> m_latestSnapshot;
        EpochId m_epoch = 0;
        DemandGeneration m_stopGeneration = 0;
        std::uint64_t m_servoTicks = 0;
        bool m_begun = false;
        bool m_motionMayBeActive = false;
        bool m_terminalObserved = false;
        bool m_faultReported = false;
    };
}
