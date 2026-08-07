#include "machine/ServicedMotionOperation.h"

#include <exception>

namespace ngc {
    ServicedMotionOperation::ServicedMotionOperation(
        MotionBackend &backend, ExecutorDemandController &demand,
        ServicedMotionRuntimeCallbacks callbacks)
        : m_backend(backend), m_demand(demand), m_callbacks(std::move(callbacks)) { }

    std::expected<void, std::string> ServicedMotionOperation::begin(
        const EpochId epoch, const ExecutorDemandMode mode) {
        if (!m_callbacks.serviceImmediate
            || !m_callbacks.advanceServiceMotionPeriod
            || !m_callbacks.waitForServiceMotion) {
            return std::unexpected("serviced-motion runtime callbacks are incomplete");
        }

        m_epoch = epoch;
        m_backend.discardPendingOutput();
        if (!m_demand.request(epoch, mode)) {
            return std::unexpected("motion backend demand mailbox rejected operation initialization");
        }
        m_begun = true;
        m_callbacks.serviceImmediate();
        drainSnapshots();

        return {};
    }

    std::expected<void, std::string> ServicedMotionOperation::requestDemand(
        const ExecutorDemandMode mode) {
        if (!m_begun || !m_demand.request(m_epoch, mode)) {
            return std::unexpected("motion backend demand mailbox rejected lifecycle demand");
        }
        m_callbacks.serviceImmediate();
        drainSnapshots();

        return {};
    }

    bool ServicedMotionOperation::submit(const ControlRequest &request) {
        if (m_backend.trySubmit(request) != SubmitResult::Submitted) {
            return false;
        }
        m_callbacks.serviceImmediate();
        drainSnapshots();

        return true;
    }

    bool ServicedMotionOperation::publish(const ExecutionItem &item) {
        if (m_backend.tryPublish(item) != PublishResult::Published) {
            return false;
        }
        m_callbacks.serviceImmediate();
        drainSnapshots();

        return true;
    }

    void ServicedMotionOperation::discardPendingEvents() noexcept {
        m_backend.discardPendingEvents();
    }

    void ServicedMotionOperation::motionMayBeActive() noexcept {
        m_motionMayBeActive = true;
        m_terminalObserved = false;
    }

    void ServicedMotionOperation::observeTerminalJoints(
        const JointMotionState &joints, const BackendState state) {
        ExecutionSnapshot snapshot;
        if (m_latestSnapshot) {
            snapshot = *m_latestSnapshot;
        }
        snapshot.state = state;
        snapshot.activeJoints = 0;
        snapshot.commandedJoints = joints;
        observeTerminalSnapshot(snapshot);
        if (state == BackendState::Faulted) {
            reportFault();
        }
    }

    void ServicedMotionOperation::observeTerminalSnapshot(
        const ExecutionSnapshot &snapshot) {
        m_latestSnapshot = snapshot;
        m_terminalObserved = true;
        m_motionMayBeActive = false;
        if (m_callbacks.observeSnapshot) {
            m_callbacks.observeSnapshot(snapshot, m_servoTicks);
        }
    }

    void ServicedMotionOperation::observeFault(const std::uint32_t code) {
        ExecutionSnapshot snapshot;
        if (m_latestSnapshot) {
            snapshot = *m_latestSnapshot;
        }
        snapshot.state = BackendState::Faulted;
        snapshot.activeJoints = 0;
        snapshot.faultCode = code;
        observeTerminalSnapshot(snapshot);
        reportFault();
    }

    void ServicedMotionOperation::drainSnapshots() {
        ExecutionSnapshot snapshot;
        while (m_backend.tryTakeSnapshot(snapshot)) {
            m_latestSnapshot = snapshot;
            if (snapshot.state == BackendState::Faulted
                || snapshot.state == BackendState::Disabled) {
                m_terminalObserved = true;
                m_motionMayBeActive = false;
                if (snapshot.state == BackendState::Faulted) {
                    reportFault();
                }
            }
            if (m_callbacks.observeSnapshot) {
                m_callbacks.observeSnapshot(snapshot, m_servoTicks);
            }
        }
    }

    std::optional<std::string>
    ServicedMotionOperation::observedTerminalFailure() const {
        if (!m_latestSnapshot) {
            return std::nullopt;
        }
        if (m_latestSnapshot->state == BackendState::Faulted) {
            return "motion backend fault "
                + std::to_string(m_latestSnapshot->faultCode);
        }
        if (m_latestSnapshot->state == BackendState::Disabled) {
            return "motion backend became disabled during serviced motion";
        }

        return std::nullopt;
    }

    bool ServicedMotionOperation::stationaryAfterStop() const noexcept {
        if (!m_latestSnapshot) {
            return false;
        }
        const auto &snapshot = *m_latestSnapshot;
        if (snapshot.state == BackendState::Faulted
            || snapshot.state == BackendState::Disabled) {
            return true;
        }

        return snapshot.state == BackendState::Held
            && snapshot.activeJoints == 0
            && snapshot.acknowledgedDemandGeneration >= m_stopGeneration
            && snapshot.demandedMode == ExecutorDemandMode::Stop
            && snapshot.demandAccepted;
    }

    std::expected<void, std::string> ServicedMotionOperation::stopAndQuiesce() {
        if (!m_demand.request(m_epoch, ExecutorDemandMode::Stop)) {
            return std::unexpected(stopRuntimeFallback(
                "motion backend demand mailbox rejected cleanup Stop"));
        }
        m_stopGeneration = m_demand.lastGeneration();
        m_callbacks.serviceImmediate();

        for (std::size_t guard = 0; guard < SERVICE_ITERATION_LIMIT; ++guard) {
            drainSnapshots();
            if (stationaryAfterStop()) {
                m_terminalObserved = true;
                m_motionMayBeActive = false;

                return {};
            }

            ExecutionEvent event;
            while (m_backend.tryTakeEvent(event)) {
                if (const auto *fault = std::get_if<BackendFault>(&event)) {
                    observeFault(fault->code);

                    return {};
                }
            }

            m_servoTicks += m_callbacks.advanceServiceMotionPeriod();
            m_callbacks.waitForServiceMotion();
        }

        return std::unexpected(stopRuntimeFallback(
            "cleanup exceeded its bounded service iteration limit"));
    }

    std::string ServicedMotionOperation::stopRuntimeFallback(
        const std::string &error) {
        std::string fallbackError;
        if (!m_callbacks.stopRuntime) {
            fallbackError = "runtime-stop fallback is unavailable";
        } else {
            try {
                m_callbacks.stopRuntime();
                m_terminalObserved = true;
                m_motionMayBeActive = false;
            } catch (const std::exception &exception) {
                fallbackError = std::string("runtime stop failed: ") + exception.what();
            } catch (...) {
                fallbackError = "runtime stop failed with an unknown exception";
            }
        }
        reportFault();

        return fallbackError.empty() ? error : error + "; " + fallbackError;
    }

    void ServicedMotionOperation::reportFault() {
        if (!m_faultReported && m_callbacks.faultSession) {
            m_callbacks.faultSession();
            m_faultReported = true;
        }
    }
}
