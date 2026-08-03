#include "machine/ProgramExecutionController.h"

#include <cmath>
#include <utility>

#include "machine/MachineSession.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"

namespace ngc {
    ProgramExecutionController::ProgramExecutionController(
        MotionBackend &backend, PreparedTrajectoryExecutionDriver &driver,
        SessionCommandQueue &commands)
        : m_backend(backend), m_driver(driver), m_commands(commands) { }

    void ProgramExecutionController::begin(const EpochId epoch) {
        std::scoped_lock lock(m_mutex);
        m_state = ProgramExecutionState::Running;
        m_epoch = epoch;
        m_stopRequested = false;
        m_controlledStopInProgress = false;
        m_programPaused = false;
        m_programResumeRequested = false;
        m_feedHoldRequested = false;
        m_feedHoldInProgress = false;
        m_feedHoldHeld = false;
        m_feedResumeRequested = false;
        m_feedResumeInProgress = false;
        m_failureStopComplete = false;
        m_pendingFeedHoldRequest.reset();
        m_pendingFeedResumeRequest.reset();
        m_pendingControlledStopRequest.reset();
        m_pendingAbortRequest.reset();
        m_stoppedPosition.reset();
        m_error.reset();
    }

    void ProgramExecutionController::service(
        const ExecutionSnapshot &snapshot, const bool shutdownRequested) {
        std::scoped_lock lock(m_mutex);
        if (!activeUnlocked()) {
            return;
        }

        if (m_feedResumeInProgress && snapshot.epoch == m_epoch
            && snapshot.state == BackendState::Running
            && snapshot.executionRate >= 1.0 - 1e-10
            && std::abs(snapshot.executionRateAcceleration) <= 1e-10) {
            m_feedResumeInProgress = false;
        }
        consumeCommands();
        if (shutdownRequested) {
            m_stopRequested = true;
            m_feedHoldRequested = false;
            m_feedResumeRequested = false;
            m_programResumeRequested = false;
        }
        if (m_stopRequested) {
            requestControlledStop(snapshot);
        } else {
            requestProgramResume();
            requestFeedHold();
            requestFeedResume();
        }
        observeDriverStateUnlocked();
    }

    void ProgramExecutionController::consumeCommands() {
        while (auto command = m_commands.tryPop()) {
            if (std::holds_alternative<Stop>(*command)) {
                m_stopRequested = true;
                m_feedHoldRequested = false;
                m_feedResumeRequested = false;
                m_programResumeRequested = false;
                m_state = ProgramExecutionState::Holding;
            } else if (std::holds_alternative<FeedHold>(*command)) {
                if (m_state == ProgramExecutionState::Running && !m_feedHoldInProgress
                    && !m_feedHoldHeld && !m_feedResumeInProgress) {
                    m_feedHoldRequested = true;
                    m_feedHoldInProgress = true;
                    m_state = ProgramExecutionState::Holding;
                }
            } else if (std::holds_alternative<Resume>(*command)) {
                if (m_programPaused) {
                    m_programResumeRequested = true;
                    m_state = ProgramExecutionState::Running;
                } else if (m_feedHoldHeld && !m_feedResumeInProgress) {
                    m_feedResumeRequested = true;
                    m_feedResumeInProgress = true;
                    m_state = ProgramExecutionState::Running;
                }
            }
        }
    }

    void ProgramExecutionController::requestControlledStop(
        const ExecutionSnapshot &snapshot) {
        if (m_controlledStopInProgress || m_state == ProgramExecutionState::StopComplete
            || m_feedHoldInProgress || m_pendingAbortRequest) {
            return;
        }

        const auto alreadyStationary =
            snapshot.state == BackendState::Held
            && snapshot.commanded.velocity.length() <= 1e-10
            && snapshot.commanded.acceleration.length() <= 1e-10;
        if (m_error) {
            if (snapshot.state == BackendState::Faulted
                || snapshot.state == BackendState::Disabled) {
                m_state = ProgramExecutionState::Error;

                return;
            }
            if (m_failureStopComplete || alreadyStationary) {
                m_failureStopComplete = true;
                m_stoppedPosition = snapshot.commanded.position;
                requestFailureAbort();

                return;
            }
        } else if (m_programPaused || alreadyStationary) {
            m_stoppedPosition = snapshot.commanded.position;
            m_state = ProgramExecutionState::StopComplete;

            return;
        }

        const auto request = m_nextRequest++;
        if (m_backend.trySubmit(ControlledStopRequest { request })
            != SubmitResult::Submitted) {
            if (!m_error) {
                fail("motion backend control channel is full while requesting stop");
            }

            return;
        }
        m_pendingControlledStopRequest = request;
        m_controlledStopInProgress = true;
    }

    void ProgramExecutionController::requestFailureAbort() {
        if (m_pendingAbortRequest) {
            return;
        }

        const auto request = m_nextRequest++;
        if (m_backend.trySubmit(AbortRequest { request })
            != SubmitResult::Submitted) {
            return;
        }
        m_pendingAbortRequest = request;
    }

    void ProgramExecutionController::requestProgramResume() {
        if (!m_programResumeRequested) {
            return;
        }
        m_programResumeRequested = false;
        if (!m_driver.resumeProgram()) {
            fail("prepared trajectory driver rejected the M0 program resume");

            return;
        }
        m_programPaused = false;
    }

    void ProgramExecutionController::requestFeedHold() {
        if (!m_feedHoldRequested) {
            return;
        }
        m_feedHoldRequested = false;
        const auto request = m_nextRequest++;
        if (m_backend.trySubmit(FeedHoldRequest { request }) != SubmitResult::Submitted) {
            fail("motion backend control channel is full while requesting feed hold");

            return;
        }
        m_pendingFeedHoldRequest = request;
    }

    void ProgramExecutionController::requestFeedResume() {
        if (!m_feedResumeRequested) {
            return;
        }
        m_feedResumeRequested = false;
        const auto request = m_nextRequest++;
        if (m_backend.trySubmit(ResumeRequest { request, m_epoch }) != SubmitResult::Submitted) {
            fail("motion backend control channel is full while resuming feed");

            return;
        }
        m_pendingFeedResumeRequest = request;
    }

    void ProgramExecutionController::observeBackendEvent(const ExecutionEvent &event) {
        std::scoped_lock lock(m_mutex);
        if (!activeUnlocked()) {
            return;
        }

        if (std::holds_alternative<TriggeredMoveCompleted>(event)) {
            if (m_feedHoldInProgress) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = false;
                m_state = ProgramExecutionState::Running;
            }

            return;
        }
        if (const auto *held = std::get_if<BackendHeld>(&event)) {
            if (held->epoch != m_epoch) {
                return;
            }
            if (m_error) {
                m_pendingControlledStopRequest.reset();
                m_controlledStopInProgress = false;
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = false;
                m_pendingFeedResumeRequest.reset();
                m_feedResumeInProgress = false;
                m_failureStopComplete = true;
                m_stoppedPosition = held->state.position;
                m_state = ProgramExecutionState::Holding;
                requestFailureAbort();

                return;
            }
            if (held->reason == BackendHoldReason::FeedHold) {
                m_pendingFeedHoldRequest.reset();
                m_feedHoldInProgress = false;
                m_feedHoldHeld = true;
                m_state = ProgramExecutionState::Paused;
            } else if (held->reason == BackendHoldReason::ControlledStop) {
                m_pendingControlledStopRequest.reset();
                m_controlledStopInProgress = false;
                m_stoppedPosition = held->state.position;
                m_state = ProgramExecutionState::StopComplete;
            }

            return;
        }
        const auto *completed = std::get_if<RequestCompleted>(&event);
        if (!completed) {
            return;
        }
        if (m_pendingAbortRequest
            && completed->request == *m_pendingAbortRequest) {
            m_pendingAbortRequest.reset();
            if (completed->succeeded) {
                m_state = ProgramExecutionState::Error;
            }
        } else if (m_pendingFeedHoldRequest && completed->request == *m_pendingFeedHoldRequest
            && !completed->succeeded) {
            m_pendingFeedHoldRequest.reset();
            m_feedHoldInProgress = false;
            m_state = ProgramExecutionState::Running;
        } else if (m_pendingFeedResumeRequest
                   && completed->request == *m_pendingFeedResumeRequest) {
            m_pendingFeedResumeRequest.reset();
            if (completed->succeeded) {
                m_feedHoldHeld = false;
                m_state = ProgramExecutionState::Running;
            } else {
                fail("motion backend rejected the feed-resume request");
            }
        } else if (m_pendingControlledStopRequest
                   && completed->request == *m_pendingControlledStopRequest
                   && !completed->succeeded) {
            m_pendingControlledStopRequest.reset();
            m_controlledStopInProgress = false;
            if (m_error) {
                requestFailureAbort();
            } else {
                fail("motion backend rejected the controlled-stop request");
            }
        }
    }

    void ProgramExecutionController::observeDriverState() {
        std::scoped_lock lock(m_mutex);

        observeDriverStateUnlocked();
    }

    void ProgramExecutionController::observeDriverStateUnlocked() {
        if (!activeUnlocked()) {
            return;
        }

        if (m_driver.state() == PreparedDriverState::ProgramPaused) {
            m_programPaused = true;
            m_state = ProgramExecutionState::Paused;
        } else if (m_driver.state() == PreparedDriverState::Error && m_driver.error()) {
            beginFailureStop(*m_driver.error());
        }
    }

    void ProgramExecutionController::finish() {
        std::scoped_lock lock(m_mutex);
        m_state = ProgramExecutionState::Inactive;
        m_stopRequested = false;
        m_controlledStopInProgress = false;
        m_programPaused = false;
        m_programResumeRequested = false;
        m_feedHoldRequested = false;
        m_feedHoldInProgress = false;
        m_feedHoldHeld = false;
        m_feedResumeRequested = false;
        m_feedResumeInProgress = false;
        m_failureStopComplete = false;
        m_pendingFeedHoldRequest.reset();
        m_pendingFeedResumeRequest.reset();
        m_pendingControlledStopRequest.reset();
        m_pendingAbortRequest.reset();
    }

    ProgramExecutionState ProgramExecutionController::state() const {
        std::scoped_lock lock(m_mutex);

        return m_state;
    }

    bool ProgramExecutionController::active() const {
        std::scoped_lock lock(m_mutex);

        return activeUnlocked();
    }

    bool ProgramExecutionController::activeUnlocked() const noexcept {
        return m_state != ProgramExecutionState::Inactive
            && m_state != ProgramExecutionState::StopComplete
            && m_state != ProgramExecutionState::Error;
    }

    bool ProgramExecutionController::paused() const {
        std::scoped_lock lock(m_mutex);

        return m_state == ProgramExecutionState::Paused;
    }

    bool ProgramExecutionController::programPaused() const {
        std::scoped_lock lock(m_mutex);

        return m_programPaused;
    }

    bool ProgramExecutionController::feedHoldInProgress() const {
        std::scoped_lock lock(m_mutex);

        return m_feedHoldInProgress;
    }

    bool ProgramExecutionController::feedResumeInProgress() const {
        std::scoped_lock lock(m_mutex);

        return m_feedResumeInProgress;
    }

    bool ProgramExecutionController::stopRequested() const {
        std::scoped_lock lock(m_mutex);

        return m_stopRequested;
    }

    ProgramOperationPresentation ProgramExecutionController::presentation() const {
        std::scoped_lock lock(m_mutex);
        if (m_state == ProgramExecutionState::Error) {
            return ProgramOperationPresentation::Failed;
        }
        if (m_state == ProgramExecutionState::StopComplete) {
            return ProgramOperationPresentation::Stopped;
        }
        if (m_state == ProgramExecutionState::Inactive) {
            return ProgramOperationPresentation::Inactive;
        }
        if (m_stopRequested || m_controlledStopInProgress) {
            return ProgramOperationPresentation::Stopping;
        }
        if (m_programPaused) {
            return ProgramOperationPresentation::ProgramPaused;
        }
        if (m_feedHoldInProgress) {
            return ProgramOperationPresentation::FeedHoldPending;
        }
        if (m_feedResumeInProgress) {
            return ProgramOperationPresentation::Resuming;
        }
        if (m_feedHoldHeld) {
            return ProgramOperationPresentation::Held;
        }

        return ProgramOperationPresentation::Running;
    }

    std::optional<position_t> ProgramExecutionController::stoppedPosition() const {
        std::scoped_lock lock(m_mutex);

        return m_stoppedPosition;
    }

    std::optional<std::string> ProgramExecutionController::error() const {
        std::scoped_lock lock(m_mutex);

        return m_error;
    }

    void ProgramExecutionController::beginFailureStop(std::string error) {
        if (m_error) {
            return;
        }

        m_error = std::move(error);
        m_stopRequested = true;
        m_programResumeRequested = false;
        m_feedHoldRequested = false;
        m_feedResumeRequested = false;
        m_state = ProgramExecutionState::Holding;
    }

    void ProgramExecutionController::fail(std::string error) {
        if (m_error && m_state != ProgramExecutionState::Error) {
            return;
        }

        m_error = std::move(error);
        m_state = ProgramExecutionState::Error;
    }
}
