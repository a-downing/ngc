#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "machine/MotionBackend.h"
#include "machine/ExecutorDemandController.h"
#include "machine/ProgramOperationPresentation.h"

namespace ngc {
    class PreparedTrajectoryExecutionDriver;
    class SessionCommandQueue;

    enum class ProgramExecutionState {
        Inactive,
        Running,
        Holding,
        Paused,
        StopComplete,
        Error,
    };

    class ProgramExecutionController {
    public:
        ProgramExecutionController(ExecutorDemandController &demand,
                                   PreparedTrajectoryExecutionDriver &driver,
                                   SessionCommandQueue &commands);

        void begin(EpochId epoch);
        void service(const ExecutionSnapshot &snapshot, bool shutdownRequested);
        void observeBackendEvent(const ExecutionEvent &event);
        void observeDriverState();
        void finish();

        [[nodiscard]] ProgramExecutionState state() const;
        [[nodiscard]] bool active() const;
        [[nodiscard]] bool paused() const;
        [[nodiscard]] bool programPaused() const;
        [[nodiscard]] bool feedHoldInProgress() const;
        [[nodiscard]] bool feedResumeInProgress() const;
        [[nodiscard]] bool stopRequested() const;
        [[nodiscard]] ProgramOperationPresentation presentation() const;
        [[nodiscard]] std::optional<position_t> stoppedPosition() const;
        [[nodiscard]] std::optional<std::string> error() const;

    private:
        void consumeCommands();
        void requestControlledStop(const ExecutionSnapshot &snapshot);
        void requestProgramResume();
        void requestFeedHold();
        void requestFeedResume();
        void observeDriverStateUnlocked();
        void beginFailureStop(std::string error);
        [[nodiscard]] bool activeUnlocked() const noexcept;
        void fail(std::string error);

        ExecutorDemandController &m_demand;
        PreparedTrajectoryExecutionDriver &m_driver;
        SessionCommandQueue &m_commands;
        mutable std::mutex m_mutex;
        ProgramExecutionState m_state = ProgramExecutionState::Inactive;
        EpochId m_epoch = 0;
        bool m_stopRequested = false;
        bool m_controlledStopInProgress = false;
        bool m_programPaused = false;
        bool m_programResumeRequested = false;
        bool m_feedHoldRequested = false;
        bool m_feedHoldInProgress = false;
        bool m_feedHoldHeld = false;
        bool m_feedResumeRequested = false;
        bool m_feedResumeInProgress = false;
        bool m_failureStopComplete = false;
        std::optional<position_t> m_stoppedPosition;
        std::optional<std::string> m_error;
    };
}
