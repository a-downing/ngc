#pragma once

namespace ngc {
    enum class SessionCommandRejection {
        None,
        SessionNotPowered,
        MotionOwned,
        NoMotionOwner,
        EmptyProgram,
        ToolTableUnavailable,
        HomingUnavailable,
        InvalidJogRequest,
        ProgramNotRunning,
        ProgramAlreadyPaused,
        ProgramNotPaused,
        FeedHoldInProgress,
        FeedResumeInProgress,
        CommandAlreadyQueued,
        CommandQueueFull,
        CommandUnavailable,
    };

    struct SessionCommandResult {
        SessionCommandRejection rejection = SessionCommandRejection::None;

        operator bool() const noexcept {
            return rejection == SessionCommandRejection::None;
        }
    };
}
