#pragma once

namespace ngc {
    enum class ProgramOperationPresentation {
        Inactive,
        Running,
        FeedHoldPending,
        Held,
        Resuming,
        ProgramPaused,
        Stopping,
        Completed,
        Stopped,
        Failed,
    };
}
