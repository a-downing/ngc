#pragma once

#include <optional>
#include <string>

#include "machine/MachineConfiguration.h"
#include "pendant/PendantIntent.h"

namespace ngc::operator_control {
    struct TouchOffCommit {
        pendant::Axis axis = pendant::Axis::X;
        double workPosition = 0.0;
    };

    struct TouchOffSnapshot {
        std::optional<pendant::Axis> axis;
        double workPosition = 0.0;
    };

    // NRT staging for pendant axis touch-off. Wheel input edits the desired
    // coordinate at the current physical point; only an explicit double-click
    // publishes a commit.
    class TouchOffController {
    public:
        explicit TouchOffController(const MachineConfiguration &configuration);

        void consume(const pendant::Intent &intent);
        std::optional<TouchOffCommit> takeCommit();
        const TouchOffSnapshot &snapshot() const noexcept { return m_snapshot; }
        std::optional<std::string> takeError();

    private:
        double m_increment = 0.0;
        TouchOffSnapshot m_snapshot;
        std::optional<TouchOffCommit> m_commit;
        std::optional<std::string> m_error;
    };
}
