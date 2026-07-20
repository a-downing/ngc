#pragma once

#include <optional>
#include <vector>

#include "pendant/VistaCncP2sManager.h"
#include "pendant/PendantIntent.h"

namespace ngc::pendant::vista_cnc_p2s {
    struct ProfileSnapshot {
        bool connected = false;
        Mode mode = Mode::None;
        std::optional<Axis> selectedAxis;
        bool rightSelectorTransient = false;
        WheelButton wheelButton = WheelButton::Released;
        bool wheelArmed = false;
        bool machineOff = false;
        bool emergencyStop = false;
    };

    class Profile {
    public:
        std::vector<Intent> consume(const ManagerEvent &event);
        const ProfileSnapshot &snapshot() const noexcept { return m_snapshot; }

    private:
        void resetVelocityEstimator() noexcept;

        ProfileSnapshot m_snapshot;
        bool m_canArmWheel = false;
        std::int32_t m_velocityDirection = 0;
        std::int32_t m_lastVelocityCountsPerSecond = 0;
        std::optional<std::chrono::steady_clock::time_point> m_previousVelocityDetentTime;
        std::optional<std::chrono::steady_clock::time_point> m_velocityDeadline;
        double m_smoothedVelocityDetentPeriodSeconds = 0.0;
    };
}
