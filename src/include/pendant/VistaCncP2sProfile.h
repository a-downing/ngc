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
        ProfileSnapshot m_snapshot;
        bool m_canArmWheel = false;
        std::int32_t m_velocityDirection = 0;
    };
}
