#include "pendant/VistaCncP2sProtocol.h"

namespace ngc::pendant::vista_cnc_p2s {
    InputState InputDecoder::decode(const InputReport &report) noexcept {
        InputState result {
            .raw = report,
            .wheelPosition = report[0],
            .wheelRateCode = report[1],
            .wheelMotionAccumulator = static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(report[4]) << 8 | report[5]),
            .leftSelector = static_cast<LeftSelector>(report[3] & 0x07),
            .rightSelector = static_cast<RightSelector>(report[2] & 0x07),
            .wheelButton = static_cast<WheelButton>((report[2] >> 6) & 0x03),
            .machineOff = (report[2] & 0x10) != 0,
            .emergencyStop = (report[2] & 0x20) != 0,
            .heartbeat = (report[3] & 0x10) != 0,
            .unknownTail = { report[6], report[7] },
        };
        if(m_havePreviousWheelPosition) {
            const auto wrappingDelta = static_cast<std::uint8_t>(
                report[0] - m_previousWheelPosition);
            const auto signedDelta = wrappingDelta <= 0x7f
                ? static_cast<std::int16_t>(wrappingDelta)
                : static_cast<std::int16_t>(wrappingDelta) - 0x100;
            result.wheelDelta = static_cast<std::int8_t>(signedDelta);
            result.wheelDeltaValid = true;
        }
        m_previousWheelPosition = report[0];
        m_havePreviousWheelPosition = true;
        return result;
    }

    void InputDecoder::reset() noexcept {
        m_previousWheelPosition = 0;
        m_havePreviousWheelPosition = false;
    }

    std::string_view name(const LeftSelector selector) noexcept {
        switch(selector) {
            case LeftSelector::Transient: return "transient";
            case LeftSelector::Step: return "step";
            case LeftSelector::Velocity: return "velocity";
            case LeftSelector::Continuous: return "continuous";
            case LeftSelector::Speed: return "speed";
            case LeftSelector::Zero: return "zero";
            case LeftSelector::Function: return "function";
            case LeftSelector::Unknown: return "unknown";
        }
        return "unknown";
    }

    std::string_view name(const RightSelector selector) noexcept {
        switch(selector) {
            case RightSelector::Transient: return "transient";
            case RightSelector::X_F1: return "X/F1";
            case RightSelector::Y_F2: return "Y/F2";
            case RightSelector::Z_F3: return "Z/F3";
            case RightSelector::StartPause: return "start/pause";
            case RightSelector::StopBack: return "stop/back";
            case RightSelector::Spindle: return "spindle";
            case RightSelector::A_F4: return "A/F4";
        }
        return "unknown";
    }
}
