#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ngc::pendant::vista_cnc_p2s {
    inline constexpr std::uint16_t VENDOR_ID = 0x04d8;
    inline constexpr std::uint16_t PRODUCT_ID = 0xfce2;
    inline constexpr std::size_t INPUT_PAYLOAD_SIZE = 8;

    using InputReport = std::array<std::uint8_t, INPUT_PAYLOAD_SIZE>;

    enum class LeftSelector : std::uint8_t {
        Transient = 0,
        Step = 1,
        Velocity = 2,
        Continuous = 3,
        Speed = 4,
        Zero = 5,
        Function = 6,
        Unknown = 7,
    };

    enum class RightSelector : std::uint8_t {
        Transient = 0,
        X_F1 = 1,
        Y_F2 = 2,
        Z_F3 = 3,
        StartPause = 4,
        StopBack = 5,
        Spindle = 6,
        A_F4 = 7,
    };

    enum class WheelButton : std::uint8_t {
        Released = 0,
        Pressed = 1,
        Unknown = 2,
        DoublePressed = 3,
    };

    struct InputState {
        InputReport raw{};
        std::uint8_t wheelPosition = 0;
        std::int8_t wheelDelta = 0;
        bool wheelDeltaValid = false;
        // Firmware samples wheel counts in roughly 50 ms windows and reports
        // five times the unsigned count in that window.
        std::uint8_t wheelRateCode = 0;
        // Unsigned, wrapping firmware motion accumulator. It is diagnostic,
        // not an authoritative velocity value.
        std::uint16_t wheelMotionAccumulator = 0;
        LeftSelector leftSelector = LeftSelector::Transient;
        RightSelector rightSelector = RightSelector::Transient;
        WheelButton wheelButton = WheelButton::Released;
        bool machineOff = false;
        bool emergencyStop = false;
        bool heartbeat = false;
        std::array<std::uint8_t, 2> unknownTail{};
    };

    class InputDecoder {
    public:
        InputState decode(const InputReport &report) noexcept;
        void reset() noexcept;

    private:
        std::uint8_t m_previousWheelPosition = 0;
        bool m_havePreviousWheelPosition = false;
    };

    std::string_view name(LeftSelector selector) noexcept;
    std::string_view name(RightSelector selector) noexcept;
}
