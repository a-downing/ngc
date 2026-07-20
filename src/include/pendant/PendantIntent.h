#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace ngc::pendant {
    enum class Axis : std::uint8_t { X, Y, Z, A, B, C };

    enum class Mode : std::uint8_t {
        None,
        Step,
        Velocity,
        Continuous,
        Speed,
        Zero,
        Function,
        Off,
    };

    enum class CancelReason : std::uint8_t {
        SelectionChanged,
        ButtonReleased,
        MachineOff,
        EmergencyStop,
        Disconnected,
        Stopped,
    };

    struct ConnectionChanged {
        bool connected = false;
        std::string detail;
    };

    struct SelectionChanged {
        Mode mode = Mode::None;
        std::optional<Axis> axis;
        // The stable selection is retained during the P2S selector's observed
        // break-before-make zero, but actions are inhibited while transient.
        bool selectorTransient = false;
    };

    struct MachineOffChanged { bool active = false; };

    // This is the level of a userspace pendant switch, not an authoritative
    // machine safety input and never permission to reset or enable motion.
    struct EmergencyStopSwitchChanged { bool active = false; };

    struct CancelPendantActivity { CancelReason reason = CancelReason::SelectionChanged; };

    enum class JogIncrement : std::uint8_t { Fine, Coarse };

    struct JogWheel {
        Axis axis = Axis::X;
        std::int32_t counts = 0;
        JogIncrement increment = JogIncrement::Fine;
    };

    // Signed wheel rate decoded by a device profile. Zero requests a stop.
    struct JogVelocity {
        Axis axis = Axis::X;
        std::int32_t countsPerSecond = 0;
    };

    struct AdjustTouchOff {
        Axis axis = Axis::X;
        std::int32_t counts = 0;
    };

    struct CommitTouchOff { Axis axis = Axis::X; };
    struct AdjustFeedOverride { std::int32_t counts = 0; };

    using Intent = std::variant<ConnectionChanged, SelectionChanged, MachineOffChanged,
                                EmergencyStopSwitchChanged, CancelPendantActivity, JogWheel, JogVelocity,
                                AdjustTouchOff, CommitTouchOff, AdjustFeedOverride>;
}
