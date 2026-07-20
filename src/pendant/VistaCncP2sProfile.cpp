#include "pendant/VistaCncP2sProfile.h"

#include <type_traits>

namespace ngc::pendant::vista_cnc_p2s {
    namespace {
        Mode mode(const InputState &state) noexcept {
            if(state.machineOff) return Mode::Off;
            switch(state.leftSelector) {
                case LeftSelector::Step: return Mode::Step;
                case LeftSelector::Velocity: return Mode::Velocity;
                case LeftSelector::Continuous: return Mode::Continuous;
                case LeftSelector::Speed: return Mode::Speed;
                case LeftSelector::Zero: return Mode::Zero;
                case LeftSelector::Function: return Mode::Function;
                case LeftSelector::Transient:
                case LeftSelector::Unknown: return Mode::None;
            }
            return Mode::None;
        }

        std::optional<Axis> selectedAxis(const RightSelector selector) noexcept {
            switch(selector) {
                case RightSelector::X_F1: return Axis::X;
                case RightSelector::Y_F2: return Axis::Y;
                case RightSelector::Z_F3: return Axis::Z;
                case RightSelector::A_F4: return Axis::A;
                case RightSelector::Transient:
                case RightSelector::StartPause:
                case RightSelector::StopBack:
                case RightSelector::Spindle: return std::nullopt;
            }
            return std::nullopt;
        }

        void initialize(ProfileSnapshot &snapshot, const InputState &state) {
            snapshot.connected = true;
            snapshot.mode = mode(state);
            snapshot.selectedAxis = selectedAxis(state.rightSelector);
            snapshot.rightSelectorTransient = state.rightSelector == RightSelector::Transient;
            snapshot.wheelButton = state.wheelButton;
            snapshot.machineOff = state.machineOff;
            snapshot.emergencyStop = state.emergencyStop;
        }
    }

    std::vector<Intent> Profile::consume(const ManagerEvent &event) {
        std::vector<Intent> intents;
        std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, Connected>) {
                initialize(m_snapshot, value.state);
                m_snapshot.wheelArmed = false;
                m_canArmWheel = value.state.wheelButton == WheelButton::Released;
                intents.emplace_back(ConnectionChanged { true, {} });
                intents.emplace_back(MachineOffChanged { m_snapshot.machineOff });
                intents.emplace_back(EmergencyStopSwitchChanged { m_snapshot.emergencyStop });
                intents.emplace_back(SelectionChanged {
                    m_snapshot.mode, m_snapshot.selectedAxis, m_snapshot.rightSelectorTransient,
                });
                if(m_snapshot.emergencyStop)
                    intents.emplace_back(CancelPendantActivity { CancelReason::EmergencyStop });
                else if(m_snapshot.machineOff)
                    intents.emplace_back(CancelPendantActivity { CancelReason::MachineOff });
            } else if constexpr(std::same_as<T, InputChanged>) {
                const auto previous = m_snapshot;
                m_snapshot.connected = true;
                m_snapshot.mode = mode(value.state);
                m_snapshot.rightSelectorTransient = value.state.rightSelector == RightSelector::Transient;
                if(!m_snapshot.rightSelectorTransient)
                    m_snapshot.selectedAxis = selectedAxis(value.state.rightSelector);
                m_snapshot.wheelButton = value.state.wheelButton;
                m_snapshot.machineOff = value.state.machineOff;
                m_snapshot.emergencyStop = value.state.emergencyStop;

                const auto selectionChanged = m_snapshot.mode != previous.mode
                    || m_snapshot.selectedAxis != previous.selectedAxis
                    || m_snapshot.rightSelectorTransient != previous.rightSelectorTransient;
                const auto safetyLevelChanged = m_snapshot.machineOff != previous.machineOff
                    || m_snapshot.emergencyStop != previous.emergencyStop;
                if(m_snapshot.wheelButton == WheelButton::Released) {
                    m_snapshot.wheelArmed = false;
                    m_canArmWheel = true;
                } else if(m_snapshot.wheelButton == WheelButton::Pressed
                          && previous.wheelButton == WheelButton::Released && m_canArmWheel) {
                    m_snapshot.wheelArmed = true;
                } else if(m_snapshot.wheelButton != WheelButton::Pressed) {
                    m_snapshot.wheelArmed = false;
                }
                if(selectionChanged || safetyLevelChanged) m_snapshot.wheelArmed = false;
                if(selectionChanged || safetyLevelChanged
                   || m_snapshot.wheelButton != WheelButton::Pressed)
                    m_velocityDirection = 0;
                if(m_snapshot.machineOff != previous.machineOff)
                    intents.emplace_back(MachineOffChanged { m_snapshot.machineOff });
                if(m_snapshot.emergencyStop != previous.emergencyStop)
                    intents.emplace_back(EmergencyStopSwitchChanged { m_snapshot.emergencyStop });
                if(selectionChanged)
                    intents.emplace_back(SelectionChanged {
                        m_snapshot.mode, m_snapshot.selectedAxis, m_snapshot.rightSelectorTransient,
                    });

                if(m_snapshot.emergencyStop && !previous.emergencyStop)
                    intents.emplace_back(CancelPendantActivity { CancelReason::EmergencyStop });
                else if(m_snapshot.machineOff && !previous.machineOff)
                    intents.emplace_back(CancelPendantActivity { CancelReason::MachineOff });
                else if(previous.mode != Mode::Step
                        && previous.wheelButton == WheelButton::Pressed
                        && m_snapshot.wheelButton != WheelButton::Pressed)
                    intents.emplace_back(CancelPendantActivity { CancelReason::ButtonReleased });
                else if(selectionChanged)
                    intents.emplace_back(CancelPendantActivity { CancelReason::SelectionChanged });

                const auto actionsAllowed = !m_snapshot.machineOff && !m_snapshot.emergencyStop
                    && !m_snapshot.rightSelectorTransient && !selectionChanged && !safetyLevelChanged;
                const auto wheelChanged = contains(value.changes, InputChange::Wheel)
                    && value.state.wheelDelta != 0;
                if(wheelChanged) m_velocityDirection = value.state.wheelDelta < 0 ? -1 : 1;
                if(actionsAllowed && wheelChanged && m_snapshot.mode == Mode::Step
                   && m_snapshot.selectedAxis) {
                    const auto increment = m_snapshot.wheelButton == WheelButton::Pressed
                            || m_snapshot.wheelButton == WheelButton::DoublePressed
                        ? JogIncrement::Coarse : JogIncrement::Fine;
                    intents.emplace_back(JogWheel {
                        *m_snapshot.selectedAxis, value.state.wheelDelta, increment,
                    });
                }
                const auto wheelActionsAllowed = actionsAllowed
                    && m_snapshot.wheelButton == WheelButton::Pressed && m_snapshot.wheelArmed;
                const auto velocityChanged = wheelChanged
                    || contains(value.changes, InputChange::WheelRate);
                if(wheelActionsAllowed && velocityChanged && m_snapshot.mode == Mode::Velocity
                   && m_snapshot.selectedAxis) {
                    constexpr std::int32_t RATE_CODE_TO_COUNTS_PER_SECOND = 4;
                    intents.emplace_back(JogVelocity {
                        *m_snapshot.selectedAxis,
                        m_velocityDirection * static_cast<std::int32_t>(value.state.wheelRateCode)
                            * RATE_CODE_TO_COUNTS_PER_SECOND,
                    });
                }
                if(wheelActionsAllowed && wheelChanged) {
                    if(m_snapshot.mode == Mode::Zero && m_snapshot.selectedAxis)
                        intents.emplace_back(AdjustTouchOff {
                            *m_snapshot.selectedAxis, value.state.wheelDelta,
                        });
                    else if(m_snapshot.mode == Mode::Speed)
                        intents.emplace_back(AdjustFeedOverride { value.state.wheelDelta });
                }
                if(actionsAllowed && m_snapshot.mode == Mode::Zero && m_snapshot.selectedAxis
                   && contains(value.changes, InputChange::WheelButton)
                   && m_snapshot.wheelButton == WheelButton::DoublePressed)
                    intents.emplace_back(CommitTouchOff { *m_snapshot.selectedAxis });
            } else if constexpr(std::same_as<T, Disconnected>) {
                m_snapshot.connected = false;
                m_snapshot.wheelArmed = false;
                m_canArmWheel = false;
                intents.emplace_back(CancelPendantActivity { CancelReason::Disconnected });
                intents.emplace_back(ConnectionChanged { false, value.error.message });
            } else if constexpr(std::same_as<T, Stopped>) {
                m_snapshot.connected = false;
                m_snapshot.wheelArmed = false;
                m_canArmWheel = false;
                intents.emplace_back(CancelPendantActivity { CancelReason::Stopped });
                intents.emplace_back(ConnectionChanged { false, {} });
            }
        }, event);
        return intents;
    }
}
