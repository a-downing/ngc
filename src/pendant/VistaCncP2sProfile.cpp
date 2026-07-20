#include "pendant/VistaCncP2sProfile.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

    void Profile::resetVelocityEstimator() noexcept {
        m_velocityDirection = 0;
        m_lastVelocityCountsPerSecond = 0;
        m_previousVelocityDetentTime.reset();
        m_velocityDeadline.reset();
        m_smoothedVelocityDetentPeriodSeconds = 0.0;
    }

    std::vector<Intent> Profile::consume(const ManagerEvent &event) {
        std::vector<Intent> intents;
        std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::same_as<T, Connected>) {
                initialize(m_snapshot, value.state);
                m_snapshot.wheelArmed = false;
                m_canArmWheel = value.state.wheelButton == WheelButton::Released;
                resetVelocityEstimator();
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
                    resetVelocityEstimator();
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
                    || contains(value.changes, InputChange::WheelRate)
                    || contains(value.changes, InputChange::WheelMotionAccumulator);
                if(wheelActionsAllowed && velocityChanged && m_snapshot.mode == Mode::Velocity
                   && m_snapshot.selectedAxis) {
                    using Seconds = std::chrono::duration<double>;
                    constexpr double SMOOTHING_PREVIOUS_WEIGHT = 0.5;
                    constexpr double MAX_PAIR_PERIOD_SECONDS = 0.3;
                    constexpr double MIN_DEADLINE_SECONDS = 0.05;
                    constexpr double MAX_DEADLINE_SECONDS = 0.2;
                    constexpr double DEADLINE_PERIODS = 1.5;

                    if(wheelChanged) {
                        const auto direction = value.state.wheelDelta < 0 ? -1 : 1;
                        const auto elapsedSeconds = m_previousVelocityDetentTime
                            ? Seconds(value.arrivalTime - *m_previousVelocityDetentTime).count()
                            : 0.0;
                        const auto sameDirection = direction == m_velocityDirection;
                        const auto haveUsablePair = sameDirection && elapsedSeconds > 0.0
                            && elapsedSeconds <= MAX_PAIR_PERIOD_SECONDS;
                        if(!haveUsablePair) {
                            const auto wasMoving = m_lastVelocityCountsPerSecond != 0;
                            resetVelocityEstimator();
                            m_velocityDirection = direction;
                            m_previousVelocityDetentTime = value.arrivalTime;
                            m_velocityDeadline = value.arrivalTime
                                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    Seconds(MAX_PAIR_PERIOD_SECONDS));
                            if(wasMoving)
                                intents.emplace_back(JogVelocity {
                                    *m_snapshot.selectedAxis, 0,
                                });
                        } else {
                            const auto counts = std::abs(
                                static_cast<int>(value.state.wheelDelta));
                            const auto samplePeriod = elapsedSeconds / counts;
                            m_smoothedVelocityDetentPeriodSeconds
                                = m_smoothedVelocityDetentPeriodSeconds > 0.0
                                ? SMOOTHING_PREVIOUS_WEIGHT
                                        * m_smoothedVelocityDetentPeriodSeconds
                                    + (1.0 - SMOOTHING_PREVIOUS_WEIGHT) * samplePeriod
                                : samplePeriod;
                            m_velocityDirection = direction;
                            m_previousVelocityDetentTime = value.arrivalTime;
                            m_lastVelocityCountsPerSecond = direction
                                * std::max(1, static_cast<int>(std::lround(
                                    1.0 / m_smoothedVelocityDetentPeriodSeconds)));
                            const auto deadlineSeconds = std::clamp(
                                DEADLINE_PERIODS * m_smoothedVelocityDetentPeriodSeconds,
                                MIN_DEADLINE_SECONDS, MAX_DEADLINE_SECONDS);
                            m_velocityDeadline = value.arrivalTime
                                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    Seconds(deadlineSeconds));
                            intents.emplace_back(JogVelocity {
                                *m_snapshot.selectedAxis, m_lastVelocityCountsPerSecond,
                            });
                        }
                    } else if(m_velocityDeadline && value.arrivalTime >= *m_velocityDeadline) {
                        const auto wasMoving = m_lastVelocityCountsPerSecond != 0;
                        resetVelocityEstimator();
                        if(wasMoving)
                            intents.emplace_back(JogVelocity { *m_snapshot.selectedAxis, 0 });
                    } else if(m_lastVelocityCountsPerSecond != 0) {
                        intents.emplace_back(JogVelocity {
                            *m_snapshot.selectedAxis, m_lastVelocityCountsPerSecond,
                        });
                    }
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
                resetVelocityEstimator();
                intents.emplace_back(CancelPendantActivity { CancelReason::Disconnected });
                intents.emplace_back(ConnectionChanged { false, value.error.message });
            } else if constexpr(std::same_as<T, Stopped>) {
                m_snapshot.connected = false;
                m_snapshot.wheelArmed = false;
                m_canArmWheel = false;
                resetVelocityEstimator();
                intents.emplace_back(CancelPendantActivity { CancelReason::Stopped });
                intents.emplace_back(ConnectionChanged { false, {} });
            }
        }, event);
        return intents;
    }
}
