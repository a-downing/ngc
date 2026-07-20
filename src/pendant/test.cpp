#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

#include "pendant/VistaCncP2sDriver.h"
#include "pendant/VistaCncP2sDisplay.h"
#include "pendant/VistaCncP2sManager.h"
#include "pendant/VistaCncP2sProfile.h"
#include "pendant/VistaCncP2sProtocol.h"

namespace {
    void require(const bool condition, const std::string_view message) {
        if(!condition) throw std::runtime_error(std::string(message));
    }

    template<typename T>
    const T *findIntent(const std::vector<ngc::pendant::Intent> &intents) {
        for(const auto &intent : intents)
            if(const auto *value = std::get_if<T>(&intent)) return value;
        return nullptr;
    }

    class MockHidTransport final : public ngc::pendant::HidTransport {
    public:
        std::size_t inputLength = 9;
        std::size_t outputLength = 21;
        std::vector<std::vector<std::uint8_t>> inputs;
        std::vector<std::vector<std::uint8_t>> outputs;
        std::size_t nextInput = 0;
        std::atomic_bool cancelled = false;

        std::size_t inputReportLength() const noexcept override { return inputLength; }
        std::size_t outputReportLength() const noexcept override { return outputLength; }

        std::expected<std::size_t, ngc::pendant::HidError>
        readInputReport(const std::span<std::uint8_t> report,
                        std::chrono::milliseconds) override {
            if(cancelled.load())
                return std::unexpected(ngc::pendant::HidError {
                    ngc::pendant::HidErrorCode::Cancelled, 0, "cancelled",
                });
            if(nextInput == inputs.size())
                return std::unexpected(ngc::pendant::HidError {
                    ngc::pendant::HidErrorCode::Disconnected, 0, "no input",
                });
            const auto &input = inputs[nextInput++];
            if(input.size() != report.size())
                return std::unexpected(ngc::pendant::HidError {
                    ngc::pendant::HidErrorCode::InvalidReport, 0, "wrong input size",
                });
            std::ranges::copy(input, report.begin());
            return input.size();
        }

        std::expected<void, ngc::pendant::HidError>
        writeOutputReport(const std::span<const std::uint8_t> report) override {
            outputs.emplace_back(report.begin(), report.end());
            return {};
        }

        void cancel() noexcept override { cancelled = true; }
    };

    class BlockingMockHidTransport final : public ngc::pendant::HidTransport {
    public:
        std::size_t inputReportLength() const noexcept override { return 9; }
        std::size_t outputReportLength() const noexcept override { return 21; }

        std::expected<std::size_t, ngc::pendant::HidError>
        readInputReport(const std::span<std::uint8_t> report,
                        const std::chrono::milliseconds timeout) override {
            std::unique_lock lock(m_mutex);
            if(!m_sentInitial) {
                const std::array<std::uint8_t, 9> initial { 0, 10, 0, 1, 1, 0, 0, 0, 0 };
                std::ranges::copy(initial, report.begin());
                m_sentInitial = true;
                return initial.size();
            }
            if(timeout == std::chrono::milliseconds::max())
                m_cancelled.wait(lock, [&] { return m_cancelRequested; });
            else if(!m_cancelled.wait_for(lock, timeout, [&] { return m_cancelRequested; }))
                return std::unexpected(ngc::pendant::HidError {
                    ngc::pendant::HidErrorCode::TimedOut, 0, "timed out",
                });
            return std::unexpected(ngc::pendant::HidError {
                ngc::pendant::HidErrorCode::Cancelled, 0, "cancelled",
            });
        }

        std::expected<void, ngc::pendant::HidError>
        writeOutputReport(const std::span<const std::uint8_t> report) override {
            {
                std::scoped_lock lock(m_mutex);
                m_outputs.emplace_back(report.begin(), report.end());
            }
            m_outputChanged.notify_all();
            return {};
        }

        bool waitForOutputCount(const std::size_t count,
                                const std::chrono::milliseconds timeout) {
            std::unique_lock lock(m_mutex);
            return m_outputChanged.wait_for(lock, timeout, [&] {
                return m_outputs.size() >= count;
            });
        }

        std::string latestDisplayText() {
            std::scoped_lock lock(m_mutex);
            if(m_outputs.empty()) return {};
            return std::string(reinterpret_cast<const char *>(m_outputs.back().data() + 1), 16);
        }

        void cancel() noexcept override {
            {
                std::scoped_lock lock(m_mutex);
                m_cancelRequested = true;
            }
            m_cancelled.notify_all();
        }

    private:
        std::mutex m_mutex;
        std::condition_variable m_cancelled;
        std::condition_variable m_outputChanged;
        bool m_sentInitial = false;
        bool m_cancelRequested = false;
        std::vector<std::vector<std::uint8_t>> m_outputs;
    };

    void testInputDecoder() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        protocol::InputDecoder decoder;

        protocol::InputReport first { 0xff, 0x05, 0xf7, 0x15, 0x02, 0x22, 0xaa, 0x55 };
        const auto initial = decoder.decode(first);
        require(!initial.wheelDeltaValid && initial.wheelDelta == 0,
                "the first report must establish the wheel baseline");
        require(initial.leftSelector == protocol::LeftSelector::Zero,
                "left selector should decode from byte 3");
        require(initial.rightSelector == protocol::RightSelector::A_F4,
                "right selector should decode from byte 2");
        require(initial.wheelButton == protocol::WheelButton::DoublePressed,
                "wheel button bits should preserve the firmware double-press state");
        require(initial.machineOff && initial.emergencyStop && initial.heartbeat,
                "level and heartbeat bits should decode independently");
        require(initial.wheelRateCode == 0x05
               && initial.wheelMotionAccumulator == 0x0222,
                "wheel rate and accumulator fields should retain their observed byte order");
        require(initial.unknownTail == std::array<std::uint8_t, 2> { 0xaa, 0x55 },
                "unknown tail bytes should be preserved");

        auto wrappedForward = first;
        wrappedForward[0] = 0x00;
        const auto forward = decoder.decode(wrappedForward);
        require(forward.wheelDeltaValid && forward.wheelDelta == 1,
                "wheel delta should wrap forward from 255 to 0");

        auto wrappedBackward = wrappedForward;
        wrappedBackward[0] = 0xff;
        const auto backward = decoder.decode(wrappedBackward);
        require(backward.wheelDelta == -1,
                "wheel delta should wrap backward from 0 to 255");

        decoder.reset();
        const auto reconnected = decoder.decode(wrappedBackward);
        require(!reconnected.wheelDeltaValid && reconnected.wheelDelta == 0,
                "reset must discard the old wheel baseline");

        protocol::InputReport transient{};
        transient[2] = 0;
        transient[3] = 6;
        const auto selectorState = decoder.decode(transient);
        require(selectorState.rightSelector == protocol::RightSelector::Transient,
                "right selector zero must remain an explicit transient state");
        require(selectorState.leftSelector == protocol::LeftSelector::Function
               && !selectorState.machineOff,
                "the raw function selector must not be synthesized into off");
    }

    void testDriverFramesWindowsReportsAndDisplay() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        auto transport = std::make_unique<MockHidTransport>();
        auto *observedTransport = transport.get();
        transport->inputs = {
            { 0, 0xff, 0x05, 0x31, 0x14, 0x02, 0x22, 0xaa, 0x55 },
            { 0, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 },
        };
        protocol::Driver driver(std::move(transport));

        const auto first = driver.read();
        require(first.has_value() && !first->wheelDeltaValid,
                "driver should strip report ID zero and establish a wheel baseline");
        require(first->machineOff && first->emergencyStop && first->heartbeat,
                "driver should pass the stripped payload to the protocol decoder");
        const auto second = driver.read();
        require(second.has_value() && second->wheelDeltaValid && second->wheelDelta == 1,
                "driver should retain decoder state across Windows input reports");

        require(driver.writeDisplay("12345678ABCDEFGHmore").has_value(),
                "driver should write the first display report");
        require(driver.writeDisplay("X").has_value(),
                "driver should write a subsequent display report");
        require(observedTransport->outputs.size() == 2,
                "mock transport should receive both display reports");
        const auto &firstDisplay = observedTransport->outputs[0];
        require(firstDisplay.size() == 21 && firstDisplay[0] == 0,
                "Windows display reports should include report ID zero");
        require(std::string_view(reinterpret_cast<const char *>(firstDisplay.data() + 1), 16)
                    == "12345678ABCDEFGH",
                "display text should be truncated to the two eight-character rows");
        require(firstDisplay[17] == static_cast<std::uint8_t>(' ')
               && firstDisplay[18] == 0 && firstDisplay[19] == 0 && firstDisplay[20] == 0,
                "first display report should preserve the observed reserved and sequence bytes");
        const auto &secondDisplay = observedTransport->outputs[1];
        require(secondDisplay[1] == static_cast<std::uint8_t>('X')
               && secondDisplay[2] == static_cast<std::uint8_t>(' ')
               && secondDisplay[18] == 1,
                "display reports should be space-padded and increment their sequence");

        observedTransport->inputs.push_back(
            { 0, 'X', ' ', ' ', ' ', ' ', ' ', ' ', ' ' });
        observedTransport->inputs.push_back(
            { 0, 'X', ' ', ' ', ' ', 0, 0, 0, 0 });
        observedTransport->inputs.push_back(
            { 0, '1', '2', '3', '4', '5', '6', '7', '8' });
        observedTransport->inputs.push_back(
            { 0, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 });
        const auto afterEcho = driver.read();
        require(afterEcho && afterEcho->raw[0] == 0x01,
                "driver should discard current, partial, and recently superseded LCD echoes");

        driver.cancel();
        require(observedTransport->cancelled,
                "driver cancellation should release the underlying transport");
    }

    void testPositionDisplayUsesBothEightCharacterRows() {
        using ngc::pendant::Axis;
        using ngc::pendant::vista_cnc_p2s::formatPositionDisplay;
        const auto display = formatPositionDisplay("G54", Axis::X, 99.9999);
        require(display == "G54 X   +99.9999",
                "position display should identify the active WCS and selected axis above its value");
        require(formatPositionDisplay("G59.3", Axis::Z, -123.456).size() == 16,
                "position display should remain exactly two LCD rows at wider values");
        require(ngc::pendant::vista_cnc_p2s::formatZeroDisplay(
                    "G54", Axis::X, 99.999, -0.001) == "X+99.999G54-.001",
                "zero display should show live position above the staged WCS coordinate");
    }

    void testManagerPublishesOrderedMeaningfulChanges() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        auto transport = std::make_unique<MockHidTransport>();
        transport->inputs = {
            { 0, 10, 0, 0x01, 0x01, 0, 0, 0, 0 },
            { 0, 12, 0, 0x41, 0x01, 0x02, 0x22, 0, 0 },
            { 0, 12, 0, 0x41, 0x11, 0x02, 0x22, 0, 0 },
        };
        auto driver = std::make_unique<protocol::Driver>(std::move(transport));
        protocol::Manager manager(std::move(driver));

        auto event = manager.waitTakeEvent(std::chrono::seconds(1));
        const auto *connected = event ? std::get_if<protocol::Connected>(&*event) : nullptr;
        require(connected && connected->reportSequence == 1
                && connected->cumulativeWheelCounts == 0
                && connected->arrivalTime.time_since_epoch().count() != 0,
                "manager should publish the initial decoded state as a connection event");

        event = manager.waitTakeEvent(std::chrono::seconds(1));
        const auto *changed = event ? std::get_if<protocol::InputChanged>(&*event) : nullptr;
        require(changed && changed->reportSequence == 2
                && changed->cumulativeWheelCounts == 2,
                "manager should preserve ordered cumulative wheel counts");
        require(protocol::contains(changed->changes, protocol::InputChange::Wheel)
                && protocol::contains(changed->changes, protocol::InputChange::WheelButton)
                && protocol::contains(
                    changed->changes, protocol::InputChange::WheelMotionAccumulator),
                "manager should report every meaningful change from one input report atomically");
        require(!protocol::contains(changed->changes, protocol::InputChange::LeftSelector),
                "manager should not mark unchanged controls");

        event = manager.waitTakeEvent(std::chrono::seconds(1));
        require(event && std::holds_alternative<protocol::Disconnected>(*event),
                "heartbeat-only reports should update the snapshot without creating control events");
        const auto snapshot = manager.snapshot();
        require(!snapshot.connected && snapshot.reportSequence == 3
                && snapshot.cumulativeWheelCounts == 2 && snapshot.state.has_value(),
                "manager snapshot should retain the last complete input state after disconnect");
    }

    void testManagerCancellationReleasesBlockingRead() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        auto driver = std::make_unique<protocol::Driver>(
            std::make_unique<BlockingMockHidTransport>());
        protocol::Manager manager(std::move(driver));
        const auto connected = manager.waitTakeEvent(std::chrono::seconds(1));
        require(connected && std::holds_alternative<protocol::Connected>(*connected),
                "blocking manager test should connect before stopping");
        manager.stop();
        const auto stopped = manager.waitTakeEvent(std::chrono::seconds(1));
        require(stopped && std::holds_alternative<protocol::Stopped>(*stopped),
                "stopping the manager should cancel and join its blocking input read");
        require(manager.snapshot().stopped,
                "manager snapshot should distinguish an intentional stop from disconnect");
    }

    void testManagerPublishesDisplayChangesAsynchronouslyAndRefreshes() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        auto transport = std::make_unique<BlockingMockHidTransport>();
        auto *observed = transport.get();
        auto driver = std::make_unique<protocol::Driver>(std::move(transport));
        protocol::Manager manager(std::move(driver));
        const auto connected = manager.waitTakeEvent(std::chrono::seconds(1));
        require(connected && std::holds_alternative<protocol::Connected>(*connected),
                "asynchronous display test should connect before publishing");

        require(manager.setDisplay("FIRST").has_value(),
                "display publication should enqueue without performing caller-thread HID I/O");
        require(observed->waitForOutputCount(1, std::chrono::seconds(1))
                && observed->latestDisplayText().starts_with("FIRST"),
                "display worker should send a new value immediately");
        require(manager.setDisplay("SECOND").has_value(),
                "a replacement display value should be accepted");
        require(observed->waitForOutputCount(2, std::chrono::seconds(1))
                && observed->latestDisplayText().starts_with("SECOND"),
                "display worker should wake immediately for changed presentation data");
        require(observed->waitForOutputCount(3, std::chrono::milliseconds(300)),
                "display worker should periodically refresh an unchanged value");
    }

    void testManagerTimesOutMissingHeartbeatReports() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        auto driver = std::make_unique<protocol::Driver>(
            std::make_unique<BlockingMockHidTransport>());
        protocol::Manager manager(std::move(driver), std::chrono::milliseconds(20));
        const auto connected = manager.waitTakeEvent(std::chrono::seconds(1));
        require(connected && std::holds_alternative<protocol::Connected>(*connected),
                "watchdog test should receive the initial device report");
        const auto disconnected = manager.waitTakeEvent(std::chrono::seconds(1));
        const auto *failure = disconnected
            ? std::get_if<protocol::Disconnected>(&*disconnected) : nullptr;
        require(failure && failure->error.code == ngc::pendant::HidErrorCode::TimedOut,
                "missing reports should disconnect the device session with a timeout error");
        const auto snapshot = manager.snapshot();
        require(!snapshot.connected && snapshot.error
                && snapshot.error->code == ngc::pendant::HidErrorCode::TimedOut,
                "watchdog timeout should remain visible in the manager snapshot");

        protocol::Profile profile;
        const auto initial = std::get<protocol::Connected>(*connected);
        (void)profile.consume(initial);
        const auto intents = profile.consume(*failure);
        const auto *cancel = findIntent<ngc::pendant::CancelPendantActivity>(intents);
        require(cancel && cancel->reason == ngc::pendant::CancelReason::Disconnected,
                "watchdog timeout should cancel all pendant-owned activity");
    }

    void testProfileMapsVerifiedWheelModesConservatively() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        protocol::Profile profile;
        protocol::InputState state;
        state.leftSelector = protocol::LeftSelector::Step;
        state.rightSelector = protocol::RightSelector::X_F1;
        auto intents = profile.consume(protocol::Connected { 1, 0, state });
        const auto *selection = findIntent<ngc::pendant::SelectionChanged>(intents);
        require(selection && selection->mode == ngc::pendant::Mode::Step
                && selection->axis == ngc::pendant::Axis::X,
                "P2S profile should expose its initial stable step-axis selection");

        state.wheelDeltaValid = true;
        state.wheelDelta = 3;
        intents = profile.consume(protocol::InputChanged {
            2, 3, protocol::InputChange::Wheel, state,
        });
        const auto *fineJog = findIntent<ngc::pendant::JogWheel>(intents);
        require(fineJog && fineJog->axis == ngc::pendant::Axis::X && fineJog->counts == 3
                && fineJog->increment == ngc::pendant::JogIncrement::Fine,
                "released wheel movement in Step mode should request the fine increment");

        state.wheelDelta = -2;
        state.wheelButton = protocol::WheelButton::Pressed;
        intents = profile.consume(protocol::InputChanged {
            3, 1, protocol::InputChange::Wheel | protocol::InputChange::WheelButton, state,
        });
        const auto *heldJog = findIntent<ngc::pendant::JogWheel>(intents);
        require(heldJog && heldJog->axis == ngc::pendant::Axis::X && heldJog->counts == -2
                && heldJog->increment == ngc::pendant::JogIncrement::Coarse,
                "held wheel movement in Step mode should request the coarse increment");

        state.wheelDelta = 1;
        state.wheelButton = protocol::WheelButton::Released;
        intents = profile.consume(protocol::InputChanged {
            4, 2, protocol::InputChange::Wheel | protocol::InputChange::WheelButton, state,
        });
        const auto *releasedJog = findIntent<ngc::pendant::JogWheel>(intents);
        require(!findIntent<ngc::pendant::CancelPendantActivity>(intents)
                && releasedJog
                && releasedJog->increment == ngc::pendant::JogIncrement::Fine,
                "button release in Step mode should select the fine increment without cancelling");

        state.rightSelector = protocol::RightSelector::Transient;
        state.wheelDelta = 1;
        state.wheelButton = protocol::WheelButton::Pressed;
        intents = profile.consume(protocol::InputChanged {
            5, 3, protocol::InputChange::RightSelector | protocol::InputChange::Wheel
                | protocol::InputChange::WheelButton, state,
        });
        require(!findIntent<ngc::pendant::JogWheel>(intents),
                "wheel motion must be inhibited during the right selector's transient zero");
        require(profile.snapshot().selectedAxis == ngc::pendant::Axis::X
                && profile.snapshot().rightSelectorTransient,
                "transient selector zero should retain but inhibit the last stable selection");

        state.rightSelector = protocol::RightSelector::Y_F2;
        intents = profile.consume(protocol::InputChanged {
            6, 4, protocol::InputChange::RightSelector | protocol::InputChange::Wheel, state,
        });
        require(!findIntent<ngc::pendant::JogWheel>(intents)
                && profile.snapshot().selectedAxis == ngc::pendant::Axis::Y,
                "a report that establishes a new selector position must not also cause motion");

        state.machineOff = true;
        state.leftSelector = protocol::LeftSelector::Function;
        intents = profile.consume(protocol::InputChanged {
            7, 4, protocol::InputChange::MachineOff | protocol::InputChange::LeftSelector, state,
        });
        const auto *off = findIntent<ngc::pendant::MachineOffChanged>(intents);
        const auto *cancel = findIntent<ngc::pendant::CancelPendantActivity>(intents);
        require(off && off->active && cancel
                && cancel->reason == ngc::pendant::CancelReason::MachineOff,
                "the physical OFF combination should cancel pendant-owned activity");
        require(profile.snapshot().mode == ngc::pendant::Mode::Off,
                "the profile should derive OFF from its separate level bit");
    }

    void testProfileMapsTouchOffFeedOverrideAndSafetyLevels() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        protocol::Profile profile;
        protocol::InputState state;
        state.leftSelector = protocol::LeftSelector::Zero;
        state.rightSelector = protocol::RightSelector::Z_F3;
        (void)profile.consume(protocol::Connected { 1, 0, state });

        state.wheelButton = protocol::WheelButton::Pressed;
        (void)profile.consume(protocol::InputChanged {
            2, 0, protocol::InputChange::WheelButton, state,
        });
        state.wheelDeltaValid = true;
        state.wheelDelta = 1;
        auto intents = profile.consume(protocol::InputChanged {
            3, 1, protocol::InputChange::Wheel, state,
        });
        const auto *adjust = findIntent<ngc::pendant::AdjustTouchOff>(intents);
        require(adjust && adjust->axis == ngc::pendant::Axis::Z && adjust->counts == 1,
                "wheel movement in Zero mode should adjust a pending touch-off coordinate");

        state.wheelDelta = 0;
        state.wheelButton = protocol::WheelButton::DoublePressed;
        intents = profile.consume(protocol::InputChanged {
            4, 1, protocol::InputChange::WheelButton, state,
        });
        const auto *commit = findIntent<ngc::pendant::CommitTouchOff>(intents);
        require(commit && commit->axis == ngc::pendant::Axis::Z,
                "firmware-recognized double press in Zero mode should commit touch-off");

        state.leftSelector = protocol::LeftSelector::Speed;
        state.wheelButton = protocol::WheelButton::Pressed;
        (void)profile.consume(protocol::InputChanged {
            5, 1, protocol::InputChange::LeftSelector | protocol::InputChange::WheelButton, state,
        });
        state.wheelButton = protocol::WheelButton::Released;
        (void)profile.consume(protocol::InputChanged {
            6, 1, protocol::InputChange::WheelButton, state,
        });
        state.wheelButton = protocol::WheelButton::Pressed;
        (void)profile.consume(protocol::InputChanged {
            7, 1, protocol::InputChange::WheelButton, state,
        });
        state.wheelDelta = -4;
        intents = profile.consume(protocol::InputChanged {
            8, -3, protocol::InputChange::Wheel, state,
        });
        const auto *feed = findIntent<ngc::pendant::AdjustFeedOverride>(intents);
        require(feed && feed->counts == -4,
                "wheel movement in Speed mode should request feed-override count adjustment");

        state.emergencyStop = true;
        state.wheelDelta = 2;
        intents = profile.consume(protocol::InputChanged {
            9, -1, protocol::InputChange::EmergencyStop | protocol::InputChange::Wheel, state,
        });
        const auto *emergency = findIntent<ngc::pendant::EmergencyStopSwitchChanged>(intents);
        const auto *cancel = findIntent<ngc::pendant::CancelPendantActivity>(intents);
        require(emergency && emergency->active && cancel
                && cancel->reason == ngc::pendant::CancelReason::EmergencyStop
                && !findIntent<ngc::pendant::AdjustFeedOverride>(intents),
                "pendant E-stop level should cancel and suppress same-report wheel actions");

        state.emergencyStop = false;
        state.wheelDelta = 1;
        intents = profile.consume(protocol::InputChanged {
            10, 0, protocol::InputChange::EmergencyStop | protocol::InputChange::Wheel, state,
        });
        emergency = findIntent<ngc::pendant::EmergencyStopSwitchChanged>(intents);
        require(emergency && !emergency->active
                && !findIntent<ngc::pendant::AdjustFeedOverride>(intents),
                "releasing the pendant E-stop must not authorize a same-report wheel action");

        intents = profile.consume(protocol::Disconnected {
            { ngc::pendant::HidErrorCode::Disconnected, 1, "unplugged" },
        });
        const auto *connection = findIntent<ngc::pendant::ConnectionChanged>(intents);
        cancel = findIntent<ngc::pendant::CancelPendantActivity>(intents);
        require(connection && !connection->connected && connection->detail == "unplugged"
                && cancel && cancel->reason == ngc::pendant::CancelReason::Disconnected,
                "disconnect should be visible and cancel all pendant-owned activity");
    }

    void testProfileStepModeUsesHeldStateImmediatelyAfterConnection() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        protocol::Profile profile;
        protocol::InputState state;
        state.leftSelector = protocol::LeftSelector::Step;
        state.rightSelector = protocol::RightSelector::X_F1;
        state.wheelButton = protocol::WheelButton::Pressed;
        (void)profile.consume(protocol::Connected { 1, 0, state });
        state.wheelDeltaValid = true;
        state.wheelDelta = 1;
        auto intents = profile.consume(protocol::InputChanged {
            2, 1, protocol::InputChange::Wheel, state,
        });
        const auto *coarse = findIntent<ngc::pendant::JogWheel>(intents);
        require(coarse && coarse->increment == ngc::pendant::JogIncrement::Coarse,
                "Step mode should use the held coarse increment immediately after connection");

        state.wheelDelta = 0;
        state.wheelButton = protocol::WheelButton::Released;
        (void)profile.consume(protocol::InputChanged {
            3, 1, protocol::InputChange::WheelButton, state,
        });
        state.wheelButton = protocol::WheelButton::Pressed;
        (void)profile.consume(protocol::InputChanged {
            4, 1, protocol::InputChange::WheelButton, state,
        });
        state.wheelDelta = 1;
        intents = profile.consume(protocol::InputChanged {
            5, 2, protocol::InputChange::Wheel, state,
        });
        coarse = findIntent<ngc::pendant::JogWheel>(intents);
        require(coarse && coarse->increment == ngc::pendant::JogIncrement::Coarse,
                "a later held wheel movement should continue using the coarse increment");
    }

    void testProfileEstimatesHeldVelocityFromDetentTiming() {
        namespace protocol = ngc::pendant::vista_cnc_p2s;
        const auto at = [](const std::int64_t milliseconds) {
            return std::chrono::steady_clock::time_point(std::chrono::milliseconds(milliseconds));
        };
        protocol::Profile profile;
        protocol::InputState state;
        state.leftSelector = protocol::LeftSelector::Velocity;
        state.rightSelector = protocol::RightSelector::X_F1;
        state.wheelButton = protocol::WheelButton::Released;
        (void)profile.consume(protocol::Connected { 1, 0, state });

        state.wheelButton = protocol::WheelButton::Pressed;
        (void)profile.consume(protocol::InputChanged {
            2, 0, protocol::InputChange::WheelButton, state,
        });
        state.wheelDeltaValid = true;
        state.wheelDelta = -1;
        state.wheelRateCode = 5;
        state.wheelMotionAccumulator = 0x0222;
        auto intents = profile.consume(protocol::InputChanged {
            3, -1, protocol::InputChange::Wheel | protocol::InputChange::WheelRate
                | protocol::InputChange::WheelMotionAccumulator, state, at(10),
        });
        auto *velocity = findIntent<ngc::pendant::JogVelocity>(intents);
        require(!velocity,
                "the first velocity-mode detent should only prime the timing estimator");

        state.wheelDelta = 0;
        state.wheelRateCode = 0;
        state.wheelMotionAccumulator = 0x01ec;
        intents = profile.consume(protocol::InputChanged {
            4, -1, protocol::InputChange::WheelRate
                | protocol::InputChange::WheelMotionAccumulator, state, at(60),
        });
        velocity = findIntent<ngc::pendant::JogVelocity>(intents);
        require(!velocity,
                "firmware decay alone should not turn one detent into velocity motion");

        state.wheelDelta = -1;
        state.wheelMotionAccumulator = 0x02f0;
        intents = profile.consume(protocol::InputChanged {
            5, -2, protocol::InputChange::Wheel
                | protocol::InputChange::WheelMotionAccumulator, state, at(110),
        });
        velocity = findIntent<ngc::pendant::JogVelocity>(intents);
        require(velocity && velocity->axis == ngc::pendant::Axis::X
                && velocity->countsPerSecond == -10,
                "two detents 100 ms apart should start a 10-count-per-second jog");

        state.wheelDelta = 0;
        state.wheelMotionAccumulator = 0x02ba;
        intents = profile.consume(protocol::InputChanged {
            6, -2, protocol::InputChange::WheelMotionAccumulator, state, at(160),
        });
        velocity = findIntent<ngc::pendant::JogVelocity>(intents);
        require(velocity && velocity->countsPerSecond == -10,
                "accumulator reports before the missed-detent deadline should renew the lease");

        state.wheelMotionAccumulator = 0x0214;
        intents = profile.consume(protocol::InputChanged {
            7, -2, protocol::InputChange::WheelMotionAccumulator, state, at(260),
        });
        velocity = findIntent<ngc::pendant::JogVelocity>(intents);
        require(velocity && velocity->countsPerSecond == 0,
                "missing the next expected detent should stop without waiting for accumulator zero");

        state.wheelButton = protocol::WheelButton::Released;
        intents = profile.consume(protocol::InputChanged {
            8, -2, protocol::InputChange::WheelButton, state, at(270),
        });
        const auto *cancel = findIntent<ngc::pendant::CancelPendantActivity>(intents);
        require(cancel && cancel->reason == ngc::pendant::CancelReason::ButtonReleased,
                "releasing the P2-S wheel button should independently cancel velocity jogging");
    }
}

int main() {
    try {
        testInputDecoder();
        testDriverFramesWindowsReportsAndDisplay();
        testPositionDisplayUsesBothEightCharacterRows();
        testManagerPublishesOrderedMeaningfulChanges();
        testManagerCancellationReleasesBlockingRead();
        testManagerPublishesDisplayChangesAsynchronouslyAndRefreshes();
        testManagerTimesOutMissingHeartbeatReports();
        testProfileMapsVerifiedWheelModesConservatively();
        testProfileMapsTouchOffFeedOverrideAndSafetyLevels();
        testProfileStepModeUsesHeldStateImmediatelyAfterConnection();
        testProfileEstimatesHeldVelocityFromDetentTiming();
    } catch(const std::exception &error) {
        std::cerr << "ngc_pendant_tests failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ngc_pendant_tests passed\n";
}
