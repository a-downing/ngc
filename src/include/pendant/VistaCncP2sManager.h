#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

#include "pendant/VistaCncP2sDriver.h"

namespace ngc::pendant::vista_cnc_p2s {
    enum class InputChange : std::uint16_t {
        None = 0,
        Wheel = 1 << 0,
        LeftSelector = 1 << 1,
        RightSelector = 1 << 2,
        WheelButton = 1 << 3,
        MachineOff = 1 << 4,
        EmergencyStop = 1 << 5,
        WheelRate = 1 << 6,
        WheelMotionAccumulator = 1 << 7,
    };

    constexpr InputChange operator|(const InputChange left, const InputChange right) noexcept {
        return static_cast<InputChange>(static_cast<std::uint16_t>(left)
                                        | static_cast<std::uint16_t>(right));
    }

    constexpr InputChange &operator|=(InputChange &left, const InputChange right) noexcept {
        left = left | right;
        return left;
    }

    constexpr bool contains(const InputChange changes, const InputChange change) noexcept {
        return (static_cast<std::uint16_t>(changes) & static_cast<std::uint16_t>(change)) != 0;
    }

    struct Connected {
        std::uint64_t reportSequence = 0;
        std::int64_t cumulativeWheelCounts = 0;
        InputState state;
        std::chrono::steady_clock::time_point arrivalTime{};
    };

    struct InputChanged {
        std::uint64_t reportSequence = 0;
        std::int64_t cumulativeWheelCounts = 0;
        InputChange changes = InputChange::None;
        InputState state;
        std::chrono::steady_clock::time_point arrivalTime{};
    };

    struct Disconnected { HidError error; };
    struct DisplayFailed { HidError error; };
    struct Stopped { };
    using ManagerEvent = std::variant<Connected, InputChanged, Disconnected, DisplayFailed, Stopped>;

    struct ManagerSnapshot {
        bool connected = false;
        bool stopped = false;
        std::uint64_t reportSequence = 0;
        std::int64_t cumulativeWheelCounts = 0;
        std::optional<InputState> state;
        std::optional<HidError> error;
    };

    // NRT device-session owner. It does not interpret controls as machine
    // actions and does not call Machine or MotionBackend.
    class Manager {
    public:
        explicit Manager(std::unique_ptr<Driver> driver,
                         std::chrono::milliseconds reportTimeout = REPORT_TIMEOUT);
        ~Manager();

        Manager(const Manager &) = delete;
        Manager &operator=(const Manager &) = delete;

        static std::expected<std::unique_ptr<Manager>, HidError> open();

        void stop();
        // Latest-value asynchronous display publication. A changed value wakes
        // the output worker immediately; an unchanged value is still refreshed
        // periodically for the P2-S firmware.
        std::expected<void, HidError> setDisplay(std::string_view text);
        bool tryTakeEvent(ManagerEvent &event);
        std::optional<ManagerEvent> waitTakeEvent(std::chrono::milliseconds timeout);
        ManagerSnapshot snapshot() const;

    private:
        void work();
        void displayWork();
        void publish(ManagerEvent event);

        std::unique_ptr<Driver> m_driver;
        std::chrono::milliseconds m_reportTimeout;
        std::thread m_thread;
        std::thread m_displayThread;
        std::atomic_bool m_stopRequested = false;
        std::atomic_bool m_inputEnded = false;
        mutable std::mutex m_mutex;
        std::condition_variable m_eventAvailable;
        std::deque<ManagerEvent> m_events;
        ManagerSnapshot m_snapshot;
        std::mutex m_displayMutex;
        std::condition_variable m_displayChanged;
        std::string m_displayText;
        std::uint64_t m_displayRevision = 0;
        std::optional<HidError> m_displayError;
    };
}
