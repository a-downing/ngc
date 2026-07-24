#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "machine/MotionBackend.h"

namespace ngc {
    enum class MachinePowerState { Off, Starting, On, Stopping, Faulted };
    enum class MachineActivity {
        Idle,
        Program,
        Mdi,
        Homing,
        Jogging,
        Holding,
        Stopping,
        Faulted,
    };

    struct StartProgram {
        std::vector<std::tuple<std::string, std::string>> programs;
        bool preserveState = false;
    };

    struct ExecuteMdi {
        std::string source;
        std::string program;
    };

    struct StartHoming {};
    struct StartJog {
        ControlRequest request;
    };
    struct RenewJog {
        RequestId request = 0;
        JogId jog = 0;
    };
    struct SetJogVelocity {
        SetContinuousJogVelocityRequest request;
    };
    struct StopJog {
        RequestId request = 0;
        JogId jog = 0;
    };
    struct FeedHold {};
    struct Resume {};
    struct Stop {};

    using SessionCommand = std::variant<
        StartProgram,
        ExecuteMdi,
        StartHoming,
        StartJog,
        RenewJog,
        SetJogVelocity,
        StopJog,
        FeedHold,
        Resume,
        Stop>;

    class SessionCommandQueue {
    public:
        static constexpr std::size_t DEFAULT_CAPACITY = 32;

        explicit SessionCommandQueue(std::size_t capacity = DEFAULT_CAPACITY);

        [[nodiscard]] bool tryPush(SessionCommand command);
        [[nodiscard]] std::optional<SessionCommand> tryPop();
        void clear() noexcept;
        [[nodiscard]] bool empty() const noexcept;
        [[nodiscard]] std::size_t size() const noexcept;
        [[nodiscard]] std::size_t capacity() const noexcept;

        template<typename Command>
        [[nodiscard]] bool contains() const noexcept {
            for (const auto &queued : m_commands) {
                if (std::holds_alternative<Command>(queued)) {
                    return true;
                }
            }

            return false;
        }

    private:
        std::size_t m_capacity;
        std::deque<SessionCommand> m_commands;
    };
}
