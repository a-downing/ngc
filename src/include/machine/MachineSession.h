#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <deque>
#include <expected>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/HomingController.h"
#include "machine/MotionBackend.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"
#include "machine/PresentationTracker.h"
#include "machine/ToolTable.h"

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
        MachineActivity activity = MachineActivity::Program;
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

        template<typename Predicate>
        [[nodiscard]] bool anyOf(Predicate predicate) const {
            return std::ranges::any_of(m_commands, std::move(predicate));
        }

        template<typename Predicate>
        void eraseIf(Predicate predicate) {
            std::erase_if(m_commands, std::move(predicate));
        }

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

    class ExecutionCoordinator {
    public:
        explicit ExecutionCoordinator(std::size_t commandCapacity = SessionCommandQueue::DEFAULT_CAPACITY);

        [[nodiscard]] bool powerOn() noexcept;
        [[nodiscard]] bool powerOff() noexcept;
        [[nodiscard]] bool beginActivity(MachineActivity activity) noexcept;
        void setActivity(MachineActivity activity) noexcept;
        void finishActivity() noexcept;
        void fault() noexcept;

        [[nodiscard]] MachinePowerState powerState() const noexcept;
        [[nodiscard]] MachineActivity activity() const noexcept;
        template<typename Self> auto &commands(this Self &&self) {
            return std::forward<Self>(self).m_commands;
        }

    private:
        std::atomic<MachinePowerState> m_powerState { MachinePowerState::Off };
        std::atomic<MachineActivity> m_activity { MachineActivity::Idle };
        SessionCommandQueue m_commands;
    };

    class MachineSession {
    public:
        MachineSession(Machine::Unit unit, InterpretationMode mode, MotionBackend &backend,
                       const TrajectoryLimits &limits, GeometryStreamPolicy geometryPolicy = {});
        ~MachineSession();
        MachineSession(const MachineSession &) = delete;
        MachineSession &operator=(const MachineSession &) = delete;

        [[nodiscard]] bool powerOn() noexcept;
        [[nodiscard]] bool powerOff() noexcept;
        [[nodiscard]] std::expected<GeometryEpoch, std::string> beginExecutionEpoch(
            StartProgram start, ToolTable tools, const position_t &startingPosition);
        [[nodiscard]] GeometryStreamDiagnostics finishExecutionEpoch();
        void configureHoming(std::vector<AxisConfiguration> axes,
                             std::vector<JointConfiguration> joints,
                             HomingConfiguration homing);
        [[nodiscard]] bool homingAvailable() const noexcept;
        [[nodiscard]] std::expected<HomingResult, std::string> runHoming(
            const position_t &startingPosition, const HomingRuntimeCallbacks &callbacks);
        [[nodiscard]] JointMask homedJoints() const noexcept;
        void requestGeometryStop();
        [[nodiscard]] bool executionEpochActive() const noexcept;
        [[nodiscard]] GeometryEpoch nextEpoch() noexcept;

        template<typename Self> auto &interpreter(this Self &&self) {
            return std::forward<Self>(self).m_interpreter;
        }
        template<typename Self> auto &driver(this Self &&self) {
            return std::forward<Self>(self).m_driver;
        }
        template<typename Self> auto &presentationTracker(this Self &&self) {
            return std::forward<Self>(self).m_presentationTracker;
        }
        template<typename Self> auto &coordinator(this Self &&self) {
            return std::forward<Self>(self).m_coordinator;
        }
        template<typename Self> auto &geometryPolicy(this Self &&self) {
            return std::forward<Self>(self).m_geometryPolicy;
        }
        [[nodiscard]] const GeometryStreamProducer *geometryProducer() const noexcept;

    private:
        InterpreterSession m_interpreter;
        GeometryStreamPolicy m_geometryPolicy;
        PreparedGeometryForwardChannel m_geometryForward;
        GeometryFeedbackChannel m_geometryFeedback;
        std::atomic<bool> m_geometryCancelled { false };
        std::unique_ptr<GeometryStreamProducer> m_geometryProducer;
        std::thread m_geometryThread;
        MotionBackend &m_backend;
        PreparedTrajectoryExecutionDriver m_driver;
        PresentationTracker m_presentationTracker;
        std::unique_ptr<HomingController> m_homingController;
        JointMask m_homedJoints = 0;
        ExecutionCoordinator m_coordinator;
        TrajectoryLimits m_limits;
        GeometryEpoch m_nextEpoch = 1;
    };
}
