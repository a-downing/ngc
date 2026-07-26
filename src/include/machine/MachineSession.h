#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <deque>
#include <expected>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "evaluator/InterpreterSession.h"
#include "machine/BackendRuntime.h"
#include "machine/GeometryStreamProducer.h"
#include "machine/HomingController.h"
#include "machine/JoggingController.h"
#include "machine/MotionBackend.h"
#include "machine/PreparedTrajectoryExecutionDriver.h"
#include "machine/PresentationTracker.h"
#include "machine/ProgramExecutionController.h"
#include "machine/ToolTable.h"

namespace ngc {
    using ParameterSnapshot = std::unordered_map<Var, double>;

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

    enum class ProgramOperationState {
        Running,
        Holding,
        Paused,
        StopComplete,
        Completed,
        Error,
    };

    struct ProgramOperationUpdate {
        ProgramOperationState state = ProgramOperationState::Running;
        std::size_t pumped = 0;
        bool workDeferred = false;
        std::optional<position_t> stoppedPosition;
        std::optional<std::string> error;
    };

    enum class ExecutionEpochOutcome {
        Abandoned,
        Completed,
        Stopped,
        Failed,
    };

    struct ExecutionEpochFinish {
        GeometryStreamDiagnostics diagnostics;
        std::optional<std::string> persistenceError;
    };

    struct MachineSessionCheckpoint {
        Machine::Checkpoint controller;
        StationaryBackendState backend;
        JointMask homedJoints = 0;
        MachinePresentationSnapshot presentation;
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

    using DispatchedSessionOperation = std::variant<
        StartProgram,
        StartHoming,
        StartJog,
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
            std::scoped_lock lock(m_mutex);
            return std::ranges::any_of(m_commands, std::move(predicate));
        }

        template<typename Predicate>
        void eraseIf(Predicate predicate) {
            std::scoped_lock lock(m_mutex);
            std::erase_if(m_commands, std::move(predicate));
        }

        template<typename Command>
        [[nodiscard]] bool contains() const noexcept {
            std::scoped_lock lock(m_mutex);
            for (const auto &queued : m_commands) {
                if (std::holds_alternative<Command>(queued)) {
                    return true;
                }
            }

            return false;
        }

    private:
        mutable std::mutex m_mutex;
        std::size_t m_capacity;
        std::deque<SessionCommand> m_commands;
    };

    class ExecutionCoordinator {
    public:
        explicit ExecutionCoordinator(std::size_t commandCapacity = SessionCommandQueue::DEFAULT_CAPACITY);

        [[nodiscard]] bool beginPowerOn() noexcept;
        void completePowerOn() noexcept;
        [[nodiscard]] bool beginPowerOff() noexcept;
        void completePowerOff() noexcept;
        [[nodiscard]] bool powerOn() noexcept;
        [[nodiscard]] bool powerOff() noexcept;
        [[nodiscard]] bool beginActivity(MachineActivity activity) noexcept;
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
        MachineSession(Machine::Unit unit, InterpretationMode mode, BackendRuntime &runtime,
                       const TrajectoryLimits &limits, GeometryStreamPolicy geometryPolicy = {});
        ~MachineSession();
        MachineSession(const MachineSession &) = delete;
        MachineSession &operator=(const MachineSession &) = delete;

        [[nodiscard]] bool powerOn() noexcept;
        [[nodiscard]] bool powerOff() noexcept;
        [[nodiscard]] bool queueProgram(StartProgram start);
        [[nodiscard]] bool queueHoming();
        [[nodiscard]] bool queueJog(ControlRequest request);
        [[nodiscard]] std::expected<std::optional<DispatchedSessionOperation>, std::string>
        dispatchNextOperation();
        [[nodiscard]] std::expected<GeometryEpoch, std::string> beginExecutionEpoch(
            StartProgram start, const position_t &startingPosition);
        [[nodiscard]] ExecutionEpochFinish finishExecutionEpoch(ExecutionEpochOutcome outcome);
        [[nodiscard]] bool setToolTable(const ToolTable &tools);
        [[nodiscard]] bool toolTableInitialized() const noexcept;
        [[nodiscard]] ToolTable toolTable() const;
        [[nodiscard]] std::pair<ToolTable, std::uint64_t> toolTableSnapshot() const;
        [[nodiscard]] ParameterSnapshot parameterSnapshot() const;
        [[nodiscard]] bool controllerDataMutable() const noexcept;
        [[nodiscard]] std::expected<MachineSessionCheckpoint, std::string>
        checkpoint(const StationaryBackendState &backend) const;
        [[nodiscard]] std::expected<void, std::string>
        restoreCheckpoint(const MachineSessionCheckpoint &checkpoint);
        [[nodiscard]] std::expected<void, std::string>
        setToolTableStorePath(const std::filesystem::path &path);
        [[nodiscard]] std::expected<void, std::string>
        saveToolTable(const std::filesystem::path &path) const;
        [[nodiscard]] std::expected<void, std::string>
        setPersistentParameterStorePath(const std::filesystem::path &path);
        [[nodiscard]] std::expected<void, std::string>
        loadPersistentParameters(const std::filesystem::path &path);
        [[nodiscard]] std::expected<void, std::string>
        savePersistentParameters(const std::filesystem::path &path) const;
        [[nodiscard]] std::expected<void, std::string> persistParametersAtBoundary() const;
        void configureHoming(std::vector<AxisConfiguration> axes,
                             std::vector<JointConfiguration> joints,
                             HomingConfiguration homing);
        void configureJogging(std::vector<AxisConfiguration> axes,
                              std::vector<JointConfiguration> joints);
        [[nodiscard]] bool homingAvailable() const noexcept;
        [[nodiscard]] std::expected<HomingResult, std::string> runHoming(
            const position_t &startingPosition);
        [[nodiscard]] std::optional<HomingObservation> homingObservation() const;
        [[nodiscard]] JointMask homedJoints() const noexcept;
        [[nodiscard]] std::expected<JoggingResult, std::string> runJogging(
            const position_t &startingPosition, const ControlRequest &firstRequest);
        [[nodiscard]] std::optional<JoggingObservation> joggingObservation() const;
        void requestGeometryStop();
        [[nodiscard]] bool executionEpochActive() const noexcept;
        [[nodiscard]] GeometryEpoch nextEpoch() noexcept;
        [[nodiscard]] bool applyProgramPresentationUpdate();
        void completeProgramPresentation();

        template<typename BackendWorkAllowed, typename WithPresentationLock, typename ObserveEvent,
                 typename ObserveCommand, typename ObserveStatus>
        ProgramOperationUpdate serviceProgramOperation(
            const ExecutionSnapshot &snapshot, const bool shutdownRequested,
            const std::size_t pumpLimit, BackendWorkAllowed &&backendWorkAllowed,
            WithPresentationLock &&withPresentationLock,
            ObserveEvent &&observeEvent, ObserveCommand &&observeCommand,
            ObserveStatus &&observeStatus) {
            m_programExecution.service(snapshot, shutdownRequested);
            auto update = programOperationUpdate();
            if (update.state == ProgramOperationState::Paused
                || update.state == ProgramOperationState::StopComplete
                || update.state == ProgramOperationState::Completed
                || update.state == ProgramOperationState::Error) {
                return update;
            }
            if (!backendWorkAllowed()) {
                update.workDeferred = true;

                return update;
            }

            m_driver.serviceBackend([&](const ExecutionEvent &event) {
                m_programExecution.observeBackendEvent(event);
                withPresentationLock([&] {
                    if (const auto *accepted = std::get_if<ChunkAccepted>(&event)) {
                        m_presentationTracker.observeChunkAccepted(*accepted);
                    } else if (const auto *marker =
                                   std::get_if<ExecutionMarkerReached>(&event)) {
                        m_presentationTracker.observeMarkerReached(*marker);
                    } else if (const auto *retired = std::get_if<ChunkRetired>(&event)) {
                        m_presentationTracker.observeChunkRetired(*retired);
                    }
                    observeEvent(event);
                });
            });
            withPresentationLock([&] {
                (void)applyProgramPresentationUpdate();
            });
            m_programExecution.observeDriverState();
            update = programOperationUpdate();
            if (update.state == ProgramOperationState::Paused
                || update.state == ProgramOperationState::StopComplete
                || update.state == ProgramOperationState::Completed
                || update.state == ProgramOperationState::Error) {
                return update;
            }
            if (!backendWorkAllowed()) {
                update.workDeferred = true;

                return update;
            }
            for (std::size_t fill = 0; fill < pumpLimit; ++fill) {
                if (!backendWorkAllowed()) {
                    update.workDeferred = true;
                    break;
                }
                const auto pumped = m_driver.pumpOne(
                    [&](const MachineCommand &command, const ExecutionItem &item,
                        const TrajectoryPlanningMetadata &,
                        const TrajectoryCommandPresentation &presentation,
                        const ExecutionMarkerId activationMarker) {
                        withPresentationLock([&] {
                            observeCommand(command, item, presentation, activationMarker);
                            m_presentationTracker.observeCommand(
                                command, item, presentation, activationMarker);
                        });
                    },
                    [&](const InterpreterBlockLifecycle &lifecycle) {
                        withPresentationLock([&] {
                            m_presentationTracker.observeLifecycle(lifecycle);
                        });
                    },
                    [&](const InterpreterStatusMessage &status) {
                        withPresentationLock([&] {
                            observeStatus(status);
                        });
                    });
                withPresentationLock([&] {
                    (void)applyProgramPresentationUpdate();
                });
                if (!pumped) {
                    break;
                }

                ++update.pumped;
                if (m_programExecution.stopRequested() || m_programExecution.paused()
                    || !m_coordinator.commands().empty()) {
                    break;
                }
            }

            m_programExecution.observeDriverState();
            auto finalUpdate = programOperationUpdate();
            finalUpdate.pumped = update.pumped;
            finalUpdate.workDeferred = update.workDeferred;

            return finalUpdate;
        }

        template<typename Self> auto &interpreter(this Self &&self) {
            return std::forward<Self>(self).m_interpreter;
        }
        template<typename Self> auto &driver(this Self &&self) {
            return std::forward<Self>(self).m_driver;
        }
        template<typename Self> auto &presentationTracker(this Self &&self) {
            return std::forward<Self>(self).m_presentationTracker;
        }
        template<typename Self> auto &programExecution(this Self &&self) {
            return std::forward<Self>(self).m_programExecution;
        }
        template<typename Self> auto &coordinator(this Self &&self) {
            return std::forward<Self>(self).m_coordinator;
        }
        template<typename Self> auto &geometryPolicy(this Self &&self) {
            return std::forward<Self>(self).m_geometryPolicy;
        }

    private:
        [[nodiscard]] ProgramOperationUpdate programOperationUpdate() const;
        [[nodiscard]] std::expected<void, std::string> persistToolTableAtBoundary();

        Machine::Unit m_unit;
        InterpreterSession m_interpreter;
        GeometryStreamPolicy m_geometryPolicy;
        PreparedGeometryForwardChannel m_geometryForward;
        GeometryFeedbackChannel m_geometryFeedback;
        std::atomic<bool> m_geometryCancelled { false };
        std::unique_ptr<GeometryStreamProducer> m_geometryProducer;
        std::thread m_geometryThread;
        BackendRuntime &m_runtime;
        MotionBackend &m_backend;
        PreparedTrajectoryExecutionDriver m_driver;
        PresentationTracker m_presentationTracker;
        ExecutionCoordinator m_coordinator;
        ProgramExecutionController m_programExecution;
        std::unique_ptr<HomingController> m_homingController;
        std::unique_ptr<JoggingController> m_joggingController;
        mutable std::mutex m_serviceObservationMutex;
        std::optional<HomingObservation> m_homingObservation;
        std::optional<JoggingObservation> m_joggingObservation;
        JointMask m_homedJoints = 0;
        TrajectoryLimits m_limits;
        GeometryEpoch m_nextEpoch = 1;
        std::optional<std::filesystem::path> m_parameterStorePath;
        std::optional<std::filesystem::path> m_toolTableStorePath;
        ToolTable m_observedToolTable;
        bool m_toolTableInitialized = false;
        std::uint64_t m_toolTableRevision = 0;
    };
}
