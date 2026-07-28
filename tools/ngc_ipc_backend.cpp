#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

#include "ExecutionItemOperations.h"
#include "IpcPlatform.h"
#include "machine/IpcExecutorBridge.h"
#include "machine/IpcProtocol.h"
#include "machine/MachineConfiguration.h"
#include "machine/ProductionExecutorRuntime.h"

namespace {
    constexpr double TEMPORARY_TRIGGER_DISTANCE = 0.5;

    struct Options {
        std::string mapping;
        ngc::IpcIdentity expected{};
        std::optional<std::filesystem::path> machineConfiguration;
        bool consume = true;
        bool nonRealtime = false;
        std::optional<std::uint64_t> exitAfterControls;
        std::optional<std::chrono::milliseconds> exitAfterHandshake;
    };

    std::uint64_t parseUnsigned(const std::string_view value) {
        std::size_t consumed = 0;
        const auto result = std::stoull(std::string(value), &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("invalid unsigned integer");
        }

        return result;
    }

    Options parseOptions(const int argc, char **argv) {
        Options options;
        for (auto index = 1; index < argc; ++index) {
            const auto option = std::string_view(argv[index]);
            const auto value = [&]() -> std::string_view {
                if (++index == argc) {
                    throw std::runtime_error("missing option value");
                }

                return argv[index];
            };

            if (option == "--mapping") {
                options.mapping = value();
            } else if (option == "--configuration") {
                options.expected.configurationFingerprint = parseUnsigned(value());
            } else if (option == "--topology") {
                options.expected.topologyFingerprint = parseUnsigned(value());
            } else if (option == "--session") {
                options.expected.sessionGeneration = parseUnsigned(value());
            } else if (option == "--epoch") {
                options.expected.epochGeneration = parseUnsigned(value());
            } else if (option == "--authority") {
                options.expected.authorityGeneration = parseUnsigned(value());
            } else if (option == "--machine-configuration") {
                options.machineConfiguration = value();
            } else if (option == "--no-consume") {
                options.consume = false;
            } else if (option == "--non-realtime") {
                options.nonRealtime = true;
            } else if (option == "--exit-after-controls") {
                options.exitAfterControls = parseUnsigned(value());
            } else if (option == "--exit-after-handshake-ms") {
                options.exitAfterHandshake = std::chrono::milliseconds(
                    parseUnsigned(value()));
            } else {
                throw std::runtime_error("unknown option: " + std::string(option));
            }
        }
        if (options.mapping.empty()) {
            throw std::runtime_error("--mapping is required");
        }

        return options;
    }

    class TemporaryTriggeredInputs final
        : public ngc::ProductionExecutorIo,
          public ngc::IpcExecutorPolicy {
    public:
        void sampleDigitalInputs(ngc::ProductionExecutorDigitalInputs &inputs) noexcept override {
            inputs.reset();
            for (auto &input : m_inputs) {
                if (input.enabled.load(std::memory_order_acquire)) {
                    inputs.set(input.input.load(std::memory_order_relaxed),
                               input.level.load(std::memory_order_relaxed));
                }
            }
        }

        void applyOutputs(const ngc::ProductionExecutorOutputState &) noexcept override { }

        void prepareExecutionItem(
            ngc::ExecutionItem &item,
            const ngc::ExecutionSnapshot &snapshot) noexcept override {
            m_preparedInputsArmed = false;
            if (const auto *axisMove =
                    std::get_if<ngc::TriggeredMove>(&item)) {
                arm(*axisMove);
                m_preparedInputsArmed = true;
            } else if (auto *jointMove =
                           std::get_if<ngc::TriggeredJointMove>(&item);
                       jointMove != nullptr
                       && jointMove->triggers.size != 0) {
                arm(*jointMove, snapshot.commandedJoints);
                m_preparedInputsArmed = true;
            }
        }

        void executionItemPublished() noexcept override {
            m_preparedInputsArmed = false;
        }

        void executionItemRejected() noexcept override {
            if (m_preparedInputsArmed) {
                clear();
            }
            m_preparedInputsArmed = false;
        }

        void observeEvent(
            const ngc::ExecutionEvent &event) noexcept override {
            if (std::holds_alternative<ngc::TriggeredMoveCompleted>(
                    event)
                || std::holds_alternative<
                    ngc::TriggeredJointMoveCompleted>(event)) {
                clear();
            }
        }

        void observeSnapshot(
            const ngc::ExecutionSnapshot &snapshot) noexcept override {
            observe(snapshot);
        }

        void arm(const ngc::TriggeredMove &move) noexcept {
            clear();
            m_epoch = move.epoch;
            m_chunk = move.id;
            m_axisTarget = move.target;
            m_kind = Kind::Axis;
            armInput(m_inputs[0], move.input, move.condition);
            m_inputCount = 1;
        }

        void arm(ngc::TriggeredJointMove &move,
                 const ngc::JointMotionState &starting) noexcept {
            clear();
            m_epoch = move.epoch;
            m_chunk = move.id;
            m_kind = Kind::Joint;
            for (const auto &trigger : move.triggers) {
                const auto start = starting.position[trigger.joint];
                const auto delta = move.targetMode == ngc::JointTargetMode::Absolute
                    ? move.target[trigger.joint] - start
                    : move.target[trigger.joint];
                if (std::abs(delta) <= TEMPORARY_TRIGGER_DISTANCE) {
                    const auto extended =
                        std::copysign(2.0 * TEMPORARY_TRIGGER_DISTANCE, delta);
                    move.target[trigger.joint] =
                        move.targetMode == ngc::JointTargetMode::Absolute
                        ? start + extended : extended;
                }
                auto &input = m_inputs[m_inputCount];
                input.joint = trigger.joint;
                input.jointStart = start;
                armInput(input, trigger.input, trigger.condition);
                ++m_inputCount;
            }
        }

        void observe(const ngc::ExecutionSnapshot &snapshot) noexcept {
            if (m_kind == Kind::None || snapshot.epoch != m_epoch
                || snapshot.activeChunk != m_chunk) {
                return;
            }

            if (m_kind == Kind::Axis) {
                if ((m_axisTarget - snapshot.commanded.position).length()
                    <= TEMPORARY_TRIGGER_DISTANCE) {
                    trigger(m_inputs[0]);
                }
                return;
            }

            for (std::size_t index = 0; index < m_inputCount; ++index) {
                auto &input = m_inputs[index];
                if (std::abs(snapshot.commandedJoints.position[input.joint]
                             - input.jointStart)
                    >= TEMPORARY_TRIGGER_DISTANCE) {
                    trigger(input);
                }
            }
        }

        void clear() noexcept {
            for (auto &input : m_inputs) {
                input.enabled.store(false, std::memory_order_release);
            }
            m_kind = Kind::None;
            m_inputCount = 0;
        }

    private:
        enum class Kind { None, Axis, Joint };

        struct Input {
            std::atomic<bool> enabled = false;
            std::atomic<ngc::DigitalInputId> input = 0;
            std::atomic<bool> level = false;
            bool triggerLevel = false;
            ngc::JointId joint = 0;
            double jointStart = 0.0;
        };

        static void armInput(Input &input, const ngc::DigitalInputId id,
                             const ngc::InputCondition condition) noexcept {
            input.input.store(id, std::memory_order_relaxed);
            input.triggerLevel = condition == ngc::InputCondition::Active
                || condition == ngc::InputCondition::RisingEdge;
            input.level.store(!input.triggerLevel, std::memory_order_relaxed);
            input.enabled.store(true, std::memory_order_release);
        }

        static void trigger(Input &input) noexcept {
            input.level.store(input.triggerLevel, std::memory_order_release);
        }

        std::array<Input, ngc::MAX_JOINTS> m_inputs;
        Kind m_kind = Kind::None;
        std::size_t m_inputCount = 0;
        ngc::EpochId m_epoch = 0;
        ngc::ChunkId m_chunk = 0;
        ngc::position_t m_axisTarget{};
        bool m_preparedInputsArmed = false;
    };

    std::unique_ptr<ngc::ProductionExecutorRuntime> makeRuntime(
        const Options &options, std::unique_ptr<ngc::ProductionExecutorIo> io) {
        if (!options.machineConfiguration.has_value()) {
            return std::make_unique<ngc::ProductionExecutorRuntime>(
                ngc::ProductionExecutorRuntimeConfiguration{}, std::move(io));
        }

        const auto configuration =
            ngc::loadMachineConfiguration(*options.machineConfiguration);
        if (!configuration.has_value()) {
            throw std::runtime_error(
                "failed to load machine configuration: "
                + configuration.error());
        }

        auto runtimeConfiguration =
            ngc::productionExecutorRuntimeConfiguration(*configuration);
        if (options.nonRealtime) {
            runtimeConfiguration.realtime = {};
        }

        return std::make_unique<ngc::ProductionExecutorRuntime>(
            std::move(runtimeConfiguration), std::move(io));
    }

    int run(const Options &options) {
        auto memory = ngc::ipc_detail::SharedMemory::open(
            options.mapping, sizeof(ngc::IpcSharedRegion));
        auto &region = *static_cast<ngc::IpcSharedRegion *>(memory.data());
        const auto rejection = ngc::validateIpcSharedRegion(
            region, options.expected);
        if (rejection != ngc::IpcRejection::None) {
            ngc::setIpcRejection(region, rejection);
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::Rejected);

            return 2;
        }

        auto triggeredInputs = std::make_unique<TemporaryTriggeredInputs>();
        auto *triggeredInputsPointer = triggeredInputs.get();
        auto runtime = makeRuntime(options, std::move(triggeredInputs));
        runtime->start();
        region.peerProcessId = ngc::ipc_detail::currentProcessId();
        ngc::setIpcConnectionState(region, ngc::IpcConnectionState::PeerReady);
        while (ngc::ipcConnectionState(region)
               == ngc::IpcConnectionState::PeerReady) {
            std::this_thread::yield();
        }
        if (ngc::ipcConnectionState(region)
            != ngc::IpcConnectionState::Running) {
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::PeerStopped);

            return 0;
        }

        ngc::IpcExecutorBridge bridge(
            region, *runtime, runtime->endpoint(),
            triggeredInputsPointer);
        const auto started = std::chrono::steady_clock::now();

        for (;;) {
            const auto state = ngc::ipcConnectionState(region);
            if (state == ngc::IpcConnectionState::StopRequested) {
                runtime->stop();
                ngc::setIpcConnectionState(
                    region, ngc::IpcConnectionState::PeerStopped);

                return 0;
            }
            if (options.exitAfterHandshake.has_value()
                && std::chrono::steady_clock::now() - started
                    >= *options.exitAfterHandshake) {
                return 3;
            }

            std::atomic_ref(region.peerHeartbeat).fetch_add(
                1, std::memory_order_relaxed);
            const auto progressed = bridge.service(options.consume);
            if (options.exitAfterControls.has_value()
                && bridge.completedControls() >= *options.exitAfterControls) {
                return 4;
            }

            if (!progressed) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(100));
            }
        }
    }
}

int main(const int argc, char **argv) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception &error) {
        std::cerr << "ngc_ipc_backend failed: " << error.what() << '\n';

        return 1;
    }
}
