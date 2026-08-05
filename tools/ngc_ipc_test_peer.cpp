#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <variant>

#include <arpa/inet.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ExecutionItemOperations.h"
#include "IpcPlatform.h"
#include "config/BackendRuntimeConfiguration.h"
#include "config/ConfigurationFingerprint.h"
#include "config/TomlConfiguration.h"
#include "machine/IpcExecutorBridge.h"
#include "machine/IpcProtocol.h"
#include "machine/MachineConfiguration.h"
#include "machine/HostedExecutorRuntime.h"

namespace {
    constexpr double TEMPORARY_TRIGGER_DISTANCE = 0.5;
    constexpr std::size_t SIMULATED_UDP_PAYLOAD_SIZE = 256;

    struct SimulatedUdpConfiguration {
        std::chrono::microseconds responseDelay{0};
        std::optional<std::uint32_t> responderCpu;
    };

    struct Options {
        std::string mapping;
        ngc::IpcIdentity expected{};
        std::optional<std::filesystem::path> machineConfiguration;
        std::optional<std::filesystem::path> backendConfiguration;
        bool consume = true;
        bool nonRealtime = false;
        bool validateConfigurationOnly = false;
        std::optional<std::uint64_t> exitAfterControls;
        std::optional<std::chrono::milliseconds> exitAfterHandshake;
        std::optional<SimulatedUdpConfiguration> simulatedUdp;
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
            } else if (option == "--session") {
                options.expected.sessionGeneration = parseUnsigned(value());
            } else if (option == "--epoch") {
                options.expected.epochGeneration = parseUnsigned(value());
            } else if (option == "--authority") {
                options.expected.authorityGeneration = parseUnsigned(value());
            } else if (option == "--machine-configuration") {
                options.machineConfiguration = value();
            } else if (option == "--backend-configuration") {
                options.backendConfiguration = value();
            } else if (option == "--no-consume") {
                options.consume = false;
            } else if (option == "--non-realtime") {
                options.nonRealtime = true;
            } else if (option == "--validate-config-only") {
                options.validateConfigurationOnly = true;
            } else if (option == "--exit-after-controls") {
                options.exitAfterControls = parseUnsigned(value());
            } else if (option == "--exit-after-handshake-ms") {
                options.exitAfterHandshake = std::chrono::milliseconds(
                    parseUnsigned(value()));
            } else if (option == "--simulated-udp-response-us") {
                if (!options.simulatedUdp.has_value()) {
                    options.simulatedUdp.emplace();
                }
                options.simulatedUdp->responseDelay =
                    std::chrono::microseconds(parseUnsigned(value()));
            } else if (option == "--simulated-udp-responder-cpu") {
                if (!options.simulatedUdp.has_value()) {
                    options.simulatedUdp.emplace();
                }
                options.simulatedUdp->responderCpu =
                    static_cast<std::uint32_t>(parseUnsigned(value()));
            } else {
                throw std::runtime_error("unknown option: " + std::string(option));
            }
        }
        if (!options.validateConfigurationOnly
            && options.mapping.empty()) {
            throw std::runtime_error("--mapping is required");
        }

        return options;
    }

    class SimulatedUdpExchange {
    public:
        explicit SimulatedUdpExchange(SimulatedUdpConfiguration configuration)
            : m_configuration(configuration) {
            openSockets();
            try {
                m_responder = std::thread(
                    &SimulatedUdpExchange::runResponder, this);
            } catch (...) {
                closeSockets();
                throw;
            }

            std::unique_lock lock(m_startupMutex);
            m_startupCv.wait(lock, [&] {
                return m_startupComplete;
            });
            if (!m_startupError.empty()) {
                lock.unlock();
                stop();
                throw std::runtime_error(m_startupError);
            }
        }

        ~SimulatedUdpExchange() {
            stop();
        }

        SimulatedUdpExchange(const SimulatedUdpExchange &) = delete;
        SimulatedUdpExchange &operator=(const SimulatedUdpExchange &) = delete;

        bool exchange() noexcept {
            ++m_sequence;
            std::memcpy(
                m_request.data(), &m_sequence,
                sizeof(m_sequence));
            const auto sent = ::send(
                m_clientSocket, m_request.data(),
                m_request.size(), 0);
            if (sent != static_cast<ssize_t>(m_request.size())) {
                return false;
            }

            const auto received = ::recv(
                m_clientSocket, m_response.data(),
                m_response.size(), 0);
            if (received != static_cast<ssize_t>(m_response.size())
                || std::memcmp(
                    m_request.data(), m_response.data(),
                    m_request.size()) != 0) {
                return false;
            }

            return true;
        }

    private:
        void openSockets() {
            m_serverSocket = ::socket(
                AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
            if (m_serverSocket < 0) {
                throw std::system_error(
                    errno, std::generic_category(),
                    "failed to create simulated UDP responder socket");
            }

            auto bindAddress = sockaddr_in{};
            bindAddress.sin_family = AF_INET;
            bindAddress.sin_port = 0;
            bindAddress.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            if (::bind(
                    m_serverSocket,
                    reinterpret_cast<const sockaddr *>(&bindAddress),
                    sizeof(bindAddress)) != 0) {
                const auto error = errno;
                closeSockets();
                throw std::system_error(
                    error, std::generic_category(),
                    "failed to bind simulated UDP responder socket");
            }

            auto serverAddress = sockaddr_in{};
            auto addressSize = static_cast<socklen_t>(
                sizeof(serverAddress));
            if (::getsockname(
                    m_serverSocket,
                    reinterpret_cast<sockaddr *>(&serverAddress),
                    &addressSize) != 0) {
                const auto error = errno;
                closeSockets();
                throw std::system_error(
                    error, std::generic_category(),
                    "failed to inspect simulated UDP responder socket");
            }

            m_clientSocket = ::socket(
                AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
            if (m_clientSocket < 0) {
                const auto error = errno;
                closeSockets();
                throw std::system_error(
                    error, std::generic_category(),
                    "failed to create simulated UDP client socket");
            }
            if (::connect(
                    m_clientSocket,
                    reinterpret_cast<const sockaddr *>(&serverAddress),
                    sizeof(serverAddress)) != 0) {
                const auto error = errno;
                closeSockets();
                throw std::system_error(
                    error, std::generic_category(),
                    "failed to connect simulated UDP client socket");
            }

            const timeval timeout{
                .tv_sec = 0,
                .tv_usec = 100'000,
            };
            if (::setsockopt(
                    m_clientSocket, SOL_SOCKET, SO_RCVTIMEO,
                    &timeout, sizeof(timeout)) != 0) {
                const auto error = errno;
                closeSockets();
                throw std::system_error(
                    error, std::generic_category(),
                    "failed to configure simulated UDP client timeout");
            }
        }

        void closeSockets() noexcept {
            if (m_clientSocket >= 0) {
                ::close(m_clientSocket);
                m_clientSocket = -1;
            }
            if (m_serverSocket >= 0) {
                ::close(m_serverSocket);
                m_serverSocket = -1;
            }
        }

        void stop() noexcept {
            if (!m_responder.joinable()) {
                closeSockets();

                return;
            }

            m_stopping.store(true, std::memory_order_release);
            static_cast<void>(::send(
                m_clientSocket, m_request.data(),
                m_request.size(), 0));
            m_responder.join();
            closeSockets();
        }

        void configureResponderCpu() {
            if (!m_configuration.responderCpu.has_value()) {
                return;
            }
            if (*m_configuration.responderCpu >= CPU_SETSIZE) {
                throw std::runtime_error(
                    "simulated UDP responder CPU exceeds CPU_SETSIZE");
            }

            cpu_set_t affinity;
            CPU_ZERO(&affinity);
            CPU_SET(*m_configuration.responderCpu, &affinity);
            if (const auto error = pthread_setaffinity_np(
                    pthread_self(), sizeof(affinity), &affinity);
                error != 0) {
                throw std::system_error(
                    error, std::generic_category(),
                    "failed to pin simulated UDP responder");
            }
        }

        void completeStartup(const std::string &error = {}) {
            {
                std::scoped_lock lock(m_startupMutex);
                m_startupError = error;
                m_startupComplete = true;
            }
            m_startupCv.notify_all();
        }

        void runResponder() noexcept {
            try {
                configureResponderCpu();
            } catch (const std::exception &error) {
                completeStartup(error.what());

                return;
            }
            completeStartup();

            auto request = std::array<std::byte,
                SIMULATED_UDP_PAYLOAD_SIZE>{};
            auto peer = sockaddr_in{};
            for (;;) {
                auto peerSize = static_cast<socklen_t>(sizeof(peer));
                const auto received = ::recvfrom(
                    m_serverSocket, request.data(), request.size(), 0,
                    reinterpret_cast<sockaddr *>(&peer), &peerSize);
                if (m_stopping.load(std::memory_order_acquire)) {
                    return;
                }
                if (received != static_cast<ssize_t>(request.size())) {
                    continue;
                }

                const auto responseTime = std::chrono::steady_clock::now()
                    + m_configuration.responseDelay;
                while (std::chrono::steady_clock::now() < responseTime) {
                    std::atomic_signal_fence(
                        std::memory_order_seq_cst);
                }
                const auto sent = ::sendto(
                    m_serverSocket, request.data(), request.size(), 0,
                    reinterpret_cast<const sockaddr *>(&peer), peerSize);
                static_cast<void>(sent);
            }
        }

        SimulatedUdpConfiguration m_configuration;
        int m_serverSocket = -1;
        int m_clientSocket = -1;
        std::thread m_responder;
        std::mutex m_startupMutex;
        std::condition_variable m_startupCv;
        std::string m_startupError;
        bool m_startupComplete = false;
        std::atomic<bool> m_stopping{false};
        std::uint64_t m_sequence = 0;
        std::array<std::byte, SIMULATED_UDP_PAYLOAD_SIZE> m_request{};
        std::array<std::byte, SIMULATED_UDP_PAYLOAD_SIZE> m_response{};
    };

    class TemporaryTriggeredInputs final
        : public ngc::ProductionExecutorIo,
          public ngc::IpcExecutorPolicy {
    public:
        explicit TemporaryTriggeredInputs(
            const std::optional<SimulatedUdpConfiguration> simulatedUdp =
                std::nullopt) {
            if (simulatedUdp.has_value()) {
                m_simulatedUdp =
                    std::make_unique<SimulatedUdpExchange>(*simulatedUdp);
            }
        }

        void sampleDigitalInputs(
            const ngc::ProductionExecutorMotionContext &,
            ngc::ProductionExecutorDigitalInputs &inputs) noexcept override {
            if (m_simulatedUdp && !m_simulatedUdp->exchange()) {
                m_faultCode.store(1, std::memory_order_release);
            }
            inputs.reset();
            for (auto &input : m_inputs) {
                if (input.enabled.load(std::memory_order_acquire)) {
                    inputs.set(input.input.load(std::memory_order_relaxed),
                               input.level.load(std::memory_order_relaxed));
                }
            }
        }

        void applyOutputs(const ngc::ProductionExecutorOutputState &) noexcept override { }

        void establishSafeOutputs() noexcept override { }

        std::uint32_t faultCode() const noexcept override {
            return m_faultCode.load(std::memory_order_acquire);
        }

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
            auto targetChanged = false;
            for (const auto &jointTrigger : move.triggers) {
                const auto start = starting.position[jointTrigger.joint];
                const auto delta = move.targetMode == ngc::JointTargetMode::Absolute
                    ? move.target[jointTrigger.joint] - start
                    : move.target[jointTrigger.joint];
                if (!move.checkTriggersAtStart
                    && std::abs(delta) <= TEMPORARY_TRIGGER_DISTANCE) {
                    const auto extended =
                        std::copysign(2.0 * TEMPORARY_TRIGGER_DISTANCE, delta);
                    move.target[jointTrigger.joint] =
                        move.targetMode == ngc::JointTargetMode::Absolute
                        ? start + extended : extended;
                    targetChanged = true;
                }
                auto &input = m_inputs[m_inputCount];
                input.joint = jointTrigger.joint;
                input.jointStart = start;
                armInput(
                    input, jointTrigger.input, jointTrigger.condition);
                if (move.checkTriggersAtStart) {
                    trigger(input);
                }
                ++m_inputCount;
            }
            if (targetChanged && move.positionEnvelope.joints != 0) {
                ngc::assignJointPositionEnvelope(
                    move, starting.position,
                    move.positionEnvelope.enforceAxisLimits);
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
        std::unique_ptr<SimulatedUdpExchange> m_simulatedUdp;
        std::atomic<std::uint32_t> m_faultCode{0};
    };

    struct LoadedExecutorConfiguration {
        ngc::HostedExecutorRuntimeConfiguration runtime;
        std::uint64_t fingerprint = 0;
    };

    LoadedExecutorConfiguration loadExecutorConfiguration(
        const Options &options) {
        auto runtimeConfiguration =
            ngc::HostedExecutorRuntimeConfiguration{};
        auto machineFingerprint = std::uint64_t{0};
        if (options.machineConfiguration.has_value()) {
            const auto configuration =
                ngc::loadMachineConfiguration(
                    *options.machineConfiguration);
            if (!configuration.has_value()) {
                throw std::runtime_error(
                    "failed to load machine configuration: "
                    + configuration.error());
            }
            runtimeConfiguration =
                ngc::hostedExecutorRuntimeConfiguration(
                    *configuration);
            machineFingerprint =
                configuration->sourceFingerprint;
        }

        auto backendFingerprint =
            std::optional<std::uint64_t>{};
        if (options.backendConfiguration.has_value()) {
            const auto document =
                ngc::toml_configuration::loadDocument(
                    *options.backendConfiguration);
            if (!document) {
                throw std::runtime_error(
                    "failed to load backend configuration: "
                    + document.error());
            }
            const auto host =
                ngc::loadBackendRuntimeHostConfiguration(
                    document->table,
                    *options.backendConfiguration);
            if (!host) {
                throw std::runtime_error(
                    "failed to load backend configuration: "
                    + host.error());
            }
            runtimeConfiguration.realtime = *host;
            backendFingerprint = document->fingerprint;
        }
        if (options.nonRealtime) {
            runtimeConfiguration.realtime = {};
        }
        if (options.simulatedUdp.has_value()
            && options.simulatedUdp->responseDelay
                >= std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::duration<double>(
                        runtimeConfiguration.servoPeriod))) {
            throw std::runtime_error(
                "simulated UDP response delay must be shorter than the servo period");
        }
        const auto fingerprint =
            ngc::toml_configuration::combinedFingerprint(
                machineFingerprint,
                backendFingerprint,
                runtimeConfiguration.servoPeriod);

        return {
            .runtime = std::move(runtimeConfiguration),
            .fingerprint = fingerprint,
        };
    }

    std::unique_ptr<ngc::HostedExecutorRuntime> makeRuntime(
        ngc::HostedExecutorRuntimeConfiguration configuration,
        std::unique_ptr<ngc::ProductionExecutorIo> io) {
        return std::make_unique<ngc::HostedExecutorRuntime>(
            std::move(configuration), std::move(io));
    }

    int run(const Options &options) {
        auto loaded = loadExecutorConfiguration(options);
        if (options.validateConfigurationOnly) {
            static_cast<void>(makeRuntime(
                std::move(loaded.runtime),
                std::make_unique<TemporaryTriggeredInputs>(
                    options.simulatedUdp)));

            return 0;
        }

        auto memory = ngc::ipc_detail::SharedMemory::open(
            options.mapping, sizeof(ngc::IpcSharedRegion));
        auto &region = *static_cast<ngc::IpcSharedRegion *>(memory.data());
        auto expected = options.expected;
        expected.configurationFingerprint =
            loaded.fingerprint;
        const auto rejection = ngc::validateIpcSharedRegion(
            region, expected);
        if (rejection != ngc::IpcRejection::None) {
            ngc::setIpcRejection(region, rejection);
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::Rejected);

            return 2;
        }
        if (ngc::ipc_detail::parentProcessId()
            != region.frontendProcessId) {
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::PeerLost);

            return 3;
        }

        auto triggeredInputs = std::make_unique<TemporaryTriggeredInputs>(
            options.simulatedUdp);
        auto *triggeredInputsPointer = triggeredInputs.get();
        auto runtime = makeRuntime(
            std::move(loaded.runtime),
            std::move(triggeredInputs));
        runtime->attachEmergencyStopControl(region.emergencyStop);
        if (ngc::ipc_detail::parentProcessId()
            != region.frontendProcessId) {
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::PeerLost);

            return 3;
        }
        runtime->start();
        const auto stopAfterFrontendLoss = [&] {
            ngc::setIpcConnectionState(
                region, ngc::IpcConnectionState::PeerLost);
            static_cast<void>(
                ngc::stopExecutorSafely(*runtime));
            runtime->stop();

            return 3;
        };
        if (ngc::ipc_detail::parentProcessId()
            != region.frontendProcessId) {
            return stopAfterFrontendLoss();
        }
        region.peerProcessId = ngc::ipc_detail::currentProcessId();
        ngc::setIpcConnectionState(region, ngc::IpcConnectionState::PeerReady);
        while (ngc::ipcConnectionState(region)
               == ngc::IpcConnectionState::PeerReady) {
            if (ngc::ipc_detail::parentProcessId()
                != region.frontendProcessId) {
                return stopAfterFrontendLoss();
            }
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
                static_cast<void>(
                    ngc::stopExecutorSafely(*runtime));
                runtime->stop();
                ngc::setIpcConnectionState(
                    region, ngc::IpcConnectionState::PeerStopped);

                return 0;
            }
            if (ngc::ipc_detail::parentProcessId()
                != region.frontendProcessId) {
                return stopAfterFrontendLoss();
            }
            if (options.exitAfterHandshake.has_value()
                && std::chrono::steady_clock::now() - started
                    >= *options.exitAfterHandshake) {
                return 3;
            }

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
        std::cerr << "ngc_ipc_test_peer failed: "
                  << error.what() << '\n';

        return 1;
    }
}
