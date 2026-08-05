#include "machine/ExternalExecutorRuntime.h"

#include <chrono>
#include <format>
#include <new>
#include <optional>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>

#include "ExecutionItemOperations.h"
#include "IpcPlatform.h"

namespace ngc {
    namespace {
        constexpr std::uint32_t IPC_PEER_LOST_FAULT = 0x49504301;

        bool emptyIdentity(const IpcIdentity &identity) noexcept {
            return identity.configurationFingerprint == 0
                && identity.sessionGeneration == 0
                && identity.epochGeneration == 0
                && identity.authorityGeneration == 0;
        }

        std::string rejectionMessage(const IpcRejection rejection) {
            switch (rejection) {
                case IpcRejection::None: return "none";
                case IpcRejection::InvalidMagic: return "invalid magic";
                case IpcRejection::AbiVersion: return "ABI version mismatch";
                case IpcRejection::RegionLayout: return "shared-region layout mismatch";
                case IpcRejection::ConfigurationFingerprint:
                    return "configuration fingerprint mismatch";
                case IpcRejection::SessionGeneration:
                    return "session generation mismatch";
                case IpcRejection::EpochGeneration:
                    return "epoch generation mismatch";
                case IpcRejection::AuthorityGeneration:
                    return "control-authority generation mismatch";
            }

            return "unknown rejection";
        }
    }

    class ExternalExecutorRuntime::Impl {
        class Endpoint final : public MotionBackend {
        public:
            explicit Endpoint(Impl &owner) : m_owner(owner) { }

            PublishResult tryPublish(const ExecutionItem &item) noexcept override {
                if (!execution_item::valid(item)) {
                    return PublishResult::Invalid;
                }

                m_owner.refreshPeerState();
                if (!m_owner.running()) {
                    return PublishResult::Invalid;
                }
                if (!ipcTryPush(m_owner.m_region->executionItems, item)) {
                    return PublishResult::Full;
                }

                const auto epoch = execution_item::epoch(item);
                if (epoch != 0) {
                    m_owner.m_lastPublishedEpoch = epoch;
                }

                return PublishResult::Published;
            }

            DemandPublishResult publishDemand(
                const ExecutorDemand &demand) noexcept override {
                m_owner.refreshPeerState();
                if (!m_owner.running()
                    || !m_owner.m_demandProducer.has_value()) {
                    return DemandPublishResult::Unavailable;
                }
                if (demand.generation == 0
                    || (demand.mode != ExecutorDemandMode::Disabled
                        && demand.epoch == 0)
                    || demand.generation
                        <= m_owner.m_lastDemandGeneration) {
                    return DemandPublishResult::Invalid;
                }

                m_owner.m_demandProducer->publish(demand);
                m_owner.m_lastDemandGeneration = demand.generation;

                return DemandPublishResult::Published;
            }

            SubmitResult trySubmit(const ControlRequest &request) noexcept override {
                m_owner.refreshPeerState();
                if (!m_owner.running()) {
                    return SubmitResult::Full;
                }
                if (m_owner.m_localEvent.has_value()) {
                    return SubmitResult::Full;
                }

                if (const auto *resume = std::get_if<ResumeRequest>(&request);
                    resume != nullptr && resume->epoch != 0
                    && resume->epoch == m_owner.m_interruptedEpoch) {
                    m_owner.m_localEvent = RequestCompleted{resume->id, false};

                    return SubmitResult::Submitted;
                }

                if (!ipcTryPush(m_owner.m_region->controls, request)) {
                    return SubmitResult::Full;
                }

                if (const auto *start = std::get_if<StartRequest>(&request)) {
                    m_owner.m_activeEpoch = start->epoch;
                } else if (const auto *resume = std::get_if<ResumeRequest>(&request)) {
                    m_owner.m_activeEpoch = resume->epoch;
                } else if (std::holds_alternative<AbortRequest>(request)
                           || std::holds_alternative<ResetRequest>(request)) {
                    m_owner.m_activeEpoch = 0;
                }

                return SubmitResult::Submitted;
            }

            bool tryTakeEvent(ExecutionEvent &event) noexcept override {
                m_owner.refreshPeerState();
                if (m_owner.m_localEvent.has_value()) {
                    event = *m_owner.m_localEvent;
                    m_owner.m_localEvent.reset();

                    return true;
                }
                if (m_owner.m_region != nullptr
                    && ipcTryPop(m_owner.m_region->events, event)) {
                    return true;
                }
                if (m_owner.m_peerLossEventPending) {
                    event = BackendFault{IPC_PEER_LOST_FAULT};
                    m_owner.m_peerLossEventPending = false;

                    return true;
                }

                return false;
            }

            bool tryTakeSnapshot(ExecutionSnapshot &snapshot) noexcept override {
                m_owner.refreshPeerState();

                return m_owner.m_region != nullptr
                    && ipcTryPop(m_owner.m_region->snapshots, snapshot);
            }

        private:
            Impl &m_owner;
        };

    public:
        explicit Impl(ExternalExecutorRuntimeConfiguration configuration)
            : m_configuration(std::move(configuration)), m_endpoint(*this) {
            if (emptyIdentity(m_configuration.peerExpectedIdentity)) {
                m_configuration.peerExpectedIdentity = m_configuration.identity;
            }
        }

        ~Impl() {
            stop();
        }

        void start() {
            refreshPeerState();
            if (running()) {
                return;
            }
            closeConnection();

            m_sharedMemory = ipc_detail::SharedMemory::create(
                ipc_detail::uniqueSharedMemoryName(), sizeof(IpcSharedRegion));
            m_region = ::new (m_sharedMemory.data()) IpcSharedRegion{};
            initializeIpcSharedRegion(
                *m_region, m_configuration.identity,
                ipc_detail::currentProcessId());
            m_demandProducer.emplace(m_region->demand);
            m_lastDemandGeneration = 0;

            std::vector<std::string> arguments{
                "--mapping", m_sharedMemory.name(),
                "--session", std::to_string(
                    m_configuration.peerExpectedIdentity.sessionGeneration),
                "--epoch", std::to_string(
                    m_configuration.peerExpectedIdentity.epochGeneration),
                "--authority", std::to_string(
                    m_configuration.peerExpectedIdentity.authorityGeneration),
            };
            arguments.insert(
                arguments.end(), m_configuration.peerArguments.begin(),
                m_configuration.peerArguments.end());

            try {
                m_process = ipc_detail::ChildProcess::start(
                    m_configuration.peerExecutable, arguments);
                awaitHandshake();
            } catch (...) {
                if (m_process.running()) {
                    setIpcConnectionState(
                        *m_region, IpcConnectionState::StopRequested);
                    if (!m_process.wait(std::chrono::milliseconds(50))) {
                        m_process.terminate();
                        static_cast<void>(m_process.wait(
                            m_configuration.shutdownTimeout));
                    }
                }
                closeConnection();
                throw;
            }
        }

        void stop() noexcept {
            if (m_region == nullptr) {
                closeConnection();
                return;
            }

            refreshPeerState();
            if (m_process.running()) {
                const auto state = ipcConnectionState(*m_region);
                if (state == IpcConnectionState::Running
                    || state == IpcConnectionState::PeerReady) {
                    setIpcConnectionState(
                        *m_region, IpcConnectionState::StopRequested);
                }

                const auto deadline = std::chrono::steady_clock::now()
                    + m_configuration.shutdownTimeout;
                while (m_process.running()
                       && std::chrono::steady_clock::now() < deadline) {
                    if (ipcConnectionState(*m_region)
                        == IpcConnectionState::PeerStopped) {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                if (!m_process.wait(std::chrono::milliseconds(50))) {
                    m_process.terminate();
                    static_cast<void>(m_process.wait(
                        m_configuration.shutdownTimeout));
                }
            }

            closeConnection();
        }

        void serviceImmediate() noexcept {
            refreshPeerState();
        }

        bool connected() const noexcept {
            return m_region != nullptr && m_process.running()
                && ipcConnectionState(*m_region) == IpcConnectionState::Running;
        }

        IpcRejection lastRejection() const noexcept {
            return m_lastRejection;
        }

        bool tryTakeRealtimeTiming(
            RealtimeTimingSummary &summary) noexcept {
            refreshPeerState();

            return m_region != nullptr
                && ipcTryPop(m_region->realtimeTiming, summary);
        }

        void requestEmergencyStop(const EmergencyStopSource source) noexcept {
            if (m_region != nullptr) {
                EmergencyStopInterface(m_region->emergencyStop).request(source);
            }
        }

        void releaseEmergencyStop(const EmergencyStopSource source) noexcept {
            if (m_region != nullptr) {
                EmergencyStopInterface(m_region->emergencyStop).release(source);
            }
        }

        std::uint64_t requestEmergencyStopReset() noexcept {
            return m_region != nullptr
                ? EmergencyStopInterface(m_region->emergencyStop).requestReset()
                : 0;
        }

        EmergencyStopStatus emergencyStopStatus() const noexcept {
            return m_region != nullptr
                ? EmergencyStopInterface(m_region->emergencyStop).status()
                : EmergencyStopStatus{};
        }

        MotionBackend &endpoint() noexcept {
            return m_endpoint;
        }

    private:
        bool running() const noexcept {
            return m_region != nullptr
                && ipcConnectionState(*m_region) == IpcConnectionState::Running;
        }

        void awaitHandshake() {
            const auto deadline = std::chrono::steady_clock::now()
                + m_configuration.handshakeTimeout;
            while (std::chrono::steady_clock::now() < deadline) {
                const auto state = ipcConnectionState(*m_region);
                if (state == IpcConnectionState::PeerReady) {
                    setIpcConnectionState(*m_region, IpcConnectionState::Running);

                    return;
                }
                if (state == IpcConnectionState::Rejected) {
                    m_lastRejection = ipcRejection(*m_region);
                    throw std::runtime_error(std::format(
                        "external backend rejected IPC handshake: {}",
                        rejectionMessage(m_lastRejection)));
                }
                if (!m_process.running()) {
                    throw std::runtime_error(
                        "external backend exited during IPC handshake");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            throw std::runtime_error("external backend IPC handshake timed out");
        }

        void refreshPeerState() noexcept {
            if (m_region == nullptr || m_process.running()) {
                return;
            }

            const auto state = ipcConnectionState(*m_region);
            if (state != IpcConnectionState::PeerStopped
                && state != IpcConnectionState::Rejected
                && state != IpcConnectionState::PeerLost) {
                setIpcConnectionState(*m_region, IpcConnectionState::PeerLost);
                if (m_activeEpoch != 0) {
                    m_interruptedEpoch = m_activeEpoch;
                } else if (m_lastPublishedEpoch != 0) {
                    m_interruptedEpoch = m_lastPublishedEpoch;
                }
                m_activeEpoch = 0;
                m_peerLossEventPending = true;
            }
        }

        void closeConnection() noexcept {
            m_demandProducer.reset();
            m_region = nullptr;
            m_process.close();
            m_sharedMemory.close();
            m_activeEpoch = 0;
            m_lastPublishedEpoch = 0;
            m_localEvent.reset();
            m_peerLossEventPending = false;
        }

        ExternalExecutorRuntimeConfiguration m_configuration;
        ipc_detail::SharedMemory m_sharedMemory;
        ipc_detail::ChildProcess m_process;
        IpcSharedRegion *m_region = nullptr;
        Endpoint m_endpoint;
        std::optional<SharedLatestValueProducer<ExecutorDemand>>
            m_demandProducer;
        DemandGeneration m_lastDemandGeneration = 0;
        IpcRejection m_lastRejection = IpcRejection::None;
        EpochId m_activeEpoch = 0;
        EpochId m_lastPublishedEpoch = 0;
        EpochId m_interruptedEpoch = 0;
        std::optional<ExecutionEvent> m_localEvent;
        bool m_peerLossEventPending = false;
    };

    ExternalExecutorRuntime::ExternalExecutorRuntime(
        ExternalExecutorRuntimeConfiguration configuration)
        : m_impl(std::make_unique<Impl>(std::move(configuration))) { }

    ExternalExecutorRuntime::~ExternalExecutorRuntime() = default;

    MotionBackend &ExternalExecutorRuntime::endpoint() noexcept {
        return m_impl->endpoint();
    }

    void ExternalExecutorRuntime::start() {
        m_impl->start();
    }

    void ExternalExecutorRuntime::stop() {
        m_impl->stop();
    }

    BackendCapabilities ExternalExecutorRuntime::capabilities() const noexcept {
        return {};
    }

    bool ExternalExecutorRuntime::restoreStationaryState(
        const StationaryBackendState &) noexcept {
        return false;
    }

    bool ExternalExecutorRuntime::prepareTriggeredJointMove(
        const TriggeredJointMove &) noexcept {
        return m_impl->connected();
    }

    void ExternalExecutorRuntime::serviceImmediate() {
        m_impl->serviceImmediate();
    }

    std::uint64_t ExternalExecutorRuntime::advanceServiceMotionPeriod() {
        m_impl->serviceImmediate();

        return 0;
    }

    void ExternalExecutorRuntime::waitForServiceMotion() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    bool ExternalExecutorRuntime::connected() const noexcept {
        return m_impl->connected();
    }

    IpcRejection ExternalExecutorRuntime::lastRejection() const noexcept {
        return m_impl->lastRejection();
    }

    bool ExternalExecutorRuntime::tryTakeRealtimeTiming(
        RealtimeTimingSummary &summary) noexcept {
        return m_impl->tryTakeRealtimeTiming(summary);
    }

    void ExternalExecutorRuntime::requestEmergencyStop(
        const EmergencyStopSource source) noexcept {
        m_impl->requestEmergencyStop(source);
    }

    void ExternalExecutorRuntime::releaseEmergencyStop(
        const EmergencyStopSource source) noexcept {
        m_impl->releaseEmergencyStop(source);
    }

    std::uint64_t ExternalExecutorRuntime::requestEmergencyStopReset() noexcept {
        return m_impl->requestEmergencyStopReset();
    }

    EmergencyStopStatus ExternalExecutorRuntime::emergencyStopStatus() const noexcept {
        return m_impl->emergencyStopStatus();
    }
}
