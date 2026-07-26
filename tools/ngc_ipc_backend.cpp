#include <atomic>
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

#include "IpcPlatform.h"
#include "machine/IpcProtocol.h"
#include "machine/MachineConfiguration.h"
#include "machine/ProductionExecutorRuntime.h"

namespace {
    struct Options {
        std::string mapping;
        ngc::IpcIdentity expected{};
        std::optional<std::filesystem::path> machineConfiguration;
        bool consume = true;
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

    ngc::EpochId itemEpoch(const ngc::ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.epoch;
        }, item);
    }

    ngc::ChunkId itemId(const ngc::ExecutionItem &item) noexcept {
        return std::visit([](const auto &value) {
            return value.id;
        }, item);
    }

    std::unique_ptr<ngc::ProductionExecutorRuntime> makeRuntime(
        const Options &options) {
        if (!options.machineConfiguration.has_value()) {
            return std::make_unique<ngc::ProductionExecutorRuntime>(
                ngc::ProductionExecutorRuntimeConfiguration{});
        }

        const auto configuration =
            ngc::loadMachineConfiguration(*options.machineConfiguration);
        if (!configuration.has_value()) {
            throw std::runtime_error(
                "failed to load machine configuration: "
                + configuration.error());
        }

        return std::make_unique<ngc::ProductionExecutorRuntime>(
            *configuration);
    }

    class IpcExecutorBridge {
    public:
        IpcExecutorBridge(ngc::IpcSharedRegion &region,
                          ngc::MotionBackend &backend)
            : m_region(region), m_backend(backend) { }

        bool service(const bool consume) noexcept {
            auto progressed = publishOutputs();
            if (consume) {
                progressed = submitInputs() || progressed;
            }

            return progressed;
        }

        [[nodiscard]] std::uint64_t completedControls() const noexcept {
            return m_completedControls;
        }

    private:
        bool publishOutputs() noexcept {
            auto progressed = false;
            if (!m_pendingEvent.has_value()) {
                ngc::ExecutionEvent event;
                if (m_backend.tryTakeEvent(event)) {
                    m_pendingEvent = event;
                    progressed = true;
                }
            }
            if (m_pendingEvent.has_value()
                && ngc::ipcTryPush(m_region.events, *m_pendingEvent)) {
                if (std::holds_alternative<ngc::RequestCompleted>(
                        *m_pendingEvent)) {
                    ++m_completedControls;
                }
                m_pendingEvent.reset();
                progressed = true;
            }

            if (!m_pendingSnapshot.has_value()) {
                ngc::ExecutionSnapshot snapshot;
                if (m_backend.tryTakeSnapshot(snapshot)) {
                    m_pendingSnapshot = snapshot;
                    progressed = true;
                }
            }
            if (m_pendingSnapshot.has_value()
                && ngc::ipcTryPush(
                    m_region.snapshots, *m_pendingSnapshot)) {
                m_pendingSnapshot.reset();
                progressed = true;
            }

            return progressed;
        }

        bool submitInputs() noexcept {
            auto progressed = false;
            if (!m_pendingItem.has_value()) {
                ngc::ExecutionItem item;
                if (ngc::ipcTryPop(m_region.executionItems, item)) {
                    m_pendingItem = item;
                    progressed = true;
                }
            }
            if (m_pendingItem.has_value()) {
                const auto result = m_backend.tryPublish(*m_pendingItem);
                if (result == ngc::PublishResult::Published) {
                    m_pendingItem.reset();
                    progressed = true;
                } else if (result == ngc::PublishResult::Invalid
                           && !m_pendingEvent.has_value()) {
                    m_pendingEvent = ngc::ChunkRejected{
                        itemEpoch(*m_pendingItem),
                        itemId(*m_pendingItem),
                    };
                    m_pendingItem.reset();
                    progressed = true;
                }
            }

            if (!m_pendingControl.has_value()) {
                ngc::ControlRequest request;
                if (ngc::ipcTryPop(m_region.controls, request)) {
                    m_pendingControl = request;
                    progressed = true;
                }
            }
            if (m_pendingControl.has_value()
                && m_backend.trySubmit(*m_pendingControl)
                    == ngc::SubmitResult::Submitted) {
                m_pendingControl.reset();
                progressed = true;
            }

            return progressed;
        }

        ngc::IpcSharedRegion &m_region;
        ngc::MotionBackend &m_backend;
        std::optional<ngc::ExecutionItem> m_pendingItem;
        std::optional<ngc::ControlRequest> m_pendingControl;
        std::optional<ngc::ExecutionEvent> m_pendingEvent;
        std::optional<ngc::ExecutionSnapshot> m_pendingSnapshot;
        std::uint64_t m_completedControls = 0;
    };

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

        auto runtime = makeRuntime(options);
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

        IpcExecutorBridge bridge(region, runtime->endpoint());
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
