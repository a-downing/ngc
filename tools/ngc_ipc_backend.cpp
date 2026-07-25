#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <variant>

#include "IpcPlatform.h"
#include "machine/IpcProtocol.h"

namespace {
    struct Options {
        std::string mapping;
        ngc::IpcIdentity expected{};
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

    ngc::RequestId requestId(const ngc::ControlRequest &request) {
        return std::visit([](const auto &value) {
            return value.id;
        }, request);
    }

    void applyControl(const ngc::ControlRequest &request,
                      ngc::ExecutionSnapshot &snapshot) {
        std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, ngc::EnableRequest>) {
                snapshot.state = ngc::BackendState::Held;
            } else if constexpr (std::is_same_v<T, ngc::DisableRequest>) {
                snapshot.state = ngc::BackendState::Disabled;
                snapshot.epoch = 0;
            } else if constexpr (std::is_same_v<T, ngc::StartRequest>
                                 || std::is_same_v<T, ngc::ResumeRequest>) {
                snapshot.state = ngc::BackendState::Running;
                snapshot.epoch = value.epoch;
            } else if constexpr (std::is_same_v<T, ngc::FeedHoldRequest>
                                 || std::is_same_v<T, ngc::ControlledStopRequest>) {
                snapshot.state = ngc::BackendState::Held;
            } else if constexpr (std::is_same_v<T, ngc::AbortRequest>
                                 || std::is_same_v<T, ngc::ResetRequest>) {
                snapshot.state = ngc::BackendState::Held;
                snapshot.epoch = 0;
            }
        }, request);
    }

    bool publishEvent(ngc::IpcSharedRegion &region,
                      const ngc::ExecutionEvent &event) {
        while (!ngc::ipcTryPush(region.events, event)) {
            if (ngc::ipcConnectionState(region)
                == ngc::IpcConnectionState::StopRequested) {
                return false;
            }
            std::this_thread::yield();
        }

        return true;
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

        const auto started = std::chrono::steady_clock::now();
        ngc::ExecutionSnapshot snapshot;
        static_cast<void>(ngc::ipcTryPush(region.snapshots, snapshot));
        std::uint64_t controls = 0;

        for (;;) {
            const auto state = ngc::ipcConnectionState(region);
            if (state == ngc::IpcConnectionState::StopRequested) {
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
            auto progressed = false;
            if (options.consume) {
                ngc::ExecutionItem item;
                if (ngc::ipcTryPop(region.executionItems, item)) {
                    const auto accepted = std::visit([](const auto &value) {
                        return ngc::ExecutionEvent{
                            ngc::ChunkAccepted{value.epoch, value.id}};
                    }, item);
                    if (!publishEvent(region, accepted)) {
                        continue;
                    }
                    progressed = true;
                }

                ngc::ControlRequest request;
                if (ngc::ipcTryPop(region.controls, request)) {
                    applyControl(request, snapshot);
                    if (!publishEvent(region, ngc::RequestCompleted{
                            requestId(request), true})) {
                        continue;
                    }
                    static_cast<void>(ngc::ipcTryPush(
                        region.snapshots, snapshot));
                    progressed = true;
                    ++controls;
                    if (options.exitAfterControls.has_value()
                        && controls >= *options.exitAfterControls) {
                        return 4;
                    }
                }
            }

            if (!progressed) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
