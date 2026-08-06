#include "machine/IpcExecutorPeer.h"

#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "IpcPlatform.h"

namespace ngc {
    std::uint64_t parseUnsignedCommandLineValue(
        const std::string_view value) {
        std::size_t consumed = 0;
        const auto result = std::stoull(std::string(value), &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("invalid unsigned integer");
        }

        return result;
    }

    bool parseIpcExecutorPeerOption(
        int &index, const int argc, char **argv,
        IpcExecutorPeerOptions &options) {
        const auto option = std::string_view(argv[index]);
        const auto value = [&]() -> std::string_view {
            if (++index == argc) {
                throw std::runtime_error(
                    "missing option value for " + std::string(option));
            }

            return argv[index];
        };

        if (option == "--mapping") {
            options.mapping = value();
        } else if (option == "--session") {
            options.expected.sessionGeneration =
                parseUnsignedCommandLineValue(value());
        } else if (option == "--epoch") {
            options.expected.epochGeneration =
                parseUnsignedCommandLineValue(value());
        } else if (option == "--authority") {
            options.expected.authorityGeneration =
                parseUnsignedCommandLineValue(value());
        } else if (option == "--validate-config-only") {
            options.validateConfigurationOnly = true;
        } else {
            return false;
        }

        return true;
    }

    void validateIpcExecutorPeerOptions(
        const IpcExecutorPeerOptions &options) {
        if (!options.validateConfigurationOnly && options.mapping.empty()) {
            throw std::runtime_error("--mapping is required");
        }
    }

    int runIpcExecutorPeer(
        const IpcExecutorPeerOptions &options,
        const std::uint64_t configurationFingerprint,
        IpcExecutorPeerRuntimeFactory makeRuntime) {
        auto memory = ipc_detail::SharedMemory::open(
            options.mapping, sizeof(IpcSharedRegion));
        auto &region = *static_cast<IpcSharedRegion *>(memory.data());
        auto expected = options.expected;
        expected.configurationFingerprint = configurationFingerprint;
        const auto rejection = validateIpcSharedRegion(region, expected);
        if (rejection != IpcRejection::None) {
            setIpcRejection(region, rejection);
            setIpcConnectionState(region, IpcConnectionState::Rejected);

            return 2;
        }
        if (ipc_detail::parentProcessId() != region.frontendProcessId) {
            setIpcConnectionState(region, IpcConnectionState::PeerLost);

            return 3;
        }

        auto peerRuntime = makeRuntime();
        if (!peerRuntime.runtime) {
            throw std::runtime_error(
                "IPC executor peer runtime factory returned no runtime");
        }
        auto &runtime = *peerRuntime.runtime;
        runtime.attachEmergencyStopControl(region.emergencyStop);
        if (ipc_detail::parentProcessId() != region.frontendProcessId) {
            setIpcConnectionState(region, IpcConnectionState::PeerLost);

            return 3;
        }

        runtime.start();
        const auto stopAfterFrontendLoss = [&] {
            setIpcConnectionState(region, IpcConnectionState::PeerLost);
            static_cast<void>(stopExecutorSafely(runtime));
            runtime.stop();

            return 3;
        };
        if (ipc_detail::parentProcessId() != region.frontendProcessId) {
            return stopAfterFrontendLoss();
        }

        region.peerProcessId = ipc_detail::currentProcessId();
        setIpcConnectionState(region, IpcConnectionState::PeerReady);
        while (ipcConnectionState(region) == IpcConnectionState::PeerReady) {
            if (ipc_detail::parentProcessId() != region.frontendProcessId) {
                return stopAfterFrontendLoss();
            }
            std::this_thread::yield();
        }
        if (ipcConnectionState(region) != IpcConnectionState::Running) {
            runtime.stop();
            setIpcConnectionState(region, IpcConnectionState::PeerStopped);

            return 0;
        }

        IpcExecutorBridge bridge(
            region, runtime, runtime.endpoint(), peerRuntime.policy);
        const auto started = std::chrono::steady_clock::now();
        for (;;) {
            if (ipcConnectionState(region) == IpcConnectionState::StopRequested) {
                static_cast<void>(stopExecutorSafely(runtime));
                runtime.stop();
                setIpcConnectionState(region, IpcConnectionState::PeerStopped);

                return 0;
            }
            if (ipc_detail::parentProcessId() != region.frontendProcessId) {
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
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    }
}
