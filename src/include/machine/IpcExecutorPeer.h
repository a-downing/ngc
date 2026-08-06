#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "machine/HostedExecutorRuntime.h"
#include "machine/IpcExecutorBridge.h"
#include "machine/IpcProtocol.h"

namespace ngc {
    [[nodiscard]] std::uint64_t parseUnsignedCommandLineValue(
        std::string_view value);

    struct IpcExecutorPeerOptions {
        std::string mapping;
        IpcIdentity expected{};
        bool consume = true;
        bool validateConfigurationOnly = false;
        std::optional<std::uint64_t> exitAfterControls;
        std::optional<std::chrono::milliseconds> exitAfterHandshake;
    };

    [[nodiscard]] bool parseIpcExecutorPeerOption(
        int &index, int argc, char **argv, IpcExecutorPeerOptions &options);
    void validateIpcExecutorPeerOptions(const IpcExecutorPeerOptions &options);

    struct IpcExecutorPeerRuntime {
        std::unique_ptr<HostedExecutorRuntime> runtime;
        IpcExecutorPolicy *policy = nullptr;
    };

    using IpcExecutorPeerRuntimeFactory =
        std::function<IpcExecutorPeerRuntime()>;

    [[nodiscard]] int runIpcExecutorPeer(
        const IpcExecutorPeerOptions &options,
        std::uint64_t configurationFingerprint,
        IpcExecutorPeerRuntimeFactory makeRuntime);
}
