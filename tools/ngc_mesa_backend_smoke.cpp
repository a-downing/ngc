#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <format>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

#include "machine/ExternalRealtimeRuntime.h"
#include "mesa/MesaProductionExecutorIo.h"

namespace {
    using namespace std::chrono_literals;

    struct Options {
        std::filesystem::path peer = "build/ngc_mesa_backend";
        std::filesystem::path machine = "machine.toml";
        std::filesystem::path backend =
            "physical_backend.toml";
        bool expectExternalEnableLoss = false;
    };

    Options parseOptions(const int argc, char **argv) {
        Options result;
        for (auto index = 1; index < argc; ++index) {
            const auto option = std::string_view(argv[index]);
            const auto value = [&]() -> std::string_view {
                if (++index == argc) {
                    throw std::runtime_error(
                        "missing option value for "
                        + std::string(option));
                }

                return argv[index];
            };

            if (option == "--peer") {
                result.peer = value();
            } else if (option == "--machine-config") {
                result.machine = value();
            } else if (option == "--backend-config") {
                result.backend = value();
            } else if (option == "--expect-external-enable-loss") {
                result.expectExternalEnableLoss = true;
            } else {
                throw std::runtime_error(
                    "unknown option: " + std::string(option));
            }
        }

        return result;
    }

    ngc::RequestCompleted waitForRequest(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::RequestId request) {
        const auto deadline =
            std::chrono::steady_clock::now() + 5s;
        while (std::chrono::steady_clock::now() < deadline) {
            runtime.serviceImmediate();
            ngc::ExecutionEvent event;
            while (runtime.endpoint().tryTakeEvent(event)) {
                if (const auto *fault =
                        std::get_if<ngc::BackendFault>(&event)) {
                    throw std::runtime_error(std::format(
                        "physical backend faulted with code 0x{:08X}",
                        fault->code));
                }
                if (const auto *completed =
                        std::get_if<ngc::RequestCompleted>(&event);
                    completed != nullptr
                    && completed->request == request) {
                    return *completed;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error(
            "timed out waiting for physical backend request");
    }

    ngc::ExecutionSnapshot waitForState(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::BackendState state) {
        const auto deadline =
            std::chrono::steady_clock::now() + 5s;
        while (std::chrono::steady_clock::now() < deadline) {
            runtime.serviceImmediate();
            ngc::ExecutionSnapshot snapshot;
            while (runtime.endpoint().tryTakeSnapshot(snapshot)) {
                if (snapshot.faultCode != 0) {
                    throw std::runtime_error(std::format(
                        "physical backend snapshot faulted with code 0x{:08X}",
                        snapshot.faultCode));
                }
                if (snapshot.state == state) {
                    return snapshot;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error(
            "timed out waiting for physical backend state");
    }

    ngc::RealtimeTimingSummary waitForTiming(
        ngc::ExternalRealtimeRuntime &runtime) {
        const auto deadline =
            std::chrono::steady_clock::now() + 5s;
        while (std::chrono::steady_clock::now() < deadline) {
            runtime.serviceImmediate();
            ngc::RealtimeTimingSummary timing;
            if (runtime.tryTakeRealtimeTiming(timing)
                && timing.sampleCount != 0) {
                return timing;
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error(
            "timed out waiting for physical backend timing");
    }

    ngc::ExecutionSnapshot waitForExternalEnableLoss(
        ngc::ExternalRealtimeRuntime &runtime) {
        const auto deadline =
            std::chrono::steady_clock::now()
            + std::chrono::minutes(2);
        auto eventSeen = false;
        auto snapshot = ngc::ExecutionSnapshot{};
        auto snapshotSeen = false;
        while (std::chrono::steady_clock::now() < deadline) {
            runtime.serviceImmediate();
            ngc::ExecutionEvent event;
            while (runtime.endpoint().tryTakeEvent(event)) {
                if (const auto *fault =
                        std::get_if<ngc::BackendFault>(&event)) {
                    if (fault->code
                        != ngc::mesa::MESA_EXTERNAL_ENABLE_FAULT) {
                        throw std::runtime_error(std::format(
                            "physical backend faulted with unexpected "
                            "code 0x{:08X}",
                            fault->code));
                    }
                    eventSeen = true;
                }
            }
            ngc::ExecutionSnapshot candidate;
            while (runtime.endpoint().tryTakeSnapshot(candidate)) {
                if (candidate.faultCode != 0
                    && candidate.faultCode
                        != ngc::mesa::MESA_EXTERNAL_ENABLE_FAULT) {
                    throw std::runtime_error(std::format(
                        "physical backend snapshot faulted with "
                        "unexpected code 0x{:08X}",
                        candidate.faultCode));
                }
                if (candidate.state == ngc::BackendState::Faulted
                    && candidate.faultCode
                        == ngc::mesa::MESA_EXTERNAL_ENABLE_FAULT) {
                    snapshot = candidate;
                    snapshotSeen = true;
                }
            }
            if (eventSeen && snapshotSeen) {
                return snapshot;
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error(
            "timed out waiting for external-enable loss");
    }

    void requireRequest(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::ControlRequest &request,
        const ngc::RequestId id) {
        if (runtime.endpoint().trySubmit(request)
            != ngc::SubmitResult::Submitted) {
            throw std::runtime_error(
                "physical backend control channel was full");
        }
        if (!waitForRequest(runtime, id).succeeded) {
            throw std::runtime_error(
                "physical backend rejected a smoke-test control");
        }
    }

    int run(const Options &options) {
        const auto identity = ngc::IpcIdentity{
            .configurationFingerprint = 0x4D455341,
            .topologyFingerprint = 0x37493936,
            .sessionGeneration = 1,
            .epochGeneration = 1,
            .authorityGeneration = 1,
        };
        auto runtime = ngc::ExternalRealtimeRuntime({
            .peerExecutable = std::filesystem::absolute(
                options.peer),
            .identity = identity,
            .peerExpectedIdentity = identity,
            .handshakeTimeout = 5s,
            .shutdownTimeout = 5s,
            .peerArguments = {
                "--machine-configuration",
                std::filesystem::absolute(options.machine).string(),
                "--backend-configuration",
                std::filesystem::absolute(
                    options.backend).string(),
            },
        });
        runtime.start();
        static_cast<void>(waitForState(
            runtime, ngc::BackendState::Disabled));

        requireRequest(
            runtime, ngc::ResetRequest{1, 1}, 1);
        requireRequest(
            runtime, ngc::EnableRequest{2}, 2);
        const auto enabled = waitForState(
            runtime, ngc::BackendState::Held);
        const auto zero = [](const ngc::JointVector &values) {
            return std::ranges::all_of(
                values.values, [](const double value) {
                    return value == 0.0;
                });
        };
        if (!zero(enabled.commandedJoints.position)
            || !zero(enabled.commandedJoints.velocity)
            || !zero(enabled.commandedJoints.acceleration)) {
            throw std::runtime_error(
                "zero-motion smoke test observed commanded joint motion");
        }

        const auto timing = waitForTiming(runtime);
        if (options.expectExternalEnableLoss) {
            std::println(
                "READY: remove +5V from INPUT2 now; "
                "waiting for external-enable loss");
            std::fflush(stdout);
            const auto faulted =
                waitForExternalEnableLoss(runtime);
            if (!zero(faulted.commandedJoints.velocity)
                || !zero(faulted.commandedJoints.acceleration)) {
                throw std::runtime_error(
                    "external-enable fault retained commanded motion");
            }
            runtime.stop();
            std::println(
                "Mesa external-enable loss passed: "
                "fault=0x{:08X} commanded joint velocity and "
                "acceleration remained zero",
                faulted.faultCode);

            return 0;
        }

        requireRequest(
            runtime, ngc::DisableRequest{3}, 3);
        static_cast<void>(waitForState(
            runtime, ngc::BackendState::Disabled));
        runtime.stop();

        std::println(
            "Mesa physical backend zero-motion smoke passed: "
            "timing_samples={} maximum_execution={} ns "
            "minimum_slack={} ns missed_deadlines={}",
            timing.sampleCount,
            timing.maximumExecutionNanoseconds,
            timing.minimumDeadlineSlackNanoseconds,
            timing.missedDeadlines);

        return 0;
    }
}

int main(const int argc, char **argv) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception &error) {
        std::println(
            stderr,
            "Mesa physical backend smoke failed: {}",
            error.what());

        return 1;
    }
}
