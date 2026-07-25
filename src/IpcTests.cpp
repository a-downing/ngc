#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "machine/ExternalRealtimeRuntime.h"
#include "machine/IpcProtocol.h"

namespace {
    using namespace std::chrono_literals;

    void require(const bool condition, const std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    ngc::IpcIdentity identity() {
        return {
            .configurationFingerprint = 0x10101010,
            .topologyFingerprint = 0x20202020,
            .sessionGeneration = 7,
            .epochGeneration = 9,
            .authorityGeneration = 11,
        };
    }

    ngc::ExternalRealtimeRuntimeConfiguration configuration(
        const std::filesystem::path &peer) {
        return {
            .peerExecutable = peer,
            .identity = identity(),
            .peerExpectedIdentity = identity(),
            .handshakeTimeout = 2s,
            .shutdownTimeout = 2s,
            .peerArguments = {},
        };
    }

    ngc::ExecutionEvent waitForEvent(
        ngc::ExternalRealtimeRuntime &runtime,
        const std::chrono::milliseconds timeout = 2s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        ngc::ExecutionEvent event;
        while (std::chrono::steady_clock::now() < deadline) {
            if (runtime.endpoint().tryTakeEvent(event)) {
                return event;
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error("timed out waiting for IPC event");
    }

    ngc::ExecutionSnapshot waitForSnapshot(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::BackendState expected) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        ngc::ExecutionSnapshot snapshot;
        while (std::chrono::steady_clock::now() < deadline) {
            while (runtime.endpoint().tryTakeSnapshot(snapshot)) {
                if (snapshot.state == expected) {
                    return snapshot;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error("timed out waiting for IPC snapshot");
    }

    void testProtocolLayoutAndBoundedRings() {
        const auto region = std::make_unique<ngc::IpcSharedRegion>();
        ngc::initializeIpcSharedRegion(*region, identity(), 42);
        require(ngc::validateIpcSharedRegion(*region, identity())
                    == ngc::IpcRejection::None,
                "valid IPC region should pass its ABI and identity checks");

        ++region->abiVersion;
        require(ngc::validateIpcSharedRegion(*region, identity())
                    == ngc::IpcRejection::AbiVersion,
                "IPC ABI version mismatch should be rejected");
        --region->abiVersion;
        ++region->regionSize;
        require(ngc::validateIpcSharedRegion(*region, identity())
                    == ngc::IpcRejection::RegionLayout,
                "IPC region layout mismatch should be rejected");
        --region->regionSize;

        auto wrong = identity();
        ++wrong.configurationFingerprint;
        require(ngc::validateIpcSharedRegion(*region, wrong)
                    == ngc::IpcRejection::ConfigurationFingerprint,
                "configuration mismatch should be rejected");
        wrong = identity();
        ++wrong.topologyFingerprint;
        require(ngc::validateIpcSharedRegion(*region, wrong)
                    == ngc::IpcRejection::TopologyFingerprint,
                "topology mismatch should be rejected");
        wrong = identity();
        ++wrong.sessionGeneration;
        require(ngc::validateIpcSharedRegion(*region, wrong)
                    == ngc::IpcRejection::SessionGeneration,
                "stale session should be rejected");
        wrong = identity();
        ++wrong.epochGeneration;
        require(ngc::validateIpcSharedRegion(*region, wrong)
                    == ngc::IpcRejection::EpochGeneration,
                "stale epoch generation should be rejected");
        wrong = identity();
        ++wrong.authorityGeneration;
        require(ngc::validateIpcSharedRegion(*region, wrong)
                    == ngc::IpcRejection::AuthorityGeneration,
                "stale authority should be rejected");

        for (std::uint64_t index = 0;
             index < ngc::IPC_CONTROL_CAPACITY; ++index) {
            const ngc::ControlRequest request = ngc::EnableRequest{index + 1};
            require(ngc::ipcTryPush(region->controls, request),
                    "IPC control ring should accept its advertised capacity");
        }
        const ngc::ControlRequest overflow = ngc::EnableRequest{100};
        require(!ngc::ipcTryPush(region->controls, overflow),
                "IPC control ring should report backpressure when full");
        for (std::uint64_t index = 0;
             index < ngc::IPC_CONTROL_CAPACITY; ++index) {
            ngc::ControlRequest request;
            require(ngc::ipcTryPop(region->controls, request),
                    "IPC control ring should retain every accepted request");
            require(std::get<ngc::EnableRequest>(request).id == index + 1,
                    "IPC control ring should preserve FIFO order");
        }

        for (std::uint64_t index = 0;
             index < ngc::IPC_EVENT_CAPACITY; ++index) {
            const ngc::ExecutionEvent event = ngc::RequestCompleted{index + 1, true};
            require(ngc::ipcTryPush(region->events, event),
                    "IPC event ring should accept its advertised capacity");
        }
        const ngc::ExecutionEvent eventOverflow =
            ngc::RequestCompleted{1000, true};
        require(!ngc::ipcTryPush(region->events, eventOverflow),
                "IPC event ring should report backpressure when full");

        for (std::uint64_t index = 0;
             index < ngc::IPC_SNAPSHOT_CAPACITY; ++index) {
            ngc::ExecutionSnapshot snapshot;
            snapshot.epoch = index + 1;
            require(ngc::ipcTryPush(region->snapshots, snapshot),
                    "IPC snapshot ring should accept its advertised capacity");
        }
        ngc::ExecutionSnapshot snapshotOverflow;
        snapshotOverflow.epoch = 1000;
        require(!ngc::ipcTryPush(region->snapshots, snapshotOverflow),
                "IPC snapshot ring should report backpressure when full");
    }

    void testExternalRuntimeHandshakeAndTransport(
        const std::filesystem::path &peer) {
        ngc::ExternalRealtimeRuntime runtime(configuration(peer));
        runtime.start();
        runtime.start();
        require(runtime.connected(), "external runtime should complete its handshake");

        ngc::PlanChunk chunk;
        chunk.epoch = 17;
        chunk.id = 23;
        require(runtime.endpoint().tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "external runtime should publish an execution item");
        const auto accepted = waitForEvent(runtime);
        require(std::get<ngc::ChunkAccepted>(accepted).epoch == 17
                    && std::get<ngc::ChunkAccepted>(accepted).chunk == 23,
                "IPC peer should acknowledge the exact execution identity");

        require(runtime.endpoint().trySubmit(ngc::EnableRequest{31})
                    == ngc::SubmitResult::Submitted,
                "external runtime should submit a control request");
        const auto completed = waitForEvent(runtime);
        require(std::get<ngc::RequestCompleted>(completed).request == 31
                    && std::get<ngc::RequestCompleted>(completed).succeeded,
                "IPC peer should acknowledge the exact control identity");
        static_cast<void>(waitForSnapshot(runtime, ngc::BackendState::Held));

        runtime.stop();
        runtime.stop();
        require(!runtime.connected(), "stopped external runtime should disconnect");
        runtime.start();
        require(runtime.connected(), "external runtime should support a fresh restart");
        runtime.stop();
    }

    void testExternalRuntimeRejectsStaleHandshake(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        ++options.peerExpectedIdentity.authorityGeneration;
        ngc::ExternalRealtimeRuntime runtime(std::move(options));

        auto rejected = false;
        try {
            runtime.start();
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        require(rejected, "stale authority handshake should fail startup");
        require(runtime.lastRejection()
                    == ngc::IpcRejection::AuthorityGeneration,
                "stale authority handshake should retain its rejection reason");
    }

    void testExternalRuntimeReportsBackpressure(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        options.peerArguments = {"--no-consume"};
        ngc::ExternalRealtimeRuntime runtime(std::move(options));
        runtime.start();

        for (std::uint64_t index = 0;
             index < ngc::IPC_EXECUTION_CAPACITY; ++index) {
            ngc::PlanChunk chunk;
            chunk.epoch = 1;
            chunk.id = index + 1;
            require(runtime.endpoint().tryPublish(chunk)
                        == ngc::PublishResult::Published,
                    "non-consuming IPC peer should expose every bounded slot");
        }
        ngc::PlanChunk overflow;
        overflow.epoch = 1;
        overflow.id = 100;
        require(runtime.endpoint().tryPublish(overflow)
                    == ngc::PublishResult::Full,
                "full IPC execution ring should report backpressure");

        for (std::uint64_t index = 0;
             index < ngc::IPC_CONTROL_CAPACITY; ++index) {
            require(runtime.endpoint().trySubmit(ngc::EnableRequest{index + 1})
                        == ngc::SubmitResult::Submitted,
                    "non-consuming IPC peer should expose every control slot");
        }
        require(runtime.endpoint().trySubmit(ngc::EnableRequest{100})
                    == ngc::SubmitResult::Full,
                "full IPC control ring should report backpressure");
        runtime.stop();
    }

    void testPeerLossAndInterruptedEpochRefusal(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        options.peerArguments = {"--exit-after-controls", "1"};
        ngc::ExternalRealtimeRuntime runtime(std::move(options));
        runtime.start();
        require(runtime.endpoint().trySubmit(ngc::StartRequest{41, 9})
                    == ngc::SubmitResult::Submitted,
                "start request should reach the IPC peer");
        const auto completed = waitForEvent(runtime);
        require(std::get<ngc::RequestCompleted>(completed).request == 41,
                "peer should acknowledge start before exiting");

        const auto loss = waitForEvent(runtime);
        require(std::holds_alternative<ngc::BackendFault>(loss),
                "peer death should become a bounded backend fault");
        require(runtime.endpoint().tryPublish(ngc::PlanChunk{})
                    == ngc::PublishResult::Invalid,
                "peer death should reject later execution publication");

        runtime.stop();
        runtime.start();
        require(runtime.endpoint().trySubmit(ngc::ResumeRequest{42, 9})
                    == ngc::SubmitResult::Submitted,
                "interrupted resume refusal should be reported as a completion");
        const auto refused = waitForEvent(runtime);
        require(std::get<ngc::RequestCompleted>(refused).request == 42
                    && !std::get<ngc::RequestCompleted>(refused).succeeded,
                "an interrupted epoch must not resume after peer restart");
        runtime.stop();
    }
}

int main(const int argc, char **argv) {
    try {
        require(argc == 2, "ngc_ipc_tests requires the peer executable path");
        const std::filesystem::path peer = argv[1];
        testProtocolLayoutAndBoundedRings();
        testExternalRuntimeHandshakeAndTransport(peer);
        testExternalRuntimeRejectsStaleHandshake(peer);
        testExternalRuntimeReportsBackpressure(peer);
        testPeerLossAndInterruptedEpochRefusal(peer);
        std::cout << "ngc_ipc_tests passed\n";

        return 0;
    } catch (const std::exception &error) {
        std::cerr << "ngc_ipc_tests failed: " << error.what() << '\n';

        return 1;
    }
}
