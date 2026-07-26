#include <chrono>
#include <cmath>
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
            .peerArguments = {
                "--machine-configuration",
                std::filesystem::absolute("machine.toml").string(),
            },
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

    ngc::RequestCompleted waitForRequest(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::RequestId request) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto event = waitForEvent(runtime);
            if (const auto *completed =
                    std::get_if<ngc::RequestCompleted>(&event);
                completed != nullptr && completed->request == request) {
                return *completed;
            }
        }

        throw std::runtime_error("timed out waiting for IPC control completion");
    }

    ngc::ExecutionSnapshot waitForSnapshot(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::BackendState expected,
        const std::optional<double> expectedX = std::nullopt) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        ngc::ExecutionSnapshot snapshot;
        while (std::chrono::steady_clock::now() < deadline) {
            while (runtime.endpoint().tryTakeSnapshot(snapshot)) {
                if (snapshot.state == expected
                    && (!expectedX.has_value()
                        || std::abs(
                            snapshot.commanded.position.x - *expectedX)
                            < 1e-9)) {
                    return snapshot;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error("timed out waiting for IPC snapshot");
    }

    ngc::ExecutionSnapshot waitForMotionProgress(
        ngc::ExternalRealtimeRuntime &runtime, const double minimumX) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        ngc::ExecutionSnapshot snapshot;
        while (std::chrono::steady_clock::now() < deadline) {
            while (runtime.endpoint().tryTakeSnapshot(snapshot)) {
                if (snapshot.state == ngc::BackendState::Running
                    && snapshot.commanded.position.x >= minimumX) {
                    return snapshot;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error("timed out waiting for IPC motion progress");
    }

    ngc::BackendHeld waitForHeld(
        ngc::ExternalRealtimeRuntime &runtime,
        const ngc::BackendHoldReason reason) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto event = waitForEvent(runtime);
            if (const auto *fault = std::get_if<ngc::BackendFault>(&event)) {
                throw std::runtime_error(
                    "IPC executor faulted while waiting for held state: "
                    + std::to_string(fault->code));
            }
            if (const auto *held = std::get_if<ngc::BackendHeld>(&event);
                held != nullptr && held->reason == reason) {
                return *held;
            }
        }

        throw std::runtime_error("timed out waiting for IPC held event");
    }

    ngc::AxisPolynomialSpan linearSpan(
        const ngc::SpanId id, const double from, const double to,
        const double duration) {
        ngc::AxisPolynomialSpan span;
        span.id = id;
        span.duration = duration;
        span.inverseDuration = 1.0 / duration;
        span.inverseDurationSquared =
            span.inverseDuration * span.inverseDuration;
        span.inverseDurationCubed =
            span.inverseDurationSquared * span.inverseDuration;
        span.origin.x = from;
        span.coefficients[0].x = to - from;

        return span;
    }

    ngc::PlanChunk linearChunk(
        const ngc::EpochId epoch, const ngc::ChunkId id,
        const double from, const double to, const double duration) {
        ngc::PlanChunk chunk;
        chunk.epoch = epoch;
        chunk.id = id;
        chunk.branch = id + 100;
        require(chunk.normalMotion.push(
                    linearSpan(id + 200, from, to, duration)),
                "IPC normal span fixture did not fit");
        require(chunk.stopTail.push(
                    linearSpan(id + 201, to, to, 0.01)),
                "IPC stop span fixture did not fit");
        require(chunk.markers.push({id + 300, 0, 0.5}),
                "IPC marker fixture did not fit");
        chunk.branchState = ngc::executionSpanEnd(chunk.normalMotion[0]);
        chunk.stopState.position.x = to;

        return chunk;
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

    void testExternalRuntimeExecutesThroughProductionCore(
        const std::filesystem::path &peer) {
        ngc::ExternalRealtimeRuntime runtime(configuration(peer));
        runtime.start();
        runtime.start();
        require(runtime.connected(), "external runtime should complete its handshake");

        constexpr ngc::EpochId epoch = 17;
        require(runtime.endpoint().trySubmit(ngc::ResetRequest{21, epoch})
                    == ngc::SubmitResult::Submitted,
                "external runtime should submit an executor reset");
        require(waitForRequest(runtime, 21).succeeded,
                "IPC executor should accept its epoch reset");
        require(runtime.endpoint().trySubmit(ngc::EnableRequest{22})
                    == ngc::SubmitResult::Submitted,
                "external runtime should submit an enable request");
        require(waitForRequest(runtime, 22).succeeded,
                "IPC executor should enable through the production core");
        static_cast<void>(waitForSnapshot(runtime, ngc::BackendState::Held));

        const auto chunk = linearChunk(epoch, 23, 0.0, 0.125, 0.03);
        require(runtime.endpoint().tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "external runtime should publish an execution item");
        require(runtime.endpoint().trySubmit(ngc::StartRequest{24, epoch})
                    == ngc::SubmitResult::Submitted,
                "external runtime should start the published epoch");
        require(waitForRequest(runtime, 24).succeeded,
                "IPC executor should start the published epoch");

        const auto accepted = waitForEvent(runtime);
        require(std::get<ngc::ChunkAccepted>(accepted).epoch == epoch
                    && std::get<ngc::ChunkAccepted>(accepted).chunk == chunk.id,
                "IPC executor should accept the exact execution identity");
        const auto marker = waitForEvent(runtime);
        require(std::get<ngc::ExecutionMarkerReached>(marker).marker
                    == chunk.markers[0].id,
                "IPC executor should report a marker from its timed cursor");
        const auto branch = waitForEvent(runtime);
        require(std::get<ngc::BranchSelected>(branch).choice
                    == ngc::BranchChoice::Stop,
                "IPC executor should select the terminal stop branch");
        const auto retired = waitForEvent(runtime);
        require(std::get<ngc::ChunkRetired>(retired).chunk == chunk.id,
                "IPC executor should retire the executed chunk");
        const auto held = waitForEvent(runtime);
        require(std::get<ngc::BackendHeld>(held).reason
                    == ngc::BackendHoldReason::StopBranch,
                "IPC executor should report its terminal held state");
        static_cast<void>(waitForSnapshot(
            runtime, ngc::BackendState::Held, 0.125));

        runtime.stop();
        runtime.stop();
        require(!runtime.connected(), "stopped external runtime should disconnect");
        runtime.start();
        require(runtime.connected(), "external runtime should support a fresh restart");
        runtime.stop();
    }

    void testExternalRuntimeFeedHoldAndResume(
        const std::filesystem::path &peer) {
        ngc::ExternalRealtimeRuntime runtime(configuration(peer));
        runtime.start();

        constexpr ngc::EpochId epoch = 18;
        require(runtime.endpoint().trySubmit(ngc::ResetRequest{41, epoch})
                    == ngc::SubmitResult::Submitted,
                "feed-hold IPC reset should fit");
        require(waitForRequest(runtime, 41).succeeded,
                "feed-hold IPC reset should succeed");
        require(runtime.endpoint().trySubmit(ngc::EnableRequest{42})
                    == ngc::SubmitResult::Submitted,
                "feed-hold IPC enable should fit");
        require(waitForRequest(runtime, 42).succeeded,
                "feed-hold IPC enable should succeed");

        const auto chunk = linearChunk(epoch, 43, 0.0, 10.0, 10.0);
        require(runtime.endpoint().tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "feed-hold IPC chunk should fit");
        require(runtime.endpoint().trySubmit(ngc::StartRequest{44, epoch})
                    == ngc::SubmitResult::Submitted,
                "feed-hold IPC start should fit");
        require(waitForRequest(runtime, 44).succeeded,
                "feed-hold IPC start should succeed");
        const auto accepted = waitForEvent(runtime);
        require(std::get<ngc::ChunkAccepted>(accepted).chunk == chunk.id,
                "feed-hold IPC chunk should activate");
        static_cast<void>(waitForMotionProgress(runtime, 0.1));

        require(runtime.endpoint().trySubmit(ngc::FeedHoldRequest{45})
                    == ngc::SubmitResult::Submitted,
                "IPC feed hold should fit");
        require(waitForRequest(runtime, 45).succeeded,
                "production executor should accept IPC feed hold");
        const auto feedHeld = waitForHeld(
            runtime, ngc::BackendHoldReason::FeedHold);
        require(feedHeld.state.position.x > 0.0
                    && feedHeld.state.position.x < 10.0,
                "IPC feed hold should stop on the active path");

        require(runtime.endpoint().trySubmit(
                    ngc::ResumeRequest{46, epoch})
                    == ngc::SubmitResult::Submitted,
                "IPC resume should fit");
        require(waitForRequest(runtime, 46).succeeded,
                "production executor should resume the held IPC cursor");
        static_cast<void>(waitForMotionProgress(
            runtime, feedHeld.state.position.x + 0.01));
        require(runtime.endpoint().trySubmit(ngc::AbortRequest{47})
                    == ngc::SubmitResult::Submitted,
                "resumed IPC abort should fit");
        require(waitForRequest(runtime, 47).succeeded,
                "resumed IPC motion should accept Abort");
        static_cast<void>(waitForSnapshot(
            runtime, ngc::BackendState::Held));

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

    void testExternalRuntimeRejectsInvalidExecutorConfiguration(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        options.peerArguments = {
            "--machine-configuration",
            std::filesystem::absolute(
                "missing_ipc_machine_configuration.toml").string(),
        };
        ngc::ExternalRealtimeRuntime runtime(std::move(options));

        auto rejected = false;
        try {
            runtime.start();
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        require(rejected,
                "invalid executor configuration should fail IPC startup");
        require(!runtime.connected(),
                "invalid executor configuration should not advertise readiness");
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
        testExternalRuntimeExecutesThroughProductionCore(peer);
        testExternalRuntimeFeedHoldAndResume(peer);
        testExternalRuntimeRejectsStaleHandshake(peer);
        testExternalRuntimeRejectsInvalidExecutorConfiguration(peer);
        testExternalRuntimeReportsBackpressure(peer);
        testPeerLossAndInterruptedEpochRefusal(peer);
        std::cout << "ngc_ipc_tests passed\n";

        return 0;
    } catch (const std::exception &error) {
        std::cerr << "ngc_ipc_tests failed: " << error.what() << '\n';

        return 1;
    }
}
