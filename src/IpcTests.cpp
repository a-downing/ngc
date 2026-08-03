#include <bit>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "config/ConfigurationFingerprint.h"
#include "machine/ExternalExecutorRuntime.h"
#include "machine/IpcExecutorBridge.h"
#include "machine/IpcProtocol.h"
#include "machine/MachineConfiguration.h"
#include "machine/MachineSessionManager.h"

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
            .sessionGeneration = 7,
            .epochGeneration = 9,
            .authorityGeneration = 11,
        };
    }

    ngc::ExternalExecutorRuntimeConfiguration configuration(
        const std::filesystem::path &peer) {
        const auto machine =
            ngc::loadMachineConfiguration("machine.toml");
        require(machine.has_value(),
                machine ? "" : machine.error());
        const auto servoPeriod =
            machine->machineExecutor.has_value()
                ? machine->machineExecutor->servoPeriod
                : machine->simulation.servoPeriod;
        auto configuredIdentity = identity();
        configuredIdentity.configurationFingerprint =
            ngc::toml_configuration::combinedFingerprint(
                machine->sourceFingerprint,
                std::nullopt,
                servoPeriod);

        return {
            .peerExecutable = peer,
            .identity = configuredIdentity,
            .peerExpectedIdentity = configuredIdentity,
            .handshakeTimeout = 2s,
            .shutdownTimeout = 2s,
            .peerArguments = {
                "--machine-configuration",
                std::filesystem::absolute("machine.toml").string(),
                "--non-realtime",
            },
        };
    }

    ngc::ExecutionEvent waitForEvent(
        ngc::ExternalExecutorRuntime &runtime,
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
        ngc::ExternalExecutorRuntime &runtime,
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
        ngc::ExternalExecutorRuntime &runtime,
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

    ngc::RealtimeTimingSummary waitForRealtimeTiming(
        ngc::ExternalExecutorRuntime &runtime,
        const std::chrono::milliseconds timeout = 2s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        ngc::RealtimeTimingSummary timing;
        while (std::chrono::steady_clock::now() < deadline) {
            if (runtime.tryTakeRealtimeTiming(timing)) {
                return timing;
            }
            std::this_thread::sleep_for(1ms);
        }

        throw std::runtime_error(
            "timed out waiting for IPC real-time timing summary");
    }

    ngc::ExecutionSnapshot waitForMotionProgress(
        ngc::ExternalExecutorRuntime &runtime, const double minimumX) {
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
        ngc::ExternalExecutorRuntime &runtime,
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

        for (std::uint64_t index = 0;
             index < ngc::IPC_REALTIME_TIMING_CAPACITY; ++index) {
            ngc::RealtimeTimingSummary timing;
            timing.firstTick = index + 1;
            require(ngc::ipcTryPush(region->realtimeTiming, timing),
                    "IPC timing ring should accept its advertised capacity");
        }
        ngc::RealtimeTimingSummary timingOverflow;
        timingOverflow.firstTick = 1000;
        require(!ngc::ipcTryPush(
                    region->realtimeTiming, timingOverflow),
                "IPC timing ring should report backpressure when full");
        for (std::uint64_t index = 0;
             index < ngc::IPC_REALTIME_TIMING_CAPACITY; ++index) {
            ngc::RealtimeTimingSummary timing;
            require(ngc::ipcTryPop(region->realtimeTiming, timing),
                    "IPC timing ring should retain every accepted summary");
            require(timing.firstTick == index + 1,
                    "IPC timing ring should preserve FIFO order");
        }
    }

    void testExternalRuntimeExecutesThroughProductionCore(
        const std::filesystem::path &peer) {
        ngc::ExternalExecutorRuntime runtime(configuration(peer));
        runtime.start();
        runtime.start();
        require(runtime.connected(), "external runtime should complete its handshake");
        const auto timing = waitForRealtimeTiming(runtime);
        require(timing.sampleCount != 0
                    && timing.maximumExecutionNanoseconds != 0,
                "external runtime should expose peer servo timing");

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

    void testExternalRuntimeSimulatesUdpExchange(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        options.peerArguments.push_back(
            "--simulated-udp-response-us");
        options.peerArguments.push_back("50");
        ngc::ExternalExecutorRuntime runtime(std::move(options));
        runtime.start();

        const auto timing = waitForRealtimeTiming(runtime);
        require(timing.sampleCount != 0
                    && timing.maximumExecutionNanoseconds >= 50'000,
                "simulated UDP exchange was not included in peer execution timing");

        runtime.stop();
    }

    void testExternalRuntimeFeedHoldAndResume(
        const std::filesystem::path &peer) {
        ngc::ExternalExecutorRuntime runtime(configuration(peer));
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

    void testFrontendLossShutdownStopsAndDisables(
        const std::filesystem::path &peer) {
        ngc::ExternalExecutorRuntime runtime(configuration(peer));
        runtime.start();

        constexpr ngc::EpochId epoch = 20;
        require(runtime.endpoint().trySubmit(
                    ngc::ResetRequest{61, epoch})
                    == ngc::SubmitResult::Submitted,
                "frontend-loss reset should fit");
        require(waitForRequest(runtime, 61).succeeded,
                "frontend-loss reset should succeed");
        require(runtime.endpoint().trySubmit(ngc::EnableRequest{62})
                    == ngc::SubmitResult::Submitted,
                "frontend-loss enable should fit");
        require(waitForRequest(runtime, 62).succeeded,
                "frontend-loss enable should succeed");

        const auto chunk = linearChunk(
            epoch, 63, 0.0, 10.0, 10.0);
        require(runtime.endpoint().tryPublish(chunk)
                    == ngc::PublishResult::Published,
                "frontend-loss chunk should fit");
        require(runtime.endpoint().trySubmit(
                    ngc::StartRequest{64, epoch})
                    == ngc::SubmitResult::Submitted,
                "frontend-loss start should fit");
        require(waitForRequest(runtime, 64).succeeded,
                "frontend-loss start should succeed");
        const auto accepted = waitForEvent(runtime);
        require(std::get<ngc::ChunkAccepted>(accepted).chunk
                    == chunk.id,
                "frontend-loss chunk should activate");
        const auto moving = waitForMotionProgress(runtime, 0.1);

        const auto stopped =
            ngc::stopExecutorAfterFrontendLoss(runtime);
        require(stopped.state == ngc::BackendState::Disabled,
                "frontend-loss shutdown should disable the executor");
        require(stopped.commanded.position.x
                    > moving.commanded.position.x
                    && stopped.commanded.position.x < 10.0,
                "frontend-loss shutdown should stop from the current "
                "state before the published target");
        require(std::abs(stopped.commanded.velocity.x) < 1e-12
                    && std::abs(stopped.commanded.acceleration.x)
                        < 1e-12,
                "frontend-loss shutdown should finish at rest");

        runtime.stop();
    }

    void testExternalRuntimeFakesTriggeredJointInput(
        const std::filesystem::path &peer) {
        ngc::ExternalExecutorRuntime runtime(configuration(peer));
        runtime.start();

        constexpr ngc::EpochId epoch = 19;
        require(runtime.endpoint().trySubmit(ngc::ResetRequest{51, epoch})
                    == ngc::SubmitResult::Submitted,
                "triggered-joint IPC reset should fit");
        require(waitForRequest(runtime, 51).succeeded,
                "triggered-joint IPC reset should succeed");
        require(runtime.endpoint().trySubmit(ngc::EnableRequest{52})
                    == ngc::SubmitResult::Submitted,
                "triggered-joint IPC enable should fit");
        require(waitForRequest(runtime, 52).succeeded,
                "triggered-joint IPC enable should succeed");

        ngc::TriggeredJointMove move;
        move.epoch = epoch;
        move.id = 53;
        move.branch = 54;
        move.moveId = 55;
        move.joints = ngc::JointMask{1};
        move.targetMode = ngc::JointTargetMode::Relative;
        move.target[0] = 2.0;
        move.limits.velocity[0] = 2.0;
        move.limits.acceleration[0] = 5.0;
        move.limits.jerk[0] = 100.0;
        require(move.triggers.push({
                    0, 1, ngc::InputCondition::Active,
                }),
                "triggered-joint IPC fixture should fit its input");
        move.triggerRequired = true;

        require(runtime.endpoint().tryPublish(move)
                    == ngc::PublishResult::Published,
                "triggered-joint IPC move should fit");
        require(runtime.endpoint().trySubmit(ngc::StartRequest{56, epoch})
                    == ngc::SubmitResult::Submitted,
                "triggered-joint IPC start should fit");
        require(waitForRequest(runtime, 56).succeeded,
                "triggered-joint IPC start should succeed");

        std::optional<ngc::TriggeredJointMoveCompleted> completed;
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!completed.has_value()
               && std::chrono::steady_clock::now() < deadline) {
            ngc::ExecutionSnapshot snapshot;
            while (runtime.endpoint().tryTakeSnapshot(snapshot)) { }
            ngc::ExecutionEvent event;
            while (runtime.endpoint().tryTakeEvent(event)) {
                if (const auto *result =
                        std::get_if<ngc::TriggeredJointMoveCompleted>(&event)) {
                    completed = *result;
                }
            }
            std::this_thread::sleep_for(1ms);
        }

        require(completed.has_value(),
                "temporary IPC joint input should complete the triggered move");
        require(completed->status == ngc::TriggeredMoveStatus::Triggered
                    && completed->triggeredJoints == ngc::JointMask{1},
                "temporary IPC joint input should report the selected joint");
        require(completed->triggerState.position[0] >= 0.5
                    && completed->stoppedState.position[0]
                        < move.target[0],
                "temporary IPC joint input should trigger 0.5 units after "
                "the relative move starts and stop short of its target");

        runtime.stop();
    }

    void testConfiguredMachineSessionRunsThroughIpcExecutor(
        const std::filesystem::path &peer,
        const bool realtime) {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(),
                configuration ? "" : configuration.error());
        require(configuration->machineExecutor.has_value(),
                "IPC test machine configuration should enable Machine");
        configuration->machineExecutor->executable =
            std::filesystem::absolute(peer).lexically_normal();
        auto ordinaryBackendConfiguration =
            std::optional<std::filesystem::path>{};
        if (!realtime) {
            ordinaryBackendConfiguration =
                std::filesystem::temp_directory_path()
                / "ngc-ipc-test-peer-ordinary.toml";
            std::ofstream file(
                *ordinaryBackendConfiguration,
                std::ios::binary | std::ios::trunc);
            file << "[runtime]\nlock_memory = false\n";
            require(
                static_cast<bool>(file),
                "could not write the ordinary IPC test-peer "
                "configuration");
            configuration->machineExecutor->backendConfiguration =
                *ordinaryBackendConfiguration;
        } else {
            configuration->machineExecutor->backendConfiguration =
                std::filesystem::absolute(
                    "ipc_test_peer.toml").lexically_normal();
        }

        ngc::MachineSessionManager manager(*configuration);
        const auto initial = manager.state();
        require(initial.simulationAvailable && initial.machineAvailable,
                "configured IPC test peer should expose Simulation and Machine");
        auto machineAuthority =
            manager.selectControlTarget(ngc::MachineControlTarget::Machine);
        require(machineAuthority.has_value(),
                machineAuthority ? "" : machineAuthority.error());
        const auto temporaryToolStore =
            std::filesystem::temp_directory_path()
            / "ngc_ipc_real_session_tools.txt";
        const auto temporaryParameterStore =
            std::filesystem::temp_directory_path()
            / "ngc_ipc_real_session_parameters.var";
        std::error_code cleanupError;
        std::filesystem::remove(temporaryToolStore, cleanupError);
        cleanupError.clear();
        std::filesystem::remove(temporaryParameterStore, cleanupError);
        require(manager.setToolTableStorePath(
                    *machineAuthority, temporaryToolStore).has_value()
                    && manager.setPersistentParameterStorePath(
                        *machineAuthority,
                        temporaryParameterStore).has_value(),
                "configured IPC Machine test should isolate persistent stores");
        require(manager.powerOn(*machineAuthority),
                "configured IPC Machine session should power on");
        const auto powered = manager.snapshot();
        require(powered.powerState == ngc::MachinePowerState::On
                    && !powered.simulationDiagnostics.has_value(),
                "IPC Machine session should be powered without mock diagnostics");

        require(manager.home(*machineAuthority),
                "configured IPC Machine session should accept homing");
        auto homed = manager.snapshot();
        for (auto attempt = 0; attempt < 60'000
             && homed.status != ngc::SimulationStatus::Completed
             && homed.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            homed = manager.snapshot();
        }
        require(homed.status == ngc::SimulationStatus::Completed,
                homed.error.empty()
                    ? "configured IPC Machine homing should complete"
                    : homed.error);
        require(static_cast<std::size_t>(std::popcount(homed.homedJoints))
                    == configuration->joints.size(),
                "configured IPC Machine homing should mark every joint homed");

        const auto xAxis = std::ranges::find(
            configuration->axes, ngc::Machine::Axis::X,
            &ngc::AxisConfiguration::axis);
        require(xAxis != configuration->axes.end(),
                "configured IPC Machine jog should find X");
        const auto xJoint = std::ranges::find(
            configuration->joints, xAxis->joints.front(),
            &ngc::JointConfiguration::id);
        require(xJoint != configuration->joints.end(),
                "configured IPC Machine jog should find the X joint");
        const auto jogLeaseTicks = static_cast<std::uint32_t>(std::ceil(
            configuration->pendant.velocity.leaseDuration
            / configuration->simulation.servoPeriod));
        constexpr ngc::JogId jog = 700;
        const ngc::StartContinuousJogRequest jogRequest {
            .id = 701,
            .jog = jog,
            .target = {
                ngc::JogTargetType::JointGroup,
                ngc::AxisId::X,
                static_cast<ngc::JointMask>(
                    ngc::JointMask {1} << xJoint->id),
            },
            .signedVelocity = -0.5,
            .limits = {
                xAxis->maxVelocity,
                std::min(xAxis->maxAcceleration,
                         configuration->jogging.acceleration),
                std::min(xJoint->maxJerk, configuration->jogging.jerk),
            },
            .stopLimits = {
                xAxis->maxVelocity,
                xAxis->maxAcceleration,
                xJoint->maxJerk,
            },
            .travel = {
                xAxis->minimum,
                xAxis->maximum,
                true,
            },
            .leaseTicks = jogLeaseTicks,
        };
        require(manager.startJog(
                    *machineAuthority, ngc::ControlRequest {jogRequest}),
                "configured IPC Machine session should accept a post-home jog");
        for (ngc::RequestId request = 702; request < 722; ++request) {
            std::this_thread::sleep_for(5ms);
            if (!manager.renewJog(*machineAuthority, request, jog)) {
                const auto failed = manager.snapshot();
                require(false, std::format(
                    "configured IPC Machine session should queue post-home jog "
                    "renewal {}, status={}, jogging={}, reason={}, error={}",
                    request, static_cast<int>(failed.status), failed.jogging,
                    failed.lastJogStopReason.has_value()
                        ? static_cast<int>(*failed.lastJogStopReason) : -1,
                    failed.error));
            }
        }
        require(manager.stopJog(*machineAuthority, 722, jog),
                "configured IPC Machine session should accept the post-home jog stop");
        auto jogged = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && jogged.jogging
             && jogged.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            jogged = manager.snapshot();
        }
        require(jogged.status != ngc::SimulationStatus::Error,
                jogged.error.empty()
                    ? "configured IPC Machine post-home jog should not fail"
                    : jogged.error);
        require(jogged.lastJogStopReason
                    == ngc::JogStopReason::RequestedStop,
                "configured IPC Machine post-home jog should remain leased until stopped");
        require(jogged.machinePosition.x < homed.machinePosition.x,
                "configured IPC Machine post-home jog should move X");

        auto simulationAuthority =
            manager.simulateFromMachine(*machineAuthority);
        require(simulationAuthority.has_value(),
                simulationAuthority
                    ? ""
                    : std::format(
                        "configured IPC Machine homing checkpoint failed: {}",
                        simulationAuthority.error()));
        require(manager.powerOff(*simulationAuthority),
                "homing-derived Simulation should power off");
        auto simulationOff = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && simulationOff.powerState != ngc::MachinePowerState::Off
             && simulationOff.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            simulationOff = manager.snapshot();
        }
        require(simulationOff.powerState == ngc::MachinePowerState::Off,
                simulationOff.error.empty()
                    ? "homing-derived Simulation should finish powering off"
                    : simulationOff.error);
        machineAuthority =
            manager.selectControlTarget(ngc::MachineControlTarget::Machine);
        require(machineAuthority.has_value(),
                machineAuthority ? "" : machineAuthority.error());

        require(manager.start(
                    *machineAuthority,
                    {{"G1 F120 X10\n", "ipc-machine-feed-hold.ngc"}},
                    {}),
                "configured IPC Machine session should accept feed-hold motion");
        auto moving = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && moving.trajectoryBackendVelocity <= 0.01
             && moving.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            moving = manager.snapshot();
        }
        require(moving.status != ngc::SimulationStatus::Error
                    && moving.trajectoryBackendVelocity > 0.01,
                moving.error.empty()
                    ? "configured IPC Machine motion should begin before Feed Hold"
                    : moving.error);
        require(manager.feedHold(*machineAuthority),
                "configured IPC Machine motion should accept Feed Hold");
        auto feedHeld = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && feedHeld.status != ngc::SimulationStatus::Paused
             && feedHeld.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            feedHeld = manager.snapshot();
        }
        require(feedHeld.status == ngc::SimulationStatus::Paused,
                feedHeld.error.empty()
                    ? "configured IPC Machine Feed Hold should reach Paused"
                    : feedHeld.error);
        require(manager.stop(*machineAuthority),
                "configured IPC Machine Feed Hold should accept Stop");
        auto stopped = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && stopped.status != ngc::SimulationStatus::Stopped
             && stopped.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            stopped = manager.snapshot();
        }
        require(stopped.status == ngc::SimulationStatus::Stopped,
                stopped.error.empty()
                    ? "configured IPC Machine controlled Stop should complete"
                    : stopped.error);
        require(stopped.realtimeTiming.has_value()
                    && stopped.realtimeTiming->sampleCount != 0,
                "configured IPC Machine snapshot should expose executor timing");
        simulationAuthority = manager.simulateFromMachine(*machineAuthority);
        require(simulationAuthority.has_value(),
                simulationAuthority
                    ? ""
                    : std::format(
                        "configured IPC Machine stopped checkpoint failed: {}",
                        simulationAuthority.error()));
        require(manager.powerOff(*simulationAuthority),
                "stopped checkpoint Simulation should power off");
        simulationOff = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && simulationOff.powerState != ngc::MachinePowerState::Off
             && simulationOff.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            simulationOff = manager.snapshot();
        }
        require(simulationOff.powerState == ngc::MachinePowerState::Off,
                simulationOff.error.empty()
                    ? "stopped checkpoint Simulation should finish powering off"
                    : simulationOff.error);
        machineAuthority =
            manager.selectControlTarget(ngc::MachineControlTarget::Machine);
        require(machineAuthority.has_value(),
                machineAuthority ? "" : machineAuthority.error());

        require(manager.start(
                    *machineAuthority,
                    {{"G0 X0.01\n", "ipc-machine-session.ngc"}},
                    {}),
                "configured IPC Machine session should accept a program");
        auto completed = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && completed.status != ngc::SimulationStatus::Completed
             && completed.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            completed = manager.snapshot();
        }
        require(completed.status == ngc::SimulationStatus::Completed,
                completed.error.empty()
                    ? "configured IPC Machine program should complete"
                    : completed.error);
        require(std::abs(completed.machinePosition.x - 0.01) < 1e-9,
                std::format(
                    "configured IPC Machine program should report its executed "
                    "position, got {}", completed.machinePosition.x));

        require(manager.start(
                    *machineAuthority,
                    {{"G38.3 F60 X2\n", "ipc-machine-probe.ngc"}},
                    {}),
                "configured IPC Machine session should accept a probe");
        completed = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && completed.status != ngc::SimulationStatus::Completed
             && completed.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            completed = manager.snapshot();
        }
        require(completed.status == ngc::SimulationStatus::Completed,
                completed.error.empty()
                    ? "configured IPC Machine probe should complete"
                    : completed.error);
        require(completed.machinePosition.x >= 1.5
                    && completed.machinePosition.x < 2.0,
                std::format(
                    "temporary IPC probe input should trigger 0.5 inches "
                    "before the target and stop short of it, got {}",
                    completed.machinePosition.x));

        constexpr std::string_view toolChange = R"NGC(
sub _tool_change[#tool_number] {
    print["tool change: probing tool ", #tool_number]
    G90
    G49
    G20
    G53 G0 Z#5163
    alert["Install tool ", #tool_number, ", then press Resume"]
    M0
    return 1
}
)NGC";
        ngc::ToolTable tools;
        tools.set(2, {
            .number = 2,
            .x = 0.0,
            .y = 0.0,
            .z = 2.0,
            .a = 0.0,
            .b = 0.0,
            .c = 0.0,
            .diameter = 0.25,
            .comment = "IPC tool-change fixture",
        });
        require(manager.setToolTable(*machineAuthority, tools),
                "configured IPC Machine session should accept its tool table");
        require(manager.start(
                    *machineAuthority,
                    {
                        {std::string(toolChange), "autoload/tool_change.ngc"},
                        {"T2 M6\n", "ipc-machine-tool-change.ngc"},
                    },
                    tools, true),
                "configured IPC Machine session should accept a tool change");
        auto paused = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && paused.status != ngc::SimulationStatus::Paused
             && paused.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            paused = manager.snapshot();
        }
        require(paused.status == ngc::SimulationStatus::Paused,
                paused.error.empty()
                    ? std::format(
                        "configured IPC Machine tool change should reach M0; "
                        "operation {} driver '{}'",
                        static_cast<int>(paused.programOperation),
                        paused.trajectoryDriverActivity)
                    : paused.error);
        require(paused.operatorAlert
                    == "Install tool 2, then press Resume",
                "configured IPC Machine tool change should publish its operator alert");
        require(manager.resume(*machineAuthority),
                "configured IPC Machine tool change should accept Resume");
        completed = manager.snapshot();
        for (auto attempt = 0; attempt < 10'000
             && completed.status != ngc::SimulationStatus::Completed
             && completed.status != ngc::SimulationStatus::Error; ++attempt) {
            std::this_thread::sleep_for(1ms);
            completed = manager.snapshot();
        }
        require(completed.status == ngc::SimulationStatus::Completed,
                completed.error.empty()
                    ? "resumed IPC Machine tool change should complete"
                    : completed.error);
        require(manager.powerOff(*machineAuthority),
                "configured IPC Machine session should power off");
        manager.join();
        cleanupError.clear();
        std::filesystem::remove(temporaryToolStore, cleanupError);
        cleanupError.clear();
        std::filesystem::remove(temporaryParameterStore, cleanupError);
        if (ordinaryBackendConfiguration.has_value()) {
            cleanupError.clear();
            std::filesystem::remove(
                *ordinaryBackendConfiguration, cleanupError);
        }
    }

    void testExternalRuntimeRejectsStaleHandshake(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        ++options.peerExpectedIdentity.authorityGeneration;
        ngc::ExternalExecutorRuntime runtime(std::move(options));

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

    void testExternalRuntimeRejectsConfigurationMismatch(
        const std::filesystem::path &peer) {
        const auto alternateMachine =
            std::filesystem::temp_directory_path()
            / "ngc-ipc-alternate-machine.toml";
        std::error_code error;
        std::filesystem::copy_file(
            "machine.toml", alternateMachine,
            std::filesystem::copy_options::overwrite_existing,
            error);
        require(!error,
                "could not create alternate IPC machine configuration");
        {
            std::ofstream file(
                alternateMachine,
                std::ios::binary | std::ios::app);
            file << "\n# fingerprint mismatch\n";
            require(static_cast<bool>(file),
                    "could not modify alternate IPC machine configuration");
        }

        auto options = configuration(peer);
        options.peerArguments = {
            "--machine-configuration",
            alternateMachine.string(),
            "--non-realtime",
        };
        ngc::ExternalExecutorRuntime runtime(std::move(options));

        auto rejected = false;
        try {
            runtime.start();
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        require(rejected,
                "different executor configuration should fail IPC startup");
        require(runtime.lastRejection()
                    == ngc::IpcRejection::ConfigurationFingerprint,
                "configuration mismatch should retain its rejection reason");

        error.clear();
        std::filesystem::remove(alternateMachine, error);
    }

    void testExternalRuntimeRejectsInvalidExecutorConfiguration(
        const std::filesystem::path &peer) {
        auto options = configuration(peer);
        options.peerArguments = {
            "--machine-configuration",
            std::filesystem::absolute(
                "missing_ipc_machine_configuration.toml").string(),
        };
        ngc::ExternalExecutorRuntime runtime(std::move(options));

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
        ngc::ExternalExecutorRuntime runtime(std::move(options));
        runtime.start();

        require(runtime.endpoint().tryPublish(ngc::PlanChunk{})
                    == ngc::PublishResult::Invalid,
                "external runtime accepted an invalid execution item");

        for (std::uint64_t index = 0;
             index < ngc::IPC_EXECUTION_CAPACITY; ++index) {
            const auto chunk = linearChunk(
                1, index + 1, 0.0, 0.01, 0.01);
            require(runtime.endpoint().tryPublish(chunk)
                        == ngc::PublishResult::Published,
                    "non-consuming IPC peer should expose every bounded slot");
        }
        const auto overflow =
            linearChunk(1, 100, 0.0, 0.01, 0.01);
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
        ngc::ExternalExecutorRuntime runtime(std::move(options));
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
        require(
            argc == 2
                || (argc == 3
                    && (std::string_view(argv[2]) == "--realtime"
                        || std::string_view(argv[2])
                            == "--simulated-udp-only")),
            "ngc_ipc_tests requires the peer executable path and optional --realtime or --simulated-udp-only");
        const std::filesystem::path peer = argv[1];
        if (argc == 3
            && std::string_view(argv[2])
                == "--simulated-udp-only") {
            testExternalRuntimeSimulatesUdpExchange(peer);
            std::cout << "ngc_ipc_tests simulated UDP passed\n";

            return 0;
        }
        const auto realtime = argc == 3;
        testProtocolLayoutAndBoundedRings();
        testExternalRuntimeExecutesThroughProductionCore(peer);
        testExternalRuntimeSimulatesUdpExchange(peer);
        testExternalRuntimeFeedHoldAndResume(peer);
        testFrontendLossShutdownStopsAndDisables(peer);
        testExternalRuntimeFakesTriggeredJointInput(peer);
        testConfiguredMachineSessionRunsThroughIpcExecutor(
            peer, realtime);
        testExternalRuntimeRejectsStaleHandshake(peer);
        testExternalRuntimeRejectsConfigurationMismatch(peer);
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
