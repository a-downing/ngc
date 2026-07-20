#include <cmath>
#include <iostream>
#include <ranges>
#include <stdexcept>
#include <string_view>

#include "operator/OperatorJogController.h"
#include "operator/PendantTouchOffController.h"
#include "machine/MockMotionBackend.h"

namespace {
    constexpr double EPSILON = 1e-12;

    void require(const bool condition, const std::string_view message) {
        if(!condition) throw std::runtime_error(std::string(message));
    }

    ngc::JointMask allConfiguredJoints(const ngc::MachineConfiguration &configuration) {
        ngc::JointMask result = 0;
        for(const auto &joint : configuration.joints)
            result |= ngc::JointMask { 1 } << joint.id;
        return result;
    }

    void testPendantStepBuildsConfiguredIncrementalJog() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        ngc::operator_control::JogController controller(*configuration);
        ngc::SimulationSnapshot snapshot;
        snapshot.status = ngc::SimulationStatus::Stopped;
        snapshot.homedJoints = allConfiguredJoints(*configuration);

        controller.consume(ngc::pendant::JogWheel { ngc::pendant::Axis::X, 3 });
        const auto action = controller.next(snapshot);
        const auto *start = action
            ? std::get_if<ngc::StartIncrementalJogRequest>(&*action) : nullptr;
        require(start, "pendant wheel counts should produce an incremental jog request");
        require(start->target.type == ngc::JogTargetType::Axis && start->target.joints == 0,
                "a homed pendant selection should request logical-axis motion");
        require(std::abs(start->distance - configuration->pendant.step.fineDistance) < EPSILON,
                "one released wheel report should produce exactly one configured fine increment");
        const auto axis = std::ranges::find(
            configuration->axes, ngc::Machine::Axis::X, &ngc::AxisConfiguration::axis);
        require(axis != configuration->axes.end(), "operator test should find configured X axis");
        require(std::abs(start->velocity - axis->maxVelocity) < EPSILON
                && std::abs(start->limits.acceleration - axis->maxAcceleration) < EPSILON
                && std::abs(start->limits.jerk - axis->maxJerk) < EPSILON,
                "pendant step motion should use the selected machine axis's full limits");
        require(start->travel.enabled && start->travel.minimum == axis->minimum
                && start->travel.maximum == axis->maximum,
                "pendant jogging should retain configured soft limits");
        controller.submitted(*action, true);

        snapshot.jogging = true;
        snapshot.status = ngc::SimulationStatus::Running;
        snapshot.activity = ngc::SimulationActivity::Jogging;
        controller.consume(ngc::pendant::JogWheel {
            ngc::pendant::Axis::X, 2, ngc::pendant::JogIncrement::Coarse,
        });
        controller.consume(ngc::pendant::JogWheel {
            ngc::pendant::Axis::X, -2, ngc::pendant::JogIncrement::Coarse,
        });
        require(!controller.next(snapshot),
                "one follow-up Step increment must wait for the active jog to finish");
        snapshot.jogging = false;
        require(!controller.next(snapshot),
                "a follow-up Step increment must survive the pendant completion handoff");
        require(!controller.takeError(),
                "pendant-owned motion must not be reported as a non-idle machine");
        snapshot.status = ngc::SimulationStatus::Completed;
        snapshot.activity = ngc::SimulationActivity::Idle;
        const auto coarseAction = controller.next(snapshot);
        const auto *coarseStart = coarseAction
            ? std::get_if<ngc::StartIncrementalJogRequest>(&*coarseAction) : nullptr;
        require(coarseStart
                && std::abs(coarseStart->distance
                            - configuration->pendant.step.coarseDistance) < EPSILON,
                "Step mode should retain the first follow-up increment and discard later detents");
    }

    void testPendantCancellationStopsAndClearsQueuedMotion() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        ngc::operator_control::JogController controller(*configuration);
        ngc::SimulationSnapshot snapshot;
        snapshot.homedJoints = allConfiguredJoints(*configuration);

        controller.consume(ngc::pendant::JogWheel { ngc::pendant::Axis::Y, 1 });
        const auto start = controller.next(snapshot);
        require(start && std::holds_alternative<ngc::StartIncrementalJogRequest>(*start),
                "cancellation test should start a pendant jog");
        const auto jog = std::get<ngc::StartIncrementalJogRequest>(*start).jog;
        controller.submitted(*start, true);
        snapshot.jogging = true;
        snapshot.status = ngc::SimulationStatus::Running;
        snapshot.activity = ngc::SimulationActivity::Jogging;
        controller.consume(ngc::pendant::JogWheel { ngc::pendant::Axis::Y, 5 });
        controller.consume(ngc::pendant::CancelPendantActivity {
            ngc::pendant::CancelReason::SelectionChanged,
        });
        const auto stop = controller.next(snapshot);
        const auto *stopRequest = stop ? std::get_if<ngc::StopJogRequest>(&*stop) : nullptr;
        require(stopRequest && stopRequest->jog == jog,
                "selector cancellation should stop the exactly matching pendant jog token");
        controller.submitted(*stop, true);
        snapshot.jogging = false;
        snapshot.status = ngc::SimulationStatus::Completed;
        snapshot.activity = ngc::SimulationActivity::Idle;
        require(!controller.next(snapshot),
                "selector cancellation should discard wheel counts queued behind the active jog");
    }

    void testPendantRejectsUnhomedOrBusyDelayedMotion() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        ngc::operator_control::JogController controller(*configuration);
        ngc::SimulationSnapshot snapshot;
        controller.consume(ngc::pendant::JogWheel { ngc::pendant::Axis::Z, 1 });
        require(!controller.next(snapshot), "pendant should not jog an unhomed axis");
        auto error = controller.takeError();
        require(error && error->find("unhomed") != std::string::npos,
                "unhomed pendant rejection should be diagnosable");

        snapshot.homedJoints = allConfiguredJoints(*configuration);
        snapshot.status = ngc::SimulationStatus::Running;
        snapshot.activity = ngc::SimulationActivity::Program;
        snapshot.jogging = false;
        controller.consume(ngc::pendant::JogWheel { ngc::pendant::Axis::Z, 1 });
        const auto busyAction = controller.next(snapshot);
        require(busyAction && std::holds_alternative<ngc::StartIncrementalJogRequest>(*busyAction),
                "the worker, not a stale presentation snapshot, should arbitrate pendant ownership");
        controller.submitted(*busyAction, false);
        error = controller.takeError();
        require(error && error->find("rejected") != std::string::npos,
                "busy pendant rejection should be diagnosable");
        snapshot.status = ngc::SimulationStatus::Completed;
        snapshot.activity = ngc::SimulationActivity::Idle;
        require(!controller.next(snapshot),
                "a rejected busy pendant step must never execute later when the machine becomes idle");
    }

    void testPendantStepExecutesAsConfiguredAxisMotion() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        ngc::operator_control::JogController controller(*configuration);
        ngc::SimulationSnapshot simulation;
        simulation.status = ngc::SimulationStatus::Stopped;
        simulation.homedJoints = allConfiguredJoints(*configuration);
        controller.consume(ngc::pendant::JogWheel { ngc::pendant::Axis::X, 4 });
        const auto action = controller.next(simulation);
        const auto *start = action
            ? std::get_if<ngc::StartIncrementalJogRequest>(&*action) : nullptr;
        require(start, "end-to-end pendant step should produce a start request");

        ngc::MockMotionBackend backend(configuration->axes, configuration->joints);
        require(backend.trySubmit(ngc::ResetRequest { 1, 1 }) == ngc::SubmitResult::Submitted,
                "configured mock backend should accept reset");
        backend.advance(0.0);
        require(backend.trySubmit(ngc::EnableRequest { 2 }) == ngc::SubmitResult::Submitted,
                "configured mock backend should accept enable");
        backend.advance(0.0);
        ngc::JointVector initial;
        ngc::JointMask joints = 0;
        for(const auto &joint : configuration->joints) {
            joints |= ngc::JointMask { 1 } << joint.id;
            initial[joint.id] = joint.axis == ngc::Machine::Axis::X
                ? 2.0 * joint.coordinateScale : 0.0;
        }
        require(backend.trySubmit(ngc::SetJointPositionRequest { 3, joints, initial })
                    == ngc::SubmitResult::Submitted,
                "configured mock backend should accept initial joint coordinates");
        backend.advance(0.0);
        ngc::ExecutionSnapshot discardedSnapshot;
        while(backend.tryTakeSnapshot(discardedSnapshot)) { }
        ngc::ExecutionEvent discardedEvent;
        while(backend.tryTakeEvent(discardedEvent)) { }

        require(backend.trySubmit(ngc::ControlRequest { *start }) == ngc::SubmitResult::Submitted,
                "configured mock backend should accept the pendant axis jog");
        backend.runUntilIdle(0.001);
        ngc::ExecutionSnapshot executed;
        bool haveSnapshot = false;
        while(backend.tryTakeSnapshot(executed)) haveSnapshot = true;
        require(haveSnapshot && executed.state == ngc::BackendState::Held,
                "pendant axis jog should finish in the held state");
        const auto expected = 2.0 + start->distance;
        require(std::abs(executed.commanded.position.x - expected) < EPSILON,
                "pendant axis jog should advance the logical X coordinate by its configured step");
        for(const auto &joint : configuration->joints) {
            if(joint.axis != ngc::Machine::Axis::X) continue;
            require(std::abs(executed.commandedJoints.position[joint.id]
                             - expected * joint.coordinateScale) < EPSILON,
                    "configured backend should map a logical-axis jog to each coupled joint");
        }
    }

    void testPendantVelocityUsesStableTokenUpdatesAndDeadManStop() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        ngc::operator_control::JogController controller(*configuration);
        ngc::SimulationSnapshot snapshot;
        snapshot.status = ngc::SimulationStatus::Stopped;
        snapshot.homedJoints = allConfiguredJoints(*configuration);

        controller.consume(ngc::pendant::JogVelocity { ngc::pendant::Axis::X, 100 });
        const auto startAction = controller.next(snapshot);
        const auto *start = startAction
            ? std::get_if<ngc::StartContinuousJogRequest>(&*startAction) : nullptr;
        require(start && start->signedVelocity > 0.0 && start->leaseTicks > 0,
                "nonzero pendant wheel rate should start a leased continuous jog");
        const auto jog = start->jog;
        controller.submitted(*startAction, true);

        snapshot.status = ngc::SimulationStatus::Running;
        snapshot.jogging = true;
        controller.consume(ngc::pendant::JogVelocity { ngc::pendant::Axis::X, 200 });
        const auto updateAction = controller.next(snapshot);
        const auto *update = updateAction
            ? std::get_if<ngc::SetContinuousJogVelocityRequest>(&*updateAction) : nullptr;
        require(update && update->jog == jog
                && std::abs(update->signedVelocity - 2.0 * start->signedVelocity) < EPSILON,
                "changed wheel rate should update velocity on the stable jog token");
        controller.submitted(*updateAction, true);

        controller.consume(ngc::pendant::JogVelocity { ngc::pendant::Axis::X, 200 });
        const auto renewAction = controller.next(snapshot);
        const auto *renew = renewAction
            ? std::get_if<ngc::RenewJogLeaseRequest>(&*renewAction) : nullptr;
        require(renew && renew->jog == jog,
                "an unchanged nonzero wheel rate should renew the active dead-man lease");
        controller.submitted(*renewAction, true);

        controller.consume(ngc::pendant::JogVelocity { ngc::pendant::Axis::X, 0 });
        const auto stopAction = controller.next(snapshot);
        const auto *stop = stopAction
            ? std::get_if<ngc::StopJogRequest>(&*stopAction) : nullptr;
        require(stop && stop->jog == jog,
                "zero pendant wheel rate should request a constrained stop immediately");

        ngc::MockMotionBackend backend(configuration->axes, configuration->joints);
        require(backend.trySubmit(ngc::ResetRequest { 1, 1 }) == ngc::SubmitResult::Submitted,
                "velocity backend regression should accept reset");
        backend.advance(0.0);
        require(backend.trySubmit(ngc::EnableRequest { 2 }) == ngc::SubmitResult::Submitted,
                "velocity backend regression should accept enable");
        backend.advance(0.0);
        auto backendStart = *start;
        backendStart.id = 3;
        backendStart.jog = 4;
        backendStart.leaseTicks = 2000;
        require(backend.trySubmit(ngc::ControlRequest { backendStart })
                    == ngc::SubmitResult::Submitted,
                "backend should accept the configured pendant velocity jog");
        backend.advance(0.0);
        ngc::ExecutionSnapshot executed;
        while(backend.tryTakeSnapshot(executed)) { }
        for(int tick = 0; tick < 300; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(executed)) { }
        }
        require(executed.commanded.velocity.x > 0.0,
                "pendant velocity jog should accelerate in the wheel direction");
        require(backend.trySubmit(ngc::SetContinuousJogVelocityRequest {
                    5, backendStart.jog, -backendStart.signedVelocity })
                    == ngc::SubmitResult::Submitted,
                "backend should accept a same-token pendant direction reversal");
        backend.advance(0.0);
        for(int tick = 0; tick < 500; ++tick) {
            backend.advanceTick(0.001, true);
            while(backend.tryTakeSnapshot(executed)) { }
        }
        require(executed.commanded.velocity.x < 0.0,
                "same-token pendant update should reverse through constrained motion");
        require(backend.trySubmit(ngc::StopJogRequest { 6, backendStart.jog })
                    == ngc::SubmitResult::Submitted,
                "backend should accept the matching pendant velocity stop");
        backend.runUntilIdle(0.001);
    }

    void testPendantTouchOffStagesTargetCoordinateUntilDoubleClick() {
        auto configuration = ngc::loadMachineConfiguration("machine.toml");
        require(configuration.has_value(), configuration ? "" : configuration.error());
        ngc::operator_control::TouchOffController controller(*configuration);
        controller.consume(ngc::pendant::SelectionChanged {
            ngc::pendant::Mode::Zero, ngc::pendant::Axis::X, false,
        });
        controller.consume(ngc::pendant::AdjustTouchOff {
            ngc::pendant::Axis::X, -1,
        });
        require(controller.snapshot().axis == ngc::pendant::Axis::X
                && std::abs(controller.snapshot().workPosition + 0.001) < EPSILON,
                "one negative Zero-mode count should stage the configured fine coordinate");
        controller.consume(ngc::pendant::CancelPendantActivity {
            ngc::pendant::CancelReason::ButtonReleased,
        });
        require(std::abs(controller.snapshot().workPosition + 0.001) < EPSILON,
                "button release must preserve the staged coordinate for double-click commit");
        controller.consume(ngc::pendant::CommitTouchOff { ngc::pendant::Axis::X });
        const auto commit = controller.takeCommit();
        require(commit && commit->axis == ngc::pendant::Axis::X
                && std::abs(commit->workPosition + 0.001) < EPSILON,
                "double click should publish the staged target work coordinate exactly once");
        require(!controller.takeCommit() && std::abs(controller.snapshot().workPosition) < EPSILON,
                "a committed touch-off should reset its staging value to zero");

        ngc::Machine machine(configuration->unit);
        machine.setActiveWorkOffset(ngc::Machine::Axis::X, 3.25);
        require(std::abs(machine.workOffset().x - 3.25) < EPSILON
                && std::abs(machine.memory().read(ngc::Var::G54_X) - 3.25) < EPSILON,
                "touch-off application should update both active WCS state and canonical memory");
    }
}

int main() {
    try {
        testPendantStepBuildsConfiguredIncrementalJog();
        testPendantCancellationStopsAndClearsQueuedMotion();
        testPendantRejectsUnhomedOrBusyDelayedMotion();
        testPendantStepExecutesAsConfiguredAxisMotion();
        testPendantVelocityUsesStableTokenUpdatesAndDeadManStop();
        testPendantTouchOffStagesTargetCoordinateUntilDoubleClick();
    } catch(const std::exception &error) {
        std::cerr << "ngc_operator_tests failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ngc_operator_tests passed\n";
}
