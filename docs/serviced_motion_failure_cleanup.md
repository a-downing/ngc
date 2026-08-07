# Serviced motion must retain ownership until the backend is quiescent

Status: implemented.

The implementation is `ServicedMotionOperation`, shared by jogging and homing.
It owns their common bounded service loop, demand publication, snapshot and
event draining, explicit post-start cleanup, final observation, and
runtime-stop/session-fault fallback. `MachineSession` retains a separate
serviced-motion ownership barrier until the scope returns a backend-observed
terminal result. Regression coverage injects backpressure into both jog update
and token-matched jog-stop submission, rejects cleanup Stop demand, and covers
a post-start homing failure.

## Problem statement

A live continuous jog can outlive its frontend controller after an ordinary backend control submission fails. The controller reports an error and the session manager publishes an idle frontend state even though the executor can still be moving.

This is a lifecycle and motion-ownership defect, not a configuration problem. It is reachable with valid machine configuration because the ordered backend control channel is intentionally bounded and `trySubmit()` is allowed to return `SubmitResult::Full`.

The executor's jog dead-man lease eventually initiates a constrained stop, so the residual motion is bounded. The configured lease is currently 0.1 seconds. That fallback does not make it correct to release ownership before the stop completes: another operation can be admitted while the old jog is still active or braking.

## Evidence and failure path

1. After a jog has started, [`JoggingController::run`](../src/JoggingController.cpp#L105) consumes updates and submits them to the backend.
2. If an update cannot be submitted, it immediately returns `"motion backend jog control channel is full"` at [`JoggingController.cpp`](../src/JoggingController.cpp#L107).
3. The shutdown-generated `StopJogRequest` has the same immediate-return behavior when submission fails at [`JoggingController.cpp`](../src/JoggingController.cpp#L112).
4. [`MachineSession::runJogging`](../src/MachineSession.cpp#L851) synchronizes the canonical position only on success, then unconditionally calls `finishActivity()` at line 856.
5. The session host then unconditionally clears `m_running`, `m_activeJog`, the jogging presentation state, and `hasActiveMotion` at [`MachineSessionManager.h`](../src/include/machine/MachineSessionManager.h#L1581), including when `runJogging()` returned an error.
6. [`motionOwnedOrQueued()`](../src/include/machine/MachineSessionManager.h#L316) relies on those frontend flags, the command queue, coordinator activity, and `m_activeJog`. Once they are cleared, it can report no owner while the backend still owns the jog.
7. The executor retains `m_jog`. For a continuous jog it counts down the lease and begins a constrained stop only when the lease expires at [`ProductionExecutorCore.cpp`](../src/ProductionExecutorCore.cpp#L2501).

The bounded failure is part of the backend contract, not merely theoretical defensive code. [`ProductionExecutorCore::trySubmit`](../src/ProductionExecutorCore.cpp#L390) returns `SubmitResult::Full` when its fixed control capacity or shared ingress queue is exhausted.

The dangerous state sequence is therefore:

```text
frontend owns jogging; backend is moving
    -> ordinary jog control submission returns Full
    -> JoggingController returns an error without quiescing the backend
    -> MachineSession releases MachineActivity::Jogging
    -> host clears all frontend motion ownership indicators
    -> a new operation may be admitted
    -> old backend jog remains active until its lease expires and braking completes
```

## Required invariant

Starting a serviced motion operation transfers motion ownership to an operation scope. That scope must retain ownership until one of these backend-observed terminal conditions is established:

- the requested operation completes and the backend is stationary;
- a constrained stop completes and the backend is stationary;
- the backend becomes faulted or disabled and safe output behavior has been established.

A local error is not a terminal motion state. Frontend activity flags must describe backend-observed state, not the lifetime of a controller function call.

The jog dead-man lease remains an authoritative fallback, but it must not be the normal cleanup mechanism for a failed NRT submission.

## Implemented abstraction

The NRT-only operation scope for motion driven by service callbacks is named
`ServicedMotionOperation`. It owns:

- the execution epoch and persistent lifecycle demand;
- immediate service, one-period advancement, and waiting callbacks;
- event and latest-snapshot draining;
- bounded service-loop policy;
- backend fault and terminal-state recognition;
- stop publication and acknowledgement;
- final stationary observation used to synchronize canonical position;
- ownership release only after terminal convergence.

A conceptual interface is:

```cpp
class ServicedMotionOperation {
public:
    std::expected<void, std::string> begin(
        ExecutionEpoch epoch, ExecutorDemandMode mode);

    std::expected<void, std::string> submit(const ControlRequest &request);

    template<typename TerminalPredicate>
    std::expected<ExecutionEvent, std::string> serviceUntil(
        TerminalPredicate terminal);

    std::expected<ExecutionSnapshot, std::string> stopAndQuiesce();

    template<typename Result>
    std::expected<Result, std::string> finish(
        std::expected<Result, std::string> operationResult);
};
```

The exact public shape should follow the existing backend and callback types. The important abstraction boundary is that every post-start exit, including an error exit, passes through `finish()` or equivalent terminal convergence logic.

This should not be destructor-only RAII. Constrained braking is asynchronous and requires active servicing, so a destructor cannot perform the normal cleanup safely or report failure. A destructor may assert that the scope has reached a terminal state or invoke an emergency fallback, but explicit finalization should perform the ordinary stop-and-wait protocol.

## Cleanup behavior

For any failure after motion has started, the operation scope should:

1. Preserve the original operation error.
2. Publish a generation-tagged lifecycle demand with `ExecutorDemandMode::Stop`.
3. Continue servicing the runtime and consuming events/snapshots until the relevant stop/completion event and a stationary terminal state are observed.
4. Synchronize the session's canonical position from the final backend observation.
5. Only then release `MachineActivity`, clear the active jog, and publish idle/no-active-motion presentation state.
6. Return the original error, augmented with cleanup failure details if cleanup also failed.

The lifecycle-demand mailbox is a suitable primary cleanup route because it is separate from the ordered control ingress whose backpressure caused the failure. The normal token-matched `StopJogRequest` remains appropriate for ordinary operator stops. A lifecycle Stop can cover failures in that ordinary control path; the executor's demand-stop handling already includes active jog motion.

If Stop cannot be published or terminal convergence cannot be established within the bounded policy, the final fallback should stop the `BackendRuntime` and fault the session. Runtime shutdown is already required to establish safe outputs. The implementation must distinguish an already faulted/disabled backend from loss of the external process and preserve the strongest available diagnostic.

## Code reduction opportunity

Jogging and homing currently duplicate the mechanics that belong in this operation scope:

- immediate servicing and service-period advancement;
- service-clock waiting;
- event and snapshot draining;
- large bounded guard loops;
- backend-fault checks;
- lifecycle-demand publication;
- stop acknowledgement;
- final observation capture.

Moving those mechanics into `ServicedMotionOperation` should make `JoggingController` and `HomingController` describe their operation-specific state machines only. It also prevents each controller from inventing a different error-exit policy.

Program execution has a more developed controlled-stop state machine and can be used as a behavioral reference. It should not be folded into the new abstraction during the first fix unless doing so is clearly mechanical; jogging is the urgent path and homing is the closest second consumer.

## Defense in depth

The session host should expose backend-owned operation state independently of presentation fields. Until terminal convergence, `motionOwnedOrQueued()` must continue to return true even if an operation has recorded an error.

Do not set any of the following merely because `runJogging()` returned:

- `m_running = false`;
- `m_activeJog.reset()`;
- coordinator activity to idle;
- `m_snapshot.hasActiveMotion = false`.

Those transitions should occur from the operation scope's backend-observed terminal result. This secondary guard protects admission even if a future controller error path is incomplete.

The abstraction must remain NRT. It must preserve the existing single-NRT-owner rule for plan publication, demand publication, and transactional control submission, and it must not add allocation or unbounded data to the executor's real-time path.

## Implementation coverage

1. The shared operation scope owns terminal convergence and cleanup-error composition.
2. Jogging start, normal completion, submission failure, operator stop, and shutdown pass through the scope.
3. Session and host cleanup release ownership only after a terminal operation result carrying the final backend observation.
4. `motionOwnedOrQueued()` includes the session's independent serviced-motion ownership barrier.
5. Homing's post-start exits use the same stop-and-quiesce mechanism.
6. Program execution remains separate because it already has its own controlled-stop state machine.

The shared implementation is located at:

```text
src/include/machine/ServicedMotionOperation.h
src/ServicedMotionOperation.cpp
```

## Regression tests

Add a controllable fake or injected `MotionBackend` that can return `SubmitResult::Full` after an active jog has been established. Cover both an update submission and a stop-control submission.

Each test should prove:

- the jog is active before the injected failure;
- the original submission error is retained;
- cleanup publishes lifecycle Stop through the separate demand path;
- runtime servicing continues until `JogStopped`, faulted, or another valid stationary terminal state;
- the manager retains motion ownership and rejects a new program, homing operation, or jog while braking is in progress;
- the final canonical and presentation positions come from the backend observation;
- activity becomes idle only after terminal convergence;
- a cleanup failure faults/stops the runtime and is included in diagnostics;
- the dead-man lease still acts as fallback, but successful explicit cleanup does not wait for lease expiry.

Add corresponding homing tests once homing uses the shared scope, especially for a post-start Stop-demand publication failure and bounded service-loop exhaustion.

## Acceptance criteria

- No post-start return from jogging or homing can release session ownership while the backend may still be moving.
- A full ordered control channel results in a constrained stop or a faulted/stopped runtime before another operation is admitted.
- All success and failure paths synchronize final position from a backend observation when one is available.
- Ordinary operator jog controls remain token matched.
- Emergency stop behavior remains out of band and unchanged.
- The dead-man lease remains enabled and tested as defense in depth.
- Shared service and cleanup mechanics are removed from the individual controllers rather than copied into another helper per controller.
- Existing executor fixed-capacity and real-time constraints remain intact.

## Validation

Run from the repository root:

```powershell
cmake --build build
build\ngc_tests.exe
ctest --test-dir build -E "^ngc_tests$" --output-on-failure
```

Running `ngc_tests.exe` from the repository root matters because the core suite uses repository-relative fixtures. The existing suite passed from that location during the investigation, but it did not exercise this injected post-start backpressure path.

## Out of scope

- Adding a heartbeat or liveness lease for a live-but-unresponsive external frontend.
- Treating intentionally unsafe or invalid operator configuration as the root cause.
- Replacing the executor's bounded queues with unbounded or blocking channels.
- Changing emergency-stop semantics.
