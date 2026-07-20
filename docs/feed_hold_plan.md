# Limited Mock-Backend Feed Hold

## Status and scope

This branch implements the first experimental feed-hold path for
`MockMotionBackend` and timed Simulation. It replaces the old simulated pause,
which froze executor ticks at a potentially nonzero velocity and acceleration,
with backend-owned braking that starts on the next servo tick.

The implementation is intentionally one-way for its first hardware-style trial:

```text
SimulationStatus::Running / BackendState::Running
    -> Feed Hold request accepted by the backend
    -> SimulationStatus::Holding / BackendState::Holding
    -> execution rate and rate acceleration reach zero
    -> BackendHeld(BackendHoldReason::FeedHold)
    -> SimulationStatus::Paused / BackendState::Held
```

Resume is not implemented. After reaching `Paused`, use Stop to end the run.
This is deliberate so feed-hold braking can be validated before adding a proved
resume path.

Read `AGENTS.md` before extending this work. Its backend, streaming, stop-tail,
terminology, and safety constraints remain authoritative. The implementation is
mock-only and is not yet a production real-time safety guarantee.

## Implemented behavior

- The GUI exposes a `Feed Hold` button while a timed simulation is Running.
- The GUI submits the request to `SimulationWorker`; it never writes directly
  to the backend control channel.
- `SimulationWorker` continues ticking the executor throughout Holding and
  Paused. It no longer implements pause by suspending executor calls.
- The backend accepts `FeedHoldRequest`, reports `BackendState::Holding`, and
  begins changing the executed command on the next mock servo tick.
- Braking remains on the precomputed trajectory by slowing reference time. It
  does not independently stop axes or reset the active execution span.
- Retiming state is preserved across execution spans and packet continuations.
- The backend reports `BackendHeld` with reason `FeedHold` only after execution
  rate and execution-rate acceleration are zero. The worker then reports
  `SimulationStatus::Paused`.
- Stop remains available during Running, Holding, and Paused.
- Existing exact-stop holds use `BackendHoldReason::StopBranch`, keeping them
  distinct from an operator feed hold. The trajectory driver does not apply its
  exact-stop auto-resume or rolling-stop error logic to a feed-hold event.
- If normal motion reaches a validated packet branch before braking completes,
  the existing branch machinery may select the packet's precomputed stop tail.
  A stop tail is never entered from an arbitrary point in a normal span.

## Retiming model

Let `tau` be reference time in the precomputed execution spans and `t` be
physical servo time. The backend maintains:

```text
rate              = d(tau) / dt
rate_acceleration = d(rate) / dt
rate_jerk         = d(rate_acceleration) / dt
```

Normal execution uses rate 1, rate acceleration 0, and rate jerk 0. During feed
hold, the backend integrates a scalar S-curve-like braking command and advances
reference time by the integrated rate instead of by the full servo period.
Rate is constrained to the interval from zero to one and cannot reverse the
reference trajectory.

For a reference execution-span polynomial `x(tau)`, the executed derivatives
are calculated analytically per XYZABC component:

```text
executed velocity = reference velocity * rate

executed acceleration = reference acceleration * rate^2
                      + reference velocity * rate_acceleration

executed jerk = reference jerk * rate^3
              + 3 * reference acceleration * rate * rate_acceleration
              + reference velocity * rate_jerk
```

The backend uses the exact cubic-span derivatives. It does not finite-difference
position to produce the commanded velocity, acceleration, or jerk.

## Limits and deliberate safety boundary

Typed configuration is loaded once through `MachineConfiguration`:

```toml
[feed_hold]
# Requested tangential braking acceleration, machine-units / second^2.
tangential_acceleration = 5.0
# Requested change in tangential braking acceleration, machine-units / second^3.
tangential_jerk = 25.0
```

Both values must be finite and positive. The mock backend receives typed values;
TOML parsing does not enter the RT-facing backend.

On each holding tick, the backend derives a feasible interval for
`rate_acceleration`. It intersects the per-axis acceleration bounds with the
aggregate path-acceleration bound, then limits the requested tangential braking
command to that interval. The configured feed-hold jerk controls the requested
change in tangential braking acceleration.

This initial implementation does not prove either of the following during
retiming:

- the configured full coupled path-jerk limit; or
- the configured per-axis jerk limits.

Actual executed jerk is calculated and recorded. It must not be described as
constrained merely because the scalar tangential braking request is jerk
limited. Normal, unretimed program execution retains all existing planner
validation guarantees. Do not weaken those guarantees to accommodate this
experimental hold.

The current per-tick acceleration feasibility calculation uses the state at the
tick boundary. Exact or conservative extrema over the complete tick, especially
on curved spans and span-crossing ticks, remain future hardening work before a
physical backend can claim the same behavior.

## Stop-tail behavior

Every continuous-motion packet already has a validated branch state and a
precomputed stop tail. The branch gate proves position, velocity, and
acceleration continuity between normal motion and the tail, and proves that the
tail ends stationary.

Feed-hold retiming carries the same scalar rate state across that branch. If a
normal continuation is available, normal motion continues while its reference
clock slows. If the normal queue runs out at a branch while rate remains
nonzero, existing branch selection enters the stop tail and continues braking.
If the tail reaches its stationary end, the resulting hold is reported as a
feed hold. Resume after entering a stop tail remains unsupported because the
old moving continuation is no longer a proved continuation.

The stop tail is a bounded fallback, not permission to jump to tail geometry,
discard an unpublished moving suffix, or resume an invalidated continuation.

## Mock telemetry

`MockTrajectoryDiagnostics` records one executed sample per mock servo tick.
The sample includes:

- physical and reference elapsed time;
- position, velocity, acceleration, and analytic executed jerk;
- aggregate jerk magnitude;
- execution rate, rate acceleration, and rate jerk; and
- whether feed hold is active and whether the sample is on a stop tail.

The retained diagnostic vector is mock-only NRT data. It is not part of
`MotionBackend` and no unbounded data crosses an RT-style channel. The GUI
currently displays execution rate and rate acceleration; plotting or exporting
the full samples is deferred.

## Regression coverage

Focused framework-free tests in `src/test.cpp` cover:

- a feed hold during a long linear backend span;
- the observable Holding state before Held;
- decreasing execution velocity and rate bounds;
- aggregate and per-axis acceleration limits for recorded samples;
- reaching rest before the programmed branch without entering a stop tail;
- a single `BackendHeld` event with `BackendHoldReason::FeedHold` and stationary
  velocity and acceleration; and
- the end-to-end `SimulationWorker` transition from Running through Holding to
  Paused, stopping before the programmed endpoint.

The configuration-loader regression also verifies positive feed-hold values.
The branch builds both `ngc_tests` and `imgui_main`. The focused spline CTest
targets pass. The complete `ngc_tests` executable reaches and passes the new
feed-hold regressions, then encounters the separately existing local
`1002_3d.ngc` fixture modification; that user change is intentionally excluded
from this branch's commit.

## Manual trial

1. Start a timed simulation with a move long enough to observe braking.
2. Click `Feed Hold` while the machine is moving.
3. Confirm the status changes to Holding immediately and then to Paused only
   after the displayed backend velocity and acceleration reach zero.
4. Confirm position remains before the programmed endpoint and does not drift
   after Paused.
5. Use Stop to end the held run. Resume is intentionally absent.

## Deferred work

- Validate the current braking profile against recorded servo samples on
  straight and curved trajectories.
- Add focused span-crossing, packet-continuation, stop-tail fallback, abort, and
  invalid/duplicate request regressions.
- Check exact or conservative acceleration extrema throughout every servo tick.
- Add explicit diagnostic excess fields for aggregate/per-axis acceleration and
  jerk limits.
- Implement and prove resume from a held normal-motion cursor without resetting
  span progress, replaying events, or violating activation ownership.
- Reject resume after stop-tail entry with a specific diagnostic until
  replanning or a proved rejoin exists.
- Add a bounded per-axis jerk feasibility solver and full coupled path-jerk
  enforcement.
- Add optional operator graphs or sample export.
- Design a production RT implementation and bounded telemetry transport.
- Define separate certified normal, feed-hold, abort, and emergency-stop limits.

## Branch hygiene

- Branch: `feature/backend-feed-hold`, created from `main`.
- `1002_3d.ngc` had an existing uncommitted user modification before this branch
  was created. Preserve it and do not include it in feed-hold commits.
