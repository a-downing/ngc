# Code audit findings

Audit date: 2026-08-07

## Executive summary

The highest-impact code defect is in the optional Huanyang spindle path: a
failed serial stop can be reported as a successful safe stop. The checked-in
configuration currently disables the spindle, so this is dormant until
commissioning.

The most serious defect reachable in the default Simulation path is that a
backend fault cannot be recovered through the advertised power cycle.

## Finding 1: spindle safe-stop false acknowledgment

`SpindleWorker` marks the spindle `SafeStopped` after calling
`SpindleHardware::safeStop()`, without receiving a success result. The
Huanyang implementation ignores the result of its `CONTROL_STOP` transaction.
`PhysicalExecutorIo` then waits for and accepts that state as safe output.

Evidence:

- [`SpindleHardware.cpp`](../src/SpindleHardware.cpp#L107) calls the hardware
  stop and unconditionally transitions to `SafeStopped`.
- [`SpindleHardware.h`](../src/include/machine/SpindleHardware.h#L41) defines
  `safeStop()` with no success/failure result.
- [`HuanyangSpindleHardware.cpp`](../src/physical/HuanyangSpindleHardware.cpp#L230)
  discards the result of `writeControl(CONTROL_STOP)`.
- [`PhysicalExecutorIo.cpp`](../src/PhysicalExecutorIo.cpp#L56) waits for the
  worker's safe-stop state and returns.
- [`physical_backend.toml`](../physical_backend.toml#L168) currently keeps the
  spindle disabled pending commissioning.

If serial communication fails while the VFD is running, the stop packet may
never reach the VFD while NGC believes the spindle is stopped. This can leave a
running spindle behind an executor fault or shutdown.

The stop operation should propagate failure and require a confirmed safe state.
If serial confirmation is impossible, the physical design needs an independent
hardware-safe fallback such as STO or a contactor; software must not report
`SafeStopped` merely because the request was attempted.

The existing communication-failure test uses a fixture whose `safeStop()`
always succeeds, so it does not cover a failed stop while the spindle is
active. The Huanyang failure test starts from an already stopped state.

## Finding 2: Simulation backend faults cannot be recovered by power cycling

When a timed program encounters a backend fault, the manager records a failed
program operation but does not fault the session coordinator:

- [`MachineSessionManager.h`](../src/include/machine/MachineSessionManager.h#L1849)
  handles `ProgramOperationState::Error` without calling the coordinator's
  `fault()` method.
- [`InProcessSimulationRuntime.cpp`](../src/InProcessSimulationRuntime.cpp#L656)
  stops only the persistent scheduler; it retains the same executor core.
- [`ProductionExecutorCore.cpp`](../src/ProductionExecutorCore.cpp#L751)
  refuses all lifecycle-demand convergence while the core is `Faulted`.
- [`MachineSession.cpp`](../src/MachineSession.cpp#L187) reports power-on
  success after publishing the idle demand without verifying that the backend
  left `Faulted`.
- New motion is then rejected by the backend-fault check at
  [`MachineSessionManager.h`](../src/include/machine/MachineSessionManager.h#L591).

The resulting sequence is:

1. A timed program faults the executor.
2. The program ends with an error, but the session coordinator remains `On`.
3. Power Off succeeds because the session appears idle.
4. Power On succeeds even though the persistent executor remains `Faulted`.
5. Start, homing, and jogging remain rejected indefinitely.

The normal recovery path should transition the session to a faulted state and
provide an explicit backend-fault reset that actually resets the retained
executor. Power-on should not report success unless the backend reaches its
requested non-faulted state.

## Verification

- The Windows build completed successfully.
- Available Windows unit and integration test executables passed.
- `git diff --check` passed.
- Linux-only physical-backend tests could not be executed in this Windows
  environment.
- No source files were modified as part of the audit.
