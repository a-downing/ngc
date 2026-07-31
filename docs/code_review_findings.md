# Code Review Findings

Reviewed the project-owned C++ under `src/` and `tools/`, configuration,
architecture docs, and tests. Vendored submodules were not audited. No source
files were modified during the review.

All findings below have been resolved. The descriptions have been tightened
where the original review overstated a consequence or described a configured
homing group imprecisely.

## Findings

### Resolved: frontend loss stops the physical executor

The Mesa backend and hardware-free IPC peer now compare their direct Linux
parent PID with the frontend PID recorded in the shared region during startup,
handshake, and steady-state service. Parent loss stops IPC consumption, requests
an executor-owned controlled stop, disables the executor at rest, transmits one
additional safe output cycle, and shuts down the child. This deliberately
covers frontend process exit; detecting a still-live but hung frontend remains
outside the current process-loss contract.

### Resolved: execution/planner errors stop already-published motion

`ProgramExecutionController` now retains a driver error as nonterminal while it
reuses the ordinary constrained controlled-stop path. Once the backend is
stationary, it submits `AbortRequest` to discard any retained horizon and
establish the safe spindle and digital-output image. Only the successful Abort
acknowledgement, or an already safe Faulted/Disabled backend, exposes the
terminal error to `MachineSessionManager`. Full control channels and transient
controlled-stop or Abort rejection are retried without terminalizing early.

The regression test injects a late prepared-geometry failure while a published
execution span is moving and requires acknowledged controlled stop and Abort
before the original error becomes terminal.

### Resolved: IPC configuration fingerprint is independently derived

The frontend and executor now independently fingerprint the exact machine and
optional backend TOML sources they parse, together with the effective executor
servo period. The child compares its derived fingerprint directly with shared
memory instead of accepting a parent-supplied command-line expectation.
Machine topology is covered by the complete configuration fingerprint, so the
redundant topology fingerprint was removed and the IPC ABI was advanced.

### Resolved: a normal new program starts from the retained backend position

Every program and MDI epoch now captures the stationary position observed from
the persistent backend. Before its evaluator thread starts, the interpreter
uses that position as its canonical pose; the trajectory driver uses the same
position as its planner origin. A non-preserving program still resets its
program and modal state, but no longer resets physical presentation state.
`ResetRequest` continues to retain backend position. As a final fixed-period
safety check, the executor rejects and faults a first normal execution span
whose starting position disagrees with its retained commanded position.

The regressions run consecutive normal programs from nonzero position and
verify that a deliberately discontinuous first plan faults without changing
the executor's commanded position.

### Resolved: `homing.require_before_motion` gates program and MDI admission

`MachineSession` now retains the configured policy and complete configured-joint
mask alongside its homed mask. When the option is enabled, the shared Program
and MDI admission path rejects execution with a structured `HomingRequired`
reason until every configured joint is homed. The session queue repeats the
check as an internal invariant. Homing and pre-homing joint service motion
remain available.

### Resolved: failed or stopped re-homing invalidates the previous homed mask

Accepting a homing operation now clears the session's homed mask immediately and
publishes that invalidation in the session snapshot. Only complete successful
homing restores the result mask, so stopped and failed attempts remain unhomed
for program admission and Machine-to-Simulation checkpoint validation. The
regression covers unhomed Program and MDI rejection, successful admission after
homing, and renewed rejection after stopped re-homing.

### Resolved: malformed numeric G/M words and standalone `M6`

G- and M-code conversion now uses one checked exact-integer helper before any
floating-to-integer conversion. Non-finite, out-of-range, and excess-precision
codes are rejected instead of being cast, rounded, or truncated.

The tool-change path resolves the block-local `T` first and otherwise uses the
Machine's retained modal selection. It reports a normal interpreter error when
no tool is selected or the selection is not a finite integer, and uses the one
validated integer throughout tool preparation, routine invocation, and
diagnostics. The regressions cover fractional and non-finite codes, invalid
tool selections, and `T1` followed by standalone `M6`.

### Resolved: unsupported homing modes are rejected

Encoder-index homing has been removed from the typed configuration and the
checked-in machine configuration. The loader explicitly rejects the legacy
`use_index` key, so it cannot be silently accepted through TOML's otherwise
open tables.

The three group-policy booleans are now configuration-boundary capability
assertions rather than unused runtime state. Single-joint groups reject the
meaningless options. Multi-joint groups must explicitly set
`start_together`, `stop_each_joint_on_trigger`, and `final_move_together` to
true, matching the one implemented policy: grouped phase starts, independent
constrained stops for each joint's switch, and a grouped final move. Missing or
false values are source-aware startup errors. The unused booleans were removed
from `HomingGroupConfiguration`.

A dedicated fast configuration test verifies the checked-in configuration and
rejects the legacy index key, single-joint policy declarations, every false
multi-joint option, and a missing required multi-joint option.

## Verification

- The current Windows build completes successfully.
- The focused homing-admission and stopped re-homing regression passed.
- The dedicated machine-configuration regression passed.
- CTest excluding the separately run `ngc_tests` target passed 9/9.
- Two full `ngc_tests.exe` runs reached the existing `checkpoint preview`
  checkpoint without reporting a failure, but did not complete within the
  two-minute and ten-minute verification limits and were terminated.
- The modified Linux-only sources and new IPC test pass Clang syntax checks.
  Linux external-executor and Mesa end-to-end execution was not runnable in
  this environment.

No code-review finding in this document remains open. This does not supersede
the separate staged physical-commissioning requirements.
