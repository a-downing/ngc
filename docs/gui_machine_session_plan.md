# GUI alignment with persistent machine sessions

## Purpose

This document tracks the work needed to align the ImGui application with the
persistent machine-session architecture described in
[Machine sessions, persistent Simulation, and the physical backend](machine_session_backend_architecture.md).

It is an implementation plan, not a description of functionality that already
exists. Completed claims must be verified against the current code and tests.

## Current mismatch

The application owns Simulation and the configured non-hardware Real session
through `MachineSessionManager`, presents their backend-neutral power and
activity state, and exposes the powered-session lifecycle explicitly. Phases 1
through 6 are complete. The remaining mismatch is physical status and
checkpoint presentation: the Windows Real target has null I/O, and there is no
Real-to-Simulation checkpoint operation in the GUI.

## Target operator model

The GUI must present power, control target, and activity as independent state:

```text
Control target: Simulation
Power: On
Activity: Idle
Homing: 3/3 joints
```

The main machine controls should follow the powered-session lifecycle:

```text
[On/Off]  [Start]  [Feed Hold/Resume]  [Stop]  [Home]  [Jog]
```

Preview remains a geometry and source-inspection workflow. It must not be
presented as the current state of the controlled machine.

The initial implementation may expose only Simulation as an available control
target. It should still show that target explicitly so adding Real does not
require another conceptual redesign.

## Proposed layout

### Machine-session header

Keep a compact, always-visible header containing:

- selected control target;
- power state;
- current activity;
- homing summary;
- prominent fault or inhibited state.

When Real support is added, the header must make concurrent state unambiguous.
For example:

```text
REAL: ON / HOMED / IDLE
CONTROL TARGET: SIMULATION
SIMULATION: ON / PROGRAM
```

### Primary controls

Keep machine operations together and derive their availability from explicit
power, activity, and program-operation state. Preview actions and engineering
visualization toggles should be in a separate group.

Disabled controls should provide a concise reason in a tooltip. Examples:

- `Power on the Simulation session first.`
- `Program currently owns motion.`
- `Waiting for feed-hold acknowledgement.`
- `Axis X must be homed before logical-axis jogging.`
- `The Real machine session is not configured.`

### DRO and motion pane

Place the operator DRO with the jog controls. It should show:

- machine coordinates;
- work coordinates;
- tool-tip coordinates;
- active work coordinate system;
- active tool and XYZABC tool offset;
- per-axis or per-joint homed state;
- spindle state, direction, and commanded speed;
- current motion owner.

Coordinates must be derived from the controlled session snapshot. Preview
coordinates and modal state belong in Preview-specific inspection UI.

### Status and messages

Keep the persistent status area concise and operator-oriented:

```text
SIMULATION · ON · PROGRAM · RUNNING
G54 · T3 · S12000 CW
Latest status message
```

Present the chronological typed `Print`, `Alert`, and `Error` stream in a
scrollable message console. Preserve its original ordering.

Move the following to a separate, collapsible **Simulation Diagnostics**
window:

- scheduler and servo timing;
- deadline and wake-lateness measurements;
- geometry-stream diagnostics;
- trajectory-planning activity and correction history;
- backend epoch, chunk, execution span, queue, branch, and fault details;
- executed jerk diagnostics.

The diagnostics UI must consume optional `SimulationDiagnostics` rather than
adding Simulation-only fields to the generic backend contract.

### Operator pauses and alerts

Visually distinguish:

- an `alert[...]` operator message;
- an M0 continuation boundary;
- feed hold and its acknowledgement transition;
- controlled Stop.

When an M0 boundary is ready for operator input, its window should contain
direct **Resume** and **Stop** actions. It should not require the operator to
find the corresponding toolbar button.

## Implementation phases

### Phase 1: Centralize GUI session-state interpretation

Status: complete. `gui/MachineSessionView.h` now provides the shared
power/activity labels and command-availability derivation used by the toolbar,
MDI, and jog controls. `MachineSessionManager` now returns structured rejection
details for primary program, MDI, homing, jogging, feed-control, Stop, and reset
requests, and the GUI reports their precise operator-facing reason. The
derivation has focused core tests. `MachineSessionSnapshot` now exposes a
backend-neutral program-operation presentation that distinguishes running,
feed-hold acknowledgement, held, feed-resume acknowledgement, M0 pause,
controlled Stop, and terminal outcomes; the toolbar labels and availability
derive from that presentation. Continuous-jog renewal and stop queue failures
are also reported instead of being discarded.

- Add GUI-side helpers that derive labels, colors, and command availability
  from `MachinePowerState`, `MachineActivity`, backend state, and the
  program-operation presentation.
- Remove repeated `SimulationStatus` expressions used as a proxy for general
  motion ownership.
- Continue using `SimulationStatus` only where the compatibility presentation
  requires its specific completed, stopped, or failed outcome.
- Route rejected GUI requests to a visible operator message instead of
  discarding their return values.

Acceptance criteria:

- Program, MDI, homing, and jogging are displayed as distinct activities.
- The same authoritative helper controls Start, MDI, Home, and Jog
  availability.
- Holding, program pause, and Stop-in-progress have distinct labels.
- Every enabled primary action either succeeds or reports its rejection.

### Phase 2: Add the session header and operator DRO

Status: complete. The main toolbar displays the selected control target, power
state, machine activity, compatibility execution state, and homed-joint
summary. The motion pane displays live machine, work, and physical tool-tip
coordinates together with the active WCS, tool, applied XYZABC tool offset,
spindle state, motion owner, and homing summary.

- Display control target, power, activity, and homing summary.
- Add machine, work, and tool-tip coordinates.
- Display active WCS, tool, tool offset, and spindle state.
- Keep the current target explicit even while Simulation is the only available
  session.

Acceptance criteria:

- The operator can identify the controlled session and motion owner without
  opening diagnostics.
- Coordinate labels identify their coordinate system.
- Unhomed joints are visible without opening the jog pane.
- The display remains live during timed backend execution.

### Phase 3: Separate Preview from machine state

Status: complete. The machine status area derives modal presentation only from
the controlled Simulation session snapshot. Preview modal codes are not shown;
Preview remains a geometry and source-selection workflow. Preview selection and
machine execution retain distinct highlighting.

- Display machine modal codes only from the controlled session snapshot.
- Do not add a separate Preview modal-code presentation.
- Ensure program highlighting clearly differentiates Preview selection from
  active or completed machine execution.
- Keep prepared geometry shared between Preview and timed planning; this is a
  presentation separation only.

Acceptance criteria:

- A powered idle Simulation session continues to display its own state.
- Compiling or selecting Preview geometry cannot change the displayed machine
  modal state.
- Preview and execution highlighting cannot be mistaken for one another.

### Phase 4: Correct parameter and tool-table ownership

Status: complete. `MachineSession` now exports a read-only canonical parameter
snapshot, while `MachineSessionManager` publishes a mutex-protected coherent
copy refreshed at initialization, load, reset, and execution-epoch boundaries.
The **Simulation Parameters** window reads that session-owned snapshot and
identifies its isolated store. The **Simulation Tool Table** window identifies
its owner and store, disables all editing while the session cannot accept
controller-data changes or its store is unavailable, and explains the
inhibition. Saving routes through the manager/session boundary. Focused tests
cover program/MDI publication, Preview isolation, and controller-data edit
inhibition.

- Add a read-only, thread-safe parameter snapshot or lookup API at the
  machine-session manager boundary.
- Make the System Parameters window read the controlled session's parameter
  bank.
- Label the active store, initially **Simulation Parameters**.
- Label the tool-table editor with its owning session.
- Disable edits while the owning session cannot accept them and explain why.
- Preserve the independent Simulation and Real persistence paths.

Acceptance criteria:

- Parameter values shown after MDI or a program epoch match the persistent
  Simulation session.
- Preview evaluation cannot alter the displayed Simulation parameter values.
- Tool-table edits name and modify only the selected session's table.
- Store failures remain visible and do not permit overwriting an invalid store.

### Phase 5: Split operator status from engineering diagnostics

Status: complete. The persistent status pane now presents a compact
backend-neutral Simulation power, activity, and program-operation summary above
a scrollable message console. Operator rejections, Preview parser failures, and
the Preview and Simulation typed status streams remain visible, with each typed
stream retaining its original order and failures emphasized. Scheduler,
geometry-stream, trajectory-planning, backend execution, and correction-history
details are available in a separate **Simulation Diagnostics** window.
Simulation runtime timing reads the optional `SimulationDiagnostics`
presentation, and the cluster geometric and executed jerk-comb controls have
moved from the primary toolbar into that window.

- Replace the diagnostic text block in the status bar with a concise operator
  summary.
- Add a scrollable chronological message console.
- Add a separate Simulation Diagnostics window.
- Move the cluster and executed jerk-comb controls into Preview or diagnostics
  controls.

Acceptance criteria:

- Normal operation does not require scanning planner or packet diagnostics.
- All existing diagnostics remain accessible.
- Typed interpreter messages retain chronological order and severity.
- A backend, planning, execution, or interpreter failure remains prominent.

### Phase 6: Expose the powered-session lifecycle

Status: complete. `MachineSessionManager` now starts with Simulation powered
off and serializes explicit power-on and power-off operations through its
persistent worker. Snapshots expose Starting, Stopping, On, Off, and Faulted
power states. Power-off rejects while motion or a queued operation owns the
session. The primary toolbar has a distinct On/Off lifecycle control and keeps
Start and Stop separate. The obsolete **Reset Simulation** API and GUI action
have been removed; power cycles retain the persistent session position, modal
state, parameters, and live tool table.

- Add manager operations for queued or safely synchronized power on and power
  off.
- Do not power Simulation solely as an incidental constructor side effect.
- Reject Off while an unsafe transition is in progress, or perform the
  required controlled Stop before disabling according to the session contract.
- Replace **Simulate** with **Start**.
- Replace **Reset Simulation** with explicitly scoped operations such as
  **Reset Controller State** or a documented power cycle. Do not make a reset
  silently erase persistent parameters or the session-owned tool table.

Acceptance criteria:

- On/Off and Start/Stop are visibly and behaviorally separate.
- Repeated program and MDI epochs reuse position, homing, live tools, and
  session state as specified by the architecture.
- Power transitions expose Starting, Stopping, and Faulted states.
- No GUI action bypasses `MachineSessionManager` control authority.

### Phase 7: Prepare for Real and control transfer

Status: in progress. The standalone application presents an explicit
Simulation/Real selector and simultaneous live session rows. `[real_backend]`
enables Real through `ExternalRealtimeRuntime` and `ngc_ipc_backend`; selecting
and powering Real runs program motion through the production IPC and executor
path. The GUI tags target-dependent motion and controller-data operations with
current authority, switches live tool-table ownership and isolated stores with
the target, and rejects stale generations or wrong targets before admission.
Mock scheduler diagnostics and accelerated playback remain Simulation-only.
The manager's target router and checkpoint boundary remain covered by the
explicit in-process test host, while IPC tests cover a complete configured Real
program epoch. Physical safety/I/O state and the Real-to-Simulation checkpoint
UI remain unfinished.

Implement this phase with the corresponding Real-session manager work.

- Add the Simulation/Real target selector.
- Disable unavailable targets with a precise explanation.
- Display both sessions when Real remains powered while Simulation owns
  control.
- Tag GUI commands with current control authority where required so stale
  commands cannot reach a newly selected target. Complete for the standalone
  Simulation manager boundary and dual-session manager tests.
- Keep inactive Real safety, communication, E-stop, and fault state visible.
- Add the explicit **Simulate from Real** checkpoint operation. Its quiescence
  and validation contract already exists at the manager boundary.

Acceptance criteria:

- The controlled target is unambiguous at all times.
- A control transfer cannot route a stale motion command to the prior target.
- Simulation changes never propagate back to Real.
- Returning control to Real refreshes the GUI from a new Real snapshot.

## Expected API work

The GUI work is likely to require the following backend-neutral additions:

- manager-level power-on and power-off requests;
- an explicit operation-state presentation sufficient to distinguish running,
  feed-hold transition, held, M0 pause, Stop transition, completed, and failed;
- thread-safe session parameter lookup or snapshot access;
- structured command rejection information where a boolean result cannot
  explain the disabled or rejected state;
- eventually, snapshots for both available sessions and generation-tagged
  command submission.

These additions should remain NRT. Do not place UI strings, parameter maps,
tool tables, or other unbounded presentation ownership in `MotionBackend`.

## Testing strategy

Prefer extracting state derivation into small functions that can be tested
without rendering ImGui.

Add focused tests for:

- control availability across every power and activity state;
- Program versus MDI ownership;
- feed hold, held, Resume, M0 pause, and Stop transitions;
- homing and jogging inhibition;
- session parameter display after program and MDI mutation;
- Preview/session state separation;
- tool-table ownership and edit inhibition;
- unavailable Real selection;
- stale control-authority rejection at the standalone manager boundary and
  across an actual target transfer. Complete at the in-process manager-test
  boundary; repeat against the production Real session when available.

Retain manual GUI checks for layout, legibility, tooltip clarity, resizing, and
fault prominence.

## Out of scope

This plan does not:

- claim that the selectable non-hardware Real target is a commissioned physical
  backend;
- move geometry construction into the GUI;
- give Preview a separate geometry implementation;
- place diagnostics or presentation objects into the RT-facing backend;
- merge Real and Simulation persistent state;
- redesign pendant device handling, except where the common session status and
  command-availability presentation affects it.
