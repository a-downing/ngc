# Code-reduction refactoring candidates

This document records independent, low-risk opportunities to reduce repeated
code in the application, Machine/Simulation session orchestration, and geometry
pipeline. It is intended to provide enough context to implement each item in a
fresh working session.

Before implementing an item, re-read the repository `AGENTS.md` and
`docs/cpp_style.md`, inspect the current code, and confirm that the duplication
still exists. Keep each item as a separate change unless one item directly
requires another.

## Recommended order

1. [Completed] Consolidate authority-aware `MachineSessionManager` delegation.
2. [Completed] Share `Machine::Axis` position-component access.
3. [Completed] Consolidate per-target persistence-store initialization.
4. [Completed] Share pure `MachineCommand` classification and endpoint helpers.
5. [Pending] Consider RAII cleanup of `imgui_main.cpp` only if it produces a meaningful
   net reduction.

The first item has the best expected code reduction for its risk. The second is
the most mechanical. The third touches persistence and therefore deserves more
careful error-path testing. The fourth touches geometry/planning classification.
The fifth is optional because most of `imgui_main.cpp` is unavoidable framework
bootstrap code.

## 1. Consolidate authority-aware manager delegation

### Completion record

Completed in the current worktree. `MachineSessionManager` now has const and
non-const variadic `delegateControlled()` helpers that retain the manager lock,
validate manager authority, translate to session-local authority, and invoke a
typed `MachineSessionHost` member function. Seventeen matching wrappers use the
helper. `powerOn()` remains explicit because it also enforces the GUI
emergency-stop latch, and target selection, checkpoint import, and emergency-stop
operations remain separate.

The implementation removes 102 lines and adds 69 lines in
`src/include/machine/MachineSessionManager.h`, for a net reduction of 33 lines.
More importantly, authority validation and translation for these wrappers now
have one implementation. The original estimate below was intentionally broader
than the final result; explicit per-operation fallbacks and the const overload
were retained for readability and type safety.

Verification completed:

- `cmake --build build --target ngc_tests ngc_emergency_stop_tests`
- `ctest --test-dir build --output-on-failure -R
  "^(ngc_tests|ngc_emergency_stop_tests)$"` passed the emergency-stop test; the
  general test was launched from the wrong working directory by this CTest
  configuration and could not find `autoload/tool_change.ngc`.
- `.\build\ngc_tests.exe` from the repository root passed.
- `cmake --build build --target imgui_main` passed.
- `git diff --check` passed.

### Motivation

`src/include/machine/MachineSessionManager.h` repeats the same sequence in many
public operations:

1. Lock `m_mutex`.
2. Resolve the session with `controlledSessionLocked(authority)`.
3. Return an operation-specific stale-authority result when resolution fails.
4. Translate manager authority with `localAuthority(*session)`.
5. Delegate to `MachineSessionHost` while retaining the current locking
   semantics.

The pattern appears in program start, homing, jogging, feed hold, Resume, Stop,
tool-table updates, and parameter/tool-table persistence operations.

### Proposed shape

Add one private, result-generic helper to `MachineSessionManager`, conceptually:

```cpp
template <typename Result, typename Operation>
Result delegateControlled(
    MachineControlAuthority authority,
    Result staleResult,
    Operation&& operation);
```

The helper should acquire `m_mutex`, validate the supplied manager authority,
derive the session-local authority, and invoke the operation. Individual public
methods should continue to provide the result appropriate to their API:
`false`, `SessionCommandRejection::StaleControlAuthority`, or
`staleControlAuthority()`.

### Expected payoff

Approximately 80-120 lines removed from a large header, with no intended state
machine or backend behavior change.

### Invariants and non-goals

- Preserve the exact manager-lock lifetime currently used by each operation.
- Preserve generation and target validation and the translation to local
  authority.
- Do not weaken stale- or wrong-target rejection.
- Keep `selectControlTarget()`, `simulateFromMachine()`, emergency-stop handling,
  and other operations with additional state transitions explicit.
- Keep the GUI emergency-stop check in the `powerOn()` path explicit.
- Do not change `MachineSessionHost`, `BackendRuntime`, IPC, or executor behavior.
- Avoid a hierarchy or macro for this; a small private callable helper is enough.

### Verification

- Build all affected targets.
- Run session-manager tests covering valid, stale, and wrong-target authority.
- Run tests for program, homing, jogging, Feed Hold, Resume, and Stop routing.
- Run persistence tests covering parameter and tool-table operations.
- Run emergency-stop tests to confirm the special paths were not absorbed into
  generic delegation.

## 2. Share `Machine::Axis` position-component access

### Completion record

Completed in the current worktree. `machine/MachineAxis.h` now provides const
and mutable `machineAxisPositionComponent()` overloads with one canonical
X/Y/Z/A/B/C mapping. Invalid `Machine::Axis` values fail explicitly through
`PANIC` instead of using the previous controller, manager, or UI fallbacks.
All production callers pass validated enum values.

The shared helper replaces the copies in `HomingController.cpp`,
`JoggingController.cpp`, `MachineSessionManager.h`, and `Application.cpp`. The
implementation review also found and replaced the same mutable mapping in
`MachineConfiguration.cpp`. Label generation, conversion to other axis enums,
and test-only independent mappings remain separate.

Production code removes 117 lines and adds 54 lines, including the new header,
for a net reduction of 63 lines. Focused test coverage adds 28 lines, leaving
the complete change with a net reduction of 35 lines.

Verification completed:

- `cmake --build build --target ngc_tests ngc_machine_configuration_tests
  imgui_main`
- `.\build\ngc_tests.exe` passed from the repository root.
- `.\build\ngc_machine_configuration_tests.exe` passed.
- `git diff --check` passed.

### Motivation

The same `Machine::Axis` switch for reading or mutating `position_t` components
is implemented independently in at least:

- `src/HomingController.cpp`
- `src/JoggingController.cpp`
- `src/include/machine/MachineSessionManager.h`
- `src/Application.cpp`

These copies cover X, Y, Z, A, B, and C and can drift in fallback behavior or
supported axes.

### Proposed shape

Add a small shared utility in the machine layer with const and mutable overloads
for selecting a `position_t` component by `Machine::Axis`. Use a name that makes
the axis type explicit and does not collide ambiguously with the existing
`AxisId` helpers in `AxisJointStateProjection.h` and the executor.

After introducing the utility, replace only exact switch duplicates. Keep UI
label generation and conversions from pendant-specific axis enums separate
unless they become clearly simpler through the same utility.

### Expected payoff

Approximately 40-60 lines removed, plus one canonical mapping for all six
machine axes.

### Invariants and non-goals

- Do not conflate `Machine::Axis` with executor `AxisId`.
- Preserve current behavior for every valid X/Y/Z/A/B/C value.
- Decide and document invalid-enum behavior from current callers; do not silently
  change a configuration-validation failure into a fallback coordinate.
- Do not introduce allocations or runtime lookup containers.
- Do not widen this task into a general axis-type redesign.

### Verification

- Add or retain focused coverage for all six axes through const and mutable
  access.
- Run homing and jogging controller tests.
- Run session touch-off/work-coordinate tests that use manager axis access.
- Build the application to cover the UI callers.

### Possible follow-up

`HomingController::observeSnapshot()` and
`JoggingController::observeSnapshot()` also contain nearly identical projection
of joint positions into machine-axis observations. Consider a separate shared
projection helper only after the axis-component utility is in place. Keep this
follow-up separate if its callback or observation-type differences require a
more elaborate abstraction.

## 3. Consolidate per-target persistence-store initialization

### Completion record

Completed in the current worktree. `ApplicationImpl` now keeps parameter and
tool-table paths and readiness in one `PersistenceStores` value per target.
Shared helpers initialize parameter and tool-table stores, while caller-provided
tool-table application keeps the Simulation-only Preview update explicit.
Orderly shutdown consumes the same per-target state, and active-store access
uses one target-selection helper.

The refactor removes 135 lines and adds 94 lines in `src/Application.cpp`, for a
net reduction of 41 lines. Parameter initialization remains independent of
legacy tool-table migration failure, preserving the existing protection against
overwriting an invalid parameter store.

Verification completed:

- `cmake --build build --target imgui_main ngc_tests`
- `.\build\ngc_tests.exe` passed from the repository root, including parameter
  persistence and tool-table migration coverage.
- `git diff --check` passed.

### Motivation

`ApplicationImpl::init()` in `src/Application.cpp` performs structurally similar
parameter-store and tool-table-store setup for Simulation and Machine:

- inspect the configured store path;
- load an existing store or configure a new path;
- update the target session;
- track whether orderly shutdown may save that store;
- report path-specific failures.

`ApplicationImpl::terminate()` already recognizes the common structure through
its `saveStores` helper. Initialization should have a similarly explicit shared
shape.

### Proposed shape

Introduce a compact per-target store state, for example readiness flags and the
two paths, plus a helper that initializes one target's parameter and tool-table
stores under the correct control authority.

Keep the Simulation-only side effect explicit: its loaded tool table is also
applied to the Preview `Worker` and the application's preview-facing tool state.
This can be a small caller-side step or an explicit callback/policy argument;
avoid hiding it in a target-name conditional deep inside the helper.

Use the same state representation during orderly shutdown so the initialization
and save paths cannot disagree about readiness.

### Expected payoff

Approximately 60-90 lines removed from `Application.cpp`, with clearer symmetry
between Simulation and Machine persistence handling.

### Invariants and non-goals

- Preserve isolated Simulation and Machine parameter and tool-table stores.
- Preserve legacy tool-table migration and partial-migration rejection.
- Preserve the rule that a failed Simulation load inhibits overwriting the
  invalid store during shutdown.
- Preserve target selection and generation-tagged control authority behavior.
- Preserve the Simulation-only Preview tool-table update.
- Preserve all existing error messages or improve them without losing path and
  target context.
- Do not create a generic persistence framework beyond these four stores.

### Verification

- Test missing stores, valid existing stores, malformed stores, and failed
  migration.
- Test orderly save after successful initialization.
- Confirm an invalid loaded store is not overwritten.
- Test Machine-unavailable startup.
- Test independent Machine and Simulation tool tables and parameters.
- Confirm Preview receives the Simulation tool table and never the Machine table.

## 4. Share pure `MachineCommand` helpers

### Completion record

Completed in the current worktree. `MachineCommand.h` now owns continuous-motion
classification and ordinary line/arc endpoint extraction. The geometry producer,
prepared-geometry construction, and trajectory planner use those helpers, so
Preview preparation and trajectory timing share one definition of these command
semantics.

Production code removes 71 lines and adds 54 lines for a net reduction of 17
lines. The original estimate was optimistic because the canonical implementation
uses the repository's expanded control-flow style. Focused tests cover exact-stop
mode, feed and rapid lines, zero-speed and machine-coordinate lines, feed and
zero-speed arcs, probes, nonmotion commands, and line/arc endpoints.

Verification completed:

- `cmake --build build --target ngc_tests`
- `.\build\ngc_tests.exe` passed from the repository root.
- `cmake --build build --target imgui_main`
- `git diff --check` passed.

### Motivation

`src/include/machine/GeometryStreamProducer.h` and
`src/include/machine/TrajectoryPlanner.h` independently implement the same or
nearly the same operations:

- decide whether a command is continuous line/arc motion under its path mode;
- extract the start position of a line or arc;
- extract the end position of a line or arc.

Keeping these predicates separate risks Preview/preparation and trajectory
planning disagreeing about command boundaries.

### Proposed shape

Add pure helpers close to the `MachineCommand` model for:

- continuous-motion classification from a `MachineCommand` and path mode;
- motion start position;
- motion end position.

The record/input wrappers in the producer and planner should only unpack their
local metadata before calling these helpers. Keep prepared-geometry construction,
source-entity handling, and trajectory timing in their existing layers.

### Expected payoff

Approximately 30-40 lines removed and one source of truth for command
classification and endpoints.

### Invariants and non-goals

- Preserve rapid/zero-speed behavior.
- Preserve the exclusion of machine-coordinate line motion where currently
  required.
- Preserve line and arc endpoint semantics.
- Do not classify probes, dwell, spindle, tool, or other nonordinary commands as
  continuous motion.
- Do not move geometry construction into `MachineCommand`.
- Do not make geometry behavior depend on Preview versus Simulation.

### Verification

- Add focused cases for feed line, rapid line, zero-speed line, machine-coordinate
  line, feed arc, zero-speed arc, probe, and nonmotion commands.
- Run geometry preparation and trajectory-planner tests.
- Confirm exact-stop and continuous-path boundary behavior remains unchanged.

## 5. Optional RAII cleanup of `imgui_main.cpp`

### Motivation

`src/imgui_main.cpp` contains GLFW window creation, ImGui context/backend setup,
the frame loop, and matching teardown. RAII could make early failure and teardown
less repetitive.

### Decision rule

Proceed only if a narrowly scoped owner for GLFW/ImGui resources produces a
clear net line reduction and improves cleanup behavior. Most of this file is
one-time framework bootstrap, so moving the same statements into another file
does not count as useful reduction.

### Possible scope

A single local/application-private owner could manage successful initialization
stages and destroy only resources that were created. Keep command-line parsing,
machine-configuration loading, `Application` lifetime, the frame loop, and frame
timing visible in `main()`.

### Invariants and non-goals

- Preserve platform-specific OpenGL and GLSL selection.
- Preserve callback installation, maximized-window persistence, scroll
  accumulation, viewport setup, 3D rendering order, and frame timing.
- Ensure `Application::terminate()` runs before graphics resources it depends on
  are destroyed.
- Do not share GUI resource ownership with Machine or Simulation backend code.
- Do not add a general framework abstraction for a single executable.

### Verification

- Build and launch the application on supported platforms.
- Check normal startup and shutdown.
- Check failed GLFW initialization and failed window creation.
- Check persisted maximized state, scrolling, resizing, and rendering order.

## Candidates intentionally deferred

### `SessionBackendRuntime` forwarding methods

`SessionBackendRuntime` contains many one-line forwards to its owned
`BackendRuntime`, but a forwarding base class would mostly relocate those lines.
It becomes worthwhile only if another production owner can reuse the exact same
adapter. Do not move Simulation-only pacing, synthetic inputs, snapshots, or
jerk diagnostics into the generic `BackendRuntime` contract merely to remove
forwarders.

### `SimulationExecutor` and external-runtime PImpl forwards

These forwards preserve implementation hiding and stable headers. Replacing
them with templates or exposing implementation details is not a favorable
risk/reduction tradeoff.

### Production executor motion branches

The executor's normal motion, triggered moves, stop branches, feed hold,
homing, and jogging may contain superficially similar state transitions, but
their safety and lifecycle semantics differ. They are not low-risk
code-reduction targets. Any refactoring there should begin from a specific bug,
proved invariant, or dedicated design task rather than a line-count goal.

## Working discipline for each item

- Start from a clean understanding of the current worktree; preserve unrelated
  user changes.
- Record baseline relevant tests before editing.
- Make the smallest abstraction that removes the identified duplication.
- Avoid opportunistic formatting or adjacent redesign.
- Compare the final diff's removed and added lines; reject an abstraction that
  merely moves code without simplifying ownership or behavior.
- Update this document after completing an item so a future context does not
  repeat finished work.
