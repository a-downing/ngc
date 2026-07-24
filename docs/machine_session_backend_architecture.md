# Machine sessions, persistent Simulation, and the physical backend

## Purpose

This document records the intended architecture for persistent machine sessions,
standalone and Real-derived Simulation, and the future physical motion backend.
It also defines the incremental path from the current `SimulationWorker` design
to that architecture.

The design preserves the existing interpretation, prepared-geometry, trajectory
planning, and bounded `MotionBackend` contracts. It changes how those components
are owned and how their lifetimes relate to operator power, program execution,
MDI, Simulation, and Real operation.

## Core decisions

1. Machine power and program execution are separate lifecycles.
   - The operator selects Simulation or Real before pressing **On**.
   - **On** creates or connects the selected machine session and enables it.
   - **Start** begins a program run on the already-powered session.
   - Homing, jogging, MDI, and repeated program runs reuse that session.
   - **Off** stops as required, disables the session, and ends its powered
     lifetime.

2. Simulation is always available.
   - It does not require a configured, connected, or running Real backend.
   - Standalone Simulation begins from typed Simulation startup configuration.
   - Simulation uses an in-process backend and in-process bounded rings.

3. Real operation uses a separate physical-backend executable.
   - The NGC front end remains NRT.
   - A local shared-memory proxy implements the front end's `MotionBackend`
     endpoint.
   - The physical-backend process owns the RT executor and Mesa communication.
   - The Mesa 7I96 is reached through a dedicated Ethernet interface.

4. A powered, homed, stationary Real session may be used to initialize
   Simulation.
   - Real remains online, stationary, and monitored.
   - Simulation receives a consistent NRT checkpoint of Real state.
   - Simulation subsequently evolves independently.
   - No simulated command or state change is applied back to Real.

5. Simulation has the same durable state model as the intended Real machine.
   - Persistent parameter cells survive application shutdown and simulated
     power cycles.
   - A complete Simulation tool table survives application shutdown and
     simulated power cycles independently of the Real tool table.
   - Position, homing, modal state, tool selection, active tool offset, active
     execution, and other controller-runtime state are not durably serialized.
   - A live Simulation session may retain volatile state while it remains
     powered in the current application lifetime.

6. Real and Simulation parameter and tool-table stores are isolated.
   - Real parameter changes never implicitly mutate Simulation.
   - Simulation parameter changes never implicitly mutate Real.
   - Real tool-table changes never implicitly mutate Simulation.
   - Simulation tool-table changes never implicitly mutate Real.
   - An explicit **Simulate from Real** operation copies Real's parameter values
     and complete live tool table into Simulation as part of the new checkpoint.

7. Failure is explicit.
   - RealRun never silently falls back to Simulation.
   - An unavailable or incompatible physical backend prevents Real power-on.
   - Bounded-channel exhaustion, invalid state transitions, configuration
     mismatch, lost physical communication, and unproved motion remain fatal to
     the affected operation.

## Operator state model

Power, control target, and activity are separate state dimensions.

```text
Selected power mode:  Simulation | Real
Power state:          Off | Starting | On | Stopping | Faulted
Control target:       Simulation | Real
Activity:             Idle | Program | MDI | Homing | Jogging |
                      Holding | Stopping | Faulted
```

The ordinary startup flow is:

```text
select Simulation or Real
    -> On
        -> validate and start/connect the selected session
        -> enable it
        -> enter On + Idle
    -> Home, Jog, MDI, or Start
    -> return to On + Idle
    -> Off
```

Changing the selected power mode ordinarily requires going Off. The exception
is entering Simulation from an already-powered Real session. In that case Real
remains powered and the control target moves to a separately owned Simulation
session.

The UI must make this state unambiguous, for example:

```text
REAL: ON / HOMED / IDLE
CONTROL TARGET: SIMULATION
```

When Simulation owns operator control, Real motion-producing requests are
inhibited. Real continues to report communication, input, E-stop, and fault
state.

## Session architecture

```text
MachineSessionManager
|-- optional Real MachineSession
|   |-- InterpreterSession in RealRun mode
|   |-- ExecutionCoordinator
|   |-- HomingController
|   |-- PresentationTracker
|   `-- ExternalRealtimeRuntime
|       `-- shared-memory MotionBackend proxy
|
`-- optional Simulation MachineSession
    |-- InterpreterSession in Simulation mode
    |-- ExecutionCoordinator
    |-- HomingController
    |-- PresentationTracker
    `-- InProcessSimulationRuntime
        |-- MockMotionBackend
        `-- simulated servo scheduler
```

### `MachineSessionManager`

The manager owns the available sessions and the current control target. It:

- powers sessions on and off;
- creates standalone Simulation sessions;
- creates or rebases Simulation from a stationary Real checkpoint;
- keeps Real event and snapshot channels serviced while Simulation is active;
- routes program, MDI, homing, jogging, hold, resume, and stop requests only to
  the current control target;
- continuously exposes Real safety and fault state even when Simulation owns
  operator control;
- discards simulated volatile state rather than merging it into Real; and
- refreshes the UI from a new Real snapshot before returning operator control
  to Real.

### `MachineSession`

`MachineSession` is the backend-neutral, persistent powered-controller
abstraction used by the application. Its interface is expected to cover:

```cpp
powerOn();
powerOff();

startProgram(...);
executeMdi(...);
home();
startJog(...);
feedHold();
resume();
stop();

snapshot();
checkpoint();
```

It owns canonical NRT machine state and accepts an `InterpretationMode` during
construction. Simulation continues to expose `_task = 1`; RealRun exposes
`_task = 2`.

A session owns its live parameter bank and complete mutable tool table. Startup
loads them from that session's isolated stores. Program and MDI epochs reuse
both objects rather than receiving a fresh application-owned tool-table copy.
Persistence remains an NRT session-boundary concern and never enters
`MotionBackend`.

A new program or MDI operation starts a new execution epoch without constructing
or destroying the backend. Execution-epoch reset clears only execution-owned
queues, markers, feed-hold state, block lifecycle, and prior-run bookkeeping.
It does not reset physical position, homing, squaring offsets, backend
connection, or other powered-session state.

### `ExecutionCoordinator`

The coordinator owns at most one motion-producing activity. It replaces the
current collection of interdependent `SimulationWorker` boolean flags with an
explicit activity state and an NRT request queue.

Representative requests are:

```cpp
using SessionCommand = std::variant<
    StartProgram,
    ExecuteMdi,
    StartHoming,
    StartJog,
    RenewJog,
    StopJog,
    FeedHold,
    Resume,
    Stop>;
```

The coordinator owns `GeometryStreamProducer` and
`PreparedTrajectoryExecutionDriver` lifecycles for program and MDI epochs.
Homing and jogging remain backend controls, not generated G-code.

Ordered operator effects use the same execution-boundary discipline as other
stateful interpreter operations. `alert[...]` appends a typed persistent
operator message but does not itself suspend interpretation. M0 publishes a
program-pause boundary after prior motion and ordered effects; Resume is the
authoritative continuation action. Geometry-only Preview acknowledges that
boundary automatically, while Simulation and Real wait for an operator Resume.

### `PresentationTracker`

Presentation tracking is NRT and backend-independent. This component owns:

- execution-marker-to-command presentation activation;
- active and completed block lifecycle;
- tool, tool-offset, WCS, and modal presentation;
- chunk, span, and marker diagnostic association; and
- production of the presentation portion of `MachineSessionSnapshot`.

It never moves G-code commands or presentation objects into `MotionBackend`.

### `HomingController`

The existing NRT homing sequence becomes a backend-neutral component using
`TriggeredJointMove` and bounded backend controls. Mock input policy remains in
the Simulation runtime. Mesa input acquisition remains in the physical
transport. The generic controller sees typed logical inputs and completion
events.

### `BackendRuntime`

`BackendRuntime` owns the lifetime mechanics surrounding a `MotionBackend`
endpoint:

```cpp
class BackendRuntime {
public:
    virtual ~BackendRuntime() = default;
    virtual MotionBackend &endpoint() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual BackendCapabilities capabilities() const = 0;
};
```

This is an NRT ownership abstraction. It does not broaden the RT-facing
`MotionBackend` contract.

`InProcessSimulationRuntime` owns `MockMotionBackend`, the simulated servo
scheduler, accelerated playback coordination, synthetic input policy, and
mock-only diagnostics.

`ExternalRealtimeRuntime` owns the local IPC connection and a
`MotionBackend` proxy. It does not run a local servo loop.

## Real-to-Simulation checkpoint

Entering Simulation from Real is allowed only at a proved quiescent boundary:

- commanded and feedback velocity and acceleration are zero;
- no program, MDI, probe, homing, or jog owns motion;
- no feed-hold or stop transition is in progress;
- no queued execution item or synchronization operation remains; and
- Real has not faulted or lost position confidence.

The NRT session manager assembles a consistent in-memory checkpoint from the
stationary backend snapshot and canonical interpreter state. The checkpoint
contains the state needed to make Simulation begin at the same logical and
physical point, including:

- axis and joint state;
- homed-joint state and gantry squaring offsets;
- canonical parameter values and WCS memory;
- active tool, tool offset, and modal state where required for MDI continuity;
- the complete live Real tool table; and
- current presentation state.

The checkpoint does not contain active RT execution state, queued
`ExecutionItem` values, planner state, geometry channels, or partially evaluated
blocks.

After import, Simulation changes only its own state, parameter store, and
tool-table store. Leaving Simulation never copies its endpoint, homing, tools,
offsets, parameters, tool-table changes, outputs, or presentation back to Real.

Real must continue servicing the 7I96 and its watchdog while Simulation is
active. The front end must also continue draining Real events and snapshots so
bounded reverse channels cannot fill. A Real fault remains visible and blocks
returning operator control until the fault is handled.

A generation-tagged control-authority mechanism should make stale Real commands
incapable of taking effect after control has moved to Simulation. Real must
reject new program, MDI, jog, and homing requests while inhibited, while still
accepting safety, abort, disable, and required supervisory traffic.

## Parameter and tool-table persistence

### Parameter scope

Only explicitly eligible, nonvolatile canonical parameter cells are persisted.
The current memory model marks ordinary user parameters through `#5000` as
volatile and marks predefined G28, G30, G92, and WCS cells as nonvolatile.
`_task`, probe results, local stack values, and dynamically allocated program
storage are not persisted.

If a persistent user-parameter range is added later, it must be made explicit
in the memory model rather than inferred by persisting every allocated cell.

### Tool-table scope

Real and Simulation each persist one complete tool table. A tool record keeps
its identity, XYZABC offsets, diameter, comment, and future tool-geometry fields
together. Do not split measured offsets into a partial overlay file: doing so
would require ambiguous merge rules when tools are added, removed, renumbered,
or extended with new geometry.

Loading a tool with `T`/`M6`, applying `G43`, or changing the active tool does
not dirty the tool table. Operations that deliberately edit tool data, such as
the tool-table editor and supported `G10 L1`, `G10 L10`, or `G10 L11`
operations, mutate only the current session's live table.

`G10 L11` is a physical-state-dependent tool calibration operation. It requires
an explicit synchronization boundary before reading the current machine state.
For each programmed axis it computes the selected tool's offset so that the
current physical position would have the requested coordinate in G59.3 with no
G52/G92 offset. Unspecified tool fields remain unchanged. Updating a table entry
does not silently replace an already-applied `G43` offset; the new offset takes
effect when tool compensation is reapplied.

Real tool calibration saves only the Real table. Simulation tool calibration
saves only the Simulation table, including when Simulation was initialized from
Real. Returning control to Real never applies simulated tool-table changes.

### Tool-change execution scope

M6 is an ordered session operation, not an unrestricted modal subprogram call.
It first validates the selected tool and emits a spindle stop. That stop must
retire before `_tool_change` begins, and the spindle remains stopped when the
operation completes even if the routine starts it temporarily.

After the intrinsic M6 effects, the interpreter captures a typed tool-change
modal checkpoint. When `_tool_change` returns, it restores the caller's ordinary
modal groups, feed and spindle-speed settings, selected tool, active WCS and its
`#5220` selector, and the exact applied G43/G49 offset. Restoring the exact
applied offset is intentional: a `G10 L11` update changes the live tool record
but does not retroactively change compensation already active in the caller.

The restore policy deliberately retains physical and calibration results:

- the newly loaded physical tool;
- machine position and probe results;
- persistent parameter changes; and
- live tool-table mutations, including `G10 L11`.

A zero return from `_tool_change` or any nested interpreter failure is fatal to
the active run. The checkpoint is restored on that failure path as well.
Successful restored presentation is published only at the ordered
post-routine boundary, after the final spindle stop.

### Files

Real and Simulation use separate parameter and complete tool-table files:

```text
real_parameters.var
simulation_parameters.var
real_tool_table.txt
simulation_tool_table.txt
```

The paths are startup/application configuration. File parsing and disk access
remain NRT and never enter `MotionBackend` or the RT executor.

Both tool-table files use the same complete format. A load parses and validates
the entire file before replacing a live table. A save emits the complete table
to same-directory temporary output and atomically replaces the destination,
retaining the last valid file when replacement fails. Tool-table save failures
are visible operation failures.

For one-time migration, if neither isolated tool-table file exists and the
legacy `tool_table.txt` does exist, startup parses it once, prepares both
outputs completely, and publishes each through atomic replacement. Migration
never overwrites an existing isolated file. Because publishing two files is not
one filesystem transaction, finding only one isolated file alongside the legacy
file is an explicit startup error requiring operator-directed recovery. After
successful migration, the legacy file has no live ownership role.

### Parameter version-1 format

The parameter file is versioned, line-oriented text:

```text
NGC_PARAMETERS 1
unit inch

5161 0
5162 0
5163 -0.10000000000000001
5220 1
5221 7.25
5222 -3.5
```

Each data record is:

```text
parameter-address value
```

Format requirements:

- addresses are emitted in ascending order;
- floating-point values use a locale-independent representation with
  `max_digits10` precision for exact round trips;
- blank lines and `#` comments are accepted;
- a final unterminated line is accepted;
- numeric parsing consumes each complete token;
- duplicate, ineligible, unknown, or out-of-range addresses are rejected;
- non-finite values are rejected;
- unknown versions and unit mismatches are rejected;
- the complete file is parsed and validated before applying any value; and
- saving uses same-directory temporary output followed by atomic replacement,
  retaining the last valid file if the new save cannot complete.

Parameter stores are saved only from NRT code at an ordered, consistent
machine-session boundary. A failed save is visible and does not silently discard
the prior valid store.

### Startup and branching behavior

Standalone Simulation startup:

```text
volatile controller state <- typed Simulation startup configuration
persistent parameters     <- simulation_parameters.var
tool table                <- simulation_tool_table.txt
```

Real startup:

```text
volatile controller state <- typed Real startup configuration/backend snapshot
persistent parameters     <- real_parameters.var
tool table                <- real_tool_table.txt
```

Real-to-Simulation branching:

```text
volatile controller state <- current stationary Real checkpoint
persistent parameters     <- copy of current Real parameters
tool table                <- copy of current live Real tool table
future Simulation writes  -> simulation_parameters.var only
future Simulation edits   -> simulation_tool_table.txt only
```

After an application restart, Simulation does not regain its prior position or
homed state. It loads only its saved parameter bank and isolated Simulation tool
table.

## Configuration boundaries

`machine.toml` continues to define logical machine behavior:

- units;
- isolated Real and Simulation parameter-store and complete tool-table paths;
- axes, joints, and topology;
- motion and trajectory limits;
- homing behavior;
- logical digital-input identities;
- probing input identity;
- Simulation timing and typed Simulation startup state; and
- the optional Real backend kind and backend-configuration path.

Illustrative selection:

```toml
[operation]
default_mode = "simulation"

[real_run]
backend = "mesa_7i96"
backend_config = "mesa_7i96.toml"
```

The entire `[real_run]` section may be absent. Simulation remains available.

The backend-specific file maps the logical machine to physical hardware:

```toml
[board]
model = "7i96"
address = "192.168.1.121"

[inputs.probe]
pin = 0
active_low = false

[outputs.drive_enable]
pin = 0
active_low = false

[[stepgens]]
joint = 0
channel = 0
steps_per_machine_unit = 4000.0
invert_direction = false
```

The physical configuration owns:

- board address and expected identity/firmware capabilities;
- HostMot2 module and pin assignments;
- logical-input/output-to-pin mapping and electrical polarity;
- joint-to-step-generator mapping;
- steps per machine unit;
- step length, space, direction setup, and direction hold;
- watchdog and communication policy; and
- physical enable and fault wiring.

Startup code loads both files, validates their composition completely, and
passes typed configuration to the physical runtime. The backend and RT executor
never parse TOML or access configuration files.

## Physical backend process

The production topology is:

```text
NGC front end process
  interpreter, geometry, PathTempo, planning, UI
            |
            | bounded local shared-memory IPC
            v
ngc_rt_backend process
  RT executor, execution ownership, joint mapping
            |
            | Mesa HostMot2 transport on dedicated NIC
            v
Mesa 7I96
```

The NRT front-end proxy preserves the existing `MotionBackend` operations:

- `tryPublish()` writes one bounded `ExecutionItem`;
- `trySubmit()` writes one bounded `ControlRequest`;
- `tryTakeEvent()` reads one bounded `ExecutionEvent`; and
- `tryTakeSnapshot()` reads one bounded `ExecutionSnapshot`.

The shared-memory protocol requires:

- fixed-capacity SPSC rings;
- an explicit ABI/version handshake;
- configuration and machine-topology fingerprints;
- session and control-authority generations;
- clear producer/consumer ownership;
- no unbounded data or UI/interpreter objects;
- explicit peer-loss behavior; and
- no silent dropping, overwriting, reordering, or combining of messages.

The physical executable has an NRT startup phase that validates typed
configuration, connects to and discovers the board, allocates and locks memory,
and initializes communication. Its fixed-period RT thread owns all cyclic motion
execution. Optional low-priority diagnostics remain outside the RT loop.

Frontend loss causes the executor to select a proved constrained-stop path
rather than continue indefinitely. Backend-process or host loss is covered by
the HostMot2 watchdog and external drive-safety design. Neither response permits
automatic continuation of an interrupted execution epoch.

## Mesa 7I96 target

The first physical target is the Mesa 7I96:

- five hardware step/direction channels;
- one encoder channel, normally used for spindle feedback;
- eleven isolated inputs;
- six isolated outputs;
- RS-422/RS-485 and a parallel expansion connector; and
- HostMot2 access over UDP/LBP16.

The current four-joint X, Y1, Y2, Z topology fits four step generators and leaves
one spare channel.

The HostMot2 transport must discover and validate IDROM/module descriptors
rather than hard-code an assumed register layout. The FPGA generates individual
step pulses. The RT executor evaluates NGC's planned axis polynomials at each
servo tick, maps logical axes to joint references, and supplies bounded
step-generator commands.

Step-generator feedback represents the FPGA-generated pulse accumulator, not
measured motor position. Without added joint encoders, axis motion is open-loop.
NGC diagnostics and snapshots must distinguish planned state, generated-step
state, and actual measured state when the latter exists.

The dedicated Ethernet interface must not carry ordinary network traffic.
Communication timeout, packet-sequence failure, missed RT deadlines, watchdog
trip, drive fault, following error, and incompatible firmware become explicit
latched faults under typed policy.

The Mesa watchdog supplements but does not replace a hardwired E-stop,
drive-enable, and STO/safety design.

## RT executor boundary

The production RT executor consumes only the existing bounded RT-facing types.
It owns:

- plan and triggered-move execution;
- execution-marker crossing;
- stop-branch selection;
- feed hold and resume;
- probing input sampling and constrained stop;
- homing/service triggered-joint execution;
- jogging and jog lease expiry;
- motion ownership;
- logical-axis-to-joint mapping and gantry squaring offsets;
- digital input acquisition and bounded output application; and
- fault detection and snapshots.

The physical backend must not inherit mock-only policy or diagnostics.
Synthetic input transitions, accelerated time, and executed-servo diagnostic
history remain Simulation-only.

The current mock feed-hold retimer is not a production jerk guarantee. A
bounded, proved production hold/resume design is required before physical
program motion is enabled.

## Replacing `SimulationWorker`

The current `SimulationWorker` is the only complete timed-execution integration
path, but it combines:

- interpreter and canonical machine ownership;
- program/MDI lifecycle;
- prepared-geometry production;
- trajectory-driver ownership;
- mock backend ownership;
- Windows Simulation pacing and accelerated playback;
- homing and jogging coordination;
- feed-hold state;
- presentation reduction;
- diagnostics; and
- the UI snapshot.

It should be replaced incrementally, not rewritten in one step.

The common UI model becomes `MachineSessionSnapshot`. Simulation-only
diagnostics are optional data attached by `InProcessSimulationRuntime`; they do
not enter the generic backend.

During migration, a small compatibility facade may retain the
`SimulationWorker` API while delegating to the extracted components. The facade
is removed after the application uses `MachineSessionManager`.

## Implementation roadmap

### Phase 1: Lock down current behavior

- Add focused lifecycle tests for repeated program runs, MDI continuation,
  homing, jogging, feed hold, stop, parameter mutation, and presentation
  activation.
- Record current ownership and state-transition assumptions in tests before
  extracting components.
- Preserve all existing geometry, planning, backend, and accelerated-Simulation
  regressions.

### Phase 2: Implement persistent controller data

Status: implemented in the current application/`SimulationWorker`
compatibility layer. The Real parameter and tool-table paths are reserved for
the future Real session; Simulation does not write them.

- Add explicit iteration/export of eligible persistent `Memory` cells.
- Implement the strict version-1 parameter parser and serializer.
- Implement complete validation and atomic replacement.
- Add separate typed Real and Simulation parameter-store and complete
  tool-table paths.
- Load parameters and the session's isolated tool table at startup and save
  mutations at ordered NRT boundaries.
- Make the live tool table session-owned and retain its mutations across
  program and MDI epochs; starting a new operation must not replace it with an
  application-owned startup copy.
- Upgrade tool-table loading and saving to complete validation and atomic
  replacement without splitting offsets from their owning tool records.
- Migrate the legacy shared `tool_table.txt` only when neither isolated table
  exists, and reject partial or ambiguous migration.
- Implement synchronized `G10 L11` mutation of the current session's live tool
  table without changing an already-applied `G43` offset.
- Make M6 an ordered spindle-stopped operation with a typed modal checkpoint,
  a tool-change-specific restore policy, and fatal zero-return/nested-error
  handling.
- Add typed `alert[...]` status and an ordered M0 program-pause boundary for
  operator tool changes; Resume is the authoritative continuation action.
- Test exact floating-point round trips, duplicates, partial input, unit
  mismatch, corruption, failed replacement, volatile-cell exclusion,
  tool-record preservation, `G10 L11`, tool-change modal restoration,
  operator pause ordering, one-time migration, and Real/Simulation isolation.

### Phase 3: Extract presentation tracking

Status: implemented behind the current `SimulationWorker` compatibility facade.
`PresentationTracker` owns presentation activation, block lifecycle, spindle and
modal presentation, WCS history, and chunk/span diagnostic associations.

- Move command/marker, block lifecycle, tool/WCS/modal presentation, and
  diagnostic associations into `PresentationTracker`.
- Preserve chronological activation and completed-line semantics.
- Keep the application behavior unchanged through the compatibility facade.

### Phase 4: Extract the in-process Simulation runtime

Status: implemented behind the current `SimulationWorker` compatibility facade.
`InProcessSimulationRuntime` owns the persistent mock backend, sleeping scheduler
thread, timed-epoch activation, accelerated refill coordination, synthetic input
policy, synchronous service stepping, and mock-only timing and jerk diagnostics.

- Move `MockMotionBackend`, `WindowsServoPacer`, simulated servo scheduling,
  accelerated playback coordination, synthetic input policy, and mock-only
  diagnostics into `InProcessSimulationRuntime`.
- Give it a persistent powered lifetime independent of individual program runs.
- Keep the generic session unaware of tick multiplication and mock diagnostic
  storage.

### Phase 5: Introduce `MachineSession`

- Move interpreter, geometry producer, trajectory driver, homing, jogging, and
  operation coordination into backend-neutral components.
- Replace boolean control flags with explicit power/activity state and queued
  NRT commands.
- Make On/Off separate from Start/Stop.
- Ensure repeated program and MDI epochs reuse backend position, homing, and
  session state.
- Generalize `SimulationSnapshot` into `MachineSessionSnapshot`.

### Phase 6: Introduce `MachineSessionManager`

- Support standalone Simulation with no `[real_run]` configuration.
- Support optional Real and Simulation sessions concurrently.
- Add active-control-target routing and visible dual-session state.
- Add generation-tagged control authority.
- Keep inactive Real events and snapshots drained and faults visible.

### Phase 7: Add in-memory Real-to-Simulation branching

- Define and validate `MachineSessionCheckpoint`.
- Require a stationary, idle Real boundary.
- Import live Real state into a Simulation session.
- Copy Real parameters into Simulation's isolated parameter bank.
- Copy the complete live Real tool table into Simulation's isolated tool-table
  store.
- Prove through tests that simulated programs, MDI, jogging, homing, WCS/tool
  changes, parameter writes, and tool calibration cannot reach or mutate Real.
- Discard Simulation changes when returning to Real and refresh from a new Real
  snapshot.

### Phase 8: Add backend conformance tests

- Build a reusable behavioral suite for enable/disable, repeated epochs,
  publication and retirement, marker ordering, triggered moves, homing,
  jogging leases, stop/abort, channel capacity, and faults.
- Run it against the in-process backend and later against the reusable
  production executor core.

### Phase 9: Add the IPC skeleton

- Implement fixed shared-memory rings and the physical `MotionBackend` proxy.
- Add ABI, topology, configuration, session, epoch, and authority handshakes.
- Run a non-hardware backend process using the production IPC path.
- Test peer death, stale generations, full channels, restart, and refusal to
  resume interrupted epochs.

### Phase 10: Extract and prove the production executor core

- Factor reusable allocation-free execution mechanics without importing
  synthetic-input or mock-diagnostic policy.
- Add fixed-period and bounded-resource tests.
- Complete a production-grade feed-hold/resume design.
- Verify stop branches, triggered stops, jogging, homing, markers, and fault
  transitions under the production executor.

### Phase 11: Implement Mesa transport

- Begin with a read-only discovery/IDROM and cyclic-latency utility.
- Add typed 7I96 HostMot2 discovery and capability validation.
- Add cyclic input, step-generator, output, watchdog, packet-sequence, and fault
  handling behind a transport interface.
- Validate on a dedicated Linux RT host and NIC before enabling outputs.

### Phase 12: Staged physical commissioning

- Verify E-stop, STO/drive enable, watchdog, and output-safe states without
  motors enabled.
- Verify input polarity, home/limit switches, probe, drive faults, and spindle
  encoder.
- Verify one joint at low speed with conservative pulse timing and limits.
- Verify all independent joints, then coupled gantry motion and squaring.
- Verify triggered stops, jogging leases, queue-starvation stop, feed hold,
  abort, frontend loss, communication loss, and backend-process loss.
- Enable complete Real program execution only after every bounded safety and
  dynamic proof is exercised.

### Phase 13: Update durable repository guidance

- Update `AGENTS.md` when the implemented architecture changes its current
  repository-wide invariants.
- Update build and test instructions for the Linux RT executable and hardware
  tests.
- Keep implementation checkpoints and commissioning records out of
  `AGENTS.md`; retain them in focused documents or test evidence.

## Required invariants

- Preview remains geometry-only and never constructs a motion backend.
- Geometry preparation remains identical for Simulation and Real.
- Simulation works without Real configuration or hardware.
- RealRun never silently becomes Simulation.
- Backend selection and power are separate from program Start.
- A backend persists across program, MDI, homing, and jogging operations while
  its machine session remains powered.
- Simulated actions cannot publish motion or control mutations to Real.
- Real remains monitored while Simulation owns operator control.
- Simulation-to-Real state merge is forbidden.
- Only eligible parameters, not volatile controller state, survive shutdown.
- Real and Simulation parameter files are isolated.
- Real and Simulation persist isolated complete tool tables.
- Tool-table offsets are not split from their owning tool records.
- Real-to-Simulation tool-table inheritance is explicit and one-way.
- M6 stops the spindle before `_tool_change`, restores only its typed caller
  modal checkpoint, retains physical/calibration effects, and leaves the
  spindle stopped.
- M0 continuation requires Resume after the ordered pause boundary; alerts do
  not independently control execution.
- Configuration, parameter, and tool-table parsing are NRT.
- The RT contract remains bounded, allocation-free, and free of interpreter,
  geometry, TOML, filesystem, and UI objects.
- No bounded-capacity or safety proof is weakened to make hardware motion run.

## Open implementation inputs

These do not block the session and persistence refactor, but are required before
physical commissioning:

- Linux RT host, kernel/runtime choice, CPU affinity, and NIC selection;
- stepper/servo drive models and differential or single-ended wiring;
- steps per revolution, electronic gearing, screw pitch, and axis direction;
- required step length, step space, direction setup, and direction hold;
- E-stop, STO, drive-enable, fault, and contactor wiring;
- spindle/VFD command interface and spindle-encoder details;
- exact allocation of the 7I96 isolated inputs and outputs; and
- whether an explicit persistent user-parameter range beyond the current
  predefined nonvolatile cells is desired.
