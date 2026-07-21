# NGC Repository Guide

## Communication and document scope

Never use LaTeX in responses for this repository because the Codex UI does not render it. Write equations and mathematical notation as plain text or in plain-text code blocks.

Keep this file focused on durable repository-wide constraints. Do not turn it into a changelog, benchmark notebook, dated checkpoint, or task backlog. Verify implementation claims against current code and tests, and update this guide when an architectural invariant changes.

For C++ changes, follow the repository's dedicated [C++ style guide](docs/cpp_style.md). Treat it as the formatting authority for new and modified C++ code.

## Continuous-path terminology

Use these names consistently in code, diagnostics, research notes, and discussion:

- **Source entity**: one canonical CAM line or arc.
- **Retained primitive section**: the untrimmed middle of a sufficiently long source line or arc.
- **Junction blend spline** (short form: **junction blend**): a local spline joining two retained primitive sections. It replaces only the trimmed ends near their junction. The current construction uses six-control cubic splines.
- **Short-entity cluster**: consecutive short source entities selected for replacement as one group.
- **Short-entity cluster spline** (short form: **cluster spline**): one variable-control spline that reconstructs and replaces an entire short-entity cluster. It does not merely connect the entities.
- **Execution span**: one timed axis-space cubic polynomial emitted by the trajectory planner. Reserve the word "span" for this meaning; do not use it for a group of source entities or a spline knot interval.

Use these geometry and jerk terms distinctly:

- **Normal sharpness**: `|kappa'|`, the normal component of the arc-length curvature-vector derivative. It omits the tangential `-kappa^2 T` component.
- **Full geometric jerk coefficient**: `|q'''|`, the complete arc-length third derivative of path position. For a planar curve, `q''' = kappa' N - kappa^2 T`.
- **Coupled path jerk**: the complete time-domain path jerk `q' j + 3 q'' v a + q''' v^3` used for trajectory feasibility and limits.

The two replacement structures are `retained primitive section -> junction blend -> retained primitive section` and `short-entity cluster -> cluster spline`.

## Architecture

NGC is a C++23 non-real-time CNC front end. Its interpretation pipeline is:

```text
lexer -> parser/AST -> InterpreterSession/evaluator -> modal Machine -> MachineCommand stream
```

`InterpreterSession` incrementally evaluates a program and emits one `MachineCommand` at a time. During Preview and Simulation, `GeometryStreamProducer` owns interpretation and constructs the shared prepared geometry before any trajectory timing:

```text
GeometryStreamProducer
    -> Worker / PreviewGeometryCollector                  (geometry-only Preview)
    -> PreparedTrajectoryExecutionDriver                 (timed Simulation)
       -> BoundedLookaheadTrajectoryPlanner
       -> ExactStopTrajectoryPlanner
       -> MotionBackend
```

`Worker` retains immutable prepared curves and presentation metadata as Preview's sole toolpath representation. `SimulationWorker` consumes the same prepared stream through `PreparedTrajectoryExecutionDriver`. Ordinary motion becomes timed axis-polynomial `PlanChunk` values; probes and homing/service moves use executor-owned triggered moves.

`MockMotionBackend` is a non-RT implementation of the production-shaped backend contract. There is no real-time executor, HAL component, or physical backend yet. Preserve the separation between interpretation, geometry preparation, trajectory planning, execution, and hardware access. Never make geometry construction depend on whether the consumer is Preview or Simulation.

Pendant device access is NRT and separate from `MotionBackend`. `HidTransport` owns platform HID discovery and report I/O, while model drivers such as `vista_cnc_p2s::Driver` own device report framing, decoding, and display payloads. A model device-session manager owns its reader thread, cumulative wheel counts, ordered control-change events, latest snapshot, display serialization, and cancellation; heartbeat-only reports update its snapshot without filling the control-event queue. VistaCNC P2-S reads have a two-second report deadline, after which the outstanding platform read is cancelled and the session disconnects. Keep Windows handles and future Linux descriptors behind the transport boundary; pendant drivers and managers must not call `Machine` or the backend directly.

Model pendant profiles translate device-session events into model-independent operator intentions without executing them. In P2-S Step mode, wheel movement uses the configured fine increment with the EN button released and the configured coarse increment while it is held; changing the button level does not cancel Step motion. Touch-off adjustment and feed-override adjustment still require the ordinary held-button state after an observed release and fresh press. Connection and selector/safety transitions disarm held-button actions until another release/press cycle. Selector transitions, OFF/E-stop level changes, and disconnects cancel or inhibit actions; releasing OFF or the pendant E-stop never enables motion or allows a same-report wheel action. Preserve the right selector's observed transient zero as an inhibited break-before-make state rather than a command selection.

P2-S Zero mode stages a desired work coordinate at the current physical point in configured fine-distance increments. Button release preserves this staging so a firmware-recognized double click can commit it; selection or safety changes discard it. Commit updates the active WCS canonical memory and cached offset only while no motion owner is active. It must not be implemented as generated G-code.

P2-S Velocity mode requires the EN button to remain held after a fresh release/press cycle. Timestamp byte-0 detents when their reports arrive and require two same-direction detent reports within the bounded pairing interval before starting motion. Derive velocity from the smoothed inter-detent period; do not command velocity from the byte-1 50 ms rate code or the wrapping bytes 4-5 firmware motion accumulator. Accumulator changes may provide periodic opportunities to renew the command or evaluate the missed-detent deadline, but accumulator magnitude or decay must not prolong motion. A direction reversal resets the estimator and requires a new pair. Velocity updates retain one stable jog token, renew its dead-man lease, and recompute jerk-limited motion from the current PVA state. A missed-detent deadline, button release, selector/safety change, disconnect, or lease expiry requests a constrained stop.

`operator_control::JogController` is the shared NRT authority that converts model-independent pendant jog intentions into bounded backend controls. After homing, pendant axis selections produce logical-axis jog targets; joint-group and individual-joint targets remain service/pre-homing operations. It must reject rather than defer pendant motion while another owner is active. Step jogging is bounded to one active/submitted increment plus at most one pending follow-up: one accepted wheel report produces one signed configured increment regardless of its raw count magnitude, and additional detents are ignored once that single lookahead slot is occupied. A cancellation discards the follow-up and requests a token-matched constrained stop. Fine/coarse Step distances are typed machine configuration; Step execution uses the selected axis/coupled joints' full physical velocity, acceleration, and jerk limits.

The VistaCNC P2-S LCD is two eight-character rows and requires periodic output reports with advancing sequence bytes. While an axis is selected, identify the active WCS and axis on the first row and show its signed work position, including the active tool offset, on the second. Zero mode instead shows the axis and live work position on row one and the active WCS plus staged target coordinate on row two. LCD formatting and refresh remain NRT presentation concerns. The device manager owns a latest-value asynchronous display worker: changed presentation wakes it immediately and unchanged text is refreshed every 20 ms. Because output can echo into input after a newer frame is sent, retain and filter a bounded history of recent display prefixes.

## Build and test

The supported development environment is Windows with Clang, Ninja, CMake, and vcpkg. GLM comes from vcpkg; GLFW, ImGui, toml++, and PathTempo are submodules. PathTempo privately provides the pinned unmodified upstream HiGHS and Ruckig dependencies used by trajectory timing, while NGC's other NRT motion generation shares PathTempo's Ruckig target.

Configure a fresh Release build with:

```powershell
cmake -S . -B build -G Ninja `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -Dglm_DIR=C:\vcpkg\installed\x64-windows\share\glm `
  -DCMAKE_C_COMPILER=clang `
  -DCMAKE_CXX_COMPILER=clang++
```

Build and test with:

```powershell
cmake --build build
.\build\ngc_tests.exe
ctest --test-dir build -E "^ngc_tests$" --output-on-failure
```

Tests are framework-free executables, with the core suite in `src/test.cpp`. The core suite loads `machine.toml` and `tool_table.txt` relative to its working directory, so run `build\ngc_tests.exe` from the source directory and exclude its build-directory CTest registration as shown above. Project warning and debug-symbol flags are target-scoped; dependency targets must not inherit them. Optimization is also applied target-by-target: every locally compiled C/C++ project and dependency target uses `-O2` in both Debug and Release builds.

## Configuration and interpreter semantics

`machine.toml` is loaded and validated once at startup through `MachineConfiguration`. Keep toml++ and disk access inside that loader. Planners, workers, and backends receive typed configuration; configuration parsing must never enter the RT-facing backend.

- `machine.units` selects the fixed internal `Machine::Unit`.
- Trajectory, axis, joint, jogging, probing, simulation, and homing values use the configured machine unit and seconds as documented in `machine.toml` and `MachineConfiguration`.
- `rapid_velocity` is converted at the loader boundary to the per-minute representation expected by canonical motion.
- `simulation.scheduler_period` must be an integer multiple of `servo_period`.
- The loader owns and validates logical axes, axis-to-joint topology, digital-input IDs, probing input, per-joint motion/homing values, and ordered homing groups. Logical coordinates without configured axes, duplicate or out-of-range IDs, and incomplete joint/group mappings are startup errors.
- A positive homing `backoff_distance` means clearance behind the fast-trigger position. `switch_position` is assigned at the slow latch and `home_position` is the final post-latch destination.
- Jog start limits come from `[jogging]`; jog stop and lease-expiry authority comes from the physical axis/joint limits carried in `stopLimits`.

Preserve these interpreter decisions:

- G20/G21 program values are converted into the configured machine unit as blocks are consumed. Scale X/Y/Z, I/J/K, G94 F, and G10 linear offsets; do not scale A/B/C or selector/index words.
- Tool-table values and persistent coordinate offsets are already in configured machine units and are not rescaled when G20/G21 changes. Startup/application code owns tool-table disk I/O and injects the resulting table; `Machine` construction performs no filesystem access.
- `beginProgramRun()` resets transient pose, modal state, tool offset, dynamically allocated program storage, and the local stack while preserving the tool table and predefined parameter cells through address 6000. `Machine` does not retain generated commands.
- Numeric parsing must consume the entire input. Tool-table parsing is line-oriented, accepts a final unterminated line, rejects duplicate tools with row context, and must not accept partial file I/O.
- G43 applies the full XYZABC tool offset; G49 clears it. G53 bypasses work-coordinate offsets but retains the tool offset. Preserve explicit-G53 metadata through every consumer.
- `InterpretationMode` maps `_task` to Preview = 0, Simulation = 1, and RealRun = 2. Parameter 6000 is read-only.
- IJK arcs allow one in-plane center word to be omitted as zero, but require at least one applicable center word. G91.1 makes IJK incremental; G90.1 makes it absolute in the active work coordinate system without changing endpoint mode. Reject zero radius and radius mismatch beyond `Machine::arcTolerance()`.
- G64 is executable. Its optional non-negative P value is a blend/control-spacing scale in machine units, not a strict maximum-deviation tolerance. G61 and G61.1 clear it.
- Rapid `MoveLine` speed `-1` is a sentinel, not a physical feedrate. G1/G2/G3 require an established positive modal feedrate and must report a recoverable `InterpreterError` when it is absent.

Reserve `PANIC` for internal invariants and impossible program states. Unsupported or malformed G-code, missing tools, invalid G10 operations, and other program-input failures use the recoverable interpreter-error path. Source-related errors include source name, line, and column without duplicate prefixes.

`InterpreterSession` exposes one chronological typed `Print`/`Error` status stream. Do not split terminal errors into a separate collection or regroup messages by severity. Planning, execution, backend, and interpreter failures are fatal to the active G-code run: report them to the chronological UI status stream and `stderr`, and never silently skip commands, discard a buffered horizon, publish an unproved plan, or fall back to a less capable motion mode.

## Prepared geometry and G64 construction

`PreparedGeometry` is immutable NRT data shared by Preview and timed planning. A prepared curve is an exact line, the shared endpoint-exact arc reference, or a spline containing its degree, controls, knots, derivative control nets, arc-length bracket table, and exact inverse-cache policy. `PreparedPathPiece` selects an interval of that curve and carries stable piece/command identity, programmed feed, ordered curve-distance presentation activation stations, replaced source intervals, and geometric samples. The geometry producer owns source-command-to-curve activation placement; the trajectory compiler only maps those prepared stations to emitted execution spans and packets. Every spline carries timing data and geometric samples per non-empty knot interval, so each interval becomes one timing piece. Cluster intervals additionally carry producer-computed static geometric velocity caps and local programmed feeds. These caps remain separate from programmed feed so later feed scaling can reuse them. Dynamic time laws, execution spans, packetization, and stop tails do not belong in prepared geometry.

The endpoint-exact arc reference is authoritative for Preview and planning. It supports directed CW/CCW major arcs, full circles, helices, non-XY planes, and accepted decimal-IJK radius mismatch while reaching both canonical endpoints exactly. Preserve ordered distance association and certified adaptive-integral inversion; do not substitute tolerance-near cache matches, unchecked table interpolation, unordered nearest-geometry tests, or a constant-radius assumption.

An executable G64 chain contains only:

- exact retained line or arc sections;
- local six-control cubic junction blends; and
- variable-control cluster splines replacing bounded short-entity clusters.

For each source entity, `p_entity = min(P, entity_arc_length / 6)`. An ordinary junction replaces the final `3*p_incoming` and first `3*p_outgoing`; line controls remain on the source lines, while arc-side controls preserve endpoint tangent and curvature. The junction itself is not a control point. Retained primitive sections keep their own feed; a junction blend uses the arithmetic mean of its adjacent entity feeds.

A maximal run of entities with length at most `6P`, bounded by entities longer than `6P`, is reconstructed as one cluster spline. Cluster knot intervals retain local programmed-feed boundaries for timing. `spline_detail::continuousSplineFitSolver()` is the one production selection point; the current default is `VelocityTargetedBandedFairness`. Only cluster reconstruction uses the variable-control quintic B-spline path. Junction blends remain six-control cubic B-splines. Both use clamped open-uniform knot vectors. Preview and timed planning must consume the exact same prepared controls, degree, knots, samples, feeds, and source-interval metadata; neither may independently reconstruct geometry. Replaced source geometry may be displayed for inspection but is not executable motion.

Rapids, explicit G53 motion, probes, non-G64 motion, P changes, discontinuous endpoints, protected presentation changes, and non-motion commands are protected boundaries. Do not blend across them or solve planning/capacity problems with synthetic connectors or a whole-path spline fit.

## Prepared streaming and lifecycle

`GeometryStreamProducer` is the sole producer for the owning bounded `PreparedGeometryForwardChannel`. `Worker` or `PreparedTrajectoryExecutionDriver` is the sole consumer. `GeometryFeedbackChannel` carries synchronization releases, canonical Preview probe results or executed probe results, and aborts in the reverse direction. These NRT move-owning channels are distinct from the allocation-free RT-style `MotionBackend` channels. They must never drop, overwrite, reorder, or silently combine messages. Keep the nonblocking release/acquire SPSC fast path and a sleeping path without lost wakeups.

Prepared slices are storage and streaming boundaries only. Consecutive slices with the same chain ID are one continuous geometry chain for Preview and timed planning. A slice boundary must not create an exact stop, reset velocity or acceleration, split a cluster spline, duplicate command activation, or require a connector. `PreparedContinuousEnd` is the semantic chain terminator. Preserve canonical endpoint continuity and unique piece IDs across every slice transition.

The default 0.25-second preparation target is a positive minimum based on `path_length / programmed_feed`, not a maximum. For continuous G64 geometry, the producer waits for sufficient long-source context, finalizes the replacement entering the next long anchor, publishes eligible pending work, and carries the outgoing replacement into the next slice. Construct a deferred retained anchor section once, after its outgoing replacement is known. An all-short region cannot be split merely to meet the duration target.

Synchronization is explicit. Parameter reads, M6 preparation, externally visible `print`, probing, and homing must not run ahead of required motion. `GeometryStreamProducer` publishes synchronization/probe fences and stops interpretation until the matching feedback arrives. Preview releases a generic fence after consuming prior geometry; timed execution releases it only after already-published motion reaches the required held boundary. New operations that consume executor/hardware state or create externally ordered physical effects require an explicit barrier.

`InterpreterSession::next()` is the compatibility NRT interface and acknowledges generic synchronization immediately because it has no executor. Prepared consumers use source-aware `nextWithBlocks()` through the geometry producer. Block lifecycle records have stable execution ID, source, line, and text; nested blocks complete only after evaluator return.

Each run starts from reset state and must not append to a prior run. Consumer-thread evaluator exceptions become `InterpreterError`, not assertions, process exits, or GUI-thread exceptions. Timed simulation stops and clears active/queued mock motion on fatal error, then copies the chronological status stream into its snapshot.

## Trajectory planning and RT-facing execution

`ExactStopTrajectoryPlanner` treats canonical exact-stop motion as rest-to-rest jerk-limited motion. It uses PathTempo's scalar transition solver, maps its time-domain cubic phases to XYZABC cubic polynomials, and validates aggregate path plus per-axis velocity, acceleration, and jerk. Lines and endpoint-exact arcs share the fixed-capacity `PlanChunk` representation. Exact polynomial extrema are final authority; preliminary samples or analytic caps are not proof.

`BoundedLookaheadTrajectoryPlanner` collects compatible prepared G64 pieces across slice boundaries. `trajectory.lookahead_duration` is a minimum predicted duration for attempting a rolling prefix, not a hard horizon or semantic stop. Rolling splits require a dynamically feasible nonzero PVA boundary and a proved stop-feasible initial suffix section; the proof section is bounded near the lookahead duration and may end before the retained suffix so an unbounded cluster does not enter every feasibility probe. Rolling boundaries may use retained-line interiors or prepared cluster-spline knot boundaries; cluster splits partition the existing immutable curve, knot-interval timing metadata, and geometric samples without reconstructing geometry. Do not enable arc-interior anchors by weakening exact boundary equivalence. Protected boundaries or `PreparedContinuousEnd` finalize the remaining chain.

PathTempo owns continuous scalar timing. NGC supplies ordered timing pieces with differential stations and boundary PVA, then materializes PathTempo's time-domain cubics into axis-space execution spans. Continuous timing must enforce programmed feed, aggregate path limits, per-axis limits, coupled acceleration `q' a + q'' v^2`, and coupled jerk `q' j + 3 q'' v a + q''' v^3`. NGC's exact emitted-polynomial extrema, ordered geometry verification, C2 continuity, capacity, activation ownership, and stop safety remain the final gates; any exact materialization correction must be returned as tighter per-piece scalar limits and re-solved by PathTempo. Bounded planning resource exhaustion is a detailed fatal error, not permission to publish partial or unproved work.

Continuous compilation stages verified NRT polynomials and packetizes them into `PlanChunk` values with at most 256 normal execution spans and 16 stop-tail spans. Geometry pieces may cross packet boundaries. Every packet must pass the stop-branch gate: the tail starts from the immutable branch state, is position/velocity/acceleration continuous, respects aggregate and axis limits, matches its declared terminal state, and ends stationary. Selecting a stop tail while an unpublished moving suffix exists is fatal until a proved resume path exists.

The RT-facing contract contains only bounded, allocation-free execution, control, event, and snapshot data transported by SPSC channels. `ExecutionItem` contains `PlanChunk`, axis-space `TriggeredMove`, or joint-space `TriggeredJointMove`. Do not add G-code entities, prepared curves, UI strings, TOML objects, simulated-input policy, debug geometry, or unbounded ownership to `MotionBackend`. Planning diagnostics are NRT-only and may be exposed through `SimulationSnapshot` or mock-only diagnostic interfaces, never the generic backend.

## Triggered motion, homing, and jogging

HAL must never call `Machine` directly. Preserve the probe ownership flow:

```text
Machine emits ProbeMove
    -> driver publishes TriggeredMove
    -> executor samples the configured input and owns the constrained stop
    -> driver converts completion to ProbeResult
    -> Machine resumes after the probe barrier
```

`TriggeredMove` is a generic signal-terminated axis-space move. The executor generates the approach, samples the digital input every servo cycle, latches the full trigger state, and owns the jerk/acceleration-limited stop. Completion distinguishes trigger and stopped states from `ReachedTarget`, `Aborted`, and `Fault`; it contains no probe-specific interpreter or tool geometry.

Logical XYZABC axes are not motors. The backend owns axis-to-joint mapping and gantry squaring offsets. Use `TriggeredJointMove` only for homing, squaring, and service operations that temporarily decouple joints. Initial search and pull-off are joint-relative because machine coordinates do not yet exist. Establish joint position only while held and only after all participating joints stop; coordinate soft limits apply after homing.

Mock probing and homing policy stay outside `MotionBackend`. Synthetic mock input transitions change only simulated input state and must not supply a predicted stop point. Preview resolves a probe at its canonical target; timed Simulation returns the backend's latched physical-style result. Canonical preview toolpaths use emitted command coordinates and G53/WCS/tool-offset semantics; physical tool dimensions affect only the tool overlay and simulated contact.

Jogging uses the bounded backend control/event channel, never generated G-code or `ExecutionItem`. Continuous jogs require renewal of a stable token before a fixed-tick dead-man lease expires; expiry and UI release request a constrained stop. Axis, coupled-joint-group, and individual-joint targets are mutually exclusive with other motion ownership. `SimulationWorker` is the sole NRT producer of mock jog controls; the GUI must not write directly to the backend. The backend lease remains authoritative if UI renewal stops.

Program feed hold is backend-owned motion, not a frozen NRT scheduler or servo clock. A hold request begins on-path braking on the next servo tick by retiming the precomputed trajectory, reports `Holding` while moving, and reports `Paused` only after `BackendHeld` establishes zero velocity and acceleration. Resume from a feed hold retains the normal-branch execution cursor and ramps the reference execution rate back to one; it must not reconstruct geometry, reset span progress, or replay events. During an axis-space triggered probe approach, feed hold uses the executor's constrained-stop machinery while retaining the active `TriggeredMove`, target, and input transition; it must not report probe completion or select the probe branch. Continue sampling the probe input while feed-hold braking: a detected transition latches the servo-sampled trigger state, supersedes the feed hold, and completes the probe through its ordinary constrained-stop result. If no transition occurs, Resume regenerates the remaining approach from the held zero-PVA state. Reaching a stop branch while normal-path feed-hold braking or resume retiming remains active is a fatal backend condition, not permission to enter the stop tail or resume from it. The initial mock-only retimer constrains acceleration and its requested tangential braking/acceleration profile while retaining executed per-axis and aggregate jerk as diagnostics; it does not establish a production jerk guarantee.

`MockTrajectoryDiagnostics` and executed-servo jerk samples are mock-only. They record states actually calculated at the configured servo period and must not be reconstructed by the GUI or added to `MotionBackend`. Keep the executed time-domain jerk diagnostic distinct from the prepared cluster's full geometric jerk coefficient and from normal sharpness. Accelerated mock playback coordinates scheduler batches with NRT planning, publication, and presentation bookkeeping so the simulated clock cannot manufacture a queue-starvation stop while its sole producer is actively refilling the bounded horizon.

## Preview and presentation

Preview is geometry-only: it performs no trajectory timing or emitted-polynomial proof. It renders immutable `PreparedPreviewScene` data and must not reconstruct motion from `MachineCommand`. The background visibility thread performs CPU tessellation/culling and publishes complete replacement batches; only the render thread performs OpenGL calls and GPU uploads.

Presentation state is command-boundary data outside the RT contract. Timed Simulation applies tool, tool offset, WCS, modal state, and active-block state only when the owning activation span executes, not when interpretation reads ahead. Prepared activation stations must remain ordered through slicing and rolling splits; cluster source commands activate progressively at their producer-associated curve distances rather than being collected at the cluster beginning. Keep UI/lifecycle bookkeeping and command-to-packet searches out of the planner hot path. Preserve nested block lifecycle semantics and compact completed-line flags in UI snapshots.

Keep canonical toolpath geometry separate from physical tool display. MCS remains available independently of WCS presentation; when no tool is loaded, represent the machine position directly rather than inventing tool geometry.

## Change discipline

- Preserve unrelated user changes in a dirty worktree.
- Prefer current code and regression tests over historical Markdown notes.
- Add focused regressions for changed interpretation, geometry, streaming, planning, synchronization, or backend behavior.
- Preserve exact endpoint and activation ownership across commands, slices, packets, barriers, and runs.
- Never weaken a bounded-capacity or safety proof to make a failing path execute.
