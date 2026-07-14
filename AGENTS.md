# NGC Development Context

## Project role

This repository is a C++23 non-real-time CNC front end. Its current pipeline is:

```text
lexer -> parser/AST -> InterpreterSession/evaluator -> modal Machine -> MachineCommand stream -> consumer(s)
```

`InterpreterSession` incrementally evaluates the program and emits one `MachineCommand` at a time. `TrajectoryExecutionDriver` connects execution consumers to `ExactStopTrajectoryPlanner`, publishes ordered `ExecutionItem` values through the generic `MotionBackend` SPSC contract, and owns the execution probe-result barrier handshake. Ordinary executed motion is compiled into timed axis-polynomial `PlanChunk` values; executed probes are compiled into executor-owned `TriggeredMove` values. `MockMotionBackend` is the current non-RT implementation of that production-shaped contract. `Worker` performs geometry-only immediate preview and retains a `ToolpathRecorder`; `SimulationWorker` runs timed playback and exposes NRT `SimulationSnapshot` presentation state to the OpenGL UI.

The repository now contains an exact-stop trajectory planner and a non-RT mock backend, but not a real-time executor or HAL component. Preserve the separation between interpretation, planning, real-time execution, and hardware access.

## Build environment

The active development environment is Windows with Clang, Ninja, CMake, and vcpkg. GLM is supplied by vcpkg; GLFW, ImGui, Ruckig, and toml++ are Git submodules.

Configure a fresh build with:

```powershell
cmake -S . -B build -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -Dglm_DIR=C:\vcpkg\installed\x64-windows\share\glm `
  -DCMAKE_C_COMPILER=clang `
  -DCMAKE_CXX_COMPILER=clang++
```

Build and test with:

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

Tests are framework-free in `src/test.cpp`. CTest runs them from the source directory because `Machine` loads `tool_table.txt` relative to the working directory.

Project warning, optimization, and debug flags are target-scoped. Bundled GLFW and ImGui sources live in their own dependency targets and must not inherit project-wide compiler flags.

## Machine configuration

`machine.toml` is loaded and validated once at application startup through the project-owned `MachineConfiguration` layer. toml++ must remain isolated to that loader; planners, workers, and backends receive typed configuration values rather than parsing files. A missing or invalid configuration is a startup error with source location context.

`machine.units` selects the fixed internal `Machine::Unit`. Current `[trajectory]` values use that machine unit and seconds: `path_acceleration`, `path_jerk`, `rapid_velocity`, and `arc_chord_tolerance`. `rapid_velocity` is converted at the loader boundary to the canonical per-minute representation currently expected by the planner. Each configured logical `[axes.*]` table supplies independent positive `max_velocity`, `max_acceleration`, and `max_jerk` values. The loader copies those values into fixed XYZABC `TrajectoryLimits`; absent logical coordinates remain unbounded. `[simulation]` contains seconds-based `servo_period` and `scheduler_period` values; the scheduler period must be an integer multiple of the fixed servo period. Configuration parsing and disk access must never enter the RT backend.

`[jogging]` supplies one global positive `acceleration` and `jerk` used to limit acceleration into all axis, coupled-joint, and individual-joint jogs. The UI clamps these values against the selected axis/joint physical limits. Jog stop requests and lease expiry use the separate physical axis/joint acceleration and jerk carried in `stopLimits`, so a gentle jog start does not weaken stopping authority.

The loader also owns the typed axis/joint topology, logical digital-input map, probing input, per-joint motion and homing values, and ordered homing groups. `switch_position` is the coordinate assigned at the slow latch, `home_position` is the final post-latch destination, positive `backoff_distance` is clearance behind the fast-trigger position in machine units, and `debounce` is a non-negative duration in seconds. It validates that configured coordinates have axis tables, every joint belongs to exactly one matching logical axis and one homing group, input names resolve to unique backend IDs, and joint/group IDs are bounded and unique. Following-error, step-generator, Mesa-card, and pendant configuration are deliberately out of scope for now.

## Important current decisions

- `Machine::Unit` is the fixed internal machine unit: inch or millimetre.
- G20/G21 values are converted into the configured machine unit as blocks are consumed.
- Scale X/Y/Z, I/J/K, G94 F, and G10 linear offsets. Do not scale A/B/C or selector/index words.
- Tool-table and persistent coordinate-offset values are already stored in configured machine units and must not be rescaled when G20/G21 changes.
- `beginProgramRun()` clears transient pose, modal state, and tool offset while preserving parameter memory and the tool table. `Machine` does not retain generated commands; each consumer owns any retained representation.
- `Memory::init()` establishes the persistent machine-parameter boundary. `beginProgramRun()` calls `resetProgramStorage()` to discard per-run named globals and local stack storage while preserving predefined parameter cells (currently through address 6000). Do not reinitialize all memory between runs.
- Numeric text parsing must consume the entire input; a valid numeric prefix followed by trailing text is invalid.
- Tool-table loading is line-oriented, accepts a final line without a trailing newline, and rejects duplicate tool numbers with row context. File helpers must report incomplete seek/read/write operations rather than returning partial success.
- G43 currently applies the full XYZABC tool-table offset. G49 clears it.
- G53 bypasses work-coordinate offsets but retains the active tool offset.
- `MoveLine::machineCoordinates()` records whether a line was emitted by explicit G53 motion. Preserve this metadata through consumers.
- `InterpretationMode` has three distinct `_task` values: Preview = 0, Simulation = 1, and RealRun = 2. `_task` is read-only at parameter 6000.
- IJK arcs allow either in-plane center word to be omitted; an omitted word means zero. At least one applicable center word is required.
- G91.1 uses incremental IJK center coordinates. G90.1 uses absolute center coordinates in the active work coordinate system without changing G90/G91 endpoint mode.
- IJK arc validation rejects zero radius and radius mismatch beyond `Machine::arcTolerance()`. The tolerance is 0.0005 inch or the equivalent 0.0127 mm.
- Arc preview and exact-stop planning share the project-owned endpoint-exact arc reference. It uses the directed sweep derived from the signed axis, supports CW/CCW major arcs, full circles, helical and non-XY arcs, and blends both radial arms so accepted decimal-IJK radius mismatch still reaches both canonical endpoints exactly.
- G64 is an executable modal path mode. Its optional P word establishes a non-negative machine-unit blend scale; G61/G61.1 clear the retained G64 P value. Compatible G64 feed lines and arcs execute as the same exact retained primitive sections and local clamped six-control degree-three B-splines shown by preview. P is not a strict maximum-deviation tolerance.
- Rapid `MoveLine` commands currently use speed `-1` as a sentinel; this is not a physical negative feedrate.
- G1, G2, and G3 require an established positive modal feedrate. A missing/non-positive feedrate must produce an `InterpreterError`; never dereference an absent modal `F` value or terminate the process.
- `PANIC` is reserved for internal invariants, impossible enum/state combinations, and other program logic errors. Unsupported G/M codes, unsupported words or modes, malformed arcs, missing tools, invalid G10 operations, and other program-input failures must throw through the recoverable interpreter-error path.
- `InterpreterSession` retains one chronological typed status stream containing `Print` and `Error` entries. Do not split terminal interpreter errors back into a separate print/error collection or regroup messages by severity in the UI. Prints are blue and prefixed `PRINT:`; errors are red and prefixed `ERROR:`.
- Errors associated with a source statement or block must include source name, line, and column. Statement evaluation and consumer-thread block processing both add location context without duplicating an existing location prefix.
- Planning, execution, backend, and interpreter failures are fatal to the active G-code run. Do not silently skip work, discard buffered commands, or fall back to a less capable motion mode after a failed proof. Publish information-rich errors to the chronological UI status stream and `stderr` so development failures remain visible.

## Exact-stop trajectory planning

`ExactStopTrajectoryPlanner` currently treats every canonical motion as an exact stop. It uses one-degree-of-freedom Ruckig rest-to-rest timing to produce jerk-limited S-curve phases, maps those phases onto axis cubic position polynomials, and validates the emitted axis-space velocity, acceleration, and jerk independently for XYZABC. Aggregate path acceleration and jerk remain additional caps. Lines derive scalar timing limits directly from their constant tangent; arcs use analytic-parameter tangent sampling for preliminary caps. Exact extrema of every emitted cubic are the final authority, and violations are corrected by uniform time stretching. Triggered axis moves also reduce their scalar approach limits against the configured physical-axis limits.

Lines and adaptively subdivided circular/helical arcs are emitted through the same fixed-capacity `PlanChunk` representation. The NRT arc reference adaptively integrates the full XYZABC derivative, inverts path distance to normalized arc parameter, supplies unit tangents, and explicitly preserves the canonical start and end positions. Arc speed receives a conservative curvature/centripetal acceleration cap, followed by emitted polynomial constraint validation and uniform time stretching.

Arc cubic subdivision is tolerance-verified. Verification preserves ordered association between each polynomial interval and its source-arc interval, recursively subdivides the polynomial Bezier control hull, and accepts it only inside an ordered source-chord capsule after reserving the reference curve's conservative second-derivative chord-error bound from `arc_chord_tolerance`. Do not replace this with unordered nearest-geometry sampling or a constant-radius bound: accepted rounded IJK arcs may have slightly different start and end radii. Consecutive planned commands must meet at their canonical endpoints without synthetic connector spans.

`BoundedLookaheadTrajectoryPlanner` collects compatible G64 feed lines and arcs with the same P value, protected presentation state, and continuous canonical endpoints until a protected boundary, synchronization point, or interpreter completion. The old fixed 32-command flush has been removed; RT packet capacity must not create a semantic G-code stop. Rapids, explicit G53 moves, probes, non-G64 motion, P changes, discontinuities, and non-motion commands terminate the compatible horizon. It retains complete NRT command-boundary presentation outside `MachineCommand` and the RT contract; ordered activation span IDs and owning packet identities apply tool, tool-offset, WCS, modal, and active-block state only when execution reaches the owning command.

An executable G64 window consists only of exact trimmed line/arc sections and the local six-control cubic B-spline at each compatible junction. Retained primitives use their own programmed feed after unit conversion; each junction spline uses the arithmetic mean of its two entity feeds. Every piece receives sampled tangent, curvature, and curvature-derivative caps from the aggregate and per-axis limits; B-spline curvature derivatives use analytic first/second/third parameter derivatives. Jerk-aware forward and backward passes currently establish a velocity envelope using zero-acceleration transition distances. A conservative minmod estimate then proposes one shared scalar acceleration at internal stations, and one-dimensional local Ruckig position solves time each piece with those PVA boundary states. The complete zero-acceleration timing is also solved and retained whenever the acceleration-carrying candidate is not faster. Straight pieces preserve Ruckig's constant-jerk phases exactly. Curved pieces fit across the complete local solve and recursively subdivide only as ordered geometry verification requires, avoiding numerically unstable micro-phase fits. Exact polynomial extrema remain the final aggregate and per-axis velocity, acceleration, and jerk authority; violations reduce only the owning piece's local limits and repeat the reachability/timing pass rather than stretching the entire horizon. Full acceleration-aware `(s,v,a)` reachability and adaptive timing stations remain unfinished near-time-optimal work.

Continuous compilation stages its verified polynomial stream in NRT memory and packetizes it into consecutive `PlanChunk` values containing at most 256 normal spans. A primitive, spline, or local three-span chain may cross a packet boundary. Internal packets retain the exact moving terminal state and receive a synchronized multi-axis Ruckig stop represented by constant-jerk cubic spans; the existing 16-span stop capacity remains until measured failures justify changing the RT contract. The driver publishes the packet batch in branch order and associates each command with the packet containing its activation span.

Every planner-produced `PlanChunk` passes a stop-branch gate before publication. The stop tail must be non-empty, continuous in position/velocity/acceleration from the immutable branch state through every span, respect aggregate and per-axis limits, match its declared terminal state, and finish stationary. If continuous compilation, packetization, fixed capacity, geometry, constraint, C2, activation ownership, or stop-branch verification cannot be proved, planning must stop the G-code run with a detailed fatal error; it must not fall back to exact-stop execution. The current implementation still waits for a protected boundary before planning the complete compatible horizon and forces zero speed only at the horizon ends. Rolling immutable-prefix publication, bounded horizon duration, and bounded feed-hold latency remain future work.

Planning diagnostics are NRT-only and report command, `PlanChunk`, G64, and individually planned continuous-command exact-stop counts, lookahead and span high-water marks, planned duration, and last/maximum planning latency. Timed simulation exposes them through `SimulationSnapshot`; do not add them to `MotionBackend`.

The RT-facing contract contains only bounded, allocation-free execution/control/event/snapshot data transported by SPSC channels. `ExecutionItem` is the ordered forward stream and currently contains `PlanChunk`, axis-space `TriggeredMove`, and joint-space `TriggeredJointMove`. Cubic spans provide position as a function of span time and include cached terminal state. Triggered moves contain bounded targets, limits, digital-input IDs and conditions, and no G-code or simulation geometry. Do not add UI strings, TOML objects, G-code entities, synthetic input settings, or debug trajectory storage to `MotionBackend`.

Jogging uses the bounded `MotionBackend` control/event channel rather than synthetic G-code or planned `ExecutionItem` values. Continuous jogs carry a dead-man lease in fixed servo ticks and require renewals bearing the same stable jog token; lease expiry causes a jerk/acceleration-constrained stop. Targets distinguish coordinated axes, atomic joint groups, and individual joints so configured gantries can move coupled or be adjusted independently before homing. Homed targets enforce their configured travel range in the backend; unhomed joint/group jogging remains bounded by its lease and kinematic limits. Axis, joint-group, and individual-joint jogs are mutually exclusive with other motion ownership.

`SimulationWorker` is the sole NRT producer for mock jog controls. Its fixed right-side Jog panel offers continuous and incremental modes, post-home axis jogging, pre/post-home coupled-joint jogging, and an explicitly enabled reduced-speed individual-joint mode. The current UI requests a 20 ms backend lease and renews it every 5 ms while a continuous button remains held; simulation scales the tick count with its speed multiplier so this remains a real-time dead-man interval. Release, panel closure, mode change, application focus loss, or STOP MOTION requests a controlled stop. The UI retains the active jog token until that release/stop path runs; do not clear it from a pre-command presentation snapshot. The backend lease remains authoritative if the UI or worker fails to renew. Do not issue jog controls directly from the GUI to `MotionBackend` or implement jogging as generated G-code.

`MockMotionBackend` implements the same contract without claiming real-time behavior. Its separate `MockTrajectoryDiagnostics` interface records the positions actually calculated on each mock servo tick for focused development tests. Normal/approach positions and controlled-stop positions remain distinct diagnostic spans. Geometry-only Preview neither owns nor renders these samples. That diagnostic path is mock-only and must not be added to the production RT interface.

## Concurrency and lifecycle

`Worker` and `SimulationWorker` each own a persistent `InterpreterSession` on a background thread. Immediate Preview does not use `TrajectoryExecutionDriver`, a trajectory planner, or `MockMotionBackend`: it consumes canonical commands directly into an off-to-the-side `ToolpathRecorder`, captures per-command G64/P and presentation metadata, resolves probe barriers at the emitted probe target, and publishes the completed recorder in one revision. It performs no velocity, acceleration, jerk, timing, packetization, stop-tail, or servo-sample calculation. Timed simulation uses `TrajectoryExecutionDriver` and `MockMotionBackend` with a fixed configured servo timestep and a separate steady-clock scheduler period. Its integer speed multiplier changes the number of fixed servo ticks executed per scheduler wake; it must never enlarge the servo timestep. UI-only strings, WCS/modal metadata, tool overlays, and block lifecycle state remain in NRT presentation storage keyed by epoch/chunk and epoch/span ID and never cross the RT contract. Block completion discovered during execution lookahead is deferred to the chunk that owns the captured active block. Any mutation or traversal shared with the GUI must be protected by the owning worker mutex. Do not hold a GUI-facing mutex while waiting for future physical motion or probe completion.

Timed simulation fills the executor's bounded queue before advancing so dense short-segment paths are not limited to one command per display/update cycle. Queue filling must stop at a probe barrier and resume only after the matching result is delivered. When the executor drains and interpretation can immediately provide more commands, do not impose the normal 8 ms timed wait.

Accelerated timed simulation must not manufacture a queue underflow by consuming an entire multi-packet batch while excluding the NRT producer for one enlarged scheduler batch. `MockMotionBackend::advanceTick()` therefore reports a successful packet continuation through its mock-only API. `SimulationWorker` yields between continued packets, services retirements, and refills the freed slot before executing the remaining fixed servo ticks. Preserve the fixed timestep and sole-NRT-producer rule; do not add this coordination signal to generic `MotionBackend` or weaken the backend rule that a genuinely missing continuation selects the current packet's stop tail. Before leaving timed execution on any terminal driver state, copy the session's chronological status stream into `SimulationSnapshot` so fatal errors remain visible in the GUI as well as `stderr`.

Synchronization is narrow and explicit. Every parameter read publishes a generic interpreter synchronization wait, including unbracketed reads such as `X#5400`; therefore decisions based on memory cannot run ahead of previously emitted motion. M6 explicitly synchronizes before tool-change preparation, `print` synchronizes before becoming externally visible, and probing/homing retain their dedicated completion barriers. Pure bracketed literals/arithmetic (`X[1]`, `X[1+1]`), literal assignments, generic calls, declarations, and control-flow syntax do not synchronize by themselves; parameter reads or explicit physical operations reached inside them still do. `TrajectoryExecutionDriver` stops interpretation at a generic wait, lets all already-published motion drain to a held exact-stop boundary, and only then resumes the evaluator. Consecutive ordinary G-code blocks may still fill the bounded execution queue. The compatibility `InterpreterSession::next()` interface acknowledges generic synchronization immediately because it has no executor; execution consumers use `nextWithBlocks()` through the driver. New operations that consume executor/hardware state or create externally ordered physical effects must add an explicit barrier.

The Windows mock scheduler is paced independently from the servo tick. Preserve its deadline-miss, maximum-wake-lateness, maximum-tick-execution, and servo-tick counters in NRT presentation state. Focused backend diagnostics must continue to record positions actually calculated at the fixed servo period; shortening `servo_period` must therefore produce shorter diagnostic segments. Do not reconstruct mock servo samples in the UI or add these diagnostics to `MotionBackend`.

Program execution must not append to a previous run or start from its final position. Keep the reset regression tests intact.

`InterpreterSession::next()` is the compatibility incremental non-real-time interface. It returns commands, probe waits, completion, or errors while skipping UI lifecycle records. `nextWithBlocks()` additionally returns source-aware `InterpreterBlockLifecycle` entered/completed records. Every execution has a stable ID, source name, line, and text. Completion is emitted only after nested evaluator work returns; this is generic call lifecycle behavior, not a tool-change UI special case.

Exceptions raised while processing evaluator messages on the consumer thread must be caught by `InterpreterSession` and returned as `InterpreterError`. Do not allow invalid G-code or modal state to escape into a Visual C++ assertion, `std::exit`, or GUI-thread termination.

Timed simulation must stop immediately on an interpreter error and discard active/queued simulated motion. Its GUI snapshot is deliberately lightweight and uses compact per-source completed-line flags rather than copying every completed block and its strings each frame.

A consumer must return a matching `ProbeResult` with `provideProbeResult()` before evaluation can pass a probe barrier.

## Triggered motion, probing, and homing

The core rule is that HAL must not call `Machine` directly:

```text
Machine emits ProbeMove -> driver publishes TriggeredMove -> real-time executor/HAL
    -> driver translates TriggeredMoveCompleted to ProbeResult -> Machine resumes
```

`TriggeredMove` is a generic signal-terminated point-to-point primitive intended for both probing and future homing phases. The executor owns jerk-limited approach generation, samples the selected digital input every servo cycle, latches the complete `MotionState` when the requested condition occurs, and then owns the jerk/acceleration-limited stop. `TriggeredMoveCompleted` reports distinct trigger and stopped states, or `ReachedTarget`, `Aborted`, or `Fault`. It must never contain probe-specific tool geometry or interpreter state.

Machine coordinates XYZABC are logical axes, not physical motors. A physical backend owns its configured axis-to-joint mapping; ordinary axis-space motion may therefore drive multiple joints from one logical axis while applying retained squaring offsets. Do not duplicate a logical axis in `position_t` or expose motor topology to the interpreter or trajectory planner.

`TriggeredJointMove` is reserved for homing, gantry squaring, and service operations that must temporarily decouple those joints. It uses a fixed joint mask, fixed-capacity joint vectors, one input binding per participating joint, and an explicit absolute/relative target mode. Initial homing search and pull-off moves use relative targets because machine coordinates are not established yet. Each joint independently latches its trigger state and performs a constrained stop while other joints continue. `TriggeredJointMoveCompleted` reports the triggered mask plus per-joint trigger and stopped states. After all joints are stationary, `SetJointPositionRequest` may establish joint home/squaring offsets only while the backend is held. Coordinate soft limits become meaningful only after the corresponding joints have been homed.

The interpreter pauses at a probe barrier because later blocks may read `#5061` through `#5070`. `TrajectoryExecutionDriver` retains that interpreter concern outside the RT contract and translates generic completion data into the matching `ProbeResult`.

Timed simulation uses `MockMotionBackend`. Its mock-only synthetic digital-input transition accounts for selected physical tool offset even when G43 is inactive. It changes only the simulated input state; it does not provide a predicted stop point to the executor. The mock backend detects the transition while executing, latches the sampled state, and continues through its constrained stop, so trigger and stopped positions normally differ. Immediate Preview does not synthesize physical contact: it records the canonical `ProbeMove` through normal active-offset display conversion and returns a `Triggered` result whose trigger and stopped positions are exactly the emitted probe target. This synthetic signal configuration is deliberately absent from `MotionBackend`.

Mock homing is sequenced outside the servo loop in configured group order: a fast debounced active-switch search, a switch-ignoring move to `backoff_distance` behind the latched fast-trigger position, verification that the final backoff position cleared the switch, a slow debounced active-switch search, held-state joint coordinate calibration from the slow trigger positions, and a grouped final move to `home_position`. Backoff therefore includes recovery from any constrained-stop overshoot and remains stable when search speed changes. The simulated machine powers up at X6 Y6 Z-6. Multi-motor gantries use `TriggeredJointMove` so every motor stops on its own switch. Homing policy and coordinate mutation remain outside the triggered-move item itself.

Toolpath preview geometry is canonical program geometry: it is derived from emitted command coordinates plus G53/active work/tool-offset semantics. Do not shift retained toolpath lines using physical tool geometry. Physical tool length and diameter belong to the simulated tool overlay and synthetic contact behavior.

`TrajectoryExecutionDriver` associates command presentation metadata with epoch/chunk identity outside the RT contract. G53 commands retain the modal WCS metadata even though their motion bypasses the work offset. `ToolpathRecorder` retains distinct WCS frames used by preview motion; timed simulation applies WCS/modal metadata only when the corresponding queued command becomes active, not when the interpreter reads ahead.

## G64 spline geometry and execution

`ToolpathRecorder` retains the active G64 flag and optional machine-unit P value alongside each command without changing `MachineCommand`. Preview rebuilds the piecewise geometry in revision-cached display storage. The NRT executable planner independently constructs the same primitive layout and compiles it into fixed-capacity axis-polynomial `PlanChunk` spans; G-code entities, spline controls, and preview geometry never enter `MotionBackend`.

Preserve exact canonical lines and arcs outside local blend neighborhoods. G64 P is a control-spacing/blend-scale parameter for this experiment, not a strict maximum-deviation tolerance. Each compatible entity receives the local scale

```text
p_entity = min(P, entity_arc_length / 6)
```

Every compatible junction uses one clamped spline with exactly six controls. Its clamped endpoints are sampled `3*p_incoming` before and `3*p_outgoing` after the junction. On lines, its remaining controls lie exactly at incoming `2*p_incoming` and `p_incoming`, then outgoing `p_outgoing` and `2*p_outgoing`. For arcs, the interior controls are positioned from the clamped endpoint tangent and curvature so the spline is curvature-continuous with the retained arc. The tangent-handle length is fitted from the entity position two control steps into the blend, reducing unnecessary departure from the retained arc without sacrificing endpoint curvature. The junction itself is not a control point. Adjacent junction blends use the same local scale on their shared entity.

An entity longer than `6P` retains an exact middle section. For an entity of arc length at most `6P`, the two trims meet at its arc-length midpoint, so the neighboring junction splines replace it continuously without a separate cluster fitter. Different incoming and outgoing entities may and normally do contribute different local scales to one junction.

Every displayed junction spline is a clamped degree-three B-spline with exactly six controls and knot vector `[0,0,0,0,1,2,3,3,3,3]`. Do not evaluate the six controls as a degree-five Bezier curve, expose knot-inserted Bezier controls, or restore tolerance fitting or recursive cluster fitting.

The overlay conventions are:

- Simple junction splines are one-pixel magenta.
- Junction splines involving at least one entity of length `<= 6P` are one-pixel cyan.
- Fitted control points are pale magenta and the control polygon is a thin dashed pale-magenta line.
- G64 arc display tessellation is scale-adaptive so an analytically C2 spline/arc junction does not appear displaced from a coarse rendered arc.

Rapids, explicit G53 moves, probes, non-G64 motion, P changes, discontinuous endpoints, protected presentation changes, and non-motion commands are protected boundaries and must not be blended across. Reject only zero-length entities or discontinuous adjacent endpoints at the shared geometry level; do not add angle, curvature, or locality rejection without a demonstrated failing path. If execution cannot prove bounded capacity, geometry, continuity, or safety, it must stop the G-code run and report a detailed fatal error; it must not substitute exact-stop execution.

## UI toolpath conventions

- The primary layout is a top action toolbar, horizontally resizable G-code pane on the left, central OpenGL viewport, and vertically resizable status pane along the bottom. Do not reintroduce the old File menu or separate Program/Messages windows.
- The G-code pane defaults to Edit. Its fixed header contains Edit, Compile, Save, and the filename; only the source child scrolls. Compile is enabled only in Edit mode, Edit only in compiled mode, and Preview/Simulate only in compiled mode. Editing marks the main program dirty; saving writes the main program file.
- The compiled view is read-only and highlights blocks by source file and line using generic entered/completed lifecycle records. `SimulationSnapshot::activeBlocks` is a nested execution stack: the top entry is yellow/current, suspended parents are amber, and blocks turn green only after return. The GUI uses `completedLineFlags`; do not restore per-line scans of the full `completedBlocks` history.
- Main-program source lines are cached when a file is loaded or edited. The compiled pane must use `ImGuiListClipper` and submit only visible rows; do not split, trim, format, or submit every source line on every frame.
- Parser errors, chronological interpreter print/error messages, file errors, simulation errors, and simulation/tool status belong in the bottom status pane. The header shows the active modal G-codes right-aligned; during timed playback these are the modes captured with the active command rather than interpreter read-ahead state.
- Viewport navigation applies only when the cursor is inside the central viewport and ImGui is not capturing the mouse. Scrolling or dragging floating Parameters/Tool Table windows over the viewport must not zoom, pan, or rotate the 3D view.
- Feed lines are blue; arcs are green; G38.3 probe moves are orange.
- Explicit G53 line moves are yellow.
- Rapid G0 lines use the normal coordinate-system color. Blue rapid lines use four-pixel dash/gap runs; yellow G53 rapid lines use ten-pixel runs. Yellow rapids render before blue rapids so blue remains visible where they overlap; rapid batches render after solid geometry.
- Toolpath lines are two pixels wide with line/point smoothing and alpha blending enabled. Line endpoints and dark-green arc endpoints are three pixels.
- Preview rendering is revision-cached. After `ToolpathRecorder` settles, commands are converted once into batched vertex arrays for feed, rapid, G53, arc, probe, spline/control, and endpoint geometry; arcs are tessellated once and the combined static preview data is uploaded to a GPU buffer. Normal frames use a small number of buffered `glDrawArrays` calls and must not traverse the command stream, retessellate arcs, stream the complete preview repeatedly from CPU memory, or hold the worker mutex while drawing.
- MCS is always shown. The final/current WCS is prominent; other WCS frames actually used by retained preview motion are dimmed and labeled. Timed simulation emphasizes the WCS captured with the active playback command.
- The viewport FPS overlay is anchored to the bottom-right of the resizable 3D viewport, not the application window.
- The simulated tool is a yellow wireframe cone with its diameter circle and center point at the wide spindle end and its tip at the physical cutter position.
- When no tool is loaded, show the machine position as a world-space three-axis crosshair instead of inventing placeholder tool geometry. It must enlarge normally when the view zooms in. The status pane always reports current MCS XYZ, including the calibrated position after homing.

## Near-term work

MDI is currently connected only to simulated execution. Preserve a target-selection boundary: the intended future UI has explicit Real and Simulated MDI modes. Entering Simulated mode will snapshot the authoritative real-machine state into an isolated simulation branch; returning to Real mode will discard that branch without merging simulated pose, modal state, offsets, parameters, or tool state back into the real machine. Do not let this future mode switch bypass executor ownership, probe barriers, or command-boundary synchronization.

### Current executable G64 handoff (updated 2026-07-14)

The first executable G64 implementation remains an incomplete development checkpoint. Investigation of the partial preview and timed-simulation stall found three concrete issues:

- Failed continuous compilation was silently converted into one-command exact-stop execution, hiding the real failure and allowing incomplete behavior to look successful.
- Held-state recovery called `BoundedLookaheadTrajectoryPlanner::reset()`, which discarded an unpublished G64 window. Later commands then began ahead of the planner's retained position.
- Algebraically C2 three-span chains lost acceleration precision by converting short local controls through absolute machine coordinates before forming polynomial coefficients. Real failures had exact position continuity, velocity jumps around `1e-11`, and acceleration jumps around `1e-7` to `1e-6`.

The current worktree removes failure fallback, reports fatal planner/driver/interpreter errors to the UI status stream and `stderr`, preserves buffered horizons while rebasing held-state kinematics, and forms chain coefficients directly from local states. Continuous compilation now stages spans independently of RT capacity, partitions them into chained 256-span packets, generates and verifies moving-boundary stop tails under the existing 16-span bound, and publishes command presentation with the packet that owns its activation span. It also replaces the window-minimum-feed/global-stretch law with per-piece feed and geometry caps, jerk-aware forward/backward station-speed passes, and local Ruckig solves. Local timing can now carry shared nonzero scalar acceleration across artificial geometry-piece boundaries, but keeps the full zero-acceleration horizon when that candidate is not faster. A focused dense line/arc fixture records that very small-P cubic C2 blends can have real `q'''(s)v^3` jerk ceilings near one quarter of programmed speed; acceleration-aware timing cannot legally erase those geometry limits. A focused large-coordinate regression protects the `1e-7` C2 gate; a 150-command compiler regression exceeds 256 spans; an 800-command timed regression crosses the backend queue repeatedly; and feed-change regressions protect arithmetic-mean spline caps and prevent a distant F1 entity from throttling an earlier F80 prefix.

Pick up here:

1. Replace velocity-only reachability with acceleration-aware forward/backward `(s,v,a)` reachability and adaptive timing stations. Add a fixture that proves a duration improvement where acceleration carry is physically useful.
2. Turn that whole-horizon timing into a rolling mutable suffix and publish only a stop-proven immutable prefix. Do not reintroduce a semantic command-count boundary.
3. Bound NRT horizon duration/resource use and feed-hold latency while preserving nonzero motion across rolling horizon boundaries.
4. Add the still-missing end-to-end regression containing a real probe followed by enough G64 lines/arcs to cross multiple rolling horizons. Assert final completion, exact recorder counts, packet acceptance/retirement counts, endpoint, and cleared barriers.
5. Measure stop-tail span use on real programs. Redesign its bounded RT storage only if the current 16 spans are demonstrably insufficient.
6. Verify terminal/UI fatal errors manually in immediate preview and timed simulation with the real programs.
7. Keep the approved geometry invariant: exact retained line/arc middles plus only local six-control clamped degree-three B-splines. Do not solve capacity or timing problems by reverting to a whole-path spline fit.

The separately configured Windows AddressSanitizer test executable currently aborts with a stack overflow before entering `main`; the normal Clang/Ninja application build and CTest suite pass. Investigate that sanitizer configuration separately unless it becomes useful for the runtime stall.

Likely next steps are:

1. Finish visually and numerically validating executable piecewise G64 geometry across lines, rounded-IJK arcs, helical/non-XY arcs, short-entity midpoint joins, and protected boundaries.
2. Introduce moving-boundary immutable prefixes with a capacity-proven stop continuation, then bound planning horizon duration and feed-hold latency.
3. Profile and reduce whole-horizon planning work while the rolling planner is introduced; `adaptive_pockets.ngc` currently plans its 5,164-command window in about 22.75 seconds.
4. Extend planning diagnostics with queue reserve and constraint margins. Later compare representative paths against a development-only offline optimizer before pursuing extra time-optimal complexity.
5. Add a physical implementation of `MotionBackend` using the existing bounded SPSC execution/control/event/snapshot contract, including physical digital-input sampling for `TriggeredMove`, and define abort, fault, cancellation, and pending-trigger behavior before connecting hardware.
6. Expand G38 support beyond G38.3 and test unsuccessful, aborted, and faulted probe results; later add R-form arcs and richer canonical records.

Shared geometric fitting does not reduce executable proof obligations: executable G64 must independently prove ordered path tolerance, per-axis constraints, C2 time-domain continuity, bounded capacity, and stop safety.

## Worktree discipline

Preserve unrelated user changes in the dirty worktree. Verify architectural claims against current code rather than relying on deleted or historical Markdown notes.
