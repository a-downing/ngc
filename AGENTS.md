# NGC Development Context

## Project role

This repository is a C++23 non-real-time CNC front end. Its current pipeline is:

```text
lexer -> parser/AST -> InterpreterSession/evaluator -> modal Machine -> MachineCommand stream -> consumer(s)
```

`InterpreterSession` incrementally evaluates the program and emits one `MachineCommand` at a time. `ExecutionDriver` connects an interpreter session to `SimulationExecutor` and owns command pumping and the probe-result barrier handshake. `Worker` runs immediate preview and retains a `ToolpathRecorder`; `SimulationWorker` runs timed playback and exposes `SimulationSnapshot` state to the OpenGL UI. Preview and timed simulation intentionally remain separate retained products while sharing the same executor/probing path.

This is not yet a trajectory planner, real-time executor, or HAL component. Preserve the separation between interpretation, planning, real-time execution, and hardware access.

## Build environment

The active development environment is Windows with Clang, Ninja, CMake, and vcpkg. GLM is supplied by vcpkg; GLFW and ImGui are Git submodules.

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
- Arc preview uses a directed sweep derived from the signed axis. It supports CW/CCW major arcs, full circles, and helical interpolation. It blends from both radial arms so rounded G-code arcs end exactly at the commanded endpoint.
- Rapid `MoveLine` commands currently use speed `-1` as a sentinel; this is not a physical negative feedrate.
- G1, G2, and G3 require an established positive modal feedrate. A missing/non-positive feedrate must produce an `InterpreterError`; never dereference an absent modal `F` value or terminate the process.
- `PANIC` is reserved for internal invariants, impossible enum/state combinations, and other program logic errors. Unsupported G/M codes, unsupported words or modes, malformed arcs, missing tools, invalid G10 operations, and other program-input failures must throw through the recoverable interpreter-error path.
- `InterpreterSession` retains one chronological typed status stream containing `Print` and `Error` entries. Do not split terminal interpreter errors back into a separate print/error collection or regroup messages by severity in the UI. Prints are blue and prefixed `PRINT:`; errors are red and prefixed `ERROR:`.
- Errors associated with a source statement or block must include source name, line, and column. Statement evaluation and consumer-thread block processing both add location context without duplicating an existing location prefix.

## Concurrency and lifecycle

`Worker` and `SimulationWorker` each own a persistent `InterpreterSession` on a background thread. Both use `ExecutionDriver` and `SimulationExecutor`; preview calls `completeQueued()` for immediate pacing, while timed simulation advances from a steady clock and playback-rate multiplier. Any mutation or traversal shared with the GUI must be protected by the owning worker mutex. Do not hold a GUI-facing mutex while waiting for future physical motion or probe completion.

Timed simulation fills the executor's bounded queue before advancing so dense short-segment paths are not limited to one command per display/update cycle. Queue filling must stop at a probe barrier and resume only after the matching result is delivered. When the executor drains and interpretation can immediately provide more commands, do not impose the normal 8 ms timed wait.

Program execution must not append to a previous run or start from its final position. Keep the reset regression tests intact.

`InterpreterSession::next()` is the compatibility incremental non-real-time interface. It returns commands, probe waits, completion, or errors while skipping UI lifecycle records. `nextWithBlocks()` additionally returns source-aware `InterpreterBlockLifecycle` entered/completed records. Every execution has a stable ID, source name, line, and text. Completion is emitted only after nested evaluator work returns; this is generic call lifecycle behavior, not a tool-change UI special case.

Exceptions raised while processing evaluator messages on the consumer thread must be caught by `InterpreterSession` and returned as `InterpreterError`. Do not allow invalid G-code or modal state to escape into a Visual C++ assertion, `std::exit`, or GUI-thread termination.

Timed simulation must stop immediately on an interpreter error and discard active/queued simulated motion. Its GUI snapshot is deliberately lightweight: the full completed `BlockExecution` history remains available from `SimulationExecutor::snapshot()`, while `SimulationWorker` uses compact per-source completed-line flags rather than copying every completed block and its strings each frame.

A consumer must return a matching `ProbeResult` with `provideProbeResult()` before evaluation can pass a probe barrier.

## Probing direction

The core rule is that HAL must not call `Machine` directly:

```text
Machine emits ProbeMove -> real-time executor/HAL -> executor returns ProbeResult -> Machine resumes
```

The real-time executor must latch the position at the probe signal transition and report it separately from the final stopped position. The interpreter pauses at a probe barrier because later blocks may read `#5061` through `#5070`.

Current preview and timed simulation both use `SimulationExecutor`. Its preview-only synthetic probe contact accounts for the selected physical tool offset even when G43 is inactive, reports matching trigger/stopped machine positions, and lets the interpreter resume. This is simulation behavior, not a physical-execution implementation. A physical executor must sample the probe input, latch the transition position, stop motion, and return a real `ProbeResult`.

Toolpath preview geometry is canonical program geometry: it is derived from emitted command coordinates plus G53/active work/tool-offset semantics. Do not shift retained toolpath lines using physical tool geometry. Physical tool length and diameter belong to the simulated tool overlay and synthetic contact behavior.

`ExecutionDriver` captures the active WCS name/offset and active modal G-code set with each command. G53 commands retain the modal WCS metadata even though their motion bypasses the work offset. `ToolpathRecorder` retains distinct WCS frames used by preview motion; timed simulation applies WCS/modal metadata only when the corresponding queued command becomes active, not when the interpreter reads ahead.

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
- Preview rendering is revision-cached. After `ToolpathRecorder` settles, commands are converted once into batched CPU vertex arrays for feed, rapid, G53, arc, probe, and endpoint geometry; arcs are tessellated once. Normal frames use a small number of `glDrawArrays` calls and must not traverse the command stream, retessellate arcs, or hold the worker mutex while drawing.
- MCS is always shown. The final/current WCS is prominent; other WCS frames actually used by retained preview motion are dimmed and labeled. Timed simulation emphasizes the WCS captured with the active playback command.
- The viewport FPS overlay is anchored to the bottom-right of the resizable 3D viewport, not the application window.
- The simulated tool is a yellow wireframe cone with its diameter circle and center point at the wide spindle end and its tip at the physical cutter position.

## Near-term work

MDI is currently connected only to simulated execution. Preserve a target-selection boundary: the intended future UI has explicit Real and Simulated MDI modes. Entering Simulated mode will snapshot the authoritative real-machine state into an isolated simulation branch; returning to Real mode will discard that branch without merging simulated pose, modal state, offsets, parameters, or tool state back into the real machine. Do not let this future mode switch bypass executor ownership, probe barriers, or command-boundary synchronization.

Likely next steps are:

1. Add a motion/planner consumer that owns bounded command buffering and supplies real probe results to `InterpreterSession`; reuse the `ExecutionDriver` barrier contract without treating `SimulationExecutor` as a real executor.
2. Define abort, fault, and cancellation behavior for a pending probe and its evaluator thread.
3. Expand G38 support beyond G38.3 and test unsuccessful, aborted, and faulted probe results.
4. Later add R-form arcs, G64/path-control semantics, richer canonical records, and trajectory planning.

## Worktree discipline

Preserve unrelated user changes in the dirty worktree. Verify architectural claims against current code rather than relying on deleted or historical Markdown notes.
