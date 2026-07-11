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

## Important current decisions

- `Machine::Unit` is the fixed internal machine unit: inch or millimetre.
- G20/G21 values are converted into the configured machine unit as blocks are consumed.
- Scale X/Y/Z, I/J/K, G94 F, and G10 linear offsets. Do not scale A/B/C or selector/index words.
- Tool-table and persistent coordinate-offset values are already stored in configured machine units and must not be rescaled when G20/G21 changes.
- `beginProgramRun()` clears transient pose, modal state, and tool offset while preserving parameter memory and the tool table. `Machine` does not retain generated commands; each consumer owns any retained representation.
- G43 currently applies the full XYZABC tool-table offset. G49 clears it.
- G53 bypasses work-coordinate offsets but retains the active tool offset.
- `MoveLine::machineCoordinates()` records whether a line was emitted by explicit G53 motion. Preserve this metadata through consumers.
- `InterpretationMode` has three distinct `_task` values: Preview = 0, Simulation = 1, and RealRun = 2. `_task` is read-only at parameter 6000.
- IJK arcs allow either in-plane center word to be omitted; an omitted word means zero. At least one applicable center word is required.
- G91.1 uses incremental IJK center coordinates. G90.1 uses absolute center coordinates in the active work coordinate system without changing G90/G91 endpoint mode.
- IJK arc validation rejects zero radius and radius mismatch beyond `Machine::arcTolerance()`. The tolerance is 0.0005 inch or the equivalent 0.0127 mm.
- Arc preview uses a directed sweep derived from the signed axis. It supports CW/CCW major arcs, full circles, and helical interpolation. It blends from both radial arms so rounded G-code arcs end exactly at the commanded endpoint.
- Rapid `MoveLine` commands currently use speed `-1` as a sentinel; this is not a physical negative feedrate.

## Concurrency and lifecycle

`Worker` and `SimulationWorker` each own a persistent `InterpreterSession` on a background thread. Both use `ExecutionDriver` and `SimulationExecutor`; preview calls `completeQueued()` for immediate pacing, while timed simulation advances from a steady clock and playback-rate multiplier. Any mutation or traversal shared with the GUI must be protected by the owning worker mutex. Do not hold a GUI-facing mutex while waiting for future physical motion or probe completion.

Timed simulation fills the executor's bounded queue before advancing so dense short-segment paths are not limited to one command per display/update cycle. Queue filling must stop at a probe barrier and resume only after the matching result is delivered. When the executor drains and interpretation can immediately provide more commands, do not impose the normal 8 ms timed wait.

Program execution must not append to a previous run or start from its final position. Keep the reset regression tests intact.

`InterpreterSession::next()` is the incremental non-real-time interface. It returns a command, `InterpreterWaitingForProbe`, completion, or an interpreter error. A consumer must return a matching `ProbeResult` with `provideProbeResult()` before evaluation can pass a probe barrier.

## Probing direction

The core rule is that HAL must not call `Machine` directly:

```text
Machine emits ProbeMove -> real-time executor/HAL -> executor returns ProbeResult -> Machine resumes
```

The real-time executor must latch the position at the probe signal transition and report it separately from the final stopped position. The interpreter pauses at a probe barrier because later blocks may read `#5061` through `#5070`.

Current preview and timed simulation both use `SimulationExecutor`. Its preview-only synthetic probe contact accounts for the selected physical tool offset even when G43 is inactive, reports matching trigger/stopped machine positions, and lets the interpreter resume. This is simulation behavior, not a physical-execution implementation. A physical executor must sample the probe input, latch the transition position, stop motion, and return a real `ProbeResult`.

Toolpath preview geometry is canonical program geometry: it is derived from emitted command coordinates plus G53/active work/tool-offset semantics. Do not shift retained toolpath lines using physical tool geometry. Physical tool length and diameter belong to the simulated tool overlay and synthetic contact behavior.

## UI toolpath conventions

- Feed lines are blue; arcs are green; G38.3 probe moves are orange.
- Explicit G53 line moves are yellow.
- Rapid G0 lines use the normal coordinate-system color but are dashed with six-pixel dash/gap runs.
- Toolpath lines are two pixels wide with line/point smoothing and alpha blending enabled.
- The simulated tool is a yellow wireframe cone with its diameter circle and center point at the wide spindle end and its tip at the physical cutter position.

## Near-term work

Likely next steps are:

1. Add a motion/planner consumer that owns bounded command buffering and supplies real probe results to `InterpreterSession`; reuse the `ExecutionDriver` barrier contract without treating `SimulationExecutor` as a real executor.
2. Define abort, fault, and cancellation behavior for a pending probe and its evaluator thread.
3. Expand G38 support beyond G38.3 and test unsuccessful, aborted, and faulted probe results.
4. Later add R-form arcs, G64/path-control semantics, richer canonical records, and trajectory planning.

## Worktree discipline

Preserve unrelated user changes in the dirty worktree. Verify architectural claims against current code rather than relying on deleted or historical Markdown notes.
