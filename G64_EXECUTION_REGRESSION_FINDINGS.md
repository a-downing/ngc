# Executable G64 regression findings

Date investigated: 2026-07-14
Checkpoint: `71a4e73` (`Checkpoint executable G64 spline planning`)

## Executive summary

The partial immediate preview and the probe-followed timed-simulation stop are reproducible planner failures. The primary observed failure is not a stale probe barrier, a backend stop-branch selection, or missing iteration in `TrajectoryExecutionDriver::observePlanned()`.

Real G64 programs produce very small acceleration differences at adjacent polynomial-span boundaries. Position is exactly continuous and velocity differs only around `1e-11`, but acceleration differs around `4.8e-7` to `1.0e-6`. `BoundedLookaheadTrajectoryPlanner::verifyContinuousNormalMotion()` applies a fixed absolute C2 acceleration tolerance of `1e-7`, so it rejects these plans with:

```text
continuous plan is not C2 at a polynomial boundary
```

At the investigated checkpoint, compilation failures were silently converted into one-command exact-stop execution while post-compilation verification failures were fatal. The project policy has since been changed: every failed continuous-motion compilation or proof is fatal and must stop the G-code run with detailed UI and terminal diagnostics. There is no failed-G64 exact-stop fallback.

## Resolution implemented after the investigation

The current worktree contains these follow-up changes:

- Continuous compilation failures no longer call `planOne()`; compilation, C2, capacity, constraint, and stop-branch failures terminate the run.
- Fatal driver and interpreter errors enter the chronological UI status stream and print to `stderr`.
- G64 errors include the window size and P value, first/last source blocks, endpoints, chunk/span counts, boundary classification, full motion states, measured jumps, tolerances, and detailed capacity/recursive-verification context where applicable.
- Held-state recovery rebases exact planner kinematics without clearing the buffered lookahead window. An invariant error stops execution rather than discarding or reordering commands if pending and buffered plans overlap unexpectedly.
- The `1e-7` acceleration failure was traced to numerical cancellation. The algebraically C2 chain constructed local controls, added them to absolute machine coordinates, and then subtracted those nearly equal absolute values while generating coefficients. Short durations amplified the lost precision in acceleration.
- Polynomial coefficients are now generated directly from local control offsets. Only the span origin and canonical endpoint are stored in absolute coordinates.
- A large-coordinate C2 regression and a 70-command held-recovery regression protect these fixes.

With those changes, `1001.ngc` initially passed the formerly failing C2 boundaries and exposed the next genuine failure: 255 existing spans plus a required three-span chain could not fit the old one-chunk result.

The first multi-packet implementation now stages the complete verified normal span stream in NRT storage, partitions it across chained 256-span `PlanChunk` packets, generates a complete Ruckig moving-state stop tail for every internal packet boundary, and publishes command presentation with the packet that owns each activation span. The fixed 32-command flush has been removed. A 70-command regression now crosses both the old command boundary and the normal-span capacity in one blended horizon, while the strengthened `1001.ngc` preview regression completes without a status error.

This is meaningful progress but not the final rolling planner. The current implementation waits for a protected boundary, synchronization point, or interpreter completion before timing and committing the compatible horizon. It does not yet publish an immutable prefix while retaining a mutable forward/backward-planned suffix. A very long uninterrupted G64 run can therefore consume unbounded NRT horizon memory and delay initial publication; rolling prefix commitment remains the next trajectory-planning task.

The window-minimum-feed/global-stretch timing law has now also been removed. Retained primitive pieces use their own programmed feed, junction splines use the arithmetic mean of their incoming and outgoing feeds, sampled geometry supplies local physical caps, and jerk-aware forward/backward passes establish reachable station speeds. Each piece is timed by a local Ruckig solve. Exact emitted-polynomial constraint failures reduce only the owning piece's local limits and rerun reachability; they no longer slow the complete G64 horizon uniformly.

### Timed-simulation queue starvation found after multi-packet publication

Manual `adaptive_pockets.ngc` simulation exposed a separate orchestration failure after planning a 115-packet batch:

```text
held-state recovery reached a partially published continuous packet batch at packet 8 of 115
```

The packet number matched the mock backend's eight-slot plan capacity. During one accelerated fixed-tick scheduler batch, the executor could consume all eight resident packets while `SimulationWorker` deliberately avoided concurrent NRT backend access. Packet 8 then reached its branch without packet 9, correctly selected its guaranteed stop tail, and reported `BackendHeld`. The driver's fatal guard also behaved correctly: silently discarding packets 9 through 115 would have lost G-code trajectory state.

Timed mock playback now treats every successful chunk continuation as a mock-only NRT refill point. `MockMotionBackend::advanceTick()` reports that a packet boundary was crossed; during an accelerated scheduler batch the executor temporarily yields ownership, the sole NRT producer services retirement events and fills the newly freed slot, and the executor then continues the remaining fixed servo ticks. This does not enlarge the servo timestep, weaken the stop-branch rule, or add queue diagnostics/control to the generic `MotionBackend` contract. A real RT backend must still receive sufficient time-horizon reserve from the future rolling planner.

Immediate Preview originally remained affected after that timed-simulation fix because `Worker` had a separate single-threaded backend drain path. It published the eight packets that fit, called `MockMotionBackend::runUntilIdle()`, and did not regain control until packet 8 had already selected its stop tail. An interim continuation-yield fixed that queue starvation, but immediate Preview no longer executes trajectory packets at all; the geometry-only design below supersedes the interim fix.

The GUI visibility defect had a separate cause. `SimulationWorker` could observe driver `Error` and leave its loop before copying the session's newly appended chronological status messages into `SimulationSnapshot`. It now copies the stream before terminal-state handling. A synthetic fatal G64 regression asserts that the same error text reaches both `SimulationSnapshot::error` and its GUI status stream.

Regression coverage includes an 800-command continuous trajectory through both modes. At 1000x timed playback it produces more packets than the eight-slot queue and must complete without a stop selection or error. Immediate Preview must retain all 800 canonical commands and their G64/P metadata through its backend-free geometry path. A separate end-to-end regression now previews the unmodified `adaptive_pockets.ngc`, including its tool-change probes, and requires canonical G64 geometry. Completing that entire program in timed simulation is intentionally not an ordinary CTest assertion yet: the 5,164-command executable horizon currently takes about 22.75 seconds merely to plan, which is the remaining rolling-prefix problem rather than a useful unit-test runtime.

## Chosen next architecture: rolling span stream and chained bounded PlanChunks

The fixed 256-span limit is part of the RT-facing `PlanChunk` contract, not a preview-only limit. Each `AxisPolynomialSpan` contains the timed six-axis cubic coefficients and cached terminal motion state. A complete `PlanChunk` is published as an `ExecutionItem`; the mock backend copies it into a preallocated plan slot and sends only that slot's index through its SPSC queue. Increasing the constant would enlarge every slot and every publication copy without providing a general bound for complicated geometry.

The selected solution is therefore to retain the fixed per-chunk capacity, remove the fixed 32-G-code semantic window, and incrementally turn compatible G64 input into a rolling verified span stream. `PlanChunk` is an RT transport packet, not a G-code grouping or speed-planning boundary.

1. Read compatible G64 commands incrementally and finalize each local junction after its following entity is known. Earlier local geometry does not depend on an arbitrary command-count boundary.
2. Maintain an NRT rolling planning horizon containing enough future geometry to calculate feasible forward motion and a guaranteed stop. Retain a mutable suffix while additional lookahead can still change its speeds.
3. Commit only a verified immutable polynomial prefix. Accumulate that prefix into consecutive `PlanChunk::normalMotion` packets, splitting at any verified C2 span boundary when the 256-span array fills. A command, arc, spline, or local three-span chain may cross a packet boundary.
4. Preserve the exact moving position, velocity, and acceleration at every packet boundary and assign consecutive branch identities so the backend selects the next queued chunk without stopping.
5. Give every published chunk its own independently verified stop branch beginning at its immutable moving boundary state. The complete stop is stored in that chunk's separate `stopTail`, even when it travels over geometry that normal execution represents in later chunks.
6. Preserve command activation span IDs and presentation ownership across chunk boundaries. A G-code command becomes active when its owning span is reached, not merely when a batch containing it is published.
7. Publish only after the committed prefix, packetization, normal-motion continuity, stop branches, capacities, constraints, and metadata association have all been proved. Never publish a prefix whose guaranteed stop depends on another item arriving later.
8. Keep every inability to prove requested continuous motion fatal and information-rich. Multi-chunk output is the requested G64 trajectory, not an exact-stop fallback.

The existing backend continuation protocol already expresses the required runtime choice: when a normal branch finishes and the matching successor is queued, execution continues into it; otherwise the backend selects that chunk's stop tail and reaches a held state. The missing work is the rolling forward/backward trajectory calculation, planner-side moving-boundary stop generation, and driver-side ownership/publication of consecutive packets.

The current 16-span `stopTail` storage remains unchanged until measurements show that it is insufficient. Internal moving boundaries still require complete jerk-limited braking motion rather than the current stationary one-span hold, and a chunk is publishable only if that entire stop fits and verifies. If real stops exceed 16 spans, planning remains fatal with measured capacity diagnostics; expanding the stop-tail storage or RT representation is deliberately deferred because it is simpler than, and should be driven by, the trajectory mathematics.

## Current preview algorithm

Immediate Preview is an interpretation and geometry operation, not motion execution:

```text
InterpreterSession
  -> canonical MachineCommand stream
  -> off-to-the-side ToolpathRecorder
  -> one completed recorder publication
  -> revision-cached Application preview batches
```

It deliberately does not construct timed axis polynomials, enforce velocity/acceleration/jerk limits, packetize `PlanChunk` values, generate stop tails, publish `ExecutionItem` values, or run `MockMotionBackend`. Those operations remain mandatory for Simulation and future real execution but add no preview geometry.

`Worker` captures active tool offset, WCS, G64 state, and optional P with each emitted command. It assembles a private recorder while interpretation runs, then replaces the GUI-visible recorder once so the render cache does not repeatedly rebuild a growing partial program. Generic interpreter synchronization is acknowledged by the compatibility preview consumer because no physical executor state exists.

For a `ProbeMove`, Preview records the canonical probe geometry and immediately returns a matching `Triggered` result with both trigger and stopped positions equal to the emitted probe target. It does not use selected physical tool length to predict contact and does not create mock servo samples. This allows later blocks and probe-parameter reads to be interpreted consistently while drawing the requested target endpoint.

`ToolpathRecorder` stores canonical `MachineCommand` values plus per-command G64-active and P-value metadata. The application groups compatible recorded G64 feed lines and arcs, flushing on rapid motion, explicit G53, probes, non-motion commands, non-G64 motion, and P changes.

For each compatible junction, preview does the following:

1. Convert the incoming and outgoing commands into arc-length entities that provide position, tangent, and curvature.
2. Compute the local entity scales:

   ```text
   p_entity = min(programmed P, entity length / 6)
   ```

3. Sample the incoming endpoint at `3*p_incoming` before the canonical junction and the outgoing endpoint at `3*p_outgoing` after it.
4. Fit exactly six controls using endpoint position, tangent, curvature, and the entity sample two control steps into each side.
5. Evaluate those controls as a clamped degree-three B-spline with knot vector `[0,0,0,0,1,2,3,3,3,3]`.
6. Tessellate the spline into the magenta/cyan overlay and retain its control polygon and points.

The current renderer also batches the complete canonical lines/arcs, so the G64 curve is displayed as an overlay on those primitive batches. Relevant code:

- `src/include/machine/ToolpathRecorder.h`
- `src/Worker.h`
- `src/Application.cpp`, especially `rebuildPreviewRenderCache()`
- `src/PreviewSpline.h`

## Current executable planning algorithm

`BoundedLookaheadTrajectoryPlanner` currently collects compatible G64 feed lines/arcs until a protected boundary, synchronization point, or interpreter completion. There is no longer a 32-command flush. Compatibility requires:

- continuous G64 mode;
- the same optional P value;
- the same protected tool, tool-offset, WCS, and non-motion modal presentation;
- a positive feed;
- no rapid or explicit G53 line;
- exact canonical endpoint continuity within the current `1e-12` position comparison.

A protected boundary, synchronization point, or interpreter completion causes planning. Replacing this whole-horizon flush with rolling immutable-prefix commitment is still pending.

`ExactStopTrajectoryPlanner::compileContinuous()` performs these steps:

1. Convert lines and endpoint-exact arcs into `ContinuousEntity` values parameterized by path distance.
2. Compute `min(P, length/6)` for each entity.
3. Construct a piecewise path containing exact trimmed primitive sections and one local six-control clamped cubic B-spline at each junction.
4. Assign each retained primitive its own programmed feed in machine units per second. Assign each junction spline the arithmetic mean of its incoming and outgoing entity feeds.
5. Sample each piece's tangent, curvature, and curvature derivative to derive conservative aggregate and per-axis velocity, acceleration, and jerk caps. Junction B-splines now provide their arc-length curvature derivative from analytic first/second/third parameter derivatives instead of differencing adjacent curvature samples.
6. Apply jerk-aware forward and backward reachability passes over piece-boundary stations. The horizon starts and ends at zero speed; internal station speeds are reduced only as far as neighboring piece length and limits require.
7. Estimate one shared scalar acceleration at each internal station with a conservative minmod spatial-velocity slope. Run local one-dimensional Ruckig position solves with those nonzero boundary accelerations, reducing the complete acceleration estimate if Ruckig cannot prove every piece. Also solve the zero-acceleration baseline and retain it whenever the acceleration-carrying candidate is not faster.
8. Emit straight Ruckig constant-jerk phases exactly. For curved pieces, construct stable local three-span C2 cubic chains across the complete local solve and recursively subdivide only where ordered geometry verification requires it. This avoids fitting numerical micro-phases independently.
9. Recursively verify each emitted polynomial against its ordered line, arc, or B-spline source interval and the configured chord tolerance.
10. Compute exact polynomial velocity, acceleration, and jerk extrema. When a span violates an aggregate or per-axis limit, reduce only its owning piece's local velocity/acceleration/jerk limits, rerun the forward/backward passes and local solves, and fail with per-pass diagnostics if correction does not converge.
11. Stage the complete normal span stream in NRT storage and partition it across consecutive `PlanChunk` packets of at most 256 normal spans.
12. Preserve the moving PVA state at internal packet boundaries, generate a synchronized multi-axis Ruckig velocity-control stop, convert its constant-jerk phases to cubic stop spans, and reject the plan if the verified stop exceeds the existing 16-span capacity.
13. Add the final stationary stop tail and associate every command activation span with its owning packet.

The resulting compatible horizon is a batch of one or more `PlanChunk` values. Consecutive packets use predecessor/branch identity so the backend can select `Continue` at nonzero velocity when the next packet is already queued. Otherwise it selects the current packet's complete stop tail and reports `BackendHeld`.

## This is a temporary continuous-motion stage

The current executable G64 planner is an architectural checkpoint, not the intended final velocity planner. It now validates blended geometry, multi-packet publication, moving packet boundaries, command activation, polynomial constraints, and stop-safety machinery before introducing a rolling committed prefix.

Only one meaningful timed trajectory is currently generated for a successful G64 window:

```text
zero speed
  -> accelerate through the transformed piecewise path
  -> reach the conservative allowed middle speed
  -> decelerate
  -> zero speed at the window end
```

That rest-to-rest trajectory is divided across consecutive `PlanChunk::normalMotion` arrays. Internal packets end at moving states and contain a real alternate Ruckig stop tail; only the final packet has the tiny stationary hold used at the protected rest boundary.

Therefore a backend `Continue` choice between packets from the same compatible horizon preserves nonzero PVA motion. The machine no longer stops because 32 commands were collected or because 256 span slots filled. It still stops at the end of the whole currently collected horizon, which today is a protected boundary, synchronization point, or program completion.

The current speed law is locally limited and much less conservative than the old window-wide law, but it is not a globally time-optimal path-parameterization solver. It uses sampled differential-geometry caps, analytic jerk-aware velocity reachability distances, a conservative acceleration-carrying station candidate, local Ruckig solves, and exact emitted-polynomial correction. The acceleration candidate cannot make a plan slower because the complete zero-acceleration timing is retained as a comparison baseline. The reachability pass still calculates its velocity envelope with zero-acceleration transition distances, so it does not yet exploit the complete `(s,v,a)` reachable set. It can keep distant slow geometry from throttling an unrelated prefix, but it still forces rest at both ends of the complete compatible horizon and repeats whole-horizon work during correction.

The intended progression is:

```text
current checkpoint:
    zero -> locally capped forward/backward velocity envelope
         -> acceleration-carrying local candidate, if faster
         -> moving PlanChunk packets -> zero

next trajectory-math stage:
    replace velocity-only reachability with acceleration-aware (s,v,a) passes
    + use local Ruckig feasibility to maximize reachable station state
    + retain exact emitted-polynomial correction as final authority

following rolling-publication stage:
    reuse the local velocity ceilings and forward/backward reachability
    + publish an immutable prefix while retaining a mutable suffix
    -> avoid buffering the complete uninterrupted G64 run
    -> slow only where upcoming geometry or limits require it

later bounded-horizon stage:
    retain a capacity-proven stop continuation from every committed branch state
    -> cross rolling planning-horizon boundaries at nonzero velocity
    -> preserve bounded feed-hold latency and safe stopping
```

In the intended moving-boundary design, a chunk branch becomes meaningful:

- the normal branch continues moving into an already available compatible successor;
- the stop branch decelerates from the same immutable moving branch state to a verified stationary state.

Do not mistake successful execution of the current whole-horizon plan for completion of G64 velocity planning. The approved piecewise geometry and implemented local forward/backward timing must be preserved while adding moving immutable prefixes, bounded horizon duration, and bounded stop latency.

Relevant code:

- `src/include/machine/BoundedLookaheadTrajectoryPlanner.h`
- `src/ExactStopTrajectoryPlanner.cpp`, especially `compileContinuous()`
- `src/MockMotionBackend.cpp`, especially `activateNext()` and `completeSpan()`

## Reproduction results

Temporary diagnostics were added to the existing framework-free test executable, run, and then completely removed. The worktree was restored and verified clean afterward.

### Full `1002_3d.ngc` preview pipeline

The existing preview-prefix harness was temporarily changed to retain G64 instead of rewriting it to G61, and it was run over the full file.

Observed result:

```text
observed canonical commands: 141
interpretation complete: false
outstanding chunks: 0
probe pending: false
waiting for held: false
driver error: continuous plan is not C2 at a polynomial boundary
blended windows completed: 4
blended commands completed: 128
exact-stop fallbacks under the old policy: 5
maximum normal spans: 204
planned duration before failure: 41.383780 seconds
```

The driver had successfully observed multiple complete G64 windows before failing on a later one. This demonstrates that `observePlanned()` does iterate the inputs of successful combined windows; the missing remainder is caused by the subsequent planner error.

### `adaptive_pockets.ngc` immediate-style pipeline

The first continuous window failed verification. With detailed temporary reporting, the rejected boundary was:

```text
position jump:     0
velocity jump:     5.5276908350181065e-11
acceleration jump: 1.0299376730488228e-06
```

The acceleration jump is above the hard `1e-7` threshold even though position and velocity are effectively identical.

### Real `SimulationWorker` probe-followed path

`adaptive_pockets.ngc` was run through the actual `SimulationWorker`, including its autoloaded M6 tool-change program and synthetic probe contact. The tick multiplier was temporarily raised to 1000 so the result could be reached quickly.

Observed result:

```text
simulation status: Error
driver/snapshot error: continuous plan is not C2 at a polynomial boundary
servo ticks: 290000
planned duration before failure: 123.83871483431014 seconds
blended windows completed: 3
blended commands completed: 96
exact-stop fallbacks under the old policy: 1
maximum normal spans: 222
```

The detailed rejected boundary was:

```text
position jump:     0
velocity jump:     2.632828947344451e-11
acceleration jump: 4.823515130202374e-07
```

This is the strongest reproduction because it includes the real probe barrier and timed-worker orchestration. The probe path cleared and G64 planning advanced through several windows before the same verification failure. It did not remain stuck with a stale `probePending` flag in the direct driver measurements.

### Current local-timing measurement on `adaptive_pockets.ngc`

After replacing the minimum-feed/global-stretch law, the unmodified program was run again through `SimulationWorker` at 1000x until the large N55 horizon had compiled. Temporary measurement output was removed afterward.

```text
G64 window commands:        5164
first block:                N55 G1 Z1.5205 F80.
last block:                 N25870 Z1.7
compile result:             success
planned duration total:     639.8672054459346 seconds
large-horizon plan latency: 22.7545126 seconds
maximum packet spans:       256
```

The same horizon previously had a raw minimum-feed duration of 229.18 seconds followed by a 75.245x global constraint stretch, producing 17,258.06 seconds. The local plan is about 27 times shorter. Focused regressions additionally prove that an F60-to-F180 straight junction caps its spline at the F120 arithmetic mean and that a distant retained F1 entity does not prevent an earlier long F80 entity from reaching essentially F80.

The 22.75-second compile latency is still unacceptable as an initial real-machine pause. It is primarily an architectural consequence of buffering and repeatedly verifying all 5,164 commands before publishing anything. Rolling immutable-prefix planning is therefore still required even though the local velocity calculation is now useful.

### Focused dense-line/arc timing fixture and hard blend limit

`g64_dense_timing_test.ngc` is a short deterministic development program with one-inch entry/exit reference lines, ten 0.02-inch shallow line segments, and ten 0.02-inch large-radius arcs. It uses `G64 P0.001` and `F120` with the configured 20-unit/s² path acceleration and 100-unit/s³ path jerk. It compiles in well under a second and is now a permanent regression.

Before the acceleration-carrying work, its measured maxima were:

```text
long entry line:  2.000000 in/s
dense lines:      0.487689 in/s
dense arcs:       0.515323 in/s
long exit line:   2.000000 in/s
planned duration: 2.412522 s
```

The important finding is that this roughly 25% speed is not primarily caused by resetting scalar acceleration at every geometry-piece boundary. The local six-control cubic blend is C2, so its curvature is continuous, but its curvature derivative need not be continuous or small. At a line/spline endpoint with zero curvature, neither scalar acceleration nor scalar jerk can cancel the normal component of

```text
q_jerk = q'''(s) v^3 + 3 q''(s) v a + q'(s) j.
```

There the normal term reduces to `q'''(s) v³`; with `P=0.001` and the configured jerk limit, several line-line junctions have proven local ceilings around 0.45--0.48 in/s. The later arc/line junction also creates a low ceiling that propagates backward through the short arc sequence. Increasing timing optimality cannot legally exceed those geometry/jerk bounds. Higher speed in this exact case requires a larger blend scale, a higher physical jerk allowance, or a smoother-than-cubic/C2 blend geometry; it must not be obtained by ignoring the bound.

The new planner stage nevertheless removes the unconditional zero-acceleration assumption: local Ruckig solves accept shared nonzero station acceleration, and the emitted axis PVA remains C2. Because the present velocity envelope was itself derived with zero-acceleration transition distances, the acceleration estimate is still only a conservative candidate. A complete near-time-optimal implementation must carry `(s,v,a)` reachability through its forward/backward passes and adapt timing stations to local constraint changes. The zero-acceleration horizon comparison prevents this incomplete optimization from making a valid plan slower.

An experimental follow-up solved maximal equal-limit piece runs as one Ruckig trajectory and sampled its PVA at geometry boundaries. A direct 800-short-line compile remained quick, but the existing timed 800-command multi-packet refill regression no longer completed within its 20-second diagnostic run. Limiting the solve to acceleration/deceleration portions did not remove that playback regression. The experiment was backed out completely; do not restore it without first tracing the interaction among sampled moving acceleration, packet-boundary stop tails, continuation/refill, and simulated completion. The stable implementation keeps only per-piece nonzero-boundary solves plus the whole-horizon no-regression comparison.

## Original failure-policy defect

`BoundedLookaheadTrajectoryPlanner::planWindow()` behaves differently depending on where continuous planning fails:

- At checkpoint `71a4e73`, if `compileContinuous()` returned `unexpected`, it called `planOne()` and silently substituted exact-stop execution. That behavior has been removed.
- If `compileContinuous()` succeeds but `verifyContinuousNormalMotion()` or `verifyStopBranch()` fails, it returns `unexpected` to the driver and permanently stops the run.

This inconsistent policy hid continuous-planning failures and made partial execution look successful. The current rule is instead uniform: inability to prove the requested continuous motion is a critical error that stops the G-code program.

### Historical transactional warning

This warning explains why fallback was unsafe even before the policy changed. Do not reintroduce it by replacing an error with `return planOne()`.

On successful return, `compileContinuous()` has already advanced internal `ExactStopTrajectoryPlanner` state:

- `m_nextChunk`
- `m_nextSpan`
- `m_previousBranch`
- `m_position`

The lookahead input deque is still intact when the outer verification fails. Calling `planOne()` after that mutation would try to compile the first retained command from the continuous window's final position and with already-advanced branch/span identities. The current implementation correctly treats this as fatal. If transactional plan construction is introduced for other reasons, suitable designs include:

- making continuous compilation transactional and committing planner state only after all verification succeeds;
- snapshotting and restoring exact planner state before abandoning an uncommitted candidate;
- moving the outer verification into `compileContinuous()` before it commits state; or
- returning an uncommitted plan plus proposed next planner state, then committing in `planWindow()`.

The first or fourth design is likely the cleanest long-term boundary.

## Likely source of the C2 rejection

The measured discontinuities are small and have this characteristic shape:

- position is exactly equal;
- velocity differs by only tens of picounits per second;
- acceleration differs by several tenths of a micro-unit per second squared.

That points toward independently calculated floating-point boundary acceleration rather than a visibly discontinuous geometric path. Candidate locations include:

- the algebra in `c2CubicChain()` when deriving the three adjacent Bezier spans;
- reconstruction of each scalar Ruckig phase through `scalarPhase()`;
- separate curvature evaluations on the two sides of a source-piece boundary;
- cancellation caused by short interval durations and conversion between Bezier controls and cached terminal state.

Do not merely loosen the tolerance without first identifying which type of boundary failed. Useful temporary diagnostics would record, for every rejected boundary:

- chunk and span IDs;
- local span index in `normalMotion`;
- whether the boundary is inside one three-span chain, between emitted chains, between scalar phases, or between geometry pieces;
- source input and piece indices on both sides;
- scalar phase/cut parameters and duration;
- both full position, velocity, and acceleration vectors;
- absolute and scale-relative differences.

If the two spans represent the same mathematical boundary, prefer constructing one canonical `KinematicPathState` and using it for both sides, or deriving the next span's start directly from the previous span's cached terminal state. A scale-aware verification tolerance may still be appropriate, but it should remain tight enough to detect real C2 construction errors.

## Resolved preview visibility and workload defect

Immediate Preview no longer runs the trajectory planner or backend. It interprets canonical geometry, returns canonical probe targets through the compatibility path, and publishes one completed `ToolpathRecorder`. Planner failures remain fatal and visible in timed Simulation and future real execution; Preview geometry/interpreter failures enter the same chronological UI status stream.

## Hypotheses not supported as the primary failure

The following were investigated but were not the proximate failure in the reproductions above:

- `observePlanned()` recording only the representative command: it explicitly iterates every retained input, and successful windows recorded all their inputs.
- A lookahead window left buffered after `InterpreterCompleted`: `pumpOne()` repeatedly flushes remaining planner contents while interpretation is complete.
- A permanently stale probe barrier: the real probe-followed worker progressed into multiple G64 windows, while direct measurements reported `probePending == false` at failure.
- A permanently stale held-state wait: direct measurements reported `waitingForHeld == false` at failure.
- Rendering or arc tessellation dropping already recorded commands: the recorder itself stopped growing because planning entered `Error`.

Backend continuation/held orchestration should still be covered by the new end-to-end regression after the verification failure is fixed. The current evidence rules it out only as the cause reached first in these programs.

## Test gaps that allowed the checkpoint to pass

The original focused tests missed the real failure:

- `test1001PreviewCompletesBoundedly()` removes the M6 block and only requires the retained toolpath to be non-empty; it does not compare recorder count against canonical command count.
- `testN70N75PreviewPrefixesCompleteBoundedly()` rewrites `G64 P0.001` to `G61`, so it exercises exact-stop orchestration rather than executable G64.
- `testAdaptivePocketsStartsSimulation()` only waits for some block lifecycle presentation and then joins; it does not require final completion.
- `testTrajectoryDriverConnectsInterpreterToMockRtBackend()` uses only three simple G64 line commands in one window.
- Focused spline/planner tests use small synthetic geometry that does not trigger the real-program numerical boundary case.

Coverage now includes large-coordinate C2 continuity, 150-command compiler packetization, 800-command timed queue refill, unmodified adaptive geometry Preview, local primitive/blend feed caps, distant-slow-feature isolation, and information-rich fatal correction failure diagnostics. Full timed completion of `adaptive_pockets.ngc` remains excluded from ordinary CTest because rolling publication and bounded planning latency are not implemented yet.

## Clarabel acceleration-limited development oracle

A standalone solver comparison now lives in `tools/clarabel_trajectory_oracle`. It is deliberately outside production planning and the RT contract. `ngc_g64_oracle_export` evaluates one compatible positive-`P` G64 horizon using the normal interpreter, geometry builder, local constraint correction, and jerk-limited planner. Only when explicitly requested, `compileContinuous()` also emits an NRT model containing the final conservative velocity cap plus midpoint tangent and curvature for 16 intervals per geometry piece. The exporter writes those values and the current planner duration to a small versioned text format.

The Rust sidecar pins Clarabel 0.11.1 and solves a discretized convex minimum-time path parameterization. For each station it uses squared speed `x = v^2` and speed `v`; every interval also has duration `dt`. Rotated second-order cones impose `v^2 <= x` and the exact constant-scalar-acceleration interval time

```text
dt = 2 ds / (v_i + v_(i+1)).
```

With

```text
s_ddot = (x_(i+1) - x_i) / (2 ds)
s_dot^2 = (x_i + x_(i+1)) / 2,
```

ordinary second-order cones impose the coupled midpoint limit

```text
norm(q'(s) * s_ddot + q''(s) * s_dot^2) <= path_acceleration,
```

and linear inequalities impose each finite physical-axis acceleration limit. Station `x` values retain the planner's programmed-feed, axis-velocity, curvature, and curvature-derivative velocity ceilings. A ten-interval straight-line regression matches the analytic 0.6-second rest-to-rest acceleration-limited result.

On `g64_dense_timing_test.ngc`, the first validated comparison produced:

```text
model intervals:                      688
current jerk-limited duration:        2.424964579 s
Clarabel acceleration-only duration:  1.767305546 s
optimistic duration gap:              0.657659033 s (27.120%)
Clarabel iterations / solve time:     22 / about 0.070 s
```

The solver reaches the programmed 2.0 unit/s station speed and the configured 20.0 unit/s^2 aggregate acceleration boundary. The result is not executable and is not a safety proof: midpoint discretization needs refinement/convergence checks, and the SOCP omits dynamic jerk terms, especially `q'(s) s_jerk + 3 q''(s) s_dot s_ddot`. Geometry-derived `q'''(s) s_dot^3` velocity caps are retained, but the remaining jerk coupling means the 27.12% gap is an optimistic upper estimate of recoverable improvement. Ruckig construction and exact emitted-polynomial extrema remain the only execution authority.

Use the oracle next as a diagnostic target: refine its stations until its acceleration-only duration converges, compare its station envelope against the current Ruckig boundaries, then add jerk-feasible `(s,v,a)` reachability toward that envelope. Do not copy the Clarabel profile directly into `PlanChunk`, do not call the solver from RT code, and do not weaken fatal post-generation verification.

## Current implementation plan

The C2 precision, fatal-error visibility, held-recovery, NRT span staging, multi-packet publication, moving packet stop-tail, command-activation ownership, local geometry caps, forward/backward velocity reachability, and first acceleration-carrying local Ruckig timing stage above are implemented. Continue in this order:

1. Refine the Clarabel acceleration-only model adaptively until the dense fixture's duration and active acceleration constraints converge. Use it to distinguish discretization artifacts from real planner loss; retain the fixed short analytic regression.
2. Replace the velocity-only station reachability envelope with acceleration-aware forward/backward `(s,v,a)` reachability. Maximize locally reachable station velocity/acceleration with Ruckig feasibility toward the Clarabel envelope, while preserving the zero-acceleration comparison and exact emitted-polynomial authority.
3. Add adaptive production timing stations only where geometry/constraint variation requires them. Do not create a fixed multiplier of spans per G-code entity and do not treat timing stations as semantic stops. The oracle's fixed 16-way diagnostic subdivision is not the production strategy.
4. Use the dense fixture to distinguish algorithmic loss from hard `q'''(s)v^3` blend limits. Add at least one fixture whose constraint transition genuinely benefits from acceleration carry and assert a measured duration reduction.
5. Retain the resulting local caps and acceleration-aware timing in a rolling mutable suffix.
6. Commit and publish only an immutable prefix whose moving boundary state and complete stop branch are proved, so an uninterrupted G64 program does not need to reach a protected boundary before motion can start or repeat whole-horizon correction work.
7. Add an explicit NRT horizon-duration/resource guard and bounded feed-hold latency without reintroducing a semantic G-code command-count stop.
8. Preserve fatal termination for every geometry, C2, constraint, capacity, stop-tail, or publication proof failure. Never substitute exact-stop motion for failed requested G64.
9. Add an end-to-end regression containing a real probe barrier followed by enough compatible G64 geometry to require multiple rolling planning horizons as well as multiple `PlanChunk` packets.
10. Assert all of the following:
   - the interpreter canonical command count equals the recorder command count;
   - all expected multi-command G64 inputs are blended, while individually planned one-command G64 sections are counted separately;
   - every published chunk is accepted and retired exactly once;
   - no probe or synchronization barrier remains pending;
   - no stale held-state recovery remains pending;
   - timed execution reaches the final canonical endpoint;
   - driver and simulation states reach `Completed`, not merely a non-error prefix;
   - a deliberately unprovable continuous trajectory stops fatally without publishing a partial batch or losing/duplicating commands.
11. Measure real moving stop tails. Redesign their bounded RT storage only if the existing 16 spans are demonstrably insufficient.
12. Re-run the complete real programs, then the ordinary build and CTest suite.

## Baseline verification

After removing all temporary diagnostics:

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

Both commands passed. The repository was clean before this report file was added.
