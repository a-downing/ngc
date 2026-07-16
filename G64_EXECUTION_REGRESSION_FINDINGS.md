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

A standalone solver comparison now lives in `tools/clarabel_trajectory_oracle`. It is deliberately outside production planning and the RT contract. `ngc_g64_oracle_export` evaluates one compatible positive-`P` G64 horizon using the normal interpreter, geometry builder, local constraint correction, and jerk-limited planner. Only when explicitly requested, `compileContinuous()` also emits an NRT model containing the final conservative velocity cap plus midpoint tangent and curvature for 32 intervals per geometry piece. The exporter writes those values and the current planner duration to a small versioned text format.

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

Additional one-off measurements isolated the first compatible G64 feed/arc horizon from complete programs. The exporter source was restored afterward. `adaptive_pockets.ngc` also required temporarily raising `MAX_LOCAL_CORRECTION_PASSES` from 12 to 64 so the existing local correction algorithm could finish instead of returning its previously observed near-limit jerk failure.

| Program/model | G64 motions | Oracle intervals | Current planner duration | Clarabel duration | Optimistic gap | Clarabel solve |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `1001.ngc`, 16 intervals/piece | 242 | 5,504 | 95.447292693 s | 81.535928748 s | 13.911363945 s (14.575%) | 0.619791 s |
| `adaptive_pockets.ngc`, 4 intervals/piece | 5,164 | 38,640 | 622.540851901 s | 463.583039019 s | 158.957812883 s (25.534%) | 10.052023 s |
| `adaptive_pockets.ngc`, 16 intervals/piece | 5,164 | 154,560 | 622.540851901 s | 447.064353307 s | 175.476498594 s (28.187%) | 35.462374 s |
| `adaptive_pockets.ngc`, 32 intervals/piece | 5,164 | 309,120 | 622.540851901 s | 446.160170792 s | 176.380681109 s (28.332%) | 50.661873 s |

The corresponding planner/export wall times were about 17.3 seconds for `1001.ngc`, 56.4 seconds for the four-interval adaptive model, and 60.3 seconds for the sixteen-interval adaptive model. Relative to the Clarabel result, the current trajectory was about 17.1% longer on `1001.ngc` and 39.25% longer on the refined adaptive model.

Increasing adaptive-pocket resolution from four to sixteen intervals per piece reduced the oracle duration by 16.518685712 seconds, or about 3.56% of the four-interval result. Increasing it again from sixteen to thirty-two reduced duration by another 0.904182515 seconds, or about 0.202% of the sixteen-interval result. Four intervals are therefore demonstrably too coarse. Thirty-two intervals are closer but have not been shown converged; future comparisons should refine adaptively and stop only when duration and active constraints stabilize.

The correction-pass experiment is also important. The 12-pass production ceiling failed with the last reported path-jerk excess already near the limit, while the otherwise unchanged algorithm completed the entire 5,164-motion horizon under a temporary 64-pass ceiling and produced the 622.540851901-second verified trajectory. This shows that the reported adaptive-pocket failure is a correction-iteration ceiling rather than proof that the path is infeasible. Merely raising the ceiling is not the intended near-time-optimal solution: it still repeats expensive whole-horizon reachability, Ruckig timing, polynomial emission, and exact verification, as reflected in the roughly one-minute export. The source was restored to 12 passes after measurement; no planner behavior was changed by this documentation-only checkpoint.

The solver reaches the programmed 2.0 unit/s station speed and the configured 20.0 unit/s^2 aggregate acceleration boundary. The result is not executable and is not a safety proof: midpoint discretization needs refinement/convergence checks, and the SOCP omits dynamic jerk terms, especially `q'(s) s_jerk + 3 q''(s) s_dot s_ddot`. Geometry-derived `q'''(s) s_dot^3` velocity caps are retained, but the remaining jerk coupling means the 27.12% gap is an optimistic upper estimate of recoverable improvement. Ruckig construction and exact emitted-polynomial extrema remain the only execution authority.

Use the oracle next as a diagnostic target: refine its stations until its acceleration-only duration converges, compare its station envelope against the current Ruckig boundaries, then add jerk-feasible `(s,v,a)` reachability toward that envelope. Do not copy the Clarabel profile directly into `PlanChunk`, do not call the solver from RT code, and do not weaken fatal post-generation verification.

## First coupled-limit-aware `(s,v,a)` reachability stage

The production planner now keeps the velocity-only forward/backward result only as a feasible diagnostic seed. Three bidirectional coordinate sweeps optimize shared station velocity and acceleration on monotone ramps. Every accepted update must reduce the combined duration of its adjacent pieces, pass both local Ruckig position solves, and admit aggregate/per-axis acceleration and jerk at both geometry boundaries including the coupled curvature terms. There is no separately timed velocity-only fallback selection. Exact emitted-polynomial verification remains authoritative and local correction now retains a 1% reserve so re-optimization does not repeatedly consume a marginal correction.

| Program | Previous planner | Velocity-only seed with current limits | Coupled `(v,a)` planner | Current 32-way Clarabel | Gain versus current seed |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dense fixture | 2.424964579 s | 2.447639143 s | 2.423589511 s | 1.753531942 s | 0.024049632 s (0.983%) |
| `adaptive_pockets.ngc` | 622.540851901 s | 622.279618933 s | 621.916364725 s | 446.679202750 s | 0.363254207 s (0.058%) |

The dense result is 0.001375068 seconds faster than the previous planner. The adaptive result is now 0.624487176 seconds faster than the previous recorded run while retaining the requested 1% correction reserve. The improvement came from preserving Ruckig's exact outgoing constant jerk in every scalar phase instead of reconstructing it from rounded boundary accelerations. The current station optimizer is useful but is not complete reachable-set propagation: it can still settle in a local state sequence, and fixed geometry-piece stations remain too coarse for some constraint transitions. A development attempt using one greedy forward/backward frontier and another using a small discrete global state lattice both produced worse verified trajectories because scalar-time improvements ignored coupled geometric jerk and caused stronger exact correction. Those approaches were removed.

### Jerk-limit diagnostic

Scaling aggregate and finite per-axis jerk together while holding velocity and acceleration fixed sharply closes the gap to a freshly generated 32-way Clarabel model:

| Program | Jerk | Verified planner | Clarabel | Planner excess over oracle |
| --- | ---: | ---: | ---: | ---: |
| Dense fixture | 1x | 2.423589511 s | 1.753531942 s | 38.212% |
| Dense fixture | 10x | 1.601229453 s | 1.478985519 s | 8.265% |
| Dense fixture | 100x | 1.333088266 s | 1.329211801 s | 0.292% |
| Dense fixture | 1000x | 1.352727075 s | 1.351002408 s | 0.128% |
| `adaptive_pockets.ngc` | 1x | 621.916364725 s | 446.679202750 s | 39.231% |
| `adaptive_pockets.ngc` | 10x | 309.467499743 s | 277.916090073 s | 11.353% |
| `adaptive_pockets.ngc` | 30x | 233.634337079 s | 224.038614824 s | 4.283% |

Adaptive 100x now stops fatally at the per-pass geometry-verification budget after 28,981 attempts rather than recursing into a 27.8 ns interval. Adaptive 1000x stops at the curvature-coupled acceleration-aware reachability budget after 316,529 total candidate evaluations (about 14 seconds in the recorded run) rather than consuming minutes of repeated correction work. Production, 10x, and 30x complete below both bounds. Dense 1000x was slightly slower than 100x in both models. These extreme points expose numerical and representation limits, not a clean infinite-jerk asymptote.

Curved emission first retries a failed complete-solve cubic at the nearest exact Ruckig phase boundary, then uses ordered midpoint subdivision. Each ordered Bezier-hull proof has bounded internal work, and the complete horizon has explicit per-pass and cumulative reachability, geometry-verification, and staged-span budgets. Exceeding a budget is an information-rich fatal planning error; it never falls back to exact-stop motion.

The successful trend is strong evidence that dynamic jerk accounts for most of the normal-limit Clarabel gap and that the planner approaches the acceleration-only optimum when jerk becomes nonbinding. It does not establish normal-jerk optimality: Clarabel omits the dynamic `q' j + 3 q'' v a + q''' v^3` constraint, so a jerk-aware reference is required before attributing the remaining normal-limit time to reachability.

### Jerk-aware Clarabel follow-up

The oracle v3 export now includes path/axis jerk, analytic `q'''`, and production piece-boundary PVA/duration diagnostics. A development `--jerk-aware` mode repeatedly linearizes the complete finite-difference jerk vector about the latest squared-speed profile and solves the trust-region conic subproblem with Clarabel. Every reported result is checked against the nonlinear discrete jerk afterward. This is sequential convex programming, not a proof of the nonconvex global optimum, and it does not replace exact emitted-polynomial verification.

At 64 dynamics intervals per geometry piece, the dense fixture produced a 2.216415722-second discretely jerk-feasible reference versus the planner's 2.423589511 seconds, leaving 9.347% planner excess relative to the reference. The 32-way value was 2.209524075 seconds, a 0.31% grid change. On `1001.ngc`, 32 intervals per piece produced 89.638000361 seconds versus the planner's 95.338245391 seconds, leaving 6.359% excess; its 16-way result was 88.823845456 seconds. The attempted 64-way `1001.ngc` SCP did not converge numerically.

The full 309,120-interval adaptive-pocket v3 model exports successfully, but a four-way 38,640-segment jerk SCP did not complete within five minutes. A one-way solve was demonstrably too coarse and is not a comparison result. A scalable jerk-aware reference for that horizon therefore remains open.

The v3 profile comparison localizes the dense loss to alternating 0.006/0.014-unit blend pieces where the oracle carries about 0.3–0.44 unit/s more boundary speed. On `1001.ngc`, the largest repeated 0.080-second gaps retain nearly identical boundary speeds but carry about +4.24/-4.24 unit/s² oracle acceleration. Three production experiments were rejected and removed: relaxing the local station veto made dense slower after correction; a bounded global acceleration lattice either regressed `1001.ngc` or produced no post-verification gain; and inserting midpoint stations into long curved pieces drove exact correction to a 353-second `1001.ngc` trajectory. The stable planner durations remain 2.423589511 seconds and 95.338245391 seconds. Do not restore those scalar-profile experiments without solving the emitted-polynomial/coupled-correction interaction as part of the frontier state.

### P0.002 adaptive-pocket resource follow-up

Changing `adaptive_pockets.ngc` from G64 P0.001 to P0.002 initially stopped at the curved reachability per-pass guard after 290,146 evaluations. Profiling showed that 54,330 of the first 63,626 velocity candidate sets were the unconditional 12-step refinement below an infeasible station cap. The production search now deduplicates clamped acceleration candidates, reuses the already-timed current state, stops unchanged bidirectional sweeps, and uses four velocity-bracket refinements only when a horizon exceeds 1,024 curved stations. Small horizons retain all 12 steps.

Removing that excess work exposed a separate correctness bug. Some acceleration-carrying Ruckig solutions include a two-phase brake pre-profile, but scalar extraction retained only the seven main phases and sampled jerk ambiguously at exact boundaries. The reconstructed distance could reverse inside a curved fit and pass inverted bounds to `std::clamp`. Scalar extraction now integrates the brake and main phases in order with their exact outgoing jerks, checks the exact interior velocity minimum of every constant-jerk phase, and rejects any decreasing curved-emission interval before geometry proof.

Arc and six-control spline distance inversion now start from their accurate preintegrated lookup bracket and use at most 12 safeguarded Newton steps against the actual adaptive length integral instead of 48 and 44 fixed bisections respectively. Local correction has a measured 24-pass ceiling; cumulative work guards were scaled without changing the per-pass curved-candidate or geometry-verification limits.

| Program | Verified duration | Plan calculation wall | Candidate evaluations | Geometry attempts |
| --- | ---: | ---: | ---: | ---: |
| Dense fixture | 2.423589511 s | 0.466 s | 59,294 | 553 |
| `1001.ngc` | 95.338245390 s | 0.501 s | 7,478 | 27,022 |
| `adaptive_pockets.ngc`, P0.001 | 621.995691618 s | 2.806 s | 169,366 | 109,870 |
| `adaptive_pockets.ngc`, P0.002 | 586.628780107 s | 18.690 s | 2,629,123 | 242,864 |

Dense and `1001.ngc` retain their established durations. The large-horizon P0.001 policy is 0.079326893 seconds (0.0128%) slower than the previous 621.916364725-second result while reducing measured planning wall time from about 25.38 to 2.81 seconds. P0.002 now completes well below the previous three-minute observation. A 1000x jerk stress run still terminates explicitly at a cumulative geometry-verification bound instead of consuming unbounded work.

## Current implementation plan

The C2 precision, fatal-error visibility, held-recovery, NRT span staging, multi-packet publication, moving packet stop-tail, command-activation ownership, local geometry caps, forward/backward velocity reachability, and first acceleration-carrying local Ruckig timing stage above are implemented. Continue in this order:

1. Refine the Clarabel acceleration-only model adaptively until duration and active acceleration constraints converge on the dense fixture and representative full-program horizons. The adaptive-pocket four-versus-sixteen result proves that four intervals per piece are insufficient. Retain the fixed short analytic regression.
2. Extend the coupled-limit-aware station optimizer into complete forward/backward `(s,v,a)` reachable-set propagation. Keep Ruckig feasibility and exact emitted-polynomial authority; do not restore a separately selected zero-acceleration fallback.
3. Add adaptive production timing stations only where geometry/constraint variation requires them. Do not create a fixed multiplier of spans per G-code entity and do not treat timing stations as semantic stops. The oracle's fixed 32-way diagnostic subdivision is not the production strategy.
4. Use the dense fixture to distinguish algorithmic loss from hard `q'''(s)v^3` blend limits. Add at least one fixture whose constraint transition genuinely benefits from acceleration carry and assert a measured duration reduction.
5. Reduce the implemented rolling prefix's suffix-probe CPU cost and add numerically identical arc-interior split states; do not weaken the strict boundary PVA proof.
6. Extend the implemented incremental publication to numerically identical arc-interior anchors so arc-only regions do not need to extend to a later retained line or protected boundary.
7. Add bounded resumable feed-hold latency without reintroducing a semantic G-code command-count stop or a synthetic connector from an off-path stop state.
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

## Rolling immutable-prefix checkpoint (2026-07-15)

`trajectory.lookahead_duration` now configures a positive minimum predicted duration for NRT rolling G64 planning. While compatible commands are still being interpreted, `BoundedLookaheadTrajectoryPlanner` searches beyond that minimum for a retained-line interior where both the committed prefix and a following stop-feasible suffix accept the same nonzero PVA boundary. A failed provisional search retains the window and retries after more lookahead; it never creates an artificial terminal stop. The split preserves the entity's original G64 scale on both halves, including a short entity split at the midpoint shared by its neighboring splines. The retained half is marked as already activated so command presentation is not duplicated. Arc-interior anchors remain disabled because independently reconstructed partial-arc curvature differed at the strict PVA boundary gate; arc-only regions extend rather than weakening continuity proof.

The exact continuous compiler accepts explicit start/end `MotionState` values, projects them onto the path tangent/curvature state, fixes their scalar velocity and acceleration during reachability, verifies emitted boundary PVA, and generates a normal moving-boundary stop branch at the last packet. A focused regression crosses two horizons at nonzero speed and asserts exact position/velocity/acceleration continuity, a real stop tail, one-time source activation, final rest, and per-horizon/total timing diagnostics.

With `adaptive_pockets.ngc` at P0.002 and a 2-second configured minimum, the incremental rolling benchmark produced 54 horizons and 161 chunks. Safe boundaries made actual horizons about 2.96 to 16.04 seconds long. Rolling boundaries now warm-start one velocity step above the previous successful fraction, then halve only as required; their bounded prefix/suffix solves use six velocity-bracket refinements while ordinary full-horizon solves retain twelve. Boundary candidates fell from 306 to 241 and failed suffix probes from 252 to 186. The first horizon calculated in about 0.78 seconds, the maximum was about 1.28 seconds, published-horizon calculation totaled about 28.67 seconds, failed incremental searches added about 0.47 seconds, and total planning calculation was about 29.14 seconds. This is about 40.7% less computation than the previous 49.14-second rolling result. Planned motion duration was 589.677061658 seconds: +1.560036445 seconds or about +0.265% versus the previous rolling result, and +3.048281551 seconds or about +0.520% versus the 586.628780107-second full-horizon result. Full-horizon refinement and the established dense/`1001.ngc` comparison baselines are unchanged. The exporter supports `--rolling` for a full comparison with per-horizon rows and `--rolling-only` for policy profiling without rebuilding the Clarabel model.

## Short-entity spline curvature-derivative investigation (2026-07-16)

The remaining visibly slow adaptive-pocket trochoids were isolated by enabling `--trace-slow`. The exporter now retains an `ngc_spline_geometry_v1` snapshot containing the exact source line/arc records, every captured spline control polygon, and its timing-piece boundaries; a separate CSV retains all planned piece boundary states and limits. The standalone `ngc_spline_geometry_analyzer` evaluates analytic first, second, and third B-spline parameter derivatives and converts them to arc-length tangent, curvature, and curvature-vector derivative. Finite-difference curvature checks at the reported extrema agree with the analytic values, including the large normal components, so the principal spikes are not floating-point noise or a mistaken parameter-space third derivative.

At constant scalar speed `v`, the diagnostic uses

```text
axis acceleration = q''(s) v^2
axis jerk         = q'''(s) v^3
```

For a planar curve, `q'''` decomposes into turning jerk along the tangent and curvature-change jerk normal to the tangent. Turning jerk remains nonzero on a perfect circle because the centripetal-acceleration vector rotates. The large differences between visually similar trochoids were dominated instead by the normal component: the direct-control cubic is C2, but its curvature derivative can change abruptly at simple knots and at the transition from the constrained endpoint controls to the interior controls.

For captured spline 627 at F80 (1.333333333 inch/s), the production cubic measured 30.342 inch/s^2 peak acceleration and 9,517.8 inch/s^3 peak total jerk. Curvature-change jerk contributed 9,516.8 inch/s^3 while turning jerk peaked at 690.5 inch/s^3. The production planner does not cap that complete 0.480835-inch spline at one velocity: it creates eleven timing pieces. The recorded boundary speeds rose from F17.6 at entry through F39.8, F47.7, and F50.1 in the middle before falling to F19.9 at exit. A constant-F80 graph alone is not a reachability proof because accelerating motion also includes the `q' j + 3 q'' v a` terms and must reserve distance and jerk for the restricted exit.

The original bounded control-polygon conditioner was intentionally weak. A standalone traversal-cost coordinate search with the same fixed first/last three controls reduced spline 627's peak F80 jerk only from 9,517.8 to 8,963.0 inch/s^3 and increased its mean jerk. Directly optimizing peak and integrated normal curvature derivative while retaining those fixed controls improved the peak to 8,835.0 inch/s^3. The decisive cubic experiment allowed the endpoint tangent-handle length and the adjacent tangential control to vary together while analytically preserving exact endpoint position, tangent, and curvature. Under a sampled `0.2P` deviation bound it reduced spline 627's normal curvature-derivative peak from 4,014.91 to 498.824 and its arc-length integral from 81.3209 to 31.3805. Maximum displacement was 0.000793304 inch; endpoint position error was zero, tangent error was `5.39e-14`, and curvature error was `1.64e-11`. The same experiment reduced the peak from 14,110.8 to 1,963.33 on spline 744 and from 1,278.39 to 732.786 on spline 863.

The separate `ngc_quintic_spline_analyzer` performs a genuine primitive-chain reconstruction rather than degree-elevating the cubic. It uses the same effective span count, simple degree-five knots, and endpoint primitive derivatives through `q'''`. Its free controls are then optimized against peak plus integrated normal curvature derivative while a high-resolution check enforces maximum primitive deviation no greater than `0.2P` (0.001 inch for these P0.005 captures). Results were:

| Captured spline | Production cubic peak | Optimized cubic peak | Optimized quintic peak | Quintic integral | Verified quintic deviation |
| --- | ---: | ---: | ---: | ---: | ---: |
| 627 | 4,014.91 | 498.824 | 389.731 | 25.2135 | 0.000971435 in |
| 744 | 14,120.7 | 1,963.33 | 1,252.77 | 49.2376 | 0.000977076 in |
| 863 | 1,278.39 | 732.786 | 718.319 | 39.9638 | 0.000974564 in |

The optimized quintic reduced peak normal curvature derivative by another 21.9%, 36.2%, and 2.0% relative to the optimized cubic on those three captures. It also removed the cubic's mathematical C2 knot jumps by providing C4 continuity. The gain is path-dependent rather than automatic: most of the benefit came from explicitly conditioning the endpoint transition and the physical arc-length objective, not from degree elevation alone.

These analyzers are not production geometry. Their primitive-deviation checks are sampled, and the standalone arc evaluator is a directed endpoint-exact approximation rather than the production adaptive `ArcReference`. No experimental controls enter preview, the continuous compiler, `PlanChunk`, or `MotionBackend`. Before adopting degree five, implement one shared preview/planner construction using the production arc reference, prove endpoint and ordered deviation bounds, preserve source feed/activation mapping, measure planning cost, and rerun the complete geometry, timing, capacity, stop-branch, rolling-boundary, and UI regressions.
