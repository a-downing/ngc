# Decoupling geometry preparation from trajectory planning

Date: 2026-07-17

## Question

> I have an idea. What if we have one thread that reads gcode and accumulates geometry until the cumulative distance and feedrate correspond to some number of seconds of movement, or a synchronization point is reached and in G64 mode also generate smoothed geometry and publish this data in a SPSC ring buffer. Another thread reads from this SPSC buffer and performs the trajectory planning and publishing to the backend. This would allow for more expensive smoothing operations while keeping the trajectory planning fast and low latency. This will matter because I would like to eventually allow feedrate override and be able to adjust max acceleration and jerk while the machine is running.

## Response

Yes—this is a strong architectural direction. It separates expensive, feed-independent geometry reconstruction from timing-sensitive trajectory generation:

```text
Interpreter + geometry worker
    -> prepared geometry SPSC
Trajectory-planning worker
    -> timed execution SPSC
Backend / servo executor
```

The important distinction is that the first worker should publish only immutable geometry, not timing decisions.

A prepared geometry batch could contain:

- exact retained line/arc sections and certified smoothing splines;
- arc-length and derivative data needed by the planner;
- ordered source-entity and presentation activation mappings;
- programmed feed sections and G64 P;
- its canonical start and end states;
- whether its end is a synchronization or protected boundary;
- enough overlap or retained-suffix information to avoid artificial smoothing boundaries.

The geometry worker could maintain a much deeper lookahead—perhaps several seconds or tens of seconds—because changing feed override, acceleration, or jerk does not invalidate a curve whose deviation certificate depends only on geometry and P.

The trajectory planner should maintain a much shallower timed horizon. That is what determines control-change latency. For example:

- prepared geometry: 10–30 seconds ahead;
- committed executable trajectory: perhaps 100–500 ms ahead;
- uncommitted trajectory-planner lookahead: several seconds.

That lets expensive smoothing run comfortably ahead while limiting how much timed motion must be discarded or regenerated after a constraint change.

There are several critical details.

First, “seconds of geometry” must initially be an estimate. Actual duration depends on curvature, axis limits, acceleration, jerk, and surrounding boundary states. The geometry worker should use programmed-feed nominal duration:

```text
nominal duration = path length / programmed feed
```

It can accumulate until that duration reaches a target, while allowing the batch to grow when necessary to find a valid smoothing boundary.

Second, batches cannot arbitrarily cut spline clusters. The recently corrected rolling-anchor defect demonstrates why. The geometry worker must retain an editable suffix and publish only at a geometry-invariant boundary:

- inside the exact middle of a line longer than `6P`;
- at a protected synchronization boundary;
- at a naturally independent geometric boundary.

It should never cut an all-short cluster merely because it reached its nominal time target.

Third, feed override and changing physical limits are different operations.

Feed override can sometimes be implemented by varying progression through an existing path, but the current `PlanChunk` objects already contain timed axis polynomials. Their time laws cannot simply be accelerated without rechecking velocity, acceleration, and jerk. A feed-override change therefore needs either:

- replanning the uncommitted geometry with the new feed; or
- a future backend time-scaling mechanism with its own real-time constraint proof.

Replanning is the simpler and safer first implementation.

Changing maximum acceleration or jerk definitely requires replanning. Each planning request should carry an immutable, versioned constraint snapshot:

```text
PreparedGeometryBatch
ConstraintSnapshot { generation, feed_override, acceleration limits, jerk limits }
    -> PlannedBatch tagged with generation
```

When constraints change, the planner discards obsolete uncommitted results and replans from a valid boundary using the newest generation.

The main blocker is not smoothing or the SPSC buffer—it is transitioning from already-published timed motion to newly planned motion. The planner needs a precisely known starting position, velocity, and acceleration. There are two practical stages:

1. Initially apply updates only at an already-proved packet boundary, keeping the currently published trajectory immutable.
2. Later add an explicit moving replan boundary: the backend promises to follow the old plan to a selected future PVA state, and the planner generates the replacement trajectory from exactly that state.

Every committed boundary must retain a valid stop branch. If replanning fails or cannot finish in time, the backend must take that stop branch rather than running out of trajectory.

Synchronization also remains owned by interpretation semantics. The geometry worker may read ahead through ordinary motion, but it cannot cross parameter-read waits, probing, tool changes, prints, or other synchronization operations until the corresponding execution result is available.

The recommended evolution is:

1. Extract the current G64 geometry construction into a `PreparedGeometry` representation.
2. Make Preview and timed execution consume that same representation.
3. Add the geometry-worker-to-planner SPSC queue.
4. Keep planner limits fixed initially and verify byte-equivalent trajectories.
5. Add versioned feed override with changes applied at packet boundaries.
6. Add versioned acceleration and jerk limits.
7. Finally implement bounded moving-boundary replanning for lower response latency.

This decomposition should materially help. It allows banded fairness, arc recognition, recursive deviation proof, and derivative precomputation to become more expensive without placing that work in the latency-critical trajectory-planning loop. The timed planner would receive certified, arc-length-ready geometry and concentrate on reachable PVA states, constraint corrections, polynomial proof, and stop branches.
