# Prepared trajectory test backlog

The command-driven G64 compiler, `TrajectoryExecutionDriver`, and their tests were
removed when continuous planning became prepared-geometry-only. Restore the useful
behavioral coverage through `GeometryStreamProducer`, `PreparedGeometrySlice`,
`TrajectoryPlanner::enqueuePrepared()`, and `PreparedTrajectoryExecutionDriver`.
Do not recreate a planner-owned geometry-construction path.

## Geometry and continuous timing

- Verify bounded prepared lookahead retains G64 metadata and exact-stops a single
  continuous command when a protected boundary forces finalization.
- Verify a prepared retained-section/junction-blend sequence executes piecewise and
  respects independent axis limits.
- Verify a short-entity cluster becomes exactly one cluster spline, including the
  configured quintic solver's controls and knot-span boundaries.
- Verify rolling prepared horizons preserve a nonzero position/velocity/acceleration
  boundary and retain a valid stop branch on every published packet.
- Verify rolling anchors are selected only inside retained line sections and reject
  junction blends, cluster splines, and arc interiors.
- Verify an exact-stop lead-in advances the prepared rolling boundary correctly.
- Verify C2 boundary checks remain stable at large absolute machine coordinates.
- Verify prepared continuous geometry packetizes beyond normal-span capacity without
  adding a semantic stop at packet boundaries.
- Restore the dense continuous-timing fixture using geometry produced once by
  `prepareContinuousGeometry()`.
- Verify local time laws retain Ruckig brake pre-profiles and their exact outgoing jerk.
- Verify retained primitive sections use their source feed, junction blends use the
  arithmetic mean of adjacent feeds, and cluster spline feed mapping follows the
  prepared piece metadata.
- Verify feed changes inside one cluster spline retain their local prepared-piece
  velocity caps without reconstructing the cluster in the compiler.
- Verify a distant slow prepared piece does not throttle an already publishable fast
  rolling prefix.
- Verify a prepared line-to-arc window executes retained arc sections and its local
  junction blend with canonical endpoint continuity.
- Verify executable prepared G64 geometry retains exact primitive middles.

## Failure and lifecycle behavior

- Verify invalid continuous limits fail a prepared run fatally and publish one
  information-rich chronological status error.
- Verify tool-change presentation applies nested and final work-coordinate systems
  when their owning prepared activation spans execute.
- Verify immediate draining stops at Held and does not execute stale descendants.
- Verify the historical N70/N75 preview-prefix fixtures complete within a bounded
  number of prepared-driver pump cycles.
- Verify the prepared execution driver connects interpretation, geometry streaming,
  planning, and `MockMotionBackend` execution end to end.
- Verify `MockMotionBackend` drains a complete multi-packet prepared horizon without
  overflowing its bounded event transport.
- Verify a prepared rolling prefix is published before a later protected boundary is
  interpreted or executed.
- Verify Held recovery preserves buffered prepared G64 commands without replaying or
  dropping them.
- Verify a parameter read waits for completion of prior prepared motion.

## Diagnostics formerly supplied by the exporter

- Add a library-level fixture for infinite-jerk comparison over an already prepared
  smoothed path; ordinary planning must continue to skip oracle work.
- Add focused prepared-curve inverse diagnostics for bit-exact cache hits, integral
  evaluations, safeguarded Newton iterations, and endpoint queries.
- Add an optional test helper that serializes prepared spline controls and verified
  piece timing when diagnosing slow fixtures. Keep it out of production targets.
