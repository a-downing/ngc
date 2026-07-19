# HiGHS-backed SCP planner review (2026-07-19)

Review of the HiGHS-backed sequential-linearization feedrate planner introduced in
`23c4f66` ("Experiment with HiGHS-backed SCP feedrate planning") and `13a8121`
("Add bounded quality rescue to SCP planner"), at `HEAD` (`8c47ef0`).

Method: LP algebra checked term by term against the coupled kinematics, HiGHS API
usage checked against the vendored submodule, `23c4f66^..HEAD` diff reviewed, full
build and CTest run (all 3 targets pass, 7.2 s).

Findings are ordered by severity. File references are to
`src/TrajectoryCompiler.cpp` unless noted.

## Bug: station-visit replay cache is wiped every correction pass

Lines 2741-2745:

```cpp
previousStationVisits=std::move(currentStationVisits);      // inside if(useAccelerationAwareRescue)
previousStationVisitSlots=std::move(currentStationVisitSlots);
}                                                            // end of rescue block
previousStationVisits.clear();        // <-- runs EVERY pass, unconditionally
previousStationVisitSlots.clear();
```

`23c4f66` added these two clears when the old search was wrapped in `if(false)`
(they were no-ops then). `13a8121` re-enabled the block as the rescue path but
left the clears in place. At the end of every correction pass the replay records
the rescue just produced are destroyed, so on the next pass `optimizeStation`
(line 2427) always sees an empty `previousStationVisitSlots`.
`enableStationVisitReplay` is dead code: `comparableVisits` /
`exactInputMatches` / `exactOutputMatches` can never be nonzero, and every rescue
pass re-pays the full station-search cost.

Fix: move the two clears into an `else` (clear only when the rescue did not run),
or delete them; the rescue-enable path already clears stale records at lines
3153-3154.

## Robustness: a HiGHS non-optimal status kills the whole run

Lines 2240-2248 return `std::unexpected` whenever the solve is not `kOptimal`,
including `kTimeLimit` and `kIterationLimit`. The LP is only an improver: the
velocity-only reachability seed is already a feasible, materialized plan (lines
1911-1916). Aggravating factors:

- `scpSolveTimeLimit` is 0.5 s per solve, and the rolling planner
  (`src/include/machine/TrajectoryPlanner.h:949-994`) runs up to 8 candidates x 6
  attempts of suffix probes per window split; each probe copies the full effort
  config and re-runs SCP on every correction pass. Large horizons can plausibly
  exceed the limit.
- An optional optimization becomes a hard dependency of the run.

Recommendation: on `kTimeLimit` / `kIterationLimit` / non-optimal, break out of
the SCP loop and keep the current feasible station state (record a diagnostic)
instead of failing planning. Every downstream gate (exact materialization,
geometric verification, correction passes) still applies. Related: consider
skipping SCP entirely for suffix probes, which only need a stop-feasible timed
plan the seed already provides; that removes the largest multiplier of LP solves
in the current integration.

## Time-optimality: the LP has no reachability coupling between stations

The only rows are per-station endpoint constraints (linearized axis/path
acceleration and jerk in the station's own v, a, j) plus deviation regularizers.
Nothing relates `v[p]` to `v[p+1]` across a piece. Consequences:

- The LP's optimal velocity solution is essentially "raise every station to its
  trust-region ceiling" (all objective coefficients are negative), regardless of
  whether adjacent stations can reach each other under acceleration/jerk limits.
- The exact pairwise acceptance test (lines 2270-2324) salvages what it can,
  station by station. Raising a station next to a curvature-capped station is
  almost always rejected because the deceleration distance does not exist.

Adding linearized energy rows in both directions would shape LP proposals into
mostly realizable profiles:

```text
v_ref+^2 - v_ref-^2 + 2 v_ref+ (v+ - v_ref+) - 2 v_ref- (v- - v_ref-) <= 2 a_lim L   (accel side)
v_ref-^2 - v_ref+^2 + 2 v_ref- (v- - v_ref-) - 2 v_ref+ (v+ - v_ref+) <= 2 a_lim L   (decel side)
```

with `a_lim = localLimits[p].acceleration`. Jerk-aware reachability is stricter,
so these are relaxations: reference containment is preserved up to existing
slack, and the exact acceptance test remains the feasibility gate. This is the
single biggest improvement available for accepted-step fraction and final
durations.

## Time-optimality: raw-velocity variables instead of squared-speed variables

The research note (`jerk_limited_feedrate_scheduling_research_2026-07-19.md`,
lines 14 and 245) specifies an SCP "over squared speed and path acceleration"
with `(w,a)` state. The implementation uses raw `v`. In `w = v^2` space the
centripetal term `kappa * v^2` becomes `kappa * w`, exactly linear, so all
axis/path acceleration rows would be exact rather than tangent-line
approximations. The current v-space linearization has gradient `2*kappa*v_ref`,
which vanishes as `v_ref -> 0`: the LP loses the centripetal coupling precisely
at low-speed stations where it dominates. Jerk terms get worse in w-space
(`kappa' * w^1.5`), which the doc anticipates ("requires sequential
convexification or conservative bounds"). A hybrid (w for acceleration rows, v or
conservative bounds for jerk rows) is worth considering.

## Time-optimality: objective model quality

- The trapezoid gradient `-2L/(v0+v1)^2` is exact only for a
  constant-acceleration time model; the true time laws are jerk-limited. Cheap
  standard fix: rescale each piece's gradient by `T_actual / T_trapezoid` (both
  known; `pieceTiming` holds the exact current duration) so the linear model
  matches the true objective value at the reference point. Noticeably improves
  step quality for one-iteration SCP.
- With `scpIterations = 1` (default) and the greedy pairwise acceptance rule
  (`newDuration > oldDuration` rejected), the method cannot take a step that is
  locally neutral but globally enabling. More iterations would help, but see warm
  starts below before raising the count.

## Solver-time performance opportunities

1. No warm starts. The research doc cites basis warm starts as a reason for
   choosing HiGHS (line 231), but the code constructs a fresh `Highs` per solve
   (line 2215). LP dimensions are identical across SCP iterations within a pass
   and across correction passes; `highs.getBasis()` / `setBasis()` after the
   first solve typically cuts later solves to a fraction of the iterations. This
   is the enabler for raising `scpIterations` above 1 cheaply.
2. Uncached trial materializations. The acceptance loop (lines 2291-2310) calls
   `timeLawBetween` (no cache) for up to 2 x `scpLineSearchSteps` x stations full
   Ruckig solves per iteration. The rescue path uses `candidateTimeLawBetween`
   (bit-exact result cache) and only calls `materializeCandidateTimeLaw` for the
   winning candidate. Routing SCP trials through the same candidate/cache path
   and materializing only accepted steps eliminates most redundant solves.
3. Redundant rows for exactly-linear piece ends. For a piece end with zero
   curvature and zero curvature derivative (retained line sections), every
   acceleration row is implied by the station's acceleration column bound, and
   every jerk row by the jerk column bound, given how `localLimits` are built
   (lines 1523-1527: `limits.acceleration <= axisAcc/|t|`,
   `limits.jerk <= axisJerk/|t|`). Up to 28 redundant rows per linear piece;
   skipping them shrinks line-heavy LPs substantially.
4. Minor: reserve `a_matrix_` capacity in `SparseLpBuilder` instead of repeated
   `push_back` growth; consider `presolve=off` for very small LPs.

## Minor issues and hygiene

- `scpDuration` (lines 1911, 2319) is written but never read; dead store.
- The reference-containment check hard-codes `firstDeviationRow = 2*station`
  (line 2128), which silently breaks if rows are ever added to the station loop.
  Track the row base explicitly like `scpPieceRowOffsets`.
- `addRow` drops only `|v| <= 1e-18`; coefficients in (1e-18, 1e-9] reach HiGHS,
  which counts them as small matrix values. Dropping below ~1e-12 keeps the model
  cleaner without changing semantics.
- The objective guard `max(1e-6, v0+v1)` (line 2007) is absolute; on a degenerate
  near-zero-speed piece the gradient reaches ~1e12. Practically rare (the seed
  keeps interior stations well above zero), but a physics-scaled floor (a
  fraction of the local velocity cap) is safer.
- `proposedAcceleration` is not clamped the way `proposedVelocity` is (line
  2260); cosmetic, since solver bounds and the exact acceptance test apply.

## Verified correct

- All LP coefficient algebra (axis/path acceleration rows, axis/path jerk rows,
  deviation rows, trapezoid objective gradient), checked term by term against the
  coupled kinematics.
- The reference-containment invariant is mathematically sound: for path-norm
  rows, `d . x <= |x|` (Cauchy-Schwarz) guarantees the halfspace always contains
  the verified reference point.
- Incremental `pieceTiming` bookkeeping in the acceptance loop: only the two
  pieces adjacent to an updated station are re-materialized, other timings stay
  valid. The acceptance rule makes exact total duration monotone non-increasing;
  the SCP can never make the plan worse or infeasible.
- HiGHS integration: `passModel` internally calls `setMatrixDimensions`;
  `time_limit` / `simplex_iteration_limit` / `threads=1` option types are
  correct; small matrix values are warnings not errors; the `kOptimal`
  requirement and primal-value validation are handled.
- Build clean; all 3 CTest targets pass.
