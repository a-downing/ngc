# Quintic materialization prototype status

This document records the state and open questions of the
`wip/quintic-materialization` branch. The branch is experimental. Its quintic
results are diagnostics only and never enter `PlanChunk` or the RT-facing
backend.

## Objective

NGC currently materializes PathTempo's scalar time law as axis-space cubic
execution spans. Curved motion uses C2 cubic chains because one cubic cannot
match position, velocity, and acceleration at both endpoints. Those chains can
remain close to the prepared geometry while producing large, short-lived jerk
artifacts.

The prototype investigates a different curved-motion representation:

- retain the existing exact cubics for geometrically linear motion;
- use one endpoint-PVA quintic for each accepted curved interval;
- adaptively subdivide and group PathTempo scalar phases;
- certify programmed path speed, aggregate acceleration and jerk, and every
  axis velocity, acceleration, and jerk limit;
- prove ordered proximity to the actual prepared geometry and forward
  progress; and
- avoid PathTempo replanning for polynomial-representation artifacts when
  refinement alone can produce a valid representation.

An endpoint-PVA quintic matches position, velocity, and acceleration at both
ends of its interval. Adjacent quintics that share the same evaluated boundary
state are therefore C2 continuous.

## Work present on the branch

The branch includes the updated PathTempo integration and pins PathTempo commit
`a1d13ee` (`Fix short-phase constraint checks and boundary micro-phases`).
PathTempo now checks each scalar jerk-phase endpoint against the bracketing
differential stations and suppresses insignificant repaired boundary
micro-phases.

The NGC changes include:

- `--mode=zero|optimized` and sampled-correction controls in the relevant
  diagnostics;
- `sampled`, production `materialized`, and diagnostic `geometry` continuous
  checking modes;
- actual prepared-geometry differential checks in geometry mode;
- 99 percent diagnostic planning limits for acceleration and jerk;
- removal of duplicated caller-side per-piece dynamic acceleration and jerk
  reductions, leaving those reductions to PathTempo;
- solve-local caching of bit-identical materialized timing pieces;
- one-sided `q'''` samples at C2 spline knots instead of incorrectly sharing a
  third derivative across the knot;
- detailed emitted-cubic constraint severity and provenance;
- adaptive endpoint-PVA quintic construction using Bézier controls;
- certified derivative-control bounds for path and axis constraints;
- ordered prepared-geometry and forward-progress proofs;
- grouping across optional PathTempo phase boundaries within one timing
  PathPiece; and
- explicit bounded failure and resource diagnostics.

The existing cubic execution path remains unchanged. The quintic prototype
does not change packetization, stop tails, feed hold, activation ownership, or
servo evaluation.

## Current benchmark

The primary diagnostic command is:

```powershell
.\build\ngc_simulation_diagnostic.exe adaptive_pockets.ngc 100000 15 `
  --smoother=uniform `
  --mode=optimized `
  --continuous-check=geometry `
  --sampled-corrections=on
```

Representative results with `trajectory.lookahead_duration = 400` are:

| Measurement | Result |
|---|---:|
| Timing pieces | 7,759 |
| Path duration | 260.595 s |
| Current emitted cubic spans | 24,325 |
| Cubic spans with material constraint violations | 5,415 |
| Worst cubic ratio | 1.925392 path jerk |
| Accepted curved quintics | 19,041 |
| Retained linear cubics | 1,675 |
| Incomplete candidate span total | 20,716 |
| PathTempo phase boundaries absorbed by grouping | 4,734 |
| Maximum phases in one accepted quintic | 5 |
| Unresolved quintic intervals | 15 |
| Quintic resource exhaustion | No |
| Worst accepted quintic ratio | 0.999989 |
| Current cubic materialization time | approximately 0.10 s |
| Quintic diagnostic time | approximately 0.68 s |
| Total planner time | approximately 3.49 s |

The candidate total is incomplete because it excludes the 15 unresolved
intervals. It must not be interpreted as an RT-ready span count.

## The 15 unresolved intervals

Every unresolved interval passes geometry and forward-progress proof and fails
only the continuous path-jerk constraint:

| Failure classification | Count |
|---|---:|
| Whole timing PathPiece | 4 |
| Beginning of a timing PathPiece | 3 |
| End of a timing PathPiece | 8 |
| Interior of a timing PathPiece | 0 |
| Geometry failures | 0 |
| Forward-progress failures | 0 |
| Path-jerk failures | 15 |

The certified Bézier bound and an independent 128-interval evaluation agree for
these failures. They are not false positives caused by sparse checking.

Eleven failures have nonzero scalar acceleration at at least one endpoint, but
four already have zero acceleration at both endpoints. The first whole-piece
failure lasts about 112.6 microseconds, has zero endpoint acceleration, and
reaches 1.259600879 times the path-jerk limit. The worst unresolved interval
reaches 1.677399798 times the path-jerk limit.

This evidence rules out local zero boundary acceleration as a complete
solution. It also shows that the remaining problem is not merely PathTempo's
discrete differential stations. The likely source is the unique quintic
interpolant required to match the interval's endpoint PVA.

## Approaches investigated

### Grouping within one timing PathPiece

PathTempo scalar phase boundaries are treated as optional rather than mandatory
execution boundaries. The grouping search can absorb up to five phases into
one certified quintic and reduces the candidate representation further.

It does not eliminate the remaining failures because all of them touch timing
PathPiece boundaries. Absorbing those intervals would require grouping and
geometry proof across adjacent PathPieces.

### Fixed minimum quintic durations

Replacing the state-dependent numerical conditioning gate with fixed duration
floors produced non-monotonic results:

| Minimum quintic duration | Unresolved intervals |
|---:|---:|
| Existing state-dependent gate | 15 |
| 50 microseconds | 7 |
| 25 microseconds | 5 |
| 10 microseconds | 10 |
| 1 microsecond | 80 |

At one microsecond, endpoint-state cancellation produced jerk ratios as high as
approximately 1,800. A tuned constant is therefore not a sound general
solution. The state-dependent gate has been restored.

### PathTempo materialization correction

NGC could return all unresolved jerk corrections to PathTempo together. The
worst ratio implies a local time-scale correction of approximately
`cuberoot(1.6774) = 1.188`, before a safety margin.

This is the simplest correctness mechanism and reuses the existing correction
contract. It may, however, be expensive. PathTempo's internal scalar
optimization passes are much cheaper than NGC materialization callback passes.
A callback pass reconstructs and proves axis polynomials and can invalidate
neighboring cache entries when boundary states propagate.

Returning all corrections together should be much better than a long sequence
of small corrections, but its cost has not yet been measured.

## Open decision

The branch currently leaves the 15 failures explicit and has no curved-cubic
fallback. The leading choices are:

1. **Cross-PathPiece quintic grouping.** This avoids replanning but requires a
   composite geometry proof and careful treatment of per-piece feed limits,
   activations, packet boundaries, stop tails, and ownership.
2. **One batched PathTempo correction pass.** This is architecturally simpler
   and uses an existing interface, but may add significant NRT window-planning
   latency.
3. **A different local polynomial construction.** A coupled multi-quintic
   solve could add degrees of freedom without crossing semantic boundaries,
   but it would be a new algorithm and proof burden.

Before selecting one, evaluate the original PathTempo scalar law and actual
geometry at the exact time of each failed quintic's maximum jerk. This will
measure the original coupled geometric jerk beside the quintic jerk and
conclusively separate time-law feasibility from materialization error.

## Requirements before RT integration

Even after resolving all constraint failures, quintics must not enter the
backend until the following work is complete:

- define a bounded degree-aware execution representation;
- prove packet and fixed-capacity limits;
- preserve command and presentation activation ownership;
- generate and verify stop tails from quintic branch states;
- verify feed hold and resume over quintic spans;
- evaluate quintic PVA and jerk at servo time without allocation;
- add exact execution and packet-boundary regressions; and
- benchmark RT evaluation cost against the current cubic representation.

## Validation

At this checkpoint:

- the Release build succeeds;
- `ngc_tests.exe` passes;
- all eight auxiliary CTest registrations pass;
- `path_tempo_tests.exe` passes;
- `path_tempo_continuous_path_options_tests.exe` passes; and
- scoped `git diff --check` passes.
