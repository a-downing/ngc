# Response to the SCP planner follow-up (2026-07-19)

Assessment of [`scp_planner_followup_2026-07-19.md`](scp_planner_followup_2026-07-19.md),
which itself responds to [`scp_planner_review_2026-07-19.md`](scp_planner_review_2026-07-19.md).
Each factual correction in the follow-up was re-verified against current code
(`HEAD` `8c47ef0`) and the vendored HiGHS submodule.

Verdict: the follow-up is accurate and well-prioritized. All of its corrections to
the original review hold; one turns out to be slightly stronger than the follow-up
itself states.

## Corrections to the original review, re-verified

### 1. Station acceptance rule

Follow-up claim: the acceptance rule already accepts an exactly neutral local
step; it only rejects steps that worsen the two adjacent exact time laws beyond
tolerance. The original review's "locally neutral but globally enabling" phrasing
was imprecise.

Verified correct. `src/TrajectoryCompiler.cpp:2313`:

```cpp
if(newDuration>oldDuration*(1.0+1e-10)) continue;
```

Exactly neutral steps are accepted, and slightly-worsening steps within the
relative tolerance are accepted. The genuine limitation is rejection of
locally-worsening-beyond-tolerance steps and of coordinated multi-station moves.

### 2. Squared speed and the aggregate path-norm constraint

Follow-up claim: squared speed makes each coupled-acceleration vector component
affine in squared speed and path acceleration, but it does not make the aggregate
path-norm constraint an exact LP constraint; that still needs an SOCP or
conservative linear directions.

Verified correct. In `w = v^2` space, per-axis components of `t*a + kappa*w` are
affine in `(w, a)` and become exact LP rows. The aggregate
`|t*a + kappa*w| <= pathAcc` remains a norm constraint requiring tangent
directions (or an SOCP). The original review's "all axis/path acceleration rows
would be exact" overstated the path-norm side; per-axis exactness is the actual
win.

### 3. HiGHS small matrix values are discarded, not just counted

Follow-up claim: vendored HiGHS defaults `small_matrix_value` to `1e-9`; the
builder cutoff and the configured HiGHS threshold must be chosen together, since
HiGHS warns about and discards small values.

Verified correct, and stronger than either document stated. In
`highs/highs/util/HighsMatrixUtils.cpp:257-276`, entries with
`|value| <= small_matrix_value` are not copied into the packed matrix; the index
and value vectors are resized afterward (`HighsLpUtils.cpp:93-94`). The only
signal is a `kWarning` (`HighsMatrixUtils.cpp:321-329`) that `output_flag=false`
hides. Default `1e-9` confirmed in `highs/highs/lp_data/HighsOptions.h:787`.

So the LP HiGHS solves is not exactly the LP the builder emitted: a row
coefficient such as `2*kappa*v_ref` below `1e-9` (low-speed stations,
small-curvature axes) silently vanishes, loosening that row. The exact acceptance
gates still prevent any unsafe outcome, but this is arguably more than hygiene.
The original review's "harmless extra nonzeros" framing was wrong.

Practical implication: align `SparseLpBuilder::addRow`'s cutoff with an
explicitly configured HiGHS `small_matrix_value` (e.g. both `1e-9`, or both at
HiGHS's minimum allowed `1e-12`), so that what the builder intends is what the
solver sees.

### 4. Objective rescale and suffix-probe caveats

Follow-up qualifications accepted:

- Scaling the trapezoid gradient by `actual_time / trapezoid_time` is a plausible
  surrogate weighting, not the true derivative of jerk-limited duration and not a
  guaranteed improvement.
- Skipping SCP for rolling suffix probes is not guaranteed to preserve boundary
  acceptance; some seeds may need SCP improvement to avoid pathological
  correction. Treat it as a measured feasibility-only mode.

## Assessment of the prioritized work plan

The ordering is right: availability/correctness first (replay repair,
resource-exhaustion fallback), then measured cost items (trial caching),
formulation experiments behind benchmarks (reachability rows, basis reuse,
feasibility-only suffix probe), hygiene batch, and squared speed as a deferred
redesign.

### Item 4 (adjacent-station reachability relaxations)

Both row directions re-derived; they are the correct linearization of
`v1^2 - v0^2 <= 2*a_limit*length` at the reference:

```text
2*v1_ref*v1 - 2*v0_ref*v0 <= 2*a_limit*length + (v1_ref^2 - v0_ref^2)
2*v0_ref*v0 - 2*v1_ref*v1 <= 2*a_limit*length - (v1_ref^2 - v0_ref^2)
```

Reference containment holds with the reachability seed because the seed's
jerk-limited `reachableVelocity` implies the energy bound at the same `a_limit`.
After correction passes shrink `localLimits`, the rows tighten consistently.
Sound as specified. The note about tracking row offsets explicitly (instead of
relying on `firstDeviationRow = 2*station`) is good hygiene regardless of where
the new rows are inserted.

### Item 3 (trial materialization caching)

The baseline numbers support investigating it: 9390 materialization attempts
against 723 accepted steps is about 13 trials per acceptance. Caveat worth
recording: the bit-exact candidate cache only hits on identical 8-field keys
(length, boundary velocities/accelerations, and local limits), so repeats come
mostly from uncorrected regions across correction passes. The follow-up's
criterion of measuring decreased solver calls and materializations on a
repeat-heavy workload is the right way to validate it rather than assuming.

### Item 2 (resource-exhaustion fallback)

Forcing the policy path deterministically in a regression is straightforward:
a tiny `scpSolveTimeLimit` or `scpSimplexIterationLimitMultiplier = 1` passes the
existing validated effort checks (`scpSolveTimeLimit` only needs
`0 < limit <= 60`; the multiplier only needs `1..4096`). Keeping model errors,
infeasibility, unboundedness, and containment failures fatal is correct, since
reference containment makes a well-formed LP feasible by construction.

### Item 5 (basis reuse)

Applying a saved basis only after checking row/column dimensions is right;
HiGHS `setBasis` validates dimensions and returns an error on mismatch, so
misuse fails safe. Agree with not raising default `scpIterations` until basis
reuse and trial caching are benchmarked together.

### Item 6 (feasibility-only suffix probe)

An explicit mode is the right shape, since `scpIterations == 0` is rejected by
the validated effort range (`TrajectoryCompiler.cpp:1219`). The mode must retain
the velocity-only seed, correction passes, exact materialization, full geometric
and emitted-polynomial verification, capacity proof, and stop safety, exactly as
the follow-up lists.

## Additions to the plan

- The original review's objective-floor note did not make it into the follow-up's
  item 7 hygiene list: the `max(1e-6, v0+v1)` guard in the objective
  (`TrajectoryCompiler.cpp:2007`) is absolute and yields ~1e12-scaled gradients
  on degenerate near-zero-speed pieces. Rare in practice (the seed keeps interior
  stations well above zero), but a physics-scaled floor (a fraction of the local
  velocity cap) is safer. Add it to item 7.
- The follow-up's flaky persistent-G43 simulation assertion (intermittent failure
  on a no-op rebuild and rerun, then passing on retry) is a separate issue worth
  isolating. Intermittent failure without code changes suggests state leakage
  between runs, not the SCP changes. It should be tracked independently of this
  planner work.

## Status

No code changed by this document. Implementation of follow-up items 1-3 (replay
repair, resource-exhaustion fallback, trial caching) is unambiguous and can
proceed; items 4-6 are gated on benchmark evidence as the follow-up specifies.
