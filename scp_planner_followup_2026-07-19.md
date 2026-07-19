# SCP planner investigation and follow-up

This is a working companion to
[`scp_planner_review_2026-07-19.md`](scp_planner_review_2026-07-19.md). It records
the investigation outcome and the intended implementation order. Current code
and regression tests remain authoritative. When the work is complete, mark this
document historical rather than treating it as a permanent specification.

[`scp_planner_review_response_2026-07-19.md`](scp_planner_review_response_2026-07-19.md)
independently re-verified the investigation baseline and agreed with the
follow-up's corrections and ordering. That response reviewed `HEAD` `8c47ef0`
before item 1 was implemented, so its statement that replay repair could proceed
is historical rather than a review of the completed fix below. Its additional
model-fidelity and testing conclusions are incorporated here.

Investigation baseline: `HEAD` `8c47ef0`. The working tree had unrelated local
changes in `machine.toml`, `TrajectoryPlanner.h`, `test.cpp`, and the simulation
diagnostic. `src/TrajectoryCompiler.cpp` was unchanged, so the review's planner
findings still applied.

## Overall assessment

The review is technically strong and mostly accurate. The station-visit replay
bug, fatal handling of HiGHS resource limits, lack of inter-station reachability
rows, raw-velocity formulation, and principal solver-time opportunities are
confirmed.

No unsafe trajectory publication was found. The existing exact time-law
materialization, coupled-limit checks, emitted-polynomial verification, capacity
gates, and stop-branch gates remain final authority. The confirmed defects affect
planner availability, planning cost, and trajectory quality rather than bypassing
those safety gates.

## Corrections and qualifications to the review

- Graceful SCP fallback should apply to expected resource exhaustion, especially
  `HighsModelStatus::kTimeLimit` and `kIterationLimit`. Model errors, infeasibility,
  unboundedness, and broken reference containment should remain fatal because
  they indicate an invariant or integration failure.
- The station acceptance rule already accepts an exactly neutral local step. It
  rejects a step only when the two adjacent exact time laws increase in duration
  beyond tolerance. The actual limitation is rejection of a locally worsening
  step, or a coordinated multi-station change, that could enable a better global
  profile.
- Using squared speed makes each coupled-acceleration vector component affine in
  squared speed and path acceleration. It does not make the aggregate path-norm
  constraint an exact LP constraint. That constraint still requires an SOCP or a
  conservative set of linear directions.
- Scaling the trapezoid objective gradient by
  `actual_time / trapezoid_time` is a plausible surrogate weighting, not the true
  derivative of jerk-limited duration and not a guaranteed improvement.
- Vendored HiGHS defaults `small_matrix_value` to `1e-9`, while
  `SparseLpBuilder::addRow()` drops only coefficients at or below `1e-18`.
  HiGHS removes coefficients at or below its threshold while normalizing the
  passed model and returns a warning. The planner accepts that warning, and
  `output_flag=false` hides its log, so HiGHS can solve a looser LP than the
  builder expressed without a visible diagnostic. Exact acceptance and emission
  gates still prevent unsafe publication, but this is LP model fidelity rather
  than merely sparse-storage hygiene. Choose the builder cutoff and an explicitly
  configured HiGHS threshold together; changing only one does not align the
  constructed and solved models.
- Skipping SCP for rolling suffix probes is promising because their generated
  trajectory is discarded after proving stop feasibility. It is not guaranteed
  to preserve boundary acceptance: some seeds may need SCP improvement to avoid
  pathological correction. Treat this as a measured feasibility-only mode.

## Prioritized work

### 1. Repair station-visit replay

Status: completed

The rescue block moves `currentStationVisits` and `currentStationVisitSlots` into
the previous-pass vectors, then immediately clears both vectors. Remove the
unconditional clears, or execute them only when the rescue did not run. Preserve
the existing clears when acceleration-aware rescue is first enabled because they
discard state from the pre-rescue regime.

Implementation: the post-rescue clears now execute only when acceleration-aware
rescue did not run. The explicit reset when rescue is first enabled remains in
place. `StationVisitReplayDiagnostics::replayedVisits` records visits that take
the actual replay branch, separately from replay-disabled shadow matches.

Required regression evidence:

- Exercise acceleration-aware rescue for at least two correction passes.
- Show that a later rescue pass observes nonzero `comparableVisits`.
- Construct an unchanged visit where possible and show a nonzero exact input
  match and actual replay.
- Compare replay enabled and disabled plans to ensure identical accepted station
  state, time laws, emitted execution spans, and verification outcome.
- Show reduced candidate or scalar time-law work when an exact replay occurs.

Regression evidence: the focused cluster-spline timing case enters rescue after
pass 0 and continues through pass 4. It observes 6 comparable visits, 4 exact
input matches, and 4 actual replays. With replay disabled, all 4 exact inputs
produce exact output matches with no mismatches. A bit-exact semantic fingerprint
of piece timing, accepted station PVA boundaries, activation ownership, normal
execution spans, stop-tail spans, and packet branch states is identical between
the two plans; geometry-verification counts, high-water mark, and correction
history also match. With cross-compilation time-law cache sharing disabled for a
controlled comparison, replay reduces scalar time-law calls from 520 to 472,
Ruckig solver calls from 445 to 417, and winning-candidate materializations from
20 to 0. The reductions exactly equal the replay-disabled shadow savings.

### 2. Make expected HiGHS resource exhaustion nonfatal

Status: completed

When an SCP solve reaches its time or simplex-iteration limit, stop the SCP
improvement loop and retain the current feasible, exactly materialized station
state. Continue through ordinary emission and every downstream verification gate.
Do not use a partial HiGHS primal solution.

Keep configuration errors, model-passing errors, reference-containment failures,
infeasible or unbounded models, missing primal values after an allegedly optimal
solve, and other solver failures fatal.

Implementation: HiGHS `kTimeLimit` and `kIterationLimit` model statuses now stop
the current SCP improvement loop before `getSolution()` and retain the feasible,
exactly materialized reference. Every other non-optimal or failed solve still
uses the fatal error path. `ContinuousTrajectoryPlan::scpResourceFallback` is a
fixed-size NRT diagnostic containing the first fallback reason, correction pass,
SCP iteration, and the total number of fallback occurrences. The continuous-plan
summary exposes those fields.

Required regression evidence:

- Deterministically force the time-limit or iteration-limit policy path.
- Verify that the retained reference is emitted only after all normal exact gates
  pass.
- Record a bounded NRT diagnostic identifying the fallback status, correction
  pass, and SCP iteration.
- Verify that a genuine model or reference invariant failure is still fatal.

Testing qualification: a tiny wall-clock limit is not deterministic, and
`scpSimplexIterationLimitMultiplier = 1` still permits `variableCount` simplex
iterations rather than one iteration. Use a proven fixture that reliably reaches
the selected resource status, or add a narrow test seam for the status-policy
branch; do not accept a timing-dependent regression.

Regression evidence: the existing cluster-spline timing fixture, with a
`1e-12`-second solve limit, reliably returns HiGHS `kTimeLimit` on correction pass
0, SCP iteration 0. Compilation succeeds with a time-limit diagnostic, zero
accepted SCP steps, nonzero geometry-verification work, populated normal
execution spans, and populated stop tails. This proves that no partial HiGHS
primal is consumed and that the retained reference still passes the ordinary
exact gates. Compile-time policy assertions cover optimal acceptance, time- and
iteration-limit fallback, and fatal handling of every other classification. A
zero solve-time configuration remains a fatal bounded-effort error.

### 3. Reduce SCP trial materialization cost

Status: completed

Route line-search trials through the existing bit-exact candidate time-law cache.
Use cached success and duration while evaluating trials, then materialize only
the accepted left and right candidates. Preserve current instrumentation and
ensure the materialized winner exactly matches the candidate key.

Implementation: sparse-SCP working-set trials now query the compile-local
bit-exact time-law cache. A cache hit supplies only its recorded success and
duration during line search. Failed hits remain rejected; a successful hit is
re-solved only after the adjacent pair is selected for acceptance. Exact
materialization verifies the complete eight-field input key and bit-exact
duration before the time law can replace executable timing. A mismatch or a
previously successful result that cannot be reproduced is a detailed fatal
planning error. NRT effort controls retain an uncached A/B path and permit a
power-of-two cache-size override for focused measurement; production defaults
use cached trials and the automatic horizon-sized table.

Required regression evidence:

- Accepted station states, exact time laws, and plan duration remain unchanged.
- Solver calls and full materializations decrease on a repeat-heavy correction
  workload.
- Cached failures remain failures and cannot be selected.
- Cache collisions cannot return a nonmatching result.

Regression evidence: on the repeat-heavy cluster-spline correction fixture,
cached and uncached SCP trials produce identical semantic fingerprints covering
piece timing, accepted station PVA states, activation ownership, normal
execution spans, stop-tail spans, and packet branch states. Correction history,
geometry-verification count, and verification high-water mark also match. With
cross-compilation sharing disabled, caching reduces Ruckig solver calls from 445
to 408 and full SCP materialization attempts from 228 to 189. The cached run
observes 96 repeated failure hits, all of which remain rejected. A forced
one-entry direct-mapped cache records collisions while producing the same exact
plan and correction history, proving that a colliding nonmatching key is treated
as a miss. These are deterministic work-count reductions, not a wall-time
speedup claim.

### 4. Add adjacent-station reachability relaxations

Status: rejected experiment

Add linearized acceleration-energy inequalities in both directions for each
piece. They should shape the LP proposal without replacing exact jerk-limited
Ruckig acceptance:

```text
v1_ref^2 - v0_ref^2
    + 2*v1_ref*(v1 - v1_ref)
    - 2*v0_ref*(v0 - v0_ref)
    <= 2*a_limit*length

v0_ref^2 - v1_ref^2
    + 2*v0_ref*(v0 - v0_ref)
    - 2*v1_ref*(v1 - v1_ref)
    <= 2*a_limit*length
```

Track row offsets explicitly so reference-containment diagnostics do not depend
on the current assumption that the first deviation row is `2*station`.

Implementation: an explicit default-off NRT effort switch adds both linearized
rows per timing piece using the current local scalar acceleration limit and
piece length. Station-deviation and per-piece row-group bases are recorded
explicitly. Every constructed model still passes the complete reference-point
containment check before HiGHS runs. Plan diagnostics report the number of added
rows, station proposals, line-search trials, accepted steps, full
materializations, SCP time, and correction-pass count. The offline simulation
diagnostic accepts `--scp-reachability-rows=on|off` for repeatable A/B runs.

Evaluation criteria:

- The verified reference remains contained in every generated LP.
- Accepted-step fraction improves on horizons containing nearby curvature or
  feed caps.
- Final exact duration is no worse on the selected benchmark set.
- Solver time, materialization attempts, and correction-pass count are reported;
  no claim of improvement should be accepted from LP time alone.

Evaluation evidence: the focused pathological cluster fixture adds 120 rows but
produces the same bit-exact plan, 50 accepted steps from 55 proposals, 118 line
search trials, 189 full materializations, five correction passes, and the same
acceleration-aware rescue activation. Four representative first rolling
horizons produced these deterministic planning counts:

| Horizon | Accepted/proposed, off -> on | Trials, off -> on | Materializations, off -> on | Passes, off -> on |
| --- | ---: | ---: | ---: | ---: |
| `adaptive_pockets.ngc` (178 pieces) | 723/1239 -> 723/1239 | 5830 -> 5828 | 5995 -> 6021 | 7 -> 7 |
| `adaptive.ngc` (77 pieces) | 300/304 -> 300/304 | 332 -> 332 | 953 -> 953 | 4 -> 4 |
| `1001.ngc` (11 pieces) | 20/20 -> 20/20 | 20 -> 20 | 62 -> 62 | 2 -> 2 |
| `1002_3d.ngc` (179 pieces) | 694/1068 -> 690/1068 | 3762 -> 3765 | 4846 -> 4831 | 6 -> 6 |

No selected workload improved accepted-step fraction or correction count. The
178-piece horizon's final scalar timing increased slightly from
8.237769412604234 seconds to 8.237800265554284 seconds, and the 179-piece case
accepted four fewer station steps. Single-run SCP wall times were also higher in
all five comparisons, but that noisy observation is not used as proof. The
velocity-only seed is already adjacent-reachability limited; the remaining
exact rejections and pathological correction are dominated by jerk-limited and
emitted-polynomial behavior that these acceleration-energy rows do not model.
The switch therefore remains off by default, and the experiment does not
justify enabling the rows or removing acceleration-aware rescue.

### 5. Add basis reuse before increasing SCP iterations

Status: pending experiment

Retain a valid HiGHS basis across same-dimension SCP iterations and, where valid,
across correction passes. Apply it only after checking row and column dimensions.
Measure whether changed bounds, coefficients, and presolve behavior make reuse a
net improvement.

Do not raise the default `scpIterations` until basis reuse and trial caching are
benchmarked together. More iterations do not by themselves solve the greedy
working set's inability to accept locally worsening coordinated moves.

### 6. Evaluate a feasibility-only rolling suffix probe

Status: pending experiment

Add an explicit mode rather than setting `scpIterations` to zero through the
existing effort structure. The mode may skip quality improvement but must retain
the velocity-only seed, correction passes, exact materialization, complete
geometric and emitted-polynomial verification, capacity proof, and stop safety.

Compare boundary acceptance rate, suffix-probe latency, total rolling-search
latency, and the final published prefix duration against the normal SCP probe.

### 7. Apply low-risk LP model-fidelity and hygiene fixes

Status: pending

- Remove the unused `scpDuration` value.
- Record deviation-row bases explicitly.
- Clamp proposed acceleration to the same stored column bounds used by HiGHS.
- Skip endpoint acceleration and jerk rows that are provably implied for exact
  retained-line geometry.
- Reserve sparse row, index, and value storage using a conservative bound.
- Align the builder's coefficient cutoff with an explicitly selected HiGHS
  `small_matrix_value`, and verify that the passed model retains exactly the
  intended coefficients. Treat this as model-fidelity work, not just a sparse
  storage optimization.
- Replace the absolute `max(1e-6, v0+v1)` objective-gradient floor with a
  physics-scaled floor derived from the adjacent local velocity caps. Keep the
  objective finite and well-scaled on degenerate near-zero-speed pieces.
- Benchmark `presolve=off` only for a defined small-model threshold; do not make
  it an unconditional assumption.

### 8. Treat squared speed as a separate redesign

Status: deferred experiment

The research direction remains credible, especially for centripetal acceleration
near low-speed stations, but it is not a mechanical variable substitution. A
design must address:

- aggregate path-acceleration norm constraints;
- full geometric jerk terms containing powers of speed;
- the relationship between squared speed, path acceleration, and piece length;
- exact nonzero PVA rolling boundaries;
- feasible behavior near zero speed; and
- compatibility with the current exact time-law and emitted-polynomial gates.

Prototype and compare it as an alternative master formulation before replacing
the raw-velocity LP.

## Baseline observations

At the investigation baseline, the existing focused regression asserted only
that an SCP subproblem was solved and that at least one station update was
accepted. Item 1 now adds multi-pass rescue, actual replay, replay-disabled
equivalence, exact emitted-plan, verification-outcome, and scalar-work evidence.
Items 2 and 3 now cover expected solver-resource fallback and cached SCP trials.
Item 4 evaluated and rejected adjacent-station LP coupling for the selected
workloads. Basis reuse remains uncovered pending its experiment.

A bounded diagnostic using the locally modified machine configuration observed:

```text
pieces:                    178
correction passes:         7
HiGHS solves:              7
HiGHS simplex iterations:  3288
accepted station steps:    723
materialization attempts:  9390
SCP processing:            0.052061 seconds
total planner processing:  0.136593 seconds
```

This supports investigating trial materialization cost, but it is not a controlled
benchmark and must not be used as proof of a proposed speedup.

The unrelated persistent-G43 simulation failure was isolated as a terminal
snapshot-publication race: `Completed` became observable before final modal state
was copied. Completion now remains internal until final session state is
published atomically, and the fix is merged from `main` into this branch.

## Completion criteria

This follow-up is complete when every item is either implemented and covered by
focused regressions, or explicitly rejected with recorded benchmark evidence and
rationale. Final implementation claims should then be summarized in code-facing
documentation or tests, and this file should be marked historical.
