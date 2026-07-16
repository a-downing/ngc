# Clarabel trajectory oracle

This is a non-production development oracle for the G64 scalar timing law. It does not enter `MotionBackend`, packetization, stop-tail generation, or the real-time contract.

The C++ exporter evaluates one compatible positive-`P` G64 horizon with the normal NGC geometry and planner. Every retained primitive or blend piece is divided into 32 midpoint intervals so a long line can contain acceleration, cruise, and braking states while curved geometry exposes its changing acceleration cones. It records the current jerk-limited planner duration, conservative local velocity caps, tangent, curvature, and configured aggregate/per-axis acceleration limits.

The Rust program uses Clarabel 0.11.1 to solve a convex discretized minimum-time path parameterization. Its variables include squared station speed `x = v^2`, station speed, and interval duration. Rotated second-order cones enforce `v^2 <= x` and the exact constant-path-acceleration interval time `dt = 2 ds / (v_i + v_{i+1})`. Standard second-order cones enforce

```text
norm(q'(s) * s_ddot + q''(s) * s_dot^2) <= path_acceleration
```

and linear inequalities enforce every configured per-axis acceleration limit.

The default solve remains acceleration-only. The v3 model also exports aggregate/per-axis jerk, `q'''`, and the production planner's piece-boundary velocity/acceleration states and durations. Passing `--jerk-aware` runs sequential convex Clarabel subproblems around the latest squared-speed profile. Each subproblem linearizes the finite-difference collocation form of `q' s_jerk + 3 q'' s_dot s_ddot + q''' s_dot^3`, uses a bounded trust region and normalized jerk slack, and is accepted only after the nonlinear discrete jerk is recomputed independently. This is a locally converged jerk-aware reference, not a certified global optimum or an executable trajectory. Existing emitted-polynomial verification remains authoritative.

Build and run from the repository root:

```powershell
cmake --build build --target ngc_g64_oracle_export
build\ngc_g64_oracle_export.exe g64_dense_timing_test.ngc build\dense_oracle.txt machine.toml
cargo run --release --manifest-path tools\clarabel_trajectory_oracle\Cargo.toml -- build\dense_oracle.txt build\dense_oracle_solution.csv
tools\clarabel_trajectory_oracle\target\release\ngc-clarabel-trajectory-oracle.exe build\dense_oracle.txt --jerk-aware
```

An optional final argument scales aggregate and every finite per-axis jerk limit together without changing velocity or acceleration. This is a development diagnostic for separating dynamic-jerk loss from acceleration-reachability loss:

```powershell
build\ngc_g64_oracle_export.exe adaptive_pockets.ngc build\adaptive_jerk_10.txt machine.toml 10
```

Rolling planner convergence can be profiled without changing production defaults. The
`--effort` profiles vary reachability sweeps and velocity refinement, and `--rolling-only`
avoids generating the full-horizon oracle model:

```powershell
build\ngc_g64_oracle_export.exe adaptive_pockets.ngc build\adaptive_effort.txt machine.toml 1 --rolling-only --effort=combined40
```

`current` reproduces production effort. `velocity6`, `velocity8`, `velocity10`, and
`velocity12` isolate velocity-search refinement; `medium`, `high`, and `extreme` isolate
8, 20, and 40 reachability sweeps; `combined`, `combined20`, and `combined40` use eight
velocity refinements with 8, 20, and 40 sweeps. These remain local-search diagnostics,
not global-optimality proofs. On `adaptive_pockets.ngc`, velocity refinements above eight
have produced severe verified-correction slowdowns and should not be treated as higher-quality
solutions merely because they perform more local search.

`--effort=no-derivative-cap` is an offline experiment that removes only the preliminary
path/per-axis curvature-derivative velocity ceilings. Coupled `(v,a,j)` checks and exact
emitted-polynomial verification remain active. It is deliberately not a production default.
`derivative125`, `derivative150`, `derivative200`, `derivative250`, and `derivative300`
instead multiply those preliminary velocity ceilings while retaining the same final gates.
The response is geometry-dependent: larger multipliers can trigger severe local correction
and are not inherently higher-quality solutions.

The optional CSV contains the solved interval speeds, scalar acceleration, duration, and coupled acceleration norm for inspection.

`--coarsen=N` combines each group of exported intervals within one geometry piece. `--refine=N` splits intervals while retaining their sampled geometry. They are convergence diagnostics and cannot be combined.

## Jerk-aware sequential-convex comparisons

The full dynamic jerk problem is nonconvex, so Clarabel cannot certify its global time optimum in one conic solve. These results are independently jerk-feasible at the finite-difference stations but remain local, discretized references. They also retain the planner-exported local velocity caps, which are conservative sufficient limits rather than a relaxation of every physically possible cancellation.

| Program | Intervals/piece | Planner | Jerk-aware Clarabel SCP | Planner excess over reference |
| --- | ---: | ---: | ---: | ---: |
| Dense fixture | 8 | 2.423589511 s | 2.156303418 s | 12.395% |
| Dense fixture | 16 | 2.423589511 s | 2.196934691 s | 10.317% |
| Dense fixture | 32 | 2.423589511 s | 2.209524075 s | 9.688% |
| Dense fixture | 64 dynamics refinement | 2.423589511 s | 2.216415722 s | 9.347% |
| `1001.ngc` | 16 | 95.338245391 s | 88.823845456 s | 7.334% |
| `1001.ngc` | 32 | 95.338245391 s | 89.638000361 s | 6.359% |

The dense 32-to-64 change is 0.006891647 seconds (0.31% of the 64-way result), so its remaining gap is reasonably stable. The `1001.ngc` 16-to-32 change is 0.814154905 seconds (0.91%); a 64-way SCP encountered repeated Clarabel numerical/insufficient-progress statuses and did not reach discrete jerk feasibility.

For `adaptive_pockets.ngc`, the 309,120-interval v3 export succeeds. A four-interval-per-piece jerk SCP (38,640 segments) did not complete within five minutes. A one-interval-per-piece solve converged but produced an obviously unusable 1,737.19-second result versus the 621.92-second planner because its dynamics grid was too coarse. No meaningful jerk-aware adaptive-pocket duration is currently claimed.

The jerk-aware report ranks production-versus-oracle duration gaps by geometry piece. On the dense fixture the loss clusters in alternating 0.006/0.014-unit blend pieces where the oracle carries roughly 0.3–0.44 unit/s more station speed. On `1001.ngc`, the largest repeated 0.080-second piece gaps have essentially identical boundary speeds but about +4.24/-4.24 unit/s² oracle acceleration carry. These values are diagnostics, not states that may be copied into production.

## Recorded comparisons

These are development measurements, not executable trajectories or guaranteed lower bounds. The first table records the earlier subdivision study; the current exporter isolates the first compatible G64 feed/arc horizon without changing the correction ceiling.

| Program/model | Motions | Intervals | Planner duration | Clarabel duration | Gap versus planner | Solve time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Dense fixture, 16/piece | 22 | 688 | 2.424964579 s | 1.767305546 s | 27.120% | about 0.070 s |
| `1001.ngc`, 16/piece | 242 | 5,504 | 95.447292693 s | 81.535928748 s | 14.575% | 0.619791 s |
| `adaptive_pockets.ngc`, 4/piece | 5,164 | 38,640 | 622.540851901 s | 463.583039019 s | 25.534% | 10.052023 s |
| `adaptive_pockets.ngc`, 16/piece | 5,164 | 154,560 | 622.540851901 s | 447.064353307 s | 28.187% | 35.462374 s |
| `adaptive_pockets.ngc`, 32/piece | 5,164 | 309,120 | 622.540851901 s | 446.160170792 s | 28.332% | 50.661873 s |

The adaptive result changed by 16.518685712 seconds between four and sixteen intervals per piece, so the four-interval model is too coarse. It changed by another 0.904182515 seconds, or about 0.202%, between sixteen and thirty-two intervals. Thirty-two intervals have not yet been proven converged. The refined result makes the current adaptive trajectory about 39.53% longer than the acceleration-only oracle, but dynamic jerk and discretization error prevent treating that entire difference as recoverable production time.

The older 622.540851901-second measurement required a temporary 64-pass correction experiment. Subsequent coupled-state timing, exact Ruckig phase preservation, and the retained 1% correction reserve compile the same horizon under the normal ceiling; current measurements are below.

## Jerk-limit sweep

The coupled-limit-aware planner was compared with a freshly generated 32-way oracle after scaling aggregate and finite axis jerk limits together. Velocity and acceleration limits remained unchanged.

| Program | Jerk | Verified planner | Clarabel | Planner excess over oracle |
| --- | ---: | ---: | ---: | ---: |
| Dense fixture | 1x | 2.423589511 s | 1.753531942 s | 38.212% |
| Dense fixture | 10x | 1.601229453 s | 1.478985519 s | 8.265% |
| Dense fixture | 100x | 1.333088266 s | 1.329211801 s | 0.292% |
| Dense fixture | 1000x | 1.352727075 s | 1.351002408 s | 0.128% |
| `adaptive_pockets.ngc` | 1x | 621.916364725 s | 446.679202750 s | 39.231% |
| `adaptive_pockets.ngc` | 10x | 309.467499743 s | 277.916090073 s | 11.353% |
| `adaptive_pockets.ngc` | 30x | 233.634337079 s | 224.038614824 s | 4.283% |

The dense planner nearly coincides with the acceleration-only oracle by 100x jerk. The adaptive gap also collapses rapidly through 30x. Exact outgoing Ruckig phase jerk removed reconstruction error and made every supported adaptive result slightly faster. Adaptive 100x now stops at the explicit per-pass geometry-proof budget; 1000x stops at the acceleration-aware candidate-evaluation budget. These deterministic failures replace recursion-depth/correction instability in an extreme, poorly conditioned regime. Dense 1000x was slightly slower than 100x in both planner and oracle results, so it should not be interpreted as a monotone physical limit.

This is strong evidence that dynamic jerk accounts for most of the normal-limit Clarabel gap and that the existing planner is close to the acceleration-only optimum when jerk becomes nonbinding. It does not prove normal-jerk time optimality because Clarabel omits `q' j + 3 q'' v a + q''' v^3`; a jerk-aware oracle would be needed for that claim.
