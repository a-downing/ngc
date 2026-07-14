# Clarabel trajectory oracle

This is a non-production development oracle for the G64 scalar timing law. It does not enter `MotionBackend`, packetization, stop-tail generation, or the real-time contract.

The C++ exporter evaluates one compatible positive-`P` G64 horizon with the normal NGC geometry and planner. Every retained primitive or blend piece is divided into 16 midpoint intervals so a long line can contain acceleration, cruise, and braking states while curved geometry exposes its changing acceleration cones. It records the current jerk-limited planner duration, conservative local velocity caps, tangent, curvature, and configured aggregate/per-axis acceleration limits.

The Rust program uses Clarabel 0.11.1 to solve a convex discretized minimum-time path parameterization. Its variables include squared station speed `x = v^2`, station speed, and interval duration. Rotated second-order cones enforce `v^2 <= x` and the exact constant-path-acceleration interval time `dt = 2 ds / (v_i + v_{i+1})`. Standard second-order cones enforce

```text
norm(q'(s) * s_ddot + q''(s) * s_dot^2) <= path_acceleration
```

and linear inequalities enforce every configured per-axis acceleration limit.

Dynamic jerk is intentionally not modeled. Geometry-derived velocity caps already include the current planner's sampled curvature and curvature-derivative ceilings, but the omitted `q' s_jerk + 3 q'' s_dot s_ddot` terms mean the Clarabel duration is an optimistic reference, not an executable trajectory. Existing emitted-polynomial verification remains authoritative.

Build and run from the repository root:

```powershell
cmake --build build --target ngc_g64_oracle_export
build\ngc_g64_oracle_export.exe g64_dense_timing_test.ngc build\dense_oracle.txt machine.toml
cargo run --release --manifest-path tools\clarabel_trajectory_oracle\Cargo.toml -- build\dense_oracle.txt build\dense_oracle_solution.csv
```

The optional CSV contains the solved interval speeds, scalar acceleration, duration, and coupled acceleration norm for inspection.

## Recorded comparisons

These are development measurements, not executable trajectories or guaranteed lower bounds. The complete-program tests temporarily isolated their first compatible G64 feed/arc horizon. `adaptive_pockets.ngc` additionally used a temporary 64-pass local-correction ceiling because the committed 12-pass ceiling stops at its previously reported near-limit path-jerk failure.

| Program/model | Motions | Intervals | Planner duration | Clarabel duration | Gap versus planner | Solve time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Dense fixture, 16/piece | 22 | 688 | 2.424964579 s | 1.767305546 s | 27.120% | about 0.070 s |
| `1001.ngc`, 16/piece | 242 | 5,504 | 95.447292693 s | 81.535928748 s | 14.575% | 0.619791 s |
| `adaptive_pockets.ngc`, 4/piece | 5,164 | 38,640 | 622.540851901 s | 463.583039019 s | 25.534% | 10.052023 s |
| `adaptive_pockets.ngc`, 16/piece | 5,164 | 154,560 | 622.540851901 s | 447.064353307 s | 28.187% | 35.462374 s |

The adaptive result changed by 16.518685712 seconds between four and sixteen intervals per piece, so the four-interval model is too coarse. Sixteen intervals have not yet been proven converged. The refined result makes the current adaptive trajectory about 39.25% longer than the acceleration-only oracle, but dynamic jerk and discretization error prevent treating that entire difference as recoverable production time.

Temporarily allowing up to 64 local correction passes let the existing planner compile all 5,164 adaptive-pocket motions and verify a 622.540851901-second trajectory. This indicates that the old 12-pass failure is an iteration-ceiling problem, not demonstrated path infeasibility. The source was restored afterward; these recorded measurements do not change planner behavior.
