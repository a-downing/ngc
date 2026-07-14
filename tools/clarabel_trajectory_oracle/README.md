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
