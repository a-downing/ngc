# Arc Endpoint Continuity Handoff

## Objective (completed)

Fix a real position discontinuity in the RT-planned red trajectory at command boundaries, most visibly where rounded-IJK arcs meet another arc or line.

The completed fix must guarantee:

```cpp
plannedArcPosition(0.0) == MoveArc::from();
plannedArcPosition(pathLength) == MoveArc::to();
```

Consecutive planned commands must therefore share the same position exactly (within normal floating-point comparison tolerance), without drawing a synthetic connector to hide a planning error.

## Confirmed cause

`ExactStopTrajectoryPlanner` calculates both the start and end radial arms, but its current `positionAt()` rotates only `startArm`:

```cpp
const auto arm = rotate(startArm, sweep*u, axis);
const auto xyz = value.center() + arm + scale(axial, u);
```

This preserves the starting radius for the entire planned arc. G-code arc validation deliberately permits a small start/end radius mismatch caused by decimal IJK rounding. When the radii differ within `Machine::arcTolerance()`, the planned polynomial finishes at the reconstructed start-radius circle rather than at `MoveArc::to()`.

Immediately afterward, the planner assigns:

```cpp
m_position = value.to();
```

The next command therefore starts at the canonical commanded endpoint, leaving a real gap between the previous polynomial endpoint and the next polynomial start.

The relevant implementation is in:

- `src/ExactStopTrajectoryPlanner.cpp`
- `src/ArcInterpolation.cpp`
- `src/include/machine/ArcInterpolation.h`
- `src/test.cpp`

## Why this must be fixed in planning

- A physical executor cannot jump instantaneously across the gap.
- Exact-stop mode requires zero boundary velocity and acceleration as applicable, but it still requires continuous position.
- The red diagnostic overlay correctly exposes the planned gap.
- Adding a display-only connector would conceal an invalid trajectory.
- The servo period and red tessellation did not cause this defect; zooming merely revealed it.

## Existing endpoint-exact preview geometry

The canonical preview already blends both radial arms in `src/ArcInterpolation.cpp`:

```cpp
const auto radial =
    scale(rotate(startArm, sweep*t, axisUnit), 1.0-t)
  + scale(rotate(endArm, -sweep*(1.0-t), axisUnit), t);
```

This reaches both endpoints exactly. Because the two rotated arms point in the same traversal direction at a given parameter, this is effectively a linearly changing radius along the angular sweep. It is slightly non-circular when the accepted start/end radii differ.

Do not simply copy this expression into the planner without updating its derivatives, path-length parameterization, curvature limit, and tolerance verification.

## Recommended implementation

### 1. Unify endpoint-exact arc reference geometry

Move or expose reusable arc-reference operations through the project-owned arc geometry layer so preview and planning cannot silently diverge again.

The shared reference should provide, at minimum:

- Position at a normalized parameter.
- First derivative with respect to that parameter.
- Path length or a verified path-length approximation.
- Mapping from path distance to normalized parameter.
- Sufficient bounds for cubic tolerance verification.

Keep this NRT geometry code out of `MotionBackend`.

### 2. Use both radial arms

For normalized parameter `u`:

```text
startRotated(u) = rotate(startArm, sweep*u, axis)
endRotated(u)   = rotate(endArm, -sweep*(1-u), axis)
radial(u)       = (1-u)*startRotated(u) + u*endRotated(u)
xyz(u)          = center + radial(u) + axial*u
```

This guarantees exact start and end positions.

The analytical derivative is:

```text
dStart/du = sweep * cross(axis, startRotated)
dEnd/du   = sweep * cross(axis, endRotated)

dRadial/du = -startRotated
             + (1-u)*dStart/du
             + endRotated
             + u*dEnd/du
```

Add `axial` and rotary XYZABC derivatives as appropriate.

### 3. Preserve feed semantics with path-length parameterization

When the radii differ, the magnitude of `dPosition/du` is not constant. The current shortcut `u = distance / length` is therefore not an exact arc-length parameterization.

Use an NRT-only method such as:

1. Adaptively integrate `|dPosition/du|` over `[0,1]` to a strict tolerance.
2. Retain a monotonic cumulative-length table or local integration structure.
3. Invert distance to `u` with bounded binary/Newton refinement.
4. Return the unit path tangent:

   ```text
   tangent(distance) = (dPosition/du) / |dPosition/du|
   ```

XYZABC contributions must remain consistent with the planner's current aggregate path metric.

This is all non-real-time work and may use dynamic storage. The published RT representation must remain bounded `AxisPolynomialSpan` values.

### 4. Update curvature and speed limiting

The current curvature cap assumes a constant-radius circular helix:

```cpp
const auto curvature = radius * (sweep / length) * (sweep / length);
```

That is no longer authoritative for linearly changing radius. Calculate or conservatively bound curvature over the endpoint-exact reference curve. Adaptive interval sampling with a proven margin is acceptable in this NRT planner, followed by the existing emitted polynomial acceleration/jerk validation and time stretching.

Final axis-space verification remains authoritative.

### 5. Update cubic tolerance verification

The current verifier in `ExactStopTrajectoryPlanner.cpp` uses a constant-radius circular/helical chord-error bound:

```cpp
radius * (1.0 - cos(angle/2))
```

This bound is insufficient once radius varies between the accepted start and end arms.

Preserve the existing important properties:

- Ordered association between each polynomial interval and its source-arc interval.
- Recursive Bezier control-hull subdivision.
- Acceptance only when the full polynomial hull is inside the configured tolerance tube.
- No unordered-nearest-geometry checks.

Possible safe approaches:

- Derive a conservative chord bound for the linearly varying-radius spiral/helix, including both angular sagitta and the radius-change term.
- Or recursively bound/sample the endpoint-exact reference interval with a reserved reference-error budget, then verify the polynomial hull against the resulting ordered capsules.

The reference approximation error plus polynomial-to-reference-capsule error must not exceed `arcChordTolerance`.

### 6. Preserve exact endpoints explicitly

Even after reference evaluation, construct or overwrite final boundary samples from the canonical command endpoints where appropriate:

```cpp
sample(0).position      = value.from();
sample(pathLength).position = value.to();
```

Do not use this as a substitute for correct interior geometry or tangent calculation. It is only a final floating-point endpoint guarantee.

## Required regression tests

Add tests in `src/test.cpp` covering all of the following.

### Endpoint guarantee

- An arc with a small accepted start/end radius mismatch.
- First polynomial begins exactly at `MoveArc::from()`.
- Last polynomial ends at `MoveArc::to()`.

### Consecutive command continuity

- Two consecutive rounded-IJK arcs with a shared canonical junction.
- Arc followed by line.
- Line followed by arc.
- Previous chunk's final `normalMotion` endpoint equals the next chunk's first polynomial start.
- No position gap at the shared junction.

### Geometry variants

- CW and CCW traversal through the signed axis convention.
- Minor and major arcs.
- Full circle.
- Helical arc.
- Non-XY active-plane equivalents if constructed through canonical `MoveArc` axis values.
- Accepted radius mismatch near, but not beyond, `Machine::arcTolerance()`.

### Tolerance and dynamics

- Dense independent sampling confirms geometric deviation is within `arcChordTolerance`.
- Acceleration and jerk checks continue to pass.
- Span-count regression expectations are updated only if the safer reference geometry legitimately changes them.

### Rejection boundary

- Radius mismatch beyond `Machine::arcTolerance()` remains a recoverable interpreter error and never reaches the planner as valid motion.

## Current related behavior

The red diagnostic overlay now tessellates retained executed polynomial spans according to `simulation.servo_period`, rather than using a fixed 32 samples. The Windows scheduler is independently configured:

```toml
[simulation]
servo_period = 0.001
scheduler_period = 0.01
```

At 1x speed, the 100 Hz scheduler calls the mock backend ten times per wake, each with a fixed 1 ms timestep. The integer speed multiplier multiplies that tick batch. Scheduler batching must remain independent of the arc endpoint fix.

## Worktree context

The worktree is intentionally dirty from the completed work in the current session. Preserve these changes:

- Dear ImGui updated to v1.92.8 with required API renames.
- GLFW remains at latest stable 3.4.
- Adaptive tolerance-verified cubic arc span generation and span-count tests.
- Fixed servo timestep plus independent 100 Hz Windows scheduler batching.
- Servo-period-based red diagnostic tessellation.
- Timing diagnostics and batch-completion synchronization.

The arc endpoint-continuity fix described in this document has now been implemented. Preview and exact-stop planning share endpoint-exact reference geometry; planning uses arc-length parameterization, updated tangent/curvature/tolerance handling, and explicit canonical boundary samples. Regression coverage includes rounded-radius endpoint continuity, arc/line command junctions, geometry variants, cubic tolerance, dynamic limits, and recoverable rejection beyond `Machine::arcTolerance()`.

Each arc reference now also retains a fixed 16-entry bit-exact inverse-result cache. A cache miss still starts from the preintegrated ordered bracket and runs safeguarded Newton against the actual adaptive integral; only the result for that exact distance and reference is reused. On `adaptive_pockets.ngc` this reduced adaptive arc inverse integrals from 7,593,087 to 3,130,549 without changing the exported model, canonical endpoints, verified spans, duration, or proof counters. Dense line/arc, `1001.ngc`, rounded-IJK, full-circle, helical, major-sweep, and non-XY regression coverage remains authoritative. Do not generalize this cache into tolerance-near matching or unchecked inverse interpolation.

Run before editing:

```powershell
git status --short
git diff --check
```

## Build and test

From the repository root:

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

In the Codex sandbox, the WinGet Ninja shim may be inaccessible. The vcpkg-downloaded Ninja used successfully in this session was:

```powershell
& 'C:\Users\Andrew\Desktop\development\vcpkg\downloads\tools\ninja-1.13.2-windows\ninja.exe' -C build
ctest --test-dir build --output-on-failure
```

The suite passed repeatedly before this handoff was written.
