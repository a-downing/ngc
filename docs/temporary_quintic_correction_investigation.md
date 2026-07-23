# Temporary quintic correction investigation

This is a temporary handoff note for fixing the unnecessary first-pass
quintic materialization corrections from a fresh Codex context. Remove this
file after the fix and its regressions are complete.

## Repository state

- Branch: `wip/quintic-materialization`
- Production-quintic commit before this note: `5e878d9`
- The user's local `machine.toml` changes are intentionally uncommitted.
- PathTempo receives the configured limits without caller-side derating and
  keeps sampled differential-station corrections enabled.
- NGC's emitted-polynomial gates allow 1% above configured aggregate and
  per-axis acceleration and jerk limits. Velocity remains strict.

## Reproduction

Use the source-directory working directory and the user's current
`machine.toml`:

```powershell
cmake --build build --target ngc_simulation_diagnostic
.\build\ngc_simulation_diagnostic.exe adaptive_pockets.ngc 100000 15 `
  --smoother=uniform `
  --mode=optimized `
  --stop-after-plan-pieces=7759
```

The accepted plan has no unresolved quintics, but PathTempo receives one
batched materialization correction and solves again. The first callback
reports ten corrected timing pieces:

```text
448, 508, 565, 3528, 3588, 3645, 6549, 6609, 6666, 7631
```

The largest reported time scale is approximately `1.36003`. The second
callback succeeds without corrections.

## Captured first-pass evidence

Temporary instrumentation retained the first callback's
`ContinuousQuinticMaterializationDiagnostics::Failure` values through the
second callback. The diagnostic exporter captured 1,846 rejected adaptive
leaves:

| Classification | Leaves |
|---|---:|
| `path_velocity` | 1,814 |
| `path_jerk` | 32 |
| Total | 1,846 |

All rejected leaves were approximately 50 to 62 microseconds long, far below
the configured 1 ms servo period. Their normalized intervals showed that
refinement stopped around depth 9. This is below the depth-20 limit, so the
next split was rejected by `stableSplit()` and its 1%-of-jerk endpoint
resolution guard.

The 32 jerk-classified leaves occurred in eight timing pieces:

| Timing piece | Jerk leaves | Maximum pointwise path-jerk ratio |
|---|---:|---:|
| 448 | 8 | 2.52123 |
| 508 | 1 | 1.11229 |
| 3528 | 10 | 2.54078 |
| 3588 | 1 | 1.24129 |
| 3645 | 1 | 1.05474 |
| 6549 | 9 | 2.48893 |
| 6609 | 1 | 1.26639 |
| 6666 | 1 | 1.13015 |

The certified and densely sampled emitted-quintic jerk ratios agreed closely,
so these pointwise peaks are not derivative-control-bound looseness.
Prepared-geometry coupled jerk at the corresponding scalar states remained
near or below the configured limit; its maximum observed ratio among the ten
pieces was approximately `1.00089`. The large peaks are local endpoint-PVA
quintic derivative behavior rather than prepared-geometry jerk.

Every jerk-classified leaf passed the servo-interval acceleration-excursion
rule when evaluated independently. Per-piece maximum excursion ratios were:

| Timing piece | Maximum excursion ratio |
|---|---:|
| 448 | 0.10121 |
| 508 | 0.04462 |
| 3528 | 0.10157 |
| 3588 | 0.04999 |
| 3645 | 0.04185 |
| 6549 | 0.09971 |
| 6609 | 0.05100 |
| 6666 | 0.04494 |

The acceptance threshold is `1.01`, so these are well within the permitted
servo-period budget.

The two timing pieces without jerk-classified leaves had only negligible
velocity corrections:

| Timing piece | Requested scale |
|---|---:|
| 565 | approximately `1.000000021` |
| 7631 | approximately `1.00000000115` |

## Root cause

`TrajectoryCompiler.cpp` calculates the sub-servo acceleration excursion only
inside a condition that first requires velocity and acceleration to pass:

```cpp
if (duration < servoPeriod
   && bounds.maximumVelocityRatio <= 1.0 + MATERIAL_VIOLATION_TOLERANCE
   && trajectory_detail::dynamicLimitRatioAccepted(
        bounds.maximumAccelerationRatio)
   && bounds.maximumJerkRatio
        > trajectory_detail::DYNAMIC_LIMIT_RATIO
            + trajectory_detail::POLYNOMIAL_RATIO_TOLERANCE) {
    // Calculate and classify the servo-aware jerk exception.
}
```

The emitted local quintics have tiny velocity overshoots. Once the strict
velocity predicate fails, NGC skips the acceleration-excursion calculation.
`candidate.subServoJerkAccepted` therefore remains false and the worst
correction category remains pointwise jerk. The later combined predicate calls
`servoAwareJerkAccepted()` with the default zero excursion and happens to
accept jerk, but the candidate still fails velocity and its correction
ownership is not recomputed. The accepted pointwise jerk ratio therefore owns
the correction, producing large cube-root time scales instead of retaining
only the actual velocity correction.

This also explains why the earlier prepared-geometry diagnostic did not need
the same correction: it checked the scalar law composed with prepared
geometry, not the emitted local quintic's small velocity overshoot and
derivative oscillation.

## Recommended fix

Evaluate the three constraint categories independently:

```text
velocityAccepted =
    maximumVelocityRatio satisfies the velocity policy

accelerationAccepted =
    maximumAccelerationRatio <= 1.01 plus numerical tolerance

jerkAccepted =
    pointwise jerk ratio <= 1.01 plus numerical tolerance
    or
    duration < servo period and acceleration excursion ratio <= 1.01

constraintsVerified =
    velocityAccepted and accelerationAccepted and jerkAccepted
```

In particular:

1. Calculate acceleration excursion whenever the span is sub-servo and
   pointwise jerk exceeds the tolerated limit. Do not gate this calculation on
   velocity or acceleration acceptance.
2. If servo-aware jerk passes but another category fails, select the worst
   correction only from the categories that actually failed. Do not allow the
   accepted pointwise jerk ratio to own the correction.
3. Add a regression where velocity is just over its strict threshold, jerk is
   above 101%, duration is sub-servo, and acceleration excursion passes. It
   must classify jerk as accepted and retain only the velocity correction.
4. Decide separately whether emitted-quintic velocity should remain strict,
   receive only a larger numerical tolerance, or receive the same 1%
   verification tolerance. The captured velocity-only correction scales for
   two pieces are numerical in size; after correcting jerk ownership, measure
   the actual velocity-only factors for the other eight pieces before deciding.

Do not solve this by restoring 99% limits sent to PathTempo. Keep PathTempo on
the configured limits and apply NGC's explicit emitted-polynomial policy at
materialization and final verification.

## Verification

After the fix:

```powershell
cmake --build build
.\build\ngc_tests.exe
ctest --test-dir build -E "^ngc_tests$" --output-on-failure
```

Then rerun the 7,759-piece diagnostic window. Confirm:

- the eight large jerk-derived correction factors disappear;
- every one of the 32 captured jerk leaves is classified as servo-aware
  acceptable independently of its velocity result;
- every remaining correction is velocity-only and is reported at its actual
  scale;
- final emitted quintics, packet verification, stop branches, activations, and
  C2 checks still pass; and
- planning time and callback counts are remeasured.
