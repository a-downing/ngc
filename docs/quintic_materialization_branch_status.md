# Quintic materialization branch status

Materialized continuous motion now uses proved local-coordinate quintics from
PathTempo timing through `PlanChunk` and backend execution. There is no
intermediate cubic production plan.

## Production representation

`AxisPolynomialSpan` is the single bounded, trivially copyable execution type.
It stores an absolute origin, five local normalized power coefficients, a
degree tag, duration, and inverse-duration powers. Quintic G64 normal motion and
padded cubic exact-stop or stop-tail motion use this same type.

The backend evaluates position, velocity, acceleration, and jerk directly from
the coefficients without allocation. It emits exact in-span presentation
markers as the execution cursor crosses them; activation timing does not split
the polynomial.

## Materialization and proof

For each PathTempo timing piece, NGC:

1. Constructs endpoint-PVA quintics in local coordinates.
2. Adaptively subdivides until geometry, forward-progress, and certified
   derivative-control bounds pass.
3. Applies a one-percent verification tolerance to aggregate and per-axis
   acceleration and jerk, and only a `1e-7` relative numerical ratio tolerance
   to velocity.
4. Accepts a sub-servo pointwise jerk peak beyond that tolerance only when the
   complete acceleration-control hull proves that its acceleration excursion
   fits within the same tolerated servo-period jerk budget.
5. Batches unresolved time-scale corrections back to PathTempo.
6. Retains a complete ordered sequence only after every timing piece succeeds.
7. Verifies time and distance coverage, endpoints, C2 continuity, activation
   ownership, and deterministic coefficients.
8. Packetizes the verified quintics and generates a proved cubic stop tail from
   each packet's evaluated quintic branch state.
9. Applies the same tolerance and servo-aware jerk rule to emitted-polynomial extrema as the
   final axis and aggregate constraint gate.

There is one continuous-planning mode. PathTempo always uses prepared
differential stations and the unmodified configured limits for its inexpensive
initial correction passes. NGC does not send 99-percent limits to PathTempo.
The one-percent allowance belongs only to NGC's verification of emitted
acceleration and jerk polynomials. Velocity remains outside that allowance and
uses the same small numerical tolerance at materialization and the final
packet gate. The adaptive quintic materializer and final packet gate provide
the stronger continuous emitted-polynomial checks. Cubic construction remains
only where the execution architecture requires it, including exact-stop motion
and stop tails.

## Removed development scaffolding

The following experimental code has been removed:

- the separate quintic execution prototype type and evaluator;
- cubic-versus-quintic servo replay and microbenchmarking;
- activation-driven quintic splitting;
- degree-aware prototype packet construction and replay;
- prototype packet CSV exports and summary fields; and
- production construction, geometry proof, and exact checking of an
  intermediate cubic sequence that was immediately discarded.

Production `PlanChunk` packetization, backend execution, execution-marker
tests, feed-hold tests, and stop-branch proof now provide the authoritative
coverage.

## Single-window materialization benchmark

With `trajectory.lookahead_duration = 400`, the production diagnostic command
is:

```powershell
.\build\ngc_simulation_diagnostic.exe adaptive_pockets.ngc 100000 15 `
  --smoother=uniform `
  --mode=optimized `
  --stop-after-plan-pieces=7759
```

With the single servo-aware production pipeline, the 7,759-piece window
produces:

| Measurement | Result |
|---|---:|
| Production quintics | 20,843 |
| Adaptive subdivisions | 13,084 |
| PathTempo correction passes, including sampled passes | 7 |
| Quintic materialization callbacks | 1 |
| Intermediate cubic materializations | 0 |
| Discarded exact cubic span checks | 0 |
| Quintic geometry proofs in the accepted pass | 22,361 |
| Presentation markers | 5,164 |
| Interior markers | 4,667 |
| Maximum markers per packet | 84 |
| Unresolved quintics | 0 |
| Servo-aware sub-servo acceptances beyond the 1% tolerance | 17 |
| Quintic construction and proof time in the accepted pass | 0.234 s |
| Reported materialization overhead | 0.005 s |
| Planner time | 2.667 s |

The previous intermediate cubic path required 32 callbacks, performed 155,967
discarded exact span checks, and took 49.041 seconds. Removing it first reduced
the planner to 10.917 seconds and nine quintic callbacks. The servo-aware
production rule reduced that to two quintic callbacks. Independently
classifying velocity, acceleration, and jerk, then applying the small numerical
velocity tolerance, removes the unnecessary correction callback while
retaining PathTempo's sampled initial passes.

## Two-second rolling-horizon benchmark

With `trajectory.lookahead_duration = 2.0`, the same program, smoother, and
optimized boundary mode were run to 280 seconds of simulated program time:

```powershell
.\build\ngc_simulation_diagnostic.exe adaptive_pockets.ngc 280 15 `
  --smoother=uniform `
  --mode=optimized
```

| Measurement | Result |
|---|---:|
| Planned path duration | 281.239 s |
| Best continuous window | 0.000125 s |
| Average continuous window | 0.068253 s |
| Worst continuous window | 0.109327 s |
| Total trajectory planning | 5.051 s |
| Geometry preparation | 0.766 s |
| Total processing | 5.817 s |
| Rolling boundary candidates | 141 |
| Suffix proof failures | 57 |
| Prefix proof failures | 11 |

## Remaining work

Quintic construction and proof remain well under one second. The materializer
accepts the single-window benchmark on its first callback, so it causes no
additional PathTempo solve.

A physical backend still requires target-specific WCET measurement, numerical
policy review, fault handling, and hardware-in-the-loop validation.
