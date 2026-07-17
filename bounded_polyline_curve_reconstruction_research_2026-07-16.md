# Research notes: bounded polyline curve reconstruction and trajectory planning

Date: 2026-07-16
Updated: 2026-07-17
Scope: fast reconstruction of fair curves from CAM polylines under the G64 `P` tube, followed by the original certified trajectory-planning research and profiling record.

## Polyline reconstruction update: the current primary blocker

### Required contract

The reconstruction problem is not ordinary point fitting. For an ordered source polyline and programmed scale `P`, NGC needs a regular replacement curve that:

- remains within `P` of the corresponding source entities, with ordered association rather than an unordered nearest-point match;
- preserves protected endpoints, activation order, feed sections, and any genuine corner that cannot be smoothed inside the tube;
- has continuous curvature derivative wherever execution is intended to remain continuous;
- keeps curvature and curvature derivative low enough to avoid destroying the jerk-limited feed;
- is constructed in bounded NRT time and then passes a separate conservative geometry proof.

For a planar arc-length curve, `q''' = kappa' N - kappa^2 T`. Minimizing only signed-curvature variation `kappa'` is therefore insufficient: a small-radius circle has zero normal curvature derivative but retains the unavoidable tangential term `kappa^2`. The practical geometry objective must control both maximum curvature and the full vector magnitude of `q'''`, or use the actual coupled axis-jerk cap already consumed by the planner.

The tolerance should be treated as a budget. A useful initial split is to reserve part for model construction and part for numerical/proof margin, for example `0.8P` for fitting and `0.2P` for certification headroom. The exact split should be measured rather than embedded as G64 semantics.

### What the literature says

The closest directly relevant published work divides into five families:

1. **Confined-error global B-spline fitting.** Yang, Shen, Yuan, and Gao formulate CNC polyline fitting as a quadratic B-spline optimization with an explicit line-segment/curve Hausdorff model in [Curve fitting and optimal interpolation for CNC machining under confined error using quadratic B-splines](https://doi.org/10.1016/j.cad.2015.04.010). This is the strongest lead for treating the source as complete entities rather than point samples. Its degree and continuity are not sufficient for NGC's curvature-derivative goal, but its distance formulation is relevant to the proof front end.
2. **Dominant-point compression followed by constrained fitting.** Du, Huang, Zhu, and Ding select dominant points, fit a B-spline, insert missed points, and add midpoint constraints for edge-to-curve error in [An error-bounded B-spline curve approximation scheme using dominant points for CNC interpolation of micro-line toolpath](https://doi.org/10.1016/j.rcim.2019.101930). This supports fast adaptive knot/control insertion. The paper itself notes that dense sampling alone is not an analytic distance guarantee, so NGC should retain its own recursive ordered verification.
3. **Analytical G3 corner and chain constructions.** Sun and Altintas use sixth-degree, seven-control Bezier pieces and analytical error correction in [A G3 continuous tool path smoothing method for 5-axis CNC machining](https://doi.org/10.1016/j.cirpj.2020.11.002). The method avoids iterative solutions and demonstrates that bounded-error G3 line-chain smoothing can be computationally practical. It is a local construction, while NGC also needs long noisy strips to share curvature information globally.
4. **Clothoid and PH spiral transitions.** Shahzadeh, Khosravi, Nahavandi, and Robinette bound line/arc fillet deviation with arc-length-parameterized biclothoids in [Smooth path planning using biclothoid fillets for high speed CNC machines](https://doi.org/10.1016/j.ijmachtools.2018.04.003). Li, Ait-Haddou, and Biard construct degree-seven rational PH spirals with G3 contact between compatible circle data in [Pythagorean hodograph spline spirals that match G3 Hermite data from circles](https://doi.org/10.1016/j.cam.2014.10.005). These are attractive bridges between accepted arc fits, but their planar/admissibility restrictions make them a specialized primitive rather than the universal polyline fitter.
5. **Fairness objectives inside a tolerance band.** Moreton and Sequin motivate minimizing curvature variation in [Minimum Curvature Variation Curves, Networks, and Surfaces for Fair Free-Form Shape Design](https://digicoll.lib.berkeley.edu/record/134037), while Naseath and Red explicitly move Bezier toolpaths inside a machining tolerance band to reduce curvature in [Reducing Curvature by Deviating CAM Tool Paths within a Tolerance Band](https://doi.org/10.3722/cadaps.2008.921-931). Levien's [From Spiral to Spline](https://www2.eecs.berkeley.edu/Pubs/TechRpts/2009/EECS-2009-162.pdf) explains why curvature-variation fairness reproduces lines and circles better than point interpolation alone. These sources support the objective, but do not supply NGC's complete ordered certificate.

Exact or optimal polyline simplification is useful only as preprocessing. Local Frechet methods preserve order, but optimal two-dimensional algorithms remain roughly quadratic; see [Polyline Simplification under the Local Frechet Distance has Almost-Quadratic Runtime in 2D](https://arxiv.org/abs/2201.01344). A bounded greedy recognizer is more appropriate for the online NRT path, with a slower algorithm retained only as an offline oracle.

### Assessment of arc-first reconstruction

Replacing maximal portions of a line strip with fitted arcs is promising, but only as a compression and initialization stage:

- A long, nearly circular run can be represented by one radius and center, giving excellent curvature information at negligible evaluation cost.
- Existing line/arc junction blending can then bridge recognized arcs, preferably with a G3 spiral or higher-degree spline when curvature derivative must be continuous.
- Arc recognition must verify the whole ordered source subchain against the arc tube; fitting only vertices can cut across a loop or miss an edge bulge.
- An arc has constant signed curvature but full vector curvature derivative magnitude `kappa^2`. Arc compression therefore reduces curvature noise, not necessarily physical jerk when the fitted radius is small.
- Tangent circular arcs of different radii are only G1 at their contact. Existing cubic G2 bridges still leave curvature-derivative jumps; the bridge must explicitly match G3 circle data or the complete reconstructed chain should be refit globally.
- Inflections, torsion, simultaneous rotary motion, and nonplanar strips need a spline fallback.

The recommended use of arcs is therefore: cheaply recognize maximal low-residual planar runs, spend perhaps `0.3P` to `0.5P` of the tube on those approximations, use the resulting line/arc chain as a low-noise guide, and perform one global fair fit over compatible regions. Do not make independently fitted arcs the final executable curve merely because every arc is locally within `P`.

### Recommended production-oriented algorithm

The best next prototype is a hybrid with a fast common path and a proof-preserving fallback:

1. Partition at protected state, feed changes that cannot share a piece, discontinuities, explicit G53, probes, and geometrically hard reversals.
2. Run a linear-time recognizer over each compatible strip: collapse exact collinear lines; greedily propose maximal planar circle fits; reject candidates by an ordered tube check; retain dominant points at changes of turn sign or failed fits.
3. Build one degree-five open B-spline per retained region. With simple interior knots it is C4 in its parameter and therefore has continuous arc-length curvature derivative wherever parameter speed remains positive. Use line/arc endpoint jets only at protected outer boundaries; do not force zero curvature at every original line midpoint.
4. Initialize controls from the simplified line/arc guide using chord-length or approximate arc-length parameters.
5. Minimize a sparse quadratic fairness proxy based on third control differences, with a smaller fidelity term. Solve the banded system directly in linear time. This should replace coordinate descent or a generic nonlinear optimizer.
6. Add violated tube locations as active linearized constraints and repeat for a small bounded number of passes. If a pass cannot recover adequate margin, insert a knot/control at the worst ordered source interval and solve the two smaller regions.
7. Certify the final curve with recursive Bezier-hull subdivision against ordered source-segment capsules, reserving a conservative bound for the source interval. Check both candidate-to-source deviation and sufficient ordered coverage to prevent shortcutting across folds.
8. Measure exact curvature and curvature-derivative extrema conservatively, require a positive speed lower bound, and fall back by subdivision. Failure remains fatal; it is not permission to publish a sampled-only curve.

This structure puts the expensive work where it belongs: a narrow banded solve and local proof subdivisions around active constraints. Arc recognition reduces system size but is not required for correctness.

### Experiment with the existing tool

`tools/ngc_quintic_spline_analyzer.cpp` was updated so the programmed `P` and allowed fraction of `P` are explicit, and so it reports fitting time. The experiment used the captured `adaptive.ngc` spline ID 1 from `build/adaptive_spline_geometry.txt`:

- sources 1 through 48: 42 tiny lines followed by six arcs;
- production representation: 32 cubic controls and 29 spans;
- `P = 0.005 inch`, allowed deviation `1.0P`, feed `80 inch/minute`;
- candidate: one open quintic B-spline with endpoint position/tangent/curvature/third-derivative conditions;
- search: one deterministic coordinate sweep at each of three scales (`0.25P`, `0.0625P`, and `0.015625P`), four samples per span during search, followed by 256 samples per span for the reported check.

Release result:

| Metric | Production cubic | Fast quintic | Change |
|---|---:|---:|---:|
| Peak normal curvature derivative | 5,894,600 | 193,090 | 96.7% lower, 30.5x reduction |
| Integrated normal curvature derivative | 6,305.6 | 1,667.96 | 73.5% lower |
| Peak curvature | 2,331.12 | 542.94 | 76.7% lower |
| Reported ordered deviation | n/a | 0.00357254 inch | 0.7145P |
| Reconstructed length | 0.138259 inch | 0.144234 inch | 4.3% longer |
| Fit time | n/a | 21.7-22.4 ms | Release, three runs; median 22.19 ms |

A much denser coordinate search reached peak normal curvature derivative 59,158.6 and peak curvature 316.9 while staying at 0.004914 inch, but required 3.08 seconds. That establishes that the tube contains a much fairer curve, while also rejecting coordinate descent as the production solver. The bounded pass retains most of the benefit at about 1/139 of that search time, but roughly 22 ms for only 48 entities is still too expensive for full CAM horizons. Because every coordinate candidate remeasures the whole spline, this experiment remains quadratic in control count despite its bounded number of sweeps. A banded fairness solve is the immediate performance experiment.

The generated comparison artifacts are `build/polyline_quintic_fast_release.csv` and `build/polyline_quintic_fast_release.svg`.

This is evidence, not a production proof. The analyzer currently compares dense equal-progress samples to the ordered source composite; it does not compute a certified Hausdorff/Frechet bound, its arc geometry uses numerical estimates, and it does not prove extrema between samples. The experiment also mixes a line-dominated strip with six terminal arcs. The result justifies implementing the banded quintic and recursive tube verifier in the standalone tool before changing `ExactStopTrajectoryPlanner`.

#### `adaptive_pockets.ngc` follow-up

A rolling trace of `adaptive_pockets.ngc` captured 3,940 production spline records. Eight records replace at least eight source entities and are at least 75% lines; these were tested with the program's actual `G64 P0.002`. All eight fast quintic candidates passed the analyzer's dense ordered sampled-deviation check:

| Spline | Sources | Line share | Cubic peak normal curvature derivative | Quintic peak | Reduction | Deviation | Release fit time |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 1..43 | 42/43 | 49,109,100 | 1,942,480 | 96.04%, 25.28x | 0.000841719, 0.421P | 47.53 ms |
| 1590 | 2056..2075 | 19/20 | 6,384.77 | 4,991.21 | 21.83%, 1.28x | 0.000272923, 0.136P | 10.49 ms |
| 1594 | 2078..2120 | 42/43 | 49,297,900 | 1,753,440 | 96.44%, 28.11x | 0.000859433, 0.430P | 47.18 ms |
| 3173 | 4137..4148 | 11/12 | 22,721.1 | 5,897.96 | 74.04%, 3.85x | 0.000263466, 0.132P | 4.04 ms |
| 3177 | 4151..4193 | 42/43 | 49,223,400 | 2,525,220 | 94.87%, 19.49x | 0.000778954, 0.389P | 47.35 ms |
| 3718 | 4858..4875 | 17/18 | 17,240.3 | 6,620.26 | 61.60%, 2.60x | 0.000532395, 0.266P | 9.05 ms |
| 3722 | 4878..4920 | 42/43 | 60,535,500 | 2,365,810 | 96.09%, 25.59x | 0.000921083, 0.461P | 47.36 ms |
| 3939 | 5152..5163 | 11/12 | 22,980.4 | 6,452.75 | 71.92%, 3.56x | 0.000273025, 0.137P | 3.60 ms |

The four pathological 43-entity strips consistently improve by roughly 19.5x to 28.1x and use less than `0.47P`, so the favorable `adaptive.ngc` result was not isolated. The moderate strips improve less because their production curvature derivative is already several orders of magnitude lower. Runtime again scales poorly for the long records: roughly 47 ms each confirms that bounded coordinate sweeps are still not the desired solver even when their geometry is successful.

The trace, measurements, and candidate profiles are in `build/adaptive_pockets_spline_geometry.txt`, `build/adaptive_pockets_spline_measurements.csv`, and `build/adaptive_pockets_quintic_p002_<id>.{csv,svg}`. These remain sampled research artifacts, not certified production geometry.

#### Fixed-bandwidth fairness-solver experiment

The standalone quintic analyzer now also accepts the `banded` solver mode. It fixes the first and last four controls, preserving the constructed endpoint position and first three parameter derivatives, and solves

```text
minimize  sum ||control - initial_control||^2
        + lambda * sum ||third control difference||^2
```

for every coordinate. Each third difference touches four adjacent controls, so the normal equations have fixed half-bandwidth three. A direct banded Cholesky factorization and two triangular solves are linear in control count. The experiment evaluates 13 fixed logarithmic `lambda` candidates, rejects candidates beyond `0.98P` or above the initial curvature allowance, and performs the same dense 256-sample-per-span final measurement. The weight scan and sampled measurement are bounded experimental policy rather than production semantics.

On the same eight `adaptive_pockets.ngc` regions with `P = 0.002`:

| Spline | Production peak normal curvature derivative | Coordinate-search peak | Banded peak | Banded reduction | Banded deviation | Banded Release time |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 49,109,100 | 1,942,480 | 10,058,300 | 4.88x | 0.00180436, 0.902P | 5.70 ms |
| 1590 | 6,384.77 | 4,991.21 | 5,376.48 | 1.19x | 0.000259275, 0.130P | 2.26 ms |
| 1594 | 49,297,900 | 1,753,440 | 10,149,700 | 4.86x | 0.00181503, 0.908P | 5.85 ms |
| 3173 | 22,721.1 | 5,897.96 | 6,438.16 | 3.53x | 0.000334849, 0.167P | 1.35 ms |
| 3177 | 49,223,400 | 2,525,220 | 10,141,700 | 4.85x | 0.00181328, 0.907P | 5.35 ms |
| 3718 | 17,240.3 | 6,620.26 | 5,571.63 | 3.09x | 0.000388823, 0.194P | 1.98 ms |
| 3722 | 60,535,500 | 2,365,810 | 10,867,300 | 5.57x | 0.00191576, 0.958P | 5.46 ms |
| 3939 | 22,980.4 | 6,452.75 | 6,565.72 | 3.50x | 0.000273402, 0.137P | 1.43 ms |

The long-strip solve is about eight times faster than the bounded coordinate search and every candidate stays inside the sampled tube. Uniform squared third differences are not a sufficient proxy for the peak physical arc-length curvature derivative, however: the pathological strips improve only 4.85x to 5.57x instead of the coordinate search's 19.49x to 28.11x. A quick inverse-local-spacing-to-the-sixth row weighting did not improve the worst case and was removed.

This result changes the next step. Keep the fixed-bandwidth solve as the fast linear algebra backbone, but replace its one global fairness weight with a small active set of locally weighted rows driven by the measured physical `q'''` peaks and tube slack. A promising bounded loop is:

1. solve the uniform banded system;
2. measure physical curvature derivative and ordered deviation;
3. increase weights only on rows influencing the largest physical peaks, while adding linearized tube constraints where slack is low;
4. refactor the same bandwidth-three matrix and repeat for a small fixed number of passes;
5. insert a knot and split only if the active loop cannot obtain margin.

That experiment should compare complete `q'''`, not only its normal component, and should move to recursive deviation certification before planner integration. The generated profiles are `build/adaptive_pockets_banded_p002_<id>.{csv,svg}`.

#### Peak-targeted banded fairness experiment

The `banded-active` analyzer mode implements the bounded active loop while retaining the same half-bandwidth-three factorization. The uniform scan supplies at most one seed from each of three sampled deviation-utilization ranges: below `0.35P`, from `0.35P` to `0.70P`, and above `0.70P`. This avoids retrying every global weight. For at most four passes per seed, the analyzer samples the physical arc-length curvature derivative, finds spans whose normal-component peak is at least 35% of the global peak, and raises the squared-third-difference weights on their overlapping rows. A pass is retained only when the combined normalized peak/integral score improves and the candidate remains below `0.98P` and the initial curvature allowance.

On the same `adaptive_pockets.ngc` regions:

| Spline | Production peak | Uniform banded peak | Peak-targeted peak | Production reduction | Coordinate-search peak | Deviation | Release time | Banded solves |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 49,109,100 | 10,058,300 | 3,812,290 | 12.88x | 1,942,480 | 0.00172529, 0.863P | 10.37 ms | 22 |
| 1590 | 6,384.77 | 5,376.48 | 5,376.48 | 1.19x | 4,991.21 | 0.000259275, 0.130P | 2.99 ms | 16 |
| 1594 | 49,297,900 | 10,149,700 | 3,780,660 | 13.04x | 1,753,440 | 0.00173287, 0.866P | 10.67 ms | 22 |
| 3173 | 22,721.1 | 6,438.16 | 6,438.16 | 3.53x | 5,897.96 | 0.000334849, 0.167P | 1.69 ms | 15 |
| 3177 | 49,223,400 | 10,141,700 | 3,824,530 | 12.87x | 2,525,220 | 0.00172981, 0.865P | 10.36 ms | 22 |
| 3718 | 17,240.3 | 5,571.63 | 5,571.63 | 3.09x | 6,620.26 | 0.000388823, 0.194P | 3.42 ms | 17 |
| 3722 | 60,535,500 | 10,867,300 | 3,919,580 | 15.44x | 2,365,810 | 0.00171735, 0.859P | 10.11 ms | 21 |
| 3939 | 22,980.4 | 6,565.72 | 6,565.72 | 3.50x | 6,452.75 | 0.000273402, 0.137P | 1.73 ms | 15 |

For the four pathological 43-entity strips, local reweighting improves the uniform banded result by another 2.64x to 2.77x. Their total production-to-candidate reduction becomes 12.87x to 15.44x. The targeted solve takes about 10-11 ms per long strip, roughly twice the uniform solve but 4.4-4.7 times faster than the coordinate search. It does not quite match the coordinate search's peak on those strips, but it uses 22 or fewer linear-time banded solves instead of coordinate-wise perturbation sweeps.

The moderate strips mostly retain the uniform result, which is useful bounded behavior: active passes do not spend the tube merely because slack exists. An exhaustive all-weight active prototype did improve spline 3718 to 4,688.01, but raised the severe-strip cost to about 25-26 ms and was rejected in favor of the three-regime seed policy.

This is encouraging but not production-ready. Deviation and physical derivatives are still sampled, not recursively certified; the reweighting targets only the normal component of `q'''`; and fixed endpoint controls consume a meaningful part of the tube on these very short strips. The next geometry experiment should now be greedy maximal arc recognition before fitting. It should measure whether removing nearly circular runs lowers control count and endpoint-boundary peaks enough that the active banded solve can use fewer passes. The generated profiles are `build/adaptive_pockets_banded_active_p002_<id>.{csv,svg}`.

#### Planner integration

The three experimental quintic fitters are now integrated into `ExactStopTrajectoryPlanner` behind the single `continuousSplineFitSolver()` selection point. The available selections are `CoordinateSearch`, `UniformBandedFairness`, and `PeakTargetedBandedFairness`; `CubicBaseline` is retained for controlled A/B measurement. Only variable-control short-entity clusters are reconstructed. Ordinary six-control junction splines remain cubic because fixing four quintic controls at each end leaves them no optimization freedom and converting them imposed needless proof and runtime cost.

Unlike the standalone experiment, planner acceptance does not make sampled deviation authoritative. The selected quintic is checked by an ordered recursive source-tube proof. Each interval combines the maximum endpoint association error with a conservative source chord-error bound and a quintic second-derivative control-hull chord bound. Source entity boundaries are explicit initial proof boundaries, the work count is finite, and a failure is fatal rather than permission to publish the candidate. The ordinary emitted-polynomial geometry and XYZABC constraint proofs remain downstream authority.

The reference implementation is degree-aware for cubic and quintic B-splines, and NRT geometry diagnostics now record degree explicitly in the version-two snapshot format. Version-one cubic snapshots remain readable by both geometry tools.

Release rolling-planner A/B results on the complete current `adaptive_pockets.ngc` workload (`G64 P0.005`), using the current source tree and identical planning effort, were:

| Selection | Verified rolling duration | Total rolling planning | Maximum horizon planning | Chunks |
|---|---:|---:|---:|---:|
| Cubic baseline | 347.7150 s | 9.781 s | 0.374 s | 107 |
| Coordinate search | 336.4937 s | 28.946 s | 1.760 s | 107 |
| Uniform banded | 334.4529 s | 14.023 s | 0.645 s | 107 |
| Peak-targeted banded | 334.5554 s | 14.563 s | 0.650 s | 107 |

All four selections completed the rolling planner. Uniform banding is therefore the current default: it shortened verified motion by 13.2621 seconds, or 3.81%, relative to the cubic baseline and was slightly better than peak-targeted banding in both final motion and computation on the complete workload. Coordinate search remained much too expensive. The active solver's lower sampled peak on the four pathological strips did not translate into a better complete trajectory than uniform banding; local normal-peak reduction alone is not a sufficient whole-program selection objective.

The full-horizon export reached the optional infinite-jerk oracle after planning but that diagnostic did not converge within its existing nine refinements on the new geometry. Rolling production-shaped planning and all final trajectory proofs completed. The oracle refinement policy needs a separate investigation before using its duration as the quintic comparison baseline.

Geometry-only ImGui Preview now calls the same `spline_detail::reconstructSpline()` function and reads the same `continuousSplineFitSolver()` selection as timed planning. Preview supplies its retained canonical XYZ source geometry, receives the same cubic or quintic degree and controls, and tessellates that degree directly. It does not invoke `ExactStopTrajectoryPlanner`, Ruckig, trajectory timing, polynomial emission, packetization, or a backend. Planner calls request the recursive source-tube certificate; Preview calls remain display-only. A focused regression reconstructs one long line strip through both paths and requires every XYZ control to agree within `1e-12`.

#### Execution visualization and remaining timing question

Two separate combs now make the geometry/timing distinction visible. The Preview-only short-entity cluster comb is evaluated at the same 65 arc-length sample locations used for the planner's sampled geometric caps. Its tooth magnitude represents the full geometric jerk coefficient `|q'''|`, not only normal sharpness, and its color reports whether the resulting geometric jerk cap is below programmed feed.

Timed Simulation has a different executed-jerk comb. `MockMotionBackend` records the analytic XYZABC jerk-vector magnitude of the executed axis cubic at the position actually calculated on each fixed servo tick. A mock-only incremental diagnostic carries epoch, chunk, and span identity to `SimulationWorker`, which applies the matching tool geometry offset before presentation so teeth originate on the tool-tip path rather than at the spindle reference point. The UI retains one tooth every ten executed servo periods. Tooth density is therefore temporal: closer teeth mean lower speed. The green part is used aggregate path-jerk capacity and the red remainder is unused capacity. This diagnostic remains outside `MotionBackend` and does not claim a production RT telemetry contract.

A subsequent executed-comb comparison exposed a rolling-geometry defect. Rolling split eligibility used the automatically reduced entity scale `min(P, entity_length / 6)`. Consequently every line shorter than `6P` passed the test by equality and could be cut at its canonical midpoint. This forced independently reconstructed prefix and suffix geometry through the original line state and broke the full-window short-entity cluster partition shown by Preview. Rolling anchors now require a line strictly longer than `6P`, so they lie only inside true retained-line interiors. The complete production-shaped `adaptive_pockets.ngc` rolling export succeeds with the corrected anchor policy. Its final all-short region must remain one geometry-consistent horizon; this required raising the finite exact-correction ceiling from 24 to 32 passes rather than reintroducing a source-line split.

The `adaptive_pockets.ngc` trochoidal regions visibly contain dense teeth with large red remainders. That is direct evidence that those executed regions are slow while using little of the configured aggregate path-jerk limit. It does not yet identify the binding constraint: programmed/geometric velocity caps, aggregate acceleration, or an individual axis velocity, acceleration, or jerk limit can still dominate. The next diagnostic should label the active limiting constraint and show aggregate plus per-axis velocity/acceleration/jerk utilization at the same servo samples. This is more informative than assuming every slow curved region is failing the strict jerk proof.

Simulation also reports program elapsed time in seconds and `hours:minutes:seconds`; changing the playback multiplier changes wall-clock playback but not the final program duration. The GLFW UI now uses swap interval one. Its lower-corner performance label reports the preceding frame's CPU construction and OpenGL submission time in milliseconds, measured after event polling and before `glfwSwapBuffers()`, so it excludes the vsync wait rather than presenting inter-frame time as render cost.

### Immediate experiment sequence

1. Completed experimentally: replace coordinate search with uniform and peak-targeted fixed-layout banded quintic fairness solves and report time per source entity.
2. Next: add greedy maximal arc recognition and compare control count, solve time, full `q'''` rather than only its normal component, and deviation-proof workload with recognition enabled/disabled.
3. Replace sampled deviation with ordered recursive Bezier-hull-to-polyline-capsule verification, reporting proof attempts and worst certified interval.
4. Run on all line-dominated strips from `adaptive.ngc`, `adaptive_pockets.ngc`, and `1002_3d.ngc`, including inflections and rotary-axis motion.
5. Only after geometry is certified, feed the candidate references into the existing timing oracle and compare verified traversal duration and total planning latency.

## Executive summary

The current NGC design is already stricter than most published CNC trajectory planners. Many papers prove feasibility only at grid points, approximate jerk by acceleration differences, or use a filtered trajectory whose contour error is accepted empirically. NGC instead requires exact emitted-polynomial extrema checks, ordered geometry verification, continuous state at packet boundaries, and a valid stop branch. Those guarantees should remain the final authority.

The most promising speedups are therefore not replacements for Ruckig or the final proof gates. They are ways to make fewer expensive candidate solves and to reuse more work:

1. **Prototype conservative backward-reachable sets in `(v, a)` for the retained suffix.** Recent third-order path-parameterization work represents stop-feasible states as small 2-D polytopes. Used only as an inner feasibility filter, this could reject hopeless station candidates and rolling split points before two neighboring Ruckig solves, without weakening the exact acceptance path.
2. **Separate optimization stations from source-command activation boundaries.** Exactly collinear, same-feed straight runs can share one timing problem even if emitted spans must still be split at command activations. This attacks the station count generated by dense CAM output without changing geometry or presentation ownership.
3. **Reuse rolling-horizon work.** Preserve geometry derivatives, arc-length inverse data, local constraint caps, suffix reachable sets, and useful candidate states when the retained suffix is unchanged. A newly appended tail should not normally force reconstruction of the entire retained window.
4. **Reuse bit-exact certified inverse results and investigate interval/Bernstein bounds.** Exact repeated geometry queries can avoid adaptive work without changing authority. Any future approximation must supply a bracket or conservative bound and fall back to the existing true integral/proof path.
5. **Use convex/sequential methods as offline oracles and seed generators.** TOPP-RA, sequential convex/linear programming, and McCormick relaxations provide good envelopes, lower bounds, and active-set hints. Published third-order versions do not generally provide the same continuous guarantees as NGC, so they are poor drop-in production replacements.

The original highest-value immediate research experiment was to instrument where planning time actually goes: Ruckig calls by reason, candidate rejection stage, geometry derivative/inverse queries, correction-pass invalidations, and reusable suffix work. The literature suggested reducing local-solve count, but the counters needed to decide the order of implementation. The follow-up below records what those measurements changed.

### Implementation and profiling follow-up

The first Windows Release profiles overturned the initial assumption that Ruckig was necessarily the dominant cost. WPR/WPA sampling of the 5,164-motion `adaptive_pockets.ngc` full horizon showed repeated spline and arc distance inversion ahead of local timing. NRT counters then established the exact workload:

- 8,724,852 spline inverse queries caused 11,979,293 adaptive inverse-integral evaluations;
- 3,809,104 arc inverse queries caused 7,593,087 adaptive inverse-integral evaluations;
- repeated bit-identical distances were common because tangent, curvature, curvature-derivative, timing, and ordered-proof paths revisit the same stations.

A piecewise monotone cubic spline inverse seed was prototyped first. It reduced inverse integrations by about 9.2%, but changed a threshold-sensitive `adaptive_pockets.ngc` trajectory from 280.926324594 s and 14,307 verified spans to 281.534705422 s and 14,313 spans. That violated the required output-identity criterion, so the surrogate was removed.

The accepted implementation instead gives each spline and arc reference a fixed 16-entry cache keyed by the bit-exact requested distance. A miss executes the original lookup-bracket, adaptive-integral, safeguarded-Newton path unchanged; a hit returns the parameter already certified by that path. This preserves ordered distance association and does not turn interpolation into authority.

On the Release `adaptive_pockets.ngc` full horizon:

- spline exact-cache hits were 7,377,859 and spline inverse integrals fell to 1,616,340, an 86.5% reduction;
- arc exact-cache hits were 2,237,411 and arc inverse integrals fell to 3,130,549, a 58.8% reduction;
- the exported oracle model remained byte-identical, including duration, verified spans, reachability candidates, and geometry attempts;
- full-horizon planning fell from roughly 5.40 seconds before either cache to about 3.01-3.08 seconds after both;
- the production-effort 54-horizon rolling run fell from about 12.69 seconds to 6.23 seconds while retaining 103 chunks, 329.769361022 seconds of planned motion, and identical rolling-boundary failure counts.

The same arc cache reduced inverse integrals by 65.7% on the dense line/arc fixture and 68.8% on `1001.ngc`, again with byte-identical exports. After both caches, WPR moved the leading cost to local timing: `timeLawBetween()` represented about 43% of `compileContinuous()` samples and Ruckig target calculation about 22%.

Call-purpose instrumentation then counted every `timeLawBetween()` result and measured its wrapper time. It classifies exact stops, continuous seeds, current-velocity station candidates, cap probes, and binary velocity-bracket probes; it also records correction-pass calls and bit-exact repeated eight-value input tuples within each compile operation. On the full `adaptive_pockets.ngc` horizon, 869,860 of 1,196,089 calls (72.7%) were exact repeats, 1,041,608 calls (87.1%) occurred after the initial correction pass, and 846,103 calls were bracket probes. The bracket category alone contained 603,701 repeats and consumed 1.174 of the 1.601 measured seconds. The 54-horizon rolling run made 2,002,238 calls across all published and failed attempts; 1,204,804 (60.2%) repeated inside their individual compile operation. Suffix probes accounted for 1,018,986 calls and prefix probes for 955,420. The full exported model retained SHA-256 `CA41770AD49C184156C1B4F915455931B139873095F4855166FA74545D4CC2C4`.

The initial implementation was a compile-local direct-mapped cache keyed by the eight input bit patterns. It stores only validated feasibility and duration, keeping entries compact. A cached failure rejects the same station candidate immediately. A cached success supplies its duration for comparison; if it becomes the new best candidate, Ruckig and the complete validation path run again to materialize the retained phases. Thus cached metadata can avoid work but cannot become emitted-trajectory authority. Its storage lifetime was later extended across compilations as recorded below without changing these authority rules.

Small horizons use 32,768 entries and horizons above 1,024 geometry pieces use 131,072 entries. The large table was the measured knee: doubling it to 262,144 eliminated another 77,707 solver calls on the full workload but did not improve its 2.45-second wall time. With 131,072 entries, full-horizon solver executions fell from 1,196,089 to 589,679, including 35,803 selected-hit materializations. Instrumented planning fell from 3.99 seconds to 2.45-2.60 seconds. Adaptive 32K/128K sizing reduced the rolling calculation from 6.80 to 5.00 seconds and actual solver executions from 2,002,238 to 920,136. Relative to the best pre-instrumentation cache checkpoints, the practical wall-time reduction is approximately 16-20% for the full horizon and 20% for rolling. All structural diagnostics and the oracle hash remained identical.

The endpoint-interval experiment then exposed a different repeated cost. Every candidate re-evaluated the same one-sided `T`, `kappa`, and `kappa_s` through spline/arc distance inversion on both sides of a station and on every correction pass. Caching those exact geometry values once per piece reduced the full workload to 3,750,131 spline inverse queries and 884,094 arc inverse queries. Full planning measured 1.88-1.92 seconds and rolling measured 3.76 seconds, with unchanged Ruckig call/cache counts and the same oracle hash. Relative to the timing-cache checkpoint, this is about a further 23-25% full-horizon and 25% rolling reduction.

The explicit analytic projection itself was rejected after an enabled/disabled Release A/B. Across 191,834 full-horizon velocity queries it found no empty acceleration or jerk interval. It preclassified 19,154 jerk-infeasible candidates, but those same candidates were already rejected by the exact endpoint authority check before Ruckig, so solver calls did not change. Projection increased full planning from 1.88 to 1.92 seconds and rolling from 3.76 to 4.09 seconds. The production code therefore retains exact one-sided geometry caching and lightweight rejection-stage counters, but not the interval projection. The full workload recorded 852,831 cached-geometry endpoint checks, zero acceleration rejections, and 19,154 jerk rejections; rolling recorded 33,312 jerk rejections across all attempts.

A separate 1% emitted acceleration/jerk allowance was also tested. The experiment applied the allowance to the analytic extrema correction gate and the independent publication gate while leaving velocity and continuity strict. Full-horizon motion shortened only 0.252653 seconds (0.090%), from 280.926324594 to 280.673671200 seconds, while planning rose from 1.917 to 2.056 seconds, verified spans rose from 14,307 to 14,322, candidate evaluations rose from 852,831 to 956,187, and the planner required one additional whole-horizon correction pass. Rolling motion shortened 0.207063 seconds (0.063%), but planning rose from 3.755 to 3.983 seconds and packet count rose from 103 to 104. The allowance was removed: on this workload it changes threshold-sensitive correction topology and costs substantially more planning work for negligible traversal-time benefit. Any future physical over-limit policy must be explicit and consistent across compilation and publication rather than hidden in the numerical `1e-9` comparison margin.

Correction-pass locality instrumentation now compares every replan with its preceding pass using bit-exact station `(v,a)` values and complete scalar `TimeLaw` phase boundaries. It records the pieces whose exact extrema reduced local limits, changed timing islands, unchanged timing islands, reusable prefix/suffix sizes, and propagation distance from the nearest corrected piece. On the full 3,053-piece `adaptive_pockets.ngc` horizon, seven replans compared 21,371 piece timings. Only 4,730 changed; 16,641 (77.9%) remained bit-identical. The changed timings formed 1,135 disjoint runs, and no change was more than six pieces from a corrected piece. A single min/max affected interval is therefore ineffective—the corrected set spans nearly the whole horizon in early passes—but an exact-keyed worklist of local islands has substantial potential. The final pass changed five timings around two corrected pieces while 3,048 remained exact.

The rolling workload strengthens that result. Across all published and failed probes, 457 replans compared 32,746 piece timings: 23,097 (70.5%) were bit-identical, and the farthest changed timing was seven pieces from a corrected piece. Published horizons alone retained 11,438 of 16,269 timings (70.3%). The next implementation should replay a station visit only when its complete local input state, neighboring retained timings, caps, and limits match the preceding pass bit-for-bit; otherwise it should enqueue that local island and expand until exact boundary state matches are recovered. This preserves pass ordering and avoids assuming a fixed geometric halo. The diagnostic-only implementation preserved the 280.9263245941939-second trajectory, all structural counters, and oracle SHA-256 `CA41770AD49C184156C1B4F915455931B139873095F4855166FA74545D4CC2C4`.

That station-visit replay was then tested in shadow mode. A visit is compared only with the same station, sweep, and direction in the immediately preceding correction pass. Its collision-free exact key contains the three neighboring station `(v,a)` states, station cap, both piece lengths and local limit triples, search effort, and every field of both adjacent scalar phase sequences. The planner still executes the visit; reuse is counted only if the resulting station state and both output phase sequences are also bit-identical. The mode is off by default because its vector serialization and double execution accounting are deliberately diagnostic; the exporter enables it with `--effort=replay-shadow`.

The full horizon made 32,611 active visits. Of 27,089 comparable visits, 17,368 had exact inputs and all 17,368 produced exact outputs, with zero mismatches. Those verified matches account for 453,274 candidate/endpoint evaluations, 628,532 logical timing calls (60.3% of correction-pass calls), 212,647 actual solver calls (36.1% of all solver calls), 33,341 selected-hit materializations, and about 0.51 seconds of measured visit work. Across all rolling attempts, 13,879 of 29,022 comparable visits matched exactly and again produced zero mismatches. They account for 488,148 candidate checks, 690,919 logical timing calls (43.9% of correction calls), 80,887 solver calls (8.8% of all rolling solver calls), 46,370 materializations, and about 0.19 seconds of visit work. Prefix and suffix probes independently had zero mismatches.

The lower rolling solver percentage was important at this checkpoint: the then-compile-local timing cache already made most repeated rolling calls cheap. A production replay therefore needed fixed-layout snapshots or direct bitwise field comparison, authoritative `TimeLaw` restoration, and reproduced candidate/resource-budget accounting. The experiment passed that gate and justified the lean real-replay A/B.

A deliberately quick real-replay prototype was then added as the explicit `--effort=replay` profile. It uses the already-proven serialized exact key, restores the retained station state and both authoritative adjacent `TimeLaw` values, and replays logical candidate, endpoint-rejection, velocity-search, and cumulative resource-budget counts. It does not replay timing-cache traffic, so the measured solver/call reductions are real. Full-horizon logical timing calls fell from 1,196,089 to 567,557 and actual solver calls from 589,679 to 373,819 while all 852,831 candidate evaluations and endpoint diagnostics remained unchanged. An initial run improved 2.11 to 1.55 seconds; a short paired run improved 1.92 to 1.67 seconds (13.1%). Rolling calls fell from 2,002,238 to 1,311,319 and solver calls from 920,136 to 838,465. A paired rolling run improved 4.066 to 3.859 seconds (5.1%).

The real replay retained 280.9263245941939 full duration, 14,307 spans, 58,416 geometry attempts, 54 rolling horizons, 103 chunks, 329.7693610220958 rolling duration, and all rolling failure counts. The model hash remained unchanged. The initial prototype still allocated serialized input/output vectors for every active visit. Replacing those vectors with one fixed 160-word exact key and direct bit-exact comparison of the retained output state and two `TimeLaw` values improved a paired full run from 1.979 to 1.513 seconds (23.6%) and rolling from 4.201 to 3.981 seconds (5.2%). Compact replay is therefore the production default; `--effort=no-replay` is the controlled baseline and `--effort=replay-shadow` continues to execute matching visits for validation without skipping them. Replay is intentionally limited to the production maximum of three station sweeps so high-effort offline profiles cannot multiply retained visit memory by 8-40 sweeps.

A planner-local prepared-geometry cache was tested next. It reused endpoint-exact arc/spline references, trimmed pieces, controls, and inverse tables across velocity attempts at one split. Alternating rolling runs averaged 3.722 seconds versus 3.855 seconds without it, only 3.45% faster. That result did not justify the exact command key, cached-object lifetime, diagnostic deltas, or changed suffix-planner ownership, so the experiment was removed.

The existing exact eight-scalar time-law cache was then made thread-local and process-lifetime, shared by every planner and compilation on one planning thread. Thread isolation avoids synchronization and data races between independent workers; within a thread, the bounded 128K allocation persists while small rolling horizons continue indexing their measured 32K range. Exact keys still contain length, both boundary `(v,a)` states, and velocity/acceleration/jerk limits, so geometry identity and planner lifetime are irrelevant. Selected successful hits still rematerialize authoritative Ruckig phases and pass validation.

With prepared geometry disabled, two alternating rolling comparisons averaged 3.609 seconds with the shared cache versus 3.829 seconds with compile-local caches, a 5.74% improvement. Solver executions fell 22.7%, from 838,465 to 648,024; cache hits rose from 477,956 to 700,107. Logical calls, candidate evaluations, endpoint diagnostics, horizons, chunks, duration, and rolling failure counts remained identical. The full-plus-rolling model retained SHA-256 `CA41770AD49C184156C1B4F915455931B139873095F4855166FA74545D4CC2C4`. The shared cache is now the default, `--effort=local-time-cache` is the control, and the prepared-geometry experiment remains removed in favor of this smaller and faster change.

The dependency was subsequently reduced from a complete Git submodule to a pinned vendored subset. NGC retains the 11 upstream Community Edition solver translation units and 12 headers required by its fixed-size offline state-to-state path, plus an NGC-owned offline facade and the verbatim MIT license. Link-map inspection showed that none of the 11 core solver objects could be dropped safely for the modes NGC exercises; the package-level reduction instead removes cloud/waypoint and online APIs, dynamic-DoF conveniences, wrappers, examples, tests, benchmarks, documentation, and upstream build machinery. Post-vendoring Debug/Release builds and tests passed, and the full `adaptive_pockets.ngc` oracle export remained byte-identical to the pre-vendoring cached export.

A follow-up Ruckig-only audit rejected two apparent micro-optimizations. Alternating Release runs showed no measurable end-to-end benefit from returning the profile container by const reference rather than by value: 10-run means differed by about 0.005%, within timing noise, so the extension was removed. Compiling only the Ruckig target at `-O3` preserved byte-identical output but did not improve timing and was often slower than `-O2`. WPR and source inspection place the remaining solver work in third-order quartic/root calculation and candidate-profile checks. Those are numerically sensitive feasibility machinery, not redundant packaging work. The defensible next step is still to avoid provably unnecessary calls before entering Ruckig, not to approximate or reorder its root/profile search.

### Direct smoothed-path infinite-jerk reference

The standalone exporter now also requests a library-owned acceleration-only timing reference directly from the exact smoothed line/arc/B-spline piece stream. For arc-length path coordinate `s`, squared speed `x = v^2`, scalar acceleration `a`, tangent `q'`, and curvature vector `q''`, the implementation uses:

```text
axis acceleration = q'(s) a + q''(s) x
dx/ds = 2a
```

At each geometry query it analytically intersects the aggregate path-acceleration ball and every per-axis acceleration interval, applies programmed-feed and per-axis velocity caps, propagates forward and backward maximum squared-speed envelopes, and integrates `dt = 2 ds / (v0 + v1)`. Successive uniform refinements must converge in duration. Exact circular/helical arc curvature comes from analytic first and second parameter derivatives; B-spline curvature uses the existing analytic arc-length derivative. The calculation does not call an external optimizer and does not use Ruckig to choose local transitions. It is opt-in through `compileContinuous()` and never enters normal planning, `PlanChunk`, `MotionBackend`, or RT execution.

The direct result is a numerically converged acceleration-only comparison, not an executable trajectory or a formal continuous lower-bound certificate. It uses the same complete smoothed geometry while removing jerk-derived local caps. Recorded full-horizon results are:

| Program | Jerk multiplier | Verified planner | Direct infinite-jerk | Planner excess |
| --- | ---: | ---: | ---: | ---: |
| `adaptive.ngc` | 1x | 307.532106312 s | 155.522529678 s | 97.7412% |
| `adaptive.ngc` | 10x | 180.944350029 s | 155.522529678 s | 16.3461% |
| `adaptive.ngc` | 100x | 175.358673959 s | 155.522529678 s | 12.7545% |
| `adaptive_pockets.ngc` | 1x | 305.274659685 s | 128.319614704 s | 137.9018% |
| `adaptive_pockets.ngc` | 10x | 165.646505067 s | 128.319614704 s | 29.0890% |
| `adaptive_pockets.ngc` | 100x | 154.051257265 s | 128.319614704 s | 20.0528% |

The last-refinement deltas were 0.000801955 seconds for `adaptive.ngc` and 0.000466037 seconds for `adaptive_pockets.ngc`. High-jerk exporter runs required more proof work rather than relaxed proof: `adaptive.ngc` converged in 35 correction passes at 10x and 34 at 100x, while `adaptive_pockets.ngc` 100x accumulated 943,434 ordered geometry-verification attempts before satisfying every final constraint. The production defaults are now 32 correction passes and 36 cumulative attempts per piece; only explicit offline jerk sweeps use the bounded larger ceilings.

## 1. Mathematical frame

For a path `q(s)` parameterized by arc length, define

```text
T       = q'(s)       unit tangent
kappa   = q''(s)      curvature vector
kappa_s = q'''(s)     curvature derivative with respect to arc length
v       = ds/dt
a       = d2s/dt2
j       = d3s/dt3
```

Then the axis-space derivatives are

```text
q_dot   = T v
q_ddot  = T a + kappa v^2
q_dddot = T j + 3 kappa v a + kappa_s v^3.
```

This is the right decomposition for the project because it exposes two different problems:

- geometry supplies `T`, `kappa`, and `kappa_s`;
- timing chooses scalar `(v, a, j)` while satisfying aggregate and per-axis constraints.

It also explains why curvature alone is insufficient. At nonzero speed and acceleration, jerk contains both the `3 kappa v a` term and the curvature-derivative term `kappa_s v^3`. The current planner is correct to retain both.

### Cheap exact endpoint filters

At a fixed station and speed, each axis acceleration constraint is a scalar interval for `a`:

```text
-A_i <= T_i a + kappa_i v^2 <= A_i.
```

The intervals from all axes can be intersected. Because arc length gives `||T|| = 1` and `T . kappa = 0`, the aggregate acceleration constraint has the especially simple form

```text
||q_ddot||^2 = a^2 + ||kappa||^2 v^4 <= A_path^2.
```

Thus it also yields an interval in `a`, or immediately proves the speed infeasible.

For jerk, let

```text
g = 3 kappa v a + kappa_s v^3.
```

Each axis bound constrains `j` through `-J_i <= T_i j + g_i <= J_i`. The aggregate bound constrains the distance from `-g` to the line spanned by `T`. With `g_perp = g - T(T . g)`, it is feasible only if `||g_perp|| <= J_path`; when feasible, it supplies a scalar interval centered at `-T . g`. Intersecting every interval proves whether *some* scalar jerk exists at that endpoint.

NGC already performs closely related coupled endpoint checks. Production measurements separated the two proposed changes: caching the one-sided geometry values was highly effective, while explicitly projecting candidate `a` intervals was not. On the measured dense workload no trial velocity had an empty projected interval, and every candidate rejected by projection was already rejected before Ruckig by the cheaper cached-geometry authority check. The interval formulas remain useful for future reachable-set construction, but are not a profitable standalone front end for the current candidate lattice.

## 2. What the path-parameterization literature contributes

### 2.1 Second-order reachability is mature and cheap

[TOPP-RA](https://arxiv.org/abs/1707.07239) makes the change of variables

```text
x = v^2
u = a,
```

which turns many velocity and acceleration constraints into linear inequalities at discretized path positions. It propagates reachable/controllable intervals backward and forward in time linear in the number of grid points and constraint inequalities. The authors report reliable behavior and better discretization error than common first-order approaches.

For NGC, TOPP-RA is valuable as:

- a fast velocity envelope;
- a lower-bound or diagnostic oracle;
- a way to identify acceleration-limited rather than jerk-limited regions;
- an initial seed for the exact third-order search.

It is not a complete solution because jerk makes the next state two-dimensional `(v, a)` and destroys the simple convex interval structure.

### 2.2 Exact third-order path parameterization remains difficult

[TOPP3](https://arxiv.org/abs/1609.05307) formulates time-optimal third-order parameterization as alternating maximum- and minimum-jerk profiles, connected by bridge and extension operations. The paper is useful for understanding switching structure and singular curves, but it also illustrates why a robust globally complete third-order solver is much harder than TOPP-RA. It should be treated as theory and an oracle direction, not as a proven production replacement for the current Ruckig-based local reachability tests.

[Ruckig](https://arxiv.org/abs/2105.04830) solves jerk-constrained state-to-state problems with arbitrary target position, velocity, and acceleration. Its implementation is extensively tested and supports directional limits. NGC's use of Ruckig as a local exact transition oracle is therefore a strong architectural choice. The first optimization target should be the *number of calls*, not the solver itself.

### 2.3 Sequential convex/linear methods are useful but have weaker guarantees

[A sequential approach for speed planning under jerk constraints](https://arxiv.org/pdf/2105.15095) also uses squared speed. Its discretized travel-time objective is convex, while the jerk term remains nonconvex because it contains a product such as `w'' sqrt(w)`. The authors solve a sequence of structured convex subproblems and report better scaling than a generic nonlinear optimizer.

[Lee et al., 2024](https://arxiv.org/pdf/2404.07889) use sequential linear programming for jerk-constrained industrial manipulator trajectories. The important caveat is that their discrete jerk is based on acceleration change between adjacent trajectory points divided by an average interval; it is deliberately more forgiving than an instantaneous continuous jerk bound. That can produce a useful seed or comparator but cannot replace NGC's exact cubic extrema checks.

[Path-Constrained Time-Optimal Motion Planning With Third-Order Constraints](https://doi.org/10.1109/TMECH.2023.3234584) uses a pragmatic four-stage strategy: obtain a second-order optimum, locate third-order violations, repair those regions through numerical integration, and use bisection to find switching points. This supports an attractive project hypothesis: solve the easy velocity/acceleration envelope globally, then spend jerk-aware optimization effort only in expanded neighborhoods that are actually jerk-active. Final exact compilation remains mandatory, and a failed proof would expand/revisit the active region rather than fall back.

[Time-Optimal Path Tracking for Jerk Controlled Robots](https://arpi.unipi.it/bitstream/11568/1003896/5/opt_path_jcr.pdf) uses McCormick envelopes to convexify nonconvex products. Such relaxations can provide lower bounds and prune candidate regions. Unless the relaxation gap is closed, however, they prove neither executable feasibility nor continuous jerk compliance.

### 2.4 A new reachability result is unusually relevant

[Reachability-Augmented Dual Dynamic Programming for Optimal Path Parameterization](https://arxiv.org/pdf/2605.19089) is a May 2026 preprint. For third-order path parameterization it represents backward-reachable `(squared speed, acceleration)` states with conservative two-dimensional polytopes, built from low-dimensional projections and sampled support directions. The paper reports substantial speedups over its convex-optimization baselines while maintaining success on its experiments.

This is highly relevant to NGC's rolling suffix and stop-branch obligations. A small conservative inner polytope at each station could answer:

```text
Can this proposed (v, a) still reach a valid future state or stop through the suffix?
```

before running both neighboring Ruckig transitions. It may also screen rolling split candidates. Because the method is recent and discretized, it should first be built as a standalone oracle and compared against current exact results. It must never accept a state by itself; conservative membership can reject or prioritize, while Ruckig and emitted-polynomial proof remain authoritative.

## 3. Rolling horizons and incremental planning

The most directly comparable CNC work is [Online time-optimal trajectory planning along parametric toolpaths with strict constraint satisfaction and certifiable feasibility guarantee](https://doi.org/10.1016/j.ijmachtools.2025.104355). It combines sequential piecewise linear programming with hierarchical look-ahead windows. A central idea is consistency of linearization points across overlapping windows so that one window does not invalidate the state assumed by another. The reported method removes many infeasibilities observed with older windowed strategies and scales to very large grids.

The paper's between-grid guarantee is still not the same as NGC's exact polynomial extrema and stop-tail proof. Its transferable ideas are instead:

- retain consistent boundary state between windows;
- reuse the overlapping window's linearization/solution data;
- carry a certified suffix-feasibility object, not just a preferred velocity;
- treat the newly appended tail as the region most likely to require recomputation.

For NGC, incremental reuse could include:

- retained line/arc and spline geometry objects;
- arc-length lookup and inverse brackets;
- one-sided `T`, `kappa`, and `kappa_s` at stations;
- sampled/certified local caps;
- last accepted `(v, a)` candidates and active constraints;
- conservative backward-reachable suffix sets;
- exact local transition results whose endpoints and limits are unchanged.

Backward reachable data must be invalidated from the changed tail toward the prefix until a mathematically sufficient fixed point is reached. Merely observing that numbers changed little is not a proof. Forward timing may likewise need propagation when the boundary state changes. Even partial reuse of geometry and failed-candidate knowledge is likely worthwhile if reachability reuse proves too difficult initially.

## 4. Reducing station count without changing program geometry

The [LinuxCNC trajectory-planning documentation](https://linuxcnc.org/docs/html/user/user-concepts.html) describes "naive CAM" handling that combines short, nearly collinear moves before blending. NGC cannot directly copy this: its G64 `P` is a blend/control scale rather than LinuxCNC's path tolerance, and accepted canonical geometry must not be silently altered.

There is a narrower exact opportunity. A run composed of:

- exactly collinear, same-direction line geometry;
- equal programmed feed and compatible timing limits;
- any intervening blend pieces that are themselves exactly linear on the same line;

can be treated as one timing entity. Source-command activations remain interior markers. The final polynomial stream can be split at the corresponding times/distances so that each activation still belongs to the correct span, without forcing the optimizer to regard every activation as a dynamic station.

This could be particularly valuable for CAM output containing many tiny straight fragments. It is less risky than near-collinear simplification and does not change G64 semantics. Arc coalescing should wait: decimal-IJK endpoint blending, directed sweep, and exact source-interval association make equivalence much harder to prove.

## 5. Faster certified geometry evaluation

### 5.1 Bernstein/Bezier interval bounds

For a polynomial in Bernstein form, its range lies in the convex hull of its coefficients. Recursive subdivision tightens that enclosure. This is the same family of ideas already used by NGC's ordered Bezier geometry verification. Background references include [range enclosure with Bernstein polynomials](https://doi.org/10.1007/s11633-007-0342-7), [Bernstein methods for global optimization](https://doi.org/10.1007/s10898-008-9382-y), and [Bezier clipping for polynomial roots](https://www.cise.ufl.edu/~jyoungqu/Youngquist.pdf).

Potential uses in the planner are:

- bound derivative norms over a whole spline interval;
- prove a velocity/acceleration/jerk cap without dense sampling;
- locate intervals that might contain the active extremum;
- subdivide only ambiguous intervals;
- establish monotonicity or denominator separation before using a rational bound.

Curvature and curvature derivative are rational expressions in spline derivatives, so careful denominator lower bounds are required. If these bounds become part of a proof, outward-rounded interval arithmetic or an equivalent conservative error policy is needed. Simple control-coefficient bounds can be too loose; their value comes from cheap rejection/acceptance plus targeted subdivision.

The exact extrema of already emitted cubics are comparatively cheap and should remain unchanged. The better target is repeated path-geometry cap estimation and verification before emission.

### 5.2 Cached inverse arc-length maps

[Arc Length Parameterization of Spline Curves](https://www.saccade.com/writing/graphics/RE-PARAM.PDF) combines adaptive quadrature with a Chebyshev approximation of the inverse arc-length map. A similar per-piece cache could provide a fast initial `u` and a small certified bracket for repeated `sampleAtDistance` calls. Safeguarded Newton iteration against the true integral can then preserve NGC's existing authority and ordered distance association.

A viable cache would need:

- monotone subintervals;
- an explicit inverse-error bound or verified bracket;
- exact endpoint preservation;
- fallback to the existing integration/inversion path;
- invalidation only when the geometry changes, not on every timing correction pass.

Unchecked table interpolation would violate the project's stated arc rules. A surrogate is useful only as a seed or bound.

The implemented result is deliberately narrower and stronger than the proposed surrogate. Exact requested distances are memoized only after the existing adaptive-integral/Newton path certifies their parameters. A 16-entry per-reference cache was enough to remove most repeated spline integrations and more than half of repeated arc integrations without changing any tested trajectory output. The monotone cubic seed experiment is retained as negative evidence: a numerically accurate seed can still change a threshold-sensitive planning decision, so approximate inverse seeds require stronger equivalence or interval reasoning before production use.

### 5.3 Pythagorean-hodograph curves

For a general polynomial spline, speed is the square root of a polynomial, so arc length generally requires numerical integration and the unit tangent is not rational. Pythagorean-hodograph (PH) curves are constructed so speed is polynomial. This gives polynomial arc length and rational tangent/curvature quantities. Relevant references include [Pythagorean-Hodograph B-Spline Curves](https://arxiv.org/abs/1609.07888), [quintic PH splines for high-speed cornering](https://escholarship.org/uc/item/2v5420wr), and [time-optimal algorithms for curved CNC paths](https://escholarship.org/uc/item/1x12k114).

PH geometry could remove much of the quadrature/inversion cost for future blend representations. The price is substantial:

- common constructions are planar and quintic rather than general six-axis cubic splines;
- PH constraints reduce the available control freedom;
- replacing current G64 geometry would change semantics and preview agreement;
- compilation and exact proof would need higher-degree machinery.

It is therefore a long-term geometry experiment, perhaps first for common XYZ planar blends, not a transparent optimization of the existing representation.

## 6. Smoothing, curvature variation, and machine excitation

[Path planning for CNC machines considering centripetal acceleration and jerk](https://www.shahzadeh.com/papers/Path-planning-for-CNC-machines-considering-centripetal-acceleration-and-jerk.pdf) emphasizes curvature derivative ("sharpness") as well as curvature. It also exposes an important tradeoff: subdividing a transition more finely can reduce geometric error while increasing sharpness, which can lower the jerk-limited feed. More geometric pieces are not automatically better dynamically or computationally.

Clothoids have curvature linear in arc length, making sharpness constant, and are attractive for planar `G2` transitions. They do not eliminate timing optimization and are awkward for mixed XYZABC paths. Like PH curves, they are best considered a future alternative representation rather than a change to the present G64 experiment.

Finite-impulse-response filtering is another common CNC strategy. Examples include [accurate FIR interpolation for CNC systems](https://nagoya.repo.nii.ac.jp/record/26378/file_preview/PE_Accurate_interpolation_Burak_Manuscript-JJSPE-D-17-00135.pdf) and [FIR interpolation with bounded axial and tangential kinematics](https://doi.org/10.1016/j.ijmecsci.2019.105325). Cascaded filters can generate bounded higher derivatives online and can incorporate frequency-domain shaping. Their disadvantages are delay, corner error, and transition durations that need not adapt optimally to the size of a speed change. Filtering also changes the path, so it is incompatible with NGC's current exact retained G64 geometry unless introduced as a separately specified mode.

The broader lesson is useful for future hardware work: a scalar jerk bound does not by itself minimize excitation at a structural resonance. Once a real plant model is in scope, frequency-shaped input or snap/jerk-variation limits may matter more than reducing traversal time by the last few percent. That concern should remain outside the present generic motion contract until the hardware model exists.

## 7. Surface and neighboring-path consistency

[On the consistency of path smoothing and trajectory planning in CNC machining: A surface-centric evaluation](https://www.sciencedirect.com/science/article/pii/S0736584524001601) argues that independently acceptable toolpaths can still create isolated surface marks if adjacent, nearly identical paths make discontinuously different smoothing or feed decisions.

This suggests a valuable benchmark category for NGC, even before physical cutting:

- translate a source path by a very small offset and compare blend topology;
- perturb an entity length across the `6P` short-run threshold;
- perturb feed or curvature slightly across a station-refinement threshold;
- compare resulting speed profiles, active constraints, and correction-pass choices;
- detect large discontinuities caused by discrete planner decisions.

This is not an RT-contract requirement, but it could reveal quality problems that ordinary single-path correctness and time-optimality tests miss.

## 8. Ranked project-specific research hypotheses

| Priority | Hypothesis | Expected benefit | Main risk / proof obligation |
|---|---|---|---|
| 1 | Lean exact correction-pass station-visit replay | Implemented with a fixed exact key; paired full/rolling gains are 23.6%/5.2% | Preserve exact-key coverage, pass order, budget accounting, and exact final proof |
| 2 | Conservative backward `(v,a)` suffix polytopes | Prunes local solves and rolling-boundary failures; directly encodes stop feasibility | New/discretized method; use only as inner rejection/priority filter until independently verified |
| 3 | Collapse exact collinear same-feed timing runs while retaining activation markers | Reduces station/sweep count for dense CAM paths without geometry change | Must split emission/activation ownership exactly; differing feeds prevent simple collapse |
| 4 | Reuse retained rolling-window geometry and reachability data | Avoids repeating work on immutable suffixes | Must conservatively propagate invalidation when a changed tail affects earlier states |
| 5 | Cache exact one-sided endpoint geometry | Implemented; cuts full planning to 1.88-1.92 s and rolling to 3.76 s | Geometry must remain bit-identical and local-limit independent across correction passes |
| 6 | Compile-local bit-exact `timeLawBetween()` result memoization | Implemented; removes 50.7% of full-horizon and 54.0% of rolling solver executions | Keep cached duration non-authoritative; selected hits must rematerialize through Ruckig |
| 7 | Extend certified geometry caching and Bernstein bounds | Exact-distance spline/arc caches are implemented; bounds may reduce remaining unique work | Approximation cannot become unverified authority; rational bounds need denominator proof |
| 8 | Optimize jerk-active regions after a second-order envelope | Concentrates expensive search where jerk matters | Active regions may expand after exact compilation; requires iterative conservative repair |
| 9 | Extend call instrumentation with candidate rejection-stage timing | Rejection counts are implemented; stage timing can explain remaining unique work | Counters must stay NRT-only and cheap enough not to distort profiles |
| 10 | Parallel geometry preprocessing and per-span proof checks | Exploits NRT cores without touching RT execution | Preserve deterministic diagnostics; station sweeps themselves are sequential |
| 11 | PH/clothoid geometry experiment | Potentially cheaper arc length and smoother curvature behavior | Large semantic and proof change; not a near-term optimization |

### Suggested rejection pipeline

For each proposed station state, the following order appears promising:

1. velocity cap / second-order reachable envelope;
2. cached-geometry exact coupled endpoint check;
3. conservative backward suffix/stop-feasible membership;
4. cheap scalar transition lower bound;
5. the two neighboring Ruckig position solves;
6. exact emitted polynomial and stop-branch proof.

Every stage should report how many candidates it rejected and how much time it consumed. If an earlier stage costs more than the Ruckig calls it avoids, it should be simplified or removed.

## 9. Suggested standalone experiments

These are research programs/oracles, not integration proposals.

### Experiment A: cost anatomy

Replay representative horizons and count:

- Ruckig calls by seed, candidate, velocity bracket, correction, and stop tail;
- failure/success counts and time distribution;
- endpoint rejections before Ruckig;
- arc/spline derivative and inverse-distance evaluations;
- geometry subdivision nodes;
- correction passes and the portion of the horizon recomputed;
- identical/reusable station or transition queries;
- rolling suffix size reused versus rebuilt.

Include straight dense CAM, arc-only regions, mixed line/arc G64, short-entity clusters, six-axis motion, and inputs that require local correction.

Initial WPR/WPA sampling, inverse-query counters, `timeLawBetween()` call-purpose instrumentation, compile-local exact-result memoization, endpoint rejection-stage instrumentation, correction-pass locality measurement, shadow replay, and compact real station-visit replay are complete. One-sided station geometry is computed once per piece; compact replay is now the default after byte-identical paired gains of 23.6% full and 5.2% rolling. Continue by measuring remaining unique transitions and rolling suffix reconstruction.

### Experiment B: analytic interval front end

Completed as a rejection-only production prototype and removed after the controlled A/B. It found no empty velocity-level interval on `adaptive_pockets.ngc`, avoided no Ruckig calls, and cost roughly 0.04 seconds full-horizon and 0.34 seconds rolling relative to cached endpoint geometry alone. The successful geometry precomputation and rejection counters remain; the projection formulas should be revisited only as part of a stronger backward-reachable-set representation.

### Experiment C: backward suffix polytopes

Implement a small offline `(v^2, a)` backward-reachability oracle inspired by the 2026 RDDP paper. Compare:

- membership against states accepted by the exact current planner;
- false rejection rate from conservative approximation;
- Ruckig calls avoided at 8, 16, and 32 support directions;
- behavior specifically at rolling line-interior split candidates;
- construction/update time when one new entity is appended.

The target is not to generate final trajectories. It is to discover whether a small inner set is an effective filter.

### Experiment D: exact timing compression

Analyze existing/test CAM programs and count how many dynamic stations remain after collapsing only provably collinear, same-direction, equal-feed linear timing pieces. Retain all source activation markers in the analysis. This provides a likely upper bound on benefit before changing planner structures.

### Experiment E: certified geometry cache

Completed in production with a bit-exact result cache rather than a Chebyshev authority. The accepted implementation preserves the original miss path and byte-identical exports. A monotone cubic inverse seed was measured and rejected because it changed threshold-sensitive output even though it reduced integral calls. Future approximate inverse work must first explain and bound that semantic sensitivity.

### Experiment F: offline optimality oracle

Use a separate offline SLP/SCP model to compute discretized lower bounds and seeds if that research becomes useful again. If repeated fixed-sparsity QPs become useful, [OSQP](https://osqp.org/docs/) supports cached factorization, warm starts, and vector/matrix-value updates. A discretized optimizer result is not an executable proof.

## 10. Approaches that should not be imported uncritically

- **Sample-only constraint checking.** It can miss polynomial extrema and does not satisfy the current contract.
- **Average discrete jerk.** It is not equivalent to instantaneous jerk and may admit violations between stations.
- **Unchecked inverse arc-length interpolation.** It can break ordered source association and endpoint-exact geometry.
- **A generic nonlinear optimizer in the production loop.** Convergence, feasibility restoration, and latency are weaker than the present fatal-on-unproved-result policy.
- **FIR smoothing under the existing G64 mode.** It changes path geometry, adds delay, and has different semantics.
- **Near-collinear CAM simplification without a new tolerance contract.** Exact collinear compression is safe; approximate simplification is a separate language/planner decision.
- **Whole-horizon time stretching after a local violation.** It conflicts with the present local-limit correction design and wastes timing quality.
- **Assuming monotonic jerk feasibility in speed.** Coupled `(v,a)` reachability and switching structure need proof; bisection is safe only for a demonstrated monotone predicate.
- **Treating a new preprint's reported speedups as established.** The RDDP idea is promising enough to prototype, not enough to make authoritative.

## 11. Recommended reading order

1. [TOPP-RA](https://arxiv.org/abs/1707.07239) - the cleanest foundation for second-order reachability and squared-speed variables.
2. [Ruckig](https://arxiv.org/abs/2105.04830) - context for why the current local transition oracle is valuable.
3. [Sequential jerk-constrained speed planning](https://arxiv.org/pdf/2105.15095) - a clear formulation of where jerk introduces nonconvexity.
4. [RDDP for optimal path parameterization](https://arxiv.org/pdf/2605.19089) - the most relevant new idea for small backward stop-feasible sets.
5. [Online CNC planning with hierarchical look-ahead](https://doi.org/10.1016/j.ijmachtools.2025.104355) - useful rolling-window consistency and reuse concepts.
6. [Centripetal acceleration and jerk in CNC paths](https://www.shahzadeh.com/papers/Path-planning-for-CNC-machines-considering-centripetal-acceleration-and-jerk.pdf) - geometry/timing coupling and the sharpness tradeoff.
7. [PH B-spline curves](https://arxiv.org/abs/1609.07888) - longer-term analytic geometry alternative.
8. [Surface-centric path consistency](https://www.sciencedirect.com/science/article/pii/S0736584524001601) - broader quality metrics beyond feasibility and traversal time.

## Bottom line

NGC should keep its current proof architecture. The profiling follow-up validates the research strategy: first remove repeated work with proof-preserving caches, then optimize the measured next layer. Exact-distance caches, compile-local scalar timing memoization, and exact one-sided geometry reuse now remove most repeated adaptive integration, local-solver, and endpoint-sampling work without changing motion semantics or any tested export. The standalone analytic interval front end did not earn its cost and was removed. The next measured choice is a conservative backward stop-feasible set versus exact straight-run timing compression. Ruckig rematerialization and exact emitted-polynomial/stop-branch checks remain authoritative for every accepted trajectory.
