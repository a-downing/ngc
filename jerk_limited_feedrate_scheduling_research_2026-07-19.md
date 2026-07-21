# Research: Alternatives to NGC's Verifier-Guided Jerk-Limited Feedrate Scheduling

Status: the HiGHS-backed sequential-refinement experiment proposed here was implemented and then removed from production. On the measured NGC workloads, the reachability/Ruckig candidate was faster and slightly more time-optimal without that refinement. References to HiGHS and SCP below are retained as research alternatives, not descriptions of the current planner or build dependencies.

## Executive conclusion

NGC's strict scheduler is best described as a hybrid of:

- Third-order time-optimal path parameterization in state `(s, v, a)` with scalar jerk control.
- Discretized forward/backward reachability.
- Local one-dimensional Ruckig boundary-value solves.
- A counterexample-guided outer loop in which the exact execution-span verifier tightens sampled scalar limits and replans.

No single established algorithm exactly matches that combination. Dynamically, the closest established method is TOPP3. Algorithmically, the correction loop resembles an exchange or constraint-generation method, but its corrections are indirect scalar-cap reductions rather than persistent constraints on the violated execution polynomial.

The most promising replacement is not a pure TOPP3 implementation. It is a sparse, feasibility-preserving sequential convex programming scheduler over squared speed and path acceleration, coupled to the existing exact verifier as a persistent constraint-generation oracle. Reoptimize only an affected band bounded by verified PVA seams, and retain the current packet, capacity, C2, and stop-tail gates unchanged.

This should attack the observed ten-pass, 25-second behavior directly: stop discarding work between passes, stop converting exact axis-space counterexamples into broad scalar limit reductions, and stop replanning the entire horizon when only a small active set changed.

This recommendation is an engineering inference based on the cited algorithms. No published method I found provides all of NGC's requirements—full coupled jerk, arbitrary nonzero PVA boundaries, fixed geometry, exact emitted-cubic certification, rolling horizons, packet bounds, and independently certified stop tails—in one proven implementation.

## What the current code does

The implementation is stricter and more sophisticated than a conventional sampled feedrate scheduler.

In [`TrajectoryCompiler.cpp`](src/TrajectoryCompiler.cpp), continuous compilation:

- Splits prepared curves into timing pieces, including cluster-spline knot intervals.
- Projects rolling boundary states into scalar path velocity and acceleration.
- Initializes per-piece velocity, acceleration, and jerk limits from discrete geometric samples.
- Performs velocity reachability sweeps and acceleration-aware station searches.
- Solves each local scalar segment as a position-constrained, jerk-limited Ruckig problem.
- Emits lines directly as cubic axis polynomials and curved motion through a C2 cubic approximation chain.
- Exactly verifies the resulting axis-space execution polynomials.
- Reduces local scalar limits according to observed velocity, acceleration, and jerk ratios and repeats.

The final verifier evaluates exact extrema of cubic execution spans: velocity is quadratic, acceleration is affine, and jerk is constant. The associated helpers are in [`TrajectoryCompiler.h`](src/include/machine/TrajectoryCompiler.h). The default correction ceiling is 32 passes in the same header.

The rolling planner in [`TrajectoryPlanner.h`](src/include/machine/TrajectoryPlanner.h) already supports prescribed nonzero PVA boundaries at retained-line anchors, proves the suffix remains stoppable, verifies packet boundaries, and separately verifies stop tails.

[`InfiniteJerkTrajectoryTime.cpp`](src/InfiniteJerkTrajectoryTime.cpp) is a second-order baseline, despite its name. It uses `w = v^2`, derives velocity and acceleration feasibility from path derivatives, and integrates forward/backward acceleration envelopes with adaptive spatial refinement. It does not carry path acceleration as a boundary state and does not impose jerk. It is closest to the Bobrow/TOPP/TOPP-RA family, not to NGC's strict third-order compiler.

## Answers to the ten questions

### 1. What established algorithm is closest?

For the strict scheduler, TOPP3 is closest in its dynamics. It treats path position, path velocity, and path acceleration as state and scalar path jerk as control. Its generic third-order constraint has the form:

```text
A_i(s) * j + B_i(s) * v * a + C_i(s) * v^3 + D_i(s) <= 0
```

That is directly compatible with an axis component of NGC's coupled jerk:

```text
q1_axis * j + 3 * q2_axis * v * a + q3_axis * v^3
```

TOPP3 integrates maximum- and minimum-jerk profiles, handles velocity and acceleration boundary conditions, and uses multiple shooting to bridge profiles. Its authors explicitly address singular curves, but also identify unresolved complexity when lower-order and third-order constraints interact. [Pham and Pham, "Time-Optimal Path Parameterization with Third-Order Constraints"](https://arxiv.org/pdf/1609.05307).

NGC differs in several important ways:

- It discretizes the path and stitches local Ruckig solutions instead of integrating continuous extremal profiles.
- It supports mixed programmed-feed, path, per-axis, and coupled constraints.
- It verifies emitted axis-space cubics, not merely the scalar time law.
- Its outer loop resembles counterexample-guided refinement or an exchange method, but the generated correction is a reduced scalar cap rather than an explicit violated constraint.

For `InfiniteJerkTrajectoryTime`, TOPP-RA is the closest established algorithm. TOPP-RA uses squared speed and reachability sets to solve discretized first- and second-order path constraints with small linear programs and linear scaling in grid size. [Pham and Pham, TOPP-RA](https://arxiv.org/abs/1707.07239).

### 2. Can algorithms handle third-order path constraints directly?

Yes, but each has limitations.

TOPP3 directly evolves `(s, v, a)` under jerk control. It avoids repeatedly converting jerk violations into sampled scalar jerk caps. It supports prescribed initial and final velocity and acceleration. Its main risks are singularities, difficult switching structures, and incomplete handling of arbitrary mixtures of first-, second-, and third-order constraints. [TOPP3 paper](https://arxiv.org/pdf/1609.05307).

Palleschi et al. formulate fixed-path planning with variables related to squared velocity, acceleration, spatial acceleration derivative, and time. They retain the full axis jerk expression and use McCormick envelopes plus spatial branch-and-bound to handle the bilinear term. This provides a route to global lower bounds and global solution to a selected tolerance, but reported solve times range from seconds to thousands of seconds, depending on the instance. [Palleschi et al., "Time-optimal path tracking for jerk controlled robots"](https://arpi.unipi.it/bitstream/11568/1003896/5/opt_path_jcr.pdf).

Direct collocation can impose the full dynamics and coupled jerk as nonlinear constraints at collocation points. It readily supports arbitrary PVA boundaries, but produces a local nonlinear program and does not certify between-point behavior without additional mesh-error or interval analysis. [Kelly, "An Introduction to Trajectory Optimization"](https://epubs.siam.org/doi/full/10.1137/16M1062569).

A recent reachability-augmented dual dynamic programming preprint targets second- and third-order path parameterization, reporting substantial speedups over convex baselines. It is promising, but too new to treat as an established production choice, and no mature reusable implementation was located. [RDDP preprint](https://arxiv.org/abs/2605.19089).

### 3. How do published methods handle the nonconvex coupled jerk term?

There are five main strategies.

1. Direct state-space treatment

   TOPP3 observes that, at a fixed `(s, v, a)`, the third-order inequalities are affine in scalar jerk. They therefore define an admissible jerk interval and integrate its upper or lower boundary. The global dependence on `v*a` and `v^3` remains nonlinear, but it is embedded in the state evolution rather than globally convexified. [TOPP3](https://arxiv.org/pdf/1609.05307).

   Per-axis NGC jerk fits this form directly. NGC's aggregate Euclidean jerk norm is not a collection of linear inequalities, but because `j` is scalar, the feasible set at fixed `(s, v, a)` is still an interval obtainable from a scalar quadratic inequality. Extending TOPP3 this way is an NGC-specific inference, not a result demonstrated in the paper.

2. Sequential linear or convex approximation

   Lee et al. use squared-speed variables and conservatively linearize a nonlinear positive term in their discretized jerk constraint. Each iteration is an LP. They report millisecond-scale solutions for small grids, but warn that discretization and spline reconstruction can cause between-knot jerk violations. Their jerk formulation is not NGC's complete continuous coupled jerk. [Lee et al., ICRA 2024](https://arxiv.org/abs/2404.07889).

   Consolini, Locatelli, and Minari use sequential convex approximation with feasible-direction and line-search machinery. Their discrete scalar formulation maintains feasible iterates and converges to a stationary point under their assumptions. The published model is simpler than NGC's complete axis-space jerk constraint. [Consolini et al., T-ASE](https://air.unipr.it/retrieve/e177fbc7-8016-50b0-e053-d805fe0adaee/TASE3111758.pdf).

3. Convex relaxation

   McCormick envelopes relax products such as `v*a` or equivalent spatial variables. Spatial branch-and-bound can close the relaxation gap, but worst-case complexity is exponential. [Palleschi et al.](https://arpi.unipi.it/bitstream/11568/1003896/5/opt_path_jcr.pdf).

   A newer SOCP formulation by Consolini and Locatelli gives strong results for scalar path velocity, acceleration, and jerk bounds and can be exact under stated assumptions. Those assumptions do not cover NGC's spatially varying axis derivatives and complete vector coupled jerk. [Consolini and Locatelli, SOCP formulation](https://arxiv.org/abs/2310.07583).

4. General nonlinear programming

   Direct collocation or multiple shooting sends the nonlinear term to an SQP or interior-point solver. This is flexible but provides local, numerical feasibility unless paired with certified continuous-time constraint checking. [Kelly](https://epubs.siam.org/doi/full/10.1137/16M1062569), [GPOPS-II](https://www.anilvrao.com/Publications/JournalPublications/GPOPS-II-TOMS-ACM.pdf).

5. Conservative analytic envelopes

   A sound cell-wise envelope follows from the triangle inequality:

```text
full_jerk_bound =
    max_norm(q1) * abs(j)
  + 3 * max_norm(q2) * max(v) * max(abs(a))
  + max_norm(q3) * max(v)^3
```

For an individual axis, use absolute component bounds instead of vector norms. If the derivative maxima are certified over the complete cell, this is a continuous conservative constraint, not a sampled approximation. It can be quite restrictive when jerk-vector components cancel.

That particular envelope is my inference from NGC's exact chain rule. It is a useful feasibility-restoration and initialization constraint, not a claim from one of the cited optimization papers.

### 4. Can execution-polynomial constraints be optimized directly?

Partly, yes.

Once an execution span's topology and duration are fixed, NGC's cubic axis polynomial has tractable exact constraints:

- Axis jerk is constant.
- Axis acceleration is affine, so component extrema occur at endpoints.
- Axis velocity is quadratic, so extrema occur at endpoints or its one stationary point.
- The norm of affine acceleration reaches its maximum at an interval endpoint.
- Aggregate jerk is constant.

These constraints can be embedded as nonlinear inequalities. The complications are:

- Span duration appears in denominators and changes the coefficients.
- The velocity stationary point exists only under a coefficient-dependent case condition.
- Curved motion's cubic subdivision topology currently changes as timing and approximation tolerances change.
- Packet and stop-tail construction introduces additional discrete structure.

Three viable strategies are:

- Freeze the execution-span topology for an optimization epoch and impose its exact nonlinear extrema constraints.
- Use case-aware SQP constraints for exact extrema.
- Use Bernstein control-point bounds as sufficient constraints, subdividing with de Casteljau when too conservative. Bernstein convex-hull bounds and subdivision are established tools for continuous polynomial constraint certification. [BeBOT](https://arxiv.org/abs/2010.09992).

An LP or SOCP master cannot generally represent all exact emitted-span constraints without either conservative bounds or fixed/case-split structure. The exact current verifier should therefore remain the final authority even after some constraints move into the optimizer.

### 5. Could an active-set method avoid whole-horizon replanning?

Yes, and this is likely the highest-value improvement.

Semi-infinite programming exchange methods solve a finite master problem, ask a continuous-time oracle for the worst violation, add the violated constraint, and warm-start the next master. Trust regions prevent large changes that invalidate the local model. Hauser demonstrates this pattern for continuously constrained trajectory optimization. [Hauser, "Semi-infinite programming for trajectory optimization"](https://journals.sagepub.com/doi/10.1177/0278364920983353).

For NGC, the existing exact verifier can serve as the separation oracle:

```text
candidate time law
    -> emit exact execution cubics
    -> verifier returns constraint, span, and exact extremum time
    -> add a persistent cut or local refinement
    -> reoptimize the affected PVA-bounded band
```

This mapping is an engineering inference. The current verifier reports counterexamples, but a useful optimization cut additionally needs:

- The exact extremum location.
- The relevant polynomial coefficients.
- A mapping back to station variables.
- A locally valid sensitivity or conservative trust-region inequality.

Even before such derivatives exist, a working-set scheduler can preserve unchanged verified prefixes and suffixes and re-solve only the connected band influenced by a correction.

### 6. How can violations be prevented from moving?

A scalar cap reduction does not guarantee this. It changes station states, which changes neighboring timing, span subdivision, and the location of active jerk extrema.

A safer correction policy is:

- Maintain a fully verified feasible incumbent.
- Never drop a verifier-generated constraint merely because it is currently inactive.
- Use feasibility-preserving inner approximations or a feasible line search.
- Restrict each update to a trust region around the incumbent.
- Freeze a verified prefix or suffix only at a position-, velocity-, and acceleration-continuous seam.
- Require the frozen suffix to remain controllable and stop-feasible from that seam.
- Include at least one guard span on both sides of the nominal affected region.
- Expand the optimization band if the fixed PVA seam is infeasible.
- Reverify the entire changed region, both joins, packet boundaries, and all affected stop tails.
- Reverify the complete plan before publication.

This is a monotone feasibility-restoration approach: optimization can improve time only while retaining a proven incumbent. If no improving feasible step exists within the budget, keep the incumbent rather than publishing an unproved candidate.

For rolling boundaries with prescribed nonzero PVA, simple global time dilation is not valid because it changes boundary velocity and acceleration. Feasibility restoration must preserve those boundaries, typically by retaining a verified boundary segment, constructing a boundary bridge, or solving a fixed-boundary feasibility problem.

### 7. Which methods support nonzero PVA boundaries and rolling horizons?

| Method | Nonzero velocity | Nonzero acceleration | Rolling-horizon suitability |
|---|---:|---:|---|
| TOPP-RA | Yes | No, not as a path-state boundary | Good warm-starting, but insufficient state |
| TOPP3 | Yes | Yes | Conceptually good; shooting and singularities complicate incremental use |
| McCormick/global formulation | Yes | Yes | Poor due to solve-time variability |
| Direct collocation/multiple shooting | Yes | Yes | Good formulation; warm-startable but no tight runtime guarantee |
| Feasible SCP/SLP with `(w,a)` state | Yes | Yes | Very good if active set and factorization are reused |
| Ruckig | Yes | Yes | Excellent for local state-to-state bridges, not fixed curved-path TOPP |
| OpenCN-style receding B-spline planning | Yes | Tangential acceleration can be inherited | Explicit receding-horizon design, but conservative and not exact-span certified |

Ruckig supports arbitrary initial and target PVA for jerk-limited state-to-state motion and has extensive validation, but it does not solve fixed-geometry curved-path timing by itself. It remains highly useful for launch, landing, repair bridges, and stop tails. [Ruckig paper](https://arxiv.org/abs/2105.04830).

OpenCN explicitly documents a receding-horizon B-spline feedrate planner that retains the first curve and carries forward velocity and tangential acceleration. It uses conservative LP approximations for jerk and documents infeasibility and performance limitations. [OpenCN feedrate-planning documentation](https://mecatronyx.gitlab.io/opencnc/opencn/CNC_Path_Planning_Algorithms/Feedrate_Planning/Feedrate_Planning.html).

### 8. What guarantees are available?

No candidate provides all the guarantees NGC needs.

- TOPP-RA: linear grid scaling and asymptotic optimality under the paper's discretization assumptions, but only first- and second-order constraints. It does not prove emitted cubic execution. [TOPP-RA](https://arxiv.org/abs/1707.07239).
- TOPP3: directly treats third-order dynamics and prescribed PVA, but does not provide a general completeness or global optimality guarantee for arbitrary mixed constraint switching. Singularities and profile connection can fail. [TOPP3](https://arxiv.org/pdf/1609.05307).
- Feasible SCA: feasible, monotonically improving iterates and convergence to a stationary point for the published discrete formulation. Those results do not automatically extend to NGC's continuous full-jerk and emitted-span constraints. [Consolini et al.](https://air.unipr.it/retrieve/e177fbc7-8016-50b0-e053-d805fe0adaee/TASE3111758.pdf).
- SOCP: polynomial-time convex solution and strong exactness results for its restricted scalar model; not a general proof for spatially varying full coupled jerk. [SOCP paper](https://arxiv.org/abs/2310.07583).
- McCormick branch-and-bound: global convergence to a selected tolerance in principle, with exponential worst-case complexity and unsuitable runtime variability. [Palleschi et al.](https://arpi.unipi.it/bitstream/11568/1003896/5/opt_path_jcr.pdf).
- Direct collocation: sparse and flexible but normally converges only to a local NLP solution. Adaptive mesh error estimation is not the same as NGC's exact execution-polynomial proof. [GPOPS-II](https://www.anilvrao.com/Publications/JournalPublications/GPOPS-II-TOMS-ACM.pdf).
- Constraint generation: can converge to a locally valid continuously constrained solution under suitable oracle and regularity assumptions. A bounded number of cuts gives bounded runtime but not completeness. [Hauser](https://journals.sagepub.com/doi/10.1177/0278364920983353).
- Current exact verifier: sound publication gate for the constraints it checks. Explicit iteration, span, cut, and solver-time ceilings provide bounded failure, not guaranteed success.

For NGC, "bounded and safe" should continue to mean: either return a fully verified plan within fixed budgets or fail the active run with a detailed error.

### 9. Reference implementations and licenses

| Project | Capability | License | Assessment |
|---|---|---|---|
| [toppra](https://github.com/hungpham2511/toppra) | TOPP-RA, Python with C++ components/bindings | MIT | Permissive and useful as a second-order baseline; no jerk state |
| [Ruckig](https://github.com/pantor/ruckig) | Jerk-limited PVA state-to-state trajectories | MIT | Already vendored; retain for local bridges and stop tails |
| [HiGHS](https://github.com/ERGO-Code/HiGHS) | Sparse LP/QP solver, basis warm starts, time limits | MIT | Best fit for an SLP/QP master in C++23 |
| [Clarabel.cpp](https://github.com/oxfordcontrol/Clarabel.cpp) | Conic optimization | Apache-2.0 | Technically suitable for SOCP; Rust-backed integration is heavier |
| [Ipopt](https://github.com/coin-or/Ipopt) | Sparse nonlinear programming | EPL-2.0 | Good direct-collocation research baseline; larger dependency and local solutions |
| [qpOASES](https://github.com/coin-or/qpOASES) | Online active-set QP | LGPL-2.1 | Warm-start/MPC oriented, but older and license obligations need review |
| [TOPP](https://github.com/quangounet/TOPP) | Older TOPP algorithms | LGPL-3.0 | Unmaintained and not a production TOPP3 library |
| [OpenCN](https://gitlab.com/mecatronyx/opencnc/opencn) | CNC receding-horizon feed planning | GPL-2.0-or-later | Valuable reference; code reuse depends on NGC's eventual project license |
| GPOPS-II | hp-adaptive collocation | Commercial MATLAB software | Research comparator, not a C++ dependency |

I did not find a root NGC license file, so definitive compatibility cannot be established from this repository alone. MIT dependencies are the lowest-risk choices. Apache, EPL, LGPL, and GPL dependencies require an explicit project licensing decision; this is not legal advice.

No maintained, reusable TOPP3 implementation comparable to `toppra` was found.

### 10. Which approach is most likely to reduce planning time?

A sparse, feasible SCP/SLP scheduler with:

- State `(w, a)`, where `w = v^2`.
- Exact PVA boundary conditions.
- Conservative certified cell constraints for initialization.
- Warm-started sparse LP or QP solves.
- A persistent active set.
- Exact-verifier-driven adaptive constraint generation.
- Local reoptimization between verified PVA seams.
- Existing Ruckig launch, bridge, and stop-tail machinery.
- Existing exact execution-span and packet gates.

This approach should reduce planning time more reliably than a pure TOPP3 rewrite because it preserves NGC's mixed constraints and existing discretized architecture while eliminating broad whole-horizon correction passes.

That performance conclusion is an inference to validate experimentally. Published SLP/SCA results show that sparse jerk-limited path problems can be solved in milliseconds to hundreds of milliseconds for hundreds of stations, but their constraint sets are simpler and their timing numbers are not directly transferable to NGC. For example, Consolini et al. report roughly 39 ms on a representative 100-point case and feasible suboptimal solutions within about 150 ms on larger cases; Lee et al. report millisecond-scale LP iterations for small grids. [Consolini et al.](https://air.unipr.it/retrieve/e177fbc7-8016-50b0-e053-d805fe0adaee/TASE3111758.pdf), [Lee et al.](https://arxiv.org/abs/2404.07889).

## Serious candidate comparison

### Candidate A: Feasible SCP/SLP with verifier-generated cuts

Primary foundations:

- [Consolini et al., feasible sequential convex approximation](https://air.unipr.it/retrieve/e177fbc7-8016-50b0-e053-d805fe0adaee/TASE3111758.pdf)
- [Lee et al., conservative sequential LP](https://arxiv.org/abs/2404.07889)
- [Hauser, semi-infinite constraint generation](https://journals.sagepub.com/doi/10.1177/0278364920983353)

State and constraints:

```text
w = v^2
state = (w, a)
control = j

dw/ds = 2*a
da/ds = j/sqrt(w), when w > 0
```

A trapezoidal spatial discretization can use:

```text
w[k+1] - w[k] = ds[k] * (a[k] + a[k+1])

j_mid approximately =
    sqrt(w_mid) * (a[k+1] - a[k]) / ds[k]
```

Axis constraints are evaluated from:

```text
axis_velocity     = q1 * sqrt(w)
axis_acceleration = q1 * a + q2 * w
axis_jerk         = q1 * j + 3*q2*sqrt(w)*a + q3*w*sqrt(w)
```

Assessment:

- Jerk: full formula can be represented, but requires sequential convexification or conservative bounds. Exact emitted-span jerk remains verifier-certified.
- Complexity: each master is a sparse LP or QP. Formal worst-case solver complexity is polynomial for each convex subproblem; iteration count is not generally bounded by theory.
- PVA boundaries: natural.
- Rolling horizon: strong, because prior active sets, bases, and solutions can be shifted and reused.
- Failure modes: poor linearization near `w = 0`, trust-region stagnation, active-set churn, infeasible fixed seams, and mismatch between station and emitted-span constraints.
- Integration difficulty: medium.
- Expected planning time: best chance of a substantial reduction.
- Expected trajectory duration: equal or shorter than the current cap-tightening method if the same constraints are active; possibly slightly longer if conservative envelopes dominate.

Near zero speed, the spatial dynamics are singular. Use a time-domain launch/landing primitive, an analytic first cell, or a verified Ruckig bridge rather than dividing by `sqrt(w)`.

### Candidate B: Conservative SLP or SOCP on a certified adaptive mesh

Primary references:

- [Lee et al.](https://arxiv.org/abs/2404.07889)
- [Consolini and Locatelli, SOCP](https://arxiv.org/abs/2310.07583)
- [Giannelli et al., jerk-constrained CNC feedrate scheduling](https://arxiv.org/abs/1711.08035)

Use squared speed and convex inner constraints. Bound geometry derivatives over each cell rather than only sampling them, then solve one or a small number of LP/SOCP problems.

Assessment:

- Jerk: conservative, unless the exact emitted polynomial constraint is later activated.
- Complexity: predictable convex solves; strong warm-start opportunities.
- Axis limits: straightforward component-wise bounds.
- Coupled norm constraints: SOC-representable after conservative separation of nonlinear products.
- PVA boundaries: velocity is easy; acceleration requires extending formulations that are often published for rest-to-rest motion.
- Rolling horizon: good.
- Failure modes: excessive conservatism, especially where jerk components cancel; certified geometry bounds may be expensive for splines and endpoint-exact arcs.
- Integration difficulty: medium-low.
- Expected planning time: likely much faster than the current repeated global correction.
- Expected trajectory duration: potentially longer, especially on high-curvature cluster splines.

This is a good feasibility seed and fallback initializer for Candidate A, but probably not the best final scheduler by itself.

### Candidate C: TOPP3-inspired continuous third-order reachability

Primary reference:

- [Pham and Pham, TOPP3](https://arxiv.org/pdf/1609.05307)

Assessment:

- State: `(s, v, a)`.
- Control: scalar jerk.
- Jerk: model-exact and continuous for represented constraints.
- Complexity: profile integrations are nominally linear in the number of integration steps, plus nonlinear multiple-shooting work. Runtime is sensitive to singularities and connection attempts.
- Published performance: the paper reports representative total times around 72 ms without a jerk bound, 181 ms for a looser jerk case, and 754 ms for a stricter case on its hardware.
- Axis limits: per-axis jerk maps directly; velocity and acceleration are lower-order constraints.
- Coupled jerk norm: computable as an admissible scalar jerk interval at each state, but not demonstrated by the paper.
- PVA boundaries: yes.
- Rolling horizon: possible, though difficult switching structures reduce predictability.
- Failure modes: singular curves, multiple shooting failure, mixed-order switch handling, and numerical profile sensitivity.
- Integration difficulty: high.
- Expected planning time: potentially excellent when the switching structure is simple; unpredictable otherwise.
- Expected trajectory duration: closest to time-optimal among practical direct third-order candidates.

TOPP3 should be built as a research comparator before being considered as the production scheduler.

### Candidate D: Execution-span direct collocation or SQP

Primary references:

- [Kelly, direct collocation tutorial](https://epubs.siam.org/doi/full/10.1137/16M1062569)
- [GPOPS-II hp-adaptive collocation](https://www.anilvrao.com/Publications/JournalPublications/GPOPS-II-TOMS-ACM.pdf)
- [Continuous-time successive convexification](https://arxiv.org/abs/2404.16826)

Optimize the execution-span durations and boundary states directly. Add the exact cubic-extrema constraints or conservative Bernstein constraints.

Assessment:

- Jerk: can be exact for a fixed cubic topology.
- Complexity: sparse nonlinear optimization; no reliable small upper bound on iterations.
- Axis and coupled constraints: fully expressible.
- PVA boundaries: fully supported.
- Rolling horizon: warm-startable.
- Failure modes: local minima, poor scaling, topology changes, inconsistent collocation and exact verification, and solver timeouts.
- Integration difficulty: high.
- Expected planning time: likely slower than sparse SLP for long horizons, though fewer outer verifier passes may compensate.
- Expected trajectory duration: potentially best locally optimized duration.

This is a valuable offline reference solver and a way to measure how much trajectory duration the faster schedulers leave on the table.

### Candidate E: McCormick relaxation with spatial branch-and-bound

Primary reference:

- [Palleschi et al.](https://arpi.unipi.it/bitstream/11568/1003896/5/opt_path_jcr.pdf)

Assessment:

- Jerk: full axis expression at discretization nodes.
- PVA boundaries: yes.
- Guarantees: global lower bounds and convergence to a chosen tolerance if branch-and-bound completes.
- Complexity: exponential worst case.
- Runtime: published examples vary from about a second to thousands of seconds.
- Rolling horizon: poor.
- Integration difficulty: very high.
- Expected trajectory duration: globally strong for the discretized model.
- Production value: unsuitable as the main NGC planner, but useful as a small-problem oracle.

## CNC-specific literature

CNC feedrate schedulers commonly use B-splines, windowing, or linearized pseudo-jerk constraints to obtain predictable runtime:

- Sencer et al. optimize a cubic B-spline feed profile under drive velocity, acceleration, and jerk constraints for five-axis CNC. [Sencer et al.](https://research.engr.oregonstate.edu/mpcl/resources/Feed-optimization-for-%EF%AC%81ve-axis-CNC-machine-tools-with-drive-constraints.pdf).
- Erkorkmaz et al. use windowing and LP-based pseudo-jerk scheduling, exploiting local structure for parallelism and approximately linear path-length scaling. [Erkorkmaz et al.](https://www.sciencedirect.com/science/article/abs/pii/S0007850617300586).
- Giannelli et al. construct a C2 piecewise-quintic feedrate and enforce conservative Cartesian velocity, acceleration, and jerk conditions. [Giannelli et al.](https://arxiv.org/abs/1711.08035).
- OpenCN uses B-spline squared-speed optimization and a receding-horizon architecture. [OpenCN documentation](https://mecatronyx.gitlab.io/opencnc/opencn/CNC_Path_Planning_Algorithms/Feedrate_Planning/Feedrate_Planning.html).

These support NGC's bounded-lookahead direction, but their published constraint treatment is generally sampled, conservative, pseudo-jerk based, or dependent on a specific spline feed representation. None replaces NGC's final exact axis-polynomial proof.

## Ranked shortlist

1. Feasible sparse SCP/SLP with persistent verifier-generated cuts and local active-set repair.
2. Conservative certified-mesh SLP/SOCP as initializer and guaranteed-feasibility restoration.
3. TOPP3-inspired direct third-order reachability as a research comparator and possible future fast path.
4. Execution-span direct collocation/SQP as an offline quality oracle.
5. Reachability-augmented DDP as a watchlist item pending independent validation and reusable code.

## Recommended NGC architecture

Keep geometry, execution span emission, packetization, stop tails, and exact verification as separate stages.

```text
Prepared geometry
    -> derivative-bound and execution-aware mesh
    -> conservative feasible seed
    -> sparse third-order SCP/SLP master
       state: squared speed and path acceleration
       control: path jerk
       fixed PVA boundaries
       persistent active constraints
    -> current C2 cubic execution-span emitter
    -> current exact polynomial verifier
       success:
           verify stop tails
           packetize
       failure:
           generate local constraint/cut
           refine local mesh or fixed span topology
           reoptimize affected PVA-bounded band
```

The optimizer should never own publication authority. The exact verifier and stop-branch gate continue to decide whether a plan is executable.

Use HiGHS initially for the LP/QP master. It is permissively licensed, supports warm starts and explicit time limits, and avoids introducing a general nonlinear optimizer into the production path.

## Pseudocode

```text
function schedule(chain, initial_pva, final_pva, budgets):
    mesh = build_geometry_and_execution_aware_mesh(chain)

    bounds = certify_cell_bounds(
        q1, q2, q3,
        programmed_feed,
        path_limits,
        axis_limits)

    incumbent = construct_fixed_boundary_feasible_seed(
        mesh,
        bounds,
        initial_pva,
        final_pva,
        existing_ruckig_bridges)

    if incumbent is not exactly verified:
        incumbent = restore_feasibility_with_conservative_constraints()

    active_constraints = initial_conservative_constraints(bounds)
    trust_region = initial_trust_region
    affected_band = whole_chain

    for outer_iteration in 0 .. max_outer_iterations:
        master = build_sparse_convex_master(
            mesh,
            affected_band,
            fixed_pva_at_band_boundaries,
            incumbent,
            active_constraints,
            trust_region)

        candidate = warm_started_solve(
            master,
            max_solver_iterations,
            solver_time_budget)

        if master is infeasible:
            if affected_band can expand:
                affected_band = expand_to_next_verified_pva_seam()
                continue
            return fatal_bounded_planning_error()

        candidate = feasible_line_search(
            incumbent,
            candidate,
            active_constraints)

        spans = emit_current_c2_axis_cubics(candidate)

        report = exact_verify_execution_spans(spans)
        stop_report = exact_verify_all_required_stop_tails(spans)

        if report.ok and stop_report.ok:
            return packetize_verified_spans(spans)

        counterexamples = collect_all_violations_and_near_active_extrema(
            report,
            stop_report)

        for counterexample in counterexamples:
            active_constraints.add_persistent(
                derive_local_cut_or_conservative_bound(counterexample))

            mesh.refine_at(counterexample.exact_span_time)

        affected_band = connected_influence_band(
            counterexamples,
            station_dependency_graph,
            guard_spans)

        trust_region = reduce_if_model_was_inaccurate(
            trust_region,
            predicted_change,
            measured_exact_change)

        if candidate is exactly verified:
            incumbent = candidate

    return fatal_bounded_planning_error()
```

A cut should not be retained across an incompatible execution-span topology change. Either keep the topology fixed for the relevant optimization epoch or regenerate the cut from a topology-independent certified interval bound.

## Migration plan

1. Add diagnostics only.

   Capture exact extremum locations, all near-active constraints, solver-stage timings, and station-to-span dependencies. Preserve current behavior.

2. Build an offline scheduling harness.

   Run the current compiler, conservative SLP, TOPP3 prototype, and direct-collocation oracle against identical immutable prepared geometry. Do not route any experimental plan to packet publication.

3. Add a conservative feasible seed.

   Replace or supplement sampled initial scalar caps with certified cell derivative bounds. Feed the result through the existing emitter and exact verifier.

4. Add a sparse SLP/QP scheduler behind an explicit configuration option.

   Keep current execution emission, exact extrema verification, capacity checks, packetization, and stop tails unchanged. Exhaustion in the selected experimental mode remains fatal; do not silently fall back.

5. Add persistent constraint generation.

   Initially regenerate only conservative local bounds at verifier counterexamples. Later add sensitivity-based cuts where their validity can be proved within a trust region.

6. Add affected-band reoptimization.

   Freeze verified regions only at exact PVA seams. Reuse the existing stable-prefix/suffix and replay diagnostics to validate that no hidden propagation occurs.

7. Optimize exact execution spans selectively.

   Start with exact constant jerk and affine acceleration constraints. Add quadratic velocity-extremum constraints after the topology and sensitivity mapping are stable.

8. Make a default-planner decision only after adversarial testing.

   Retain the old strict scheduler as an explicit comparison mode during the transition, not as a silent runtime downgrade.

## Focused experiments

Use both ordinary and deliberately adversarial cases:

- The representative horizon currently requiring about ten correction passes.
- A horizon that takes about 25 seconds strict and about 3 seconds with enforcement disabled.
- Long retained lines with local high-curvature junction blends.
- Cluster splines with large full geometric jerk coefficient but modest normal sharpness.
- Alternating curvature or derivative signs that make the worst jerk ratio move between pieces.
- Axis derivative components passing through zero, exercising TOPP3 singular behavior.
- Helical and non-XY endpoint-exact arcs.
- Programmed-feed changes at cluster knot intervals.
- Nonzero rolling PVA boundaries at every supported retained-line anchor fraction.
- Near-zero launch and landing speeds.
- Short pieces and all-short clusters.
- Six-axis XYZABC cases in which aggregate path jerk and different per-axis jerks become active at different points.
- Packet-boundary stress near 256 normal spans and 16 stop-tail spans.
- Pieces crossing packet boundaries.
- Stop-tail selection at every eligible packet.
- Randomized and adversarial horizons with thousands of prepared pieces.
- Cases in which a local correction is known to invalidate a previously valid neighbor.

## Benchmark metrics

Record:

- Total wall time: median, p95, p99, and maximum.
- Per-stage time: geometry bounds, master construction, solver, Ruckig calls, emission, exact verification, and stop-tail verification.
- Number of outer iterations, solver iterations, cuts, retained cuts, and mesh refinements.
- Number of stations and execution spans.
- Maximum exact velocity, acceleration, and jerk ratios.
- Minimum constraint margin among accepted spans.
- Planned trajectory duration versus the current strict scheduler.
- Prefix/suffix reuse and number of stations changed per correction.
- Distance that a correction propagates from its originating counterexample.
- Number of previously valid spans made invalid by a candidate update.
- Packet and stop-tail capacity high-water marks.
- Solver residuals, conditioning indicators, trust-region radius, and line-search reductions.
- Deterministic replay equality.
- Resource exhaustion, infeasibility, and timeout counts.
- Stop-tail duration and constraint slack at every branch.
- Feasible-seed conservatism and any feasibility-restoration time.

A reasonable initial experimental target—not a published guarantee—is:

- Every published plan passes the existing verifier.
- At least a 5x p95 speedup on the representative strict workload.
- No more than 1–2% median trajectory-duration regression.
- No more than 5% worst-case duration regression on the validation corpus.
- No increase in bounded-capacity or stop-tail failures.

## Missing diagnostic data

NGC already records useful pass-locality, replay, propagation, and resource high-water information. Before choosing an algorithm, add:

- Exact normalized time of every violating or near-active polynomial extremum.
- Constraint kind, axis, value, limit, margin, and polynomial coefficients.
- All violations per span, not only the worst ratio per piece.
- Stable mapping from execution span to prepared piece, source interval, and optimization stations.
- Dependency graph showing which station variables affect which emitted spans.
- Per-pass active-set membership and persistent cut identity.
- Predicted versus measured constraint change for each correction.
- Solver primal/dual residuals, objective, slacks, and dual multipliers.
- Trust-region and feasible-line-search histories.
- Certified per-cell maxima of `q1`, `q2`, and `q3`, plus the gap from the current 65 samples.
- Timing breakdown for every correction pass.
- Which correction first invalidated each previously valid span.
- Exact PVA and controllability margin at every frozen or rolling seam.
- Constraint slack and capacity margin for every stop tail.
- Near-zero-speed singularity counts and the mechanism used to handle them.
- The feasible seed's time penalty relative to the final verified result.

The decisive experiment is whether persistent local constraints make the number and geographic extent of changed spans shrink monotonically. If that happens, the active-set architecture is addressing the observed problem. If the active region repeatedly expands across most of the horizon, the limiting issue is global third-order coupling, and a TOPP3-style reachability prototype becomes more attractive.
