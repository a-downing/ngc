# Initial spline curvature-derivative velocity cap

This document describes only the planner's **initial maximum-velocity check based on a spline's curvature derivative**. It does not describe acceleration-aware reachability, Ruckig timing, or final emitted-polynomial verification.

The implementation is in [`src/ExactStopTrajectoryPlanner.cpp`](src/ExactStopTrajectoryPlanner.cpp), primarily in `CubicBSplineReference::curvatureDerivativeAtDistance()` and the `LocalLimits` sampling loop inside `ExactStopTrajectoryPlanner::compileContinuous()`.

## What the check is limiting

Let the spline be expressed by arc length (s):

\[
\mathbf q(s) = [X(s),Y(s),Z(s),A(s),B(s),C(s)].
\]

Define:

\[
\mathbf T(s)=\mathbf q'(s),
\qquad
\boldsymbol\kappa(s)=\mathbf q''(s),
\qquad
\boldsymbol\kappa_s(s)=\mathbf q'''(s)
    =\frac{d\boldsymbol\kappa}{ds}.
\]

Here `curvatureDerivativeAtDistance()` returns the vector

\[
\boldsymbol\kappa_s=\frac{d\boldsymbol\kappa}{ds},
\]

not merely the derivative of the scalar curvature magnitude.

If the tool moves along the path with scalar path speed

\[
v=\dot s,
\]

then its axis-space jerk is

\[
\frac{d^3\mathbf q}{dt^3}
=
\mathbf T\,j
+3\boldsymbol\kappa\,v a
+\boldsymbol\kappa_s\,v^3,
\]

where

\[
a=\ddot s,
\qquad
j=\dddot s.
\]

The initial curvature-derivative check isolates the last term:

\[
\mathbf J_{\text{curve-change}}=\boldsymbol\kappa_s v^3.
\]

It asks: **at this location on the spline, how fast could the tool move before this geometric jerk term alone reaches a configured jerk limit?**

Because this term grows with the cube of speed, the resulting speed cap uses a cube root.

## 1. Evaluate the cubic B-spline and its parameter derivatives

The planner first represents the spline using its ordinary B-spline parameter (u):

\[
\mathbf r(u).
\]

For a normal six-control junction spline, this is a clamped degree-three B-spline with knot vector

```text
[0, 0, 0, 0, 1, 2, 3, 3, 3, 3]
```

and parameter range (0\le u\le3). Longer spline clusters use the same clamped uniform cubic construction with more controls and knot spans.

The planner constructs derivative B-splines analytically from the control points. It then evaluates, using de Boor's algorithm:

\[
\mathbf r_u=\frac{d\mathbf r}{du},
\qquad
\mathbf r_{uu}=\frac{d^2\mathbf r}{du^2},
\qquad
\mathbf r_{uuu}=\frac{d^3\mathbf r}{du^3}.
\]

Therefore, the third derivative used by this calculation is **not estimated by subtracting nearby sampled positions**. Within each cubic knot span, \(\mathbf r_{uuu}\) is the exact constant third parameter derivative of that cubic polynomial span. The curvature derivative is not constant, however, because converting from (u) to arc length introduces powers of \(\lVert\mathbf r_u\rVert\) and dot products involving all three derivatives.

## 2. Convert distance along the spline into parameter (u)

Velocity limits must be sampled at evenly spaced path distances, not evenly spaced values of (u). The two are generally different because a B-spline does not move through parameter space at constant geometric speed.

The differential arc length is

\[
\frac{ds}{du}=\lVert\mathbf r_u\rVert.
\]

The distance from the beginning of the spline to parameter (u) is

\[
s(u)=\int_0^u \lVert\mathbf r_u(\xi)\rVert\,d\xi.
\]

The planner evaluates this integral with adaptive Simpson integration. It builds a distance lookup containing 32 integration intervals per B-spline knot span. For a requested distance (s), it:

1. Finds the lookup interval containing (s).
2. Linearly interpolates an initial parameter estimate inside that interval.
3. Performs up to 12 safeguarded Newton iterations.
4. Uses

   \[
   u_{n+1}=u_n-\frac{s(u_n)-s_{\text{requested}}}
                         {\lVert\mathbf r_u(u_n)\rVert}
   \]

   when that Newton step remains inside the bracket.
5. Falls back to the bracket midpoint when the Newton step would leave the bracket or the local parameter speed is too small.

This produces the parameter value used to evaluate the spline derivatives at a particular arc-length distance.

## 3. Calculate the curvature vector

At the recovered parameter (u), define

\[
\mathbf r_1=\mathbf r_u,
\qquad
\mathbf r_2=\mathbf r_{uu},
\qquad
w=\lVert\mathbf r_1\rVert,
\qquad
\mathbf T=\frac{\mathbf r_1}{w}.
\]

Because (d/ds=(1/w)d/du), the spline's curvature vector with respect to arc length is

\[
\boldsymbol\kappa
=\frac{d\mathbf T}{ds}
=\frac{\mathbf r_2-\mathbf T(\mathbf r_2\cdot\mathbf T)}{w^2}.
\]

An algebraically equivalent form is

\[
\boldsymbol\kappa
=\frac{\mathbf r_2}{w^2}
-\frac{\mathbf r_1(\mathbf r_1\cdot\mathbf r_2)}{w^4}.
\]

The vector points toward the local center of curvature. Its magnitude is the familiar scalar curvature:

\[
\kappa=\lVert\boldsymbol\kappa\rVert.
\]

## 4. Calculate the curvature-derivative vector

The required quantity is

\[
\boldsymbol\kappa_s
=\frac{d\boldsymbol\kappa}{ds}.
\]

The code calculates this analytically from the first three parameter derivatives. Define:

\[
\mathbf r_3=\mathbf r_{uuu},
\qquad
h=\mathbf r_1\cdot\mathbf r_2.
\]

It first differentiates the curvature vector with respect to (u):

\[
\frac{d\boldsymbol\kappa}{du}
=
\frac{\mathbf r_3}{w^2}
-\frac{3\mathbf r_2 h}{w^4}
-\frac{\mathbf r_1
        (\mathbf r_2\cdot\mathbf r_2+\mathbf r_1\cdot\mathbf r_3)}{w^4}
+\frac{4\mathbf r_1 h^2}{w^6}.
\]

It then converts the derivative from parameter distance to arc length:

\[
\boxed{
\boldsymbol\kappa_s
=\frac{1}{w}\frac{d\boldsymbol\kappa}{du}
}
\]

or, fully expanded:

\[
\boxed{
\boldsymbol\kappa_s
=
\frac{\mathbf r_3}{w^3}
-\frac{3\mathbf r_2 h}{w^5}
-\frac{\mathbf r_1
        (\mathbf r_2\cdot\mathbf r_2+\mathbf r_1\cdot\mathbf r_3)}{w^5}
+\frac{4\mathbf r_1 h^2}{w^7}
}
\]

For a planar curve, the same vector can be understood through the Frenet frame:

\[
\boldsymbol\kappa_s=-\kappa^2\mathbf T+\kappa'\mathbf N.
\]

This is an important detail: even a perfect constant-radius circle has \(\kappa'=0\), but it still has

\[
\boldsymbol\kappa_s=-\kappa^2\mathbf T\ne0
\]

because the curvature vector continuously rotates as the tool moves around the circle. Thus, `curvatureDerivativeAtDistance()` is not expected to return zero on a constant-curvature circle.

If (w\le10^{-15}), the implementation returns a zero vector to avoid division by an effectively zero parameter speed. A properly conditioned spline should not approach that case.

## 5. Sample the spline piece at 65 distances

The initial limit is calculated independently for every planner `GeometryPiece`, not necessarily once for an entire source spline.

- A normal six-control junction spline has three knot spans and is currently one geometry/timing piece.
- A longer cluster spline is divided into pieces containing at most three B-spline knot spans.
- A feedrate transition can introduce an additional piece boundary.

For each resulting piece of length (L), the code evaluates 65 evenly spaced arc-length positions:

\[
s_i=L\frac{i}{64},
\qquad i=0,1,\ldots,64.
\]

Both endpoints are included. At every (s_i), the planner evaluates

\[
\boldsymbol\kappa_s(s_i).
\]

The worst speed candidate found at any of the 65 samples becomes the curvature-derivative velocity cap for that whole piece. Consequently, one sharp value near one part of a three-knot-span piece can limit the initial speed cap across the entire piece.

This is a fixed-resolution preliminary measurement. It does not analytically solve for the exact maximum of \(\lVert\boldsymbol\kappa_s(s)\rVert\) between samples.

## 6. Convert curvature derivative into path and per-axis speed caps

At one sample, suppose the curvature-derivative vector is

\[
\boldsymbol\kappa_s=
[K_X,K_Y,K_Z,K_A,K_B,K_C].
\]

### Aggregate path-jerk cap

The magnitude of the isolated geometric jerk term is

\[
\lVert\mathbf J_{\text{curve-change}}\rVert
=\lVert\boldsymbol\kappa_s\rVert v^3.
\]

Requiring it not to exceed the configured aggregate path jerk (J_{\text{path}}) gives

\[
\lVert\boldsymbol\kappa_s\rVert v^3\le J_{\text{path}},
\]

so the sample's aggregate speed candidate is

\[
\boxed{
v_{\text{path},i}
=\sqrt[3]{\frac{J_{\text{path}}}
                 {\lVert\boldsymbol\kappa_s(s_i)\rVert}}
}.
\]

The check is skipped when

\[
\lVert\boldsymbol\kappa_s\rVert\le10^{-15}.
\]

### Per-axis jerk caps

For each axis (d\in\{X,Y,Z,A,B,C\}), the isolated geometric jerk component is

\[
J_d=K_dv^3.
\]

Requiring its magnitude not to exceed that axis's configured jerk limit (J_{d,\max}) gives

\[
|K_d|v^3\le J_{d,\max},
\]

and therefore

\[
\boxed{
v_{d,i}
=\sqrt[3]{\frac{J_{d,\max}}{|K_d|}}
}.
\]

An axis candidate is skipped when

\[
|K_d|\le10^{-15}.
\]

### Optional experimental multiplier

The implementation multiplies these candidates by

```cpp
curvatureDerivativeVelocityCapMultiplier
```

whose production default is (M=1.0):

\[
v_{\text{candidate}}=M
\sqrt[3]{\frac{J_{\max}}{K}}.
\]

The multiplier changes the **preliminary speed cap**, not the configured physical jerk limit. Because this jerk term varies as (v^3), multiplying the candidate speed by (M) permits (M^3) times as much isolated curvature-derivative jerk at this preliminary check. For example:

\[
1.5^3=3.375.
\]

Thus, a 1.5 multiplier is much more aggressive than a 50% increase in this isolated jerk allowance. Later checks still use the original configured jerk limits.

## 7. Select the initial cap for the piece

Before spline sampling, the piece's velocity limit begins at its programmed feedrate in machine units per second. For an ordinary junction spline, that feed is the arithmetic mean of the two neighboring entity feeds, converted from units per minute by dividing by 60.

During all 65 samples, the planner also computes other preliminary candidates, such as axis velocity and centripetal-acceleration caps. The final initial local velocity limit is simply the minimum of every applicable candidate:

\[
\boxed{
v_{\text{initial,piece}}
=\min\left(
v_{\text{programmed}},
v_{\text{axis velocity candidates}},
v_{\text{curvature acceleration candidates}},
v_{\text{curvature-derivative jerk candidates}}
\right).
}
\]

Considering only the curvature-derivative part requested in this document, that contribution is

\[
v_{\kappa_s,\text{piece}}
=
\min_{i=0}^{64}
\left[
M\sqrt[3]{\frac{J_{\text{path}}}
{\lVert\boldsymbol\kappa_s(s_i)\rVert}},
\quad
\min_d M\sqrt[3]{\frac{J_{d,\max}}
{|K_d(s_i)|}}
\right].
\]

If this value is the smallest candidate, the diagnostic cause is recorded as either:

- `PathCurvatureDerivativeJerk`, or
- `AxisCurvatureDerivativeJerk`.

## Compact pseudocode

```text
piece_limit = programmed_feed / 60

for i = 0 through 64:
    s = piece_length * i / 64
    u = invert_arc_length(s)

    r1 = spline_first_parameter_derivative(u)
    r2 = spline_second_parameter_derivative(u)
    r3 = spline_third_parameter_derivative(u)

    w = norm(r1)
    if w <= 1e-15:
        curvature_derivative = zero
    else:
        h = dot(r1, r2)

        d_curvature_du =
              r3 / w^2
            - 3 * r2 * h / w^4
            - r1 * (dot(r2,r2) + dot(r1,r3)) / w^4
            + 4 * r1 * h^2 / w^6

        curvature_derivative = d_curvature_du / w

    K = norm(curvature_derivative)
    if K > 1e-15:
        piece_limit = min(piece_limit,
                          multiplier * cbrt(path_jerk / K))

    for axis in X,Y,Z,A,B,C:
        K_axis = abs(curvature_derivative[axis])
        if K_axis > 1e-15:
            piece_limit = min(piece_limit,
                              multiplier * cbrt(axis_jerk[axis] / K_axis))
```

## What this check does and does not mean

This initial check is deliberately simple:

- It measures the actual analytic cubic B-spline derivatives, rather than estimating the third derivative from position samples.
- It samples by physical arc-length distance, rather than by raw spline parameter.
- It includes both aggregate and individual-axis jerk limits.
- It uses the worst of 65 samples for each geometry piece.

However:

- It is not an exact continuous maximization of curvature derivative between sample locations.
- It isolates only the \(\boldsymbol\kappa_s v^3\) term of the full jerk equation.
- The other jerk terms, \(\mathbf Tj\) and (3\boldsymbol\kappa va\), can reinforce or partially oppose it during actual motion.
- It produces a local candidate speed ceiling, not a command to maintain that speed throughout the spline.

The full planner deals with those interactions later, but those later stages are intentionally outside the scope of this document.
