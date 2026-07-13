# CNC Trajectory Planning Architecture

## Consolidated design conclusions

This document summarizes the proposed architecture for a CNC controller in which:

- trajectory planning runs in non-real-time code (NRT);
- the real-time code (RT) is kept as small and deterministic as possible;
- `G64` motion is converted into a tolerance-bounded cubic B-spline path;
- `G61` and `G61.1` retain their exact line, arc, and optional helix geometry;
- velocity, acceleration, and jerk are limited;
- position, velocity, and acceleration remain continuous;
- jerk is allowed to change discontinuously at spline-span boundaries, provided the one-sided jerk values remain within limits;
- NRT starvation always results in a controlled, dynamically feasible stop.

---

# 1. Main architectural decision

The most important invariant is:

> At every instant, RT must already possess a complete, dynamically feasible trajectory to rest.

A conventional FIFO containing only normal-motion commands is not sufficient. If NRT stops producing commands, the final queued command may end at nonzero velocity, leaving RT with no legal continuation.

Instead, each committed motion chunk contains:

1. a normal-motion portion;
2. a branch or decision point;
3. a precomputed stop tail beginning exactly at that decision point;
4. a terminal state at rest.

NRT may publish a continuation before RT reaches the branch point. At the branch point, RT makes one irrevocable decision:

```text
if a valid continuation is ready:
    execute the continuation
else:
    execute the existing stop tail
```

Once RT selects the stop branch, it never switches back to the late continuation.

This keeps the starvation behavior deterministic and allows RT to remain very small.

---

# 2. Overall processing pipeline

A practical planning pipeline is:

```text
G-code parser and modal state
        |
        v
Primitive path representation
(lines, arcs, helices, synchronized moves, events)
        |
        +-----------------------------+
        |                             |
        | G61 / G61.1                 | G64
        |                             |
        v                             v
Retain exact primitives       Lookahead spline fitting
                              within G64 tolerance
                                    |
                                    v
                          Cubic C2 B-spline geometry
                                    |
                                    v
                         Kinematics and axis analysis
                                    |
                                    v
                  Velocity/acceleration/jerk-constrained
                           time parameterization
                                    |
                                    v
                    RT-friendly trajectory compilation
                                    |
                                    v
                 Normal chunks with guaranteed stop tails
                                    |
                                    v
                              RT execution
```

The canonical geometric representation does not have to be the same as the final RT representation.

For `G64`, the geometric planner may work with a cubic B-spline. NRT can then compile the timed path into whatever fixed polynomial format is simplest for RT.

---

# 3. Recommended modal behavior

## 3.1 G64

For `G64`, collect a finite lookahead window of lines, arcs, helices, and other eligible geometric primitives, then fit a cubic B-spline that:

- follows the programmed path in order;
- stays within the configured path-deviation tolerance;
- rounds eligible corners;
- remains `C2` through ordinary internal knots;
- preserves protected events and non-blendable boundaries;
- is validated before it is committed.

The spline fitting performs the geometric blending.

A 90-degree line-to-line junction will naturally become rounded, but the result must not be accepted merely because it visually appears smooth. It must be constrained and verified.

## 3.2 G61 and G61.1

For `G61` and `G61.1`, retain exact line, arc, and optional helix primitives and implement their respective exact-path or exact-stop semantics.

The downstream RT interface may still be unified by compiling those primitives into the same RT polynomial-command format used for `G64`.

Thus, RT does not need separate line, arc, helix, and spline evaluators unless retaining those evaluators is advantageous.

---

# 4. Why cubic B-splines are the recommended G64 geometry

A degree-3 B-spline with simple interior knots is `C2`.

That means the geometric curve has continuous:

- position;
- first derivative;
- second derivative.

After a correctly constructed time parameterization, the commanded motion can have continuous:

- position;
- velocity;
- acceleration.

The third derivative of the geometric curve may jump at a knot. Therefore physical jerk may also jump at that knot while remaining finite on both sides.

That matches the selected requirement:

> Jerk must be limited, but jerk itself does not have to be continuous.

A cubic spline is easier to implement than a quintic spline because it has:

- four Bézier control points per span rather than six;
- fewer coefficients;
- simpler fitting matrices;
- simpler derivative calculations;
- simpler knot insertion and Bézier extraction;
- especially simple third derivatives;
- broad algorithmic and library support.

A quintic spline would be justified if continuous jerk or smoother higher-order behavior later becomes necessary, but it is not required for the current design goal.

---

# 5. What “jerk-limited but discontinuous” means

Let the geometric path be

\[
\mathbf r(u)
\]

and let the scalar path parameter be driven by a time law

\[
u=u(t).
\]

Then:

\[
\dot{\mathbf r}
=
\mathbf r'(u)\dot u
\]

\[
\ddot{\mathbf r}
=
\mathbf r''(u)\dot u^2
+
\mathbf r'(u)\ddot u
\]

\[
\dddot{\mathbf r}
=
\mathbf r'''(u)\dot u^3
+
3\mathbf r''(u)\dot u\ddot u
+
\mathbf r'(u)\dddot u.
\]

For a cubic B-spline at a simple knot:

- \(\mathbf r'\) is continuous;
- \(\mathbf r''\) is continuous;
- \(\mathbf r'''\) may have different left- and right-hand values.

Therefore the physical jerk can step at the knot.

The planner should enforce:

\[
|j_i(t_k^-)| \le J_{i,\max}
\]

and

\[
|j_i(t_k^+)| \le J_{i,\max}
\]

for every axis \(i\) and every knot \(t_k\).

The jump itself does not violate the selected jerk-magnitude requirement, although it creates a snap impulse in the mathematical idealization. If testing shows that those jerk steps excite structural resonances or damage surface finish, the geometry or time law can later be upgraded.

---

# 6. Cubic Bézier representation of B-spline spans

A cubic B-spline can be converted into cubic Bézier spans by knot insertion. Knot insertion changes the representation but does not change the curve.

Each span can be stored as:

```cpp
struct CubicBezier {
    AxisVector p0;
    AxisVector p1;
    AxisVector p2;
    AxisVector p3;
};
```

For normalized parameter \(u \in [0,1]\),

\[
Q(u)
=
(1-u)^3P_0
+
3(1-u)^2uP_1
+
3(1-u)u^2P_2
+
u^3P_3.
\]

The equivalent power-basis representation is:

\[
Q(u)=au^3+bu^2+cu+d
\]

with:

\[
d=P_0
\]

\[
c=3(P_1-P_0)
\]

\[
b=3(P_2-2P_1+P_0)
\]

\[
a=P_3-3P_2+3P_1-P_0.
\]

RT can evaluate the power basis efficiently using Horner form:

```cpp
q = ((a * u + b) * u + c) * u + d;
```

---

# 7. Derivative bounds for a cubic Bézier span

For a cubic Bézier span:

\[
Q'(u)
=
3\left[
(1-u)^2(P_1-P_0)
+
2(1-u)u(P_2-P_1)
+
u^2(P_3-P_2)
\right]
\]

\[
Q''(u)
=
6\left[
(1-u)(P_2-2P_1+P_0)
+
u(P_3-2P_2+P_1)
\right]
\]

\[
Q'''(u)
=
6(P_3-3P_2+3P_1-P_0).
\]

The third derivative is constant over the span.

Useful conservative bounds are:

\[
\max |Q'(u)|
\le
3\max\left(
|P_1-P_0|,
|P_2-P_1|,
|P_3-P_2|
\right)
\]

\[
\max |Q''(u)|
\le
6\max\left(
|P_2-2P_1+P_0|,
|P_3-2P_2+P_1|
\right)
\]

\[
|Q'''(u)|
=
6|P_3-3P_2+3P_1-P_0|.
\]

These convex-hull bounds are cheap and deterministic. NRT may use exact extrema where worthwhile, but conservative bounds are often preferable during early implementation.

---

# 8. Duration and derivative limiting

A duration does not contain velocity, acceleration, or jerk limits by itself.

Duration is sufficient only when it time-scales a known normalized polynomial whose derivatives NRT has already analyzed.

Let:

\[
q(t)=Q(u),
\qquad
u=\frac{t}{T},
\qquad
0\le t\le T.
\]

Then:

\[
\dot q(t)=\frac{1}{T}Q'(u)
\]

\[
\ddot q(t)=\frac{1}{T^2}Q''(u)
\]

\[
\dddot q(t)=\frac{1}{T^3}Q'''(u).
\]

If:

\[
V_u=\max |Q'(u)|
\]

\[
A_u=\max |Q''(u)|
\]

\[
J_u=\max |Q'''(u)|,
\]

then a sufficient duration is:

\[
T
\ge
\max\left(
\frac{V_u}{V_{\max}},
\sqrt{\frac{A_u}{A_{\max}}},
\sqrt[3]{\frac{J_u}{J_{\max}}}
\right).
\]

For multiple axes, take the maximum over all axis requirements.

However, independently selecting a minimum duration for every geometric span will generally destroy physical-time continuity.

For adjacent normalized spans \(Q_1\) and \(Q_2\), physical velocity continuity requires:

\[
\frac{Q_1'(1)}{T_1}
=
\frac{Q_2'(0)}{T_2}.
\]

Physical acceleration continuity requires:

\[
\frac{Q_1''(1)}{T_1^2}
=
\frac{Q_2''(0)}{T_2^2}.
\]

Therefore:

> Timing must be solved across the connected trajectory, not independently per Bézier span.

---

# 9. Geometry fitting and time parameterization are separate

The G64 fitter should produce a geometric curve:

\[
\mathbf x=\mathbf r(s).
\]

The trajectory planner should then determine:

\[
s=s(t).
\]

The final machine-axis trajectory is:

\[
\mathbf q(t)
=
\operatorname{IK}\left(\mathbf r(s(t))\right).
\]

These stages should remain conceptually separate:

## Geometric fitting

Responsible for:

- G64 path-deviation tolerance;
- rounding corners;
- preserving source-path order;
- continuity of the geometric curve;
- avoiding invalid shortcuts;
- handling protected boundaries.

## Time parameterization

Responsible for:

- axis velocity limits;
- axis acceleration limits;
- axis jerk limits;
- feed-rate requests;
- stop capability;
- synchronized completion;
- kinematic singularities and rotary-axis effects.

A geometrically smooth curve may still require very low feed because of curvature or machine kinematics.

---

# 10. G64 spline fitting must be constrained

A generic least-squares fit is not sufficient.

Ordinary least-squares fitting minimizes a quantity such as:

\[
\sum_i
\|\mathbf r(u_i)-\mathbf p_i\|^2.
\]

It does not guarantee:

\[
\max d(\mathbf r,\mathcal P)\le P,
\]

where \(P\) is the G64 path-deviation tolerance and \(\mathcal P\) is the programmed path.

A valid G64 spline fit should satisfy the following hierarchy.

## Hard constraints

1. Never exceed the allowed geometric deviation.
2. Preserve traversal order along the programmed path.
3. Respect protected events and non-blendable boundaries.
4. Meet the required geometric continuity.
5. Join committed geometry without discontinuity.

## Preferences

1. Preserve straight and circular regions away from corners.
2. Minimize unnecessary geometric deviation.
3. Prefer lower curvature.
4. Prefer lower curvature variation.
5. Prefer geometry that permits higher dynamically feasible feed.

The implementation does not require a single global nonlinear optimization. An adaptive local fitting procedure can enforce the same policy.

---

# 11. Path-order preservation

Closest distance to the union of all source geometry is not enough.

If the source path passes close to itself, a spline point could be near the wrong portion of the toolpath and still have a small global distance.

Each spline parameter interval should be associated with a specific ordered source interval:

\[
u\in[u_a,u_b]
\longleftrightarrow
s\in[s_a,s_b].
\]

Deviation should be measured against the corresponding local source interval, not against unrelated nearby geometry.

The mapping should be monotonic so the spline cannot:

- reverse along the source path;
- skip a programmed section;
- jump to a neighboring pass;
- shortcut through a self-near region.

---

# 12. Adaptive fitting procedure

A practical fitting process for a lookahead window is:

1. Preserve the exact input primitives as the reference path.
2. Sample or parameterize them by cumulative path length.
3. Choose an initial cubic B-spline knot vector and control-point count.
4. Fit with endpoint and continuity constraints.
5. Associate each spline interval with its ordered source interval.
6. Verify the full spline against the source geometry.
7. Check curvature and dynamic implications.
8. Insert knots or subdivide the source region where verification fails.
9. Refit.
10. Commit only the portion that is sufficiently far behind the editable lookahead frontier.

The source primitives should not be discarded until the fit has been validated.

---

# 13. Fitting-window boundaries and streaming

Do not independently fit disjoint windows and join them only by position. That could create tangent or acceleration discontinuities.

A streaming implementation should keep an editable overlap region:

```text
committed geometry | editable overlap | new source geometry
```

When extending the path:

- committed control points are frozen;
- several trailing uncommitted control points remain editable;
- the next fitting operation preserves the committed boundary position, tangent, and second derivative;
- new control points are added from the incoming path.

The committed boundary must satisfy the selected continuity requirements.

For a cubic `C2` trajectory, adjoining windows must match:

\[
\mathbf r_{\text{old}}=\mathbf r_{\text{new}}
\]

\[
\mathbf r'_{\text{old}}=\mathbf r'_{\text{new}}
\]

\[
\mathbf r''_{\text{old}}=\mathbf r''_{\text{new}}.
\]

After timing, the corresponding physical position, velocity, and acceleration must also match.

---

# 14. Exact primitives and approximation error

For G64 spline fitting:

- lines can be represented exactly by cubic B-splines;
- circular arcs are approximated by ordinary polynomial B-splines;
- helices are approximated by ordinary polynomial B-splines.

The primitive-to-spline approximation tolerance should consume only a controlled fraction of the total path error budget.

Conceptually:

\[
\epsilon_{\text{conversion}}
+
\epsilon_{\text{blending}}
+
\epsilon_{\text{numerical}}
+
\epsilon_{\text{servo}}
\le
\epsilon_{\text{allowed}}.
\]

Do not allow the fitting stage to consume the entire user-visible G64 tolerance unless the rest of the budget is explicitly accounted for.

---

# 15. Machine-axis constraints

For a simple Cartesian machine, Cartesian coordinates may correspond directly to axis coordinates. For machines with nonlinear kinematics, rotary axes, tool-center-point control, or orientation motion, constraints must be checked in actual commanded axis coordinates.

The final limits are:

\[
|\dot q_i(t)|\le V_{i,\max}
\]

\[
|\ddot q_i(t)|\le A_{i,\max}
\]

\[
|\dddot q_i(t)|\le J_{i,\max}
\]

for every axis \(i\).

A smooth Cartesian spline can still create excessive rotary-axis speed or acceleration near a singularity or inverse-kinematics branch change.

The NRT planner should resolve kinematic branches before publishing executable motion.

---

# 16. Recommended RT command representation

There are two reasonable interfaces.

## Option A: geometric spline plus scalar time law

RT evaluates:

- the geometric spline;
- the scalar progress law;
- inverse kinematics if not already resolved.

This preserves the exact NRT path representation but increases RT work.

## Option B: compiled axis-polynomial spans

NRT compiles the fully timed motion into fixed polynomial spans in axis space.

RT evaluates only the active polynomial.

This is the preferred choice when minimizing RT.

A possible structure is:

```cpp
struct RtPolynomialSpan {
    uint64_t span_id;
    double duration;
    double inv_duration;
    double inv_duration_squared;
    double inv_duration_cubed;

    // Normalized power basis:
    // q(u) = a*u^3 + b*u^2 + c*u + d
    AxisVector a;
    AxisVector b;
    AxisVector c;
    AxisVector d;

    AxisVector end_position;
    AxisVector end_velocity;
    AxisVector end_acceleration;
};
```

RT evaluates:

```cpp
double u = elapsed * span.inv_duration;

AxisVector position =
    ((span.a * u + span.b) * u + span.c) * u
    + span.d;

AxisVector velocity =
    (3.0 * span.a * u * u
     + 2.0 * span.b * u
     + span.c)
    * span.inv_duration;

AxisVector acceleration =
    (6.0 * span.a * u
     + 2.0 * span.b)
    * span.inv_duration_squared;

AxisVector jerk =
    6.0 * span.a
    * span.inv_duration_cubed;
```

If the final timed trajectory is approximated into cubic axis spans, NRT must verify:

- trajectory-position error;
- velocity error;
- acceleration error;
- jerk limits;
- `C2` continuity;
- axis limits over the complete spans.

The final RT polynomial format does not have to be identical to the G64 geometric B-spline.

---

# 17. RT responsibilities

RT should contain only bounded and deterministic operations:

1. Read the current immutable plan.
2. Select the active span.
3. Evaluate the polynomial at the servo time.
4. Output axis position and optional feedforward values.
5. Advance the span index.
6. At a branch point, accept a valid continuation or select the stop tail.
7. Publish execution status and branch decisions.
8. Check watchdogs, drive state, and hard safety conditions.

RT should avoid:

- dynamic allocation;
- path fitting;
- knot insertion;
- general optimization;
- parsing;
- file operations;
- logging that can block;
- unbounded searches;
- iterative numerical solvers.

---

# 18. NRT responsibilities

NRT performs the complex work:

1. Parse G-code and modal state.
2. Maintain exact source primitives.
3. Apply G61, G61.1, and G64 policy.
4. Build and validate G64 cubic spline fits.
5. Resolve kinematics.
6. Perform lookahead.
7. Apply velocity, acceleration, and jerk limits.
8. Generate stopping trajectories.
9. Compile the motion into RT polynomial spans.
10. Build immutable plan chunks.
11. Publish them atomically.
12. Process RT acknowledgements.
13. Replan after starvation stops.
14. Maintain program-position and event mappings.
15. Record diagnostics and performance telemetry.

---

# 19. Plan chunks and guaranteed stop tails

A plan chunk should contain:

- an epoch identifier;
- a chunk identifier;
- its predecessor branch identifier;
- normal-motion spans;
- a branch point;
- a fallback stop tail;
- program-position metadata;
- side-effect events;
- validation metadata.

Conceptually:

```cpp
struct MotionState {
    AxisVector position;
    AxisVector velocity;
    AxisVector acceleration;
};

struct ProgramCursor {
    uint64_t block_number;
    double path_parameter;
    uint32_t event_index;
};

struct PlanChunk {
    uint64_t epoch;
    uint64_t chunk_id;
    uint64_t predecessor_branch_seq;

    SpanArray normal_motion;
    SpanArray stop_tail;

    MotionState branch_state;
    MotionState stop_state;

    ProgramCursor branch_cursor;
    ProgramCursor stop_cursor;
};
```

The stop tail must begin at exactly the branch state:

\[
q_{\text{stop}}(0)=q_{\text{normal}}(t_b)
\]

\[
\dot q_{\text{stop}}(0)=\dot q_{\text{normal}}(t_b)
\]

\[
\ddot q_{\text{stop}}(0)=\ddot q_{\text{normal}}(t_b).
\]

Its end state should be:

\[
\dot q(T)=0
\]

\[
\ddot q(T)=0.
\]

Jerk may end at a nonzero value only if the next held state handles the transition safely, but the preferred terminal condition is:

\[
\dddot q(T)=0.
\]

That makes restart planning cleaner.

---

# 20. Irrevocable branch decisions

At every branch point, RT samples the continuation state once.

Conceptually:

```cpp
if (continuation_is_ready_and_valid()) {
    latch_decision(CONTINUE, continuation_id);
    activate_continuation();
} else {
    latch_decision(STOP, 0);
    activate_stop_tail();
}
```

The decision is irrevocable.

If NRT publishes the continuation immediately after RT selects the stop tail, the continuation is late and must never be executed.

This rule eliminates race ambiguity.

---

# 21. RT acknowledgement to NRT

NRT must not infer the branch decision from queue occupancy or timing.

RT should explicitly publish a coherent status snapshot:

```cpp
enum class BranchChoice : uint8_t {
    NONE,
    CONTINUE,
    STOP
};

enum class RtMotionState : uint8_t {
    IDLE,
    RUNNING,
    STOPPING,
    HELD,
    FAULTED
};

struct RtTrajectoryStatus {
    uint64_t active_epoch;
    uint64_t decided_branch_seq;
    BranchChoice branch_choice;
    uint64_t accepted_chunk_id;

    uint64_t active_chunk_id;
    uint32_t active_span;
    double trajectory_time;

    RtMotionState motion_state;
};
```

Use a deterministic publication method such as:

- a sequence lock;
- a double-buffered status record;
- a small single-producer/single-consumer status ring.

NRT treats this RT status as authoritative.

---

# 22. Handling a late continuation

Example:

```text
RT:  continuation for branch 107 is not ready
RT:  latches STOP for branch 107
NRT: publishes chunk 108 for branch 107
RT:  continues executing the stop tail
```

Chunk 108 is permanently invalid because its predecessor branch has already been resolved as `STOP`.

RT can reject stale publications using:

```cpp
if (published.predecessor_branch_seq <= decided_branch_seq) {
    // Too late or otherwise obsolete.
}
```

The epoch also prevents a stale plan from a previous recovery cycle from being accepted.

---

# 23. What NRT does after RT selects STOP

After observing the RT acknowledgement, NRT should:

## 23.1 Abandon speculative descendants

Any unaccepted chunk that assumes the normal continuation is obsolete.

```text
chunk 108 -> obsolete
chunk 109 -> obsolete
chunk 110 -> obsolete
```

## 23.2 Stop extending the old nominal plan

The machine is now executing the stop tail, so the nominal post-branch state is no longer valid.

## 23.3 Increment the plan epoch

The recovery plan should use a new monotonically increasing epoch.

```text
old epoch: 42
new epoch: 43
```

RT ignores old-epoch chunks once the new epoch is armed.

## 23.4 Determine the stopped program position

Because NRT generated the stop tail, it knows:

- the commanded stop position;
- the program cursor at the stop point;
- which auxiliary events have already occurred;
- which events remain pending.

## 23.5 Wait for the held state

The simplest policy is to complete the full stop tail and reach rest.

Do not attempt to splice back into normal motion during deceleration unless that feature is explicitly designed and verified.

## 23.6 Replan from rest

Build a new plan beginning from:

\[
q=q_{\text{stop}}
\]

\[
\dot q=0
\]

\[
\ddot q=0.
\]

Then regenerate:

- lookahead;
- G64 fitting where applicable;
- timing;
- events;
- a new stop tail.

## 23.7 Apply the resume policy

A starvation stop may require:

- explicit operator resume;
- automatic resume after sufficient lookahead is rebuilt;
- alarm acknowledgement;
- operation-specific recovery.

For cutting operations, explicit resume is often safer because restarting after a feed hold can affect the process.

---

# 24. RT and NRT state machines

## RT state machine

```text
IDLE
  |
  | start accepted
  v
RUNNING
  |\
  | \ continuation accepted
  |  -----------------------> RUNNING
  |
  | continuation unavailable
  v
STOPPING
  |
  | stop tail complete
  v
HELD
  |
  | new epoch armed and resume accepted
  v
RUNNING
```

## NRT state machine

```text
PLANNING
  |
  +-- RT accepted continuation
  |       |
  |       v
  |   retire predecessor and continue planning
  |
  +-- RT selected stop
          |
          v
      invalidate speculative descendants
          |
          v
      increment epoch
          |
          v
      plan from stop endpoint
          |
          v
      wait for RT HELD
          |
          v
      arm restart plan
```

---

# 25. Buffer ownership and publication

Use preallocated memory and immutable published data.

A useful slot lifecycle is:

```text
FREE
  |
  v
WRITING       NRT owns the slot
  |
  v
READY         immutable; RT may inspect it
  |
  v
ACCEPTED      RT owns the slot
  |
  v
RETIRED       safe for NRT to reuse
  |
  v
FREE
```

NRT must fully construct and validate a chunk before making it visible.

A publication pattern can be:

1. NRT obtains a free slot.
2. NRT writes all fields.
3. NRT runs validation.
4. NRT performs a release-store to publish `READY`.
5. RT performs an acquire-load.
6. RT verifies identity and predecessor.
7. RT accepts or ignores the chunk.
8. RT later advances the retired index.
9. NRT reuses only retired slots.

Do not let RT observe partially written coefficients or metadata.

---

# 26. Time-based buffering and watermarks

Buffer health should be measured in trajectory time, not command count.

Define:

\[
H
=
t_{\text{scheduled end}}
-
t_{\text{current trajectory}}.
\]

Useful thresholds are:

- startup watermark;
- low watermark;
- critical watermark;
- branch deadline;
- guaranteed stop horizon.

The controller should not start motion until enough executable trajectory and a complete stop tail are committed.

Watermarks improve availability, but they are not the safety guarantee. The stop-tail invariant is the guarantee.

---

# 27. Optional feed derating

When the lookahead horizon shrinks, NRT may reduce the commanded feed to gain planning time.

However:

- feed scaling must not create an acceleration discontinuity;
- it must not create an excessive jerk command;
- the resulting slowed trajectory must remain synchronized with process events.

The simplest implementation is for NRT to publish slower future spans rather than letting RT abruptly scale the current feed.

Feed derating should supplement, not replace, the guaranteed stop tail.

---

# 28. Stop-tail design

The stop tail should preferably:

- remain on the already validated path;
- satisfy every axis velocity limit;
- satisfy every axis acceleration limit;
- satisfy every axis jerk limit;
- preserve position, velocity, and acceleration continuity at its start;
- end at zero velocity and zero acceleration;
- avoid invalid kinematic branches;
- carry a precise program cursor and event state.

Independent per-axis braking is easier but may leave the programmed path.

For ordinary feed hold, coordinated on-path stopping is preferable.

For faults where the planned path is no longer trustworthy, drive-level or safety-system stopping may supersede the path-preserving stop tail.

---

# 29. Safety separation

NRT starvation handling is not the complete machine-safety system.

Treat these cases separately:

## Planner starvation

Use the precomputed on-path stop tail and transition to `HELD`.

## Following error or invalid feedback

Use servo or drive fault handling.

## Hard limits, communication loss, emergency stop

Use an independent safety mechanism, potentially including controlled stop and safe torque off.

## Corrupt trajectory data

Reject the continuation and execute the already valid stop tail.

## Kinematic or numerical failure during planning

Do not publish the failed plan. Allow RT to take the current stop tail.

---

# 30. Side effects and synchronized processes

Events such as:

- spindle commands;
- coolant;
- laser enable;
- probing;
- digital outputs;
- tool synchronization;
- threading;
- rigid tapping;

must be associated with committed trajectory locations.

Do not execute an event belonging to a continuation until RT has committed to that continuation.

If the stop branch is selected:

- events before the stop cursor are complete;
- events after the stop cursor remain pending;
- boundary events need deterministic inclusion rules;
- spindle-synchronous operations may require operation-specific recovery rather than geometric resume.

---

# 31. Program-cursor mapping

Every executable trajectory region should map back to the source program.

For example:

```cpp
struct ProgramCursor {
    uint64_t block_number;
    double primitive_parameter;
    uint32_t event_index;
};
```

The stop endpoint should identify the exact remaining-program position.

Example:

```text
block 500 ---- block 501 ---- block 502

stop tail begins:
    block 501, parameter 0.32

stop tail ends:
    block 501, parameter 0.41

replanning resumes:
    block 501, parameter 0.41
```

This mapping is also important for:

- display;
- restart;
- single-block execution;
- probing;
- cutter compensation;
- event reconciliation;
- diagnostics.

---

# 32. Validation before publication

Every NRT chunk should pass a complete validation stage.

## Identity and protocol

- epoch is current;
- chunk ID is unique;
- predecessor branch is correct;
- buffer ownership is valid;
- all referenced memory is immutable.

## Geometric validity

- G64 deviation is within tolerance;
- source-path order is preserved;
- no protected boundary is blended;
- fitting-window joins are continuous;
- no invalid shortcut occurs.

## Kinematic validity

- inverse-kinematics branch is valid;
- axis positions are within limits;
- no singularity or branch transition is crossed unexpectedly.

## Dynamic validity

- axis velocity limits are satisfied;
- axis acceleration limits are satisfied;
- one-sided axis jerk limits are satisfied at knots;
- stop tail is feasible;
- branch-state matching is exact within the numerical design.

## Execution validity

- all durations are positive and finite;
- coefficient values are finite;
- endpoint states are consistent;
- event ordering is deterministic;
- program cursors are monotonic.

Only after all checks pass should the chunk be published.

---

# 33. Numerical continuity strategy

Avoid independently rounded duplicate boundary states where possible.

A strong construction is:

1. Generate the first span from an explicit initial state.
2. Compute its endpoint using the same arithmetic used by the evaluator.
3. Use that computed endpoint as the next span’s initial state.
4. Continue recursively.

For independently generated spans, validate:

\[
q_k(T_k)=q_{k+1}(0)
\]

\[
\dot q_k(T_k)=\dot q_{k+1}(0)
\]

\[
\ddot q_k(T_k)=\ddot q_{k+1}(0).
\]

RT does not need to repeat expensive floating-point validation every servo cycle. It can trust committed data and optionally perform lightweight consistency checks at chunk acceptance.

---

# 34. Suggested minimal RT loop

Conceptually:

```cpp
void servo_cycle(double now)
{
    RtPlanView plan = load_active_plan();

    if (!plan.valid()) {
        enter_fault_or_hold();
        return;
    }

    RtPolynomialSpan const& span = plan.active_span();

    double elapsed = now - span_start_time;
    double u = clamp(elapsed * span.inv_duration, 0.0, 1.0);

    AxisVector q =
        ((span.a * u + span.b) * u + span.c) * u
        + span.d;

    AxisVector v =
        (3.0 * span.a * u * u
         + 2.0 * span.b * u
         + span.c)
        * span.inv_duration;

    AxisVector a =
        (6.0 * span.a * u
         + 2.0 * span.b)
        * span.inv_duration_squared;

    write_axis_commands(q, v, a);

    if (elapsed >= span.duration) {
        advance_span_or_branch();
    }

    publish_rt_status();
}
```

The branch routine is:

```cpp
void advance_span_or_branch()
{
    if (!at_branch_point()) {
        advance_to_next_span();
        return;
    }

    Continuation const* next = inspect_published_continuation();

    if (next != nullptr &&
        next->epoch == active_epoch &&
        next->predecessor_branch_seq == current_branch_seq &&
        next->is_committed) {

        latch_branch_result(
            BranchChoice::CONTINUE,
            next->chunk_id);

        activate(*next);
    } else {
        latch_branch_result(
            BranchChoice::STOP,
            0);

        activate(current_chunk.stop_tail);
    }
}
```

All validation that can be performed in NRT should be removed from the servo loop.

---

# 35. Suggested NRT control loop

Conceptually:

```cpp
void planner_iteration()
{
    RtTrajectoryStatus status = read_rt_status();

    process_retired_buffers(status);

    if (status.branch_choice == BranchChoice::STOP &&
        status.decided_branch_seq > last_processed_branch) {

        invalidate_descendants(status.decided_branch_seq);
        begin_recovery_epoch();
        plan_from_known_stop_endpoint();
        last_processed_branch = status.decided_branch_seq;
    }

    if (need_more_horizon()) {
        SourceWindow source = collect_lookahead();

        GeometricPath geometry =
            apply_modal_path_policy(source);

        TimedTrajectory timed =
            time_parameterize_and_limit(geometry);

        PlanChunk chunk =
            compile_with_stop_tail(timed);

        validate_chunk(chunk);
        publish_chunk(chunk);
    }
}
```

The real implementation will separate parsing, fitting, timing, compilation, validation, and publication into distinct modules.

---

# 36. Recommended module boundaries

A maintainable NRT architecture could use:

```text
gcode/
    parser
    modal_state
    program_cursor

geometry/
    line
    arc
    helix
    ordered_path
    cubic_bspline
    bezier_extraction
    distance_validation

blend/
    g64_window_builder
    constrained_fitter
    knot_refinement
    boundary_constraints

kinematics/
    forward
    inverse
    branch_tracking
    singularity_analysis

timing/
    path_derivatives
    axis_constraints
    lookahead
    jerk_limited_parameterization
    stop_tail_generator

compile/
    axis_polynomial_fit
    continuity_validation
    chunk_builder

transport/
    preallocated_ring
    publication_protocol
    rt_status_reader
    epoch_manager

rt/
    polynomial_evaluator
    branch_selector
    status_publisher
    watchdogs
```

This keeps spline fitting independent of the RT transport protocol and keeps motion policy independent of low-level servo execution.

---

# 37. Recommended implementation sequence

A staged implementation reduces risk.

## Stage 1: RT transport and stop-tail protocol

Implement:

- preallocated buffers;
- immutable publication;
- epochs and sequence numbers;
- RT status acknowledgement;
- continuation-or-stop branching;
- held-state recovery.

Test this using simple synthetic axis polynomials before adding G-code geometry.

## Stage 2: Exact G61/G61.1 motion

Implement:

- line timing;
- arc timing;
- optional helix timing;
- exact-path/exact-stop semantics;
- stop tails;
- program-cursor tracking.

Compile those motions into the RT polynomial format.

## Stage 3: Cubic G64 spline fitting

Implement:

- local lookahead windows;
- ordered path parameterization;
- cubic B-spline fitting;
- endpoint and overlap constraints;
- tolerance validation;
- adaptive knot insertion.

Initially use conservative speed limits.

## Stage 4: Full axis dynamic constraints

Add:

- axis-space velocity limits;
- acceleration limits;
- one-sided jerk limits;
- curvature-aware feed planning;
- kinematic singularity handling.

## Stage 5: Performance improvements

Add:

- better fitting objectives;
- tighter derivative bounds;
- feed derating near low-watermark conditions;
- larger planning windows;
- optimized event scheduling;
- diagnostics and profiling.

## Stage 6: Optional smoothness upgrade

Only if testing requires it, consider:

- degree-4 or degree-5 G64 geometry;
- continuous-jerk time laws;
- mid-stop interception;
- more advanced global optimization.

---

# 38. Final design recommendation

The recommended design is:

## Geometry

- `G61` and `G61.1`: retain exact lines, arcs, and optional helices.
- `G64`: fit a tolerance-bounded cubic B-spline over a finite lookahead window.
- Use simple interior knots in ordinary blended regions to obtain `C2` geometry.
- Preserve source-path order and protected boundaries.
- Use adaptive knot insertion and full-curve deviation verification.

## Dynamics

- Perform timing in NRT.
- Limit actual machine-axis velocity, acceleration, and jerk.
- Require physical position, velocity, and acceleration continuity.
- Permit jerk steps at cubic knots.
- Bound both one-sided jerk values at every knot.
- Do not assign unrelated durations independently to geometric spans.

## RT interface

- Compile timed motion into prevalidated axis-polynomial spans.
- Use precomputed reciprocal durations.
- Use immutable, preallocated chunks.
- RT only evaluates polynomials, advances spans, chooses continuation or stop, and reports status.

## Starvation protection

- Every executable chunk contains a complete feasible stop tail.
- RT makes an irrevocable branch decision.
- Late continuations are obsolete.
- RT explicitly acknowledges `CONTINUE` or `STOP`.
- After `STOP`, NRT invalidates speculative descendants, increments the epoch, waits for `HELD`, replans from the known stop endpoint, and applies the resume policy.

## Safety

- Planner-starvation stopping is separate from emergency stop, hard-limit, drive-fault, and safe-torque-off mechanisms.
- Synchronized events are tied only to committed trajectory locations.

---

# 39. Core invariants

The implementation should preserve these invariants at all times:

1. RT never depends on NRT producing a command by an unbounded future deadline.
2. RT always has a valid trajectory to rest.
3. Published trajectory data is immutable.
4. RT branch decisions are irrevocable and explicitly acknowledged.
5. A continuation is valid only for its exact epoch and predecessor branch.
6. Position, velocity, and acceleration are continuous across executable span boundaries.
7. Both one-sided jerk values remain within configured axis limits.
8. G64 geometry remains within its allowed ordered-path deviation.
9. Uncommitted speculative motion may be discarded without affecting RT.
10. Safety shutdown does not depend solely on the trajectory planner.

These invariants provide the foundation for a minimal RT implementation without sacrificing deterministic behavior during NRT delays or failures.
