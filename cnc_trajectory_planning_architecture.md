# CNC trajectory-planning architecture

Updated: 2026-07-17

This document describes the current NGC motion architecture and its intended
boundaries. Historical implementation plans and completed debugging handoffs
are deliberately omitted. `AGENTS.md` remains the concise source of repository
rules; this document explains how the major pieces fit together.

## Architectural boundary

NGC separates four kinds of work:

```text
interpretation -> geometry preparation -> trajectory timing -> execution
```

- Interpretation evaluates G-code and emits canonical `MachineCommand` values.
- Geometry preparation constructs feed-independent path geometry.
- Trajectory timing applies velocity, acceleration, and jerk limits and emits
  timed axis-space cubic polynomials.
- Execution consumes bounded packets and interacts with motion control.

Preview stops after geometry preparation. It must not run trajectory timing or
simulate servo execution merely to draw the requested toolpath.

## Current pipelines

### Geometry-only Preview

```text
InterpreterSession
    -> GeometryStreamProducer
    -> PreparedGeometryForwardChannel (owning NRT SPSC)
    -> Worker
       -> PreparedPreviewScene
       -> PreviewGeometryCollector
    -> OpenGL presentation
```

`GeometryStreamProducer` owns calls to `InterpreterSession::nextWithBlocks()`.
It captures presentation metadata, prepares continuous geometry, resolves
ordering barriers through feedback messages, and publishes immutable owning
messages.

`Worker` retains the producer's prepared slices, standalone prepared motion,
and presentation metadata as Preview's sole geometry representation. It
returns a canonical target result for preview probes so interpretation can
continue without a backend.

### Preview presentation

The OpenGL presentation does not submit the complete prepared scene every
frame. A dedicated NRT visibility thread snapshots a completed
`PreparedPreviewScene` revision and rebuilds visible CPU draw batches at no
more than 10 Hz while the camera or preview changes. Segment/viewport
intersection uses a small off-screen margin so ordinary camera motion does not
expose gaps between updates.

Lines remain exact single display segments. Arcs and splines each start with
eight ordered sections and adaptively subdivide to a two-pixel projected chord
error under a fixed recursion bound. Tessellation therefore follows viewport
size, orientation, and zoom without changing prepared geometry. The visibility
pass also computes a length-weighted centroid of path geometry clipped to the
actual viewport; camera rotation uses that centroid while compensating pan so
updating the pivot does not move the current view.

The visibility thread performs CPU work only and publishes complete replacement
batches. The render thread alone uploads a ready batch to the OpenGL buffer and
draws the feed, rapid, G53, arc, probe, spline/control, and endpoint ranges.
Full-scene bounds remain available for fit-to-view even though off-screen path
segments are not submitted for drawing.

### Timed simulation and future execution

```text
GeometryStreamProducer
    -> PreparedGeometryForwardChannel
    -> PreparedTrajectoryExecutionDriver
    -> BoundedLookaheadTrajectoryPlanner
    -> ExactStopTrajectoryPlanner
    -> MotionBackend execution SPSC
    -> MockMotionBackend or future RT backend
```

The geometry stream and prepared trajectory driver exist, but multi-slice timed
simulation remains an active integration area. Simulation behavior must not
change how geometry is generated. A prepared-slice boundary is not an exact
stop, a planning horizon, or permission to discard path continuity.

## Interpretation

`InterpreterSession` incrementally evaluates the program and emits one
canonical command at a time. It retains chronological status and explicit
barrier events.

Interpretation may run ahead through ordinary commands, but it must stop at
operations whose result depends on prior physical execution. These include:

- generic parameter-read synchronization;
- probing;
- homing;
- tool-change synchronization;
- ordered externally visible operations such as `print`.

The thread that owns `InterpreterSession` is the only thread allowed to mutate
the session or its `Machine`.

## Prepared geometry

Prepared geometry is immutable, NRT-owned, and independent of dynamic machine
limits. It contains exact parameterized curves and the metadata required to
associate geometry with source commands and presentation state.

### Source and replacement structures

One canonical CAM line or arc is a source entity. Continuous G64 geometry uses
only these structures:

```text
retained primitive section
    -> junction blend
    -> retained primitive section

short-entity cluster
    -> cluster spline
```

A retained primitive section is the middle of a sufficiently long source line
or arc after local endpoint trimming. A junction blend is the current local
six-control cubic spline replacing the trimmed ends around one junction.

A short-entity cluster spline replaces a complete sequence of short source
entities. It is not a connector laid over retained short entities. Cluster
selection and reconstruction are shared by Preview and timed planning.

### Prepared curve data

`PreparedCurve` currently holds one of:

- an exact line;
- an endpoint-exact arc;
- a prepared spline with controls, knots, derivative control nets, and an
  immutable arc-length bracket table.

`PreparedPathPiece` selects an interval of a prepared curve and carries:

- stable piece and command identity;
- piece kind;
- programmed feed;
- presentation activation ownership;
- 65 feed-independent geometric samples used by Preview diagnostics.

The spline table is only an inverse bracket. Exact requested distances are
still certified against the adaptive arc-length integral and cached by their
bit-exact distance.

## Incremental geometry slicing

The default nominal slice target is 0.25 seconds, estimated from prepared path
length divided by programmed feed. It is a minimum accumulation target, not a
hard maximum.

After pending geometry reaches the target, the producer continues until it has
enough source context to identify a source line or arc longer than `6P`. That
long source provides a safe retained primitive section around which the stream
can be divided.

For a rolling division:

1. Finalize the replacement entering the long source.
2. Finalize that source's retained primitive section, trimmed at both ends.
3. Publish the pending slice when its nominal duration has reached the target.
4. Generate the outgoing junction blend or cluster spline after enough future
   source context is available.
5. Put that outgoing replacement first in the next slice.

The final 3P of the long source is replaced by the outgoing replacement; it is
not retained as separate suffix geometry.

The producer does not construct or sample the newly encountered anchor's
retained primitive section in the preceding preparation. It retains only the
source record needed for the next preparation and immediately removes earlier
source records. The retained section is constructed once, when its outgoing
replacement has become known and the section is final.

An all-short region cannot be divided at an invented boundary. It remains one
geometry-consistent region until a valid long anchor or a natural/protected end
is reached.

## Slice continuity

Continuity across slices is checked and interpreted exactly like continuity
between adjacent pieces inside one slice.

The end of slice N and beginning of slice N+1 must agree directly in position
and geometric derivatives required by their curve definitions. The stream does
not carry speculative duplicate suffix geometry as a continuity check.

A slice boundary has no execution semantics. In particular, it must not:

- create an exact stop;
- reset velocity or acceleration;
- create a synthetic connector;
- divide a cluster spline;
- activate a source command twice.

## Owning NRT SPSC channels

`OwningSpscChannel` transports move-only heap-owned messages between NRT
participants. It has a fixed number of usable slots and never overwrites or
drops a message.

The ring fast path uses one producer index, one consumer index, and
release/acquire publication. Optional sleeping waits register the waiter before
checking the condition and synchronize notification through a mutex, avoiding
the condition-variable lost-wakeup gap. The fast path takes no mutex unless the
opposite endpoint is registered as sleeping.

The forward channel carries prepared stream messages. The feedback channel
returns probe results, synchronization releases, and abort information to the
thread that owns interpretation.

These owning channels are NRT-to-NRT infrastructure. They do not replace the
bounded allocation-free SPSC contract used by `MotionBackend`.

## Trajectory timing

`ExactStopTrajectoryPlanner` consumes path geometry and produces timed
axis-space cubic execution spans. Dynamic work remains on the planning side:

- programmed-feed and per-axis velocity limits;
- aggregate and per-axis acceleration limits;
- aggregate and per-axis jerk limits;
- reachable boundary velocity and acceleration;
- Ruckig scalar transition timing;
- exact emitted-polynomial extrema checks;
- correction passes;
- packetization and stop-tail construction.

For continuous geometry, sampled tangent, curvature, and full geometric jerk
coefficient establish conservative local limits. The planner then performs
jerk-aware forward/backward reachability and bidirectional station sweeps over
complete scalar `(v,a)` states.

The complete coupled path quantities are:

```text
axis acceleration = q' a + q'' v^2
axis jerk         = q' j + 3 q'' v a + q''' v^3
```

Initial geometric caps are sufficient local bounds, not the final trajectory
proof. Exact extrema of emitted axis polynomials remain authoritative.

## Rolling trajectory planning

`BoundedLookaheadTrajectoryPlanner` collects compatible G64 commands and may
publish an immutable timed prefix while retaining a mutable suffix. A valid
rolling boundary must provide:

- an exact path position;
- a dynamically feasible nonzero position/velocity/acceleration state;
- a following suffix that remains plan-feasible;
- a complete stop branch for every published packet.

The configured lookahead duration is a positive minimum prediction, not a hard
maximum. Geometry safety and dynamic feasibility may require a larger horizon.

Prepared geometry slices should eventually feed one continuous planning
horizon without reconstructing their curves. They must not be treated as
independent calls that force terminal stops.

## Execution packets

Normal trajectory output is packetized into `PlanChunk` values with at most 256
normal execution spans. Packet capacity is a transport constraint, not a G-code
or geometric boundary. A primitive, junction blend, cluster spline, or local
timing chain may cross a packet boundary.

Each packet contains:

- an ordered normal-motion branch;
- exact terminal motion state;
- activation span identities;
- a separately verified stop branch beginning at the packet's immutable branch
  state.

The backend may continue into the next compatible packet or select the current
packet's stop branch. It must never execute uninitialized memory or synthesize
geometry after starvation.

## RT-facing contract

The RT-facing contract remains bounded and allocation-free. `ExecutionItem`
contains only execution data such as `PlanChunk`, `TriggeredMove`, and
`TriggeredJointMove`.

Do not put any of the following into `MotionBackend`:

- G-code strings or AST objects;
- prepared curves or spline controls;
- TOML configuration objects;
- UI presentation metadata;
- unbounded diagnostic vectors;
- preview tessellation.

## Presentation activation

Command presentation state is associated with stable command and execution
identities. Tool, tool-offset, WCS, modal, and active-block state becomes active
when execution reaches the owning activation span.

Prepared slices may reference the same source command on adjacent pieces, but
only the first owning message has `presentationActivation=true`. A transport
boundary must not duplicate user-visible command activation.

## Failure policy

Planning and proof failures are fatal to the active run. NGC must not silently:

- fall back from failed G64 planning to exact-stop execution;
- discard buffered commands;
- bridge a geometry gap with a connector;
- publish an unproved trajectory after a resource bound is exceeded;
- hide an error only in terminal output.

Failures enter the chronological UI status stream and `stderr` with source and
planning context.

## Current performance boundary

Geometry preparation precomputes spline derivative control nets, arc-length
brackets, and 65 geometric samples. Preview consumes those samples directly.

The trajectory planner currently evaluates another 65 locations per prepared
piece when constructing its local limits instead of consuming the prepared
sample array. That duplicate geometric evaluation is known unfinished work. A
future change may pass the prepared samples into timing, provided the planner
continues to use exact curve evaluation for adaptive geometry proof, boundary
states, and any query not represented by those samples.

## Core invariants

1. Interpretation, geometry preparation, timing, and execution remain separate.
2. Preview consumes prepared geometry but performs no trajectory timing.
3. Source entities, retained primitive sections, junction blends, and cluster
   splines retain their precise meanings.
4. Every prepared piece is generated once after it becomes final.
5. A short-entity cluster is reconstructed as one cluster spline and is never
   divided by a slice boundary.
6. Slice and packet boundaries do not imply an execution stop.
7. Adjacent pieces meet directly; synthetic connectors are forbidden.
8. Dynamic limits and exact polynomial proof remain planner responsibilities.
9. Every published execution packet retains a verified stop branch.
10. The RT-facing contract remains bounded, allocation-free, and free of UI or
    geometry-preparation objects.
