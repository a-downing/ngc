# G64 geometry text format

`ngc_g64_geometry_export` writes one text file for each contiguous executable G64 chain. The file contains only executable curve geometry and its programmed feed rates. NGC preparation samples, arc-length inversion tables, geometric caps, timing metadata, identities, and smoother diagnostics are not exported.

The files use whitespace-separated ASCII tokens and decimal integers or doubles, so they can be parsed with the C++ standard library. Blank lines are not significant. The current format version begins with:

```text
ngc_g64_geometry 1
```

All positions contain three values in `X Y Z` order. Linear coordinates and arc distances use the unit named by `units`; feeds use machine units per second.

The file header is:

```text
ngc_g64_geometry 1
units inch|millimeter
curve_count <count>
```

Each curve begins with its executable arc-length interval and one feed per `PathPiece` produced by the corresponding PathTempo geometry helper:

```text
curve_interval <fromDistance> <toDistance>
feed_count <count>
feed <programmedFeed>
...
```

`curve_interval` selects an arc-length interval on the following curve. Lines and arcs have one feed. A B-spline has one feed for each non-empty knot interval, in the same order as the `PathPiece` values returned by PathTempo's `sampleBSpline` helper. This preserves cluster-local programmed feeds without exporting NGC timing data.

A line curve is:

```text
curve line
from <X> <Y> <Z>
to <X> <Y> <Z>
```

An endpoint-exact directed arc or helix is:

```text
curve arc
from <X> <Y> <Z>
to <X> <Y> <Z>
center <X> <Y> <Z>
axis <X> <Y> <Z>
```

The axis direction carries CW/CCW orientation. The endpoint-exact representation supports major arcs, full circles, helices, non-XY planes, and the small start/end radius mismatch accepted by NGC. PathTempo's current constant-radius `sampleArc` input cannot reproduce a radius-mismatched arc by itself; its loader must construct the endpoint-exact evaluator and pass that geometry through `sampleArcLengthCurve`, or PathTempo must provide a dedicated endpoint-exact arc helper.

A clamped open-uniform B-spline is:

```text
curve bspline
degree <3|5>
control_count <count>
control <X> <Y> <Z>
...
knot_count <count>
knot <value>
...
```

Junction blends are six-control cubic splines. Cluster splines are variable-control quintic splines. PathTempo constructs one sampled `PathPiece` per non-empty knot interval directly from these controls and knots.

Every curve ends with `end_curve`, and the file ends with `end_geometry`.

From the repository root, build and export with:

```powershell
cmake --build build --target ngc_g64_geometry_export
.\build\ngc_g64_geometry_export.exe --smoother=uniform adaptive_pockets.ngc
```

For input `adaptive_pockets.ngc`, output files are named `adaptive_pockets.1.txt`, `adaptive_pockets.2.txt`, and so on in the input file's directory. Existing files with those names are replaced.
