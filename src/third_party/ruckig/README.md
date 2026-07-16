# Vendored Ruckig subset

This directory contains the state-to-state trajectory-generation subset used by NGC.
It was copied from [Ruckig](https://github.com/pantor/ruckig) commit
`2249d57ffaa19ecdadeaab62daf97857813629ff` (upstream version description
`v0.17.3-8-g2249d57`) on 2026-07-16.

Ruckig is copyright Lars Berscheid and distributed under the MIT License. The
upstream license is reproduced verbatim in [LICENSE](LICENSE).

## Included scope

NGC uses fixed-size one- and six-degree-of-freedom, offline, state-to-state
calculation. The vendored files therefore contain the 11 core Community Edition
solver translation units and the headers reachable by that path. Cloud and
waypoint calculation, the online update/output API, dynamic-DoF conveniences,
language wrappers, examples, tests, benchmarks, documentation, and upstream build
packaging are intentionally omitted.

All 11 solver translation units are retained because NGC exercises position and
velocity control, first/second/third-order limit cases, braking, disabled axes,
and synchronized multi-axis solves. A linker-map audit confirmed that each solver
object is reachable from the final oracle exporter.

## Local changes

The solver implementation files and all copied upstream headers are unmodified
from the recorded commit. The NGC-owned replacement header is listed below.

- `include/ruckig/ruckig.hpp` is an NGC-owned fixed-DoF, offline-only facade over
  upstream `TargetCalculator`; it preserves only the `calculate` API NGC uses.

When updating this subset, start from the recorded upstream commit, audit the
include graph and linked objects again, copy the required files, reapply the
offline facade, and run both the test suite and trajectory benchmarks.
