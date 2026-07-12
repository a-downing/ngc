# Code audit findings

This file tracks the remaining findings from the July 2026 project audit. Items are removed as they are fixed and covered by regression tests.

5. `Worker` lifecycle is not RAII-safe or idempotent. It has no destructor and `join()` does not check `joinable()`.
8. Large implementation-heavy headers increase coupling and rebuild cost. In particular, `Application`, `Machine`, `Evaluator`, and `SimulationExecutor` would benefit from smaller interfaces and `.cpp` implementation files.
