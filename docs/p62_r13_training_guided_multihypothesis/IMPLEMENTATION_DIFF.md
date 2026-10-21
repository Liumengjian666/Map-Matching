# R13 actual code delta

Only docs/p62_r13_training_guided_multihypothesis changed. R6–R12 are imported without modifications.

- tools/coarse.hpp: fixed45 right-body full-SE3 grid; double and actual native-float bounds; deterministic256 training-only indices; same-Target clipped squared NN score; frozen local exclusion/tie/diversity rules; no validation argument, optimization callback or GT input.
- tools/shadow.cpp: exact five Source/TP/T0 identity; no nominal baseline align; phase1 freezes every grid/selection then phase2 loads nominal heldout evidence and runs <=10 captured native aligns. New validation and pool decisions use unchanged Coverage V2/V3. Historic baselines are read-only archived V3 decisions, not rerun or borrowed optimizer candidates.
- Native calls use frozen NativeNdt/capturedAlign, actual .8m Gaussian grid, same source/map/config, no G or Hessian search. Original result is flushed before auxiliary fitness. All returned candidates, including duplicates and ineligible poses, remain recorded.
- runtime CSV separates source, coarse NN, full coarse phase, selection, align, capture/fitness auxiliary, validation and V3 decision. extra_event_wall_ms excludes Source, but source_plus_extra is explicitly computed offline. Process wall includes cold map/init, not old R11 runtime.
- execute.py: complete hashes, source-bound cold build/tests, exclusive one-run directory; Popen launch/kill/wait records true launched count and returncode, failure phases are isolated and FINALIZATION remains in RUN. Only read-only snapshot helpers are loaded from the old script; no old action/protection executes.
- reference_inventory.py: single bounded local name/metadata inventory. No reference fit, download or new dataset audit.
- analyze.py: offline exact-matrix/call/selection lineage, independent SE3 matrix exponentials and LP checks. It does not run NDT, nearest-neighbor scoring, old experiments or new trajectory estimation. --reviewed refines derived reporting once while retaining the initial report.

The search seed box .60m/12deg does NOT change .8m/5deg final admission. V3 dedup .10m/1deg and D>.02 are unchanged. A selected zero-grid initializer may equal historical TP; it is not separately exempted from the allowed10 native calls or counted as a new physical hypothesis.
