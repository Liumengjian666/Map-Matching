# Research implementation delta

Only docs/p62_r9_coverage_aware_recovery_v2/ changes. R6–R8, production C++, EKF/OOSM, maps and all historical outputs are read-only. No new dependency. The generic definition-of-done reference linked by the implementation skill is absent locally; the explicit task gates, builds, tests and asset receipts are used instead.

Reused byte-identically:
- R6 tools/native_kernel.hpp and contract.hpp: PCL1.10 NativeNdt, configure/checkGrid, actual objective/optimizer, .8/.08/.001/40/.55, voxel, hash, split, Candidate, bounds, physical dedup, Budget and SMALL seed.
- R8 tools/preprocess.hpp: exact original float arithmetic, range .5–80, .25m voxel, deterministic cap1400 and point order.
- R7 Recovery data structure and its normalized G eigenanalysis/guards; the latter is extracted verbatim into a direction-only helper. No native Hessian is substituted or claimed.
- R7 raw10641–10740 extracted inputs, original manifest, preregistration, historical causal initial guesses, and frozen map.

New coverage.hpp supplies pre-match active masks, active-only losses/mean/median/overlap and capped common-weight comparisons for B4–7; B8 delegates to V1. The original pool/dedup/output physical bounds remain. A local review found vacuous alternative acceptance in a singleton with missing nominal: V2 now explicitly requires a rival. This conservative guard is covered by a regression, not hidden as a numerical scoring change.

io.hpp mirrors the frozen minimal CSV/pose serialization but calls the canonical R8 preprocessing helper. preflight.cpp checks the fixed100 and historical Source identity, writes coverage and scanner-frame block distributions, with zero align. shadow.cpp keeps FULL/SPLIT separate, caches every SPLIT nominal, executes natural recovery then the fixed two-scale diagnostic schedule with one shared Budget. Every raw candidate and duplicate is retained. No diagnostic candidate can enter a natural frame's pool or affect a later historical initial.

run_stage.py requires committed source, exclusive stage tokens, new output directories, complete input SHA checks, exact historical initial fields, protected source and production state checks, and after-run rehash. Formal execution verifies the preflight's registered rules/coverage/source-manifest and canonical preprocessor hashes; a fresh current hash cannot silently replace a registered one. Driver/report changes after the coverage-only preflight are separately versioned; frozen preprocessing and math remain identical.

The previous B8 coverage requirement is deliberately changed by this approved task, not claimed as unchanged. B<4 structural evidence is unavailable; generic diagnostics may still study the frozen training geometry but cannot accept alternatives from unavailable holdout evidence. In this fixed window all100 actually satisfy B>=6 and training>=300, so the insufficient-coverage path remains unit-tested rather than observed.

FULL uses validation points in optimization and is an engineering control, not independent holdout validation. Raw inputs remain undeskewed with historical midpoint initials. Beta=0; gyro is not used for candidate decisions. Spatial validation blocks remain correlated and missing sectors can hide errors. No GT, ATE/RPE, accuracy, correct-basin recovery, novelty or sustained10Hz claim is supported by an engineering pass.
