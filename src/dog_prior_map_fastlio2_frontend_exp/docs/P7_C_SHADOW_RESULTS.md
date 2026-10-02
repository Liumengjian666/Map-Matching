# P7-C shadow integration and acceptance

## Scope and provenance

P7-C starts at `29d1f7def41eced4a197f9dec54f58aec7f1fa32`.
The C0 audit was completed and committed before production edits:
`cba57a42b44a6b564a87f7b6e0662009cebc2dab`.
C1 geometry/classifier/tests: `b679aff4fdbffb741e82c0969d826bbf95d2228e`.
See [mature-source audit and innovation boundary](P7_C_MATURE_REFERENCE_AUDIT.md).

The formal runner still performs one current-frame NDT and the original full-pose
IKFoM update on SUCCESS. Geometry, classifier, rank and Schur outputs are shadow
diagnostics only. No projected update, NIS state effect, recovery, visual input,
GT input, step limiter or nonlocal probes were added. No FAST-LIO2/IKFoM core,
PCL optimizer, historical P6 runner or machine-dog source was rewritten.

## Outputs and reproducibility

Prechange output: `/tmp/p7c_prechange_100`.
Shadow output: `/tmp/p7c_shadow_100`.
The wrapper validates all frozen input/map/calibration hashes. It reads any
`--shadow-reference` artifacts **only after the estimator exits**.

Runner arguments now end with:
`TRAJECTORY_CSV REGISTRATION_CSV RUNTIME_CSV UOBS_CSV FRAME_LIMIT INITIALIZATION_STAMP_NS`.
`uobs.csv` stores 6D spectra, H_phys/H_bar and 6x6 bases in named row-major columns.
Only the first weak_dimension/reliable_dimension basis columns are active.
Schur columns are explicitly `DIAGNOSTIC_ONLY`. Skipped frames are marked
NOT_COMPUTED and are not included in the weak-dimension histogram.

NDT alignment time remains its own timer. `ndt_total_ms` excludes geometric
analysis; `uobs_ms` covers extraction plus information analysis and
`classifier_ms` covers joint classification. Frame computation time includes
these operations but, as in B, excludes CSV serialization. RSS is also sampled
per frame, while `/usr/bin/time -v` records process peak RSS. MB fields use
KiB/1024 (MiB convention).

Build uses the existing pinned FAST-LIO2 checkout and Release C++17. C++17 is
needed by the existing reliability_metrics implementation. P7 links that helper,
not dual_reliability, DCReg, X-ICP, hdl_localization or Autoware. As in B, the
standalone build and child process remove the inherited camera SDK's
LD_LIBRARY_PATH; no system/SDK configuration was changed. With CTest 3.16, run
CTest **from the build directory**, not with the newer `--test-dir` option.

## Exact 100-frame parity

Both runs: 100 frames, 100 NDT alignments, 93 SUCCESS, 7 ITERATION_LIMIT_EXHAUSTED,
first ineffective tx=47, 93 full-pose updates and 7 prediction-only frames.
All state fields (prediction/correction, velocity, biases, gravity, timestamps
and update flags) are exactly identical in 17-digit output. Every registration
field other than alignment timing is exactly identical, including source hashes,
raw terminal, convergence, iterations, fitness and probability. Zero source-hash
mismatches. Frozen source-hash metadata are unavailable for all 100 frames;
equality here is against the independently rerun P7-B source sequence.

Both trajectory SHA256:
`db06698efb51a7431dd52bdc403859485c2f956c8cf430ebb80e403e71488218`.
Maximum corrected translation delta=0 m; rotation delta=0 degrees.
`p7c_shadow_report.json`/`.txt` record the posthoc comparison.
State and IKFoM postconditions pass for every frame. No accuracy/ATE claim is made.

## Shadow statistics

UOBS_COMPUTED=93; UOBS_VALID=93; UOBS_INVALID=0.
MAP_SUPPORT_INSUFFICIENT=0; NO_VALID_GEOMETRIC_CORRESPONDENCES=0;
UOBS_NUMERICAL_FAILURE=0; SCHUR_DIAGNOSTIC_UNAVAILABLE=0;
JOINT_CLASSIFIER_INVALID=0.

| Weak dimension | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Classified frames | 0 | 0 | 0 | 48 | 45 | 0 | 0 |

L is fixed at configured NDT resolution 0.8 m; weak ratio stays 0.05. The joint
basis preserves coupling; these dimensions do not remove any measurement rows.
No parameter or threshold was tuned after viewing the histogram.

## Resource observations

| Metric | Prechange B | Shadow C |
| --- | ---: | ---: |
| Mean frame computation ms | 49.320952 | 49.912822 |
| Peak RSS MiB | 52.191406 | 52.695313 |

Uobs mean=1.611498 ms, P95=2.026519 ms (93 computed frames).
Classifier mean=0.002203 ms, P95=0.002863 ms.
Shadow RSS frames 11–30 mean=49.617188 MiB; last 20 mean=49.617188 MiB;
last 50 span=0 MiB. Observed frame-local allocation is stable after warmup;
no linear growth in this 100-frame prefix. This is not a full-run leak proof.
The single timing pair is not an optimization or localization-performance claim.

## Tests and protection gates

P7 standalone CTest: 4/4 PASS (physical joint Uobs, actual current-frame PCL NDT,
replay IO, shadow reporting). Tests cover fixed normalization, no frame-group
equalization, coupled directions, global positive scaling, zero information,
joint ranks 0–6, orthogonal complements, significant negative information,
Schur-unavailable classification, Schur reference solves, nonfinite inputs and
low-support accumulation overflow. Real target-grid queries cover supported A,
unsupported B, then supported C; B contributes no stale neighbors.

P7-A catkin regression: 7/7 PASS. Preserved P6 research regression: 8/8 PASS;
historical closed-loop runner builds. git diff --check PASS.
Support shortage, absent correspondences and numerical failures are separately
counted; a reporting regression guards against conflating those categories.

Protected localization/interfaces diff against START: EMPTY. Stable machine-dog
workspace HEAD remains `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`; its pre-existing
status fingerprint is unchanged. Rescue manifest fingerprint is unchanged.
GT_USED=false; VISUAL=NONE; PROJECTED_UPDATE=false; NIS_STATE_EFFECT=false.

## Next-stage boundary

P7-C is ready for a separately instructed P7-D. It establishes diagnostics only,
not effectiveness or novelty. Exact residual-chart mapping, projected filtering,
NIS admission and later visual ablation remain future work. The audit's
EFFECTIVENESS_GATE and NOVELTY_GATE remain mandatory; no-effect or uneconomical
candidates must be discarded as main paper contributions.
