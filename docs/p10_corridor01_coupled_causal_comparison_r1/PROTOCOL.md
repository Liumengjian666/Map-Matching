# Frozen run protocol

## Fixed Control reference

The prior prospective Control remains frozen and is not rerun:

- 396 scans: 5 s startup + 35 s evaluation; 7,989 IMU messages including the
  actual boundary sample; 11,473,279 raw points.
- Slice SHA256:
  `f6e77c96d97402e551835e81a1ce8d780d075da2f18a9cb5e4fc0646628b86d6`.
- Normalized map SHA256:
  `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.
- Extrinsics SHA256:
  `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`.
- Frozen Control evaluator transform: use the exact rotation, translation,
  and evaluation-origin values in
  `docs/p10_corridor01_frozen_control/posthoc_metrics.json`. Never refit a
  separate transform for Weak-only or Coupled.
- Frozen tracking metrics: 0–10 s translation RMSE 0.462 m; 3–35 s RMSE/P95/max
  1.885/5.004/5.563 m; sustained 0.5/1/2 m drift at +6.410/+26.738/+29.778 s.

These are Control values, not Coupled results. Evaluation is
`PREFIX-ALIGNED RELATIVE DRIFT`, not absolute map-frame ATE.

## Runtime configuration

All runs use the same input slice, normalized map, extrinsics, dataset/base
configs, deskew chain, scan-start timestamp, NDT objective/parameters, nominal
initial-guess logic, step limiter (0.5 m / 5 deg), EKF process settings, and
initial state as the frozen Control. Only `coupled_mode` changes. Existing
configuration SHAs are listed in `docs/p10_corridor01_prospective_causal_baseline_r1/input_manifest.json`.

The new NDT node calls inherited PCL `align()` once per frame. R6 local work is
objective/jet evaluation only: it adds no full NDT align. No GT is loaded until
the Shadow, Weak-only, and Coupled runtime artifacts are stopped, closed, and
SHA256-frozen.

## Required order

1. Run one full-slice `COUPLED_SHADOW` replay. Compare its input/source and
   nominal NDT diagnostics and all three output pose topics against frozen
   Control using `compare_shadow_control.py`. Any mismatch blocks feedback
   runs.
2. If Shadow parity passes, run one complete `WEAK_ONLY_FEEDBACK` replay.
3. Run one complete `COUPLED_FEEDBACK` replay.
4. Freeze/hash all runtime artifacts, then run
   `evaluate_with_frozen_control_alignment.py` for each feedback bag using the
   single Control transform.

Keep the full 39.96 s slice, all 396 scans, and all available output messages.
Do not select frames or accept candidates using GT. Do not extend to the 280 s
bag.

## Run outputs

Persist run outputs outside Git under:

```text
/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/
  shadow/
  weak_only_feedback/
  coupled_feedback/
```

For each run retain `topics.bag`, NDT determinism/diagnostic CSV, deskew CSV,
OOSM CSV, EKF runtime and prediction-lineage CSV, launch/roscore/player logs,
input/binary hash receipt, and run start/end times. The prior Control external
bag and diagnostics are read-only references.
