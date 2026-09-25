# PAPER-P3-R9C protocol

This is an offline, post-hoc accuracy and failure-timeline comparison between two formal Floor01 Run A closed-loop motion-compensation pipelines. It does not isolate point-wise deskew as a single causal variable and is not a novelty claim.

## Frozen inputs

- Paper source start: `2804be14cb78449d3dd8afefb915719082cb7e03` on `paper` and `origin/paper`.
- Frozen baseline: `41999ea700c66c4cadf0eca9e0c5d73caa2783fd` on `feature/visual-factor-window`; read-only.
- Legacy: R7H formal Run A, prior-NDT causal CV translation plus IMU gyro rotation deskew.
- Mature: R9B formal full Run A, EKF/IMU full-SE(3) deskew and closed-loop NDT correction.
- Same raw Floor01 input bag, normalized H1 map, official Floor01 SP1 calibration, Velodyne decoder lineage, and frozen NDT implementation/configuration are required. All absolute inputs and SHA-256 expectations are listed in `p3_r9c_manifest.csv`.
- Run B outputs are explicitly excluded.

## Gate before GT access

The analyzer checks hashes of all non-GT assets and audits actual R7H effective parameters against R9B's recorded runtime parameter dump and profile. The R7H adapter launch and Run A launch transcript are separately hash-verified; the legacy extrinsic is extracted from that launch and checked against the official calibration. The mature extrinsic comes from the hashed R9B profile, and the launch record's profile path, calibration-hash prefix, and runtime translation are checked. The old NDT source SHA is parsed from R7H run provenance and compared with the hashed R9B frozen-start source file; it is not compared against a duplicate copy of one manifest value. An unexpected NDT-affecting difference stops analysis before GT is opened. Expected pipeline changes are limited to full-SE(3) motion compensation, IMU state propagation / NDT feedback required by that closed loop, gyro-bias static initialization, and associated experimental plumbing.

The evaluator is imported from the hash-verified `r3b_evaluator` path in the manifest, so the external result-directory copy of this analyzer can be rerun without relying on its own directory being inside the source repository.

## Evaluation convention

The existing P3-R3B evaluator implementation is imported unchanged for quaternion SLERP, linear translation interpolation, strict no-extrapolation behavior, rotation-angle calculation, and the `> threshold`, `gap <= 0.25 s`, `duration >= 5 s` crossing rule. The R7I metadata supplies the frozen anchor and W0–W4 relative windows. A single exact common timestamp with GT support anchors both pipelines. The GT's documented origin is IMU; each estimated LiDAR pose is converted with `T_W_I = T_W_L * inverse(T_I_L)` using Floor01's `T_imu_lidar` calibration.

For each pipeline `X`:

```text
T_X_rel(k) = inverse(T_X(anchor)) * T_X(k)
T_GT_rel(k) = inverse(T_GT(anchor)) * T_GT(k)
T_err_X(k) = inverse(T_GT_rel(k)) * T_X_rel(k)
translation_error = norm(translation(T_err_X))
rotation_error = angle(rotation(T_err_X))
```

Primary paired estimates use the exact decimal timestamp intersection of R7H and R9B, restricted to GT-supported stamps. Each-pipeline full GT-supported samples are reported as secondary populations, still relative to the same common anchor. Raw NDT and `final_used` poses are kept separate. GT bracket width `<=0.25 s` is the fixed HQ subset; no extrapolation, alignment fit, or time-offset search is permitted.

## Frozen analysis choices

- R7I windows are inclusive and retain the original absolute-time anchor and relative bounds without looking at R9C errors.
- `RAW_IMU_SHOCK` is centered at the preidentified raw accelerometer-norm peak stamp `1660857533.043056`, with `±1.0 s`; the old `[138,168] s` broad interval is interpreted on the same R7I-relative axis and is converted to absolute stamps from its frozen anchor.
- Persistent crossings: thresholds `0.25, 0.5, 1, 2, 5 m`, strict `>`, continuity gap `<=0.25 s`, persistence `>=5 s`; these are recomputed on the common-anchor trajectory.
- Paired bootstrap: NumPy PCG64, seed `20260925`, 10,000 resamples, percentile 95% intervals. Ties use `1e-6 m` and `1e-6 deg` absolute paired-error tolerances.
- NDT fitness, iterations, convergence and limiter deltas are descriptive only, not correctness certificates.

## Scientific limits

This compares two closed-loop motion-compensation infrastructures. It does not establish deskew-only causality, physical-impact causality, a physical root cause, wrong mode, multimodality, Hessian/geometry degeneracy, or novelty. GT is post-hoc evaluation only and never enters localization execution or parameter selection. `P4_ALLOWED = NO`.
