# PAPER-P2C Dataset Configuration Standard

Date: 2026-09-23

## Rule

Keep `config/dog_prior_map_localization_ndt.yaml` as the default/reference algorithm configuration. Dataset overlays contain only dataset identity, topics, frames, map path, scan timing, preprocessing, initialization, extrinsics, and evaluation-reference metadata. Do not tune or fork NDT/EKF algorithm parameters in a dataset overlay.

The split launch accepts `dataset_config`; when supplied, it loads the dataset overlay after the base config. `scan_reference_time` is left unset by default so that the dataset overlay owns the scan-time convention. An explicitly supplied launch argument still overrides it. These launch changes only select dataset values and do not change estimator algorithms.

## SuperLoc Corridor01

File: `config/superloc_corridor01.yaml`.

It records VLP-16/Epson topics and message type, the `cmu_rc2_velodyne` LiDAR frame, `epson` input IMU frame, map/normalized-map path, audited extrinsics, scan-start reference, per-point time semantics (`seconds_from_scan_start`), upstream full-SE(3) CV deskew, frozen baseline internal deskew disabled, first-segment initialization, and the official GT as post-run evaluation reference only. `GT_used=false` is explicit in the deskew configuration.

## Own Loop2

File: `config/own_loop2.yaml`.

This is an audit-based overlay, not a claim of completed motion compensation. It records the Livox topics, `offset_time` in nanoseconds from scan start, shared `livox_frame` convention, current map path, and current gravity initialization. Current own-loop2 deskew remains disabled (rotational and translational both false); no SuperLoc calibration or method is silently applied.

## Separation check

The default YAML checksum at P2C creation is recorded in `runtime_params.txt`. The new dataset overlays do not modify NDT resolution, iteration limits, filtering, step caps, EKF noise, OOSM behavior, or the frozen baseline workspace. The baseline workspace HEAD remains pinned at `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`.
