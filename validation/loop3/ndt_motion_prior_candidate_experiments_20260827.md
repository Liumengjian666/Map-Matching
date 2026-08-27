# NDT Motion-Prior Candidate Experiments - 2026-08-27

Goal: test the user's proposed idea: when NDT has ambiguous matches around repeated corners, prefer solutions closer to the propagated motion prior, while still using only `/livox/lidar`, `/livox/imu`, `/clock`, and the prior map at runtime. FASTLIO2Location `/localization` is reference-only for full-bag offline evaluation.

## Disk cleanup

Before running new full-bag experiments, rejected historical result bags were removed from known failed run folders under `/home/jian/rosbag/loop3/`. Eval JSON/TXT, logs, and config files were retained. Free space improved from about `17G` to about `74G`.

## Code hooks added, default off

Added two experimental hooks to `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`, with all runtime-changing switches defaulting to `false` in `src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml`:

- `lidar_update/ndt_prior_candidate_enable`: run a small local candidate set around the propagated motion prior and choose by `NDT fitness + nearest-map residual + prior distance + yaw distance`.
- `lidar_update/ndt_motion_prior_guard_enable`: lightweight guard that keeps the motion prior when raw NDT moves far from the initial guess but the motion prior has comparable nearest-map residual.

Default config behavior remains the previous best NDT baseline because both switches are disabled.

## Baseline retained

Current best run remains `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`:

| Metric | Value |
| --- | ---: |
| mean | 0.417 m |
| p95 | 1.535 m |
| max | 2.694 m |
| corrected rate | about 10 Hz |

Important windows:

| Window | mean | p95 | max |
| --- | ---: | ---: | ---: |
| 345-351 s | 0.755 m | 2.321 m | 2.429 m |
| 398-403 s | 0.451 m | 1.465 m | 1.781 m |
| 565-576 s | 1.717 m | 2.581 m | 2.694 m |

## Full-bag experiments

| Run | Mode | mean | p95 | max | corrected rate | Decision |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `ndt_prior_candidate_v1_loop3_20260827_073817` | multi-NDT candidates, permissive trigger | 24.801 m | 97.224 m | 106.016 m | 7.46 Hz | reject |
| `ndt_prior_candidate_v2_loop3_20260827_075039` | multi-NDT candidates, stricter trigger | 13.143 m | 69.408 m | 77.004 m | 8.77 Hz | reject |
| `ndt_prior_candidate_v3_loop3_20260827_080308` | multi-NDT candidates with hard prior-distance gate | 11.472 m | 69.516 m | 81.974 m | 8.71 Hz | reject |
| `ndt_motion_prior_guard_v4_loop3_20260827_081544` | lightweight motion-prior guard, no extra NDT | 0.496 m | 1.558 m | 5.476 m | 9.86 Hz | reject |

Window comparison for the only compute-safe variant (`v4`):

| Run | 345-351 s max | 398-403 s max | 565-576 s max |
| --- | ---: | ---: | ---: |
| baseline | 2.429 m | 1.781 m | 2.694 m |
| motion-prior guard v4 | 2.419 m | 5.317 m | 5.476 m |

## Diagnosis

- Multi-NDT candidates are not suitable in the current 10 Hz node: even with few selected replacements, extra NDT calls drop the corrected stream to about `7.5-8.8 Hz`, causing worse downstream propagation and large trajectory divergence.
- The candidate cost is not enough to distinguish correct and wrong repeated-corner attractors; a candidate can be closer to the motion prior but still pull the subsequent chain into a bad corridor/corner.
- The lightweight guard preserves compute, but replacing raw NDT with the motion prior at a few late frames worsens the 398 s and 565 s windows. Pure residual comparison is too weak in repeated geometry.

## Decision

Do not enable these hooks in the default config. Keep the previous best NDT parameters unchanged. The hooks are retained only as default-off experimental code for future targeted diagnostics; they are not an accepted accuracy improvement.

Next promising direction is not more NDT local candidates in the online 10 Hz loop. A safer next experiment would be a separate offline/localization smoother that uses a short sliding window and validates temporal consistency over several frames before applying any correction, or a visual/semantic corner disambiguation cue if camera features are reliable in those two locations.
