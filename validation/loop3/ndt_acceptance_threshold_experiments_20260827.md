# NDT Acceptance Threshold Experiments - 2026-08-27

Goal: test whether `lidar_update/ndt_accept_max_translation` and `lidar_update/ndt_accept_max_rotation_deg` can remove the remaining repeated-corner mismatches.

Important detail: these two parameters have no runtime effect unless `lidar_update/ndt_acceptance_enable: true` is also set. The default config keeps `ndt_acceptance_enable: false`, so the current best baseline is preserved.

## Baseline

Current best run remains `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`:

| Metric | Value |
| --- | ---: |
| mean | 0.417 m |
| RMSE | 0.618 m |
| p90 | 1.243 m |
| p95 | 1.535 m |
| max | 2.694 m |
| corrected rate | about 10 Hz |

Baseline diagnostic scan:

- Across the full bag, only 7 frames have `previous_to_result_translation > 1.2 m`.
- Only 5 frames exceed `1.5 m`.
- Only 2 frames exceed `1.8 m`.
- No frames exceed `2.2 m`.
- The largest baseline raw NDT step is about `2.07 m`; largest raw rotation is about `39.8 deg`.

## Full-bag experiments

| Run | Acceptance settings | mean | p95 | max | corrected rate | Decision |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `ndt_accept_gate_120_12_loop3_20260827_174518` | enable acceptance, translation `1.20 m`, rotation `12 deg`, fitness `3.0` | 44.891 m | 101.813 m | 109.825 m | 4.18 Hz | reject |
| `ndt_accept_trans180_nofitness_loop3_20260827_175624` | enable acceptance, translation `1.80 m`, rotation `45 deg`, fitness disabled | 0.437 m | 1.547 m | 3.755 m | 9.96 Hz | reject |
| `ndt_accept_trans205_nofitness_loop3_20260827_180710` | enable acceptance, translation `2.05 m`, rotation `45 deg`, fitness disabled | 0.434 m | 1.545 m | 3.755 m | 9.98 Hz | reject |

Window comparison for the least intrusive acceptance run (`2.05 m`, no fitness gate):

| Run | 345-351 s max | 398-403 s max | 565-576 s max |
| --- | ---: | ---: | ---: |
| baseline | 2.429 m | 1.781 m | 2.694 m |
| accept trans 2.05 | 2.552 m | 1.781 m | 2.694 m |

## Diagnosis

- `ndt_accept_max_translation` / `ndt_accept_max_rotation_deg` are hard rejection gates, not soft constraints.
- When the default fitness and rotation gates are enabled, many frames are rejected, the corrected stream drops to about `4.18 Hz`, and the trajectory diverges badly.
- When only very large translation jumps are rejected, the algorithm still worsens: the rejected frames near 349-350 s and 400 s appear to be part of the current baseline's recovery path, so rejecting them creates a larger downstream error.
- A threshold above `2.2 m` would effectively accept all baseline frames and therefore be equivalent to leaving acceptance disabled.

## Decision

Do not enable NDT acceptance gating in the default config. Keep `ndt_acceptance_enable: false`, `ndt_accept_max_translation: 1.20`, and `ndt_accept_max_rotation_deg: 12.0` as inactive/default-off parameters. The best validated behavior remains the existing step-limited baseline.
