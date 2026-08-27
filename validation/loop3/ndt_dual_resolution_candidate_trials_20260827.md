# NDT Dual-Resolution Candidate Trials - 2026-08-27

Goal: test an online soft candidate selector that keeps the current fine NDT as baseline, but on risky frames also runs a coarse NDT candidate using the old `12154e5`-style parameters (`source 0.35`, `target 0.25`, `resolution 1.0`, `max source 900`). Runtime inputs remain `/livox/lidar`, `/livox/imu`, `/clock`, and the prior map only.

## Code Change

- Added default-off `ndt_dual_resolution_candidate_enable` and related weights to the independent NDT node.
- The extra coarse NDT is only evaluated when the fine NDT result moves far from the propagated motion initial guess.
- Candidate selection uses a soft cost: NDT fitness, nearest-map residual, distance from motion initial guess, yaw deviation, and Z jump penalty.

## Full-Bag Results

| run | mean | p90 | p95 | max | 345-351 max | 398-403 max | 565-576 max | dual selections |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline | 0.417 | 1.243 | 1.535 | 2.694 | 2.429 | 1.781 | 2.694 | 0 |
| dual_v1 | 0.505 | 1.481 | 1.567 | 5.361 | 2.556 | 5.361 | 5.184 | 12 |
| dual_v2_strict | 0.434 | 1.442 | 1.551 | 2.716 | 2.551 | 1.780 | 2.716 | 5 |
| conf_step_v1 | 0.418 | 1.260 | 1.536 | 2.695 | 2.567 | 1.780 | 2.695 | 0 |

## Diagnosis

- `dual_v1` selected 12 coarse candidates and worsened full-bag max to `5.361 m`; one selected candidate had much lower NDT fitness but about `28 deg` yaw deviation, showing that coarse NDT can still choose a wrong attractor if only soft cost is used.
- `dual_v2_strict` selected only 5 candidates and stayed close to baseline, but still slightly worsened mean, p95, max, and the 345-351 s and 565-576 s windows.
- `conf_step_v1` was a conservative comparison using the existing confident step-limit hook; it is nearly identical to baseline but does not improve the remaining corner errors.

## Decision

- Keep the dual-resolution hook in code as a default-off experiment because it compiles and is useful diagnostic infrastructure.
- Do not enable `ndt_dual_resolution_candidate_enable` in the default config.
- The next improvement likely needs stronger rejection constraints for candidate selection, such as requiring the alternative candidate not to increase yaw/prior distance beyond the fine result, or using a short segment smoother instead of single-frame candidate replacement.
