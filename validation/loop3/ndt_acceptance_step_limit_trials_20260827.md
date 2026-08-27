# NDT Acceptance and Step-Limit Trials - 2026-08-27

Goal: test whether `ndt_accept_max_translation`, `ndt_accept_max_rotation_deg`, and nearby step-limit thresholds can remove the remaining corner/stair mismatches without worsening full-bag loop3 accuracy against FASTLIO2Location.

Important: `ndt_accept_max_translation` and `ndt_accept_max_rotation_deg` only affect the independent NDT node when `ndt_acceptance_enable: true`. With the current default `false`, they are inactive.

## Results

| run | mean | p90 | p95 | max | 345-351 max | 398-403 max | 565-576 max | corrected Hz |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline | 0.417 | 1.243 | 1.535 | 2.694 | 2.429 | 1.781 | 2.694 | 9.98 |
| step055_rot5 | 0.412 | 1.278 | 1.537 | 2.702 | 2.702 | 1.828 | 2.678 | 9.96 |
| step060_rot6 | 0.381 | 1.041 | 1.516 | 2.818 | 0.637 | 2.818 | 1.104 | 9.96 |
| step060_rot5 | 0.429 | 1.353 | 1.544 | 3.783 | 2.377 | 3.783 | 2.727 | 9.94 |
| step058_rot55 | 15.081 | 57.154 | 72.581 | 87.719 | 0.905 | 25.873 | 87.384 | 9.87 |
| accept220_rot45 | 6.145 | 27.481 | 30.344 | 44.381 | 2.431 | 5.317 | 44.381 | 9.81 |
| accept220_rot20 | 12.554 | 43.017 | 64.453 | 71.071 | 2.866 | 24.773 | 70.907 | 9.69 |

## Diagnosis

- Hard acceptance gating is not a safe fix here. Even `2.20 m / 45 deg` rejects 22 frames and causes large downstream drift, because once a frame is rejected the next NDT result is compared to an older accepted pose and the apparent jump grows.
- The stricter `2.20 m / 20 deg` run rejects 75 frames and diverges more severely, so using `ndt_accept_max_rotation_deg` as a hard gate is especially risky in turns/stairs.
- `ndt_step_limit_max_translation: 0.60` with `ndt_step_limit_max_rotation_deg: 6.0` improves mean, p90, and two windows, but worsens the 398-403 s window and raises full-bag max to 2.818 m, so it is not a validated default replacement.
- `0.55/5`, `0.60/5`, and `0.58/5.5` are not better than the current baseline under the full-bag max criterion.

## Decision

- Keep the default config unchanged: `ndt_acceptance_enable: false`, `ndt_step_limit_max_translation: 0.5`, `ndt_step_limit_max_rotation_deg: 5.0`.
- Do not tune `ndt_accept_max_translation` / `ndt_accept_max_rotation_deg` further as hard rejection thresholds unless the acceptance logic is changed from hard reject to a soft prior-aware selector.
- If we want to address the two remaining corners, the next useful change should be algorithmic: a prior-aware soft candidate selector or segment smoother, not another scalar threshold sweep.
