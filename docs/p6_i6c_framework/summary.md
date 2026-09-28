# PAPER-P6-I6C — Framework Integration

## Outcome

I6C integration and both authorized Corridor01 closed-loop runs completed. The framework is runnable, but this experiment does **not** show a useful Corridor01 localization improvement from U_nonlocal. This is a negative/descriptive result on the prepared Corridor01 v2 input, not a claim that the reliability method is generally ineffective.

Implementation branch: `research/p6-i6c-framework-integration`.

- Algorithm/executable source SHA used for both full runs: `ffe4cd82d28820f4abb74ae0990d9a97dfb51ba6`.
- Prepared input manifest SHA-256: `6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043`.
- Prepared bundle and both full-run outputs are under `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_i6c_framework/` (`input/`, `formal/`, and `vision/`).
- Ablation configuration SHA-256: `496129fdcd1089e3b88fc72956cb85bdca2f5de06580e3d348cb0b685d8d4d0a`.
- Corridor01 map SHA-256: `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.
- Raw bag SHA-256: `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.
- Derived bag SHA-256: `7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0`.
- Camera/IMU calibration SHA-256: `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`; camera intrinsics SHA-256: `083ff73553f6df25734bfddc439fbda7eb7b01c8baeede2b4949e960f72fd370`.
- GT-free initialization file SHA-256: `d3e6f560895cb4f6a9efb7058bcff8a13313783e799d82c828e51adcd4bafd1e`. GT was not used in either replay, initialization, mode selection, or state update.
- The frozen Floor01 I6B V1 result directory was not modified; the previous I6B artifacts remain byte-for-byte untouched in the repository.

## Input preparation and limits

The exporter consumes the existing Corridor01 adapter output. It reads 55,957 raw `/imu/data` samples and 2,776 adapter clouds; it excludes the established first 50 initialization clouds and replays the remaining 2,726. All timing uses ROS sensor-header stamps, never bag record time. The cloud topic is `/superloc_adapter/points_rot_only` (`cmu_rc2_velodyne`): it is rotationally deskewed only, receives no second deskew, and is paired with the official `T_imu_lidar`. Static initialization uses 200 causal IMU samples ending at the existing evaluation epoch; a held-input propagation of at most two median IMU periods brings the state to the exact epoch without using future IMU.

Important reproducibility limitation: the expected `corridor01_adapted_v1.bag` was not available. The run therefore uses the existing `corridor01_adapted_full_se3_v2.bag`, whose source SHA is pinned above. The selected `points_rot_only` stream has 28 per-cloud hash differences among 2,776 frames relative to the available v1 comparison record. Exact v1 parity must not be claimed. Per-cloud NDT source/request hashes are unavailable in the adapter; the bundle explicitly marks them unavailable (not as zero-valued hashes), while source bag and packed XYZ bundle SHA-256 are checked before every runner invocation and the C++ runner reports `source_cloud_hash_verified=0`.

The established Corridor01 evaluation epoch is first adapter LiDAR stamp +5 s. Full replay outputs cover all 2,726 scans. Post-hoc evaluation uses the existing fixed `PREFIX_10S` rigid SE(3) alignment with scale fixed to 1, as specified by `P2B_R1_EVALUATION_AUDIT.md`; it does not align on the full trajectory. The Corridor01 GT is SHA-256 `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03`. GT ends approximately 0.101 s before the final replay scan; no extrapolation is performed, leaving 2,725 timestamp-overlapping evaluation samples.

## Corridor01 closed-loop comparison

Both modes use the same shared I6B core, map, cloud/IMU data, GT-free initialization and strict NDT solver (`resolution=0.8`, `step=0.08`, `epsilon=1e-5`, `max_iterations=80`). Each replays all 2,726 frames with fresh propagation, predicted NDT initialization, NDT measurement and subsequent state update; no corrected poses were spliced offline.

| Mode | Translation RMSE / P95 / max (m) | Rotation RMSE / P95 / max (deg) | NDT nominal mean / P95 / max (ms) | All-align work mean / P95 / max (ms) | NDT calls; probes | NDT work (s) | Replay wall (s) | CPU user/system (s) | Peak RSS (MiB) | M0 nonconverged / prediction-only |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 172.5800 / 257.2316 / 260.3429 | 43.8422 / 49.9575 / 51.9220 | 75.91 / 172.66 / 207.28 | 75.91 / 172.66 / 207.28 | 2,726; 0 | 206.92 | 213.337 | 203.25 / 9.39 | 54.5 | 0 / 0 |
| UNONLOCAL_ONLY | 190.0265 / 409.0226 / 461.8789 | 119.8198 / 175.6852 / 179.9515 | 161.47 / 467.17 / 605.44 | 188.81 / 517.45 / 1709.63 | 3,522; 398 probe frames | 514.69 | 521.269 | 499.34 / 20.61 | 55.0 | 0 / 0 |

`All-align work` adds the measured nominal align and measured positive/negative probe-align section. UNONLOCAL_ONLY used 796 additional NDT align calls (two on each of 398 probe frames), a 1.292 calls/scan mean. Its measured NDT work was 2.49× STRICT and full replay wall time 2.44× STRICT. All 2,726 STRICT nominal calls converged. UNONLOCAL_ONLY decisions were 2,328 `NORMAL_UPDATE`, 220 `DIRECTIONAL_UPDATE`, and 178 `CAUTIOUS_UPDATE`; there were no nonconverged M0 calls or prediction-only commits.

Persistent post-hoc translation-error crossings (error continuously above threshold for ≥5 s, sample gaps ≤0.5 s; seconds from evaluation epoch):

| Mode | 0.25 m | 0.5 m | 1 m | 2 m | 5 m |
|---|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 3.875 | 9.120 | 9.523 | 10.128 | 11.137 |
| UNONLOCAL_ONLY | 3.875 | 8.918 | 9.321 | 10.027 | 10.935 |

On this prepared input, UNONLOCAL_ONLY worsened translation RMSE by about 10.1% (172.58→190.03 m) and rotation RMSE by about 173.2% (43.84→119.82°), while it did not materially delay any listed crossing. These large errors and the v2/v1 data discrepancy make the result a diagnostic, not a clean cross-dataset generalization claim. Do not call it proof that the reliability concept is invalid.

## U_obs focused gate

The requested limited step-size comparison was performed at `h=1e-4`, `5e-5`, and `2.5e-5`; the 10% criterion was not expanded. The two normal-coordinate nominal terminals yielded only 3/12 passing axes, so `U_obs` remains invalid and was not allowed to affect either full replay. On sample 0 all six coordinates fail; on sample 1 only `tx`, `ty`, and `tz` pass. The smaller step improves sample-1 `ry` (relative error about 1.08% at `5e-5`, 2.16% at `2.5e-5`), but sample-1 `rx` remains inconsistent and `rz` remains sign-inconsistent; sample 0 also remains inconsistent. No broader audit or tolerance tuning was done.

Consequently the ablation configuration exposes B0 and B2 as available. B1 and B3 remain unavailable because the U_obs gate fails. B4 is `NOT_AVAILABLE` because U_obs is invalid and the real visual geometry factor is not consumed by the filter core; a deliberate B4 dispatch returned exit 3 and `ndt_align_calls=0`. All profiles route through the same C++ core; disabling U_obs/U_nonlocal skips their actual gate/curvature or covariance/probe calculations and removes their update contribution.

## VisionAssist first usable layer

The offline front end processed all 6,720 raw camera image messages (6,719 adjacent pairs) using sensor header timestamps, OpenCV forward/backward LK tracking, normalized Mei bearings, deterministic translation-direction RANSAC with IMU-derived relative rotation, and two-view epipolar residuals. It emitted 6,594 timestamped geometry factors / normal-equation rows and analytic Jacobians with respect to camera-relative translation and left relative rotation. Pair processing time: mean 21.516 ms, P95 26.706 ms, max 136.971 ms.

Fusion deliberately remains disabled. Monocular two-view translation scale is unobservable, the current offline pose interpolation brackets image stamps with future scan-end states (noncausal), and the state-history-to-factor Jacobian chain has not been wired into IKFoM. These are explicit gates, not an unimplemented claim. No visual rotation or absolute map-position observation is fabricated. See the external output manifest for CSV SHA-256 and detailed per-pair results.

## Validation and preserved artifacts

- Release build of `p6_i6b_closed_loop` and `p6_i6b_dual_reliability_test`: PASS.
- CTest from `/tmp/p6-i6c-build`: 1/1 PASS.
- Python syntax checks for all five new I6C helpers and `git diff --check`: PASS.
- B4 unavailable-profile gate: PASS (correctly returns NOT_AVAILABLE before any NDT call).
- 10-frame smoke, B0/STRICT and B2/UNONLOCAL_ONLY: 10/10 output rows in trajectory, reliability and runtime files; passed the frozen manifest SHA, component SHA, source asset, timestamp/init contract and map guards.
- Full STRICT and UNONLOCAL_ONLY replay: 2,726/2,726 validated rows per output table; full runs passed the runner's integrity/convergence guards.
- `docs/p6_i6b_dual_reliability_v1/` and all I6A/I6B historical result artifacts were not modified. No bag, map, point cloud, camera image or full trajectory was added to Git.

## Historical results and workspace registry

`paper_experiments/EXPERIMENT_INDEX.csv` records I6A STRICT, I6B V0, I6B V1 and the two I6C formal modes. Historical runs remain `HISTORICAL_RESULT`; only this I6C comparison is tagged `FORMAL_ABLATION_RESULT`. Existing CSVs are referenced in place, not duplicated. `docs/WORKSPACE_REGISTRY.md` records the P6 repository's `git worktree list` plus the separately rooted `dog_visual_loc_ws`; no worktree was removed.

## Questions for the research controller

1. Should the Corridor01 comparison be repeated only if the exact v1 derived bag/cloud stream is recovered, given 28/2,776 per-cloud differences in the available v2 adapter output?
2. The GT-free initialization and adapter coordinate semantics are inherited from P2B, but the official map-to-GT world transform remains unresolved. What additional provenance is required before interpreting the very large PREFIX_10S-aligned full-trajectory drift scientifically?
3. Should I6C retain the current U_nonlocal trigger/risk outputs as an engineering diagnostic only, or is a separate solver/input correctness diagnosis now required before any joint decision policy is considered?
4. VisionAssist has real timestamped relative-geometry factors but no metric scale, causal image-time state history, or filter chain-rule fusion. Those gates should remain closed until the controller specifies a scale source and state-history semantics.

No I6D stage, online vision subscriber, parameter search, runtime algorithm change, or worktree cleanup was started.
