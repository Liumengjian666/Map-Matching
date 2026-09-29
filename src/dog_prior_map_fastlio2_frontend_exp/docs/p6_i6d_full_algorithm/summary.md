# PAPER-P6-I6D-FULL-ALGORITHM-FIRST

## Outcome

The shared offline closed-loop core now runs the five profiles B0–B4 on both pinned datasets. U_obs is a regularized NDT-voxel geometric Gauss–Newton information proxy; U_nonlocal reuses the I6B V1 prior-direction M+/M− terminal probes; B4 adds causal metric LiDAR-depth PnP translation updates at image timestamps. No ROS runtime was changed and the stable `dog_visual_loc_ws` baseline was not modified.

This establishes a real end-to-end research prototype, not a successful localization result on both scenes. Floor01 B4 is essentially indistinguishable from STRICT. Corridor01 remains catastrophically inaccurate in every mode; Corridor01 B4 slightly worsens translation error while reducing rotation RMSE. No thresholds were retuned and no result was selected using GT.

## Implementation and mathematical scope

- `GeometricObservability` builds weighted `JᵀΣ⁻¹J` terms from the existing NDT target voxel means/covariances, with `J=[-[Rp]×, I]` in the map-spatial perturbation convention. Voxel covariance eigenvalues are checked and regularized before inversion; the normalized symmetric information proxy is checked for finite PSD structure. Its weakest rotation/translation block directions feed the existing directional measurement-noise decision path. This is a geometric Gauss–Newton proxy, not a calibrated posterior covariance or the exact PCL score Hessian.
- The legacy `PCL_SCORE_HESSIAN` path remains explicitly `INDETERMINATE`; the new geometric module is independent of that legacy finite-difference gate. The analytic geometric point Jacobian central-difference test and invalid-covariance cases pass. We do not claim to have established that the PCL score Hessian is a valid local-minimum certificate.
- U_nonlocal retains I6B V1's M0/M+/M−, prior-covariance principal perturbation, terminal response, fixed-objective, and call accounting. It contributes through the existing joint reliability decision and adaptive pose covariance rather than by adding Boolean flags.
- The visual factor uses only valid metric LiDAR-depth PnP translation rows. The camera-to-IMU conjugation preserves the P4-I3 transform direction; PnP rotation is discarded. Images, clouds, and IMU are ordered by sensor header timestamps. Past-only depth association is enforced (20 ms maximum age); Corridor01 never uses bag record time and does not deskew the already rotationally deskewed cloud a second time.
- Each reference image retains a causal filter snapshot. The factor becomes `p_ref + R_ref z_ref_cur`; its current-state Jacobian is the 3D position selector. The IKFoM update uses the live covariance, including current position cross-covariances to velocity/bias, Joseph covariance update, manifold covariance reset, finite/SPD checks, and no visual rotation update. Exact historical/current state cross-covariance is not retained by this minimal filter API. Instead, the code uses a conservative decorrelation covariance upper bound; this is a research-prototype approximation and must not be described as an exact augmented-state correlated update.
- `config/ablation.yaml` dispatches B0 STRICT, B1 U_obs, B2 U_nonlocal, B3 both, B4 FULL from the same runner/core. Disabled modules skip their calculation. B4 is marked available with the research-prototype covariance caveat.

The frozen Corridor01 v2 source manifest is `6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043`. Its official calibration parameter file and final causal visual factors are SHA-bound in the provenance files. Corridor01's rounded official LiDAR rotation was projected to SO(3) by the P2C prescribed SVD step; the original I6C v2 bundle was not changed. A preliminary factor CSV generated before the calibration-handling correction is retained as a diagnostic artifact, but is not used in the final B4 result.

## Smoke and full replay gates

| Dataset/run | Frames | U_obs valid/used | U_nonlocal probes | Visual factors / updates / nonzero | NDT aligns | M0 nonconverged | Wall / peak RSS |
|---|---:|---:|---:|---:|---:|---:|---:|
| Floor01 B4 partial smoke | 600 | 600 / 600 | 134 | 127 / 0 / 0 | 868 | 0 | recorded as partial smoke |
| Floor01 B4 full | 4127 | 4127 / 4127 | 1184 | 963 / 5 / 5 | 6495 | 0 | 191.335 s / 101.76 MiB |
| Corridor01 B4 v2 smoke | 600 | 214 / 214 | 205 | 127 / 26 / 26 | 1010 | 0 | 52.566 s / 55.70 MiB |
| Corridor01 B4 v2 full | 2726 | 214 / 214 | 290 | 482 / 26 / 26 | 3306 | 0 | 164.354 s / 55.58 MiB |

The Corridor01 smoke/full visual path passed its nonzero-update gate. Floor01's full B4 also passed the nonzero-update gate, though only five factors were admitted by the causal reliability trigger. The Floor01 600-frame partial run is not treated as a full vision exercise.

## Formal descriptive metrics

Each row uses the same dataset-specific frozen post-hoc protocol. Floor01 uses the fixed left anchor from the frozen I6A baseline and the official Floor01 GT. Corridor01 uses the established scale-free fixed 10-second `PREFIX_10S` SE(3) alignment and official Corridor01 GT. GT was read only after trajectory generation.

### Floor01

| Mode | Translation RMSE / P95 / max (m) | Rotation RMSE (deg) | U_obs used | U_nonlocal probes | Visual factors / updates | NDT calls | wall (s) / RSS (MiB) |
|---|---:|---:|---:|---:|---:|---:|---:|
| B0 STRICT | 0.88966 / 1.60597 / 1.96355 | 2.80108 | 0 | 0 | 0 / 0 | 4127 | 90.819 / 98.45 |
| B1 U_obs | 1.06885 / 2.27645 / 3.71785 | 3.64587 | 4127 | 0 | 0 / 0 | 4127 | 102.200 / 98.76 |
| B2 U_nonlocal | 0.88929 / 1.60046 / 1.95508 | 2.80341 | 0 | 1173 | 0 / 0 | 6473 | 176.888 / 98.52 |
| B3 U_obs + U_nonlocal | 0.93328 / 1.74429 / 2.51148 | 3.14568 | 4127 | 1195 | 0 / 0 | 6517 | 157.593 / 98.73 |
| B4 FULL | 0.88886 / 1.59821 / 1.97333 | 2.82164 | 4127 | 1184 | 963 / 5 | 6495 | 191.335 / 101.76 |

The STRICT replay reproduces the frozen I6A translation and rotation metrics. B4 changes translation RMSE by less than 0.001 m versus B0 and slightly worsens rotation RMSE; this is not a meaningful accuracy improvement. U_obs-only is worse on this trajectory. B2's translation improvement is negligible relative to the added probe cost.

### Corridor01

| Mode | Translation RMSE / P95 / max (m) | Rotation RMSE (deg) | U_obs used | U_nonlocal probes | Visual factors / updates | NDT calls | wall (s) / RSS (MiB) |
|---|---:|---:|---:|---:|---:|---:|---:|
| B0 STRICT | 279.870 / 419.246 / 431.251 | 113.473 | 0 | 0 | 0 / 0 | 2726 | 46.945 / 55.90 |
| B1 U_obs | 214.920 / 357.525 / 384.613 | 105.553 | 193 | 0 | 0 / 0 | 2726 | 316.807 / 55.06 |
| B2 U_nonlocal | 217.342 / 325.458 / 357.480 | 107.193 | 0 | 269 | 0 / 0 | 3264 | 376.384 / 55.79 |
| B3 U_obs + U_nonlocal | 229.604 / 330.503 / 335.697 | 109.590 | 190 | 266 | 0 / 0 | 3258 | 477.070 / 55.18 |
| B4 FULL v2 | 284.548 / 426.071 / 439.455 | 105.306 | 214 | 290 | 482 / 26 | 3306 | 164.354 / 55.58 |

Corridor01 remains unusable as a localization trajectory: every mode has hundreds of metres of translation error, and persistent threshold crossings happen within roughly 0–11 seconds of the evaluation start. B4 lowers rotation RMSE by about 8.17° versus B0, but translation RMSE rises by about 4.68 m and the visual module does not rescue the corridor trajectory. B1–B3 are descriptively less bad in translation, but still catastrophically inaccurate; this is not evidence of successful localization or a basis for tuning.

NDT mean/P95/max, total align work, CPU time, all threshold-crossing times, input hashes, and trajectory hashes are in `floor01_metrics.csv`, `corridor01_metrics.csv`, and their crossing companions. The main CPU extremes were 75.77 user + 14.60 system seconds for Floor01 B0, versus 122.77 + 25.16 seconds for Floor01 B4; and 39.11 + 7.57 seconds for Corridor01 B0, versus 149.72 + 13.89 seconds for Corridor01 B4. U_obs computation is especially costly on Corridor01 (B1 wall 316.8 s); B2/B3 also incur hundreds of extra NDT calls.

## Build and tests

- Release CMake targets `p6_i6b_closed_loop` and `p6_i6b_dual_reliability_test`: PASS.
- `p6_i6b_dual_reliability_test`: `P6_I6B_DUAL_RELIABILITY_TEST_PASS`, including geometric Jacobian finite-difference, PSD/null-direction, and invalid voxel-covariance checks.
- Catkin Release `ndt_pose_update_runtime_test`: `NDT_POSE_UPDATE_RUNTIME_CONTRACT_PASS`, including position-only IKFoM update and invalid covariance no-mutation checks.
- Python syntax checks for I6D data preparation, runner, frontend, and report scripts: PASS.
- `git diff --check`: PASS.
- The configured CTest directories reported “No tests were found”; the report therefore claims the standalone test executables above, not a passing CTest suite.

## Reproducibility, artifacts, and status

- Implementation/input commit: `9b8df604e11db1ac8919522130004ac321d0ad0c` (parent `c1fd2c10aade05f09e1e320e9f4bf49049ed7b41`). The experiment index binds every new formal row to this exact source commit.
- Full trajectories, per-scan reliability/runtime logs, `/usr/bin/time -v` resource records, smoke outputs, and B4 visual update rows remain in the corresponding external Floor01/Corridor01 result directories. No bag, PCD, or raw image was added to Git.
- `paper_experiments/EXPERIMENT_INDEX.csv` preserves prior I6A/I6B/I6C rows and appends ten `FORMAL_ABLATION_RESULT` rows (B0–B4 × two datasets). `docs/WORKSPACE_REGISTRY.md` records the currently used existing research worktree and all previously listed workspaces; none were deleted and no new I6D worktree was created.
- The active branch is `research/p6-i6d-full-algorithm`. The stable `dog_visual_loc_ws` worktree and the historical I6A/I6B/I6C experiment assets remain untouched.

## Decision-relevant limits

1. The vision update is genuine, timestamp-causal, metric, translation-only, and nonzero, but sparse (5 Floor01 and 26 Corridor01 updates). The historic/current cross-covariance is conservatively upper-bounded rather than represented exactly; a mathematically exact correlated multi-state visual factor remains open.
2. The Corridor01 catastrophic trajectory demands a separate diagnosis of the frozen prepared-input/frame/initialization and estimator behavior before claiming cross-scene viability. It is not appropriate to tune reliability thresholds from this GT result within this task.
3. U_obs is intentionally an information proxy, not a calibrated covariance. Floor01 B1/B3 degradation and Corridor01's very high U_obs runtime show that validity alone is not performance evidence.
4. The I6C Corridor01 `PRE_FULL_DIAGNOSTIC` numbers remain historical and are not merged with this I6D formal profile table.

This closes the authorized I6D implementation and two-dataset smoke/full execution. No I6E/I6F work or runtime integration was started.
