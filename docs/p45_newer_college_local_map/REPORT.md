# P45 — Newer College local-target CONTROL NDT

**Measured result: `LOCAL_MAP_ACCURACY_AND_MEMORY_PASS` for the frozen 300-scan segment, with an important scope limitation: the live segment did not cross a tile boundary.** The manager’s bounded remote-tile replacement was separately exercised as a component test using a real frozen scan and an explicitly synthetic pose fixture; this is not evidence of a remote localization result.

## Git and build

- Repository: `https://github.com/Liumengjian666/Map-Matching`
- Branch: `research/p9-r4-heldout-visual-evidence`
- Start: `e02e7eb5a8d59c13e0fabdc5ed3c7b2b47b671cf`
- Final code/commit SHA is supplied in the execution handoff; no self-referential commit hash is embedded in this report.
- Build: `catkin_make --pkg dog_prior_map_localization --make-args -j2` succeeded. CMake emitted pre-existing optional VTK component warnings; all package targets, including the NDT node, tile builder and switch probe, built.
- XML validation, Python byte-compilation, `roslaunch --nodes`, output-wrapper path-with-spaces probe, and `git diff --check` passed.
- No push was performed. The final remote HEAD is reported in the handoff.
- Final built executable SHA256:
  - NDT node: `c5e028637a38fb8877486c9c7ef783e40de071e61c981b874d82db8bfcbb266c`
  - EKF node: `930a3ca96e178d63afd58874ca3974dadcbd87908e58e39a3d15c85807d25d2d`
  - sequence publisher: `024b8f679c16a325acb73bf6b4e91698f4a7a025dd47ed90de3cfb6fef565e66`
  - tile builder: `29b941bf9c8c3655d9f397872dffd96665d9db2a6653499e86feee82c4c5950a`
  - switch probe: `8606825c2f2918d74910461b2a32fe0cebaadb958a50f7edea53a04082e2b218`

## Map and local-target behavior

The P44 input map has 10,277,007 XYZ points. The offline builder repeats the frozen P44 preprocessing (finite XYZ filtering followed by two sequential global PCL VoxelGrid passes at 0.15 m) and obtains a 3,142,751-point target. It partitions that already-filtered target into 74 tiles of 20 m × 20 m in XY, preserving the full Z range; it does not independently downsample tiles. The manifest’s summed point count and multiset fingerprint are preserved, and each tile has an SHA256. Tile data are external to Git at `/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/tiles/` (about 38.0 MB).

For each scan, the manager receives the same preprocessed NDT source and the causal `map_T_lidar` initial guess that is passed to this frame’s `align`. It projects the source into map coordinates and selects intersecting tiles with a 10 m safety margin. A 30 m expanded selection is loaded on update to provide one-tile hysteresis. If the active set covers the required tiles, the same merged target and NDT target model are reused. Tile/hash/read/coverage errors fail closed. The current NDT result and GT are never used to select that frame’s map.

The local-mode startup log states `global XYZ was not loaded`; only manifest metadata is read at startup. The NDT node holds the active local tile target, not the 10.27M-point source map or 3.14M-point global target. The whole PCD is loaded only by the separate optional RViz visualization publisher. The first live update selected 21 tiles and 552,716 target points (estimated XYZ payload 8.4338 MiB, 17.59% of the global target).

## Frozen 300-scan result

The exact P44 segment was replayed once from the same first state: 300/300 publisher receipts, NDT acknowledgements, converged CONTROL alignments, geometry-gate passes, deskew outputs, EKF correction acknowledgements, and OOSM `APPLIED` results. There were zero future-IMU-after-scan-end violations, zero step-limited frames, and strictly increasing timestamps from `1583836591182590976` to `1583836621082438144` (29.899847168 s sensor span). No GT was read by the runtime; evaluation used the frozen P43 conditional Base-frame chain after runtime logs were frozen, with no trajectory alignment.

P45 EKF high-rate: translation RMSE/P95/max = **0.061612 / 0.077927 / 0.216396 m**; rotation RMSE/P95/max = **1.87495 / 2.55837 / 4.33185°**. Against P44 global-target values (0.067985 / 0.086808 / 0.119675 m; 1.81802 / 2.47857 / 4.21200°), translation RMSE and P95 improved, but the maximum error and maximum adjacent position jump increased. The jump was 0.202451 m near startup, versus 0.096566 m for P44; this is a real early correction/tail-risk counterexample and is not attributed to a tile switch. Rotation errors were slightly worse. The preset RMSE and P95 limits pass, but this segment alone does not establish long-run robustness.

P45 NDT align mean/P95/max including first frame was 8.334 / 12.763 / 84.497 ms; excluding startup it was 8.079 / 12.761 / 16.378 ms. P44 steady NDT was 7.086 / 10.956 / 16.988 ms, so P45 steady mean increased by about 14.0%, with similar high-tail latency. Deskew mean/P95/max was 21.114 / 26.087 / 31.927 ms.

The P45 NDT process reached `VmHWM=138,200 KiB` (about 135.0 MiB) by completion. P44’s recorded in-run `VmHWM` was 775,308 KiB (about 757.1 MiB), so the observed NDT-process high-water reduction is **82.2%**. These are per-process observations from different sampling arrangements, not a summed system-memory figure. P45 EKF `VmHWM` was 23,940 KiB. The formal run’s aggregate CPU monitor missed ROS child process groups, so no valid whole-run CPU total is claimed. The headless run wall time from first publish to final acknowledgement was 33.82 s, longer than the 29.90 s sensor span; `NOT_REALTIME_VALIDATED`.

There was one map update during the 300 scans (the initial tile load); all subsequent scans reused the same 21-tile target. Because the path did not exercise a live tile crossing, a bounded switch-probe executable was run with a frozen remote scan around +150 s and a deliberately synthetic pose fixture `(-30,-230,0)` to force selection outside the active cache. It replaced the target with 22 tiles / 1,186,101 points and reported 28.36 ms tile I/O plus 7.69 ms merge. The probe made zero NDT align calls and read zero GT. Its 444,332 KiB process HWM belongs to this standalone PCL component test, not the NDT node; it is not used as an NDT or total-system memory result. Thus manager-level tile replacement is exercised, but an end-to-end remote trajectory switch is not validated.

## RPE correction and RViz

The P44 evaluator’s 1 s RPE stepping bug was corrected once. Running the corrected evaluator on frozen P44 outputs leaves all ATE values bit-for-bit unchanged; P44 EKF-high-rate corrected 1 s RPE translation RMSE/P95 is 0.028663 / 0.043909 m and rotation RMSE/P95 is 0.95717 / 1.90798°. Legacy values are retained alongside corrected values in `corrected_rpe_metrics.json`.

The final one-scan RViz launch test created a previously nonexistent output directory, loaded the 10,277,007-point prior-map topic in the visualization-only process, published the 552,716-point local target and the real NDT-aligned scan, and completed a converged CONTROL align (4 iterations, fitness 0.030953, 98.941 ms). The screenshot is `rviz_acceptance.png`. With `rviz:=true`, the visualization process intentionally loads the full map and can dominate system memory; headless performance measurements use `rviz:=false`.

## Limits and final assessment

For the evaluated 300-scan region, local-target matching preserved and slightly improved translation RMSE/P95 while reducing NDT-process HWM by 82.2%, with about 14% more steady NDT mean time. This supports the low-memory local target path for this segment. It does **not** establish global long-run tile-transition behavior, a 100–150 MiB whole-system budget, 10 Hz end-to-end real-time performance, or deployment readiness on a robot. Initialization remains `CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`.

**FINAL_RESULT: `LOCAL_MAP_ACCURACY_AND_MEMORY_PASS` for the frozen segment; live tile transition, whole-system memory, and real-time behavior remain unvalidated.**
