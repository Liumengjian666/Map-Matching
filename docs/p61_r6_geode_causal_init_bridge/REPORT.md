# P61-R6 — CAUSAL_INIT_ESTABLISHED

The real non-GT initialization bridge succeeded. R5 prior map is unchanged; the initializer was frozen before test LiDAR was opened. One raw-native NDT diagnostic then converged. No full return localization, EKF-localizer/OOSM integration run, Coupled/Recovery or GT evaluation occurred.

## Git and source

- Worktree `/home/jian/livox_ws/p61_r6_geode_init_bridge_ws`.
- Branch `research/p61-r6-geode-causal-init-bridge`.
- START_SHA `610f74e6503edad2c3bbf4eb865f0076b0d447eb` (R5 remote queried and confirmed).
- Bridge runtime CODE_SHA `0532c3b2d882327c7f22f57a53add6c7ceb11983`.
- Probe CODE_SHA `8751df36f19d4a72e09f9602afe603973028bd6b`.
- Author FAST-LIO remains `GEODE_Evaluation@1f008a7249e36393a1752622de50660b77b5b7f4`, same R5 numerical parameters and external Xsens calibration. Only an unchanged shared native integration block, terminal wrapper, raw arrival observation and private output plumbing were added to an internal research copy. No protected/production source was edited.
- Final END/remote/push receipt is generated after the archive commit at `/home/jian/livox_ws/p61_r6_geode_runs/final_git_receipt.json` and returned directly to the user. No push is authorized/executed here; R5 did not need another push.

## Gates A and B — causal input and common M

R5 prior PCD `/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd` has321053 points,10273947bytes, SHA256:
`4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4`.
Verified before preparation, after bridge, before/after NDT and after visualization. It was not modified.

Original internal bag SHA256:
`162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091`.
Filtered bridge bag:1372733659bytes; SHA256
`b2baeb7d91bafb48d51b580b41a33c8f4d324cce8bf3d95f9b1030acceb48fb8`.
It was independently re-traversed and contains2984 complete LiDAR scans and29844 measured Xsens IMUs only. Every delivered LiDAR serialized SHA matched the R2 ledger. No test LiDAR, GT or Livox internal IMU was delivered.

One fresh author instance/master replayed from the original start. It paused at ACK2441 and passed the direct R5 pose/time gate **before** receiving scan2442:

| Check | Actual result |
|---|---:|
| End-state translation difference | 0m |
| End-state rotation difference | 0deg |
| Same native scan-end time | true |
| Whole2441-frame prefix maximum translation difference | 0m |
| Whole2441-frame prefix maximum rotation difference | 0deg |
| Alignment applied | none |

After the gate,543 additional complete scans were processed:542 inside the strict middle window plus original mapping-boundary scan2442. That scan remains excluded from R5 PCD. Strict middle-window IMUs:5426. Newly published IMUs after the R5 state gate:5432, a distinct count including remaining first-window measurements. Total published/available raw IMUs:29844. See `time_contract.md` for exact interval/count definitions.

Complete run:2984/2984 ACKs; FIRST_SCAN1, IMU_INITIALIZATION1, TREE_INITIALIZED1, MAPPED2981. All recorded states are finite SE(3); maximum transform-chain error2.49e-16. No input skips after initial native stages, no future measurement and no GT. The instance's359920-point temporary map is retained only as its private state-bridge artifact; it is **not** the frozen prior and must not replace it.

Maximum adjacent translation is0.650159m; maximum adjacent rotation14.008047deg ends at scan2481. A bounded read of the already frozen nearby Xsens messages finds measured gyro norm up to2.513329rad/s and about14.177deg raw angular-norm integral over that interval. This supports a real rapid turn rather than interpreting the large angle alone as a discontinuity; it does not certify motion/pose accuracy. No motion threshold, source choice or parameter was changed on this basis.

## Actual initialization

Logical timestamp: **1706671587829840123ns**.

```text
T_M_L(test_start) =
[-0.9691186949  0.2433030141 -0.0401571744 328.5407325445]
[-0.2415420901 -0.9693867948 -0.0441209792  71.7864841038]
[-0.0496626018 -0.0330588180  0.9982187839   3.0360057482]
[ 0             0             0             1           ]
```

Position m: `(328.540732545,71.786484104,3.036005748)`.
RPY deg (`xyz`): `(-1.896817357,2.846628442,-166.004805826)`.
IMU origin velocity in M m/s: `(4.288392175,1.077925441,0.110519275)`.
Gyro bias in I rad/s: `(-0.000872524,0.002375748,-0.010976615)`.
Acceleration bias in I m/s²: `(0.029645063,-0.040965143,-0.005129251)`.
Gravity in M m/s²: `(0.033449617,0.546784016,-9.793691304)`.

Last contributing LiDAR: scan2984, start1706671587699897289ns, raw tail1706671587799724734ns. Scan2985 straddles the test boundary and was excluded. The terminal operation uses3 pending real IMU measurements and the unchanged native held pair-average integration to bridge30.115389ms to test start. Last raw IMU:1706671587824846982ns; native rounded header:1706671587824846983ns. Final hold after last raw sample:4.993141ms. No future sample, synthetic LiDAR or new motion model.

Actual23×23 filter covariance is archived, not fabricated. Symmetry error1.69775e-17, minimum eigenvalue1.39147e-7. Its state/chart definitions are documented separately; do not treat it as certified LiDAR pose uncertainty.

Raw initial-state SHA256:
`163d1c6fbeec969c3b950dbc6c8cd86a45327cc0fac90bdaaeed2f81449c06b2`.
Covariance SHA256:
`55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39`.
Freeze metadata SHA256:
`9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d`.

These establish a causal, numerical map-frame estimate, **not independently validated physical accuracy**. No GT-to-map transform or trajectory fit was used.

## Gate C — one bounded raw geometry/NDT probe

Only after initializer/covariance hashes were frozen, scan2986 was opened. Its header is70.524037ms after the initializer, with raw points through170.364222ms after it. This probe uses **raw, un-deskewed points and the earlier frozen t0 pose**; the time mismatch is explicitly retained. It is not a synchronized/fused localization Smoke or an accuracy test.

Input24000points; inherited baseline0.5–80m range,0.25m source voxel and1400-point cap. Frozen PCD remains unchanged; inherited0.15m×2 target preparation gives318377target points. Actual Gaussian leaf dimensions inspect as float32(0.8m), set before target. Step0.08,epsilon0.001,max40,outlier0.55 remain frozen.

| Diagnostic | Frozen initial projection | NDT output only |
|---|---:|---:|
| NN median m | 0.170427 | 0.145810 |
| NN P90 m | 0.455776 | 0.206305 |
| <=0.5m | 1286/1400 =91.8571% | 1375/1400 =98.2143% |
| <=1m | 1383/1400 =98.7857% | 1398/1400 =99.8571% |
| <=2m | 1399/1400 =99.9286% | 1400/1400 =100% |

Exactly1 align,1 convergence,0 failures; valid rigid output. Iterations5,fitness0.0350832553,align60.558422ms,target build28.236478ms. Correction0.316022247m and1.230863161deg. The output position `(328.456055,72.075661,3.131269)` belongs only to the raw diagnostic, **not** the earlier initializer. No measurement or candidate was fed back to any estimator.

The inherited derivative-only diagnostic made1 jet request,5.779268ms. No Weak/Coupled refinement or candidate search. Native score sum4546.493435,mean energy−3.24749531,Gaussian source support1378/1400=98.4286%. Normalized-chart Hessian eigenvalues:
`[2.09727183,2.50551110,4.35691184,54.87867726,4118.81626237,5310.88632258]`.
Ratio3.9490053e-4,condition2532.2833; translation-block ratio0.600806. Chart: translation0.8m per normalized unit, rotation radians, position `p0+0.8eta_t`, orientation `Exp(eta_r)R0`. This Hessian is not covariance. One sample's anisotropy does not establish a sustained degeneracy interval, a wrong match or a competing basin. No new threshold or stopping search was introduced.

## Coverage and acceptance limits

Original50 sample rows and hashes are preserved byte-for-byte in `fixed_50_coverage.csv`. We did not run a full test-segment LIO merely to acquire50 poses. Thus **POINTCLOUD_COVERAGE_NOT_CERTIFIED** remains for that set. The first-frame raw-at-earlier-t0 correspondence is a bounded diagnostic, not certified true geometric overlap and not a50-frame coverage result.

This remains a **same-recording time-held-out engineering experiment**, not independent cross-traversal localization. No GT error, rotationRMSE, full return availability or continuous drift metric is reported. NDT convergence and lower NN distances do not certify the true pose.

## Resources, review and visualization

Bridge wall223.058190s; mapper CPU213.957431s; actual peak RSS371.605469MiB including private map export. The separate single-frame probe process peaked79.574219MiB; its align60.56ms does not include all cold-start/I/O/jet/target-build overhead. These peaks are separate processes/runs and are not a simultaneous unique physical-memory sum or10Hz realtime certification. Internal disk had62GiB free initially, about59GiB after assets;20GiB reserve maintained. HIKVISION was never accessed.

7 Python contract tests, compiled native-time regression, literal native-integration-block comparison, full-prefix numerical comparison and frozen asset hashes passed. Read-only adversarial review found a real1ns raw/native timestamp equality bug before execution. It was fixed by adding only a raw integer arrival watermark, retaining native timing/integration. No actual bridge rerun or runtime repair was needed. External cross-model CLI review was offered but no CLI authorization/execution occurred.

Actual RViz snapshot review ran on a separate localhost11397 master using existing `pcl_ros/pcd_to_pointcloud`, TF and RViz. Topics/map frame and1400-point initial cloud were checked; real own-window screenshot is `rviz_snapshot.png`. The ROS1 EOL modal was dismissed only in this task's own RViz window. All task-owned visualization processes were then stopped. This is frozen diagnostic snapshot acceptance, not continuous localization acceptance; GUI memory is not included in the measured NDT-process peak.

## Commands and logs

From this worktree, actual bridge invocation:

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p61_r6_geode_runs/author_ws/devel/setup.bash
/usr/bin/python3 docs/p61_r6_geode_causal_init_bridge/run_bridge.py
```

The command intentionally refuses an existing `bridge_run` directory. Do not delete/overwrite this run to repeat it; future scientific reruns need a separate approved output identity. Full setup/build/test/probe commands are in `REPRODUCE.md`.

Safe repeatable snapshot-only viewer:

```bash
source /opt/ros/noetic/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11397 ROS_IP=127.0.0.1 roslaunch --port 11397 /home/jian/livox_ws/p61_r6_geode_init_bridge_ws/docs/p61_r6_geode_causal_init_bridge/review_snapshots.launch
```

For isolated visualization, use the private-master command recorded in `REPRODUCE.md`. Viewer does not execute NDT, LIO or replay.

Key internal logs: `/home/jian/livox_ws/p61_r6_geode_runs/bridge_controller.log`, `bridge_run/{mapper.log,replay.csv,mapping_trajectory.csv,initial_state.json,initial_covariance.csv,initialization_freeze.json,author_logs/}`, `probe.log`, `ndt_probe/{ndt.csv,geometry.csv,hessian.csv,grid.csv}`, `probe_resource.txt`, `visualization/launch.log`. Large bag/PCD/binaries remain internal, not Git.

## Decision and unique NEXT

The unique initialization blocker is resolved. The frozen map has demonstrated first-frame correspondences and one valid raw NDT result, sufficient to **request the next controlled baseline stage**, not to claim full return localization passed. There is no certified wrong-match or sustained severe-degeneracy finding here.

Unique NEXT proposal: separately approve20 real frames of Deskew+CONTROL NDT+EKF/OOSM fusion Smoke from this frozen non-GT initializer. Keep the map/parameters unchanged and report failures without resets. Full return replay still requires subsequent approval. This task has stopped.
