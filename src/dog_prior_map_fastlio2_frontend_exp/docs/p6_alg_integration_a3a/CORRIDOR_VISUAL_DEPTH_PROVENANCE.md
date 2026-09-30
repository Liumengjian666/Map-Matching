# Corridor visual depth: conservative formal rejection

Existing visual CSV: `docs/p6_i6d_full_algorithm/corridor01_metric_visual_causal_v2.csv`; SHA `9aea15ce31411c08b61d878dbc5ccd76df2c9ebed33d112c9119bd430e18bc67`; 482 valid pairs. No numerical values are regenerated or modified in A3A.

Current audited preparation source `scripts/p6_i6d_prepare_corridor01_visual.py` (SHA `bcccef3bc9e5434346b5d6b5594ba8a9b40f44332a994187a34062fc74d15fd5`) reads camera header stamps from the original raw bag and causal `/superloc_adapter/points_rot_only` depth from `corridor01_adapted_full_se3_v2.bag` (SHA `7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0`). It does not read GT. At the CSV's first artifact commit `9b8df60`, that script's SHA was `80da946e9c8dbd9be558f8c492c9b63a52964dca440d7b66cc2ba1063d313fd8`; current script adds quality columns absent from the frozen CSV. The old provenance file pins data/calibration identities but does not record the exact generation script hash. We do not pretend the current source hash proves execution history.

## Actual adapter call chain

Corridor wrapper includes the shared `/home/jian/livox_ws/superloc_adapter_ws/src/superloc_adapter/src/superloc_sensor_adapter.cpp`. Actual SHA `58c4073839247098b6f0f545065056be647336cc64631563f5d070d7dcd3b4e8` matches `derived/corridor01_adapted_full_se3_v2.meta.yaml`. `odom_history_in` defaults to `/dog_livo/ndt_odom`.

Raw IMU callback integrates gyro into q_delta. Fixed calibration and raw rotation yield `R_l0_li=R_ilᵀ R_i0_ii R_il`, `t_l0_li=R_ilᵀ(R_i0_ii t_il-t_il)`. Numerically `rot_point=R_l0_li*p+t_l0_li` does not use map/NDT position.

However, `processPendingLocked()` calls `hasRequiredPriorOdometryLocked()` before publishing either branch. After the first two scans it waits for prior corrected odometry poses. `processCloudLocked()` computes a full-SE3 branch using those prior poses/velocity; its nonfinite/exploded-point check can abort **both** full and rotational output. Thus sensor-local coordinate algebra alone is insufficient evidence that the complete persisted depth product is independent of legacy estimator state.

Decision: `CORRIDOR_VISUAL_FORMAL_INPUT_BLOCKED`. A3A sidecar lists all 482 existing pairs as **UNKNOWN**. This is an honest refusal to promote the existing product, not a claim that global pose was numerically multiplied into every rotational point. `MAP_POSE_USED=true` describes generation control flow; `MAP_POSE_USAGE=publication/selection dependency only, not rot_point coordinate formula` disambiguates it.

Formal V3 rejects these factors through its existing provenance gate. No source-independent raw-IMU-only depth product was rebuilt this round. The current CSV remains a compatibility/historical fixture. A clean independent generation chain would be needed before claiming RAW_SENSOR_LOCAL_DEPTH; merely writing a sidecar with a new name is not sufficient.

The blocked visual manifest records actual raw/depth bag, calibration, extrinsic, script, CSV and sidecar identities. GT_USED=false. This visual limitation does not block legal raw-LiDAR input export or sparse covariance closure.
