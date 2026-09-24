# P3-R9A experiment protocol and asset lineage

## Scope

This stage is an engineering infrastructure port of mature LIO-style IMU
propagation and point-wise full-SE(3) LiDAR deskew. It is not P4, a novelty
claim, an accuracy experiment, or a physical failure attribution. No GT was
read by the runtime, window selector, or point-cloud comparison.

## Immutable source assets

| Asset | Absolute path | SHA-256 | Size |
|---|---|---|---:|
| Floor01 raw canonical bag | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag` | `6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b` | 413,681,649 bytes |
| Floor01 H1 map | `/tmp/floor01_candidates/floor01_h1_map.pcd` | `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570` | 6,595,818 bytes |
| Floor01 extrinsics | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml` | `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414` | 919 bytes |
| Official VLP16 decoder calibration | `/home/jian/livox_ws/ndt_landscape_ws/third_party/velodyne/velodyne_pointcloud/params/VLP16db.yaml` | Read-only ROS driver asset; decoder lineage below | — |

Bag metadata: duration 417.621513 s; 83,342 IMU messages and 4,139 packet scans.
Topics in the bag are `/input/imu` (`sensor_msgs/Imu`) and
`/input/velodyne_packets` (`velodyne_msgs/VelodyneScan`). The packet decoder is
`ros-drivers/velodyne` 1.7.0, commit
`89faa698688a48d4a5080f73c04d7aed8117eec0`. Floor01 calibration convention is
`p_imu = T_imu_lidar * p_lidar`; the source YAML supplies translation
`[0.08, 0.029, 0.03] m` and the rotation recorded in the experimental dataset
YAML. No Corridor01 or MID360 extrinsic was substituted.

FAST-LIO source audit used the read-only checkout
`/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`, commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`, upstream
`https://github.com/hku-mars/FAST_LIO`, GPLv2. The package is not a Git
checkout, so there was no source edit to it. No FAST-LIO code was copied.

The frozen baseline `/home/jian/livox_ws/dog_visual_loc_ws` remained read-only;
HEAD at close was `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`. Its pre-existing
staged/dirty inventory was left untouched.

## Window selection, before localization inspection

All offsets are relative to first IMU header stamp `1660857392.515903950`.

- Normal/startup window: `[0,30) s`. It preserves the sequence-start gravity
  initialization and was designated before looking at NDT results. IMU-only
  summary used during selection: gyro-norm mean `0.065228 rad/s`, deviation of
  specific-force norm mean `0.629598 m/s²`, jerk proxy mean `151.694345` in the
  selector's units.
- High-dynamic window: `[138,168) s`, selected using only IMU gyro norm,
  specific-force norm variation, and jerk proxy. The selector summary was
  `0.530003 rad/s`, `2.611392 m/s²`, and `693.030824`, respectively.
- A lower-activity candidate `[387,417) s` had `0.066202 rad/s`,
  `0.520201 m/s²`, and `127.866752`; the normal smoke instead uses the sequence
  start so the same replay includes the actual initialization boundary. The
  selection was not changed after seeing localization results.

First 200 IMU samples span 1.004928 s. Accelerometer norm mean/std is
`9.819504 / 0.009397 m/s²`; gyro norm mean/max is
`0.002255 / 0.006741 rad/s`. This supports `INIT_STATIC_SUPPORTED` for that
initialization interval. It does not add gyro-bias estimation: the existing
zero-initialized `bg` strategy remains in use.

## Executed replays

Successful IMU-deskew runs, each from sequence start through 168 s:

- Run A: `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/run_a_v3/`
- Run B: `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/run_b_v1/`
- Locked recapture Run C: `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/run_c_locked_v1/`

The locked capture was necessary because the first capture helper revision
shared one binary file across two ROS subscriber callback threads. Its CSV
hash fields were still valid, but a subset of binary offsets did not point to a
complete record. The helper now serializes offset, record, and index writes
with a mutex. Run C's 584 normal-window and 589 high-window records all passed
independent stamp/count/SHA verification. The legacy pointwise comparator was
also recaptured with the locked helper:

`/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/legacy_cv_168s_locked_v1/`

Two earlier attempts (`run_a` readiness failure and `run_a_v2` incorrect
positional topic arguments) did not play sensor data; they are preserved and
excluded from results. No old output directory was overwritten.

Both replay families used the same canonical bag start and 1x ROS-time replay.
IMU mode command shape:

```bash
env -u V_API_KEY roslaunch dog_prior_map_localization \
  p3_r9a_floor01_imu_deskew_experimental.launch \
  rviz:=false reference_time:=start \
  deskew_csv_path:=<run>/deskew.csv \
  runtime_csv_path:=<run>/runtime.csv \
  ndt_diagnostics_csv_path:=<run>/ndt.csv \
  oosm_csv_path:=<run>/oosm.csv

env -u V_API_KEY rosbag play --clock -r 1.0 --start 0.0 \
  --duration 168.0 \
  "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag"
```

The legacy-CV comparator used the preserved read-only adapter launch
`p3_r7h_floor01_closed_loop.launch`, `/tmp/floor01_ndt.yaml`, the same packet
decoder, and relays `/input/imu -> /cmu_sp1/imu/data` and
`/input/velodyne_packets -> /cmu_sp1/velodyne_packets`; it also replayed 168 s
from sequence start. Its output is only a registration-input engineering
comparator, not a reference trajectory.

Run A/B/C used the same experimental YAML with `reference_time:=start`.
A separate 30 s smoke used `reference_time:=end`; all 285 published deskew
references equaled `scan_end`, and all 285 NDT input header stamps matched the
published reference stamps. No RViz, GT topic, or evaluator was started.

## Capture integrity and large derived arrays

The locked XYZ stream records are ordered by callback arrival, but their index
rows carry the exact topic/stamp/count and byte offset. The generator verifies
each binary record header and recomputes its SHA-256 before using the arrays;
it then pairs points strictly by their original array index, without sorting.

| Array | Absolute path | SHA-256 | Size |
|---|---|---|---:|
| New normal-window arrays | `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/run_c_locked_v1/normal_clouds.xyzbin` | `80f83359b5caf9140af40784154371b1ee0707b5eb0f20eb58fedc7a963d665f` | 198,997,440 bytes |
| New high-dynamic arrays | `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/run_c_locked_v1/high_dynamic_clouds.xyzbin` | `73d011617139bd6199327cb5ef1d0059c14a3118ad1ac315663e137e81072cae` | 157,801,788 bytes |
| Legacy normal-window arrays | `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/legacy_cv_168s_locked_v1/normal_clouds.xyzbin` | `b703fb5895144c1ce471a3dab04c05ad3e597838ca3c5c0c6c6d7fa961e488b7` | 201,380,904 bytes |
| Legacy high-dynamic arrays | `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a/legacy_cv_168s_locked_v1/high_dynamic_clouds.xyzbin` | `b3c54bb1142c74f876b5f5614aefd25e1fae9f9c60f17ede997cd5d2ea28fcca` | 157,801,788 bytes |

The `.bag`, `.pcd`, `.xyzbin`, and full playback logs remain outside Git. Only
small CSV/Markdown review products are copied into this review snapshot.

## Reproduction of review products

```bash
cd /home/jian/livox_ws/dog_loc_paper_ws
python3 src/dog_prior_map_localization/docs/p3_r9a_artifacts/generate_p3_r9a_review.py
```

The generator rejects malformed capture offsets or a failed record hash, then
writes the required window, determinism, causality, and resource CSVs into the
external result directory and this small Git review snapshot.
