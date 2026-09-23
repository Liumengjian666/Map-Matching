# Dataset configuration schema (P3-R3)

`own_loop2.yaml` and `superloc_corridor01.yaml` now use the same ordered top-level schema:

```text
dataset → frames → topics → map → lidar → imu → camera → extrinsics
        → preprocessing → initialization → ndt_observation → lidar_update
        → output → evaluation
```

The blocks remain present when a dataset lacks a sensor or calibration. Such values are written as explicit status strings (`UNAVAILABLE`, `NOT_PRESENT`, `NOT_APPLICABLE`) rather than fabricated identity transforms or YAML nulls. This also keeps ROS parameter loading safe.

## Runtime boundary

The dataset file is loaded after the generic YAML by `dog_prior_map_localization_split.launch`; explicit launch parameters are applied afterwards. The following dataset fields are intentionally runtime-facing:

| YAML path | Use |
|---|---|
| `frames.map_frame`, `frames.base_frame` | NDT odometry parent/child frame IDs |
| `topics.lidar`, `topics.imu`, `topics.lidar_msg_type` | NDT inputs |
| `map.pcd_fallback_path` | NDT prior-map path |
| `lidar_update.deskew_enable`, `lidar_update.scan_reference_time` | NDT cloud preprocessing/reference stamp |
| `topics.image`, `topics.camera_info` | Relevant only if the separately controlled camera frontend is enabled |

The sensor descriptions (`lidar`, `imu`, `camera`), calibration (`extrinsics`), upstream point-time/deskew description (`preprocessing`), evaluation reference, and fields named `source`, `status`, `available`, or `runtime_assumption` are metadata. The current split launch's `camera_enable` argument controls `camera_update/enable`; the descriptive `camera.enabled` field does not change that argument. The `ndt_observation` and `output` blocks are empty in both dataset overlays because the generic profile and launch own those runtime parameters.

No `dog_prior_map_localization/src/` or `include/` code changed in P3-R3. No compatibility loader was added.

## Transform convention

Every transform uses `T_target_source` and maps source-frame coordinates into target-frame coordinates:

```text
p_target = T_target_source * p_source
```

The uniform transform record has:

```yaml
T_target_source:
  status: "..."
  translation_m: [tx, ty, tz]
  rotation_row_major: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
  source: "..."
```

When calibration is genuinely unavailable, the same keys remain but values are status strings; they must not be parsed as a transform. In particular, the own MID-360 shared frame ID is only a runtime assumption; no physical `T_lidar_imu` was found, so it is not represented as identity.

### SuperLoc Corridor01

- Official `laser_to_imu` is interpreted as `T_imu_lidar`: `p_imu = T_imu_lidar * p_lidar`.
- The normalized config also records its inverse `T_lidar_imu`, used to convert an estimated LiDAR pose to an IMU pose via `T_map_imu = T_map_lidar * T_lidar_imu` if and only if a GT pose is later confirmed to be IMU-origin.
- `T_camera_lidar` is derived as `inverse(T_imu_camera) * T_imu_lidar` from the official camera-IMU and LiDAR-IMU calibration. The rounded source rotation matrices are projected to the nearest proper SO(3) before composing; the closure residual is at numerical precision (`1.11e-16` max absolute matrix entry).
- A fixed `T_map_world` was not found. The normalized PCD uses the configured estimator frame ID `camera_init`, not an asserted official GT-world identity.

### Own loop2

- Camera calibration source is the user's last active FAST-LIVO2/localization configuration and the raw `hikrobot_camera.yaml`.
- The source's raw `Rcl/Pcl` calibration defines `p_camera = T_camera_lidar * p_lidar`; the active localization config stores the inverse transform as `T_lidar_camera`/`T_base_camera` under the current `base≈LiDAR` assumption. The normalized config records the direct `T_camera_lidar` source result and does not relabel the inverse as direct.
- Numeric inverse-composition audit: the normalized direct transform matches the archived active FAST-LIVO2 `Rcl/Pcl` values exactly; composing it with the active localization config's `T_base_camera`/`R_base_camera` (with `T_base_lidar = I`) gives max absolute residual `9.46e-11`. The active inverse differs from a freshly computed inverse of archived `Rcl/Pcl` by `9.74e-11`, consistent with the stored values' decimal truncation.
- The active camera stream is rectified. The active algorithm intrinsics are the `projection_matrix` values; the raw `plumb_bob` coefficients are also preserved as raw-calibration metadata and are not applied again to the rectified stream.
- No separate physical LiDAR-IMU calibration was found for the own MID-360 data. `frames/imu_frame == frames/lidar_frame` documents runtime frame naming only.

## Camera calibration audit

| Dataset | Camera status | Intrinsic/model source | Distortion representation | Extrinsic status |
|---|---|---|---|---|
| own loop2 | Physical sensor present; current split launch defaults camera updates off | 720×540 rectified Pinhole, `/home/jian/.ros/camera_info/hikrobot_camera.yaml` projection matrix; same values in active config and `camera_ours.yaml` | Raw `plumb_bob` D is recorded separately; rectified topic uses zero effective distortion | `T_camera_lidar` available from active/archived user config; inverse composition checked |
| SuperLoc Corridor01 | Official RGB stream exists; current baseline disables visual updates | 640×480 MEI, official `corridor01_intrinsics.yaml` (`xi`, `gamma1/2`, `u0/v0`) | Official radial-tangential coefficients | Derived from official `T_imu_camera` and `T_imu_lidar`; camera frame ID is not asserted because the current bag audit did not establish it |

The own raw and rectified camera parameters are not contradictory: they describe different image domains. Their topic/model association is retained explicitly to avoid applying raw distortion twice.

## Schema and behavior checks

- Both files have the same 14 top-level keys in the same order and the same nested key names/order throughout the schema. Dataset-specific details occupy the same slots; unavailable values use explicit status strings.
- `preprocessing.deskew` also uses the same fields in both files. Own loop2 marks motion deskew disabled and identifies non-applicable method details; Corridor01 records the upstream `FULL_SE3_CV_DESKEW` model.
- Generic `dog_prior_map_localization_ndt.yaml` was not changed.
- The current Corridor01 run used upstream `FULL_SE3_CV_DESKEW` with scan-start reference; the NDT node's own `lidar_update/deskew_enable` remains false to prevent a second deskew.
- The own loop2 overlay truthfully records deskew disabled and retains its existing `scan_reference_time: mid` value.
- Config normalization is descriptive/structural only; it does not authorize runtime-algorithm, map, or bag changes.
