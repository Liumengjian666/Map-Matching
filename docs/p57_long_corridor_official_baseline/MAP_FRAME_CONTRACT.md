# Map, trajectory, and sensor-frame contract

Convention: `T_A_B` maps coordinates expressed in frame B into frame A.

## Verified asset facts

- The downloaded official PCD is binary PCD 0.7, 338,210 points, SHA-256 `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`.
- Header `VIEWPOINT` is `2.08626 -9.77624 0.0196389 -0.0844444 -0.00417902 -0.00205154 0.996417` (PCD convention: translation then quaternion). Existing P10 loader/source audits report that the localization loader uses XYZ and does not apply `VIEWPOINT` again. The header alone does not define the map frame.
- The official calibration file includes a `laser_to_imu` 4x4 matrix. Its hash `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d` matches the historically audited SuperLoc release. Existing official-code/schema review interprets this field as `T_imu_lidar`; the reverse transform is its inverse. This calibration does not establish the GT reference point or map/world relation.
- The official initialization YAML has a valid-looking `extrinsicRotation_world_darpa` and translation. Its rotation determinant and orthogonality were previously measured near a proper rotation. The adjacent `# s 67` is a comment; it does not itself define the time origin, sensor, or transform direction.

## Unresolved relations

| Relation | Status | Evidence / missing link |
|---|---|---|
| `T_imu_lidar` | Available from official `laser_to_imu`; interpreted as laser-to-IMU by the archived official-source review | Not re-applied or re-tested in P57 because no sensor bag was available. |
| `T_lidar_imu` | `inverse(T_imu_lidar)` if the official key interpretation is accepted | No separate measurement needed; not used in P57. |
| GT reference point (`L`, `I`, or another body point) | Not fully certified for the SuperLoc TUM export | Historical evidence ties pose samples to challenge GT and challenge rules use IMU reference, but the SuperLoc GT exporter/reference-point binding is absent. |
| `T_map_gt` | Unconfirmed | No publisher statement or source code located that binds the PCD XYZ frame to GT world with a fixed transform. |
| `T_map_lidar` | Unconfirmed for a given scan | Requires both `T_map_gt` and GT physical reference point/time plus the correct sensor extrinsic. |
| `T_map_imu` | Unconfirmed for a given scan | Requires `T_map_gt` and certified GT reference point. |
| `T_world_darpa` vs `T_darpa_world` | Unconfirmed | YAML field names do not establish transform direction; no official consumer code found for these keys. |
| `T_map_gt` from `corridor01.yaml` | Not accepted as fact | P36/P38 prior checks did not establish a valid 3D point/map overlap or official semantics. |

Previous P10 audits report GT bounds of approximately x `[-2.96, 244.50]`, y `[-0.06, 57.86]`, z `[-19.95, 0.31]`, while the PCD XYZ bounds are x `[-69.23, 68.44]`, y `[-77.57, 248.63]`, z `[-55.04, 85.35]`. Different axis-aligned bounds alone neither prove nor disprove a fixed rotation/translation. No trajectory-fit transform was estimated in P57.

**Evaluation rule:** do not report absolute ATE until the GT physical reference point, time association, and fixed map-to-GT transform are established. Do not apply `VIEWPOINT` or `world_darpa` by guesswork.
