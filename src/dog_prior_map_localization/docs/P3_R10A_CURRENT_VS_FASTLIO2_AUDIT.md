# P3-R10A Current-vs-FAST-LIO2 Frontend Audit

Status: pre-implementation source audit. No runtime source, configuration, or
launch file was changed for this audit. Dependency source/API is resolved by
the exact official FAST-LIO snapshot below. R9B is **not** a complete FAST-LIO2
inertial frontend.

## Reference and reuse boundary

- FAST-LIO reference: [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO)
- Pinned reference commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- Verified clean external checkout:
  `/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`
- Verified remote: `https://github.com/hku-mars/FAST_LIO.git`
- Required `use-ikfom.hpp`, toolkit headers, process functions, and generic
  iterated update API exist at that exact SHA.
- Studied files: `include/use-ikfom.hpp`, `src/IMU_Processing.hpp`,
  `src/laserMapping.cpp`, and the IKFoM ESEKF/manifold API.
- The previously investigated local candidate toolkit is under
  `/home/jian/fastlio_localization_ws/src/FAST_LIO_LOCALIZATION_NOETIC_20260807_195533/include/IKFoM_toolkit`,
  in checkout `davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC` at
  `7fae5d1f9c0d9f6a64c6ac936f2239b9ee9cb18d`. Its parent checkout's root
  `LICENSE` says GPL-2.0, its `package.xml` says BSD, and there is no separate
  license file inside `include/IKFoM_toolkit`; several individual headers carry
  BSD-3-Clause notices. The toolkit files `esekfom.hpp`, `S2.hpp`, `SOn.hpp`,
  `vect.hpp`, and `build_manifold.hpp` do not byte-match (even after CR removal)
  the official `hku-mars/IKFoM` files at commit
  `59cfc095ca74425f9b330c7c04a5d74f68c6dd62`. Exact code lineage and applicable
  reuse terms for this fork are therefore unresolved. It is explicitly **not**
  the R10A dependency.
- Project engineering policy treats the complete pinned FAST-LIO checkout as
  an external GPL-2.0 research dependency. Its source is not vendored. The R10A
  runtime belongs in the separate `dog_prior_map_fastlio2_frontend_exp`
  package marked `GPL-2.0-only`; the existing `dog_prior_map_localization`
  package remains MIT and must not have its license changed. This is the
  project owner's conservative engineering/distribution boundary, not a legal
  opinion.
- Direct FAST-LIO/IKFoM source copied into this project: **NO**.
- Dependency reused: **NOT YET** (candidate found; integration gate remains
  open). Do not vendor GPL source. Any external dependency arrangement must
  document GPL-2.0 and remain explicit; this audit is not a legal opinion.
- Permitted port boundary: project-specific implementation of the documented
  state/process/scan semantics. FAST-LIO point-to-plane measurement,
  ikd-tree map increment, and its raw-LiDAR iterated update are **not ported**;
  this project retains prior-map NDT as its LiDAR pose measurement.

## State and initialization

| Item | R9B current | R10A target contract |
|---|---|---|
| Nominal state | `p, v, R, ba, bg`; `g_` is a separate fixed vector. | Directly use pinned `state_ikfom`: 23 tangent DoF and 24 nominal dimensions, including `offset_R_L_I` and `offset_T_L_I` fields. Calibrated extrinsic values are fixed by an explicit constraint; they are not removed from or redefined outside the official state layout. |
| Manifold / error dimension | Eigen vectors and rotation matrix; 15D error state `[p(3), v(3), theta(3), ba(3), bg(3)]`. | Directly use exact snapshot `state_ikfom` from `use-ikfom.hpp`: 23 tangent DoF, including fixed-but-present 6-DoF extrinsic state, SO(3), and 2-DoF S2 gravity. `extrinsic_est_en=false`; do not redefine a reduced/lookalike state. |
| Gravity | Initialized by rotating `R_` so the static mean accelerometer aligns to world `+Z`; `g_=[0,0,-gravity_norm]` remains outside `P_` and receives no measurement correction. | Use exact FAST-LIO `S2<double,98090,10000,1>`: **2 tangent DoF and fixed magnitude 9.809 m/s²**; only direction is estimated. Its vector must remain in the same `camera_init` map frame `W` used by NDT. |
| Gyro bias | Optional static mean initialization (`initialize_gyro_bias_from_imu`; enabled in R9B); then propagated as a 3D bias state. | Initialize `bg=mean_gyr`; retain it in covariance and allow coupled NDT correction. |
| Accelerometer bias | Zero initialized; propagated as a 3D bias state. | Zero or prior initialized; retained in process/covariance and allowed coupled NDT correction; not fully estimated from static gravity samples. |
| Extrinsic | Fixed `T_imu_lidar` from the Floor01 profile. | Fixed, explicitly named `T_I_L`; official state fields remain present but are constrained to calibrated values, with covariance rows/columns decoupled from active state. Contract tests must prove its direction and invariance under generic pose updates. |
| IMU initialization | Running sums over the configured first 200 samples; R9B aligns attitude from mean acceleration and optionally sets `bg`; initializes the existing 15D covariance. | Port FAST-LIO `IMU_init()` semantics in the R10A package. Preserve the existing no-GT map-frame convention: compute `a_W,prior = R_WI,prior * mean_acc_I`, set `R_WI,0 = q_align(normalize(a_W,prior), +Z) * R_WI,prior`, then `g_W,0 = -R_WI,0 * normalize(mean_acc_I) * 9.809`. Current seed is identity. Do not load unresolved `world_darpa` pose or use GT. Contract-test frame relation and recompute Floor01 static statistics. |
| Acceleration scale | R9B records `ACC_SCALE_NOT_PORTED`; its prior static report measured `||mean_acc||=9.8195038998 m/s²`. This must be rechecked for R10A. | Do **not** mechanically port `G/||mean_acc||`; decide from SI-unit/topic evidence and new static measurements, never GT. |

## Propagation, noise, and scan timing

| Item | R9B current | R10A target contract |
|---|---|---|
| Process model | Project-specific midpoint/tail propagation, with optional velocity damping and a fixed world gravity. R9B enables acceleration integration and disables continuous gravity correction. | Directly call pinned `get_f`, `df_dx`, `df_dw`, and `kf.predict()` with official `state_ikfom`, `input_ikfom`, and `process_noise_ikfom`; do not rewrite F/G/manifold propagation. |
| State Jacobian F | Hand-coded discrete 15x15 blocks in `[p,v,theta,ba,bg]`; does not model gravity uncertainty. | Use exact pinned `df_dx` (24x23 nominal/tangent mapping) through the official IKFoM predictor. |
| Noise Jacobian G / process noise Q | No explicit `G`; hand-built block-diagonal discrete `Q` uses configured acceleration/gyro noise squared times `dt²`, bias random walk squared times `dt`. | Use exact pinned `df_dw` and its 12 channels (`ng`, `na`, `nbg`, `nba`); map the R9B engineering noise parameters transparently and do not tune from GT. |
| Interval input | R9B midpoint mode uses head/tail average for propagation and OOSM replay; scan-bounded fragments avoid using a post-scan sample. | One head/tail averaged input semantics shared by scan-local prediction, IMUpose construction, and OOSM replay. |
| Prediction organization | IMU callback advances the filter at each received sample; invalid/large `dt` is skipped/rebased. | A scan processing group contains one LiDAR frame and the IMU deque covering it; restore frame-start state, predict intervals forward to exact scan end, and retain each interval pose. |
| Scan reference | R9B Floor01 launch/profile defaults to scan start; code supports start/end selection. | New mode requires `SCAN_END_REFERENCE`; output cloud header, NDT pose timestamp, filter correction timestamp, and OOSM timestamp all equal exact scan end. |
| IMUpose / deskew | No scan-local `IMUpose` trajectory. Deskew waits for callback-generated state history and interpolates poses at point timestamps. It already applies full SE(3) point transforms and preserves original PointCloud2 index/order. | Build one scan-local forward `IMUpose` trajectory and use backward, point-time-specific full-SE(3) compensation into `L(t_end)`, with no post-end IMU sample. Preserve source point order by writing results to original indices. |
| Point time | Floor01 contract uses float `time`, seconds from scan start; `point_stamp=header.stamp+time`. R7E/R9B evidence is retained, but the R10A launch input must be revalidated. | Revalidate PointCloud2 field layout/time semantics before enabling the mode; use temporary time-sorted indices only if needed, restoring original acquisition order. |

## NDT measurement, covariance, and replay

| Item | R9B current | R10A target contract |
|---|---|---|
| NDT pose frame | NDT produces `T_W_L`; the deskew-enabled path converts it to IMU pose using fixed `T_I_L`. | Keep the NDT optimizer/configuration unchanged. Convert `T_W_L_NDT` to `T_W_I_NDT = T_W_L_NDT * inverse(T_I_L)` and verify with transform contracts. |
| Residual / update | Translation difference plus axis-angle of `R_target*R_pred^T`; apply-ratio and magnitude clamps; direct nominal `p/R` correction. A separate NDT-delta velocity blend is applied. | Six-DoF NDT pose measurement in an `R^3 × SO(3)` product manifold, SO(3) boxminus/log orientation residual, direct observation Jacobian only on pose tangent columns, and all other corrections only through full covariance cross-terms. Use the exact snapshot's generic iterated manifold update; never the FAST-LIO point-to-plane modified update. |
| Measurement covariance | No pose-measurement `R` used in a Kalman update. | Fixed, documented engineering-prior `R` (`sigma_p`, `sigma_rot`), independent of fitness and GT. |
| Covariance update | Prediction uses `P=F P Fᵀ+Q`; NDT update only scales selected pose diagonal entries by `0.85`; no cross-covariance measurement update. | Use exact pinned generic `update_iterated()` or `update_iterated_dyn_share()` for full 23-DoF state/covariance. Do not use `update_iterated_dyn_share_modified()`, which is the FAST-LIO point-to-plane specialization. Verify full-state cross-covariance injection and SO(3)/S2 reset with synthetic tests. |
| Bias/state correction | NDT does not update `v/bg/ba/gravity` through a Kalman gain; only the separate heuristic velocity blend affects `v`. | `v/bg/ba/gravity` are columns in the gain and may respond through cross-covariance. Log all correction deltas; claim only coupled-state participation, not bias observability/accuracy. |
| OOSM snapshot | Current snapshot contains `p,v,R,ba,bg,P`, raw and interval IMU metadata; gravity is omitted because it is external/fixed. Replay calls the custom propagation. | Roll back/replay the complete IKFoM state and covariance, including gravity and both biases, with the same interval model and scan-end observation semantics. |

## Required isolation

The current runtime has `deskew/mode` (`legacy_prior_ndt_cv` or
`ekf_imu_fastlio`), not the required three-way frontend selector. R10A must add a
separate opt-in `frontend/mode` for `fastlio2_esekf_ndt`, without changing the
canonical default, legacy path, or R9B custom path. R10A must use only its
explicitly named Floor01 experimental YAML and launch.

This document is the required pre-coding audit. Dependency source/API and
engineering license policy are resolved by R10A-D1. The pinned external
checkout is verified and will be consumed only by the separate GPL-2.0-only
experimental package through an explicit hard-pinned CMake path. No upstream
source is copied into this repository. This document does not assert that R10A
is implemented or validated. Add a no-GT contract test for the initial
`W`-frame gravity/attitude relationship before any scan-end prediction or map
update.
The D2 protocol closure additionally requires a candidate/shadow state,
prediction-only commit for the two ordinary NDT rejects, fail-stop for invalid
source and internal/protocol/filter failures, and exact transaction identity
including integer scan start/end nanoseconds. The companion protocol documents
record these rules. The initial pose and physical IMU/LiDAR offset remain
independent Floor01 runtime gates; the API spike does not close either gate.

The current Floor01 adapter/config identifies the NDT map frame as
`camera_init`; the present IMU initializer maps static mean acceleration to
world `+Z`. This is the explicit inherited frame convention to validate, not a
claim that the unresolved `world_darpa` pose file provides an alignment.
The pinned exact snapshot contains `update_iterated()`,
`update_iterated_dyn_share()`, and `update_iterated_dyn_share_modified()`.
R10A must use a generic pose-manifold update whose Jacobian spans the full
23-DoF state and must not use the point-to-plane modified update.

## Primary reference files

- [`use-ikfom.hpp` at the pinned FAST-LIO commit](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/include/use-ikfom.hpp)
- [`IMU_Processing.hpp` at the pinned FAST-LIO commit](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/IMU_Processing.hpp)
- [`laserMapping.cpp` at the pinned FAST-LIO commit](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/laserMapping.cpp)
- [`IKFoM` repository and license](https://github.com/hku-mars/IKFoM)
