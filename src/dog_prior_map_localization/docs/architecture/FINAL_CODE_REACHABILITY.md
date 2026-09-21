# Final delivery code reachability

This audit records the package surface used by the canonical delivery path.
The canonical launch is `launch/dog_prior_map_localization_split.launch` and
the canonical configuration is `config/dog_prior_map_localization_ndt.yaml`.
The independent runtime chain is NDT -> `/dog_livo/ndt_odom` -> EKF/OOSM,
with IMU propagation supplying the high-rate state.

## Runtime required

| Item | Role |
| --- | --- |
| `src/dog_prior_map_ndt_node.cpp` | Livox preprocessing, prior-map loading, NDT alignment, step limiting, output and determinism diagnostics |
| `src/dog_prior_map_ekf_node.cpp` | ROS entry point for the IMU/NDT EKF |
| `src/dog_prior_map_ekf_node_core.cpp` | Parameter setup, subscriptions, state and output orchestration |
| `src/imu_processor.cpp` | IMU propagation, gravity handling and state history insertion |
| `src/fusion/ndt_observation.cpp` | Timestamped NDT correction and OOSM replay dispatch |
| `src/core/state_history.cpp` | ROS-independent state history |
| `src/core/oosm_replay_planner.cpp` | ROS-independent rollback/replay planning |
| `src/ros_output.cpp` | Odometry, path, TF and runtime output |
| `include/dog_prior_map_localization/core/*.hpp` | Shared estimator and core contracts |
| `launch/dog_prior_map_localization_split.launch` | Canonical two-node launch (user-owned dirty copy is preserved) |
| `config/dog_prior_map_localization_ndt.yaml` | Canonical NDT + IMU + EKF/OOSM parameters |

`core/math_utils.hpp` contains the two inline helpers still used by the EKF
(`skew` and `limitVector`).  The unused Euler helper and its translation unit
were removed; no runtime math changed.

## Offline and validation tools

These are not linked into either runtime executable:

- `tools/offline/ndt_uncertainty_probe.cpp`
- `tools/offline/ndt_direction_alignment_probe.cpp`
- `scripts/evaluate_runtime_and_accuracy.py`
- `scripts/compare_odom_to_reference.py`
- `scripts/build_livox_keyframe_scan_db.py`
- `scripts/pcd_to_npz_map.py`
- `scripts/run_loop2_determinism_trial.sh`
- `scripts/run_loop3_absolute_ndt_compare.sh`
- `scripts/run_loop5_record_raw.sh`

They remain because they support map preparation, reproducibility checks, or
offline evaluation. The research probes are enabled only with
`DOG_PRIOR_BUILD_RESEARCH_TOOLS=ON`.

## Legacy or historical surface

The following profiles have no active canonical launch or runtime consumer and
are removed by FINAL-CLEANUP-5:

- `config/dog_light_odom_only.yaml`
- `config/dog_light_odom_only_tuned.yaml`
- `config/dog_prior_map_localization.yaml`
- `config/dog_prior_map_localization_kiss_external_prior.yaml`

The old integrated launch, Python EKF fallback, and deploy-light-odom scripts
remain historical files for this cleanup step because user-owned staged
evaluation scripts still reference them. They are not part of the canonical
build or launch and are explicitly deferred rather than modified:

```text
DEFERRED_DUE_TO_USER_STAGED_WORK
```

The dirty `launch/dog_prior_map_localization_split.launch` is likewise not
edited. Its `map/load_in_ekf` change is outside the canonical HEAD and remains
owned by the user.

## Configuration reachability

The canonical YAML retains only parameters read by the active NDT/EKF path,
plus the output and deterministic diagnostics needed for validation. Camera
intrinsics/extrinsics, image topics, camera update settings, disabled
directional-fusion state-machine parameters, and the removed NDT
information/Schur/degeneracy diagnostics are not part of the delivery
configuration.
