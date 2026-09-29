# A1 changed-file inventory

## R4 semantics

* `include/dog_prior_map_fastlio2_frontend_exp/measurement_noise_model.hpp`
* `src/measurement_noise_model.cpp`
* `include/dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp`
* `src/dual_reliability.cpp`
* `scripts/p6_i6f_measurement_noise_test.cpp`

## I6F foundation

* `include/.../window_factors.hpp`
* `include/.../fixed_lag_window.hpp`
* `include/.../fixed_lag_experiment.hpp`
* `src/window_imu_factor.cpp`
* `src/window_lidar_factor.cpp`
* `src/window_visual_factor.cpp`
* `src/window_marginalization.cpp`
* `src/fixed_lag_window.cpp`
* `src/fixed_lag_experiment.cpp`
* `scripts/p6_i6f_imu_preintegration_test.cpp`
* `scripts/p6_i6f_joint_window_test.cpp`
* `scripts/p6_i6b/CMakeLists.txt`

## Explicit mode and feedback boundary

* `config/ablation.yaml`
* `include/.../fastlio2_frontend.hpp`
* `src/fastlio2_frontend_ikfom.cpp`

The formal default remains `FORMAL_FULL_LEGACY`; no formal runtime behavior
was replaced.
