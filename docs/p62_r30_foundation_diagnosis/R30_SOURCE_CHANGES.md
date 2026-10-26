# One controlled repair; observation-interface issue remains explicit

START84428ee944d5f8732caf283a45d13a3bba79f742 contains the same core src as R29 formal CODE f66a55457ba3e63d55ad8d588fb2cf5b1ead169b. Remote fetch exactly matched the supplied HEAD. No other worktree was checked out/reset/overwritten.

Repair CODE02490a7ee7376baa76abff8495865a3a020c7210:

- `src/dog_prior_map_localization/include/dog_prior_map_localization/ndt_target_configuration.hpp`: small shared setter establishes resolution before setInputTarget builds Gaussian cells.
- `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`: both global-PCD and local-target setters call it. The latter already had resolution established in R29; its configured value remains.8m.
- `tests/ndt_target_resolution_contract_test.cpp` and package CMake: reproduce old PCL order, verify actual leaf sizes in first/replacement/tile target paths. Zero align calls.

Before evidence: grid_reproducer_before.txt records `configured_resolution=0.8 actual_leaf=1 1 1` and exit1. After: six core tests pass; target regression reports actual.8/.8/.8; full native node Release build and catkin CTest pass. CONTROLLED_REPAIR.patch retains exact changes. Old source is preserved by START and R29 commits, old binaries by their unchanged paths/hashes. New compiled executable identity is in FINAL_IDENTITY.json. No repaired complete trajectory is claimed.

Unchanged: NDT resolution/step/epsilon/iterations/outlier intended values; C+G yaw seeds/budget/quality thresholds; IMU/Deskew/EKF/OOSM mathematical source; local-map selector; extrinsics; raw assets. No global covariance or motion-model repair is inferred from source inspection. Full build links historical optional modules but no Coupled/D/E/F module was run.

The post-admission step limiter changes a quality-certified pose before publication. Its raw62 translation effect is exactly reproduced from original logs. This is NOT repaired here, because it would add a second observation/selection semantic change and needs an explicit contract. Do not read the successful Gaussian-grid regression as full foundation acceptance or as proof the original R29 trajectory was repaired.

Diagnostic code, logs and trace reducers live only under docs/p62_r30_foundation_diagnosis. Probe CODEe5dffad5b23485078e9ab17fdf4987107913626f predates the production repair; later post-processing tools have their archived checksum identities. Git START/probe CODE/repair CODE/archive END must remain separate.
