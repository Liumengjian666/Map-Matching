# I1 current code map

Audit base: `739079facf5a298d160b7efc643d17b3839ef975`, 2026-10-02.
The initial `research/p7-single-state-innovation` tree was clean; fetched origin
agreed with that SHA. Work continues on `research/i1-uobs-mature-integration`.
This document records inspected implementation, not an intended architecture.

## Formal path and ownership

`scripts/p7_run_corridor01.py` defaults to **FULL_POSE** and invokes
`scripts/p7/p7_single_state_runner.cpp`. The ROS frontend/external NDT server
and historical P6 giant runner are separate paths, not this experiment entry.

| Owner | Persistent objects | Per-frame work |
|---|---|---|
| Runner | Parameters, all IMU/scan metadata, output streams, one frontend, one registration object; optional admission counter | Causal IMU window, packed float XYZ cloud, results, classification, selected mode policy |
| `FastLio2IkfomFrontend::Impl` | One pinned IKFoM filter state and covariance, fixed extrinsics, noise | IMU prediction, existing measurement update, postconditions; no state window |
| `CurrentFrameNdtRegistration::Impl` | Final target cloud and one `ObservableNdt`/PCL target covariance grid | Source preparation, one alignment, raw terminal, frame-local geometric observations |

The target map is loaded once. Raw map/finite/first-downsample temporaries are
released. PCL retains the current source until replacement; observations are
not accumulated. Input metadata and IMU are loaded for the sequence, so total
memory is not strictly independent of sequence length.

## Actual scan call chain

1. Read frozen metadata and 27-scalar calibration/noise schema. Initialize from
   causal static IMU; check epoch gap and use held input to reach the epoch.
2. `imuWindow` -> `predictImuSequence` -> state exactly at scan timestamp.
3. Compose predicted `map_T_lidar = map_T_imu * T_imu_lidar`.
4. Read XYZ float triplets by offset; `align` prepares source once and uses this
   prediction as its sole seed. No saved predictor/corrected pose is read.
5. Classify NDT terminal. Only SUCCESS computes geometric U_obs at the **raw
   terminal**, then the runner calls `classifyFixedPhysicalJointSubspace`.
6. Apply the mode policy below; successful measurement uses
   `lidarMeasurementToImu` and existing `applyPoseMeasurement`.
7. Check timestamp and state/covariance postconditions; serialize diagnostics.

No per-point timestamp is present in this packed XYZ input. This runner is not
performing new online scan deskew; the preserved scan-end deskew module belongs
to another runtime path. No GT, visual, U_nonlocal or window state is used here.

| Mode | U_obs state effect | Measurement policy | Status |
|---|---|---|---|
| FULL_POSE (default) | None; shadow diagnostics only | SUCCESS raw pose, full 6-DoF update | Current carrier; NOT a validated stable full-Corridor baseline |
| MATURE_SOL_REMAP | Reliable-basis projector changes measurement mean | Invalid/rank-zero -> prediction-only; otherwise remapped full-pose update | P7-D comparison asset; covariance is not direction-selective |
| MATURE_ADMISSION | None | Terminal, <=3 m, existing rank-6 exact-residual NIS <=16.812; fifth consecutive rejection -> LOST | P7-E fail-safe comparison, not default |

P7-D/E are retained unchanged. Recorded D full attempts failed at tx1682/1543;
E stopped normally LOST at tx60. These are historical results, not I1 A/B data.

## U_obs producer and contracts

`current_frame_ndt.cpp::ObservableNdt::geometricObservations` queries the same
prepared source and same target grid, with cleared output vectors per query.
For each source/leaf pair, it forms `Rp`, residual `Rp+t-mean`, leaf covariance
and `exp(-distance_squared/(2*resolution_squared))`. Support counts pairs,
not unique points; multiple neighboring leaves are not independent samples.

`reliability_metrics.cpp::analyzeGeometricObservability` forms
`J=[-skew(Rp), I]` and **weight-averaged**
`H_phys = sum(w J^T Sigma^-1 J)/sum(w)`. The coordinates are map-spatial
rotation and additive map translation about the LiDAR origin, not an arbitrary
SE(3) left-twist chart. Covariance inversion uses absolute 1e-6 m² and relative
1e-3 floors. This is a geometric information proxy, not calibrated posterior
information, and not a proof of full inertial-system observability.

With run-fixed `L=parameters.resolution_m=0.8 m`, `D=diag(I,L I)` and
`H_bar=D^T H_phys D`, the classifier directly partitions its full 6D eigenbasis
at `max(numerical_floor, 0.05*lambda_max)`. Coupled rotation/translation modes
remain intact. Schur is diagnostic-only. The historical per-block equalization
in `dual_reliability.cpp` is not linked into the P7 target.

## Filter distinction important for architecture selection

The public frontend exposes both full-pose and projected update APIs. The
current runner's `applyPoseMeasurement` calls pinned IKFoM `update_iterated`.
The separate projected path implements Joseph/reset handling, checks Euclidean
orthonormal measurement columns, and supports exact SO3 residual linearization.
It is **not** called for state updates in the current runner. E reuses only its
const innovation evaluation. There is no general-linear-projection API from
the superseded P7-D proposal. Do not describe all updates as Joseph updates.

`registration_geometry` provides SO3 helpers and an audited registration-to-
pose-residual Jacobian. The runner does not use that Jacobian for P7-D mean
remapping. Its output chart is `[position map, right/body SO3 residual]`, unlike
the U_obs input chart. These bases cannot be passed between APIs unchanged.

## Source-grounded numerical caveats (not silently repaired)

Installed PCL is 1.10. In
`/usr/include/pcl-1.10/pcl/filters/impl/voxel_grid_covariance.hpp`, `applyFilter`
already modifies `leaf.cov_` by an eigenvalue floor before `getCov()` returns it.
The default multiplier in `voxel_grid_covariance.h` is **0.01**. P7's later
1e-3 relative floor cannot undo that earlier regularization. Therefore “raw
target voxel covariance” is inaccurate: U_obs consumes PCL's regularized cell
covariance, with possible information/degeneracy consequences to audit.

`loadMap` calls `setInputTarget` before `setResolution(0.8)`. Installed PCL's
constructor sets resolution=1.0; `setInputTarget` immediately initializes the
grid. `setResolution` only reinitializes when a source already exists, which
is false at map load. `computeTransformation` does not rebuild the grid.
An independent installed-PCL probe confirmed **1.0 m target cells with configured
0.8 m search/score/L**, including after `align`. This is an inherited baseline
contract issue, not evidence by itself for the cause of P7-D loss of tracking.
No parameter, ordering or production implementation was changed during I1 audit.

Probe: `/tmp/i1_reference_audit.1Mep3d/pcl_grid_probe.cpp`, compiled with the
existing include/library locations (no dependency installation). It subclasses
PCL NDT only to print protected `target_cells_.getLeafSize()`, uses 1000 finite
synthetic XYZ points, and performs the production call order. Its output:

```text
after_load_order configured=0.8 grid=1 1 1
after_source configured=0.8 grid=1 1 1
after_align configured=0.8 grid=1 1 1
```

Minimal reproduction with installed PCL 1.10:

```cpp
#include <pcl/registration/ndt.h>
#include <iostream>
class Probe : public pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> {
 public:
  void print() const {
    std::cout << getResolution() << " / "
              << target_cells_.getLeafSize().transpose() << '\n';
  }
};
int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  for(int x=0;x<10;++x) for(int y=0;y<10;++y) for(int z=0;z<10;++z)
    c->push_back(pcl::PointXYZ(.02f*x,.03f*y,.04f*z));
  Probe n;
  n.setInputTarget(c); n.setResolution(.8f); n.print();
  n.setInputSource(c); n.print();
  n.setMaximumIterations(1);
  pcl::PointCloud<pcl::PointXYZ> aligned;
  n.align(aligned); n.print();
}
```

Compile using `-std=c++17 -I/usr/include/pcl-1.10 -I/usr/include/eigen3`
and link `-lpcl_registration -lpcl_filters -lpcl_common -lpcl_search
-lpcl_kdtree -lpcl_sample_consensus -lflann_cpp -llz4`.
Run with `LD_LIBRARY_PATH` unset to avoid the unrelated camera SDK library path.

## Audited assets and preservation

Read both headers/implementations of current NDT, reliability, registration
geometry, frontend, solution remapping and admission; the entire runner and
standalone CMake; P7-C audit/results and P7-D/E results. Their facts above take
precedence over earlier proposed architectures. No production file, protected
dog package, stable workspace, rescue, calibration or frozen input was edited.
