# PAPER-P3 NDT objective audit

## Frozen pipeline

The probe used the Run-C derived input bag and normalized map without replaying
the adapter or changing the online node.  The selected clouds were converted
from `sensor_msgs/PointCloud2` XYZ fields, filtered with the frozen range rule,
then passed through the same PCL `VoxelGrid` leaves (source 0.25 m; map and
target 0.15 m).  All twelve resulting source cloud hashes match the Run-C
`ndt_determinism.csv`; the target cloud has 226164 points.

The online sequence audited from the frozen
`dog_prior_map_ndt_node.cpp` is:

`PointCloud2 -> preprocess() -> range/finite filter -> voxelDown() ->
setInputSource() -> NDT align(initial_guess) -> getFitnessScore() -> optional
step limit -> final_used pose`.

The runtime values are recorded in `p3_ndt_runtime_parameters.txt`.

## Fitness is not the NDT likelihood

The online diagnostic value `ndt_fitness` is assigned from
`ndt_.getFitnessScore()` in `handleCloudLocked()` (the frozen source lines
around 820--829).  This is PCL's nearest-neighbour fitness distance; lower is
better.  It is not `getTransformationProbability()` and is not the scalar
objective optimized by PCL NDT.

PCL 1.10 implements the actual score in
`/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp`: `computeTransformation()`
calls protected `computeDerivatives()`, which accumulates
`updateDerivatives()`'s Gaussian score and gradient/Hessian.  The internal
`trans_probability_` is the accumulated score divided by source point count.
For this installed implementation, larger raw score (and larger normalized
probability) is better.  The exact installed version is
`libpcl-dev 1.10.0+dfsg-5ubuntu1`.

## Fixed-pose evaluator

`ndt_landscape_ws` contains a thin subclass exposing only PCL's protected
`computeDerivatives()` and initializing the same Gaussian constants as PCL's
`computeTransformation()`.  It transforms a source cloud at a supplied fixed
SE(3) pose and calls PCL's original derivative/objective implementation.  No
NDT score formula was reimplemented.

The tempting `setMaximumIterations(0); align()` route is not safe in PCL 1.10:
the test pose changed by 0.0788244 m and 0.77329 deg because the iteration limit
is checked after a Newton step.  It is therefore not used for landscapes.

## Correctness gate

* **A (equivalent internal score):** for every selected frame, the fixed score
  at a fresh PCL `align()` final transform equals PCL's own
  `getTransformationProbability()` (maximum absolute difference 0).
  Replaying the complete online endpoint from the CSV initial guess gives a
  maximum 0.05035 m endpoint difference and 0.06542 nearest-neighbour fitness
  difference; this is reported separately rather than silently calling the
  replay bitwise.  The fixed objective itself is still the exact installed
  PCL objective.
* **B (repeatability):** ten repeated fixed evaluations of the same scan and
  pose have maximum raw-score difference 0.
* **C (no optimizer mutation):** fixed-pose matrix delta is 0.

Thus the objective evaluator passes its mathematical/evaluation gate; the
online endpoint replay discrepancy remains a runtime reproduction diagnostic,
not a change to the frozen algorithm.

## Files

The raw probe outputs are under the external `results/p3_landscape/` directory:
`p3_pose_score_summary.csv`, `p3_landscape_1d.csv`, 2-D grids, candidate modes,
multi-start endpoints, Hessian summaries, and correctness logs.  They are not
copied into the paper source tree.
