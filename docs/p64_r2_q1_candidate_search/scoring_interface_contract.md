# Native read-only fixed-pose scoring interface

Input is immutable PCL XYZI scan and a prebuilt map, with externally supplied float32 rigid 4×4 pose T_map_lidar. Scan and map are filtered once per process frame to exact 0.1 m historical centroids. Native model uses 1 m cells, DIRECT7, epsilon 0.01 and default outlier ratio 0.55. Scorer::fixed directly calls the pinned upstream updateDerivatives scalar path with zero Jacobian and Hessian arguments and no Hessian output. The moment, lookup order, quadratic expression, exponential and error guard share the upstream implementation. The direct scorer receives no reference, optimizer, filter or IMU state. Scorer::oracle is a separate Gate 1/testing path using upstream fixed-cloud computeDerivatives.

Outputs represent larger-is-better native raw sum and normalized native score over the **full fed source point count**. Absent neighborhoods remain in the denominator with zero reward. Association here means a valid voxel was queried, which differs from the historical nearest-neighbor diagnostic inlier fraction. Floating intermediate math and double point-order reductions are intentionally preserved.

- point_scores.csv stores each filtered source ID, transformed XYZ in meters, sum of native neighbor rewards, Park point log likelihood, lookup flags and shared source count.
- voxel_contributions.csv stores pairs of filtered point ID and stable native flattened cell key, native squared Mahalanobis distance and reward.
- cells_H.csv and cells_O.csv store each usable native cell's mean, covariance and map-point count. Cell keys have scope within each map.
- each membership CSV maps the stable ordered filtered-source ID to all original IDs in that 0.1 m voxel. A centroid can represent multiple originals. This avoids inventing a single original-point identity for an averaged source point.
- grid.csv stores aggregate sums and weights for 58,212 poses. No per-point payload is materialized at grid poses outside the frozen Gate 1 and fit diagnostic packets. These can be re-evaluated with the supplied interface.
- timestamps remain in the locked manifest as integer header nanoseconds. XYZ uses meter units. Angles in search offset columns are degrees. Pose row order is 00 to 33.
