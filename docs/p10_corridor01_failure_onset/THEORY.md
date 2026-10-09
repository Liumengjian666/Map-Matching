# Paired comparison semantics

This experiment makes no change to the R6 Degeneracy-Aware Coupled Subspace
NDT equations. It reuses the frozen decomposition `eta = W*u + S*v`, the
existing weak-direction regularized update, the displaced joint jet with full
map-product pullback, and one conditional strong-direction correction.

For each scan, all methods share one prediction pose, one deskewed source,
one target map, one timestamp and one nominal PCL NDT alignment. Weak-only and
Coupled are therefore paired per-frame conditional candidates, not separate
state histories. Their selected output is exactly:

```text
T_selected = T_candidate  if the frozen R6 non-GT recommendation is true
             T_nominal    otherwise
```

This selected candidate measurement does not alter the next scan's prediction
in this experiment. Coupled's conditional strong correction was selected in
21 rows, but whether it reduces tracking drift must be tested in an actual
causal feedback replay; no such claim follows from this paired run.

For offline relative evaluation only, all candidate LiDAR poses are converted
with the same frozen laser-to-IMU extrinsic, and one proper rigid transform is
fit from Control nominal positions in the first 10 seconds:

```text
R,t = argmin_(R in SO(3), t) sum_i ||R p_control_i + t - p_GT_i||^2
```

The identical `(R,t)` is applied to Nominal, Weak-only and Coupled for the full
window. Scale remains exactly one. Because the map-to-GT relation is not
uniquely closed and the Control prefix residual is large, all resulting
position/orientation quantities are labeled PREFIX-ALIGNED RELATIVE DRIFT,
not absolute localization accuracy.
