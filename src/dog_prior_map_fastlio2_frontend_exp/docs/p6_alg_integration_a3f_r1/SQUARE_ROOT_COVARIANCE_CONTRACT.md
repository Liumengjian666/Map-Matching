# A3F-R1 square-root pre-measurement covariance contract

START_SHA: 34ac3f7692ff51efc3c168fb45e080b8573dee28.

## Authoritative inputs and timing

V3 explicitly selects `MarginalCovarianceBackend::SQUARE_ROOT_QR`. Historical
callers retain `LEGACY_NORMAL_SPARSE_LLT` by default. There is no automatic
fallback. The A3E-R2 square-root marginalization implementation is unchanged.

After `prepareStateAt(scan_end)` admits the predicted node and IMU factor,
before current LiDAR admission, the covariance query constructs one frozen
LiDAR snapshot. It assembles current-chart authoritative prior A/b, all active
IMU factors, all historical active LiDAR factors, and all active visual factors.
It never re-roots the prior H cache. Each raw factor uses its existing residual,
Jacobian and covariance, whitened with `L.solve(J)` and `L.solve(r)` for R=LL^T.
This ALL_FACTORS assembler is distinct from frozen PRIOR+TOUCHING_OLDEST
marginalization. No consumed-factor or lifecycle rule is changed.

## Derivation

For full-column-rank A, column-pivoted QR gives AP=QR and
H=A^T A=P R^T R P^T. Select E with identity in the latest fifteen rows.
Solve R^T Y=P^T E, using a lower triangular solve, not an explicit inverse.
Then P15=Y^T Y=E^T H^-1 E. Production never forms H or solves normal columns.

The unchanged map selector T has R_map_imu in block (0,0), I in block (3,3),
and zero elsewhere. Ymap=Y T^T; Pmap=Ymap^T Ymap. State ordering is right
rotation, map position, velocity, gyro bias, accel bias. No coordinate or NIS
pose convention changes.

Eigen CPQR threshold is relative internally: eps*max(rows,cols). The reported
absolute rank threshold is that value times maxPivot. Rank must equal columns.
Rank deficiency fails closed without pseudoinverse, jitter, rank truncation or
legacy fallback. Triangular residual is ||R^T Y-P^T E||F/||P^T E||F <= 1e-10.
Finite Gram products are alias-safe symmetrized and checked by LLT. No clamp
or artificial information is introduced.

## Read-only and shadow isolation

Queries preserve states, reference chart, A/b, H/g cache, factor records, active
IDs, watermarks and both revisions. Only pre-existing diagnostic request/time
counters are mutable. LiDAR reliability callbacks run once per factor/request.
With diagnostics on, legacy normal sparse LLT uses exactly the same snapshot.
It cannot affect returned P, NIS, probes, factor admission or estimator state.
Non-LiDAR events do not request either covariance backend.

Comparison NIS/admission uses the SAME current predicted state and SAME
selected measurement, with only P switched. It is not a second trajectory or
a hypothetical alternate NDT probe replay. Different probe seeds/terminals
cannot be inferred from an unexecuted alternate probe; trigger differences are
reported independently.

## Frozen behavior

Optimizer, QR marginalization, IMU integration/noise, NDT, deskew, handoff,
U_obs/U_nonlocal definitions, reliable rank/basis, NIS threshold and visual
policy remain unchanged. No square-root optimizer or SparseQR optimization
is introduced. Dense CPQR cost is reported as engineering observation only.
