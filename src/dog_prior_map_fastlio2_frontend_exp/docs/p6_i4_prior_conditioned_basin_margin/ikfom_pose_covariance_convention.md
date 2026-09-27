# IKFoM pose covariance convention audit

## Frozen implementation inspected

The audited FAST-LIO2/IKFoM source snapshot is
`/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`, commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`.

`include/use-ikfom.hpp` declares the state manifold in this order:

1. `pos` (`vect3`),
2. `rot` (`SO3`),
3. LiDAR/IMU extrinsic rotation,
4. LiDAR/IMU extrinsic translation,
5. velocity,
6. gyro bias,
7. accelerometer bias,
8. gravity direction (`S2`).

The tangent start indices are obtained from the same `state_ikfom` type using
`MTK::getStartIdx`; no hard-coded 6x6 covariance block is assumed. The full
`FilterSnapshot::covariance` is projected through `J_pose`, including all
pose-to-rest and rotation-position cross terms.

## Rotation and position perturbation

In the frozen IKFoM manifold, `MTK::SO3::boxplus` computes
`SO3 delta = exp(vec, scale); *this = *this * delta`. Its `boxminus` computes
`Log(other.conjugate() * *this)`. Therefore the estimated orientation uses a
right/body perturbation:

\[
R_{true}=R_{est}\operatorname{Exp}(\widehat{\delta\theta_{body}}),
\qquad
\delta\phi_{map}\simeq R_{est}\delta\theta_{body}.
\]

The frozen `MTK::vect::boxplus` is additive (`*this += vec`), so position error
is map/world additive. For the required map product tangent
`[dphi_map,dp_map]`, the pose Jacobian has blocks

\[
J_{pose}[0:3,\texttt{rot}]=R_{est},\qquad
J_{pose}[3:6,\texttt{pos}]=I_3,
\]

with zeros in the remaining pose-Jacobian columns. The baseline runner captures
the covariance after the IMU prediction and before applying that scan's NDT
pose measurement. NDT seeds are composed from `map_T_imu` with the frozen
`T_imu_lidar`; probe perturbations act on `map_T_imu` in exactly the physical
product coordinates above, then the registration wrapper forms the LiDAR seed.

## Numerical validation procedure

Ten prediction states, evenly selected by transaction order from the frozen
88-frame P5-I2 cohort, were audited. At each state all six relevant state
error-state basis directions were perturbed through official
`state_ikfom::boxplus` with `epsilon=1e-7`. The resulting FD pose tangent was
`[Log(R_perturbed R_nominal^T), p_perturbed-p_nominal] / epsilon` and was
compared column-wise with `J_pose`. The complete per-column record is in
`pose_covariance_fd_validation.csv`; any column over `1e-5` aborts before basin
search.

## Recorded result

Filled by the frozen-data report after baseline covariance capture:

- selected prediction states: 88;
- covariance-valid fraction: reported below;
- effective-rank distribution: reported below;
- FD states / columns: 10 / 60;
- maximum absolute FD-column error: reported below;
- FD gate (`<=1e-5`): reported below.











## Frozen P6-I4 covariance results

- Valid covariance contexts: 88/88 (100.00%).
- Effective rank counts: `{"6": 88}`.
- FD validation: 10 prediction states / 60 pose columns; maximum absolute column error `2.97580537e-06`; gate `<=1e-5`: PASS.
