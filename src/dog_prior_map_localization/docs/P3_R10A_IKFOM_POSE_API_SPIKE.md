# PAPER-P3-R10A exact IKFoM pose API spike

Status: compile-and-execute API spike only; no runtime node or bag evaluation.

## Pinned dependency and build boundary

- Repository: `https://github.com/hku-mars/FAST_LIO.git`
- Commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- External checkout: `/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`
- `use-ikfom.hpp` is included by exactly one package translation unit,
  `dog_prior_map_fastlio2_frontend_exp/src/ikfom_pose_api_spike.cpp`.
- Public API header exposes only a function facade and does not include the
  external header. A second translation unit calls that facade; the successful
  link checks the intended ODR boundary for this spike.
- `DOG_PRIOR_ENABLE_FASTLIO2_EXP` defaults to `OFF`. The SHA/path/header checks,
  external include path, OpenMP dependency, and executable are all inside the
  option's `ON` branch.

## Measurement and update exercised

The test declares a manifold measurement containing `MTK::vect<3,double>` and
`MTK::SO3<double>`, then instantiates exactly:

```cpp
esekfom::esekf<state_ikfom, 12, input_ikfom, PoseMeasurement, 6>
```

Its `h`, `h_x`, and `h_v` expose position and rotation using the pinned state's
manifold indices. The test reconstructs a candidate/shadow filter from the
committed filter's `x` and `P`, rebinds the same process/measurement callbacks,
and invokes the pinned generic `update_iterated()` API, whose return type is
`void`.

The adversarial case injects nonzero pose-to-extrinsic cross covariance,
applies `enforceFixedExtrinsicConstraint()`, and requires those cross terms to
be zero before update. After update it checks finite state/covariance,
covariance symmetry and diagonal tolerance, unchanged calibrated extrinsic
values, retained 9.809 m/s² S2 gravity norm, no reintroduced tested
pose/extrinsic cross terms, and a nonzero position correction.

The official IKFoM filter has no sensor timestamp member; timestamp immutability
is therefore a wrapper transaction invariant to test in the later runtime, not
a property claimed by this API-only executable.

## Verification performed

The message package was generated in a fresh temporary Catkin build space. The
full paper source space was then configured in another clean temporary build
space with the experimental option left at its default `OFF`; the existing MIT
localizer and NDT targets built without a `FASTLIO2_REF_ROOT` cache entry.
Finally, the experimental package was explicitly configured `ON` against the
pinned external checkout and the two-translation-unit spike executable built
and printed:

```text
IKFOM_POSE_API_SPIKE_PASS
```

This does not close the initial-map-pose or physical IMU/LiDAR time-offset
gates. It does not authorize Floor01 replay, full runtime integration, or P4.
