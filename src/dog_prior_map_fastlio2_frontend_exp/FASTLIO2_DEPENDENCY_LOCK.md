# FAST-LIO2 dependency lock

The opt-in research package consumes, without copying or modifying it, the
external checkout `hku-mars/FAST_LIO` at commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`.

The root is supplied only when configuring with
`DOG_PRIOR_ENABLE_FASTLIO2_EXP=ON`. The default `OFF` configure path must not
read `FASTLIO2_REF_ROOT`, inspect the checkout, search for IKFoM, or add an
experimental executable. When enabled, CMake checks the exact Git SHA and the
required pinned headers before building the API spike.

`use-ikfom.hpp` defines non-inline free functions. Exactly one translation unit
may include it; public headers expose only the dependency-light façade in
`include/dog_prior_map_fastlio2_frontend_exp/ikfom_pose_api_spike.hpp`. The
two-source API spike is an ODR/build-boundary check, not runtime localization.

This package is marked `GPL-2.0-only`; the existing MIT package is not changed
or made dependent on this package. This is an engineering dependency boundary,
not a legal opinion.
