# Mathematical and engineering test receipt

Build used the research workspace implementation with PCL 1.10.0 and Ubuntu `c++` 9.4.0, Release, C++14. Commands:

```bash
cmake -S docs/p62_r15_coupled_subspace_refinement/tools -B /tmp/p62_r15_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/p62_r15_build -j2
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /tmp/p62_r15_build/test_coupled
```

Result:

```text
MATHEMATICAL_TESTS_PASS score_parity=0 geometry_rel=2.95402e-17 derivative_rel=0.000684582 hessian_antisym=0.0126312 scalar_hessian_rel=0.0185438 step_spread=1.01977
```

The synthetic suite exercises full SE(3) `Exp/Log`, right/body perturbation and physical scales, exact PCL Gaussian score parity, R6-to-normalized-geometry conversion, PSD/eigenbasis checks, right-tangent finite-difference gradient/Hessian checks, association-boundary rejection, `Hsw=0` and nonzero Schur responses, multiweak handling, trust limits, and raw `Hss` rejection while retaining weak-only availability.

These are deterministic synthetic/unit tests. They do not imply that any real frame admits a smooth fixed-association neighborhood; all 25 frozen real frames instead failed that gate. The measured synthetic R6 geometry discrepancy `2.95402e-17` is not the real-data conversion result. Real frames ranged up to `2.27121e-7`, crossing the frozen `1e-7` limit on 11 frames.
