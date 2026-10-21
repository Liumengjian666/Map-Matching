# Tests and adversarial review

## Final cold build

- Final fresh Release build directory: `/tmp/p62_r15b_piecewise_build_frozen02` (the earlier frozen01 build was superseded after the last test-source clarification; final identities are in `BUILD_PROVENANCE.json`).
- Compiler: Ubuntu GCC 9.4.0; C++14; PCL 1.10.0; explicit existing `usb-1.0` link dependency.
- Final configure/build/test logs: `final_configure.log`, `final_cold_build.log`, `final_test.log`.
- Final test binary SHA and output are recorded in `BUILD_PROVENANCE.json` and `final_test.log`.
- Final deterministic test result: `MATHEMATICAL_TESTS_PASS`; PCL score parity 0; direct geometry relative error `2.95402e-17`; same-arithmetic direct/R6 algebraic conversion relative error `2.95655e-17`; fixed-branch gradient directional relative error `1.66913e-7`; Hessian antisymmetry `6.48887e-7`; scalar Hessian relative error `6.45233e-6`; step-spread `1.00002`; association-boundary test changes one point.

## Covered contracts

The tests cover right SE(3) Exp/Log and physical scaling; exact PCL score parity; direct and legacy geometry conversion including distinct accounting; fixed branch z=0 score identity; stable Gaussian grid across source changes; independent fresh-grid score agreement; a dynamic association change that does not gate fixed-branch derivatives; Schur equation and zero/nonzero Hsw; an exploratory-only minimum spectral direction with remaining formal weak directions held fixed; preservation of the independent weak-only ablation when the coupled block is invalid; and safe rejection of a nonfinite local model.

## Preserved development failures

The first build exposed Eigen fixed-vector range iteration, Boost pointer explicit-bool conversion, and PCL 1.10 non-const grid query APIs; these were fixed with indexed vector access and correct mutability/explicit pointer tests. A later test initially expected two selected weak directions in exploratory-minimum mode, which contradicted the R15B contract of exactly one probe direction; the test was corrected to expect one probe, keep remaining formal weak directions unmodelled, and solve only the complement. A nonfinite test initially asserted that the rejected weak pose itself must be a NaN matrix; the implementation correctly kept the computed pose finite but marked it unavailable, so the assertion now tests the actual safety contract (unavailable result, no coupled candidate, invalid status). The sequence and outcomes are retained in `BUILD_DEBUG.md`; the final cold test passes.

Synthetic checks establish algebraic/numerical behavior only. They do not establish real-data coupled response, geometric gain, held-out gain, reference-relative improvement, formal degeneracy recovery, or novelty.

## Real-data execution and geometry comparison caveat

The one frozen 25-frame mechanism run completed in 8.24 s wall time, with peak process RSS 89,284 KiB, two Gaussian Grid initializations (one per map), and zero `align()`, EKF, or pre-freeze reference-pose reads. Fixed-association derivative checks passed on 25/25 frames. All 25 frames nevertheless changed at least one PCL association somewhere in the finite-difference stencil; derivatives therefore describe only the frozen nominal branch, not the dynamic PCL objective.

The R6-vs-direct geometry diagnostics are not a pass at the historical 1e-7 threshold: the R6 path exceeds it on 11/25 unique frames (22/50 alpha rows). The output column called `same_path_algebraic_conversion_relative_error` is also not bitwise-identical arithmetic: it shares transformed PCL points, associations, residuals, and Gaussian weights, but constructs the Jacobian by an alternative algebraic factorization. It exceeds 1e-7 on 7/25 unique frames. The small discrepancies are consistent with float-level rotation non-orthogonality and floating evaluation order, but the real-data comparison does not isolate those contributions; no threshold was changed or old R15 result reclassified.

The reference evaluator had two preserved post-freeze table-join failures (`REFERENCE_EVALUATION_ATTEMPT_01_FAILURE.json`, `REFERENCE_EVALUATION_ATTEMPT_02_FAILURE.json`) after parsing the same 20 selected reference rows. Both stopped before writing error outputs and did not alter the sealed model data. The third offline pass used the actual `frame_key` schema and frozen GEODE mapping; no NDT or candidate calculation was repeated. Thus 20 unique reference poses were used, but their selected values were parsed in three post-freeze evaluation attempts (60 row parses total).

This review is a same-context adversarial review because no independent review agent was available. A material finding was corrected in the written interpretation: the alternate Jacobian conversion is algebraically equivalent, not identical arithmetic. R15B remains a mechanism probe only, not formal degeneracy recovery or accuracy evidence.
