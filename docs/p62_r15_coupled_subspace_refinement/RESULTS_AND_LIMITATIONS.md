# Results and limitations

## Observations

- 25/25 frozen frames had an exact PCL-score parity result of 0 absolute difference.
- 25/25 normalized information matrices passed the PSD/basis checks, with condition number `29.6649–69.0236`.
- 25/25 had zero eigenvalue ratios `<=0.01`; no weak direction was selected. The ratio range was `0.0144878–0.0337099`.
- 25/25 local finite-difference attempts stopped at `LOCAL_ASSOCIATION_SET_CHANGED`; no real gradient/Hessian or coupling block is available.
- Direct/R6 geometry support counts agreed on all 25; the relative matrix cross-check exceeded `1e-7` on 11/25 (max `2.27121e-7`). This failure is preserved, not tuned.
- 0 weak-only poses, 0 coupled poses, 0 robust pairwise outcomes, 0 accepted replacements, 0 new NDT aligns, 0 EKF runs.
- The frozen Newer College reference comparison has 20 valid TP and T0 poses. No refined pose was available. It is a map-anchored benchmark reference under a conditional base-frame assumption, not independent motion-capture truth.

## Interpretive boundary

The exact score API and synthetic derivative/Schur tests support a mathematical prototype. The current fixed-association local model did not pass the frozen real-frame contract. In particular, no claim can be made about actual `Hsw`, nonzero strong response, training or held-out benefit, or coupled pose accuracy. The earlier R11/R9 weak-direction result for R13 scan 60 used a different, unit-dependent geometry coordinate scale; the R15 normalized eigenvalue ratio is not a directly interchangeable threshold statistic.

The prior R14B reference-error summary was inspected in three rows before R15 candidate freeze. Consequently the resulting reference-relative comparison is exploratory/non-blind even though the R15 output freeze was verified before the 20 raw reference pose rows were read. The file-level CSV base-axis binding remains uncertified.

The measured 48.37 ms/frame local precheck includes a fresh PCL target Gaussian-grid initialization per input and a finite-difference check that aborts early. Successful Hessian and refinement costs were not measured. The timing does not establish a cheap successful method or 10 Hz operation.
