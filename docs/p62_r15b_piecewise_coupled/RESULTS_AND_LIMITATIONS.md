# Results and limitations

Observed in the one frozen R15B run:

- The nominal-association PCL Gaussian branch admitted stable numerical derivatives on all 25 frames, while dynamic Gaussian association sets changed in every frame's derivative stencil.
- The 0.01 formal degeneracy criterion identified no weak direction. All 50 coupled responses used the preregistered exploratory minimum spectral direction, not a formal degeneracy direction.
- Finite nonzero `Hsw` and bounded strong responses were observed. Coupled outputs differed from Weak-only by a mean 9.45 mm and 0.096 degrees.
- Dynamic PCL objective was lower for Coupled than Weak-only in 41/50 comparisons, and held-out mean was lower in 49/50 comparisons. No common-weight robust pairwise advantage exceeded 0.02; no refinement candidate was formally eligible or accepted.
- Reference-relative results on 20 Newer College frames were mixed: Coupled lowered rotation RMSE slightly, but did not lower translation RMSE. The benchmark reference is map-anchored, the Base-axis interpretation is not exporter-bound, and this evaluation is non-blind/context-informed.

Not established:

- Formal degeneracy-aware refinement.
- Recovery of an erroneous NDT local minimum or a distinct converged NDT basin.
- Robust structural-holdout preference under the frozen gate.
- Accepted replacement observation or EKF benefit.
- Independent physical accuracy improvement, generalization, real-time 10 Hz performance, or scientific novelty.

The R6 geometry comparison remains numerically unresolved at the original 1e-7 criterion. The alternative Jacobian conversion is algebraically equivalent, not bitwise-identical arithmetic. The observed discrepancies are retained and discussed in `REPORT.md`; no threshold was relaxed.

Unique next step:科研总控先核验本轮固定分支导数、耦合块、共同权重证据和受限参考误差，再决定是否保留该机制作为次级离线研究，不自动扩大实验。
