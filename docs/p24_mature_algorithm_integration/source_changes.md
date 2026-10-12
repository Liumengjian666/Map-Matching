# P24 source changes

- `dog_prior_map_ndt_node.cpp`: add a default-off P24 configuration path; evaluate the existing P10-style NDT objective jet and adapted signed Schur detector after nominal NDT; run bounded weak-axis previews and at most two independent extra NDT aligns; retain nominal unless the frozen finite/convergence/objective/motion checks pass; log per-frame module evidence. No NDT objective, EKF, velocity blend, OOSM or Step Limit math was changed.
- `mature_ndt_algorithms.hpp/.cpp`: small P24 detector/config/candidate/gate helpers. The Schur solve retains LDLT for its normal path and adds a signed-eigen fallback for a well-conditioned invertible indefinite block when the LDLT residual fails. Singular or ill-conditioned blocks remain a conservative no-candidate condition.
- `mature_ndt_algorithms_test.cpp`: validate signed nonconvex curvature, off-diagonal indefinite Schur solve fallback, finite pose candidates, safety ranking and SKF skip/double-count semantics.
- `dog_prior_map_localization_split.launch`, `p17_visual/run.py`: optional P24 overlay and per-run diagnostics path; validate frozen bag, map, visual and dataset config hashes, loaded runtime map path, mature parameters and frozen EKF binary. Without an overlay, the historical run mode remains the default.
- `visual_aided_initial_guess.hpp/.cpp`: add a read-only valid-measurement query used by the SKF gate; it does not change P4 visual math.
- `evaluate_p18.py`: add an optional visual-application-ledger path while preserving the historical default, allowing P24 to reuse the frozen evaluator and transform.
- `config/p24_mature_baseline.yaml` and `config/p24_mature_full.yaml`: centralized all-off and fixed all-on settings; the latter is the sole experimental overlay used in the complete run.
- `scripts/p24_mature_integration/{freeze_outputs.py,summarize.py}`: hash-freeze pre-GT outputs, derive the actual P24 visual application ledger, verify completeness/budgets, summarize P19-vs-P24 metrics and archive lightweight evidence.

No DCReg, Autoware or SKF third-party implementation was copied into the repository. The P24 path is an adaptation/prototype using existing project NDT objective and PCL align interfaces; it is not a claim of exact reproduction of any complete external system. DCReg repository license status was not clear, so no source was vendored.
