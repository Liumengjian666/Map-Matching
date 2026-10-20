# Pre-run tests and adversarial review

1. C++ test_coverage compiles against actual system PCL1.10/Eigen3.3.7. It never calls align.
2. B0–3 rejects unavailable paired evidence; training299 remains insufficient.
3. Independent capped-simplex vertex enumeration verifies B4–8 common-weight comparisons, including odd-B partial weights.
4. B8 direct pair/mean/median comparisons are bit-identical to V1 for1000 deterministic fixtures. B8 NN validation loss/overlap/admission also match V1 exactly on an analytic KD fixture, without an optimizer call.
5. Positive alternative, tie abstention, missing-nominal singleton abstention, noncontiguous active masks, under10 exclusion, inactive NA, active nonfinite rejection, both physical seed limits, SMALL exact old seed and shared20-token exhaustion are tested.
6. First local mathematical review found a vacuous singleton acceptance path; fixed before any real align and regression added. Second review found missing preregistration-to-formal identity binding; fixed by verifying the registered coverage/rules/source_manifest and canonical preflight helpers, not merely freezing current files.
7. Separate local execution review found no substantive violations in fixed-window scheduling, candidate pool isolation, shared budget or guards. The four diagnostic frames have legal training counts. Reviewers only read files; no native execution, GT read or edits.
8. First driver build failed due unsupported range-for on Eigen3.3 vectors. Explicit indexed serialization fixed the cause; BUILD_DEBUG.md preserves the failure. Full build passed before formal execution.
9. Actual native preflight processed all100 fixed R7 inputs, count/FNV/training/8-block counts identical to R8's frozen full ledger. V1=4/100, V2=100/100, B6=27/B7=69/B8=4. This does not certify candidate correctness or selector discrimination.

Full formal run, accounting, observed failures and post-run result review will be added without rewriting these facts. Any external CLI requires separate explicit authorization; none has been executed.
