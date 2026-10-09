# One targeted algorithm improvement: anchor-aware pending terminal rank

Version 0 is retained. Two full 4127-frame runs completed and frozen before GT.
Independent non-GT diagnosis found 160 Pending creations across both runs,
81 with two eligible refined terminals; 33 selected a terminal whose creation
weak-anchor cost exceeded another already observed eligible terminal's cost.
Maximum difference 2.289220388202209 dimensionless squared cost. This diagnosis
was produced before the corrected pre-GT audit/GT evaluation. No oracle IDs.

V0 executed feedback only twice and corrected translation RMSE improved 0.0797%,
not 5%. GT was subsequently observed but does not select this change, candidates,
frames, thresholds or noise. The mechanism diagnosed without GT is that the old
R3 absolute-current-prediction merit ignores the new position reference when
choosing the single pending terminal; the later anchor gate cannot recover an
alternative that was not tracked.

For version 1, select a pending terminal from the already refined (<=2) terminals:
same successful/converged/rigid/positive-score/quality/strict-separation/physical
conditions as R3, also respecting the creation value-kernel quality check of V0.
Primary deterministic rank: current frozen-W anchor squared error; exact ties
lower normalized NDT energy, then lower candidate ID. No added NDT/preview/jet,
no changed source/map or candidate pool; no additional quality threshold.
Three-frame anchor margin (.01 mean and 10% relative), lifetime2s, propagation,
frozen W, confirmation, feedback, all scientific parameters unchanged.

This is the only R5 targeted algorithm modification. Run one full V1 shadow
then, if legal admission, one full V1 feedback. Freeze before V1 GT, preserve
all V0 results even if V1 is worse. No further refinement or threshold tuning.
