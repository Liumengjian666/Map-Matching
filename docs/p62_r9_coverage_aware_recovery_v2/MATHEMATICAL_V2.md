# Coverage-aware structural validation V2 — pre-run contract

R8 START_SHA=36930e56bcd154d48d0f4cb79d66a7defcd43a0d. Research-only change; frozen R6–R8 and production stay untouched. Fixed original R7 raw10641–10740, not a newly selected window. No GT, feedback, EKF, timing/bias fit, or parameter retuning.

## Mask and evidence

Use the exact R8 C++ raw Source preprocessing and fixed16 scanner-frame sectors. Split before any align, G, seed or score. For odd-sector validation blocks define A={b:n_b>=10}, B=|A| using pre-match finite point counts only. Training still requires >=300. V1 requires all8 blocks; V2 requires B>=4. Counts/mask cannot change by candidate. No inactive point enters training.

For b in A, ell_b(T)=mean_x min(d_NN(Tx,map)^2/0.5^2,1). Other block losses are NA, never zero. Heldout mean and median use equally weighted active block losses; overlap counts hits among active validation points only. Report excluded validation fraction and excluded fraction of full Source separately. Angular coverage=B*22.5deg is the union of validation sector intervals, not certified full FoV, geometric observability or coverage of true poses. Block centroids/bounds are in scanner coordinates.

D_B(j,k)=min sum_b w_b(ell_b(T_k)-ell_b(T_j)), sum w=1, 0<=w<=2/B, b in A.
Sort the common differences ascending and fill the smallest with cap2/B, then a fractional last weight if B is odd. B8 invokes the frozen V1 arithmetic exactly, not a reordered equivalent sum. Tests compare exact doubles and independently enumerate capped-simplex vertices for B4–8. These deterministic robust weights are not a statistical confidence statement; spatial blocks remain correlated. Shrinking B enlarges the cap and changes the supported region, so V2 evidence is not identical to full angular evidence.

Alternative admission otherwise stays frozen: convergence, finite SE(3), <=0.8m/5deg from common historical prediction, active-point overlap>=0.50, physical dedup<=0.10m AND1deg; robust replacement must beat every finite/evaluable deduplicated rival by D>0.02. Ineligible rivals remain comparisons, not selectable candidates. Native/mean/robust use identical pools. T0 is not a supported recovery; a singleton has NA margin and robust abstains.

## Trigger and fallback

Valid frame: !converged OR iterations>=40 OR median_active_loss>0.25. The numeric thresholds do not change. B<4 (or training<300) is COVERAGE_INSUFFICIENT (or TRAINING_INSUFFICIENT); the heldout median and full structural trigger are NOT_EVALUABLE, not fabricated zero/false. Independently record optimizer failure/max-iteration flags. Run FULL and SPLIT engineering controls on nonempty inputs even when structural evidence is unavailable, but no holdout recovery/alternative acceptance. Shadow fallback is keep the nominal engineering result if legal or mark it invalid; no formal system is updated.

## Budget and scale diagnostic

All100 natural frames first, max2 SMALL recovery aligns/event, shared extra Budget20. Ordinary natural frames incur zero extra align. Keep FULL100 separate; SPLIT100 is reused as the nominal method, not aligned again. First20 are same-instance smoke; invalid structural frames are not smoke failures by themselves.

Then only local20/40/60/80, in that order. For each use its cached Training/validation/T0/prediction/Target. Freeze one normalized G direction at cached T0 for both scales. G is the existing PSD direction matrix, NOT the native Hessian. Relative weak threshold stays0.01. Schedule SMALL+, SMALL-, LARGE+, LARGE-; if the shared budget runs out preserve explicit NOT_RUN_BUDGET records. No extra frame, direction, nominal rerun or third scale. Missing weak direction is recorded, not repaired. Invalid coverage disallows structural selection, but does not forbid the preapproved attraction-domain seed diagnostic if training/G are legal.

SMALL uses frozen0.30m/3deg seed function. LARGE uniformly rescales the same chart direction with limits1.2m/8deg. These are seed limits, not accepted-output bounds. Diagnostic candidates never enter natural candidate pools or feedback, even if inside old bounds. Out-of-bound results remain attraction-domain records and rivals but cannot be accepted as recovery. All diagnostic selectors use the same diagnostic deduplicated pool and old formal eligibility; their choices are diagnostic only. Retain every raw result and duplicate.

## Pre-registered reading of outcomes

V2 engineering coverage target is >=60/100. Below it report COVERAGE_REVISION_INSUFFICIENT. Passing this target only supports wider evaluability, not correct-matching discrimination. Report natural and diagnostic distinct counts separately. No distinct candidate -> HOLDOUT_DISCRIMINATION_NOT_EVALUABLE. Distinct candidates without strict robust acceptance -> HOLDOUT_DISCRIMINATION_NOT_DEMONSTRATED; report raw loss/ranking differences descriptively. Strict accepted alternative -> deterministic selection evidence only, not absolute truth, localization accuracy, correct basin or novelty. No second experiment is authorized by any outcome.
