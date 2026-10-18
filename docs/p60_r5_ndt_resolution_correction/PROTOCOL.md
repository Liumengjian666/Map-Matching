# P60-R5 fixed engineering correction

START_SHA: 5a12a77639a7fa6c2efa69d610a0b27adf8337f8.
Registered before new align. All R4 files, results and code remain untouched.
No old1.0m registration rerun. No production source changes.

## Sole optimizer change

R5 adapter sets `setResolution(.8f)` BEFORE calling the unchanged R4 Target
configuration helper. Thus setInputTarget constructs0.8m Gaussian cells.
All other optimizer parameters, preprocessing, point cap, initial identity poses,
FoV90 scanner+X selection, seed6042026+k and Target inputs are unchanged.

Actual `target_cells_.getLeafSize()` is read through the existing native getter.
All three components must equal float32(0.8) EXACTLY (double print
0.800000011920929). A getter-only resolution check is insufficient. Failures
abort before further align calls. Audit before align and after align/each jet.

## Frozen data

Reuse R4's100 pairs,200 internal raw PCD exports and frozen align/Hessian CSV.
Recheck original internal bag SHA256:
6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea.
Verify all R4 archive hashes, all200 raw PCD SHA, and all300 regenerated
Source/Target counts and stable hashes BEFORE any new align. Input conflicts stop.

## One bounded pass

Exactly100 pairs x3 groups, one identity-start native CONTROL align each.
Do not initialize with the old R4 result; old poses are diagnostic references only.
No state feedback, GT, IMU, Deskew, EKF, Coupled, Recovery or new search.

Two separately labeled derivative sets (300 records EACH,600 total):

1. `R4_FULL_360_FROZEN`: new0.8m grid, all three groups at R4's frozen FULL pose.
   Compare directly with the old1.0m R4 common-pose Hessian at that SAME pose.
2. `R5_FULL_360_COMMON`: new0.8m grid, all three groups at new R5 FULL pose.
   This repeats R4's within-run common-pose definition, NOT the cross-grid
   fixed-pose comparison. Never pool the two references.

Existing ObservablePclNdt, derivative-only coupledNominalJet, mean negative
native score per Source point and nonlinear chart unchanged:
p=p0+0.8*eta_t[m], R=Exp(eta_r[rad])*R0. Keep non-SPD/failed rows; these objective
Hessians are not covariance or automatically valid Fisher information.

## Frozen inference

Reuse R4 statistics_core.py unchanged. Source>=30, valid rigid converged FULL
reference, finite jet, nonzero score, Gaussian-neighbor support>=50% define
comparable rows. Each reference must provide>=80 comparable triplets; otherwise
stop scientific classification and report insufficient contract, not a positive claim.

R4 weak label remains six-DoF min/max<=1e-3 AND translation-block min/max<=1e-2,
on SPD Hessians. Native candidate run:3 successive sampled pairs with translation
weak-axis similarity>=cos20deg. R4 FoV evidence requires>=80 SPD triplets,
median B/C ratio6<=0.5, B lower than C in>=75%, and>=20 FoV-only weak labels.

Report all new weak-label samples separately for both references. If either
reference meets the unchanged R4 positive evidence rule, report
RESOLUTION_SENSITIVE_WEAKNESS_OBSERVED, not localization failure. Otherwise,
with validity gates passed, report DEGENERACY_STILL_NOT_DEMONSTRATED. Do not
change thresholds, masks or budgets after seeing data. Non-SPD Hessians at the
old pose can indicate changed local landscape, not automatically degeneracy.

All results/input/output identities frozen before analysis. No additional FoV
values or candidate experiments. Complete and stop.
