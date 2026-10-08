# R6 nonlocal admission formulation development

FINAL_RESULT = `NEAROPTIMAL_ADMISSION_PROMISING_EXPLORATORY`

NEXT = `CROSS_DATASET_NEAROPTIMAL_ADMISSION_VALIDATION`

POSTHOC FORMULATION DEVELOPMENT. EXPLORATORY ONLY, NOT CONFIRMATORY.
R4 FAIL and R5 MIXED remain unchanged. No new held-out claim is made.

## Frozen rule and information boundary

gamma=0.05. Near-optimal: (S_star-S_k)/N <= 0.05*max(1,abs(S_star/N)).
Each pool supplies its own S_star=max(S0, observed representative scores).
TYPE A requires T0 in band; TYPE B requires T0 outside band. Both require
a near-optimal non-nominal representative separated from T0 by >0.2m OR >2deg.
The original converged-only complete-link memberships and maximum-score representatives
are reused byte-pinned and verified; mixed inside/outside clusters are flagged, not changed.
T0 uses its original S0, not the nominal cluster representative score.
A_near and A_disfavored remain separate sums of xi xi^T in the frozen P9 spatial chart.
They are directional evidence tensors, not covariances or probabilities.
The read guard and evidence_freeze.json record the label-blind build; labels are opened
only by the separate post-freeze evaluator. No canonical basin pose/ID or GT is read.

## Event rates

| Pool | Event | MAJOR | NO_MAJOR | Sensitivity proxy | Specificity proxy |
| --- | --- | ---: | ---: | ---: | ---: |
| B12 | STRICT | 17/40 | 3/56 | 0.425000 | 0.946429 |
| B12 | TYPE_A | 27/40 | 10/56 | 0.675000 | 0.821429 |
| B12 | TYPE_B | 3/40 | 2/56 | 0.075000 | 0.964286 |
| B12 | UNION | 30/40 | 12/56 | 0.750000 | 0.785714 |
| FULL263 | STRICT | 30/40 | 5/56 | 0.750000 | 0.910714 |
| FULL263 | TYPE_A | 33/40 | 33/56 | 0.825000 | 0.410714 |
| FULL263 | TYPE_B | 7/40 | 1/56 | 0.175000 | 0.982143 |
| FULL263 | UNION | 40/40 | 34/56 | 1.000000 | 0.392857 |

Original STRICT is exact: B12 17/40 and 3/56; FULL263 30/40 and 5/56.
NO_MAJOR is only the frozen oracle proxy, not proof of no optimizer ambiguity.
Thus false-alert rate means NO_MAJOR event rate, not independently verified false positives.

## Separate scalar channels (exploratory only)

| Pool | Score | AUC | MAJOR mean/median | NO_MAJOR mean/median |
| --- | --- | ---: | --- | --- |
| B12 | U_near | 0.777678571 | 1.356407 / 0.304589 | 0.136803 / 0.000000 |
| B12 | U_disfavored | 0.520535714 | 0.305423 / 0.000000 | 0.092718 / 0.000000 |
| FULL263 | U_near | 0.779687500 | 2.193464 / 0.679037 | 0.350990 / 0.086147 |
| FULL263 | U_disfavored | 0.580133929 | 0.957675 / 0.000000 | 0.025520 / 0.000000 |

P05/P95, minima/maxima and zero masses are in results.json.
No U_near+U_disfavored fused score or fitted scalar threshold is evaluated.

## R5 failures and NO_MAJOR controls

| Frame | B12 event | B12 eligible clusters | FULL263 event | FULL263 eligible clusters |
| --- | --- | ---: | --- | ---: |
| 2932 | NONE | 0 | TYPE_A | 3 |
| 3813 | TYPE_A | 2 | TYPE_A | 5 |
| 3929 | TYPE_A | 2 | TYPE_A | 5 |
| 2398 | NONE | 0 | TYPE_A | 7 |
| 1944 | NONE | 0 | TYPE_A | 2 |
| 3783 | TYPE_A | 1 | TYPE_A | 4 |
| 2374 | TYPE_A | 1 | TYPE_A | 1 |
| 184 | TYPE_A | 2 | TYPE_A | 5 |
| 655 | TYPE_A | 3 | TYPE_B | 7 |
| 3200 | TYPE_A | 1 | TYPE_A | 2 |
| 2675 | NONE | 0 | TYPE_A | 2 |
| 2889 | TYPE_A | 1 | TYPE_A | 8 |
| 3899 | TYPE_A | 1 | TYPE_A | 5 |
| 2483 | NONE | 0 | TYPE_B | 2 |
| 4110 | TYPE_A | 3 | TYPE_A | 2 |
| 542 | TYPE_A | 2 | TYPE_A | 6 |
| 3829 | TYPE_A | 3 | TYPE_A | 5 |
| 3147 | TYPE_A | 1 | TYPE_B | 6 |
| 3951 | TYPE_B | 3 | TYPE_A | 3 |
| 167 | TYPE_B | 1 | NONE | 0 |

For the 17 ID-disagreement frames, B12 transitions are: `{"NONE": 5, "TYPE_A": 12}`.
Boundary/suppression flags, raw supporting member counts and one-versus-multiple cluster
indicators are retained in r5_failure_case_transition.csv and no_major_safety.csv.
Distinct terminal clusters are not certified independent attractors.
2932 still has no B12 event and has a boundary-straddling suppressed cluster.
3829 retains a suppression flag but now has TYPE A evidence from other unchanged representatives.
The pool-dependent A/B changes in the controls are caused by different observed S_star and
terminal sets; FULL263 never supplies B12 with a best score or extra candidate.

All originally STRICT-positive NO_MAJOR transitions:

| Pool | Frame | New event | Eligible clusters | Eligible outside members |
| --- | --- | --- | ---: | ---: |
| B12 | 3147 | TYPE_A | 1 | 2 |
| B12 | 3951 | TYPE_B | 3 | 4 |
| B12 | 167 | TYPE_B | 1 | 1 |
| FULL263 | 2859 | TYPE_A | 6 | 10 |
| FULL263 | 223 | TYPE_A | 1 | 2 |
| FULL263 | 3147 | TYPE_B | 6 | 13 |
| FULL263 | 2213 | TYPE_A | 1 | 1 |
| FULL263 | 3169 | TYPE_A | 6 | 6 |

NO_MAJOR positive support structure: `{"B12": {"multiple_eligible_clusters": 1, "one_eligible_cluster": 11, "single_outside_terminal": 10}, "FULL263": {"multiple_eligible_clusters": 10, "one_eligible_cluster": 24, "single_outside_terminal": 18}}`.

## Descriptive conclusion, not a new acceptance gate

B12 TYPE A has 27/40 MAJOR versus 10/56 NO_MAJOR events, with U_near AUC 0.777679 and higher MAJOR amplitudes. TYPE B is uncommon (3/40 and 2/56), not the dominant event. This is a directional exploratory signal, offset by increased proxy false alerts and severe FULL263 binary saturation. It motivates new-data validation only, not established usability.
This PROMISING classification is limited to fixed B12 formulation development. R6 specified
no numerical acceptance gate; no new one is invented. U_near AUC is below 0.80, and none of
the old R4 gates are reinterpreted or declared passed.
B12 union false-alert proxy rises from 3/56 to 12/56. FULL263 union reaches 34/56 NO_MAJOR
(60.7%); therefore FULL263 binary admission is not a successful safety discriminator.
More sampling changes S_star, event type, and the number of near-optimal clusters. Neither
coverage nor the unnormalized tensor is budget-invariant. Larger FULL263 evidence must not
be interpreted as calibrated confidence or an online improvement.
The sole next step is validation on a new dataset with the same frozen formulation. No
gamma tuning on these frames, pose switching, EKF or fusion is justified.

## Cost and verification

Blind hash/load and evidence build wall: 4.130222 s.
Post-freeze evaluation wall: 0.500107 s.
Cached-cluster verification/evidence times are in runtime_breakdown.csv and results.json.
No cluster recomputation was performed; these are not production online timings.
Historical B12 NDT alignment remains about827ms/frame. Admission does not solve low-compute feasibility.
NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO.
Release/P9 and numerical/isolation checks are recorded in verification/.
The containing commit supplies end_sha; R4/R5 archives are unchanged.
