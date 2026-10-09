# P10-R4 non-oracle branch admission and causal replay

Task: PAPER-P10-R4-NONORACLE-BRANCH-ADMISSION-AND-CAUSAL-REPLAY.
Branch: research/p9-r4-heldout-visual-evidence.
Start: b87504877cab938fdfc606e4d93ed5ad5be931fc.
Initial code: 434ae03c4a14076647d2e72b434a72b01601ac27.
Single targeted improvement code: fa1912d3061800c67ef3ec6f33390af12ee97120.

## Scope and actual implementation

The PCL objective, full product-chart second-order pullback, residual-corrected
strong step, weak proposals and R3 trigger/search/confirmation budgets are
unchanged. The coupled core, original IKFoM, input reader and scan-end deskew
are byte-identical to the start commit, verified before each real execution.
Production and legacy modes remain nominal. Only this explicitly selected paper
experiment can use admitted alternative measurements.

Changed functions/interfaces:

- `runEventCoupledNdtShadow`: creation plus two future confirmations, cumulative
  normalized energy/motion differences, finite/tie guards, copy-before-clear.
- `coupledBranchMotionCost`: true SE(3) Log using existing so3Log and inverse
  left Jacobian, with 2m/15deg scales and causal LiDAR-frame IMU propagation.
- `CurrentFrameNdtRegistration::eventShadow`: optional causal interval parameter;
  uses the same owner-held source and NDT map/backend, not a second map.
- `p10_r2_replay.cpp::main`: opt-in event_admission / guarded_feedback modes;
  admitted LiDAR pose -> existing lidarMeasurementToImu -> existing unchanged
  applyPoseMeasurement. The following scan reads the actually corrected state.
- `AdmissionLogger::write`: distinct candidate/admission/actual measurement and
  update receipts, with origin, D_E/D_M/D, terminal pose and update magnitude.
- `run_admission.py`, `evaluate_admission.py`, `diagnose_admission.py`,
  `archive_admission.py`: frozen full-sequence jobs, guards, attribution and
  post-hoc GT evaluation. No builder GT/canonical access or Oracle ranking.

    E = -score/N
    D_E = sum((E_alt-E_nom)/max(1,abs(E_nom)))
    r_b = Log(inverse(delta_IMU)*inverse(T_b_prev)*T_b_cur)
    M_b = ||rho/2m||^2 + ||phi/15deg||^2
    D_M = sum(M_alt-M_nom)
    D = D_E + 0.05*D_M

Initial admission: all R3 validity/quality/physical/confirmation gates and
`D < -1e-6`; numerical equality retains nominal. Creation-frame nominal uses
the same value kernel as the alternative, without changing R3's jet/ranking
carrier. One extra value evaluation is counted; no additional full alignment.

The true causal IMU increment is previous corrected map_T_lidar inverse times
current predicted map_T_lidar, including the frozen extrinsic. It does not
include a preceding measurement correction. R3 alternative propagation remains
its original previous-prediction formula. Pending is copied before clearing,
and completed admission is consumed once, never reinjected later.

## Initial attempt (immutable)

Both full replays process all 4127 transactions with 4127 nominal successes and
successful updates. EVENT_SHADOW exactly reproduces CONTROL: 8254 registration
and trajectory row comparisons pass. No source, pose, velocity, bias or gravity
changes are hidden in shadow mode.

| Quantity | EVENT_SHADOW | GUARDED_FEEDBACK |
|---|---:|---:|
| Raw trigger | 776 | 782 |
| Search frames | 545 | 550 |
| Pending created | 180 | 187 |
| Temporally supported | 57 | 53 |
| Admitted | 35 | 35 |
| Actually used alternative | 0 | 35 |
| Complete NDT calls | 5129 | 5137 |
| Processing mean / P95, ms | 32.634 / 77.946 | 33.203 / 80.022 |
| Process wall, s | 135.052 | 137.417 |

GT is read only after both blind outputs freeze. Corrected executed IMU
translation RMSE: CONTROL 0.869398048621m; initial Feedback 0.870767799148m
(0.1575516% worse). Rotation RMSE: 2.60345414352deg -> 2.58622304354deg.
At the admitted current measurement, post-hoc translation improves/worsens:
shadow 9/26; feedback 13/22. These are not independent localization successes.

Most errors are not just a positive-motion-cost dominance issue: 28 admitted
feedback poses already have D_M<=0, yet 18 are worse in post-hoc translation.
Accepted alternatives reach 1.441064m / 13.766515deg nominal separation.
Matching quality plus incremental consistency is insufficient to establish
which persistent branch is positionally correct. This is a failure attribution,
not proof that all such poses are mathematically incorrect.

## One targeted improvement

See the separately frozen TARGETED_IMPROVEMENT_1.md. Add only a three-frame
orientation nondegradation guard against the causal IMU prediction, using the
existing R3 1e-6deg tolerance. Do not impose absolute translation closeness to
nominal, change lambda/scales/physical search, or use GT in the gate. The initial
attempt remains unchanged. Only one such improvement and one pair of complete
replays are executed.

## Final attempt: complete causal results

FINAL_RESULT: NONORACLE_CAUSAL_FEEDBACK_IMPLEMENTED_ACCURACY_NOT_IMPROVED.
NEXT (proposal for decision AI, not executed): P10_R5_WEAK_DIRECTION_BRANCH_ADMISSION.

| Quantity | Improved EVENT_SHADOW | Improved GUARDED_FEEDBACK |
|---|---:|---:|
| Full input / nominal successes / updates | 4127 / 4127 / 4127 | 4127 / 4127 / 4127 |
| Raw trigger | 776 (18.803%) | 761 (18.439%) |
| Search / Pending-confirmation frames | 545 / 264 | 528 / 260 |
| Pending created | 180 | 178 |
| Temporally supported | 57 | 56 |
| Admitted | 5 | 5 |
| Actually used alternative | 0 | 5 |
| Complete NDT calls | 5129 | 5103 |
| Jet / preview calls | 7505 / 6960 | 7256 / 6728 |
| Processing mean / P95 / max, ms | 33.241 / 79.283 / 198.824 | 33.464 / 79.623 / 210.388 |
| Process wall, s | 137.570 | 138.506 |
| Peak RSS, KiB | 112172 | 112864 |

CONTROL is immutable R3, not a new replay. In both versions EVENT_SHADOW passes
8254/8254 byte-exact registration/state comparisons. Per-version engineering
guards pass 8254/8254 rows. Ordinary frames have zero extra jet/value/align work;
pending frames have one extra alignment, zero jet/previews; maximum complete
calls are three per frame and previews sixteen. Recommended/used state outputs
are finite. Both sequences have only one map/backend each.

The improved Feedback's first real alternative update is TX591. The next scan
prediction differs from shadow, and 3536 following prediction rows differ,
demonstrating actual closed-loop feedback rather than replaying nominal priors.
The original Feedback first updates TX510 and 3617 following predictions differ.
Pending is cleared before feedback and consumed exactly once. The improved
branch score/rotation accumulation is independently reproduced on 637 valid
contribution rows; maximum score difference is 7.4941e-16. Initial contribution
audit has 644 rows with the same maximum. No GT participates in either audit.

At the improved three-frame comparison, rejected supported branches: shadow
22 D-comparison rejects and 30 orientation vetoes; Feedback 22 and 29. Other
Feedback pending outcomes: 70 collapses to nominal, 51 continuity failures and
one match-quality failure
(see frozen events and episode tables for exact ledger). There is no unconfirmed
pending state left at sequence end. The five admitted measurements have post-hoc
translation outcomes 1 improved / 4 worse, in both modes. Temporal support and
orientation nondegradation still do not identify absolute weak-position truth.

### Primary: corrected executed IMU trajectory

GT count 4126, full experimental ledger count 4127. All errors use the original
fixed anchor and extrinsic, without post-hoc fitting or GT extrapolation.

| Executed trajectory | Translation RMSE / P95 / max, m | Rotation RMSE / P95 / max, deg |
|---|---:|---:|
| CONTROL (and both EVENT_SHADOW) | 0.869398 / 1.535711 / 1.763296 | 2.603454 / 5.785784 / 11.589847 |
| Initial Feedback | 0.870768 / 1.539772 / 1.757365 | 2.586223 / 5.759198 / 11.502702 |
| Improved Feedback | 0.870879 / 1.557781 / 1.749467 | 2.576896 / 5.761446 / 11.429903 |

Improved corrected translation RMSE is 0.1703875% worse than CONTROL; the >=5%
improvement target fails in both versions. Rotation RMSE improves ~1.020%; that
does not substitute for translation success. The improvement is retained as a
failed development comparison, not selected away in favor of a better-looking
variant. Engineering and mean/P95 performance pass; localization accuracy
improvement is not established.

### Separate: raw measurement errors

| Raw measurement | Translation RMSE / P95 / max, m | Rotation RMSE / P95 / max, deg |
|---|---:|---:|
| CONTROL nominal | 0.863610 / 1.533448 / 1.784590 | 4.479269 / 11.013434 / 20.800058 |
| Initial actually-used measurement | 0.865801 / 1.538409 / 1.799393 | 4.547711 / 11.262409 / 24.185953 |
| Improved actually-used measurement | 0.865639 / 1.537946 / 1.760057 | 4.430321 / 10.886849 / 20.203802 |

Within improved Feedback, its own nominal measurement (before optional
replacement) RMSE is 0.865344622702m / 4.438216883716deg. This nominal already
differs from CONTROL because prior admitted updates changed prediction, deskew
and later measurement inputs. Do not confuse this with a fixed shadow proposal.

Large-jump counts: CONTROL 3, initial Feedback 3, improved Feedback 3 under the
predeclared >0.5m OR >10deg rule. Maximum improved IKFoM update correction is
0.248473348713m / 2.202638471048deg; maximum admitted measurement separation from
the current nominal is 1.311053135938m / 11.871136232404deg. No extra nonfinite
state or hidden update fallback occurs. Small RMSE degradation and translation
P95 degradation remain real, even though jump count and maxima do not worsen.

### Cost and direct contribution

Four complete experiments (initial+one improvement, two modes each), 16508 frames
and 20498 real complete NDT calls; no newly repeated CONTROL. Preparation/source
recovery/Oracle/B12/visual/Corridor costs are zero. Improved Feedback average
33.463643ms / P95 79.622889ms versus R3 event 33.035294 / 79.599593ms; timing is
descriptive across runs, not a worst-case guarantee. Peak RSS 112864KiB (~110.22MiB),
888KiB above archived CONTROL (high-water proxy only).

Actual contribution: a working non-oracle three-frame comparison, distinct
candidate/temporal/admitted outputs, and real opt-in causal candidate-to-IKFoM
integration with frozen noise and numerical/budget guards. It demonstrates
engineering feasibility, not a successful absolute-position branch selector.

Main accuracy bottleneck: matching score plus short-window incremental motion
can preserve wrong weak-position branches. The orientation-only improvement
removes most initial acceptances but leaves four worse positional choices out
of five. Simply tightening the same rule on this cohort would not constitute
new evidence. Retain nominal production behavior and stop this task.

Single next-stage proposal: P10_R5_WEAK_DIRECTION_BRANCH_ADMISSION, to determine
whether any weak-position branch correction has sufficient non-GT support for
measurement use, before permitting further feedback. The decision AI must
verify this archive and freeze the next contract; no next-stage code is started.

## Verification and interpretation limits

Release build and P7/P10 tests 5/5, P9 tests 41/41, R1 tests 2/2 pass for both
code revisions. Synthetic tests cover normalized cumulative scores, equality,
two future confirmations, failure-state snapshots, single consumption, missing
IMU interval, noncommuting transforms, map-coordinate invariance, true SE(3)
translation Log, and constant-rotation branch veto. The new veto test fails
before implementation and passes afterward. The original static initializer
and filter update source are unchanged.

One first compile failure concerned a C++ test default argument referencing a
local variable; fixed before any real NDT replay. Independent review identified
and fixed rejected-pending history loss and zero-episode evaluator handling.
Only single-model read-only review was used, as selected by the user; no external
CLI. The skills' optional generic reference files were absent; repository code,
explicit task contract, tests and bounded review supplied the checks.

All 4127 experimental frames are retained. TX4127 is outside GT bracketing;
GT count is 4126 with no extrapolation, fitted alignment or frame deletion.
Use corrected executed IMU trajectory as the primary accuracy output; raw NDT
measurement and candidate errors are separate diagnostics. Floor01 is development
data, and the improvement follows inspection of attempt 0. No independent
generalization or production safety claim is made.

Large-jump descriptive threshold was frozen before replay: consecutive corrected
translation >0.5m OR rotation >10deg. CONTROL already has three such rotational
events. Counts alone are not truth labels. Runtime includes scan-end, point I/O,
source preparation, NDT, IKFoM and CSV logging, except writing its own final cost
row. Startup components were not individually timed; process wall includes them.
Peak-RSS differences against the archived CONTROL are proxies, not allocators'
exact incremental-map memory. Max-frame latency is reported; no hard real-time
guarantee is asserted. Large raw data/binary caches remain outside Git.

No new Oracle263, B12, visual extraction, Corridor bootstrap, source recovery,
noise tuning or production state switching is performed. No active push has
been authorized or executed. The final archive commit and portable Git bundle
are reported in the handoff; local commits are not remote delivery.
The pre-run remote query confirmed b87504877cab938fdfc606e4d93ed5ad5be931fc;
the final query failed with Couldn't connect to server. New remote delivery is
not confirmed. See remote_receipt.json; do not assume network remained available.
