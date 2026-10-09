# P10-R5: anchored weak-subspace coupled NDT

FINAL_RESULT: ANCHORED_CAUSAL_FEEDBACK_IMPLEMENTED_ACCURACY_GAIN_BELOW_TARGET.
NEXT proposal (not executed): P10_R6_USEFUL_WEAK_BRANCH_ADMISSION_RECALL.

Short inertial anchor and real causal feedback are implemented. Full corrected
translation RMSE improves **0.0796657%**, far below the 5% development goal.
Do not label this precision validation PASS or an independent generalization
test. R4's unfavorable accuracy finding is unchanged. Production remains nominal.

## Git, implementation and frozen inputs

Branch research/p9-r4-heldout-visual-evidence.
START 65a416ec19069efa0405e9fc56a0505dc9c3018e.
V0 runtime CODE 5f099ea8d8908ca046f907baadff7a0b7fcb1469.
Independent audit repair f5e0b6a is postprocessing only, not another runtime.
V1 runtime CODE b4b4f6a65176a1c9fd27d99f0af1039bb2dbd339.
Actual writable worktree /tmp/dog_loc_paper_r4_ws.Fq21k2; original .git is read-only.
Remote START was verified by git ls-remote. No push authorized/executed. Final
archive commit/bundle are reported by execution handoff, not self-referentially
claimed inside their own commit. Do not confuse local commits with remote delivery.

New coupled_ndt_anchor.hpp/cpp implement advanceCoupledAnchor,
settleCoupledAnchor, coupledAnchorChart and runAnchoredCoupledNdtShadow.
CurrentFrameNdtRegistration::anchoredEventShadow reuses its existing source/map
backend. Replay adds explicit anchored_shadow/anchored_guarded_feedback modes,
post-update anchor establishment and feedback invalidation. AnchorLogger stores
the before-consumption event receipt plus after-update state. New builder,
independent evaluator, rank diagnosis and archive scripts are traceable.

Coupled math and old temporal policy are byte-exact to START:

    eta = W*u + S*v
    H = J^T H_native J + sum(g_native[a] K_a)
    (Hvv+lambda I) delta_v = -(gv+Hvu delta_u)

Anchor poses are map_T_lidar. Seed only actual corrected state after an ordinary
untriggered effective frame, never reset a valid anchor from NDT. Propagate

    delta_imu = inverse(previous corrected map_T_lidar)*current predicted map_T_lidar
    anchor_prediction_current = anchor_prediction_previous * delta_imu

Maximum lifetime2s; invalid/gapped/nonmonotonic propagation, ineffective nominal
or actual alternative feedback invalidate it. No same-frame renewal after
invalidation. Pending cannot renew its anchor. Missing anchor retains nominal
with zero additional search/refine/value calls. Creation W is fixed throughout
the three-frame episode; future models' default identity is never substituted.

    eta_t = (p_candidate-p_anchor_prediction)/0.8
    eta_r = Log_SO3(R_candidate*inverse(R_anchor_prediction))
    A_branch = ||W^T eta_branch||^2
    gain = (sum A_nominal - sum A_alternative)/3
    admit iff gain > .01 AND sum A_alternative <= .90*sum A_nominal

Also require existing rigidity/convergence/matching/physical/two-future-frame
confirmation guards. R4 D remains independently audited diagnostic, not a gate.
The old prediction-to-prediction alternative propagation is deliberately
unchanged; only anchor reference excludes preceding pose corrections. Neither
anchor nor temporal support is an absolute-position truth or probability.

Six true Floor01 inputs and immutable R3 CONTROL CSV hashes PASS, same .8/.08/
1e-5/80 NDT, map, initialization, extrinsic, source preprocessing, causal deskew,
filter/noise. Every one of 4127 IDs is processed in original order. No new
source extraction/recovery, baseline control replay, Oracle/B12/Corridor/visual.

## One permitted targeted improvement, not a grid

Non-GT frozen diagnosis: among160 creations in V0,81 had two eligible refined
terminals and33 ignored a lower weak-anchor-cost observed terminal. V1 ranks
only existing eligible refined terminals by anchor cost, then energy, then ID.
No candidate pool, quality threshold, search bound, budget or admission change.
V1 ignored-lower-anchor-cost count is **0/160**. It does not create missing
branches. This removes a real ranking inconsistency but produces no additional
actual admissions/trajectory benefit. V0 and V1 preserved. No second change.

## Full causal experiment counts and cost

| Quantity | V0 shadow | V0 feedback | V1 shadow | V1 feedback |
|---|---:|---:|---:|---:|
| Input/nominal successes/updates | 4127 | 4127 | 4127 | 4127 |
| Anchor valid frames | 3397 | 3420 | 3397 | 3420 |
| Anchor established | 179 | 181 | 179 | 181 |
| Raw trigger | 776 | 765 | 776 | 765 |
| Search frames | 294 | 303 | 296 | 303 |
| Pending creations | 75 | 85 | 75 | 85 |
| Temporal support | 25 | 26 | 23 | 25 |
| Anchor admitted | 4 | 2 | 4 | 2 |
| Actual feedback use | 0 | 2 | 0 | 2 |
| Complete NDT calls | 4569 | 4604 | 4571 | 4604 |
| Jet calls | 4190 | 4311 | 4216 | 4311 |
| Preview calls | 3896 | 4008 | 3920 | 4008 |
| Mean / P95, ms | 29.934 / 65.765 | 29.762 / 66.128 | 29.925 / 65.537 | 29.761 / 66.206 |
| Max, ms | 194.845 | 245.472 | 195.434 | 239.576 |
| Process wall, s | 123.926 | 123.207 | 123.883 | 123.206 |
| Peak RSS, KiB | 112812 | 112712 | 112752 | 112188 |

Four runs,16508 frames,18348 real experimental complete NDT calls;1840 extras.
R3 CONTROL cost 25.982/49.596ms; R3 event33.035/79.600ms; R4 final feedback
33.464/79.623ms. Cross-run timing is descriptive, not a paired CPU estimate or
worst-case real-time guarantee. New final RSS109.56MiB,212KiB above archived
CONTROL high-water mark; difference is only an incremental-memory proxy.
Anchor arithmetic is included in full processing; not separately timed.
Full processing includes point I/O, prediction/deskew, nominal alignment,
extra work, filter update and logs except the cost row itself. Process wall
also includes startup. Phases/groups are fully retained in evaluation.json.

Every frame respects <=3 complete alignments/<=16 previews. Ordinary and
anchor-unavailable frames have zero additional jet/value/align calls. Pending
confirmation has one extra alignment, zero new jet/preview. One map instance,
no nonfinite actual/recommended state,4127 successful filter updates per run.
Each shadow has8254/8254 exact nominal source/registration/filter-state parity.
Anchor cost/fixed-W/propagation audit maximum difference1.77636e-15. Real
feedback first changes TX616; TX617 prediction changes and3511 later prediction
rows differ from shadow. Pending is consumed once. This is not pasted poses.

V1 feedback rejected pending outcomes:29 collapse to nominal,23 continuity
failures,7 unavailable/expired anchor,1 match-quality failure,25 temporal support.
Among25 supported,23 fail meaningful anchor advantage and2 are admitted.
Four legal shadow admissions do not imply four feedback admissions: causal
state/anchor invalidation changes later evidence, and shadow never replaces it.

## New position evidence, not merely fewer R4 acceptances

| Actually used frame | Mean weak gain | Weak displacement fraction | R4 D | Raw nominal→alternative translation GT error, m | IKFoM correction, m |
|---|---:|---:|---:|---:|---:|
| TX616 | .0570823 | .997431 | +.00392517 | .607066→.350043 | .0511917 |
| TX2274 | .252566 | .835905 | +.01757547 | .709796→.609875 | .0629542 |

Both admitted branches would fail R4 D<0. New position-sensitive weak projection
therefore supplies distinct information, not just a tighter score threshold.
Both actual raw translation choices improve post-hoc. TX2274 raw rotation
error worsens4.90037→6.53826deg, so the rule is not a guarantee of correctness
in every component. Across all evaluated pending contributions mean weak
fraction .740786. Shadow's four choices have3 raw translation improvements
and1 worsening. Feedback's23 rejected supported branches contain18 worsened
and5 improved raw translation proposals. These are descriptive post-hoc cases;
no candidate is added, removed, ranked or tuned with this GT.

## Primary corrected executed IMU trajectory

GT freezes/engineering checks precede GT loading; same original fixed anchor
and extrinsic, no fitted alignment or GT extrapolation.4126 GT-supported frames;
TX4127 remains in all full ledgers. Both feedback versions produce byte-identical
corrected trajectories and the same two actual feedback measurements.

| Executed method | Translation RMSE / P95 / max, m | Rotation RMSE / P95 / max, deg |
|---|---:|---:|
| Immutable CONTROL; both shadows | .869398049 /1.535711390 /1.763295591 | 2.603454144 /5.785783730 /11.589847175 |
| V0 feedback | .868705437 /1.537219202 /1.751469184 | 2.587419594 /5.749947484 /11.594126987 |
| V1 feedback | .868705437 /1.537219202 /1.751469184 | 2.587419594 /5.749947484 /11.594126987 |

Translation improvement0.0796657%, not5%; translation P95 slightly worsens.
Rotation RMSE/P95 improve slightly, rotation maximum slightly worsens. Large
jump rule >.5m OR >10deg:3 for CONTROL and each new run, no increase. Final
maximum IKFoM correction over all frames .187571503m/2.202638471deg.
Identical outcomes across the two algorithm versions are consistency evidence,
not a separately scheduled same-version replication or independent validation.

Raw actual-measurement metrics, separate from primary trajectory:

| Measurement | Translation RMSE / P95 / max, m | Rotation RMSE / P95 / max, deg |
|---|---:|---:|
| CONTROL nominal | .863610094 /1.533448187 /1.784590472 | 4.479268533 /11.013433986 /20.800057753 |
| Final actually used measurements | .863042738 /1.529391004 /1.747790228 | 4.467472398 /11.006564302 /20.464954302 |

Known +/-10-frame development windows: translation RMSE CONTROL→final feedback
TX616 .613993→.555840;TX2350 .744368→.713501;TX3341 .547828→.555446m.
The last window worsens. These do not replace full-sequence metrics and are
not held-out validation. All window and rotation results are retained.

## Main limitation, direct contribution, stop

Useful feedback remains2/4127. The observed displacement is largely weak, so
failure is not simply projection onto the wrong subspace. Anchor is valid in
~83% of frames, but25 temporally supported branches yield only2 useful admissions.
V1 removes known terminal-rank omissions without increasing feedback or RMSE
gain. Short anchors inherit earlier filter position bias; they distinguish
new short-time offsets, not accumulated absolute localization drift. Some
post-hoc useful branches are rejected. Counts support limited effective
correction opportunities; they do not prove that arbitrary looser admission
or longer anchors would fix the problem. Do not tune these on this cohort.

Direct contribution: lightweight causal non-absorption of recent NDT pose
corrections, frozen weak-subspace position evidence, anchor-aware observed
terminal selection, and real budgeted non-GT branch-to-IKFoM feedback. It
demonstrates the requested algorithm/integration, not substantial localization
accuracy gain. Engineering/performance PASS;5% accuracy goal FAIL. No production
deployment. Single next-stage proposal: useful weak-branch admission recall
under the existing budget, with anchor reliability explicitly examined; decision
AI must inspect the true source and archives and freeze that next contract.
No next-stage implementation or additional experiment is started.

## Verification and delivery limits

Release build;P7/P10 6/6,P9 41/41,R1 2/2; fresh read-only bounded review; hashes,
GT isolation, exact shadow-state parity, independent costs and CSV/JSON audits.
See VERIFICATION.md and AUDIT_CORRECTION.md for evidence, including the preserved
pre-GT Python carrier mismatch. Tolerance never loosened and runtime not repeated
for this evaluator fix. Historical R1-R4 data unchanged. Original workspace
user files/preferences untouched; code/commit in writable clone. Persistent
bundle/import/push instructions are supplied separately; push is NOT executed.
