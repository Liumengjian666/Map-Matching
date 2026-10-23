# R21 frozen developmental method

Start `9ffb2349ebe83022f3f03e49bd618e00ed267fb9`; R19 windows/maps/122 scan identities reused, WEAK anchor14542 eval14543–14602; STRONG anchor10972 eval10973–11032. Only C0 and C3(+8° map yaw at anchor). R19 BASELINE and R20 Gate-only/Multistart are historical references, never rerun. This is development data already examined against benchmark reference.

## Common quality and ranking

Native Full PCL NDT, exact R6 parameters 0.8/.08/.001/40/.55, R14B Source preprocessing and R20 basic quality unchanged. Coverage mean<=.30 is the absolute gate. Nominal pass is accepted without extra search. Event nominal failure permits the two R20 yaw seeds: left-map-axis rotation `Rz(±5°)R_TP`, position unchanged, + then −, both frozen before either align result. If any candidate passes the absolute gate, choose the R20 maximum native score; exact score tie uses `hypot(translation_from_TP/.8, rotation_deg_from_TP/5)` then lower ID. No .8m/5° feedback bounds are added. These are optimizer-only policies, not EKF.

Only when no candidate passes the absolute gate, an extra candidate may be relatively admitted if it passes basic quality and strictly improves all three same-frame nominal metrics: higher native score, higher overlap, lower active Coverage mean; plus physical difference beyond <=.10m AND <=1°. The nominal comparator must have a finite valid SE3, native score/fitness, valid active block losses and finite overlap in [0,1]. It need not be converged or meet overlap>=.50; invalid metrics disable comparison. Pareto is strict, no new margin/absolute threshold. Multiple Pareto candidates use the same fixed ranking.

Full Source includes the inherited odd-sector Coverage points. The `heldout_mean` field is a geometric quality statistic here, not independent held-out evidence. GT never enters candidate ranking.

## D: SINGLE_FRAME_RELATIVE_ADMISSION

Absolute candidate acceptance has priority. Otherwise directly admit the highest ranked Pareto candidate, explicitly `RELATIVE_ONLY_ADMISSION`. If none, current output is TP fallback; no rejected pose enters accepted history. Each policy/arm has an independent R14B Predictor. Only current accepted poses append history. Rejection sets hold; next prediction uses last accepted pose.

## E: TWO_FRAME_SHADOW_CONFIRMATION

For the same new-search result that D would relatively admit, E creates one `PENDING_SHADOW_CANDIDATE`, returns TP fallback and does not update the official Predictor. A pending candidate has one pose and timestamp; its next-frame seed uses the R14B one-history rule: hold that pose, i.e. `T_shadow_seed(t+1)=T_pending(t)`. No velocity is inferred from the large recovery correction, and the official accepted history is neither modified nor copied into a fabricated moving shadow history.

**User-confirmed precedence:** if next-frame nominal Full passes the absolute gate, accept nominal and cancel pending with `CANCELLED_NOMINAL_ABSOLUTE_PASS`, zero extra calls. Otherwise spend one extra native Full align at the pending seed using the actual current scan. Accept if current shadow passes absolute gate (`CONFIRMED_ABSOLUTE`); else require current shadow Pareto-dominates current comparable nominal (`CONFIRMED_RELATIVE_ONLY_ADMISSION`). Reject/clear if neither. Only the current confirmed pose is appended to official history; the earlier pending pose is never backfilled. Confirmation frame never simultaneously starts a new yaw search. Pending is cleared after exactly the next frame, including failure/cancel. No multi-frame shadow window.

## Shared budget and boundary cases

Each of eight 60-frame trajectories permits 60 nominal and <=12 extra aligns. Search costs exactly two; confirmation costs one. A confirmation can use the last remaining slot; a new yaw search requires two remaining slots. Search does not reserve an extra slot retroactively. If a pending candidate was formed with no budget left, next-frame nominal still runs: nominal absolute pass cancels it; otherwise it is dropped as `CONFIRMATION_BUDGET_EXHAUSTED` without uncounted work. At frame60 a newly formed pending is immediately marked `PENDING_EXPIRED_WINDOW_END`, never evaluated on a 61st test frame. These non-attempt cancellations are recorded separately from real confirmation failure. No extra search on ordinary accepted nominal frames. Per-frame extra<=2, per-trajectory<=12, global nominal480+extra<=96=576 maximum. No Split, Coupled, EKF or historical baseline rerun.

Candidate IDs: 0 nominal, 1 +5°, 2 −5°, 3 true next-frame confirmation. All raw candidates retained. `selected_id` is the actual ID rather than vector position. Raw output, candidate finite/converged flags, all losses, Pareto components, budgets, pending lineage and both official history timestamps are logged. Physics-only duplicate counts are diagnostics and never delete candidates from this method.

## Frozen post-output evaluation

All eight native trajectories and decisions are sealed before reading evaluation reference values. Reuse R20 reference evaluator and conditional Base T_M_L chain. No SE3 fitting. Reference is MAP_ANCHORED_BENCHMARK_REFERENCE, not independent motion capture.

Report every full60-frame trajectory: Translation/Rotation RMSE/P95/Max, event/relative/direct/pending/confirmation/cancel/fallback counts, longest fallback, all extra costs and RSS. Selected observations missing on fallback remain NA. Recovery is separately described by accepted extra/relative decisions and post-hoc reference changes. Use R20 descriptive high error (>0.5m OR >5°), three consecutive low error (<=0.3m AND <=3°), subsequent high-error episodes and delays. These labels never select online candidates.

STRONG C3 target requires BOTH output RMSE<=.35m and <=3°. Compare E vs D and R20 Multistart, retaining complete raw10973 onward chain. C0 material side effect uses same-window R19 C0: increase>.03m OR >.3°. Report the same deltas vs R20. If E matches or worsens D, do not claim temporal benefit. No reference-based parameter changes or second run.
