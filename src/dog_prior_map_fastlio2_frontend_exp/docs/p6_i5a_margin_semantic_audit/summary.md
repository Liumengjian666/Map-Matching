# PAPER-P6-I5A operational margin semantic stress test

Stage: `NDT_ONLY` complete; all figures and tables are existing-data offline analysis.

## Environment isolation

- New worktree: `/home/jian/livox_ws/dog_loc_p6_i5a_audit_ws`; branch `research/p6-i5a-semantic-audit`; start/analysis base SHA `ec35ced556f0f1eba192b928bf07452543c5c0dc`.
- Original dirty workspace `/home/jian/livox_ws/dog_loc_paper_ws` remained unchanged: before/after HEAD, branch, porcelain status, and tracked diff snapshots all compare byte-identically in `/home/jian/p6_i5a_environment_audit/`.
- `origin/paper` was verified at the frozen start SHA before analysis; P6-I4 files and runtime sources were not modified.

DATA_INTEGRITY: **PASS**
SEMANTIC_AUDIT: **COMPLETE** (GT used only for the frozen post-hoc diagnostics below)

## Frozen inputs and integrity

- Frozen P6-I4 CSV SHA256 entries checked: 9; all match `input_integrity.sha256`.
- Principal: 88 frames, 75 finite, 13 `SEARCH_CAPPED_NO_DETECTED_EXIT`.
- Dense: 24 frames, 20 finite, 4 `SEARCH_CAPPED_NO_DETECTED_EXIT`.
- Probe rows: 33815; recomputed acceptance mismatches: 0; duplicate extra-key conflicts: 0.
- Finite boundary endpoints: principal 75/75, dense 20/20; missing or conflicting endpoints: 0.
- Sampled reject-to-accept re-entry through the recorded outside endpoint: principal 0/75; dense 0/20. `sampled_reentry_audit.csv` records every winning ray; no re-entry among saved samples does not rule out unsampled transitions.
- Extra logical retention outcomes: 6144/6144; missing: 0.
- Retention conservation: 96 checks, errors 0; frozen dense reconstruction matches 24/24.
- Direction-held-out fold minimum: min(margin_fold_A, margin_fold_B) matches frozen m_dense for 24/24 frames; finite values use 1e-6 tolerance and censor state is checked separately.
- Frozen baseline replay rows: 4127; source-hash mismatches: 0; audit-script NDT calls: **0 by design** (offline code has no ROS/PCL/NDT import, subprocess, or registration runner). Frozen call-accounting totals describe P6-I4 history only.
- Censored is reported only as `SEARCH_CAPPED_NO_DETECTED_EXIT`: no detected transition on the saved 0.25 grid through alpha=3; this does not prove margin >3 because narrow accept/reject/accept intervals may be missed. Likewise, bisection brackets the first sampled transition only and cannot certify a continuous first-exit infimum between sampled alphas.

## First-exit phenotype

Counts below are for finite winning boundaries; principal and dense sets overlap and must not be summed as independent samples.

- Principal (n=75): non-convergence 0, translation only 31, rotation only 39, both 5.
- Dense (n=20): non-convergence 0, translation only 10, rotation only 9, both 1.
- Threshold-near descriptive ratio bin [1.0,1.1]: principal 51/75; dense 13/20. This bin describes tolerance proximity only; it is not a basin-switch rule.
- Principal terminal jumps (translation m, rotation deg): median 0.00626739, 0.0926254; P95 0.494201, 3.62256; max 0.900961, 6.80971.
- Dense terminal jumps (translation m, rotation deg): median 0.0242213, 0.352027; P95 0.298012, 3.62256; max 0.657605, 3.73161.
- Many winning endpoints lie just beyond a frozen tolerance, with small median terminal jumps, compatible with smooth-like terminal sensitivity/tolerance crossing. The upper tails also contain abrupt terminal responses. Neither observation proves nor excludes a different optimizer local minimum; no jump-based classifier is defined.

## Non-overlapping parity holdout

Odd D01..D31 form group A; even D02..D32 form group B. Fold AB uses principal+A margins and B retention; Fold BA uses principal+B margins and A retention. The primary correlations use finite margins only; censored rows are reported separately in `parity_holdout_statistics.csv`.

| Fold | Validation alpha | n finite | Spearman | LOO min / median / max | Excluding tx 1 |
|---|---:|---:|---:|---:|---:|
| AB | 1 | 20 | 0.809253 | 0.772899 / 0.822920 / 0.822920 | 0.772899 (n=19) |
| AB | 2 | 20 | 0.948489 | 0.939751 / 0.945590 / 0.965400 | 0.939751 (n=19) |
| BA | 1 | 20 | 0.794247 | 0.754544 / 0.806783 / 0.821684 | 0.754544 (n=19) |
| BA | 2 | 20 | 0.940120 | 0.929890 / 0.939734 / 0.958836 | 0.929890 (n=19) |

This is retrospective direction-heldout analysis, not independent-dataset validation or correctness validation; all folds reuse the same frames, objective, covariance, and sequence.

## Prior covariance and tangent range

- Transaction 1 vs 95: rotation trace 3.00004 vs 0.000857953 rad² (ratio 3.5e+03); translation trace 3.14789 vs 0.0335789 m² (ratio 93.7). Their principal margins are 0.0546875 and 2.17969.
- Finite principal-margin Spearman with rotation trace: 0.430243 (n=75); without tx 1: 0.486919. Translation trace correlations: 0.193691 / 0.242038. Descriptive only; rotation and translation units are kept separate.
- Winning principal delta-phi norm: mean 0.0145309 rad, P95 0.0408413 rad, max 0.0546878 rad; >=pi count 0.
- All saved probe rotation tangent norm `ALL_SAVED_ROWS`: n=33815, mean=0.0232418, P95=0.0362307, max=2.90234 rad, >=pi count=0.
- All saved probe rotation tangent norm `EXTRA_MARGIN`: n=18546, mean=0.0163993, P95=0.0343471, max=0.294597 rad, >=pi count=0.
- All saved probe rotation tangent norm `EXTRA_RETENTION`: n=817, mean=0.361319, P95=1.93489, max=2.90234 rad, >=pi count=0.
- All saved probe rotation tangent norm `PRINCIPAL`: n=13013, mean=0.0126514, P95=0.0349386, max=0.250002 rad, >=pi count=0.
- All saved probe rotation tangent norm `REPEATABILITY`: n=1439, mean=0.0152528, P95=0.0418831, max=0.250002 rad, >=pi count=0.
- Tangent norms are local coordinates, not globally unique SO(3) distances.
- For the ideal full-rank definition, scaling P to cP gives m(cP)=m(P)/sqrt(c); finite-grid/capped estimates need not obey this pointwise.

## Scientific answers (NDT-only)

1. Among finite winners, exits are classified by convergence and the frozen tolerances as tabulated above; non-convergence was not observed in the winning boundaries if its count is zero.
2. Near-tolerance endpoints plus small median inside/outside jumps provide smooth-like evidence, not proof of smoothness or absence of mode hopping.
3. The upper-tail jumps are observed abrupt terminal responses; they are not confirmed alternative local optima.
4. Direction-heldout associations remain positive in these retrospective folds; this does not establish cross-dataset prediction.
5. Margin is covariance-scale dependent by construction. Transaction 1 is a strong startup-scale outlier; its inclusion/exclusion sensitivity is reported, not corrected.
6. `wrong-basin detector`: **NOT DEMONSTRATED**.
7. Keep the frozen P6-I4 term for this audit; evidence supports considering the weaker phrase `Operational Registration Terminal Stability Margin` for future claims, but this report does not rename the frozen construct.

## Frozen state and limitations

`U_obs=PARTIAL`; `U_nonlocal=SUPPORTED MATHEMATICAL CANDIDATE`; Dual Reliability `INCOMPLETE`; novelty `UNVERIFIED`. These are not upgraded by this audit. No covariance, acceptance, threshold, P6-I4 result, or runtime algorithm is changed. All findings are based on saved probes, coarse/capped finite searches, one objective, one sequence, and a direction split on the same frames.

GT post-hoc is complete from the frozen table; official GT was not reopened.

## GT post-hoc counterexamples (frozen table only)

This section was produced only after `ndt_only_audit.csv` was hashed and verified. It reads only frozen `margin_with_gt_posthoc.csv`; official GT was not reopened and these values did not affect acceptance, boundaries, directions, parity, margin, retention, or thresholds.

- Descriptive nearest-rank quartiles: finite principal margin Q25/Q75 = 1.03125/2.21094; corrected translation error Q25/Q75 = 1.18231/8.88219 m. These cohort quartiles are only for finding counterexamples, not new operational gates.
- Bottom-quartile finite margin and bottom-quartile error examples: P2F001.
- Top-quartile or censored margin together with top-quartile error examples: NONE OBSERVED.
- Censored means no detected exit through the saved alpha cap, not a measured margin >3.
- `P2F001`, `P2F011`, `P2F014`, and `P2F017` are all listed below as pre-specified diagnostic examples.

| Frame | tx | m_principal | censored | corrected t error (m) | corrected r error (deg) | pattern |
|---|---:|---:|---:|---:|---:|---|
| P2F001 | 1 | 0.0546875 | False | 0 | 2.04921e-16 | BOTTOM_QUARTILE_MARGIN_AND_ERROR |
| P2F011 | 829 | 3 | True | 0.47824 | 1.57012 | OTHER |
| P2F014 | 888 | 3 | True | 0.843429 | 1.53034 | OTHER |
| P2F017 | 948 | 3 | True | 1.15354 | 1.31352 | OTHER |

Correctness prediction proven: **NO**. These examples constrain interpretation: operational terminal stability is not equivalent to localization correctness.
