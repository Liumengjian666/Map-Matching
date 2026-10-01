# PAPER-P6-ALG-INTEGRATION-A3E-R1

A3E_R1_TX155_NUMERICAL_ROOT_CAUSE_ISOLATED = PASS

PRIMARY_ROOT_CAUSE_CLASS = A3E-R1-A:
ROUNDED_NORMAL_EQUATION_COMPONENT_INDEFINITENESS_AMPLIFIED_BY_SCHUR.

MINIMAL_REPAIR_FAMILY = FAMILY-3: SQUARE-ROOT / QR MARGINALIZATION.
No repair implemented. REAL_REPLAY_COUNT=0. READY_FOR_FORMAL_EXPERIMENT=NO.
All Schur minima refer to the active retained 15D block. When this block is
positive, the full 600D Schur minimum is 0 because of 585 exact disconnected modes.

## Outcome

The -97.635 production Schur mode is **already present in the rounded
consumed information matrix**: exact-binary-entry 100-digit Schur minimum
is -97.63861721468, confirmed at 120 digits. The dominant identified source
is the aggregate residual between accumulated production H_c and separately
computed component matrices. It is not evidence of a theoretically indefinite
IMU measurement, nor primarily a new error in the double Schur solve.

| Check | Result |
|---|---|
| Frozen capsule SHA and reconstruction | PASS |
| Three negative + three small-positive directional oracles | PASS |
| Lifted-Hc identity and component accounting | error <1e-65 |
| Independent component sum, 100-digit Schur min | +3.29438085912 |
| Rounded H_c, 100-digit Schur min | -97.63861721468 |
| Component PSD cleanup, independent HP sum | +3.29793595644 |
| Cleanup + original assembly residual | -97.63475048898 |
| H_mm 2-norm condition, HP | 3.12026006228e11 |
| Scaled H_mm condition, double | 135.305384333 |
| Scaled-double recovered Schur min | -610.924324147 |
| Scaling congruence, high precision | relative error 4.84e-89 |
| Rank: eps*15 / 1e-14 / 1e-12 / 1e-10 | 15 / 15 / 15 / 12 |
| Release build | PASS |
| Full Release CTest | 33/33 PASS (old 32 preserved) |
| New SPD oracle self-test | PASS |
| git diff --check | PASS |
| Production estimator files changed | NONE |
| Real replay / GT / production clamp | 0 / NO / NO |

## Limits

The residual includes normal-information accumulation, final symmetrization
and independently recomputed diagnostic arithmetic. The capsule cannot isolate
one addition instruction or recover original factor J/R. Independent synthetic
stress showed weak-curvature errors but did not reproduce PSD loss; that outcome
is explicitly retained. Scaling is not a validated repair. QR success is not
claimed; it is the single prioritized repair family for the decision AI.

Independent read-only skill-driven review corrected mpmath negative-index
handling and strengthened the known-SPD self-test. It also caused the aggregate
assembly attribution limit to be recorded explicitly. External cross-model
review remains manual after handoff.

## Git / reproducibility

START_SHA=03119f8e0afeae9ed12b9ca6c6ec10c9a2e9ad39
CODE_SHA=9c116de72d489fc4cc494a94723f2d2742534f31
Branch=research/p6-i6d-full-algorithm
Workspace=/home/jian/livox_ws/dog_loc_p6_i6b_ws

END_SHA is the result commit containing this file, reported literally in the
final execution message. The source commit is recorded in `reconstruction.json`.
No self-referential commit hash is embedded in its own commit.

Read `ROOT_CAUSE.md` for the full attribution, `TX155_RECONSTRUCTION.md` for
input/trace gates, and `DECISION_AI_HANDOFF.md` for the detailed Chinese handoff.
