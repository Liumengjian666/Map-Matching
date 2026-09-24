# P3-R8 cross-sequence registration mechanism synthesis

## Scope and provenance

Offline synthesis only. Inputs are Corridor01 R4/R5/R6 and Floor01 R7G/H/I formal reports and CSVs. No runtime source, YAML, launch, map, bag, or frozen baseline was changed; no bag was replayed. Input CSV SHA-256 values:

- Corridor R6 initialization: `c271adab1bed02e91c6a9e825b101807c9ee8ab67cb7272ab73ef42745c631e2`
- Corridor R6 objective: `851aa8371c9b2c8ae30bd8d196dd58cd3cb0dd7923ba2a91c299851f88a7707a`
- Corridor R6 perturbation: `3b4611b9c113537f17a06248562aee8cac088e70a7162ebfc33b5ac2210f1128`
- Floor R7I initialization: `e4e352455800d79f2fd9af063c09f46f80789cc684f5ce5d5d1dc2d9efe7623d`
- Floor R7I objective: `2581df8b852f400cba9c97729f63197fd42685dd8f842aa94bfca2913775415a`
- Floor R7I perturbation: `56b9587af351c2b7fb354a2421cb695966048a58444f4f5c251cc121a898b5eb`

Pose displacement audit: maximum absolute difference between each source `distance_raw_to_init_m` and independently recomputed translation norm from the logged 4x4 initial/raw poses was `4.95e-06 m`. All three initializations converged on all selected frames (Corridor 65/65 per arm; Floor 279/279 per arm).

P95 values here use NumPy linear percentile; standard deviations use sample `ddof=1`. Oracle correction ratio is numerically ill-conditioned because its measured initialization error is zero in these CSVs; use oracle push-away and endpoint errors, not that ratio, for interpretation.

## Primary counterfactual windows

| Sequence/window | n | baseline raw mean | oracle raw mean | perfect-increment raw mean | paired baseline−oracle mean | oracle strong retention |
|---|---:|---:|---:|---:|---:|---:|
| Corridor01 R6 W1+W2 | 34 | 0.472038 | 0.138888 | 0.481061 | 0.333150 | 25/34 |
| Floor01 R7I W3+W4 | 119 | 1.535763 | 0.645010 | 1.493783 | 0.890753 | 0/119 |

The phase/window populations are intentionally not time-aligned: Corridor R6 samples the early failure window; Floor R7I primary samples persistent failure. The fixed cloud means each oracle run is conditional on the same baseline-conditioned observation, not an oracle closed-loop trajectory.

## Findings

1. Both frozen baseline executions passed their repeatability gates, and both sequences show sustained relative failure (R4/R3B; R7H).
2. On the tested fixed clouds, changing initialization measurably changes raw NDT endpoint error in both sequences. This is the smallest common fact; it does not establish a shared physical root cause.
3. Corridor01 adds an early-window near-reference preservation pattern: R6 reports 52/65 oracle endpoints inside 0.15 m/2 deg versus 20/65 baseline endpoints; within W1+W2, oracle retention is 25/34. R6’s 0.5 m anchor endpoint separation is 0.621507 m / 2.992 deg.
4. Floor01 bounds that result: in persistent W3+W4, oracle improves paired mean translation error by 0.890753 m, but zero of 119 oracle endpoints are inside 0.15 m/2 deg. The effect is dominated by W4 (baseline−oracle mean 2.131530 m); W3 mean improvement is 0.022210 m.
5. Perfect current increment is not sufficient for recovery in either tested regime. Corridor W1+W2 perfect-increment mean is 0.009023 m worse than baseline; Floor W3+W4 improves by 0.041980 m but still has 1.493783 m mean error. Thus Floor has a small measurable benefit, not zero effect.
6. PCL objective preferences are mixed by metric and/or frame. In Floor W3+W4 all 119 converged; aggregate means favor oracle in fitness (0.167364 vs 4.054451) and transformation probability (1.603575 vs 1.437539), but per-frame wins are nearly balanced. This is not evidence of a wrong mode or multimodality.
7. Corridor predictor evidence is sequence-specific: R4/R5 describe previous-NDT-delta reuse and insufficient raw correction in part of onset/maintenance; R5 leaves the initial 0–3.068 s mechanism unresolved. Floor has logged initial-guess error only, not a complete predictor/carry decomposition. A predictor-only solution is NOT_SUFFICIENTLY_SUPPORTED.
8. Both full-SE3 deskew clouds use causal prior NDT pose history for translation. Each counterfactual holds those observations fixed; conclusions are conditional on baseline/prior-NDT-conditioned clouds.

## Protocol corrections

- The R6 C label is legacy descriptive logic: its branch means the oracle endpoint is outside the near-reference box, not that oracle translation error exceeds baseline. The E branch is shadowed by earlier B/C decisions and is structurally unreachable. No A–F label is used as a primary P3-R8 mechanism metric.
- R6 narrative perturbation counts 16/24, 0/24, 0/24 are inconsistent with the advertised 0.15 m/2 deg threshold: the stored stable bit used 0.10 m/1 deg. Standardized recomputation at 0.15 m/2 deg is Corridor 23/24, 22/24, 24/24; Floor is 24/24, 0/24, 0/24, 0/24.

## Required answers

- Q1 common minimum fact: measured registration-level initialization sensitivity on fixed input clouds in both sequences, under different predeclared failure windows.
- Q2 Corridor01 adds: in its early-failure windows, a reference-near initialization often remains near reference after NDT; the effect is not universal and does not identify a mode.
- Q3 Floor01 boundary: even oracle initialization can be pushed far from reference during persistent failure; aggregate gain is window-dependent and does not imply endpoint preservation.
- Q4 predictor-only proposal: `NOT_SUFFICIENTLY_SUPPORTED`.
- Physical root cause: `UNRESOLVED`; multimodality/wrong mode/Hessian degeneracy: `NOT_PROVEN`.
- P4 eligibility: no candidate satisfies cross-sequence mechanism + online-observable + GT-independent criteria; `P4_ALLOWED = NO`.
- Next recommendation: `P3-R8A-MINIMAL-DISCRIMINATIVE-EXPERIMENT` — predeclare a small set of Corridor01 persistent-phase anchor frames and rerun the same per-frame baseline/oracle/perfect-increment comparison on already frozen baseline clouds, with Floor W3/W4 retained as the contrasting boundary. This discriminates whether Corridor’s early oracle-retention pattern persists into late failure without changing runtime or adding a dataset. Do not implement a fix in that experiment.

## Files

This directory contains the required CSV matrix, phase mapping, per-frame sequence metrics, oracle-retention and NDT-displacement ledgers, grouped summary, candidate hypotheses, online observability audit, and four plots. The paper-workspace report is the only intended Git artifact.


## Standardized local perturbation table

| Sequence | Anchor | Near / total at 0.15 m and 2 deg |
|---|---|---:|
| Corridor01 | ANCHOR_0P25 | 23/24 |
| Corridor01 | ANCHOR_0P5 | 24/24 |
| Corridor01 | ANCHOR_LIDAR_0P5 | 22/24 |
| Floor01 | A1_INSTANT_050 | 24/24 |
| Floor01 | A2_PERSISTENT_025 | 0/24 |
| Floor01 | A3_PERSISTENT_050 | 0/24 |
| Floor01 | A4_PERSISTENT_100 | 0/24 |
