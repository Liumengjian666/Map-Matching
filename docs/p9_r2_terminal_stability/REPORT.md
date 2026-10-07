# P9-R2 Low-Budget Terminal Stability

FINAL_RESULT = `TERMINAL_STABILITY_NOT_DISCRIMINATIVE`

This is perturb-and-reoptimize terminal-instability evidence. It is not basin recovery, a posterior, or a covariance.

Frozen cohort: 32 frames; 9 major-competitor and 23 no-major frames. New NDT calls: 2560.
NDT statuses: `{"ITERATION_LIMIT": 275, "SUCCESS": 2285}`.

## Probe contract

The 263 archived base seeds (indices 0–262) were projected into WEAK2, STRONG2, and three frozen RANDOM2 subspaces. Each projected 2-D pool was ordered by deterministic farthest-point selection; no proposals were renormalized or deduplicated. Selection manifests were frozen before any NDT call.

RANDOM2 uses PCG64 and seeds `20261007 + 1000*frame_index + replicate` (replicates 0–2). Budgets are 4, 8, 12, and 16. All 2,560 selected probes were evaluated once.

## Budget results

| B | Method | Major mean / median S | Healthy mean / median S | ROC AUC | frame permutation p | LOFO accuracy |
|---:|---|---:|---:|---:|---:|---:|
| 4 | WEAK2 | 1.631 / 1.826 | 1.809 / 1.798 | 0.420 | 0.7596 | 0.500 |
| 4 | STRONG2 | 0.3733 / 0.2894 | 0.3924 / 0.3138 | 0.498 | 0.5152 | 0.719 |
| 4 | RANDOM2_MEDIAN | 1.792 / 1.878 | 1.618 / 1.651 | 0.623 | 0.1540 | 0.625 |
| 8 | WEAK2 | 1.233 / 1.442 | 1.337 / 1.271 | 0.420 | 0.7604 | 0.531 |
| 8 | STRONG2 | 0.2875 / 0.2165 | 0.3415 / 0.4687 | 0.459 | 0.6467 | 0.688 |
| 8 | RANDOM2_MEDIAN | 1.425 / 1.369 | 1.404 / 1.388 | 0.498 | 0.5125 | 0.750 |
| 12 | WEAK2 | 1.1 / 1.259 | 1.272 / 1.322 | 0.362 | 0.8839 | 0.500 |
| 12 | STRONG2 | 0.2712 / 0.1772 | 0.3223 / 0.4329 | 0.415 | 0.7705 | 0.750 |
| 12 | RANDOM2_MEDIAN | 1.38 / 1.378 | 1.296 / 1.232 | 0.541 | 0.3664 | 0.469 |
| 16 | WEAK2 | 0.9974 / 1.144 | 1.162 / 1.145 | 0.372 | 0.8680 | 0.469 |
| 16 | STRONG2 | 0.2681 / 0.1544 | 0.2903 / 0.375 | 0.411 | 0.7824 | 0.750 |
| 16 | RANDOM2_MEDIAN | 1.3 / 1.275 | 1.254 / 1.25 | 0.531 | 0.3946 | 0.344 |

Random2 median rows use the per-frame median of the three pre-frozen random-subspace scores; individual random replicate ROC AUC values and ranges are retained in `budget_statistics.csv`.

## Frozen major frames and controls at B=16

Each cell shows `S_terminal / trace(A_terminal) / escape fraction`; RANDOM2 reports the median across its three frozen subspaces and the bracketed range for S.

| Frame | Label | WEAK2 | STRONG2 | RANDOM2 median [S range] |
|---:|---|---:|---:|---:|
| 368 | MAJOR_COMPETITOR | 0.741 / 0.667 / 0.25 | 0.118 / 0.0158 / 0.25 | 1.24 / 1.75 / 0.62 [0.881, 1.41] |
| 2226 | MAJOR_COMPETITOR | 1.09 / 1.26 / 0.81 | 0.106 / 0.012 / 0.44 | 0.864 / 1.34 / 0.62 [0.835, 1.11] |
| 2350 | MAJOR_COMPETITOR | 1.25 / 1.92 / 1.00 | 0.263 / 0.072 / 1.00 | 1.21 / 2.46 / 0.94 [1, 1.29] |
| 2722 | MAJOR_COMPETITOR | 1.14 / 1.31 / 0.31 | 0.0704 / 0.00625 / 0.19 | 1.28 / 2.64 / 0.94 [1.02, 1.31] |
| 3796 | MAJOR_COMPETITOR | 1.33 / 1.78 / 0.50 | 0.154 / 0.0253 / 0.25 | 1.72 / 5.21 / 0.88 [1.61, 2.03] |
| 120 | NO_MAJOR_BASIN | 1.61 / 2.81 / 0.62 | 0.477 / 0.238 / 0.31 | 1.65 / 5.14 / 0.94 [1.61, 1.81] |
| 244 | NO_MAJOR_BASIN | 1.58 / 2.5 / 0.31 | 0.0349 / 0.00147 / 0.06 | 1.83 / 5.23 / 0.88 [1.04, 2.17] |
| 740 | NO_MAJOR_BASIN | 1.43 / 2.05 / 0.62 | 0.0402 / 0.00179 / 0.06 | 0.453 / 0.431 / 0.81 [0.318, 1.29] |
| 838 | NO_MAJOR_BASIN | 1.48 / 2.2 / 0.25 | 0.527 / 0.314 / 0.19 | 1.5 / 3.31 / 0.88 [1.24, 1.54] |
| 839 | NO_MAJOR_BASIN | 1.48 / 2.2 / 0.25 | 0.143 / 0.0298 / 0.19 | 1.51 / 4.74 / 0.94 [1.28, 1.72] |

## Cost

Across all 2,560 calls, mean NDT alignment time was 69.216 ms/probe and mean iterations were 35.41/probe. Proposal generation, projection, and deterministic selection took 18.10 s total (0.565 s/frame).
At budget B, WEAK2 and STRONG2 each consume B extra calls/frame; RANDOM2 consumes 3B calls/frame across its three fixed subspaces.

| B | Method | NDT calls/frame | Mean NDT ms/frame | Mean iterations/frame |
|---:|---|---:|---:|---:|
| 4 | WEAK2 | 4 | 245.7 | 115.9 |
| 4 | STRONG2 | 4 | 192.5 | 93.3 |
| 4 | RANDOM2_REP0 | 4 | 277.8 | 156.3 |
| 4 | RANDOM2_REP1 | 4 | 275.5 | 153.5 |
| 4 | RANDOM2_REP2 | 4 | 268.3 | 151.1 |
| 4 | RANDOM2_ALL | 12 | 821.7 | 460.9 |
| 8 | WEAK2 | 8 | 478.8 | 220.0 |
| 8 | STRONG2 | 8 | 373.3 | 175.5 |
| 8 | RANDOM2_REP0 | 8 | 660.7 | 359.4 |
| 8 | RANDOM2_REP1 | 8 | 633.7 | 338.5 |
| 8 | RANDOM2_REP2 | 8 | 628.2 | 349.4 |
| 8 | RANDOM2_ALL | 24 | 1922.6 | 1047.3 |
| 12 | WEAK2 | 12 | 717.2 | 327.6 |
| 12 | STRONG2 | 12 | 520.3 | 244.3 |
| 12 | RANDOM2_REP0 | 12 | 1005.0 | 542.9 |
| 12 | RANDOM2_REP1 | 12 | 974.8 | 512.3 |
| 12 | RANDOM2_REP2 | 12 | 974.0 | 534.1 |
| 12 | RANDOM2_ALL | 36 | 2953.8 | 1589.3 |
| 16 | WEAK2 | 16 | 917.5 | 415.0 |
| 16 | STRONG2 | 16 | 657.4 | 308.6 |
| 16 | RANDOM2_REP0 | 16 | 1329.0 | 710.2 |
| 16 | RANDOM2_REP1 | 16 | 1309.4 | 684.1 |
| 16 | RANDOM2_REP2 | 16 | 1324.0 | 715.3 |
| 16 | RANDOM2_ALL | 48 | 3962.4 | 2109.6 |

Full all-method per-frame totals are retained in `results.json`; the same cost rows are archived in `budget_costs.csv`.

## Decision

| B | WEAK AUC | WEAK p | WEAK LOFO accuracy | STRONG AUC | RANDOM2 median AUC | Gate |
|---:|---:|---:|---:|---:|---:|---|
| 4 | 0.420 | 0.7596 | 0.500 | 0.498 | 0.623 | FAIL |
| 8 | 0.420 | 0.7604 | 0.531 | 0.459 | 0.498 | FAIL |
| 12 | 0.362 | 0.8839 | 0.500 | 0.415 | 0.541 | FAIL |
| 16 | 0.372 | 0.8680 | 0.469 | 0.411 | 0.531 | FAIL |

Best qualifying budget: `None`.

No GT, support event, canonical basin pose, posterior weighting, or EKF state was used.
