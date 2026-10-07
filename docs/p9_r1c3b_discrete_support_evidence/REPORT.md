# P9-R1C3B discrete support-transition evidence

## Result

`FINAL_RESULT = DISCRETE_SUPPORT_EVIDENCE_NOT_DISCRIMINATIVE`

Interpretation is restricted to **material fixed-support response to support transitions**. Dynamic NDT multiple self-consistent local minima were not established.

## Frozen contract

- 32 frozen Floor01 frames: 9 major-competitor frames / 22 frozen basins and 23 predeclared no-major frames.
- Local event construction used the fixed P9 map-product chart and six 1-D deterministic stencils at offsets -0.02, -0.01, 0, +0.01, +0.02; 768 adjacent edges total.
- Each changed edge compared its endpoint supports at the same midpoint `(u,v)` using fixed-support DOUBLE strong minimization. H1 basin labels were joined only after event construction.
- The 24 R1C3A archived support-pair rows are supplemental historical examples, separate from the cohort classifier.
- `NDT_ALIGN_CALLS=0`; no GT, posterior, trajectory error, or EKF was used.

## Cohort event counts

- Support transitions: 768
- Moderate-only material responses (`EVENT_MATERIAL`): 0
- Strongly material responses: 768
- Unresolved fixed-support pairs: 0
- Historical supplemental pairs: 24
- Dynamic support evaluations: 3433
- DOUBLE energy evaluations: 1540383
- Cohort engine wall time (seconds): 124.6
- Pairs whose endpoints both match their frozen support hypothesis: 0

Resolved event classes are mutually exclusive; `EVENT_MATERIAL` is the moderate-only band, while `EVENT_STRONGLY_MATERIAL` takes precedence.

## Frame-level major vs no-major tests

| Feature | Major mean | No-major mean | Difference | ROC-AUC | one-sided permutation p | LOFO accuracy | LOFO AUC range | positive omitted gaps |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| repeatable_event_count | 5.11111 | 5.30435 | -0.193237 | 0.410628 | 0.830292 | 0.375 | 0.366848:0.451087 | 0/32 |
| FRAME_MAX_Iv | 1011.53 | 1081.75 | -70.2195 | 0.507246 | 0.636204 | 0.4375 | 0.472826:0.532609 | 0/32 |
| FRAME_MAX_IE | 1.50538e+08 | 5.79094e+08 | -4.28556e+08 | 0.231884 | 0.99635 | 0.40625 | 0.20202:0.26087 | 0/32 |

The primary discriminator is `repeatable_event_count`. The 100,000-replicate frame-label test uses PCG64 seed 20261016 and plus-one correction. The 32 frames—not the individual events—are the statistical units.

## Per-frame evidence

| tx | class | major basins | rho_W2 | events | moderate-only | strong | repeatable stencils | max Iv | max IE |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 120 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 596.579 | 3.24865e+08 |
| 244 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 645.173 | 1.95042e+08 |
| 368 | MAJOR_COMPETITOR | 1 | 0.130827 | 24 | 0 | 24 | 6 | 889.289 | 1.32266e+08 |
| 616 | MAJOR_COMPETITOR | 6 | 0.882682 | 24 | 0 | 24 | 4 | 1502.27 | 1.27472e+08 |
| 740 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 744.414 | 5.97523e+07 |
| 838 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 556.883 | 2.29993e+08 |
| 839 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 541.389 | 2.74713e+08 |
| 864 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 881.439 | 2.60978e+08 |
| 924 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 1111.55 | 3.2198e+08 |
| 925 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 4 | 938.95 | 2.95862e+08 |
| 1111 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 811.414 | 6.15482e+08 |
| 1235 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 1505.74 | 1.36706e+09 |
| 1359 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 4 | 2332.5 | 2.00393e+09 |
| 1497 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 1764.71 | 1.52397e+09 |
| 1498 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 4 | 1906.62 | 1.70148e+09 |
| 1556 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 4 | 1430.28 | 1.04109e+09 |
| 1557 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 1829.75 | 8.84243e+08 |
| 1606 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 1457.15 | 7.12632e+08 |
| 1730 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 1014.08 | 5.54525e+08 |
| 1854 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 1175.81 | 5.16733e+08 |
| 2102 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 506.517 | 6.33739e+07 |
| 2226 | MAJOR_COMPETITOR | 2 | 0.907247 | 24 | 0 | 24 | 5 | 1044.05 | 1.01243e+08 |
| 2350 | MAJOR_COMPETITOR | 3 | 0.885889 | 24 | 0 | 24 | 5 | 1329.13 | 1.4122e+08 |
| 2598 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 510.237 | 1.73171e+08 |
| 2722 | MAJOR_COMPETITOR | 2 | 0.190744 | 24 | 0 | 24 | 6 | 695.692 | 1.29176e+08 |
| 2846 | MAJOR_COMPETITOR | 1 | 0.545116 | 24 | 0 | 24 | 5 | 951.78 | 5.26079e+07 |
| 3094 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 1108.1 | 5.8023e+07 |
| 3217 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 5 | 966.527 | 7.25682e+07 |
| 3341 | MAJOR_COMPETITOR | 5 | 0.543078 | 24 | 0 | 24 | 5 | 1314.4 | 1.87586e+08 |
| 3631 | NO_MAJOR_BASIN | 0 | N/A | 24 | 0 | 24 | 6 | 544.482 | 6.76969e+07 |
| 3796 | MAJOR_COMPETITOR | 1 | 0.999311 | 24 | 0 | 24 | 5 | 713.029 | 2.76837e+08 |
| 3962 | MAJOR_COMPETITOR | 1 | 0.567916 | 24 | 0 | 24 | 5 | 664.16 | 2.06436e+08 |

## H1 association (secondary, nine major frames)

| Feature | Spearman rho with H1 rho_W2 | two-sided permutation p |
|---|---:|---:|
| repeatable_event_count | -0.627495 | 0.0636394 |
| FRAME_MAX_Iv | 0.233333 | 0.549115 |
| FRAME_MAX_IE | 0.166667 | 0.678203 |

## Interpretation

The output keeps support hash changes, fixed-support impact, dynamic-support self-consistency, and frame discrimination distinct. A support hash change alone is not evidence. Even a material fixed-support response is not a certificate of multiple dynamic local minima.

The result gate uses the preregistered primary feature: one-sided permutation p < 0.05, ROC-AUC >= 0.80, LOFO threshold accuracy >= 0.80, LOFO minimum AUC >= 0.70, and a positive major-minus-no-major gap after every single-frame omission. See `results.json` and the CSVs for complete detail.

## Provenance

Branch: `research/p9-r1c3b-discrete-support-evidence`<br>
Start SHA: `81d90c60d06ba0eba155072727f98d7600c2c558`<br>
PCL: 1.10; Release binary SHA256: `a86e518def47f95716e40241d9a88303dae70ff24be07103d028721988799fe3`
R1C3A results.json has an obsolete embedded self-hash; its independent artifact_hashes.json matches the actual file and was used as the authoritative check. Historical files were not modified.
All input and source SHA256 values are recorded in `results.json` and `execution_manifest.json`.
