# P62-R16 — real NDT weak-constraint case

## Result

`FROZEN_G_WEAK_DIRECTIONS_OBSERVED`: all 30 evaluated scans have three directions satisfying the unchanged R9 ratio threshold 0.01. This establishes a real-input weak-spectrum case under that inherited geometry diagnostic. A wrong NDT basin or a causal link from those modes to localization failure is **not established**.

`REFERENCE_RELATIVE_ADMISSION_FALSE_REJECTION_OBSERVED`: local frame 18, one-based raw ordinal **7570**, timestamp **1583837348070227968 ns**, provides a concrete prediction/admission problem. Both native NDT outputs are closer to the official reference than the common prediction in translation and rotation, but both are rejected by the unchanged 5° prediction-correction boundary. This is a conditional benchmark-reference observation, not independent physical-truth certification.

No recovery or Coupled method ran. The task stopped after the one window, 60 native align calls and one offline reference evaluation.

## Git and executed source

- Repository: `https://github.com/Liumengjian666/Map-Matching`.
- Workspace: `/home/jian/livox_ws/p62_r16_degeneracy_ws`.
- Branch: `research/p62-r16-real-ndt-degeneracy-case`.
- START: `8cfbdb48fd4ceb540ade908364ce6d98ed4b9ffa`.
- Source archive commit: `65b2c92edb0d5a48c02799f9a824653073d90ff8`. This commit was created **after** the run; there was no separate pre-run Git CODE_SHA. Actual executed source bytes and binary were frozen before NDT in `NDT_PRE_RUN_FREEZE.json` and still match this archive.
- Executed `tools/run_ndt.cpp` SHA256: `1616e1db7ba8d5b1b92f113e9e56b3817f22600141256e2a7b02b6580bf7a8b6`.
- Executed binary SHA256: `3f838fc7620706caa02653b85bc31d950c396d2ef398d29a52f00d12a36c636d`.
- Local commits only; no push was executed.

The runner reuses R14B `common.hpp` for PCD conversion and the causal predictor, R6 `NativeNdt`, Coverage V2, and R10 detection/admission. Its new code performs input selection, 30-frame orchestration and reporting. Historical headers and archives have no edits. There were no writes to the stable machine-dog workspace.

## Input-only selection and frozen assets

The canonical R14B inventory has 15,301 frames. After the first 5% of sequence time, fixed stride 20 selected 726 scans for scanner-frame XYZ covariance analysis after the exact R14B preprocessing. The lowest `lambda_min/lambda_max` chose one window. Including its unsampled frames, the prescreen read 756 unique PCDs; no map, reference pose values, NDT quality or localization error was used to rank scans.

The winner was ordinal 7567 (ratio 0.0062502251). The frozen window is **7552–7582**, anchor 7552 and evaluations **7553–7582**. These are one-based canonical inventory ordinals. This differs from old raw768–787 and does not reuse it as a new test window. `WINDOW_SELECTION.md` and `TEST_FRAME_PREREGISTRATION.json` contain the exact paths, times, Source FNVs and SHA256 identities.

The official Leica PLY SHA256 remains `e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0`; `registered_poses.csv` remains `b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`. Only selected scans and these key assets were checked, not another full-sequence hash scan. Before/after receipts agree.

All 31 original PCDs contain 65,536 declared points with point step 29 bytes and 3,846 extra zero bytes. Native PCL exposes exactly the declared binary records; their XYZ bits equal the original records. The padding is not parsed as points. No original or canonicalized PCD was written. Source cap is 1,400; all frames satisfy Training >=300 and Coverage V2 >=4 active blocks.

One static 50 m sphere ROI was centered at anchor LiDAR translation `[17.31460266, -56.21804509, -0.09963786]` m and voxelized at 0.20 m. It contains 239,748 points; anchor Overlap@0.5m is 0.777857. Derived map SHA256 is `ecc3b8d11e68ad28633e2aa4fb6effae1a5ccaeb6a8e1eb20d81e6fcc4ae7eca`, path `/home/jian/livox_ws/p62_r16_degeneracy_data/newer_college_7552_7582_roi.pcd`. No evaluation reference pose was used to crop the ROI.

## Coordinates, time and prediction

R14B's conditional `PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION` is retained:

`T_M_L = T_M_B * T_B_C * T_C_I * T_I_L`.

The first frame reference initializes the map pose once. Evaluation-frame pose values do not enter prediction or NDT. All 31 PCD filenames have exact integer-nanosecond reference timestamp pairs; no time offset or trajectory alignment was fitted. No new deskew was enabled; intra-scan motion distortion remains a limitation.

All methods share the R14B body/right constant-velocity prediction from historical admitted CONTROL_FULL poses. With less than two states, or after a Full rejection, it holds the previous valid pose. Split results do not update this predictor. This is a common-prediction comparison, not two independent closed-loop systems.

The reference is `MAP_ANCHORED_BENCHMARK_REFERENCE`: Ouster-to-Leica registration, not independent mocap. The Base interpretation has paper/model support but no direct CSV-export identity binding. Errors are **Reference-Relative Pose Error** under this inherited assumption.

## Native experiment and geometry

PCL 1.10.0 NDT used unchanged resolution 0.8 m, step 0.08, epsilon 0.001, 40 iterations and outlier ratio 0.55. Each of the 30 frames ran one Full and one Training Split align: **30+30**, extra align **0**, Coupled **0**, EKF **0**. Both methods converged on 30/30 frames. Both admitted 29/30 observations; the one rejection is the same local frame 18. Split R10 events: old predicate 0, complete state-machine event 1 (3.33%).

`G` is evaluated at the final Split pose on Training points using the exact R6/R9 geometry function. It is a PSD geometric information approximation, not the native PCL objective Hessian. With `rx=R*p`, its Jacobian is `[0.8 I, -skew(rx)]`; Gaussian inverse covariances and inherited guarded Gaussian weights contribute `J^T C^-1 J`, averaged over Training. Therefore the eigenvalue ratios and direction coordinates inherit R9's physical scaling.

All 30 G matrices are valid and have exactly three ratios <=0.01. Minimum-ratio range is 0.00135078–0.00195616, median 0.00163321. The fourth ratio is always above 0.0215365. Condition-number range is 511.20–740.31. This does not certify complete translational unobservability: spectra depend on the inherited scale and the matrix is not a calibrated pose covariance.

At raw7570 the six eigenvalues are approximately `[5.730, 6.818, 8.049, 87.254, 2041.576, 3509.481]`. The weakest eigenvector is approximately `[-0.3108, 0.9504, 0.0126, -0.0006, -0.0036, 0.0016]` in the recorded six-component G coordinates; the three weak modes are translation-dominated mixtures. Their signs are arbitrary. The event causing rejection is rotational, so this observation alone does not attribute it to these weak modes.

## Reference-relative errors (30 valid samples each)

Native outputs and decisions were frozen in `NATIVE_OUTPUT_FREEZE.json` before evaluation-frame reference poses were parsed. No per-method SE3 alignment was applied.

| Method | Translation RMSE / P95 / Max (m) | Rotation RMSE / P95 / Max (deg) |
|---|---|---|
| Prediction | 0.276983 / 0.405369 / 0.503635 | 2.417265 / 3.816524 / 4.874107 |
| CONTROL_FULL | 0.251744 / 0.319795 / 0.320876 | 1.820046 / 2.924292 / 3.399635 |
| CONTROL_SPLIT | 0.273339 / 0.356910 / 0.414515 | 1.913452 / 2.661948 / 3.475724 |

Full is closer than Prediction on 20/30 translation and 21/30 rotation comparisons; Split on 13/30 and 19/30. These are correlated frames from one roughly three-second window, not independent scenes or a full-trajectory ATE result.

At **local18/raw7570**:

| Method | Translation reference error (m) | Rotation reference error (deg) | Rotation from TP (deg) | Overlap@0.5m | Admission |
|---|---:|---:|---:|---:|---|
| Prediction | 0.292366 | 4.874107 | — | — | prediction only |
| Full | 0.238731 | 1.833534 | 5.627697 | 0.876984 | rejected |
| Split | 0.269044 | 1.958694 | 5.829730 | 0.874339 | rejected |

The observed disagreement reflects a prediction-relative gate rejecting two reference-closer measurements. It is not evidence that a >5° correction is intrinsically wrong. After Full18 was rejected, Prediction19 exactly equals accepted Full17 (matrix max-absolute difference 0); its translation error is 0.503635 m. Full19/Split19 reduce that to 0.320876/0.414515 m. This traces the hold fallback, without asserting that another feedback policy is already validated.

No wrong NDT basin is established. All weak-spectrum flags coexist with moderate reference-relative errors; there is no non-weak control window and no intervention establishing causation. The causes of the remaining pose errors—geometry, raw-scan motion distortion, reference/base convention or matching bias—remain UNKNOWN.

## Cost and limitations

Native process wall time: **69.3849 s**. Mean/P95/Max Full align: **1567.81/2489.20/2805.52 ms**; Split: **690.98/1055.04/1481.49 ms**. Mean PCD load 6.94 ms, preprocessing 8.89 ms, Coverage 5.29 ms, G calculation 24.92 ms; mean frame wall 2309.92 ms. Formal-run reported peak RSS is 70,264 KiB. One-time ROI construction took 6269.84 ms, reported peak RSS 575,084 KiB; NDT map-load/grid initialization 43.65 ms.

The CMake build type was empty, actual CXX flags `-std=gnu++14` without Release optimization. These are measured prototype costs; they neither establish production speed nor a fair optimized runtime comparison. `RUNTIME_AND_MEMORY.csv` is a documented normalized copy: the immutable raw run CSV has one extra startup-row field; no per-frame measurements changed.

`TESTS_AND_REVIEW.md` records the bounded build/format review and all preparation failures. No second native experiment or new window followed them. Input identities, historical files, and native output hashes remain unchanged. The stable workspace was never a write target; no independent stable-worktree before/after snapshot is claimed.

## H1–H3 and NEXT

- **H1:** Yes under the frozen R9 G diagnostic: 30/30 scans, three weak modes each. Physical loss-of-observability certification remains open.
- **H2:** A reference-relative 5° admission false rejection is observed at raw7570; a causal geometry-to-NDT-failure relation remains UNKNOWN.
- **H3:** The directly exposed issue is prediction-relative hard admission and its hold fallback, rather than proven NDT wrong-basin convergence. `INNOVATION_OPPORTUNITY.md` gives one conditional, admission-level idea and its literature overlap.

**Unique NEXT:** ask the research lead to preregister one literature-established long-corridor case with a traceable reference and reproduce an actual NDT failure before approving another mitigation algorithm. Do not tune this window, relax thresholds, resume Coupled, or treat the admission example as proof of a new recovery method.
