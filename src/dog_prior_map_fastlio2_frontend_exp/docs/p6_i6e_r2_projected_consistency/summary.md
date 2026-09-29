# PAPER-P6-I6E-R2 — Projected Measurement Consistency

- Code baseline: `c9fec80b3cef5106ae415d0cf16a674e4bce9ce3`; implementation checkpoint: `7f9febec7cac4b4627af09f2ff51a47c9d0da485`; the final result commit returned in the handoff identifies the exact source and reports.
- Branch: `research/p6-i6d-full-algorithm`; frozen I6D baseline remains `a561c310e97510dc18265ebb3057ac9d586daaee`. Pinned FAST-LIO2 `7cc4175de6f8ba2edf34bab02a42195b141027e9`; DCReg `ce7db8220f549a4a4391729e3bf4de4d4ab74635`.
- Frozen maps, calibration, params, and input bundles were hash-validated by the existing runner. Frozen source and archived visual CSV hashes are retained per-run in provenance; GT was not used online.
- Release build and 3 standalone CTests passed; separate Catkin build and 6 Catkin CTests passed. Coupled weak-direction linear NIS difference `0`; center finite-difference epsilon `1e-7`, relative selected leakage `4.41622e-10` (`<1e-5`). Exact legacy replay regression passed all archived trajectory/reliability/visual-update fields excluding timing: see `legacy_regression.txt`.

## Eight closed-loop runs

P0 = LEGACY_BASE_NO_GATE, P1 = ADAPTIVE_NO_GATE, P2 = BASE_SELECTED_NIS, P3 = ADAPTIVE_SELECTED_NIS. Each run starts independently from identical frozen input/init.

| dataset | policy | frames | ndt_calls | m0_converged | uobs_valid | map_support_insufficient | zero_correspondences | lidar_attempted | lidar_committed | lidar_nis_rejected | visual_triggered | visual_quality_passed | visual_updated | imu_only_intervals | first_nis_rejection_time | first_map_support_loss_time | first_0_support_transaction | position_sigma_max | rotation_sigma_max | velocity_norm_max | runtime_mean_ms | runtime_p95_ms | runtime_max_ms | process_wall_s | peak_rss_mib |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Floor01 | P0 | 144 | 154 | 144 | 144 | 0 | 0 | 144 | 144 | 0 | 67 | 0 | 0 | 0 | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | 1.0283487 | 1.0000062 | 0.32165353 | 36.261145 | 59.698005 | 146.21997 | 5.64 | 99.085938 |
| Floor01 | P1 | 144 | 154 | 144 | 144 | 0 | 0 | 144 | 144 | 0 | 67 | 0 | 0 | 0 | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | 1.0283487 | 1.0000062 | 0.32156595 | 33.814061 | 53.385888 | 128.06298 | 5.21 | 98.414062 |
| Floor01 | P2 | 144 | 154 | 144 | 144 | 0 | 0 | 144 | 144 | 0 | 67 | 0 | 0 | 0 | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | 1.0283487 | 1.0000062 | 0.32165353 | 34.399638 | 55.602685 | 143.43248 | 5.32 | 98.960938 |
| Floor01 | P3 | 144 | 154 | 144 | 144 | 0 | 0 | 144 | 144 | 0 | 67 | 0 | 0 | 0 | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | NONE_WITHIN_WINDOW | 1.0283487 | 1.0000062 | 0.32156595 | 34.026727 | 55.974354 | 131.23526 | 5.25 | 98.761719 |
| Corridor01 | P0 | 149 | 377 | 149 | 127 | 22 | 9 | 125 | 125 | 0 | 39 | 43 | 31 | 24 | NONE_WITHIN_WINDOW | 12.447744995000001 | 125 | 1.0009459 | 1.0000001 | 19.696911 | 161.4258 | 373.51626 | 483.36737 | 24.24 | 55.449219 |
| Corridor01 | P1 | 149 | 339 | 149 | 116 | 33 | 28 | 116 | 116 | 0 | 35 | 43 | 28 | 33 | NONE_WITHIN_WINDOW | 11.741785913999999 | 122 | 1.0009459 | 1.0000001 | 23.599128 | 134.02658 | 374.92723 | 492.17376 | 20.14 | 55.828125 |
| Corridor01 | P2 | 149 | 287 | 149 | 87 | 62 | 57 | 62 | 57 | 5 | 24 | 44 | 19 | 82 | 3.1691741050000002 | 8.8169681129999997 | 92 | 4.368087 | 1.0000001 | 18.652131 | 121.63756 | 350.13574 | 365.69619 | 18.29 | 55.984375 |
| Corridor01 | P3 | 149 | 395 | 149 | 144 | 5 | 0 | 141 | 107 | 34 | 49 | 43 | 42 | 26 | 3.1691741050000002 | 13.859664112000001 | NONE_WITHIN_WINDOW | 1.0009459 | 1.0000001 | 14.106661 | 176.03511 | 364.72905 | 489.46342 | 26.42 | 55.890625 |

Runtime/resource fields in the table report elapsed wall seconds and peak RSS MiB from `/usr/bin/time -v`; per-frame mean/P95/max comes from the logged runtime CSV. This is a short 15-second validation only.

## Corridor01 diagnostic interpretation

- P0 source regression is exact. P0 first zero geometric correspondences is tx125 / 12.5486 s (support already below the frozen sufficiency rule at tx124 / 12.4477 s). P1 first zero is tx122 / 11.7418 s. P2 first NIS rejection is tx32 / 3.1692 s; first zero support tx92 / 8.8170 s. P3 first rejection tx32 / 3.1692 s; no zero-correspondence event in 149 frames, with first support-insufficient tx138 / 13.8597 s. These support indicators do not establish accurate map localization.
- NIS rejection counts P0/P1/P2/P3: 0/0/5/34. P2 retains 57 LiDAR updates, has 82 IMU-only intervals and 10 visual-only intervals, and ends with maximum position sigma 4.368 m; that is an observed operational cost of fixed-R gating on this sequence. P3 retains 107 LiDAR updates, rejects 34, has 26 IMU-only and 16 visual-only intervals, maximum position sigma 1.001 m. No policy is selected as final.
- Corridor01 tx47 shadow: selected residual norm 3.4048, base-R NIS 312.475, adaptive-R NIS 300.276, rank 5 / threshold 15.086. tx88: residual 3.5589, base NIS 297.744, adaptive 280.504, rank 5 / threshold 15.086. Both shadow gate variants reject without changing official state; P0 closed-loop still commits these updates.
- The first real gate rejection is tx32 / 3.1692 s, before the highlighted tx47/88 large innovations. At tx32 the projected fixed/adaptive covariance difference is effectively zero (`1.26e-14` Frobenius), so both gate variants make the same one-step decision. After that rejection P2 retains sufficient geometric support through tx87 and first reaches zero support at tx92; P3 first loses the frozen support criterion at tx138 and has no zero-support row through tx149. This temporal order is descriptive and does not prove that the gate caused either outcome.
- At tx60, local-only rank 5 and NIS 36.370; local+nonlocal rank 3 and NIS 2.734 (removed two additional dimensions). Comparing NIS across different ranks is not a quality ranking. Across Corridor frames 1–149, appended nonlocal response removes 2 dims on 39 rows and 1 dim on 2 rows.
- U_nonlocal first removes two additional dimensions at tx24 / 2.3624 s, before the first P2/P3 gate rejection and before the first material closed-loop departure from P0. At tx47 and tx88, projected base/adaptive R Frobenius differences are 0.02927 and 0.02904; adaptation reduces NIS but leaves both far above 15.086.
- Nonlinear nominal-vs-predicted leakage rises at tx47 to 0.5301 with 0.78035 rad pose difference and 0.2835 finite-difference selected response; tx88 leakage 0.4055, difference 0.6180 rad and response 0.2131. The nominal weak complement cancels nominal mapping to machine precision; at large prior/measurement attitude separation it does not cancel the measured nonlinear residual to the same degree. This identifies a consistency behavior requiring scientific review, not an automatic proof that the current Jacobian is invalid.
- Visual counts reflect the archived quality and conditional-update chain. Across Corridor P0 39 trigger events, 43 quality-passed events, 31 applied updates; P1 35/43/28; P2 24/44/19; P3 49/43/42. Trigger and quality-pass counts are not nested because the CSV has observations evaluated outside a trigger request.

## Evidence files

- [`FIRST_DIVERGENCE_ANALYSIS.md`](FIRST_DIVERGENCE_ANALYSIS.md) contains first divergence, exact predicted states, same-state P0–P3 branches, projected residuals/noise, and key U_nonlocal / nonlinear checks.
- [`mode_metrics.csv`](mode_metrics.csv), [`window_metrics.csv`](window_metrics.csv), [`legacy_regression.txt`](legacy_regression.txt) provide tabular summary and regression details.
- Every dataset/policy directory contains trajectory, reliability, runtime, resource, provenance, map-support, visual, projected innovation, shadow, nonlocal-basis, and linearization CSV outputs.
- Build logs, standalone and Catkin CTest logs, and focused mathematical test output are stored at this report root.

CONFIRMED_CODE_DEFECTS

The FULL directional LiDAR branch computed `decision.measurement_noise.covariance` but discarded it and always passed fixed `base_noise`; this code path is now explicitly policy-selected and exercised. The projected update lacked a read-only selected-row-space innovation diagnostic and gate; the refactor shares the same builder for evaluation/update and checks the selected NIS before state mutation.

CONFIRMED_MECHANISM_BEHAVIOR

Projected weak-direction checks, gate atomicity, covariance selection, clone isolation, and the physical coupled finite difference pass. Strict LEGACY output exactly matches R1 on the required fields. In the real Corridor short runs, selected-space gate rejects some updates; fixed-R P2 loses map support earlier, while P3 has fewer zero-support events but 34 rejects and more NDT calls. Shadow comparisons show materially large NIS at tx47/88 and additional U_nonlocal dimension removal at tx60.

UNVERIFIED_CAUSAL_HYPOTHESES

The results do not establish that a gate improves true localization, that the adaptive R is statistically calibrated, that retaining map support means correct localization, or that large NIS alone explains later map-support loss. The short closed-loop comparisons have no GT-based conclusion by design. The measured nonlinear leakage may contribute to directional-update behavior; causality and an appropriate linearization convention remain for the research controller to assess.

REMAINING_LIMITATIONS

Only the prescribed 15-second Floor01/Corridor01 runs were executed. Nominal/adaptive covariance remains an engineering model rather than calibrated innovation statistics. The fixed 99% chi-square values therefore describe a controlled gate experiment, not a certified probabilistic detector. P0 is the algorithm baseline, not GT. No final FULL policy is recommended, no thresholds were retuned, and no FixedLag, new visual factor, or further phase was introduced.
