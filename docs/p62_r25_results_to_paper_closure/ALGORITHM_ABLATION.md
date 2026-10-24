# G-only causal ablation: MULTISTART_ADDITIONAL_CONTRIBUTION_OBSERVED_ON_DEVELOPMENT_CASE

A is frozen R23 W1/C3 MULTISTART_C under the original pre-gate (search unreachable). B is a new independent G-only run with the same post-quality rule and zero search. C is frozen R24 G-mode, i.e. C+G, not rerun. All60 raw scans, real IMU/calibration/initialization/Deskew and map-selection parameters are identical. Processed points may differ once filter histories diverge, as part of the intended causal effect. Actual target set remains the same2297615-point FNV11658115094751181825.

|Method|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|Nominal/extra Full|Map accepted/rejected|High-error / longest no-map|First sustained low-error raw|Callback Mean/P95/Max ms|NDT / pair peak KiB|
|---|---|---|---|---|---|---|---|---|
|A_ORIGINAL_R23|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|0/0|0/60|60/60|NONE|30.352/39.254/602.224|259060/282196|
|B_G_ONLY_R25|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|60/0|0/60|60/60|NONE|110.496/179.406/977.302|369136/389380|
|C_C_PLUS_G_R24|0.209943/0.300124/0.444282|2.984149/3.723287/11.245245|60/8|57/3|2/2|2603|55.304/111.302/1006.031|369096/392536|

B performed60 native Full alignments,60 converged,0 map admissions,0 extra calls; its entire EKF and TP matrix sequences equal A. Removing the pre-gate alone made the optimizer reachable, but did not yield a candidate passing frozen post-quality. This is not a claim that all rejected raw NDT poses are incorrect.

C used60 nominal+8 extra calls; the -5deg seed at raw2603 yielded the first accepted extra candidate (Coverage .296356), after two rejection frames. raw2604 was rejected again; raw2605 onward nominal matching and fusion succeeded. Total57 observations,3 rejections. The first actual map update and first frame satisfying the existing .5m/5deg reference-relative criterion for the entire remaining approved window are raw2603. That recovery-time statistic is posthoc, not an online decision.

Both translation and rotation complete-trajectory RMSE engineering targets pass for C; neither passes for A/B. C still has2 high-error frames, maximum rotation11.245245deg. G enables access; fixed multistart bridges the initial post-quality gap in this one known development case. Neither component is established as generally sufficient, novel, or cross-dataset safe. No parameters/methods were revised after B.

Native B outputs were sealed in0d4a56f55b6550a61d5c97fa6945d853cc8e49a5 before reference evaluation. Reference remains conditional-Base, map-anchored Benchmark, not independent mocap. Errors use actual EKF current-state times, not scan-start reference. Rejections are evaluated using real IMU outputs, not zero errors or fake measurements.

![Complete errors](figures/ablation_errors.png)
The A/B curves coincide. X-axis is scan-start elapsed time, whereas each error uses its own actual EKF state time. Green line denotes first C+G map update, not an independent truth certification.

![Coverage](figures/coverage_admission.png)
Missing selected scores are NaN gaps, never filled with zero. Full NDT uses these points; the plotted quality is not independent holdout evidence.
