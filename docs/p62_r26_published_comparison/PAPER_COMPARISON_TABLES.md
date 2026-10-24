# Three protocol-separated manuscript tables

NR = not reported at this scope. TableA is published reference context; TableB is a controlled causal comparison. They are deliberately not merged into an algorithm leaderboard.

## Table A. Published results on original Newer College Short

|Algorithm|Primary result source|Sequence / range|Task / position metric|Alignment|Published m|Compute, original scope|Grade|
|---|---|---|---|---|---:|---|---|
|FAST-LIO2|Chen2023 TableI,p5; later quoted by2Fast Table7|Short1609.40m, nominal complete sequence*|Odometry ATE RMSE|evo; exact original flags NR; compiled later tables state Umeyama|0.3775|42.86ms, five-sequence average, i7-11800H|B|
|DLIO continuous|Chen2023 TableI,p5|Same|Odometry ATE RMSE|evo; exact original flags NR|0.3606|35.74ms, same five-sequence average/hardware|B|
|STEAM-LIO|Burnett T-RO2025 TableII,author PDFp10|01-Short, full sequence|Odometry ATE RMSE|Umeyama SE(3)|0.3042|74ms NC average, XeonE5-2698v4/16threads|B|
|Singer-LIO|Burnett Robotica2025 TableIII,printedp698|01-Short, full sequence|Odometry ATE RMSE|Umeyama|0.3020|NR at NC scope|B|
|2Fast-2Lamaa global-map odometry|Le Gentil IJRR2026 Table7,author v3p18|01_short1609m/1530s|Odometry ATE RMSE|Umeyama|0.262|Real-time claim; separate Boreas resource profile in TableC|B|
|2Fast-2Lamaa global-map localization|Same Table7,localization row|Same|Fixed prior-map localization position RMSE|Direct map-frame comparison; no trajectory fitting in metric definition|0.128|Separate Table12 resource profile, not NC timing|B|

*Chen §IV-B1 omits the first100 FAST-LIO2 poses for some sequences; which sequences NR. Rotational Short results NR for all six rows. 2Fast's compiled baseline values are FAST-LIO2 .378,DLIO .360,STEAM .304, sourced from earlier publications, not new independent runs. Retain the originals above for attribution and record all printed variants in `PUBLISHED_BASELINE_RESULTS.md`.

The official1530s Short description, originalOS1 sensor suite and exact15301 public-trajectory timestamps substantiate the sequence aliases. They do not equalize task, initialization, sample exclusions, body frame or metric. No cross-table performance percentage is valid.

## Table B. Our controlled causal comparison, W1/C3

Same60 raw scans2601–2660, reference-initialized anchor2600 with map-yaw+8deg, realIMU/Deskew/EKF, unchanged NDT/quality parameters, target2297615points/FNV11658115094751181825. The methods have independent causal histories; processed/deskewed point coordinates may diverge with those histories. Reference-relative EKF error uses actual fused-state time, no trajectory fit. A/C are sealed historical runs, B is the completed R25 ablation; R26 reruns none.

|Method|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|Nominal+extra Full|Map accepted/rejected|High-error / longest no-map|First sustained low-error raw|
|---|---|---|---|---|---|---|
|Original hard pre-gate (R23)|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|0+0|0/60|60/60|NONE|
|G-only (R25)|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|60+0|0/60|60/60|NONE|
|C+G (R24)|0.209943/0.300124/0.444282|2.984149/3.723287/11.245245|60+8|57/3|2/2|2603|

All60 G-only calls converge, but no candidate passes unchanged post-quality admission. C+G admits the -5deg restart at raw2603, Q=.296356, and later resumes nominal fusion. Max rotation11.245245deg and the initial2 high-error outputs remain. This exposed development case supports the joint mechanism, not independent generalization. A is the old integrated path with its recovery branch unreachable, not 60 failed native NDT calls. Same-protocol normal controls remain in R24 (W1/C0 .191979m/2.115287deg; W3/C0 .175791m/2.009151deg). The R20/R21/R22 historical-predictor tables belong to separate cohorts and are not appended as if they used this IMU/EKF protocol.

## Table C. Computational and memory characteristics

|Experiment / method|Timing, measured scope|Memory, reported scope|Hardware / restrictions|Original locator|
|---|---|---|---|---|
|P45 global target|Steady align7.086ms|NDT process HWM775308KiB|Historical measured profile; model not recorded in selected evidence; not an identical-binary benchmark|P45 REPORT/runtime table; R25 RESOURCE_COMPARISON|
|P45 local target|Steady align8.079ms; input→finalack33.82s/29.90s sensor span|NDT process HWM138200KiB; active552716 vs3142751targetpoints|Same matched300-scan study;82.2% observed per-process reduction; tail-error and latency tradeoffs; no live tile crossing|Same|
|R23 original W1/C3|Callback Mean/P95/Max30.352/39.254/602.224ms|NDT259060KiB; simultaneous NDT+EKF282196KiB|50ms RSS sampling; excludes Deskew/publisher/RViz; hardware model NR in selected archived receipt|R25 RESOURCE_COMPARISON citing R23|
|R25 G-only W1/C3|110.496/179.406/977.302ms|NDT369136KiB; pair389380KiB|Independent causal run;60aligns; single run, no repeated-trial speed inference|R25 RESOURCE_COMPARISON|
|R24 C+G W1/C3|55.304/111.302/1006.031ms|NDT369096KiB; pair392536KiB|68aligns; larger2297615point target; not P45 whole-system budget|Same citing R24|
|FAST-LIO2 run by Chen|42.86ms per scan, five NC sequences averaged|NR|Intel i7-11800H, paper describes16-core; RAM NR; no matched on-host test|DLIO TableI and §IV,p5–6|
|DLIO continuous|35.74ms, same scope|NR|Same Chen platform|Same|
|STEAM-LIO|74ms, NC average|NR|Intel XeonE5-2698v4,16threads|T-RO TableIIp10, hardwarep9|
|Singer-LIO|NR for NC|NR|NR|Robotica §7/TableIII|
|2Fast baseline odometry|41ms undistortion,61ms keyframe registration,54ms map update (19ms no carving)|0.17GB undistortion +0.33GB localization final RAM (separate modules)|i7-1370p/32GB; typical Boreas-RT Suburbs, not Short; averages not end-to-end sum per LiDAR frame|Author v3§9.7/Table12,p21|
|2Fast baseline localization|MeanCPU8.86%; component cores1.06/0.71|0.17/0.32GB final component RAM|Same platform/Boreas resource ablation; not HWM/RSS-pair|Table12,p21|
|2Fast global-map localization|MeanCPU9.16%; component cores1.05/0.78|0.16/2.24GB final component RAM|Same platform; not measured NC Table7 row cost|Table12,p21|
|SuperLoc|45ms/frame,22FPS|NR|AMD Ryzen73700X8-Core; different experiments; no shared-code/on-host timing|Paper§IV-E,p6|
|FAST-LIO-LOCALIZATION|README map corrections0.2–0.5Hz|NR|Hardware NR; correction cadence is not full estimator rate|Official README Features|

No inter-paper speedup or memory-reduction percentage is computed. GB is left as published; KiB is the archived measurement unit. Component final RAM, process high-water memory and simultaneous sampled two-node RSS measure different things. C+G callback P95 exceeds100ms and maximum includes initialization; these data do not certify10Hz system operation. Lower C+G mean than G-only reflects changed optimization histories/work, not proof that more starts reduce computational complexity.
