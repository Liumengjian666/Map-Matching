# P3-R9C summary

## Gate and populations

- Parameter-equivalence gate: PASS; unexpected NDT-affecting differences: 0.
- Formal legacy Run A NDT outputs: 4136; mature Run A: 4126.
- Exact common NDT timestamps: 4126; legacy-only: 10; mature-only: 0.
- GT-supported full populations: legacy 4130; mature 4126.
- Common GT-supported paired population: 4126; HQ bracket <=0.25 s: 4106.
- Common anchor: `1660857393.6012471`; all raw/final anchor errors are zero within numerical tolerance (max=1.42e-17).
- GT support exclusions (no extrapolation): legacy early=6, late=0; mature early=0, late=0.
- GT bracket mean/P95/max on common pairs: 0.202589/ 0.201720/ 0.403440 s.
- Expected startup history difference: `EXPECTED_PIPELINE_DIFFERENCE`; only the first exact common frame has a different logged initial-guess source/prior availability, as recorded in R9B PREINIT_REJECTED. This is disclosed, not treated as numeric initialization equivalence.

## Full GT-supported final_used metrics

- Legacy translation: mean=31.9253030435 m, rmse=41.9642866363 m, median=32.3746172829 m, p95=64.4726989112 m, max=68.1564586934 m; rotation: mean=62.7727914712 deg, rmse=80.5181707683 deg, median=67.324591682 deg, p95=138.924649104 deg, max=178.827388842 deg.
- Mature IMU translation: mean=42.8430766096 m, rmse=55.5987724985 m, median=56.8677088222 m, p95=83.7174382986 m, max=85.6932648211 m; rotation: mean=32.4418704637 deg, rmse=43.276836168 deg, median=31.3997806936 deg, p95=87.3520459076 deg, max=111.884917645 deg.
- Paired-common final translation legacy-minus-mature: mean=-10.8868680531 m, bootstrap 95% CI [-11.197034733, -10.5715055842]; median=-12.7030298716 m, bootstrap 95% CI [-12.8341852875, -12.6245998198].
- Paired-common final rotation legacy-minus-mature: mean=30.391593149 deg, bootstrap 95% CI [29.1874675721, 31.6338184858]; median=1.30332439249 deg, bootstrap 95% CI [1.15664394798, 2.34101679647].
- Paired final translation counts (tie tolerance 1e-6 m): mature better=1181, legacy better=2944, tie=1; rotation counts (1e-6 deg): mature better=3186, legacy better=939, tie=1.
- Raw-NDT and final-used metrics are separately retained in `p3_r9c_global_summary.csv`; per-frame raw-to-final pose changes are separately retained as translation/rotation deltas and are not interpreted causally.

## Persistent crossings (seconds relative to the common anchor; exact sample stamps in CSV)

- Legacy: 0.25m=37.8203678131s; 0.5m=98.534662962s; 1m=138.271308899s; 2m=138.977272987s; 5m=140.086675882s.
- Mature IMU: 0.25m=37.8203678131s; 0.5m=98.534662962s; 1m=138.674692869s; 2m=138.977272987s; 5m=140.389256001s.
- Rapid persistent 1 m to 5 m: legacy=1.81536698341 s; mature_imu=1.71456313133 s.
- Rapid classifier evidence: relation=`WITHIN_ONE_COMMON_NDT_FRAME`, mature-minus-legacy interval=-0.10080385208 s, one-sample resolution=0.100859880447 s. Resolution is the median spacing of exact common NDT timestamps; the classifier treats an interval difference within that measured cadence as the same rapid-failure timing pattern.

## Classification

- FAILURE_CLASSIFICATION: `MIXED_STAGE_DEPENDENT_EFFECT`. The classifier consumes 1 m/5 m persistent status and the 1 m→5 m interval relation as well as paired full-population metric signs and the W0/W1-versus-W3/W4 translation-delta sign pattern. A rapid interval unavailable on only one pipeline or a difference greater than one median common-frame interval is a mixed-effect signal; when paired/window signs conflict, both rapid intervals must be present for the mixed label. The one-frame resolution is data-derived from this run's common NDT sampling cadence, not a claim of a preregistered scientific threshold.
- OLD-MECHANISM TRANSFERABILITY: `NOT_DIRECTLY_TRANSFERABLE`.
- W3 paired final translation mean delta (legacy−mature): 0.0134767500223 m; W4: 0.187969521276 m.
- W0/W1 paired final translation deltas (legacy−mature): -0.0153022450742/-0.13170531351 m; full paired final translation/rotation mean deltas: -10.8868680531 m / 30.391593149 deg.
- Persistent severe thresholds remain present in both pipelines: legacy 1 m=True, 5 m=True; mature IMU 1 m=True, 5 m=True. The mixed label reflects different error effects across stages/metrics; it does not mean the sustained failure disappeared.
- The detailed raw-NDT/final-used, window, bootstrap, better/tie-count, and high-dynamic tables are in the accompanying CSVs. NDT fitness/iterations/convergence are descriptive only.

This is a comparison of two closed-loop motion-compensation infrastructures, not an isolated deskew ablation. No physical root cause, collision causality, wrong mode, multimodality, Hessian/geometry degeneracy, or novelty is claimed. `P4_ALLOWED = NO`.
