# Corridor01 deterministic OOSM replay R1

## Outcome

The fixed sensor-time scheduler removed the asynchronous Control/Shadow OOSM
endpoint difference. The pre-GT deterministic gate passed over the complete
396-scan run. Weak-only feedback improved the primary relative-drift score on
this development sequence; Coupled feedback did not. Coupled's primary
translation RMSE was substantially worse than both Control and Weak-only.

This is an engineering determinism result plus a negative/contrasting
algorithm result on a repeatedly used development sequence. It is not
independent generalization evidence and does not establish absolute map-frame
ATE.

## Replay integrity

Each mode processed 396 scans and ran 396 full PCL NDT alignments; all 396
reported convergence. All 396 deskew records were `PUBLISHED`. Each mode had
396 fixed-lag `QUEUED` and 396 fixed-lag `APPLIED` entries, zero late
observations, and 396 OOSM `APPLIED` results. There were 8,004 high-rate IMU
publication ledger entries and 396 correction/replay events per mode. No
non-finite EKF state was found.

Control and Shadow matched on all 396 NDT diagnostics, cloud identity fields,
deskew outputs (excluding processing-time fields), fixed-lag release nodes,
OOSM outcomes, and the three recorded pose topics. Pose differences were
zero at the output-topic comparison. Internal lineage-to-topic comparison
had maximum translation difference `1.43e-14 m`; rotation difference was at
most approximately `2.1e-6 deg`, due to serialization/roundoff. In particular,
the historical event at NDT stamp `1517157219.693280 s` released in both runs
at `1517157219.844208 s` and replayed 31 IMU samples.

Shadow performed 19 event triggers, 7 refinement attempts/recommendations,
14 joint-jet and 13 value-only calls, and zero feedback. Weak-only performed
22 triggers, 9 recommendations and 9 actual feedback changes. Coupled
performed 27 triggers, 8 recommendations and 8 actual feedback changes; its
strong-direction step was selected 6 times. No mode exceeded 396 full NDT
align calls; the shadow/refinement computations reused the existing NDT
backend.

## Posthoc evaluation

The evaluation origin is `1517157224.188979 s`, i.e. five seconds after the
first NDT sensor stamp. A single full-pose SE(3) transform was fit from the
first 3 seconds of Control_V2 and shared unchanged by all methods. Its fit
translation RMSE was `0.0891 m`, rotation RMSE `1.337 deg`. The position-only
cross-covariance singular-value ratio was `5.45e-8`; position-only Kabsch
rotation differed from the full-pose transform by `4.82 deg`. Therefore the
frozen full-pose fit is the declared protocol; the near-rank-deficient
position-only fit is only a diagnostic.

Primary results use evaluation time 3–35 s (6,396 high-rate samples per
method), outside the fit interval:

| Method | Translation RMSE / P95 / max (m) | Rotation RMSE / P95 / max (deg) |
|---|---:|---:|
| Control_V2 | 1.888 / 5.011 / 5.667 | 3.026 / 3.706 / 16.394 |
| Weak-only feedback | 1.064 / 2.212 / 4.530 | 2.679 / 3.646 / 13.187 |
| Coupled feedback | 6.399 / 15.040 / 16.120 | 3.316 / 3.590 / 17.537 |

Weak-only reduced translation RMSE by `43.62%` relative to Control_V2. Coupled
increased it by `238.94%` relative to Control_V2 and by `501.13%` relative to
Weak-only (i.e. its error was about six times Weak-only's). The 0–10 s
translation RMSEs were similar: Control `0.469 m`, Weak-only `0.466 m`,
Coupled `0.467 m`; the large Coupled degradation emerged later.

Persistent crossing is defined as error staying above the threshold for at
least 5 seconds, allowing sample gaps up to 0.25 seconds. The 0.5 m crossing
was `6.450 s` in all three branches. Persistent 1 m crossing was `26.783 s`
for Control, absent for Weak-only, and `22.284 s` for Coupled. Persistent 2 m
crossing was `29.823 s` for Control, absent for Weak-only (it first crossed
briefly at `33.273 s`), and `24.298 s` for Coupled. Thus Coupled did not delay
the Control drift onset; it entered sustained large drift earlier.

All three algorithms had 396/396 optimizer convergence. That is an optimizer
status, not localization correctness. NDT score improvement or convergence
must not be equated with lower trajectory error.

## Candidate/causal interpretation

The coupled strong step was selected on six feedback frames. On those frames,
the recorded candidate differed from the same-run weak-only pose by only
`1.0–6.2 mm` and `0.026–0.158 deg`. Nonetheless the independent causal
branches diverged over time: by evaluation `21.93 s`, Coupled translation
error was `1.035 m` versus `0.630 m` for Weak-only; Coupled crossed 5 m at
`26.283 s` and 10 m at `28.833 s`. At evaluation `31.005 s`, its error was
`12.454 m`, compared with `0.994 m` for Weak-only and `3.295 m` for Control.

Because the two feedback runs have different causal state histories, this
does not isolate a per-frame strong-term effect under identical later states.
The complete algorithmic result is still clear: this Coupled feedback run was
not accurate or robust on the fixed sequence. The tiny local strong corrections
and large downstream divergence point to path sensitivity/branch evolution;
the present evidence does not identify a unique single-frame root cause.

## Runtime and resource accounting

Per-frame component timing is summarized as follows. `Deskew + NDT callback
to odometry publish` is a stage-sum from the logged per-frame components; it
is not a hard real-time guarantee. Fixed-lag arrival-to-apply time includes
sensor-time scheduling wait and is not CPU time.

| Mode | Deskew mean / P95 / max (ms) | NDT callback→publish mean / P95 / max (ms) | Stage-sum mean / P95 / max (ms) |
|---|---:|---:|---:|
| Control_V2 | 14.22 / 18.58 / 23.88 | 14.02 / 19.87 / 80.26 | 28.24 / 35.43 / 91.71 |
| Shadow_V2 | 14.13 / 18.62 / 22.88 | 14.17 / 20.86 / 81.05 | 28.30 / 36.28 / 95.94 |
| Weak-only | 14.18 / 18.84 / 23.27 | 14.29 / 20.23 / 71.31 | 28.48 / 35.75 / 85.74 |
| Coupled | 13.96 / 18.35 / 23.07 | 14.21 / 20.41 / 73.38 | 28.17 / 35.80 / 90.36 |

The measured fixed-lag queue-arrival to application wall delay was about
`168.4 ms` mean (approximately `178 ms` P95); it is scheduling latency, not
per-frame compute. Peak RSS was not captured and is reported as
`NOT_CAPTURED`, not estimated.

## Coupled rosbag limitation

The Coupled feedback `topics.bag.active` contains only 366 NDT and corrected
messages and 7,391 high-rate messages because the recorder did not finalize
the final part of the bag. It is explicitly marked incomplete. The full
`ekf_lineage.csv` is synchronously flushed for each publish event and contains
all 396 correction/replay events and all 8,004 high-rate publish events with
finite state. Its available output-topic prefix was pose-checked against the
recorded bag; the full coupled metrics above use the complete internal
publication ledger. The incomplete bag is not described as a complete bag.

## Final interpretation

- Fixed-lag sensor-time OOSM determinism: passed for Control_V2/Shadow_V2.
- Weak-only feedback: lower relative drift than Control_V2 on this sequence.
- Coupled feedback: no accuracy advantage; substantially worse translation
  error and earlier sustained 1 m / 2 m drift.
- The result does not support an accuracy claim for Coupled on this Corridor01
  run. It also does not license posthoc parameter changes on this same run.
