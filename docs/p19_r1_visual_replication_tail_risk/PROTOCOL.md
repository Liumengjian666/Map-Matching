# P19-R1 frozen Visual replication and tail-risk protocol

Frozen before the single new replay. START_SHA: 5df7f8c729dd2ff3d53ffe44f25d183963be87fb.

- Reuse the P18 SSD map, V2 bag, visual CSV, dataset configuration and compiled binaries. No build, extraction, algorithm/parameter change, or external-disk read.
- Execute existing `scripts/p17_visual/run.py` exactly once with `--enable`, P18 dataset config, full duration, rate 0.25, scan-start, CONTROL, camera disabled, fixed lag 0.15 s, geometry disabled. Independent cache `.p19_r1_visual_replication_tail_risk/runs/repeat_visual`.
- Before launch check every frozen hash. Before playback the existing runner checks loaded node paths, map path and records actual ROS parameters. Compare all algorithm parameters after excluding only output paths and ROS process identities.
- Require 396 NDT, 396 APPLIED OOSM, 8004 high-rate states, 396 corrected states, 169 visual applications and zero late/nonfinite states.
- Compare full P18/P19 sensor-time arrays, cloud hashes/counts, deskew results, all pose stages, iteration/convergence and visual reasons, fixed-lag releases/replay counts, corrected/high-rate position, orientation and velocity. Translation tolerance 1e-9 m; rotation 1e-8 deg; score tolerance 1e-12*max(1,abs(score)); timestamps 1e-12 s; discrete data exact. Velocity exact unless an existing frozen tolerance can be demonstrated; do not invent one.
- Wall clocks, runtime measurements, callback arrival state/time and legal pre-deadline history watermarks are explicitly timing evidence, not fixed-release state equivalence. NDT source/pose and all sensor-time state outputs must still match. Compare callback/order-dependent ledgers separately from sensor-keyed physical outputs.
- Stop on important discrepancy. No third replay. Report metric similarity separately from trajectory replication.
- Freeze outputs before opening raw GT. Reuse unchanged `evaluate_p18.py` with its P18 freeze-format identifier for compatibility, additionally recording the P19 task. Reuse original Control V2 0–3 s full-pose transform; primary 3–35 s. No refit.
- Tail diagnostics: fixed 26–30, 30–35, 32–35 s; one-second half-open bins [t,t+1), last bin includes endpoint. Report all samples and >1/2/5 m counts.
- Error vectors in map frame: p_est - R_fit^T*(p_GT-t_fit). Longitudinal component uses offline GT motion tangent mapped into map frame; lateral is the orthogonal norm. Guard zero motion; do not call the tangent a Hessian weak direction.
- Locate both EKF maxima and matched counterpart errors, latest accepted scan and its Initial/Raw/Used stages at scan time, corrected output at actual release time, nearest visual application and age. Preserve timestamp/refpoint distinctions (NDT LiDAR converted to IMU; lineage already IMU).
- Report earliest Control/Visual divergence above frozen pose tolerances, plus first 0.12 m / 0.5 m / 1 m separation as descriptive existing-innovation/step/error scales, not new admission rules.
- Runtime: new NDT/deskew timings, VmHWM/CPU and wall time. CSV injection is not an online camera frontend.
- Only analysis scripts/tests, lightweight evidence and reports may change. No runtime source or configuration change. No next algorithm experiment in this task.
