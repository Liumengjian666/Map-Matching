# P20-R1 single-variable causal ablation (frozen before run/GT)

START_SHA: 389243ae84cedb9a75964c8d43584cf60e233961.

One new full Visual replay only, no baseline rerun. Baseline is archived P19 repeat_visual (beta=.6). Experiment is beta=0 in a separate YAML, differing structurally only at ndt_observation.velocity_blend. Same map/bag/visual CSV/binaries/init/IMU/NDT/OOSM/step limit/vision formula. No build or runtime-source modification. No geometry/camera/Coupled feedback.

Reuse existing p17_visual/run.py with --dataset-config P20, --enable, no duration/geometry, rate .25 and lag .15s. An external read-only parameter guard checks the actual ROS root parameters against P19 before playback; only output paths/process identities and beta may differ. Do not use the external disk.

Require396 NDT/converged,396 OOSM APPLIED/corrected,8004 high-rate,169 visual applications, same sensor-time arrays and fixed-lag releases/replay,0 late/nonfinite. Later source hashes/poses are allowed to differ as causal effects; differing raw frozen inputs are not.

Freeze all actual outputs and baseline references before reading raw GT. P20 receipt identity is P20_PRE_GT_OUTPUT_FREEZE; do not impersonate P18. Reuse original P10 Control0–3s full-pose transform with no fitting; primary3–35s plus26–30,30–35,32–35,34–35. LiDAR poses convert to IMU origin; EKF lineage already IMU. Relative drift, not map-frame absolute ATE.

Sensor-keyed analysis must locate first eligible velocity mix (dt in(1e-3,1)), its pre-mix state, NDT velocity, computed .4v+.6v_ndt or v unchanged, and actual post-IMU-replay velocity. Use NDT_CORRECTION_APPLIED before blend, and OOSM_REPLAY_COMPLETE after replay. Assert prefix equivalence before the intervention, then report first speed/state/deskew/source/initial/raw divergences. Position tolerance1e-9m, rotation1e-8deg; velocity exact comparison with reported amplitudes, no invented tolerance. Do not confuse future deskew coverage with acausal visual input.

All four pose layers: count and translation/rotation RMSE/P95/max, delta and percent change (experiment minus baseline). Tail high-rate threshold counts >1/>2/>5m and unchanged5s persistent crossings. Maxima matched on common sensor times; observations and corrected states use their own actual times. Maximum state jump is not GT error.

Predefined engineering screen: global max<=5.667156999562539m, global RMSE<=1.05*1.2481083256675167m and P95<=1.05*2.552406529827499m. Report individual metrics even if screen fails. Select supported factor/not sufficient/regression/mixed/invalid using mechanism, motion and accuracy evidence; no parameter search.

Measure actual NDT/deskew/callback timings, cumulative CPU/VmHWM/wall. Offline visual CSV is not an online frontend. Only experiment config, bounded analysis/tests/reports/lightweight evidence may be committed. No bags/PCDs, no earlier archive overwrite. Stop after this one run regardless of accuracy. Any NEXT is a proposal for the decision AI, never executed here.
