# P61-R8 one full-return CONTROL run

Start aa8eadbff8c4238095105e2219669ff3eccbe12f. C++ sources identical to
R7 actual runtime17922f2b63dae1f2af34f353287728d50a312f73; archived launch and
recording changes are not claimed to have participated in R7's actual run.

Require full point intervals inside1706671587829840123..1706671831999708652ns.
Frozen ledger admits exactly2986..5426 (2441);2985 and5427 are whole-frame
exclusions. No timestamp shifts or partial-cloud boundary cropping. Source bag,
map, initialization,23D covariance and ledger SHA are verified. Only internal
disk; retain20GiB. New input and formal run paths, no overwrite or retry.

Use the actual frozen R7 NDT/EKF binaries and exact R7 numerical configuration,
including fixed15D restart uncertainty, velocity_blend=.6, step limiter,
scan-end prediction/deskew and the accepted OOSM boundary repair. New processes
restart at R6 t0, not R7 final state. No Coupled/Recovery/extra align/GT/SLAM.

R8 changes only input preparation, passive telemetry, launch/viewer paths and
bounded ACK terminal handling. Do not change C++ estimator code. Before data
publication, verify complete TCPROS peer connections and active numerical params.
The read-only R7 executable/overlay is reused; all ROS logs/output belong to R8.

For each scan, deliver real Xsens samples through its end bracket, await true
EKF high-rate watermark, publish its one real cloud, then await explicit terminal:
normal NDT+fusion; NDT failure with finite prediction/current IMU state;
fusion rejection; or fatal state/time/timeout. Never synthesize observations.
On NDT failure do not require an odometry message that the production node
does not publish. Record its diagnostics instead. Keep all failures. Continue
only for finite, time-legal IMU prediction states. Any actual time-contract or
nonfinite failure stops and preserves the run. No retries or reinitialization.

No GT is read before or after the run. NN/fitness are conditional diagnostics,
not absolute localization error. All native double stamps and exact input ns
are separately retained. OOSM correction must equal measurement native time;
future IMU may be replayed to current now, not used to predict measurement end.

One full run only. Real ROS+RViz with latched frozen map, clouds and actual
prediction/NDT/corrected trails. Record screenshot progress sidecars. Full CSV
trajectories are complete even where production RViz trails are bounded.
Record process VmRSS/VmHWM independently, do not sum HWM as unique memory.
Completion is not accuracy, basin correctness or sustained10Hz certification.
