# P61-R7 frozen twenty-frame contract

Start: c649c9fa1743aada3301ddb4f75bce54eaf08532. Branch:
research/p61-r7-geode-continuous-baseline. No HIKVISION, GT, mapping,
Coupled, Recovery, candidate search, or full-return run.

Exactly raw ledger scans 2986..3005 are selected before registration; retain
all failures. Every scan is wholly after the R6 initialization boundary.
Original bag and map SHA are checked before extracting. Only /imu/data Xsens
and the twenty Livox messages enter the adapter. Real offset_time UINT32 ns
is retained beside FLOAT32 seconds for legacy layout compatibility. No
sensor timestamp is changed. Output reference header is exact integer
header_ns + maximum raw offset_ns. Native state arithmetic remains double
seconds; record its representational error separately.

Initialize existing 15D estimator's p,v,R,ba,bg,gravity from full-precision R6
state. Its error order is [p,v,theta,ba,bg], whereas native FAST-LIO has
[p,theta,extrinsic_R,extrinsic_t,v,bg,ba,gravity_S2]. The 23D covariance is NOT
copied. Use the existing fixed policy: position/velocity std 0.5m/0.5m/s,
orientation 5deg, biases .05 in their respective SI units. This is an
explicit heuristic restart uncertainty, not a posterior/cross-covariance
continuation. Frozen R6 covariance remains unchanged and is hashed.

The existing deskew's causal scan-end reference is IMU pose; publish its
LiDAR pose via T_M_L=T_M_I*T_I_L. Require identical integer reference stamp
before accepting it as NDT's initial guess, including frame one. A bounded
prediction queue handles cross-topic delivery order. No fallback to stale
initializer or previous-scan pose when this prediction is unavailable.

NDT uses existing source .25m voxel, cap1400, range .5..80m; target .15m twice;
Gaussian .8m, step .08, epsilon .001, max40, outlier .55. Check actual
target_cells_ leaves, not getResolution only. R7 opt-in fixes global load
order (resolution BEFORE input target). One normal align per scan only.
Existing step limiter .5m/5deg and NDT fusion ratios/velocity blend .6 remain.

## One bounded time repair

Existing OOSM applies a measurement to the IMU snapshot preceding its timestamp.
The 20ms admission tolerance does not make those times equal. Constant speed
v gives an extra v*delta position after full-interval replay. In the opt-in
R7 path only, advance the original head snapshot to measurement time with
its real held IMU value using the SAME propagation function, apply correction
there, save that timestamp, then replay remaining real IMUs to original now.
No future tail is used to predict the measurement time. Remaining fragments
keep the existing midpoint convention. This split is intentionally not claimed
numerically equivalent to old unsplit nonlinear/covariance propagation.
Default-off paths retain legacy behavior. No optimizer or filter formula is
replaced. The scan-local/nominal-consistent experimental modes remain false.

## Execution and interpretation

Use a private ROS master and output directory. Wall scheduling/use_sim_time=false,
all estimator math uses unchanged sensor stamps. Publish real IMUs through the
first end bracket; wait EKF high-rate ACK then publish the cloud; associate
OOSM receipt with that measurement_ns and wait corrected output before the
next scan. No sleeps substitute for algorithm completion. Bounded timeout
preserves failure artifacts. Live RViz uses real nodes, clouds and trails.
Twenty-frame continuity is not absolute accuracy, independent traversal,
or 10Hz certification. No GT evaluation is performed.
