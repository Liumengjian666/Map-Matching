Adversarial read-only review. Find what is wrong, assume author overconfident. Report only concrete contract-breaking issues or 'No substantive issue.' Do NOT edit, run tools, launch ROS/NDT, search web, or access credentials. Assess only the supplied artifact and contract. No scientific accuracy or unique-cause claim is being made.

CONTRACT:
# P30-R1 scan-local IMU Deskew feedback isolation

START_SHA: 9ed197900419660845e41eebce603606852ae126.
One research branch and the existing P10 workspace. Protected robot/paper
workspaces and P28/P29 archives/cache are not modified.

Sole physical intervention: `deskew/experimental_scan_local_imu_enable=true`,
default false. Existing relative geometry `(T_ref*T_il)^-1*(T_point*T_il)`
does not change. Freeze map/bag/features/calibration/initialization and all
NDT/DCReg/Multi-NDT/MSCKF/noise/OOSM/velocity_blend parameters from P29 B.
NDT executable remains exactly P29; rebuild only the EKF containing Deskew.
Same compiler flags as P29; standalone isolated devel directory.

New motion path: copy one causal EKF snapshot at/before scan_start; advance
its p,v,R,ba,bg to start with raw head-sample IMU, then integrate once through
actual IMU boundaries until scan_end. Retain P28 effective causal ZOH model:
right rotation exponential, end-rotation acceleration, frozen map gravity
and biases, without its unused legacy gravity-alignment heuristic. No tail
sample values after point time are used, including the final scan fragment.
Point queries use immutable continuous nodes, not subsequent EKF pose samples.
This isolates intra-scan corrected-node resets, NOT prior visual effects on
the legal anchor. It does not guarantee equal A/B Source.

Directed tests: stationary, constant velocity/angular rate, combined motion
vs fine-step reference, common-left SE(3) including gravity/velocity transform,
future-IMU and later-EKF poisoning, query order, partial causal anchor,
out-of-order/duplicate rejection. Existing Deskew contract test verifies the
untouched disabled path. No additional default-off ROS replay is budgeted;
default-off binary-level trajectory parity is NOT claimed.

One 21-second C prefix on MSCKF ON; capture the same bounded eight P29 requests
(only frames7/193 lie within the prefix). No GT is opened. Require expected
P29 prefix sensor timeline, all due fixed-lag transactions, real nonzero MSCKF
updates, finite states/roots, no future IMU, no new >2m adjacent-state jumps.
Compare ordered raw/deskew points, actual captured Source, Initial/Raw/Used,
and same-current-history legacy/local relative motion diagnostics. Do not pair
preprocessed Source point indexes when voxel membership/order changes.

Only if short mathematical/runtime gate passes and motion differences are
explained: one full40s C capture, no A/B reruns. Require396 NDT/396 due NDT
OOSM/8004 high-rate, finite outputs and actual visual updates. Any timing,
nonfinite, unexplained new severe jump or runtime fault stops further replay.
No noise/threshold/strategy search or repair experiments after full C.

Freeze all complete outputs and Sources BEFORE GT. Formal evaluation reuses
Control V2 fixed0–3s full SE(3), same interpolation/IMU origin, windows3–35,
26–35 and34–35s. Compare P29 B vs C, A only historical reference. Include
approximately1s RPE, actual intervals and adjacent-state jumps separately.
No refitting/selecting samples using GT. Correctness/feedback isolation and
accuracy gains are distinct. Point/relative-motion differences alone do not
prove real motion accuracy or a unique root cause.

Record Deskew/NDT/MSCKF mean/P95/max, per-process CPU/RSS, extra diagnostic
cost and build/input hashes. Offline feature extraction cost uses measured P28
provenance, not a free online-camera claim. No 10Hz/100MiB claim.

ARTIFACT (working diff):
diff --git a/src/dog_prior_map_localization/include/dog_prior_map_localization/core/imu_deskew.hpp b/src/dog_prior_map_localization/include/dog_prior_map_localization/core/imu_deskew.hpp
index 61c10093a..ec2cf1643 100644
--- a/src/dog_prior_map_localization/include/dog_prior_map_localization/core/imu_deskew.hpp
+++ b/src/dog_prior_map_localization/include/dog_prior_map_localization/core/imu_deskew.hpp
@@ -3,6 +3,7 @@
 #include <cstddef>
 #include <limits>
 #include <string>
+#include <vector>
 
 #include <Eigen/Dense>
 #include <Eigen/Geometry>
@@ -41,6 +42,33 @@ struct ImuDeskewCoverage
   std::size_t samples_in_scan = 0;
 };
 
+// One immutable scan-start anchor followed by raw, causal head-sample IMU
+// intervals. No later EKF pose is used as a trajectory node. Point queries
+// never mutate this trajectory, so their result is independent of point order.
+struct ScanLocalImuTrajectory
+{
+  std::vector<ImuDeskewPose> nodes;
+  Eigen::Vector3d gravity_world = Eigen::Vector3d::Zero();
+  ImuKinematicsConfig config;
+  double scan_start = 0.0;
+  double scan_end = 0.0;
+  double anchor_state_stamp = 0.0;
+  double max_imu_source_stamp = 0.0;
+  double max_gap_sec = 0.0;
+};
+
+bool buildScanLocalImuTrajectory(const StateHistory &history,
+                                const std::deque<ImuSample> &imu_samples,
+                                double scan_start, double scan_end,
+                                double max_gap_sec,
+                                const Eigen::Vector3d &gravity_world,
+                                const ImuKinematicsConfig &config,
+                                ScanLocalImuTrajectory &trajectory,
+                                std::string &reason);
+
+bool scanLocalImuPoseAt(const ScanLocalImuTrajectory &trajectory, double stamp,
+                       ImuDeskewPose &pose, std::string &reason);
+
 // Checks that propagated EKF states bracket the full scan interval without
 // extrapolation and that every state interval used is below max_gap_sec.
 bool inspectImuDeskewCoverage(const StateHistory &history,
diff --git a/src/dog_prior_map_localization/include/dog_prior_map_localization/dog_prior_map_ekf_node.hpp b/src/dog_prior_map_localization/include/dog_prior_map_localization/dog_prior_map_ekf_node.hpp
index 20b2a0f88..66bd6ee5b 100644
--- a/src/dog_prior_map_localization/include/dog_prior_map_localization/dog_prior_map_ekf_node.hpp
+++ b/src/dog_prior_map_localization/include/dog_prior_map_localization/dog_prior_map_ekf_node.hpp
@@ -300,6 +300,9 @@ private:
   std::string deskew_state_frame_ = "lidar";
   std::string deskew_csv_path_;
   bool imu_deskew_enable_ = false;
+  bool experimental_scan_local_imu_enable_ = false;
+  std::ofstream scan_local_motion_csv_;
+  std::set<uint64_t> scan_local_diagnostic_stamps_;
   double deskew_max_scan_duration_sec_ = 0.15;
   double deskew_max_imu_gap_sec_ = 0.02;
   double deskew_history_keep_sec_ = 2.0;
diff --git a/src/dog_prior_map_localization/src/core/imu_deskew.cpp b/src/dog_prior_map_localization/src/core/imu_deskew.cpp
index ec90ba348..88d75a97e 100644
--- a/src/dog_prior_map_localization/src/core/imu_deskew.cpp
+++ b/src/dog_prior_map_localization/src/core/imu_deskew.cpp
@@ -191,6 +191,143 @@ bool interpolateImuDeskewPose(const StateHistory &history,
   return true;
 }
 
+bool buildScanLocalImuTrajectory(const StateHistory &history,
+                                const std::deque<ImuSample> &imu_samples,
+                                double scan_start, double scan_end,
+                                double max_gap_sec,
+                                const Eigen::Vector3d &gravity_world,
+                                const ImuKinematicsConfig &config,
+                                ScanLocalImuTrajectory &trajectory,
+                                std::string &reason)
+{
+  trajectory = ScanLocalImuTrajectory{};
+  if (!std::isfinite(scan_start) || !std::isfinite(scan_end) ||
+      scan_end < scan_start || !std::isfinite(max_gap_sec) || max_gap_sec <= 0 ||
+      !gravity_world.allFinite() || !config.use_acc_for_position)
+  {
+    reason = "invalid_scan_local_interval_or_model";
+    return false;
+  }
+  const auto &states = history.samples();
+  auto state = std::upper_bound(states.begin(), states.end(), scan_start,
+      [](double t, const FilterStateSnapshot &s) { return t < s.stamp; });
+  if (state == states.begin()) { reason = "no_causal_scan_start_state"; return false; }
+  --state;
+  if (scan_start - state->stamp > max_gap_sec)
+  { reason = "stale_scan_start_state"; return false; }
+  if (imu_samples.empty() || imu_samples.back().stamp < scan_end)
+  { reason = "waiting_for_raw_imu_end_coverage"; return false; }
+  for (std::size_t i=0; i<imu_samples.size(); ++i)
+    if (!std::isfinite(imu_samples[i].stamp) ||
+        (i && imu_samples[i].stamp <= imu_samples[i-1].stamp))
+    { reason = "nonmonotonic_raw_imu_history"; return false; }
+  auto head = std::upper_bound(imu_samples.begin(), imu_samples.end(), state->stamp,
+      [](double t, const ImuSample &s) { return t < s.stamp; });
+  if (head == imu_samples.begin()) { reason = "no_causal_anchor_imu"; return false; }
+  --head;
+  if (state->stamp - head->stamp > max_gap_sec)
+  { reason = "stale_anchor_imu"; return false; }
+
+  trajectory.scan_start = scan_start;
+  trajectory.scan_end = scan_end;
+  trajectory.anchor_state_stamp = state->stamp;
+  trajectory.gravity_world = gravity_world;
+  trajectory.config = config;
+  // P28's mature camera-time propagator uses causal ZOH, not the legacy
+  // midpoint flag. Retain its kinematics/physical parameters, with the same
+  // head-sample convention and without consuming a future tail measurement.
+  trajectory.config.midpoint_interval_input_enable = false;
+  ImuDeskewPose pose = toPose(*state);
+  if (!pose.p.allFinite() || !pose.v.allFinite() || !pose.R.allFinite() ||
+      !pose.ba.allFinite() || !pose.bg.allFinite() ||
+      (pose.R.transpose()*pose.R-Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
+      pose.R.determinant() <= 0)
+  { reason = "invalid_scan_start_state"; return false; }
+  auto set_input = [&]() {
+    pose.acc_measurement = head->acc;
+    pose.gyro_measurement = head->gyro;
+    pose.interval_acc_input = head->acc;
+    pose.interval_gyro_input = head->gyro;
+    pose.has_interval_input = true;
+    pose.latest_source_stamp = head->stamp;
+    pose.acc_world = pose.R*(head->acc-pose.ba)+gravity_world;
+    pose.gyro_unbiased = head->gyro-pose.bg;
+    return head->acc.allFinite() && head->gyro.allFinite();
+  };
+  auto advance = [&](double target) {
+    const double dt = target - pose.stamp;
+    if (dt < 0 || dt > max_gap_sec || !set_input()) return false;
+    if (dt > 0)
+      propagateImuKinematics(pose.p, pose.v, pose.R, pose.ba, pose.bg,
+          head->acc, head->gyro, dt, gravity_world, trajectory.config,
+          pose.acc_world, pose.gyro_unbiased);
+    pose.stamp = target;
+    trajectory.max_imu_source_stamp = std::max(trajectory.max_imu_source_stamp, head->stamp);
+    return pose.p.allFinite() && pose.v.allFinite() && pose.R.allFinite();
+  };
+  // Advance the one historical anchor to scan_start. Raw IMU timestamps may
+  // fall between an image clone's timestamp and the start; do not skip them.
+  while (pose.stamp < scan_start)
+  {
+    auto next = head + 1;
+    const double target = next != imu_samples.end() ? std::min(scan_start, next->stamp) : scan_start;
+    if (next != imu_samples.end() &&
+        (!(next->stamp > head->stamp) || next->stamp-head->stamp > max_gap_sec))
+    { reason = "anchor_raw_imu_gap"; return false; }
+    if (!(target > pose.stamp) || !advance(target))
+    { reason = "anchor_partial_propagation_failed"; return false; }
+    if (next != imu_samples.end() && target == next->stamp) head = next;
+  }
+  if (!set_input()) { reason = "nonfinite_anchor_imu"; return false; }
+  trajectory.nodes.push_back(pose);
+  while (pose.stamp < scan_end)
+  {
+    auto next = head + 1;
+    if (next == imu_samples.end() || !(next->stamp > head->stamp) ||
+        next->stamp-head->stamp > max_gap_sec)
+    { reason = "scan_local_raw_imu_gap"; return false; }
+    trajectory.max_gap_sec = std::max(trajectory.max_gap_sec, next->stamp-head->stamp);
+    const double target = std::min(scan_end, next->stamp);
+    if (!(target > pose.stamp) || !advance(target))
+    { reason = "scan_local_propagation_failed"; return false; }
+    if (target == next->stamp)
+    {
+      head = next;
+      if (!set_input()) { reason = "nonfinite_scan_imu"; return false; }
+    }
+    trajectory.nodes.push_back(pose);
+  }
+  trajectory.max_imu_source_stamp = std::max(trajectory.max_imu_source_stamp, pose.latest_source_stamp);
+  if (trajectory.max_imu_source_stamp > scan_end)
+  { reason = "post_scan_end_imu_used"; return false; }
+  reason = "scan_local_imu_trajectory";
+  return true;
+}
+
+bool scanLocalImuPoseAt(const ScanLocalImuTrajectory &trajectory, double stamp,
+                       ImuDeskewPose &pose, std::string &reason)
+{
+  if (!std::isfinite(stamp) || trajectory.nodes.empty() ||
+      stamp < trajectory.scan_start || stamp > trajectory.scan_end)
+  { reason = "scan_local_query_outside_interval"; return false; }
+  const auto &nodes = trajectory.nodes;
+  auto upper = std::upper_bound(nodes.begin(), nodes.end(), stamp,
+      [](double t, const ImuDeskewPose &p) { return t < p.stamp; });
+  if (upper == nodes.begin()) { reason = "no_scan_local_query_anchor"; return false; }
+  pose = *(upper-1);
+  const double dt = stamp - pose.stamp;
+  if (dt > 0)
+    propagateImuKinematics(pose.p, pose.v, pose.R, pose.ba, pose.bg,
+        pose.acc_measurement, pose.gyro_measurement, dt,
+        trajectory.gravity_world, trajectory.config, pose.acc_world, pose.gyro_unbiased);
+  pose.stamp = stamp;
+  if (!pose.p.allFinite() || !pose.v.allFinite() || !pose.R.allFinite() ||
+      pose.latest_source_stamp > stamp)
+  { reason = "invalid_or_future_scan_local_prediction"; return false; }
+  reason = "scan_local_causal_query";
+  return true;
+}
+
 bool transformPointToReferenceLidar(const ImuDeskewPose &point_pose,
                                      const ImuDeskewPose &reference_pose,
                                      const Eigen::Isometry3d &T_imu_lidar,
diff --git a/src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp b/src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp
index cef2ddd23..c3bcaeb22 100644
--- a/src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp
+++ b/src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp
@@ -105,6 +105,9 @@ DogPriorMapEkfNode::DogPriorMapEkfNode() : nh_(), pnh_("~")
     deskew_mode_ = "legacy_prior_ndt_cv";
   }
   imu_deskew_enable_ = deskew_mode_ == "ekf_imu_fastlio";
+  experimental_scan_local_imu_enable_ = getParam<bool>("deskew/experimental_scan_local_imu_enable", false);
+  if (experimental_scan_local_imu_enable_ && !imu_deskew_enable_)
+    throw std::runtime_error("SCAN_LOCAL_REQUIRES_IMU_DESKEW");
   if (imu_deskew_enable_)
   {
     deskew_input_topic_ = getParam<std::string>("deskew/input_topic", "");
@@ -173,6 +176,24 @@ DogPriorMapEkfNode::DogPriorMapEkfNode() : nh_(), pnh_("~")
     imu_kinematics_config_.gravity_correction_max_angle = gravity_correction_max_angle_;
     imu_kinematics_config_.gravity_correction_acc_tolerance = gravity_correction_acc_tolerance_;
     imu_kinematics_config_.gravity_correction_gyro_max = gravity_correction_gyro_max_;
+    if (experimental_scan_local_imu_enable_)
+    {
+      // This experiment follows the already-frozen P28 causal propagation.
+      // Do not silently apply it to an incompatible legacy interval policy.
+      if (!getParam<bool>("mature_visual/time_adapter_enable", false))
+        throw std::runtime_error("SCAN_LOCAL_REQUIRES_FROZEN_CAUSAL_IMU_MODEL");
+      const auto path = getParam<std::string>("output/experimental_scan_local_motion_csv", "");
+      if (!path.empty())
+      {
+        if (std::ifstream(path).good()) throw std::runtime_error("SCAN_LOCAL_DIAGNOSTIC_PATH_EXISTS");
+        scan_local_motion_csv_.open(path);
+        if (!scan_local_motion_csv_) throw std::runtime_error("SCAN_LOCAL_DIAGNOSTIC_OPEN_FAILED");
+        scan_local_motion_csv_ << "scan_ns,scan_index,scan_start,scan_end,anchor_state_stamp,node_stamp,imu_source_stamp,local_px,local_py,local_pz,local_vx,local_vy,local_vz,local_rx,local_ry,local_rz,history_relative_delta_m,history_relative_delta_deg,history_valid,state_revision,max_imu_source_stamp,history_boundary_residual_m,history_boundary_residual_deg,local_boundary_residual_m,local_boundary_residual_deg\n";
+      }
+      for (const auto &request : getParam<std::vector<std::string>>(
+          "output/experimental_ndt_source_capture_requests", std::vector<std::string>()))
+        scan_local_diagnostic_stamps_.insert(std::stoull(request.substr(request.find(':')+1)));
+    }
   }
   if (midpoint_interval_input_enable_ && !imu_deskew_enable_)
   {
diff --git a/src/dog_prior_map_localization/src/imu_deskew_runtime.cpp b/src/dog_prior_map_localization/src/imu_deskew_runtime.cpp
index d6409a885..8f19b5004 100644
--- a/src/dog_prior_map_localization/src/imu_deskew_runtime.cpp
+++ b/src/dog_prior_map_localization/src/imu_deskew_runtime.cpp
@@ -234,10 +234,27 @@ void DogPriorMapEkfNode::processReadyImuDeskewCloudsLocked()
 
     const double reference_stamp = deskew_reference_time_ == "end" ?
         pending.scan_end : pending.scan_start;
+    ScanLocalImuTrajectory scan_local;
+    ImuKinematicsConfig scan_local_config = imu_kinematics_config_;
+    // Match propagateMsckfImu(): its causal nominal model does not apply the
+    // legacy gravity-alignment heuristic. The configured gravity and biases
+    // themselves are unchanged.
+    if (experimental_scan_local_imu_enable_)
+      scan_local_config.continuous_gravity_correction_enable = false;
+    if (experimental_scan_local_imu_enable_ &&
+        !buildScanLocalImuTrajectory(state_history_, imu_history_, pending.scan_start,
+            pending.scan_end, deskew_max_imu_gap_sec_, g_, scan_local_config,
+            scan_local, reason))
+      throw std::runtime_error("SCAN_LOCAL_TRAJECTORY_FAILED:" + reason);
+    const auto pose_at = [&](double stamp, ImuDeskewPose &pose, std::string &why) {
+      if (experimental_scan_local_imu_enable_)
+        return scanLocalImuPoseAt(scan_local, stamp, pose, why);
+      return interpolateImuDeskewPose(state_history_, stamp,
+          deskew_max_imu_gap_sec_, g_, imu_kinematics_config_, pose, why,
+          pending.scan_end);
+    };
     ImuDeskewPose reference_pose;
-    if (!interpolateImuDeskewPose(state_history_, reference_stamp,
-            deskew_max_imu_gap_sec_, g_, imu_kinematics_config_, reference_pose, reason,
-            pending.scan_end))
+    if (!pose_at(reference_stamp, reference_pose, reason))
     {
       writeImuDeskewDiagnostic(pending.scan_index, "REJECTED", "reference_" + reason,
           pending.scan_start, pending.scan_end, reference_stamp,
@@ -289,9 +306,7 @@ void DogPriorMapEkfNode::processReadyImuDeskewCloudsLocked()
         break;
       }
       ImuDeskewPose point_pose;
-      if (!interpolateImuDeskewPose(state_history_, point_stamp,
-              deskew_max_imu_gap_sec_, g_, imu_kinematics_config_, point_pose, reason,
-              pending.scan_end))
+      if (!pose_at(point_stamp, point_pose, reason))
       {
         valid = false;
         failure_reason = "point_" + reason;
@@ -359,6 +374,84 @@ void DogPriorMapEkfNode::processReadyImuDeskewCloudsLocked()
     const double displacement_mean = displacement_sum / static_cast<double>(displacements.size());
     const double displacement_median = percentile(displacements, 0.5);
     const double displacement_p95 = percentile(displacements, 0.95);
+    if (experimental_scan_local_imu_enable_ && scan_local_motion_csv_.is_open() &&
+        scan_local_diagnostic_stamps_.count(pending.msg->header.stamp.toNSec()))
+    {
+      // Read-only diagnostic: same anchor/state history and no extra Deskew
+      // publication, filter update or NDT optimization. Finite selected scans.
+      ImuDeskewPose old_reference;
+      std::string diagnostic_reason;
+      const bool old_reference_valid = interpolateImuDeskewPose(state_history_,
+          reference_stamp, deskew_max_imu_gap_sec_, g_, imu_kinematics_config_,
+          old_reference, diagnostic_reason, pending.scan_end);
+      std::set<double> diagnostic_times;
+      for (const auto &node : scan_local.nodes) diagnostic_times.insert(node.stamp);
+      for (const auto &state : state_history_.samples())
+        if (state.stamp >= pending.scan_start && state.stamp <= pending.scan_end)
+          diagnostic_times.insert(state.stamp);
+      for (double diagnostic_stamp : diagnostic_times)
+      {
+        ImuDeskewPose node;
+        if (!scanLocalImuPoseAt(scan_local, diagnostic_stamp, node, diagnostic_reason))
+          throw std::runtime_error("SCAN_LOCAL_DIAGNOSTIC_QUERY_FAILED");
+        Eigen::Isometry3d T_ref = Eigen::Isometry3d::Identity(), T_node = T_ref;
+        T_ref.linear() = reference_pose.R; T_ref.translation() = reference_pose.p;
+        T_node.linear() = node.R; T_node.translation() = node.p;
+        const Eigen::Isometry3d relative = (T_ref*T_imu_lidar_).inverse()*(T_node*T_imu_lidar_);
+        const Eigen::AngleAxisd aa(relative.linear());
+        const Eigen::Vector3d rv = aa.axis()*aa.angle();
+        ImuDeskewPose old_node;
+        const bool old_valid = old_reference_valid && interpolateImuDeskewPose(state_history_,
+            node.stamp, deskew_max_imu_gap_sec_, g_, imu_kinematics_config_,
+            old_node, diagnostic_reason, pending.scan_end);
+        double dp = std::numeric_limits<double>::quiet_NaN(), dr = dp;
+        if (old_valid)
+        {
+          Eigen::Isometry3d T_old_ref = Eigen::Isometry3d::Identity(), T_old_node = T_old_ref;
+          T_old_ref.linear() = old_reference.R; T_old_ref.translation() = old_reference.p;
+          T_old_node.linear() = old_node.R; T_old_node.translation() = old_node.p;
+          const Eigen::Isometry3d old_relative = (T_old_ref*T_imu_lidar_).inverse()*(T_old_node*T_imu_lidar_);
+          dp = (relative.translation()-old_relative.translation()).norm();
+          dr = Eigen::AngleAxisd(old_relative.linear().transpose()*relative.linear()).angle()*180.0/M_PI;
+        }
+        double old_boundary_m = std::numeric_limits<double>::quiet_NaN(), old_boundary_deg = old_boundary_m;
+        double local_boundary_m = old_boundary_m, local_boundary_deg = old_boundary_m;
+        // Residual across a 1us left limit, with physical propagation removed.
+        // Same-current-history counterfactual only, NOT reconstructed P29.
+        if (node.stamp > pending.scan_start+1e-6)
+        {
+          ImuDeskewPose left;
+          if (old_valid && interpolateImuDeskewPose(state_history_,node.stamp-1e-6,
+              deskew_max_imu_gap_sec_,g_,imu_kinematics_config_,left,diagnostic_reason,pending.scan_end))
+          {
+            propagateImuKinematics(left.p,left.v,left.R,left.ba,left.bg,
+                left.interval_acc_input,left.interval_gyro_input,node.stamp-left.stamp,
+                g_,imu_kinematics_config_,left.acc_world,left.gyro_unbiased);
+            old_boundary_m=(left.p-old_node.p).norm();
+            old_boundary_deg=Eigen::AngleAxisd(left.R.transpose()*old_node.R).angle()*180.0/M_PI;
+          }
+          if (scanLocalImuPoseAt(scan_local,node.stamp-1e-6,left,diagnostic_reason))
+          {
+            propagateImuKinematics(left.p,left.v,left.R,left.ba,left.bg,
+                left.acc_measurement,left.gyro_measurement,node.stamp-left.stamp,
+                g_,scan_local.config,left.acc_world,left.gyro_unbiased);
+            local_boundary_m=(left.p-node.p).norm();
+            local_boundary_deg=Eigen::AngleAxisd(left.R.transpose()*node.R).angle()*180.0/M_PI;
+          }
+        }
+        scan_local_motion_csv_ << std::setprecision(17) << pending.msg->header.stamp.toNSec()
+          << ',' << pending.scan_index << ',' << pending.scan_start << ',' << pending.scan_end
+          << ',' << scan_local.anchor_state_stamp << ',' << node.stamp << ',' << node.latest_source_stamp
+          << ',' << relative.translation().x() << ',' << relative.translation().y() << ',' << relative.translation().z()
+          << ',' << node.v.x() << ',' << node.v.y() << ',' << node.v.z()
+          << ',' << rv.x() << ',' << rv.y() << ',' << rv.z()
+          << ',' << dp << ',' << dr << ',' << old_valid << ',' << ekf_state_revision_
+          << ',' << scan_local.max_imu_source_stamp
+          << ',' << old_boundary_m << ',' << old_boundary_deg
+          << ',' << local_boundary_m << ',' << local_boundary_deg << '\n';
+      }
+      scan_local_motion_csv_.flush();
+    }
     pub_imu_deskew_cloud_.publish(output);
     writeImuDeskewDiagnostic(pending.scan_index, "PUBLISHED", "covered_and_deskewed",
         pending.scan_start, pending.scan_end, reference_stamp,

ARTIFACT (targeted tests):
#include "dog_prior_map_localization/core/imu_deskew.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_localization;

namespace {
void require(bool ok, const std::string &why) {
  if (!ok) throw std::runtime_error(why);
}
ScanLocalImuTrajectory build(const FilterStateSnapshot &anchor,
                            const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &gravity, double dt=0.005,
                            bool corrupt_inputs=true) {
  StateHistory history; history.insertMonotonic(anchor);
  // Deliberately corrupt every later EKF pose. Scan-local must ignore them.
  auto corrupt=anchor; corrupt.stamp=0.1; corrupt.p.setConstant(900);
  corrupt.R=Eigen::AngleAxisd(1.5,Eigen::Vector3d::UnitX()).toRotationMatrix();
  if (corrupt_inputs) history.insertMonotonic(corrupt);
  std::deque<ImuSample> imu;
  const int count=static_cast<int>(std::round(0.1/dt));
  for(int i=0;i<=count+1;++i) imu.push_back({i*dt,acc,gyro});
  // Future sample values must never enter scan_end or point-time predictions.
  if (corrupt_inputs) {
    imu.back().acc.setConstant(1e20); imu.back().gyro.setConstant(1e20);
  }
  ImuKinematicsConfig config; config.use_acc_for_position=true;
  config.continuous_gravity_correction_enable=false;
  ScanLocalImuTrajectory t; std::string reason;
  require(buildScanLocalImuTrajectory(history,imu,0,0.1,0.02,gravity,config,t,reason),reason);
  require(t.max_imu_source_stamp<=0.1,"future IMU consumed");
  return t;
}
ImuDeskewPose at(const ScanLocalImuTrajectory &t,double stamp) {
  ImuDeskewPose p; std::string reason;
  require(scanLocalImuPoseAt(t,stamp,p,reason),reason); return p;
}
Eigen::Vector3d deskew(const ScanLocalImuTrajectory &t,double stamp) {
  Eigen::Isometry3d E=Eigen::Isometry3d::Identity(); E.translation()<<.1,-.02,.03;
  Eigen::Vector3d p; std::string reason;
  require(transformPointToReferenceLidar(at(t,stamp),at(t,0),E,{2,1,3},p,reason),reason);
  return p;
}
}

int main() {
  try {
    FilterStateSnapshot s; s.stamp=0; const Eigen::Vector3d g(0,0,-9.80665);
    auto stationary=build(s,-g,Eigen::Vector3d::Zero(),g);
    require((deskew(stationary,.083)-Eigen::Vector3d(2,1,3)).norm()<1e-12,"stationary");
    s.v<<2,-1,.5;
    auto translation=build(s,-g,Eigen::Vector3d::Zero(),g);
    require((at(translation,.083).p-s.v*.083).norm()<1e-12,"constant velocity");
    require((deskew(translation,.083)-Eigen::Vector3d(2,1,3)-s.v*.083).norm()<1e-12,"translation compensation");
    s.v.setZero();
    auto rotation=build(s,-g,{0,0,1.2},g);
    require((at(rotation,.083).R-Eigen::AngleAxisd(1.2*.083,Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm()<1e-12,"constant angular rate");
    s.v<<1,.2,0;
    auto combined=build(s,{.7,-.2,9.80665},{0,0,1.2},g);
    auto precise=build(s,{.7,-.2,9.80665},{0,0,1.2},g,0.0001);
    require((at(combined,.1).p-at(precise,.1).p).norm()<3e-5,"combined p reference");
    require((at(combined,.1).v-at(precise,.1).v).norm()<3e-4,"combined v reference");
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    T.linear()=Eigen::AngleAxisd(.6,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix(); T.translation()<<4,-2,7;
    auto transformed=s; transformed.R=T.linear()*s.R;
    transformed.p=T*s.p; transformed.v=T.linear()*s.v;
    auto common=build(transformed,{.7,-.2,9.80665},{0,0,1.2},T.linear()*g);
    require((deskew(common,.083)-deskew(combined,.083)).norm()<1e-12,"common left SE3 invariance");
    const auto early=at(combined,.083);
    auto later=at(combined,.095); (void)later;
    require((early.p-at(combined,.083).p).norm()==0 &&
            (early.R-at(combined,.083).R).norm()==0,"query order");
    auto unchanged=build(s,{.7,-.2,9.80665},{0,0,1.2},g,.005,false);
    require((deskew(unchanged,.083)-deskew(combined,.083)).norm()==0,
            "future IMU or later EKF state contaminated trajectory");
    ImuDeskewPose p; std::string reason;
    require(!scanLocalImuPoseAt(combined,.100001,p,reason),"post-end extrapolation");
    require(!scanLocalImuPoseAt(combined,-.001,p,reason),"pre-start extrapolation");
    // An image clone inside the first IMU interval: causal partial anchor,
    // never consume the tail to obtain scan_start.
    StateHistory history; s.stamp=.002; history.insertMonotonic(s);
    std::deque<ImuSample> imu{{0,-g,Eigen::Vector3d::Zero()},{.005,-g,Eigen::Vector3d::Zero()},
                              {.01,-g,Eigen::Vector3d::Zero()},{.015,-g,Eigen::Vector3d::Zero()}};
    ImuKinematicsConfig cfg; cfg.use_acc_for_position=true; cfg.continuous_gravity_correction_enable=false;
    ScanLocalImuTrajectory partial;
    require(buildScanLocalImuTrajectory(history,imu,.003,.012,.02,g,cfg,partial,reason),reason);
    require((at(partial,.003).p-s.p-s.v*.001).norm()<1e-12,"partial anchor");
    require(partial.nodes.front().latest_source_stamp==0,"future anchor IMU");
    auto bad=imu; std::swap(bad[1],bad[2]);
    require(!buildScanLocalImuTrajectory(history,bad,.003,.012,.02,g,cfg,partial,reason) &&
            reason=="nonmonotonic_raw_imu_history","out-of-order raw IMU accepted");
    bad=imu; bad[2].stamp=bad[1].stamp;
    require(!buildScanLocalImuTrajectory(history,bad,.003,.012,.02,g,cfg,partial,reason) &&
            reason=="nonmonotonic_raw_imu_history","duplicate raw IMU accepted");
    std::cout<<"SCAN_LOCAL_IMU_MATH_PASS: stationary, velocity, angular rate, combined reference, common-left, causal horizon, immutable anchor/query\n";
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}

ARTIFACT (existing nominal IMU formula and old interpolation, unmodified):
  const double end=state_stamp_+dt;
  const Eigen::Vector3d head_acc=last_acc_measurement_,head_gyro=last_gyro_measurement_;
  if(!visual_joint_.driver_active) {
    const auto L=visual_joint_.factor;
    visual_joint_.factor=Eigen::MatrixXd::Zero(L.rows()+6,L.cols()+6);
    visual_joint_.factor.topLeftCorner(L.rows(),L.cols())=L;
    visual_joint_.factor.bottomRightCorner(6,6).diagonal()<<acc_noise_,acc_noise_,acc_noise_,gyro_noise_,gyro_noise_,gyro_noise_;
    visual_joint_.driver_active=true;
    visual_joint_.driver_head_stamp=state_stamp_;
  }
  auto step=[&](double target) {
    const double d=target-state_stamp_; if(d<=0) return;
    const Eigen::Vector3d a=head_acc-ba_,w=head_gyro-bg_,phi=w*d;
    const Eigen::Matrix3d Rend=R_*expRotation(phi);
    Matrix15d F; Eigen::Matrix<double,15,6> G;
    linearizeCausalImu(Rend,a,w,d,&F,&G);
    const auto L=visual_joint_.factor;
    Eigen::MatrixXd columns=Eigen::MatrixXd::Zero(L.rows(),L.cols()+6);
    columns.leftCols(L.cols())=L;
    columns.topLeftCorner(15,L.cols())=F*L.topRows(15)+G*L.bottomRows(6);
    columns.block(9,L.cols(),3,3)=acc_bias_noise_*std::sqrt(d)*Eigen::Matrix3d::Identity();
    columns.block(12,L.cols()+3,3,3)=gyro_bias_noise_*std::sqrt(d)*Eigen::Matrix3d::Identity();
    visual_joint_.factor=compressed(columns);
    R_=Rend; last_acc_world_=R_*a+g_; last_unbiased_gyro_=w;
    p_+=v_*d+0.5*last_acc_world_*d*d; v_+=last_acc_world_*d;
    state_stamp_=target; last_imu_time_=target;
    last_interval_acc_input_=head_acc; last_interval_gyro_input_=head_gyro; has_last_interval_input_=true;
    synchronizeMsckfCoreRoot();
  };
  // Times are only used as an event schedule. Pixel values are not consumed
  // until their own fixed-lag transaction is released.
  for(auto& item:visual_frames_) {
    const double t=ros::Time().fromNSec(item.first).toSec();
    if(t<=state_stamp_+1e-9 || t>end+1e-9) continue;
    step(t);
    VisualPoseClone clone; clone.image_ns=item.first; clone.p=clone.p_fej=p_; clone.R=clone.R_fej=R_;
    if(!item.second.first_estimate_set) {
      item.second.first_estimate=clone; item.second.first_estimate_set=true;
    } else {
      // OOSM rewrites nominal poses, not the immutable first-estimate chart.
  }
  const auto &samples = history.samples();
  if (samples.empty())
  {
    reason = "no_state_history";
    return false;
  }
  auto upper = std::lower_bound(samples.begin(), samples.end(), stamp,
      [](const FilterStateSnapshot &sample, double target) { return sample.stamp < target; });
  if (upper != samples.end() && std::abs(upper->stamp - stamp) <= kStampEpsilon)
  {
    pose = toPose(*upper);
    reason = "exact_state_sample";
    return pose.p.allFinite() && pose.v.allFinite() && pose.R.allFinite();
  }
  if (upper == samples.begin() || upper == samples.end())
  {
    reason = "interpolation_would_extrapolate";
    return false;
  }
  const auto lower = upper - 1;
  const double gap = upper->stamp - lower->stamp;
  if (!std::isfinite(gap) || gap <= 0.0 || gap > max_gap_sec + kStampEpsilon)
  {
    reason = "interpolation_gap_exceeds_limit";
    return false;
  }
  const double dt = stamp - lower->stamp;
  if (!std::isfinite(dt) || dt < 0.0 || dt > gap + kStampEpsilon)
  {
    reason = "invalid_partial_propagation_time";
    return false;
  }

  // Snapshot measurements are raw samples at their own timestamps. Use the
  // mature head/tail mean only when its tail is within the caller's admitted
  // data horizon; the final scan-boundary fragment must not consume an IMU
  // sample after scan_end.
  pose = toPose(*lower);
  const bool tail_is_within_horizon = upper->stamp <= max_source_stamp + kStampEpsilon;
  const bool midpoint = propagation_config.midpoint_interval_input_enable &&
                        tail_is_within_horizon;
  const ImuIntervalInput interval = makeImuIntervalInput(
      lower->acc_measurement, lower->gyro_measurement,
      upper->acc_measurement, upper->gyro_measurement,
      midpoint ? ImuIntervalInputPolicy::kMidpointAverage :
                 ImuIntervalInputPolicy::kHeadSample);
  pose.interval_acc_input = interval.acc;
  pose.interval_gyro_input = interval.gyro;
  pose.has_interval_input = true;
  propagateImuKinematics(pose.p, pose.v, pose.R, pose.ba, pose.bg,
                         interval.acc, interval.gyro, dt,
                         gravity_world, propagation_config,
                         pose.acc_world, pose.gyro_unbiased);
  pose.stamp = stamp;
  pose.latest_source_stamp = midpoint ? upper->stamp : lower->stamp;
  if (!pose.p.allFinite() || !pose.v.allFinite() || !pose.R.allFinite() ||
      !pose.acc_world.allFinite() || !pose.gyro_unbiased.allFinite())
  {
    reason = "nonfinite_interpolated_state";
    return false;
  }
  reason = "causal_partial_propagation";
  return true;
}

bool buildScanLocalImuTrajectory(const StateHistory &history,
                                const std::deque<ImuSample> &imu_samples,
