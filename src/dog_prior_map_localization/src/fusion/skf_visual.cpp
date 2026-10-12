#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

#include "dog_prior_map_localization/core/math_utils.hpp"
#include "dog_prior_map_localization/core/oosm_replay_planner.hpp"

namespace dog_prior_map_localization
{
namespace
{
constexpr uint64_t kSkfFixedLagIdMask = uint64_t{1} << 63;

bool allFinite(const FilterStateSnapshot& state)
{
  return state.p.allFinite() && state.v.allFinite() && state.R.allFinite() &&
      state.ba.allFinite() && state.bg.allFinite() && state.P.allFinite();
}

}  // namespace

void DogPriorMapEkfNode::skfVisualMeasurementCallback(
    const ros::MessageEvent<dog_prior_map_interfaces::VisualRelativeMeasurement const>& event)
{
  const auto msg = event.getMessage();
  if (!skf_relative_visual_update_enable_ || !msg) return;
  std::lock_guard<std::mutex> lock(mutex_);
  ++skf_visual_received_count_;

  const uint64_t ref_ns = msg->reference_stamp.toNSec();
  const uint64_t cur_ns = msg->header.stamp.toNSec();
  const uint64_t depth_ns = msg->depth_max_point_stamp.toNSec();
  const uint64_t depth_imu_ns = msg->depth_max_imu_stamp.toNSec();
  const uint64_t scan_ns = msg->scan_ns;
  std::string status = "QUEUED";
  Matrix6d covariance;
  Eigen::Quaterniond q(msg->relative_pose.orientation.w,
                       msg->relative_pose.orientation.x,
                       msg->relative_pose.orientation.y,
                       msg->relative_pose.orientation.z);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      covariance(i, j) = msg->covariance_left_imu_ref[static_cast<std::size_t>(i * 6 + j)];
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> covariance_eigen(covariance, Eigen::EigenvaluesOnly);
  const double qnorm = q.norm();
  if (event.getPublisherName() != "/dog_prior_map_ndt") status = "UNVERIFIED_PUBLISHER";
  else if (msg->header.frame_id != "imu_cur" || msg->reference_frame != "imu_ref" ||
           msg->current_frame != "imu_cur") status = "INVALID_RELATIVE_FRAME";
  else if (ref_ns == 0 || cur_ns <= ref_ns || cur_ns != msg->header.stamp.toNSec() ||
           cur_ns > scan_ns || depth_ns == 0 || depth_ns > ref_ns ||
           depth_imu_ns == 0 || depth_imu_ns > ref_ns)
    status = "NONCAUSAL_OR_INVALID_SENSOR_TIME";
  else if (scan_ns >= kSkfFixedLagIdMask) status = "VISUAL_EVENT_ID_OVERFLOW";
  else if (!std::isfinite(qnorm) || qnorm < 1e-9 || !q.coeffs().allFinite() ||
           !std::isfinite(msg->relative_pose.position.x) ||
           !std::isfinite(msg->relative_pose.position.y) ||
           !std::isfinite(msg->relative_pose.position.z)) status = "NONFINITE_RELATIVE_POSE";
  else if (covariance_eigen.info() != Eigen::Success || !covariance_eigen.eigenvalues().allFinite() ||
           covariance_eigen.eigenvalues().minCoeff() <= 0.0)
    status = "INVALID_RELATIVE_COVARIANCE";
  else if (msg->pnp_inliers < 20 || !std::isfinite(msg->inlier_ratio) ||
           msg->inlier_ratio <= 0.0 || msg->inlier_ratio > 1.0 ||
           !std::isfinite(msg->reprojection_rmse_px) || msg->reprojection_rmse_px < 0.0 ||
           msg->covariance_model != "P4_REPROJECTION_JACOBIAN_RANGE_NOISE_FLOORS_COV4")
    status = "INVALID_VISUAL_QUALITY_OR_COVARIANCE_MODEL";
  if (status != "QUEUED")
  {
    ++skf_visual_rejected_count_;
    SelectiveRelativeVisualUpdateResult empty;
    writeSkfVisualDiagnostic(*msg, status,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), empty);
    return;
  }

  const uint64_t event_id = kSkfFixedLagIdMask | scan_ns;
  if (fixed_lag_visual_messages_.count(event_id) != 0)
  {
    ++skf_visual_rejected_count_;
    status = "DUPLICATE_SKF_VISUAL_EVENT";
    SelectiveRelativeVisualUpdateResult empty;
    writeSkfVisualDiagnostic(*msg, status,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), empty);
    return;
  }
  const double arrival_state_stamp = has_state_stamp_ ? state_stamp_ :
      std::numeric_limits<double>::quiet_NaN();
  const double arrival_wall_sec = ros::WallTime::now().toSec();
  const FixedLagEnqueueStatus enqueue_status = oosm_fixed_lag_scheduler_.enqueue(
      event_id, static_cast<double>(cur_ns) * 1e-9, arrival_state_stamp, arrival_wall_sec);
  if (enqueue_status != FixedLagEnqueueStatus::kQueued)
  {
    ++skf_visual_rejected_count_;
    status = fixedLagEnqueueStatusName(enqueue_status);
    SelectiveRelativeVisualUpdateResult empty;
    writeSkfVisualDiagnostic(*msg, status,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), empty);
    writeFixedLagDiagnostic(event_id, static_cast<double>(cur_ns) * 1e-9,
        static_cast<double>(cur_ns) * 1e-9 + oosm_fixed_lag_sec_, arrival_state_stamp,
        arrival_wall_sec, std::numeric_limits<double>::quiet_NaN(), arrival_wall_sec,
        0, false, status);
    return;
  }
  fixed_lag_visual_messages_.emplace(event_id, msg);
  writeFixedLagDiagnostic(event_id, static_cast<double>(cur_ns) * 1e-9,
      static_cast<double>(cur_ns) * 1e-9 + oosm_fixed_lag_sec_, arrival_state_stamp,
      arrival_wall_sec, std::numeric_limits<double>::quiet_NaN(), arrival_wall_sec,
      0, false, "QUEUED_SKF_VISUAL");
}

bool DogPriorMapEkfNode::stateAtCausalTimeLocked(double target_stamp,
    FilterStateSnapshot& state, double& sample_age_ms, std::string& reason) const
{
  sample_age_ms = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(target_stamp) || !has_state_stamp_ || target_stamp > state_stamp_ + 1e-9)
  {
    reason = "FUTURE_OR_INVALID_STATE_TIME";
    return false;
  }
  std::size_t index = 0;
  double age = 0.0;
  if (!state_history_.findAtOrBefore(target_stamp, index, age))
  {
    reason = "NO_STATE_HISTORY_AT_OR_BEFORE_SENSOR_TIME";
    return false;
  }
  sample_age_ms = age * 1000.0;
  const double alignment_limit = std::min(oosm_max_alignment_sec_, max_imu_dt_);
  if (!std::isfinite(age) || age > alignment_limit + 1e-9)
  {
    reason = "STATE_ALIGNMENT_EXCEEDS_20MS_CONTRACT";
    return false;
  }
  state = state_history_.at(index);
  if (age <= 1e-9)
  {
    reason = "EXACT_SENSOR_TIME_STATE";
    return allFinite(state);
  }
  const double dt = age;
  if (dt <= 0.0 || dt > max_imu_dt_ + 1e-9 ||
      !state.acc_measurement.allFinite() || !state.gyro_measurement.allFinite())
  {
    reason = "INVALID_CAUSAL_PARTIAL_IMU_INTERVAL";
    return false;
  }

  const Eigen::Vector3d acc_unbiased = state.acc_measurement - state.ba;
  ImuKinematicsConfig config;
  config.use_acc_for_position = use_acc_for_position_;
  config.velocity_damping = velocity_damping_;
  config.continuous_gravity_correction_enable = continuous_gravity_correction_enable_;
  config.gravity_correction_expected_acc_norm = gravity_correction_expected_acc_norm_;
  config.gravity_correction_gain = gravity_correction_gain_;
  config.gravity_correction_max_angle = gravity_correction_max_angle_;
  config.gravity_correction_acc_tolerance = gravity_correction_acc_tolerance_;
  config.gravity_correction_gyro_max = gravity_correction_gyro_max_;
  propagateImuKinematics(state.p, state.v, state.R, state.ba, state.bg,
      state.acc_measurement, state.gyro_measurement, dt, g_, config,
      state.acc_world, state.gyro_unbiased);

  Matrix15d F = Matrix15d::Identity();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(3, 6) = -state.R * skew(acc_unbiased) * dt;
  F.block<3, 3>(3, 9) = -state.R * dt;
  F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
  Matrix15d Q = Matrix15d::Zero();
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * acc_noise_ * acc_noise_ * dt * dt;
  Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyro_noise_ * gyro_noise_ * dt * dt;
  Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * acc_bias_noise_ * acc_bias_noise_ * dt;
  Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * gyro_bias_noise_ * gyro_bias_noise_ * dt;
  if (midpoint_interval_input_enable_ && state.has_interval_input)
  {
    // The production filter uses a midpoint only after the tail IMU sample
    // arrives. For a sensor-time query between samples, using that future tail
    // would violate causality. Use the already-arrived head sample above and
    // add a 2-sigma local-variation bound inferred from the last completed
    // midpoint interval. This affects only the cloned endpoint covariance.
    const Eigen::Vector3d delta_acc = state.acc_measurement - state.interval_acc_input;
    const Eigen::Vector3d delta_gyro = state.gyro_measurement - state.interval_gyro_input;
    constexpr double kPartialIntervalVarianceInflation = 4.0;
    if (use_acc_for_position_)
    {
      const Eigen::Matrix3d acceleration_covariance = kPartialIntervalVarianceInflation *
          delta_acc * delta_acc.transpose();
      Q.block<3, 3>(0, 0) += 0.25 * dt * dt * dt * dt * acceleration_covariance;
      Q.block<3, 3>(0, 3) += 0.5 * dt * dt * dt * acceleration_covariance;
      Q.block<3, 3>(3, 0) += 0.5 * dt * dt * dt * acceleration_covariance;
      Q.block<3, 3>(3, 3) += dt * dt * acceleration_covariance;
    }
    Q.block<3, 3>(6, 6) += kPartialIntervalVarianceInflation * dt * dt *
        delta_gyro * delta_gyro.transpose();
  }
  const Matrix15d propagated_covariance = (F * state.P * F.transpose() + Q).eval();
  state.P = 0.5 * (propagated_covariance + propagated_covariance.transpose());
  state.stamp = target_stamp;
  state.interval_acc_input = state.acc_measurement;
  state.interval_gyro_input = state.gyro_measurement;
  state.has_interval_input = true;
  if (!allFinite(state))
  {
    reason = "NONFINITE_CAUSAL_PARTIAL_STATE";
    return false;
  }
  reason = midpoint_interval_input_enable_ ?
      "CAUSAL_HEAD_SAMPLE_PARTIAL_WITH_MIDPOINT_UNCERTAINTY" :
      "CAUSAL_HEAD_SAMPLE_PARTIAL_PROPAGATION";
  return true;
}

void DogPriorMapEkfNode::processSkfVisualLocked(
    const dog_prior_map_interfaces::VisualRelativeMeasurementConstPtr& msg,
    double expected_release_state_stamp,
    SkfVisualReceipt* receipt)
{
  if (receipt) *receipt = SkfVisualReceipt();
  if (!msg || !skf_relative_visual_update_enable_)
  {
    if (receipt) receipt->status = "SKF_DISABLED_OR_NULL";
    return;
  }
  const double t_reference = msg->reference_stamp.toSec();
  const double t_current = msg->header.stamp.toSec();
  const double t_now = has_state_stamp_ ? state_stamp_ :
      std::numeric_limits<double>::quiet_NaN();
  if (receipt && std::abs(t_now - expected_release_state_stamp) > 1e-9)
  {
    receipt->status = "FIXED_LAG_RELEASE_NODE_MISMATCH";
    writeSkfVisualDiagnostic(*msg, receipt->status, NAN, NAN, receipt->update);
    return;
  }
  const OosmReplayPlanner planner;
  const OosmReplayPlan plan = planner.makePlan(state_history_, imu_history_, t_current,
      t_now, oosm_max_alignment_sec_, max_imu_dt_);
  if (plan.status != OosmPlanStatus::kReady)
  {
    if (receipt) receipt->status = std::string("OOSM_") + oosmPlanStatusName(plan.status);
    writeSkfVisualDiagnostic(*msg, receipt ? receipt->status : "OOSM_PLAN_REJECTED",
        std::numeric_limits<double>::quiet_NaN(), plan.alignment_error_sec * 1000.0,
        receipt ? receipt->update : SelectiveRelativeVisualUpdateResult());
    writeOosmDiagnostic(t_current, t_now, plan.rollback_stamp, t_now - t_current,
        plan.alignment_error_sec, 0, state_history_.size(), imu_history_.size(),
        receipt ? receipt->status : "OOSM_PLAN_REJECTED");
    return;
  }

  FilterStateSnapshot reference_state, current_state;
  double reference_alignment_ms = 0.0, current_alignment_ms = 0.0;
  std::string reference_reason, current_reason;
  if (!stateAtCausalTimeLocked(t_reference, reference_state,
          reference_alignment_ms, reference_reason) ||
      !stateAtCausalTimeLocked(t_current, current_state,
          current_alignment_ms, current_reason))
  {
    if (receipt) receipt->status = !reference_reason.empty() && reference_reason.find("FUTURE") != std::string::npos ?
        "FUTURE_EKF_STATE_REJECTED" : "EKF_ENDPOINT_STATE_UNAVAILABLE:" +
        (reference_reason.empty() ? current_reason : reference_reason);
    writeSkfVisualDiagnostic(*msg, receipt ? receipt->status : "EKF_ENDPOINT_STATE_UNAVAILABLE",
        reference_alignment_ms, current_alignment_ms,
        receipt ? receipt->update : SelectiveRelativeVisualUpdateResult());
    writeOosmDiagnostic(t_current, t_now, plan.rollback_stamp, t_now - t_current,
        plan.alignment_error_sec, 0, state_history_.size(), imu_history_.size(),
        receipt ? receipt->status : "EKF_ENDPOINT_STATE_UNAVAILABLE");
    return;
  }

  Eigen::Matrix4d relative = Eigen::Matrix4d::Identity();
  relative.block<3, 1>(0, 3) = Eigen::Vector3d(msg->relative_pose.position.x,
      msg->relative_pose.position.y, msg->relative_pose.position.z);
  Eigen::Quaterniond relative_q(msg->relative_pose.orientation.w,
      msg->relative_pose.orientation.x, msg->relative_pose.orientation.y,
      msg->relative_pose.orientation.z);
  if (!relative_q.coeffs().allFinite() || relative_q.norm() < 1e-9)
  {
    if (receipt) receipt->status = "INVALID_RELATIVE_QUATERNION";
    writeSkfVisualDiagnostic(*msg, receipt ? receipt->status : "INVALID_RELATIVE_QUATERNION",
        reference_alignment_ms, current_alignment_ms,
        receipt ? receipt->update : SelectiveRelativeVisualUpdateResult());
    return;
  }
  relative.block<3, 3>(0, 0) = relative_q.normalized().toRotationMatrix();
  Matrix6d relative_covariance;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      relative_covariance(i, j) = msg->covariance_left_imu_ref[
          static_cast<std::size_t>(i * 6 + j)];
  Eigen::Matrix3d translation_axes, rotation_axes;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      translation_axes(i, j) = msg->translation_axes_map[static_cast<std::size_t>(i * 3 + j)];
      rotation_axes(i, j) = msg->rotation_axes_map[static_cast<std::size_t>(i * 3 + j)];
    }

  const FilterStateSnapshot live_state = state_history_.empty() ? current_state : [&]() {
    FilterStateSnapshot snapshot;
    snapshot.stamp = state_stamp_; snapshot.p = p_; snapshot.v = v_; snapshot.R = R_;
    snapshot.ba = ba_; snapshot.bg = bg_; snapshot.acc_measurement = last_acc_measurement_;
    snapshot.gyro_measurement = last_gyro_measurement_;
    snapshot.interval_acc_input = last_interval_acc_input_;
    snapshot.interval_gyro_input = last_interval_gyro_input_;
    snapshot.has_interval_input = has_last_interval_input_;
    snapshot.acc_world = last_acc_world_; snapshot.gyro_unbiased = last_unbiased_gyro_;
    snapshot.P = P_; return snapshot;
  }();
  FilterStateSnapshot updated_state = current_state;
  const SelectiveRelativeVisualUpdateResult update = applySelectiveRelativeVisualUpdate(
      &updated_state, reference_state, relative, relative_covariance,
      msg->translation_degenerate, translation_axes,
      msg->rotation_degenerate, rotation_axes);
  if (receipt) receipt->update = update;
  if (!update.accepted)
  {
    if (receipt) receipt->status = update.status;
    writeSkfVisualDiagnostic(*msg, update.status, reference_alignment_ms,
        current_alignment_ms, update);
    writeOosmDiagnostic(t_current, t_now, plan.rollback_stamp, t_now - t_current,
        plan.alignment_error_sec, 0, state_history_.size(), imu_history_.size(), update.status);
    return;
  }

  const StateHistory history_before = state_history_;
  const uint64_t revision_before = ekf_state_revision_;
  eraseStateHistoryAfter(t_current);
  restoreStateSnapshot(updated_state);
  saveStateSnapshot(t_current);
  std::size_t replay_count = 0;
  bool replay_ok = true;
  for (const ImuSample& sample : plan.replay_samples)
  {
    if (sample.stamp <= t_current + 1e-9) continue;
    const double dt = sample.stamp - state_stamp_;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_imu_dt_ + 1e-9 ||
        !sample.acc.allFinite() || !sample.gyro.allFinite())
    {
      replay_ok = false;
      break;
    }
    propagateImu(sample.acc, sample.gyro, dt);
    state_stamp_ = sample.stamp;
    last_imu_time_ = sample.stamp;
    saveStateSnapshot(state_stamp_);
    ++replay_count;
  }
  if (!replay_ok || !has_state_stamp_ || std::abs(state_stamp_ - t_now) > 1e-4 ||
      !p_.allFinite() || !v_.allFinite() || !R_.allFinite() || !P_.allFinite())
  {
    restoreStateSnapshot(live_state);
    state_history_ = history_before;
    ekf_state_revision_ = revision_before;
    if (receipt) receipt->status = "REPLAY_INCOMPLETE";
    writeSkfVisualDiagnostic(*msg, "REPLAY_INCOMPLETE", reference_alignment_ms,
        current_alignment_ms, update);
    writeOosmDiagnostic(t_current, t_now, plan.rollback_stamp, t_now - t_current,
        plan.alignment_error_sec, replay_count, state_history_.size(), imu_history_.size(),
        "SKF_REPLAY_INCOMPLETE");
    return;
  }

  ++ekf_state_revision_;
  ++skf_visual_applied_count_;
  if (receipt)
  {
    receipt->status = "APPLIED";
    receipt->replay_imu_count = replay_count;
    receipt->correction_applied = true;
  }
  writeEkfPredictionLineage("SKF_RELATIVE_VISUAL_REPLAY_COMPLETE", ros::Time::now().toSec(),
      0, t_current, t_now, state_stamp_, plan.rollback_stamp, replay_count,
      (t_now - t_current) * 1000.0, update.correction.segment<3>(0).norm(),
      update.correction.segment<3>(6).norm() * 180.0 / M_PI,
      update.correction.segment<3>(3).norm());
  writeSkfVisualDiagnostic(*msg, "APPLIED", reference_alignment_ms,
      current_alignment_ms, update);
  writeOosmDiagnostic(t_current, t_now, plan.rollback_stamp, t_now - t_current,
      plan.alignment_error_sec, replay_count, state_history_.size(), imu_history_.size(),
      "SKF_RELATIVE_VISUAL_APPLIED");
  publishState(ros::Time(t_now), true);
  processReadyImuDeskewCloudsLocked();
}

void DogPriorMapEkfNode::writeSkfVisualDiagnostic(
    const dog_prior_map_interfaces::VisualRelativeMeasurement& msg,
    const std::string& status,
    double reference_alignment_ms,
    double current_alignment_ms,
    const SelectiveRelativeVisualUpdateResult& update)
{
  if (!skf_update_diagnostics_csv_.is_open()) return;
  double innovation0 = std::numeric_limits<double>::quiet_NaN();
  double innovation1 = std::numeric_limits<double>::quiet_NaN();
  double correction_p = std::numeric_limits<double>::quiet_NaN();
  double correction_v = std::numeric_limits<double>::quiet_NaN();
  double correction_theta = std::numeric_limits<double>::quiet_NaN();
  double correction_ba = std::numeric_limits<double>::quiet_NaN();
  double correction_bg = std::numeric_limits<double>::quiet_NaN();
  if (update.innovation.size() > 0) innovation0 = update.innovation(0);
  if (update.innovation.size() > 1) innovation1 = update.innovation(1);
  if (update.correction.size() == 15)
  {
    correction_p = update.correction.segment<3>(0).norm();
    correction_v = update.correction.segment<3>(3).norm();
    correction_theta = update.correction.segment<3>(6).norm();
    correction_ba = update.correction.segment<3>(9).norm();
    correction_bg = update.correction.segment<3>(12).norm();
  }
  skf_update_diagnostics_csv_ << std::setprecision(17) << msg.scan_ns << ','
      << msg.reference_stamp.toNSec() << ',' << msg.header.stamp.toNSec() << ','
      << state_stamp_ << ','
      << reference_alignment_ms << ',' << current_alignment_ms << ','
      << (msg.translation_degenerate ? 1 : 0) << ',' << (msg.rotation_degenerate ? 1 : 0) << ','
      << update.dimension << ',' << innovation0 << ',' << innovation1 << ','
      << update.normalized_innovation_squared << ',' << update.current_covariance_trace << ','
      << update.pseudo_covariance_trace << ','
      << update.minimum_input_state_covariance_eigenvalue << ','
      << update.minimum_conditional_covariance_eigenvalue << ','
      << update.minimum_updated_covariance_eigenvalue << ','
      << update.input_state_covariance_asymmetry << ','
      << correction_p << ',' << correction_v << ','
      << correction_theta << ',' << correction_ba << ',' << correction_bg << ',' << status << '\n';
  skf_update_diagnostics_csv_.flush();
}

}  // namespace dog_prior_map_localization
