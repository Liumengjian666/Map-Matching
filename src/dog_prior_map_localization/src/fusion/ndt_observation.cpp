#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

#include <utility>

#include "dog_prior_map_localization/core/oosm_replay_planner.hpp"

namespace dog_prior_map_localization
{

// 接收 NDT 观测并负责锁内调度；实际校正逻辑位于
// processNdtObservationLocked()，以便 deferred measurement 复用同一条路径。
void DogPriorMapEkfNode::ndtObservationCallback(const nav_msgs::OdometryConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ndt_observation_enable_ || !msg) return;

  const double t_ndt = msg->header.stamp.toSec();
  const double t_now = has_state_stamp_ ? state_stamp_ :
      std::numeric_limits<double>::quiet_NaN();
  writeEkfPredictionLineage("NDT_CALLBACK_ENTER", ros::Time::now().toSec(), 0,
                            t_ndt, t_now);

  if (oosm_enable_ && future_deferral_enable_ && has_state_stamp_ &&
      std::isfinite(t_ndt) && std::isfinite(t_now) && t_ndt > t_now + 1e-9)
  {
    const double future_lead_sec = t_ndt - t_now;
    if (future_lead_sec <= future_deferral_max_sec_ + 1e-9)
    {
      if (deferred_ndt_observations_.size() >= future_deferral_max_queue_)
      {
        ++deferred_queue_full_count_;
        writeOosmDiagnostic(t_ndt, t_now,
                            std::numeric_limits<double>::quiet_NaN(),
                            t_now - t_ndt,
                            std::numeric_limits<double>::quiet_NaN(), 0,
                            state_history_.size(), imu_history_.size(),
                            "FUTURE_QUEUE_FULL");
        writeDeferredDiagnostic("NDT_DEFERRED_FUTURE", t_ndt, t_now,
                                future_lead_sec, deferred_ndt_observations_.size(),
                                0.0, "FUTURE_QUEUE_FULL");
        return;
      }

      DeferredNdtObservation deferred;
      deferred.msg = msg;
      deferred.stamp = t_ndt;
      deferred.received_state_stamp = t_now;
      deferred.future_lead_sec = future_lead_sec;
      deferred.received_wall_sec = ros::WallTime::now().toSec();
      auto insert_at = deferred_ndt_observations_.begin();
      while (insert_at != deferred_ndt_observations_.end() &&
             insert_at->stamp <= deferred.stamp)
      {
        ++insert_at;
      }
      deferred_ndt_observations_.insert(insert_at, deferred);
      ++deferred_received_count_;
      max_deferred_queue_size_ = std::max(max_deferred_queue_size_,
                                          deferred_ndt_observations_.size());
      writeDeferredDiagnostic("NDT_DEFERRED_FUTURE", t_ndt, t_now,
                              future_lead_sec, deferred_ndt_observations_.size(),
                              0.0, "DEFERRED");
      return;
    }

    ++deferred_over_limit_count_;
    writeDeferredDiagnostic("NDT_DEFERRED_FUTURE", t_ndt, t_now,
                            future_lead_sec, deferred_ndt_observations_.size(),
                            0.0, "FUTURE_OVER_LIMIT");
  }

  processNdtObservationLocked(msg);
}

// 融合独立 NDT 节点的绝对位姿观测，同时用相邻观测估计并平滑速度。
// 调用方必须已经持有 mutex_；该函数只保留一份原有校正、回滚和 replay 逻辑。
void DogPriorMapEkfNode::processNdtObservationLocked(const nav_msgs::OdometryConstPtr &msg)
{
  if (!ndt_observation_enable_) return;
  if (!msg) return;

  const double t_ndt = msg->header.stamp.toSec();
  const double t_now = has_state_stamp_ ? state_stamp_ :
      std::numeric_limits<double>::quiet_NaN();
  const size_t state_history_count = state_history_.size();
  const size_t imu_history_count = imu_history_.size();
  double rollback_stamp = std::numeric_limits<double>::quiet_NaN();
  double alignment_error = std::numeric_limits<double>::quiet_NaN();
  size_t replay_imu_count = 0;
  std::string oosm_result = oosm_enable_ ? "NOT_DELAYED" : "DISABLED";
  bool oosm_active = false;
  FilterStateSnapshot state_before_oosm;
  StateHistory history_before_oosm;
  std::vector<ImuSample> replay_samples;
  const double last_ndt_time_before = last_ndt_observation_time_;
  const Eigen::Vector3d last_ndt_p_before = last_ndt_observation_p_map_;
  const uint64_t lidar_update_ok_before = lidar_update_ok_count_;
  const uint64_t icp_update_ok_before = icp_update_ok_count_;
  const uint64_t state_revision_before_oosm = ekf_state_revision_;
  const int last_used_points_before = last_used_points_;
  const double last_mean_residual_before = last_mean_residual_;

  const auto restoreOosmAttempt = [&]() {
    restoreStateSnapshot(state_before_oosm);
    state_stamp_ = t_now;
    state_history_ = history_before_oosm;
    last_ndt_observation_time_ = last_ndt_time_before;
    last_ndt_observation_p_map_ = last_ndt_p_before;
    lidar_update_ok_count_ = lidar_update_ok_before;
    icp_update_ok_count_ = icp_update_ok_before;
    ekf_state_revision_ = state_revision_before_oosm;
    last_used_points_ = last_used_points_before;
    last_mean_residual_ = last_mean_residual_before;
  };

  const auto writeOosmResult = [&](const std::string &result) {
    writeOosmDiagnostic(t_ndt, t_now, rollback_stamp,
                        std::isfinite(t_now) && std::isfinite(t_ndt) ? t_now - t_ndt :
                            std::numeric_limits<double>::quiet_NaN(),
                        alignment_error, replay_imu_count, state_history_count,
                        imu_history_count, result);
  };

  Eigen::Vector3d p_target(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  Eigen::Quaterniond q_target(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);
  if (!std::isfinite(t_ndt) || !p_target.allFinite() ||
      !q_target.coeffs().allFinite() || q_target.norm() < 1e-9)
  {
    writeOosmResult("INVALID_MEASUREMENT");
    return;
  }
  Eigen::Matrix3d R_target = q_target.normalized().toRotationMatrix();

  if (oosm_enable_)
  {
    if (!has_state_stamp_ || !std::isfinite(t_now))
    {
      writeOosmResult("NO_HISTORY");
      return;
    }
    if (t_ndt > t_now + 1e-9)
    {
      writeOosmResult("FUTURE_MEASUREMENT");
      return;
    }

    const OosmReplayPlanner replay_planner;
    OosmReplayPlan replay_plan = replay_planner.makePlan(
        state_history_, imu_history_, t_ndt, t_now,
        oosm_max_alignment_sec_, max_imu_dt_);
    alignment_error = replay_plan.alignment_error_sec;
    rollback_stamp = replay_plan.rollback_stamp;
    replay_samples = std::move(replay_plan.replay_samples);
    replay_imu_count = replay_samples.size();
    if (replay_plan.status != OosmPlanStatus::kReady)
    {
      const std::string result = replay_plan.status == OosmPlanStatus::kInvalidTimestamp ?
          "INVALID_MEASUREMENT" : oosmPlanStatusName(replay_plan.status);
      writeOosmResult(result);
      return;
    }
    const size_t rollback_index = replay_plan.rollback_index;

    state_before_oosm.stamp = t_now;
    state_before_oosm.p = p_;
    state_before_oosm.v = v_;
    state_before_oosm.R = R_;
    state_before_oosm.ba = ba_;
    state_before_oosm.bg = bg_;
    state_before_oosm.P = P_;
    history_before_oosm = state_history_;
    eraseStateHistoryAfter(rollback_stamp);
    restoreStateSnapshot(state_history_.at(rollback_index));
    state_stamp_ = rollback_stamp;
    oosm_active = true;
    oosm_result = "APPLIED";
    writeEkfPredictionLineage("OOSM_ROLLBACK", ros::Time::now().toSec(), 0,
                              t_ndt, t_now, std::numeric_limits<double>::quiet_NaN(),
                              rollback_stamp, replay_imu_count,
                              (t_now - t_ndt) * 1000.0);
  }

  Eigen::Vector3d dp = p_target - p_;
  Eigen::AngleAxisd aa(R_target * R_.transpose());
  Eigen::Vector3d dtheta = aa.axis() * aa.angle();
  if (!dp.allFinite() || !dtheta.allFinite())
  {
    if (oosm_active)
    {
      restoreOosmAttempt();
      writeOosmResult("REPLAY_INCOMPLETE");
    }
    else
    {
      writeOosmResult("INVALID_MEASUREMENT");
    }
    return;
  }

  const double ratio = std::max(0.0, std::min(1.0, ndt_observation_apply_ratio_));
  const double z_ratio = std::max(0.0, std::min(1.0, ndt_observation_z_apply_ratio_));
  const double roll_pitch_ratio = std::max(0.0, std::min(1.0, ndt_observation_roll_pitch_apply_ratio_));
  dp.x() *= ratio;
  dp.y() *= ratio;
  dp.z() *= ratio * z_ratio;
  dtheta = limitVector(dtheta * ratio, ndt_observation_max_rotation_correction_);
  dtheta.x() *= roll_pitch_ratio;
  dtheta.y() *= roll_pitch_ratio;
  dp = limitVector(dp, ndt_observation_max_translation_correction_);

  applyPoseCorrection(dp, dtheta);
  writeEkfPredictionLineage("NDT_CORRECTION_APPLIED", ros::Time::now().toSec(), 0,
                            t_ndt, t_now,
                            oosm_active ? std::numeric_limits<double>::quiet_NaN() : state_stamp_,
                            rollback_stamp, replay_imu_count,
                            std::isfinite(t_now) && std::isfinite(t_ndt) ?
                                (t_now - t_ndt) * 1000.0 :
                                std::numeric_limits<double>::quiet_NaN());
  ++lidar_update_ok_count_;
  ++icp_update_ok_count_;
  last_used_points_ = 0;
  last_mean_residual_ = dp.norm();
  if (!oosm_active) publishState(msg->header.stamp, true);

  const double dt = t_ndt - last_ndt_observation_time_;
  if (dt > 1e-3 && dt < 1.0)
  {
    Eigen::Vector3d odom_velocity = (p_target - last_ndt_observation_p_map_) / dt;
    odom_velocity.z() *= z_ratio;
    const double blend = std::max(0.0, std::min(1.0, ndt_observation_velocity_blend_));
    v_ = (1.0 - blend) * v_ + blend * odom_velocity;
  }
  last_ndt_observation_p_map_ = p_target;
  last_ndt_observation_time_ = t_ndt;

  for (int i = 0; i < 3; ++i) P_(i, i) = std::max(P_(i, i) * 0.85, 1e-4);
  for (int i = 6; i < 9; ++i) P_(i, i) = std::max(P_(i, i) * 0.85, 1e-5);

  if (oosm_active)
  {
    saveStateSnapshot(rollback_stamp);
    for (const auto &sample : replay_samples)
    {
      if (sample.stamp <= state_stamp_ + 1e-9) continue;
      const double replay_dt = sample.stamp - state_stamp_;
      if (replay_dt <= 0.0 || replay_dt > max_imu_dt_ + 1e-9)
      {
        restoreOosmAttempt();
        writeOosmResult("REPLAY_INCOMPLETE");
        return;
      }
      propagateImu(sample.acc, sample.gyro, replay_dt);
      state_stamp_ = sample.stamp;
      saveStateSnapshot(state_stamp_);
    }

    if (!has_state_stamp_ || std::abs(state_stamp_ - t_now) > 1e-4 ||
        !p_.allFinite() || !v_.allFinite() || !R_.allFinite() ||
        !ba_.allFinite() || !bg_.allFinite() || !P_.allFinite())
    {
      restoreOosmAttempt();
      writeOosmResult("REPLAY_INCOMPLETE");
      return;
    }
    const double rewrite_translation_m = (p_ - state_before_oosm.p).norm();
    const double rewrite_rotation_deg =
        Eigen::AngleAxisd(state_before_oosm.R.transpose() * R_).angle() * 180.0 / M_PI;
    const double rewrite_velocity_mps = (v_ - state_before_oosm.v).norm();
    writeEkfPredictionLineage("OOSM_REPLAY_COMPLETE", ros::Time::now().toSec(), 0,
                              t_ndt, t_now, t_now, rollback_stamp, replay_imu_count,
                              (t_now - t_ndt) * 1000.0,
                              rewrite_translation_m, rewrite_rotation_deg,
                              rewrite_velocity_mps);
    // The replayed state is current-time state, so its publication timestamp
    // must be t_now rather than the older NDT measurement timestamp.
    publishState(ros::Time(t_now), true);
  }

  writeOosmResult(oosm_result);
}

}  // namespace dog_prior_map_localization
