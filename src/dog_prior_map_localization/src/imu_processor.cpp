#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"
#include "dog_prior_map_localization/core/math_utils.hpp"

namespace dog_prior_map_localization
{

// IMU 主回调：维护短时历史、完成重力初始化并驱动高频状态传播。
void DogPriorMapEkfNode::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++imu_msg_count_;
  const double t = msg->header.stamp.toSec();
  imu_history_.push_back(ImuSample{t,
                                   Eigen::Vector3d(msg->linear_acceleration.x,
                                                   msg->linear_acceleration.y,
                                                   msg->linear_acceleration.z),
                                   Eigen::Vector3d(msg->angular_velocity.x,
                                                   msg->angular_velocity.y,
                                                   msg->angular_velocity.z)});
  while (!imu_history_.empty() && t - imu_history_.front().stamp > imu_history_keep_sec_)
  {
    imu_history_.pop_front();
  }

  if (initialize_gravity_from_imu_ && !gravity_initialized_)
  {
    // ------------------------- 启动重力初始化 -------------------------
    // MID360的IMU在静止时，加速度计测到的是“向上约9.8m/s^2”的比力。
    // 若直接假设初始姿态为单位阵，而设备有一点俯仰/横滚，积分会把重力当运动加速度，
    // 于是Z轴会快速飘到几十米甚至几万米。这里用前N帧静止IMU平均值对齐roll/pitch。
    imu_acc_sum_ += Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    ++imu_init_count_;
    if (imu_init_count_ < init_imu_samples_)
    {
      last_imu_time_ = t;
      has_last_imu_ = true;
      return;
    }

    Eigen::Vector3d acc_avg = imu_acc_sum_ / static_cast<double>(imu_init_count_);
    if (acc_avg.norm() > 1e-3)
    {
      Eigen::Quaterniond q_align;
      q_align.setFromTwoVectors(acc_avg.normalized(), Eigen::Vector3d::UnitZ());
      R_ = q_align.toRotationMatrix() * R_;
      ROS_INFO("[DogPriorMap C++] IMU gravity init complete: samples=%d acc_avg=(%.3f %.3f %.3f)",
               imu_init_count_, acc_avg.x(), acc_avg.y(), acc_avg.z());
    }
    gravity_initialized_ = true;
    last_acc_measurement_ = Eigen::Vector3d(msg->linear_acceleration.x,
                                             msg->linear_acceleration.y,
                                             msg->linear_acceleration.z);
    last_gyro_measurement_ = Eigen::Vector3d(msg->angular_velocity.x,
                                              msg->angular_velocity.y,
                                              msg->angular_velocity.z);
    last_unbiased_gyro_ = last_gyro_measurement_ - bg_;
    last_acc_world_ = R_ * (last_acc_measurement_ - ba_) + g_;
    last_imu_time_ = t;
    has_last_imu_ = true;
    has_state_stamp_ = true;
    state_stamp_ = t;
    saveStateSnapshot(state_stamp_);
    if (future_deferral_enable_)
      processReadyDeferredNdtLocked();
    processReadyImuDeskewCloudsLocked();
    return;
  }

  if (!has_last_imu_)
  {
    last_acc_measurement_ = Eigen::Vector3d(msg->linear_acceleration.x,
                                             msg->linear_acceleration.y,
                                             msg->linear_acceleration.z);
    last_gyro_measurement_ = Eigen::Vector3d(msg->angular_velocity.x,
                                              msg->angular_velocity.y,
                                              msg->angular_velocity.z);
    last_unbiased_gyro_ = last_gyro_measurement_ - bg_;
    last_acc_world_ = R_ * (last_acc_measurement_ - ba_) + g_;
    last_imu_time_ = t;
    has_last_imu_ = true;
    has_state_stamp_ = true;
    state_stamp_ = t;
    saveStateSnapshot(state_stamp_);
    if (future_deferral_enable_)
      processReadyDeferredNdtLocked();
    processReadyImuDeskewCloudsLocked();
    return;
  }

  const double dt = t - last_imu_time_;
  last_imu_time_ = t;
  if (dt <= 0.0 || dt > max_imu_dt_) return;

  Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  Eigen::Vector3d gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  propagateImu(acc, gyr, dt);
  has_state_stamp_ = true;
  state_stamp_ = t;
  saveStateSnapshot(state_stamp_);
  if (future_deferral_enable_)
    processReadyDeferredNdtLocked();
  processReadyImuDeskewCloudsLocked();
  if (publish_high_rate_) publishState(msg->header.stamp, false);
  maybePrintRuntime(msg->header.stamp);
}

void DogPriorMapEkfNode::processReadyDeferredNdtLocked()
{
  if (!future_deferral_enable_ || deferred_ndt_observations_.empty() ||
      !has_state_stamp_ || !std::isfinite(state_stamp_))
    return;

  while (!deferred_ndt_observations_.empty() &&
         deferred_ndt_observations_.front().stamp <= state_stamp_ + 1e-9)
  {
    DeferredNdtObservation deferred = deferred_ndt_observations_.front();
    deferred_ndt_observations_.pop_front();
    ++deferred_processed_count_;
    const double wait_ms = std::max(0.0,
        (ros::WallTime::now().toSec() - deferred.received_wall_sec) * 1000.0);
    writeDeferredDiagnostic("NDT_DEFERRED_PROCESS", deferred.stamp, state_stamp_,
                            deferred.future_lead_sec,
                            deferred_ndt_observations_.size(), wait_ms,
                            "PROCESS");
    writeEkfPredictionLineage("NDT_DEFERRED_PROCESS", ros::Time::now().toSec(), 0,
                              deferred.stamp, state_stamp_);
    processNdtObservationLocked(deferred.msg);
  }
}

void DogPriorMapEkfNode::saveStateSnapshot(double stamp)
{
  if ((!oosm_enable_ && !imu_deskew_enable_) || !has_state_stamp_ || !std::isfinite(stamp)) return;

  FilterStateSnapshot snapshot;
  snapshot.stamp = stamp;
  snapshot.p = p_;
  snapshot.v = v_;
  snapshot.R = R_;
  snapshot.ba = ba_;
  snapshot.bg = bg_;
  snapshot.acc_measurement = last_acc_measurement_;
  snapshot.gyro_measurement = last_gyro_measurement_;
  snapshot.acc_world = last_acc_world_;
  snapshot.gyro_unbiased = last_unbiased_gyro_;
  snapshot.P = P_;
  state_history_.insertMonotonic(snapshot);
  pruneStateHistory(stamp);
}

void DogPriorMapEkfNode::restoreStateSnapshot(const FilterStateSnapshot &snapshot)
{
  p_ = snapshot.p;
  v_ = snapshot.v;
  R_ = snapshot.R;
  ba_ = snapshot.ba;
  bg_ = snapshot.bg;
  last_acc_measurement_ = snapshot.acc_measurement;
  last_gyro_measurement_ = snapshot.gyro_measurement;
  last_acc_world_ = snapshot.acc_world;
  last_unbiased_gyro_ = snapshot.gyro_unbiased;
  P_ = snapshot.P;
}

void DogPriorMapEkfNode::pruneStateHistory(double current_stamp)
{
  if ((!oosm_enable_ && !imu_deskew_enable_) || !std::isfinite(current_stamp)) return;
  state_history_.pruneOlderThan(current_stamp, imu_history_keep_sec_);
}

void DogPriorMapEkfNode::eraseStateHistoryAfter(double stamp)
{
  if ((!oosm_enable_ && !imu_deskew_enable_) || !std::isfinite(stamp)) return;
  state_history_.eraseAfter(stamp);
}

// 惯性传播：扣除零偏后积分姿态，并更新速度、位置及 15 维协方差。
void DogPriorMapEkfNode::propagateImu(const Eigen::Vector3d &acc_m, const Eigen::Vector3d &gyr_m, double dt)
{
  // ------------------------- IMU高频传播 -------------------------
  // 机器狗端真正高频输出靠这一段：每个IMU到来就积分一次姿态、速度、位置。
  // 低频地图匹配只负责把漂移拉回来，不需要每帧都做重计算。
  const Eigen::Vector3d acc = acc_m - ba_;
  ImuKinematicsConfig kinematics_config;
  kinematics_config.use_acc_for_position = use_acc_for_position_;
  kinematics_config.velocity_damping = velocity_damping_;
  kinematics_config.continuous_gravity_correction_enable = continuous_gravity_correction_enable_;
  kinematics_config.gravity_correction_expected_acc_norm = gravity_correction_expected_acc_norm_;
  kinematics_config.gravity_correction_gain = gravity_correction_gain_;
  kinematics_config.gravity_correction_max_angle = gravity_correction_max_angle_;
  kinematics_config.gravity_correction_acc_tolerance = gravity_correction_acc_tolerance_;
  kinematics_config.gravity_correction_gyro_max = gravity_correction_gyro_max_;
  propagateImuKinematics(p_, v_, R_, ba_, bg_, acc_m, gyr_m, dt, g_,
                         kinematics_config, last_acc_world_, last_unbiased_gyro_);
  last_acc_measurement_ = acc_m;
  last_gyro_measurement_ = gyr_m;

  // ------------------------- 协方差预测 -------------------------
  // 状态顺序: [位置p, 速度v, 姿态theta, 加计零偏ba, 陀螺零偏bg]，共15维。
  Matrix15d F = Matrix15d::Identity();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(3, 6) = -R_ * skew(acc) * dt;
  F.block<3, 3>(3, 9) = -R_ * dt;
  F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  Matrix15d Q = Matrix15d::Zero();
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * acc_noise_ * acc_noise_ * dt * dt;
  Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * gyro_noise_ * gyro_noise_ * dt * dt;
  Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * acc_bias_noise_ * acc_bias_noise_ * dt;
  Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * gyro_bias_noise_ * gyro_bias_noise_ * dt;
  P_ = F * P_ * F.transpose() + Q;
}

}  // namespace dog_prior_map_localization
