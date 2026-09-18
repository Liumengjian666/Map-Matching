#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

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
    last_imu_time_ = t;
    has_last_imu_ = true;
    has_state_stamp_ = true;
    state_stamp_ = t;
    saveStateSnapshot(state_stamp_);
    return;
  }

  if (!has_last_imu_)
  {
    last_imu_time_ = t;
    has_last_imu_ = true;
    has_state_stamp_ = true;
    state_stamp_ = t;
    saveStateSnapshot(state_stamp_);
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
  if (publish_high_rate_) publishState(msg->header.stamp, false);
  maybePrintRuntime(msg->header.stamp);
}

void DogPriorMapEkfNode::saveStateSnapshot(double stamp)
{
  if (!oosm_enable_ || !has_state_stamp_ || !std::isfinite(stamp)) return;
  if (!state_history_.empty() && stamp < state_history_.back().stamp - 1e-9) return;

  while (!state_history_.empty() &&
         std::abs(state_history_.back().stamp - stamp) <= 1e-9)
  {
    state_history_.pop_back();
  }

  FilterStateSnapshot snapshot;
  snapshot.stamp = stamp;
  snapshot.p = p_;
  snapshot.v = v_;
  snapshot.R = R_;
  snapshot.ba = ba_;
  snapshot.bg = bg_;
  snapshot.P = P_;
  state_history_.push_back(snapshot);
  pruneStateHistory(stamp);
}

void DogPriorMapEkfNode::restoreStateSnapshot(const FilterStateSnapshot &snapshot)
{
  p_ = snapshot.p;
  v_ = snapshot.v;
  R_ = snapshot.R;
  ba_ = snapshot.ba;
  bg_ = snapshot.bg;
  P_ = snapshot.P;
}

void DogPriorMapEkfNode::pruneStateHistory(double current_stamp)
{
  if (!oosm_enable_ || !std::isfinite(current_stamp)) return;
  while (!state_history_.empty() &&
         current_stamp - state_history_.front().stamp > imu_history_keep_sec_)
  {
    state_history_.pop_front();
  }
}

bool DogPriorMapEkfNode::findStateSnapshotAtOrBefore(double target_stamp,
                                                      size_t &index,
                                                      double &alignment_error) const
{
  index = 0;
  alignment_error = std::numeric_limits<double>::quiet_NaN();
  if (!oosm_enable_ || !std::isfinite(target_stamp) || state_history_.empty()) return false;

  for (size_t i = state_history_.size(); i > 0; --i)
  {
    const FilterStateSnapshot &snapshot = state_history_[i - 1];
    if (snapshot.stamp <= target_stamp + 1e-9)
    {
      index = i - 1;
      alignment_error = std::max(0.0, target_stamp - snapshot.stamp);
      return std::isfinite(alignment_error);
    }
  }
  return false;
}

void DogPriorMapEkfNode::eraseStateHistoryAfter(double stamp)
{
  if (!oosm_enable_ || !std::isfinite(stamp)) return;
  while (!state_history_.empty() && state_history_.back().stamp > stamp + 1e-9)
  {
    state_history_.pop_back();
  }
}

// 惯性传播：扣除零偏后积分姿态，并更新速度、位置及 15 维协方差。
void DogPriorMapEkfNode::propagateImu(const Eigen::Vector3d &acc_m, const Eigen::Vector3d &gyr_m, double dt)
{
  // ------------------------- IMU高频传播 -------------------------
  // 机器狗端真正高频输出靠这一段：每个IMU到来就积分一次姿态、速度、位置。
  // 低频地图匹配只负责把漂移拉回来，不需要每帧都做重计算。
  const Eigen::Vector3d acc = acc_m - ba_;
  const Eigen::Vector3d gyr = gyr_m - bg_;
  const Eigen::Matrix3d dR = Eigen::AngleAxisd(gyr.norm() * dt, gyr.norm() > 1e-12 ? gyr.normalized() : Eigen::Vector3d::UnitX()).toRotationMatrix();
  R_ = R_ * dR;

  if (continuous_gravity_correction_enable_)
  {
    // ------------------------- 连续重力方向姿态约束 -------------------------
    // 这不是Z轴位置阻尼：不直接修改p_.z，也不假设机器人必须在某个固定高度。
    // 只在加速度模长接近9.8、角速度不大时，把“当前IMU测到的重力/比力方向”
    // 缓慢对齐到地图Z轴，用来抑制roll/pitch漂移。roll/pitch一旦漂，雷达点投到
    // 先验地图时高度会系统性偏掉，长走廊里就会表现成Z轴越走越歪。
    const double acc_norm = acc_m.norm();
    const double gyro_norm = gyr_m.norm();
    if (acc_norm > 1e-3 &&
        std::abs(acc_norm - gravity_correction_expected_acc_norm_) <= gravity_correction_acc_tolerance_ &&
        gyro_norm <= gravity_correction_gyro_max_)
    {
      const Eigen::Vector3d measured_up_map = (R_ * acc_m.normalized()).normalized();
      const Eigen::Vector3d expected_up_map = Eigen::Vector3d::UnitZ();
      Eigen::Quaterniond q_full;
      q_full.setFromTwoVectors(measured_up_map, expected_up_map);
      Eigen::AngleAxisd aa(q_full);
      Eigen::Vector3d rotvec = aa.axis() * aa.angle();
      rotvec = limitVector(rotvec * gravity_correction_gain_, gravity_correction_max_angle_);
      if (rotvec.allFinite() && rotvec.norm() > 1e-12)
      {
        R_ = Eigen::AngleAxisd(rotvec.norm(), rotvec.normalized()).toRotationMatrix() * R_;
      }
    }
  }

  if (use_acc_for_position_)
  {
    // 高精度IMU且零偏估计稳定时，可以启用完整加速度位置积分。
    // 但MID360板载IMU直接裸积分会很快被零偏和重力误差放大，所以机器狗定位默认关闭。
    const Eigen::Vector3d acc_world = R_ * acc + g_;
    p_ = p_ + v_ * dt + 0.5 * acc_world * dt * dt;
    v_ = v_ + acc_world * dt;
  }
  else
  {
    // 低算力定位默认模式：IMU负责高频姿态/短时平滑，位置主要由先验地图匹配修正。
    // 这样不会因为几mg的加速度计零偏，在几十秒内把Z轴积分到几百米外。
    p_ = p_ + v_ * dt;
    v_ *= std::pow(std::max(0.0, std::min(1.0, velocity_damping_)), dt * 200.0);
  }

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
