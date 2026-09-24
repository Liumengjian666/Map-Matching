#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

// 发布当前导航状态；corrected 区分 LiDAR 校正结果和纯 IMU 高频传播结果。
void DogPriorMapEkfNode::publishState(const ros::Time &stamp, bool corrected)
{
  Eigen::Vector3d output_position = p_;
  Eigen::Vector3d output_linear_velocity = v_;
  Eigen::Vector3d output_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d output_rotation = R_;
  if (imu_deskew_enable_)
  {
    Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
    T_world_imu.linear() = R_;
    T_world_imu.translation() = p_;
    const Eigen::Isometry3d T_world_lidar = T_world_imu * T_imu_lidar_;
    output_position = T_world_lidar.translation();
    output_rotation = T_world_lidar.linear();
    const Eigen::Vector3d lidar_lever_arm_velocity =
        R_ * last_unbiased_gyro_.cross(T_imu_lidar_.translation());
    output_linear_velocity = output_rotation.transpose() * (v_ + lidar_lever_arm_velocity);
    output_angular_velocity = T_imu_lidar_.linear().transpose() * last_unbiased_gyro_;
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = output_position.x();
  odom.pose.pose.position.y = output_position.y();
  odom.pose.pose.position.z = output_position.z();
  Eigen::Quaterniond q(output_rotation);
  q.normalize();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = output_linear_velocity.x();
  odom.twist.twist.linear.y = output_linear_velocity.y();
  odom.twist.twist.linear.z = output_linear_velocity.z();
  odom.twist.twist.angular.x = output_angular_velocity.x();
  odom.twist.twist.angular.y = output_angular_velocity.y();
  odom.twist.twist.angular.z = output_angular_velocity.z();

  // Keep bounded, sensor-time-sampled trajectory histories.  The odometry
  // topics below remain high-rate; only the RViz Path messages are sampled.
  if (publish_path_)
  {
    if (!corrected && shouldSamplePath(stamp, last_path_high_sample_stamp_))
      appendPath(path_high_, odom);
    if (shouldSamplePath(stamp, last_path_corr_sample_stamp_))
      appendPath(path_corr_, odom);
  }

  if (corrected)
  {
    writeEkfPredictionLineage("CORRECTED_PUBLISH", ros::Time::now().toSec());
    pub_corr_.publish(odom);
  }
  else
  {
    const uint64_t publish_seq = ++high_rate_publish_seq_;
    writeEkfPredictionLineage("IMU_HIGH_RATE_PUBLISH", ros::Time::now().toSec(), publish_seq);
    pub_high_.publish(odom);
    if (publish_imu_propagate_alias_)
    {
      pub_imu_propagate_.publish(odom);
    }
  }

  publishPathsIfDue(stamp);

  // The high-rate current-state stream is the single TF authority.  A
  // corrected/OOSM publication may share the same sensor timestamp as the
  // following IMU publication; sending both creates TF_REPEATED_DATA and
  // gives TF two competing writers for camera_init -> livox_frame.
  if (publish_tf_ && !corrected)
  {
    tf::Transform tf_msg;
    tf_msg.setOrigin(tf::Vector3(output_position.x(), output_position.y(), output_position.z()));
    tf_msg.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_msg, stamp, map_frame_, base_frame_));
  }
}

void DogPriorMapEkfNode::writeOosmDiagnostic(double ndt_stamp,
                                             double state_now_stamp,
                                             double rollback_stamp,
                                             double lag_sec,
                                             double alignment_error_sec,
                                             size_t replay_imu_count,
                                             size_t state_history_count,
                                             size_t imu_history_count,
                                             const std::string &result)
{
  if (!oosm_csv_.is_open()) return;
  oosm_csv_ << ++oosm_frame_index_ << ","
            << ndt_stamp << ","
            << state_now_stamp << ","
            << rollback_stamp << ","
            << lag_sec * 1000.0 << ","
            << alignment_error_sec * 1000.0 << ","
            << replay_imu_count << ","
            << state_history_count << ","
            << imu_history_count << ","
            << result << "\n";
  oosm_csv_.flush();
}

void DogPriorMapEkfNode::writeEkfPredictionLineage(const std::string &event_type,
                                                   double event_ros_stamp,
                                                   uint64_t publish_seq,
                                                   double ndt_measurement_stamp,
                                                   double state_now_before_ndt,
                                                   double state_now_after_replay,
                                                   double rollback_stamp,
                                                   size_t replay_imu_count,
                                                   double lag_ms,
                                                   double rewrite_translation_m,
                                                   double rewrite_rotation_deg,
                                                   double rewrite_velocity_mps)
{
  if (!ekf_prediction_diagnostics_csv_.is_open()) return;

  Eigen::Quaterniond q(R_);
  q.normalize();
  ekf_prediction_diagnostics_csv_ << ++ekf_prediction_event_index_ << ","
      << event_type << ","
      << event_ros_stamp << ","
      << state_stamp_ << ","
      << p_.x() << "," << p_.y() << "," << p_.z() << ","
      << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
      << v_.x() << "," << v_.y() << "," << v_.z() << ","
      << last_ndt_observation_time_ << ","
      << ndt_correction_count_ << ","
      << imu_msg_count_ << ","
      << oosm_frame_index_ << ","
      << state_history_.size() << ","
      << imu_history_.size() << ","
      << ekf_state_revision_ << ","
      << publish_seq << ","
      << ndt_measurement_stamp << ","
      << state_now_before_ndt << ","
      << state_now_after_replay << ","
      << rollback_stamp << ","
      << replay_imu_count << ","
      << lag_ms << ","
      << rewrite_translation_m << ","
      << rewrite_rotation_deg << ","
      << rewrite_velocity_mps << "\n";
  ekf_prediction_diagnostics_csv_.flush();
}

// 将里程计位姿转换为 PoseStamped 追加到轨迹，并裁剪过长的历史缓存。
void DogPriorMapEkfNode::appendPath(nav_msgs::Path &path, const nav_msgs::Odometry &odom)
{
  geometry_msgs::PoseStamped ps;
  ps.header = odom.header;
  ps.pose = odom.pose.pose;
  path.header.stamp = odom.header.stamp;
  path.header.frame_id = map_frame_;
  path.poses.push_back(ps);
  if (static_cast<int>(path.poses.size()) > path_max_length_)
  {
    path.poses.erase(path.poses.begin(), path.poses.begin() + (path.poses.size() - path_max_length_));
  }
}

bool DogPriorMapEkfNode::shouldSamplePath(const ros::Time &stamp,
                                          double &last_sample_stamp) const
{
  const double sensor_stamp = stamp.toSec();
  if (!std::isfinite(sensor_stamp) || !std::isfinite(path_sample_rate_hz_) ||
      path_sample_rate_hz_ <= 0.0)
    return false;
  const double interval = 1.0 / path_sample_rate_hz_;
  if (last_sample_stamp >= 0.0 &&
      (sensor_stamp <= last_sample_stamp || sensor_stamp - last_sample_stamp < interval))
    return false;
  last_sample_stamp = sensor_stamp;
  return true;
}

void DogPriorMapEkfNode::publishPathsIfDue(const ros::Time &stamp)
{
  if (!publish_path_) return;
  const double sensor_stamp = stamp.toSec();
  if (!std::isfinite(sensor_stamp) || !std::isfinite(path_publish_rate_hz_) ||
      path_publish_rate_hz_ <= 0.0)
    return;
  const double interval = 1.0 / path_publish_rate_hz_;
  if (last_path_publish_stamp_ >= 0.0 &&
      (sensor_stamp <= last_path_publish_stamp_ ||
       sensor_stamp - last_path_publish_stamp_ < interval))
    return;
  last_path_publish_stamp_ = sensor_stamp;
  path_high_.header.stamp = stamp;
  path_high_.header.frame_id = map_frame_;
  path_corr_.header.stamp = stamp;
  path_corr_.header.frame_id = map_frame_;
  pub_path_high_.publish(path_high_);
  pub_path_corr_.publish(path_corr_);
}

// 定期汇总 IMU、NDT 校正和 OOSM 运行统计，并可追加写入 CSV。
void DogPriorMapEkfNode::maybePrintRuntime(const ros::Time &stamp)
{
  if (!print_debug_) return;
  const double now = stamp.toSec();
  if (last_debug_time_ > 0.0 && now - last_debug_time_ < debug_interval_sec_) return;
  if (last_debug_time_ <= 0.0)
  {
    last_debug_time_ = now;
    last_debug_imu_count_ = imu_msg_count_;
    last_debug_ndt_correction_count_ = ndt_correction_count_;
    return;
  }

  const double dt = std::max(now - last_debug_time_, 1e-6);
  const uint64_t imu_delta = imu_msg_count_ - last_debug_imu_count_;
  const uint64_t correction_delta = ndt_correction_count_ - last_debug_ndt_correction_count_;

  ROS_INFO("[DogPriorMap C++] runtime: imu=%.1fHz corr=%.2fHz oosm=%lu deferred=%lu/%lu ndt_corrections=%lu",
           static_cast<double>(imu_delta) / dt,
           static_cast<double>(correction_delta) / dt,
           static_cast<unsigned long>(oosm_frame_index_),
           static_cast<unsigned long>(deferred_received_count_),
           static_cast<unsigned long>(deferred_processed_count_),
           static_cast<unsigned long>(ndt_correction_count_));

  if (runtime_csv_.is_open())
  {
    runtime_csv_ << now << ","
                 << static_cast<double>(imu_delta) / dt << ","
                 << static_cast<double>(correction_delta) / dt << ","
                 << ndt_correction_count_ << ","
                 << oosm_frame_index_ << ","
                 << deferred_received_count_ << ","
                 << deferred_processed_count_ << ","
                 << deferred_over_limit_count_ << ","
                 << deferred_queue_full_count_ << ","
                 << max_deferred_queue_size_ << ","
                 << state_history_.size() << ","
                 << imu_history_.size()
                 << "\n";
    runtime_csv_.flush();
  }

  last_debug_time_ = now;
  last_debug_imu_count_ = imu_msg_count_;
  last_debug_ndt_correction_count_ = ndt_correction_count_;
}

}  // namespace dog_prior_map_localization
