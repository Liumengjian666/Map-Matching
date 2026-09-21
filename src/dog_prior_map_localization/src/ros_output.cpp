#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

namespace
{
const char *localizationModeName(DogPriorMapEkfNode::LocalizationMode mode)
{
  switch (mode)
  {
    case DogPriorMapEkfNode::LocalizationMode::LIDAR_DEGRADED: return "LIDAR_DEGRADED";
    case DogPriorMapEkfNode::LocalizationMode::VISION_ASSISTED: return "VISION_ASSISTED";
    case DogPriorMapEkfNode::LocalizationMode::BOTH_DEGRADED: return "BOTH_DEGRADED";
    case DogPriorMapEkfNode::LocalizationMode::RECOVERY: return "RECOVERY";
    case DogPriorMapEkfNode::LocalizationMode::NORMAL:
    default: return "NORMAL";
  }
}
}  // namespace

// 发布当前导航状态；corrected 区分 LiDAR 校正结果和纯 IMU 高频传播结果。
void DogPriorMapEkfNode::publishState(const ros::Time &stamp, bool corrected)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = p_.x();
  odom.pose.pose.position.y = p_.y();
  odom.pose.pose.position.z = p_.z();
  Eigen::Quaterniond q(R_);
  q.normalize();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = v_.x();
  odom.twist.twist.linear.y = v_.y();
  odom.twist.twist.linear.z = v_.z();

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
    tf_msg.setOrigin(tf::Vector3(p_.x(), p_.y(), p_.z()));
    tf_msg.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_msg, stamp, map_frame_, base_frame_));
  }
}

// 发布已经完成坐标变换与滤波的当前帧点云，供 RViz 和离线诊断使用。
void DogPriorMapEkfNode::publishFilteredCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_body,
                                              const ros::Time &stamp)
{
  if (!publish_filtered_points_ || !pub_filtered_points_ || !scan_body) return;

  // Publish the current scan in the map frame, matching the split NDT
  // visualisation (/points_aligned).  This avoids relying on RViz to apply a
  // moving base_link TF to a body-frame cloud and makes map/scan overlap
  // immediately visible.
  pcl::PointCloud<pcl::PointXYZ> scan_map;
  scan_map.reserve(scan_body->size());
  for (const auto &pt : scan_body->points)
  {
    const Eigen::Vector3d q = R_ * Eigen::Vector3d(pt.x, pt.y, pt.z) + p_;
    scan_map.emplace_back(static_cast<float>(q.x()),
                          static_cast<float>(q.y()),
                          static_cast<float>(q.z()));
  }
  scan_map.width = static_cast<uint32_t>(scan_map.size());
  scan_map.height = 1;
  scan_map.is_dense = true;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(scan_map, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;
  pub_filtered_points_.publish(msg);
}

// 将匹配是否成功、耗时、点数和退化程度封装为 ROS diagnostics 消息。
void DogPriorMapEkfNode::publishDiagnostics(const ros::Time &stamp,
                                            bool converged,
                                            double match_time_ms,
                                            int scan_points,
                                            int map_points,
                                            double score)
{
  if (!publish_diagnostics_ || !pub_diagnostics_) return;

  // 统一追加诊断键值，避免重复构造 KeyValue 消息。
  auto addValue = [](diagnostic_msgs::DiagnosticStatus &status,
                     const std::string &key,
                     const std::string &value) {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "dog_prior_map_localization";
  status.hardware_id = base_frame_;
  status.level = converged ? diagnostic_msgs::DiagnosticStatus::OK
                           : diagnostic_msgs::DiagnosticStatus::WARN;
  status.message = converged ? "NDT/map matching converged"
                             : "NDT/map matching failed or skipped";

  addValue(status, "registration_method", registration_method_);
  addValue(status, "ndt_converged", converged ? "true" : "false");
  addValue(status, "match_time_ms", std::to_string(match_time_ms));
  addValue(status, "scan_points", std::to_string(scan_points));
  addValue(status, "map_points", std::to_string(map_points));
  addValue(status, "score_or_residual", std::to_string(score));
  addValue(status, "lidar_update_ok", std::to_string(lidar_update_ok_count_));
  addValue(status, "lidar_update_fail", std::to_string(lidar_update_fail_count_));
  addValue(status, "icp_ndt_ok", std::to_string(icp_update_ok_count_));
  addValue(status, "icp_ndt_fail", std::to_string(icp_update_fail_count_));
  addValue(status, "degenerate", lidar_degenerate_ ? "true" : "false");
  addValue(status, "degeneracy_score", std::to_string(lidar_degeneracy_score_));
  addValue(status, "localization_mode", localizationModeName(localization_mode_));
  addValue(status, "lidar_information_received", lidar_information_received_ ? "true" : "false");
  addValue(status, "lidar_information_valid", lidar_information_valid_ ? "true" : "false");
  addValue(status, "lidar_information_stale", lidar_information_stale_ ? "true" : "false");
  addValue(status, "lidar_projector_valid", lidar_projector_valid_ ? "true" : "false");
  addValue(status, "lidar_information_stamp", std::to_string(lidar_information_stamp_));
  addValue(status, "information_rotation_scale_m", std::to_string(information_rotation_scale_m_));
  addValue(status, "lidar_information_degenerate", lidar_information_degenerate_ ? "true" : "false");
  addValue(status, "lidar_information_condition", std::to_string(lidar_information_condition_));
  addValue(status, "lidar_information_eigenvalues",
           std::to_string(lidar_information_eigenvalues_(0)) + "," +
           std::to_string(lidar_information_eigenvalues_(1)) + "," +
           std::to_string(lidar_information_eigenvalues_(2)) + "," +
           std::to_string(lidar_information_eigenvalues_(3)) + "," +
           std::to_string(lidar_information_eigenvalues_(4)) + "," +
           std::to_string(lidar_information_eigenvalues_(5)));
  addValue(status, "lidar_information_weak_eigenvector",
           std::to_string(lidar_information_weak_eigenvector_(0)) + "," +
           std::to_string(lidar_information_weak_eigenvector_(1)) + "," +
           std::to_string(lidar_information_weak_eigenvector_(2)) + "," +
           std::to_string(lidar_information_weak_eigenvector_(3)) + "," +
           std::to_string(lidar_information_weak_eigenvector_(4)) + "," +
           std::to_string(lidar_information_weak_eigenvector_(5)));
  addValue(status, "lidar_geometry_degeneracy_score", std::to_string(lidar_geometry_degeneracy_score_));
  addValue(status, "lidar_geometry_degeneracy_valid", lidar_geometry_degeneracy_valid_ ? "true" : "false");
  addValue(status, "visual_feature_count", std::to_string(last_visual_feature_count_));
  addValue(status, "visual_tracked_count", std::to_string(last_visual_tracked_count_));
  addValue(status, "visual_inlier_count", std::to_string(last_visual_inlier_count_));
  addValue(status, "visual_flow_residual_px", std::to_string(last_visual_flow_residual_px_));
  addValue(status, "visual_flow_residual_valid", last_visual_flow_residual_valid_ ? "true" : "false");
  addValue(status, "visual_reprojection_error_px", std::to_string(last_visual_reprojection_error_px_));
  addValue(status, "visual_relative_pose",
           std::to_string(last_visual_relative_pose_(0)) + "," +
           std::to_string(last_visual_relative_pose_(1)) + "," +
           std::to_string(last_visual_relative_pose_(2)) + "," +
           std::to_string(last_visual_relative_pose_(3)) + "," +
           std::to_string(last_visual_relative_pose_(4)) + "," +
           std::to_string(last_visual_relative_pose_(5)));
  addValue(status, "visual_relative_pose_valid", last_visual_relative_pose_valid_ ? "true" : "false");
  addValue(status, "visual_imu_rotation_residual_deg", std::to_string(last_visual_imu_rotation_residual_deg_));
  addValue(status, "visual_imu_rotation_valid", last_visual_imu_rotation_valid_ ? "true" : "false");
  addValue(status, "visual_translation_scale_m", std::to_string(last_visual_translation_scale_m_));
  addValue(status, "visual_covariance_diag",
           std::to_string(last_visual_covariance_diag_(0)) + "," +
           std::to_string(last_visual_covariance_diag_(1)) + "," +
           std::to_string(last_visual_covariance_diag_(2)) + "," +
           std::to_string(last_visual_covariance_diag_(3)) + "," +
           std::to_string(last_visual_covariance_diag_(4)) + "," +
           std::to_string(last_visual_covariance_diag_(5)));
  addValue(status, "visual_metric_translation_valid", last_visual_metric_translation_valid_ ? "true" : "false");
  addValue(status, "visual_reprojection_valid", last_visual_reprojection_valid_ ? "true" : "false");
  addValue(status, "visual_covariance_valid", last_visual_covariance_valid_ ? "true" : "false");
  addValue(status, "visual_update_reason", last_visual_update_reason_);
  addValue(status, "pose_xyz", std::to_string(p_.x()) + "," +
                              std::to_string(p_.y()) + "," +
                              std::to_string(p_.z()));
  array.status.push_back(status);
  pub_diagnostics_.publish(array);
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
      << lidar_update_ok_count_ << ","
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

// 定期汇总输入频率、匹配成功率和耗时，并可追加写入运行统计 CSV。
void DogPriorMapEkfNode::maybePrintRuntime(const ros::Time &stamp)
{
  if (!print_debug_) return;
  const double now = stamp.toSec();
  if (last_debug_time_ > 0.0 && now - last_debug_time_ < debug_interval_sec_) return;
  if (last_debug_time_ <= 0.0)
  {
    last_debug_time_ = now;
    last_debug_imu_count_ = imu_msg_count_;
    last_debug_lidar_count_ = lidar_msg_count_;
    last_debug_update_ok_count_ = lidar_update_ok_count_;
    last_debug_image_count_ = image_msg_count_;
    return;
  }

  const double dt = std::max(now - last_debug_time_, 1e-6);
  const uint64_t imu_delta = imu_msg_count_ - last_debug_imu_count_;
  const uint64_t lidar_delta = lidar_msg_count_ - last_debug_lidar_count_;
  const uint64_t update_delta = lidar_update_ok_count_ - last_debug_update_ok_count_;
  const uint64_t image_delta = image_msg_count_ - last_debug_image_count_;
  const uint64_t update_total = std::max<uint64_t>(lidar_update_ok_count_ + lidar_update_fail_count_, 1);
  const double avg_update_ms = lidar_update_time_sum_ms_ / static_cast<double>(update_total);
  const uint64_t icp_total = std::max<uint64_t>(icp_update_ok_count_ + icp_update_fail_count_, 1);
  const double avg_icp_ms = icp_update_time_sum_ms_ / static_cast<double>(icp_total);
  const uint64_t image_total = std::max<uint64_t>(image_msg_count_, 1);
  const double avg_visual_ms = visual_update_time_sum_ms_ / static_cast<double>(image_total);

  ROS_INFO("[DogPriorMap C++] runtime: imu=%.1fHz lidar=%.1fHz image=%.1fHz corr=%.2fHz lidar_ms(avg/max)=%.2f/%.2f ndt_ms(avg/max)=%.2f/%.2f visual_ms(avg/max)=%.2f/%.2f feature=%.2f v_weight=%.2f degen=%d score=%.2f ok/fail=%lu/%lu ndt=%lu/%lu",
           static_cast<double>(imu_delta) / dt,
           static_cast<double>(lidar_delta) / dt,
           static_cast<double>(image_delta) / dt,
           static_cast<double>(update_delta) / dt,
           avg_update_ms,
           lidar_update_time_max_ms_,
           avg_icp_ms,
           icp_update_time_max_ms_,
           avg_visual_ms,
           visual_update_time_max_ms_,
           last_feature_ratio_,
           visual_constraint_weight_scale_,
           lidar_degenerate_ ? 1 : 0,
           lidar_degeneracy_score_,
           static_cast<unsigned long>(lidar_update_ok_count_),
           static_cast<unsigned long>(lidar_update_fail_count_),
           static_cast<unsigned long>(icp_update_ok_count_),
           static_cast<unsigned long>(icp_update_fail_count_));

  if (runtime_csv_.is_open())
  {
    runtime_csv_ << now << ","
                 << static_cast<double>(imu_delta) / dt << ","
                 << static_cast<double>(lidar_delta) / dt << ","
                 << static_cast<double>(update_delta) / dt << ","
                 << avg_update_ms << ","
                 << lidar_update_time_max_ms_ << ","
                 << avg_icp_ms << ","
                 << icp_update_time_max_ms_ << ","
                 << icp_update_ok_count_ << ","
                 << icp_update_fail_count_ << ","
                 << last_used_points_ << ","
                 << last_mean_residual_ << ","
                 << lidar_update_ok_count_ << ","
                 << lidar_update_fail_count_ << ","
                 << static_cast<double>(image_delta) / dt << ","
                 << avg_visual_ms << ","
                 << visual_update_time_max_ms_ << ","
                 << last_feature_ratio_ << ","
                 << visual_constraint_weight_scale_ << ","
                 << (lidar_degenerate_ ? 1 : 0) << ","
                 << lidar_degeneracy_score_ << ","
                 << visual_update_ok_count_ << ","
                 << visual_update_fail_count_ << ","
                 << "rss_sampled_by_ps" << ","
                 << localizationModeName(localization_mode_) << ","
                 << (lidar_information_received_ ? 1 : 0) << ","
                 << (lidar_information_valid_ ? 1 : 0) << ","
                 << (lidar_information_degenerate_ ? 1 : 0) << ","
                 << (lidar_information_stale_ ? 1 : 0) << ","
                 << (lidar_projector_valid_ ? 1 : 0) << ","
                 << lidar_information_stamp_ << ","
                 << lidar_information_condition_ << ","
                 << lidar_information_eigenvalues_(0) << ","
                 << lidar_information_eigenvalues_(1) << ","
                 << lidar_information_eigenvalues_(2) << ","
                 << lidar_information_eigenvalues_(3) << ","
                 << lidar_information_eigenvalues_(4) << ","
                 << lidar_information_eigenvalues_(5) << ","
                 << lidar_information_weak_eigenvector_(0) << ","
                 << lidar_information_weak_eigenvector_(1) << ","
                 << lidar_information_weak_eigenvector_(2) << ","
                 << lidar_information_weak_eigenvector_(3) << ","
                 << lidar_information_weak_eigenvector_(4) << ","
                 << lidar_information_weak_eigenvector_(5) << ","
                 << lidar_geometry_degeneracy_score_ << ","
                 << (lidar_geometry_degeneracy_valid_ ? 1 : 0) << ","
                 << last_visual_feature_count_ << ","
                 << last_visual_tracked_count_ << ","
                 << last_visual_inlier_count_ << ","
                 << last_visual_flow_residual_px_ << ","
                 << (last_visual_flow_residual_valid_ ? 1 : 0) << ","
                 << last_visual_reprojection_error_px_ << ","
                 << last_visual_relative_pose_(0) << ","
                 << last_visual_relative_pose_(1) << ","
                 << last_visual_relative_pose_(2) << ","
                 << last_visual_relative_pose_(3) << ","
                 << last_visual_relative_pose_(4) << ","
                 << last_visual_relative_pose_(5) << ","
                 << (last_visual_relative_pose_valid_ ? 1 : 0) << ","
                 << last_visual_imu_rotation_residual_deg_ << ","
                 << (last_visual_imu_rotation_valid_ ? 1 : 0) << ","
                 << last_visual_translation_scale_m_ << ","
                 << last_visual_covariance_diag_(0) << ","
                 << last_visual_covariance_diag_(1) << ","
                 << last_visual_covariance_diag_(2) << ","
                 << last_visual_covariance_diag_(3) << ","
                 << last_visual_covariance_diag_(4) << ","
                 << last_visual_covariance_diag_(5) << ","
                 << (last_visual_metric_translation_valid_ ? 1 : 0) << ","
                 << (last_visual_reprojection_valid_ ? 1 : 0) << ","
                 << (last_visual_covariance_valid_ ? 1 : 0) << ","
                 << last_visual_update_reason_
                 << "\n";
    runtime_csv_.flush();
  }

  last_debug_time_ = now;
  last_debug_imu_count_ = imu_msg_count_;
  last_debug_lidar_count_ = lidar_msg_count_;
  last_debug_update_ok_count_ = lidar_update_ok_count_;
  last_debug_image_count_ = image_msg_count_;
}

}  // namespace dog_prior_map_localization
