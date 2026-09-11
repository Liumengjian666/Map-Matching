#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

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

  // Keep a complete fused trajectory history.  The split NDT node appends
  // every accepted frame to its path; doing the same here avoids a path that
  // only contains successful correction instants.
  if (publish_path_)
  {
    appendPath(path_corr_, odom);
  }

  if (corrected)
  {
    pub_corr_.publish(odom);
    if (publish_path_)
    {
      pub_path_corr_.publish(path_corr_);
    }
  }
  else
  {
    pub_high_.publish(odom);
    if (publish_imu_propagate_alias_)
    {
      pub_imu_propagate_.publish(odom);
    }
    if (publish_path_)
    {
      appendPath(path_high_, odom);
      pub_path_high_.publish(path_high_);
      // Publish the fused trajectory on every high-rate propagation as well.
      // Otherwise path_corrected only refreshes when a LiDAR correction is
      // accepted and appears frozen/short during corridor degeneracy.
      pub_path_corr_.publish(path_corr_);
    }
  }

  if (publish_tf_)
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
  addValue(status, "pose_xyz", std::to_string(p_.x()) + "," +
                              std::to_string(p_.y()) + "," +
                              std::to_string(p_.z()));
  array.status.push_back(status);
  pub_diagnostics_.publish(array);
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
                 << "rss_sampled_by_ps"
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
