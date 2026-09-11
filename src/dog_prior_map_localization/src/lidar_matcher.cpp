#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

#include <pcl/registration/ndt.h>

namespace dog_prior_map_localization
{

namespace
{
// 按 FAST-LIVO2 的量测模型估计单个雷达点在机体系中的各向异性协方差。
Eigen::Matrix3d calcFastLivoBodyCov(const Eigen::Vector3d &point_body,
                                    double depth_noise,
                                    double beam_noise_deg)
{
  Eigen::Vector3d direction = point_body;
  if (std::abs(direction.z()) < 1e-4)
  {
    direction.z() = direction.z() >= 0.0 ? 1e-4 : -1e-4;
  }
  const double range = std::max(direction.norm(), 1e-6);
  direction /= range;

  Eigen::Vector3d base_vector1(1.0, 1.0, -(direction.x() + direction.y()) / direction.z());
  if (base_vector1.norm() < 1e-9)
  {
    base_vector1 = Eigen::Vector3d::UnitX();
  }
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  if (base_vector2.norm() < 1e-9)
  {
    base_vector2 = Eigen::Vector3d::UnitY();
  }
  base_vector2.normalize();

  Eigen::Matrix3d direction_hat;
  direction_hat << 0.0, -direction.z(), direction.y(),
                   direction.z(), 0.0, -direction.x(),
                   -direction.y(), direction.x(), 0.0;
  Eigen::Matrix<double, 3, 2> normal_basis;
  normal_basis << base_vector1.x(), base_vector2.x(),
                  base_vector1.y(), base_vector2.y(),
                  base_vector1.z(), base_vector2.z();

  const double beam_sigma = std::sin(beam_noise_deg * M_PI / 180.0);
  Eigen::Matrix2d direction_var = Eigen::Matrix2d::Identity() * beam_sigma * beam_sigma;
  Eigen::Matrix<double, 3, 2> angular_jacobian = range * direction_hat * normal_basis;
  return direction * depth_noise * depth_noise * direction.transpose() +
         angular_jacobian * direction_var * angular_jacobian.transpose();
}
}  // namespace

// Livox 点云入口：可选执行帧内去畸变，再交给统一地图匹配流程。
void DogPriorMapEkfNode::livoxCallback(const livox_ros_driver2::CustomMsgConstPtr &msg)
{
  if (lidar_deskew_enable_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = deskewLivoxCloud(msg);
    handleLidarCloud(cloud, msg->header.stamp);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(msg->points.size());
  for (const auto &pt : msg->points)
  {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
    {
      cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }
  }
  handleLidarCloud(cloud, msg->header.stamp);
}

// 将 Livox 一帧中不同采样时刻的点补偿到帧尾，降低运动造成的点云弯曲。
pcl::PointCloud<pcl::PointXYZ>::Ptr DogPriorMapEkfNode::deskewLivoxCloud(const livox_ros_driver2::CustomMsgConstPtr &msg)
{
  // ------------------------- Livox完整帧内去畸变 -------------------------
  // Livox ROS Driver2 的 CustomPoint::offset_time 是相对本帧base_time/header.stamp的纳秒偏移。
  // MID360一帧点云不是同一时刻采到的；如果机器狗在走/转，直接拿整帧匹配会把墙、地面拉弯。
  // 地图匹配里雷达点云是主约束，所以这里默认做旋转+平移补偿：
  // 1) 用IMU角速度积分点时刻 -> 帧尾时刻的相对旋转；
  // 2) 用IMU加速度在短时间内估计相对平移；
  // 3) 把每个点统一到帧尾坐标系。
  // 注意：平移补偿只跨一帧(约0.1s)，不会像长时间裸积分那样无限漂。
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(msg->points.size());

  if (msg->points.empty() || imu_history_.size() < 2)
  {
    for (const auto &pt : msg->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      {
        cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      }
    }
    return cloud;
  }

  double max_offset_sec = 0.0;
  for (const auto &pt : msg->points)
  {
    max_offset_sec = std::max(max_offset_sec, static_cast<double>(pt.offset_time) * lidar_offset_time_scale_);
  }

  const double frame_start = msg->header.stamp.toSec();
  const double frame_end = frame_start + max_offset_sec;
  if (imu_history_.front().stamp > frame_start || imu_history_.back().stamp < frame_end)
  {
    ROS_WARN_THROTTLE(2.0,
                      "[DogPriorMap C++] IMU history insufficient, skip lidar deskew: imu=[%.6f, %.6f] lidar=[%.6f, %.6f]",
                      imu_history_.front().stamp, imu_history_.back().stamp, frame_start, frame_end);
    for (const auto &pt : msg->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      {
        cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      }
    }
    return cloud;
  }

  for (const auto &pt : msg->points)
  {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    const double point_time = frame_start + static_cast<double>(pt.offset_time) * lidar_offset_time_scale_;
    Eigen::Matrix3d R_point_to_end = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_point_to_end = Eigen::Vector3d::Zero();
    if (!integrateImuDelta(point_time, frame_end, R_point_to_end, p_point_to_end))
    {
      R_point_to_end = integrateImuRotation(point_time, frame_end);
      p_point_to_end.setZero();
    }
    if (!lidar_deskew_translation_enable_)
    {
      p_point_to_end.setZero();
    }
    Eigen::Vector3d p_raw(pt.x, pt.y, pt.z);
    Eigen::Vector3d p_deskew = R_point_to_end * p_raw + p_point_to_end;
    cloud->push_back(pcl::PointXYZ(p_deskew.x(), p_deskew.y(), p_deskew.z()));
  }
  return cloud;
}

// 在短时间区间内积分 IMU，计算点时刻到帧尾时刻的相对旋转和平移。
bool DogPriorMapEkfNode::integrateImuDelta(double t0, double t1, Eigen::Matrix3d &R_delta, Eigen::Vector3d &p_delta) const
{
  if (t1 <= t0 || imu_history_.size() < 2)
  {
    R_delta = Eigen::Matrix3d::Identity();
    p_delta.setZero();
    return false;
  }

  R_delta = Eigen::Matrix3d::Identity();
  Eigen::Vector3d v_delta = Eigen::Vector3d::Zero();
  p_delta.setZero();
  double last_t = t0;
  bool used = false;

  for (size_t i = 0; i + 1 < imu_history_.size(); ++i)
  {
    const ImuSample &a = imu_history_[i];
    const ImuSample &b = imu_history_[i + 1];
    if (b.stamp <= t0 || a.stamp >= t1) continue;

    const double seg0 = std::max(t0, a.stamp);
    const double seg1 = std::min(t1, b.stamp);
    if (seg1 <= seg0) continue;

    const double alpha = (seg0 - a.stamp) / std::max(b.stamp - a.stamp, 1e-9);
    const Eigen::Vector3d gyro = ((1.0 - alpha) * a.gyro + alpha * b.gyro) - bg_;
    const Eigen::Vector3d acc = ((1.0 - alpha) * a.acc + alpha * b.acc) - ba_;
    const double dt = seg1 - seg0;

    const Eigen::Vector3d acc_local = R_delta * acc;
    p_delta += v_delta * dt + 0.5 * acc_local * dt * dt;
    v_delta += acc_local * dt;

    const double angle = gyro.norm() * dt;
    if (angle > 1e-12)
    {
      R_delta = R_delta * Eigen::AngleAxisd(angle, gyro.normalized()).toRotationMatrix();
    }
    last_t = seg1;
    used = true;
  }

  if (!used) return false;
  if (last_t < t1)
  {
    const Eigen::Vector3d gyro = imu_history_.back().gyro - bg_;
    const Eigen::Vector3d acc = imu_history_.back().acc - ba_;
    const double dt = t1 - last_t;
    const Eigen::Vector3d acc_local = R_delta * acc;
    p_delta += v_delta * dt + 0.5 * acc_local * dt * dt;
    const double angle = gyro.norm() * dt;
    if (angle > 1e-12)
    {
      R_delta = R_delta * Eigen::AngleAxisd(angle, gyro.normalized()).toRotationMatrix();
    }
  }
  return R_delta.allFinite() && p_delta.allFinite();
}

// 仅积分陀螺仪得到相对旋转，作为平移积分不可用时的去畸变降级方案。
Eigen::Matrix3d DogPriorMapEkfNode::integrateImuRotation(double t0, double t1) const
{
  if (t1 <= t0 || imu_history_.size() < 2) return Eigen::Matrix3d::Identity();

  Eigen::Matrix3d R_delta = Eigen::Matrix3d::Identity();
  double last_t = t0;

  for (size_t i = 0; i + 1 < imu_history_.size(); ++i)
  {
    const ImuSample &a = imu_history_[i];
    const ImuSample &b = imu_history_[i + 1];
    if (b.stamp <= t0 || a.stamp >= t1) continue;

    const double seg0 = std::max(t0, a.stamp);
    const double seg1 = std::min(t1, b.stamp);
    if (seg1 <= seg0) continue;

    const double alpha = (seg0 - a.stamp) / std::max(b.stamp - a.stamp, 1e-9);
    Eigen::Vector3d gyro = (1.0 - alpha) * a.gyro + alpha * b.gyro - bg_;
    const double dt = seg1 - seg0;
    const double angle = gyro.norm() * dt;
    if (angle > 1e-12)
    {
      R_delta = R_delta * Eigen::AngleAxisd(angle, gyro.normalized()).toRotationMatrix();
    }
    last_t = seg1;
  }

  if (last_t < t1)
  {
    const Eigen::Vector3d gyro = imu_history_.back().gyro - bg_;
    const double dt = t1 - last_t;
    const double angle = gyro.norm() * dt;
    if (angle > 1e-12)
    {
      R_delta = R_delta * Eigen::AngleAxisd(angle, gyro.normalized()).toRotationMatrix();
    }
  }
  return R_delta;
}

// 标准 PointCloud2 入口：转换为 PCL XYZ 点云后进入统一处理流程。
void DogPriorMapEkfNode::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);
  handleLidarCloud(cloud, msg->header.stamp);
}

// LiDAR 主流程：预处理扫描、构建局部地图、执行配准并发布校正状态与诊断。
void DogPriorMapEkfNode::handleLidarCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_lidar, const ros::Time &stamp)
{
  if (!cloud_lidar || cloud_lidar->empty()) return;
  ++lidar_msg_count_;
  ++scan_count_;

  std::lock_guard<std::mutex> lock(mutex_);
  const ros::WallTime wall_start = ros::WallTime::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_body = preprocessScan(cloud_lidar);
  publishFilteredCloud(scan_body, stamp);
  if (static_cast<int>(scan_body->size()) < min_effective_points_)
  {
    ++lidar_update_fail_count_;
    publishDiagnostics(stamp, false, 0.0, static_cast<int>(scan_body->size()),
                       map_cloud_ ? static_cast<int>(map_cloud_->size()) : 0,
                       std::numeric_limits<double>::infinity());
    return;
  }

  int used = 0;
  double mean_residual = 0.0;
  const bool ndt_main_mode = registration_method_ == "ndt";
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map =
      (ndt_main_mode && ndt_absolute_pose_mode_ && ndt_use_full_map_target_) ? map_cloud_ : buildLocalSubmap();

  if (ndt_main_mode && local_map && static_cast<int>(local_map->size()) >= min_effective_points_)
  {
    const ros::WallTime ndt_start = ros::WallTime::now();
    const bool ndt_ok = runNdtRefinement(scan_body, local_map);
    const double update_ms = (ros::WallTime::now() - wall_start).toSec() * 1000.0;
    const double ndt_ms = (ros::WallTime::now() - ndt_start).toSec() * 1000.0;
    icp_update_time_sum_ms_ += ndt_ms;
    icp_update_time_max_ms_ = std::max(icp_update_time_max_ms_, ndt_ms);
    lidar_update_time_sum_ms_ += update_ms;
    lidar_update_time_max_ms_ = std::max(lidar_update_time_max_ms_, update_ms);
    last_used_points_ = static_cast<int>(scan_body->size());
    last_map_points_ = static_cast<int>(local_map->size());
    last_mean_residual_ = ndt_ok ? 0.0 : std::numeric_limits<double>::infinity();
    if (ndt_ok)
    {
      ++icp_update_ok_count_;
      ++lidar_update_ok_count_;
      publishState(stamp, true);
      if (print_debug_)
      {
        ROS_INFO_THROTTLE(1.0, "[DogPriorMap C++] NDT prior-map update: scan=%zu p=(%.2f %.2f %.2f)",
                          scan_body->size(), p_.x(), p_.y(), p_.z());
      }
    }
    else
    {
      ++icp_update_fail_count_;
      ++lidar_update_fail_count_;
    }
    publishDiagnostics(stamp, ndt_ok, ndt_ms, static_cast<int>(scan_body->size()),
                       static_cast<int>(local_map->size()), last_registration_score_);
    maybePrintRuntime(stamp);
    return;
  }

  const bool ok = lidarMapUpdate(scan_body, local_map, used, mean_residual);
  const double update_ms = (ros::WallTime::now() - wall_start).toSec() * 1000.0;
  lidar_update_time_sum_ms_ += update_ms;
  lidar_update_time_max_ms_ = std::max(lidar_update_time_max_ms_, update_ms);
  last_used_points_ = used;
  last_map_points_ = local_map ? static_cast<int>(local_map->size()) : 0;
  last_mean_residual_ = mean_residual;
  if (ok)
  {
    ++lidar_update_ok_count_;
    publishState(stamp, true);
    if (print_debug_)
    {
      ROS_INFO_THROTTLE(1.0, "[DogPriorMap C++] map match EKF update: used=%d mean_res=%.3f p=(%.2f %.2f %.2f)",
                        used, mean_residual, p_.x(), p_.y(), p_.z());
    }
  }
  else
  {
    ++lidar_update_fail_count_;
  }
  publishDiagnostics(stamp, ok, update_ms, static_cast<int>(scan_body->size()),
                     last_map_points_, mean_residual);
  maybePrintRuntime(stamp);
}

// 根据当前位置和自适应半径，从全局先验地图中截取本帧配准目标。
pcl::PointCloud<pcl::PointXYZ>::Ptr DogPriorMapEkfNode::buildLocalSubmap()
{
  // ------------------------- 局部子地图筛选 -------------------------
  // FASTLIO2Location类定位算法通常不会拿整张全局地图直接做配准，
  // 而是根据当前位姿预测，只取附近一块局部地图，减少重复走廊/相似墙面带来的错误最近邻。
  // 这里用全局KDTree半径搜索实现，低算力端可进一步换成体素哈希/ikd-tree。
  if (!local_submap_enable_ || local_radius_ <= 0.0)
  {
    return map_cloud_;
  }

  pcl::PointXYZ query(p_.x(), p_.y(), p_.z());
  std::vector<int> indices;
  std::vector<float> distances;
  double used_radius = local_radius_;
  for (double radius = local_radius_; radius <= local_radius_max_ + 1e-6; radius += local_radius_step_)
  {
    indices.clear();
    distances.clear();
    map_kdtree_->radiusSearch(query, radius, indices, distances, local_submap_max_points_);
    used_radius = radius;
    if (static_cast<int>(indices.size()) >= local_submap_min_points_ ||
        static_cast<int>(indices.size()) >= local_submap_max_points_)
    {
      break;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>());
  local->reserve(indices.size());
  for (int idx : indices)
  {
    local->push_back(map_cloud_->points[static_cast<size_t>(idx)]);
  }
  if (local->empty())
  {
    return local;
  }
  ROS_DEBUG_THROTTLE(1.0, "[DogPriorMap C++] local submap radius=%.2f pts=%zu",
                     used_radius, local->size());
  return local;
}

// 执行 NDT 精配准，并通过收敛、适应度和位姿跳变门限过滤错误匹配。
bool DogPriorMapEkfNode::runNdtRefinement(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_body,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map)
{
  // ------------------------- 局部子地图NDT精配准 -------------------------
  // NDT把目标地图划成体素高斯分布，当前帧点云在这些高斯分布上最大化概率。
  // 和GICP相比，NDT常用于实时先验地图定位：速度更快，对稀疏点云也比较稳。
  // 这里同样只做“小步预校正”，随后仍由EKF点到面残差做主更新。
  if (!scan_body || !local_map || scan_body->empty() || local_map->empty()) return false;

  // 局部降采样函数：压缩 NDT 源/目标点数，并在需要时做均匀抽样限长。
  auto voxelDown = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double voxel_size, int max_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
    if (voxel_size > 0.01)
    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
      voxel.setInputCloud(cloud);
      voxel.filter(*down);
    }
    else
    {
      *down = *cloud;
    }
    if (max_points > 0 && static_cast<int>(down->size()) > max_points)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
      sampled->reserve(max_points);
      const double step = static_cast<double>(down->size() - 1) / static_cast<double>(max_points - 1);
      for (int i = 0; i < max_points; ++i)
      {
        sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
      }
      return sampled;
    }
    return down;
  };

  const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud =
      (ndt_absolute_pose_mode_ && ndt_use_full_map_target_ && map_cloud_ && !map_cloud_->empty()) ? map_cloud_ : local_map;

  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
  if (ndt_absolute_pose_mode_)
  {
    source = voxelDown(scan_body, ndt_source_voxel_size_, ndt_max_source_points_);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_world(new pcl::PointCloud<pcl::PointXYZ>());
    scan_world->reserve(scan_body->size());
    for (const auto &pt : scan_body->points)
    {
      Eigen::Vector3d pb(pt.x, pt.y, pt.z);
      Eigen::Vector3d pw = R_ * pb + p_;
      if (pw.allFinite()) scan_world->push_back(pcl::PointXYZ(pw.x(), pw.y(), pw.z()));
    }
    source = voxelDown(scan_world, ndt_source_voxel_size_, ndt_max_source_points_);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr target = voxelDown(target_cloud, ndt_target_voxel_size_, ndt_max_target_points_);
  if (static_cast<int>(source->size()) < min_effective_points_ || static_cast<int>(target->size()) < min_effective_points_)
  {
    return false;
  }

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> local_ndt;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> *ndt_ptr = &local_ndt;
  if (ndt_absolute_pose_mode_ && ndt_use_full_map_target_)
  {
    if (!ndt_full_map_target_ready_)
    {
      ndt_full_map_.setInputTarget(target);
      ndt_full_map_.setResolution(ndt_resolution_);
      ndt_full_map_.setStepSize(ndt_step_size_);
      ndt_full_map_.setTransformationEpsilon(ndt_transformation_epsilon_);
      ndt_full_map_.setMaximumIterations(ndt_max_iterations_);
      ndt_full_map_target_ready_ = true;
      ROS_INFO("[DogPriorMap C++] NDT full-map target cached: target=%zu resolution=%.2f",
               target->size(), ndt_resolution_);
    }
    ndt_ptr = &ndt_full_map_;
  }
  else
  {
    local_ndt.setInputTarget(target);
    local_ndt.setResolution(ndt_resolution_);
    local_ndt.setStepSize(ndt_step_size_);
    local_ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    local_ndt.setMaximumIterations(ndt_max_iterations_);
  }
  auto &ndt = *ndt_ptr;
  ndt.setInputSource(source);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  Eigen::Matrix4d ndt_initial_guess = Eigen::Matrix4d::Identity();
  if (ndt_absolute_pose_mode_)
  {
    Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
    current_pose.block<3, 3>(0, 0) = R_;
    current_pose.block<3, 1>(0, 3) = p_;

    ndt_initial_guess = current_pose;
    if (ndt_has_previous_pose_)
    {
      ndt_initial_guess = ndt_previous_pose_ * ndt_delta_pose_;
    }

    ndt.align(aligned, ndt_initial_guess.cast<float>());
  }
  else
  {
    ndt.align(aligned);
  }
  if (!ndt.hasConverged())
  {
    ROS_WARN_THROTTLE(1.0, "[DogPriorMap C++] NDT reject: not converged source=%zu target=%zu",
                      source->size(), target->size());
    return false;
  }
  const double fitness = ndt.getFitnessScore();
  last_registration_score_ = fitness;
  if (!ndt_absolute_pose_mode_ && (!std::isfinite(fitness) || fitness > ndt_max_fitness_score_))
  {
    ROS_WARN_THROTTLE(1.0, "[DogPriorMap C++] NDT reject: fitness=%.4f max=%.4f source=%zu target=%zu",
                      fitness, ndt_max_fitness_score_, source->size(), target->size());
    return false;
  }

  Eigen::Matrix4d final_transform = ndt.getFinalTransformation().cast<double>();
  // In a corridor, NDT's longitudinal coordinate can be physically
  // unobservable.  Keep its observable y/z/attitude result, but let the
  // temporal fingerprint tracker arbitrate x using a short sequence and a
  // small set of scalar hypotheses.
  bool corridor_degenerate = lidar_degenerate_;
  if (ndt_absolute_pose_mode_ && corridor_sequence_enable_ && target && target->size() >= 20)
  {
    // NDT itself does not expose its Hessian.  A very elongated local map is
    // therefore used as a conservative proxy for a corridor's weak axial
    // information; the full point-to-plane path still uses the exact Fisher
    // matrix test below.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &pt : target->points) mean += Eigen::Vector3d(pt.x, pt.y, pt.z);
    mean /= static_cast<double>(target->size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &pt : target->points)
    {
      const Eigen::Vector3d d = Eigen::Vector3d(pt.x, pt.y, pt.z) - mean;
      covariance += d * d.transpose();
    }
    covariance /= static_cast<double>(target->size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
    if (eig.info() == Eigen::Success && eig.eigenvalues().maxCoeff() > 1e-6)
    {
      corridor_degenerate = corridor_degenerate ||
                            eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() < degeneracy_geometry_ratio_;
    }
  }
  if (ndt_absolute_pose_mode_ && corridor_sequence_localizer_.ready())
  {
    const Eigen::Matrix3d ndt_R = final_transform.block<3, 3>(0, 0);
    const Eigen::Vector3d ndt_p = final_transform.block<3, 1>(0, 3);
    const double temporal_x = corridor_sequence_localizer_.update(
        scan_body, ndt_R, ndt_p, p_.x(), corridor_degenerate);
    if (corridor_degenerate && std::isfinite(temporal_x))
    {
      final_transform(0, 3) = temporal_x;
      ROS_DEBUG_THROTTLE(1.0,
                         "[DogPriorMap C++] corridor temporal x=%.3f ndt_x=%.3f confidence=%.3f",
                         temporal_x, ndt_p.x(), corridor_sequence_localizer_.last_score());
    }
  }
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();
  Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
  const bool had_previous_ndt_pose = ndt_has_previous_pose_;
  const Eigen::Matrix4d previous_ndt_pose = ndt_previous_pose_;
  if (ndt_absolute_pose_mode_)
  {
    const Eigen::Matrix4d delta_from_guess = ndt_initial_guess.inverse() * final_transform;
    Eigen::AngleAxisd aa(delta_from_guess.block<3, 3>(0, 0));
    dp = delta_from_guess.block<3, 1>(0, 3);
    dtheta = aa.axis() * aa.angle();
  }
  else
  {
    Eigen::Matrix3d dR = final_transform.block<3, 3>(0, 0);
    Eigen::AngleAxisd aa(dR);
    dp = final_transform.block<3, 1>(0, 3);
    dtheta = aa.axis() * aa.angle();
  }
  if (!dp.allFinite() || !dtheta.allFinite()) return false;

  if (!ndt_absolute_pose_mode_)
  {
    dp = limitVector(dp, max_translation_update_);
    dtheta = limitVector(dtheta, max_rotation_update_);
  }
  else
  {
    if (ndt.getFinalNumIteration() <= 0)
    {
      ROS_WARN_THROTTLE(1.0, "[DogPriorMap C++] NDT reject: zero iteration fitness=%.4f", fitness);
      return false;
    }
    if (!std::isfinite(fitness))
    {
      ROS_WARN_THROTTLE(1.0, "[DogPriorMap C++] NDT reject: non-finite fitness source=%zu target=%zu",
                        source->size(), target->size());
      return false;
    }

    if (had_previous_ndt_pose)
    {
      const Eigen::Matrix4d delta_from_previous = previous_ndt_pose.inverse() * final_transform;
      Eigen::Vector3d prev_dp = delta_from_previous.block<3, 1>(0, 3);
      Eigen::Matrix3d prev_dR = delta_from_previous.block<3, 3>(0, 0);
      Eigen::AngleAxisd prev_aa(prev_dR);
      Eigen::Vector3d prev_dtheta = prev_aa.axis() * prev_aa.angle();

      const double jump_trans = prev_dp.norm();
      const double jump_rot_deg = prev_dtheta.norm() * 180.0 / M_PI;
      const bool large_jump = jump_trans > ndt_accept_max_translation_ ||
                              jump_rot_deg > ndt_accept_max_rotation_ * 180.0 / M_PI;

      if (large_jump)
      {
        ROS_WARN_THROTTLE(1.0,
                          "[DogPriorMap C++] NDT reject: full-map jump prev_dp=%.3f max=%.3f prev_rot_deg=%.2f max=%.2f fitness=%.4f",
                          jump_trans, ndt_accept_max_translation_, jump_rot_deg,
                          ndt_accept_max_rotation_ * 180.0 / M_PI, fitness);
        return false;
      }
    }
  }

  if (ndt_absolute_pose_mode_)
  {
    if (had_previous_ndt_pose)
    {
      ndt_delta_pose_ = previous_ndt_pose.inverse() * final_transform;
    }
    else
    {
      ndt_delta_pose_.setIdentity();
      ndt_has_previous_pose_ = true;
    }
    ndt_previous_pose_ = final_transform;
    R_ = final_transform.block<3, 3>(0, 0);
    p_ = final_transform.block<3, 1>(0, 3);
    v_.setZero();
  }
  else
  {
    applyPoseCorrection(dp, dtheta);
  }
  return true;
}

// 将原始雷达点转换到机体系，并执行距离、高度、体素与半径离群滤波。
pcl::PointCloud<pcl::PointXYZ>::Ptr DogPriorMapEkfNode::preprocessScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_lidar)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>());
  body->reserve(cloud_lidar->size());

  // 雷达点先由livox_frame转到机器狗base_link。现在配置里默认近似重合；
  // 后续把雷达安装到机器狗上，需要把真实外参填进yaml。
  for (const auto &pt : cloud_lidar->points)
  {
    Eigen::Vector3d pl(pt.x, pt.y, pt.z);
    if (!pl.allFinite()) continue;
    Eigen::Vector3d pb = R_base_lidar_ * pl + T_base_lidar_;
    const double d = pb.norm();
    if (d > scan_min_range_ && d < scan_max_range_ && pb.z() >= scan_min_z_ && pb.z() <= scan_max_z_)
    {
      body->push_back(pcl::PointXYZ(pb.x(), pb.y(), pb.z()));
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
  if (scan_voxel_size_ > 0.01)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
    voxel.setInputCloud(body);
    voxel.filter(*down);
  }
  else
  {
    *down = *body;
  }

  if (scan_radius_outlier_enable_ &&
      static_cast<int>(down->size()) > scan_radius_outlier_min_neighbors_ &&
      scan_radius_outlier_radius_ > 0.01)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(down);
    radius_filter.setRadiusSearch(scan_radius_outlier_radius_);
    radius_filter.setMinNeighborsInRadius(scan_radius_outlier_min_neighbors_);
    radius_filter.filter(*filtered);
    down = filtered;
  }

  // 低算力限制：每帧最多只拿固定数量点参与匹配。
  if (static_cast<int>(down->size()) > max_scan_points_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
    sampled->reserve(max_scan_points_);
    const double step = static_cast<double>(down->size() - 1) / static_cast<double>(max_scan_points_ - 1);
    for (int i = 0; i < max_scan_points_; ++i)
    {
      sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
    }
    return sampled;
  }
  return down;
}

// 构建点到点/点到平面最小二乘问题，迭代求解并审核六自由度状态修正。
bool DogPriorMapEkfNode::lidarMapUpdate(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_body,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &match_map,
                                        int &used,
                                        double &mean_residual)
{
  // ------------------------- LiDAR-先验地图低频更新 -------------------------
  // 观测模型：当前帧点云经过当前位姿投到地图坐标系后，应当落在先验地图附近。
  // 支持两种模式：
  // 1) point_to_point：残差为最近邻地图点 - 当前投影点，速度快但走廊/墙面上约束较粗。
  // 2) point_to_plane：用地图近邻PCA拟合局部平面，残差为点到平面的有符号距离，精度通常更好。
  used = 0;
  mean_residual = 0.0;
  if (!match_map || static_cast<int>(match_map->size()) < min_effective_points_) return false;

  pcl::KdTreeFLANN<pcl::PointXYZ> local_kdtree;
  local_kdtree.setInputCloud(match_map);

  // 复算候选位姿的最近邻平均残差，为更新接收门控提供统一评分。
  auto scorePoseByNearestMap = [&](const Eigen::Vector3d &pose_p,
                                   const Eigen::Matrix3d &pose_R,
                                   int &score_used) -> double {
    score_used = 0;
    double score_sum = 0.0;
    std::vector<int> score_idx(1);
    std::vector<float> score_dist2(1);
    for (const auto &pt : scan_body->points)
    {
      const Eigen::Vector3d pb(pt.x, pt.y, pt.z);
      const Eigen::Vector3d pred = pose_R * pb + pose_p;
      if (!pred.allFinite()) continue;
      if (local_radius_ > 0.0 && (pred - pose_p).norm() > local_radius_) continue;
      const pcl::PointXYZ query(pred.x(), pred.y(), pred.z());
      if (local_kdtree.nearestKSearch(query, 1, score_idx, score_dist2) <= 0) continue;
      const double dist = std::sqrt(static_cast<double>(score_dist2[0]));
      if (dist > max_match_distance_) continue;
      score_sum += std::min(dist, huber_threshold_);
      ++score_used;
    }
    return score_used > 0 ? score_sum / static_cast<double>(score_used)
                          : std::numeric_limits<double>::infinity();
  };

  const ros::WallTime update_start = ros::WallTime::now();
  bool accepted_any_update = false;
  for (int iter = 0; iter < max_iterations_; ++iter)
  {
    if (max_update_time_ms_ > 0.0 &&
        (ros::WallTime::now() - update_start).toSec() * 1000.0 > max_update_time_ms_)
    {
      break;
    }
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    int effective = 0;
    double residual_sum = 0.0;
    Eigen::Vector3d matched_centroid = Eigen::Vector3d::Zero();
    Eigen::Matrix3d matched_second_moment = Eigen::Matrix3d::Zero();

    for (const auto &pt : scan_body->points)
    {
      Eigen::Vector3d pb(pt.x, pt.y, pt.z);
      Eigen::Vector3d pred = R_ * pb + p_;
      pcl::PointXYZ query(pred.x(), pred.y(), pred.z());

      const bool use_fastlio_plane = registration_method_ == "fastlio_plane" || registration_method_ == "fastlio_hybrid";
      const bool use_plane = use_fastlio_plane || registration_method_ == "point_to_plane" || registration_method_ == "hybrid";
      const int search_k = use_plane ? plane_neighbor_k_ : 1;
      std::vector<int> idx(search_k);
      std::vector<float> dist2(search_k);
      if (local_kdtree.nearestKSearch(query, search_k, idx, dist2) <= 0) continue;
      const double dist = std::sqrt(static_cast<double>(dist2[0]));
      if (dist > max_match_distance_) continue;
      if (local_radius_ > 0.0 && (pred - p_).norm() > local_radius_) continue;

      if (use_plane)
      {
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        double residual = 0.0;
        double residual_variance = plane_noise_ * plane_noise_;
        bool plane_valid = false;

        if (use_fastlio_plane)
        {
          // ------------------------- FAST-LIO式近邻平面拟合 -------------------------
          // 原版FAST-LIO/FAST-LIO-Localization不是PCL ICP，而是：
          // 1) 对每个当前点找地图中5个近邻；
          // 2) 解 Ax=-1 得到局部平面 n^T x + 1 = 0；
          // 3) 检查5个近邻是否都离平面足够近；
          // 4) 用 s = 1 - 0.9 * |点到面距离| / sqrt(点距离) 做残差门控。
          // 这比简单最近邻点到点更能利用墙面/地面法向约束，也比整帧ICP更不容易一次性拉飞。
          Eigen::MatrixXd plane_A(search_k, 3);
          Eigen::VectorXd plane_b(search_k);
          plane_b.setConstant(-1.0);
          for (int j = 0; j < search_k; ++j)
          {
            const auto &mp = match_map->points[idx[j]];
            plane_A(j, 0) = mp.x;
            plane_A(j, 1) = mp.y;
            plane_A(j, 2) = mp.z;
          }

          Eigen::Vector3d n_scaled = plane_A.colPivHouseholderQr().solve(plane_b);
          const double norm = n_scaled.norm();
          if (norm > 1e-9 && n_scaled.allFinite())
          {
            normal = n_scaled / norm;
            const double d = 1.0 / norm;
            plane_valid = true;
            for (int j = 0; j < search_k; ++j)
            {
              const auto &mp = match_map->points[idx[j]];
              const double pd = normal.x() * mp.x + normal.y() * mp.y + normal.z() * mp.z + d;
              if (std::abs(pd) > fastlio_plane_threshold_)
              {
                plane_valid = false;
                break;
              }
            }
            residual = -(normal.dot(pred) + d);
            const double score = 1.0 - 0.9 * std::abs(residual) / std::sqrt(std::max(pb.norm(), 1e-3));
            if (score <= fastlio_residual_gate_) plane_valid = false;
          }
        }
        else
        {
          Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
          for (int j = 0; j < search_k; ++j)
          {
            const auto &mp = match_map->points[idx[j]];
            centroid += Eigen::Vector3d(mp.x, mp.y, mp.z);
          }
          centroid /= static_cast<double>(search_k);

          Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
          for (int j = 0; j < search_k; ++j)
          {
            const auto &mp = match_map->points[idx[j]];
            Eigen::Vector3d d = Eigen::Vector3d(mp.x, mp.y, mp.z) - centroid;
            cov += d * d.transpose();
          }
          cov /= static_cast<double>(search_k);

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
          if (solver.info() != Eigen::Success) continue;
          const Eigen::Vector3d eval = solver.eigenvalues();
          const double denom = std::max(eval.sum(), 1e-12);
          const double plane_ratio = eval[0] / denom;
          if (plane_ratio > plane_min_eigen_ratio_) continue;

          normal = solver.eigenvectors().col(0).normalized();
          residual = normal.dot(centroid - pred);
          const double abs_residual = std::abs(residual);
          if (fastlivo_outlier_reject_enable_)
          {
            // ------------------------- FAST-LIVO2式外点剔除 -------------------------
            // FAST-LIVO2在体素平面里不是只看最近邻距离，还会判断：
            // 1) 当前点在平面切向范围内，避免拿同一无限大墙面的远处点错误匹配；
            // 2) 点到面距离小于平面/测距不确定度给出的sigma门限；
            // 3) 根据高斯概率给残差加权，越可信的平面对更新贡献越大。
            // 这里没有维护完整体素平面协方差，因此用近邻PCA特征值构造低算力近似。
            const Eigen::Vector3d pred_to_center = pred - centroid;
            const double tangent_sq = std::max(0.0, pred_to_center.squaredNorm() - abs_residual * abs_residual);
            const double tangent_dist = std::sqrt(tangent_sq);
            const double plane_radius = std::max(std::sqrt(std::max(eval[2], 0.0)), plane_min_radius_);
            if (tangent_dist > plane_radius_gate_scale_ * plane_radius) continue;

            const double range = std::max(pb.norm(), 1e-3);
            const double range_sigma = range_noise_per_meter_ * range;
            double body_normal_variance = range_sigma * range_sigma;
            if (fastlivo_body_cov_enable_)
            {
              const Eigen::Matrix3d body_cov =
                  calcFastLivoBodyCov(pb, lidar_depth_noise_, lidar_beam_noise_deg_);
              const double projected_body_variance =
                  (normal.transpose() * R_ * body_cov * R_.transpose() * normal)(0, 0);
              body_normal_variance =
                  std::max(body_normal_variance, projected_body_variance);
            }
            residual_variance = std::max(plane_min_sigma_ * plane_min_sigma_,
                                         std::max(eval[0], 0.0) + body_normal_variance);
            if (abs_residual > plane_sigma_gate_ * std::sqrt(residual_variance)) continue;
          }
          plane_valid = true;
        }
        if (!plane_valid) continue;
        const double abs_residual = std::abs(residual);

        double weight = 1.0;
        if (fastlivo_outlier_reject_enable_)
        {
          const double sigma = std::sqrt(std::max(residual_variance, 1e-9));
          const double probability_weight = std::exp(-0.5 * abs_residual * abs_residual / std::max(residual_variance, 1e-9)) /
                                            std::max(sigma, 1e-3);
          weight *= std::max(0.25, std::min(1.0, probability_weight * plane_min_sigma_));
        }
        if (abs_residual > huber_threshold_)
        {
          weight *= huber_threshold_ / std::max(abs_residual, 1e-6);
        }
        // LiDAR-先验地图匹配是定位主约束，权重只由雷达残差/鲁棒核决定。
        // 相机曝光质量不在这里削弱雷达匹配；视觉约束会在自己的观测更新里单独调权。
        if (horizontal_plane_z_observation_enable_ &&
            std::abs(normal.z()) >= horizontal_plane_normal_z_min_ &&
            abs_residual <= horizontal_plane_residual_gate_)
        {
          // ------------------------- 地图水平面Z观测增强 -------------------------
          // 这里不是把Z轴固定，也不是给Z加阻尼；它仍然来自真实地图匹配观测。
          // 当当前点匹配到地面/天花板这类法向接近Z轴的局部平面时，点到面残差
          // 对高度最敏感，因此适当提高该观测权重，用地图里的水平结构抑制Z漂移。
          // 如果机器狗真的上楼/下楼，只要先验地图里对应楼梯/楼层高度存在，残差会把
          // 位姿拉到正确高度，而不是压回初始高度。
          weight *= horizontal_plane_weight_scale_;
        }

        Eigen::Matrix<double, 1, 6> J;
        J.block<1, 3>(0, 0) = normal.transpose();
        J.block<1, 3>(0, 3) = -normal.transpose() * R_ * skew(pb);

        const double inv_var = weight / std::max(residual_variance + plane_noise_ * plane_noise_, 1e-6);
        A += J.transpose() * inv_var * J;
        b += J.transpose() * inv_var * residual;
        residual_sum += abs_residual;
        ++effective;
        matched_centroid += pred;
        matched_second_moment += pred * pred.transpose();

        if (registration_method_ == "hybrid" || registration_method_ == "fastlio_hybrid")
        {
          // 混合模式的关键：点到面只约束法向，墙面/地面切向容易滑。
          // 这里再加一个很弱的点到点约束，权重比纯点到点小很多，只负责防止切向无约束。
          const auto &target_pt = match_map->points[idx[0]];
          Eigen::Vector3d target(target_pt.x, target_pt.y, target_pt.z);
          Eigen::Vector3d r3 = target - pred;
          const double rn = r3.norm();
          double w3 = 0.35;
          if (rn > huber_threshold_)
          {
            w3 *= huber_threshold_ / std::max(rn, 1e-6);
          }
          // hybrid里的点到点弱约束仍然属于LiDAR地图匹配，不乘相机权重。
          Eigen::Matrix<double, 3, 6> J3;
          J3.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
          J3.block<3, 3>(0, 3) = -R_ * skew(pb);
          const double inv_var3 = w3 / std::max(hybrid_point_noise_ * hybrid_point_noise_, 1e-6);
          A += J3.transpose() * inv_var3 * J3;
          b += J3.transpose() * inv_var3 * r3;
        }
      }
      else
      {
        const auto &target_pt = match_map->points[idx[0]];
        Eigen::Vector3d target(target_pt.x, target_pt.y, target_pt.z);
        Eigen::Vector3d r = target - pred;
        const double rn = r.norm();

        // Huber鲁棒权重：动态物体、错误最近邻会被降权，减少被错误地图点拉飞。
        double weight = 1.0;
        if (rn > huber_threshold_)
        {
          weight = huber_threshold_ / std::max(rn, 1e-6);
        }
        // LiDAR-先验地图匹配是定位主约束，权重只由雷达残差/鲁棒核决定。
        // 如果检测到LiDAR退化，只提高后续视觉约束权重，不降低这里的雷达匹配权重。

        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J.block<3, 3>(0, 3) = -R_ * skew(pb);

        const double inv_var = weight / std::max(point_noise_ * point_noise_, 1e-6);
        A += J.transpose() * inv_var * J;
        b += J.transpose() * inv_var * r;
        residual_sum += rn;
        ++effective;
        matched_centroid += pred;
        matched_second_moment += pred * pred.transpose();
      }
    }

    used = effective;
    mean_residual = effective > 0 ? residual_sum / static_cast<double>(effective) : 0.0;
    if (effective < min_effective_points_) return false;
    double geometry_degeneracy_score = 0.0;
    if (effective > 3)
    {
      // ------------------------- 匹配点几何分布退化 -------------------------
      // 长走廊里点云常常主要分布在狭长方向或少数大平面上。
      // 即使point-to-point的H矩阵看起来可解，空间分布本身也会导致沿走廊/高度方向容易滑移。
      // 因此额外统计匹配点在地图坐标系下的PCA特征值：
      // λ_min / λ_max 越小，说明点云越接近线/面退化，视觉约束越应该被启用。
      matched_centroid /= static_cast<double>(effective);
      Eigen::Matrix3d cov = matched_second_moment / static_cast<double>(effective) -
                            matched_centroid * matched_centroid.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> geo_solver(cov);
      if (geo_solver.info() == Eigen::Success)
      {
        const Eigen::Vector3d eval = geo_solver.eigenvalues();
        const double ratio = std::max(0.0, eval[0]) / std::max(std::max(0.0, eval[2]), 1e-12);
        if (ratio < degeneracy_geometry_ratio_)
        {
          geometry_degeneracy_score =
              std::min(1.0, (degeneracy_geometry_ratio_ - ratio) / std::max(degeneracy_geometry_ratio_, 1e-12));
        }
      }
    }
    const Eigen::Matrix<double, 6, 6> information_before_damping = A;
    lidar_degenerate_ = updateLidarDegeneracyStatus(information_before_damping, geometry_degeneracy_score);

    // 加一个很小的阻尼，让走廊退化时矩阵不至于数值发散。
    A += Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
    Eigen::Matrix<double, 6, 1> dx6 = A.ldlt().solve(b);
    if (!dx6.allFinite()) return false;
    if (degeneracy_project_update_enable_ && lidar_degenerate_)
    {
      // ------------------------- 退化方向投影更新 -------------------------
      // FAST-LIVO2的稳定性来自“IMU传播状态 + LiDAR/VIO残差一起进入ESKF”：
      // 当长走廊等场景导致LiDAR信息矩阵某些特征值很小时，这些方向本来就应该
      // 更多相信IMU/视觉传播，而不是让错误最近邻给出大幅LiDAR修正。
      // 这里对6维位姿增量的信息矩阵做特征分解，在信息弱的特征方向上缩小LiDAR更新。
      // 注意这不是固定抑制Z或旋转；退化方向由当前帧H^T R^-1 H自动决定。
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> info_solver(information_before_damping);
      if (info_solver.info() == Eigen::Success)
      {
        const Eigen::Matrix<double, 6, 1> eval = info_solver.eigenvalues().cwiseMax(0.0);
        const Eigen::Matrix<double, 6, 6> evec = info_solver.eigenvectors();
        const double max_eval = std::max(eval.maxCoeff(), 1e-12);
        const double ratio_threshold = std::max(degeneracy_project_eigen_ratio_, 1e-6);
        const double min_scale = std::max(0.0, std::min(1.0, degeneracy_project_min_scale_));
        Eigen::Matrix<double, 6, 1> dx_eigen = evec.transpose() * dx6;
        for (int k = 0; k < 6; ++k)
        {
          const double ratio = eval(k) / max_eval;
          if (ratio < ratio_threshold)
          {
            const double scale = min_scale + (1.0 - min_scale) * std::max(0.0, ratio / ratio_threshold);
            dx_eigen(k) *= scale;
          }
        }
        dx6 = evec * dx_eigen;
      }
    }

    Eigen::Vector3d dp = limitVector(dx6.head<3>(), max_translation_update_);
    Eigen::Vector3d dtheta = limitVector(dx6.tail<3>(), max_rotation_update_);
    bool accept_update = true;
    double candidate_mean_residual = mean_residual;
    int candidate_used = effective;
    if (match_accept_gate_enable_)
    {
      if (match_accept_max_translation_ > 0.0)
      {
        dp = limitVector(dp, match_accept_max_translation_);
      }
      if (match_accept_max_rotation_ > 0.0)
      {
        dtheta = limitVector(dtheta, match_accept_max_rotation_);
      }

      const double angle = dtheta.norm();
      Eigen::Matrix3d candidate_R = R_;
      if (angle > 1e-12)
      {
        candidate_R = Eigen::AngleAxisd(angle, dtheta / angle).toRotationMatrix() * R_;
      }
      const Eigen::Vector3d candidate_p = p_ + dp;
      int current_used = 0;
      const double current_mean_residual = scorePoseByNearestMap(p_, R_, current_used);
      candidate_mean_residual = scorePoseByNearestMap(candidate_p, candidate_R, candidate_used);
      const int min_accept_points = std::max(min_effective_points_,
                                             static_cast<int>(std::ceil(match_accept_min_effective_ratio_ *
                                                                       static_cast<double>(scan_body->size()))));

      if (candidate_used < min_accept_points)
      {
        accept_update = false;
      }
      if (std::isfinite(match_accept_max_mean_residual_) &&
          match_accept_max_mean_residual_ > 0.0 &&
          candidate_mean_residual > match_accept_max_mean_residual_)
      {
        accept_update = false;
      }
      if (std::isfinite(current_mean_residual) && std::isfinite(candidate_mean_residual))
      {
        const double max_allowed =
            current_mean_residual * std::max(1.0, match_accept_max_residual_increase_ratio_);
        const double desired =
            current_mean_residual * std::max(0.0, 1.0 - match_accept_min_improvement_ratio_);
        if (candidate_mean_residual > max_allowed || candidate_mean_residual > desired)
        {
          accept_update = false;
        }
      }
      if (lidar_degenerate_)
      {
        if (match_accept_degenerate_max_translation_ > 0.0 &&
            dp.norm() > match_accept_degenerate_max_translation_)
        {
          accept_update = false;
        }
        if (match_accept_degenerate_max_rotation_ > 0.0 &&
            dtheta.norm() > match_accept_degenerate_max_rotation_)
        {
          accept_update = false;
        }
      }

      if (!accept_update)
      {
        if (print_debug_)
        {
          ROS_WARN_THROTTLE(1.0,
                            "[DogPriorMap C++] 拒绝可疑地图匹配: iter=%d used=%d/%d mean=%.3f cand_used=%d cand_mean=%.3f dp=%.3f dtheta=%.3fdeg degen=%.2f",
                            iter, effective, min_accept_points, mean_residual, candidate_used,
                            candidate_mean_residual, dp.norm(), dtheta.norm() * 180.0 / M_PI,
                            lidar_degeneracy_score_);
        }
        break;
      }
    }

    mean_residual = candidate_mean_residual;
    used = candidate_used;
    applyPoseCorrection(dp, dtheta);
    accepted_any_update = true;

    // 简化更新协方差：把6自由度匹配结果反馈到15维状态中。
    // 工程部署时可以进一步改成完整H/K矩阵；当前版本更省内存、实时性更好。
    for (int i = 0; i < 3; ++i) P_(i, i) = std::max(P_(i, i) * 0.8, 1e-4);
    for (int i = 6; i < 9; ++i) P_(i, i) = std::max(P_(i, i) * 0.8, 1e-5);

    if (dp.norm() < 0.01 && dtheta.norm() < 0.2 * M_PI / 180.0) break;
  }
  return accepted_any_update;
}

// 由信息矩阵特征值和匹配点几何分布判断当前 LiDAR 约束是否退化。
bool DogPriorMapEkfNode::updateLidarDegeneracyStatus(const Eigen::Matrix<double, 6, 6> &information_matrix,
                                                     double geometry_degeneracy_score)
{
  // ------------------------- LiDAR退化检测 -------------------------
  // A = H^T R^-1 H 可以近似看成这帧LiDAR地图匹配对6自由度位姿的“信息量”。
  // 长走廊、重复墙面、单一大平面等场景会让某些自由度约束很弱，表现为：
  // 1) 最小特征值很小：至少一个方向几乎不可观。
  // 2) 条件数很大：强约束和弱约束方向差异过大，容易沿弱约束方向滑移。
  // 检测到退化后，不削弱LiDAR主约束，而是通知视觉模块：如果图像质量好，后续视觉残差可以适当增权。
  lidar_degeneracy_score_ = 0.0;
  if (!degeneracy_check_enable_) return false;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information_matrix);
  if (solver.info() != Eigen::Success) return false;

  const Eigen::Matrix<double, 6, 1> eval = solver.eigenvalues();
  const double min_eval = std::max(eval[0], 0.0);
  const double max_eval = std::max(eval[5], 1e-12);
  const double condition_number = max_eval / std::max(min_eval, 1e-12);

  const double eigen_score =
      min_eval < degeneracy_min_eigenvalue_
          ? std::min(1.0, (degeneracy_min_eigenvalue_ - min_eval) / std::max(degeneracy_min_eigenvalue_, 1e-12))
          : 0.0;
  const double condition_score =
      condition_number > degeneracy_max_condition_number_
          ? std::min(1.0, std::log(condition_number / degeneracy_max_condition_number_) / std::log(100.0))
          : 0.0;

  lidar_degeneracy_score_ = std::max(std::max(eigen_score, condition_score), geometry_degeneracy_score);
  return lidar_degeneracy_score_ > 0.0;
}

// 将通过门控的位置和旋转增量反馈到当前状态估计。
void DogPriorMapEkfNode::applyPoseCorrection(const Eigen::Vector3d &dp, const Eigen::Vector3d &dtheta)
{
  p_ += dp;
  const double angle = dtheta.norm();
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  if (angle > 1e-12)
  {
    dR = Eigen::AngleAxisd(angle, dtheta / angle).toRotationMatrix();
  }
  R_ = dR * R_;
}

}  // namespace dog_prior_map_localization
