#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

namespace dog_prior_map_localization
{
namespace
{
void finalizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Eigen::Matrix4d poseToMatrix(const Eigen::Vector3d &p, const Eigen::Matrix3d &R)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

Eigen::Matrix3d rpyDegToRot(const std::vector<double> &rpy_deg)
{
  const double roll = rpy_deg[0] * M_PI / 180.0;
  const double pitch = rpy_deg[1] * M_PI / 180.0;
  const double yaw = rpy_deg[2] * M_PI / 180.0;
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Matrix3d yawToRot(double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double wrapPi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double yawFromRot(const Eigen::Matrix3d &R)
{
  return std::atan2(R(1, 0), R(0, 0));
}

double rotationAngleDeg(const Eigen::Matrix3d &R)
{
  Eigen::AngleAxisd angle_axis(R);
  return std::abs(angle_axis.angle()) * 180.0 / M_PI;
}

}  // namespace

class DogPriorMapNdtNode
{
public:
  DogPriorMapNdtNode() : nh_(), pnh_("~")
  {
    map_frame_ = getParam<std::string>("frames/map_frame", "camera_init");
    base_frame_ = getParam<std::string>("frames/base_frame", "livox_frame");
    lidar_topic_ = getParam<std::string>("topics/lidar", "/livox/lidar");
    lidar_msg_type_ = getParam<std::string>("topics/lidar_msg_type", "livox");
    filtered_points_topic_ = getParam<std::string>("topics/filtered_points", "/dog_livo/filtered_points");
    diagnostics_topic_ = getParam<std::string>("topics/diagnostics", "/dog_livo/diagnostics");
    ndt_odom_topic_ = getParam<std::string>("topics/ndt_odom", "/dog_livo/ndt_odom");
    ndt_pose_topic_ = getParam<std::string>("topics/ndt_pose", "/dog_livo/ndt_pose");
    ndt_path_topic_ = getParam<std::string>("topics/ndt_path", "/dog_livo/ndt_path");
    points_aligned_topic_ = getParam<std::string>("topics/points_aligned", "/dog_livo/points_aligned");
    initial_guess_odom_topic_ =
        getParam<std::string>("topics/ndt_initial_guess_odom", "/dog_livo/odom_high_rate");

    map_pcd_path_ = getParam<std::string>("map/pcd_fallback_path", "");
    map_voxel_size_ = getParam<double>("map/voxel_size", 0.30);
    scan_voxel_size_ = getParam<double>("lidar_update/ndt_source_voxel_size", 0.35);
    max_scan_points_ = getParam<int>("lidar_update/ndt_max_source_points", 900);
    target_voxel_size_ = getParam<double>("lidar_update/ndt_target_voxel_size", 0.30);
    max_target_points_ = getParam<int>("lidar_update/ndt_max_target_points", 0);
    min_range_ = getParam<double>("lidar_update/scan_min_range", 0.5);
    max_range_ = getParam<double>("lidar_update/scan_max_range", 80.0);
    min_z_ = getParam<double>("lidar_update/scan_min_z", -std::numeric_limits<double>::infinity());
    max_z_ = getParam<double>("lidar_update/scan_max_z", std::numeric_limits<double>::infinity());
    min_effective_points_ = getParam<int>("lidar_update/min_effective_points", 50);
    ndt_resolution_ = getParam<double>("lidar_update/ndt_resolution", 1.0);
    ndt_step_size_ = getParam<double>("lidar_update/ndt_step_size", 0.1);
    ndt_transformation_epsilon_ = getParam<double>("lidar_update/ndt_transformation_epsilon", 0.001);
    ndt_max_iterations_ = getParam<int>("lidar_update/ndt_max_iterations", 30);
    ndt_acceptance_enable_ = getParam<bool>("lidar_update/ndt_acceptance_enable", false);
    ndt_ambiguity_check_enable_ = getParam<bool>("lidar_update/ndt_ambiguity_check_enable", false);
    ndt_ambiguity_period_sec_ = getParam<double>("lidar_update/ndt_ambiguity_period_sec", 1.0);
    ndt_ambiguity_search_radius_ = getParam<double>("lidar_update/ndt_ambiguity_search_radius", 2.0);
    ndt_ambiguity_search_step_ = getParam<double>("lidar_update/ndt_ambiguity_search_step", 1.0);
    ndt_ambiguity_yaw_search_deg_ = getParam<double>("lidar_update/ndt_ambiguity_yaw_search_deg", 8.0);
    ndt_ambiguity_yaw_step_deg_ = getParam<double>("lidar_update/ndt_ambiguity_yaw_step_deg", 4.0);
    ndt_ambiguity_max_points_ = getParam<int>("lidar_update/ndt_ambiguity_max_points", 300);
    ndt_ambiguity_near_best_ratio_ = getParam<double>("lidar_update/ndt_ambiguity_near_best_ratio", 1.25);
    ndt_ambiguity_near_best_margin_ = getParam<double>("lidar_update/ndt_ambiguity_near_best_margin", 0.03);
    ndt_max_fitness_score_ = getParam<double>("lidar_update/ndt_max_fitness_score", 3.0);
    ndt_reject_zero_iteration_ = getParam<bool>("lidar_update/ndt_reject_zero_iteration", true);
    ndt_accept_max_translation_ = getParam<double>("lidar_update/ndt_accept_max_translation", 1.20);
    ndt_accept_max_rotation_deg_ = getParam<double>("lidar_update/ndt_accept_max_rotation_deg", 12.0);
    use_external_initial_guess_ = getParam<bool>("lidar_update/ndt_use_external_initial_guess", false);
    initial_guess_max_age_sec_ = getParam<double>("lidar_update/ndt_initial_guess_max_age_sec", 0.20);
    initial_guess_blend_ = getParam<double>("lidar_update/ndt_initial_guess_blend", 1.0);
    loop_relocalization_enable_ = getParam<bool>("loop_relocalization/enable", false);
    loop_pose_path_ = getParam<std::string>("loop_relocalization/pose_path", "");
    loop_scan_dir_ = getParam<std::string>("loop_relocalization/scan_dir", "");
    loop_query_period_sec_ = getParam<double>("loop_relocalization/query_period_sec", 1.0);
    loop_keyframe_spacing_m_ = getParam<double>("loop_relocalization/keyframe_spacing_m", 1.0);
    loop_prior_submap_half_width_ = getParam<int>("loop_relocalization/prior_submap_half_width", 3);
    loop_context_radius_ = getParam<double>("loop_relocalization/context_radius", 20.0);
    loop_context_max_points_ = getParam<int>("loop_relocalization/context_max_points", 2500);
    loop_num_rings_ = getParam<int>("loop_relocalization/num_rings", 20);
    loop_num_sectors_ = getParam<int>("loop_relocalization/num_sectors", 60);
    loop_max_candidates_ = getParam<int>("loop_relocalization/max_candidates", 5);
    loop_min_similarity_ = getParam<double>("loop_relocalization/min_similarity", 0.15);
    loop_accept_max_fitness_score_ = getParam<double>("loop_relocalization/accept_max_fitness_score", 1.0);
    loop_accept_max_translation_ = getParam<double>("loop_relocalization/accept_max_translation", 8.0);
    loop_accept_max_yaw_deg_ = getParam<double>("loop_relocalization/accept_max_yaw_deg", 25.0);
    loop_require_consecutive_accepts_ = getParam<int>("loop_relocalization/require_consecutive_accepts", 2);
    loop_consecutive_candidate_radius_ =
        getParam<double>("loop_relocalization/consecutive_candidate_radius", 3.0);
    loop_apply_ratio_ = getParam<double>("loop_relocalization/apply_ratio", 0.5);
    loop_max_position_correction_ = getParam<double>("loop_relocalization/max_position_correction", 1.0);
    loop_max_yaw_correction_deg_ = getParam<double>("loop_relocalization/max_yaw_correction_deg", 5.0);
    publish_tf_ = getParam<bool>("output/ndt_publish_tf", false);
    publish_path_ = getParam<bool>("output/publish_path", false);
    publish_filtered_points_ = getParam<bool>("output/publish_filtered_points", true);
    publish_diagnostics_ = getParam<bool>("output/publish_diagnostics", true);
    ndt_diagnostics_csv_path_ = getParam<std::string>("output/ndt_diagnostics_csv_path", "");
    if (!ndt_diagnostics_csv_path_.empty())
    {
      ndt_diagnostics_csv_.open(ndt_diagnostics_csv_path_, std::ios::out);
      if (ndt_diagnostics_csv_)
      {
        ndt_diagnostics_csv_
            << "stamp,converged,accepted,reject_reason,align_ms,scan_points,map_points,fitness_score,iterations,"
            << "initial_x,initial_y,initial_z,result_x,result_y,result_z,used_x,used_y,used_z,"
            << "initial_to_result_translation,previous_to_result_translation,previous_to_result_rotation_deg,"
            << "previous_to_used_translation,previous_to_used_rotation_deg,"
            << "ambiguity_checked,ambiguity_best_mean_residual,ambiguity_current_mean_residual,"
            << "ambiguity_near_best_count,ambiguity_best_dx,ambiguity_best_dy,ambiguity_best_yaw_deg\n";
      }
      else
      {
        ROS_WARN("[DogPriorMap NDT] failed to open ndt diagnostics CSV: %s",
                 ndt_diagnostics_csv_path_.c_str());
      }
    }

    const std::vector<double> init_p = getParamVec("filter/initial_position", {0.0, 0.0, 0.0});
    const std::vector<double> init_rpy = getParamVec("filter/initial_rpy_deg", {0.0, 0.0, 0.0});
    p_ = Eigen::Vector3d(init_p[0], init_p[1], init_p[2]);
    R_ = rpyDegToRot(init_rpy);

    loadMap();
    if (loop_relocalization_enable_)
    {
      buildLoopDatabase();
    }

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(ndt_odom_topic_, 20);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_, 20);
    pub_path_ = nh_.advertise<nav_msgs::Path>(ndt_path_topic_, 5);
    pub_aligned_ = nh_.advertise<sensor_msgs::PointCloud2>(points_aligned_topic_, 5);
    if (publish_filtered_points_)
    {
      pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 5);
    }
    if (publish_diagnostics_)
    {
      pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    }
    path_.header.frame_id = map_frame_;

    if (lidar_msg_type_ == "pointcloud2")
    {
      sub_pc2_ = nh_.subscribe(lidar_topic_, 5, &DogPriorMapNdtNode::pointCloud2Callback, this);
    }
    else
    {
      sub_livox_ = nh_.subscribe(lidar_topic_, 5, &DogPriorMapNdtNode::livoxCallback, this);
    }
    if (use_external_initial_guess_)
    {
      sub_initial_guess_odom_ =
          nh_.subscribe(initial_guess_odom_topic_, 50, &DogPriorMapNdtNode::initialGuessOdomCallback, this);
    }

    ROS_INFO("[DogPriorMap NDT] started: map=%s target=%zu lidar=%s output=%s",
             map_pcd_path_.c_str(), target_cloud_->size(), lidar_topic_.c_str(), ndt_odom_topic_.c_str());
  }

private:
  template <typename T>
  T getParam(const std::string &name, const T &default_value)
  {
    T value;
    if (nh_.getParam(name, value)) return value;
    if (pnh_.getParam(name, value)) return value;
    return default_value;
  }

  std::vector<double> getParamVec(const std::string &name, const std::vector<double> &default_value)
  {
    std::vector<double> value;
    if (nh_.getParam(name, value) && value.size() >= default_value.size()) return value;
    if (pnh_.getParam(name, value) && value.size() >= default_value.size()) return value;
    return default_value;
  }

  void loadMap()
  {
    if (map_pcd_path_.empty()) throw std::runtime_error("map/pcd_fallback_path is empty");

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(map_pcd_path_, *raw) != 0)
    {
      throw std::runtime_error("failed to load map pcd: " + map_pcd_path_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
    xyz->reserve(raw->size());
    for (const auto &pt : raw->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) xyz->push_back(pt);
    }
    finalizeCloud(xyz);
    map_cloud_ = voxelDown(xyz, map_voxel_size_, 0);
    target_cloud_ = voxelDown(map_cloud_, target_voxel_size_, max_target_points_);
    map_kdtree_.setInputCloud(target_cloud_);

    ndt_.setInputTarget(target_cloud_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iterations_);
  }

  struct LoopKeyframe
  {
    int id = -1;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    std::vector<float> descriptor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_world;
  };

  struct LoopCandidate
  {
    int index = -1;
    int yaw_shift = 0;
    double similarity = 0.0;
  };

  void buildLoopDatabase()
  {
    if (loop_pose_path_.empty())
    {
      ROS_WARN("[DogPriorMap NDT] loop_relocalization enabled but pose_path is empty.");
      loop_relocalization_enable_ = false;
      return;
    }
    std::ifstream fin(loop_pose_path_);
    if (!fin.is_open())
    {
      ROS_WARN("[DogPriorMap NDT] failed to open loop pose path: %s", loop_pose_path_.c_str());
      loop_relocalization_enable_ = false;
      return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(map_cloud_);

    std::string line;
    Eigen::Vector3d last_key_p = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    int raw_id = 0;
    while (std::getline(fin, line))
    {
      std::istringstream iss(line);
      double stamp = 0.0;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      double qw = 1.0;
      if (!(iss >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
      (void)stamp;
      Eigen::Vector3d p(x, y, z);
      if (last_key_p.allFinite() && (p - last_key_p).norm() < loop_keyframe_spacing_m_)
      {
        ++raw_id;
        continue;
      }

      pcl::PointXYZ search_pt;
      search_pt.x = static_cast<float>(x);
      search_pt.y = static_cast<float>(y);
      search_pt.z = static_cast<float>(z);
      std::vector<int> indices;
      std::vector<float> distances;
      kdtree.radiusSearch(search_pt, loop_context_radius_, indices, distances);
      if (indices.empty())
      {
        ++raw_id;
        continue;
      }

      Eigen::Quaterniond q(qw, qx, qy, qz);
      if (q.norm() < 1e-9)
      {
        ++raw_id;
        continue;
      }
      const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
      pcl::PointCloud<pcl::PointXYZ>::Ptr local_body(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr local_world(new pcl::PointCloud<pcl::PointXYZ>());
      local_body->reserve(std::min(static_cast<int>(indices.size()), std::max(loop_context_max_points_, 1)));
      local_world->reserve(std::min(static_cast<int>(indices.size()), std::max(loop_context_max_points_, 1)));
      const int stride = std::max(1, static_cast<int>(indices.size()) / std::max(loop_context_max_points_, 1));
      for (size_t i = 0; i < indices.size(); i += static_cast<size_t>(stride))
      {
        const auto &map_pt = map_cloud_->points[static_cast<size_t>(indices[i])];
        const Eigen::Vector3d pw(map_pt.x, map_pt.y, map_pt.z);
        const Eigen::Vector3d pl = R.transpose() * (pw - p);
        local_body->push_back(pcl::PointXYZ(pl.x(), pl.y(), pl.z()));
        local_world->push_back(map_pt);
        if (static_cast<int>(local_body->size()) >= loop_context_max_points_) break;
      }
      finalizeCloud(local_body);
      finalizeCloud(local_world);

      LoopKeyframe key;
      key.id = raw_id;
      key.p = p;
      key.R = R;
      key.descriptor = makeScanContext(local_body);
      key.local_map_world = voxelDown(local_world, map_voxel_size_, loop_context_max_points_);
      if (!loop_scan_dir_.empty())
      {
        std::ostringstream pcd_name;
        pcd_name << loop_scan_dir_ << "/key_" << std::setw(6) << std::setfill('0') << raw_id << ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_body(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name.str(), *scan_body) == 0 && !scan_body->empty())
        {
          scan_body = voxelDown(scan_body, scan_voxel_size_, loop_context_max_points_);
          pcl::PointCloud<pcl::PointXYZ>::Ptr scan_world(new pcl::PointCloud<pcl::PointXYZ>());
          scan_world->reserve(scan_body->size());
          for (const auto &pt : scan_body->points)
          {
            const Eigen::Vector3d pb(pt.x, pt.y, pt.z);
            const Eigen::Vector3d pw = R * pb + p;
            scan_world->push_back(pcl::PointXYZ(pw.x(), pw.y(), pw.z()));
          }
          finalizeCloud(scan_world);
          key.descriptor = makeScanContext(scan_body);
          key.local_map_world = voxelDown(scan_world, map_voxel_size_, loop_context_max_points_);
        }
      }
      loop_database_.push_back(key);
      last_key_p = p;
      ++raw_id;
    }

    if (loop_database_.empty())
    {
      ROS_WARN("[DogPriorMap NDT] loop database is empty; disable relocalization.");
      loop_relocalization_enable_ = false;
      return;
    }
    ROS_INFO("[DogPriorMap NDT] loop database built: %zu keyframes from %s",
             loop_database_.size(), loop_pose_path_.c_str());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildLoopPriorSubmap(int candidate_index) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    if (candidate_index < 0 || candidate_index >= static_cast<int>(loop_database_.size())) return target;
    const int half_width = std::max(0, loop_prior_submap_half_width_);
    const int begin = std::max(0, candidate_index - half_width);
    const int end = std::min(static_cast<int>(loop_database_.size()) - 1, candidate_index + half_width);
    for (int i = begin; i <= end; ++i)
    {
      const auto &key = loop_database_[static_cast<size_t>(i)];
      if (!key.local_map_world || key.local_map_world->empty()) continue;
      *target += *key.local_map_world;
    }
    finalizeCloud(target);
    return voxelDown(target, map_voxel_size_, loop_context_max_points_);
  }

  std::vector<float> makeScanContext(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
  {
    const int rings = std::max(loop_num_rings_, 1);
    const int sectors = std::max(loop_num_sectors_, 1);
    std::vector<float> descriptor(static_cast<size_t>(rings * sectors), 0.0f);
    if (!cloud) return descriptor;
    const double max_radius = std::max(loop_context_radius_, 1.0);
    for (const auto &pt : cloud->points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
      if (r < 0.5 || r > max_radius) continue;
      double theta = std::atan2(pt.y, pt.x);
      if (theta < 0.0) theta += 2.0 * M_PI;
      const int ring = std::min(rings - 1, std::max(0, static_cast<int>(std::floor(r / max_radius * rings))));
      const int sector = std::min(sectors - 1, std::max(0, static_cast<int>(std::floor(theta / (2.0 * M_PI) * sectors))));
      const size_t idx = static_cast<size_t>(ring * sectors + sector);
      descriptor[idx] = std::max(descriptor[idx], static_cast<float>(pt.z + 3.0));
    }
    return descriptor;
  }

  double scanContextSimilarity(const std::vector<float> &a,
                               const std::vector<float> &b,
                               int shift,
                               int &valid_columns) const
  {
    const int rings = std::max(loop_num_rings_, 1);
    const int sectors = std::max(loop_num_sectors_, 1);
    if (static_cast<int>(a.size()) != rings * sectors || static_cast<int>(b.size()) != rings * sectors) return 0.0;

    double sum = 0.0;
    valid_columns = 0;
    for (int sector = 0; sector < sectors; ++sector)
    {
      const int shifted_sector = (sector + shift + sectors) % sectors;
      double dot = 0.0;
      double norm_a = 0.0;
      double norm_b = 0.0;
      for (int ring = 0; ring < rings; ++ring)
      {
        const float va = a[static_cast<size_t>(ring * sectors + sector)];
        const float vb = b[static_cast<size_t>(ring * sectors + shifted_sector)];
        dot += static_cast<double>(va) * static_cast<double>(vb);
        norm_a += static_cast<double>(va) * static_cast<double>(va);
        norm_b += static_cast<double>(vb) * static_cast<double>(vb);
      }
      if (norm_a > 1e-6 && norm_b > 1e-6)
      {
        sum += dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
        ++valid_columns;
      }
    }
    if (valid_columns <= 0) return 0.0;
    return sum / static_cast<double>(valid_columns);
  }

  std::vector<LoopCandidate> queryLoopCandidates(const std::vector<float> &descriptor) const
  {
    std::vector<LoopCandidate> candidates;
    for (size_t i = 0; i < loop_database_.size(); ++i)
    {
      int best_shift = 0;
      int best_valid = 0;
      double best_similarity = 0.0;
      const int sectors = std::max(loop_num_sectors_, 1);
      for (int shift = 0; shift < sectors; ++shift)
      {
        int valid_columns = 0;
        const double similarity = scanContextSimilarity(descriptor, loop_database_[i].descriptor, shift, valid_columns);
        if (valid_columns >= sectors / 6 && similarity > best_similarity)
        {
          best_similarity = similarity;
          best_shift = shift;
          best_valid = valid_columns;
        }
      }
      if (best_valid > 0 && best_similarity >= loop_min_similarity_)
      {
        LoopCandidate candidate;
        candidate.index = static_cast<int>(i);
        candidate.yaw_shift = best_shift;
        candidate.similarity = best_similarity;
        candidates.push_back(candidate);
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const LoopCandidate &lhs, const LoopCandidate &rhs) {
      return lhs.similarity > rhs.similarity;
    });
    if (static_cast<int>(candidates.size()) > loop_max_candidates_)
    {
      candidates.resize(static_cast<size_t>(loop_max_candidates_));
    }
    return candidates;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                double voxel_size,
                                                int max_points) const
  {
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
      const double step = static_cast<double>(down->size() - 1) / static_cast<double>(std::max(max_points - 1, 1));
      for (int i = 0; i < max_points; ++i)
      {
        sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
      }
      finalizeCloud(sampled);
      return sampled;
    }
    finalizeCloud(down);
    return down;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->size());
    for (const auto &pt : cloud->points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r < min_range_ || r > max_range_) continue;
      if (pt.z < min_z_ || pt.z > max_z_) continue;
      filtered->push_back(pt);
    }
    finalizeCloud(filtered);
    return voxelDown(filtered, scan_voxel_size_, max_scan_points_);
  }

  void livoxCallback(const livox_ros_driver2::CustomMsgConstPtr &msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(msg->points.size());
    for (const auto &pt : msg->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      {
        cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      }
    }
    finalizeCloud(cloud);
    handleCloud(cloud, msg->header.stamp);
  }

  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    handleCloud(cloud, msg->header.stamp);
  }

  void initialGuessOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Vector3d p_guess(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
    Eigen::Quaterniond q_guess(msg->pose.pose.orientation.w,
                               msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z);
    if (!p_guess.allFinite() || q_guess.norm() < 1e-9) return;
    latest_initial_guess_pose_ = poseToMatrix(p_guess, q_guess.normalized().toRotationMatrix());
    latest_initial_guess_stamp_ = msg->header.stamp;
    has_latest_initial_guess_ = true;
  }

  void handleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const ros::Time &stamp)
  {
    if (!cloud || cloud->empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source = preprocess(cloud);
    publishCloud(source, stamp, base_frame_, pub_filtered_);
    if (static_cast<int>(source->size()) < min_effective_points_)
    {
      publishDiagnostics(stamp,
                         false,
                         false,
                         "too_few_points",
                         0.0,
                         static_cast<int>(source->size()),
                         target_cloud_->size(),
                         0.0,
                         0);
      return;
    }

    Eigen::Matrix4d initial_guess = poseToMatrix(p_, R_);
    if (has_previous_pose_)
    {
      initial_guess = previous_pose_ * delta_pose_;
    }
    if (use_external_initial_guess_ && has_latest_initial_guess_)
    {
      const double age = std::abs((stamp - latest_initial_guess_stamp_).toSec());
      if (age <= initial_guess_max_age_sec_)
      {
        initial_guess = blendInitialGuess(initial_guess, latest_initial_guess_pose_);
      }
    }
    const Eigen::Matrix4d pose_before_update = poseToMatrix(p_, R_);
    const bool had_previous_pose_before_update = has_previous_pose_;

    ndt_.setInputSource(source);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    const ros::WallTime align_start = ros::WallTime::now();
    ndt_.align(aligned, initial_guess.cast<float>());
    const double align_ms = (ros::WallTime::now() - align_start).toSec() * 1000.0;
    const bool converged = ndt_.hasConverged();
    const double score = ndt_.getFitnessScore();
    const int iterations = ndt_.getFinalNumIteration();
    Eigen::Matrix4d result = ndt_.getFinalTransformation().cast<double>();
    std::string reject_reason = "accepted";
    const bool accepted = acceptNdtResult(converged, score, iterations, result, reject_reason);
    bool loop_used = false;
    std::string loop_reason = "disabled";
    double loop_similarity = 0.0;
    double loop_fitness = std::numeric_limits<double>::quiet_NaN();
    double loop_correction_norm = 0.0;
    double loop_yaw_correction_deg = 0.0;

    if (accepted)
    {
      if (has_previous_pose_)
      {
        delta_pose_ = previous_pose_.inverse() * result;
      }
      else
      {
        has_previous_pose_ = true;
        delta_pose_.setIdentity();
      }
      previous_pose_ = result;
      R_ = result.block<3, 3>(0, 0);
      p_ = result.block<3, 1>(0, 3);
      publishPose(stamp);
      publishAlignedCloud(aligned, stamp);
    }
    if (loop_relocalization_enable_)
    {
      loop_used = tryLoopRelocalization(source,
                                        stamp,
                                        loop_reason,
                                        loop_similarity,
                                        loop_fitness,
                                        loop_correction_norm,
                                        loop_yaw_correction_deg);
      if (loop_used)
      {
        publishPose(stamp);
      }
    }

    AmbiguityStats ambiguity;
    if (accepted)
    {
      ambiguity = evaluateNdtAmbiguity(source, result, stamp);
    }

    writeNdtDiagnosticsCsv(stamp,
                           converged,
                           accepted,
                           reject_reason,
                           align_ms,
                           static_cast<int>(source->size()),
                           target_cloud_->size(),
                           score,
                           iterations,
                           initial_guess,
                           result,
                           pose_before_update,
                           poseToMatrix(p_, R_),
                           had_previous_pose_before_update,
                           ambiguity);

    publishDiagnostics(stamp,
                       converged,
                       accepted,
                       reject_reason,
                       align_ms,
                       static_cast<int>(source->size()),
                       target_cloud_->size(),
                       score,
                       iterations,
                       loop_used,
                       loop_reason,
                       loop_similarity,
                       loop_fitness,
                       loop_correction_norm,
                       loop_yaw_correction_deg);
    ROS_INFO_THROTTLE(1.0,
                      "[DogPriorMap NDT] conv=%d accept=%d reason=%s source=%zu target=%zu align=%.2fms score=%.4f iter=%d p=(%.2f %.2f %.2f)",
                      converged ? 1 : 0, accepted ? 1 : 0, reject_reason.c_str(), source->size(), target_cloud_->size(),
                      align_ms, score, iterations, p_.x(), p_.y(), p_.z());
  }

  bool tryLoopRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                             const ros::Time &stamp,
                             std::string &reason,
                             double &best_similarity,
                             double &best_fitness,
                             double &correction_norm,
                             double &yaw_correction_deg)
  {
    reason = "period_skip";
    best_similarity = 0.0;
    best_fitness = std::numeric_limits<double>::quiet_NaN();
    correction_norm = 0.0;
    yaw_correction_deg = 0.0;
    if (!source || source->empty() || loop_database_.empty()) return false;
    if (last_loop_query_time_ > 0.0 && stamp.toSec() - last_loop_query_time_ < loop_query_period_sec_) return false;
    last_loop_query_time_ = stamp.toSec();

    const std::vector<float> descriptor = makeScanContext(source);
    const std::vector<LoopCandidate> candidates = queryLoopCandidates(descriptor);
    if (candidates.empty())
    {
      reason = "no_candidate";
      loop_consecutive_accepts_ = 0;
      return false;
    }

    bool refined_ok = false;
    Eigen::Matrix4d best_result = Eigen::Matrix4d::Identity();
    LoopCandidate best_candidate = candidates.front();
    best_fitness = std::numeric_limits<double>::infinity();
    const int sectors = std::max(loop_num_sectors_, 1);
    for (const auto &candidate : candidates)
    {
      const LoopKeyframe &key = loop_database_[static_cast<size_t>(candidate.index)];
      pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_target = buildLoopPriorSubmap(candidate.index);
      if (!candidate_target || candidate_target->empty()) continue;
      const double shift_yaw = static_cast<double>(candidate.yaw_shift) * 2.0 * M_PI / static_cast<double>(sectors);
      for (double sign : {1.0, -1.0})
      {
        Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
        guess.block<3, 3>(0, 0) = key.R * yawToRot(sign * shift_yaw);
        guess.block<3, 1>(0, 3) = key.p;

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_check;
        ndt_check.setInputTarget(candidate_target);
        ndt_check.setInputSource(source);
        ndt_check.setResolution(ndt_resolution_);
        ndt_check.setStepSize(ndt_step_size_);
        ndt_check.setTransformationEpsilon(ndt_transformation_epsilon_);
        ndt_check.setMaximumIterations(std::max(10, ndt_max_iterations_ / 2));
        pcl::PointCloud<pcl::PointXYZ> aligned_candidate;
        ndt_check.align(aligned_candidate, guess.cast<float>());
        if (!ndt_check.hasConverged()) continue;
        const double fitness = ndt_check.getFitnessScore();
        if (fitness < best_fitness)
        {
          best_fitness = fitness;
          best_similarity = candidate.similarity;
          best_result = ndt_check.getFinalTransformation().cast<double>();
          best_candidate = candidate;
          refined_ok = true;
        }
      }
    }
    if (!refined_ok)
    {
      reason = "refine_failed";
      loop_consecutive_accepts_ = 0;
      return false;
    }

    const Eigen::Vector3d p_loop = best_result.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_loop = best_result.block<3, 3>(0, 0);
    const Eigen::Vector3d dp = p_loop - p_;
    const double yaw_delta = wrapPi(yawFromRot(R_loop) - yawFromRot(R_));
    correction_norm = dp.norm();
    yaw_correction_deg = std::abs(yaw_delta) * 180.0 / M_PI;

    if (best_fitness > loop_accept_max_fitness_score_)
    {
      reason = "fitness_reject";
      loop_consecutive_accepts_ = 0;
      return false;
    }
    if (loop_accept_max_translation_ > 0.0 && correction_norm > loop_accept_max_translation_)
    {
      reason = "translation_reject";
      loop_consecutive_accepts_ = 0;
      return false;
    }
    if (loop_accept_max_yaw_deg_ > 0.0 && yaw_correction_deg > loop_accept_max_yaw_deg_)
    {
      reason = "yaw_reject";
      loop_consecutive_accepts_ = 0;
      return false;
    }

    const LoopKeyframe &accepted_key = loop_database_[static_cast<size_t>(best_candidate.index)];
    const bool same_candidate_region =
        last_loop_candidate_index_ == best_candidate.index ||
        (last_loop_candidate_position_.allFinite() &&
         (accepted_key.p - last_loop_candidate_position_).norm() <= loop_consecutive_candidate_radius_);
    if (same_candidate_region)
    {
      ++loop_consecutive_accepts_;
    }
    else
    {
      loop_consecutive_accepts_ = 1;
    }
    last_loop_candidate_index_ = best_candidate.index;
    last_loop_candidate_position_ = accepted_key.p;
    if (loop_consecutive_accepts_ < std::max(loop_require_consecutive_accepts_, 1))
    {
      reason = "waiting_consecutive";
      return false;
    }

    const double ratio = std::max(0.0, std::min(1.0, loop_apply_ratio_));
    Eigen::Vector3d limited_dp = dp;
    if (loop_max_position_correction_ > 0.0 && limited_dp.norm() > loop_max_position_correction_)
    {
      limited_dp = limited_dp.normalized() * loop_max_position_correction_;
    }
    const double max_yaw = loop_max_yaw_correction_deg_ * M_PI / 180.0;
    const double limited_yaw = std::max(-max_yaw, std::min(max_yaw, yaw_delta));

    p_ += ratio * limited_dp;
    R_ = yawToRot(ratio * limited_yaw) * R_;
    previous_pose_ = poseToMatrix(p_, R_);
    reason = "accepted";
    return true;
  }

  bool acceptNdtResult(bool converged,
                       double score,
                       int iterations,
                       const Eigen::Matrix4d &result,
                       std::string &reject_reason) const
  {
    if (!converged)
    {
      reject_reason = "not_converged";
      return false;
    }
    if (!ndt_acceptance_enable_) return true;
    if (ndt_reject_zero_iteration_ && iterations <= 0)
    {
      reject_reason = "zero_iteration";
      return false;
    }
    if (std::isfinite(ndt_max_fitness_score_) && ndt_max_fitness_score_ > 0.0 && score > ndt_max_fitness_score_)
    {
      reject_reason = "fitness_score";
      return false;
    }
    if (has_previous_pose_)
    {
      const Eigen::Matrix4d step = previous_pose_.inverse() * result;
      const double translation = step.block<3, 1>(0, 3).norm();
      const double rotation_deg = rotationAngleDeg(step.block<3, 3>(0, 0));
      if (ndt_accept_max_translation_ > 0.0 && translation > ndt_accept_max_translation_)
      {
        reject_reason = "translation_jump";
        return false;
      }
      if (ndt_accept_max_rotation_deg_ > 0.0 && rotation_deg > ndt_accept_max_rotation_deg_)
      {
        reject_reason = "rotation_jump";
        return false;
      }
    }
    return true;
  }

  struct AmbiguityStats
  {
    bool checked = false;
    double best_mean_residual = std::numeric_limits<double>::quiet_NaN();
    double current_mean_residual = std::numeric_limits<double>::quiet_NaN();
    int near_best_count = 0;
    double best_dx = 0.0;
    double best_dy = 0.0;
    double best_yaw_deg = 0.0;
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleForAmbiguity(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source) const
  {
    if (!source) return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (ndt_ambiguity_max_points_ <= 0 || static_cast<int>(source->size()) <= ndt_ambiguity_max_points_) return source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
    sampled->reserve(ndt_ambiguity_max_points_);
    const double step = static_cast<double>(source->size() - 1) /
                        static_cast<double>(std::max(ndt_ambiguity_max_points_ - 1, 1));
    for (int i = 0; i < ndt_ambiguity_max_points_; ++i)
    {
      sampled->push_back(source->points[static_cast<size_t>(std::round(i * step))]);
    }
    finalizeCloud(sampled);
    return sampled;
  }

  double meanNearestMapResidual(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                const Eigen::Matrix4d &pose) const
  {
    if (!source || source->empty() || !target_cloud_ || target_cloud_->empty()) return std::numeric_limits<double>::infinity();
    const Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    const Eigen::Vector3d p = pose.block<3, 1>(0, 3);
    double sum = 0.0;
    int used = 0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto &pt : source->points)
    {
      const Eigen::Vector3d local(pt.x, pt.y, pt.z);
      const Eigen::Vector3d world = R * local + p;
      pcl::PointXYZ query;
      query.x = static_cast<float>(world.x());
      query.y = static_cast<float>(world.y());
      query.z = static_cast<float>(world.z());
      if (map_kdtree_.nearestKSearch(query, 1, indices, distances) > 0)
      {
        sum += std::sqrt(static_cast<double>(distances[0]));
        ++used;
      }
    }
    return used > 0 ? sum / static_cast<double>(used) : std::numeric_limits<double>::infinity();
  }

  AmbiguityStats evaluateNdtAmbiguity(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                      const Eigen::Matrix4d &pose,
                                      const ros::Time &stamp)
  {
    AmbiguityStats stats;
    if (!ndt_ambiguity_check_enable_) return stats;
    if (last_ambiguity_check_time_ > 0.0 &&
        stamp.toSec() - last_ambiguity_check_time_ < ndt_ambiguity_period_sec_)
    {
      return stats;
    }
    last_ambiguity_check_time_ = stamp.toSec();
    stats.checked = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled = sampleForAmbiguity(source);
    if (!sampled || sampled->empty()) return stats;

    const Eigen::Vector3d base_p = pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d base_R = pose.block<3, 3>(0, 0);
    const double yaw_step = std::max(ndt_ambiguity_yaw_step_deg_, 0.1);
    const double xy_step = std::max(ndt_ambiguity_search_step_, 0.1);
    const double radius = std::max(ndt_ambiguity_search_radius_, 0.0);
    const int xy_steps = static_cast<int>(std::floor(radius / xy_step));
    const int yaw_steps = static_cast<int>(std::floor(std::max(ndt_ambiguity_yaw_search_deg_, 0.0) / yaw_step));

    struct CandidateScore
    {
      double mean = std::numeric_limits<double>::infinity();
      double dx = 0.0;
      double dy = 0.0;
      double yaw_deg = 0.0;
    };
    std::vector<CandidateScore> scores;
    scores.reserve(static_cast<size_t>((2 * xy_steps + 1) * (2 * xy_steps + 1) * (2 * yaw_steps + 1)));

    for (int ix = -xy_steps; ix <= xy_steps; ++ix)
    {
      for (int iy = -xy_steps; iy <= xy_steps; ++iy)
      {
        for (int iz = -yaw_steps; iz <= yaw_steps; ++iz)
        {
          const double dx = static_cast<double>(ix) * xy_step;
          const double dy = static_cast<double>(iy) * xy_step;
          const double yaw_deg = static_cast<double>(iz) * yaw_step;
          Eigen::Matrix4d candidate = Eigen::Matrix4d::Identity();
          candidate.block<3, 3>(0, 0) =
              yawToRot(yaw_deg * M_PI / 180.0) * base_R;
          candidate.block<3, 1>(0, 3) = base_p + Eigen::Vector3d(dx, dy, 0.0);
          scores.push_back({meanNearestMapResidual(sampled, candidate), dx, dy, yaw_deg});
        }
      }
    }
    if (scores.empty()) return stats;
    const auto best_it = std::min_element(scores.begin(), scores.end(),
                                          [](const CandidateScore &a, const CandidateScore &b) {
                                            return a.mean < b.mean;
                                          });
    stats.best_mean_residual = best_it->mean;
    stats.best_dx = best_it->dx;
    stats.best_dy = best_it->dy;
    stats.best_yaw_deg = best_it->yaw_deg;
    stats.current_mean_residual = meanNearestMapResidual(sampled, pose);
    const double near_threshold =
        stats.best_mean_residual * std::max(ndt_ambiguity_near_best_ratio_, 1.0) +
        std::max(ndt_ambiguity_near_best_margin_, 0.0);
    for (const auto &score : scores)
    {
      if (score.mean <= near_threshold) ++stats.near_best_count;
    }
    return stats;
  }

  void writeNdtDiagnosticsCsv(const ros::Time &stamp,
                              bool converged,
                              bool accepted,
                              const std::string &reject_reason,
                              double align_ms,
                              int scan_points,
                              size_t map_points,
                              double score,
                              int iterations,
                              const Eigen::Matrix4d &initial_guess,
                              const Eigen::Matrix4d &result,
                              const Eigen::Matrix4d &pose_before_update,
                              const Eigen::Matrix4d &used_pose,
                              bool had_previous_pose_before_update,
                              const AmbiguityStats &ambiguity)
  {
    if (!ndt_diagnostics_csv_) return;

    const Eigen::Vector3d initial_p = initial_guess.block<3, 1>(0, 3);
    const Eigen::Vector3d result_p = result.block<3, 1>(0, 3);
    const Eigen::Vector3d used_p = used_pose.block<3, 1>(0, 3);
    const double initial_to_result_translation = (result_p - initial_p).norm();

    double previous_to_result_translation = 0.0;
    double previous_to_result_rotation_deg = 0.0;
    double previous_to_used_translation = 0.0;
    double previous_to_used_rotation_deg = 0.0;
    if (had_previous_pose_before_update)
    {
      const Eigen::Matrix4d previous_to_result = pose_before_update.inverse() * result;
      previous_to_result_translation = previous_to_result.block<3, 1>(0, 3).norm();
      previous_to_result_rotation_deg = rotationAngleDeg(previous_to_result.block<3, 3>(0, 0));

      const Eigen::Matrix4d previous_to_used = pose_before_update.inverse() * used_pose;
      previous_to_used_translation = previous_to_used.block<3, 1>(0, 3).norm();
      previous_to_used_rotation_deg = rotationAngleDeg(previous_to_used.block<3, 3>(0, 0));
    }

    ndt_diagnostics_csv_ << std::fixed << std::setprecision(6)
                         << stamp.toSec() << ","
                         << (converged ? 1 : 0) << ","
                         << (accepted ? 1 : 0) << ","
                         << reject_reason << ","
                         << align_ms << ","
                         << scan_points << ","
                         << map_points << ","
                         << score << ","
                         << iterations << ","
                         << initial_p.x() << ","
                         << initial_p.y() << ","
                         << initial_p.z() << ","
                         << result_p.x() << ","
                         << result_p.y() << ","
                         << result_p.z() << ","
                         << used_p.x() << ","
                         << used_p.y() << ","
                         << used_p.z() << ","
                         << initial_to_result_translation << ","
                         << previous_to_result_translation << ","
                         << previous_to_result_rotation_deg << ","
                         << previous_to_used_translation << ","
                         << previous_to_used_rotation_deg << ","
                         << (ambiguity.checked ? 1 : 0) << ","
                         << ambiguity.best_mean_residual << ","
                         << ambiguity.current_mean_residual << ","
                         << ambiguity.near_best_count << ","
                         << ambiguity.best_dx << ","
                         << ambiguity.best_dy << ","
                         << ambiguity.best_yaw_deg << "\n";
  }

  Eigen::Matrix4d blendInitialGuess(const Eigen::Matrix4d &motion_guess, const Eigen::Matrix4d &external_guess) const
  {
    const double blend = std::max(0.0, std::min(1.0, initial_guess_blend_));
    if (blend <= 0.0) return motion_guess;
    if (blend >= 1.0) return external_guess;

    Eigen::Matrix4d blended = Eigen::Matrix4d::Identity();
    const Eigen::Vector3d p_motion = motion_guess.block<3, 1>(0, 3);
    const Eigen::Vector3d p_external = external_guess.block<3, 1>(0, 3);
    Eigen::Quaterniond q_motion(motion_guess.block<3, 3>(0, 0));
    Eigen::Quaterniond q_external(external_guess.block<3, 3>(0, 0));
    q_motion.normalize();
    q_external.normalize();
    blended.block<3, 1>(0, 3) = (1.0 - blend) * p_motion + blend * p_external;
    blended.block<3, 3>(0, 0) = q_motion.slerp(blend, q_external).toRotationMatrix();
    return blended;
  }

  void publishPose(const ros::Time &stamp)
  {
    Eigen::Quaterniond q(R_);
    q.normalize();

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = p_.x();
    odom.pose.pose.position.y = p_.y();
    odom.pose.pose.position.z = p_.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    pub_odom_.publish(odom);

    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pub_pose_.publish(pose);

    if (publish_path_)
    {
      path_.header.stamp = stamp;
      path_.header.frame_id = map_frame_;
      path_.poses.push_back(pose);
      if (path_.poses.size() > 5000) path_.poses.erase(path_.poses.begin());
      pub_path_.publish(path_);
    }

    if (publish_tf_)
    {
      tf::Transform tf_msg;
      tf_msg.setOrigin(tf::Vector3(p_.x(), p_.y(), p_.z()));
      tf_msg.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf_msg, stamp, map_frame_, base_frame_));
    }
  }

  void publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const ros::Time &stamp,
                    const std::string &frame_id,
                    ros::Publisher &publisher)
  {
    if (!publisher || !cloud) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    publisher.publish(msg);
  }

  void publishAlignedCloud(const pcl::PointCloud<pcl::PointXYZ> &aligned, const ros::Time &stamp)
  {
    if (!pub_aligned_) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(aligned, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    pub_aligned_.publish(msg);
  }

  void publishDiagnostics(const ros::Time &stamp,
                          bool converged,
                          bool accepted,
                          const std::string &reject_reason,
                          double align_ms,
                          int scan_points,
                          size_t map_points,
                          double score,
                          int iterations,
                          bool loop_used = false,
                          const std::string &loop_reason = "disabled",
                          double loop_similarity = 0.0,
                          double loop_fitness = std::numeric_limits<double>::quiet_NaN(),
                          double loop_correction_norm = 0.0,
                          double loop_yaw_correction_deg = 0.0)
  {
    if (!publish_diagnostics_ || !pub_diagnostics_) return;
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "dog_prior_map_ndt";
    status.hardware_id = base_frame_;
    status.level = accepted ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = accepted ? "NDT accepted" : "NDT rejected";

    addDiagnosticValue(status, "ndt_converged", converged ? "true" : "false");
    addDiagnosticValue(status, "ndt_accepted", accepted ? "true" : "false");
    addDiagnosticValue(status, "ndt_reject_reason", reject_reason);
    addDiagnosticValue(status, "align_time_ms", std::to_string(align_ms));
    addDiagnosticValue(status, "scan_points", std::to_string(scan_points));
    addDiagnosticValue(status, "map_points", std::to_string(map_points));
    addDiagnosticValue(status, "fitness_score", std::to_string(score));
    addDiagnosticValue(status, "iterations", std::to_string(iterations));
    addDiagnosticValue(status, "loop_relocalization_enabled", loop_relocalization_enable_ ? "true" : "false");
    addDiagnosticValue(status, "loop_relocalization_used", loop_used ? "true" : "false");
    addDiagnosticValue(status, "loop_relocalization_rejected_reason", loop_reason);
    addDiagnosticValue(status, "loop_similarity", std::to_string(loop_similarity));
    addDiagnosticValue(status, "loop_refine_fitness", std::to_string(loop_fitness));
    addDiagnosticValue(status, "loop_correction_norm", std::to_string(loop_correction_norm));
    addDiagnosticValue(status, "loop_yaw_correction_deg", std::to_string(loop_yaw_correction_deg));
    addDiagnosticValue(status, "loop_database_keyframes", std::to_string(loop_database_.size()));
    addDiagnosticValue(status, "pose_xyz", std::to_string(p_.x()) + "," +
                                  std::to_string(p_.y()) + "," +
                                  std::to_string(p_.z()));
    array.status.push_back(status);
    pub_diagnostics_.publish(array);
  }

  void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus &status,
                          const std::string &key,
                          const std::string &value)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_livox_;
  ros::Subscriber sub_pc2_;
  ros::Subscriber sub_initial_guess_odom_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_path_;
  ros::Publisher pub_filtered_;
  ros::Publisher pub_aligned_;
  ros::Publisher pub_diagnostics_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex mutex_;

  std::string map_frame_;
  std::string base_frame_;
  std::string lidar_topic_;
  std::string lidar_msg_type_;
  std::string filtered_points_topic_;
  std::string diagnostics_topic_;
  std::string ndt_odom_topic_;
  std::string ndt_pose_topic_;
  std::string ndt_path_topic_;
  std::string points_aligned_topic_;
  std::string initial_guess_odom_topic_;
  std::string map_pcd_path_;
  std::string loop_pose_path_;
  std::string ndt_diagnostics_csv_path_;
  std::ofstream ndt_diagnostics_csv_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  nav_msgs::Path path_;

  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool has_previous_pose_ = false;
  Eigen::Matrix4d previous_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d delta_pose_ = Eigen::Matrix4d::Identity();
  bool use_external_initial_guess_ = false;
  bool has_latest_initial_guess_ = false;
  ros::Time latest_initial_guess_stamp_;
  Eigen::Matrix4d latest_initial_guess_pose_ = Eigen::Matrix4d::Identity();
  double initial_guess_max_age_sec_ = 0.20;
  double initial_guess_blend_ = 1.0;
  bool loop_relocalization_enable_ = false;
  std::string loop_scan_dir_;
  double loop_query_period_sec_ = 1.0;
  double loop_keyframe_spacing_m_ = 1.0;
  int loop_prior_submap_half_width_ = 3;
  double loop_context_radius_ = 20.0;
  int loop_context_max_points_ = 2500;
  int loop_num_rings_ = 20;
  int loop_num_sectors_ = 60;
  int loop_max_candidates_ = 5;
  double loop_min_similarity_ = 0.15;
  double loop_accept_max_fitness_score_ = 1.0;
  double loop_accept_max_translation_ = 8.0;
  double loop_accept_max_yaw_deg_ = 25.0;
  int loop_require_consecutive_accepts_ = 2;
  double loop_consecutive_candidate_radius_ = 3.0;
  double loop_apply_ratio_ = 0.5;
  double loop_max_position_correction_ = 1.0;
  double loop_max_yaw_correction_deg_ = 5.0;
  double last_loop_query_time_ = -1.0;
  int last_loop_candidate_index_ = -1;
  Eigen::Vector3d last_loop_candidate_position_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  int loop_consecutive_accepts_ = 0;
  std::vector<LoopKeyframe> loop_database_;

  double map_voxel_size_ = 0.30;
  double target_voxel_size_ = 0.30;
  double scan_voxel_size_ = 0.35;
  int max_scan_points_ = 900;
  int max_target_points_ = 0;
  double min_range_ = 0.5;
  double max_range_ = 80.0;
  double min_z_ = -std::numeric_limits<double>::infinity();
  double max_z_ = std::numeric_limits<double>::infinity();
  int min_effective_points_ = 50;
  int ndt_max_iterations_ = 30;
  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_transformation_epsilon_ = 0.001;
  bool ndt_acceptance_enable_ = false;
  bool ndt_ambiguity_check_enable_ = false;
  double ndt_ambiguity_period_sec_ = 1.0;
  double ndt_ambiguity_search_radius_ = 2.0;
  double ndt_ambiguity_search_step_ = 1.0;
  double ndt_ambiguity_yaw_search_deg_ = 8.0;
  double ndt_ambiguity_yaw_step_deg_ = 4.0;
  int ndt_ambiguity_max_points_ = 300;
  double ndt_ambiguity_near_best_ratio_ = 1.25;
  double ndt_ambiguity_near_best_margin_ = 0.03;
  double last_ambiguity_check_time_ = -1.0;
  bool ndt_reject_zero_iteration_ = true;
  double ndt_max_fitness_score_ = 3.0;
  double ndt_accept_max_translation_ = 1.20;
  double ndt_accept_max_rotation_deg_ = 12.0;
  bool publish_tf_ = true;
  bool publish_path_ = false;
  bool publish_filtered_points_ = true;
  bool publish_diagnostics_ = true;
};
}  // namespace dog_prior_map_localization

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dog_prior_map_ndt");
  try
  {
    dog_prior_map_localization::DogPriorMapNdtNode node;
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[DogPriorMap NDT] node exited: %s", e.what());
    return 1;
  }
  return 0;
}
