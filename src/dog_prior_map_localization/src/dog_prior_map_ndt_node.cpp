#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
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
    ndt_max_fitness_score_ = getParam<double>("lidar_update/ndt_max_fitness_score", 3.0);
    ndt_reject_zero_iteration_ = getParam<bool>("lidar_update/ndt_reject_zero_iteration", true);
    ndt_accept_max_translation_ = getParam<double>("lidar_update/ndt_accept_max_translation", 1.20);
    ndt_accept_max_rotation_deg_ = getParam<double>("lidar_update/ndt_accept_max_rotation_deg", 12.0);
    publish_tf_ = getParam<bool>("output/ndt_publish_tf", false);
    publish_path_ = getParam<bool>("output/publish_path", false);
    publish_filtered_points_ = getParam<bool>("output/publish_filtered_points", true);
    publish_diagnostics_ = getParam<bool>("output/publish_diagnostics", true);

    const std::vector<double> init_p = getParamVec("filter/initial_position", {0.0, 0.0, 0.0});
    const std::vector<double> init_rpy = getParamVec("filter/initial_rpy_deg", {0.0, 0.0, 0.0});
    p_ = Eigen::Vector3d(init_p[0], init_p[1], init_p[2]);
    R_ = rpyDegToRot(init_rpy);

    loadMap();

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

    ndt_.setInputTarget(target_cloud_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iterations_);
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

    publishDiagnostics(stamp,
                       converged,
                       accepted,
                       reject_reason,
                       align_ms,
                       static_cast<int>(source->size()),
                       target_cloud_->size(),
                       score,
                       iterations);
    ROS_INFO_THROTTLE(1.0,
                      "[DogPriorMap NDT] conv=%d accept=%d reason=%s source=%zu target=%zu align=%.2fms score=%.4f iter=%d p=(%.2f %.2f %.2f)",
                      converged ? 1 : 0, accepted ? 1 : 0, reject_reason.c_str(), source->size(), target_cloud_->size(),
                      align_ms, score, iterations, p_.x(), p_.y(), p_.z());
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
                          int iterations)
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
  std::string map_pcd_path_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  nav_msgs::Path path_;

  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool has_previous_pose_ = false;
  Eigen::Matrix4d previous_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d delta_pose_ = Eigen::Matrix4d::Identity();

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
