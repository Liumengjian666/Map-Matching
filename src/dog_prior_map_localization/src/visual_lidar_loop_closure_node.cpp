#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace dog_prior_map_localization
{
namespace
{
double wrapPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double yawFromRot(const Eigen::Matrix3d &R)
{
  return std::atan2(R(1, 0), R(0, 0));
}

Eigen::Matrix3d yawToRot(double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix4d poseToMatrix(const Eigen::Vector3d &p, const Eigen::Matrix3d &R)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

void finalizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                              double voxel_size,
                                              int max_points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
  if (!cloud) return down;
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
    const double step = static_cast<double>(down->size() - 1) /
                        static_cast<double>(std::max(max_points - 1, 1));
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

Eigen::Vector3d pose2dBetween(const Eigen::Vector3d &pi, double yawi,
                              const Eigen::Vector3d &pj, double yawj)
{
  const Eigen::Matrix2d Ri =
      Eigen::Rotation2Dd(yawi).toRotationMatrix();
  const Eigen::Vector2d d = Ri.transpose() * (pj.head<2>() - pi.head<2>());
  return Eigen::Vector3d(d.x(), d.y(), wrapPi(yawj - yawi));
}

}  // namespace

class VisualLidarLoopClosureNode
{
public:
  VisualLidarLoopClosureNode() : nh_(), pnh_("~")
  {
    enabled_ = getParam<bool>("visual_lidar_loop_closure/enable", false);
    map_frame_ = getParam<std::string>("frames/map_frame", "camera_init");
    base_frame_ = getParam<std::string>("frames/base_frame", "livox_frame");
    image_topic_ = getParam<std::string>("topics/image", "/image_left/image_rect");
    cloud_topic_ = getParam<std::string>("topics/filtered_points", "/dog_livo/filtered_points");
    odom_topic_ = getParam<std::string>("topics/ndt_odom", "/dog_livo/ndt_odom");
    corrected_odom_topic_ =
        getParam<std::string>("topics/loop_corrected_odom", "/dog_livo/loop_corrected_odom");
    corrected_path_topic_ =
        getParam<std::string>("topics/loop_corrected_path", "/dog_livo/loop_corrected_path");
    diagnostics_topic_ =
        getParam<std::string>("topics/loop_diagnostics", "/dog_livo/loop_diagnostics");

    keyframe_min_translation_ = getParam<double>("visual_lidar_loop_closure/keyframe_min_translation", 1.0);
    keyframe_min_rotation_deg_ = getParam<double>("visual_lidar_loop_closure/keyframe_min_rotation_deg", 8.0);
    keyframe_min_interval_sec_ = getParam<double>("visual_lidar_loop_closure/keyframe_min_interval_sec", 1.0);
    exclude_recent_keyframes_ = getParam<int>("visual_lidar_loop_closure/exclude_recent_keyframes", 30);
    max_keyframes_ = getParam<int>("visual_lidar_loop_closure/max_keyframes", 2000);

    orb_features_ = getParam<int>("visual_lidar_loop_closure/orb_features", 800);
    visual_top_k_ = getParam<int>("visual_lidar_loop_closure/visual_top_k", 5);
    min_visual_score_ = getParam<double>("visual_lidar_loop_closure/min_visual_score", 0.08);
    max_descriptor_distance_ = getParam<double>("visual_lidar_loop_closure/max_descriptor_distance", 60.0);
    ratio_test_ = getParam<double>("visual_lidar_loop_closure/ratio_test", 0.75);

    current_submap_history_width_ =
        getParam<int>("visual_lidar_loop_closure/current_submap_history_width", 4);
    candidate_submap_half_width_ = getParam<int>("visual_lidar_loop_closure/candidate_submap_half_width", 8);
    ndt_source_voxel_size_ = getParam<double>("visual_lidar_loop_closure/ndt_source_voxel_size", 0.35);
    ndt_target_voxel_size_ = getParam<double>("visual_lidar_loop_closure/ndt_target_voxel_size", 0.25);
    ndt_max_source_points_ = getParam<int>("visual_lidar_loop_closure/ndt_max_source_points", 900);
    ndt_max_multiframe_source_points_ =
        getParam<int>("visual_lidar_loop_closure/ndt_max_multiframe_source_points", 2500);
    ndt_max_target_points_ = getParam<int>("visual_lidar_loop_closure/ndt_max_target_points", 25000);
    ndt_resolution_ = getParam<double>("visual_lidar_loop_closure/ndt_resolution", 1.0);
    ndt_step_size_ = getParam<double>("visual_lidar_loop_closure/ndt_step_size", 0.1);
    ndt_transformation_epsilon_ =
        getParam<double>("visual_lidar_loop_closure/ndt_transformation_epsilon", 0.001);
    ndt_max_iterations_ = getParam<int>("visual_lidar_loop_closure/ndt_max_iterations", 30);
    accept_max_fitness_score_ =
        getParam<double>("visual_lidar_loop_closure/accept_max_fitness_score", 0.8);
    accept_max_loop_translation_ =
        getParam<double>("visual_lidar_loop_closure/accept_max_loop_translation", 20.0);
    accept_max_loop_yaw_deg_ =
        getParam<double>("visual_lidar_loop_closure/accept_max_loop_yaw_deg", 45.0);
    require_consecutive_accepts_ =
        getParam<int>("visual_lidar_loop_closure/require_consecutive_accepts", 2);

    pose_graph_iterations_ = getParam<int>("visual_lidar_loop_closure/pose_graph_iterations", 8);
    pose_graph_odom_weight_ = getParam<double>("visual_lidar_loop_closure/pose_graph_odom_weight", 1.0);
    pose_graph_loop_weight_ = getParam<double>("visual_lidar_loop_closure/pose_graph_loop_weight", 5.0);
    pose_graph_prior_weight_ = getParam<double>("visual_lidar_loop_closure/pose_graph_prior_weight", 1000.0);
    publish_debug_ = getParam<bool>("visual_lidar_loop_closure/publish_debug", true);

    orb_ = cv::ORB::create(std::max(100, orb_features_));
    pub_corrected_odom_ = nh_.advertise<nav_msgs::Odometry>(corrected_odom_topic_, 10);
    pub_corrected_path_ = nh_.advertise<nav_msgs::Path>(corrected_path_topic_, 5, true);
    pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    corrected_path_.header.frame_id = map_frame_;

    if (enabled_)
    {
      sub_image_ = nh_.subscribe(image_topic_, 3, &VisualLidarLoopClosureNode::imageCallback, this);
      sub_cloud_ = nh_.subscribe(cloud_topic_, 3, &VisualLidarLoopClosureNode::cloudCallback, this);
      sub_odom_ = nh_.subscribe(odom_topic_, 50, &VisualLidarLoopClosureNode::odomCallback, this);
      worker_ = std::thread(&VisualLidarLoopClosureNode::workerLoop, this);
    }
    ROS_INFO("[VisualLidarLoopClosure] enabled=%d image=%s cloud=%s odom=%s",
             enabled_ ? 1 : 0, image_topic_.c_str(), cloud_topic_.c_str(), odom_topic_.c_str());
  }

  ~VisualLidarLoopClosureNode()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_worker_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  struct Keyframe
  {
    int id = -1;
    ros::Time stamp;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double yaw = 0.0;
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body;
  };

  struct Edge
  {
    int i = -1;
    int j = -1;
    Eigen::Vector3d z = Eigen::Vector3d::Zero();
    double weight = 1.0;
    bool loop = false;
  };

  struct Candidate
  {
    int index = -1;
    double visual_score = 0.0;
  };

  template <typename T>
  T getParam(const std::string &name, const T &default_value)
  {
    T value;
    if (nh_.getParam(name, value)) return value;
    if (pnh_.getParam(name, value)) return value;
    return default_value;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    if (!enabled_) return;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (const cv_bridge::Exception &e)
    {
      ROS_WARN_THROTTLE(2.0, "[VisualLidarLoopClosure] image conversion failed: %s", e.what());
      return;
    }
    cv::Mat gray;
    if (cv_ptr->image.channels() == 1) gray = cv_ptr->image.clone();
    else if (cv_ptr->image.channels() == 3) cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    else if (cv_ptr->image.channels() == 4) cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGRA2GRAY);
    else return;

    std::lock_guard<std::mutex> lock(mutex_);
    latest_gray_ = gray;
    latest_image_stamp_ = msg->header.stamp;
    tryCreateKeyframeLocked(msg->header.stamp);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    finalizeCloud(cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = cloud;
    latest_cloud_stamp_ = msg->header.stamp;
    tryCreateKeyframeLocked(msg->header.stamp);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    Eigen::Vector3d p(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    if (!p.allFinite() || q.norm() < 1e-9) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_p_ = p;
    latest_R_ = q.normalized().toRotationMatrix();
    latest_odom_stamp_ = msg->header.stamp;
    has_odom_ = true;
    tryCreateKeyframeLocked(msg->header.stamp);
  }

  void tryCreateKeyframeLocked(const ros::Time &stamp)
  {
    if (!has_odom_ || latest_gray_.empty() || !latest_cloud_ || latest_cloud_->empty()) return;
    if (std::abs((stamp - latest_odom_stamp_).toSec()) > 0.2 ||
        std::abs((stamp - latest_image_stamp_).toSec()) > 0.3 ||
        std::abs((stamp - latest_cloud_stamp_).toSec()) > 0.3)
    {
      return;
    }

    if (!keyframes_.empty())
    {
      const Keyframe &last = keyframes_.back();
      const double dt = (latest_odom_stamp_ - last.stamp).toSec();
      const double dp = (latest_p_ - last.p).norm();
      const double dyaw = std::abs(wrapPi(yawFromRot(latest_R_) - last.yaw)) * 180.0 / M_PI;
      if (dt < keyframe_min_interval_sec_ && dp < keyframe_min_translation_ && dyaw < keyframe_min_rotation_deg_)
      {
        return;
      }
    }

    Keyframe key;
    key.id = next_keyframe_id_++;
    key.stamp = latest_odom_stamp_;
    key.p = latest_p_;
    key.R = latest_R_;
    key.yaw = yawFromRot(latest_R_);
    key.cloud_body = voxelDown(latest_cloud_, ndt_source_voxel_size_, ndt_max_source_points_);
    orb_->detectAndCompute(latest_gray_, cv::Mat(), key.keypoints, key.descriptors);
    if (key.descriptors.empty() || key.cloud_body->empty()) return;

    Edge odom_edge;
    bool has_odom_edge = false;
    if (!keyframes_.empty())
    {
      const Keyframe &prev = keyframes_.back();
      odom_edge.i = static_cast<int>(keyframes_.size()) - 1;
      odom_edge.j = static_cast<int>(keyframes_.size());
      odom_edge.z = pose2dBetween(prev.p, prev.yaw, key.p, key.yaw);
      odom_edge.weight = pose_graph_odom_weight_;
      odom_edge.loop = false;
      edges_.push_back(odom_edge);
      has_odom_edge = true;
    }

    keyframes_.push_back(key);
    Eigen::Vector3d optimized_state(key.p.x(), key.p.y(), key.yaw);
    if (has_odom_edge && hasLoopEdgeUnlocked())
    {
      const Eigen::Vector3d &prev_opt = optimized_xy_yaw_.back();
      const Eigen::Matrix2d R_prev = Eigen::Rotation2Dd(prev_opt.z()).toRotationMatrix();
      const Eigen::Vector2d d_world = R_prev * odom_edge.z.head<2>();
      optimized_state.x() = prev_opt.x() + d_world.x();
      optimized_state.y() = prev_opt.y() + d_world.y();
      optimized_state.z() = wrapPi(prev_opt.z() + odom_edge.z.z());
    }
    optimized_xy_yaw_.push_back(optimized_state);
    pending_indices_.push_back(static_cast<int>(keyframes_.size()) - 1);
    if (static_cast<int>(keyframes_.size()) > max_keyframes_)
    {
      ROS_WARN_THROTTLE(5.0, "[VisualLidarLoopClosure] keyframe buffer reached max_keyframes=%d; keep all for graph consistency.",
                        max_keyframes_);
    }
    cv_.notify_all();
  }

  std::vector<Candidate> retrieveVisualCandidates(const Keyframe &query,
                                                  int query_index,
                                                  const std::vector<Keyframe> &snapshot) const
  {
    std::vector<Candidate> candidates;
    if (query.descriptors.empty()) return candidates;
    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    const int search_end = std::min(query_index - exclude_recent_keyframes_,
                                    static_cast<int>(snapshot.size()));
    for (int i = 0; i < search_end; ++i)
    {
      const Keyframe &hist = snapshot[static_cast<size_t>(i)];
      if (hist.descriptors.empty()) continue;
      std::vector<std::vector<cv::DMatch>> knn;
      matcher.knnMatch(query.descriptors, hist.descriptors, knn, 2);
      int good = 0;
      for (const auto &m : knn)
      {
        if (m.size() < 2) continue;
        if (m[0].distance < ratio_test_ * m[1].distance &&
            m[0].distance < max_descriptor_distance_)
        {
          ++good;
        }
      }
      const double denom = static_cast<double>(std::max(1, std::min(query.descriptors.rows, hist.descriptors.rows)));
      const double score = static_cast<double>(good) / denom;
      if (score >= min_visual_score_)
      {
        candidates.push_back({i, score});
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
      return a.visual_score > b.visual_score;
    });
    if (static_cast<int>(candidates.size()) > visual_top_k_)
    {
      candidates.resize(static_cast<size_t>(visual_top_k_));
    }
    return candidates;
  }

  bool hasLoopEdgeUnlocked() const
  {
    return std::any_of(edges_.begin(), edges_.end(), [](const Edge &edge) {
      return edge.loop;
    });
  }

  void updateAndPublishGraph(const ros::Time &stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishCorrectedTrajectory(stamp);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildCurrentSourceSubmap(const std::vector<Keyframe> &snapshot,
                                                               int current_index) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
    if (current_index < 0 || current_index >= static_cast<int>(snapshot.size())) return source;

    const Keyframe &current = snapshot[static_cast<size_t>(current_index)];
    const Eigen::Matrix4d T_current = poseToMatrix(current.p, current.R);
    const Eigen::Matrix4d T_current_inv = T_current.inverse();
    const int begin = std::max(0, current_index - current_submap_history_width_);
    for (int i = begin; i <= current_index; ++i)
    {
      const Keyframe &kf = snapshot[static_cast<size_t>(i)];
      if (!kf.cloud_body) continue;
      const Eigen::Matrix4d T_i = poseToMatrix(kf.p, kf.R);
      const Eigen::Matrix4f T_current_i = (T_current_inv * T_i).cast<float>();
      pcl::PointCloud<pcl::PointXYZ> transformed;
      pcl::transformPointCloud(*kf.cloud_body, transformed, T_current_i);
      *source += transformed;
    }
    finalizeCloud(source);
    return voxelDown(source, ndt_source_voxel_size_, ndt_max_multiframe_source_points_);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildCandidateSubmap(const std::vector<Keyframe> &snapshot,
                                                           int candidate_index) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    const int begin = std::max(0, candidate_index - candidate_submap_half_width_);
    const int end = std::min(static_cast<int>(snapshot.size()) - 1, candidate_index + candidate_submap_half_width_);
    for (int i = begin; i <= end; ++i)
    {
      const Keyframe &kf = snapshot[static_cast<size_t>(i)];
      if (!kf.cloud_body) continue;
      Eigen::Matrix4d T = poseToMatrix(kf.p, kf.R);
      Eigen::Matrix4f Tf = T.cast<float>();
      pcl::PointCloud<pcl::PointXYZ> transformed;
      pcl::transformPointCloud(*kf.cloud_body, transformed, Tf);
      *target += transformed;
    }
    finalizeCloud(target);
    return voxelDown(target, ndt_target_voxel_size_, ndt_max_target_points_);
  }

  bool verifyCandidateByNdt(const Keyframe &current,
                            int current_index,
                            const std::vector<Keyframe> &snapshot,
                            const Candidate &candidate,
                            Eigen::Matrix4d &verified_pose,
                            double &fitness,
                            std::string &reject_reason) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = buildCandidateSubmap(snapshot, candidate.index);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source = buildCurrentSourceSubmap(snapshot, current_index);
    if (source->empty())
    {
      source = voxelDown(current.cloud_body, ndt_source_voxel_size_, ndt_max_source_points_);
    }
    if (static_cast<int>(source->size()) < 50 || static_cast<int>(target->size()) < 200)
    {
      reject_reason = "too_few_points";
      return false;
    }

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);

    const Keyframe &hist = snapshot[static_cast<size_t>(candidate.index)];
    Eigen::Matrix4d guess = poseToMatrix(hist.p, hist.R);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    ndt.align(aligned, guess.cast<float>());
    if (!ndt.hasConverged())
    {
      reject_reason = "ndt_not_converged";
      return false;
    }

    fitness = ndt.getFitnessScore();
    verified_pose = ndt.getFinalTransformation().cast<double>();
    const Eigen::Vector3d p_verified = verified_pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_verified = verified_pose.block<3, 3>(0, 0);
    const double correction = (p_verified - current.p).norm();
    const double yaw_correction = std::abs(wrapPi(yawFromRot(R_verified) - current.yaw)) * 180.0 / M_PI;
    if (!std::isfinite(fitness) || fitness > accept_max_fitness_score_)
    {
      reject_reason = "fitness_reject";
      return false;
    }
    if (correction > accept_max_loop_translation_)
    {
      reject_reason = "translation_reject";
      return false;
    }
    if (yaw_correction > accept_max_loop_yaw_deg_)
    {
      reject_reason = "yaw_reject";
      return false;
    }
    reject_reason = "accepted";
    return true;
  }

  void addLoopEdgeAndOptimize(int current_index,
                              int candidate_index,
                              const Eigen::Matrix4d &verified_pose,
                              double visual_score,
                              double fitness)
  {
    if (candidate_index < 0 || current_index <= candidate_index) return;
    const Keyframe &hist = keyframes_[static_cast<size_t>(candidate_index)];
    const double yaw_verified = yawFromRot(verified_pose.block<3, 3>(0, 0));
    const Eigen::Vector3d p_verified = verified_pose.block<3, 1>(0, 3);

    Edge loop_edge;
    loop_edge.i = candidate_index;
    loop_edge.j = current_index;
    loop_edge.z = pose2dBetween(hist.p, hist.yaw, p_verified, yaw_verified);
    loop_edge.weight = pose_graph_loop_weight_ * std::max(0.2, visual_score) /
                       std::max(0.2, fitness + 0.2);
    loop_edge.loop = true;
    edges_.push_back(loop_edge);
    optimizePoseGraph2d();
    publishCorrectedTrajectory(keyframes_[static_cast<size_t>(current_index)].stamp);
  }

  Eigen::Vector3d edgeResidual(const std::vector<Eigen::Vector3d> &states, const Edge &edge) const
  {
    const Eigen::Vector3d pred = pose2dBetween(Eigen::Vector3d(states[edge.i].x(), states[edge.i].y(), 0.0),
                                               states[edge.i].z(),
                                               Eigen::Vector3d(states[edge.j].x(), states[edge.j].y(), 0.0),
                                               states[edge.j].z());
    return Eigen::Vector3d(pred.x() - edge.z.x(),
                           pred.y() - edge.z.y(),
                           wrapPi(pred.z() - edge.z.z()));
  }

  void optimizePoseGraph2d()
  {
    const int n = static_cast<int>(optimized_xy_yaw_.size());
    if (n < 2) return;
    std::vector<Eigen::Vector3d> states = optimized_xy_yaw_;
    const double eps = 1e-4;
    for (int iter = 0; iter < pose_graph_iterations_; ++iter)
    {
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3 * n, 3 * n);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * n);

      H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity() * pose_graph_prior_weight_;
      b.segment<3>(0) += (states[0] - optimized_xy_yaw_[0]) * pose_graph_prior_weight_;

      for (const auto &edge : edges_)
      {
        if (edge.i < 0 || edge.j < 0 || edge.i >= n || edge.j >= n) continue;
        const Eigen::Vector3d r0 = edgeResidual(states, edge);
        Eigen::Matrix<double, 3, 6> J;
        for (int k = 0; k < 6; ++k)
        {
          std::vector<Eigen::Vector3d> perturbed = states;
          const int node = (k < 3) ? edge.i : edge.j;
          const int dim = k % 3;
          perturbed[static_cast<size_t>(node)](dim) += eps;
          if (dim == 2) perturbed[static_cast<size_t>(node)](dim) =
              wrapPi(perturbed[static_cast<size_t>(node)](dim));
          Eigen::Vector3d rp = edgeResidual(perturbed, edge);
          Eigen::Vector3d dr = rp - r0;
          dr.z() = wrapPi(dr.z());
          J.col(k) = dr / eps;
        }
        Eigen::Matrix3d W = Eigen::Matrix3d::Identity() * edge.weight;
        Eigen::Matrix<double, 6, 6> Hij = J.transpose() * W * J;
        Eigen::Matrix<double, 6, 1> bij = J.transpose() * W * r0;
        const int ii = 3 * edge.i;
        const int jj = 3 * edge.j;
        H.block<3, 3>(ii, ii) += Hij.block<3, 3>(0, 0);
        H.block<3, 3>(ii, jj) += Hij.block<3, 3>(0, 3);
        H.block<3, 3>(jj, ii) += Hij.block<3, 3>(3, 0);
        H.block<3, 3>(jj, jj) += Hij.block<3, 3>(3, 3);
        b.segment<3>(ii) += bij.segment<3>(0);
        b.segment<3>(jj) += bij.segment<3>(3);
      }

      H += Eigen::MatrixXd::Identity(3 * n, 3 * n) * 1e-6;
      Eigen::VectorXd dx = H.ldlt().solve(-b);
      if (!dx.allFinite()) break;
      double max_update = 0.0;
      for (int i = 1; i < n; ++i)
      {
        Eigen::Vector3d inc = dx.segment<3>(3 * i);
        inc.x() = std::max(-0.5, std::min(0.5, inc.x()));
        inc.y() = std::max(-0.5, std::min(0.5, inc.y()));
        inc.z() = std::max(-2.0 * M_PI / 180.0, std::min(2.0 * M_PI / 180.0, inc.z()));
        states[static_cast<size_t>(i)] += inc;
        states[static_cast<size_t>(i)].z() = wrapPi(states[static_cast<size_t>(i)].z());
        max_update = std::max(max_update, inc.norm());
      }
      if (max_update < 1e-4) break;
    }
    optimized_xy_yaw_ = states;
  }

  void publishCorrectedTrajectory(const ros::Time &stamp)
  {
    corrected_path_.poses.clear();
    corrected_path_.header.stamp = stamp;
    corrected_path_.header.frame_id = map_frame_;
    for (size_t i = 0; i < keyframes_.size(); ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = keyframes_[i].stamp;
      ps.header.frame_id = map_frame_;
      ps.pose.position.x = optimized_xy_yaw_[i].x();
      ps.pose.position.y = optimized_xy_yaw_[i].y();
      ps.pose.position.z = keyframes_[i].p.z();
      Eigen::Quaterniond q(yawToRot(optimized_xy_yaw_[i].z()));
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      corrected_path_.poses.push_back(ps);
    }
    pub_corrected_path_.publish(corrected_path_);

    if (!keyframes_.empty())
    {
      const size_t i = keyframes_.size() - 1;
      nav_msgs::Odometry odom;
      odom.header.stamp = keyframes_[i].stamp;
      odom.header.frame_id = map_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose = corrected_path_.poses.back().pose;
      pub_corrected_odom_.publish(odom);
    }
  }

  void publishDiagnostics(const ros::Time &stamp,
                          const std::string &message,
                          int candidate_id,
                          double visual_score,
                          double fitness,
                          bool loop_accepted)
  {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "visual_lidar_loop_closure";
    status.hardware_id = base_frame_;
    status.level = loop_accepted ? diagnostic_msgs::DiagnosticStatus::OK
                                 : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = message;
    auto add = [&status](const std::string &k, const std::string &v) {
      diagnostic_msgs::KeyValue kv;
      kv.key = k;
      kv.value = v;
      status.values.push_back(kv);
    };
    add("enabled", enabled_ ? "true" : "false");
    add("keyframes", std::to_string(keyframes_.size()));
    add("loop_edges", std::to_string(std::count_if(edges_.begin(), edges_.end(), [](const Edge &e) { return e.loop; })));
    add("candidate_id", std::to_string(candidate_id));
    add("visual_score", std::to_string(visual_score));
    add("ndt_fitness", std::to_string(fitness));
    add("loop_accepted", loop_accepted ? "true" : "false");
    array.status.push_back(status);
    pub_diagnostics_.publish(array);
  }

  void workerLoop()
  {
    while (ros::ok())
    {
      int index = -1;
      std::vector<Keyframe> snapshot;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_worker_ || !pending_indices_.empty(); });
        if (stop_worker_) return;
        index = pending_indices_.front();
        pending_indices_.pop_front();
        snapshot = keyframes_;
      }
      if (index < 0 || index >= static_cast<int>(snapshot.size())) continue;
      const Keyframe current = snapshot[static_cast<size_t>(index)];
      std::vector<Candidate> candidates = retrieveVisualCandidates(current, index, snapshot);
      if (candidates.empty())
      {
        updateAndPublishGraph(current.stamp);
        publishDiagnostics(current.stamp, "no_visual_candidate", -1, 0.0,
                           std::numeric_limits<double>::quiet_NaN(), false);
        continue;
      }

      bool accepted = false;
      int accepted_candidate = -1;
      double accepted_visual_score = 0.0;
      double accepted_fitness = std::numeric_limits<double>::quiet_NaN();
      Eigen::Matrix4d accepted_pose = Eigen::Matrix4d::Identity();
      std::string last_reason = "not_checked";
      for (const Candidate &candidate : candidates)
      {
        Eigen::Matrix4d verified_pose = Eigen::Matrix4d::Identity();
        double fitness = std::numeric_limits<double>::quiet_NaN();
        std::string reason;
        if (verifyCandidateByNdt(current, index, snapshot, candidate, verified_pose, fitness, reason))
        {
          const int key = candidate.index;
          if (key == last_candidate_index_)
          {
            ++consecutive_candidate_count_;
          }
          else
          {
            last_candidate_index_ = key;
            consecutive_candidate_count_ = 1;
          }
          if (consecutive_candidate_count_ >= std::max(1, require_consecutive_accepts_))
          {
            accepted = true;
            accepted_candidate = candidate.index;
            accepted_visual_score = candidate.visual_score;
            accepted_fitness = fitness;
            accepted_pose = verified_pose;
            break;
          }
          last_reason = "waiting_consecutive";
        }
        else
        {
          last_reason = reason;
        }
      }

      if (accepted)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        addLoopEdgeAndOptimize(index, accepted_candidate, accepted_pose,
                               accepted_visual_score, accepted_fitness);
        publishDiagnostics(current.stamp, "loop_accepted", accepted_candidate,
                           accepted_visual_score, accepted_fitness, true);
        ROS_WARN("[VisualLidarLoopClosure] loop accepted: current=%d candidate=%d visual=%.3f fitness=%.3f",
                 index, accepted_candidate, accepted_visual_score, accepted_fitness);
      }
      else
      {
        updateAndPublishGraph(current.stamp);
        publishDiagnostics(current.stamp, last_reason, candidates.front().index,
                           candidates.front().visual_score,
                           std::numeric_limits<double>::quiet_NaN(), false);
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_corrected_odom_;
  ros::Publisher pub_corrected_path_;
  ros::Publisher pub_diagnostics_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stop_worker_ = false;
  std::deque<int> pending_indices_;

  bool enabled_ = false;
  std::string map_frame_;
  std::string base_frame_;
  std::string image_topic_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string corrected_odom_topic_;
  std::string corrected_path_topic_;
  std::string diagnostics_topic_;

  cv::Ptr<cv::ORB> orb_;
  cv::Mat latest_gray_;
  ros::Time latest_image_stamp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
  ros::Time latest_cloud_stamp_;
  Eigen::Vector3d latest_p_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d latest_R_ = Eigen::Matrix3d::Identity();
  ros::Time latest_odom_stamp_;
  bool has_odom_ = false;

  std::vector<Keyframe> keyframes_;
  std::vector<Edge> edges_;
  std::vector<Eigen::Vector3d> optimized_xy_yaw_;
  nav_msgs::Path corrected_path_;
  int next_keyframe_id_ = 0;
  int last_candidate_index_ = -1;
  int consecutive_candidate_count_ = 0;

  double keyframe_min_translation_ = 1.0;
  double keyframe_min_rotation_deg_ = 8.0;
  double keyframe_min_interval_sec_ = 1.0;
  int exclude_recent_keyframes_ = 30;
  int max_keyframes_ = 2000;
  int orb_features_ = 800;
  int visual_top_k_ = 5;
  double min_visual_score_ = 0.08;
  double max_descriptor_distance_ = 60.0;
  double ratio_test_ = 0.75;
  int current_submap_history_width_ = 4;
  int candidate_submap_half_width_ = 8;
  double ndt_source_voxel_size_ = 0.35;
  double ndt_target_voxel_size_ = 0.25;
  int ndt_max_source_points_ = 900;
  int ndt_max_multiframe_source_points_ = 2500;
  int ndt_max_target_points_ = 25000;
  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_transformation_epsilon_ = 0.001;
  int ndt_max_iterations_ = 30;
  double accept_max_fitness_score_ = 0.8;
  double accept_max_loop_translation_ = 20.0;
  double accept_max_loop_yaw_deg_ = 45.0;
  int require_consecutive_accepts_ = 2;
  int pose_graph_iterations_ = 8;
  double pose_graph_odom_weight_ = 1.0;
  double pose_graph_loop_weight_ = 5.0;
  double pose_graph_prior_weight_ = 1000.0;
  bool publish_debug_ = true;
};

}  // namespace dog_prior_map_localization

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_lidar_loop_closure");
  dog_prior_map_localization::VisualLidarLoopClosureNode node;
  ros::spin();
  return 0;
}
