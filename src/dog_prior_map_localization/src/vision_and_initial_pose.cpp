#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

void DogPriorMapEkfNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::WallTime visual_start = ros::WallTime::now();
  ++image_msg_count_;

  // ------------------------- 相机质量门控 -------------------------
  // 当前C++低算力版暂时不做特征重投影，只快速统计过曝/欠曝比例。
  // 注意：相机质量不应该削弱LiDAR-先验地图匹配，因为地图匹配是定位精度的主约束。
  // 这里仅为后续视觉重投影/光度残差预留权重：
  // 1) 图像质量好，且LiDAR处于长廊等退化场景时，提高视觉约束权重。
  // 2) 图像过曝/欠曝时，降低视觉约束自身权重，而不是降低雷达匹配权重。
  if (!camera_enable_ || msg->data.empty()) return;

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (const cv_bridge::Exception &e)
  {
    ++visual_update_fail_count_;
    ROS_WARN_THROTTLE(2.0, "[DogPriorMap C++] image conversion failed: %s", e.what());
    return;
  }

  cv::Mat gray;
  if (cv_ptr->image.channels() == 1)
  {
    gray = cv_ptr->image;
  }
  else if (cv_ptr->image.channels() == 3)
  {
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
  }
  else if (cv_ptr->image.channels() == 4)
  {
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGRA2GRAY);
  }
  else
  {
    ++visual_update_fail_count_;
    return;
  }

  size_t over = 0;
  size_t under = 0;
  for (int y = 0; y < gray.rows; ++y)
  {
    const uint8_t *row = gray.ptr<uint8_t>(y);
    for (int x = 0; x < gray.cols; ++x)
    {
      const uint8_t v = row[x];
      if (v > 245) ++over;
      if (v < 10) ++under;
    }
  }
  const double total = static_cast<double>(gray.rows * gray.cols);
  const double over_ratio = static_cast<double>(over) / total;
  const double under_ratio = static_cast<double>(under) / total;
  image_quality_good_ = over_ratio < max_over_exposure_ratio_ && under_ratio < max_under_exposure_ratio_;

  std::vector<cv::Point2f> features;
  if (image_quality_good_)
  {
    // ------------------------- 稀疏角点检测 -------------------------
    // 使用goodFeaturesToTrack提取Shi-Tomasi角点，计算“有效角点占比”。
    // 这个占比越高，说明图像里可用于视觉定位的结构越多；长廊LiDAR退化时，视觉权重可适当提高。
    cv::goodFeaturesToTrack(gray, features, max_features_, 0.01, 8.0, cv::Mat(), 3, false, 0.04);
  }

  last_feature_ratio_ = std::min(1.0, static_cast<double>(features.size()) / static_cast<double>(std::max(1, max_features_)));

  if (!image_quality_good_ || last_feature_ratio_ < min_feature_ratio_)
  {
    visual_constraint_weight_scale_ = bad_image_weight_scale_;
  }
  else if (lidar_degeneracy_score_ >= min_degeneracy_score_for_visual_)
  {
    // ------------------------- 退化自适应视觉权重 -------------------------
    // 用户要求的思想：视觉权重与“相机有效角点占比 / 雷达退化程度”相关。
    // 这里用连续退化分数近似雷达弱约束程度，再乘以角点占比。
    // 雷达越退化、角点越丰富，视觉约束权重越接近degenerate_weight_scale。
    const double ratio = std::max(0.0, std::min(1.0, last_feature_ratio_ * lidar_degeneracy_score_));
    visual_constraint_weight_scale_ =
        good_image_weight_scale_ + (visual_degenerate_weight_scale_ - good_image_weight_scale_) * ratio;
  }
  else
  {
    visual_constraint_weight_scale_ = good_image_weight_scale_;
  }

  if (visual_feature_update_enable_ &&
      lidar_degeneracy_score_ >= min_degeneracy_score_for_visual_ &&
      image_quality_good_ &&
      !last_gray_.empty() && !last_features_.empty() &&
      has_last_image_pose_ &&
      static_cast<int>(last_features_.size()) >= min_tracked_features_)
  {
    std::vector<cv::Point2f> tracked;
    std::vector<uint8_t> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(last_gray_, gray, last_features_, tracked, status, err);

    std::vector<cv::Point2f> prev_good;
    std::vector<cv::Point2f> curr_good;
    for (size_t i = 0; i < status.size(); ++i)
    {
      if (!status[i]) continue;
      prev_good.push_back(last_features_[i]);
      curr_good.push_back(tracked[i]);
    }

    if (static_cast<int>(prev_good.size()) >= min_tracked_features_)
    {
      applyVisualYawCorrection(last_gray_, gray, prev_good, curr_good, visual_constraint_weight_scale_);
      ++visual_update_ok_count_;
    }
    else
    {
      ++visual_update_fail_count_;
    }
  }

  last_gray_ = gray.clone();
  last_features_ = features;
  last_image_R_ = R_;
  has_last_image_pose_ = true;

  const double visual_ms = (ros::WallTime::now() - visual_start).toSec() * 1000.0;
  visual_update_time_sum_ms_ += visual_ms;
  visual_update_time_max_ms_ = std::max(visual_update_time_max_ms_, visual_ms);
}

void DogPriorMapEkfNode::applyVisualYawCorrection(const cv::Mat &prev_gray,
                                                  const cv::Mat &curr_gray,
                                                  const std::vector<cv::Point2f> &prev_pts,
                                                  const std::vector<cv::Point2f> &curr_pts,
                                                  double weight_scale)
{
  (void)prev_gray;
  (void)curr_gray;

  // ------------------------- 轻量视觉角点约束 -------------------------
  // 优先使用FAST-LIVO2 ours配置里的相机内参和相机-雷达外参：
  // 1) LK跟踪得到相邻两帧2D特征对应；
  // 2) 用相机K估计Essential Matrix并recoverPose，得到相机相对旋转；
  // 3) 用R_base_camera把相机旋转转到机体系，只取yaw作为长廊退化时的弱姿态约束。
  // 单目平移尺度不可靠，所以这里仍不直接用视觉平移修正位置。
  if (prev_pts.size() < 6 || curr_pts.size() < 6) return;

  double yaw_base = std::numeric_limits<double>::quiet_NaN();
  if (camera_intrinsic_valid_ && prev_pts.size() >= 12)
  {
    cv::Mat K = (cv::Mat_<double>(3, 3) << cam_fx_, 0.0, cam_cx_,
                 0.0, cam_fy_, cam_cy_,
                 0.0, 0.0, 1.0);
    cv::Mat essential_inliers;
    cv::Mat E = cv::findEssentialMat(prev_pts, curr_pts, K, cv::RANSAC, 0.999, 1.5, essential_inliers);
    if (!E.empty())
    {
      cv::Mat R_cv;
      cv::Mat t_cv;
      const int inlier_count = cv::recoverPose(E, prev_pts, curr_pts, K, R_cv, t_cv, essential_inliers);
      if (R_cv.rows == 3 && R_cv.cols == 3 && inlier_count >= min_tracked_features_ / 2)
      {
        Eigen::Matrix3d R_cam;
        for (int r = 0; r < 3; ++r)
        {
          for (int c = 0; c < 3; ++c)
          {
            R_cam(r, c) = R_cv.at<double>(r, c);
          }
        }
        const Eigen::Matrix3d R_base_rel = R_base_camera_ * R_cam * R_base_camera_.transpose();
        yaw_base = std::atan2(R_base_rel(1, 0), R_base_rel(0, 0));
      }
    }
  }

  if (!std::isfinite(yaw_base))
  {
    cv::Mat inliers;
    cv::Mat affine = cv::estimateAffinePartial2D(prev_pts, curr_pts, inliers, cv::RANSAC, 3.0);
    if (affine.empty() || affine.rows != 2 || affine.cols != 3) return;

    const double a = affine.at<double>(0, 0);
    const double b = affine.at<double>(1, 0);
    yaw_base = std::atan2(b, a);
  }
  if (!std::isfinite(yaw_base)) return;

  const Eigen::Matrix3d R_pred_base_rel = last_image_R_.transpose() * R_;
  const double yaw_pred = std::atan2(R_pred_base_rel(1, 0), R_pred_base_rel(0, 0));
  double yaw_residual = yaw_base - yaw_pred;
  while (yaw_residual > M_PI) yaw_residual -= 2.0 * M_PI;
  while (yaw_residual < -M_PI) yaw_residual += 2.0 * M_PI;

  const double normalized_weight = std::max(0.0, std::min(1.0, weight_scale / std::max(visual_degenerate_weight_scale_, 1e-6)));
  const double yaw_correction = std::max(-max_visual_yaw_update_,
                                         std::min(max_visual_yaw_update_,
                                                  yaw_residual * normalized_weight * 0.1));
  if (std::abs(yaw_correction) < 1e-6) return;

  Eigen::Vector3d dtheta(0.0, 0.0, yaw_correction);
  applyPoseCorrection(Eigen::Vector3d::Zero(), dtheta);
}

void DogPriorMapEkfNode::externalOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!external_odom_enable_) return;

  // ------------------------- 外部位姿观测 -------------------------
  // 两种输入都走这个回调：
  // 1) FAST-LIVO2/FAST-LIO2这类前端里程计：其坐标系不一定等于先验地图，需要首帧对齐。
  // 2) 本工程拆分版NDT：/dog_livo/ndt_odom本身已经是camera_init/map下的绝对位姿，不能再首帧归零。
  Eigen::Vector3d p_ext(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
  Eigen::Quaterniond q_ext(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
  if (!p_ext.allFinite() || q_ext.norm() < 1e-9) return;
  Eigen::Matrix3d R_ext = q_ext.normalized().toRotationMatrix();

  if (!external_odom_same_map_frame_ && !has_external_odom_alignment_)
  {
    external_align_R_ = R_ * R_ext.transpose();
    external_align_p_ = p_ - external_align_R_ * p_ext;
    last_external_odom_p_map_ = p_;
    last_external_odom_time_ = msg->header.stamp.toSec();
    has_external_odom_alignment_ = true;
    ROS_INFO("[DogPriorMap C++] external LIO/LIVO prior init finished first-frame alignment.");
    return;
  }

  const Eigen::Vector3d p_target =
      external_odom_same_map_frame_ ? p_ext : (external_align_R_ * p_ext + external_align_p_);
  const Eigen::Matrix3d R_target =
      external_odom_same_map_frame_ ? R_ext : (external_align_R_ * R_ext);
  Eigen::Vector3d dp = p_target - p_;
  Eigen::AngleAxisd aa(R_target * R_.transpose());
  Eigen::Vector3d dtheta = aa.axis() * aa.angle();
  if (!dp.allFinite() || !dtheta.allFinite()) return;

  const double ratio = std::max(0.0, std::min(1.0, external_odom_apply_ratio_));
  const double z_ratio = std::max(0.0, std::min(1.0, external_odom_z_apply_ratio_));
  const double roll_pitch_ratio = std::max(0.0, std::min(1.0, external_odom_roll_pitch_apply_ratio_));
  dp.x() *= ratio;
  dp.y() *= ratio;
  dp.z() *= ratio * z_ratio;
  dtheta = limitVector(dtheta * ratio, external_odom_max_rotation_correction_);
  dtheta.x() *= roll_pitch_ratio;
  dtheta.y() *= roll_pitch_ratio;
  dp = limitVector(dp, external_odom_max_translation_correction_);
  applyPoseCorrection(dp, dtheta);
  ++lidar_update_ok_count_;
  ++icp_update_ok_count_;
  last_used_points_ = 0;
  last_mean_residual_ = dp.norm();
  publishState(msg->header.stamp, true);

  const double t = msg->header.stamp.toSec();
  const double dt = t - last_external_odom_time_;
  if (dt > 1e-3 && dt < 1.0)
  {
    // 外部轻量前端更适合作为“运动趋势/速度观测”，不要把它的累计漂移硬塞进位置状态。
    // 这里用外部前端在先验地图系下的相邻位移估计速度；位置是否修正仍由上面的apply_ratio控制。
    Eigen::Vector3d odom_velocity = (p_target - last_external_odom_p_map_) / dt;
    odom_velocity.z() *= z_ratio;
    const double blend = std::max(0.0, std::min(1.0, external_odom_velocity_blend_));
    v_ = (1.0 - blend) * v_ + blend * odom_velocity;
  }
  last_external_odom_p_map_ = p_target;
  last_external_odom_time_ = t;

  for (int i = 0; i < 3; ++i) P_(i, i) = std::max(P_(i, i) * 0.85, 1e-4);
  for (int i = 6; i < 9; ++i) P_(i, i) = std::max(P_(i, i) * 0.85, 1e-5);
}

void DogPriorMapEkfNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  R_ = q.normalized().toRotationMatrix();
  v_.setZero();
  has_last_imu_ = false;
  has_external_odom_alignment_ = false;
  has_last_lidar_odom_frame_ = false;
  if (lidar_odom_local_map_) lidar_odom_local_map_->clear();
  ROS_INFO("[DogPriorMap C++] received external initial pose and reset state.");
}

}  // namespace dog_prior_map_localization
