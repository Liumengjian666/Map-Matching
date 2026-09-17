#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

void DogPriorMapEkfNode::lidarDegeneracyCallback(const std_msgs::Float64ConstPtr &msg)
{
  if (!msg || !std::isfinite(msg->data)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  lidar_degeneracy_score_ = std::max(0.0, std::min(1.0, msg->data));
  lidar_degenerate_ = lidar_degeneracy_score_ >= min_degeneracy_score_for_visual_;
}

void DogPriorMapEkfNode::lidarInformationCallback(const std_msgs::Float64MultiArrayConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto invalidate = [this]() {
    lidar_directional_valid_ = false;
    lidar_directional_degenerate_ = false;
    lidar_information_valid_ = false;
    lidar_information_degenerate_ = false;
    lidar_information_condition_ = std::numeric_limits<double>::quiet_NaN();
    lidar_information_eigenvalues_.setConstant(std::numeric_limits<double>::quiet_NaN());
    lidar_information_eigenvectors_.setConstant(std::numeric_limits<double>::quiet_NaN());
    lidar_degenerate_projector_.setZero();
    lidar_reliable_projector_.setIdentity();
    lidar_projector_valid_ = false;
    lidar_information_stale_ = true;
  };

  if (!msg || msg->data.size() < 46)
  {
    invalidate();
    return;
  }

  const double stamp = msg->data[0];
  const bool valid = std::isfinite(stamp) && stamp > 0.0 && msg->data[1] > 0.5;
  if (!valid)
  {
    invalidate();
    return;
  }
  if (std::isfinite(lidar_information_stamp_) &&
      stamp + 1e-9 < lidar_information_stamp_)
  {
    ROS_WARN_THROTTLE(2.0, "[DogPriorMap C++] rejecting out-of-order lidar information stamp");
    return;
  }

  lidar_information_condition_ = msg->data[3];
  for (int i = 0; i < 6; ++i) lidar_information_eigenvalues_(i) = msg->data[4 + i];
  size_t offset = 10;
  for (int col = 0; col < 6; ++col)
    for (int row = 0; row < 6; ++row)
      lidar_information_eigenvectors_(row, col) = msg->data[offset++];

  const bool finite = std::isfinite(lidar_information_condition_) &&
      lidar_information_condition_ >= 1.0 &&
      lidar_information_eigenvalues_.allFinite() && lidar_information_eigenvectors_.allFinite();
  if (!finite)
  {
    invalidate();
    return;
  }

  const double max_eval = lidar_information_eigenvalues_.maxCoeff();
  const double eigen_tolerance = 1e-9 * std::max(1.0, std::abs(max_eval));
  if (!std::isfinite(max_eval) || max_eval <= eigen_tolerance)
  {
    invalidate();
    return;
  }
  for (int i = 0; i < 6; ++i)
  {
    if (lidar_information_eigenvalues_(i) < -eigen_tolerance)
    {
      invalidate();
      return;
    }
    if (i > 0 && lidar_information_eigenvalues_(i) + eigen_tolerance <
        lidar_information_eigenvalues_(i - 1))
    {
      invalidate();
      return;
    }
    const double norm = lidar_information_eigenvectors_.col(i).norm();
    if (!std::isfinite(norm) || norm < 1e-6)
    {
      invalidate();
      return;
    }
    lidar_information_eigenvectors_.col(i) /= norm;
  }
  const Eigen::Matrix<double, 6, 6> gram =
      lidar_information_eigenvectors_.transpose() * lidar_information_eigenvectors_;
  if (!gram.allFinite() || (gram - Eigen::Matrix<double, 6, 6>::Identity()).norm() > 1e-3)
  {
    invalidate();
    return;
  }

  lidar_information_stamp_ = stamp;
  lidar_information_received_ = true;
  lidar_information_valid_ = true;
  lidar_information_degenerate_ = msg->data[2] > 0.5;
  lidar_information_weak_eigenvector_ = lidar_information_eigenvectors_.col(0).normalized();
  rebuildDirectionalProjectors();
}

void DogPriorMapEkfNode::rebuildDirectionalProjectors()
{
  lidar_degenerate_projector_.setZero();
  lidar_reliable_projector_.setIdentity();
  lidar_directional_valid_ = false;
  lidar_directional_degenerate_ = false;
  lidar_projector_valid_ = false;
  if (!lidar_information_valid_) return;

  const double max_eval = lidar_information_eigenvalues_.maxCoeff();
  if (!std::isfinite(max_eval) || max_eval <= 0.0 ||
      !lidar_information_eigenvalues_.allFinite() ||
      !lidar_information_eigenvectors_.allFinite()) return;
  Eigen::Matrix<double, 6, 6> weak = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i)
  {
    const Eigen::Matrix<double, 6, 1> v = lidar_information_eigenvectors_.col(i);
    const double norm = v.norm();
    if (!std::isfinite(norm) || norm < 1e-9) return;
    if (lidar_information_eigenvalues_(i) >= 0.0 &&
        lidar_information_eigenvalues_(i) <= max_eval * directional_eigen_ratio_)
    {
      const Eigen::Matrix<double, 6, 1> vn = v / norm;
      weak.noalias() += vn * vn.transpose();
    }
  }
  weak = 0.5 * (weak + weak.transpose());
  const double weak_dimension = weak.trace();
  const Eigen::Matrix<double, 6, 6> reliable =
      Eigen::Matrix<double, 6, 6>::Identity() - weak;
  const double weak_projector_error = (weak * weak - weak).norm();
  const double reliable_projector_error = (reliable * reliable - reliable).norm();
  if (!std::isfinite(weak_dimension) || !weak.allFinite() || !reliable.allFinite() ||
      weak_projector_error > 1e-3 || reliable_projector_error > 1e-3)
    return;

  lidar_degenerate_projector_ = weak;
  lidar_reliable_projector_ = reliable;
  lidar_projector_valid_ = true;
  lidar_directional_valid_ = true;
  if (weak_dimension > 0.5)
  {
    lidar_directional_degenerate_ = true;
  }
}

void DogPriorMapEkfNode::updateLocalizationMode(bool lidar_event, bool visual_event)
{
  if (!state_machine_enable_)
  {
    localization_mode_ = LocalizationMode::NORMAL;
    return;
  }

  const bool lidar_bad = !lidar_information_received_ || lidar_information_stale_ ||
      !lidar_information_valid_ || lidar_information_degenerate_ ||
      lidar_directional_degenerate_ || lidar_degenerate_;
  const bool visual_good = last_visual_tracking_good_ && last_visual_relative_pose_valid_;
  if (lidar_event && lidar_bad)
  {
    ++lidar_degraded_count_;
    lidar_recovery_count_ = 0;
  }
  else if (lidar_event)
  {
    lidar_degraded_count_ = 0;
    ++lidar_recovery_count_;
  }
  if (visual_event && visual_good)
  {
    ++visual_good_count_;
    visual_bad_count_ = 0;
  }
  else if (visual_event)
  {
    visual_good_count_ = 0;
    ++visual_bad_count_;
  }

  switch (localization_mode_)
  {
    case LocalizationMode::NORMAL:
      if (lidar_degraded_count_ >= lidar_degraded_enter_frames_)
        localization_mode_ = LocalizationMode::LIDAR_DEGRADED;
      break;
    case LocalizationMode::LIDAR_DEGRADED:
      if (!lidar_bad && lidar_recovery_count_ >= recovery_exit_frames_)
        localization_mode_ = LocalizationMode::RECOVERY;
      else if (lidar_bad && visual_good_count_ >= visual_assisted_enter_frames_)
        localization_mode_ = LocalizationMode::VISION_ASSISTED;
      else if (lidar_bad && visual_bad_count_ >= both_degraded_enter_frames_)
        localization_mode_ = LocalizationMode::BOTH_DEGRADED;
      break;
    case LocalizationMode::VISION_ASSISTED:
      if (!lidar_bad && lidar_recovery_count_ >= recovery_exit_frames_)
        localization_mode_ = LocalizationMode::RECOVERY;
      else if (lidar_bad && visual_bad_count_ >= both_degraded_enter_frames_)
        localization_mode_ = LocalizationMode::BOTH_DEGRADED;
      break;
    case LocalizationMode::BOTH_DEGRADED:
      if (!lidar_bad && lidar_recovery_count_ >= recovery_exit_frames_)
        localization_mode_ = LocalizationMode::RECOVERY;
      else if (visual_good_count_ >= visual_assisted_enter_frames_)
        localization_mode_ = LocalizationMode::VISION_ASSISTED;
      break;
    case LocalizationMode::RECOVERY:
      if (lidar_bad && lidar_degraded_count_ >= lidar_degraded_enter_frames_)
        localization_mode_ = LocalizationMode::LIDAR_DEGRADED;
      else if (lidar_recovery_count_ >= recovery_exit_frames_)
        localization_mode_ = LocalizationMode::NORMAL;
      break;
  }
}

// 图像主回调：检查曝光与特征质量，跟踪相邻帧角点并决定是否提供视觉约束。
void DogPriorMapEkfNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (!msg) return;
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::WallTime visual_start = ros::WallTime::now();
  ++image_msg_count_;

  // Reset per-frame telemetry before any early return.  Invalid quantities are
  // represented explicitly instead of being mistaken for a zero-constraint.
  last_visual_feature_count_ = 0;
  last_visual_tracked_count_ = 0;
  last_visual_inlier_count_ = 0;
  last_visual_flow_residual_px_ = std::numeric_limits<double>::quiet_NaN();
  last_visual_flow_residual_valid_ = false;
  last_visual_reprojection_error_px_ = std::numeric_limits<double>::quiet_NaN();
  last_visual_relative_pose_.setConstant(std::numeric_limits<double>::quiet_NaN());
  last_visual_relative_pose_valid_ = false;
  last_visual_metric_translation_valid_ = false;
  last_visual_reprojection_valid_ = false;
  last_visual_covariance_valid_ = false;
  last_visual_tracking_good_ = false;
  last_visual_relative_rotation_.setIdentity();
  last_visual_translation_direction_base_.setZero();
  last_visual_translation_direction_valid_ = false;
  last_visual_translation_scale_m_ = std::numeric_limits<double>::quiet_NaN();
  last_visual_imu_rotation_residual_deg_ = std::numeric_limits<double>::quiet_NaN();
  last_visual_imu_rotation_valid_ = false;
  last_visual_covariance_diag_.setConstant(std::numeric_limits<double>::quiet_NaN());
  last_visual_update_reason_ = camera_enable_ ? "no_update" : "camera_disabled";

  // ------------------------- 相机质量门控 -------------------------
  // 当前C++低算力版暂时不做特征重投影，只快速统计过曝/欠曝比例。
  // 注意：相机质量不应该削弱LiDAR-先验地图匹配，因为地图匹配是定位精度的主约束。
  // 这里仅为后续视觉重投影/光度残差预留权重：
  // 1) 图像质量好，且LiDAR处于长廊等退化场景时，提高视觉约束权重。
  // 2) 图像过曝/欠曝时，降低视觉约束自身权重，而不是降低雷达匹配权重。
  if (!camera_enable_)
  {
    updateLocalizationMode(false, true);
    return;
  }
  if (msg->data.empty())
  {
    last_visual_update_reason_ = "empty_image";
    updateLocalizationMode(false, true);
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (const cv_bridge::Exception &e)
  {
    ++visual_update_fail_count_;
    last_visual_update_reason_ = "image_conversion_failed";
    ROS_WARN_THROTTLE(2.0, "[DogPriorMap C++] image conversion failed: %s", e.what());
    updateLocalizationMode(false, true);
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
    last_visual_update_reason_ = "unsupported_image_channels";
    updateLocalizationMode(false, true);
    return;
  }

  if (gray.empty() || gray.rows <= 0 || gray.cols <= 0)
  {
    ++visual_update_fail_count_;
    last_visual_update_reason_ = "empty_gray_image";
    updateLocalizationMode(false, true);
    return;
  }
  if (gray.depth() != CV_8U)
  {
    cv::Mat gray8;
    double min_value = 0.0;
    double max_value = 0.0;
    cv::minMaxLoc(gray, &min_value, &max_value);
    if (!std::isfinite(min_value) || !std::isfinite(max_value))
    {
      ++visual_update_fail_count_;
      last_visual_update_reason_ = "nonfinite_gray_image";
      updateLocalizationMode(false, true);
      return;
    }
    const double scale = max_value > min_value ? 255.0 / (max_value - min_value) : 1.0;
    gray.convertTo(gray8, CV_8U, scale, -min_value * scale);
    gray = gray8;
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
  last_visual_feature_count_ = static_cast<int>(features.size());

  if (!image_quality_good_ || last_feature_ratio_ < min_feature_ratio_)
  {
    visual_constraint_weight_scale_ = bad_image_weight_scale_;
    last_visual_update_reason_ = image_quality_good_ ? "feature_ratio_below_threshold" : "image_quality_bad";
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

  // The frontend remains hot in NORMAL mode: estimate a sparse track for
  // every valid image pair, but decide separately whether its pose is allowed
  // to modify the EKF.  This avoids a cold-start when LiDAR degeneracy begins.
  std::vector<cv::Point2f> prev_good;
  std::vector<cv::Point2f> curr_good;
  bool tracking_attempted = false;
  if (image_quality_good_ && !last_gray_.empty() && !last_features_.empty() && has_last_image_pose_)
  {
    tracking_attempted = true;
    std::vector<cv::Point2f> tracked;
    std::vector<uint8_t> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(last_gray_, gray, last_features_, tracked, status, err);

    const size_t track_count = std::min(status.size(),
        std::min(last_features_.size(), tracked.size()));
    for (size_t i = 0; i < track_count; ++i)
    {
      if (!status[i]) continue;
      const cv::Point2f &prev = last_features_[i];
      const cv::Point2f &curr = tracked[i];
      const bool finite = std::isfinite(prev.x) && std::isfinite(prev.y) &&
          std::isfinite(curr.x) && std::isfinite(curr.y);
      const bool in_prev = prev.x >= 0.0f && prev.x < static_cast<float>(last_gray_.cols) &&
          prev.y >= 0.0f && prev.y < static_cast<float>(last_gray_.rows);
      const bool in_curr = curr.x >= 0.0f && curr.x < static_cast<float>(gray.cols) &&
          curr.y >= 0.0f && curr.y < static_cast<float>(gray.rows);
      if (!finite || !in_prev || !in_curr) continue;
      prev_good.push_back(prev);
      curr_good.push_back(curr);
    }
    last_visual_tracked_count_ = static_cast<int>(prev_good.size());
    double flow_error_sum = 0.0;
    int flow_error_count = 0;
    for (size_t i = 0; i < track_count && i < err.size(); ++i)
    {
      if (status[i] && std::isfinite(err[i]) &&
          std::isfinite(last_features_[i].x) && std::isfinite(last_features_[i].y) &&
          std::isfinite(tracked[i].x) && std::isfinite(tracked[i].y))
      {
        flow_error_sum += static_cast<double>(err[i]);
        ++flow_error_count;
      }
    }
    if (flow_error_count > 0)
    {
      last_visual_flow_residual_px_ = flow_error_sum / static_cast<double>(flow_error_count);
      last_visual_flow_residual_valid_ = std::isfinite(last_visual_flow_residual_px_);
    }

  }

  last_visual_tracking_good_ = image_quality_good_ &&
      last_feature_ratio_ >= min_feature_ratio_ &&
      static_cast<int>(prev_good.size()) >= min_tracked_features_ &&
      (visual_max_flow_residual_px_ <= 0.0 ||
       (last_visual_flow_residual_valid_ &&
        last_visual_flow_residual_px_ <= visual_max_flow_residual_px_));
  if (!tracking_attempted)
  {
    last_visual_update_reason_ = image_quality_good_ ? "no_previous_image" : "image_quality_bad";
    ++visual_update_fail_count_;
  }
  else if (!last_visual_tracking_good_)
  {
    last_visual_update_reason_ = "insufficient_tracked_features";
    ++visual_update_fail_count_;
  }
  else
  {
    bool apply_correction = false;
    if (directional_fusion_enable_ && state_machine_enable_)
    {
      apply_correction = visual_feature_update_enable_ &&
          localization_mode_ == LocalizationMode::VISION_ASSISTED;
      if (!apply_correction && localization_mode_ == LocalizationMode::BOTH_DEGRADED)
        last_visual_update_reason_ = "both_degraded_imu_only";
      else if (!apply_correction && localization_mode_ == LocalizationMode::NORMAL)
        last_visual_update_reason_ = "normal_visual_standby";
    }
    else
    {
      apply_correction = legacy_visual_yaw_enable_ && visual_feature_update_enable_ &&
          lidar_degeneracy_score_ >= min_degeneracy_score_for_visual_;
    }
    last_visual_update_reason_ = apply_correction ? "relative_pose_estimation_attempt" :
        (visual_feature_update_enable_ ? "relative_pose_estimated_no_update" : "feature_update_disabled");
    const bool visual_pose_estimated = applyVisualYawCorrection(
        last_gray_, gray, prev_good, curr_good,
        visual_constraint_weight_scale_, apply_correction);
    if (apply_correction && visual_pose_estimated) ++visual_update_ok_count_;
    if (apply_correction && !visual_pose_estimated) ++visual_update_fail_count_;
  }

  updateVisualImuDiagnostic(msg->header.stamp);

  // Update the hysteresis controller after this frame's relative pose has
  // been estimated, so a "good" visual event means both valid tracking and a
  // valid geometric relative-pose estimate.  A mode transition takes effect
  // on the next frame, preventing a single callback from changing its own
  // gating decision halfway through processing.
  updateLocalizationMode(false, true);

  last_gray_ = gray.clone();
  last_features_ = curr_good.size() >= static_cast<size_t>(min_tracked_features_) ? curr_good : features;
  last_image_R_ = R_;
  last_image_p_ = p_;
  last_image_stamp_ = msg->header.stamp;
  has_last_image_pose_ = true;

  const double visual_ms = (ros::WallTime::now() - visual_start).toSec() * 1000.0;
  visual_update_time_sum_ms_ += visual_ms;
  visual_update_time_max_ms_ = std::max(visual_update_time_max_ms_, visual_ms);
}

// 根据特征对应估计相机相对旋转，并把受限航向残差反馈到机体姿态。
bool DogPriorMapEkfNode::applyVisualYawCorrection(const cv::Mat &prev_gray,
                                                  const cv::Mat &curr_gray,
                                                  const std::vector<cv::Point2f> &prev_pts,
                                                  const std::vector<cv::Point2f> &curr_pts,
                                                  double weight_scale,
                                                  bool apply_correction)
{
  (void)prev_gray;
  (void)curr_gray;

  // ------------------------- 轻量视觉角点约束 -------------------------
  // 优先使用FAST-LIVO2 ours配置里的相机内参和相机-雷达外参：
  // 1) LK跟踪得到相邻两帧2D特征对应；
  // 2) 用相机K估计Essential Matrix并recoverPose，得到相机相对旋转；
  // 3) 用R_base_camera把相机旋转转到机体系，只取yaw作为长廊退化时的弱姿态约束。
  // 单目平移尺度不可靠，所以这里仍不直接用视觉平移修正位置。
  if (prev_pts.size() < 6 || curr_pts.size() < 6)
  {
    last_visual_update_reason_ = "too_few_correspondences";
    return false;
  }

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
      int mask_inliers = 0;
      for (int i = 0; i < essential_inliers.rows * essential_inliers.cols; ++i)
      {
        if (essential_inliers.at<uint8_t>(i) != 0) ++mask_inliers;
      }
      last_visual_inlier_count_ = std::max(inlier_count, mask_inliers);
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
        last_visual_relative_rotation_ = R_base_rel;
        const Eigen::Vector3d t_cam(t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2));
        const Eigen::Vector3d t_base = R_base_camera_ * t_cam;
        if (t_base.allFinite() && t_base.norm() > 1e-9)
        {
          last_visual_translation_direction_base_ = t_base.normalized();
          last_visual_translation_direction_valid_ = true;
        }
        last_visual_relative_pose_.head<3>() = last_visual_translation_direction_valid_ ?
            last_visual_translation_direction_base_ : t_cam;
        // Keep the diagnostic state ordering consistent with the localization
        // state [x,y,z,roll,pitch,yaw].  The translation is a base-frame unit
        // direction until the optional IMU-norm scale diagnostic runs.
        last_visual_relative_pose_.tail<3>() = R_base_rel.eulerAngles(0, 1, 2);
        last_visual_relative_pose_valid_ = last_visual_relative_pose_.allFinite();
        // Essential-matrix translation has unknown monocular scale; the
        // optional diagnostic may fill it from short-window IMU integration.
        // A true reprojection covariance is still unavailable in this
        // lightweight tracker, so only the empirical diagonal covariance is
        // reported when explicitly enabled.
        last_visual_metric_translation_valid_ = false;
        last_visual_reprojection_valid_ = false;
        last_visual_covariance_valid_ = false;
      }
    }
  }

  if (!std::isfinite(yaw_base))
  {
    cv::Mat inliers;
    cv::Mat affine = cv::estimateAffinePartial2D(prev_pts, curr_pts, inliers, cv::RANSAC, 3.0);
    if (affine.empty() || affine.rows != 2 || affine.cols != 3)
    {
      last_visual_update_reason_ = "relative_pose_estimation_failed";
      return false;
    }

    last_visual_inlier_count_ = 0;
    double affine_error_sum = 0.0;
    int affine_error_count = 0;
    for (int i = 0; i < inliers.rows * inliers.cols && i < static_cast<int>(prev_pts.size()); ++i)
    {
      if (inliers.at<uint8_t>(i) == 0) continue;
      ++last_visual_inlier_count_;
      const double x = affine.at<double>(0, 0) * prev_pts[i].x +
                       affine.at<double>(0, 1) * prev_pts[i].y + affine.at<double>(0, 2);
      const double y = affine.at<double>(1, 0) * prev_pts[i].x +
                       affine.at<double>(1, 1) * prev_pts[i].y + affine.at<double>(1, 2);
      affine_error_sum += std::hypot(x - curr_pts[i].x, y - curr_pts[i].y);
      ++affine_error_count;
    }
    if (affine_error_count > 0)
    {
      last_visual_flow_residual_px_ = affine_error_sum / static_cast<double>(affine_error_count);
      last_visual_flow_residual_valid_ = std::isfinite(last_visual_flow_residual_px_);
    }

    const double a = affine.at<double>(0, 0);
    const double b = affine.at<double>(1, 0);
    if (!std::isfinite(a) || !std::isfinite(b))
    {
      last_visual_relative_pose_valid_ = false;
      last_visual_update_reason_ = "relative_pose_nonfinite";
      return false;
    }
    yaw_base = std::atan2(b, a);
    last_visual_relative_rotation_ =
        Eigen::AngleAxisd(yaw_base, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    last_visual_relative_pose_.setZero();
    last_visual_relative_pose_(5) = yaw_base;
    last_visual_relative_pose_valid_ = last_visual_relative_pose_.allFinite();
    last_visual_metric_translation_valid_ = false;
    last_visual_reprojection_valid_ = false;
    last_visual_covariance_valid_ = false;
  }
  if (!std::isfinite(yaw_base))
  {
    last_visual_update_reason_ = "relative_pose_nonfinite";
    return false;
  }

  const Eigen::Matrix3d R_pred_base_rel = R_.transpose() * last_image_R_;
  const double yaw_pred = std::atan2(R_pred_base_rel(1, 0), R_pred_base_rel(0, 0));
  double yaw_residual = yaw_base - yaw_pred;
  while (yaw_residual > M_PI) yaw_residual -= 2.0 * M_PI;
  while (yaw_residual < -M_PI) yaw_residual += 2.0 * M_PI;

  const double normalized_weight = std::max(0.0, std::min(1.0, weight_scale / std::max(visual_degenerate_weight_scale_, 1e-6)));
  if (!apply_correction)
  {
    last_visual_update_reason_ = "relative_pose_estimated_no_update";
    return true;
  }
  const double yaw_correction = std::max(-max_visual_yaw_update_,
                                         std::min(max_visual_yaw_update_,
                                                  yaw_residual * normalized_weight * 0.1));
  double directional_yaw_correction = yaw_correction;
  if (directional_fusion_enable_ && state_machine_enable_ &&
      localization_mode_ == LocalizationMode::VISION_ASSISTED &&
      lidar_projector_valid_ && !lidar_information_stale_)
  {
    // This stage only has a yaw estimate; do not invent metric visual
    // translation.  Retain the component of the yaw correction that lies in
    // the measured weak subspace, expressed in the same scaled coordinates
    // as the LiDAR projector.
    Eigen::Matrix<double, 6, 1> visual_correction =
        Eigen::Matrix<double, 6, 1>::Zero();
    visual_correction(5) = yaw_correction * information_rotation_scale_m_;
    visual_correction = lidar_degenerate_projector_ * visual_correction;
    directional_yaw_correction = visual_correction(5) / information_rotation_scale_m_;
  }
  if (std::abs(directional_yaw_correction) < 1e-6)
  {
    last_visual_update_reason_ = "relative_pose_below_update_threshold";
    return true;
  }

  Eigen::Vector3d dtheta(0.0, 0.0, directional_yaw_correction);
  applyPoseCorrection(Eigen::Vector3d::Zero(), dtheta);
  last_visual_update_reason_ = "yaw_only_correction_applied";
  return true;
}

void DogPriorMapEkfNode::updateVisualImuDiagnostic(const ros::Time &stamp)
{
  if (!local_vio_diagnostic_enable_ || !last_visual_relative_pose_valid_ ||
      !last_image_stamp_.isValid() || !stamp.isValid())
    return;
  const double t0 = last_image_stamp_.toSec();
  const double t1 = stamp.toSec();
  if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0) return;

  const Eigen::Matrix3d R_imu = integrateImuRotation(t0, t1);
  if (!R_imu.allFinite()) return;
  Eigen::Quaterniond q_error(last_visual_relative_rotation_.transpose() * R_imu);
  if (!q_error.coeffs().allFinite() || q_error.norm() < 1e-9) return;
  q_error.normalize();
  const double w = std::max(-1.0, std::min(1.0, std::abs(q_error.w())));
  last_visual_imu_rotation_residual_deg_ =
      2.0 * std::acos(w) * 180.0 / M_PI;
  last_visual_imu_rotation_valid_ = std::isfinite(last_visual_imu_rotation_residual_deg_);

  if (local_vio_metric_enable_ && last_visual_translation_direction_valid_)
  {
    Eigen::Matrix3d R_delta = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_delta = Eigen::Vector3d::Zero();
    if (integrateImuDelta(t0, t1, R_delta, p_delta) && p_delta.allFinite())
    {
      const double scale_m = p_delta.norm();
      const double dt = t1 - t0;
      if (std::isfinite(scale_m) && scale_m >= 1e-3 && scale_m <= 5.0 && dt <= 0.5)
      {
        last_visual_translation_scale_m_ = scale_m;
        last_visual_relative_pose_.head<3>() =
            last_visual_translation_direction_base_ * scale_m;
        last_visual_metric_translation_valid_ =
            last_visual_relative_pose_.head<3>().allFinite();
      }
    }
  }

  // Empirical diagonal covariance for the diagnostic stream only.  It uses
  // the observed KLT residual and focal length; no covariance is fed to the
  // EKF and no claim of a full VIO uncertainty model is made.
  if (last_visual_metric_translation_valid_ && last_visual_flow_residual_valid_ &&
      camera_intrinsic_valid_ && last_visual_inlier_count_ >= 6)
  {
    const double sigma_px = std::max(last_visual_flow_residual_px_, 0.5);
    const double sigma_rot = sigma_px / std::max(std::min(cam_fx_, cam_fy_), 1.0);
    const double sigma_trans = std::max(last_visual_translation_scale_m_ * sigma_rot, 0.01);
    if (std::isfinite(sigma_rot) && std::isfinite(sigma_trans))
    {
      last_visual_covariance_diag_.head<3>().setConstant(sigma_trans * sigma_trans);
      last_visual_covariance_diag_.tail<3>().setConstant(sigma_rot * sigma_rot);
      last_visual_covariance_valid_ = true;
    }
  }

  if (visual_imu_consistency_gate_enable_ && last_visual_imu_rotation_valid_ &&
      visual_imu_consistency_max_deg_ > 0.0 &&
      last_visual_imu_rotation_residual_deg_ > visual_imu_consistency_max_deg_)
  {
    // A large visual/gyro disagreement is a diagnostic failure, not a reason
    // to inject either measurement into the EKF.
    last_visual_tracking_good_ = false;
    last_visual_update_reason_ = "visual_imu_rotation_inconsistent";
  }
}

// 融合独立 NDT 节点的绝对位姿观测，同时用相邻观测估计并平滑速度。
void DogPriorMapEkfNode::ndtObservationCallback(const nav_msgs::OdometryConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ndt_observation_enable_) return;
  if (!msg) return;

  Eigen::Vector3d p_target(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  Eigen::Quaterniond q_target(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);
  if (!p_target.allFinite() || q_target.norm() < 1e-9) return;
  Eigen::Matrix3d R_target = q_target.normalized().toRotationMatrix();

  Eigen::Vector3d dp = p_target - p_;
  Eigen::AngleAxisd aa(R_target * R_.transpose());
  Eigen::Vector3d dtheta = aa.axis() * aa.angle();
  if (!dp.allFinite() || !dtheta.allFinite()) return;

  // Float64MultiArray carries the LiDAR sensor stamp in data[0].  The
  // eigensystem is usable for this observation only when that stamp is close
  // enough; an old projector must never be applied to a new pose.
  const double observation_stamp = msg->header.stamp.toSec();
  lidar_information_stale_ = !(lidar_information_received_ &&
      lidar_information_valid_ && std::isfinite(observation_stamp) &&
      std::isfinite(lidar_information_stamp_) &&
      std::abs(observation_stamp - lidar_information_stamp_) <= lidar_information_max_age_sec_ &&
      lidar_projector_valid_);
  updateLocalizationMode(true, false);

  const bool directional_mode = directional_fusion_enable_ && state_machine_enable_;
  if (directional_mode && localization_mode_ == LocalizationMode::BOTH_DEGRADED &&
      skip_updates_when_both_degraded_)
  {
    // Do not feed an untrusted absolute pose (or its differenced velocity)
    // into the EKF while both sensors are degraded.  Reset the velocity
    // differencer so the next accepted observation cannot span the outage.
    last_ndt_observation_time_ = 0.0;
    last_ndt_observation_p_map_ = p_target;
    last_mean_residual_ = 0.0;
    publishState(msg->header.stamp, false);
    return;
  }

  const double ratio = std::max(0.0, std::min(1.0, ndt_observation_apply_ratio_));
  const double z_ratio = std::max(0.0, std::min(1.0, ndt_observation_z_apply_ratio_));
  const double roll_pitch_ratio = std::max(0.0, std::min(1.0, ndt_observation_roll_pitch_apply_ratio_));
  dp.x() *= ratio;
  dp.y() *= ratio;
  dp.z() *= ratio * z_ratio;
  dtheta = limitVector(dtheta * ratio, ndt_observation_max_rotation_correction_);
  dtheta.x() *= roll_pitch_ratio;
  dtheta.y() *= roll_pitch_ratio;
  dp = limitVector(dp, ndt_observation_max_translation_correction_);

  // In a locally degenerate mode, keep the LiDAR correction in the reliable
  // eigenspace.  During RECOVERY, weak directions are reintroduced smoothly
  // instead of switching from zero to full weight on a single frame.
  const bool use_directional_lidar = directional_mode &&
      localization_mode_ != LocalizationMode::NORMAL &&
      lidar_directional_valid_ && lidar_directional_degenerate_ &&
      !lidar_information_stale_ && lidar_projector_valid_;
  if (use_directional_lidar)
  {
    double weak_weight = 0.0;
    if (localization_mode_ == LocalizationMode::RECOVERY)
    {
      const double progress = static_cast<double>(lidar_recovery_count_) /
          static_cast<double>(std::max(1, recovery_exit_frames_));
      weak_weight = recovery_weak_weight_start_ +
          (1.0 - recovery_weak_weight_start_) * std::max(0.0, std::min(1.0, progress));
    }
    Eigen::Matrix<double, 6, 1> correction;
    correction.head<3>() = dp;
    correction.tail<3>() = dtheta * information_rotation_scale_m_;
    const Eigen::Matrix<double, 6, 6> projector =
        lidar_reliable_projector_ + weak_weight * lidar_degenerate_projector_;
    correction = projector * correction;
    if (!correction.allFinite()) return;
    dp = correction.head<3>();
    dtheta = correction.tail<3>() / information_rotation_scale_m_;
  }

  applyPoseCorrection(dp, dtheta);
  ++lidar_update_ok_count_;
  ++icp_update_ok_count_;
  last_used_points_ = 0;
  last_mean_residual_ = dp.norm();
  publishState(msg->header.stamp, true);

  const double t = msg->header.stamp.toSec();
  const double dt = t - last_ndt_observation_time_;
  if (dt > 1e-3 && dt < 1.0)
  {
    Eigen::Vector3d odom_velocity = (p_target - last_ndt_observation_p_map_) / dt;
    odom_velocity.z() *= z_ratio;
    if (use_directional_lidar)
    {
      double weak_weight = 0.0;
      if (localization_mode_ == LocalizationMode::RECOVERY)
      {
        const double progress = static_cast<double>(lidar_recovery_count_) /
            static_cast<double>(std::max(1, recovery_exit_frames_));
        weak_weight = recovery_weak_weight_start_ +
            (1.0 - recovery_weak_weight_start_) * std::max(0.0, std::min(1.0, progress));
      }
      Eigen::Matrix<double, 6, 1> velocity6 = Eigen::Matrix<double, 6, 1>::Zero();
      velocity6.head<3>() = odom_velocity;
      velocity6 = (lidar_reliable_projector_ + weak_weight * lidar_degenerate_projector_) * velocity6;
      odom_velocity = velocity6.head<3>();
    }
    const double blend = std::max(0.0, std::min(1.0, ndt_observation_velocity_blend_));
    v_ = (1.0 - blend) * v_ + blend * odom_velocity;
  }
  last_ndt_observation_p_map_ = p_target;
  last_ndt_observation_time_ = t;

  for (int i = 0; i < 3; ++i) P_(i, i) = std::max(P_(i, i) * 0.85, 1e-4);
  for (int i = 6; i < 9; ++i) P_(i, i) = std::max(P_(i, i) * 0.85, 1e-5);
}

}  // namespace dog_prior_map_localization
