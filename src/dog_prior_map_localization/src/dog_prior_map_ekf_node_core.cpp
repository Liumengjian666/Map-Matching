#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

// 构造完整定位节点：集中读取参数、初始化状态和发布订阅关系。
DogPriorMapEkfNode::DogPriorMapEkfNode() : nh_(), pnh_("~")
{
  // ------------------------- 1. 读取ROS参数 -------------------------
  // 这些参数都放在 config/dog_prior_map_localization.yaml 里，部署到机器狗时只改yaml，不改代码。
  map_frame_ = getParam<std::string>("frames/map_frame", "map");
  odom_frame_ = getParam<std::string>("frames/odom_frame", "odom");
  base_frame_ = getParam<std::string>("frames/base_frame", "base_link");

  imu_topic_ = getParam<std::string>("topics/imu", "/livox/imu");
  lidar_topic_ = getParam<std::string>("topics/lidar", "/livox/lidar");
  lidar_msg_type_ = getParam<std::string>("topics/lidar_msg_type", "livox");
  image_topic_ = getParam<std::string>("topics/image", "/image_left/image_rect");
  ndt_observation_topic_ = getParam<std::string>("topics/ndt_odom", "/dog_livo/ndt_odom");
  odom_high_rate_topic_ = getParam<std::string>("topics/odom_high_rate", "/dog_livo/odom_high_rate");
  imu_propagate_topic_ = getParam<std::string>("topics/imu_propagate", "/LIVO2/imu_propagate");
  odom_corrected_topic_ = getParam<std::string>("topics/odom_corrected", "/dog_livo/odom_corrected");
  path_high_rate_topic_ = getParam<std::string>("topics/path_high_rate", "/dog_livo/path_high_rate");
  path_corrected_topic_ = getParam<std::string>("topics/path_corrected", "/dog_livo/path_corrected");
  filtered_points_topic_ = getParam<std::string>("topics/filtered_points", "/dog_livo/filtered_points");
  prior_map_topic_ = getParam<std::string>("topics/prior_map", "/dog_livo/prior_map");
  diagnostics_topic_ = getParam<std::string>("topics/diagnostics", "/dog_livo/diagnostics");


  gravity_norm_ = getParam<double>("imu/gravity", 9.80665);
  initialize_gravity_from_imu_ = getParam<bool>("imu/initialize_gravity_from_imu", true);
  initialize_gyro_bias_from_imu_ = getParam<bool>("imu/initialize_gyro_bias_from_imu", true);
  init_imu_samples_ = std::max(1, getParam<int>("imu/init_imu_samples", 200));
  max_imu_dt_ = getParam<double>("imu/max_dt", 0.05);
  publish_high_rate_ = getParam<bool>("imu/publish_high_rate", true);
  publish_imu_propagate_alias_ = getParam<bool>("imu/publish_imu_propagate_alias", true);
  use_acc_for_position_ = getParam<bool>("imu/use_acc_for_position", false);
  velocity_damping_ = getParam<double>("imu/velocity_damping", 0.98);
  continuous_gravity_correction_enable_ = getParam<bool>("imu/continuous_gravity_correction_enable", true);
  gravity_correction_expected_acc_norm_ = getParam<double>("imu/gravity_correction_expected_acc_norm", 1.0);
  gravity_correction_gain_ = getParam<double>("imu/gravity_correction_gain", 0.01);
  gravity_correction_max_angle_ = getParam<double>("imu/gravity_correction_max_angle_deg", 0.5) * M_PI / 180.0;
  gravity_correction_acc_tolerance_ = getParam<double>("imu/gravity_correction_acc_tolerance", 1.5);
  gravity_correction_gyro_max_ = getParam<double>("imu/gravity_correction_gyro_max", 0.8);
  acc_noise_ = getParam<double>("imu/acc_noise", 2.0);
  gyro_noise_ = getParam<double>("imu/gyro_noise", 0.1);
  acc_bias_noise_ = getParam<double>("imu/acc_bias_noise", 0.0001);
  gyro_bias_noise_ = getParam<double>("imu/gyro_bias_noise", 0.0001);
  imu_history_keep_sec_ = getParam<double>("imu/history_keep_sec", 2.0);

  ndt_observation_enable_ = getParam<bool>("ndt_observation/enable", false);
  ndt_observation_fusion_mode_ = getParam<std::string>("ndt_observation/fusion_mode", "legacy_blend");
  ndt_observation_nis_threshold_ = getParam<double>("ndt_observation/nis_threshold", 22.458);
  ndt_observation_hard_reject_nis_threshold_ =
      getParam<double>("ndt_observation/hard_reject_nis_threshold", 0.0);
  ndt_observation_max_covariance_inflation_ =
      getParam<double>("ndt_observation/max_covariance_inflation", 100.0);
  ndt_observation_fixed_position_std_ =
      getParam<double>("ndt_observation/fixed_position_std_m", 0.08);
  ndt_observation_fixed_rotation_std_ =
      getParam<double>("ndt_observation/fixed_rotation_std_deg", 1.0) * M_PI / 180.0;
  ndt_observation_apply_ratio_ = getParam<double>("ndt_observation/apply_ratio", 0.8);
  ndt_observation_z_apply_ratio_ = getParam<double>("ndt_observation/z_apply_ratio", 1.0);
  ndt_observation_roll_pitch_apply_ratio_ = getParam<double>("ndt_observation/roll_pitch_apply_ratio", 1.0);
  ndt_observation_max_translation_correction_ = getParam<double>("ndt_observation/max_translation_correction", 1.0);
  ndt_observation_max_rotation_correction_ = getParam<double>("ndt_observation/max_rotation_correction_deg", 5.0) * M_PI / 180.0;
  ndt_observation_velocity_blend_ = getParam<double>("ndt_observation/velocity_blend", 0.6);
  lidar_deskew_enable_ = getParam<bool>("lidar_update/deskew_enable", true);
  lidar_deskew_translation_enable_ = getParam<bool>("lidar_update/deskew_translation_enable", true);
  lidar_offset_time_scale_ = getParam<double>("lidar_update/offset_time_scale", 1e-9);


  lidar_enable_ = getParam<bool>("lidar_update/enable", true);
  prior_map_update_enable_ = getParam<bool>("lidar_update/prior_map_update_enable", true);
  registration_method_ = getParam<std::string>("lidar_update/registration_method", "point_to_plane");
  update_every_n_scans_ = std::max(1, getParam<int>("lidar_update/update_every_n_scans", 1));
  scan_voxel_size_ = getParam<double>("lidar_update/scan_voxel_size", 0.25);
  max_scan_points_ = getParam<int>("lidar_update/max_scan_points", 2500);
  scan_min_range_ = getParam<double>("lidar_update/scan_min_range", 0.5);
  scan_max_range_ = getParam<double>("lidar_update/scan_max_range", 80.0);
  scan_min_z_ = getParam<double>("lidar_update/scan_min_z", -std::numeric_limits<double>::infinity());
  scan_max_z_ = getParam<double>("lidar_update/scan_max_z", std::numeric_limits<double>::infinity());
  scan_radius_outlier_enable_ = getParam<bool>("lidar_update/scan_radius_outlier_enable", false);
  scan_radius_outlier_radius_ = getParam<double>("lidar_update/scan_radius_outlier_radius", 0.35);
  scan_radius_outlier_min_neighbors_ = std::max(1, getParam<int>("lidar_update/scan_radius_outlier_min_neighbors", 2));
  max_match_distance_ = getParam<double>("lidar_update/max_match_distance", 1.0);
  min_effective_points_ = getParam<int>("lidar_update/min_effective_points", 80);
  point_noise_ = getParam<double>("lidar_update/point_to_point_noise", 0.20);
  hybrid_point_noise_ = getParam<double>("lidar_update/hybrid_point_noise", 0.60);
  plane_noise_ = getParam<double>("lidar_update/point_to_plane_noise", 0.12);
  plane_neighbor_k_ = std::max(3, getParam<int>("lidar_update/plane_neighbor_k", 5));
  plane_min_eigen_ratio_ = getParam<double>("lidar_update/plane_min_eigen_ratio", 0.08);
  fastlio_plane_threshold_ = getParam<double>("lidar_update/fastlio_plane_threshold", 0.10);
  fastlio_residual_gate_ = getParam<double>("lidar_update/fastlio_residual_gate", 0.90);
  fastlivo_outlier_reject_enable_ = getParam<bool>("lidar_update/fastlivo_outlier_reject_enable", true);
  plane_radius_gate_scale_ = getParam<double>("lidar_update/plane_radius_gate_scale", 3.0);
  plane_sigma_gate_ = getParam<double>("lidar_update/plane_sigma_gate", 3.0);
  plane_min_radius_ = getParam<double>("lidar_update/plane_min_radius", 0.05);
  plane_min_sigma_ = getParam<double>("lidar_update/plane_min_sigma", 0.03);
  range_noise_per_meter_ = getParam<double>("lidar_update/range_noise_per_meter", 0.003);
  fastlivo_body_cov_enable_ = getParam<bool>("lidar_update/fastlivo_body_cov_enable", true);
  lidar_depth_noise_ = getParam<double>("lidar_update/lidar_depth_noise", 0.05);
  lidar_beam_noise_deg_ = getParam<double>("lidar_update/lidar_beam_noise_deg", 0.02);
  horizontal_plane_z_observation_enable_ = getParam<bool>("lidar_update/horizontal_plane_z_observation_enable", true);
  horizontal_plane_normal_z_min_ = getParam<double>("lidar_update/horizontal_plane_normal_z_min", 0.85);
  horizontal_plane_weight_scale_ = getParam<double>("lidar_update/horizontal_plane_weight_scale", 2.0);
  horizontal_plane_residual_gate_ = getParam<double>("lidar_update/horizontal_plane_residual_gate", 0.35);
  huber_threshold_ = getParam<double>("lidar_update/huber_threshold", 0.6);
  max_iterations_ = std::max(1, getParam<int>("lidar_update/max_iterations", 3));
  max_translation_update_ = getParam<double>("lidar_update/max_translation_update", 0.25);
  max_rotation_update_ = getParam<double>("lidar_update/max_rotation_update_deg", 3.0) * M_PI / 180.0;
  max_update_time_ms_ = getParam<double>("lidar_update/max_update_time_ms", 25.0);
  match_accept_gate_enable_ = getParam<bool>("lidar_update/match_accept_gate_enable", true);
  match_accept_max_mean_residual_ = getParam<double>("lidar_update/match_accept_max_mean_residual", 0.20);
  match_accept_max_residual_increase_ratio_ = getParam<double>("lidar_update/match_accept_max_residual_increase_ratio", 1.02);
  match_accept_min_improvement_ratio_ = getParam<double>("lidar_update/match_accept_min_improvement_ratio", 0.0);
  match_accept_min_effective_ratio_ = getParam<double>("lidar_update/match_accept_min_effective_ratio", 0.10);
  match_accept_max_translation_ = getParam<double>("lidar_update/match_accept_max_translation", max_translation_update_);
  match_accept_max_rotation_ = getParam<double>("lidar_update/match_accept_max_rotation_deg", max_rotation_update_ * 180.0 / M_PI) * M_PI / 180.0;
  match_accept_degenerate_max_translation_ =
      getParam<double>("lidar_update/match_accept_degenerate_max_translation", 0.12);
  match_accept_degenerate_max_rotation_ =
      getParam<double>("lidar_update/match_accept_degenerate_max_rotation_deg", 0.8) * M_PI / 180.0;
  local_radius_ = getParam<double>("map/local_radius", 18.0);
  local_radius_max_ = std::max(local_radius_, getParam<double>("map/local_radius_max", local_radius_));
  local_radius_step_ = std::max(0.5, getParam<double>("map/local_radius_step", 2.0));
  local_submap_enable_ = getParam<bool>("map/local_submap_enable", true);
  local_submap_max_points_ = std::max(500, getParam<int>("map/local_submap_max_points", 12000));
  local_submap_min_points_ = std::max(50, getParam<int>("map/local_submap_min_points", 500));
  ndt_source_voxel_size_ = getParam<double>("lidar_update/ndt_source_voxel_size", 0.35);
  ndt_target_voxel_size_ = getParam<double>("lidar_update/ndt_target_voxel_size", 0.30);
  ndt_max_source_points_ = std::max(50, getParam<int>("lidar_update/ndt_max_source_points", 900));
  ndt_max_target_points_ = getParam<int>("lidar_update/ndt_max_target_points", 25000);
  ndt_max_iterations_ = std::max(1, getParam<int>("lidar_update/ndt_max_iterations", 15));
  ndt_resolution_ = getParam<double>("lidar_update/ndt_resolution", 1.0);
  ndt_step_size_ = getParam<double>("lidar_update/ndt_step_size", 0.1);
  ndt_transformation_epsilon_ = getParam<double>("lidar_update/ndt_transformation_epsilon", 0.01);
  ndt_max_fitness_score_ = getParam<double>("lidar_update/ndt_max_fitness_score", 0.6);
  ndt_accept_max_translation_ = getParam<double>("lidar_update/ndt_accept_max_translation", 0.6);
  ndt_accept_max_rotation_ = getParam<double>("lidar_update/ndt_accept_max_rotation_deg", 8.0) * M_PI / 180.0;
  ndt_absolute_pose_mode_ = getParam<bool>("lidar_update/ndt_absolute_pose_mode", false);
  ndt_use_full_map_target_ = getParam<bool>("lidar_update/ndt_use_full_map_target", false);
  degeneracy_check_enable_ = getParam<bool>("lidar_update/degeneracy_check_enable", true);
  degeneracy_min_eigenvalue_ = getParam<double>("lidar_update/degeneracy_min_eigenvalue", 1e-3);
  degeneracy_max_condition_number_ = getParam<double>("lidar_update/degeneracy_max_condition_number", 1e5);
  degeneracy_geometry_ratio_ = getParam<double>("lidar_update/degeneracy_geometry_ratio", 0.08);
  degeneracy_project_update_enable_ = getParam<bool>("lidar_update/degeneracy_project_update_enable", true);
  degeneracy_project_eigen_ratio_ = getParam<double>("lidar_update/degeneracy_project_eigen_ratio", 0.03);
  degeneracy_project_min_scale_ = getParam<double>("lidar_update/degeneracy_project_min_scale", 0.10);
  camera_enable_ = getParam<bool>("camera_update/enable", true);
  visual_feature_update_enable_ = getParam<bool>("camera_update/feature_update_enable", true);
  max_over_exposure_ratio_ = getParam<double>("camera_update/max_over_exposure_ratio", 0.25);
  max_under_exposure_ratio_ = getParam<double>("camera_update/max_under_exposure_ratio", 0.35);
  max_features_ = std::max(20, getParam<int>("camera_update/max_features", 300));
  min_tracked_features_ = std::max(5, getParam<int>("camera_update/min_tracked_features", 40));
  min_feature_ratio_ = getParam<double>("camera_update/min_feature_ratio", 0.15);
  max_visual_yaw_update_ = getParam<double>("camera_update/max_yaw_update_deg", 0.5) * M_PI / 180.0;
  good_image_weight_scale_ = getParam<double>("camera_update/good_image_weight_scale", 1.0);
  bad_image_weight_scale_ = getParam<double>("camera_update/bad_image_weight_scale", 0.4);
  visual_degenerate_weight_scale_ = getParam<double>("camera_update/degenerate_weight_scale", 2.0);
  min_degeneracy_score_for_visual_ = getParam<double>("camera_update/min_degeneracy_score_for_visual", 0.15);
  cam_fx_ = getParam<double>("camera_intrinsic/fx", 0.0);
  cam_fy_ = getParam<double>("camera_intrinsic/fy", 0.0);
  cam_cx_ = getParam<double>("camera_intrinsic/cx", 0.0);
  cam_cy_ = getParam<double>("camera_intrinsic/cy", 0.0);
  camera_intrinsic_valid_ = cam_fx_ > 1.0 && cam_fy_ > 1.0;

  path_max_length_ = getParam<int>("output/path_max_length", 5000);
  publish_path_ = getParam<bool>("output/publish_path", false);
  publish_tf_ = getParam<bool>("output/publish_tf", true);
  publish_filtered_points_ = getParam<bool>("output/publish_filtered_points", true);
  publish_diagnostics_ = getParam<bool>("output/publish_diagnostics", true);
  print_debug_ = getParam<bool>("output/print_debug", true);
  debug_interval_sec_ = getParam<double>("output/debug_interval_sec", 2.0);
  runtime_csv_path_ = getParam<std::string>("output/runtime_csv_path", "");
  if (!runtime_csv_path_.empty())
  {
    runtime_csv_.open(runtime_csv_path_, std::ios::out);
    if (runtime_csv_.is_open())
    {
      runtime_csv_ << "stamp,imu_hz,lidar_hz,correct_hz,avg_update_ms,max_update_ms,avg_ndt_ms,max_ndt_ms,ndt_ok_count,ndt_fail_count,last_used_points,last_mean_residual,ok_count,fail_count,image_hz,avg_visual_ms,max_visual_ms,last_feature_ratio,visual_weight,lidar_degenerate,lidar_degeneracy_score,visual_ok_count,visual_fail_count,rss_note\n";
    }
    else
    {
      ROS_WARN("[DogPriorMap C++] failed to write runtime CSV: %s", runtime_csv_path_.c_str());
    }
  }

  p_.setZero();
  v_.setZero();
  R_.setIdentity();
  ba_.setZero();
  bg_.setZero();
  g_ = Eigen::Vector3d(0.0, 0.0, -gravity_norm_);

  const double init_pos_std = getParam<double>("filter/init_pos_std", 0.5);
  const double init_rot_std = getParam<double>("filter/init_rot_std_deg", 5.0) * M_PI / 180.0;
  const double init_vel_std = getParam<double>("filter/init_vel_std", 0.5);
  const double init_bias_std = getParam<double>("filter/init_bias_std", 0.05);
  P_.setZero();
  for (int i = 0; i < 3; ++i) P_(i, i) = init_pos_std * init_pos_std;
  for (int i = 3; i < 6; ++i) P_(i, i) = init_vel_std * init_vel_std;
  for (int i = 6; i < 9; ++i) P_(i, i) = init_rot_std * init_rot_std;
  for (int i = 9; i < 15; ++i) P_(i, i) = init_bias_std * init_bias_std;

  std::vector<double> t_bl = getParamVec("extrinsic/T_base_lidar", {0.0, 0.0, 0.0});
  std::vector<double> r_bl = getParamVec("extrinsic/R_base_lidar", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  T_base_lidar_ = Eigen::Vector3d(t_bl[0], t_bl[1], t_bl[2]);
  R_base_lidar_ << r_bl[0], r_bl[1], r_bl[2],
                   r_bl[3], r_bl[4], r_bl[5],
                   r_bl[6], r_bl[7], r_bl[8];
  std::vector<double> t_bc = getParamVec("extrinsic/T_base_camera", {0.0, 0.0, 0.0});
  std::vector<double> r_bc = getParamVec("extrinsic/R_base_camera", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  T_base_camera_ = Eigen::Vector3d(t_bc[0], t_bc[1], t_bc[2]);
  R_base_camera_ << r_bc[0], r_bc[1], r_bc[2],
                    r_bc[3], r_bc[4], r_bc[5],
                    r_bc[6], r_bc[7], r_bc[8];
  ROS_INFO("[DogPriorMap C++] camera intrinsic fx/fy/cx/cy=%.3f/%.3f/%.3f/%.3f valid=%d",
           cam_fx_, cam_fy_, cam_cx_, cam_cy_, camera_intrinsic_valid_ ? 1 : 0);
  ROS_INFO("[DogPriorMap C++] T_base_camera=[%.4f %.4f %.4f]",
           T_base_camera_.x(), T_base_camera_.y(), T_base_camera_.z());

  // 拆分模式只接收独立 NDT 节点的位姿观测，不重复持有地图和 KDTree。
  // 一体化模式仍由本节点加载地图，保持原有运行方式不变。
  if (lidar_enable_)
  {
    loadPriorMap();
  }

  // ------------------------- 3. ROS发布和订阅 -------------------------
  pub_high_ = nh_.advertise<nav_msgs::Odometry>(odom_high_rate_topic_, 50);
  if (publish_imu_propagate_alias_)
  {
    pub_imu_propagate_ = nh_.advertise<nav_msgs::Odometry>(imu_propagate_topic_, 50);
  }
  pub_corr_ = nh_.advertise<nav_msgs::Odometry>(odom_corrected_topic_, 20);
  pub_path_high_ = nh_.advertise<nav_msgs::Path>(path_high_rate_topic_, 5);
  pub_path_corr_ = nh_.advertise<nav_msgs::Path>(path_corrected_topic_, 5);
  if (lidar_enable_)
  {
    pub_prior_map_ = nh_.advertise<sensor_msgs::PointCloud2>(prior_map_topic_, 1, true);
  }
  if (publish_filtered_points_)
  {
    pub_filtered_points_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 5);
  }
  if (publish_diagnostics_)
  {
    pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
  }

  if (map_cloud_ && !map_cloud_->empty() && pub_prior_map_)
  {
    sensor_msgs::PointCloud2 prior_map_msg;
    pcl::toROSMsg(*map_cloud_, prior_map_msg);
    prior_map_msg.header.stamp = ros::Time::now();
    prior_map_msg.header.frame_id = map_frame_;
    pub_prior_map_.publish(prior_map_msg);
  }
  path_high_.header.frame_id = map_frame_;
  path_corr_.header.frame_id = map_frame_;

  sub_imu_ = nh_.subscribe(imu_topic_, 500, &DogPriorMapEkfNode::imuCallback, this);
  if (ndt_observation_enable_)
  {
    sub_ndt_observation_ = nh_.subscribe(
        ndt_observation_topic_, 10, &DogPriorMapEkfNode::ndtObservationCallback, this);
  }
  if (lidar_enable_)
  {
    if (lidar_msg_type_ == "pointcloud2")
    {
      sub_pc2_ = nh_.subscribe(lidar_topic_, 5, &DogPriorMapEkfNode::pointCloud2Callback, this);
    }
    else
    {
      sub_livox_ = nh_.subscribe(lidar_topic_, 5, &DogPriorMapEkfNode::livoxCallback, this);
    }
  }
  if (camera_enable_)
  {
    sub_image_ = nh_.subscribe(image_topic_, 2, &DogPriorMapEkfNode::imageCallback, this);
  }
  ROS_INFO("[DogPriorMap C++] node started: map=%zu pts, imu=%s, lidar=%s",
           map_cloud_ ? map_cloud_->size() : 0, imu_topic_.c_str(), lidar_topic_.c_str());
}

// 读取浮点数组参数；兼容全局和私有命名空间，并提供安全默认值。
std::vector<double> DogPriorMapEkfNode::getParamVec(const std::string &name, const std::vector<double> &default_value)
{
  std::vector<double> value;
  if (nh_.getParam(name, value)) return value;
  if (pnh_.getParam(name, value)) return value;
  return default_value;
}

}  // namespace dog_prior_map_localization
