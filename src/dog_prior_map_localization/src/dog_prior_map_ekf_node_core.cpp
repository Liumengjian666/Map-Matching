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
  ndt_observation_topic_ = getParam<std::string>("topics/ndt_odom", "/dog_livo/ndt_odom");
  odom_high_rate_topic_ = getParam<std::string>("topics/odom_high_rate", "/dog_livo/odom_high_rate");
  imu_propagate_topic_ = getParam<std::string>("topics/imu_propagate", "/LIVO2/imu_propagate");
  odom_corrected_topic_ = getParam<std::string>("topics/odom_corrected", "/dog_livo/odom_corrected");
  path_high_rate_topic_ = getParam<std::string>("topics/path_high_rate", "/dog_livo/path_high_rate");
  path_corrected_topic_ = getParam<std::string>("topics/path_corrected", "/dog_livo/path_corrected");


  gravity_norm_ = getParam<double>("imu/gravity", 9.80665);
  initialize_gravity_from_imu_ = getParam<bool>("imu/initialize_gravity_from_imu", true);
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
  oosm_enable_ = getParam<bool>("ndt_observation/oosm_enable", false);
  oosm_max_alignment_sec_ = std::max(0.0,
      getParam<double>("ndt_observation/oosm_max_alignment_sec", 0.02));
  ndt_observation_apply_ratio_ = getParam<double>("ndt_observation/apply_ratio", 0.8);
  ndt_observation_z_apply_ratio_ = getParam<double>("ndt_observation/z_apply_ratio", 1.0);
  ndt_observation_roll_pitch_apply_ratio_ = getParam<double>("ndt_observation/roll_pitch_apply_ratio", 1.0);
  ndt_observation_max_translation_correction_ = getParam<double>("ndt_observation/max_translation_correction", 1.0);
  ndt_observation_max_rotation_correction_ = getParam<double>("ndt_observation/max_rotation_correction_deg", 5.0) * M_PI / 180.0;
  ndt_observation_velocity_blend_ = getParam<double>("ndt_observation/velocity_blend", 0.6);
  future_deferral_enable_ = getParam<bool>("ndt_observation/future_deferral_enable", false);
  future_deferral_max_sec_ = std::max(0.0, std::min(0.1,
      getParam<double>("ndt_observation/future_deferral_max_sec", 0.010)));
  future_deferral_max_queue_ = static_cast<std::size_t>(std::max(1,
      getParam<int>("ndt_observation/future_deferral_max_queue", 20)));
  deferred_csv_path_ = getParam<std::string>("output/deferred_csv_path", "");

  path_max_length_ = std::max(1, getParam<int>("output/path_max_length", 5000));
  path_sample_rate_hz_ = getParam<double>("output/path_sample_rate_hz", 2.0);
  path_publish_rate_hz_ = getParam<double>("output/path_publish_rate_hz", 1.0);
  if (!std::isfinite(path_sample_rate_hz_) || path_sample_rate_hz_ <= 0.0)
    path_sample_rate_hz_ = 2.0;
  if (!std::isfinite(path_publish_rate_hz_) || path_publish_rate_hz_ <= 0.0)
    path_publish_rate_hz_ = 1.0;
  publish_path_ = getParam<bool>("output/publish_path", false);
  publish_tf_ = getParam<bool>("output/publish_tf", true);
  print_debug_ = getParam<bool>("output/print_debug", true);
  debug_interval_sec_ = getParam<double>("output/debug_interval_sec", 2.0);
  runtime_csv_path_ = getParam<std::string>("output/runtime_csv_path", "");
  if (!runtime_csv_path_.empty())
  {
    runtime_csv_.open(runtime_csv_path_, std::ios::out);
    if (runtime_csv_.is_open())
    {
      runtime_csv_ << "stamp,imu_hz,correct_hz,avg_update_ms,max_update_ms,avg_ndt_ms,max_ndt_ms,ndt_ok_count,ndt_fail_count,last_used_points,last_mean_residual,ok_count,fail_count,rss_note,oosm_event_count,deferred_received_count,deferred_processed_count,deferred_over_limit_count,deferred_queue_full_count,max_deferred_queue_size,state_history_size,imu_history_size\n";
    }
    else
    {
      ROS_WARN("[DogPriorMap C++] failed to write runtime CSV: %s", runtime_csv_path_.c_str());
    }
  }
  oosm_csv_path_ = getParam<std::string>("output/oosm_csv_path", "");
  if (!oosm_csv_path_.empty())
  {
    oosm_csv_.open(oosm_csv_path_, std::ios::out);
    if (oosm_csv_.is_open())
    {
      oosm_csv_ << std::setprecision(17)
                << "frame_index,ndt_stamp,state_now_stamp,rollback_stamp,lag_ms,"
                   "alignment_error_ms,replay_imu_count,state_history_count,imu_history_count,oosm_result\n";
      oosm_csv_.flush();
    }
    else
    {
      ROS_WARN("[DogPriorMap C++] failed to write OOSM CSV: %s", oosm_csv_path_.c_str());
    }
  }
  if (!deferred_csv_path_.empty())
  {
    deferred_csv_.open(deferred_csv_path_, std::ios::out);
    if (deferred_csv_.is_open())
    {
      deferred_csv_ << std::setprecision(17)
                    << "event,ndt_stamp,state_now_stamp,future_lead_ms,queue_size,"
                       "wait_ms,result\n";
      deferred_csv_.flush();
    }
    else
    {
      ROS_WARN("[DogPriorMap C++] failed to write deferred CSV: %s", deferred_csv_path_.c_str());
    }
  }
  ekf_prediction_diagnostics_csv_path_ = getParam<std::string>(
      "output/ekf_prediction_diagnostics_csv_path", "");
  if (!ekf_prediction_diagnostics_csv_path_.empty())
  {
    ekf_prediction_diagnostics_csv_.open(ekf_prediction_diagnostics_csv_path_, std::ios::out);
    if (ekf_prediction_diagnostics_csv_.is_open())
    {
      ekf_prediction_diagnostics_csv_ << std::setprecision(17)
          << "event_index,event_type,event_ros_stamp,state_stamp,"
             "p_x,p_y,p_z,q_x,q_y,q_z,q_w,v_x,v_y,v_z,"
             "last_ndt_observation_time,ndt_update_count,imu_msg_count,oosm_frame_index,"
             "state_history_size,imu_history_size,state_revision,high_rate_publish_seq,"
             "ndt_measurement_stamp,state_now_before_ndt,state_now_after_replay,rollback_stamp,"
             "replay_imu_count,lag_ms,oosm_rewrite_translation_m,oosm_rewrite_rotation_deg,"
             "oosm_rewrite_velocity_mps\n";
      ekf_prediction_diagnostics_csv_.flush();
    }
    else
    {
      ROS_WARN("[DogPriorMap C++] failed to write EKF prediction diagnostics CSV: %s",
               ekf_prediction_diagnostics_csv_path_.c_str());
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

  // ------------------------- 2. ROS发布和订阅 -------------------------
  pub_high_ = nh_.advertise<nav_msgs::Odometry>(odom_high_rate_topic_, 50);
  if (publish_imu_propagate_alias_)
  {
    pub_imu_propagate_ = nh_.advertise<nav_msgs::Odometry>(imu_propagate_topic_, 50);
  }
  pub_corr_ = nh_.advertise<nav_msgs::Odometry>(odom_corrected_topic_, 20);
  pub_path_high_ = nh_.advertise<nav_msgs::Path>(path_high_rate_topic_, 5);
  pub_path_corr_ = nh_.advertise<nav_msgs::Path>(path_corrected_topic_, 5);
  path_high_.header.frame_id = map_frame_;
  path_corr_.header.frame_id = map_frame_;

  sub_imu_ = nh_.subscribe(imu_topic_, 500, &DogPriorMapEkfNode::imuCallback, this);
  if (ndt_observation_enable_)
  {
    sub_ndt_observation_ = nh_.subscribe(
        ndt_observation_topic_, 10, &DogPriorMapEkfNode::ndtObservationCallback, this);
  }
  ROS_INFO("[DogPriorMap C++] node started: external_ndt=%s, imu=%s",
           ndt_observation_enable_ ? ndt_observation_topic_.c_str() : "disabled",
           imu_topic_.c_str());
  ROS_INFO("[DogPriorMap C++] NDT OOSM=%d max_alignment=%.3f s",
           oosm_enable_ ? 1 : 0, oosm_max_alignment_sec_);
  ROS_INFO("[DogPriorMap C++] future NDT deferral=%d max=%.3f ms queue=%zu",
           future_deferral_enable_ ? 1 : 0,
           future_deferral_max_sec_ * 1000.0,
           future_deferral_max_queue_);
}

void DogPriorMapEkfNode::writeDeferredDiagnostic(const std::string &event,
                                                  double ndt_stamp,
                                                  double state_now_stamp,
                                                  double future_lead_sec,
                                                  std::size_t queue_size,
                                                  double wait_ms,
                                                  const std::string &result)
{
  if (!deferred_csv_.is_open()) return;
  deferred_csv_ << event << ","
                << ndt_stamp << ","
                << state_now_stamp << ","
                << future_lead_sec * 1000.0 << ","
                << queue_size << ","
                << wait_ms << ","
                << result << "\n";
  deferred_csv_.flush();
}

// 将外部 NDT 观测修正反馈到 estimator 状态。
void DogPriorMapEkfNode::applyPoseCorrection(const Eigen::Vector3d &dp,
                                             const Eigen::Vector3d &dtheta)
{
  ++ekf_state_revision_;
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
