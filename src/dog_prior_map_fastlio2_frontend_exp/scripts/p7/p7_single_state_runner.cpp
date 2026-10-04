#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Clock = std::chrono::steady_clock;
double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
uint64_t unsignedArgument(const char* argument) {
  const std::string value(argument);
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("invalid_unsigned_argument");
  return std::stoull(value);
}
std::array<double, 16> matrixArgument(const char* argument) {
  std::array<double, 16> values{};
  std::stringstream input(argument);
  std::string field;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (!std::getline(input, field, ','))
      throw std::runtime_error("map_T_imu_matrix_requires_16_comma_separated_values");
    std::size_t parsed = 0;
    values[index] = std::stod(field, &parsed);
    if (parsed != field.size() || !std::isfinite(values[index]))
      throw std::runtime_error("invalid_map_T_imu_matrix_value");
  }
  if (std::getline(input, field, ','))
    throw std::runtime_error("map_T_imu_matrix_has_extra_values");
  return values;
}
Eigen::Vector3d vectorArgument(const char* argument, const char* name) {
  const std::string value(argument);
  std::stringstream input(value);
  std::string field;
  Eigen::Vector3d result;
  for (int index = 0; index < 3; ++index) {
    if (!std::getline(input, field, ','))
      throw std::runtime_error(std::string(name) + "_requires_3_comma_separated_values");
    std::size_t parsed = 0;
    result[index] = std::stod(field, &parsed);
    if (parsed != field.size() || !std::isfinite(result[index]))
      throw std::runtime_error(std::string("invalid_") + name + "_value");
  }
  if (std::getline(input, field, ','))
    throw std::runtime_error(std::string(name) + "_has_extra_values");
  return result;
}
double scalarArgument(const char* argument, const char* name) {
  const std::string value(argument);
  std::size_t parsed = 0;
  const double result = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(result))
    throw std::runtime_error(std::string("invalid_") + name);
  return result;
}
paper::Pose3d poseFromMatrix(const std::array<double, 16>& values) {
  Eigen::Matrix4d matrix;
  for (std::size_t row = 0; row < 4; ++row)
    for (std::size_t column = 0; column < 4; ++column)
      matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          values[row * 4 + column];
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  if ((matrix.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-10 ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
      std::abs(rotation.determinant() - 1.0) > 1e-5)
    throw std::runtime_error("map_T_imu_matrix_is_not_rigid_transform");
  paper::Pose3d pose;
  pose.position = matrix.block<3, 1>(0, 3);
  pose.orientation = Eigen::Quaterniond(rotation).normalized();
  return pose;
}
double rotationDistance(const paper::Pose3d& first, const paper::Pose3d& second) {
  const Eigen::Quaterniond relative =
      first.orientation.normalized().conjugate() * second.orientation.normalized();
  return Eigen::AngleAxisd(relative.normalized()).angle();
}
Eigen::Vector3d rpy(const paper::Pose3d& pose) {
  const Eigen::Vector3d yaw_pitch_roll =
      pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
  return Eigen::Vector3d(yaw_pitch_roll.z(), yaw_pitch_roll.y(), yaw_pitch_roll.x());
}
paper::P7ImuVector initializationSamples(const paper::P7ImuVector& all,
                                        int sample_count, uint64_t stamp) {
  if (all.size() < static_cast<std::size_t>(sample_count))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  if (stamp == 0) return paper::P7ImuVector(all.begin(), all.begin() + sample_count);
  auto end = std::lower_bound(all.begin(), all.end(), stamp,
      [](const paper::ImuSample& sample, uint64_t time) { return sample.stamp_ns < time; });
  if (end != all.end() && end->stamp_ns == stamp) ++end;
  if (std::distance(all.begin(), end) < sample_count)
    throw std::runtime_error("insufficient_causal_initialization_imu");
  paper::P7ImuVector samples(end - sample_count, end);
  const uint64_t gap = stamp - samples.back().stamp_ns;
  if (gap != 0) {
    std::vector<uint64_t> periods;
    const std::size_t first = samples.size() > 101 ? samples.size() - 101 : 1;
    for (std::size_t i = first; i < samples.size(); ++i)
      periods.push_back(samples[i].stamp_ns - samples[i - 1].stamp_ns);
    std::sort(periods.begin(), periods.end());
    const uint64_t median = periods.at(periods.size() / 2);
    if (median == 0 || gap > 2 * median)
      throw std::runtime_error("initialization_epoch_extrapolation_exceeds_two_median_imu_periods");
  }
  return samples;
}
void poseColumns(std::ostream& out, const paper::Pose3d& pose) {
  out << ',' << pose.position.x() << ',' << pose.position.y() << ',' << pose.position.z()
      << ',' << pose.orientation.x() << ',' << pose.orientation.y() << ','
      << pose.orientation.z() << ',' << pose.orientation.w();
}
void vectorColumns(std::ostream& out, const Eigen::Vector3d& vector) {
  out << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
}
}  // namespace

int main(int argc, char** argv) {
  uint64_t transaction = 0;
  try {
    if (argc != 12 && argc != 17 && argc != 18 && argc != 24 && argc != 26)
      throw std::runtime_error("usage: p7_single_state_runner IMU_CSV FILTER_SCANS_CSV "
          "RAW_TIMED_SCAN_INDEX_CSV RAW_TIMED_POINTS_BIN MAP_PCD PARAMS_TXT "
          "TRAJECTORY_CSV REGISTRATION_CSV RUNTIME_CSV FRAME_LIMIT INITIALIZATION_STAMP_NS "
          "[--official-pose-reanchor CALIBRATION_START_NS CALIBRATION_END_NS "
          "FIRST_TRANSACTION_ID MAP_T_IMU_ROW_MAJOR_16_VALUES] "
          "[--dataset-contract-reanchor CALIBRATION_START_NS CALIBRATION_END_NS "
          "FIRST_TRANSACTION_ID MAP_T_IMU_ROW_MAJOR_16_VALUES T_IMU_LIDAR_ROW_MAJOR_16_VALUES "
          "(gravity is transported from the static calibration using causal gyro data)] "
          "[--dataset-state-replay BASELINE|DATASET_VELOCITY CALIBRATION_START_NS "
          "CALIBRATION_END_NS FIRST_TRANSACTION_ID MAP_T_IMU_ROW_MAJOR_16_VALUES "
          "VELOCITY_WORLD_3 GYRO_BIAS_3 ACCEL_BIAS_3 VELOCITY_STD GYRO_BIAS_STD "
          "ACCEL_BIAS_STD [--initial-gravity-map GRAVITY_MAP_3]]");
    const auto all_imu = paper::readP7Imu(argv[1]);
    const auto scans = paper::readP7TimedScans(argv[2], argv[3]);
    const uint64_t limit = unsignedArgument(argv[10]);
    const uint64_t initialization_stamp = unsignedArgument(argv[11]);
    const bool dataset_state_replay = argc == 24 || argc == 26;
    const bool dataset_gravity_override = argc == 26;
    const bool dataset_contract_reanchor = argc == 18 || dataset_state_replay;
    const bool official_reanchor = argc == 17 || dataset_contract_reanchor;
    const std::string startup_mode = argc > 12 ? argv[12] : "";
    if (official_reanchor && startup_mode !=
            (dataset_state_replay ? "--dataset-state-replay" :
             dataset_contract_reanchor ? "--dataset-contract-reanchor" :
                                         "--official-pose-reanchor"))
      throw std::runtime_error("unknown_startup_mode");
    const std::string state_profile = dataset_state_replay ? argv[13] : "";
    if (dataset_state_replay && state_profile != "BASELINE" &&
        state_profile != "DATASET_VELOCITY")
      throw std::runtime_error("unknown_dataset_state_profile");
    if (dataset_gravity_override &&
        (state_profile != "DATASET_VELOCITY" ||
         std::string(argv[24]) != "--initial-gravity-map"))
      throw std::runtime_error("invalid_dataset_gravity_override_arguments");
    paper::Pose3d initial_lidar, extrinsic;
    const auto parameters = paper::readP7Parameters(argv[6], &initial_lidar, &extrinsic);
    if (dataset_contract_reanchor)
      if (!dataset_state_replay)
        extrinsic = poseFromMatrix(matrixArgument(argv[17]));
    paper::FastLio2IkfomFrontend frontend(parameters);
    std::string reason;
    uint64_t first_transaction_id = 1;
    if (official_reanchor) {
      const int calibration_start_arg = dataset_state_replay ? 14 : 13;
      const int first_transaction_arg = dataset_state_replay ? 16 : 15;
      const int pose_arg = dataset_state_replay ? 17 : 16;
      const uint64_t calibration_start = unsignedArgument(argv[calibration_start_arg]);
      const uint64_t calibration_end = unsignedArgument(argv[calibration_start_arg + 1]);
      first_transaction_id = unsignedArgument(argv[first_transaction_arg]);
      const paper::Pose3d official_map_T_imu = poseFromMatrix(matrixArgument(argv[pose_arg]));
      if (calibration_start == 0 || calibration_end <= calibration_start ||
          calibration_end >= initialization_stamp || first_transaction_id == 0)
        throw std::runtime_error("invalid_official_reanchor_epoch_contract");
      const auto calibration_samples = initializationSamples(
          all_imu, parameters.static_init_samples, calibration_end);
      if (calibration_samples.front().stamp_ns != calibration_start ||
          calibration_samples.back().stamp_ns != calibration_end)
        throw std::runtime_error("static_window_does_not_match_frozen_manifest");
      paper::StaticImuCalibration calibration;
      if (!frontend.calibrateStaticImu(calibration_samples, &calibration, &reason))
        throw std::runtime_error("static_imu_calibration_failed:" + reason);
      Eigen::Matrix3d R_calibration_to_anchor = Eigen::Matrix3d::Identity();
      Eigen::Vector3d gravity_map;
      if (!paper::integrateBodyRelativeRotation(all_imu, calibration.end_stamp_ns,
              initialization_stamp, calibration.gyro_bias,
              &R_calibration_to_anchor, &reason))
        throw std::runtime_error("gravity_attitude_transfer_failed:" + reason);
      // Transport only the static specific-force direction from the calibration
      // epoch to the official anchor. Navigation position and velocity are
      // freshly initialized below; no 61 s inertial pose propagation occurs.
      const Eigen::Matrix3d R_map_imu_calibration =
          official_map_T_imu.orientation.toRotationMatrix() *
          R_calibration_to_anchor.transpose();
      gravity_map = -R_map_imu_calibration *
          calibration.mean_specific_force.normalized() * calibration.gravity_mps2;
      const Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();
      paper::InitialStateOverrides initial_state;
      if (dataset_state_replay && state_profile == "DATASET_VELOCITY") {
        initial_state.use_initial_velocity = true;
        initial_state.velocity_world_m_s = vectorArgument(argv[18], "initial_velocity");
        initial_state.use_initial_biases = true;
        initial_state.gyro_bias_rad_s = vectorArgument(argv[19], "initial_gyro_bias");
        initial_state.accel_bias_m_s2 = vectorArgument(argv[20], "initial_accel_bias");
        initial_state.use_covariance_overrides = true;
        initial_state.velocity_std_m_s = scalarArgument(argv[21], "velocity_std");
        initial_state.gyro_bias_std_rad_s = scalarArgument(argv[22], "gyro_bias_std");
        initial_state.accel_bias_std_m_s2 = scalarArgument(argv[23], "accel_bias_std");
      }
      if (dataset_gravity_override) {
        initial_state.use_initial_gravity = true;
        initial_state.gravity_map_m_s2 = vectorArgument(argv[25], "initial_gravity_map");
      }
      const bool initialized = dataset_state_replay
          ? frontend.initializeFromStaticCalibration(calibration, official_map_T_imu,
                extrinsic, gravity_map, initial_state, initialization_stamp, &reason)
          : frontend.initializeFromStaticCalibration(calibration, official_map_T_imu,
                extrinsic, gravity_map, zero_velocity, initialization_stamp, &reason);
      if (!initialized)
        throw std::runtime_error("official_pose_reanchor_failed:" + reason);
      std::cout << std::setprecision(12)
          << "STARTUP_MODE=OFFICIAL_POSE_REANCHOR\n"
          << "FRAME_CONTRACT_MODE=" << dataset_contract_reanchor << '\n'
          << "DATASET_STATE_PROFILE=" << (dataset_state_replay ? state_profile : "LEGACY") << '\n'
          << "STATIC_WINDOW=" << calibration.start_stamp_ns << ','
          << calibration.end_stamp_ns << " samples=" << calibration.sample_count << '\n'
          << "STATIC_GATE=PASS accel_std=" << calibration.acceleration_std.transpose()
          << " gyro_std=" << calibration.gyro_std.transpose() << '\n'
          << "CALIBRATION gyro_bias=" << calibration.gyro_bias.transpose()
          << " accel_bias_prior=" << calibration.accel_bias_prior.transpose()
          << " mean_specific_force=" << calibration.mean_specific_force.transpose()
          << " gravity_magnitude=" << calibration.gravity_mps2 << '\n'
          << "GRAVITY_INITIALIZATION="
          << (dataset_gravity_override ? "DATASET_SPECIFIC_MAP_CONSTANT" :
                                         "STATIC_FORCE_CAUSAL_GYRO_TRANSPORT")
          << " gravity_map=" << frontend.getState().gravity.transpose() << '\n'
          << "T_IMU_LIDAR translation=" << extrinsic.position.transpose()
          << " rotation=\n" << extrinsic.orientation.toRotationMatrix() << '\n'
          << "REANCHOR timestamp_ns=" << initialization_stamp
          << " map_T_imu=\n" << official_map_T_imu.orientation.toRotationMatrix()
          << "\nposition=" << official_map_T_imu.position.transpose()
          << " velocity=" << frontend.getState().velocity.transpose()
          << " gyro_bias=" << frontend.getState().gyro_bias.transpose()
          << " accel_bias=" << frontend.getState().accel_bias.transpose()
          << " gravity_map_effective=" << frontend.getState().gravity.transpose() << '\n'
          << "INITIAL_COVARIANCE_DIAG velocity="
          << frontend.getState().covariance.block<3, 3>(12, 12).diagonal().transpose()
          << " gyro_bias="
          << frontend.getState().covariance.block<3, 3>(15, 15).diagonal().transpose()
          << " accel_bias="
          << frontend.getState().covariance.block<3, 3>(18, 18).diagonal().transpose() << '\n'
          << "FIRST_TRANSACTION_ID=" << first_transaction_id << '\n';
      if (dataset_state_replay && state_profile == "DATASET_VELOCITY")
        std::cout << "INITIAL_VELOCITY_INJECTION timestamp_ns=" << initialization_stamp
            << " velocity_world_m_s=" << frontend.getState().velocity.transpose()
            << " count=1\n";
      std::cout << "INITIAL_GRAVITY_INJECTION_COUNT="
          << (dataset_gravity_override ? 1 : 0) << '\n';
      if (dataset_gravity_override)
        std::cout << "INITIAL_GRAVITY_INJECTION timestamp_ns=" << initialization_stamp
            << " gravity_map_m_s2=" << frontend.getState().gravity.transpose()
            << " count=1\n";
    } else {
      const auto initialization_imu = initializationSamples(
          all_imu, parameters.static_init_samples, initialization_stamp);
      if (!frontend.initializeStatic(initialization_imu, initial_lidar, extrinsic, &reason))
        throw std::runtime_error("static_initialization_failed:" + reason);
      if (initialization_stamp != 0 && frontend.getState().stamp_ns < initialization_stamp) {
        if (!frontend.predictHeldInputTo(initialization_stamp,
            initialization_imu[initialization_imu.size() - 2], initialization_imu.back(), &reason))
          throw std::runtime_error("initialization_epoch_propagation_failed:" + reason);
      }
    }
    if (initialization_stamp != 0 && frontend.getState().stamp_ns != initialization_stamp)
      throw std::runtime_error("initialization_timestamp_mismatch");
    const auto first_scan = std::lower_bound(scans.begin(), scans.end(), first_transaction_id,
        [](const paper::P7TimedScanRecord& scan, uint64_t transaction_id) {
          return scan.transaction_id < transaction_id;
        });
    if (first_scan == scans.end() || first_scan->transaction_id != first_transaction_id ||
        limit == 0 || limit > static_cast<uint64_t>(std::distance(first_scan, scans.end())))
      throw std::runtime_error("invalid_frame_limit_or_first_transaction");
    if (official_reanchor)
      std::cout << "FIRST_SCAN tx=" << first_scan->transaction_id
          << " start_ns=" << first_scan->scan_start_ns
          << " end_ns=" << first_scan->scan_end_ns
          << " point_count=" << first_scan->cloud_point_count << '\n';
    paper::CurrentFrameNdtRegistration registration{paper::CurrentFrameNdtParameters{}};
    if (!registration.loadMap(argv[5], &reason)) throw std::runtime_error("map_load_failed:" + reason);
    std::ofstream trajectory(argv[7]), observations(argv[8]), runtime(argv[9]);
    if (!trajectory || !observations || !runtime) throw std::runtime_error("cannot_create_outputs");
    trajectory.exceptions(std::ios::badbit | std::ios::failbit);
    observations.exceptions(std::ios::badbit | std::ios::failbit);
    runtime.exceptions(std::ios::badbit | std::ios::failbit);
    trajectory << std::setprecision(17)
        << "transaction_id,stamp_ns,predicted_imu_x,predicted_imu_y,predicted_imu_z,"
           "predicted_imu_qx,predicted_imu_qy,predicted_imu_qz,predicted_imu_qw,"
           "propagated_velocity_x,propagated_velocity_y,propagated_velocity_z,propagated_speed_m_s,"
           "propagated_gyro_bias_x,propagated_gyro_bias_y,propagated_gyro_bias_z,"
           "propagated_accel_bias_x,propagated_accel_bias_y,propagated_accel_bias_z,"
           "propagated_gravity_x,propagated_gravity_y,propagated_gravity_z,"
           "corrected_imu_x,corrected_imu_y,corrected_imu_z,corrected_imu_qx,corrected_imu_qy,"
           "corrected_imu_qz,corrected_imu_qw,velocity_x,velocity_y,velocity_z,"
           "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
           "gravity_x,gravity_y,gravity_z,lidar_update_applied";
    if (dataset_state_replay)
      trajectory << ",speed_m_s,p_v_x,p_v_y,p_v_z,p_bg_x,p_bg_y,p_bg_z,p_ba_x,p_ba_y,p_ba_z,"
          "frame_increment_translation_m,frame_increment_rotation_rad";
    trajectory << '\n';
    observations << std::setprecision(17)
        << "transaction_id,scan_start_ns,scan_effective_start_ns,stamp_ns,raw_source_points,"
           "overlap_points_dropped,source_points,target_points,source_cloud_hash,converged,effective,status,"
           "iterations,fitness,transformation_probability,alignment_ms,"
           "initial_x,initial_y,initial_z,initial_qx,initial_qy,initial_qz,initial_qw,"
           "raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,lidar_update_applied";
    if (official_reanchor)
      observations << ",correction_translation_m,correction_rotation_rad,"
          "initial_overlap_020,initial_overlap_030,initial_overlap_050,initial_overlap_100,"
          "initial_nn_mean,initial_nn_median,initial_nn_p90,initial_nn_p95,"
          "final_overlap_020,final_overlap_030,final_overlap_050,final_overlap_100,"
          "final_nn_mean,final_nn_median,final_nn_p90,final_nn_p95";
    observations << '\n';
    runtime << std::setprecision(17)
        << "transaction_id,prediction_and_deskew_ms,cloud_io_ms,ndt_total_ms,ndt_alignment_ms,"
           "ikfom_update_ms,frame_total_ms,ndt_effective,lidar_update_applied\n";
    uint64_t updates = 0;
    paper::ScanEndProcessor scan_processor;
    paper::Pose3d previous_corrected_pose = frontend.getState().map_T_imu;
    for (uint64_t index = 0; index < limit; ++index) {
      const auto& scan = *(first_scan + static_cast<std::ptrdiff_t>(index));
      transaction = scan.transaction_id;
      const auto frame_start = Clock::now();
      const auto start = frontend.getState();
      const auto io_start = Clock::now();
      auto timed_cloud = paper::readP7PackedTimedCloud(argv[4], scan);
      const double cloud_io_ms = elapsedMs(io_start);
      paper::ScanWindowStats window_stats;
      const bool first_scan_preroll = dataset_contract_reanchor && index == 0 &&
                                      scan.scan_start_ns < start.stamp_ns;
      paper::P7ImuVector causal_imu;
      if (first_scan_preroll) {
        causal_imu = paper::imuWindow(all_imu, scan.scan_start_ns, scan.scan_end_ns);
        if (!paper::prepareScanWindowWithCausalPreroll(scan.scan_start_ns,
                scan.scan_end_ns, start.stamp_ns, causal_imu, &timed_cloud,
                &window_stats, &reason))
          throw std::runtime_error("causal_preroll_scan_window_failed:" + reason);
        std::cout << "FIRST_SCAN_COMPLETE=PASS raw_points=" << scan.cloud_point_count
            << " retained_points=" << window_stats.remaining_points
            << " dropped_points=" << window_stats.overlap_points_dropped
            << " anchor_overlap_ns=" << window_stats.overlap_duration_ns << '\n';
      } else {
        paper::ScanWindowDecision window_decision;
        if (!paper::prepareScanWindow(scan.scan_start_ns, scan.scan_end_ns,
                start.stamp_ns, &timed_cloud, &window_decision, &window_stats, &reason))
          throw std::runtime_error("scan_window_failed:" + reason);
        if (window_decision != paper::ScanWindowDecision::PROCESS)
          throw std::runtime_error("unexpected_stale_raw_scan");
        causal_imu = paper::imuWindow(all_imu, start.stamp_ns, scan.scan_end_ns);
      }
      const auto prediction_start = Clock::now();
      paper::ScanEndResult scan_end;
      if (!scan_processor.process(&frontend, extrinsic,
              window_stats.effective_scan_start_ns, scan.scan_end_ns,
              causal_imu, timed_cloud, &scan_end, &reason, first_scan_preroll))
        throw std::runtime_error("scan_end_prediction_or_deskew_failed:" + reason);
      const double prediction_ms = elapsedMs(prediction_start);
      if (scan_end.scan_end_ns != scan.scan_end_ns ||
          frontend.getState().stamp_ns != scan.scan_end_ns)
        throw std::runtime_error("scan_end_prediction_timestamp_mismatch");
      const auto propagated_state = frontend.getState();
      paper::RegistrationCloud cloud;
      cloud.reserve(scan_end.cloud_end_frame.size());
      for (const auto& point : scan_end.cloud_end_frame)
        cloud.push_back({static_cast<float>(point.position.x()),
                         static_cast<float>(point.position.y()),
                         static_cast<float>(point.position.z())});
      paper::CurrentFrameNdtOverlap initial_overlap;
      if (official_reanchor &&
          !registration.evaluateMapOverlap(cloud, scan_end.predicted_map_T_lidar,
                                            &initial_overlap, &reason))
        throw std::runtime_error("initial_overlap_failed:" + reason);
      const auto ndt_start = Clock::now();
      paper::CurrentFrameNdtResult result;
      if (!registration.align(scan.scan_end_ns, cloud,
                              scan_end.predicted_map_T_lidar, &result, &reason))
        throw std::runtime_error("registration_internal_error:" + reason);
      const double ndt_total_ms = elapsedMs(ndt_start);
      paper::CurrentFrameNdtOverlap final_overlap;
      double translation_correction = 0.0;
      double rotation_correction = 0.0;
      if (official_reanchor) {
        if (!registration.evaluateMapOverlap(cloud, result.raw_map_T_lidar,
                                              &final_overlap, &reason))
          throw std::runtime_error("final_overlap_failed:" + reason);
        translation_correction =
            (result.raw_map_T_lidar.position - result.initial_map_T_lidar.position).norm();
        rotation_correction = rotationDistance(result.initial_map_T_lidar, result.raw_map_T_lidar);
      }
      const auto update_start = Clock::now();
      if (result.effective) {
        paper::PoseCorrectionDelta delta;
        if (!frontend.applyPoseMeasurement(
            paper::lidarMeasurementToImu(result.raw_map_T_lidar, extrinsic), &delta, &reason))
          throw std::runtime_error("pose_update_failed:" + reason);
        ++updates;
      }
      const double update_ms = elapsedMs(update_start);
      const auto corrected = frontend.getState();
      if (corrected.stamp_ns != scan.scan_end_ns) throw std::runtime_error("state_timestamp_mismatch");
      if (!frontend.postconditionsValid(&reason)) throw std::runtime_error("postconditions_failed:" + reason);
      const double total_ms = elapsedMs(frame_start);
      trajectory << transaction << ',' << scan.scan_end_ns;
      poseColumns(trajectory, scan_end.predicted_map_T_imu);
      vectorColumns(trajectory, propagated_state.velocity);
      trajectory << ',' << propagated_state.velocity.norm();
      vectorColumns(trajectory, propagated_state.gyro_bias);
      vectorColumns(trajectory, propagated_state.accel_bias);
      vectorColumns(trajectory, propagated_state.gravity);
      poseColumns(trajectory, corrected.map_T_imu);
      vectorColumns(trajectory, corrected.velocity); vectorColumns(trajectory, corrected.gyro_bias);
      vectorColumns(trajectory, corrected.accel_bias); vectorColumns(trajectory, corrected.gravity);
      trajectory << ',' << result.effective;
      const double frame_translation_increment =
          (corrected.map_T_imu.position - previous_corrected_pose.position).norm();
      const double frame_rotation_increment = rotationDistance(
          previous_corrected_pose, corrected.map_T_imu);
      if (dataset_state_replay) {
        trajectory << ',' << corrected.velocity.norm();
        for (int axis = 0; axis < 3; ++axis)
          trajectory << ',' << corrected.covariance(12 + axis, 12 + axis);
        for (int axis = 0; axis < 3; ++axis)
          trajectory << ',' << corrected.covariance(15 + axis, 15 + axis);
        for (int axis = 0; axis < 3; ++axis)
          trajectory << ',' << corrected.covariance(18 + axis, 18 + axis);
        trajectory << ',' << frame_translation_increment << ',' << frame_rotation_increment;
      }
      trajectory << '\n';
      previous_corrected_pose = corrected.map_T_imu;
      observations << transaction << ',' << scan.scan_start_ns << ','
          << window_stats.effective_scan_start_ns << ',' << scan.scan_end_ns << ','
          << scan.cloud_point_count << ',' << window_stats.overlap_points_dropped << ','
          << result.source_point_count << ',' << result.target_point_count << ','
          << result.source_cloud_hash
          << ',' << result.converged << ',' << result.effective << ','
          << paper::currentFrameNdtStatusName(result.status) << ',' << result.iterations << ','
          << result.fitness << ',' << result.transformation_probability << ',' << result.alignment_ms;
      poseColumns(observations, result.initial_map_T_lidar); poseColumns(observations, result.raw_map_T_lidar);
      observations << ',' << result.effective;
      if (official_reanchor) {
        observations << ',' << translation_correction << ',' << rotation_correction;
        for (double value : initial_overlap.fraction_within) observations << ',' << value;
        observations << ',' << initial_overlap.mean_nearest_distance_m
            << ',' << initial_overlap.median_nearest_distance_m
            << ',' << initial_overlap.p90_nearest_distance_m
            << ',' << initial_overlap.p95_nearest_distance_m;
        for (double value : final_overlap.fraction_within) observations << ',' << value;
        observations << ',' << final_overlap.mean_nearest_distance_m
            << ',' << final_overlap.median_nearest_distance_m
            << ',' << final_overlap.p90_nearest_distance_m
            << ',' << final_overlap.p95_nearest_distance_m;
      }
      observations << '\n';
      runtime << transaction << ',' << prediction_ms << ',' << cloud_io_ms << ',' << ndt_total_ms
          << ',' << result.alignment_ms << ',' << update_ms << ',' << total_ms << ','
          << result.effective << ',' << result.effective << '\n';
      if (official_reanchor) {
        const Eigen::Vector3d corrected_rpy = rpy(corrected.map_T_imu);
        std::cout << std::setprecision(9) << "FRAME tx=" << transaction
            << " scan_start_ns=" << scan.scan_start_ns
            << " scan_end_ns=" << scan.scan_end_ns
            << " status=" << paper::currentFrameNdtStatusName(result.status)
            << " converged=" << result.converged
            << " iterations=" << result.iterations
            << " correction_m=" << translation_correction
            << " correction_rad=" << rotation_correction
            << " initial_overlap=" << initial_overlap.fraction_within[0] << '/'
            << initial_overlap.fraction_within[1] << '/'
            << initial_overlap.fraction_within[2] << '/'
            << initial_overlap.fraction_within[3]
            << " final_overlap=" << final_overlap.fraction_within[0] << '/'
            << final_overlap.fraction_within[1] << '/'
            << final_overlap.fraction_within[2] << '/'
            << final_overlap.fraction_within[3]
            << " fitness=" << result.fitness
            << " pose_imu_xyz=" << corrected.map_T_imu.position.transpose()
            << " pose_imu_rpy=" << corrected_rpy.transpose();
        if (dataset_state_replay)
          std::cout << " velocity=" << corrected.velocity.transpose()
              << " speed=" << corrected.velocity.norm()
              << " bg=" << corrected.gyro_bias.transpose()
              << " ba=" << corrected.accel_bias.transpose()
              << " P_v=" << corrected.covariance.block<3, 3>(12, 12).diagonal().transpose()
              << " P_bg=" << corrected.covariance.block<3, 3>(15, 15).diagonal().transpose()
              << " P_ba=" << corrected.covariance.block<3, 3>(18, 18).diagonal().transpose()
              << " frame_increment_m=" << frame_translation_increment
              << " frame_increment_rad=" << frame_rotation_increment;
        std::cout << '\n';
      }
    }
    trajectory.close(); observations.close(); runtime.close();
    std::cout << "frames=" << limit << " first_transaction=" << first_transaction_id
              << " lidar_updates=" << updates
              << " prediction_only=" << limit - updates << " state_finite=true\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FIRST_BAD_TX=" << transaction << " error=" << error.what() << '\n';
    return 1;
  }
}
