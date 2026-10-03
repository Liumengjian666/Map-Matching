#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/dual_u_architecture.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
void vector6Columns(std::ostream& out, const paper::DualUVector6d& vector) {
  out << ",\"";
  for (int i = 0; i < 6; ++i) {
    if (i) out << ';';
    out << vector(i);
  }
  out << '"';
}
void matrix6RowMajorColumn(std::ostream& out, const paper::DualUMatrix6d& matrix) {
  out << ",\"";
  bool first = true;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      if (!first) out << ';';
      first = false;
      out << matrix(row, column);
    }
  }
  out << '"';
}
struct DirectionalFdAudit {
  std::string status = "NOT_SCHEDULED";
  double relative_error = std::numeric_limits<double>::quiet_NaN();
  double step = std::numeric_limits<double>::quiet_NaN();
};
bool isCurvatureAuditFrame(uint64_t transaction_id) {
  // Two historical cohort entries and the relative spectrum extremes from
  // that frozen 32-frame cohort; this schedules diagnostics only.
  return transaction_id == 120 || transaction_id == 838 ||
      transaction_id == 1359 || transaction_id == 2350;
}
DirectionalFdAudit auditCurvatureDirection(
    paper::CurrentFrameNdtRegistration* registration,
    const paper::Pose3d& pose,
    const paper::WithinBasinObservability& observability,
    const paper::NdtObjectiveSample& center, int eigen_index) {
  DirectionalFdAudit result;
  if (!registration || !observability.valid || !center.valid ||
      eigen_index < 0 || eigen_index >= 6) {
    result.status = "INVALID_FD_INPUT";
    return result;
  }
  const paper::DualUVector6d direction =
      observability.curvature_eigenvectors.col(eigen_index);
  const double predicted = observability.curvature_eigenvalues(eigen_index);
  bool saw_support_change = false;
  bool have_previous = false;
  double previous_fd = std::numeric_limits<double>::quiet_NaN();
  double previous_step = std::numeric_limits<double>::quiet_NaN();
  for (double step : {0.02, 0.01, 0.005, 0.0025, 0.001}) {
    const paper::Pose3d positive_pose = paper::applyMapProductChartIncrement(
        pose, step * direction, observability.length_scale_m);
    const paper::Pose3d negative_pose = paper::applyMapProductChartIncrement(
        pose, -step * direction, observability.length_scale_m);
    paper::NdtObjectiveSample positive, negative;
    std::string reason;
    if (!registration->evaluateLocalObjectiveAtPose(positive_pose, &positive, &reason) ||
        !registration->evaluateLocalObjectiveAtPose(negative_pose, &negative, &reason)) {
      result.status = "OBJECTIVE_EVALUATION_FAILED:" + reason;
      return result;
    }
    if (positive.target_neighborhood_hash != center.target_neighborhood_hash ||
        negative.target_neighborhood_hash != center.target_neighborhood_hash) {
      saw_support_change = true;
      have_previous = false;
      continue;
    }
    const double n = static_cast<double>(center.source_point_count);
    const double f0 = -center.score_sum / n;
    const double fp = -positive.score_sum / n;
    const double fm = -negative.score_sum / n;
    const double fd = (fp - 2.0 * f0 + fm) / (step * step);
    const double denominator = std::max({1.0, std::abs(fd), std::abs(predicted)});
    result.relative_error = std::abs(fd - predicted) / denominator;
    result.step = step;
    if (!std::isfinite(fd) || !std::isfinite(predicted)) {
      result.status = "NONFINITE_DIRECTIONAL_CURVATURE";
      return result;
    }
    const bool same_sign = fd * predicted > 0.0;
    const bool step_stable = have_previous &&
        std::abs(fd - previous_fd) <= 0.35 *
            std::max({1.0, std::abs(fd), std::abs(previous_fd)}) &&
        std::abs(previous_step - 2.0 * step) <= 1e-12;
    if (same_sign && result.relative_error <= 0.35 && step_stable) {
      result.status = "PASS_STABLE_SUPPORT_AND_CURVATURE";
      return result;
    }
    previous_fd = fd;
    previous_step = step;
    have_previous = true;
  }
  result.status = saw_support_change ? "INVALID_TARGET_NEIGHBORHOOD_CHANGED" :
      (have_previous ? "FD_CURVATURE_OR_STEP_STABILITY_MISMATCH" :
                       "NO_STABLE_FD_STEP_PAIR");
  return result;
}
}  // namespace

int main(int argc, char** argv) {
  uint64_t transaction = 0;
  try {
    const bool dual_u_shadow = argc == 14 && std::string(argv[12]) == "--dual-u-shadow";
    if (argc != 12 && !dual_u_shadow)
      throw std::runtime_error("usage: p7_single_state_runner IMU_CSV FILTER_SCANS_CSV "
          "RAW_TIMED_SCAN_INDEX_CSV RAW_TIMED_POINTS_BIN MAP_PCD PARAMS_TXT "
          "TRAJECTORY_CSV REGISTRATION_CSV RUNTIME_CSV FRAME_LIMIT INITIALIZATION_STAMP_NS "
          "[--dual-u-shadow DUAL_U_CSV]");
    const auto all_imu = paper::readP7Imu(argv[1]);
    const auto scans = paper::readP7TimedScans(argv[2], argv[3]);
    const uint64_t limit = unsignedArgument(argv[10]);
    const uint64_t initialization_stamp = unsignedArgument(argv[11]);
    if (limit == 0 || limit > scans.size()) throw std::runtime_error("invalid_frame_limit");
    paper::Pose3d initial_lidar, extrinsic;
    const auto parameters = paper::readP7Parameters(argv[6], &initial_lidar, &extrinsic);
    const auto initialization_imu = initializationSamples(
        all_imu, parameters.static_init_samples, initialization_stamp);
    paper::FastLio2IkfomFrontend frontend(parameters);
    std::string reason;
    if (!frontend.initializeStatic(initialization_imu, initial_lidar, extrinsic, &reason))
      throw std::runtime_error("static_initialization_failed:" + reason);
    if (initialization_stamp != 0 && frontend.getState().stamp_ns < initialization_stamp) {
      if (!frontend.predictHeldInputTo(initialization_stamp,
          initialization_imu[initialization_imu.size() - 2], initialization_imu.back(), &reason))
        throw std::runtime_error("initialization_epoch_propagation_failed:" + reason);
    }
    if (initialization_stamp != 0 && frontend.getState().stamp_ns != initialization_stamp)
      throw std::runtime_error("initialization_timestamp_mismatch");
    paper::CurrentFrameNdtRegistration registration{paper::CurrentFrameNdtParameters{}};
    if (!registration.loadMap(argv[5], &reason)) throw std::runtime_error("map_load_failed:" + reason);
    std::ofstream trajectory(argv[7]), observations(argv[8]), runtime(argv[9]);
    std::ofstream dual_u_output;
    if (dual_u_shadow) dual_u_output.open(argv[13]);
    if (!trajectory || !observations || !runtime ||
        (dual_u_shadow && !dual_u_output))
      throw std::runtime_error("cannot_create_outputs");
    trajectory.exceptions(std::ios::badbit | std::ios::failbit);
    observations.exceptions(std::ios::badbit | std::ios::failbit);
    runtime.exceptions(std::ios::badbit | std::ios::failbit);
    if (dual_u_shadow) dual_u_output.exceptions(std::ios::badbit | std::ios::failbit);
    trajectory << std::setprecision(17)
        << "transaction_id,stamp_ns,predicted_imu_x,predicted_imu_y,predicted_imu_z,"
           "predicted_imu_qx,predicted_imu_qy,predicted_imu_qz,predicted_imu_qw,"
           "corrected_imu_x,corrected_imu_y,corrected_imu_z,corrected_imu_qx,corrected_imu_qy,"
           "corrected_imu_qz,corrected_imu_qw,velocity_x,velocity_y,velocity_z,"
           "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
           "gravity_x,gravity_y,gravity_z,lidar_update_applied\n";
    observations << std::setprecision(17)
        << "transaction_id,scan_start_ns,scan_effective_start_ns,stamp_ns,raw_source_points,"
           "overlap_points_dropped,source_points,target_points,source_cloud_hash,converged,effective,status,"
           "iterations,fitness,transformation_probability,alignment_ms,"
           "initial_x,initial_y,initial_z,initial_qx,initial_qy,initial_qz,initial_qw,"
           "raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,lidar_update_applied\n";
    runtime << std::setprecision(17)
        << "transaction_id,prediction_and_deskew_ms,cloud_io_ms,ndt_total_ms,ndt_alignment_ms,"
           "ikfom_update_ms,frame_total_ms,ndt_effective,lidar_update_applied\n";
    if (dual_u_shadow) {
      dual_u_output << std::setprecision(17)
          << "transaction_id,stamp_ns,ndt_status,ndt_effective,source_cloud_hash,source_points,target_points,"
             "configured_resolution_m,target_grid_leaf_x_m,target_grid_leaf_y_m,target_grid_leaf_z_m,"
             "step_size,transformation_epsilon,maximum_iterations,"
             "raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,uobs_valid,uobs_status,locally_convex,"
             "length_scale_m,objective_per_source,euler_chart_condition,score_hessian_relative_asymmetry,"
             "pulled_hessian_relative_asymmetry,curvature_eigenvalues,curvature_eigenvectors_rowmajor,"
             "local_curvature_rowmajor,fd_weak_status,fd_weak_relative_error,fd_weak_step,"
             "fd_strong_status,fd_strong_relative_error,fd_strong_step,unonlocal_status,unonlocal_diagnostic,"
             "filter_state_accessed_by_diagnostic\n";
    }
    uint64_t updates = 0;
    paper::ScanEndProcessor scan_processor;
    for (uint64_t index = 0; index < limit; ++index) {
      const auto& scan = scans.at(index);
      transaction = scan.transaction_id;
      const auto frame_start = Clock::now();
      const auto start = frontend.getState();
      const auto io_start = Clock::now();
      auto timed_cloud = paper::readP7PackedTimedCloud(argv[4], scan);
      const double cloud_io_ms = elapsedMs(io_start);
      paper::ScanWindowDecision window_decision;
      paper::ScanWindowStats window_stats;
      if (!paper::prepareScanWindow(scan.scan_start_ns, scan.scan_end_ns,
              start.stamp_ns, &timed_cloud, &window_decision, &window_stats, &reason))
        throw std::runtime_error("scan_window_failed:" + reason);
      if (window_decision != paper::ScanWindowDecision::PROCESS)
        throw std::runtime_error("unexpected_stale_raw_scan");
      const auto prediction_start = Clock::now();
      const auto causal_imu = paper::imuWindow(all_imu, start.stamp_ns, scan.scan_end_ns);
      paper::ScanEndResult scan_end;
      if (!scan_processor.process(&frontend, extrinsic,
              window_stats.effective_scan_start_ns, scan.scan_end_ns,
              causal_imu, timed_cloud, &scan_end, &reason))
        throw std::runtime_error("scan_end_prediction_or_deskew_failed:" + reason);
      const double prediction_ms = elapsedMs(prediction_start);
      if (scan_end.scan_end_ns != scan.scan_end_ns ||
          frontend.getState().stamp_ns != scan.scan_end_ns)
        throw std::runtime_error("scan_end_prediction_timestamp_mismatch");
      paper::RegistrationCloud cloud;
      cloud.reserve(scan_end.cloud_end_frame.size());
      for (const auto& point : scan_end.cloud_end_frame)
        cloud.push_back({static_cast<float>(point.position.x()),
                         static_cast<float>(point.position.y()),
                         static_cast<float>(point.position.z())});
      const auto ndt_start = Clock::now();
      paper::CurrentFrameNdtResult result;
      if (!registration.align(scan.scan_end_ns, cloud,
                              scan_end.predicted_map_T_lidar, &result, &reason))
        throw std::runtime_error("registration_internal_error:" + reason);
      const double ndt_total_ms = elapsedMs(ndt_start);
      if (dual_u_shadow) {
        paper::WithinBasinObservability u_obs;
        paper::PclNdtScoreJet jet;
        paper::NdtObjectiveSample center_objective;
        const auto target_leaf_f = registration.targetGridLeafSizeMeters();
        const Eigen::Vector3d target_leaf(target_leaf_f[0], target_leaf_f[1], target_leaf_f[2]);
        const auto& ndt_parameters = registration.parameters();
        std::string uobs_status = "NDT_NOT_EFFECTIVE";
        if (result.effective && registration.evaluateLocalScoreJetAtPose(
                result.raw_map_T_lidar, &jet, &reason) &&
            paper::analyzeWithinBasinObservability(jet, result.raw_map_T_lidar,
                target_leaf, ndt_parameters.resolution_m, &u_obs, &reason)) {
          uobs_status = u_obs.status;
          if (isCurvatureAuditFrame(scan.transaction_id)) {
            if (!registration.evaluateLocalObjectiveAtPose(
                    result.raw_map_T_lidar, &center_objective, &reason))
              uobs_status = "BASE_OBJECTIVE_FAILED:" + reason;
          }
        } else if (result.effective) {
          uobs_status = reason.empty() ? "UOBS_EVALUATION_FAILED" : reason;
        }
        DirectionalFdAudit weak_fd, strong_fd;
        if (u_obs.valid && center_objective.valid &&
            isCurvatureAuditFrame(scan.transaction_id)) {
          weak_fd = auditCurvatureDirection(&registration, result.raw_map_T_lidar,
              u_obs, center_objective, 0);
          strong_fd = auditCurvatureDirection(&registration, result.raw_map_T_lidar,
              u_obs, center_objective, 5);
        }
        dual_u_output << transaction << ',' << scan.scan_end_ns << ','
            << paper::currentFrameNdtStatusName(result.status) << ',' << result.effective << ','
            << result.source_cloud_hash << ',' << result.source_point_count << ','
            << result.target_point_count << ',' << ndt_parameters.resolution_m << ','
            << target_leaf_f[0] << ',' << target_leaf_f[1] << ',' << target_leaf_f[2] << ','
            << ndt_parameters.step_size << ',' << ndt_parameters.transformation_epsilon << ','
            << ndt_parameters.maximum_iterations;
        poseColumns(dual_u_output, result.raw_map_T_lidar);
        dual_u_output << ',' << u_obs.valid << ',' << uobs_status << ','
            << u_obs.locally_convex << ',' << u_obs.length_scale_m << ','
            << u_obs.objective_per_source << ',' << u_obs.euler_chart_condition << ','
            << u_obs.score_hessian_relative_asymmetry << ','
            << u_obs.pulled_hessian_relative_asymmetry;
        vector6Columns(dual_u_output, u_obs.curvature_eigenvalues);
        matrix6RowMajorColumn(dual_u_output, u_obs.curvature_eigenvectors);
        matrix6RowMajorColumn(dual_u_output, u_obs.local_curvature);
        dual_u_output << ',' << weak_fd.status << ',' << weak_fd.relative_error << ','
            << weak_fd.step << ',' << strong_fd.status << ',' << strong_fd.relative_error << ','
            << strong_fd.step << ",INDETERMINATE,NO_SAME_OBJECTIVE_MULTI_START_SET,NO_FILTER_HANDLE_OR_STATE_ACCESS\n";
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
      poseColumns(trajectory, scan_end.predicted_map_T_imu); poseColumns(trajectory, corrected.map_T_imu);
      vectorColumns(trajectory, corrected.velocity); vectorColumns(trajectory, corrected.gyro_bias);
      vectorColumns(trajectory, corrected.accel_bias); vectorColumns(trajectory, corrected.gravity);
      trajectory << ',' << result.effective << '\n';
      observations << transaction << ',' << scan.scan_start_ns << ','
          << window_stats.effective_scan_start_ns << ',' << scan.scan_end_ns << ','
          << scan.cloud_point_count << ',' << window_stats.overlap_points_dropped << ','
          << result.source_point_count << ',' << result.target_point_count << ','
          << result.source_cloud_hash
          << ',' << result.converged << ',' << result.effective << ','
          << paper::currentFrameNdtStatusName(result.status) << ',' << result.iterations << ','
          << result.fitness << ',' << result.transformation_probability << ',' << result.alignment_ms;
      poseColumns(observations, result.initial_map_T_lidar); poseColumns(observations, result.raw_map_T_lidar);
      observations << ',' << result.effective << '\n';
      runtime << transaction << ',' << prediction_ms << ',' << cloud_io_ms << ',' << ndt_total_ms
          << ',' << result.alignment_ms << ',' << update_ms << ',' << total_ms << ','
          << result.effective << ',' << result.effective << '\n';
    }
    trajectory.close(); observations.close(); runtime.close();
    if (dual_u_shadow) dual_u_output.close();
    std::cout << "frames=" << limit << " lidar_updates=" << updates
              << " prediction_only=" << limit - updates << " state_finite=true\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FIRST_BAD_TX=" << transaction << " error=" << error.what() << '\n';
    return 1;
  }
}
