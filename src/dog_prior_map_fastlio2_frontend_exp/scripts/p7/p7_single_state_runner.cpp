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
bool isCurvatureAuditFrame(uint64_t transaction_id) {
  // Two historical cohort entries and the relative spectrum extremes from
  // that frozen 32-frame cohort; this schedules diagnostics only.
  return transaction_id == 120 || transaction_id == 838 ||
      transaction_id == 1359 || transaction_id == 2350;
}
bool isP5I1CohortFrame(uint64_t transaction_id) {
  static const std::array<uint64_t, 32> ids{{120, 244, 368, 616, 740, 838, 839, 864,
      924, 925, 1111, 1235, 1359, 1497, 1498, 1556, 1557, 1606, 1730, 1854, 2102,
      2226, 2350, 2598, 2722, 2846, 3094, 3217, 3341, 3631, 3796, 3962}};
  return std::find(ids.begin(), ids.end(), transaction_id) != ids.end();
}
bool isP5I1TargetedFrame(uint64_t transaction_id) {
  static const std::array<uint64_t, 8> ids{{838, 839, 924, 925, 1497, 1498, 1556, 1557}};
  return std::find(ids.begin(), ids.end(), transaction_id) != ids.end();
}
std::string poseSemicolonString(const paper::Pose3d& pose) {
  std::ostringstream out;
  out << std::setprecision(17) << pose.position.x() << ';' << pose.position.y() << ';'
      << pose.position.z() << ';' << pose.orientation.x() << ';'
      << pose.orientation.y() << ';' << pose.orientation.z() << ';'
      << pose.orientation.w();
  return out.str();
}

void writeCurvatureAudit(paper::CurrentFrameNdtRegistration* registration,
    uint64_t transaction_id, uint64_t stamp_ns, const paper::Pose3d& pose,
    const paper::WithinBasinObservability& u_obs, std::ostream& output) {
  std::shared_ptr<const paper::NdtFrozenSupportSnapshot> frozen;
  paper::NdtFrozenObjectiveSample center;
  std::string reason;
  if (!registration->captureFrozenSupport(pose, &frozen, &center, &reason))
    throw std::runtime_error("frozen_support_capture_failed:" + reason);
  const double n = static_cast<double>(center.source_point_count);
  const double j0 = -center.score_sum / n;
  const std::array<int, 4> direction_indices{{0, 2, 3, 5}};
  const std::array<double, 7> steps{{0.02, 0.01, 0.005, 0.0025, 0.001, 0.0005, 0.00025}};
  output << std::setprecision(17);
  for (int eigen_index : direction_indices) {
    const paper::DualUVector6d q = u_obs.curvature_eigenvectors.col(eigen_index);
    const double gradient_projection = u_obs.objective_gradient.dot(q);
    const double directional_curvature = q.dot(u_obs.local_curvature * q);
    for (double h : steps) {
      const paper::Pose3d plus_pose = paper::applyMapProductChartIncrement(
          pose, h * q, u_obs.length_scale_m);
      const paper::Pose3d minus_pose = paper::applyMapProductChartIncrement(
          pose, -h * q, u_obs.length_scale_m);
      paper::NdtFrozenObjectiveSample frozen_plus, frozen_minus;
      paper::NdtObjectiveSample dynamic_plus, dynamic_minus;
      paper::NdtSupportChangeDiagnostic support_plus, support_minus;
      if (!registration->evaluateFrozenSupportObjective(*frozen, plus_pose,
              &frozen_plus, &reason) ||
          !registration->evaluateFrozenSupportObjective(*frozen, minus_pose,
              &frozen_minus, &reason) ||
          !registration->evaluateDynamicSupportDiagnostic(*frozen, plus_pose,
              &dynamic_plus, &support_plus, &reason) ||
          !registration->evaluateDynamicSupportDiagnostic(*frozen, minus_pose,
              &dynamic_minus, &support_minus, &reason))
        throw std::runtime_error("curvature_direction_evaluation_failed:" + reason);
      const double jp_frozen = -frozen_plus.score_sum / n;
      const double jm_frozen = -frozen_minus.score_sum / n;
      const double jp_dynamic = -dynamic_plus.score_sum / n;
      const double jm_dynamic = -dynamic_minus.score_sum / n;
      const double predicted_plus = j0 + h * gradient_projection +
          0.5 * h * h * directional_curvature;
      const double predicted_minus = j0 - h * gradient_projection +
          0.5 * h * h * directional_curvature;
      const double central_fd = (jp_frozen - 2.0 * j0 + jm_frozen) / (h * h);
      const double relative_error = std::abs(central_fd - directional_curvature) /
          std::max({1.0, std::abs(central_fd), std::abs(directional_curvature)});
      const double dynamic_central_fd = (jp_dynamic - 2.0 * j0 + jm_dynamic) / (h * h);
      output << transaction_id << ',' << stamp_ns << ',' << eigen_index << ','
          << u_obs.curvature_eigenvalues(eigen_index);
      for (int component = 0; component < 6; ++component) output << ',' << q(component);
      output << ',' << h << ',' << j0 << ',' << gradient_projection << ','
          << directional_curvature << ',' << predicted_plus << ',' << predicted_minus
          << ',' << jp_frozen << ',' << jm_frozen << ',' << central_fd << ','
          << relative_error << ',' << dynamic_plus.score_sum << ','
          << dynamic_minus.score_sum << ',' << jp_dynamic << ',' << jm_dynamic << ','
          << dynamic_central_fd << ',' << support_plus.source_point_count << ','
          << support_plus.changed_source_point_count << ','
          << support_plus.changed_source_point_fraction << ','
          << support_minus.changed_source_point_count << ','
          << support_minus.changed_source_point_fraction << ','
          << support_plus.center_gaussian_membership_count << ','
          << support_plus.perturbed_gaussian_membership_count << ','
          << support_plus.membership_symmetric_difference_count << ','
          << support_minus.perturbed_gaussian_membership_count << ','
          << support_minus.membership_symmetric_difference_count << '\n';
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
  uint64_t transaction = 0;
  try {
    const bool uobs_only_mode = argc == 14 && std::string(argv[12]) == "--uobs-only";
    const bool dual_u_shadow = (argc == 14 || argc == 16 || argc == 18) &&
        std::string(argv[12]) == "--dual-u-shadow";
    const bool curvature_audit = (argc == 16 || argc == 18) &&
        std::string(argv[14]) == "--curvature-audit";
    const bool objective_export = argc == 18 &&
        std::string(argv[16]) == "--objective-export";
    if (argc != 12 && argc != 14 && argc != 16 && argc != 18)
      throw std::runtime_error("usage: p7_single_state_runner IMU_CSV FILTER_SCANS_CSV "
          "RAW_TIMED_SCAN_INDEX_CSV RAW_TIMED_POINTS_BIN MAP_PCD PARAMS_TXT "
          "TRAJECTORY_CSV REGISTRATION_CSV RUNTIME_CSV FRAME_LIMIT INITIALIZATION_STAMP_NS "
          "[--uobs-only UOBS_CSV | --dual-u-shadow DUAL_U_CSV [--curvature-audit CURVATURE_CSV "
          "[--objective-export DIR]]]");
    if ((argc == 14 && !dual_u_shadow && !uobs_only_mode) ||
        (argc == 16 && (!dual_u_shadow || !curvature_audit)) ||
        (argc == 18 && (!dual_u_shadow || !curvature_audit || !objective_export)))
      throw std::runtime_error("invalid_optional_diagnostic_arguments");
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
    std::ofstream uobs_output;
    std::ofstream curvature_output;
    std::ofstream objective_export_output;
    const std::string objective_export_dir = objective_export ? argv[17] : std::string();
    if (dual_u_shadow) dual_u_output.open(argv[13]);
    if (uobs_only_mode) uobs_output.open(argv[13]);
    if (curvature_audit) curvature_output.open(argv[15]);
    if (objective_export) objective_export_output.open(objective_export_dir + "/cohort.csv");
    if (!trajectory || !observations || !runtime ||
        (dual_u_shadow && !dual_u_output) || (curvature_audit && !curvature_output) ||
        (uobs_only_mode && !uobs_output) ||
        (objective_export && !objective_export_output))
      throw std::runtime_error("cannot_create_outputs");
    trajectory.exceptions(std::ios::badbit | std::ios::failbit);
    observations.exceptions(std::ios::badbit | std::ios::failbit);
    runtime.exceptions(std::ios::badbit | std::ios::failbit);
    if (dual_u_shadow) dual_u_output.exceptions(std::ios::badbit | std::ios::failbit);
    if (uobs_only_mode) uobs_output.exceptions(std::ios::badbit | std::ios::failbit);
    if (curvature_audit) {
      curvature_output.exceptions(std::ios::badbit | std::ios::failbit);
      curvature_output << std::setprecision(17)
          << "transaction_id,stamp_ns,eigen_index,eigenvalue,q0,q1,q2,q3,q4,q5,h,J0,"
             "gradient_projection,qTHq,J_pred_plus,J_pred_minus,J_frozen_plus,J_frozen_minus,"
             "central_second_difference,relative_curvature_error,dynamic_score_sum_plus,"
             "dynamic_score_sum_minus,J_dynamic_plus,J_dynamic_minus,dynamic_central_second_difference,"
             "source_point_count,changed_points_plus,changed_fraction_plus,changed_points_minus,"
             "changed_fraction_minus,center_memberships,perturbed_memberships_plus,"
             "membership_symdiff_plus,perturbed_memberships_minus,membership_symdiff_minus\n";
    }
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
    if (uobs_only_mode) {
      uobs_output << std::setprecision(17)
          << "transaction_id,stamp_ns,ndt_status,ndt_effective,uobs_valid,uobs_status,"
             "locally_convex,covariance_inflation_applied,covariance_mapping_status,"
             "uobs_compute_ms,weak_weight_floor,R0_trace,Reff_trace,added_covariance_trace,"
             "curvature_eigenvalues,relative_curvature,directional_reliability,"
             "directional_variance_inflation,chart_to_residual_jacobian_rowmajor,"
             "effective_covariance_rowmajor,lidar_update_applied\n";
    }
    if (objective_export) {
      objective_export_output.exceptions(std::ios::badbit | std::ios::failbit);
      objective_export_output << std::setprecision(17)
          << "transaction_id,stamp_ns,wide_targeted,raw_point_count,raw_cloud_file,prepared_source_hash,"
             "prepared_source_point_count,target_point_count,configured_resolution_m,"
             "target_grid_leaf_x_m,target_grid_leaf_y_m,target_grid_leaf_z_m,step_size,"
             "transformation_epsilon,maximum_iterations,initial_pose_xyz_q_xyzw,"
             "raw_terminal_pose_xyz_q_xyzw\n";
    }
    uint64_t updates = 0;
    uint64_t objective_export_frames = 0;
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
      if (objective_export && isP5I1CohortFrame(transaction)) {
        const std::string cloud_file = objective_export_dir + "/raw_tx_" +
            std::to_string(transaction) + ".xyzf";
        std::ofstream cloud_output(cloud_file, std::ios::binary);
        if (!cloud_output) throw std::runtime_error("cannot_create_objective_source_cloud");
        for (const paper::RegistrationPoint& point : cloud) {
          const float xyz[3] = {point.x, point.y, point.z};
          cloud_output.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
        }
        cloud_output.close();
        const auto& export_parameters = registration.parameters();
        const auto export_leaf_f = registration.targetGridLeafSizeMeters();
        objective_export_output << transaction << ',' << scan.scan_end_ns << ','
            << isP5I1TargetedFrame(transaction) << ','
            << cloud.size() << ',' << cloud_file << ',' << result.source_cloud_hash << ','
            << result.source_point_count << ',' << result.target_point_count << ','
            << export_parameters.resolution_m << ',' << export_leaf_f[0] << ','
            << export_leaf_f[1] << ',' << export_leaf_f[2] << ','
            << export_parameters.step_size << ',' << export_parameters.transformation_epsilon
            << ',' << export_parameters.maximum_iterations << ','
            << '"' << poseSemicolonString(scan_end.predicted_map_T_lidar) << '"' << ','
            << '"' << poseSemicolonString(result.raw_map_T_lidar) << '"' << '\n';
        ++objective_export_frames;
      }
      paper::WithinBasinObservability u_obs;
      paper::UobsCovarianceInflation uobs_inflation;
      std::string uobs_status = "NOT_REQUESTED";
      double uobs_compute_ms = 0.0;
      Eigen::Matrix<double, 6, 6> baseline_pose_covariance =
          Eigen::Matrix<double, 6, 6>::Zero();
      baseline_pose_covariance.block<3, 3>(0, 0).diagonal().setConstant(
          parameters.pose_position_sigma_m * parameters.pose_position_sigma_m);
      baseline_pose_covariance.block<3, 3>(3, 3).diagonal().setConstant(
          parameters.pose_rotation_sigma_rad * parameters.pose_rotation_sigma_rad);
      Eigen::Matrix<double, 6, 6> effective_pose_covariance = baseline_pose_covariance;
      if (dual_u_shadow || uobs_only_mode) {
        paper::PclNdtScoreJet jet;
        const auto target_leaf_f = registration.targetGridLeafSizeMeters();
        const Eigen::Vector3d target_leaf(target_leaf_f[0], target_leaf_f[1], target_leaf_f[2]);
        const auto& ndt_parameters = registration.parameters();
        uobs_status = "NDT_NOT_EFFECTIVE";
        const auto uobs_start = Clock::now();
        if (result.effective && registration.evaluateLocalScoreJetAtPose(
                result.raw_map_T_lidar, &jet, &reason) &&
            paper::analyzeWithinBasinObservability(jet, result.raw_map_T_lidar,
                target_leaf, ndt_parameters.resolution_m, &u_obs, &reason)) {
          uobs_status = u_obs.status;
        } else if (result.effective) {
          uobs_status = reason.empty() ? "UOBS_EVALUATION_FAILED" : reason;
        }
        if (uobs_only_mode && result.effective) {
          const paper::Pose3d measurement_map_T_imu =
              paper::lidarMeasurementToImu(result.raw_map_T_lidar, extrinsic);
          const paper::Pose3d prediction_map_T_imu = frontend.getState().map_T_imu;
          paper::UobsCovarianceInflation inflation;
          std::string mapping_reason;
          // The fixed 0.25 floor caps each directional variance at 4x R0.
          // It is an engineering bound, not a GT-fitted parameter.
          if (!paper::buildUobsInflatedPoseMeasurementCovariance(
                  u_obs, result.raw_map_T_lidar, prediction_map_T_imu,
                  measurement_map_T_imu, baseline_pose_covariance, 0.25,
                  &inflation, &mapping_reason))
            throw std::runtime_error("uobs_covariance_mapping_failed:" + mapping_reason);
          uobs_inflation = inflation;
          effective_pose_covariance = inflation.effective_covariance;
          if (inflation.status != "UOBS_DIRECTIONAL_COVARIANCE_INFLATION_APPLIED" &&
              inflation.status != "UOBS_NO_DIRECTIONAL_INFLATION")
            uobs_status += ";" + inflation.status;
        }
        uobs_compute_ms = elapsedMs(uobs_start);
        if (curvature_audit && u_obs.valid && isCurvatureAuditFrame(scan.transaction_id))
          writeCurvatureAudit(&registration, transaction, scan.scan_end_ns,
              result.raw_map_T_lidar, u_obs, curvature_output);
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
        dual_u_output << ",BRANCH_FD_IN_CURVATURE_SIDECAR,NA,NA,"
            << "BRANCH_FD_IN_CURVATURE_SIDECAR,NA,NA,"
            << "INDETERMINATE,NO_SAME_OBJECTIVE_MULTI_START_SET,NO_FILTER_HANDLE_OR_STATE_ACCESS\n";
      }
      if (uobs_only_mode) {
        const double r0_trace = baseline_pose_covariance.trace();
        const double reff_trace = effective_pose_covariance.trace();
        uobs_output << transaction << ',' << scan.scan_end_ns << ','
            << paper::currentFrameNdtStatusName(result.status) << ',' << result.effective << ','
            << u_obs.valid << ',' << uobs_status << ',' << u_obs.locally_convex << ','
            << uobs_inflation.applied << ',' << uobs_inflation.status << ','
            << uobs_compute_ms << ',' << uobs_inflation.weak_weight_floor << ','
            << r0_trace << ',' << reff_trace << ',' << reff_trace - r0_trace;
        vector6Columns(uobs_output, u_obs.curvature_eigenvalues);
        vector6Columns(uobs_output, uobs_inflation.relative_curvature);
        vector6Columns(uobs_output, uobs_inflation.directional_reliability);
        vector6Columns(uobs_output, uobs_inflation.directional_variance_inflation);
        matrix6RowMajorColumn(uobs_output, uobs_inflation.chart_to_residual_jacobian);
        matrix6RowMajorColumn(uobs_output, effective_pose_covariance);
        uobs_output << ',' << result.effective << '\n';
      }
      const auto update_start = Clock::now();
      if (result.effective) {
        paper::PoseCorrectionDelta delta;
        const paper::Pose3d measurement_map_T_imu =
            paper::lidarMeasurementToImu(result.raw_map_T_lidar, extrinsic);
        const bool update_ok = uobs_only_mode
            ? frontend.applyPoseMeasurement(measurement_map_T_imu,
                effective_pose_covariance, &delta, &reason)
            : frontend.applyPoseMeasurement(measurement_map_T_imu, &delta, &reason);
        if (!update_ok)
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
    if (uobs_only_mode) uobs_output.close();
    if (curvature_audit) curvature_output.close();
    if (objective_export) objective_export_output.close();
    if (objective_export && objective_export_frames != 32)
      throw std::runtime_error("P5_I1_COHORT_EXPORT_COUNT_MISMATCH:" +
          std::to_string(objective_export_frames));
    std::cout << "frames=" << limit << " lidar_updates=" << updates
              << " prediction_only=" << limit - updates << " state_finite=true"
              << " objective_export_frames=" << objective_export_frames << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FIRST_BAD_TX=" << transaction << " error=" << error.what() << '\n';
    return 1;
  }
}
