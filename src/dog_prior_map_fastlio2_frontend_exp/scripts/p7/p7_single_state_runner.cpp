#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/solution_remapping_baseline.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

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
void matrixHeader(std::ostream& out, const char* prefix) {
  for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col)
    out << ',' << prefix << "_r" << row << "c" << col;
}
void matrixColumns(std::ostream& out, const paper::reliability::Matrix6d& matrix) {
  for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col)
    out << ',' << matrix(row, col);
}
double residentRssKb() {
  std::ifstream statm("/proc/self/statm");
  uint64_t total_pages = 0, resident_pages = 0;
  const long page_size = sysconf(_SC_PAGESIZE);
  if (!(statm >> total_pages >> resident_pages) || page_size <= 0)
    return std::numeric_limits<double>::quiet_NaN();
  return static_cast<double>(resident_pages) * static_cast<double>(page_size) / 1024.0;
}
}  // namespace

int main(int argc, char** argv) {
  uint64_t transaction = 0;
  try {
    if (argc != 15)
      throw std::runtime_error("usage: p7_single_state_runner IMU_CSV FILTER_SCANS_CSV SCANS_CSV "
          "REQUEST_XYZ_F32_BIN MAP_PCD PARAMS_TXT TRAJECTORY_CSV REGISTRATION_CSV RUNTIME_CSV "
          "UOBS_CSV FRAME_LIMIT INITIALIZATION_STAMP_NS FULL_POSE|MATURE_SOL_REMAP SOLUTION_REMAP_CSV");
    const std::string mode(argv[13]);
    if (mode != "FULL_POSE" && mode != "MATURE_SOL_REMAP")
      throw std::runtime_error("invalid_baseline_mode");
    const auto all_imu = paper::readP7Imu(argv[1]);
    const auto scans = paper::readP7Scans(argv[2], argv[3]);
    const uint64_t limit = unsignedArgument(argv[11]);
    const uint64_t initialization_stamp = unsignedArgument(argv[12]);
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
    const paper::CurrentFrameNdtParameters ndt_parameters;
    paper::CurrentFrameNdtRegistration registration{ndt_parameters};
    if (!registration.loadMap(argv[5], &reason)) throw std::runtime_error("map_load_failed:" + reason);
    std::ofstream trajectory(argv[7]), observations(argv[8]), runtime(argv[9]), uobs(argv[10]), remap_csv(argv[14]);
    if (!trajectory || !observations || !runtime || !uobs || !remap_csv)
      throw std::runtime_error("cannot_create_outputs");
    trajectory.exceptions(std::ios::badbit | std::ios::failbit);
    observations.exceptions(std::ios::badbit | std::ios::failbit);
    runtime.exceptions(std::ios::badbit | std::ios::failbit);
    uobs.exceptions(std::ios::badbit | std::ios::failbit);
    remap_csv.exceptions(std::ios::badbit | std::ios::failbit);
    trajectory << std::setprecision(17)
        << "transaction_id,stamp_ns,predicted_imu_x,predicted_imu_y,predicted_imu_z,"
           "predicted_imu_qx,predicted_imu_qy,predicted_imu_qz,predicted_imu_qw,"
           "corrected_imu_x,corrected_imu_y,corrected_imu_z,corrected_imu_qx,corrected_imu_qy,"
           "corrected_imu_qz,corrected_imu_qw,velocity_x,velocity_y,velocity_z,"
           "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
           "gravity_x,gravity_y,gravity_z,lidar_update_applied\n";
    observations << std::setprecision(17)
        << "transaction_id,stamp_ns,source_points,target_points,source_hash_expected_available,"
           "source_hash_expected,source_hash_actual,source_hash_match,converged,effective,status,"
           "iterations,fitness,transformation_probability,alignment_ms,"
           "initial_x,initial_y,initial_z,initial_qx,initial_qy,initial_qz,initial_qw,"
           "raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,lidar_update_applied\n";
    runtime << std::setprecision(17)
        << "transaction_id,prediction_ms,cloud_io_ms,ndt_total_ms,ndt_alignment_ms,"
           "ikfom_update_ms,frame_total_ms,ndt_effective,lidar_update_applied,"
           "uobs_ms,classifier_ms,resident_rss_kb,solution_remap_ms\n";
    remap_csv << std::setprecision(17)
        << "transaction_id,stamp_ns,mode,weak_dimension,reliable_dimension,"
           "raw_rotation_correction_norm,raw_translation_correction_norm_m,"
           "safe_rotation_correction_norm,safe_translation_correction_norm_m,"
           "removed_rotation_norm,removed_translation_norm_m,"
           "projector_symmetry_error,projector_idempotence_error,remap_valid,"
           "lidar_update_applied,remap_status\n";
    uobs << std::setprecision(17)
        << "transaction_id,stamp_ns,ndt_effective,uobs_computed,uobs_valid,uobs_status,"
           "map_support_sufficient,map_support_status,valid_correspondences,rejected_covariances,"
           "effective_weight_sum,translation_length_scale_m,schur_valid,schur_status,"
           "lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,lambda_max,numerical_floor,"
           "weak_threshold,weak_relative_ratio,weak_dimension,reliable_dimension,"
           "classification_status,classification_valid,"
           "DIAGNOSTIC_ONLY_rotation_schur_lambda0,DIAGNOSTIC_ONLY_rotation_schur_lambda1,"
           "DIAGNOSTIC_ONLY_rotation_schur_lambda2,DIAGNOSTIC_ONLY_translation_schur_lambda0,"
           "DIAGNOSTIC_ONLY_translation_schur_lambda1,DIAGNOSTIC_ONLY_translation_schur_lambda2";
    matrixHeader(uobs, "Hphys"); matrixHeader(uobs, "Hbar");
    matrixHeader(uobs, "weak_basis"); matrixHeader(uobs, "reliable_basis");
    uobs << '\n';
    uint64_t updates = 0;
    for (uint64_t index = 0; index < limit; ++index) {
      const auto& scan = scans.at(index);
      transaction = scan.transaction_id;
      const auto frame_start = Clock::now();
      const auto prediction_start = Clock::now();
      const auto start = frontend.getState();
      const auto causal_imu = paper::imuWindow(all_imu, start.stamp_ns, scan.stamp_ns);
      std::vector<paper::ImuPoseSample, Eigen::aligned_allocator<paper::ImuPoseSample>> poses;
      if (!frontend.predictImuSequence(causal_imu, scan.stamp_ns, &poses, &reason))
        throw std::runtime_error("prediction_failed:" + reason);
      const auto predicted = frontend.getState();
      const double prediction_ms = elapsedMs(prediction_start);
      const auto initial_guess = paper::fromIsometry(
          paper::asIsometry(predicted.map_T_imu) * paper::asIsometry(extrinsic));
      const auto io_start = Clock::now();
      const auto cloud = paper::readP7PackedCloud(argv[4], scan);
      const double cloud_io_ms = elapsedMs(io_start);
      const auto ndt_start = Clock::now();
      paper::CurrentFrameNdtResult result;
      if (!registration.align(scan.stamp_ns, cloud, initial_guess, &result, &reason))
        throw std::runtime_error("registration_internal_error:" + reason);
      const double uobs_ms = result.uobs_computed ? result.uobs_ms : 0.0;
      // NDT alignment timing is untouched; geometric analysis is reported separately.
      const double ndt_total_ms = elapsedMs(ndt_start) - uobs_ms;
      const bool hash_match = scan.expected_source_hash_available &&
          scan.expected_source_hash == result.source_cloud_hash;
      if (scan.expected_source_hash_available && !hash_match)
        throw std::runtime_error("source_cloud_hash_mismatch");
      paper::reliability::FixedPhysicalJointSubspace subspace;
      subspace.status = "NOT_COMPUTED";
      double classifier_ms = 0.0;
      if (result.uobs_computed) {
        const auto classifier_start = Clock::now();
        subspace = paper::reliability::classifyFixedPhysicalJointSubspace(result.local_observability);
        classifier_ms = elapsedMs(classifier_start);
      }
      // FULL_POSE retains the P7-C mean/noise/admission contract exactly.
      // MATURE_SOL_REMAP only remaps the mean, not the filter covariance update.
      paper::SolutionRemappingBaselineResult remapping;
      remapping.status = result.effective ? "FULL_POSE_NOT_REMAPPED" : "NDT_INEFFECTIVE";
      paper::Pose3d measurement = result.raw_map_T_lidar;
      bool lidar_update_applied = result.effective;
      double remap_ms = 0.0;
      if (mode == "MATURE_SOL_REMAP" && result.effective) {
        const auto remap_start = Clock::now();
        remapping = paper::remapNdtSolutionBaseline(result.initial_map_T_lidar,
            result.raw_map_T_lidar, result.local_observability, subspace);
        remap_ms = elapsedMs(remap_start);
        lidar_update_applied = remapping.valid && remapping.lidar_measurement_available;
        if (lidar_update_applied) measurement = remapping.remapped_map_T_lidar;
      }
      const auto update_start = Clock::now();
      if (lidar_update_applied) {
        paper::PoseCorrectionDelta delta;
        if (!frontend.applyPoseMeasurement(
            paper::lidarMeasurementToImu(measurement, extrinsic), &delta, &reason))
          throw std::runtime_error("pose_update_failed:" + reason);
        ++updates;
      }
      const double update_ms = elapsedMs(update_start);
      const auto corrected = frontend.getState();
      if (corrected.stamp_ns != scan.stamp_ns) throw std::runtime_error("state_timestamp_mismatch");
      if (!frontend.postconditionsValid(&reason)) throw std::runtime_error("postconditions_failed:" + reason);
      const double total_ms = elapsedMs(frame_start);
      const double rss_kb = residentRssKb();
      trajectory << transaction << ',' << scan.stamp_ns;
      poseColumns(trajectory, predicted.map_T_imu); poseColumns(trajectory, corrected.map_T_imu);
      vectorColumns(trajectory, corrected.velocity); vectorColumns(trajectory, corrected.gyro_bias);
      vectorColumns(trajectory, corrected.accel_bias); vectorColumns(trajectory, corrected.gravity);
      trajectory << ',' << lidar_update_applied << '\n';
      observations << transaction << ',' << scan.stamp_ns << ',' << result.source_point_count
          << ',' << result.target_point_count << ',' << scan.expected_source_hash_available
          << ',' << scan.expected_source_hash << ',' << result.source_cloud_hash << ','
          << (scan.expected_source_hash_available ? (hash_match ? "1" : "0") : "NA")
          << ',' << result.converged << ',' << result.effective << ','
          << paper::currentFrameNdtStatusName(result.status) << ',' << result.iterations << ','
          << result.fitness << ',' << result.transformation_probability << ',' << result.alignment_ms;
      poseColumns(observations, result.initial_map_T_lidar); poseColumns(observations, result.raw_map_T_lidar);
      observations << ',' << lidar_update_applied << '\n';
      runtime << transaction << ',' << prediction_ms << ',' << cloud_io_ms << ',' << ndt_total_ms
          << ',' << result.alignment_ms << ',' << update_ms << ',' << total_ms << ','
          << result.effective << ',' << lidar_update_applied << ',' << uobs_ms << ',' << classifier_ms
          << ',' << rss_kb << ',' << remap_ms << '\n';
      const auto& local = result.local_observability;
      uobs << transaction << ',' << scan.stamp_ns << ',' << result.effective << ','
          << result.uobs_computed << ',' << local.valid << ','
          << (result.uobs_computed ? local.status : "NOT_COMPUTED") << ','
          << local.map_support_sufficient << ',' << local.map_support_status << ','
          << local.valid_correspondence_count << ',' << local.rejected_covariance_count << ','
          << local.effective_weight_sum << ',' << ndt_parameters.resolution_m << ','
          << local.schur_decoupling_valid << ',' << local.schur_status;
      for (int i = 0; i < 6; ++i) uobs << ',' << local.joint_eigenvalues(i);
      uobs << ',' << subspace.lambda_max << ',' << subspace.numerical_floor << ','
          << subspace.weak_threshold << ',' << subspace.weak_relative_ratio << ','
          << subspace.weak_dimension << ',' << subspace.reliable_dimension << ','
          << subspace.status << ',' << subspace.valid;
      vectorColumns(uobs, local.rotation_schur_eigenvalues);
      vectorColumns(uobs, local.translation_schur_eigenvalues);
      matrixColumns(uobs, local.physical_geometric_information);
      matrixColumns(uobs, local.normalized_geometric_information);
      matrixColumns(uobs, subspace.weak_basis); matrixColumns(uobs, subspace.reliable_basis);
      uobs << '\n';
      const double nan = std::numeric_limits<double>::quiet_NaN();
      double raw_rotation = nan, raw_translation = nan;
      double safe_rotation = nan, safe_translation = nan, removed_rotation = nan, removed_translation = nan;
      if (result.effective) {
        raw_rotation = paper::so3Log(result.raw_map_T_lidar.orientation.toRotationMatrix() *
            result.initial_map_T_lidar.orientation.toRotationMatrix().transpose()).norm();
        raw_translation = (result.raw_map_T_lidar.position - result.initial_map_T_lidar.position).norm();
        if (mode == "FULL_POSE") {
          safe_rotation = raw_rotation; safe_translation = raw_translation;
          removed_rotation = removed_translation = 0.0;
        } else if (remapping.valid) {
          safe_rotation = remapping.safe_correction.head<3>().norm();
          safe_translation = remapping.translation_length_scale_m * remapping.safe_correction.tail<3>().norm();
          removed_rotation = remapping.removed_correction.head<3>().norm();
          removed_translation = remapping.translation_length_scale_m * remapping.removed_correction.tail<3>().norm();
        }
      }
      remap_csv << transaction << ',' << scan.stamp_ns << ',' << mode << ',' << subspace.weak_dimension
          << ',' << subspace.reliable_dimension << ',' << raw_rotation << ',' << raw_translation
          << ',' << safe_rotation << ',' << safe_translation << ',' << removed_rotation << ',' << removed_translation
          << ',' << remapping.projector_symmetry_error << ',' << remapping.projector_idempotence_error
          << ',' << remapping.valid << ',' << lidar_update_applied << ',' << remapping.status << '\n';
    }
    trajectory.close(); observations.close(); runtime.close(); uobs.close(); remap_csv.close();
    std::cout << "mode=" << mode << " frames=" << limit << " lidar_updates=" << updates
              << " prediction_only=" << limit - updates << " state_finite=true\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FIRST_BAD_TX=" << transaction << " error=" << error.what() << '\n';
    return 1;
  }
}
