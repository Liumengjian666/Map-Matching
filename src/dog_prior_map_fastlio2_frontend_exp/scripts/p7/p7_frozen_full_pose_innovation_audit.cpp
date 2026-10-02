// POSTHOC ONLY. Reconstruct historical FULL_POSE covariance using the unchanged
// frontend and frozen raw measurements. No NDT call, GT, new estimator or policy
// feedback. Every predicted/corrected state must exactly match the frozen CSV.
#include "dog_prior_map_fastlio2_frontend_exp/mature_measurement_admission.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Row = std::map<std::string, std::string>;
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::istringstream stream(line);
  for (std::string field; std::getline(stream, field, ',');) fields.push_back(field);
  return fields;
}
std::vector<Row> readCsv(const std::string& path) {
  std::ifstream stream(path);
  std::string line;
  if (!std::getline(stream, line)) throw std::runtime_error("missing_frozen_csv:" + path);
  const auto header = split(line);
  std::vector<Row> rows;
  while (std::getline(stream, line)) {
    const auto fields = split(line);
    if (fields.size() != header.size()) throw std::runtime_error("frozen_csv_shape");
    Row row;
    for (std::size_t i = 0; i < header.size(); ++i) row.emplace(header[i], fields[i]);
    rows.push_back(row);
  }
  return rows;
}
paper::Pose3d pose(const Row& row, const std::string& prefix) {
  paper::Pose3d value;
  value.position = Eigen::Vector3d(std::stod(row.at(prefix + "_x")),
      std::stod(row.at(prefix + "_y")), std::stod(row.at(prefix + "_z")));
  value.orientation = Eigen::Quaterniond(std::stod(row.at(prefix + "_qw")),
      std::stod(row.at(prefix + "_qx")), std::stod(row.at(prefix + "_qy")),
      std::stod(row.at(prefix + "_qz")));
  return value;
}
void require(bool value, const std::string& reason) {
  if (!value) throw std::runtime_error(reason);
}
void checkPose(const paper::Pose3d& actual, const paper::Pose3d& expected) {
  require(actual.position == expected.position &&
      actual.orientation.coeffs() == expected.orientation.coeffs(), "frozen_pose_not_exact");
}
void checkVector(const Eigen::Vector3d& actual, const Row& expected, const std::string& prefix) {
  for (int i = 0; i < 3; ++i)
    require(actual[i] == std::stod(expected.at(prefix + "_" + "xyz"[i])), "frozen_state_not_exact");
}
}

int main(int argc, char** argv) {
  uint64_t tx = 0;
  try {
    require(argc == 7, "usage: audit IMU_CSV PARAMS REGISTRATION_CSV TRAJECTORY_CSV INIT_STAMP OUTPUT_CSV");
    const auto all = paper::readP7Imu(argv[1]);
    paper::Pose3d initial, extrinsic;
    const auto parameters = paper::readP7Parameters(argv[2], &initial, &extrinsic);
    const uint64_t epoch = std::stoull(argv[5]);
    require(epoch > 0, "explicit_frozen_initialization_epoch_required");
    const auto end = std::upper_bound(all.begin(), all.end(), epoch,
        [](uint64_t stamp, const paper::ImuSample& sample) { return stamp < sample.stamp_ns; });
    require(std::distance(all.begin(), end) >= parameters.static_init_samples, "causal_init_missing");
    const paper::P7ImuVector initialization(end - parameters.static_init_samples, end);
    // Frozen production inputs already passed the epoch-gap gate. This tool
    // verifies the reconstructed state exactly instead of relaxing that gate.
    paper::FastLio2IkfomFrontend frontend(parameters);
    std::string reason;
    require(frontend.initializeStatic(initialization, initial, extrinsic, &reason), reason);
    if (frontend.getState().stamp_ns < epoch)
      require(frontend.predictHeldInputTo(epoch, initialization[initialization.size() - 2],
                                         initialization.back(), &reason), reason);
    require(frontend.getState().stamp_ns == epoch, "audit_epoch_mismatch");
    const auto registrations = readCsv(argv[3]), states = readCsv(argv[4]);
    require(!states.empty() && states.size() == registrations.size(), "frozen_count_mismatch");
    Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
    noise.diagonal().head<3>().setConstant(parameters.pose_position_sigma_m * parameters.pose_position_sigma_m);
    noise.diagonal().tail<3>().setConstant(parameters.pose_rotation_sigma_rad * parameters.pose_rotation_sigma_rad);
    std::ofstream output(argv[6]);
    require(bool(output), "audit_output_failed");
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output << std::setprecision(17)
        << "transaction_id,ndt_status,distance_m,nis_valid,nis,nis_threshold,decision,fitness,"
           "transformation_probability,predicted_and_corrected_state_exact\n";
    for (std::size_t i = 0; i < states.size(); ++i) {
      const auto& registration = registrations[i];
      const auto& state = states[i];
      tx = std::stoull(state.at("transaction_id"));
      require(tx == i + 1 && registration.at("transaction_id") == state.at("transaction_id") &&
              registration.at("stamp_ns") == state.at("stamp_ns"), "frozen_identity_mismatch");
      const auto stamp = std::stoull(state.at("stamp_ns"));
      const auto imu = paper::imuWindow(all, frontend.getState().stamp_ns, stamp);
      std::vector<paper::ImuPoseSample, Eigen::aligned_allocator<paper::ImuPoseSample>> poses;
      require(frontend.predictImuSequence(imu, stamp, &poses, &reason), reason);
      checkPose(frontend.getState().map_T_imu, pose(state, "predicted_imu"));
      const auto raw = pose(registration, "raw");
      const auto measurement = paper::lidarMeasurementToImu(raw, extrinsic);
      const double distance = (raw.position - pose(registration, "initial").position).norm();
      const bool success = registration.at("status") == "SUCCESS";
      paper::ProjectedPoseInnovation innovation;
      if (success && paper::MatureMeasurementAdmission::distanceGatePass(distance)) {
        if (!frontend.evaluateProjectedPoseInnovationLinearized(measurement, noise,
            Eigen::Matrix<double, 6, 6>::Identity(), 6,
            paper::ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL, &innovation, &reason))
          innovation.valid = false;
      }
      // Independent counterfactual frame decision, NOT an admission-policy replay.
      paper::MatureMeasurementAdmission independent_decision;
      const auto admission = independent_decision.observe(success, distance, innovation);
      require(state.at("lidar_update_applied") == (success ? "1" : "0"), "not_frozen_FULL_POSE");
      if (success) {
        paper::PoseCorrectionDelta delta;
        require(frontend.applyPoseMeasurement(measurement, &delta, &reason), reason);
      }
      const auto corrected = frontend.getState();
      checkPose(corrected.map_T_imu, pose(state, "corrected_imu"));
      checkVector(corrected.velocity, state, "velocity");
      checkVector(corrected.gyro_bias, state, "gyro_bias");
      checkVector(corrected.accel_bias, state, "accel_bias");
      checkVector(corrected.gravity, state, "gravity");
      require(frontend.postconditionsValid(&reason), reason);
      output << tx << ',' << registration.at("status") << ',' << distance << ','
          << admission.nis_valid << ',' << admission.nis << ',' << admission.nis_threshold << ','
          << admission.reason << ',' << registration.at("fitness") << ','
          << registration.at("transformation_probability") << ",1\n";
    }
    output.close();
    std::cout << "FROZEN_FULL_POSE_STATE_EXACT frames=" << states.size() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "AUDIT_FAILED_TX=" << tx << ' ' << error.what() << '\n';
    return 1;
  }
}
