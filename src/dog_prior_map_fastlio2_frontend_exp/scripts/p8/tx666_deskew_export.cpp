#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace paper = dog_prior_map_fastlio2_frontend_exp;

namespace {

uint64_t uintArgument(const char* value) {
  const std::string text(value);
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("invalid_unsigned_argument");
  return std::stoull(text);
}

std::array<double, 16> matrixArgument(const char* value) {
  std::array<double, 16> matrix{};
  std::stringstream stream(value);
  std::string field;
  for (double& element : matrix) {
    if (!std::getline(stream, field, ','))
      throw std::runtime_error("transform_requires_16_comma_separated_values");
    std::size_t parsed = 0;
    element = std::stod(field, &parsed);
    if (parsed != field.size() || !std::isfinite(element))
      throw std::runtime_error("invalid_transform_element");
  }
  if (std::getline(stream, field, ','))
    throw std::runtime_error("transform_has_extra_values");
  return matrix;
}

paper::Pose3d poseFromMatrix(const std::array<double, 16>& values) {
  Eigen::Matrix4d matrix;
  for (Eigen::Index row = 0; row < 4; ++row)
    for (Eigen::Index column = 0; column < 4; ++column)
      matrix(row, column) = values[static_cast<std::size_t>(4 * row + column)];
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  if ((matrix.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-10 ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
      std::abs(rotation.determinant() - 1.0) > 1e-5)
    throw std::runtime_error("input_transform_is_not_rigid");
  paper::Pose3d pose;
  pose.position = matrix.block<3, 1>(0, 3);
  pose.orientation = Eigen::Quaterniond(rotation).normalized();
  return pose;
}

paper::P7ImuVector exactWindow(const paper::P7ImuVector& samples,
                              uint64_t start_ns, uint64_t end_ns) {
  const auto begin = std::lower_bound(samples.begin(), samples.end(), start_ns,
      [](const paper::ImuSample& sample, uint64_t stamp) {
        return sample.stamp_ns < stamp;
      });
  const auto end = std::upper_bound(begin, samples.end(), end_ns,
      [](uint64_t stamp, const paper::ImuSample& sample) {
        return stamp < sample.stamp_ns;
      });
  paper::P7ImuVector window(begin, end);
  if (window.size() != 200 || window.front().stamp_ns != start_ns ||
      window.back().stamp_ns != end_ns)
    throw std::runtime_error("static_window_does_not_match_frozen_200_sample_contract");
  return window;
}

void printMatrix(const char* name, const Eigen::Matrix4d& matrix) {
  std::cout << std::setprecision(17) << name << "=\n" << matrix << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 13)
      throw std::runtime_error(
          "usage: p8_tx666_deskew_export IMU_CSV FILTER_CSV SCAN_INDEX_CSV TIMED_POINTS_BIN "
          "PARAMS_TXT CAL_START_NS CAL_END_NS ANCHOR_NS TX MAP_T_IMU_16 T_IMU_LIDAR_16 OUT_PCD");

    const uint64_t calibration_start_ns = uintArgument(argv[6]);
    const uint64_t calibration_end_ns = uintArgument(argv[7]);
    const uint64_t anchor_ns = uintArgument(argv[8]);
    const uint64_t transaction_id = uintArgument(argv[9]);
    const paper::Pose3d map_T_imu = poseFromMatrix(matrixArgument(argv[10]));
    const paper::Pose3d imu_T_lidar = poseFromMatrix(matrixArgument(argv[11]));

    const auto imu = paper::readP7Imu(argv[1]);
    const auto scans = paper::readP7TimedScans(argv[2], argv[3]);
    paper::Pose3d parameter_pose;
    paper::Pose3d parameter_extrinsic;
    const auto parameters = paper::readP7Parameters(argv[5], &parameter_pose,
                                                    &parameter_extrinsic);
    paper::FastLio2IkfomFrontend frontend(parameters);
    std::string reason;
    paper::StaticImuCalibration calibration;
    const auto static_samples = exactWindow(imu, calibration_start_ns, calibration_end_ns);
    if (!frontend.calibrateStaticImu(static_samples, &calibration, &reason))
      throw std::runtime_error("static_imu_calibration_failed:" + reason);

    Eigen::Matrix3d R_calibration_to_anchor = Eigen::Matrix3d::Identity();
    if (!paper::integrateBodyRelativeRotation(imu, calibration.end_stamp_ns, anchor_ns,
            calibration.gyro_bias, &R_calibration_to_anchor, &reason))
      throw std::runtime_error("gravity_attitude_transport_failed:" + reason);
    const Eigen::Matrix3d R_map_imu_calibration =
        map_T_imu.orientation.toRotationMatrix() * R_calibration_to_anchor.transpose();
    const Eigen::Vector3d gravity_map = -R_map_imu_calibration *
        calibration.mean_specific_force.normalized() * calibration.gravity_mps2;
    if (!frontend.initializeFromStaticCalibration(calibration, map_T_imu,
            imu_T_lidar, gravity_map, Eigen::Vector3d::Zero(), anchor_ns, &reason))
      throw std::runtime_error("official_reanchor_failed:" + reason);

    const auto scan_it = std::lower_bound(scans.begin(), scans.end(), transaction_id,
        [](const paper::P7TimedScanRecord& scan, uint64_t id) {
          return scan.transaction_id < id;
        });
    if (scan_it == scans.end() || scan_it->transaction_id != transaction_id)
      throw std::runtime_error("requested_transaction_not_found");
    auto cloud = paper::readP7PackedTimedCloud(argv[4], *scan_it);
    if (transaction_id == 666 && cloud.size() != 29063)
      throw std::runtime_error("tx666_full_point_count_mismatch");
    paper::ScanWindowStats window;
    const auto scan_imu = paper::imuWindow(imu, scan_it->scan_start_ns,
                                           scan_it->scan_end_ns);
    if (!paper::prepareScanWindowWithCausalPreroll(scan_it->scan_start_ns,
            scan_it->scan_end_ns, anchor_ns, scan_imu, &cloud, &window, &reason))
      throw std::runtime_error("first_scan_preroll_failed:" + reason);
    if (window.overlap_points_dropped != 0 || cloud.size() != scan_it->cloud_point_count)
      throw std::runtime_error("visualization_scan_was_truncated");

    paper::ScanEndProcessor processor;
    paper::ScanEndResult scan_end;
    if (!processor.process(&frontend, imu_T_lidar, window.effective_scan_start_ns,
            scan_it->scan_end_ns, scan_imu, cloud, &scan_end, &reason, true))
      throw std::runtime_error("full_scan_deskew_failed:" + reason);
    if (scan_end.cloud_end_frame.size() != scan_it->cloud_point_count)
      throw std::runtime_error("deskew_output_point_count_mismatch");

    pcl::PointCloud<pcl::PointXYZ> output;
    output.reserve(scan_end.cloud_end_frame.size());
    for (const auto& point : scan_end.cloud_end_frame) {
      pcl::PointXYZ xyz;
      xyz.x = static_cast<float>(point.position.x());
      xyz.y = static_cast<float>(point.position.y());
      xyz.z = static_cast<float>(point.position.z());
      output.push_back(xyz);
    }
    output.width = static_cast<uint32_t>(output.size());
    output.height = 1;
    output.is_dense = true;
    if (pcl::io::savePCDFileBinary(argv[12], output) != 0)
      throw std::runtime_error("cannot_write_deskewed_scan_pcd");

    const Eigen::Isometry3d map_T_lidar = paper::asIsometry(scan_end.predicted_map_T_lidar);
    std::cout << "TX=" << transaction_id << "\n"
        << "SCAN_START_NS=" << scan_it->scan_start_ns << "\n"
        << "SCAN_END_NS=" << scan_it->scan_end_ns << "\n"
        << "POINTS_RAW=" << cloud.size() << "\n"
        << "POINTS_DESKEWED=" << output.size() << "\n"
        << "POINTS_DROPPED=" << window.overlap_points_dropped << "\n"
        << "DESKEW_REFERENCE=scan_end_lidar_frame\n";
    printMatrix("T_NORMALIZED_PREDICTED_LIDAR_AT_SCAN_END", map_T_lidar.matrix());
    std::cout << "OUTPUT_PCD=" << argv[12] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "TX666_DESKEW_EXPORT_FAILED=" << error.what() << '\n';
    return 2;
  }
}
