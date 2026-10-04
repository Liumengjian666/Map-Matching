#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

struct FastState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp_ns = 0;
  Eigen::Matrix3d R_fast_world_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_fast_world_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_fast_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba_scaled_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_fast_world = Eigen::Vector3d::Zero();
  double mean_acc_norm_raw = 0.0;
  double accel_input_scale = 0.0;
};

uint64_t parseUnsigned(const std::string& text) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("invalid_unsigned_field");
  return std::stoull(text);
}

std::vector<double> parseCsvNumbers(const std::string& line) {
  std::vector<double> values;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) {
    std::size_t end = 0;
    const double value = std::stod(field, &end);
    if (end != field.size() || !std::isfinite(value))
      throw std::runtime_error("invalid_fastlio_state_field");
    values.push_back(value);
  }
  return values;
}

std::vector<FastState, Eigen::aligned_allocator<FastState>> readFastStates(
    const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_fastlio_anchor_state_csv");
  std::vector<FastState, Eigen::aligned_allocator<FastState>> states;
  std::string line;
  uint64_t previous = 0;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::stringstream split(line);
    std::string stamp_text;
    if (!std::getline(split, stamp_text, ','))
      throw std::runtime_error("missing_fastlio_state_timestamp");
    FastState state;
    state.stamp_ns = parseUnsigned(stamp_text);
    std::string rest;
    if (!std::getline(split, rest)) throw std::runtime_error("empty_fastlio_state_row");
    const std::vector<double> values = parseCsvNumbers(rest);
    if (values.size() != 26 || state.stamp_ns <= previous)
      throw std::runtime_error("invalid_fastlio_state_row_width_or_order");
    std::size_t i = 0;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        state.R_fast_world_imu(row, col) = values[i++];
    state.p_fast_world_imu = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
    state.v_fast_world = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
    state.bg_imu = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
    state.ba_scaled_imu = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
    state.gravity_fast_world = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
    state.mean_acc_norm_raw = values[i++];
    state.accel_input_scale = values[i++];
    if (!state.R_fast_world_imu.allFinite() || !state.p_fast_world_imu.allFinite() ||
        !state.v_fast_world.allFinite() || !state.bg_imu.allFinite() ||
        !state.ba_scaled_imu.allFinite() || !state.gravity_fast_world.allFinite() ||
        state.mean_acc_norm_raw <= 0.0 || state.accel_input_scale <= 0.0 ||
        std::abs(state.accel_input_scale - 9.81 / state.mean_acc_norm_raw) > 1e-8)
      throw std::runtime_error("invalid_fastlio_state_values_or_accel_scale");
    previous = state.stamp_ns;
    states.push_back(state);
  }
  if (states.size() < 2) throw std::runtime_error("insufficient_fastlio_scan_internal_states");
  return states;
}

FastState interpolateFastState(
    const std::vector<FastState, Eigen::aligned_allocator<FastState>>& states,
    uint64_t stamp_ns) {
  const auto upper = std::lower_bound(states.begin(), states.end(), stamp_ns,
      [](const FastState& state, uint64_t stamp) { return state.stamp_ns < stamp; });
  if (upper != states.end() && upper->stamp_ns == stamp_ns) return *upper;
  if (upper == states.begin() || upper == states.end())
    throw std::runtime_error("anchor_not_bracketed_by_fastlio_scan_states");
  const FastState& lower = *(upper - 1);
  const double alpha = static_cast<double>(stamp_ns - lower.stamp_ns) /
      static_cast<double>(upper->stamp_ns - lower.stamp_ns);
  FastState result;
  result.stamp_ns = stamp_ns;
  Eigen::Quaterniond q0(lower.R_fast_world_imu), q1(upper->R_fast_world_imu);
  result.R_fast_world_imu = q0.slerp(alpha, q1).normalized().toRotationMatrix();
  result.p_fast_world_imu = (1.0 - alpha) * lower.p_fast_world_imu + alpha * upper->p_fast_world_imu;
  result.v_fast_world = (1.0 - alpha) * lower.v_fast_world + alpha * upper->v_fast_world;
  result.bg_imu = (1.0 - alpha) * lower.bg_imu + alpha * upper->bg_imu;
  result.ba_scaled_imu = (1.0 - alpha) * lower.ba_scaled_imu + alpha * upper->ba_scaled_imu;
  result.gravity_fast_world = (1.0 - alpha) * lower.gravity_fast_world +
                              alpha * upper->gravity_fast_world;
  result.mean_acc_norm_raw = (1.0 - alpha) * lower.mean_acc_norm_raw +
                             alpha * upper->mean_acc_norm_raw;
  result.accel_input_scale = 9.81 / result.mean_acc_norm_raw;
  return result;
}

paper::Pose3d officialPose() {
  const Eigen::Matrix3d raw_rotation = (Eigen::Matrix3d() <<
      0.135990, -0.990409, -0.024406,
      0.990705,  0.136027,  0.000140,
      0.003181, -0.024198,  0.999702).finished();
  paper::Pose3d pose;
  pose.position = Eigen::Vector3d(1.968147, -6.879292, -0.896125);
  // The P7 state is SO(3)-valued. This is Eigen's direct matrix-to-quaternion
  // conversion, normalized for the state API; no SVD/nearest-SO(3) is applied.
  pose.orientation = Eigen::Quaterniond(raw_rotation).normalized();
  return pose;
}

paper::P7ImuVector staticWindow(const paper::P7ImuVector& imu, int count,
                                uint64_t expected_start, uint64_t expected_end) {
  const auto end = std::lower_bound(imu.begin(), imu.end(), expected_end,
      [](const paper::ImuSample& sample, uint64_t stamp) { return sample.stamp_ns < stamp; });
  if (end == imu.end() || end->stamp_ns != expected_end ||
      std::distance(imu.begin(), end) + 1 < count)
    throw std::runtime_error("frozen_static_window_samples_missing");
  const auto begin = end - (count - 1);
  if (begin->stamp_ns != expected_start)
    throw std::runtime_error("frozen_static_window_start_mismatch");
  return paper::P7ImuVector(begin, end + 1);
}

double quantile(std::vector<double> values, double probability) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double index = probability * static_cast<double>(values.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(index));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lo);
  return (1.0 - alpha) * values[lo] + alpha * values[hi];
}

struct DistanceSummary { double mean = 0.0, median = 0.0, p95 = 0.0, maximum = 0.0; };
DistanceSummary summarize(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("cannot_summarize_empty_distances");
  DistanceSummary result;
  double total = 0.0;
  for (double value : values) { total += value; result.maximum = std::max(result.maximum, value); }
  result.mean = total / static_cast<double>(values.size());
  result.median = quantile(values, 0.50);
  result.p95 = quantile(values, 0.95);
  return result;
}

void cloudDifference(
    const std::vector<paper::TimedLidarPoint, Eigen::aligned_allocator<paper::TimedLidarPoint>>& a,
    const std::vector<paper::TimedLidarPoint, Eigen::aligned_allocator<paper::TimedLidarPoint>>& b,
    const std::string& output_path) {
  if (a.size() != b.size() || a.empty()) throw std::runtime_error("deskew_cloud_order_or_size_mismatch");
  std::vector<double> same_index;
  same_index.reserve(a.size());
  std::array<std::vector<double>, 5> bins;
  std::ofstream rows(output_path);
  if (!rows) throw std::runtime_error("cannot_create_deskew_point_diff_csv");
  rows << std::setprecision(17) << "stamp_ns,dx,dy,dz,distance_m\n";
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].stamp_ns != b[i].stamp_ns)
      throw std::runtime_error("deskew_output_timestamp_order_mismatch");
    const Eigen::Vector3d delta = a[i].position - b[i].position;
    const double distance = delta.norm();
    same_index.push_back(distance);
    const double offset_ms = static_cast<double>(a[i].stamp_ns - 1517157286155932903ULL) * 1e-6;
    const int bin = offset_ms < 20.0 ? 0 : offset_ms < 40.0 ? 1 :
                    offset_ms < 60.0 ? 2 : offset_ms < 80.0 ? 3 : 4;
    bins[static_cast<std::size_t>(bin)].push_back(distance);
    rows << a[i].stamp_ns << ',' << delta.x() << ',' << delta.y() << ','
         << delta.z() << ',' << distance << '\n';
  }

  Cloud::Ptr ca(new Cloud), cb(new Cloud);
  ca->reserve(a.size()); cb->reserve(b.size());
  for (const auto& point : a) ca->push_back(Point(
      static_cast<float>(point.position.x()), static_cast<float>(point.position.y()),
      static_cast<float>(point.position.z())));
  for (const auto& point : b) cb->push_back(Point(
      static_cast<float>(point.position.x()), static_cast<float>(point.position.y()),
      static_cast<float>(point.position.z())));
  pcl::KdTreeFLANN<Point> tree_a, tree_b;
  tree_a.setInputCloud(ca); tree_b.setInputCloud(cb);
  auto nearest = [](const Cloud::Ptr& queries, pcl::KdTreeFLANN<Point>* tree) {
    std::vector<double> result;
    result.reserve(queries->size());
    std::vector<int> index(1);
    std::vector<float> squared(1);
    for (const Point& query : queries->points) {
      if (tree->nearestKSearch(query, 1, index, squared) != 1 || squared[0] < 0.0f)
        throw std::runtime_error("deskew_nn_query_failed");
      result.push_back(std::sqrt(static_cast<double>(squared[0])));
    }
    return summarize(std::move(result));
  };
  const DistanceSummary same = summarize(std::move(same_index));
  const DistanceSummary nn_a_to_b = nearest(ca, &tree_b);
  const DistanceSummary nn_b_to_a = nearest(cb, &tree_a);
  std::cout << "DESKEW_SAME_INDEX mean=" << same.mean << " median=" << same.median
            << " p95=" << same.p95 << " max=" << same.maximum << '\n'
            << "DESKEW_NN_CURRENT_TO_FULL mean=" << nn_a_to_b.mean << " median="
            << nn_a_to_b.median << " p95=" << nn_a_to_b.p95 << " max=" << nn_a_to_b.maximum << '\n'
            << "DESKEW_NN_FULL_TO_CURRENT mean=" << nn_b_to_a.mean << " median="
            << nn_b_to_a.median << " p95=" << nn_b_to_a.p95 << " max=" << nn_b_to_a.maximum << '\n';
  static const char* labels[] = {"0_20ms", "20_40ms", "40_60ms", "60_80ms", "80ms_end"};
  std::ofstream bins_out(output_path.substr(0, output_path.find_last_of('/')) + "/deskew_time_bins.csv");
  if (!bins_out) throw std::runtime_error("cannot_create_deskew_time_bins_csv");
  bins_out << std::setprecision(17) << "bin,count,median_m,p95_m\n";
  for (std::size_t i = 0; i < bins.size(); ++i) {
    if (bins[i].empty()) throw std::runtime_error("empty_deskew_time_bin");
    bins_out << labels[i] << ',' << bins[i].size() << ',' << quantile(bins[i], 0.50)
             << ',' << quantile(bins[i], 0.95) << '\n';
    std::cout << "DESKEW_BIN " << labels[i] << " count=" << bins[i].size()
              << " median=" << quantile(bins[i], 0.50) << " p95="
              << quantile(bins[i], 0.95) << '\n';
  }
}

struct CaseResult {
  paper::FilterSnapshot anchor;
  paper::ScanEndResult scan_end;
  paper::CurrentFrameNdtResult ndt;
  paper::CurrentFrameNdtOverlap initial_overlap;
  paper::CurrentFrameNdtOverlap final_overlap;
  std::size_t input_points = 0;
};

CaseResult runCase(paper::FastLio2IkfomFrontend* frontend,
                   const paper::Pose3d& extrinsic, const paper::P7ImuVector& imu,
                   const paper::P7TimedLidarVector& cloud,
                   const paper::P7TimedScanRecord& scan,
                   const std::string& map_path, const std::string& case_name) {
  if (!frontend || !frontend->initialized()) throw std::runtime_error("case_frontend_not_initialized");
  CaseResult result;
  result.anchor = frontend->getState();
  result.input_points = cloud.size();
  if (result.anchor.stamp_ns >= scan.scan_end_ns || scan.scan_start_ns >= scan.scan_end_ns)
    throw std::runtime_error("unexpected_tx666_anchor_scan_order");
  paper::ScanEndProcessor processor;
  std::string reason;
  if (!processor.process(frontend, extrinsic, scan.scan_start_ns, scan.scan_end_ns,
                         imu, cloud, &result.scan_end, &reason, true))
    throw std::runtime_error(case_name + "_scan_propagation_or_deskew_failed:" + reason);
  if (result.scan_end.cloud_end_frame.size() != cloud.size())
    throw std::runtime_error(case_name + "_deskew_dropped_or_added_points");
  paper::RegistrationCloud source;
  source.reserve(result.scan_end.cloud_end_frame.size());
  for (const auto& point : result.scan_end.cloud_end_frame)
    source.push_back({static_cast<float>(point.position.x()),
                      static_cast<float>(point.position.y()),
                      static_cast<float>(point.position.z())});
  paper::CurrentFrameNdtRegistration registration(paper::CurrentFrameNdtParameters{});
  if (!registration.loadMap(map_path, &reason))
    throw std::runtime_error(case_name + "_map_load_failed:" + reason);
  if (!registration.evaluateMapOverlap(source, result.scan_end.predicted_map_T_lidar,
                                       &result.initial_overlap, &reason))
    throw std::runtime_error(case_name + "_initial_overlap_failed:" + reason);
  if (!registration.align(scan.scan_end_ns, source, result.scan_end.predicted_map_T_lidar,
                          &result.ndt, &reason))
    throw std::runtime_error(case_name + "_ndt_failed:" + reason);
  if (!registration.evaluateMapOverlap(source, result.ndt.raw_map_T_lidar,
                                       &result.final_overlap, &reason))
    throw std::runtime_error(case_name + "_final_overlap_failed:" + reason);
  const auto post = frontend->getState();
  if (post.stamp_ns != scan.scan_end_ns)
    throw std::runtime_error(case_name + "_scan_end_state_stamp_mismatch");
  return result;
}

double angleBetween(const paper::Pose3d& a, const paper::Pose3d& b) {
  Eigen::Quaterniond relative = a.orientation.normalized().conjugate() * b.orientation.normalized();
  relative.normalize();
  return Eigen::AngleAxisd(relative).angle();
}

void printPose(const char* label, const paper::Pose3d& pose) {
  std::cout << label << " xyz=" << pose.position.transpose()
            << " q_xyzw=" << pose.orientation.x() << ' ' << pose.orientation.y() << ' '
            << pose.orientation.z() << ' ' << pose.orientation.w()
            << " R=\n" << pose.orientation.toRotationMatrix() << '\n';
}

void printCase(const char* label, const CaseResult& r) {
  const double correction =
      (r.ndt.raw_map_T_lidar.position - r.ndt.initial_map_T_lidar.position).norm();
  const double angle = angleBetween(r.ndt.initial_map_T_lidar, r.ndt.raw_map_T_lidar);
  std::cout << label << "_ANCHOR stamp_ns=" << r.anchor.stamp_ns
            << " pose_p=" << r.anchor.map_T_imu.position.transpose()
            << " velocity=" << r.anchor.velocity.transpose()
            << " gyro_bias=" << r.anchor.gyro_bias.transpose()
            << " accel_bias=" << r.anchor.accel_bias.transpose()
            << " gravity=" << r.anchor.gravity.transpose() << '\n';
  printPose((std::string(label) + "_ANCHOR_IMU").c_str(), r.anchor.map_T_imu);
  printPose((std::string(label) + "_SCANEND_PREDICTED_IMU").c_str(), r.scan_end.predicted_map_T_imu);
  printPose((std::string(label) + "_SCANEND_PREDICTED_LIDAR").c_str(), r.scan_end.predicted_map_T_lidar);
  std::cout << label << "_ANCHOR_TO_SCANEND translation_m="
            << (r.scan_end.predicted_map_T_imu.position - r.anchor.map_T_imu.position).norm()
            << " rotation_rad=" << angleBetween(r.anchor.map_T_imu,
                                                 r.scan_end.predicted_map_T_imu)
            << " rotation_deg=" << angleBetween(r.anchor.map_T_imu,
                                                 r.scan_end.predicted_map_T_imu) * 180.0 / M_PI
            << '\n';
  std::cout << label << "_NDT points=" << r.input_points
            << " source_points=" << r.ndt.source_point_count
            << " target_points=" << r.ndt.target_point_count
            << " source_hash=" << r.ndt.source_cloud_hash
            << " converged=" << r.ndt.converged
            << " iterations=" << r.ndt.iterations
            << " status=" << paper::currentFrameNdtStatusName(r.ndt.status)
            << " correction_m=" << correction << " correction_rad=" << angle
            << " correction_deg=" << angle * 180.0 / M_PI
            << " fitness=" << r.ndt.fitness
            << " initial_overlap=" << r.initial_overlap.fraction_within[0] << '/'
            << r.initial_overlap.fraction_within[1] << '/' << r.initial_overlap.fraction_within[2]
            << '/' << r.initial_overlap.fraction_within[3]
            << " initial_nn_mean_median_p90_p95=" << r.initial_overlap.mean_nearest_distance_m
            << '/' << r.initial_overlap.median_nearest_distance_m << '/'
            << r.initial_overlap.p90_nearest_distance_m << '/'
            << r.initial_overlap.p95_nearest_distance_m
            << " final_overlap=" << r.final_overlap.fraction_within[0] << '/'
            << r.final_overlap.fraction_within[1] << '/' << r.final_overlap.fraction_within[2]
            << '/' << r.final_overlap.fraction_within[3]
            << " final_nn_mean_median_p90_p95=" << r.final_overlap.mean_nearest_distance_m
            << '/' << r.final_overlap.median_nearest_distance_m << '/'
            << r.final_overlap.p90_nearest_distance_m << '/'
            << r.final_overlap.p95_nearest_distance_m << '\n';
  printPose((std::string(label) + "_NDT_TERMINAL_LIDAR").c_str(), r.ndt.raw_map_T_lidar);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 9)
      throw std::runtime_error("usage: p8_corridor01_full_motion_state_tx666 "
          "IMU_CSV FILTER_SCANS_CSV TIMED_SCAN_INDEX_CSV TIMED_POINTS_BIN "
          "RAW_MAP_PCD P7_PARAMS FASTLIO_TX666_STATE_CSV OUTPUT_DIR");
    const uint64_t anchor_ns = 1517157286165072000ULL;
    const uint64_t tx666_start_ns = 1517157286155932903ULL;
    const uint64_t tx666_end_ns = 1517157286256772352ULL;
    const int tx666 = 666;
    const auto all_imu = paper::readP7Imu(argv[1]);
    const auto scans = paper::readP7TimedScans(argv[2], argv[3]);
    const auto scan_it = std::lower_bound(scans.begin(), scans.end(), tx666,
        [](const paper::P7TimedScanRecord& scan, uint64_t tx) { return scan.transaction_id < tx; });
    if (scan_it == scans.end() || scan_it->transaction_id != tx666 ||
        scan_it->scan_start_ns != tx666_start_ns || scan_it->scan_end_ns != tx666_end_ns ||
        scan_it->cloud_point_count != 29063)
      throw std::runtime_error("TX666_manifest_contract_mismatch");
    const paper::P7TimedLidarVector raw_cloud = paper::readP7PackedTimedCloud(argv[4], *scan_it);
    if (raw_cloud.size() != 29063) throw std::runtime_error("TX666_timed_cloud_point_count_mismatch");

    const auto fast_states = readFastStates(argv[7]);
    const FastState fast = interpolateFastState(fast_states, anchor_ns);
    const auto all_static = staticWindow(all_imu, 200,
        1517157224023904000ULL, 1517157225018848000ULL);

    paper::Pose3d ignored_initial, T_imu_lidar;
    paper::RuntimeParameters current_parameters =
        paper::readP7Parameters(argv[6], &ignored_initial, &T_imu_lidar);
    paper::FastLio2IkfomFrontend current_frontend(current_parameters);
    paper::StaticImuCalibration current_calibration;
    std::string reason;
    if (!current_frontend.calibrateStaticImu(all_static, &current_calibration, &reason))
      throw std::runtime_error("current_static_calibration_failed:" + reason);
    const paper::Pose3d official_world_T_imu = officialPose();
    Eigen::Matrix3d R_cal_to_anchor;
    if (!paper::integrateBodyRelativeRotation(all_imu, current_calibration.end_stamp_ns,
          anchor_ns, current_calibration.gyro_bias, &R_cal_to_anchor, &reason))
      throw std::runtime_error("current_static_gravity_transport_failed:" + reason);
    const Eigen::Matrix3d R_world_imu_cal =
        official_world_T_imu.orientation.toRotationMatrix() * R_cal_to_anchor.transpose();
    const Eigen::Vector3d current_gravity = -R_world_imu_cal *
        current_calibration.mean_specific_force.normalized() * current_parameters.gravity_mps2;
    if (!current_frontend.initializeFromStaticCalibration(current_calibration,
          official_world_T_imu, T_imu_lidar, current_gravity, Eigen::Vector3d::Zero(),
          anchor_ns, &reason))
      throw std::runtime_error("current_state_initialize_failed:" + reason);

    // FAST-LIO rot maps IMU/body coordinates into FAST-LIO's local world.
    // P7 velocity/gravity use the official raw-map frame, so align only the
    // world axes while preserving the official pose as P7's absolute anchor.
    const Eigen::Matrix3d R_map_fast_world =
        official_world_T_imu.orientation.toRotationMatrix() *
        fast.R_fast_world_imu.transpose();
    const Eigen::Vector3d full_velocity = R_map_fast_world * fast.v_fast_world;
    const Eigen::Vector3d full_gyro_bias = fast.bg_imu;
    // FAST-LIO scales every raw accelerometer sample before estimation. Its
    // ba therefore scales with the measurement; divide by that factor to
    // express the same correction in P7's unscaled m/s^2 IMU input convention.
    const Eigen::Vector3d full_accel_bias = fast.ba_scaled_imu / fast.accel_input_scale;
    Eigen::Vector3d full_gravity = R_map_fast_world * fast.gravity_fast_world;
    if (full_gravity.norm() < 1e-6) throw std::runtime_error("invalid_transformed_fastlio_gravity");
    full_gravity *= current_parameters.gravity_mps2 / full_gravity.norm();

    paper::RuntimeParameters full_parameters = current_parameters;
    full_parameters.initial_accel_bias = full_accel_bias;
    paper::FastLio2IkfomFrontend full_frontend(full_parameters);
    paper::StaticImuCalibration full_calibration;
    if (!full_frontend.calibrateStaticImu(all_static, &full_calibration, &reason))
      throw std::runtime_error("full_state_gate_recalibration_failed:" + reason);
    full_calibration.gyro_bias = full_gyro_bias;
    full_calibration.accel_bias_prior = full_accel_bias;
    full_calibration.mean_specific_force =
        full_calibration.mean_acceleration - full_accel_bias;
    if (!full_frontend.initializeFromStaticCalibration(full_calibration,
          official_world_T_imu, T_imu_lidar, full_gravity, full_velocity,
          anchor_ns, &reason))
      throw std::runtime_error("full_motion_state_initialize_failed:" + reason);

    const paper::P7ImuVector scan_imu =
        paper::imuWindow(all_imu, scan_it->scan_start_ns, scan_it->scan_end_ns);
    // The committed TX666 runner's causal integration contract accepts only
    // measurements up to scan_end and holds the final observed IMU input to
    // the exact scan-end timestamp. Do not require a future IMU sample beyond
    // scan_end; that would incorrectly reject the existing causal path.
    if (scan_imu.front().stamp_ns > scan_it->scan_start_ns ||
        scan_imu.back().stamp_ns < anchor_ns || scan_imu.back().stamp_ns > scan_it->scan_end_ns)
      throw std::runtime_error("TX666_IMU_bracketing_incomplete");
    const CaseResult current = runCase(&current_frontend, T_imu_lidar, scan_imu,
        raw_cloud, *scan_it, argv[5], "CURRENT");
    const CaseResult full_result = runCase(&full_frontend, T_imu_lidar, scan_imu,
        raw_cloud, *scan_it, argv[5], "FULL_STATE");

    const double gravity_dot = full_gravity.normalized().dot(current_gravity.normalized());
    const double gravity_angle = std::acos(std::max(-1.0, std::min(1.0, gravity_dot)));
    std::cout << std::setprecision(12)
        << "FASTLIO_STATE anchor_ns=" << fast.stamp_ns
        << " interpolation_bracket_ns=" << fast_states[std::distance(fast_states.begin(),
             std::lower_bound(fast_states.begin(), fast_states.end(), anchor_ns,
               [](const FastState& state, uint64_t stamp) { return state.stamp_ns < stamp; })) - 1].stamp_ns
        << '/' << fast_states[std::distance(fast_states.begin(),
             std::lower_bound(fast_states.begin(), fast_states.end(), anchor_ns,
               [](const FastState& state, uint64_t stamp) { return state.stamp_ns < stamp; }))].stamp_ns
        << " R_fast_world_imu=\n" << fast.R_fast_world_imu
        << "\np_fast_world_imu_not_used=" << fast.p_fast_world_imu.transpose()
        << "\nv_fast_world=" << fast.v_fast_world.transpose()
        << "\nbg_imu=" << fast.bg_imu.transpose()
        << "\nba_scaled_imu=" << fast.ba_scaled_imu.transpose()
        << "\ng_fast_world=" << fast.gravity_fast_world.transpose()
        << "\nmean_acc_norm_raw=" << fast.mean_acc_norm_raw
        << " accel_input_scale=" << fast.accel_input_scale << '\n'
        << "FRAME_MAP_FAST_WORLD=\n" << R_map_fast_world << '\n'
        << "CONVERTED_full_velocity_map=" << full_velocity.transpose()
        << "\nCONVERTED_full_gyro_bias_imu=" << full_gyro_bias.transpose()
        << "\nCONVERTED_full_accel_bias_raw=" << full_accel_bias.transpose()
        << "\nCONVERTED_full_gravity_map=" << full_gravity.transpose()
        << "\nCURRENT_static_gravity_map=" << current_gravity.transpose()
        << "\nGRAVITY_angular_difference_deg=" << gravity_angle * 180.0 / M_PI
        << "\nCOVARIANCE=both conditions use the P7 fresh initialization covariance; FAST-LIO covariance not transferred\n"
        << "TX666 points=" << raw_cloud.size() << " scan_start_ns=" << scan_it->scan_start_ns
        << " scan_end_ns=" << scan_it->scan_end_ns << " anchor_ns=" << anchor_ns
        << " imu_window_first_ns=" << scan_imu.front().stamp_ns
        << " imu_window_last_ns=" << scan_imu.back().stamp_ns
        << " scan_end_policy=causal_held_input_to_exact_scan_end\n";

    printCase("CURRENT", current);
    printCase("FULL_STATE", full_result);
    cloudDifference(current.scan_end.cloud_end_frame, full_result.scan_end.cloud_end_frame,
                    std::string(argv[8]) + "/deskew_point_differences.csv");
    const double seed_translation = (current.scan_end.predicted_map_T_lidar.position -
                                     full_result.scan_end.predicted_map_T_lidar.position).norm();
    const double seed_rotation = angleBetween(current.scan_end.predicted_map_T_lidar,
                                              full_result.scan_end.predicted_map_T_lidar);
    const double current_corr = (current.ndt.raw_map_T_lidar.position -
                                 current.ndt.initial_map_T_lidar.position).norm();
    const double full_corr = (full_result.ndt.raw_map_T_lidar.position -
                              full_result.ndt.initial_map_T_lidar.position).norm();
    const double current_angle = angleBetween(current.ndt.initial_map_T_lidar,
                                              current.ndt.raw_map_T_lidar);
    const double full_angle = angleBetween(full_result.ndt.initial_map_T_lidar,
                                           full_result.ndt.raw_map_T_lidar);
    const double terminal_translation_difference =
        (current.ndt.raw_map_T_lidar.position - full_result.ndt.raw_map_T_lidar.position).norm();
    const double terminal_rotation_difference =
        angleBetween(current.ndt.raw_map_T_lidar, full_result.ndt.raw_map_T_lidar);
    std::ofstream summary(std::string(argv[8]) + "/ab_summary.csv");
    if (!summary) throw std::runtime_error("cannot_create_ab_summary_csv");
    summary << std::setprecision(17)
      << "case,seed_translation_m,seed_rotation_rad,correction_translation_m,correction_rotation_rad,"
         "iterations,converged,status,fitness,initial_overlap_020,initial_overlap_030,initial_overlap_050,"
         "initial_overlap_100,initial_nn_mean_m,initial_nn_median_m,initial_nn_p90_m,initial_nn_p95_m,"
         "final_overlap_020,final_overlap_030,final_overlap_050,final_overlap_100,"
         "final_nn_mean_m,final_nn_median_m,final_nn_p90_m,final_nn_p95_m,"
         "source_points,source_hash,alignment_ms\n";
    auto write = [&summary](const char* name, const CaseResult& r, double dt, double dr) {
      summary << name << ',' << dt << ',' << dr << ','
          << (r.ndt.raw_map_T_lidar.position - r.ndt.initial_map_T_lidar.position).norm() << ','
          << angleBetween(r.ndt.initial_map_T_lidar, r.ndt.raw_map_T_lidar) << ','
          << r.ndt.iterations << ',' << r.ndt.converged << ','
          << paper::currentFrameNdtStatusName(r.ndt.status) << ',' << r.ndt.fitness;
      for (double value : r.initial_overlap.fraction_within) summary << ',' << value;
      summary << ',' << r.initial_overlap.mean_nearest_distance_m << ','
          << r.initial_overlap.median_nearest_distance_m << ','
          << r.initial_overlap.p90_nearest_distance_m << ','
          << r.initial_overlap.p95_nearest_distance_m;
      for (double value : r.final_overlap.fraction_within) summary << ',' << value;
      summary << ',' << r.final_overlap.mean_nearest_distance_m << ','
          << r.final_overlap.median_nearest_distance_m << ','
          << r.final_overlap.p90_nearest_distance_m << ','
          << r.final_overlap.p95_nearest_distance_m;
      summary << ',' << r.ndt.source_point_count << ',' << r.ndt.source_cloud_hash
              << ',' << r.ndt.alignment_ms << '\n';
    };
    write("CURRENT", current, 0.0, 0.0);
    write("FULL_STATE", full_result, seed_translation, seed_rotation);
    std::cout << "SCANEND_SEED_DIFFERENCE translation_m=" << seed_translation
              << " rotation_rad=" << seed_rotation << " rotation_deg="
              << seed_rotation * 180.0 / M_PI << '\n'
              << "TERMINAL_DIFFERENCE translation_m=" << terminal_translation_difference
              << " rotation_rad=" << terminal_rotation_difference << " rotation_deg="
              << terminal_rotation_difference * 180.0 / M_PI << '\n'
              << "NDT_CORRECTION_IMPROVEMENT translation_m=" << current_corr - full_corr
              << " rotation_rad=" << current_angle - full_angle
              << " rotation_deg=" << (current_angle - full_angle) * 180.0 / M_PI << '\n'
              << "ARTIFACT_DIR=" << argv[8] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P8_FULL_MOTION_STATE_DIAGNOSTIC_FAIL=" << error.what() << '\n';
    return 1;
  }
}
