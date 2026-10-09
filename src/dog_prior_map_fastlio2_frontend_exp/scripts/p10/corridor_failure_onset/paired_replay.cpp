#include "../corridor_benchmark/causal_rotation.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <sys/resource.h>

namespace b = corridor_benchmark;
namespace p = dog_prior_map_fastlio2_frontend_exp;
using Matrix = Eigen::Matrix4d;
using Clock = std::chrono::steady_clock;

namespace {
double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream in(line);
  std::string field;
  while (std::getline(in, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}
Matrix readMatrix(const std::vector<std::string>& fields,
    const std::map<std::string, std::size_t>& columns, const std::string& prefix) {
  Matrix out = Matrix::Identity();
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) {
    const std::string name = prefix + "_r" + std::to_string(r) + "c" + std::to_string(c);
    const auto it = columns.find(name);
    if (it == columns.end() || it->second >= fields.size())
      throw std::runtime_error("missing_predictor_matrix_field:" + name);
    out(r, c) = std::stod(fields[it->second]);
  }
  return out;
}
const std::string& field(const std::vector<std::string>& fields,
    const std::map<std::string, std::size_t>& columns, const std::string& name) {
  const auto it = columns.find(name);
  if (it == columns.end() || it->second >= fields.size())
    throw std::runtime_error("missing_predictor_field:" + name);
  return fields[it->second];
}
Matrix poseMatrix(const p::Pose3d& value) { return p::asIsometry(value).matrix(); }
p::Pose3d pose(const Matrix& value) {
  p::Pose3d out;
  out.position = value.block<3, 1>(0, 3);
  out.orientation = Eigen::Quaterniond(value.block<3, 3>(0, 0)).normalized();
  return out;
}
bool rigid(const Matrix& value) {
  const Eigen::Matrix3d rotation = value.block<3, 3>(0, 0);
  return value.allFinite() &&
      (value.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-6 &&
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-4 &&
      std::abs(rotation.determinant() - 1.0) < 1e-4;
}
double rotationDeg(const Matrix& lhs, const Matrix& rhs) {
  return Eigen::Quaterniond(lhs.block<3, 3>(0, 0)).normalized().angularDistance(
      Eigen::Quaterniond(rhs.block<3, 3>(0, 0)).normalized()) * 180.0 / std::acos(-1.0);
}
void matrixColumns(std::ostream& out, const Matrix& value) {
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) out << ',' << value(r, c);
}
uint64_t u64(const std::string& text) {
  std::size_t used = 0;
  const auto value = std::stoull(text, &used);
  if (used != text.size()) throw std::runtime_error("invalid_integer_field");
  return value;
}
struct Predictor {
  uint64_t transaction = 0, start_ns = 0, end_ns = 0, source_count = 0, source_hash = 0;
  double elapsed_s = 0, expected_raw_score = 0;
  bool expected_effective = false;
  std::string expected_status;
  Matrix prediction = Matrix::Identity(), expected_nominal = Matrix::Identity();
  Matrix expected_executed = Matrix::Identity();
};
std::vector<Predictor> readPredictor(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  if (!in || !std::getline(in, line)) throw std::runtime_error("cannot_read_common_predictor");
  const auto header = splitCsv(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i)
    if (!columns.emplace(header[i], i).second) throw std::runtime_error("duplicate_predictor_column");
  std::vector<Predictor> result;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    Predictor row;
    row.transaction = u64(field(fields, columns, "transaction_id"));
    row.start_ns = u64(field(fields, columns, "scan_start_ns"));
    row.end_ns = u64(field(fields, columns, "scan_end_ns"));
    row.elapsed_s = std::stod(field(fields, columns, "elapsed_s"));
    row.source_count = u64(field(fields, columns, "source_count"));
    row.source_hash = u64(field(fields, columns, "source_hash"));
    row.expected_effective = field(fields, columns, "nominal_effective") == "1";
    row.expected_status = field(fields, columns, "nominal_status");
    row.expected_raw_score = std::stod(field(fields, columns, "nominal_raw_score"));
    row.prediction = readMatrix(fields, columns, "prediction");
    row.expected_nominal = readMatrix(fields, columns, "nominal");
    row.expected_executed = readMatrix(fields, columns, "executed");
    if (!rigid(row.prediction) || !rigid(row.expected_nominal) || !rigid(row.expected_executed) ||
        !std::isfinite(row.elapsed_s) || !std::isfinite(row.expected_raw_score))
      throw std::runtime_error("invalid_common_predictor_row");
    if (row.transaction != 52 + result.size()) throw std::runtime_error("predictor_rows_not_contiguous");
    result.push_back(row);
  }
  if (result.size() != 346 || result.front().transaction != 52 || result.back().transaction != 397)
    throw std::runtime_error("fixed_35s_predictor_window_mismatch");
  return result;
}
void selfTest() {
  Matrix a = Matrix::Identity(), bpose = Matrix::Identity();
  a.block<3, 1>(0, 3) << .13, 0, 0;
  const double translation_innovation =
      (a.block<3, 1>(0, 3) - bpose.block<3, 1>(0, 3)).norm();
  b::require(translation_innovation > .12, "trigger_translation_boundary");
  b::require(rigid(a) && rigid(bpose), "rigid_pose_validation");
  p::CoupledAnchorState anchor;
  p::settleWeakRefinementAnchor(&anchor, 1000000000, a, true);
  b::require(anchor.valid && anchor.origin_stamp_ns == 1000000000, "anchor_seed");
  Matrix interval = Matrix::Identity();
  interval(0, 3) = .01;
  p::advanceCoupledAnchor(&anchor, 1100000000, interval);
  b::require(anchor.valid && std::abs(anchor.prediction(0, 3) - .14) < 1e-12,
      "anchor_causal_propagation");
  std::cout << "PAIRED_FAILURE_ONSET_SELF_TEST_PASS\n";
}
}  // namespace

int main(int argc, char** argv) {
  uint64_t active_tx = 0;
  unsigned completed = 0;
  const auto run_start = Clock::now();
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") { selfTest(); return 0; }
    b::require(argc == 6, "usage: paired_replay INPUT MAP PREDICTOR OUT EXTRINSIC_TXT");
    const std::string input = argv[1], output = argv[4];
    Eigen::Matrix4d extrinsic;
    std::ifstream ext(argv[5]);
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
      b::require(static_cast<bool>(ext >> extrinsic(r, c)), "extrinsic_read_failed");
    b::require(rigid(extrinsic), "extrinsic_not_rigid");

    const auto predictors = readPredictor(argv[3]);
    const auto scans = p::readP7TimedScans(input + "/filter_scans.csv", input + "/raw_timed_scan_index.csv");
    const auto imu = p::readP7Imu(input + "/imu.csv");
    b::require(scans.size() == 2777 && imu.size() == 55957, "frozen_input_counts_mismatch");
    p::CurrentFrameNdtRegistration registration{p::CurrentFrameNdtParameters{}};
    std::string reason;
    if (!registration.loadMap(argv[2], &reason)) throw std::runtime_error(reason);
    b::require(registration.targetPointCount() == 226164, "map_target_count_mismatch");
    std::ofstream rows(output + "/paired_candidates.csv", std::ios::out | std::ios::trunc);
    std::ofstream parity(output + "/source_parity.csv", std::ios::out | std::ios::trunc);
    b::require(static_cast<bool>(rows) && static_cast<bool>(parity), "cannot_open_pair_outputs");
    rows << std::setprecision(17);
    rows << "transaction_id,elapsed_s,scan_start_ns,scan_end_ns,raw_points,source_count,source_hash,target_count,source_parity,expected_nominal_status,actual_nominal_status,nominal_effective,iterations,pcl_converged,raw_score,fitness,triggered,anchor_valid,anchor_status,anchor_age_s,weak_status,coupled_status,weak_recommended,coupled_recommended,weak_strong_selected,coupled_strong_selected,weak_dimension,weak_correction_m,weak_correction_deg,coupled_correction_m,coupled_correction_deg,weak_score,coupled_score,nominal_score,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,weak_jets,weak_values,coupled_jets,coupled_values,deskew_ms,align_complete_ms,weak_refinement_ms,coupled_refinement_ms,paired_total_ms,nominal_complete_ms,weak_complete_ms,coupled_complete_ms,peak_RSS_KiB";
    for (const std::string& prefix : {"prediction", "nominal", "weak_candidate", "coupled_candidate"})
      for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c)
        rows << ',' << prefix << "_r" << r << 'c' << c;
    rows << '\n';
    parity << "transaction_id,scan_start_ns,scan_end_ns,expected_source_count,actual_source_count,expected_prepared_hash,actual_prepared_hash,expected_status,actual_status,pose_translation_delta_m,pose_rotation_delta_deg,raw_score_delta,pass\n";
    p::CoupledAnchorState common_anchor;
    Matrix previous_executed = Matrix::Identity();
    bool has_previous = false;
    double total_wall_ms = 0;
    for (std::size_t index = 0; index < predictors.size(); ++index) {
      const Predictor& item = predictors[index];
      active_tx = item.transaction;
      b::require(item.transaction <= scans.size(), "predictor_transaction_out_of_range");
      const auto& scan = scans[item.transaction - 1];
      b::require(scan.transaction_id == item.transaction && scan.scan_start_ns == item.start_ns &&
          scan.scan_end_ns == item.end_ns, "predictor_timing_not_same_raw_scan");
      const auto frame_start = Clock::now();
      const auto deskew_start = Clock::now();
      const auto raw = p::readP7PackedTimedCloud(input + "/raw_timed_points.bin", scan);
      const b::RotationTimeline within(imu, scan.scan_start_ns, scan.scan_end_ns);
      p::RegistrationCloud cloud;
      cloud.reserve(raw.size());
      for (const auto& point : raw) {
        const Eigen::Matrix3d rotation = within.rotations.back().transpose() * within.at(point.stamp_ns);
        const Eigen::Vector3d q = b::rotationDeskew(point.position, rotation, extrinsic);
        b::require(q.allFinite(), "nonfinite_scanend_point");
        cloud.push_back({static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())});
      }
      const double deskew_ms = elapsedMs(deskew_start);
      if (common_anchor.valid && has_previous) {
        const Matrix causal_interval = previous_executed.inverse() * item.prediction;
        p::advanceCoupledAnchor(&common_anchor, item.end_ns, causal_interval);
      }
      const p::CoupledAnchorState anchor_for_pair = common_anchor;

      p::CurrentFrameNdtResult nominal;
      const auto align_start = Clock::now();
      if (!registration.align(item.end_ns, cloud, pose(item.prediction), &nominal, &reason))
        throw std::runtime_error("common_nominal_align_failed:" + reason);
      const double align_complete_ms = elapsedMs(align_start);
      const Matrix nominal_pose = poseMatrix(nominal.raw_map_T_lidar);
      const bool source_match = nominal.source_point_count == item.source_count &&
          nominal.source_cloud_hash == item.source_hash;
      const double pose_delta = (nominal_pose.block<3, 1>(0, 3) -
          item.expected_nominal.block<3, 1>(0, 3)).norm();
      const double rotation_delta = rotationDeg(nominal_pose, item.expected_nominal);
      const double current_score = nominal.transformation_probability * nominal.source_point_count;
      const double score_delta = std::abs(current_score - item.expected_raw_score);
      const bool nominal_match = p::currentFrameNdtStatusName(nominal.status) == item.expected_status &&
          nominal.effective == item.expected_effective && pose_delta <= 1e-5 && rotation_delta <= 1e-4 &&
          score_delta <= 1e-6 * std::max(1.0, std::abs(item.expected_raw_score));
      parity << item.transaction << ',' << item.start_ns << ',' << item.end_ns << ',' << item.source_count << ','
             << nominal.source_point_count << ',' << item.source_hash << ',' << nominal.source_cloud_hash << ','
             << item.expected_status << ',' << p::currentFrameNdtStatusName(nominal.status) << ','
             << pose_delta << ',' << rotation_delta << ',' << score_delta << ','
             << (source_match && nominal_match ? "PASS" : "FAIL") << '\n';
      parity.flush();
      b::require(source_match, "source_hash_or_count_parity_failed");
      b::require(nominal_match, "historical_nominal_replay_parity_failed");

      p::WeakCoupledResult weak, coupled;
      p::WeakCoupledConfig weak_config, coupled_config;
      weak_config.coupled = false;
      coupled_config.coupled = true;
      const auto weak_start = Clock::now();
      if (!registration.weakRefinement(nominal, anchor_for_pair, weak_config, &weak, &reason))
        throw std::runtime_error("weak_refinement_failed:" + reason);
      const double weak_ms = elapsedMs(weak_start);
      const auto coupled_start = Clock::now();
      if (!registration.weakRefinement(nominal, anchor_for_pair, coupled_config, &coupled, &reason))
        throw std::runtime_error("coupled_refinement_failed:" + reason);
      const double coupled_ms = elapsedMs(coupled_start);
      b::require(weak.triggered == coupled.triggered && weak.anchor_valid == coupled.anchor_valid,
          "paired_refinement_context_mismatch");
      b::require((weak.weak_eta.cast<double>() - coupled.weak_eta.cast<double>()).norm() < 1e-7 ||
          (!weak.attempted && !coupled.attempted), "weak_step_not_paired");
      const Matrix weak_pose = weak.candidate.cast<double>();
      const Matrix coupled_pose = coupled.candidate.cast<double>();
      b::require(rigid(nominal_pose) && rigid(weak_pose) && rigid(coupled_pose), "nonrigid_candidate_pose");
      const double weak_translation = (weak_pose.block<3, 1>(0, 3) - nominal_pose.block<3, 1>(0, 3)).norm();
      const double coupled_translation = (coupled_pose.block<3, 1>(0, 3) - nominal_pose.block<3, 1>(0, 3)).norm();
      const double weak_rotation = rotationDeg(weak_pose, nominal_pose);
      const double coupled_rotation = rotationDeg(coupled_pose, nominal_pose);
      struct rusage usage;
      getrusage(RUSAGE_SELF, &usage);
      const double paired_total = elapsedMs(frame_start);
      total_wall_ms += paired_total;
      rows << item.transaction << ',' << item.elapsed_s << ',' << item.start_ns << ',' << item.end_ns << ','
           << raw.size() << ',' << nominal.source_point_count << ',' << nominal.source_cloud_hash << ','
           << nominal.target_point_count << ",1," << item.expected_status << ','
           << p::currentFrameNdtStatusName(nominal.status) << ',' << nominal.effective << ',' << nominal.iterations << ','
           << nominal.converged << ',' << current_score << ',' << nominal.fitness << ',' << weak.triggered << ','
           << weak.anchor_valid << ',' << (anchor_for_pair.valid ? anchor_for_pair.status : "MISSING") << ','
           << weak.anchor_age_s << ',' << weak.status << ',' << coupled.status << ',' << weak.recommended << ','
           << coupled.recommended << ',' << weak.strong_selected << ',' << coupled.strong_selected << ','
           << weak.weak_dimension << ',' << weak_translation << ',' << weak_rotation << ',' << coupled_translation << ','
           << coupled_rotation << ',' << weak.candidate_score << ',' << coupled.candidate_score << ','
           << weak.nominal_score << ',';
      for (int i = 0; i < 6; ++i) rows << (i ? "," : "") << weak.eigenvalues(i);
      rows << ',' << weak.jet_calls << ',' << weak.value_calls << ',' << coupled.jet_calls << ',' << coupled.value_calls
           << ',' << deskew_ms << ',' << align_complete_ms << ',' << weak_ms << ',' << coupled_ms << ',' << paired_total
           << ',' << deskew_ms + align_complete_ms << ',' << deskew_ms + align_complete_ms + weak_ms << ','
           << deskew_ms + align_complete_ms + coupled_ms << ',' << usage.ru_maxrss;
      matrixColumns(rows, item.prediction);
      matrixColumns(rows, nominal_pose);
      matrixColumns(rows, weak_pose);
      matrixColumns(rows, coupled_pose);
      rows << '\n';
      rows.flush();
      b::require(static_cast<bool>(rows), "pair_output_flush_failed");

      Matrix nominal_executed = nominal.effective ? nominal_pose : item.prediction;
      if (!nominal.effective) {
        common_anchor.valid = false;
        common_anchor.frozen = false;
        common_anchor.invalidated_stamp_ns = item.end_ns;
        common_anchor.status = "NOMINAL_INEFFECTIVE";
      }
      p::settleWeakRefinementAnchor(&common_anchor, item.end_ns, nominal_executed,
          nominal.effective && !weak.triggered);
      previous_executed = item.expected_executed;
      has_previous = true;
      ++completed;
      if (completed % 50 == 0) std::cout << "PAIRED_PROGRESS " << completed << '/' << predictors.size() << '\n';
    }
    std::ofstream receipt(output + "/paired_execution.json", std::ios::out | std::ios::trunc);
    receipt << std::setprecision(17) << "{\"frames\":" << completed << ",\"full_align_calls\":" << completed
            << ",\"source_parity\":\"" << (completed == predictors.size() ? "PASS" : "INCOMPLETE")
            << "\",\"nominal_replay_parity\":\"PASS\",\"map_instances\":1,\"causal_feedback\":false"
            << ",\"GT_LOADED\":false,\"total_frame_wall_ms\":" << total_wall_ms
            << ",\"wall_s\":" << elapsedMs(run_start) / 1000.0 << "}\n";
    receipt.flush();
    b::require(static_cast<bool>(receipt), "execution_receipt_write_failed");
    std::cout << "PAIRED_COMPLETE frames=" << completed << " mean_wall_ms="
              << total_wall_ms / std::max(1U, completed) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PAIRED_STOP tx=" << active_tx << " completed=" << completed << ' ' << error.what() << '\n';
    if (argc == 6) {
      std::ofstream failure(std::string(argv[4]) + "/paired_failure.json", std::ios::out | std::ios::trunc);
      failure << "{\"failed_tx\":" << active_tx << ",\"completed_frames\":" << completed
              << ",\"error\":\"" << error.what() << "\",\"GT_LOADED\":false}\n";
    }
    return 2;
  }
}
