#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Clock = std::chrono::steady_clock;

struct CohortFrame {
  std::string frame_id;
  std::string raw_cloud_file;
  std::string input_map_sha256;
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  uint64_t raw_point_count = 0;
  uint64_t prepared_source_hash = 0;
  uint64_t prepared_source_point_count = 0;
  uint64_t target_point_count = 0;
  paper::Pose3d prediction;
};

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  if (quoted) throw std::runtime_error("unterminated_csv_quote");
  fields.push_back(field);
  return fields;
}

std::map<std::string, std::size_t> headerMap(const std::vector<std::string>& header) {
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i)
    if (!columns.emplace(header[i], i).second)
      throw std::runtime_error("duplicate_cohort_header:" + header[i]);
  return columns;
}

const std::string& value(const std::vector<std::string>& row,
    const std::map<std::string, std::size_t>& columns, const std::string& name) {
  const auto found = columns.find(name);
  if (found == columns.end() || found->second >= row.size())
    throw std::runtime_error("missing_cohort_column:" + name);
  return row[found->second];
}

paper::Pose3d parsePose(const std::string& text) {
  std::stringstream stream(text);
  std::array<double, 7> fields{};
  std::string field;
  std::size_t index = 0;
  while (std::getline(stream, field, ';')) {
    if (index >= fields.size()) throw std::runtime_error("invalid_pose_width");
    fields[index++] = std::stod(field);
  }
  if (index != fields.size()) throw std::runtime_error("invalid_pose_width");
  paper::Pose3d pose;
  pose.position = Eigen::Vector3d(fields[0], fields[1], fields[2]);
  pose.orientation = Eigen::Quaterniond(fields[6], fields[3], fields[4], fields[5]);
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12)
    throw std::runtime_error("invalid_pose_value");
  pose.orientation.normalize();
  return pose;
}

std::vector<CohortFrame> readCohort(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_frozen_cohort");
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty_frozen_cohort");
  const auto columns = headerMap(splitCsv(line));
  std::vector<CohortFrame> frames;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto row = splitCsv(line);
    CohortFrame frame;
    frame.frame_id = value(row, columns, "frame_id");
    frame.raw_cloud_file = value(row, columns, "raw_cloud_file");
    frame.input_map_sha256 = value(row, columns, "input_map_sha256");
    frame.transaction_id = std::stoull(value(row, columns, "transaction_id"));
    frame.stamp_ns = std::stoull(value(row, columns, "stamp_ns"));
    frame.raw_point_count = std::stoull(value(row, columns, "raw_point_count"));
    frame.prepared_source_hash = std::stoull(value(row, columns, "prepared_source_hash"));
    frame.prepared_source_point_count =
        std::stoull(value(row, columns, "prepared_source_point_count"));
    frame.target_point_count = std::stoull(value(row, columns, "target_point_count"));
    frame.prediction = parsePose(value(row, columns, "initial_pose_xyz_q_xyzw"));
    frames.push_back(std::move(frame));
  }
  if (frames.size() != 32) throw std::runtime_error("frozen_cohort_frame_count_not_32");
  return frames;
}

paper::RegistrationCloud readRawCloud(const CohortFrame& frame) {
  std::ifstream input(frame.raw_cloud_file, std::ios::binary);
  if (!input) throw std::runtime_error("cannot_open_raw_source:" + frame.frame_id);
  paper::RegistrationCloud cloud(static_cast<std::size_t>(frame.raw_point_count));
  for (paper::RegistrationPoint& point : cloud) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!input) throw std::runtime_error("truncated_raw_source:" + frame.frame_id);
    point = {xyz[0], xyz[1], xyz[2]};
  }
  char trailing;
  if (input.read(&trailing, 1)) throw std::runtime_error("raw_source_has_trailing_bytes");
  return cloud;
}

void poseColumns(std::ostream& output, const paper::Pose3d& pose) {
  output << ',' << pose.position.x() << ',' << pose.position.y() << ',' << pose.position.z()
      << ',' << pose.orientation.x() << ',' << pose.orientation.y() << ','
      << pose.orientation.z() << ',' << pose.orientation.w();
}

double translationDistance(const paper::Pose3d& lhs, const paper::Pose3d& rhs) {
  return (lhs.position - rhs.position).norm();
}

double rotationDistanceDegrees(const paper::Pose3d& lhs, const paper::Pose3d& rhs) {
  Eigen::Quaterniond relative =
      (lhs.orientation.normalized().conjugate() * rhs.orientation.normalized()).normalized();
  return 2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w())) * 180.0 /
      3.14159265358979323846;
}

double normalizedObjective(paper::CurrentFrameNdtRegistration* registration,
    const paper::Pose3d& pose, std::string* reason) {
  paper::NdtObjectiveSample sample;
  if (!registration->evaluateLocalObjectiveAtPose(pose, &sample, reason) || !sample.valid ||
      sample.source_point_count == 0)
    throw std::runtime_error("terminal_objective_failed:" + *reason);
  return -sample.score_sum / static_cast<double>(sample.source_point_count);
}

void run(const std::string& cohort_path, const std::string& map_path,
    const std::string& output_path) {
  const std::vector<CohortFrame> frames = readCohort(cohort_path);
  paper::CurrentFrameNdtParameters fine_parameters;
  paper::CurrentFrameNdtParameters coarse_parameters = fine_parameters;
  coarse_parameters.resolution_m = 1.6;
  paper::CurrentFrameNdtRegistration fine(fine_parameters);
  paper::CurrentFrameNdtRegistration coarse(coarse_parameters);
  std::string reason;
  if (!fine.loadMap(map_path, &reason)) throw std::runtime_error("fine_map_load_failed:" + reason);
  if (!coarse.loadMap(map_path, &reason)) throw std::runtime_error("coarse_map_load_failed:" + reason);
  const auto fine_leaf = fine.targetGridLeafSizeMeters();
  const auto coarse_leaf = coarse.targetGridLeafSizeMeters();
  if (std::abs(fine_leaf[0] - 0.8f) > 1e-6f || std::abs(fine_leaf[1] - 0.8f) > 1e-6f ||
      std::abs(fine_leaf[2] - 0.8f) > 1e-6f || std::abs(coarse_leaf[0] - 1.6f) > 1e-6f ||
      std::abs(coarse_leaf[1] - 1.6f) > 1e-6f || std::abs(coarse_leaf[2] - 1.6f) > 1e-6f)
    throw std::runtime_error("actual_target_grid_resolution_mismatch");
  for (const CohortFrame& frame : frames)
    if (frame.target_point_count != fine.targetPointCount() ||
        frame.input_map_sha256.empty() || frame.input_map_sha256 != frames.front().input_map_sha256)
      throw std::runtime_error("cohort_map_or_target_count_mismatch:" + frame.frame_id);

  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot_create_cross_resolution_output");
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output << std::setprecision(17)
      << "frame_id,transaction_id,stamp_ns,prepared_source_hash,source_points,target_points,"
         "fine_grid_leaf_x_m,coarse_grid_leaf_x_m,fine_status,fine_effective,fine_iterations,"
         "fine_fitness,fine_probability,fine_objective_per_source,fine_align_ms,"
         "coarse_status,coarse_effective,coarse_iterations,coarse_fitness,coarse_probability,"
         "coarse_objective_per_source,coarse_align_ms,coarse_to_fine_status,"
         "coarse_to_fine_effective,coarse_to_fine_iterations,coarse_to_fine_fitness,"
         "coarse_to_fine_probability,coarse_to_fine_objective_per_source,"
         "coarse_to_fine_align_ms,fine_to_coarse_translation_m,fine_to_coarse_rotation_deg,"
         "fine_to_refined_translation_m,fine_to_refined_rotation_deg,"
         "coarse_to_refined_translation_m,coarse_to_refined_rotation_deg,"
         "T_fine_x,T_fine_y,T_fine_z,T_fine_qx,T_fine_qy,T_fine_qz,T_fine_qw,"
         "T_coarse_x,T_coarse_y,T_coarse_z,T_coarse_qx,T_coarse_qy,T_coarse_qz,"
         "T_coarse_qw,T_cf_x,T_cf_y,T_cf_z,T_cf_qx,T_cf_qy,T_cf_qz,T_cf_qw\n";
  std::size_t effective_fine = 0, effective_coarse = 0, effective_refined = 0;
  double total_alignment_ms = 0.0;
  for (const CohortFrame& frame : frames) {
    const paper::RegistrationCloud raw = readRawCloud(frame);
    paper::CurrentFrameNdtResult fine_result, coarse_result, refined_result;
    const auto fine_start = Clock::now();
    if (!fine.align(frame.stamp_ns, raw, frame.prediction, &fine_result, &reason))
      throw std::runtime_error("fine_alignment_failed:" + frame.frame_id + ":" + reason);
    const double fine_wall_ms = elapsedMs(fine_start);
    const auto coarse_start = Clock::now();
    if (!coarse.align(frame.stamp_ns, raw, frame.prediction, &coarse_result, &reason))
      throw std::runtime_error("coarse_alignment_failed:" + frame.frame_id + ":" + reason);
    const double coarse_wall_ms = elapsedMs(coarse_start);
    if (fine_result.source_cloud_hash != frame.prepared_source_hash ||
        coarse_result.source_cloud_hash != frame.prepared_source_hash ||
        fine_result.source_point_count != frame.prepared_source_point_count ||
        coarse_result.source_point_count != frame.prepared_source_point_count ||
        fine_result.target_point_count != frame.target_point_count ||
        coarse_result.target_point_count != frame.target_point_count)
      throw std::runtime_error("cross_resolution_source_provenance_mismatch:" + frame.frame_id);
    if (!coarse_result.raw_map_T_lidar.position.allFinite() ||
        !coarse_result.raw_map_T_lidar.orientation.coeffs().allFinite())
      throw std::runtime_error("coarse_terminal_nonfinite:" + frame.frame_id);
    const auto refined_start = Clock::now();
    if (!fine.align(frame.stamp_ns, raw, coarse_result.raw_map_T_lidar,
            &refined_result, &reason))
      throw std::runtime_error("coarse_to_fine_alignment_failed:" + frame.frame_id + ":" + reason);
    const double refined_wall_ms = elapsedMs(refined_start);
    if (refined_result.source_cloud_hash != frame.prepared_source_hash ||
        refined_result.source_point_count != frame.prepared_source_point_count ||
        refined_result.target_point_count != frame.target_point_count)
      throw std::runtime_error("refined_source_provenance_mismatch:" + frame.frame_id);
    const double fine_objective = normalizedObjective(&fine, fine_result.raw_map_T_lidar, &reason);
    const double coarse_objective = normalizedObjective(&coarse, coarse_result.raw_map_T_lidar, &reason);
    const double refined_objective = normalizedObjective(&fine, refined_result.raw_map_T_lidar, &reason);
    effective_fine += fine_result.effective;
    effective_coarse += coarse_result.effective;
    effective_refined += refined_result.effective;
    total_alignment_ms += fine_wall_ms + coarse_wall_ms + refined_wall_ms;
    output << frame.frame_id << ',' << frame.transaction_id << ',' << frame.stamp_ns << ','
        << fine_result.source_cloud_hash << ',' << fine_result.source_point_count << ','
        << fine_result.target_point_count << ',' << fine_leaf[0] << ',' << coarse_leaf[0] << ','
        << paper::currentFrameNdtStatusName(fine_result.status) << ',' << fine_result.effective << ','
        << fine_result.iterations << ',' << fine_result.fitness << ','
        << fine_result.transformation_probability << ',' << fine_objective << ',' << fine_wall_ms << ','
        << paper::currentFrameNdtStatusName(coarse_result.status) << ',' << coarse_result.effective << ','
        << coarse_result.iterations << ',' << coarse_result.fitness << ','
        << coarse_result.transformation_probability << ',' << coarse_objective << ',' << coarse_wall_ms << ','
        << paper::currentFrameNdtStatusName(refined_result.status) << ',' << refined_result.effective << ','
        << refined_result.iterations << ',' << refined_result.fitness << ','
        << refined_result.transformation_probability << ',' << refined_objective << ',' << refined_wall_ms << ','
        << translationDistance(fine_result.raw_map_T_lidar, coarse_result.raw_map_T_lidar) << ','
        << rotationDistanceDegrees(fine_result.raw_map_T_lidar, coarse_result.raw_map_T_lidar) << ','
        << translationDistance(fine_result.raw_map_T_lidar, refined_result.raw_map_T_lidar) << ','
        << rotationDistanceDegrees(fine_result.raw_map_T_lidar, refined_result.raw_map_T_lidar) << ','
        << translationDistance(coarse_result.raw_map_T_lidar, refined_result.raw_map_T_lidar) << ','
        << rotationDistanceDegrees(coarse_result.raw_map_T_lidar, refined_result.raw_map_T_lidar);
    poseColumns(output, fine_result.raw_map_T_lidar);
    poseColumns(output, coarse_result.raw_map_T_lidar);
    poseColumns(output, refined_result.raw_map_T_lidar);
    output << '\n';
  }
  output.close();
  std::cout << "frames=" << frames.size() << " alignments=" << 3 * frames.size()
      << " extra_alignments=" << 2 * frames.size() << " fine_effective=" << effective_fine
      << " coarse_effective=" << effective_coarse << " coarse_to_fine_effective="
      << effective_refined << " total_alignment_wall_ms=" << total_alignment_ms
      << " fine_grid=" << fine_leaf[0] << ',' << fine_leaf[1] << ',' << fine_leaf[2]
      << " coarse_grid=" << coarse_leaf[0] << ',' << coarse_leaf[1] << ',' << coarse_leaf[2]
      << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("usage: dual_u_r2b_cross_resolution_runner COHORT_CSV MAP_PCD OUTPUT_CSV");
    run(argv[1], argv[2], argv[3]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "dual_u_r2b_cross_resolution_runner FAIL: " << error.what() << '\n';
    return 1;
  }
}
