#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
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
struct CohortFrame {
  std::string frame_id;
  std::string selection_labels;
  std::string segment;
  std::string raw_cloud_file;
  std::string raw_source_sha256;
  std::string input_bag_sha256;
  std::string input_map_sha256;
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  uint64_t raw_point_count = 0;
  uint64_t prepared_source_hash = 0;
  uint64_t prepared_source_point_count = 0;
  uint64_t target_point_count = 0;
  bool targeted = false;
  double time_s = 0.0;
  paper::Pose3d initial_pose;
  paper::Pose3d raw_terminal_pose;
  double resolution_m = 0.8;
};

struct Seed {
  std::string domain;
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
  double wide_radius_m = 0.0, wide_angle_deg = 0.0;
  int grid_ix = 0, grid_iy = 0, grid_iyaw = 0;
};

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
  std::map<std::string, std::size_t> result;
  for (std::size_t i = 0; i < header.size(); ++i)
    if (!result.emplace(header[i], i).second)
      throw std::runtime_error("duplicate_cohort_header:" + header[i]);
  return result;
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
    CohortFrame f;
    f.frame_id = value(row, columns, "frame_id");
    f.selection_labels = value(row, columns, "selection_labels");
    f.segment = value(row, columns, "segment");
    f.raw_cloud_file = value(row, columns, "raw_cloud_file");
    f.raw_source_sha256 = value(row, columns, "raw_source_sha256");
    f.input_bag_sha256 = value(row, columns, "input_bag_sha256");
    f.input_map_sha256 = value(row, columns, "input_map_sha256");
    f.transaction_id = std::stoull(value(row, columns, "transaction_id"));
    f.stamp_ns = std::stoull(value(row, columns, "stamp_ns"));
    f.raw_point_count = std::stoull(value(row, columns, "raw_point_count"));
    f.prepared_source_hash = std::stoull(value(row, columns, "prepared_source_hash"));
    f.prepared_source_point_count = std::stoull(value(row, columns, "prepared_source_point_count"));
    f.target_point_count = std::stoull(value(row, columns, "target_point_count"));
    f.targeted = std::stoi(value(row, columns, "wide_targeted")) != 0;
    f.time_s = std::stod(value(row, columns, "time_s"));
    f.initial_pose = parsePose(value(row, columns, "initial_pose_xyz_q_xyzw"));
    f.raw_terminal_pose = parsePose(value(row, columns, "raw_terminal_pose_xyz_q_xyzw"));
    f.resolution_m = std::stod(value(row, columns, "configured_resolution_m"));
    frames.push_back(std::move(f));
  }
  if (frames.size() != 32) throw std::runtime_error("frozen_cohort_frame_count_not_32");
  return frames;
}

paper::RegistrationCloud readRawCloud(const CohortFrame& frame) {
  std::ifstream input(frame.raw_cloud_file, std::ios::binary);
  if (!input) throw std::runtime_error("cannot_open_raw_source:" + frame.frame_id);
  paper::RegistrationCloud cloud(static_cast<std::size_t>(frame.raw_point_count));
  for (paper::RegistrationPoint& p : cloud) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!input) throw std::runtime_error("truncated_raw_source:" + frame.frame_id);
    p = {xyz[0], xyz[1], xyz[2]};
  }
  char trailing;
  if (input.read(&trailing, 1)) throw std::runtime_error("raw_source_has_trailing_bytes");
  return cloud;
}

std::vector<Seed> planarSeeds(double resolution) {
  const std::array<double, 7> offsets{{-2.0 * resolution, -resolution,
      -0.5 * resolution, 0.0, 0.5 * resolution, resolution, 2.0 * resolution}};
  const std::array<double, 5> yaws{{-10.0, -5.0, 0.0, 5.0, 10.0}};
  std::vector<Seed> seeds;
  for (std::size_t ix = 0; ix < offsets.size(); ++ix)
    for (std::size_t iy = 0; iy < offsets.size(); ++iy)
      for (std::size_t ia = 0; ia < yaws.size(); ++ia) {
        Seed s;
        s.domain = "PLANAR";
        s.dx = offsets[ix]; s.dy = offsets[iy]; s.yaw_deg = yaws[ia];
        s.grid_ix = static_cast<int>(ix) - 3;
        s.grid_iy = static_cast<int>(iy) - 3;
        s.grid_iyaw = static_cast<int>(ia) - 2;
        seeds.push_back(s);
      }
  return seeds;
}

std::vector<Seed> axialSeeds(double resolution) {
  std::vector<Seed> seeds;
  for (double d : {-resolution, -0.5 * resolution, 0.5 * resolution, resolution}) {
    Seed s; s.domain = "AXIAL_Z"; s.dz = d; seeds.push_back(s);
  }
  for (double d : {-5.0, -2.0, 2.0, 5.0}) {
    Seed s; s.domain = "AXIAL_ROLL"; s.roll_deg = d; seeds.push_back(s);
  }
  for (double d : {-5.0, -2.0, 2.0, 5.0}) {
    Seed s; s.domain = "AXIAL_PITCH"; s.pitch_deg = d; seeds.push_back(s);
  }
  for (double d : {-15.0, 15.0}) {
    Seed s; s.domain = "AXIAL_YAW"; s.yaw_deg = d; seeds.push_back(s);
  }
  for (double d : {-3.0 * resolution, 3.0 * resolution}) {
    Seed sx; sx.domain = "AXIAL_X"; sx.dx = d; seeds.push_back(sx);
    Seed sy; sy.domain = "AXIAL_Y"; sy.dy = d; seeds.push_back(sy);
  }
  return seeds;
}

std::vector<Seed> widePlanarSeeds(double resolution) {
  const std::array<double, 2> radii{{std::max(3.0 * resolution, 1.5),
                                     std::max(5.0 * resolution, 3.0)}};
  const std::array<double, 3> yaws{{-15.0, 0.0, 15.0}};
  std::vector<Seed> seeds;
  for (double radius : radii)
    for (int angle = 0; angle < 8; ++angle)
      for (double yaw : yaws) {
        const double theta = angle * M_PI / 4.0;
        Seed s;
        s.domain = "WIDE_PLANAR";
        s.dx = radius * std::cos(theta); s.dy = radius * std::sin(theta);
        s.yaw_deg = yaw; s.wide_radius_m = radius; s.wide_angle_deg = angle * 45.0;
        seeds.push_back(s);
      }
  return seeds;
}

Eigen::Matrix4f expSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d omega = xi.tail<3>();
  const double theta = omega.norm();
  Eigen::Matrix3d w;
  w << 0.0, -omega.z(), omega.y(), omega.z(), 0.0, -omega.x(),
       -omega.y(), omega.x(), 0.0;
  Eigen::Matrix3d rotation, v;
  if (theta < 1e-10) {
    rotation = Eigen::Matrix3d::Identity() + w + 0.5 * w * w;
    v = Eigen::Matrix3d::Identity() + 0.5 * w + w * w / 6.0;
  } else {
    rotation = Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * w +
        ((1.0 - std::cos(theta)) / (theta * theta)) * w * w;
    v = Eigen::Matrix3d::Identity() + ((1.0 - std::cos(theta)) / (theta * theta)) * w +
        ((theta - std::sin(theta)) / (theta * theta * theta)) * w * w;
  }
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = rotation.cast<float>();
  out.block<3, 1>(0, 3) = (v * rho).cast<float>();
  return out;
}

Eigen::Matrix4f rightPerturb(const paper::Pose3d& anchor, const Seed& seed) {
  constexpr double degrees_to_radians = M_PI / 180.0;
  Eigen::Matrix<double, 6, 1> xi;
  xi << seed.dx, seed.dy, seed.dz, seed.roll_deg * degrees_to_radians,
      seed.pitch_deg * degrees_to_radians, seed.yaw_deg * degrees_to_radians;
  Eigen::Matrix4f base = Eigen::Matrix4f::Identity();
  base.block<3, 3>(0, 0) = anchor.orientation.normalized().toRotationMatrix().cast<float>();
  base.block<3, 1>(0, 3) = anchor.position.cast<float>();
  return base * expSE3(xi);
}

paper::Pose3d fromMatrix(const Eigen::Matrix4f& matrix) {
  paper::Pose3d pose;
  pose.position = matrix.block<3, 1>(0, 3).cast<double>();
  pose.orientation = Eigen::Quaterniond(matrix.block<3, 3>(0, 0).cast<double>()).normalized();
  return pose;
}

Eigen::Matrix4d poseMatrix(const paper::Pose3d& pose) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix();
  matrix.block<3, 1>(0, 3) = pose.position;
  return matrix;
}

void poseString(std::ostream& out, const paper::Pose3d& pose) {
  out << std::setprecision(17) << pose.position.x() << ';' << pose.position.y() << ';'
      << pose.position.z() << ';' << pose.orientation.x() << ';'
      << pose.orientation.y() << ';' << pose.orientation.z() << ';'
      << pose.orientation.w();
}

void matrixString(std::ostream& out, const paper::Pose3d& pose) {
  const Eigen::Matrix4d matrix = poseMatrix(pose);
  bool first = true;
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column) {
      if (!first) out << ';';
      first = false;
      out << std::setprecision(17) << matrix(row, column);
    }
}

double translationDistance(const paper::Pose3d& a, const paper::Pose3d& b) {
  return (a.position - b.position).norm();
}

double rotationDistanceDegrees(const paper::Pose3d& a, const paper::Pose3d& b) {
  Eigen::Quaterniond relative = a.orientation.conjugate() * b.orientation;
  relative.normalize();
  return 2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w())) * 180.0 / M_PI;
}

void run(const std::string& cohort_path, const std::string& map_path,
         const std::string& output_path) {
  const std::vector<CohortFrame> frames = readCohort(cohort_path);
  paper::CurrentFrameNdtRegistration registration(paper::CurrentFrameNdtParameters{});
  std::string reason;
  if (!registration.loadMap(map_path, &reason))
    throw std::runtime_error("map_load_failed:" + reason);
  const auto leaf = registration.targetGridLeafSizeMeters();
  const auto& parameters = registration.parameters();
  if (std::abs(parameters.resolution_m - 0.8) > 1e-12 ||
      std::abs(leaf[0] - 0.8f) > 1e-6f || std::abs(leaf[1] - 0.8f) > 1e-6f ||
      std::abs(leaf[2] - 0.8f) > 1e-6f)
    throw std::runtime_error("runtime_target_grid_not_0p8m");
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot_create_candidate_output");
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output << std::setprecision(17)
      << "frame_id,transaction_id,time_s,selection_labels,segment,seed_index,seed_domain,"
         "seed_dx_m,seed_dy_m,seed_dz_m,seed_roll_deg,seed_pitch_deg,seed_yaw_deg,"
         "wide_radius_m,wide_angle_deg,grid_ix,grid_iy,grid_iyaw,start_pose_xyz_q_xyzw,"
         "final_pose_xyz_q_xyzw,final_pose_matrix16,final_translation_from_anchor_m,"
         "final_rotation_from_anchor_deg,source_points,target_points,source_hash_expected,"
         "source_hash_actual,fitness,transformation_probability,raw_ndt_score_sum,iterations,"
         "converged,runtime_ms,final_offset_from_seed_m,final_rotation_from_seed_deg,"
         "input_bag_sha256,input_map_sha256\n";
  std::size_t total_runs = 0;
  for (const CohortFrame& frame : frames) {
    const paper::RegistrationCloud raw = readRawCloud(frame);
    std::vector<Seed> seeds = planarSeeds(frame.resolution_m);
    const std::vector<Seed> axial = axialSeeds(frame.resolution_m);
    seeds.insert(seeds.end(), axial.begin(), axial.end());
    if (frame.targeted) {
      const auto wide = widePlanarSeeds(frame.resolution_m);
      seeds.insert(seeds.end(), wide.begin(), wide.end());
    }
    const std::size_t expected_seeds = frame.targeted ? 311 : 263;
    if (seeds.size() != expected_seeds)
      throw std::runtime_error("frozen_seed_schedule_count_mismatch:" + frame.frame_id);
    int zero_seed_index = -1;
    const auto frame_started = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < seeds.size(); ++index) {
      const Seed& seed = seeds[index];
      if (seed.domain == "PLANAR" && seed.dx == 0.0 && seed.dy == 0.0 && seed.yaw_deg == 0.0)
        zero_seed_index = static_cast<int>(index);
      const paper::Pose3d start_pose = fromMatrix(rightPerturb(frame.initial_pose, seed));
      paper::CurrentFrameNdtResult result;
      if (!registration.align(frame.stamp_ns, raw, start_pose, &result, &reason))
        throw std::runtime_error("candidate_ndt_call_failed:" + frame.frame_id + ":" + reason);
      if (result.source_cloud_hash != frame.prepared_source_hash ||
          result.source_point_count != frame.prepared_source_point_count ||
          result.target_point_count != frame.target_point_count)
        throw std::runtime_error("candidate_source_or_target_provenance_mismatch:" + frame.frame_id);
      paper::NdtObjectiveSample objective;
      if (!registration.evaluateLocalObjectiveAtPose(result.raw_map_T_lidar,
              &objective, &reason) || !objective.valid)
        throw std::runtime_error("candidate_terminal_score_failed:" + frame.frame_id + ":" + reason);
      const double anchor_translation = translationDistance(frame.initial_pose, result.raw_map_T_lidar);
      const double anchor_rotation = rotationDistanceDegrees(frame.initial_pose, result.raw_map_T_lidar);
      const double seed_translation = translationDistance(start_pose, result.raw_map_T_lidar);
      const double seed_rotation = rotationDistanceDegrees(start_pose, result.raw_map_T_lidar);
      output << frame.frame_id << ',' << frame.transaction_id << ',' << frame.time_s << ",\""
          << frame.selection_labels << "\"," << frame.segment << ',' << index << ','
          << seed.domain << ',' << seed.dx << ',' << seed.dy << ',' << seed.dz << ','
          << seed.roll_deg << ',' << seed.pitch_deg << ',' << seed.yaw_deg << ','
          << seed.wide_radius_m << ',' << seed.wide_angle_deg << ',' << seed.grid_ix << ','
          << seed.grid_iy << ',' << seed.grid_iyaw << ",\"";
      poseString(output, start_pose);
      output << "\",\"";
      poseString(output, result.raw_map_T_lidar);
      output << "\",\"";
      matrixString(output, result.raw_map_T_lidar);
      output << '"' << ',' << anchor_translation << ',' << anchor_rotation << ','
          << result.source_point_count << ',' << result.target_point_count << ','
          << frame.prepared_source_hash << ',' << result.source_cloud_hash << ','
          << result.fitness << ',' << result.transformation_probability << ','
          << objective.score_sum << ',' << result.iterations << ',' << result.converged << ','
          << result.alignment_ms << ',' << seed_translation << ',' << seed_rotation << ','
          << frame.input_bag_sha256 << ',' << frame.input_map_sha256 << '\n';
      ++total_runs;
    }
    if (zero_seed_index < 0) throw std::runtime_error("nominal_prediction_seed_missing");
    output.flush();
    std::cerr << "MULTISTART_FRAME=" << frame.frame_id << " tx=" << frame.transaction_id
        << " planned=" << seeds.size() << " targeted=" << frame.targeted
        << " zero_seed_index=" << zero_seed_index << " elapsed_s="
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - frame_started).count()
        << '\n';
  }
  if (total_runs != 8800) throw std::runtime_error("total_candidate_alignment_count_not_8800");
  output.close();
  std::cout << "FRAMES=" << frames.size() << " TOTAL_RUNS=" << total_runs
      << " TARGET_GRID=" << leaf[0] << ',' << leaf[1] << ',' << leaf[2]
      << " RESOLUTION=" << parameters.resolution_m << " STEP=" << parameters.step_size
      << " EPSILON=" << parameters.transformation_epsilon
      << " MAX_ITERATIONS=" << parameters.maximum_iterations << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: dual_u_r1_multistart_runner FROZEN_COHORT.csv MAP.pcd OUTPUT.csv\n";
    return 2;
  }
  try {
    run(argv[1], argv[2], argv[3]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "DUAL_U_R1_MULTISTART_FAIL=" << error.what() << '\n';
    return 1;
  }
}
