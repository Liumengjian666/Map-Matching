#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include "p5_i1_input_hash.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;
constexpr double kResolution = 0.8;

class AuditedNdt : public Ndt {
 public:
  double fixedScore(Cloud& transformed, Eigen::Matrix<double, 6, 1>& p,
                    Eigen::Matrix<double, 6, 6>* hessian_out) {
    Eigen::Matrix<double, 6, 1> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    const double score = computeDerivatives(gradient, hessian, transformed, p, true);
    if (hessian_out) *hessian_out = hessian;
    return score;
  }
};

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '"') {
      if (quoted && i + 1 < text.size() && text[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (c == delimiter && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  if (quoted) throw std::runtime_error("unterminated quoted CSV field");
  fields.push_back(field);
  return fields;
}

std::string csvQuote(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
  std::string out = "\"";
  for (char c : value) {
    if (c == '"') out.push_back('"');
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

Eigen::Matrix4f parsePose(const std::string& text) {
  const auto values = split(text, ';');
  if (values.size() != 7) throw std::runtime_error("pose must have xyz + xyzw fields");
  Eigen::Quaternionf q(std::stof(values[6]), std::stof(values[3]),
                       std::stof(values[4]), std::stof(values[5]));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f)
    throw std::runtime_error("invalid pose quaternion");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3f(std::stof(values[0]),
                                           std::stof(values[1]),
                                           std::stof(values[2]));
  return pose;
}

Eigen::Matrix4f parseMatrix16(const std::string& text) {
  const auto values = split(text, ';');
  if (values.size() != 16) throw std::runtime_error("SE3 matrix must contain 16 values");
  Eigen::Matrix4f pose;
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      pose(row, col) = std::stof(values[static_cast<std::size_t>(row * 4 + col)]);
  if (!pose.allFinite() || (pose.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() > 1e-4f)
    throw std::runtime_error("invalid homogeneous pose matrix");
  return pose;
}

Eigen::Matrix<double, 6, 1> parseXi(const std::string& text) {
  const auto values = split(text, ';');
  if (values.size() != 6) throw std::runtime_error("SE3 tangent must contain six values");
  Eigen::Matrix<double, 6, 1> xi;
  for (int i = 0; i < 6; ++i) xi[i] = std::stod(values[static_cast<std::size_t>(i)]);
  if (!xi.allFinite()) throw std::runtime_error("invalid SE3 tangent values");
  return xi;
}

std::string poseString(const Eigen::Matrix4f& pose) {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();
  std::ostringstream out;
  out << std::setprecision(15) << pose(0, 3) << ';' << pose(1, 3) << ';'
      << pose(2, 3) << ';' << q.x() << ';' << q.y() << ';' << q.z() << ';' << q.w();
  return out.str();
}

std::string matrixString(const Eigen::Matrix4f& pose) {
  std::ostringstream out;
  out << std::setprecision(15);
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) {
      if (row || col) out << ';';
      out << pose(row, col);
    }
  return out.str();
}

double rotationDistanceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Quaterniond qa(a.block<3, 3>(0, 0).cast<double>());
  const Eigen::Quaterniond qb(b.block<3, 3>(0, 0).cast<double>());
  Eigen::Quaterniond relative = qa.conjugate() * qb;
  relative.normalize();
  return 2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w())) * 180.0 / M_PI;
}

double translationDistance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).cast<double>().norm();
}

void finalize(const Cloud::Ptr& cloud) {
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf, int cap) {
  Cloud::Ptr down(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.setInputCloud(input);
  voxel.filter(*down);
  if (cap > 0 && static_cast<int>(down->size()) > cap) {
    Cloud::Ptr sampled(new Cloud);
    sampled->reserve(static_cast<std::size_t>(cap));
    const double increment = static_cast<double>(down->size() - 1) / static_cast<double>(cap - 1);
    for (int i = 0; i < cap; ++i)
      sampled->push_back(down->points[static_cast<std::size_t>(std::llround(i * increment))]);
    finalize(sampled);
    return sampled;
  }
  finalize(down);
  return down;
}

Cloud::Ptr loadTarget(const std::string& path) {
  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile<Point>(path, *raw) != 0)
    throw std::runtime_error("failed to load frozen prior map: " + path);
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite->push_back(point);
  finalize(finite);
  return voxelDown(voxelDown(finite, 0.15, 0), 0.15, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const Point& point : raw->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range >= 0.5 && range <= 80.0) filtered->push_back(point);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25, 1400);
}

uint64_t sourceCloudHash(const Cloud::Ptr& cloud) {
  constexpr uint64_t kOffset = 1469598103934665603ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  auto mix = [&hash](uint8_t byte) { hash = (hash ^ byte) * kPrime; };
  auto mixU32 = [&mix](uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8) mix(static_cast<uint8_t>(value >> shift));
  };
  mixU32(cloud->width); mixU32(cloud->height); mixU32(cloud->is_dense ? 1U : 0U);
  mixU32(static_cast<uint32_t>(cloud->size()));
  for (const Point& point : cloud->points) {
    uint32_t bits;
    std::memcpy(&bits, &point.x, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &point.y, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &point.z, sizeof(bits)); mixU32(bits);
  }
  return hash;
}

Eigen::Matrix<double, 6, 1> pclTransformVector(const Eigen::Matrix4f& pose) {
  Eigen::Transform<float, 3, Eigen::Affine> transform;
  transform.matrix() = pose;
  const Eigen::Vector3f angles = transform.rotation().eulerAngles(0, 1, 2);
  Eigen::Matrix<double, 6, 1> p;
  p << transform.translation().x(), transform.translation().y(), transform.translation().z(),
       angles.x(), angles.y(), angles.z();
  return p;
}

void configureNdt(AuditedNdt& ndt, const Cloud::Ptr& target) {
  ndt.setInputTarget(target);
  ndt.setResolution(kResolution);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(0.001);
  ndt.setMaximumIterations(40);
}

struct Frame {
  std::string id, cohorts, labels, bag_sha, map_sha, gt_sha, extrinsics_sha, corrected_anchor_sha;
  uint64_t transaction_id = 0, source_hash = 0;
  double time_s = 0.0, saved_fitness = 0.0;
  int saved_iterations = 0, saved_converged = 0, saved_step_limited = 0;
  Eigen::Matrix4f predicted = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f saved_raw = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f saved_used = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f gt_lidar = Eigen::Matrix4f::Identity();
};

struct Seed {
  int index = 0;
  std::string name, coordinate_frame;
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
};

struct Curvature {
  bool valid = false;
  bool negative_definite = false;
  double minimum = std::numeric_limits<double>::quiet_NaN();
  double maximum = std::numeric_limits<double>::quiet_NaN();
  double condition = std::numeric_limits<double>::quiet_NaN();
  double asymmetry = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 6> eigenvalues{};
  bool eigenvalues_valid = false;
};

std::string eigenvaluesString(const Curvature& curvature) {
  std::ostringstream out;
  out << std::setprecision(15);
  for (std::size_t i = 0; i < curvature.eigenvalues.size(); ++i) {
    if (i) out << ';';
    out << (curvature.eigenvalues_valid ? curvature.eigenvalues[i] :
            std::numeric_limits<double>::quiet_NaN());
  }
  return out.str();
}

std::vector<std::vector<std::string>> readCsv(const std::string& path,
                                               std::map<std::string, std::size_t>* columns) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open CSV: " + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty CSV: " + path);
  const auto header = split(line, ',');
  columns->clear();
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (!columns->emplace(header[i], i).second) throw std::runtime_error("duplicate CSV column: " + header[i]);
  }
  std::vector<std::vector<std::string>> rows;
  std::size_t line_number = 1;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) continue;
    auto row = split(line, ',');
    if (row.size() != header.size())
      throw std::runtime_error("CSV width mismatch at " + path + ":" + std::to_string(line_number));
    rows.push_back(std::move(row));
  }
  return rows;
}

const std::string& field(const std::vector<std::string>& row,
                         const std::map<std::string, std::size_t>& columns,
                         const std::string& name) {
  const auto found = columns.find(name);
  if (found == columns.end() || found->second >= row.size())
    throw std::runtime_error("missing CSV field: " + name);
  return row[found->second];
}

std::vector<Frame> readFrames(const std::string& path) {
  const std::string manifest_sha = p5_i1::sha256File(path);
  if (manifest_sha != "c520bb0232f98b3e5eb65528d14ca5732ca62291e560ed9cac285fc24b83a291")
    throw std::runtime_error("frozen P5-I2 frame-manifest SHA256 mismatch: " + manifest_sha);
  std::map<std::string, std::size_t> columns;
  const auto rows = readCsv(path, &columns);
  std::vector<Frame> frames;
  for (const auto& row : rows) {
    Frame frame;
    frame.id = field(row, columns, "frame_id");
    frame.transaction_id = std::stoull(field(row, columns, "transaction_id"));
    frame.time_s = std::stod(field(row, columns, "time_s"));
    frame.cohorts = field(row, columns, "cohorts");
    frame.labels = field(row, columns, "selection_labels");
    frame.source_hash = std::stoull(field(row, columns, "ndt_source_cloud_hash"));
    frame.saved_fitness = std::stod(field(row, columns, "fitness_saved"));
    frame.saved_iterations = std::stoi(field(row, columns, "iterations_saved"));
    frame.saved_converged = std::stoi(field(row, columns, "converged_saved"));
    frame.saved_step_limited = std::stoi(field(row, columns, "step_limited_saved"));
    frame.predicted = parsePose(field(row, columns, "predicted_pose_xyz_q_xyzw"));
    frame.saved_raw = parsePose(field(row, columns, "saved_raw_pose_xyz_q_xyzw"));
    frame.saved_used = parsePose(field(row, columns, "saved_used_pose_xyz_q_xyzw"));
    frame.gt_lidar = parsePose(field(row, columns, "gt_map_T_lidar_xyz_q_xyzw"));
    frame.bag_sha = field(row, columns, "input_bag_sha256");
    frame.map_sha = field(row, columns, "input_map_sha256");
    frame.gt_sha = field(row, columns, "gt_sha256");
    frame.extrinsics_sha = field(row, columns, "extrinsics_sha256");
    frame.corrected_anchor_sha = field(row, columns, "corrected_anchor_sha256");
    if (frame.bag_sha != p5_i1::kExpectedBagSha256 || frame.map_sha != p5_i1::kExpectedMapSha256 ||
        frame.gt_sha != "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f" ||
        frame.extrinsics_sha != "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414" ||
        frame.corrected_anchor_sha != "ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416")
      throw std::runtime_error("frame manifest input provenance mismatch at " + frame.id);
    frames.push_back(frame);
  }
  if (frames.empty()) throw std::runtime_error("frame manifest has no rows");
  return frames;
}

std::vector<Seed> readSeeds(const std::string& path) {
  std::map<std::string, std::size_t> columns;
  const auto rows = readCsv(path, &columns);
  std::vector<Seed> seeds;
  for (const auto& row : rows) {
    Seed seed;
    seed.index = std::stoi(field(row, columns, "seed_index"));
    seed.name = field(row, columns, "seed_name");
    seed.dx = std::stod(field(row, columns, "dx_m"));
    seed.dy = std::stod(field(row, columns, "dy_m"));
    seed.dz = std::stod(field(row, columns, "dz_m"));
    seed.roll_deg = std::stod(field(row, columns, "roll_deg"));
    seed.pitch_deg = std::stod(field(row, columns, "pitch_deg"));
    seed.yaw_deg = std::stod(field(row, columns, "yaw_deg"));
    seed.coordinate_frame = field(row, columns, "coordinate_frame");
    if (seed.coordinate_frame != "right_body")
      throw std::runtime_error("oracle seed must use right_body perturbations");
    seeds.push_back(seed);
  }
  std::sort(seeds.begin(), seeds.end(), [](const Seed& a, const Seed& b) { return a.index < b.index; });
  if (seeds.size() != 7) throw std::runtime_error("P5-I2 fixed oracle seed set must contain exactly seven user-specified seeds");
  if (seeds.front().name != "GT_EXACT") throw std::runtime_error("seed 0 must be GT_EXACT");
  const std::map<std::string, std::array<double, 6>> required = {
      {"GT_EXACT", {0, 0, 0, 0, 0, 0}}, {"DX_P400", {.4, 0, 0, 0, 0, 0}},
      {"DX_M400", {-.4, 0, 0, 0, 0, 0}}, {"DY_P400", {0, .4, 0, 0, 0, 0}},
      {"DY_M400", {0, -.4, 0, 0, 0, 0}}, {"YAW_P5", {0, 0, 0, 0, 0, 5}},
      {"YAW_M5", {0, 0, 0, 0, 0, -5}}};
  std::map<std::string, bool> seen;
  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const Seed& seed = seeds[i];
    if (seed.index != static_cast<int>(i)) throw std::runtime_error("oracle seed indices must be unique and contiguous from zero");
    const auto found = required.find(seed.name);
    if (found == required.end() || seen[seed.name])
      throw std::runtime_error("unexpected or duplicate fixed oracle seed: " + seed.name);
    seen[seed.name] = true;
    const std::array<double, 6> actual = {seed.dx, seed.dy, seed.dz, seed.roll_deg, seed.pitch_deg, seed.yaw_deg};
    for (std::size_t j = 0; j < actual.size(); ++j)
      if (std::abs(actual[j] - found->second[j]) > 1e-12)
        throw std::runtime_error("seed coordinates differ from frozen protocol for " + seed.name);
  }
  for (const auto& item : required)
    if (!seen[item.first]) throw std::runtime_error("missing fixed oracle seed: " + item.first);
  return seeds;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
  Eigen::Matrix3d out;
  out << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return out;
}

Eigen::Matrix4f expSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d omega = xi.tail<3>();
  const double theta = omega.norm();
  const Eigen::Matrix3d W = skew(omega);
  Eigen::Matrix3d rotation, V;
  if (theta < 1e-10) {
    rotation = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
    V = Eigen::Matrix3d::Identity() + 0.5 * W + W * W / 6.0;
  } else {
    rotation = Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * W +
               ((1.0 - std::cos(theta)) / (theta * theta)) * W * W;
    V = Eigen::Matrix3d::Identity() + ((1.0 - std::cos(theta)) / (theta * theta)) * W +
        ((theta - std::sin(theta)) / (theta * theta * theta)) * W * W;
  }
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = rotation.cast<float>();
  out.block<3, 1>(0, 3) = (V * rho).cast<float>();
  return out;
}

Eigen::Matrix4f rightPerturb(const Eigen::Matrix4f& anchor, const Seed& seed) {
  constexpr double kDeg = M_PI / 180.0;
  Eigen::Matrix<double, 6, 1> xi;
  xi << seed.dx, seed.dy, seed.dz, seed.roll_deg * kDeg,
        seed.pitch_deg * kDeg, seed.yaw_deg * kDeg;
  return anchor * expSE3(xi);
}

Curvature curvatureAt(AuditedNdt& ndt, const Cloud::Ptr& source,
                      const Eigen::Matrix4f& pose, double* score_out = nullptr) {
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, pose);
  Eigen::Matrix<double, 6, 1> p = pclTransformVector(pose);
  Eigen::Matrix<double, 6, 6> hessian;
  const double score = ndt.fixedScore(transformed, p, &hessian);
  if (score_out) *score_out = score;
  Curvature out;
  if (!std::isfinite(score) || !hessian.allFinite()) return out;
  out.asymmetry = (hessian - hessian.transpose()).norm();
  Eigen::Matrix<double, 6, 6> scale = Eigen::Matrix<double, 6, 6>::Identity();
  scale.diagonal().head<3>().setConstant(kResolution);
  Eigen::Matrix<double, 6, 6> negative_scaled =
      -scale.transpose() * (0.5 * (hessian + hessian.transpose())) * scale;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(negative_scaled);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) return out;
  out.minimum = eig.eigenvalues().minCoeff();
  out.maximum = eig.eigenvalues().maxCoeff();
  for (int i = 0; i < 6; ++i) out.eigenvalues[static_cast<std::size_t>(i)] = eig.eigenvalues()[i];
  out.eigenvalues_valid = true;
  out.negative_definite = out.minimum > 0.0;
  out.condition = out.negative_definite ? out.maximum / out.minimum : -1.0;
  out.valid = true;
  return out;
}

double fixedScore(AuditedNdt& ndt, const Cloud::Ptr& source,
                  const Eigen::Matrix4f& pose) {
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, pose);
  Eigen::Matrix<double, 6, 1> p = pclTransformVector(pose);
  return ndt.fixedScore(transformed, p, nullptr);
}

std::string csvNumbers(const std::vector<double>& values) {
  std::ostringstream out;
  out << std::setprecision(15);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) out << ';';
    out << values[i];
  }
  return out.str();
}

void runOracle(const std::string& bag_path, const std::string& map_path,
               const std::string& manifest_path, const std::string& cloud_dir,
               const std::string& seed_path, const std::string& runs_path,
               const std::string& objective_path) {
  const std::string actual_bag_sha = p5_i1::sha256File(bag_path);
  if (actual_bag_sha != p5_i1::kExpectedBagSha256)
    throw std::runtime_error("runtime-topic bag SHA256 mismatch: " + actual_bag_sha);
  p5_i1::requireFrozenMapSha256(map_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("fixed target point-count mismatch");
  const std::vector<Frame> frames = readFrames(manifest_path);
  const std::vector<Seed> seeds = readSeeds(seed_path);

  std::ofstream runs(runs_path);
  std::ofstream objective(objective_path);
  if (!runs || !objective) throw std::runtime_error("cannot open P5-I2 output CSV");
  runs << "frame_id,transaction_id,time_s,cohorts,selection_labels,seed_index,seed_name,"
          "seed_dx_m,seed_dy_m,seed_dz_m,seed_roll_deg,seed_pitch_deg,seed_yaw_deg,"
          "coordinate_frame,start_pose_xyz_q_xyzw,final_pose_xyz_q_xyzw,final_pose_matrix16,"
          "source_points,target_points,source_hash_expected,source_hash_actual,fitness,"
          "transformation_probability,raw_ndt_score_sum,per_point_score,iterations,converged,"
          "align_runtime_ms,analytic_hessian_valid,negative_definite,min_scaled_eigenvalue,"
          "max_scaled_eigenvalue,condition_number,hessian_asymmetry_frobenius,scaled_eigenvalues,"
          "input_bag_sha256,input_map_sha256\n";
  objective << "frame_id,transaction_id,time_s,cohorts,selection_labels,source_points,target_points,"
               "source_hash_expected,source_hash_actual,predicted_pose_xyz_q_xyzw,"
               "saved_raw_pose_xyz_q_xyzw,saved_used_pose_xyz_q_xyzw,gt_pose_xyz_q_xyzw,"
               "baseline_replay_pose_xyz_q_xyzw,baseline_replay_translation_delta_m,"
               "baseline_replay_rotation_delta_deg,baseline_replay_fitness_delta,"
               "baseline_replay_iterations_match,baseline_replay_converged_match,"
               "baseline_replay_runtime_ms,baseline_saved_fitness,baseline_replay_fitness,"
               "J_GT_FIXED,J_BASE,J_ORACLE_EXACT,s_GT,s_base,s_oracle,delta_oracle_base,relative_gap,"
               "baseline_hessian_valid,baseline_negative_definite,baseline_min_scaled_eigenvalue,"
               "baseline_max_scaled_eigenvalue,baseline_condition_number,baseline_hessian_asymmetry,baseline_scaled_eigenvalues,"
               "oracle_exact_hessian_valid,oracle_exact_negative_definite,oracle_exact_min_scaled_eigenvalue,"
               "oracle_exact_max_scaled_eigenvalue,oracle_exact_condition_number,oracle_exact_hessian_asymmetry,oracle_exact_scaled_eigenvalues,"
               "oracle_exact_pose_xyz_q_xyzw,oracle_exact_pose_matrix16,baseline_pose_matrix16,gt_pose_matrix16,"
               "input_bag_sha256,input_map_sha256\n";

  AuditedNdt ndt;
  configureNdt(ndt, target);
  double max_t_delta = 0.0, max_r_delta = 0.0, max_fit_delta = 0.0;
  bool baseline_gate = true;
  std::size_t oracle_run_count = 0;
  const auto all_start = std::chrono::steady_clock::now();
  for (const Frame& frame : frames) {
    const std::string source_path = cloud_dir + "/" + frame.id + ".pcd";
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<Point>(source_path, *raw) != 0)
      throw std::runtime_error("cannot load extracted cloud " + source_path);
    const Cloud::Ptr source = preprocessSource(raw);
    const uint64_t actual_source_hash = sourceCloudHash(source);
    if (actual_source_hash != frame.source_hash)
      throw std::runtime_error("preprocessed source hash mismatch for " + frame.id);
    ndt.setInputSource(source);

    Cloud baseline_aligned;
    const auto baseline_start = std::chrono::steady_clock::now();
    ndt.align(baseline_aligned, frame.predicted);
    const double baseline_runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - baseline_start).count();
    const Eigen::Matrix4f baseline_replay = ndt.getFinalTransformation();
    const double baseline_t_delta = translationDistance(baseline_replay, frame.saved_raw);
    const double baseline_r_delta = rotationDistanceDeg(baseline_replay, frame.saved_raw);
    const double baseline_replay_fitness = ndt.getFitnessScore();
    const double baseline_fit_delta = std::abs(baseline_replay_fitness - frame.saved_fitness);
    const int baseline_replay_iterations = ndt.getFinalNumIteration();
    const int baseline_replay_converged = ndt.hasConverged() ? 1 : 0;
    const bool iterations_match = baseline_replay_iterations == frame.saved_iterations;
    const bool converged_match = baseline_replay_converged == frame.saved_converged;
    max_t_delta = std::max(max_t_delta, baseline_t_delta);
    max_r_delta = std::max(max_r_delta, baseline_r_delta);
    max_fit_delta = std::max(max_fit_delta, baseline_fit_delta);
    const bool frame_baseline_pass = baseline_t_delta <= 1e-3 && baseline_r_delta <= 0.01 &&
                                     iterations_match && converged_match;
    baseline_gate = baseline_gate && frame_baseline_pass;

    double j_base = 0.0, j_gt = 0.0;
    const Curvature base_curvature = curvatureAt(ndt, source, frame.saved_raw, &j_base);
    j_gt = fixedScore(ndt, source, frame.gt_lidar);
    const double s_base = j_base / static_cast<double>(source->size());
    const double s_gt = j_gt / static_cast<double>(source->size());

    bool exact_found = false;
    double j_oracle_exact = 0.0;
    Eigen::Matrix4f oracle_exact = Eigen::Matrix4f::Identity();
    Curvature oracle_exact_curvature;
    double oracle_exact_runtime_ms = 0.0;
    for (const Seed& seed : seeds) {
      const Eigen::Matrix4f start_pose = rightPerturb(frame.gt_lidar, seed);
      Cloud aligned;
      const auto seed_start = std::chrono::steady_clock::now();
      ndt.align(aligned, start_pose);
      const double runtime_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - seed_start).count();
      const Eigen::Matrix4f final_pose = ndt.getFinalTransformation();
      const double fitness = ndt.getFitnessScore();
      const double probability = ndt.getTransformationProbability();
      const int iterations = ndt.getFinalNumIteration();
      const int converged = ndt.hasConverged() ? 1 : 0;
      double score = 0.0;
      const Curvature curv = curvatureAt(ndt, source, final_pose, &score);
      const double per_point = score / static_cast<double>(source->size());
      runs << std::setprecision(15) << frame.id << ',' << frame.transaction_id << ',' << frame.time_s << ','
           << csvQuote(frame.cohorts) << ',' << csvQuote(frame.labels) << ',' << seed.index << ','
           << csvQuote(seed.name) << ',' << seed.dx << ',' << seed.dy << ',' << seed.dz << ','
           << seed.roll_deg << ',' << seed.pitch_deg << ',' << seed.yaw_deg << ','
           << seed.coordinate_frame << ',' << poseString(start_pose) << ',' << poseString(final_pose) << ','
           << matrixString(final_pose) << ',' << source->size() << ',' << target->size() << ','
           << frame.source_hash << ',' << actual_source_hash << ',' << fitness << ','
           << probability << ',' << score << ',' << per_point << ','
           << iterations << ',' << converged << ',' << runtime_ms << ','
           << (curv.valid ? 1 : 0) << ',' << (curv.negative_definite ? 1 : 0) << ','
           << curv.minimum << ',' << curv.maximum << ',' << curv.condition << ',' << curv.asymmetry << ','
           << eigenvaluesString(curv) << ','
           << actual_bag_sha << ',' << p5_i1::kExpectedMapSha256 << '\n';
      ++oracle_run_count;
      if (seed.name == "GT_EXACT") {
        if (exact_found) throw std::runtime_error("duplicate GT_EXACT seed");
        exact_found = true;
        j_oracle_exact = score;
        oracle_exact = final_pose;
        oracle_exact_curvature = curv;
        oracle_exact_runtime_ms = runtime_ms;
      }
    }
    if (!exact_found) throw std::runtime_error("oracle seeds did not include GT_EXACT");
    const double s_oracle = j_oracle_exact / static_cast<double>(source->size());
    const double delta = s_oracle - s_base;
    const double relative_gap = delta / std::max({std::abs(s_oracle), std::abs(s_base), 1e-12});
    objective << std::setprecision(15) << frame.id << ',' << frame.transaction_id << ',' << frame.time_s << ','
              << csvQuote(frame.cohorts) << ',' << csvQuote(frame.labels) << ',' << source->size() << ','
              << target->size() << ',' << frame.source_hash << ',' << actual_source_hash << ','
              << poseString(frame.predicted) << ',' << poseString(frame.saved_raw) << ','
              << poseString(frame.saved_used) << ',' << poseString(frame.gt_lidar) << ','
              << poseString(baseline_replay) << ',' << baseline_t_delta << ',' << baseline_r_delta << ','
              << baseline_fit_delta << ',' << (iterations_match ? 1 : 0) << ',' << (converged_match ? 1 : 0) << ','
              << baseline_runtime_ms << ',' << frame.saved_fitness << ',' << baseline_replay_fitness << ','
              << j_gt << ',' << j_base << ',' << j_oracle_exact << ',' << s_gt << ',' << s_base << ','
              << s_oracle << ',' << delta << ',' << relative_gap << ','
              << (base_curvature.valid ? 1 : 0) << ',' << (base_curvature.negative_definite ? 1 : 0) << ','
              << base_curvature.minimum << ',' << base_curvature.maximum << ',' << base_curvature.condition << ','
              << base_curvature.asymmetry << ',' << eigenvaluesString(base_curvature) << ','
              << (oracle_exact_curvature.valid ? 1 : 0) << ','
              << (oracle_exact_curvature.negative_definite ? 1 : 0) << ','
              << oracle_exact_curvature.minimum << ',' << oracle_exact_curvature.maximum << ','
              << oracle_exact_curvature.condition << ',' << oracle_exact_curvature.asymmetry << ','
              << eigenvaluesString(oracle_exact_curvature) << ','
              << poseString(oracle_exact) << ',' << matrixString(oracle_exact) << ','
              << matrixString(frame.saved_raw) << ',' << matrixString(frame.gt_lidar) << ','
              << actual_bag_sha << ',' << p5_i1::kExpectedMapSha256 << '\n';
    std::cerr << "ORACLE_FRAME=" << frame.id << " time_s=" << frame.time_s
              << " seeds=" << seeds.size() << " source_points=" << source->size()
              << " baseline_replay_diff=" << baseline_t_delta << "m/" << baseline_r_delta << "deg"
              << " J_base=" << j_base << " J_oracle_exact=" << j_oracle_exact
              << " oracle_ms=" << oracle_exact_runtime_ms << '\n';
  }
  runs.flush();
  objective.flush();
  if (!runs || !objective) throw std::runtime_error("failed writing P5-I2 outputs");
  std::cout << "FRAMES=" << frames.size() << "\nSEEDS_PER_FRAME=" << seeds.size()
            << "\nORACLE_RUNS=" << oracle_run_count << "\nTARGET_POINTS=" << target->size()
            << "\nMAX_BASELINE_REPLAY_TRANSLATION_DELTA_M=" << std::setprecision(15) << max_t_delta
            << "\nMAX_BASELINE_REPLAY_ROTATION_DELTA_DEG=" << max_r_delta
            << "\nMAX_BASELINE_REPLAY_FITNESS_DELTA=" << max_fit_delta
            << "\nBASELINE_REPLAY_GATE=" << (baseline_gate ? "PASS" : "FAIL")
            << "\nELAPSED_SEC=" << std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - all_start).count() << '\n';
  if (!baseline_gate) throw std::runtime_error("baseline NDT replay did not close for all P5-I2 frames");
}

void runProfiles(const std::string& bag_path, const std::string& map_path,
                 const std::string& manifest_path, const std::string& cloud_dir,
                 const std::string& request_path, const std::string& output_path) {
  const std::string actual_bag_sha = p5_i1::sha256File(bag_path);
  if (actual_bag_sha != p5_i1::kExpectedBagSha256)
    throw std::runtime_error("runtime-topic bag SHA256 mismatch: " + actual_bag_sha);
  p5_i1::requireFrozenMapSha256(map_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("fixed target point-count mismatch");
  const std::vector<Frame> frames = readFrames(manifest_path);
  std::map<std::string, Frame> by_id;
  for (const Frame& frame : frames) by_id.emplace(frame.id, frame);

  std::map<std::string, std::size_t> columns;
  const auto requests = readCsv(request_path, &columns);
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot open geodesic profile output: " + output_path);
  output << "frame_id,time_s,alpha,source_points,target_points,source_hash_expected,source_hash_actual,"
            "score,per_point_score,pose_matrix16,endpoint_translation_delta_m,endpoint_rotation_delta_deg,"
            "evaluation_runtime_ms,input_bag_sha256,input_map_sha256\n";

  AuditedNdt ndt;
  configureNdt(ndt, target);
  std::size_t evaluations = 0;
  double max_endpoint_t = 0.0, max_endpoint_r = 0.0;
  const auto all_start = std::chrono::steady_clock::now();
  for (const auto& row : requests) {
    const std::string& id = field(row, columns, "frame_id");
    const auto frame_it = by_id.find(id);
    if (frame_it == by_id.end()) throw std::runtime_error("profile request frame absent from manifest: " + id);
    const Frame& frame = frame_it->second;
    const uint64_t expected_hash = std::stoull(field(row, columns, "source_hash_expected"));
    if (expected_hash != frame.source_hash)
      throw std::runtime_error("profile request source hash does not match manifest for " + id);
    const Eigen::Matrix4f correct = parseMatrix16(field(row, columns, "correct_pose_matrix16"));
    const Eigen::Matrix4f wrong = parseMatrix16(field(row, columns, "wrong_pose_matrix16"));
    const Eigen::Matrix<double, 6, 1> xi = parseXi(field(row, columns, "xi_rho_omega"));
    const Eigen::Matrix4f endpoint = correct * expSE3(xi);
    const double endpoint_t = translationDistance(endpoint, wrong);
    const double endpoint_r = rotationDistanceDeg(endpoint, wrong);
    max_endpoint_t = std::max(max_endpoint_t, endpoint_t);
    max_endpoint_r = std::max(max_endpoint_r, endpoint_r);
    if (endpoint_t > 1e-4 || endpoint_r > 1e-3)
      throw std::runtime_error("geodesic endpoint closure failed for " + id);

    Cloud::Ptr raw(new Cloud);
    const std::string source_path = cloud_dir + "/" + id + ".pcd";
    if (pcl::io::loadPCDFile<Point>(source_path, *raw) != 0)
      throw std::runtime_error("cannot load extracted cloud " + source_path);
    const Cloud::Ptr source = preprocessSource(raw);
    const uint64_t source_hash = sourceCloudHash(source);
    if (source_hash != expected_hash) throw std::runtime_error("profile source-cloud hash mismatch for " + id);
    ndt.setInputSource(source);

    for (int i = 0; i <= 50; ++i) {
      const double alpha = 0.02 * i;
      const Eigen::Matrix4f pose = correct * expSE3(alpha * xi);
      const auto start = std::chrono::steady_clock::now();
      const double score = fixedScore(ndt, source, pose);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      output << std::setprecision(15) << id << ',' << frame.time_s << ',' << alpha << ','
             << source->size() << ',' << target->size() << ',' << expected_hash << ',' << source_hash << ','
             << score << ',' << score / static_cast<double>(source->size()) << ','
             << matrixString(pose) << ',' << endpoint_t << ',' << endpoint_r << ',' << elapsed_ms << ','
             << actual_bag_sha << ',' << p5_i1::kExpectedMapSha256 << '\n';
      ++evaluations;
    }
    std::cerr << "PROFILE_FRAME=" << id << " samples=51 endpoint_delta=" << endpoint_t
              << "m/" << endpoint_r << "deg\n";
  }
  output.flush();
  if (!output) throw std::runtime_error("failed writing geodesic profiles");
  std::cout << "PROFILE_FRAMES=" << requests.size() << "\nPROFILE_EVALUATIONS=" << evaluations
            << "\nMAX_ENDPOINT_TRANSLATION_DELTA_M=" << std::setprecision(15) << max_endpoint_t
            << "\nMAX_ENDPOINT_ROTATION_DELTA_DEG=" << max_endpoint_r
            << "\nELAPSED_SEC=" << std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - all_start).count() << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 9 && std::string(argv[1]) == "run") {
      runOracle(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7], argv[8]);
    } else if (argc == 8 && std::string(argv[1]) == "profile") {
      runProfiles(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);
    } else {
      std::cerr << "usage:\n"
                   "  p5_i2_ndt_oracle run BAG MAP MANIFEST CLOUD_DIR SEEDS RUNS.csv OBJECTIVE.csv\n"
                   "  p5_i2_ndt_oracle profile BAG MAP MANIFEST CLOUD_DIR REQUESTS.csv PROFILES.csv\n";
      return argc == 1 ? 0 : 2;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "p5_i2 NDT oracle error: " << error.what() << '\n';
    return 1;
  }
}
