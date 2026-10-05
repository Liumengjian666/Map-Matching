#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

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
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using PointSupport = std::vector<std::array<std::uint64_t, 3>>;
using Support = std::vector<PointSupport>;
constexpr double kResolution = 0.8;
constexpr double kOutlierRatio = 0.55;
constexpr double kPi = 3.14159265358979323846;

struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, std::size_t> columns;
  std::size_t col(const std::string& name) const {
    const auto found = columns.find(name);
    if (found == columns.end()) throw std::runtime_error("missing CSV column: " + name);
    return found->second;
  }
  const std::string& get(const std::vector<std::string>& row,
                         const std::string& name) const {
    const std::size_t index = col(name);
    if (index >= row.size()) throw std::runtime_error("short CSV row at " + name);
    return row[index];
  }
};

std::vector<std::string> parseCsvLine(const std::string& line) {
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
  if (quoted) throw std::runtime_error("unterminated CSV quote");
  fields.push_back(field);
  return fields;
}

CsvTable readCsv(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open CSV: " + path);
  CsvTable table;
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty CSV: " + path);
  table.header = parseCsvLine(line);
  for (std::size_t i = 0; i < table.header.size(); ++i)
    table.columns[table.header[i]] = i;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    table.rows.push_back(parseCsvLine(line));
  }
  return table;
}

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> values;
  std::stringstream input(text);
  std::string value;
  while (std::getline(input, value, delimiter)) values.push_back(value);
  return values;
}

std::uint64_t parseU64(const std::string& text) {
  return static_cast<std::uint64_t>(std::stoull(text));
}

void finalize(const Cloud::Ptr& cloud) {
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, float leaf) {
  Cloud::Ptr down(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(input);
  voxel.filter(*down);
  finalize(down);
  return down;
}

Cloud::Ptr loadTarget(const std::string& path) {
  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile<Point>(path, *raw) != 0)
    throw std::runtime_error("cannot load target PCD: " + path);
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
      finite->push_back(point);
  finalize(finite);
  return voxelDown(voxelDown(finite, 0.15f), 0.15f);
}

Cloud::Ptr loadPackedSource(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open packed xyzf source: " + path);
  Cloud::Ptr raw(new Cloud);
  input.seekg(0, std::ios::end);
  const std::streamoff bytes = input.tellg();
  if (bytes < 0 || bytes % static_cast<std::streamoff>(3 * sizeof(float)) != 0)
    throw std::runtime_error("packed xyzf byte count is not xyz aligned: " + path);
  input.seekg(0, std::ios::beg);
  raw->reserve(static_cast<std::size_t>(bytes / (3 * sizeof(float))));
  while (input) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (input.gcount() == 0) break;
    if (input.gcount() != static_cast<std::streamsize>(sizeof(xyz)))
      throw std::runtime_error("truncated packed xyzf source: " + path);
    raw->push_back(Point(xyz[0], xyz[1], xyz[2]));
  }
  finalize(raw);
  return raw;
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range >= 0.5 && range <= 80.0) finite->push_back(point);
  }
  finalize(finite);
  Cloud::Ptr down = voxelDown(finite, 0.25f);
  if (down->size() <= 1400) return down;
  Cloud::Ptr capped(new Cloud);
  capped->reserve(1400);
  const double increment = static_cast<double>(down->size() - 1) / 1399.0;
  for (int i = 0; i < 1400; ++i)
    capped->push_back(down->points[static_cast<std::size_t>(std::llround(i * increment))]);
  finalize(capped);
  return capped;
}

std::uint64_t sourceHash(const Cloud& cloud) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](std::uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8)
      hash = (hash ^ static_cast<std::uint8_t>(value >> shift)) * 1099511628211ULL;
  };
  mix(static_cast<std::uint32_t>(cloud.size()));
  mix(1);
  mix(1);
  mix(static_cast<std::uint32_t>(cloud.size()));
  for (const Point& point : cloud.points) {
    for (float value : {point.x, point.y, point.z}) {
      std::uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      mix(bits);
    }
  }
  return hash;
}

Eigen::Matrix4f parseMatrix16(const std::string& text) {
  const auto fields = split(text, ';');
  if (fields.size() != 16) throw std::runtime_error("expected row-major 4x4 pose matrix");
  Eigen::Matrix4f matrix;
  for (int i = 0; i < 16; ++i)
    matrix(i / 4, i % 4) = std::stof(fields[static_cast<std::size_t>(i)]);
  if (!matrix.allFinite()) throw std::runtime_error("nonfinite pose matrix");
  return matrix;
}

Eigen::Matrix4f parsePose(const std::string& text) {
  const auto fields = split(text, ';');
  if (fields.size() != 7) throw std::runtime_error("expected xyz+xyzw pose");
  Eigen::Quaternionf q(std::stof(fields[6]), std::stof(fields[3]),
                       std::stof(fields[4]), std::stof(fields[5]));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f)
    throw std::runtime_error("invalid pose quaternion");
  q.normalize();
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3f(std::stof(fields[0]),
      std::stof(fields[1]), std::stof(fields[2]));
  return matrix;
}

double rotationDistanceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  Eigen::Quaterniond qa(a.block<3, 3>(0, 0).cast<double>());
  Eigen::Quaterniond qb(b.block<3, 3>(0, 0).cast<double>());
  qa.normalize();
  qb.normalize();
  Eigen::Quaterniond delta = qa.conjugate() * qb;
  delta.normalize();
  // Quaternion atan2 avoids acos(trace(R)-1), which amplified float SO(3)
  // round-off into a spurious ~0.04 deg for two bit-identical rotations.
  const double vector_norm = delta.vec().norm();
  return 2.0 * std::atan2(vector_norm, std::abs(delta.w())) * 180.0 / kPi;
}

double translationDistance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

std::uint64_t bits(double value) {
  std::uint64_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}

class ExactPclNdt : public Ndt {
 public:
  using Leaf = TargetGridLeafConstPtr;
  using FrozenSupport = std::vector<std::vector<Leaf>>;

  void configureScoreConstants() {
    const double c1 = 10.0 * (1.0 - outlier_ratio_);
    const double c2 = outlier_ratio_ / std::pow(static_cast<double>(resolution_), 3.0);
    const double d3 = -std::log(c2);
    gauss_d1_ = -std::log(c1 + c2) - d3;
    gauss_d2_ = -2.0 * std::log(
        (-std::log(c1 * std::exp(-0.5) + c2) - d3) / gauss_d1_);
    point_gradient_.setZero();
    point_gradient_.block<3, 3>(0, 0).setIdentity();
    point_hessian_.setZero();
  }

  std::array<float, 3> actualGridLeaf() const {
    const Eigen::Vector3f leaf = target_cells_.getLeafSize();
    return {{leaf.x(), leaf.y(), leaf.z()}};
  }

  double scoreJet(const Cloud::ConstPtr& source, const Eigen::Matrix4f& pose,
      Vector6d* gradient, Matrix6d* hessian, Support* support,
      FrozenSupport* frozen_support = nullptr, bool compute_hessian = true,
      const Vector6d* parameter_override = nullptr) {
    if (!source || source->empty()) throw std::runtime_error("empty prepared source");
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> affine;
    affine.matrix() = pose;
    Vector6d p;
    if (parameter_override) {
      p = *parameter_override;
    } else {
      const Eigen::Vector3f angles = affine.rotation().eulerAngles(0, 1, 2);
      p << affine.translation().x(), affine.translation().y(), affine.translation().z(),
           angles.x(), angles.y(), angles.z();
    }
    Vector6d local_gradient;
    Matrix6d local_hessian;
    const double score = computeDerivatives(local_gradient, local_hessian,
        transformed, p, compute_hessian);
    if (gradient) *gradient = local_gradient;
    if (hessian) *hessian = local_hessian;
    if (support || frozen_support) captureSupport(transformed, support, frozen_support);
    return score;
  }

  double frozenScore(const Cloud::ConstPtr& source, const Eigen::Matrix4f& pose,
                     const FrozenSupport& support) const {
    if (!source || support.size() != source->size())
      throw std::runtime_error("frozen support/source count mismatch");
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    double score = 0.0;
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      const Point& point = transformed.points[i];
      const Eigen::Vector3d x(point.x, point.y, point.z);
      for (const Leaf& leaf : support[i]) {
        const Eigen::Vector3d residual = x - leaf->getMean();
        const double exponential = std::exp(-gauss_d2_ *
            residual.dot(leaf->getInverseCov() * residual) / 2.0);
        const double derivative_guard = gauss_d2_ * exponential;
        if (!std::isfinite(derivative_guard) || derivative_guard > 1.0 ||
            derivative_guard < 0.0) continue;
        score += -gauss_d1_ * exponential;
      }
    }
    return score;
  }

  // Exact PCL score accumulation without derivative work. The R1A
  // derivative-free control uses this dynamic radius-search evaluator.
  double dynamicValueOnly(const Cloud::ConstPtr& source,
                          const Eigen::Matrix4f& pose, Support* support = nullptr) {
    if (!source || source->empty() || !pose.allFinite())
      throw std::runtime_error("invalid source/pose for dynamic score evaluation");
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    if (support) support->assign(transformed.size(), PointSupport());
    double score = 0.0;
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      std::vector<Leaf> leaves;
      std::vector<float> distances;
      target_cells_.radiusSearch(transformed.points[i], resolution_, leaves, distances);
      const Point& point = transformed.points[i];
      const Eigen::Vector3d x(point.x, point.y, point.z);
      for (const Leaf& leaf : leaves) {
        const Eigen::Vector3d residual = x - leaf->getMean();
        const double exponential = std::exp(-gauss_d2_ *
            residual.dot(leaf->getInverseCov() * residual) / 2.0);
        const double guard = gauss_d2_ * exponential;
        if (std::isfinite(guard) && guard <= 1.0 && guard >= 0.0)
          score += -gauss_d1_ * exponential;
        if (support) {
          const Eigen::Vector3d mean = leaf->getMean();
          (*support)[i].push_back({{bits(mean.x()), bits(mean.y()), bits(mean.z())}});
        }
      }
      if (support) std::sort((*support)[i].begin(), (*support)[i].end());
    }
    return score;
  }

 private:
  void captureSupport(const Cloud& transformed, Support* support,
                      FrozenSupport* frozen_support) {
    if (support) { support->clear(); support->resize(transformed.size()); }
    if (frozen_support) { frozen_support->clear(); frozen_support->resize(transformed.size()); }
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      std::vector<Leaf> leaves;
      std::vector<float> distances;
      target_cells_.radiusSearch(transformed.points[i], resolution_, leaves, distances);
      if (support) (*support)[i].reserve(leaves.size());
      if (frozen_support) (*frozen_support)[i].reserve(leaves.size());
      for (const Leaf& leaf : leaves) {
        if (frozen_support) (*frozen_support)[i].push_back(leaf);
        if (support) {
          const Eigen::Vector3d mean = leaf->getMean();
          (*support)[i].push_back({{bits(mean.x()), bits(mean.y()), bits(mean.z())}});
        }
      }
      if (support) std::sort((*support)[i].begin(), (*support)[i].end());
    }
  }
};

struct PoseCandidate {
  std::string frame_id;
  std::uint64_t transaction = 0;
  std::string cluster_id;
  std::string seed_index;
  double saved_score = 0.0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double selection_score = -std::numeric_limits<double>::infinity();
};

struct FrameSource {
  std::string frame_id;
  std::uint64_t transaction = 0;
  std::string path;
  std::uint64_t expected_hash = 0;
  std::size_t expected_points = 0;
  Eigen::Matrix4f baseline_terminal = Eigen::Matrix4f::Identity();
};

std::map<std::uint64_t, FrameSource> readFrameSources(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::map<std::uint64_t, FrameSource> result;
  for (const auto& row : table.rows) {
    FrameSource item;
    item.frame_id = table.get(row, "frame_id");
    item.transaction = parseU64(table.get(row, "transaction_id"));
    item.path = table.get(row, "raw_cloud_file");
    item.expected_hash = parseU64(table.get(row, "prepared_source_hash"));
    item.expected_points = static_cast<std::size_t>(parseU64(
        table.get(row, "prepared_source_point_count")));
    item.baseline_terminal = parsePose(table.get(row, "raw_terminal_pose_xyz_q_xyzw"));
    if (!result.emplace(item.transaction, item).second)
      throw std::runtime_error("duplicate cohort transaction");
  }
  return result;
}

std::vector<PoseCandidate> readClusterRepresentatives(
    const std::string& cluster_path, const std::string& candidate_path,
    const std::set<std::uint64_t>& requested,
    const std::map<std::uint64_t, FrameSource>& frame_sources) {
  const CsvTable candidate_table = readCsv(candidate_path);
  std::map<std::uint64_t, std::vector<PoseCandidate>> terminals;
  for (const auto& row : candidate_table.rows) {
    const std::uint64_t tx = parseU64(candidate_table.get(row, "transaction_id"));
    if (!requested.count(tx)) continue;
    PoseCandidate item;
    item.frame_id = candidate_table.get(row, "frame_id");
    item.transaction = tx;
    item.seed_index = candidate_table.get(row, "seed_index");
    item.saved_score = std::stod(candidate_table.get(row, "raw_ndt_score_sum"));
    item.pose = parseMatrix16(candidate_table.get(row, "final_pose_matrix16"));
    item.selection_score = item.saved_score;
    terminals[tx].push_back(item);
  }

  const CsvTable clusters = readCsv(cluster_path);
  std::vector<PoseCandidate> representatives;
  std::map<std::uint64_t, std::set<std::string>> matched_seed_indices;
  for (const auto& row : clusters.rows) {
    const std::uint64_t tx = parseU64(clusters.get(row, "transaction_id"));
    if (!requested.count(tx) || clusters.get(row, "threshold_set") != "primary" ||
        clusters.get(row, "stable_mode_candidate") != "1") continue;
    PoseCandidate representative;
    representative.frame_id = clusters.get(row, "frame_id");
    representative.transaction = tx;
    representative.cluster_id = clusters.get(row, "cluster_id");
    representative.pose = parseMatrix16(clusters.get(row, "representative_pose_matrix16"));
    representative.selection_score = std::stod(clusters.get(row, "best_score"));

    const auto found = terminals.find(tx);
    if (found == terminals.end()) throw std::runtime_error("no candidate terminals for selected frame");
    double best_distance = std::numeric_limits<double>::infinity();
    const PoseCandidate* matched = nullptr;
    for (const PoseCandidate& candidate : found->second) {
      const double dt = translationDistance(representative.pose, candidate.pose);
      const double dr = rotationDistanceDeg(representative.pose, candidate.pose);
      const double d = std::max(dt / 0.2, dr / 2.0);
      if (d < best_distance - 1e-10 ||
          (std::abs(d - best_distance) <= 1e-10 && matched &&
           candidate.saved_score > matched->saved_score)) {
        best_distance = d;
        matched = &candidate;
      }
    }
    if (!matched || best_distance > 1.0 + 1e-4) {
      std::ostringstream message;
      message << "cannot map primary cluster " << representative.cluster_id
              << " to an archived terminal; nearest normalized distance=" << best_distance;
      throw std::runtime_error(message.str());
    }
    if (!matched_seed_indices[tx].insert(matched->seed_index).second)
      throw std::runtime_error("two primary clusters mapped to the same archived seed terminal");
    representative.pose = matched->pose;
    representative.saved_score = matched->saved_score;
    representative.seed_index = matched->seed_index;
    representatives.push_back(representative);
  }
  std::map<std::uint64_t, std::vector<PoseCandidate>> by_frame;
  for (const PoseCandidate& candidate : representatives)
    by_frame[candidate.transaction].push_back(candidate);

  // The contract check needs the nominal archived terminal and one distinct,
  // highest-scoring supported competitor per frame, not every minor cluster.
  // Full cluster coverage remains in the oracle archive for reduced-search
  // recall evaluation.
  std::vector<PoseCandidate> selected;
  for (const std::uint64_t tx : requested) {
    const auto frame = frame_sources.find(tx);
    if (frame == frame_sources.end()) throw std::runtime_error("selected tx missing frozen cohort row");
    const auto frame_reps = by_frame.find(tx);
    if (frame_reps == by_frame.end() || frame_reps->second.empty())
      throw std::runtime_error("selected tx has no primary supported basin");

    PoseCandidate nominal;
    nominal.frame_id = frame->second.frame_id;
    nominal.transaction = tx;
    nominal.cluster_id = "BASELINE_NOMINAL";
    nominal.seed_index = "cohort_raw_terminal";
    nominal.pose = frame->second.baseline_terminal;
    nominal.selection_score = -std::numeric_limits<double>::infinity();
    double nearest = std::numeric_limits<double>::infinity();
    const PoseCandidate* matched_nominal = nullptr;
    for (const PoseCandidate& candidate : terminals[tx]) {
      const double dt = translationDistance(nominal.pose, candidate.pose);
      const double dr = rotationDistanceDeg(nominal.pose, candidate.pose);
      const double d = std::max(dt / 0.001, dr / 0.01);
      if (d < nearest) { nearest = d; matched_nominal = &candidate; }
    }
    if (!matched_nominal || nearest > 1.0 + 1e-4)
      throw std::runtime_error("frozen cohort nominal terminal cannot be matched to its archived seed");
    nominal.pose = matched_nominal->pose;
    nominal.saved_score = matched_nominal->saved_score;
    nominal.seed_index = matched_nominal->seed_index;
    nominal.cluster_id = "BASELINE_NOMINAL_ARCHIVE_MATCH";
    selected.push_back(nominal);

    std::sort(frame_reps->second.begin(), frame_reps->second.end(),
        [](const PoseCandidate& a, const PoseCandidate& b) {
          if (a.selection_score != b.selection_score) return a.selection_score > b.selection_score;
          return a.cluster_id < b.cluster_id;
        });
    const PoseCandidate* competitor = nullptr;
    for (const PoseCandidate& candidate : frame_reps->second) {
      const double dt = translationDistance(nominal.pose, candidate.pose);
      const double dr = rotationDistanceDeg(nominal.pose, candidate.pose);
      if (dt > 0.2 || dr > 2.0) { competitor = &candidate; break; }
    }
    if (competitor) selected.push_back(*competitor);
  }
  return selected;
}

std::vector<std::uint64_t> parseTransactions(const std::string& text) {
  std::vector<std::uint64_t> result;
  for (const std::string& item : split(text, ';'))
    if (!item.empty()) result.push_back(parseU64(item));
  return result;
}

void writeHeader(std::ofstream& out, const std::vector<std::string>& fields) {
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i) out << ',';
    out << fields[i];
  }
  out << '\n';
}

void writeRow(std::ofstream& out, const std::vector<std::string>& fields) {
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i) out << ',';
    out << fields[i];
  }
  out << '\n';
}

std::string number(double value) {
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

std::uint64_t supportSignature(const Support& support) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](std::uint64_t value) {
    for (int shift = 0; shift < 64; shift += 8)
      hash = (hash ^ static_cast<std::uint8_t>(value >> shift)) * 1099511628211ULL;
  };
  for (const auto& point : support) {
    mix(point.size());
    for (const auto& cell : point) {
      mix(cell[0]); mix(cell[1]); mix(cell[2]);
    }
  }
  return hash;
}

struct SupportChange {
  std::size_t changed_points = 0;
  std::size_t total_memberships = 0;
  std::size_t symmetric_difference = 0;
};

SupportChange compareSupport(const Support& center, const Support& other) {
  if (center.size() != other.size()) throw std::runtime_error("support source size mismatch");
  SupportChange result;
  for (std::size_t i = 0; i < center.size(); ++i) {
    result.total_memberships += other[i].size();
    std::vector<std::array<std::uint64_t, 3>> difference;
    std::set_symmetric_difference(center[i].begin(), center[i].end(),
        other[i].begin(), other[i].end(), std::back_inserter(difference));
    if (!difference.empty()) ++result.changed_points;
    result.symmetric_difference += difference.size();
  }
  return result;
}

struct ProductChart {
  bool valid = false;
  Matrix6d dp_deta = Matrix6d::Zero();
  std::array<Matrix6d, 6> d2p_deta2;
  double condition = std::numeric_limits<double>::infinity();
  Vector6d p0 = Vector6d::Zero();
};

bool unwrapEulerNear(const Eigen::Vector3d& raw, const Eigen::Vector3d& reference,
                     Eigen::Vector3d* output) {
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int branch = 0; branch < 2; ++branch) {
    Eigen::Vector3d candidate = raw;
    if (branch) {
      candidate.x() += kPi;
      candidate.y() = kPi - candidate.y();
      candidate.z() += kPi;
    }
    for (int axis = 0; axis < 3; ++axis)
      candidate(axis) += 2.0 * kPi * std::round((reference(axis) - candidate(axis)) / (2.0 * kPi));
    const double distance = (candidate - reference).squaredNorm();
    if (distance < best) { best = distance; *output = candidate; found = true; }
  }
  return found && output->allFinite();
}

Eigen::Matrix3d expRotation(const Eigen::Vector3d& theta) {
  const double angle = theta.norm();
  if (angle < 1e-14) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, theta / angle).toRotationMatrix();
}

bool parametersAtEta(const Eigen::Matrix4f& base, const Eigen::Vector3d& base_euler,
    const Vector6d& eta, Vector6d* p, double max_rotation_delta = 0.25,
    std::string* reason = nullptr) {
  if (reason) reason->clear();
  if (!p || !eta.allFinite()) { if (reason) *reason = "NONFINITE_INPUT"; return false; }
  const Eigen::Matrix3d rotation = expRotation(eta.tail<3>()) *
      base.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d angles;
  if (!unwrapEulerNear(rotation.eulerAngles(0, 1, 2), base_euler, &angles)) {
    if (reason) *reason = "UNWRAP_FAILED";
    return false;
  }
  const Eigen::Matrix3d rebuilt =
      (Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ())).toRotationMatrix();
  const double reconstruction_error = (rebuilt - rotation).norm();
  const double rotation_delta = (angles - base_euler).norm();
  // The pose carriers in the archived runtime are float32 matrices/quaternions;
  // use a float-aware reconstruction tolerance, while keeping the angular
  // chart-domain check independent and explicit.
  if (reconstruction_error > 1e-6 || rotation_delta > max_rotation_delta) {
    if (reason) {
      std::ostringstream message;
      message << "RECON=" << reconstruction_error << ":EULER_DELTA=" << rotation_delta;
      *reason = message.str();
    }
    return false;
  }
  p->head<3>() = base.block<3, 1>(0, 3).cast<double>() + kResolution * eta.head<3>();
  p->tail<3>() = angles;
  return p->allFinite();
}

Eigen::Matrix4f poseAtEta(const Eigen::Matrix4f& base, const Vector6d& eta) {
  Eigen::Matrix4f result = base;
  result.block<3, 1>(0, 3) += static_cast<float>(kResolution) * eta.head<3>().cast<float>();
  result.block<3, 3>(0, 0) =
      (expRotation(eta.tail<3>()) * base.block<3, 3>(0, 0).cast<double>()).cast<float>();
  return result;
}

ProductChart buildProductChart(const Eigen::Matrix4f& pose,
                                const Eigen::Vector3f& pcl_euler) {
  ProductChart result;
  const Eigen::Vector3d base_euler = pcl_euler.cast<double>();
  result.p0.head<3>() = pose.block<3, 1>(0, 3).cast<double>();
  result.p0.tail<3>() = base_euler;
  result.dp_deta.block<3, 3>(0, 0) = kResolution * Eigen::Matrix3d::Identity();
  constexpr double jac_step = 1e-5;
  for (int axis = 0; axis < 3; ++axis) {
    Vector6d plus = Vector6d::Zero(), minus = Vector6d::Zero();
    plus(3 + axis) = jac_step; minus(3 + axis) = -jac_step;
    Vector6d p_plus, p_minus;
    if (!parametersAtEta(pose, base_euler, plus, &p_plus) ||
        !parametersAtEta(pose, base_euler, minus, &p_minus)) return result;
    result.dp_deta.block<3, 1>(3, 3 + axis) =
        (p_plus.tail<3>() - p_minus.tail<3>()) / (2.0 * jac_step);
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(result.dp_deta.block<3, 3>(3, 3));
  if (!svd.singularValues().allFinite() || svd.singularValues().minCoeff() <= 1e-10) return result;
  result.condition = svd.singularValues().maxCoeff() / svd.singularValues().minCoeff();
  if (result.condition > 1e4) return result;
  for (auto& matrix : result.d2p_deta2) matrix.setZero();
  constexpr double hessian_step = 1e-3;
  for (int a = 0; a < 3; ++a) {
    Vector6d plus = Vector6d::Zero(), minus = Vector6d::Zero(), zero = Vector6d::Zero();
    plus(3 + a) = hessian_step; minus(3 + a) = -hessian_step;
    Vector6d pp, pm, p0;
    if (!parametersAtEta(pose, base_euler, plus, &pp) ||
        !parametersAtEta(pose, base_euler, minus, &pm) ||
        !parametersAtEta(pose, base_euler, zero, &p0)) return result;
    for (int parameter = 3; parameter < 6; ++parameter)
      result.d2p_deta2[static_cast<std::size_t>(parameter)](3 + a, 3 + a) =
          (pp(parameter) - 2.0 * p0(parameter) + pm(parameter)) /
          (hessian_step * hessian_step);
    for (int b = a + 1; b < 3; ++b) {
      Vector6d pp_eta = Vector6d::Zero(), pm_eta = Vector6d::Zero();
      Vector6d mp_eta = Vector6d::Zero(), mm_eta = Vector6d::Zero();
      pp_eta(3+a)=hessian_step; pp_eta(3+b)=hessian_step;
      pm_eta(3+a)=hessian_step; pm_eta(3+b)=-hessian_step;
      mp_eta(3+a)=-hessian_step; mp_eta(3+b)=hessian_step;
      mm_eta(3+a)=-hessian_step; mm_eta(3+b)=-hessian_step;
      Vector6d ppp, ppm, pmp, pmm;
      if (!parametersAtEta(pose, base_euler, pp_eta, &ppp) ||
          !parametersAtEta(pose, base_euler, pm_eta, &ppm) ||
          !parametersAtEta(pose, base_euler, mp_eta, &pmp) ||
          !parametersAtEta(pose, base_euler, mm_eta, &pmm)) return result;
      for (int parameter = 3; parameter < 6; ++parameter) {
        const double value = (ppp(parameter)-ppm(parameter)-pmp(parameter)+pmm(parameter)) /
            (4.0 * hessian_step * hessian_step);
        result.d2p_deta2[static_cast<std::size_t>(parameter)](3+a, 3+b) = value;
        result.d2p_deta2[static_cast<std::size_t>(parameter)](3+b, 3+a) = value;
      }
    }
  }
  result.valid = true;
  return result;
}

struct EnergyJet {
  double score = 0.0;
  double energy = 0.0;
  double mean_energy = 0.0;
  Vector6d gradient_eta = Vector6d::Zero();
  Matrix6d hessian_eta = Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> eigenvectors = Eigen::Matrix<double, 6, 6>::Identity();
  Support support;
  ExactPclNdt::FrozenSupport frozen_support;
  std::size_t supported_points = 0;
  std::size_t membership_count = 0;
  std::uint64_t support_hash = 0;
  double gradient_native_norm = 0.0;
  double gradient_eta_norm = 0.0;
  double chart_condition = 0.0;
  bool positive_curvature = false;
};

struct ArchivedUObs {
  bool valid = false;
  std::uint64_t transaction = 0;
  std::uint64_t source_hash = 0;
  std::size_t source_points = 0;
  double length_scale_m = 0.0;
  std::size_t target_points = 0;
  double configured_resolution_m = 0.0;
  Eigen::Vector3d actual_grid_leaf_m = Eigen::Vector3d::Zero();
  double step_size = 0.0;
  double epsilon = 0.0;
  int maximum_iterations = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Vector6d eigenvalues = Vector6d::Zero();
  Matrix6d eigenvectors = Matrix6d::Identity();
};

std::vector<double> parseSemicolonDoubles(const std::string& text, std::size_t count) {
  const auto fields = split(text, ';');
  if (fields.size() != count) throw std::runtime_error("unexpected semicolon-vector length");
  std::vector<double> values;
  values.reserve(count);
  for (const std::string& field : fields) values.push_back(std::stod(field));
  return values;
}

Eigen::Matrix4f parseUobsPose(const std::vector<std::string>& row, const CsvTable& table) {
  const Eigen::Vector3f position(std::stof(table.get(row, "raw_x")),
      std::stof(table.get(row, "raw_y")), std::stof(table.get(row, "raw_z")));
  Eigen::Quaternionf q(std::stof(table.get(row, "raw_qw")),
      std::stof(table.get(row, "raw_qx")), std::stof(table.get(row, "raw_qy")),
      std::stof(table.get(row, "raw_qz")));
  if (!position.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-6f)
    throw std::runtime_error("invalid archived U_obs terminal pose");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = position;
  return pose;
}

std::map<std::uint64_t, ArchivedUObs> readArchivedUObs(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::map<std::uint64_t, ArchivedUObs> result;
  for (const auto& row : table.rows) {
    ArchivedUObs item;
    item.transaction = parseU64(table.get(row, "transaction_id"));
    item.source_hash = parseU64(table.get(row, "source_cloud_hash"));
    item.source_points = static_cast<std::size_t>(parseU64(table.get(row, "source_points")));
    item.length_scale_m = std::stod(table.get(row, "length_scale_m"));
    item.target_points = static_cast<std::size_t>(parseU64(table.get(row, "target_points")));
    item.configured_resolution_m = std::stod(table.get(row, "configured_resolution_m"));
    item.actual_grid_leaf_m << std::stod(table.get(row, "target_grid_leaf_x_m")),
        std::stod(table.get(row, "target_grid_leaf_y_m")),
        std::stod(table.get(row, "target_grid_leaf_z_m"));
    item.step_size = std::stod(table.get(row, "step_size"));
    item.epsilon = std::stod(table.get(row, "transformation_epsilon"));
    item.maximum_iterations = std::stoi(table.get(row, "maximum_iterations"));
    item.valid = table.get(row, "uobs_valid") == "1" &&
                 table.get(row, "uobs_status") == "PASS_LOCAL_NDT_CURVATURE";
    item.pose = parseUobsPose(row, table);
    const std::vector<double> eigenvalues = parseSemicolonDoubles(
        table.get(row, "curvature_eigenvalues"), 6);
    const std::vector<double> eigenvectors = parseSemicolonDoubles(
        table.get(row, "curvature_eigenvectors_rowmajor"), 36);
    for (int i = 0; i < 6; ++i) item.eigenvalues(i) = eigenvalues[static_cast<std::size_t>(i)];
    for (int i = 0; i < 36; ++i)
      item.eigenvectors(i / 6, i % 6) = eigenvectors[static_cast<std::size_t>(i)];
    if (!result.emplace(item.transaction, item).second)
      throw std::runtime_error("duplicate archived U_obs transaction");
  }
  return result;
}

struct OracleBasin {
  std::uint64_t transaction = 0;
  std::string frame_id;
  std::string cluster_id;
  int seed_count = 0;
  double fraction = 0.0;
  double score = 0.0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
};

std::map<std::uint64_t, std::vector<OracleBasin>> readOracleBasins(
    const std::string& path, const std::set<std::uint64_t>& requested) {
  const CsvTable table = readCsv(path);
  std::map<std::uint64_t, std::vector<OracleBasin>> result;
  for (const auto& row : table.rows) {
    if (table.get(row, "threshold_set") != "primary" ||
        table.get(row, "stable_mode_candidate") != "1") continue;
    OracleBasin item;
    item.transaction = parseU64(table.get(row, "transaction_id"));
    if (!requested.count(item.transaction)) continue;
    item.frame_id = table.get(row, "frame_id");
    item.cluster_id = table.get(row, "cluster_id");
    item.seed_count = std::stoi(table.get(row, "seed_count"));
    item.fraction = std::stod(table.get(row, "basin_fraction"));
    item.score = std::stod(table.get(row, "best_score"));
    item.pose = parseMatrix16(table.get(row, "representative_pose_matrix16"));
    result[item.transaction].push_back(item);
  }
  for (const std::uint64_t tx : requested)
    if (result[tx].empty()) throw std::runtime_error("oracle lacks supported primary clusters for tx=" + std::to_string(tx));
  return result;
}

struct DynamicEnergy {
  double score = 0.0;
  double mean_energy = 0.0;
  Support support;
  std::uint64_t support_hash = 0;
};

DynamicEnergy evaluateDynamicEnergy(ExactPclNdt& ndt, const Cloud::ConstPtr& source,
                                   const Eigen::Matrix4f& pose) {
  DynamicEnergy result;
  Vector6d gradient;
  Matrix6d hessian;
  result.score = ndt.scoreJet(source, pose, &gradient, &hessian,
                              &result.support, nullptr, false);
  result.mean_energy = -result.score / static_cast<double>(source->size());
  result.support_hash = supportSignature(result.support);
  return result;
}

EnergyJet evaluate(ExactPclNdt& ndt, const Cloud::ConstPtr& source,
                   const Eigen::Matrix4f& pose) {
  EnergyJet result;
  Vector6d score_gradient;
  Matrix6d score_hessian;
  result.score = ndt.scoreJet(source, pose, &score_gradient, &score_hessian,
                              &result.support, &result.frozen_support);
  result.energy = -result.score;
  result.mean_energy = result.energy / static_cast<double>(source->size());
  result.gradient_native_norm = score_gradient.norm() / static_cast<double>(source->size());
  const Eigen::Vector3f euler_f = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
  const ProductChart chart = buildProductChart(pose, euler_f);
  if (!chart.valid) throw std::runtime_error("product-chart pullback failed");
  const Vector6d native_gradient = -score_gradient / static_cast<double>(source->size());
  const Matrix6d native_hessian = -score_hessian / static_cast<double>(source->size());
  result.gradient_eta = chart.dp_deta.transpose() * native_gradient;
  result.hessian_eta = chart.dp_deta.transpose() * native_hessian * chart.dp_deta;
  for (int i = 0; i < 6; ++i)
    result.hessian_eta += native_gradient(i) * chart.d2p_deta2[static_cast<std::size_t>(i)];
  result.hessian_eta = 0.5 * (result.hessian_eta + result.hessian_eta.transpose()).eval();
  result.gradient_eta_norm = result.gradient_eta.norm();
  result.chart_condition = chart.condition;
  Eigen::SelfAdjointEigenSolver<Matrix6d> eig(result.hessian_eta);
  if (eig.info() != Eigen::Success) throw std::runtime_error("U_obs eigensolve failed");
  result.eigenvalues = eig.eigenvalues();
  result.eigenvectors = eig.eigenvectors();
  result.positive_curvature = result.eigenvalues.minCoeff() > 0.0;
  for (const auto& point_support : result.support) {
    if (!point_support.empty()) ++result.supported_points;
    result.membership_count += point_support.size();
  }
  result.support_hash = supportSignature(result.support);
  return result;
}

void validateFrames(const std::string& map_path, const std::string& cohort_path,
    const std::string& candidate_path, const std::string& cluster_path,
    const std::string& transaction_list, const std::string& out_dir) {
  const auto requested_vector = parseTransactions(transaction_list);
  const std::set<std::uint64_t> requested(requested_vector.begin(), requested_vector.end());
  if (requested.size() != requested_vector.size() || requested.size() < 20)
    throw std::runtime_error("provide at least 20 unique transactions (10 control + 10 multimodal)");
  const auto frame_sources = readFrameSources(cohort_path);
  const auto representatives = readClusterRepresentatives(
      cluster_path, candidate_path, requested, frame_sources);
  if (representatives.empty()) throw std::runtime_error("no primary stable cluster representatives selected");

  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("target point count differs from frozen archive");
  ExactPclNdt ndt;
  ndt.setResolution(static_cast<float>(kResolution));
  ndt.setOulierRatio(kOutlierRatio);
  ndt.configureScoreConstants();
  ndt.setInputTarget(target);
  const auto grid = ndt.actualGridLeaf();
  if (std::abs(grid[0] - kResolution) > 1e-6f ||
      std::abs(grid[1] - kResolution) > 1e-6f ||
      std::abs(grid[2] - kResolution) > 1e-6f)
    throw std::runtime_error("actual PCL target grid leaf mismatch");

  std::ofstream terminals(out_dir + "/terminal_energy.csv");
  std::ofstream fd(out_dir + "/directional_fd.csv");
  if (!terminals || !fd) throw std::runtime_error("cannot create output CSVs");
  terminals << std::setprecision(17); fd << std::setprecision(17);
  writeHeader(terminals, {"frame_id","transaction_id","cluster_id","seed_index",
      "source_points","target_points","target_leaf_m","saved_score_sum","evaluated_score_sum",
      "score_abs_error","saved_probability","evaluated_probability","total_energy_neg_score",
      "mean_energy_neg_score_per_source","supported_source_points","supported_source_point_fraction",
      "gaussian_membership_count","support_signature_fnv64","native_gradient_norm_per_source",
      "chart_gradient_norm","chart_condition","curvature_eigenvalues_ascending",
      "curvature_positive_definite","frozen_center_score_sum","frozen_dynamic_score_abs_diff","status"});
  writeHeader(fd, {"frame_id","transaction_id","cluster_id","direction","eigen_index","h_eta",
      "dynamic_energy_zero","dynamic_energy_plus","dynamic_energy_minus",
      "frozen_energy_zero","frozen_energy_plus","frozen_energy_minus",
      "analytic_gradient_projection","frozen_fd_gradient","frozen_gradient_relative_error",
      "dynamic_fd_gradient","dynamic_gradient_relative_error",
      "analytic_curvature","frozen_fd_curvature","frozen_curvature_relative_error",
      "dynamic_fd_curvature","dynamic_curvature_relative_error",
      "changed_points_plus","changed_fraction_plus","membership_count_plus","changed_points_minus",
      "changed_fraction_minus","membership_count_minus","support_hash_zero","support_hash_plus",
      "support_hash_minus"});

  std::map<std::uint64_t, Cloud::Ptr> prepared_sources;
  for (const PoseCandidate& candidate : representatives) {
    const auto frame_found = frame_sources.find(candidate.transaction);
    if (frame_found == frame_sources.end()) throw std::runtime_error("selected tx missing cohort row");
    Cloud::Ptr source;
    auto prepared_found = prepared_sources.find(candidate.transaction);
    if (prepared_found == prepared_sources.end()) {
      const FrameSource& manifest = frame_found->second;
      Cloud::Ptr raw = loadPackedSource(manifest.path);
      source = preprocessSource(raw);
      const std::uint64_t actual_hash = sourceHash(*source);
      if (source->size() != manifest.expected_points || actual_hash != manifest.expected_hash) {
        std::ostringstream error;
        error << "prepared source provenance mismatch tx=" << candidate.transaction
              << " count=" << source->size() << '/' << manifest.expected_points
              << " hash=" << actual_hash << '/' << manifest.expected_hash;
        throw std::runtime_error(error.str());
      }
      prepared_sources.emplace(candidate.transaction, source);
    } else {
      source = prepared_found->second;
    }
    ndt.setInputSource(source);
    const EnergyJet center = evaluate(ndt, source, candidate.pose);
    const double score_error = std::abs(center.score - candidate.saved_score);
    const double saved_probability = candidate.saved_score / static_cast<double>(source->size());
    const double evaluated_probability = center.score / static_cast<double>(source->size());
    std::ostringstream eig_text;
    for (int i = 0; i < 6; ++i) { if (i) eig_text << ';'; eig_text << center.eigenvalues(i); }
    const double support_fraction = static_cast<double>(center.supported_points) /
                                    static_cast<double>(source->size());
    writeRow(terminals, {candidate.frame_id, std::to_string(candidate.transaction),
        candidate.cluster_id, candidate.seed_index, std::to_string(source->size()),
        std::to_string(target->size()), number(grid[0]), number(candidate.saved_score),
        number(center.score), number(score_error), number(saved_probability),
        number(evaluated_probability), number(center.energy), number(center.mean_energy),
        std::to_string(center.supported_points), number(support_fraction),
        std::to_string(center.membership_count), std::to_string(center.support_hash),
        number(center.gradient_native_norm), number(center.gradient_eta_norm),
        number(center.chart_condition), eig_text.str(), center.positive_curvature ? "1" : "0",
        number(ndt.frozenScore(source, candidate.pose, center.frozen_support)),
        number(std::abs(center.score - ndt.frozenScore(source, candidate.pose, center.frozen_support))),
        score_error <= 1e-6 * std::max(1.0, std::abs(candidate.saved_score)) ? "SCORE_MATCH" : "SCORE_MISMATCH"});

    for (int eigen_index : {0, 5}) {
      const Vector6d direction = center.eigenvectors.col(eigen_index);
      for (double h : {0.02, 0.01, 0.005, 0.0025, 0.001, 0.0005}) {
        const Eigen::Matrix4f plus_pose = poseAtEta(candidate.pose, h * direction);
        const Eigen::Matrix4f minus_pose = poseAtEta(candidate.pose, -h * direction);
        const DynamicEnergy plus = evaluateDynamicEnergy(ndt, source, plus_pose);
        const DynamicEnergy minus = evaluateDynamicEnergy(ndt, source, minus_pose);
        const double analytic_gradient = center.gradient_eta.dot(direction);
        const double frozen_zero = -ndt.frozenScore(source, candidate.pose, center.frozen_support);
        const double frozen_plus = -ndt.frozenScore(source, plus_pose, center.frozen_support);
        const double frozen_minus = -ndt.frozenScore(source, minus_pose, center.frozen_support);
        const double fd_gradient = (frozen_plus - frozen_minus) /
                                   (2.0 * h * static_cast<double>(source->size()));
        const double gradient_rel = std::abs(fd_gradient - analytic_gradient) /
            std::max({1e-8, std::abs(fd_gradient), std::abs(analytic_gradient)});
        const double dynamic_fd_gradient = (plus.mean_energy - minus.mean_energy) / (2.0 * h);
        const double dynamic_gradient_rel = std::abs(dynamic_fd_gradient - analytic_gradient) /
            std::max({1e-8, std::abs(dynamic_fd_gradient), std::abs(analytic_gradient)});
        const double analytic_curvature = direction.dot(center.hessian_eta * direction);
        const double fd_curvature = (frozen_plus - 2.0 * frozen_zero + frozen_minus) /
                                    (h * h * static_cast<double>(source->size()));
        const double curvature_rel = std::abs(fd_curvature - analytic_curvature) /
            std::max({1e-6, std::abs(fd_curvature), std::abs(analytic_curvature)});
        const double dynamic_fd_curvature = (plus.mean_energy - 2.0 * center.mean_energy + minus.mean_energy) /
                                             (h * h);
        const double dynamic_curvature_rel = std::abs(dynamic_fd_curvature - analytic_curvature) /
            std::max({1e-6, std::abs(dynamic_fd_curvature), std::abs(analytic_curvature)});
        const SupportChange plus_change = compareSupport(center.support, plus.support);
        const SupportChange minus_change = compareSupport(center.support, minus.support);
        writeRow(fd, {candidate.frame_id, std::to_string(candidate.transaction), candidate.cluster_id,
            eigen_index == 0 ? "WEAKEST" : "STRONGEST", std::to_string(eigen_index), number(h),
            number(center.mean_energy), number(plus.mean_energy), number(minus.mean_energy),
            number(frozen_zero / static_cast<double>(source->size())),
            number(frozen_plus / static_cast<double>(source->size())),
            number(frozen_minus / static_cast<double>(source->size())),
            number(analytic_gradient), number(fd_gradient), number(gradient_rel),
            number(dynamic_fd_gradient), number(dynamic_gradient_rel), number(analytic_curvature),
            number(fd_curvature), number(curvature_rel), number(dynamic_fd_curvature),
            number(dynamic_curvature_rel), std::to_string(plus_change.changed_points),
            number(static_cast<double>(plus_change.changed_points) / source->size()),
            std::to_string(plus_change.total_memberships), std::to_string(minus_change.changed_points),
            number(static_cast<double>(minus_change.changed_points) / source->size()),
            std::to_string(minus_change.total_memberships), std::to_string(center.support_hash),
            std::to_string(plus.support_hash), std::to_string(minus.support_hash)});
      }
    }
    std::cout << "TX=" << candidate.transaction << " CLUSTER=" << candidate.cluster_id
              << " N=" << source->size() << " SCORE_ERROR=" << score_error
              << " GRAD_ETA=" << center.gradient_eta_norm
              << " SUPPORT=" << center.supported_points << '/' << source->size()
              << " MEMBERSHIPS=" << center.membership_count << std::endl;
  }
  std::cout << "REGISTRATION_USED=NO\nNDT_ALIGN_CALL_COUNT=0\n"
            << "SELECTED_FRAME_COUNT=" << requested.size() << "\n"
            << "TERMINAL_EVALUATION_COUNT=" << representatives.size() << "\n"
            << "TARGET_POINTS=" << target->size() << "\n"
            << "TARGET_GRID_LEAF=" << grid[0] << ',' << grid[1] << ',' << grid[2] << '\n';
}

struct NativeEnergyJet {
  double energy = 0.0;
  double score = 0.0;
  Vector6d gradient = Vector6d::Zero();
  Matrix6d hessian = Matrix6d::Zero();
  Support support;
};

NativeEnergyJet evaluateNativeEnergy(ExactPclNdt& ndt, const Cloud::ConstPtr& source,
    const Eigen::Matrix4f& pose, const Vector6d& native_p,
    bool need_hessian, bool capture_support) {
  NativeEnergyJet result;
  Vector6d score_gradient;
  Matrix6d score_hessian;
  result.score = ndt.scoreJet(source, pose, &score_gradient, &score_hessian,
      capture_support ? &result.support : nullptr, nullptr, need_hessian, &native_p);
  const double count = static_cast<double>(source->size());
  result.energy = -result.score / count;
  result.gradient = -score_gradient / count;
  result.hessian = -score_hessian / count;
  result.hessian = 0.5 * (result.hessian + result.hessian.transpose()).eval();
  return result;
}

bool nativeParametersAtEta(const Eigen::Matrix4f& base,
    const Eigen::Vector3d& base_euler, const Vector6d& eta, Vector6d* p,
    std::string* reason = nullptr) {
  return parametersAtEta(base, base_euler, eta, p, 0.45, reason);
}

bool buildStrongPullback(const Eigen::Matrix4f& base,
    const Eigen::Vector3d& base_euler, const Vector6d& eta,
    const Eigen::MatrixXd& strong_basis, Eigen::MatrixXd* jacobian,
    std::array<Eigen::MatrixXd, 6>* second_derivatives) {
  if (!jacobian || !second_derivatives || strong_basis.rows() != 6 ||
      strong_basis.cols() <= 0) return false;
  const Eigen::Index dimensions = strong_basis.cols();
  jacobian->resize(6, dimensions);
  for (auto& matrix : *second_derivatives)
    matrix = Eigen::MatrixXd::Zero(dimensions, dimensions);
  constexpr double jacobian_step = 1e-5;
  constexpr double hessian_step = 1e-3;
  Vector6d p_plus, p_minus;
  for (Eigen::Index axis = 0; axis < dimensions; ++axis) {
    if (!nativeParametersAtEta(base, base_euler,
          eta + jacobian_step * strong_basis.col(axis), &p_plus) ||
        !nativeParametersAtEta(base, base_euler,
          eta - jacobian_step * strong_basis.col(axis), &p_minus)) return false;
    jacobian->col(axis) = (p_plus - p_minus) / (2.0 * jacobian_step);
    if (!nativeParametersAtEta(base, base_euler,
          eta + hessian_step * strong_basis.col(axis), &p_plus) ||
        !nativeParametersAtEta(base, base_euler,
          eta - hessian_step * strong_basis.col(axis), &p_minus)) return false;
    Vector6d p_zero;
    if (!nativeParametersAtEta(base, base_euler, eta, &p_zero)) return false;
    const Vector6d diagonal = (p_plus - 2.0 * p_zero + p_minus) /
        (hessian_step * hessian_step);
    for (int parameter = 0; parameter < 6; ++parameter)
      (*second_derivatives)[static_cast<std::size_t>(parameter)](axis, axis) = diagonal(parameter);
    for (Eigen::Index other = axis + 1; other < dimensions; ++other) {
      Vector6d pp_eta = eta + hessian_step * strong_basis.col(axis) +
          hessian_step * strong_basis.col(other);
      Vector6d pm_eta = eta + hessian_step * strong_basis.col(axis) -
          hessian_step * strong_basis.col(other);
      Vector6d mp_eta = eta - hessian_step * strong_basis.col(axis) +
          hessian_step * strong_basis.col(other);
      Vector6d mm_eta = eta - hessian_step * strong_basis.col(axis) -
          hessian_step * strong_basis.col(other);
      Vector6d pp, pm, mp, mm;
      if (!nativeParametersAtEta(base, base_euler, pp_eta, &pp) ||
          !nativeParametersAtEta(base, base_euler, pm_eta, &pm) ||
          !nativeParametersAtEta(base, base_euler, mp_eta, &mp) ||
          !nativeParametersAtEta(base, base_euler, mm_eta, &mm)) return false;
      const Vector6d mixed = (pp - pm - mp + mm) /
          (4.0 * hessian_step * hessian_step);
      for (int parameter = 0; parameter < 6; ++parameter) {
        (*second_derivatives)[static_cast<std::size_t>(parameter)](axis, other) = mixed(parameter);
        (*second_derivatives)[static_cast<std::size_t>(parameter)](other, axis) = mixed(parameter);
      }
    }
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(*jacobian);
  if (!svd.singularValues().allFinite() || svd.singularValues().minCoeff() < 1e-9) return false;
  const double condition = svd.singularValues().maxCoeff() / svd.singularValues().minCoeff();
  return std::isfinite(condition) && condition < 1e4;
}

struct ProfileNode {
  bool valid = false;
  int i = 0;
  int j = 0;
  double u0 = 0.0;
  double u1 = 0.0;
  double energy = std::numeric_limits<double>::infinity();
  double support_changed_fraction = 0.0;
  Vector6d eta = Vector6d::Zero();
  int strong_iterations = 0;
  int energy_evaluations = 0;
  std::string status = "INVALID";
};

struct ProfileMinimum {
  int profile_index = -1;
  ProfileNode node;
};

ProfileNode profileAt(ExactPclNdt& ndt, const Cloud::ConstPtr& source,
    const Eigen::Matrix4f& base, const Eigen::Vector3d& base_euler,
    const Eigen::MatrixXd& weak_basis, const Eigen::MatrixXd& strong_basis,
    const Eigen::VectorXd& weak_coordinates, const Support& center_support) {
  ProfileNode node;
  const int weak_dimension = static_cast<int>(weak_basis.cols());
  node.u0 = weak_coordinates(0);
  if (weak_dimension > 1) node.u1 = weak_coordinates(1);
  Eigen::VectorXd strong_coordinates = Eigen::VectorXd::Zero(strong_basis.cols());
  for (int iteration = 0; iteration < 1; ++iteration) {
    const Vector6d eta = weak_basis * weak_coordinates + strong_basis * strong_coordinates;
    Vector6d native_p;
    if (!nativeParametersAtEta(base, base_euler, eta, &native_p)) {
      std::string reason;
      nativeParametersAtEta(base, base_euler, eta, &native_p, &reason);
      node.status = "EULER_CHART_INVALID_" + reason;
      return node;
    }
    const Eigen::Matrix4f pose = poseAtEta(base, eta);
    NativeEnergyJet jet = evaluateNativeEnergy(ndt, source, pose, native_p, true, true);
    ++node.energy_evaluations;
    const SupportChange support_change = compareSupport(center_support, jet.support);
    node.support_changed_fraction = static_cast<double>(support_change.changed_points) /
                                    static_cast<double>(source->size());
    Eigen::MatrixXd jacobian;
    std::array<Eigen::MatrixXd, 6> second_derivatives;
    if (!buildStrongPullback(base, base_euler, eta, strong_basis,
                             &jacobian, &second_derivatives)) {
      node.status = "STRONG_CHART_INVALID";
      return node;
    }
    const Eigen::VectorXd gradient = jacobian.transpose() * jet.gradient;
    Eigen::MatrixXd hessian = jacobian.transpose() * jet.hessian * jacobian;
    for (int parameter = 0; parameter < 6; ++parameter)
      hessian += jet.gradient(parameter) * second_derivatives[static_cast<std::size_t>(parameter)];
    hessian = 0.5 * (hessian + hessian.transpose()).eval();
    node.energy = jet.energy;
    node.eta = eta;
    if (gradient.norm() < 1e-6) {
      node.status = "STRONG_STATIONARY";
      break;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(hessian);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
      node.status = "STRONG_HESSIAN_INVALID";
      return node;
    }
    const double scale = std::max(1.0, hessian.diagonal().cwiseAbs().maxCoeff());
    const double damping = std::max(1e-6 * scale, 1e-6 * scale - eig.eigenvalues().minCoeff());
    Eigen::MatrixXd damped = hessian + damping * Eigen::MatrixXd::Identity(hessian.rows(), hessian.cols());
    Eigen::LDLT<Eigen::MatrixXd> ldlt(damped);
    if (ldlt.info() != Eigen::Success) {
      node.status = "STRONG_SOLVE_INVALID";
      return node;
    }
    Eigen::VectorXd step = -ldlt.solve(gradient);
    if (!step.allFinite()) { node.status = "STRONG_STEP_NONFINITE"; return node; }
    if (step.norm() > 0.10) step *= 0.10 / step.norm();
    if (step.norm() < 1e-7) { node.status = "STRONG_STEP_SMALL"; break; }
    bool accepted = false;
    for (double alpha : {1.0, 0.5}) {
      Eigen::VectorXd trial_strong = strong_coordinates + alpha * step;
      if (trial_strong.norm() > 0.25) trial_strong *= 0.25 / trial_strong.norm();
      const Vector6d trial_eta = weak_basis * weak_coordinates + strong_basis * trial_strong;
      const Eigen::Matrix4f trial_pose = poseAtEta(base, trial_eta);
      Vector6d trial_p;
      if (!nativeParametersAtEta(base, base_euler, trial_eta, &trial_p)) continue;
      Vector6d trial_gradient;
      Matrix6d trial_hessian;
      const double trial_score = ndt.scoreJet(source, trial_pose, &trial_gradient,
          &trial_hessian, nullptr, nullptr, false, &trial_p);
      ++node.energy_evaluations;
      const double trial_energy = -trial_score / static_cast<double>(source->size());
      if (std::isfinite(trial_energy) && trial_energy < node.energy - 1e-12) {
        strong_coordinates = trial_strong;
        node.energy = trial_energy;
        node.eta = trial_eta;
        accepted = true;
        ++node.strong_iterations;
        break;
      }
    }
    if (!accepted) { node.status = "STRONG_NO_DESCENT"; break; }
  }
  if (node.status == "INVALID") node.status = "STRONG_MAX_ITER";
  const Vector6d final_eta = weak_basis * weak_coordinates + strong_basis * strong_coordinates;
  Vector6d final_p;
  if (!nativeParametersAtEta(base, base_euler, final_eta, &final_p)) {
    node.valid = false;
    node.status = "FINAL_EULER_CHART_INVALID";
    return node;
  }
  const NativeEnergyJet final_jet = evaluateNativeEnergy(ndt, source,
      poseAtEta(base, final_eta), final_p, false, true);
  ++node.energy_evaluations;
  node.energy = final_jet.energy;
  node.eta = final_eta;
  const SupportChange final_support_change = compareSupport(center_support, final_jet.support);
  node.support_changed_fraction = static_cast<double>(final_support_change.changed_points) /
                                  static_cast<double>(source->size());
  node.valid = std::isfinite(node.energy) && node.eta.allFinite();
  return node;
}

struct SearchTerminal {
  std::string label;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double energy = 0.0;
  double score = 0.0;
  bool converged = false;
  int iterations = 0;
  double runtime_ms = 0.0;
  int cluster_id = -1;
};

void se3Log(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to,
            Eigen::Matrix<double, 6, 1>* logarithm,
            double* translation_separation, double* rotation_deg) {
  const Eigen::Matrix3d r0 = from.block<3, 3>(0, 0).cast<double>();
  const Eigen::Matrix3d r1 = to.block<3, 3>(0, 0).cast<double>();
  const Eigen::Vector3d t0 = from.block<3, 1>(0, 3).cast<double>();
  const Eigen::Vector3d t1 = to.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond relative_rotation(r0.transpose() * r1);
  relative_rotation.normalize();
  if (relative_rotation.w() < 0.0) relative_rotation.coeffs() *= -1.0;
  const double vector_norm = relative_rotation.vec().norm();
  const double theta = 2.0 * std::atan2(vector_norm, relative_rotation.w());
  Eigen::Vector3d phi = Eigen::Vector3d::Zero();
  if (vector_norm >= 1e-12) phi = theta * relative_rotation.vec() / vector_norm;
  Eigen::Matrix3d omega_hat;
  omega_hat << 0.0, -phi.z(), phi.y(), phi.z(), 0.0, -phi.x(), -phi.y(), phi.x(), 0.0;
  Eigen::Matrix3d v_inverse = Eigen::Matrix3d::Identity() - 0.5 * omega_hat;
  if (theta < 1e-5) {
    v_inverse += (1.0 / 12.0) * omega_hat * omega_hat;
  } else {
    const double coefficient = 1.0 / (theta * theta) -
        (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
    v_inverse += coefficient * omega_hat * omega_hat;
  }
  const Eigen::Vector3d relative_translation = r0.transpose() * (t1 - t0);
  logarithm->head<3>() = v_inverse * relative_translation;
  logarithm->tail<3>() = phi;
  *translation_separation = (t1 - t0).norm();
  *rotation_deg = theta * 180.0 / kPi;
}

Vector6d mapChartDisplacement(const Eigen::Matrix4f& from,
                              const Eigen::Matrix4f& to) {
  Vector6d eta;
  eta.head<3>() = (to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).cast<double>() /
                  kResolution;
  const Eigen::Quaterniond q_from(from.block<3, 3>(0, 0).cast<double>());
  const Eigen::Quaterniond q_to(to.block<3, 3>(0, 0).cast<double>());
  Eigen::Quaterniond delta = q_to * q_from.conjugate();
  delta.normalize();
  if (delta.w() < 0.0) delta.coeffs() *= -1.0;
  const double vector_norm = delta.vec().norm();
  const double angle = 2.0 * std::atan2(vector_norm, delta.w());
  eta.tail<3>().setZero();
  if (vector_norm >= 1e-12) eta.tail<3>() = angle * delta.vec() / vector_norm;
  return eta;
}

bool runMathSelfTests() {
  const Eigen::Matrix3f rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, -1.0).normalized())
      .toRotationMatrix().cast<float>();
  Eigen::Matrix4f first = Eigen::Matrix4f::Identity();
  first.block<3, 3>(0, 0) = rotation;
  first.block<3, 1>(0, 3) = Eigen::Vector3f(1.0f, -2.0f, 0.5f);
  const Eigen::Matrix4f identical = first;
  Eigen::Matrix4f rotated = first;
  rotated.block<3, 3>(0, 0) =
      (Eigen::AngleAxisf(static_cast<float>(kPi / 180.0), Eigen::Vector3f::UnitY()) *
       Eigen::Quaternionf(rotation)).toRotationMatrix();
  if (rotationDistanceDeg(first, identical) > 1e-5) return false;
  if (std::abs(rotationDistanceDeg(first, rotated) - 1.0) > 1e-3) return false;
  if (mapChartDisplacement(first, identical).tail<3>().norm() > 1e-7) return false;
  Vector6d logarithm;
  double translation = 0.0, rotation_deg = 0.0;
  se3Log(first, identical, &logarithm, &translation, &rotation_deg);
  return logarithm.norm() < 1e-7 && translation < 1e-7 && rotation_deg < 1e-5;
}

bool sameBasin(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return translationDistance(a, b) <= 0.2 && rotationDistanceDeg(a, b) <= 2.0;
}

void runProfileSearch(const std::string& map_path, const std::string& cohort_path,
    const std::string& candidate_path, const std::string& cluster_path,
    const std::string& uobs_path, const std::string& transaction_list,
    const std::string& out_dir) {
  (void)candidate_path;
  const auto frame_sources = readFrameSources(cohort_path);
  std::set<std::uint64_t> requested;
  if (transaction_list == "ALL") {
    for (const auto& entry : frame_sources) requested.insert(entry.first);
  } else {
    const auto parsed = parseTransactions(transaction_list);
    requested.insert(parsed.begin(), parsed.end());
    if (requested.size() != parsed.size()) throw std::runtime_error("duplicate profile transaction id");
  }
  if (requested.empty()) throw std::runtime_error("profile search needs at least one archived frame");
  const auto archived_uobs = readArchivedUObs(uobs_path);
  const auto oracle = readOracleBasins(cluster_path, requested);
  std::vector<std::pair<std::size_t, std::uint64_t>> low_cluster_rank;
  low_cluster_rank.reserve(oracle.size());
  for (const auto& entry : oracle) low_cluster_rank.emplace_back(entry.second.size(), entry.first);
  std::sort(low_cluster_rank.begin(), low_cluster_rank.end(),
      [](const auto& a, const auto& b) {
        return a.first != b.first ? a.first < b.first : a.second < b.second;
      });
  std::set<std::uint64_t> low_cluster_control;
  for (std::size_t i = 0; i < std::min<std::size_t>(20, low_cluster_rank.size()); ++i)
    low_cluster_control.insert(low_cluster_rank[i].second);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("profile target count differs from same-objective archive");
  ExactPclNdt ndt;
  ndt.setResolution(static_cast<float>(kResolution));
  ndt.setOulierRatio(kOutlierRatio);
  ndt.configureScoreConstants();
  ndt.setInputTarget(target);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(1e-5);
  ndt.setMaximumIterations(80);
  const auto grid_leaf = ndt.actualGridLeaf();

  std::ofstream landscape(out_dir + "/profile_landscape.csv");
  std::ofstream terminals_out(out_dir + "/profile_terminals.csv");
  std::ofstream relations(out_dir + "/terminal_relations.csv");
  std::ofstream recovery(out_dir + "/oracle_recovery.csv");
  std::ofstream summary(out_dir + "/profile_summary.csv");
  if (!landscape || !terminals_out || !relations || !recovery || !summary)
    throw std::runtime_error("cannot create reduced-search sidecars");
  for (std::ofstream* out : {&landscape, &terminals_out, &relations, &recovery, &summary})
    *out << std::setprecision(17);
  writeHeader(landscape, {"transaction_id","frame_id","k","grid_i","grid_j","u0","u1",
      "profile_energy_mean_neg_score","support_changed_source_fraction_vs_T0",
      "strong_refinement_iterations","energy_evaluations","status","eta"});
  writeHeader(terminals_out, {"transaction_id","frame_id","terminal_label","converged","iterations",
      "runtime_ms","score_sum","mean_energy_neg_score","cluster_id","pose_matrix16"});
  writeHeader(relations, {"transaction_id","frame_id","terminal_i","terminal_j","se3_log_rho_phi",
      "translation_separation_m","rotation_separation_deg"});
  writeHeader(recovery, {"transaction_id","frame_id","oracle_supported_clusters","oracle_primary_total_clusters",
      "major_competing_oracle_clusters","recovered_major_clusters","missed_major_cluster_ids",
      "reduced_terminal_clusters","profile_minima","full_ndt_refinements","profile_energy_evaluations",
      "k","weak_span_projection_fraction_mean","status"});
  writeHeader(summary, {"transaction_id","frame_id","k","lambda0","lambda1","lambda2",
      "lambda1_over_lambda0","fallback_bound_used","u_bound_0","u_bound_1","grid_points_evaluated",
      "profile_minima","full_ndt_refinements","terminal_clusters","energy_evaluations",
      "runtime_ms","status"});

  std::map<std::uint64_t, Cloud::Ptr> source_cache;
  std::size_t total_profile_evals = 0;
  std::size_t total_full_refines = 0;
  std::size_t total_minima = 0;
  std::size_t k1_frames = 0, k2_frames = 0;
  std::size_t nominal_control_frames = 0;
  std::size_t nominal_control_with_competing_terminal = 0;
  std::size_t processed_frames = 0;
  const auto total_start = std::chrono::steady_clock::now();

  for (const std::uint64_t tx : requested) {
    const auto frame_start = std::chrono::steady_clock::now();
    const auto frame_it = frame_sources.find(tx);
    const auto uobs_it = archived_uobs.find(tx);
    if (frame_it == frame_sources.end() || uobs_it == archived_uobs.end())
      throw std::runtime_error("profile frame missing frozen cohort or U_obs row: " + std::to_string(tx));
    const FrameSource& frame = frame_it->second;
    const ArchivedUObs& uobs = uobs_it->second;
    if (!uobs.valid || uobs.source_hash != frame.expected_hash ||
        uobs.source_points != frame.expected_points || uobs.target_points != target->size() ||
        std::abs(uobs.length_scale_m - kResolution) > 1e-6 ||
        std::abs(uobs.configured_resolution_m - kResolution) > 1e-6 ||
        (uobs.actual_grid_leaf_m - Eigen::Vector3d::Constant(grid_leaf[0])).cwiseAbs().maxCoeff() > 1e-6 ||
        std::abs(uobs.step_size - 0.08) > 1e-8 || std::abs(uobs.epsilon - 1e-5) > 1e-10 ||
        uobs.maximum_iterations != 80)
      throw std::runtime_error("U_obs/objective contract mismatch at tx=" + std::to_string(tx));
    const double uobs_terminal_dt = translationDistance(uobs.pose, frame.baseline_terminal);
    const double uobs_terminal_dr = rotationDistanceDeg(uobs.pose, frame.baseline_terminal);
    if (uobs_terminal_dt > 0.001 || uobs_terminal_dr > 0.01) {
      std::ostringstream message;
      message << "archived U_obs pose does not match frozen nominal terminal at tx=" << tx
              << " (translation_m=" << number(uobs_terminal_dt)
              << ", rotation_deg=" << number(uobs_terminal_dr) << ")";
      throw std::runtime_error(message.str());
    }
    if (uobs.eigenvalues.minCoeff() <= 0.0 || !uobs.eigenvalues.allFinite() ||
        !uobs.eigenvectors.allFinite() ||
        (uobs.eigenvectors.transpose() * uobs.eigenvectors - Matrix6d::Identity()).norm() > 1e-3)
      throw std::runtime_error("invalid archived U_obs eigensystem");

    Cloud::Ptr source;
    auto source_it = source_cache.find(tx);
    if (source_it == source_cache.end()) {
      Cloud::Ptr raw = loadPackedSource(frame.path);
      source = preprocessSource(raw);
      if (source->size() != frame.expected_points || sourceHash(*source) != frame.expected_hash)
        throw std::runtime_error("profile source provenance mismatch tx=" + std::to_string(tx));
      source_cache.emplace(tx, source);
    } else source = source_it->second;
    ndt.setInputSource(source);

    const double weak_ratio = uobs.eigenvalues(1) / uobs.eigenvalues(0);
    const int k = weak_ratio >= 2.0 ? 1 : 2;
    if (k == 1) ++k1_frames; else ++k2_frames;
    Eigen::MatrixXd weak_basis(6, k), strong_basis(6, 6-k);
    for (int i = 0; i < k; ++i) weak_basis.col(i) = uobs.eigenvectors.col(i);
    for (int i = k; i < 6; ++i) strong_basis.col(i-k) = uobs.eigenvectors.col(i);

    // No prediction covariance exists in the frozen archive. Use only the
    // explicitly authorized finite fallback: <=2 m translation, <=15 deg rotation.
    const double rotation_limit = 15.0 * kPi / 180.0;
    Eigen::VectorXd bounds(k);
    for (int i = 0; i < k; ++i) {
      const Eigen::VectorXd q = weak_basis.col(i);
      const double translation_norm = q.head<3>().norm();
      const double rotation_norm = q.tail<3>().norm();
      const double translation_bound = translation_norm > 1e-12
          ? 2.0 / (kResolution * translation_norm) : std::numeric_limits<double>::infinity();
      const double rotation_bound = rotation_norm > 1e-12
          ? rotation_limit / rotation_norm : std::numeric_limits<double>::infinity();
      bounds(i) = std::min(translation_bound, rotation_bound);
      if (!std::isfinite(bounds(i)) || bounds(i) <= 0.0)
        throw std::runtime_error("invalid weak-subspace fallback bound");
    }
    const Eigen::Vector3d base_euler = uobs.pose.block<3, 3>(0, 0).eulerAngles(0,1,2).cast<double>();
    DynamicEnergy base_energy = evaluateDynamicEnergy(ndt, source, uobs.pose);
    const Support base_support = base_energy.support;
    int frame_energy_evals = 1;
    std::vector<ProfileNode> nodes;
    const int grid_n = (k == 1) ? 17 : 9;
    std::vector<int> grid(static_cast<std::size_t>(grid_n * (k == 2 ? grid_n : 1)), -1);
    auto gridIndex = [grid_n, k](int i, int j) {
      return static_cast<std::size_t>(i * (k == 2 ? grid_n : 1) + (k == 2 ? j : 0));
    };
    for (int i = 0; i < grid_n; ++i) {
      const double u0 = -bounds(0) + 2.0 * bounds(0) * i / (grid_n - 1);
      const int j_count = (k == 2) ? grid_n : 1;
      for (int j = 0; j < j_count; ++j) {
        const double u1 = (k == 2) ? (-bounds(1) + 2.0 * bounds(1) * j / (grid_n - 1)) : 0.0;
        Eigen::VectorXd coordinates(k);
        coordinates(0) = u0;
        if (k == 2) coordinates(1) = u1;
        const Vector6d weak_eta = weak_basis * coordinates;
        if (kResolution * weak_eta.head<3>().norm() > 2.0 + 1e-9 ||
            weak_eta.tail<3>().norm() > rotation_limit + 1e-9) continue;
        ProfileNode node = profileAt(ndt, source, uobs.pose, base_euler,
            weak_basis, strong_basis, coordinates, base_support);
        node.i = i; node.j = j;
        frame_energy_evals += node.energy_evaluations;
        const int node_index = static_cast<int>(nodes.size());
        grid[gridIndex(i,j)] = node_index;
        nodes.push_back(node);
      }
    }

    std::vector<ProfileMinimum> minima;
    for (int i = 1; i < grid_n-1; ++i) {
      const int j_begin = (k == 2) ? 1 : 0;
      const int j_end = (k == 2) ? grid_n-1 : 0;
      for (int j = j_begin; j <= j_end; ++j) {
        const int idx = grid[gridIndex(i,j)];
        if (idx < 0 || !nodes[static_cast<std::size_t>(idx)].valid) continue;
        const ProfileNode& center = nodes[static_cast<std::size_t>(idx)];
        bool minimum = true, strict_neighbor = false, complete_neighborhood = true;
        for (int di = -1; di <= 1; ++di) for (int dj = (k == 2 ? -1 : 0); dj <= (k == 2 ? 1 : 0); ++dj) {
          if (di == 0 && dj == 0) continue;
          const int neighbor_i = i + di, neighbor_j = j + dj;
          if (neighbor_i < 0 || neighbor_i >= grid_n || neighbor_j < 0 || neighbor_j >= (k == 2 ? grid_n : 1)) {
            complete_neighborhood = false; continue;
          }
          const int neighbor_idx = grid[gridIndex(neighbor_i,neighbor_j)];
          if (neighbor_idx < 0 || !nodes[static_cast<std::size_t>(neighbor_idx)].valid) {
            complete_neighborhood = false; continue;
          }
          const double neighbor_energy = nodes[static_cast<std::size_t>(neighbor_idx)].energy;
          if (neighbor_energy < center.energy - 1e-9) minimum = false;
          if (neighbor_energy > center.energy + 1e-9) strict_neighbor = true;
        }
        if (minimum && strict_neighbor && complete_neighborhood)
          minima.push_back({idx, center});
      }
    }
    std::sort(minima.begin(), minima.end(), [](const ProfileMinimum& a, const ProfileMinimum& b) {
      return a.node.energy < b.node.energy;
    });
    std::vector<ProfileMinimum> distinct_minima;
    const double grid_delta0 = 2.0 * bounds(0) / (grid_n - 1);
    const double grid_delta1 = (k == 2) ? 2.0 * bounds(1) / (grid_n - 1) : 0.0;
    for (const ProfileMinimum& candidate : minima) {
      bool duplicate = false;
      for (const ProfileMinimum& existing : distinct_minima) {
        const double d0 = (candidate.node.u0 - existing.node.u0) / grid_delta0;
        const double d1 = (k == 2) ? (candidate.node.u1 - existing.node.u1) / grid_delta1 : 0.0;
        if (std::sqrt(d0*d0+d1*d1) < 0.75) { duplicate = true; break; }
      }
      if (!duplicate) distinct_minima.push_back(candidate);
    }
    minima.swap(distinct_minima);

    std::vector<SearchTerminal> found_terminals;
    SearchTerminal nominal;
    nominal.label = "NOMINAL_T0";
    nominal.pose = uobs.pose;
    nominal.score = base_energy.score;
    nominal.energy = base_energy.mean_energy;
    nominal.converged = true;
    found_terminals.push_back(nominal);
    int frame_full_refinements = 0;
    for (std::size_t m = 0; m < minima.size(); ++m) {
      const ProfileNode& minimum = minima[m].node;
      const Eigen::Matrix4f initial = poseAtEta(uobs.pose, minimum.eta);
      Cloud aligned;
      const auto align_start = std::chrono::steady_clock::now();
      ndt.align(aligned, initial);
      const double runtime_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - align_start).count();
      const Eigen::Matrix4f refined = ndt.getFinalTransformation();
      DynamicEnergy terminal_energy = evaluateDynamicEnergy(ndt, source, refined);
      ++frame_energy_evals;
      SearchTerminal terminal;
      terminal.label = "PROFILE_MIN_" + std::to_string(m);
      terminal.pose = refined;
      terminal.score = terminal_energy.score;
      terminal.energy = terminal_energy.mean_energy;
      terminal.converged = ndt.hasConverged();
      terminal.iterations = ndt.getFinalNumIteration();
      terminal.runtime_ms = runtime_ms;
      found_terminals.push_back(terminal);
      ++frame_full_refinements;
    }

    std::vector<std::size_t> terminal_order(found_terminals.size());
    std::iota(terminal_order.begin(), terminal_order.end(), 0);
    std::sort(terminal_order.begin(), terminal_order.end(), [&](std::size_t a, std::size_t b) {
      return found_terminals[a].energy < found_terminals[b].energy;
    });
    struct ClusterMembers { std::vector<std::size_t> terminals; };
    std::vector<ClusterMembers> terminal_clusters;
    for (const std::size_t item : terminal_order) {
      bool placed = false;
      for (std::size_t c = 0; c < terminal_clusters.size() && !placed; ++c) {
        bool complete_link = true;
        for (const std::size_t member : terminal_clusters[c].terminals)
          if (!sameBasin(found_terminals[item].pose, found_terminals[member].pose)) {
            complete_link = false; break;
          }
        if (complete_link) {
          terminal_clusters[c].terminals.push_back(item);
          found_terminals[item].cluster_id = static_cast<int>(c);
          placed = true;
        }
      }
      if (!placed) {
        terminal_clusters.push_back({{item}});
        found_terminals[item].cluster_id = static_cast<int>(terminal_clusters.size()-1);
      }
    }
    for (const SearchTerminal& terminal : found_terminals) {
      terminals_out << tx << ',' << frame.frame_id << ',' << terminal.label << ','
          << (terminal.converged ? 1 : 0) << ',' << terminal.iterations << ','
          << number(terminal.runtime_ms) << ',' << number(terminal.score) << ','
          << number(terminal.energy) << ',' << terminal.cluster_id << ',';
      for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
        if (r || c) terminals_out << ';';
        terminals_out << number(terminal.pose(r,c));
      }
      terminals_out << '\n';
    }
    for (std::size_t i = 0; i < terminal_clusters.size(); ++i) {
      const std::size_t rep = terminal_clusters[i].terminals.front();
      for (std::size_t j = i+1; j < terminal_clusters.size(); ++j) {
        const std::size_t other = terminal_clusters[j].terminals.front();
        Eigen::Matrix<double,6,1> log;
        double dt=0, dr=0;
        se3Log(found_terminals[rep].pose, found_terminals[other].pose, &log, &dt, &dr);
        relations << tx << ',' << frame.frame_id << ',' << found_terminals[rep].label << ','
            << found_terminals[other].label << ',';
        for (int d=0; d<6; ++d) { if (d) relations << ';'; relations << number(log(d)); }
        relations << ',' << number(dt) << ',' << number(dr) << '\n';
      }
    }

    const auto oracle_it = oracle.find(tx);
    const std::vector<OracleBasin>& basins = oracle_it->second;
    double best_oracle_score = -std::numeric_limits<double>::infinity();
    std::size_t primary_total = 0;
    for (const OracleBasin& basin : basins) best_oracle_score = std::max(best_oracle_score, basin.score);
    const CsvTable cluster_table = readCsv(cluster_path);
    for (const auto& row : cluster_table.rows)
      if (parseU64(cluster_table.get(row, "transaction_id")) == tx &&
          cluster_table.get(row, "threshold_set") == "primary") ++primary_total;
    const double energy_gap_limit = 0.05 * std::max(1.0,
        std::abs(best_oracle_score / static_cast<double>(source->size())));
    std::vector<std::string> major_ids, missed_ids;
    std::vector<double> projection_fractions;
    int recovered_major = 0;
    int nominal_cluster_id = -1;
    for (const SearchTerminal& terminal : found_terminals)
      if (terminal.label == "NOMINAL_T0") nominal_cluster_id = terminal.cluster_id;
    if (nominal_cluster_id < 0)
      throw std::runtime_error("reduced terminal set has no nominal cluster");
    for (const OracleBasin& basin : basins) {
      const bool separated = translationDistance(uobs.pose, basin.pose) > 0.2 ||
                             rotationDistanceDeg(uobs.pose, basin.pose) > 2.0;
      const double gap = (best_oracle_score - basin.score) / static_cast<double>(source->size());
      if (!separated || gap > energy_gap_limit) continue;
      major_ids.push_back(basin.cluster_id);
      bool recovered_basin = false;
      for (const SearchTerminal& terminal : found_terminals)
        if (terminal.cluster_id != nominal_cluster_id &&
            sameBasin(terminal.pose, basin.pose)) { recovered_basin = true; break; }
      if (recovered_basin) ++recovered_major;
      else missed_ids.push_back(basin.cluster_id);
      const Vector6d displacement = mapChartDisplacement(uobs.pose, basin.pose);
      const double norm = displacement.norm();
      if (norm > 1e-12)
        projection_fractions.push_back((weak_basis * (weak_basis.transpose() * displacement)).norm() / norm);
    }
    double mean_projection = std::numeric_limits<double>::quiet_NaN();
    if (!projection_fractions.empty())
      mean_projection = std::accumulate(projection_fractions.begin(), projection_fractions.end(), 0.0) /
                        static_cast<double>(projection_fractions.size());
    bool any_alternative = terminal_clusters.size() > 1;
    // Post-hoc control stratum: the 20 archived frames with fewest supported
    // oracle clusters. This is an evaluation label, not an online search input.
    const bool nominal_control = low_cluster_control.count(tx) != 0;
    if (nominal_control) { ++nominal_control_frames; if (any_alternative) ++nominal_control_with_competing_terminal; }
    const std::string missed = missed_ids.empty() ? "NONE" : [&missed_ids]() {
      std::ostringstream value;
      for (std::size_t i=0;i<missed_ids.size();++i) { if (i) value << ';'; value << missed_ids[i]; }
      return value.str();
    }();
    recovery << tx << ',' << frame.frame_id << ',' << basins.size() << ',' << primary_total << ','
        << major_ids.size() << ',' << recovered_major << ',' << missed << ',' << terminal_clusters.size()
        << ',' << minima.size() << ',' << frame_full_refinements << ',' << frame_energy_evals << ','
        << k << ',' << number(mean_projection) << ",COMPLETED\n";
    for (const ProfileNode& node : nodes) {
      landscape << tx << ',' << frame.frame_id << ',' << k << ',' << node.i << ',' << node.j << ','
          << number(node.u0) << ',' << number(node.u1) << ',' << number(node.energy) << ','
          << number(node.support_changed_fraction) << ',' << node.strong_iterations << ','
          << node.energy_evaluations << ',' << node.status << ',';
      for (int i=0;i<6;++i) { if (i) landscape << ';'; landscape << number(node.eta(i)); }
      landscape << '\n';
    }
    const double lambda_ratio = uobs.eigenvalues(1) / uobs.eigenvalues(0);
    const double frame_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - frame_start).count();
    summary << tx << ',' << frame.frame_id << ',' << k << ',' << number(uobs.eigenvalues(0)) << ','
        << number(uobs.eigenvalues(1)) << ',' << number(uobs.eigenvalues(2)) << ','
        << number(lambda_ratio) << ",1," << number(bounds(0)) << ','
        << (k==2 ? number(bounds(1)) : "NA") << ',' << nodes.size() << ',' << minima.size() << ','
        << frame_full_refinements << ',' << terminal_clusters.size() << ',' << frame_energy_evals << ','
        << number(frame_ms) << ",COMPLETED\n";
    total_profile_evals += static_cast<std::size_t>(frame_energy_evals);
    total_full_refines += static_cast<std::size_t>(frame_full_refinements);
    total_minima += minima.size();
    ++processed_frames;
    std::cout << "PROFILE_TX=" << tx << " K=" << k << " GRID=" << nodes.size()
              << " MINIMA=" << minima.size() << " FULL_REFINES=" << frame_full_refinements
              << " ORACLE_MAJOR=" << major_ids.size() << " RECOVERED=" << recovered_major
              << " MISSED=" << missed_ids.size() << std::endl;
  }
  const double total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - total_start).count();
  std::cout << "PROFILE_FRAMES=" << processed_frames << " K1=" << k1_frames << " K2=" << k2_frames
            << " PROFILE_ENERGY_EVALUATIONS=" << total_profile_evals
            << " FULL_NDT_REFINEMENTS=" << total_full_refines << " PROFILE_MINIMA=" << total_minima
            << " MEAN_PROFILE_EVALS=" << (processed_frames ?
                static_cast<double>(total_profile_evals) / processed_frames : 0.0)
            << " MEAN_FULL_REFINES=" << (processed_frames ?
                static_cast<double>(total_full_refines) / processed_frames : 0.0)
            << " NOMINAL_CONTROL_FRAMES=" << nominal_control_frames
            << " CONTROL_WITH_ALTERNATIVE_TERMINAL=" << nominal_control_with_competing_terminal
            << " TOTAL_RUNTIME_MS=" << total_ms << std::endl;
}
}  // namespace

#ifndef P9_NDT_ENERGY_CONTRACT_LIBRARY
int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      if (!runMathSelfTests()) throw std::runtime_error("robust SE(3) distance self-test failed");
      std::cout << "P9_MATH_SELF_TEST=PASS" << std::endl;
      return 0;
    }
    if (argc == 9 && std::string(argv[1]) == "--profile") {
      runProfileSearch(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7], argv[8]);
      return 0;
    }
    if (argc != 7) {
      std::cerr << "usage: p9_ndt_energy_contract --self-test\n"
                   "   or: p9_ndt_energy_contract MAP.pcd cohort_frozen.csv candidates.csv "
                   "mode_clusters.csv tx1;tx2;... output_dir\n"
                   "   or: p9_ndt_energy_contract --profile MAP.pcd cohort_frozen.csv "
                   "candidates.csv mode_clusters.csv dual_u.csv ALL|tx1;tx2;... output_dir\n";
      return 2;
    }
    validateFrames(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P9_NDT_ENERGY_CONTRACT_ERROR: " << error.what() << '\n';
    return 1;
  }
}
#endif
