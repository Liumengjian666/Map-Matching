#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
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
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;
using LeafPtr = pcl::VoxelGridCovariance<Point>::LeafConstPtr;
using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;
constexpr double kResolutionM = 0.8;
constexpr double kOutlierRatio = 0.35;  // PCL 1.10 NDT default.
constexpr double kMinRangeM = 0.5;
constexpr double kMaxRangeM = 80.0;
constexpr double kPi = 3.14159265358979323846;

struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, std::size_t> column;
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
  if (quoted) throw std::runtime_error("unterminated quoted CSV field");
  fields.push_back(field);
  return fields;
}

CsvTable readCsv(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open CSV: " + path);
  std::string line;
  if (!std::getline(stream, line)) throw std::runtime_error("empty CSV: " + path);
  CsvTable table;
  table.header = parseCsvLine(line);
  for (std::size_t i = 0; i < table.header.size(); ++i)
    table.column.emplace(table.header[i], i);
  while (std::getline(stream, line)) {
    if (line.empty()) continue;
    auto row = parseCsvLine(line);
    if (row.size() != table.header.size())
      throw std::runtime_error("CSV field count mismatch in " + path);
    table.rows.push_back(std::move(row));
  }
  return table;
}

const std::string& field(const CsvTable& table,
                         const std::vector<std::string>& row,
                         const std::string& name) {
  const auto it = table.column.find(name);
  if (it == table.column.end()) throw std::runtime_error("missing CSV field: " + name);
  return row.at(it->second);
}

std::vector<std::string> split(const std::string& text, char separator) {
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string value;
  while (std::getline(stream, value, separator)) values.push_back(value);
  return values;
}

void finalize(const Cloud::Ptr& cloud) {
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf, double z_leaf, int cap) {
  Cloud::Ptr down(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                    static_cast<float>(z_leaf));
  voxel.setInputCloud(input);
  voxel.filter(*down);
  if (cap > 0 && static_cast<int>(down->size()) > cap) {
    Cloud::Ptr sampled(new Cloud);
    sampled->reserve(static_cast<std::size_t>(cap));
    const double increment = static_cast<double>(down->size() - 1) /
                            static_cast<double>(std::max(cap - 1, 1));
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
    throw std::runtime_error("cannot load map: " + path);
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& p : raw->points)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      finite->push_back(p);
  finalize(finite);
  return voxelDown(voxelDown(finite, 0.15, 0.15, 0), 0.15, 0.15, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const Point& p : raw->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                   static_cast<double>(p.y) * p.y +
                                   static_cast<double>(p.z) * p.z);
    if (range < kMinRangeM || range > kMaxRangeM) continue;
    filtered->push_back(p);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25, 0.25, 1400);
}

std::uint64_t sourceCloudHash(const Cloud::Ptr& cloud) {
  constexpr std::uint64_t kOffset = 1469598103934665603ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t hash = kOffset;
  auto mix = [&hash](std::uint8_t byte) { hash = (hash ^ byte) * kPrime; };
  auto mixU32 = [&mix](std::uint32_t v) {
    for (int shift = 0; shift < 32; shift += 8)
      mix(static_cast<std::uint8_t>(v >> shift));
  };
  mixU32(cloud->width);
  mixU32(cloud->height);
  mixU32(cloud->is_dense ? 1U : 0U);
  mixU32(static_cast<std::uint32_t>(cloud->size()));
  for (const Point& p : cloud->points) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &p.x, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &p.y, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &p.z, sizeof(bits)); mixU32(bits);
  }
  return hash;
}

Eigen::Matrix4f parsePose(const std::string& value) {
  const auto v = split(value, ';');
  if (v.size() != 7) throw std::runtime_error("expected xyz + xyzw pose");
  Eigen::Quaternionf q(std::stof(v[6]), std::stof(v[3]),
                       std::stof(v[4]), std::stof(v[5]));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f)
    throw std::runtime_error("invalid input quaternion");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3f(std::stof(v[0]),
                                           std::stof(v[1]),
                                           std::stof(v[2]));
  return pose;
}

Eigen::Matrix<double, 6, 1> pclParameterVector(const Eigen::Matrix4f& pose) {
  Eigen::Transform<float, 3, Eigen::Affine> transform;
  transform.matrix() = pose;
  const Eigen::Vector3f angles = transform.rotation().eulerAngles(0, 1, 2);
  Eigen::Matrix<double, 6, 1> p;
  p << pose(0, 3), pose(1, 3), pose(2, 3), angles.x(), angles.y(), angles.z();
  return p;
}

std::string poseString(const Eigen::Matrix4f& pose) {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();
  std::ostringstream out;
  out << std::setprecision(10) << pose(0, 3) << ';' << pose(1, 3) << ';'
      << pose(2, 3) << ';' << q.x() << ';' << q.y() << ';' << q.z() << ';' << q.w();
  return out.str();
}

std::string matrixString(const Eigen::Matrix4f& pose) {
  std::ostringstream out;
  out << std::setprecision(10);
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) {
      if (row != 0 || col != 0) out << ';';
      out << pose(row, col);
    }
  return out.str();
}

double logAddExp(double a, double b) {
  const double maximum = std::max(a, b);
  return maximum + std::log(std::exp(a - maximum) + std::exp(b - maximum));
}

struct CandidateEvidence {
  double park_log_sum = 0.0;
  double support_marginal_log_sum = 0.0;
  double neighbor_count_sum = 0.0;
  std::uint64_t supported_returns = 0;
  std::uint64_t source_returns = 0;
};

class AuditedNdt : public Ndt {
 public:
  double fixedPclScore(const Cloud::Ptr& source, const Eigen::Matrix4f& pose) {
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    Eigen::Matrix<double, 6, 1> p = pclParameterVector(pose);
    Eigen::Matrix<double, 6, 1> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    return computeDerivatives(gradient, hessian, transformed, p, true);
  }

  Eigen::Vector3f actualGridLeafSize() const { return target_cells_.getLeafSize(); }

  CandidateEvidence scoreProbabilistic(const Cloud::Ptr& source,
                                      const Eigen::Matrix4f& pose) {
    CandidateEvidence out;
    out.source_returns = source->size();
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, pose);
    const double log_inlier_weight = std::log1p(-kOutlierRatio);
    const double shell_volume = (4.0 / 3.0) * kPi *
        (std::pow(kMaxRangeM, 3) - std::pow(kMinRangeM, 3));
    const double log_outlier = std::log(kOutlierRatio) - std::log(shell_volume);
    std::vector<LeafPtr> neighborhood;
    std::vector<float> distances;
    neighborhood.reserve(32);
    distances.reserve(32);

    for (const Point& point : transformed.points) {
      neighborhood.clear();
      distances.clear();
      target_cells_.radiusSearch(point, resolution_, neighborhood, distances);
      out.neighbor_count_sum += static_cast<double>(neighborhood.size());
      if (!neighborhood.empty()) ++out.supported_returns;

      double maximum_log_phi = -std::numeric_limits<double>::infinity();
      std::vector<double> log_phi;
      log_phi.reserve(neighborhood.size());
      const Vector3d y(point.x, point.y, point.z);
      for (const LeafPtr& leaf : neighborhood) {
        const Matrix3d inverse_covariance = leaf->getInverseCov();
        const Matrix3d covariance = leaf->getCov();
        const Eigen::LLT<Matrix3d> llt(covariance);
        if (llt.info() != Eigen::Success)
          throw std::runtime_error("target voxel covariance is not positive definite");
        const double log_det = 2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
        const Vector3d delta = y - leaf->getMean();
        const double mahalanobis = delta.dot(inverse_covariance * delta);
        const double value = -0.5 * (3.0 * std::log(2.0 * kPi) + log_det + mahalanobis);
        log_phi.push_back(value);
        maximum_log_phi = std::max(maximum_log_phi, value);
      }

      if (log_phi.empty()) {
        out.park_log_sum += log_outlier;
        out.support_marginal_log_sum += log_outlier;
        continue;
      }

      // Park/Biber-style probabilistic baseline adaptation: one best local
      // NDT voxel plus a uniform outlier component. This is not an exact
      // reproduction: PCL's fixed 0.35 outlier ratio and this scan-range
      // uniform support are used because Park et al. do not publish their
      // implementation constants for this 3-D PCL candidate-set setting.
      const double park_log_likelihood = logAddExp(
          log_inlier_weight + maximum_log_phi, log_outlier);
      out.park_log_sum += park_log_likelihood;

      // Prototype rule: marginalize the same PCL radius-search association
      // set as an equally weighted, normalized local Gaussian mixture. Unlike
      // PCL's optimizer score, adding duplicate neighbor cells does not add
      // score mass merely by increasing neighborhood cardinality.
      double scaled_sum = 0.0;
      for (const double value : log_phi)
        scaled_sum += std::exp(value - maximum_log_phi);
      const double mixture_log_phi = maximum_log_phi +
          std::log(scaled_sum) - std::log(static_cast<double>(log_phi.size()));
      const double mixture_log_likelihood = logAddExp(
          log_inlier_weight + mixture_log_phi, log_outlier);
      out.support_marginal_log_sum += mixture_log_likelihood;
    }
    return out;
  }
};

struct FrameInfo {
  std::string id;
  std::string transaction_id;
  std::string time_s;
  std::string bag_sha;
  std::string map_sha;
  std::uint64_t source_hash = 0;
};

struct SeedInfo {
  std::string frame_id;
  std::string transaction_id;
  std::string time_s;
  std::string seed_index;
  std::string seed_domain;
  std::string source_hash;
  std::string bag_sha;
  std::string map_sha;
  std::string start_pose;
};

int runSelfTest() {
  const std::vector<double> log_terms{-2.0, -3.0, -1.0};
  const auto log_sum_exp = [](const std::vector<double>& values) {
    const double maximum = *std::max_element(values.begin(), values.end());
    double sum = 0.0;
    for (double value : values) sum += std::exp(value - maximum);
    return maximum + std::log(sum);
  };
  const double raw_one = log_sum_exp({log_terms[0]});
  const double raw_duplicate = log_sum_exp({log_terms[0], log_terms[0]});
  const double normalized_one = raw_one;
  const double normalized_duplicate = raw_duplicate - std::log(2.0);
  const double raw_shift = raw_duplicate - raw_one;
  const double normalized_error = std::abs(normalized_duplicate - normalized_one);
  std::cout << std::setprecision(12)
            << "DUPLICATE_CELL_RAW_LOG_SCORE_SHIFT=" << raw_shift << '\n'
            << "DUPLICATE_CELL_NORMALIZED_MIXTURE_ERROR=" << normalized_error << '\n';
  if (std::abs(raw_shift - std::log(2.0)) > 1e-12 || normalized_error > 1e-12)
    return 1;
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") return runSelfTest();
    if (argc != 6 && argc != 7) {
      std::cerr << "usage: dual_u_evidence_scan MAP.pcd MANIFEST.csv SEED_RUNS.csv CLOUD_DIR OUTPUT.csv [MAX_FRAMES]\n";
      return 2;
    }
    const std::string map_path = argv[1];
    const std::string manifest_path = argv[2];
    const std::string seed_path = argv[3];
    const std::string cloud_dir = argv[4];
    const std::string output_path = argv[5];
    const std::size_t max_frames = argc == 7 ? std::stoul(argv[6]) : 0;
    const CsvTable manifest = readCsv(manifest_path);
    const CsvTable seed_table = readCsv(seed_path);
    std::map<std::string, FrameInfo> frames;
    std::map<std::string, std::vector<SeedInfo>> seeds;
    for (const auto& row : manifest.rows) {
      FrameInfo frame;
      frame.id = field(manifest, row, "frame_id");
      frame.transaction_id = field(manifest, row, "transaction_id");
      frame.time_s = field(manifest, row, "time_s");
      frame.bag_sha = field(manifest, row, "input_bag_sha256");
      frame.map_sha = field(manifest, row, "input_map_sha256");
      frame.source_hash = std::stoull(field(manifest, row, "ndt_source_cloud_hash"));
      if (!frames.emplace(frame.id, frame).second)
        throw std::runtime_error("duplicate frame in frozen manifest: " + frame.id);
    }
    for (const auto& row : seed_table.rows) {
      SeedInfo seed;
      seed.frame_id = field(seed_table, row, "frame_id");
      seed.transaction_id = field(seed_table, row, "transaction_id");
      seed.time_s = field(seed_table, row, "time_s");
      seed.seed_index = field(seed_table, row, "seed_index");
      seed.seed_domain = field(seed_table, row, "seed_domain");
      seed.source_hash = field(seed_table, row, "source_hash_actual");
      seed.bag_sha = field(seed_table, row, "input_bag_sha256");
      seed.map_sha = field(seed_table, row, "input_map_sha256");
      seed.start_pose = field(seed_table, row, "start_pose_xyz_q_xyzw");
      if (frames.find(seed.frame_id) == frames.end())
        throw std::runtime_error("seed references frame outside frozen manifest");
      const FrameInfo& frame = frames.at(seed.frame_id);
      if (seed.transaction_id != frame.transaction_id || seed.time_s != frame.time_s ||
          seed.bag_sha != frame.bag_sha || seed.map_sha != frame.map_sha ||
          std::stoull(seed.source_hash) != frame.source_hash)
        throw std::runtime_error("seed provenance mismatch for " + seed.frame_id);
      seeds[seed.frame_id].push_back(std::move(seed));
    }
    if (frames.size() != 32 || seed_table.rows.size() != 8800)
      throw std::runtime_error("frozen experiment contract mismatch: expected 32 frames / 8800 starts");

    const Cloud::Ptr target = loadTarget(map_path);
    AuditedNdt ndt;
    ndt.setResolution(static_cast<float>(kResolutionM));
    ndt.setStepSize(0.08);
    ndt.setTransformationEpsilon(0.001);
    ndt.setMaximumIterations(40);
    ndt.setOulierRatio(kOutlierRatio);
    ndt.setInputTarget(target);
    const Eigen::Vector3f grid_leaf = ndt.actualGridLeafSize();
    if ((grid_leaf.array() - static_cast<float>(kResolutionM)).abs().maxCoeff() > 1e-6f)
      throw std::runtime_error("PCL target grid leaf does not match configured NDT resolution");

    std::ofstream out(output_path);
    if (!out) throw std::runtime_error("cannot open output CSV: " + output_path);
    out << "frame_id,transaction_id,time_s,seed_index,seed_domain,source_count,"
           "start_pose_xyz_q_xyzw,final_pose_xyz_q_xyzw,final_pose_matrix16,converged,iterations,"
           "align_ms,score_ms,pcl_raw_score_sum,pcl_raw_score_per_source,"
           "pcl_alignment_probability_per_source,"
           "park_log_evidence_per_source,support_marginal_log_evidence_per_source,"
           "mean_radius_neighbors,supported_return_fraction,"
           "configured_resolution_m,target_grid_leaf_x_m,target_grid_leaf_y_m,target_grid_leaf_z_m\n";
    out << std::setprecision(15);
    std::size_t total = 0;
    std::size_t processed_frames = 0;
    const auto whole_start = std::chrono::steady_clock::now();
    for (const auto& frame_pair : frames) {
      if (max_frames > 0 && processed_frames >= max_frames) break;
      const FrameInfo& frame = frame_pair.second;
      const std::string source_path = cloud_dir + "/" + frame.id + ".pcd";
      Cloud::Ptr raw(new Cloud);
      if (pcl::io::loadPCDFile<Point>(source_path, *raw) != 0)
        throw std::runtime_error("cannot load source scan: " + source_path);
      const Cloud::Ptr source = preprocessSource(raw);
      const std::uint64_t hash = sourceCloudHash(source);
      if (hash != frame.source_hash)
        throw std::runtime_error("preprocessed source hash mismatch for " + frame.id);
      ndt.setInputSource(source);
      const auto found_seeds = seeds.find(frame.id);
      if (found_seeds == seeds.end()) throw std::runtime_error("no seeds for " + frame.id);

      for (const SeedInfo& seed : found_seeds->second) {
        const Eigen::Matrix4f start_pose = parsePose(seed.start_pose);
        Cloud aligned;
        const auto align_start = std::chrono::steady_clock::now();
        ndt.align(aligned, start_pose);
        const double align_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - align_start).count();
        const Eigen::Matrix4f final_pose = ndt.getFinalTransformation();
        const bool converged = ndt.hasConverged();
        double pcl_score = std::numeric_limits<double>::quiet_NaN();
        double pcl_alignment_probability = std::numeric_limits<double>::quiet_NaN();
        CandidateEvidence evidence;
        double score_ms = 0.0;
        if (converged) {
          const auto score_start = std::chrono::steady_clock::now();
          pcl_score = ndt.fixedPclScore(source, final_pose);
          pcl_alignment_probability = ndt.getTransformationProbability();
          evidence = ndt.scoreProbabilistic(source, final_pose);
          score_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - score_start).count();
          if (!std::isfinite(pcl_score) || !std::isfinite(evidence.park_log_sum) ||
              !std::isfinite(evidence.support_marginal_log_sum))
            throw std::runtime_error("nonfinite candidate evidence at " + frame.id);
        }
        const double source_count = static_cast<double>(source->size());
        out << frame.id << ',' << frame.transaction_id << ',' << frame.time_s << ','
            << seed.seed_index << ',' << seed.seed_domain << ',' << source->size() << ','
            << '"' << seed.start_pose << '"' << ',' << '"' << poseString(final_pose) << '"' << ','
            << '"' << matrixString(final_pose) << '"' << ','
            << static_cast<int>(converged) << ',' << ndt.getFinalNumIteration() << ','
            << align_ms << ',' << score_ms << ',' << pcl_score << ','
            << (pcl_score / source_count) << ','
            << pcl_alignment_probability << ','
            << (evidence.park_log_sum / source_count) << ','
            << (evidence.support_marginal_log_sum / source_count) << ','
            << (evidence.neighbor_count_sum / source_count) << ','
            << (static_cast<double>(evidence.supported_returns) / source_count) << ','
            << kResolutionM << ',' << grid_leaf.x() << ',' << grid_leaf.y() << ','
            << grid_leaf.z() << '\n';
        ++total;
      }
      std::cerr << "FRAME=" << frame.id << " starts=" << found_seeds->second.size()
                << " source_points=" << source->size() << " source_hash=" << hash << '\n';
      ++processed_frames;
    }
    out.close();
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - whole_start).count();
    std::cout << std::setprecision(9)
              << "FRAMES=" << processed_frames << '\n'
              << "STARTS=" << total << '\n'
              << "CONFIGURED_RESOLUTION_M=" << kResolutionM << '\n'
              << "TARGET_GRID_LEAF_M=" << grid_leaf.transpose() << '\n'
              << "OUTLIER_RATIO=" << ndt.getOulierRatio() << '\n'
              << "ELAPSED_SEC=" << elapsed << '\n'
              << "OUTPUT=" << output_path << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "DUAL_U_EVIDENCE_ERROR=" << e.what() << '\n';
    return 1;
  }
}
