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
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace
{
using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;

std::vector<std::string> splitCsv(const std::string &line)
{
  std::vector<std::string> fields;
  std::string field;
  std::stringstream stream(line);
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

std::string trim(std::string value)
{
  while (!value.empty() && (value.back() == '\r' || value.back() == ' ' || value.back() == '\t')) value.pop_back();
  size_t first = 0;
  while (first < value.size() && (value[first] == ' ' || value[first] == '\t')) ++first;
  return value.substr(first);
}

double number(const std::unordered_map<std::string, std::string> &row, const std::string &key)
{
  const auto it = row.find(key);
  if (it == row.end() || it->second.empty() || it->second == "nan")
    return std::numeric_limits<double>::quiet_NaN();
  return std::stod(it->second);
}

std::vector<std::unordered_map<std::string, std::string>> readCsv(const std::string &path)
{
  std::ifstream input(path);
  if (!input.is_open()) throw std::runtime_error("failed to open CSV: " + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty CSV: " + path);
  const auto header = splitCsv(line);
  std::vector<std::unordered_map<std::string, std::string>> rows;
  while (std::getline(input, line))
  {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != header.size()) throw std::runtime_error("CSV field count mismatch: " + path);
    std::unordered_map<std::string, std::string> row;
    for (size_t i = 0; i < header.size(); ++i) row.emplace(trim(header[i]), trim(fields[i]));
    rows.push_back(std::move(row));
  }
  return rows;
}

void finalizeCloud(const CloudPtr &cloud)
{
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

CloudPtr voxelDown(const CloudPtr &cloud, double leaf, double z_leaf, int max_points)
{
  CloudPtr down(new Cloud());
  if (leaf > 0.01)
  {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(z_leaf));
    voxel.setInputCloud(cloud);
    voxel.filter(*down);
  }
  else *down = *cloud;

  if (max_points > 0 && static_cast<int>(down->size()) > max_points)
  {
    CloudPtr sampled(new Cloud());
    sampled->reserve(static_cast<size_t>(max_points));
    const double step = static_cast<double>(down->size() - 1) /
        static_cast<double>(std::max(max_points - 1, 1));
    for (int i = 0; i < max_points; ++i)
      sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
    finalizeCloud(sampled);
    return sampled;
  }
  finalizeCloud(down);
  return down;
}

CloudPtr preprocess(const CloudPtr &raw)
{
  CloudPtr filtered(new Cloud());
  filtered->reserve(raw->size());
  for (const auto &point : raw->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(static_cast<double>(point.x) * point.x +
                                   static_cast<double>(point.y) * point.y +
                                   static_cast<double>(point.z) * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);
  }
  finalizeCloud(filtered);
  return voxelDown(filtered, 0.25, 0.25, 1400);
}

uint64_t stableCloudHash(const CloudPtr &cloud)
{
  constexpr uint64_t kOffset = 1469598103934665603ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  const auto mixByte = [&hash](uint8_t byte) { hash = (hash ^ byte) * kPrime; };
  const auto mixU32 = [&mixByte](uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8) mixByte(static_cast<uint8_t>((value >> shift) & 0xffU));
  };
  mixU32(cloud ? cloud->width : 0U);
  mixU32(cloud ? cloud->height : 0U);
  mixU32(cloud && cloud->is_dense ? 1U : 0U);
  mixU32(cloud ? static_cast<uint32_t>(cloud->size()) : 0U);
  if (!cloud) return hash;
  for (const auto &point : cloud->points)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &point.x, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &point.y, sizeof(bits)); mixU32(bits);
    std::memcpy(&bits, &point.z, sizeof(bits)); mixU32(bits);
  }
  return hash;
}

// The PCL 1.10 evaluator is protected. This subclass exposes only a diagnostic
// score-at-pose method; it never invokes registration or changes a pose.
class ProbeNdt : public pcl::NormalDistributionsTransform<PointT, PointT>
{
public:
  using Base = pcl::NormalDistributionsTransform<PointT, PointT>;
  using Base::setInputSource;
  using Base::setInputTarget;
  using Base::setResolution;

  double scoreAtPose(const Eigen::Matrix4f &pose)
  {
    if (!this->input_ || this->input_->empty()) return std::numeric_limits<double>::quiet_NaN();
    refreshGaussianConstants();
    this->point_gradient_.setZero();
    this->point_gradient_.block<3, 3>(0, 0).setIdentity();
    this->point_hessian_.setZero();
    Cloud transformed;
    pcl::transformPointCloud(*this->input_, transformed, pose);
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> affine;
    affine.matrix() = pose;
    const Eigen::Vector3f euler = affine.rotation().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, 6, 1> parameters;
    parameters << pose(0, 3), pose(1, 3), pose(2, 3), euler.x(), euler.y(), euler.z();
    Eigen::Matrix<double, 6, 1> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    return this->computeDerivatives(gradient, hessian, transformed, parameters, true);
  }

private:
  void refreshGaussianConstants()
  {
    const double resolution = this->getResolution();
    const double outlier_ratio = this->getOulierRatio();
    const double c1 = 10.0 * (1.0 - outlier_ratio);
    const double c2 = outlier_ratio / std::pow(resolution, 3.0);
    const double d3 = -std::log(c2);
    this->gauss_d1_ = -std::log(c1 + c2) - d3;
    this->gauss_d2_ = -2.0 * std::log((-std::log(c1 * std::exp(-0.5) + c2) - d3) / this->gauss_d1_);
  }
};

Eigen::Matrix4f poseFromRow(const std::unordered_map<std::string, std::string> &row)
{
  Eigen::Quaternionf q(static_cast<float>(number(row, "final_used_qw")),
                      static_cast<float>(number(row, "final_used_qx")),
                      static_cast<float>(number(row, "final_used_qy")),
                      static_cast<float>(number(row, "final_used_qz")));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f) throw std::runtime_error("invalid final_used quaternion");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose(0, 3) = static_cast<float>(number(row, "final_used_tx"));
  pose(1, 3) = static_cast<float>(number(row, "final_used_ty"));
  pose(2, 3) = static_cast<float>(number(row, "final_used_tz"));
  return pose;
}

struct EigenDirection
{
  double lambda = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3f vector = Eigen::Vector3f::Zero();
};

struct ProfileStats
{
  double sigma = std::numeric_limits<double>::quiet_NaN();
  double mean = std::numeric_limits<double>::quiet_NaN();
  double variance = std::numeric_limits<double>::quiet_NaN();
  double entropy = 0.0;
  double credible_width95 = std::numeric_limits<double>::quiet_NaN();
  double peak_offset = std::numeric_limits<double>::quiet_NaN();
  int peak_count = 0;
  double second_peak_ratio = 0.0;
  bool range_insufficient = false;
};

const std::array<double, 4> kAlphas{{1.0, 0.1, 0.01, 0.001}};
const char *kScoreSemantics =
    "pcl_ndt_alignment_probability_objective_relative_profile_higher_is_better";

EigenDirection readDirection(const std::unordered_map<std::string, std::string> &row,
                             const std::string &type, int index)
{
  EigenDirection result;
  result.lambda = number(row, "schur_" + type + "_lambda_" + std::to_string(index));
  result.vector.x() = static_cast<float>(number(row, "schur_" + type + "_v" + std::to_string(index) + "_x"));
  result.vector.y() = static_cast<float>(number(row, "schur_" + type + "_v" + std::to_string(index) + "_y"));
  result.vector.z() = static_cast<float>(number(row, "schur_" + type + "_v" + std::to_string(index) + "_z"));
  const float norm = result.vector.norm();
  if (!std::isfinite(result.lambda) || !result.vector.allFinite() || norm < 1e-5f)
    throw std::runtime_error("invalid Schur direction for " + type + " index " + std::to_string(index));
  result.vector /= norm;
  return result;
}

// Stage3A.8 uses only the weakest direction. Stage3A.3 runtime CSVs expose
// that direction as schur_*_weak_* but do not contain the complete v1/v2
// basis. Keep the fallback local to sparse mode so the original full probe
// contract is unchanged.
EigenDirection readWeakDirection(const std::unordered_map<std::string, std::string> &row,
                                 const std::string &type)
{
  EigenDirection result;
  result.lambda = number(row, "schur_" + type + "_lambda_0");
  result.vector.x() = static_cast<float>(number(row, "schur_" + type + "_v0_x"));
  result.vector.y() = static_cast<float>(number(row, "schur_" + type + "_v0_y"));
  result.vector.z() = static_cast<float>(number(row, "schur_" + type + "_v0_z"));
  if (!result.vector.allFinite() || result.vector.norm() < 1e-5f)
  {
    result.vector.x() = static_cast<float>(number(row, "schur_" + type + "_weak_x"));
    result.vector.y() = static_cast<float>(number(row, "schur_" + type + "_weak_y"));
    result.vector.z() = static_cast<float>(number(row, "schur_" + type + "_weak_z"));
  }
  const float norm = result.vector.norm();
  if (!std::isfinite(result.lambda) || !result.vector.allFinite() || norm < 1e-5f)
    throw std::runtime_error("invalid sparse Schur weak direction for " + type);
  result.vector /= norm;
  return result;
}

Eigen::Matrix4f perturbPose(const Eigen::Matrix4f &center, const EigenDirection &direction,
                            const std::string &type, double delta)
{
  Eigen::Matrix4f pose = center;
  if (type == "translation")
  {
    pose.block<3, 1>(0, 3) += static_cast<float>(delta) * direction.vector;
  }
  else
  {
    // Stage3A Schur directions are expressed in the world frame. Apply the
    // same left/world perturbation for the rotation profile.
    const Eigen::AngleAxisf rotation(static_cast<float>(delta * M_PI / 180.0), direction.vector);
    pose.block<3, 3>(0, 0) = rotation.toRotationMatrix() * center.block<3, 3>(0, 0);
  }
  return pose;
}

double quantile(const std::vector<double> &values, const std::vector<double> &weights, double q)
{
  double cumulative = 0.0;
  for (size_t i = 0; i < values.size(); ++i)
  {
    cumulative += weights[i];
    if (cumulative >= q) return values[i];
  }
  return values.empty() ? std::numeric_limits<double>::quiet_NaN() : values.back();
}

ProfileStats summarizeProfile(const std::vector<double> &deltas, const std::vector<double> &scores,
                              double alpha)
{
  ProfileStats result;
  if (deltas.empty() || deltas.size() != scores.size()) return result;
  double maximum_log = -std::numeric_limits<double>::infinity();
  std::vector<double> log_scores(scores.size());
  for (size_t i = 0; i < scores.size(); ++i)
  {
    const double safe_score = std::max(scores[i], std::numeric_limits<double>::min());
    log_scores[i] = std::log(safe_score);
    maximum_log = std::max(maximum_log, log_scores[i]);
  }
  std::vector<double> weights(scores.size());
  double weight_sum = 0.0;
  for (size_t i = 0; i < scores.size(); ++i)
  {
    weights[i] = std::exp(alpha * (log_scores[i] - maximum_log));
    weight_sum += weights[i];
  }
  if (!(weight_sum > 0.0) || !std::isfinite(weight_sum)) return result;
  for (double &weight : weights) weight /= weight_sum;
  result.mean = std::inner_product(deltas.begin(), deltas.end(), weights.begin(), 0.0);
  result.variance = 0.0;
  for (size_t i = 0; i < deltas.size(); ++i)
    result.variance += weights[i] * (deltas[i] - result.mean) * (deltas[i] - result.mean);
  result.sigma = std::sqrt(std::max(result.variance, 0.0));
  for (double weight : weights)
    if (weight > 0.0) result.entropy -= weight * std::log(weight);
  result.credible_width95 = quantile(deltas, weights, 0.975) - quantile(deltas, weights, 0.025);
  const auto max_it = std::max_element(scores.begin(), scores.end());
  const size_t max_index = static_cast<size_t>(std::distance(scores.begin(), max_it));
  result.peak_offset = deltas[max_index];
  const double peak = *max_it;
  std::vector<double> local_peaks;
  for (size_t i = 0; i < scores.size(); ++i)
  {
    const bool endpoint = (i == 0 || i + 1 == scores.size());
    const bool local_max = endpoint || (scores[i] >= scores[i - 1] && scores[i] >= scores[i + 1]);
    if (local_max && scores[i] >= 0.25 * peak) local_peaks.push_back(scores[i]);
  }
  std::sort(local_peaks.begin(), local_peaks.end(), std::greater<double>());
  result.peak_count = static_cast<int>(local_peaks.size());
  if (local_peaks.size() > 1 && peak > 0.0) result.second_peak_ratio = local_peaks[1] / peak;
  result.range_insufficient = scores.front() >= 0.95 * peak || scores.back() >= 0.95 * peak;
  return result;
}

void writeCsvValue(std::ofstream &output, double value)
{
  if (std::isfinite(value)) output << std::setprecision(17) << value;
  else output << "nan";
}

struct SparseAxisStats
{
  double k_small = std::numeric_limits<double>::quiet_NaN();
  double k_large = std::numeric_limits<double>::quiet_NaN();
  double asym_small = std::numeric_limits<double>::quiet_NaN();
  double asym_large = std::numeric_limits<double>::quiet_NaN();
  double center_violation_small = std::numeric_limits<double>::quiet_NaN();
  double center_violation_large = std::numeric_limits<double>::quiet_NaN();
  double scale_consistency = std::numeric_limits<double>::quiet_NaN();
  bool nonconcave_small = false;
  bool nonconcave_large = false;
};

double sparseCurvature(double log_minus, double log_center, double log_plus, double delta)
{
  return -(log_plus - 2.0 * log_center + log_minus) / (delta * delta);
}

SparseAxisStats makeSparseStats(double log_center, double log_minus_small, double log_plus_small,
                                double log_minus_large, double log_plus_large,
                                double small_delta, double large_delta)
{
  SparseAxisStats stats;
  stats.k_small = sparseCurvature(log_minus_small, log_center, log_plus_small, small_delta);
  stats.k_large = sparseCurvature(log_minus_large, log_center, log_plus_large, large_delta);
  stats.asym_small = std::abs(log_plus_small - log_minus_small);
  stats.asym_large = std::abs(log_plus_large - log_minus_large);
  stats.center_violation_small = std::max(log_plus_small, log_minus_small) - log_center;
  stats.center_violation_large = std::max(log_plus_large, log_minus_large) - log_center;
  stats.nonconcave_small = !std::isfinite(stats.k_small) || stats.k_small <= 0.0;
  stats.nonconcave_large = !std::isfinite(stats.k_large) || stats.k_large <= 0.0;
  if (!stats.nonconcave_small && !stats.nonconcave_large)
    stats.scale_consistency = std::abs(std::log(stats.k_small / stats.k_large));
  return stats;
}

void runSparseProbe(const std::string &frames_path, const std::string &map_path,
                    const std::string &output_dir)
{
  const auto frames = readCsv(frames_path);
  if (frames.empty()) throw std::runtime_error("selected frame CSV is empty");

  CloudPtr raw_map(new Cloud());
  if (pcl::io::loadPCDFile(map_path, *raw_map) != 0) throw std::runtime_error("failed to load map");
  CloudPtr finite_map(new Cloud());
  for (const auto &point : raw_map->points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite_map->push_back(point);
  finalizeCloud(finite_map);
  CloudPtr map_cloud = voxelDown(finite_map, 0.15, 0.15, 0);
  CloudPtr target_cloud = voxelDown(map_cloud, 0.15, 0.15, 0);
  if (target_cloud->size() != 459154U)
    throw std::runtime_error("target size mismatch: " + std::to_string(target_cloud->size()));

  ProbeNdt ndt;
  ndt.setResolution(0.8);
  ndt.setInputTarget(target_cloud);
  std::ofstream samples(output_dir + "/sparse_score_samples.csv");
  std::ofstream features(output_dir + "/sparse_features.csv");
  std::ofstream performance(output_dir + "/performance.csv");
  if (!samples.is_open() || !features.is_open() || !performance.is_open())
    throw std::runtime_error("failed to open Stage3A.8 output files");
  samples << "frame_index,target_rel,t_rel,sample_type,axis,offset,offset_unit,raw_ndt_score,log_score,score_semantics\n";
  features << "frame_index,target_rel,t_rel,translation_k_small,translation_k_large,translation_asym_small,"
      "translation_asym_large,translation_center_violation_small,translation_center_violation_large,"
      "translation_scale_consistency,translation_nonconcave_small,translation_nonconcave_large,"
      "rotation_k_small,rotation_k_large,rotation_asym_small,rotation_asym_large,"
      "rotation_center_violation_small,rotation_center_violation_large,rotation_scale_consistency,"
      "rotation_nonconcave_small,rotation_nonconcave_large,center_score,sparse_cell_count,sparse_eval_ms\n";
  performance << "frames,target_size,score_cells,mean_cell_ms,p95_cell_ms,max_cell_ms,"
      "mean_9cell_frame_ms,p95_9cell_frame_ms,max_9cell_frame_ms\n";

  std::vector<double> cell_ms;
  std::vector<double> frame_ms;
  for (const auto &row : frames)
  {
    CloudPtr raw(new Cloud());
    const std::string pcd_path = row.at("raw_pcd_path");
    if (pcl::io::loadPCDFile(pcd_path, *raw) != 0) throw std::runtime_error("failed to load scan: " + pcd_path);
    finalizeCloud(raw);
    CloudPtr source = preprocess(raw);
    std::ostringstream hash_stream;
    hash_stream << std::hex << std::setw(16) << std::setfill('0') << stableCloudHash(source);
    if (source->size() != static_cast<size_t>(number(row, "cloud_size_after_filter")) ||
        hash_stream.str() != row.at("cloud_hash"))
      throw std::runtime_error("runtime source preprocessing mismatch at frame " + row.at("frame_index"));
    ndt.setInputSource(source);
    const Eigen::Matrix4f pose = poseFromRow(row);
    const EigenDirection translation = readWeakDirection(row, "translation");
    const EigenDirection rotation = readWeakDirection(row, "rotation");
    std::vector<double> score_times;
    const auto evaluate = [&](const std::string &sample_type, const std::string &axis,
                              double offset, const std::string &unit, const EigenDirection *direction) {
      const auto start = std::chrono::steady_clock::now();
      const double score = direction == nullptr ? ndt.scoreAtPose(pose) :
          ndt.scoreAtPose(perturbPose(pose, *direction, axis == "rotation" ? "rotation" : "translation", offset));
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      if (!std::isfinite(score)) throw std::runtime_error("non-finite sparse PCL NDT score");
      score_times.push_back(elapsed);
      samples << row.at("frame_index") << ',' << row.at("target_rel") << ',' << row.at("t_rel") << ','
              << sample_type << ',' << axis << ',';
      writeCsvValue(samples, offset); samples << ',' << unit << ',';
      writeCsvValue(samples, score); samples << ',';
      writeCsvValue(samples, std::log(std::max(score, std::numeric_limits<double>::min())));
      samples << ',' << kScoreSemantics << '\n';
      return score;
    };
    const double center_score = evaluate("center", "all", 0.0, "none", nullptr);
    const double t_minus_small = evaluate("translation", "translation", -0.10, "m", &translation);
    const double t_plus_small = evaluate("translation", "translation", 0.10, "m", &translation);
    const double t_minus_large = evaluate("translation", "translation", -0.20, "m", &translation);
    const double t_plus_large = evaluate("translation", "translation", 0.20, "m", &translation);
    const double r_minus_small = evaluate("rotation", "rotation", -0.5, "deg", &rotation);
    const double r_plus_small = evaluate("rotation", "rotation", 0.5, "deg", &rotation);
    const double r_minus_large = evaluate("rotation", "rotation", -1.0, "deg", &rotation);
    const double r_plus_large = evaluate("rotation", "rotation", 1.0, "deg", &rotation);
    const auto log_score = [](double value) { return std::log(std::max(value, std::numeric_limits<double>::min())); };
    const double log_center = log_score(center_score);
    const SparseAxisStats ts = makeSparseStats(log_center, log_score(t_minus_small), log_score(t_plus_small),
                                                log_score(t_minus_large), log_score(t_plus_large), 0.10, 0.20);
    const double rad = M_PI / 180.0;
    const SparseAxisStats rs = makeSparseStats(log_center, log_score(r_minus_small), log_score(r_plus_small),
                                                log_score(r_minus_large), log_score(r_plus_large), 0.5 * rad, 1.0 * rad);
    const double sparse_eval_ms = std::accumulate(score_times.begin(), score_times.end(), 0.0);
    features << row.at("frame_index") << ',' << row.at("target_rel") << ',' << row.at("t_rel") << ',';
    const auto write_stats = [&features](const SparseAxisStats &s) {
      writeCsvValue(features, s.k_small); features << ','; writeCsvValue(features, s.k_large); features << ',';
      writeCsvValue(features, s.asym_small); features << ','; writeCsvValue(features, s.asym_large); features << ',';
      writeCsvValue(features, s.center_violation_small); features << ','; writeCsvValue(features, s.center_violation_large); features << ',';
      writeCsvValue(features, s.scale_consistency); features << ',' << (s.nonconcave_small ? 1 : 0) << ',' << (s.nonconcave_large ? 1 : 0) << ',';
    };
    write_stats(ts); write_stats(rs); writeCsvValue(features, center_score); features << ',' << score_times.size() << ',';
    writeCsvValue(features, sparse_eval_ms); features << '\n';
    cell_ms.insert(cell_ms.end(), score_times.begin(), score_times.end());
    frame_ms.push_back(sparse_eval_ms);
  }
  auto percentile = [](std::vector<double> values, double fraction) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const size_t index = std::min(values.size() - 1, static_cast<size_t>(std::ceil(fraction * values.size()) - 1.0));
    return values[index];
  };
  const auto mean = [](const std::vector<double> &values) {
    return values.empty() ? std::numeric_limits<double>::quiet_NaN() :
        std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  };
  performance << frames.size() << ',' << target_cloud->size() << ',' << cell_ms.size() << ','
              << mean(cell_ms) << ',' << percentile(cell_ms, 0.95) << ','
              << (cell_ms.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(cell_ms.begin(), cell_ms.end())) << ','
              << mean(frame_ms) << ',' << percentile(frame_ms, 0.95) << ','
              << (frame_ms.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(frame_ms.begin(), frame_ms.end())) << '\n';
  std::cout << "frames=" << frames.size() << " target_size=" << target_cloud->size()
            << " score_cells=" << cell_ms.size() << " output_dir=" << output_dir << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  try
  {
    if (argc < 4) throw std::runtime_error("usage: ndt_uncertainty_probe <selected_frames.csv> <map.pcd> <output_dir>");
    const std::string frames_path = argv[1];
    const std::string map_path = argv[2];
    const std::string output_dir = argv[3];
    if (argc >= 5 && std::string(argv[4]) == "sparse")
    {
      runSparseProbe(frames_path, map_path, output_dir);
      return 0;
    }
    const auto frames = readCsv(frames_path);
    if (frames.empty()) throw std::runtime_error("selected frame CSV is empty");

    const auto map_load_start = std::chrono::steady_clock::now();
    CloudPtr raw_map(new Cloud());
    if (pcl::io::loadPCDFile(map_path, *raw_map) != 0) throw std::runtime_error("failed to load map");
    const double map_load_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - map_load_start).count();
    CloudPtr finite_map(new Cloud());
    for (const auto &point : raw_map->points)
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite_map->push_back(point);
    finalizeCloud(finite_map);
    // Stage3A.3 effective parameter dump: map/voxel_size=0.15 and
    // lidar_update/ndt_target_voxel_size=0.15.
    const auto target_setup_start = std::chrono::steady_clock::now();
    CloudPtr map_cloud = voxelDown(finite_map, 0.15, 0.15, 0);
    CloudPtr target_cloud = voxelDown(map_cloud, 0.15, 0.15, 0);
    if (target_cloud->size() != 459154U)
      throw std::runtime_error("target size mismatch: " + std::to_string(target_cloud->size()));

    ProbeNdt ndt;
    ndt.setResolution(0.8);
    ndt.setInputTarget(target_cloud);
    const double target_setup_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - target_setup_start).count();
    std::ofstream profiles(output_dir + "/likelihood_profiles.csv");
    std::ofstream summaries(output_dir + "/uncertainty_summary.csv");
    std::ofstream comparison(output_dir + "/schur_uncertainty_comparison.csv");
    if (!profiles.is_open() || !summaries.is_open() || !comparison.is_open())
      throw std::runtime_error("failed to open Stage3A.5 output CSVs in: " + output_dir);
    profiles << "frame_index,target_rel,t_rel,profile_type,eigen_index,delta,delta_unit,raw_ndt_score,normalized_score,score_is_higher_better,score_semantics,log_score_profile\n";
    summaries << "frame_index,target_rel,t_rel,profile_type,eigen_index,alpha,lambda,lambda_norm,sigma,mean,variance,entropy,credible_width95,peak_offset,peak_count,second_peak_ratio,range_insufficient,score_at_center,score_max,eval_ms,score_semantics\n";
    comparison << "frame_index,target_rel,t_rel,profile_type,eigen_index,lambda,lambda_norm,alpha,sigma,credible_width95,entropy,peak_offset,peak_count,second_peak_ratio,range_insufficient,score_at_center,score_max\n";
    std::vector<double> cell_eval_ms;
    std::vector<double> frame_eval_ms;
    for (const auto &row : frames)
    {
      const auto frame_start = std::chrono::steady_clock::now();
      CloudPtr raw(new Cloud());
      const std::string pcd_path = row.at("raw_pcd_path");
      if (pcl::io::loadPCDFile(pcd_path, *raw) != 0) throw std::runtime_error("failed to load scan: " + pcd_path);
      finalizeCloud(raw);
      CloudPtr source = preprocess(raw);
      std::ostringstream hash_stream;
      hash_stream << std::hex << std::setw(16) << std::setfill('0') << stableCloudHash(source);
      if (source->size() != static_cast<size_t>(number(row, "cloud_size_after_filter")) ||
          hash_stream.str() != row.at("cloud_hash"))
        throw std::runtime_error("runtime source preprocessing mismatch at frame " + row.at("frame_index"));
      ndt.setInputSource(source);
      const Eigen::Matrix4f pose = poseFromRow(row);
      const std::string frame_index = row.at("frame_index");
      const std::string target_rel = row.at("target_rel");
      const std::string t_rel = row.at("t_rel");
      for (const std::string &type : {std::string("translation"), std::string("rotation")})
      {
        const double range = (type == "translation") ? 1.0 : 5.0;
        const double step = (type == "translation") ? 0.05 : 0.25;
        const std::string unit = (type == "translation") ? "m" : "deg";
        for (int eigen_index = 0; eigen_index < 3; ++eigen_index)
        {
          const EigenDirection direction = readDirection(row, type, eigen_index);
          std::vector<double> deltas;
          std::vector<double> scores;
          deltas.reserve(41);
          scores.reserve(41);
          const auto start = std::chrono::steady_clock::now();
          for (int sample = 0; sample < 41; ++sample)
          {
            const double delta = -range + step * static_cast<double>(sample);
            const double score = ndt.scoreAtPose(perturbPose(pose, direction, type, delta));
            if (!std::isfinite(score)) throw std::runtime_error("non-finite PCL NDT score");
            deltas.push_back(delta);
            scores.push_back(score);
            profiles << frame_index << ',' << target_rel << ',' << t_rel << ',' << type << ',' << eigen_index << ',';
            writeCsvValue(profiles, delta); profiles << ',' << unit << ',';
            writeCsvValue(profiles, score); profiles << ',';
            writeCsvValue(profiles, score / static_cast<double>(source->size()));
            profiles << ",1," << kScoreSemantics << ',';
            writeCsvValue(profiles, std::log(std::max(score, std::numeric_limits<double>::min())));
            profiles << '\n';
          }
          const double center_score = scores[20];
          const double max_score = *std::max_element(scores.begin(), scores.end());
          const double lambda_norm = direction.lambda /
              std::max(number(row, "schur_" + type + "_lambda_2"), std::numeric_limits<double>::min());
          const double elapsed_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start).count();
          cell_eval_ms.push_back(elapsed_ms / 41.0);
          for (double alpha : kAlphas)
          {
            const ProfileStats stats = summarizeProfile(deltas, scores, alpha);
            summaries << frame_index << ',' << target_rel << ',' << t_rel << ',' << type << ',' << eigen_index << ',';
            writeCsvValue(summaries, alpha); summaries << ',';
            writeCsvValue(summaries, direction.lambda); summaries << ',';
            writeCsvValue(summaries, lambda_norm); summaries << ',';
            writeCsvValue(summaries, stats.sigma); summaries << ',';
            writeCsvValue(summaries, stats.mean); summaries << ',';
            writeCsvValue(summaries, stats.variance); summaries << ',';
            writeCsvValue(summaries, stats.entropy); summaries << ',';
            writeCsvValue(summaries, stats.credible_width95); summaries << ',';
            writeCsvValue(summaries, stats.peak_offset); summaries << ',' << stats.peak_count << ',';
            writeCsvValue(summaries, stats.second_peak_ratio); summaries << ',' << (stats.range_insufficient ? 1 : 0) << ',';
            writeCsvValue(summaries, center_score); summaries << ',';
            writeCsvValue(summaries, max_score); summaries << ',';
            writeCsvValue(summaries, elapsed_ms); summaries << ',' << kScoreSemantics << '\n';
            comparison << frame_index << ',' << target_rel << ',' << t_rel << ',' << type << ',' << eigen_index << ',';
            writeCsvValue(comparison, direction.lambda); comparison << ',';
            writeCsvValue(comparison, lambda_norm); comparison << ',';
            writeCsvValue(comparison, alpha); comparison << ',';
            writeCsvValue(comparison, stats.sigma); comparison << ',';
            writeCsvValue(comparison, stats.credible_width95); comparison << ',';
            writeCsvValue(comparison, stats.entropy); comparison << ',';
            writeCsvValue(comparison, stats.peak_offset); comparison << ',' << stats.peak_count << ',';
            writeCsvValue(comparison, stats.second_peak_ratio); comparison << ',' << (stats.range_insufficient ? 1 : 0) << ',';
            writeCsvValue(comparison, center_score); comparison << ',';
            writeCsvValue(comparison, max_score); comparison << '\n';
          }
        }
      }
      frame_eval_ms.push_back(std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - frame_start).count());
    }
    auto percentile = [](std::vector<double> values, double fraction) {
      if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
      std::sort(values.begin(), values.end());
      const size_t index = std::min(values.size() - 1,
                                    static_cast<size_t>(std::ceil(fraction * values.size()) - 1.0));
      return values[index];
    };
    std::ofstream performance(output_dir + "/probe_performance.csv");
    if (!performance.is_open()) throw std::runtime_error("failed to open performance output");
    performance << "map_load_ms,target_setup_ms,score_cells,mean_cell_ms,p95_cell_ms,max_cell_ms,mean_frame_6dir_ms,max_frame_6dir_ms\n";
    const double mean_cell = cell_eval_ms.empty() ? std::numeric_limits<double>::quiet_NaN() :
        std::accumulate(cell_eval_ms.begin(), cell_eval_ms.end(), 0.0) / cell_eval_ms.size();
    const double mean_frame = frame_eval_ms.empty() ? std::numeric_limits<double>::quiet_NaN() :
        std::accumulate(frame_eval_ms.begin(), frame_eval_ms.end(), 0.0) / frame_eval_ms.size();
    performance << std::setprecision(17) << map_load_ms << ',' << target_setup_ms << ',' <<
        (cell_eval_ms.size() * 41U) << ',' << mean_cell << ',' << percentile(cell_eval_ms, 0.95) << ',' <<
        (cell_eval_ms.empty() ? std::numeric_limits<double>::quiet_NaN() :
         *std::max_element(cell_eval_ms.begin(), cell_eval_ms.end())) << ',' << mean_frame << ',' <<
        (frame_eval_ms.empty() ? std::numeric_limits<double>::quiet_NaN() :
         *std::max_element(frame_eval_ms.begin(), frame_eval_ms.end())) << '\n';
    std::cout << "frames=" << frames.size() << " target_size=" << target_cloud->size()
              << " output_dir=" << output_dir << '\n';
  }
  catch (const std::exception &error)
  {
    std::cerr << "ndt_uncertainty_probe: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
