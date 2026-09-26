#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "p5_i1_input_hash.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
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
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class AuditedNdt : public Ndt {
 public:
  double fixedScore(Cloud& transformed, Vector6d& p, Matrix6d* score_hessian = nullptr) {
    Vector6d gradient;
    Matrix6d hessian;
    const double score = computeDerivatives(gradient, hessian, transformed, p, true);
    if (score_hessian) *score_hessian = hessian;
    return score;
  }
};

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, delimiter)) fields.push_back(field);
  return fields;
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
    const double increment = static_cast<double>(down->size() - 1) / std::max(cap - 1, 1);
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
  if (pcl::io::loadPCDFile<Point>(path, *raw) != 0) throw std::runtime_error("cannot load fixed map");
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& p : raw->points)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) finite->push_back(p);
  finalize(finite);
  return voxelDown(voxelDown(finite, 0.15, 0), 0.15, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const Point& p : raw->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (range >= 0.5 && range <= 80.0) filtered->push_back(p);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25, 1400);
}

uint64_t cloudHash(const Cloud::Ptr& cloud) {
  constexpr uint64_t offset = 1469598103934665603ULL, prime = 1099511628211ULL;
  uint64_t hash = offset;
  auto mix = [&hash](uint8_t b) { hash = (hash ^ b) * prime; };
  auto mix32 = [&mix](uint32_t x) { for (int i = 0; i < 32; i += 8) mix(static_cast<uint8_t>(x >> i)); };
  mix32(cloud->width); mix32(cloud->height); mix32(cloud->is_dense ? 1U : 0U);
  mix32(static_cast<uint32_t>(cloud->size()));
  for (const Point& p : cloud->points) {
    uint32_t bits;
    std::memcpy(&bits, &p.x, sizeof(bits)); mix32(bits);
    std::memcpy(&bits, &p.y, sizeof(bits)); mix32(bits);
    std::memcpy(&bits, &p.z, sizeof(bits)); mix32(bits);
  }
  return hash;
}

Eigen::Matrix4f parsePose(const std::string& text) {
  const auto v = split(text, ';');
  if (v.size() == 16) {
    Eigen::Matrix4f t;
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        t(row, col) = std::stof(v[static_cast<std::size_t>(row * 4 + col)]);
    return t;
  }
  if (v.size() != 7) throw std::runtime_error("invalid representative pose");
  Eigen::Quaternionf q(std::stof(v[6]), std::stof(v[3]), std::stof(v[4]), std::stof(v[5]));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f) throw std::runtime_error("invalid pose quaternion");
  q.normalize();
  Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
  t.block<3, 3>(0, 0) = q.toRotationMatrix();
  t.block<3, 1>(0, 3) = Eigen::Vector3f(std::stof(v[0]), std::stof(v[1]), std::stof(v[2]));
  return t;
}

struct Request {
  std::string frame, mode, segment, selection, pose_text, curvature_reason;
  std::string input_bag_sha256, input_map_sha256;
  uint64_t expected_hash = 0;
  double time_s = 0.0, basin = 0.0, score = 0.0;
  int seed_count = 0;
};

std::vector<Request> readRequests(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty curvature request file");
  const auto header = split(line, ',');
  std::map<std::string, std::size_t> col;
  for (std::size_t i = 0; i < header.size(); ++i) col[header[i]] = i;
  auto get = [&col](const std::vector<std::string>& row, const std::string& name) -> std::string {
    auto it = col.find(name);
    if (it == col.end() || it->second >= row.size()) throw std::runtime_error("missing request column " + name);
    return row[it->second];
  };
  std::vector<Request> requests;
  while (std::getline(in, line)) {
    const auto row = split(line, ',');
    Request r;
    r.frame = get(row, "frame_id"); r.mode = get(row, "cluster_id");
    r.segment = get(row, "segment"); r.selection = get(row, "selection_labels");
    r.curvature_reason = get(row, "curvature_reason");
    r.input_bag_sha256 = get(row, "input_bag_sha256");
    r.input_map_sha256 = get(row, "input_map_sha256");
    r.pose_text = get(row, "representative_pose_matrix16");
    r.expected_hash = std::stoull(get(row, "source_hash_expected"));
    r.time_s = std::stod(get(row, "time_s")); r.basin = std::stod(get(row, "basin_fraction"));
    r.score = std::stod(get(row, "representative_score")); r.seed_count = std::stoi(get(row, "seed_count"));
    requests.push_back(r);
  }
  return requests;
}

Vector6d transformVector(const Eigen::Matrix4f& pose) {
  Eigen::Transform<float, 3, Eigen::Affine> tf;
  tf.matrix() = pose;
  const Eigen::Vector3f e = tf.rotation().eulerAngles(0, 1, 2);
  Vector6d p;
  p << tf.translation().x(), tf.translation().y(), tf.translation().z(), e.x(), e.y(), e.z();
  return p;
}

Eigen::Matrix4f poseFromVector(const Vector6d& p) {
  const Eigen::AngleAxisf rx(static_cast<float>(p[3]), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf ry(static_cast<float>(p[4]), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rz(static_cast<float>(p[5]), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = (rx * ry * rz).toRotationMatrix();
  pose.block<3, 1>(0, 3) = p.head<3>().cast<float>();
  return pose;
}

double scoreAt(AuditedNdt& ndt, const Cloud::Ptr& source, const Vector6d& p) {
  const Eigen::Matrix4f tf = poseFromVector(p);
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, tf);
  Vector6d mutable_p = p;
  return ndt.fixedScore(transformed, mutable_p);
}

Matrix6d hessianAt(AuditedNdt& ndt, const Cloud::Ptr& source, const Vector6d& center,
                   const Vector6d& h, double f0, uint64_t* eval_count) {
  Matrix6d hessian = Matrix6d::Zero();
  double fp[6], fm[6];
  for (int i = 0; i < 6; ++i) {
    Vector6d p = center; p[i] += h[i];
    Vector6d m = center; m[i] -= h[i];
    fp[i] = scoreAt(ndt, source, p); fm[i] = scoreAt(ndt, source, m);
    ++*eval_count; ++*eval_count;
    hessian(i, i) = (fp[i] - 2.0 * f0 + fm[i]) / (h[i] * h[i]);
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = i + 1; j < 6; ++j) {
      Vector6d pp = center, pm = center, mp = center, mm = center;
      pp[i] += h[i]; pp[j] += h[j];
      pm[i] += h[i]; pm[j] -= h[j];
      mp[i] -= h[i]; mp[j] += h[j];
      mm[i] -= h[i]; mm[j] -= h[j];
      const double fpp = scoreAt(ndt, source, pp);
      const double fpm = scoreAt(ndt, source, pm);
      const double fmp = scoreAt(ndt, source, mp);
      const double fmm = scoreAt(ndt, source, mm);
      *eval_count += 4;
      hessian(i, j) = hessian(j, i) = (fpp - fpm - fmp + fmm) / (4.0 * h[i] * h[j]);
    }
  }
  return 0.5 * (hessian + hessian.transpose()).eval();
}

std::string flatMatrix(const Matrix6d& matrix) {
  std::ostringstream out;
  out << std::setprecision(15);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      if (i || j) out << ';';
      out << matrix(i, j);
    }
  return out.str();
}

std::string flatEigenvectors(const Matrix6d& vectors) {
  std::ostringstream out;
  out << std::setprecision(15);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) { if (i || j) out << ';'; out << vectors(i, j); }
  return out.str();
}

void configure(AuditedNdt& ndt, const Cloud::Ptr& target) {
  ndt.setInputTarget(target);
  ndt.setResolution(0.8);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(0.001);
  ndt.setMaximumIterations(40);
}

void run(const std::string& map_path, const std::string& request_path,
         const std::string& cloud_dir, const std::string& output_path) {
  constexpr double kResolution = 0.8;
  constexpr double kAngleStep = 0.25 * M_PI / 180.0;
  p5_i1::requireFrozenMapSha256(map_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("fixed target point-count mismatch");
  const auto requests = readRequests(request_path);
  if (requests.empty()) throw std::runtime_error("no curvature requests");
  const std::string frozen_bag_sha = requests.front().input_bag_sha256;
  if (frozen_bag_sha != p5_i1::kExpectedBagSha256)
    throw std::runtime_error("curvature requests do not identify the frozen runtime-topic bag");
  for (const Request& request : requests)
    if (request.input_bag_sha256 != frozen_bag_sha ||
        request.input_map_sha256 != p5_i1::kExpectedMapSha256)
      throw std::runtime_error("curvature-request provenance mismatch");
  std::cout << "INPUT_BAG_SHA256=" << frozen_bag_sha << '\n'
            << "INPUT_MAP_SHA256=" << p5_i1::kExpectedMapSha256 << '\n';
  AuditedNdt ndt;
  configure(ndt, target);
  std::ofstream out(output_path);
  if (!out) throw std::runtime_error("cannot open curvature output");
  out << "frame_id,cluster_id,time_s,segment,selection_labels,curvature_reason,seed_count,basin_fraction,"
         "representative_score,source_points,target_points,source_hash_expected,source_hash_actual,"
         "objective_at_representative,representative_score_saved,objective_minus_seedrun_score,"
         "analytic_hessian_valid,analytic_hessian_asymmetry_frobenius,"
         "negative_curvature_definite,condition_number_scaled_curvature,"
         "score_hessian_raw_base_flat36,score_hessian_scaled_base_flat36,"
         "score_hessian_raw_eigenvalues,score_hessian_scaled_eigenvalues,"
         "negative_scaled_curvature_eigenvalues,negative_scaled_curvature_eigenvectors_flat36,"
         "h_translation_base_m,h_rotation_base_rad,eval_count_half,eval_count_base,eval_count_double,"
         "analytic_runtime_ms,fd_runtime_ms,runtime_ms,fd_numerically_unstable,"
         "fd_max_spectrum_order_ratio,fd_relative_frobenius_error,"
         "fd_score_hessian_scaled_eigenvalues_half,fd_score_hessian_scaled_eigenvalues_double,"
         "fd_curvature_eigenvalues_half,fd_curvature_eigenvalues_base,fd_curvature_eigenvalues_double,"
         "fd_score_hessian_raw_half_flat36,fd_score_hessian_raw_base_flat36,fd_score_hessian_raw_double_flat36,"
         "fd_score_hessian_scaled_half_flat36,fd_score_hessian_scaled_base_flat36,"
         "fd_score_hessian_scaled_double_flat36,input_bag_sha256,input_map_sha256\n";

  std::map<std::string, Cloud::Ptr> source_cache;
  std::set<std::string> initialized_frames;
  for (const Request& request : requests) {
    Cloud::Ptr source;
    auto found = source_cache.find(request.frame);
    if (found == source_cache.end()) {
      Cloud::Ptr raw(new Cloud);
      const std::string path = cloud_dir + "/" + request.frame + ".pcd";
      if (pcl::io::loadPCDFile<Point>(path, *raw) != 0) throw std::runtime_error("cannot load " + path);
      source = preprocessSource(raw);
      source_cache[request.frame] = source;
    } else source = found->second;
    const uint64_t actual_hash = cloudHash(source);
    if (actual_hash != request.expected_hash) throw std::runtime_error("source hash mismatch: " + request.frame);
    ndt.setInputSource(source);
    if (initialized_frames.insert(request.frame).second) {
      // PCL 1.10 recomputes its Gaussian constants inside align(), not in
      // setResolution(). Match runtime objective constants before fixed-pose
      // scoring without changing any production code or stored inputs.
      Cloud warmup;
      ndt.align(warmup, Eigen::Matrix4f::Identity());
    }
    const Eigen::Matrix4f representative = parsePose(request.pose_text);
    const Vector6d center = transformVector(representative);
    const double ht = std::max(0.01, 0.02 * kResolution);
    Vector6d h;
    h << ht, ht, ht, kAngleStep, kAngleStep, kAngleStep;
    Matrix6d hessians[3];
    Cloud base_transformed;
    pcl::transformPointCloud(*source, base_transformed, representative);
    Vector6d base_p = center;
    Matrix6d analytic_raw;
    const auto analytic_started = std::chrono::steady_clock::now();
    const double f_at_rep = ndt.fixedScore(base_transformed, base_p, &analytic_raw);
    const double analytic_runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - analytic_started).count();
    uint64_t evaluations[3]{};
    const double factors[3] = {0.5, 1.0, 2.0};
    const auto fd_started = std::chrono::steady_clock::now();
    for (int scale = 0; scale < 3; ++scale)
      hessians[scale] = hessianAt(ndt, source, center, h * factors[scale],
                                  f_at_rep, &evaluations[scale]);
    const double fd_runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - fd_started).count();
    const double runtime_ms = analytic_runtime_ms + fd_runtime_ms;
    Matrix6d d = Matrix6d::Identity();
    d(0, 0) = d(1, 1) = d(2, 2) = kResolution;
    const Matrix6d analytic_sym = 0.5 * (analytic_raw + analytic_raw.transpose()).eval();
    const Matrix6d analytic_scaled = d * analytic_sym * d;
    Eigen::SelfAdjointEigenSolver<Matrix6d> analytic_raw_eig(analytic_sym);
    Eigen::SelfAdjointEigenSolver<Matrix6d> analytic_scaled_eig(analytic_scaled);
    if (analytic_raw_eig.info() != Eigen::Success || analytic_scaled_eig.info() != Eigen::Success)
      throw std::runtime_error("analytic PCL Hessian eigensolver failed");
    const double analytic_asymmetry = (analytic_raw - analytic_raw.transpose()).norm();
    const bool analytic_valid = analytic_raw.allFinite() && std::isfinite(f_at_rep);
    if (!analytic_valid) throw std::runtime_error("invalid analytic PCL Hessian");
    Matrix6d scaled[3];
    Eigen::SelfAdjointEigenSolver<Matrix6d> eig[3];
    Eigen::SelfAdjointEigenSolver<Matrix6d> raw_eig(hessians[1]);
    for (int i = 0; i < 3; ++i) {
      scaled[i] = d * hessians[i] * d;
      eig[i].compute(scaled[i]);
      if (eig[i].info() != Eigen::Success) throw std::runtime_error("Hessian eigensolver failed");
    }
    Vector6d curvature_values[3];
    for (int i = 0; i < 3; ++i) curvature_values[i] = -eig[i].eigenvalues().reverse().eval();
    double max_ratio = 1.0;
    bool unstable = false;
    for (int i = 0; i < 6; ++i) {
      const double base = curvature_values[1][i];
      for (int j : {0, 2}) {
        const double value = curvature_values[j][i];
        if ((base > 0.0) != (value > 0.0)) unstable = true;
        const double ratio = std::max(std::abs(base), std::abs(value)) /
                             std::max(std::min(std::abs(base), std::abs(value)), 1e-14);
        max_ratio = std::max(max_ratio, ratio);
      }
    }
    if (max_ratio > 10.0) unstable = true;
    const Vector6d sorted_scaled_score = analytic_scaled_eig.eigenvalues();
    const Vector6d negative_curvature = -sorted_scaled_score.reverse().eval();
    const bool negative_definite = negative_curvature.minCoeff() > 0.0;
    const double condition = negative_definite ? negative_curvature.maxCoeff() /
                                                   negative_curvature.minCoeff() :
                                                   std::numeric_limits<double>::quiet_NaN();
    const Matrix6d negative_eigenvectors = analytic_scaled_eig.eigenvectors().rowwise().reverse();
    const double fd_relative_error = (scaled[1] - analytic_scaled).norm() /
                                     std::max(analytic_scaled.norm(), 1e-12);
    Eigen::SelfAdjointEigenSolver<Matrix6d> curvature_eig[3];
    for (int i = 0; i < 3; ++i) curvature_eig[i].compute(-scaled[i]);
    std::ostringstream evh, evs, evc, eigvecs;
    evh << std::setprecision(15) << analytic_raw_eig.eigenvalues().transpose();
    evs << std::setprecision(15) << analytic_scaled_eig.eigenvalues().transpose();
    evc << std::setprecision(15) << negative_curvature.transpose();
    eigvecs << flatEigenvectors(negative_eigenvectors);
    out << std::setprecision(15) << request.frame << ',' << request.mode << ',' << request.time_s << ','
        << request.segment << ',' << '"' << request.selection << '"' << ',' << request.curvature_reason << ','
        << request.seed_count << ','
        << request.basin << ',' << request.score << ',' << source->size() << ',' << target->size() << ','
        << request.expected_hash << ',' << actual_hash << ',' << f_at_rep << ',' << request.score << ','
        << f_at_rep - request.score << ',' << (analytic_valid ? 1 : 0) << ',' << analytic_asymmetry << ','
        << (negative_definite ? 1 : 0) << ','
        << (negative_definite ? condition : -1.0) << ',' << flatMatrix(analytic_raw) << ','
        << flatMatrix(analytic_scaled) << ',' << '"' << evh.str() << '"' << ',' << '"' << evs.str() << '"' << ','
        << '"' << evc.str() << '"' << ',' << '"' << eigvecs.str() << '"' << ','
        << ht << ',' << kAngleStep << ',' << evaluations[0] << ',' << evaluations[1] << ',' << evaluations[2] << ','
        << analytic_runtime_ms << ',' << fd_runtime_ms << ',' << runtime_ms << ',' << (unstable ? 1 : 0) << ','
        << max_ratio << ',' << fd_relative_error << ','
        << '"' << eig[0].eigenvalues().transpose() << '"' << ','
        << '"' << eig[2].eigenvalues().transpose() << '"' << ','
        << '"' << curvature_eig[0].eigenvalues().reverse().eval().transpose() << '"' << ','
        << '"' << curvature_eig[1].eigenvalues().reverse().eval().transpose() << '"' << ','
        << '"' << curvature_eig[2].eigenvalues().reverse().eval().transpose() << '"' << ','
        << flatMatrix(hessians[0]) << ',' << flatMatrix(hessians[1]) << ',' << flatMatrix(hessians[2]) << ','
        << flatMatrix(scaled[0]) << ',' << flatMatrix(scaled[1]) << ',' << flatMatrix(scaled[2]) << ','
        << request.input_bag_sha256 << ',' << p5_i1::kExpectedMapSha256 << '\n';
    std::cerr << "CURVATURE " << request.frame << '/' << request.mode << " K=" << request.seed_count
              << " fd_unstable=" << unstable << " analytic_negative_definite=" << negative_definite
              << " condition=" << (negative_definite ? condition : -1.0)
              << " elapsed_ms=" << runtime_ms << '\n';
  }
  out.flush();
  if (!out) throw std::runtime_error("failed writing curvature output");
  std::cout << "CURVATURE_MODE_COUNT=" << requests.size() << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: p5_i1_ndt_curvature MAP.pcd REQUESTS.csv CLOUD_DIR OUTPUT.csv\n";
    return 2;
  }
  try {
    run(argv[1], argv[2], argv[3], argv[4]);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "curvature error: " << e.what() << '\n';
    return 1;
  }
}
