#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "p5_i1_input_hash.hpp"

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
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;

class AuditedNdt : public Ndt {
 public:
  double fixedScore(Cloud& transformed, Eigen::Matrix<double, 6, 1>& p) {
    Eigen::Matrix<double, 6, 1> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    return this->computeDerivatives(gradient, hessian, transformed, p, true);
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
    throw std::runtime_error("failed to load frozen prior map: " + path);
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
      finite->push_back(point);
  finalize(finite);
  Cloud::Ptr map = voxelDown(finite, 0.15, 0.15, 0);
  return voxelDown(map, 0.15, 0.15, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const Point& point : raw->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25, 0.25, 1400);
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

struct Frame {
  std::string id;
  std::string input_bag_sha256;
  std::string input_map_sha256;
  uint64_t transaction_id = 0;
  double time_s = 0.0;
  uint64_t expected_source_hash = 0;
  double saved_fitness = 0.0;
  uint64_t request_cloud_hash = 0;
  std::string selection_labels;
  std::string segment;
  int saved_iterations = 0;
  int saved_converged = 0;
  int step_limited = 0;
  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f saved_raw = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f saved_used = Eigen::Matrix4f::Identity();
};

std::vector<Frame> readFrames(const std::string& manifest, bool baseline_only) {
  std::ifstream input(manifest);
  if (!input) throw std::runtime_error("cannot open frame manifest");
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty frame manifest");
  const auto header = split(line, ',');
  std::map<std::string, std::size_t> col;
  for (std::size_t i = 0; i < header.size(); ++i) col[header[i]] = i;
  const auto get = [&col](const std::vector<std::string>& row, const std::string& name) -> const std::string& {
    const auto found = col.find(name);
    if (found == col.end() || found->second >= row.size()) throw std::runtime_error("missing manifest field: " + name);
    return row[found->second];
  };
  std::vector<Frame> frames;
  while (std::getline(input, line)) {
    const auto row = split(line, ',');
    if (baseline_only && get(row, "baseline_reproduction") != "1") continue;
    Frame frame;
    frame.id = get(row, "frame_id");
    frame.input_bag_sha256 = get(row, "input_bag_sha256");
    frame.input_map_sha256 = get(row, "input_map_sha256");
    frame.transaction_id = std::stoull(get(row, "transaction_id"));
    frame.time_s = std::stod(get(row, "time_s"));
    frame.expected_source_hash = std::stoull(get(row, "ndt_source_cloud_hash"));
    frame.request_cloud_hash = std::stoull(get(row, "request_cloud_hash"));
    frame.selection_labels = get(row, "selection_labels");
    frame.segment = get(row, "segment");
    frame.saved_fitness = std::stod(get(row, "fitness"));
    frame.saved_iterations = std::stoi(get(row, "iterations"));
    frame.saved_converged = std::stoi(get(row, "converged"));
    frame.step_limited = std::stoi(get(row, "step_limited"));
    frame.initial = parsePose(get(row, "predicted_pose_xyz_q_xyzw"));
    frame.saved_raw = parsePose(get(row, "saved_raw_pose_xyz_q_xyzw"));
    frame.saved_used = parsePose(get(row, "saved_used_pose_xyz_q_xyzw"));
    frames.push_back(frame);
  }
  return frames;
}

struct Seed {
  std::string domain;
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
  double polar_deg = 0.0, radius_m = 0.0;
  int grid_ix = 0, grid_iy = 0, grid_iyaw = 0;
};

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  const Eigen::Matrix3d W = (Eigen::Matrix3d() << 0.0, -w.z(), w.y(),
                                                   w.z(), 0.0, -w.x(),
                                                  -w.y(), w.x(), 0.0).finished();
  if (theta < 1e-10) return Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
  return Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * W +
         ((1.0 - std::cos(theta)) / (theta * theta)) * W * W;
}

Eigen::Matrix4f rightPerturb(const Eigen::Matrix4f& anchor, const Seed& seed) {
  constexpr double kDeg = M_PI / 180.0;
  const Eigen::Vector3d rho(seed.dx, seed.dy, seed.dz);
  const Eigen::Vector3d omega(seed.roll_deg * kDeg, seed.pitch_deg * kDeg,
                              seed.yaw_deg * kDeg);
  const double theta = omega.norm();
  const Eigen::Matrix3d W = (Eigen::Matrix3d() << 0.0, -omega.z(), omega.y(),
                                                   omega.z(), 0.0, -omega.x(),
                                                  -omega.y(), omega.x(), 0.0).finished();
  Eigen::Matrix3d V;
  if (theta < 1e-10) {
    V = Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W * W;
  } else {
    V = Eigen::Matrix3d::Identity() + ((1.0 - std::cos(theta)) / (theta * theta)) * W +
        ((theta - std::sin(theta)) / (theta * theta * theta)) * W * W;
  }
  Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
  delta.block<3, 3>(0, 0) = expSO3(omega).cast<float>();
  delta.block<3, 1>(0, 3) = (V * rho).cast<float>();
  return anchor * delta;
}

std::string poseString(const Eigen::Matrix4f& pose) {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();
  std::ostringstream out;
  out << std::setprecision(9) << pose(0, 3) << ';' << pose(1, 3) << ';'
      << pose(2, 3) << ';' << q.x() << ';' << q.y() << ';' << q.z() << ';' << q.w();
  return out.str();
}

std::string matrixString(const Eigen::Matrix4f& pose) {
  std::ostringstream out;
  out << std::setprecision(9);
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) {
      if (row || col) out << ';';
      out << pose(row, col);
    }
  return out.str();
}

std::vector<Seed> planarSeeds(double resolution) {
  const std::vector<double> offsets = {-2.0 * resolution, -resolution,
      -0.5 * resolution, 0.0, 0.5 * resolution, resolution, 2.0 * resolution};
  const std::vector<double> yaws = {-10.0, -5.0, 0.0, 5.0, 10.0};
  std::vector<Seed> seeds;
  for (std::size_t ix = 0; ix < offsets.size(); ++ix)
    for (std::size_t iy = 0; iy < offsets.size(); ++iy)
      for (std::size_t ia = 0; ia < yaws.size(); ++ia) {
        Seed seed;
        seed.domain = "PLANAR";
        seed.dx = offsets[ix]; seed.dy = offsets[iy]; seed.yaw_deg = yaws[ia];
        seed.grid_ix = static_cast<int>(ix) - 3;
        seed.grid_iy = static_cast<int>(iy) - 3;
        seed.grid_iyaw = static_cast<int>(ia) - 2;
        seeds.push_back(seed);
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
  const double radii[2] = {std::max(3.0 * resolution, 1.5),
                           std::max(5.0 * resolution, 3.0)};
  const double yaws[3] = {-15.0, 0.0, 15.0};
  std::vector<Seed> seeds;
  for (double radius : radii)
    for (int angle = 0; angle < 8; ++angle)
      for (double yaw : yaws) {
        const double theta = angle * M_PI / 4.0;
        Seed s;
        s.domain = "WIDE_PLANAR";
        s.dx = radius * std::cos(theta); s.dy = radius * std::sin(theta);
        s.yaw_deg = yaw; s.polar_deg = angle * 45.0; s.radius_m = radius;
        seeds.push_back(s);
      }
  return seeds;
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

double fixedPclScore(AuditedNdt& ndt, const Cloud::Ptr& source,
                     const Eigen::Matrix4f& pose) {
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, pose);
  Eigen::Matrix<double, 6, 1> p = pclTransformVector(pose);
  return ndt.fixedScore(transformed, p);
}

void configureNdt(Ndt& ndt, const Cloud::Ptr& target) {
  // Preserve formal node order: setInputTarget initializes PCL's target grid
  // before runtime resolution/step/epsilon/iteration values are assigned.
  ndt.setInputTarget(target);
  ndt.setResolution(0.8);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(0.001);
  ndt.setMaximumIterations(40);
}

void runSearch(const std::string& bag_path, const std::string& map_path, const std::string& manifest,
               const std::string& cloud_dir, const std::string& output_path) {
  constexpr double kResolution = 0.8;
  const std::string actual_bag_sha = p5_i1::sha256File(bag_path);
  if (actual_bag_sha != p5_i1::kExpectedBagSha256)
    throw std::runtime_error("runtime-topic bag SHA256 mismatch: " + actual_bag_sha);
  p5_i1::requireFrozenMapSha256(map_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("fixed target point-count mismatch");
  const std::vector<Frame> frames = readFrames(manifest, false);
  if (frames.size() != 32) throw std::runtime_error("expected frozen 32-frame cohort");
  for (const Frame& frame : frames)
    if (frame.input_bag_sha256 != actual_bag_sha || frame.input_map_sha256 != p5_i1::kExpectedMapSha256)
      throw std::runtime_error("manifest input provenance mismatch at " + frame.id);
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot open seed-run output");
  output << "frame_id,transaction_id,time_s,selection_labels,segment,seed_index,seed_domain,"
            "seed_dx_m,seed_dy_m,seed_dz_m,seed_roll_deg,seed_pitch_deg,seed_yaw_deg,"
            "wide_radius_m,wide_angle_deg,grid_ix,grid_iy,grid_iyaw,"
            "start_pose_xyz_q_xyzw,final_pose_xyz_q_xyzw,final_pose_matrix16,"
            "final_translation_from_anchor_m,final_rotation_from_anchor_deg,"
            "source_points,target_points,source_hash_expected,source_hash_actual,"
            "fitness,transformation_probability,raw_ndt_score_sum,iterations,converged,"
            "runtime_ms,final_offset_from_seed_m,final_rotation_from_seed_deg,input_bag_sha256,input_map_sha256\n";
  const std::vector<Seed> planar = planarSeeds(kResolution);
  const std::vector<Seed> axial = axialSeeds(kResolution);
  const std::vector<Seed> wide = widePlanarSeeds(kResolution);
  AuditedNdt ndt;
  configureNdt(ndt, target);
  const auto started_all = std::chrono::steady_clock::now();
  std::size_t total_runs = 0;
  for (const Frame& frame : frames) {
    const std::string source_path = cloud_dir + "/" + frame.id + ".pcd";
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<Point>(source_path, *raw) != 0)
      throw std::runtime_error("cannot load extracted cloud " + source_path);
    const Cloud::Ptr source = preprocessSource(raw);
    const uint64_t actual_hash = sourceCloudHash(source);
    if (actual_hash != frame.expected_source_hash)
      throw std::runtime_error("preprocessed source hash mismatch for " + frame.id);
    ndt.setInputSource(source);
    std::vector<Seed> seeds = planar;
    seeds.insert(seeds.end(), axial.begin(), axial.end());
    const bool targeted = frame.selection_labels.find("TARGETED:") != std::string::npos;
    if (targeted) seeds.insert(seeds.end(), wide.begin(), wide.end());
    for (std::size_t index = 0; index < seeds.size(); ++index) {
      const Seed& seed = seeds[index];
      const Eigen::Matrix4f start_pose = rightPerturb(frame.initial, seed);
      Cloud aligned;
      const auto start = std::chrono::steady_clock::now();
      ndt.align(aligned, start_pose);
      const double runtime_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      const Eigen::Matrix4f final_pose = ndt.getFinalTransformation();
      const Eigen::Matrix4f anchor_delta = frame.initial.inverse() * final_pose;
      const Eigen::Matrix4f seed_delta = start_pose.inverse() * final_pose;
      const double anchor_t = anchor_delta.block<3, 1>(0, 3).norm();
      const Eigen::AngleAxisf anchor_aa(anchor_delta.block<3, 3>(0, 0));
      const double anchor_r = std::abs(anchor_aa.angle()) * 180.0 / M_PI;
      const double seed_t = seed_delta.block<3, 1>(0, 3).norm();
      const Eigen::AngleAxisf seed_aa(seed_delta.block<3, 3>(0, 0));
      const double seed_r = std::abs(seed_aa.angle()) * 180.0 / M_PI;
      const double probability = ndt.getTransformationProbability();
      const double exact_score_sum = fixedPclScore(ndt, source, final_pose);
      output << std::setprecision(15) << frame.id << ',' << frame.transaction_id << ','
             << frame.time_s << ',' << '"' << frame.selection_labels << '"' << ','
             << frame.segment << ',' << index << ','
             << seed.domain << ',' << seed.dx << ',' << seed.dy << ',' << seed.dz << ','
             << seed.roll_deg << ',' << seed.pitch_deg << ',' << seed.yaw_deg << ','
             << seed.radius_m << ',' << seed.polar_deg << ',' << seed.grid_ix << ','
             << seed.grid_iy << ',' << seed.grid_iyaw << ',' << poseString(start_pose) << ','
             << poseString(final_pose) << ',' << matrixString(final_pose) << ',' << anchor_t << ',' << anchor_r << ','
             << source->size() << ',' << target->size() << ',' << frame.expected_source_hash << ','
             << actual_hash << ',' << ndt.getFitnessScore() << ',' << probability << ','
             << exact_score_sum << ',' << ndt.getFinalNumIteration() << ','
             << (ndt.hasConverged() ? 1 : 0) << ',' << runtime_ms << ',' << seed_t << ','
             << seed_r << ',' << actual_bag_sha << ',' << p5_i1::kExpectedMapSha256 << '\n';
      ++total_runs;
    }
    std::cerr << "SEARCH_FRAME=" << frame.id << " time_s=" << frame.time_s
              << " runs=" << seeds.size() << " source_points=" << source->size()
              << " targeted=" << targeted << '\n';
  }
  output.flush();
  if (!output) throw std::runtime_error("failed writing seed-run output");
  std::cout << "FRAMES=" << frames.size() << "\nSEEDS_PER_FRAME_PLANAR=" << planar.size()
            << "\nAXIAL_PER_FRAME=" << axial.size() << "\nWIDE_PER_TARGETED_FRAME="
            << wide.size() << "\nTOTAL_RUNS=" << total_runs << "\nELAPSED_SEC="
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - started_all).count()
            << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  if ((argc != 7) || (std::string(argv[1]) != "baseline" &&
                      std::string(argv[1]) != "search")) {
    std::cerr << "usage: p5_i1_ndt_mode_landscape (baseline|search) BAG MAP.pcd MANIFEST.csv CLOUD_DIR OUTPUT.csv\n";
    return 2;
  }
  try {
    if (std::string(argv[1]) == "search") {
      runSearch(argv[2], argv[3], argv[4], argv[5], argv[6]);
      return 0;
    }
    const std::string actual_bag_sha = p5_i1::sha256File(argv[2]);
    if (actual_bag_sha != p5_i1::kExpectedBagSha256)
      throw std::runtime_error("runtime-topic bag SHA256 mismatch: " + actual_bag_sha);
    p5_i1::requireFrozenMapSha256(argv[3]);
    const Cloud::Ptr target = loadTarget(argv[3]);
    if (target->size() != 549606) throw std::runtime_error("fixed target point-count mismatch");
    std::vector<Frame> frames = readFrames(argv[4], true);
    if (frames.size() != 5) throw std::runtime_error("baseline reproduction cohort must be 5 frames");
    for (const Frame& frame : frames)
      if (frame.input_bag_sha256 != actual_bag_sha || frame.input_map_sha256 != p5_i1::kExpectedMapSha256)
        throw std::runtime_error("baseline manifest provenance mismatch at " + frame.id);

    AuditedNdt ndt;
    configureNdt(ndt, target);
    std::ofstream output(argv[6]);
    if (!output) throw std::runtime_error("cannot open baseline output");
    output << "frame_id,transaction_id,time_s,source_points,target_points,source_hash_expected,source_hash_actual,"
              "translation_difference_m,rotation_difference_deg,fitness_saved,fitness_reproduced,fitness_abs_difference,"
              "iterations_saved,iterations_reproduced,converged_saved,converged_reproduced,convergence_match,"
              "transformation_probability,fixed_pose_pcl_score_sum,score_sum_minus_probability_sum,runtime_ms,input_bag_sha256,input_map_sha256\n";

    bool pass = true;
    for (const Frame& frame : frames) {
      const std::string source_path = std::string(argv[5]) + "/" + frame.id + ".pcd";
      Cloud::Ptr raw(new Cloud);
      if (pcl::io::loadPCDFile<Point>(source_path, *raw) != 0)
        throw std::runtime_error("cannot load extracted source cloud: " + source_path);
      const Cloud::Ptr source = preprocessSource(raw);
      const uint64_t actual_hash = sourceCloudHash(source);
      if (actual_hash != frame.expected_source_hash)
        throw std::runtime_error("preprocessed source hash mismatch for " + frame.id);
      ndt.setInputSource(source);
      Cloud aligned;
      const auto start = std::chrono::steady_clock::now();
      ndt.align(aligned, frame.initial);
      const double runtime_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      const Eigen::Matrix4f actual = ndt.getFinalTransformation();
      const double dt = (actual.block<3, 1>(0, 3) - frame.saved_raw.block<3, 1>(0, 3)).norm();
      const Eigen::Matrix3f dr = frame.saved_raw.block<3, 3>(0, 0).transpose() *
                                 actual.block<3, 3>(0, 0);
      const double cosine = std::max(-1.0, std::min(1.0,
          (static_cast<double>(dr.trace()) - 1.0) * 0.5));
      const double dtheta = std::acos(cosine) * 180.0 / M_PI;
      const double fitness = ndt.getFitnessScore();
      const int converged = ndt.hasConverged() ? 1 : 0;
      const bool frame_pass = dt <= 1e-3 && dtheta <= 1e-2 &&
                              converged == frame.saved_converged;
      pass = pass && frame_pass;
      const double probability = ndt.getTransformationProbability();
      const double fixed_score = fixedPclScore(ndt, source, actual);
      const double score_diff = fixed_score - probability * source->size();
      output << std::setprecision(15) << frame.id << ',' << frame.transaction_id << ',' << frame.time_s << ','
             << source->size() << ',' << target->size() << ',' << frame.expected_source_hash << ',' << actual_hash << ','
             << dt << ',' << dtheta << ',' << frame.saved_fitness << ',' << fitness << ','
             << std::abs(fitness - frame.saved_fitness) << ',' << frame.saved_iterations << ','
             << ndt.getFinalNumIteration() << ',' << frame.saved_converged << ',' << converged << ','
        << (converged == frame.saved_converged ? 1 : 0) << ',' << probability << ','
        << fixed_score << ',' << score_diff << ',' << runtime_ms << ','
        << actual_bag_sha << ',' << p5_i1::kExpectedMapSha256 << '\n';
      std::cerr << frame.id << " t=" << dt << "m r=" << dtheta
                << "deg fit_delta=" << std::abs(fitness - frame.saved_fitness)
                << " converged=" << converged << " source_hash_match="
                << (actual_hash == frame.expected_source_hash) << '\n';
    }
    std::cout << "TARGET_POINTS=" << target->size() << '\n'
              << "INPUT_BAG_SHA256=" << actual_bag_sha << '\n'
              << "INPUT_MAP_SHA256=" << p5_i1::kExpectedMapSha256 << '\n'
              << "BASELINE_REPRODUCTION=" << (pass ? "PASS" : "FAIL") << '\n';
    return pass ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "p5_i1 baseline error: " << error.what() << '\n';
    return 1;
  }
}
