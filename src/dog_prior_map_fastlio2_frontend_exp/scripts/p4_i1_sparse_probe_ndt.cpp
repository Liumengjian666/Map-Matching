#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;

void finalize(const Cloud::Ptr& cloud) {
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, const float leaf, const int cap) {
  Cloud::Ptr down(new Cloud);
  if (leaf > 0.01f) {
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(input);
    voxel.filter(*down);
  } else {
    *down = *input;
  }
  if (cap > 0 && static_cast<int>(down->size()) > cap) {
    Cloud::Ptr sampled(new Cloud);
    sampled->reserve(static_cast<std::size_t>(cap));
    const double step = static_cast<double>(down->size() - 1) /
                        static_cast<double>(std::max(cap - 1, 1));
    for (int i = 0; i < cap; ++i) {
      sampled->push_back(down->points[static_cast<std::size_t>(std::llround(i * step))]);
    }
    finalize(sampled);
    return sampled;
  }
  finalize(down);
  return down;
}

Cloud::Ptr loadTarget(const std::string& path) {
  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile<Point>(path, *raw) != 0) {
    throw std::runtime_error("failed to load map PCD: " + path);
  }
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const auto& point : raw->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite->push_back(point);
    }
  }
  finalize(finite);
  return voxelDown(voxelDown(finite, 0.15f, 0), 0.15f, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const auto& point : raw->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range >= 0.5 && range <= 80.0) filtered->push_back(point);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25f, 1400);
}

std::vector<std::string> splitTabs(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, '\t')) fields.push_back(field);
  return fields;
}

Eigen::Matrix4f poseFromFields(const std::vector<std::string>& fields, std::size_t offset) {
  const Eigen::Vector3f translation(std::stof(fields.at(offset)),
                                   std::stof(fields.at(offset + 1)),
                                   std::stof(fields.at(offset + 2)));
  Eigen::Quaternionf q(std::stof(fields.at(offset + 6)), std::stof(fields.at(offset + 3)),
                      std::stof(fields.at(offset + 4)), std::stof(fields.at(offset + 5)));
  if (!translation.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-6f) {
    throw std::runtime_error("invalid predictor pose in manifest");
  }
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = translation;
  return pose;
}

class NdtScoreOnly : public Ndt {
 public:
  // Adapt the Autoware MULTI_NDT_SCORE nearest-voxel likelihood query to the
  // protected PCL 1.10 NDT voxel grid. No optimizer is called for a probe pose.
  double nearestVoxelLikelihood(const Cloud& transformed) {
    double score_sum = 0.0;
    std::size_t matched_points = 0;
    std::vector<TargetGridLeafConstPtr> leaves;
    std::vector<float> squared_distances;
    for (const auto& point : transformed.points) {
      leaves.clear();
      squared_distances.clear();
      target_cells_.radiusSearch(point, resolution_, leaves, squared_distances);
      double best_score = -std::numeric_limits<double>::infinity();
      for (const auto* leaf : leaves) {
        const Eigen::Vector3d residual(
            static_cast<double>(point.x) - leaf->getMean().x(),
            static_cast<double>(point.y) - leaf->getMean().y(),
            static_cast<double>(point.z) - leaf->getMean().z());
        const Eigen::Matrix3d inverse_covariance = leaf->getInverseCov();
        const double mahalanobis = residual.dot(inverse_covariance * residual);
        const double cell_score = -gauss_d1_ * std::exp(-gauss_d2_ * mahalanobis / 2.0);
        if (std::isfinite(cell_score)) best_score = std::max(best_score, cell_score);
      }
      if (!leaves.empty()) {
        score_sum += best_score;
        ++matched_points;
      }
    }
    return matched_points == 0 ? 0.0 : score_sum / static_cast<double>(matched_points);
  }
};

struct Candidate {
  std::string id;
  std::string kind;
  float dx_body = 0.0f;
  float dy_body = 0.0f;
  float yaw_deg = 0.0f;
  bool center = false;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double score = 0.0;
  double score_ms = 0.0;
  double align_ms = 0.0;
  int converged = 0;
  int iterations = 0;
  double fitness = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix4f final_pose = Eigen::Matrix4f::Identity();
};

std::vector<Candidate> translationCandidates(const Eigen::Matrix4f& center) {
  const std::vector<std::pair<float, float>> offsets = {
      {0.0f, 0.0f}, {0.5f, 0.0f}, {-0.5f, 0.0f}, {0.0f, 0.5f}, {0.0f, -0.5f},
      {1.0f, 0.0f}, {-1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, -1.0f},
      {2.0f, 0.0f}, {-2.0f, 0.0f}, {0.0f, 2.0f}, {0.0f, -2.0f}};
  std::vector<Candidate> candidates;
  candidates.reserve(17);
  const Eigen::Matrix2f body_to_map = center.block<2, 2>(0, 0);
  for (std::size_t i = 0; i < offsets.size(); ++i) {
    Candidate candidate;
    candidate.id = "xy_" + std::to_string(i);
    candidate.kind = "translation";
    candidate.dx_body = offsets[i].first;
    candidate.dy_body = offsets[i].second;
    candidate.center = i == 0;
    candidate.pose = center;
    candidate.pose.block<2, 1>(0, 3) += body_to_map * Eigen::Vector2f(candidate.dx_body, candidate.dy_body);
    candidates.push_back(candidate);
  }
  return candidates;
}

void scoreCandidate(NdtScoreOnly& ndt, const Cloud& source, Candidate& candidate) {
  Cloud transformed;
  const auto start = std::chrono::steady_clock::now();
  pcl::transformPointCloud(source, transformed, candidate.pose);
  candidate.score = ndt.nearestVoxelLikelihood(transformed);
  candidate.score_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start).count();
}

void alignCandidate(NdtScoreOnly& ndt, Candidate& candidate) {
  Cloud aligned;
  const auto start = std::chrono::steady_clock::now();
  ndt.align(aligned, candidate.pose);
  candidate.align_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start).count();
  candidate.converged = ndt.hasConverged() ? 1 : 0;
  candidate.iterations = ndt.getFinalNumIteration();
  candidate.fitness = ndt.getFitnessScore();
  candidate.final_pose = ndt.getFinalTransformation();
}

void writePose(std::ostream& out, const Eigen::Matrix4f& pose) {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();
  out << pose(0, 3) << '\t' << pose(1, 3) << '\t' << pose(2, 3) << '\t'
      << q.x() << '\t' << q.y() << '\t' << q.z() << '\t' << q.w();
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: p4_i1_sparse_probe_ndt MAP.pcd MANIFEST.tsv OUTPUT.tsv\n";
    return 2;
  }
  try {
    const Cloud::Ptr target = loadTarget(argv[1]);
    std::ifstream manifest(argv[2]);
    std::ofstream output(argv[3]);
    if (!manifest || !output) throw std::runtime_error("cannot open manifest/output");

    NdtScoreOnly ndt;
    // Preserve the same target-grid initialization order as the already
    // baseline-validated P3 replay helper and the captured runtime server.
    ndt.setInputTarget(target);
    ndt.setResolution(0.8f);
    ndt.setStepSize(0.08);
    ndt.setTransformationEpsilon(0.001);
    ndt.setMaximumIterations(40);
    output << "event_id\tframe_index\ttime_s\tstamp_s\truntime_raw_fitness\tsource_points\t"
              "candidate_id\tkind\tdx_body_m\tdy_body_m\tyaw_deg\tis_center\tprobe_score\t"
              "score_ms\tprobe_rank\tsparse_top2\tbaseline_ms\tbaseline_converged\t"
              "baseline_iterations\tbaseline_fitness\tbaseline_tx\tbaseline_ty\tbaseline_tz\t"
              "baseline_qx\tbaseline_qy\tbaseline_qz\tbaseline_qw\talign_ms\tconverged\t"
              "iterations\tfitness\tfinal_tx\tfinal_ty\tfinal_tz\tfinal_qx\tfinal_qy\t"
              "final_qz\tfinal_qw\tprobe_total_ms\ttranslation_probe_ms\tselected_yaw_base\n";

    std::string line;
    std::size_t event_count = 0;
    while (std::getline(manifest, line)) {
      if (line.empty() || line[0] == '#') continue;
      const auto fields = splitTabs(line);
      if (fields.size() != 13) throw std::runtime_error("manifest line must have 13 tab fields");
      const std::string event_id = fields.at(0);
      const std::string frame_index = fields.at(1);
      const std::string time_s = fields.at(2);
      const std::string stamp_s = fields.at(3);
      const std::string pcd_path = fields.at(4);
      const double runtime_raw_fitness = std::stod(fields.at(12));

      Cloud::Ptr raw(new Cloud);
      if (pcl::io::loadPCDFile<Point>(pcd_path, *raw) != 0) {
        throw std::runtime_error("failed to load request cloud: " + pcd_path);
      }
      const Cloud::Ptr source = preprocessSource(raw);
      if (source->empty()) throw std::runtime_error("preprocessed source cloud is empty");
      const Eigen::Matrix4f predictor = poseFromFields(fields, 5);
      ndt.setInputSource(source);

      Candidate baseline;
      baseline.id = "baseline";
      baseline.pose = predictor;
      alignCandidate(ndt, baseline);

      const auto probe_start = std::chrono::steady_clock::now();
      std::vector<Candidate> candidates = translationCandidates(predictor);
      double translation_probe_ms = 0.0;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        scoreCandidate(ndt, *source, candidates[i]);
        translation_probe_ms += candidates[i].score_ms;
      }

      const auto best_translation = std::max_element(
          candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.score < b.score;
          });
      const Candidate yaw_base = *best_translation;
      for (const float yaw_deg : {3.0f, -3.0f, 6.0f, -6.0f}) {
        Candidate yaw_candidate;
        yaw_candidate.id = "yaw_" + yaw_base.id + "_" + (yaw_deg > 0 ? "p" : "m") +
                           std::to_string(std::abs(static_cast<int>(yaw_deg)));
        yaw_candidate.kind = "yaw";
        yaw_candidate.dx_body = yaw_base.dx_body;
        yaw_candidate.dy_body = yaw_base.dy_body;
        yaw_candidate.yaw_deg = yaw_deg;
        yaw_candidate.pose = yaw_base.pose;
        const Eigen::AngleAxisf yaw_delta(yaw_deg * static_cast<float>(M_PI / 180.0),
                                         Eigen::Vector3f::UnitZ());
        yaw_candidate.pose.block<3, 3>(0, 0) = yaw_delta.toRotationMatrix() *
                                               yaw_candidate.pose.block<3, 3>(0, 0);
        scoreCandidate(ndt, *source, yaw_candidate);
        candidates.push_back(yaw_candidate);
      }
      const double probe_total_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - probe_start).count();

      std::vector<std::size_t> ranked(candidates.size());
      for (std::size_t i = 0; i < ranked.size(); ++i) ranked[i] = i;
      std::stable_sort(ranked.begin(), ranked.end(), [&](const std::size_t a, const std::size_t b) {
        return candidates[a].score > candidates[b].score;
      });
      std::vector<std::size_t> alternatives;
      for (const std::size_t index : ranked) {
        if (!candidates[index].center) alternatives.push_back(index);
      }
      if (alternatives.size() != 16) throw std::runtime_error("candidate count is not 17 (1+16)");
      for (const std::size_t index : alternatives) alignCandidate(ndt, candidates[index]);

      std::vector<int> ranks(candidates.size(), -1);
      for (std::size_t rank = 0; rank < ranked.size(); ++rank) {
        ranks[ranked[rank]] = static_cast<int>(rank + 1);
      }
      std::vector<bool> sparse_top2(candidates.size(), false);
      int selected = 0;
      for (const std::size_t index : alternatives) {
        if (selected == 2) break;
        sparse_top2[index] = true;
        ++selected;
      }
      // Sparse candidates are scored first in the full comparator, so their align time is
      // directly measured once and reused for both offline method evaluations.
      const std::string yaw_base_id = yaw_base.id;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        const Candidate& candidate = candidates[i];
        const bool is_top2 = sparse_top2[i];
        output << std::setprecision(12) << event_id << '\t' << frame_index << '\t' << time_s << '\t'
               << stamp_s << '\t' << runtime_raw_fitness << '\t' << source->size() << '\t'
               << candidate.id << '\t' << candidate.kind << '\t' << candidate.dx_body << '\t'
               << candidate.dy_body << '\t' << candidate.yaw_deg << '\t' << (candidate.center ? 1 : 0)
               << '\t' << candidate.score << '\t' << candidate.score_ms << '\t' << ranks[i] << '\t'
               << (is_top2 ? 1 : 0) << '\t' << baseline.align_ms << '\t' << baseline.converged << '\t'
               << baseline.iterations << '\t' << baseline.fitness << '\t';
        writePose(output, baseline.final_pose);
        output << '\t' << candidate.align_ms << '\t' << candidate.converged << '\t'
               << candidate.iterations << '\t' << candidate.fitness << '\t';
        writePose(output, candidate.final_pose);
        output << '\t' << probe_total_ms << '\t' << translation_probe_ms << '\t'
               << yaw_base_id << '\n';
        if (!output) throw std::runtime_error("failed writing helper output");
      }
      std::cerr << "event=" << event_id << " t=" << time_s << "s score_ms=" << probe_total_ms
                << " base_ms=" << baseline.align_ms << " sparse_top2_ms="
                << candidates[alternatives[0]].align_ms + candidates[alternatives[1]].align_ms
                << " alternatives=16\n";
      ++event_count;
    }
    std::cerr << "P4_I1_HELPER_PASS events=" << event_count << " target_points=" << target->size() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P4-I1 helper error: " << error.what() << '\n';
    return 1;
  }
}
