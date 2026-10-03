#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
class ObservableNdt : public pcl::NormalDistributionsTransform<Point, Point> {
 public:
  using LeafPtr = TargetGridLeafConstPtr;

  std::array<float, 3> targetGridLeafSizeMeters() const {
    const Eigen::Vector3f size = this->target_cells_.getLeafSize();
    return {{size.x(), size.y(), size.z()}};
  }

  bool scoreJetAt(const Cloud::Ptr& source, const Pose3d& pose,
                  PclNdtScoreJet* output) {
    if (!source || !output) return false;
    const Eigen::Matrix4f transform = poseMatrix(pose);
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> affine;
    affine.matrix() = transform;
    const Eigen::Vector3f angles = affine.rotation().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, 6, 1> p;
    p << affine.translation().x(), affine.translation().y(), affine.translation().z(),
         angles.x(), angles.y(), angles.z();
    Eigen::Matrix<double, 6, 1> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    const double score = this->computeDerivatives(gradient, hessian, transformed, p, true);
    output->score_sum = score;
    output->source_point_count = source->size();
    output->pcl_euler_xyz = angles.cast<double>();
    output->score_gradient = gradient;
    output->score_hessian = hessian;
    output->valid = std::isfinite(score) && gradient.allFinite() && hessian.allFinite() &&
        output->pcl_euler_xyz.allFinite() && source->size() > 0;
    output->status = output->valid ? "PASS_PCL_SCORE_JET" : "NONFINITE_PCL_SCORE_JET";
    return output->valid;
  }

  bool objectiveAt(const Cloud::Ptr& source, const Pose3d& pose,
                   NdtObjectiveSample* output) {
    if (!source || !output) return false;
    PclNdtScoreJet jet;
    if (!scoreJetAt(source, pose, &jet)) {
      output->status = jet.status;
      return false;
    }
    const Eigen::Matrix4f transform = poseMatrix(pose);
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    std::uint64_t hash = 1469598103934665603ULL;
    std::uint64_t cell_count = 0;
    const auto mix = [&hash](std::uint64_t value) {
      for (int shift = 0; shift < 64; shift += 8)
        hash = (hash ^ static_cast<std::uint8_t>(value >> shift)) * 1099511628211ULL;
    };
    for (const Point& point : transformed.points) {
      std::vector<TargetGridLeafConstPtr> cells;
      std::vector<float> distances;
      this->target_cells_.radiusSearch(point, this->resolution_, cells, distances);
      std::vector<std::uintptr_t> identities;
      identities.reserve(cells.size());
      for (const auto& cell : cells)
        identities.push_back(reinterpret_cast<std::uintptr_t>(cell));
      std::sort(identities.begin(), identities.end());
      mix(static_cast<std::uint64_t>(identities.size()));
      for (std::uintptr_t identity : identities) mix(static_cast<std::uint64_t>(identity));
      cell_count += identities.size();
    }
    output->score_sum = jet.score_sum;
    output->source_point_count = jet.source_point_count;
    output->target_neighborhood_hash = hash;
    output->target_neighborhood_cell_count = cell_count;
    output->valid = std::isfinite(output->score_sum) && output->source_point_count > 0;
    output->status = output->valid ? "PASS_FIXED_POSE_NDT_OBJECTIVE" :
                                     "INVALID_FIXED_POSE_NDT_OBJECTIVE";
    return output->valid;
  }

  bool captureSupport(const Cloud::Ptr& source, const Pose3d& pose,
      std::vector<std::vector<LeafPtr>>* support, std::uint64_t* membership_count) {
    if (!source || !support || !membership_count) return false;
    const Eigen::Matrix4f transform = poseMatrix(pose);
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    support->clear();
    support->resize(transformed.size());
    *membership_count = 0;
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      std::vector<float> distances;
      this->target_cells_.radiusSearch(transformed.points[i], this->resolution_,
                                       (*support)[i], distances);
      *membership_count += (*support)[i].size();
    }
    return true;
  }

  bool frozenScore(const Cloud::Ptr& source, const Pose3d& pose,
      const std::vector<std::vector<LeafPtr>>& support,
      std::uint64_t center_memberships, NdtFrozenObjectiveSample* output) const {
    if (!source || !output || support.size() != source->size()) return false;
    const Eigen::Matrix4f transform = poseMatrix(pose);
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    double score = 0.0;
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      const Point& point = transformed.points[i];
      const Eigen::Vector3d x_trans(point.x, point.y, point.z);
      for (const LeafPtr& cell : support[i]) {
        const Eigen::Vector3d residual = x_trans - cell->getMean();
        const Eigen::Matrix3d inverse_covariance = cell->getInverseCov();
        const double exponential = std::exp(-this->gauss_d2_ *
            residual.dot(inverse_covariance * residual) / 2.0);
        const double derivative_guard = this->gauss_d2_ * exponential;
        // Match updateDerivatives()' invalid-contribution behavior.
        if (!std::isfinite(derivative_guard) || derivative_guard > 1.0 ||
            derivative_guard < 0.0) continue;
        score += -this->gauss_d1_ * exponential;
      }
    }
    output->score_sum = score;
    output->source_point_count = source->size();
    output->gaussian_membership_count = center_memberships;
    output->valid = std::isfinite(score) && !source->empty();
    output->status = output->valid ? "PASS_FROZEN_ACTIVE_SUPPORT" :
                                     "INVALID_FROZEN_ACTIVE_SUPPORT";
    return output->valid;
  }

  bool dynamicSupport(const Cloud::Ptr& source, const Pose3d& pose,
      const std::vector<std::vector<LeafPtr>>& center_support,
      std::uint64_t center_memberships, NdtObjectiveSample* sample,
      NdtSupportChangeDiagnostic* diagnostic) {
    if (!source || !sample || !diagnostic || center_support.size() != source->size())
      return false;
    PclNdtScoreJet jet;
    if (!scoreJetAt(source, pose, &jet)) return false;
    const Eigen::Matrix4f transform = poseMatrix(pose);
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    std::uint64_t perturbed_memberships = 0;
    std::uint64_t changed_points = 0;
    std::uint64_t symmetric_difference = 0;
    for (std::size_t i = 0; i < transformed.size(); ++i) {
      std::vector<LeafPtr> cells;
      std::vector<float> distances;
      this->target_cells_.radiusSearch(transformed.points[i], this->resolution_, cells, distances);
      perturbed_memberships += cells.size();
      std::vector<std::uintptr_t> center_ids, perturbed_ids;
      center_ids.reserve(center_support[i].size());
      perturbed_ids.reserve(cells.size());
      for (const LeafPtr& cell : center_support[i])
        center_ids.push_back(reinterpret_cast<std::uintptr_t>(cell));
      for (const LeafPtr& cell : cells)
        perturbed_ids.push_back(reinterpret_cast<std::uintptr_t>(cell));
      std::sort(center_ids.begin(), center_ids.end());
      std::sort(perturbed_ids.begin(), perturbed_ids.end());
      if (center_ids != perturbed_ids) ++changed_points;
      std::vector<std::uintptr_t> difference;
      std::set_symmetric_difference(center_ids.begin(), center_ids.end(),
          perturbed_ids.begin(), perturbed_ids.end(), std::back_inserter(difference));
      symmetric_difference += difference.size();
    }
    sample->score_sum = jet.score_sum;
    sample->source_point_count = jet.source_point_count;
    sample->target_neighborhood_cell_count = perturbed_memberships;
    sample->valid = jet.valid;
    sample->status = jet.valid ? "PASS_DYNAMIC_ACTIVE_SUPPORT" : jet.status;
    diagnostic->source_point_count = source->size();
    diagnostic->changed_source_point_count = changed_points;
    diagnostic->changed_source_point_fraction = source->empty() ?
        std::numeric_limits<double>::quiet_NaN() :
        static_cast<double>(changed_points) / static_cast<double>(source->size());
    diagnostic->center_gaussian_membership_count = center_memberships;
    diagnostic->perturbed_gaussian_membership_count = perturbed_memberships;
    diagnostic->membership_symmetric_difference_count = symmetric_difference;
    diagnostic->valid = sample->valid && source->size() > 0;
    diagnostic->status = diagnostic->valid ? "PASS_QUANTIFIED_SUPPORT_CHANGE" :
                                             "INVALID_DYNAMIC_SUPPORT";
    return diagnostic->valid;
  }

 private:
  static Eigen::Matrix4f poseMatrix(const Pose3d& pose) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    const Eigen::Quaterniond q = pose.orientation.normalized();
    matrix.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    matrix.block<3, 1>(0, 3) = pose.position.cast<float>();
    return matrix;
  }
};

void finalize(const Cloud::Ptr& cloud) {
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf) {
  Cloud::Ptr down(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                    static_cast<float>(leaf));
  voxel.setInputCloud(input);
  voxel.filter(*down);
  finalize(down);
  return down;
}

bool fail(std::string* reason, const std::string& message) {
  if (reason) *reason = message;
  return false;
}

bool parametersValid(const CurrentFrameNdtParameters& p) {
  const double positive[] = {p.map_voxel_m, p.target_voxel_m, p.source_voxel_m,
      p.max_range_m, p.resolution_m, p.step_size, p.transformation_epsilon};
  for (double value : positive)
    if (!std::isfinite(value) || value <= 0.0) return false;
  return std::isfinite(p.min_range_m) && p.min_range_m >= 0.0 &&
      p.min_range_m < p.max_range_m && p.max_source_points > 0 &&
      p.min_effective_points > 0 && p.maximum_iterations > 0;
}
}  // namespace

struct NdtFrozenSupportSnapshot::Impl {
  const void* owner = nullptr;
  Cloud::Ptr source;
  std::vector<std::vector<ObservableNdt::LeafPtr>> support;
  std::uint64_t membership_count = 0;
};

CurrentFrameNdtStatus classifyNdtTerminal(bool converged, int iterations,
    int maximum_iterations, bool terminal_pose_finite, bool fitness_finite) {
  if (!terminal_pose_finite || !fitness_finite)
    return CurrentFrameNdtStatus::NONFINITE_TERMINAL;
  if (!converged) return CurrentFrameNdtStatus::NOT_CONVERGED;
  if (iterations <= 0) return CurrentFrameNdtStatus::ZERO_ITERATION_PASSTHROUGH;
  if (iterations >= maximum_iterations)
    return CurrentFrameNdtStatus::ITERATION_LIMIT_EXHAUSTED;
  return CurrentFrameNdtStatus::SUCCESS;
}

const char* currentFrameNdtStatusName(CurrentFrameNdtStatus status) {
  switch (status) {
    case CurrentFrameNdtStatus::SUCCESS: return "SUCCESS";
    case CurrentFrameNdtStatus::INSUFFICIENT_POINTS: return "INSUFFICIENT_POINTS";
    case CurrentFrameNdtStatus::NOT_CONVERGED: return "NOT_CONVERGED";
    case CurrentFrameNdtStatus::ZERO_ITERATION_PASSTHROUGH: return "ZERO_ITERATION_PASSTHROUGH";
    case CurrentFrameNdtStatus::ITERATION_LIMIT_EXHAUSTED: return "ITERATION_LIMIT_EXHAUSTED";
    case CurrentFrameNdtStatus::NONFINITE_TERMINAL: return "NONFINITE_TERMINAL";
  }
  return "INVALID_STATUS";
}

RegistrationCloud preprocessRegistrationCloud(const RegistrationCloud& raw,
    const CurrentFrameNdtParameters& p) {
  if (!parametersValid(p)) throw std::invalid_argument("invalid_ndt_parameters");
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw.size());
  for (const RegistrationPoint& point : raw) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    // Preserve the formal float arithmetic before assigning the range to double.
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range < p.min_range_m || range > p.max_range_m) continue;
    finite->push_back(Point(point.x, point.y, point.z));
  }
  finalize(finite);
  const Cloud::Ptr down = voxelDown(finite, p.source_voxel_m);
  RegistrationCloud prepared;
  const std::size_t cap = static_cast<std::size_t>(p.max_source_points);
  if (down->size() > cap) {
    prepared.reserve(cap);
    const double increment = static_cast<double>(down->size() - 1) /
        static_cast<double>(std::max(p.max_source_points - 1, 1));
    for (int i = 0; i < p.max_source_points; ++i) {
      const Point& point = down->points[static_cast<std::size_t>(std::llround(i * increment))];
      prepared.push_back({point.x, point.y, point.z});
    }
  } else {
    prepared.reserve(down->size());
    for (const Point& point : down->points) prepared.push_back({point.x, point.y, point.z});
  }
  return prepared;
}

uint64_t registrationCloudHash(const RegistrationCloud& prepared) {
  uint64_t hash = 1469598103934665603ULL;
  auto mix = [&hash](uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8)
      hash = (hash ^ static_cast<uint8_t>(value >> shift)) * 1099511628211ULL;
  };
  mix(static_cast<uint32_t>(prepared.size()));  // width
  mix(1);  // height
  mix(1);  // is_dense after finite filtering
  mix(static_cast<uint32_t>(prepared.size()));
  for (const RegistrationPoint& point : prepared) {
    for (float value : {point.x, point.y, point.z}) {
      uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      mix(bits);
    }
  }
  return hash;
}

struct CurrentFrameNdtRegistration::Impl {
  explicit Impl(const CurrentFrameNdtParameters& p) : parameters(p) {
    if (!parametersValid(p)) throw std::invalid_argument("invalid_ndt_parameters");
  }
  CurrentFrameNdtParameters parameters;
  Cloud::Ptr target;
  Cloud::Ptr source;
  ObservableNdt ndt;
};

CurrentFrameNdtRegistration::CurrentFrameNdtRegistration(const CurrentFrameNdtParameters& p)
    : impl_(new Impl(p)) {}
CurrentFrameNdtRegistration::~CurrentFrameNdtRegistration() = default;

bool CurrentFrameNdtRegistration::loadMap(const std::string& path, std::string* reason) {
  if (reason) reason->clear();
  if (ready()) return fail(reason, "map_already_loaded");
  try {
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<Point>(path, *raw) != 0) return fail(reason, "cannot_load_map_pcd");
    Cloud::Ptr finite(new Cloud);
    finite->reserve(raw->size());
    for (const Point& point : raw->points)
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        finite->push_back(point);
    finalize(finite);
    const Cloud::Ptr first = voxelDown(finite, impl_->parameters.map_voxel_m);
    const Cloud::Ptr target = voxelDown(first, impl_->parameters.target_voxel_m);
    if (target->empty()) return fail(reason, "empty_preprocessed_target");
    // PCL 1.10 builds target_cells_ inside setInputTarget(), using the
    // resolution configured at that moment.
    impl_->ndt.setResolution(impl_->parameters.resolution_m);
    impl_->ndt.setInputTarget(target);
    impl_->ndt.setStepSize(impl_->parameters.step_size);
    impl_->ndt.setTransformationEpsilon(impl_->parameters.transformation_epsilon);
    impl_->ndt.setMaximumIterations(impl_->parameters.maximum_iterations);
    impl_->target = target;
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("map_internal_error:") + error.what());
  }
}

bool CurrentFrameNdtRegistration::ready() const { return static_cast<bool>(impl_->target); }
const CurrentFrameNdtParameters& CurrentFrameNdtRegistration::parameters() const {
  return impl_->parameters;
}
std::size_t CurrentFrameNdtRegistration::targetPointCount() const {
  return ready() ? impl_->target->size() : 0;
}
std::array<float, 3> CurrentFrameNdtRegistration::targetGridLeafSizeMeters() const {
  return ready() ? impl_->ndt.targetGridLeafSizeMeters()
                 : std::array<float, 3>{{0.0f, 0.0f, 0.0f}};
}

bool CurrentFrameNdtRegistration::evaluateLocalScoreJetAtPose(
    const Pose3d& pose, PclNdtScoreJet* result, std::string* reason) {
  if (reason) reason->clear();
  if (!result) return fail(reason, "null_score_jet");
  if (!ready()) return fail(reason, "map_not_loaded");
  if (!impl_->source) return fail(reason, "no_current_source_cloud");
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12)
    return fail(reason, "invalid_score_jet_pose");
  if (!impl_->ndt.scoreJetAt(impl_->source, pose, result))
    return fail(reason, result->status);
  return true;
}

bool CurrentFrameNdtRegistration::evaluateLocalObjectiveAtPose(
    const Pose3d& pose, NdtObjectiveSample* result, std::string* reason) {
  if (reason) reason->clear();
  if (!result) return fail(reason, "null_objective_sample");
  if (!ready()) return fail(reason, "map_not_loaded");
  if (!impl_->source) return fail(reason, "no_current_source_cloud");
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12)
    return fail(reason, "invalid_objective_pose");
  if (!impl_->ndt.objectiveAt(impl_->source, pose, result))
    return fail(reason, result->status);
  return true;
}

bool CurrentFrameNdtRegistration::captureFrozenSupport(
    const Pose3d& center_pose,
    std::shared_ptr<const NdtFrozenSupportSnapshot>* snapshot,
    NdtFrozenObjectiveSample* center_sample, std::string* reason) {
  if (reason) reason->clear();
  if (!snapshot || !center_sample) return fail(reason, "null_frozen_support_output");
  if (!ready()) return fail(reason, "map_not_loaded");
  if (!impl_->source) return fail(reason, "no_current_source_cloud");
  if (!center_pose.position.allFinite() || !center_pose.orientation.coeffs().allFinite() ||
      !std::isfinite(center_pose.orientation.norm()) || center_pose.orientation.norm() < 1e-12)
    return fail(reason, "invalid_frozen_support_center_pose");
  std::shared_ptr<NdtFrozenSupportSnapshot::Impl> frozen(
      new NdtFrozenSupportSnapshot::Impl());
  frozen->owner = &impl_->ndt;
  frozen->source = impl_->source;
  if (!impl_->ndt.captureSupport(frozen->source, center_pose,
          &frozen->support, &frozen->membership_count))
    return fail(reason, "frozen_support_capture_failed");
  NdtFrozenObjectiveSample frozen_center;
  if (!impl_->ndt.frozenScore(frozen->source, center_pose, frozen->support,
          frozen->membership_count, &frozen_center))
    return fail(reason, frozen_center.status);
  PclNdtScoreJet pcl_center;
  if (!impl_->ndt.scoreJetAt(frozen->source, center_pose, &pcl_center))
    return fail(reason, pcl_center.status);
  const double score_tolerance = 1e-11 *
      std::max({1.0, std::abs(frozen_center.score_sum), std::abs(pcl_center.score_sum)});
  if (std::abs(frozen_center.score_sum - pcl_center.score_sum) > score_tolerance)
    return fail(reason, "FROZEN_CENTER_SCORE_DIFFERS_FROM_PCL_RUNTIME_SCORE");
  *center_sample = frozen_center;
  std::shared_ptr<NdtFrozenSupportSnapshot> result(new NdtFrozenSupportSnapshot());
  result->impl = std::move(frozen);
  *snapshot = std::move(result);
  return true;
}

bool CurrentFrameNdtRegistration::evaluateFrozenSupportObjective(
    const NdtFrozenSupportSnapshot& snapshot, const Pose3d& pose,
    NdtFrozenObjectiveSample* sample, std::string* reason) {
  if (reason) reason->clear();
  if (!sample) return fail(reason, "null_frozen_objective_sample");
  if (!ready()) return fail(reason, "map_not_loaded");
  if (!snapshot.impl || snapshot.impl->owner != &impl_->ndt)
    return fail(reason, "frozen_support_snapshot_owner_mismatch");
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12)
    return fail(reason, "invalid_frozen_objective_pose");
  if (!impl_->ndt.frozenScore(snapshot.impl->source, pose,
          snapshot.impl->support, snapshot.impl->membership_count, sample))
    return fail(reason, sample->status);
  return true;
}

bool CurrentFrameNdtRegistration::evaluateDynamicSupportDiagnostic(
    const NdtFrozenSupportSnapshot& snapshot, const Pose3d& pose,
    NdtObjectiveSample* sample, NdtSupportChangeDiagnostic* diagnostic,
    std::string* reason) {
  if (reason) reason->clear();
  if (!sample || !diagnostic) return fail(reason, "null_dynamic_support_output");
  if (!ready()) return fail(reason, "map_not_loaded");
  if (!snapshot.impl || snapshot.impl->owner != &impl_->ndt)
    return fail(reason, "frozen_support_snapshot_owner_mismatch");
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12)
    return fail(reason, "invalid_dynamic_objective_pose");
  if (!impl_->ndt.dynamicSupport(snapshot.impl->source, pose,
          snapshot.impl->support, snapshot.impl->membership_count, sample, diagnostic))
    return fail(reason, diagnostic->status);
  return true;
}

bool CurrentFrameNdtRegistration::align(uint64_t stamp_ns, const RegistrationCloud& raw,
    const Pose3d& initial, CurrentFrameNdtResult* result, std::string* reason) {
  if (reason) reason->clear();
  if (!result) return fail(reason, "null_ndt_result");
  if (!ready()) return fail(reason, "map_not_loaded");
  if (!initial.position.allFinite() || !initial.orientation.coeffs().allFinite() ||
      !std::isfinite(initial.orientation.norm()) || initial.orientation.norm() < 1e-12)
    return fail(reason, "invalid_initial_pose");
  *result = CurrentFrameNdtResult();
  result->stamp_ns = stamp_ns;
  result->initial_map_T_lidar = initial;
  result->target_point_count = targetPointCount();
  try {
    const RegistrationCloud prepared = preprocessRegistrationCloud(raw, impl_->parameters);
    result->source_point_count = prepared.size();
    result->source_cloud_hash = registrationCloudHash(prepared);
    if (prepared.size() < static_cast<std::size_t>(impl_->parameters.min_effective_points)) {
      result->status = CurrentFrameNdtStatus::INSUFFICIENT_POINTS;
      result->alignment_ms = 0.0;
      return true;
    }
    Cloud::Ptr source(new Cloud);
    source->reserve(prepared.size());
    for (const RegistrationPoint& point : prepared) source->push_back(Point(point.x, point.y, point.z));
    finalize(source);
    impl_->source = source;
    impl_->ndt.setInputSource(source);
    Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
    guess.block<3, 3>(0, 0) = initial.orientation.toRotationMatrix();
    guess.block<3, 1>(0, 3) = initial.position;
    Cloud aligned;
    const auto started = std::chrono::steady_clock::now();
    impl_->ndt.align(aligned, guess.cast<float>());
    result->alignment_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    result->converged = impl_->ndt.hasConverged();
    result->iterations = impl_->ndt.getFinalNumIteration();
    result->fitness = impl_->ndt.getFitnessScore();
    result->transformation_probability = impl_->ndt.getTransformationProbability();
    const Eigen::Matrix4d raw_pose = impl_->ndt.getFinalTransformation().cast<double>();
    result->raw_map_T_lidar.position = raw_pose.block<3, 1>(0, 3);
    result->raw_map_T_lidar.orientation = Eigen::Quaterniond(raw_pose.block<3, 3>(0, 0)).normalized();
    result->status = classifyNdtTerminal(result->converged, result->iterations,
        impl_->parameters.maximum_iterations,
        raw_pose.allFinite() && result->raw_map_T_lidar.orientation.coeffs().allFinite(),
        std::isfinite(result->fitness));
    result->effective = result->status == CurrentFrameNdtStatus::SUCCESS;
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("ndt_internal_error:") + error.what());
  }
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
