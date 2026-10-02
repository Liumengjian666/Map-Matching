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
#include <stdexcept>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
class ObservableNdt : public pcl::NormalDistributionsTransform<Point, Point> {
 public:
  std::vector<reliability::GeometricObservation> geometricObservations(
      const Cloud& source, const Pose3d& map_T_lidar, double resolution) {
    std::vector<reliability::GeometricObservation> observations;
    const Eigen::Matrix3d rotation = map_T_lidar.orientation.toRotationMatrix();
    std::vector<TargetGridLeafConstPtr> leaves;
    std::vector<float> squared_distances;
    for (const Point& point : source) {
      const Eigen::Vector3d rotated = rotation * Eigen::Vector3d(point.x, point.y, point.z);
      const Eigen::Vector3d query = rotated + map_T_lidar.position;
      const Point query_point(static_cast<float>(query.x()),
          static_cast<float>(query.y()), static_cast<float>(query.z()));
      // Explicitly clear both outputs for every query, including no-neighbor queries.
      leaves.clear();
      squared_distances.clear();
      this->target_cells_.radiusSearch(query_point, resolution, leaves, squared_distances);
      const std::size_t count = std::min(leaves.size(), squared_distances.size());
      for (std::size_t i = 0; i < count; ++i) {
        if (!leaves[i] || !std::isfinite(squared_distances[i]) || squared_distances[i] < 0.0f)
          continue;
        reliability::GeometricObservation observation;
        observation.rotated_source_map = rotated;
        observation.residual_map = query - leaves[i]->getMean();
        observation.voxel_covariance_map = leaves[i]->getCov();
        observation.nonnegative_weight = std::exp(-0.5 *
            static_cast<double>(squared_distances[i]) / (resolution * resolution));
        observations.push_back(std::move(observation));
      }
    }
    return observations;
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
    // PCL 1.10 builds target cells in setInputTarget; configure their size first.
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
std::size_t CurrentFrameNdtRegistration::targetPointCount() const {
  return ready() ? impl_->target->size() : 0;
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
    if (result->effective) {
      const auto uobs_started = std::chrono::steady_clock::now();
      const auto observations = impl_->ndt.geometricObservations(
          *source, result->raw_map_T_lidar, impl_->parameters.resolution_m);
      result->local_observability = reliability::analyzeGeometricObservability(
          observations, true, impl_->parameters.resolution_m);
      result->uobs_computed = true;
      result->uobs_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - uobs_started).count();
    }
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("ndt_internal_error:") + error.what());
  }
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
