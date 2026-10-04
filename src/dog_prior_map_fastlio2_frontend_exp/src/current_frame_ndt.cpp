#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
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
  std::array<float, 3> targetGridLeafSizeMeters() const {
    const Eigen::Vector3f size = this->target_cells_.getLeafSize();
    return {{size.x(), size.y(), size.z()}};
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

double quantile(const std::vector<double>& sorted, double fraction) {
  if (sorted.empty()) return std::numeric_limits<double>::quiet_NaN();
  const double index = fraction * static_cast<double>(sorted.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lower);
  return (1.0 - alpha) * sorted[lower] + alpha * sorted[upper];
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
  pcl::KdTreeFLANN<Point>::Ptr target_tree;
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
    impl_->target_tree.reset(new pcl::KdTreeFLANN<Point>());
    impl_->target_tree->setInputCloud(target);
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("map_internal_error:") + error.what());
  }
}

bool CurrentFrameNdtRegistration::ready() const { return static_cast<bool>(impl_->target); }
std::size_t CurrentFrameNdtRegistration::targetPointCount() const {
  return ready() ? impl_->target->size() : 0;
}
std::array<float, 3> CurrentFrameNdtRegistration::targetGridLeafSizeMeters() const {
  return ready() ? impl_->ndt.targetGridLeafSizeMeters()
                 : std::array<float, 3>{{0.0f, 0.0f, 0.0f}};
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
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("ndt_internal_error:") + error.what());
  }
}

bool CurrentFrameNdtRegistration::evaluateMapOverlap(
    const RegistrationCloud& raw_cloud, const Pose3d& map_T_lidar,
    CurrentFrameNdtOverlap* result, std::string* reason) const {
  if (reason) reason->clear();
  if (!result) return fail(reason, "null_overlap_result");
  if (!ready() || !impl_->target_tree) return fail(reason, "map_not_loaded");
  if (!map_T_lidar.position.allFinite() || !map_T_lidar.orientation.coeffs().allFinite() ||
      std::abs(map_T_lidar.orientation.norm() - 1.0) > 1e-6)
    return fail(reason, "invalid_overlap_pose");
  CurrentFrameNdtOverlap pending;
  const RegistrationCloud prepared = preprocessRegistrationCloud(raw_cloud, impl_->parameters);
  if (prepared.empty()) return fail(reason, "empty_overlap_source");
  pending.evaluated_points = prepared.size();
  std::vector<double> distances;
  distances.reserve(prepared.size());
  std::array<std::size_t, 4> counts{{0, 0, 0, 0}};
  const std::array<double, 4> thresholds{{0.2, 0.3, 0.5, 1.0}};
  const Eigen::Matrix3d rotation = map_T_lidar.orientation.toRotationMatrix();
  std::vector<int> nearest_index(1);
  std::vector<float> squared_distance(1);
  double distance_sum = 0.0;
  for (const RegistrationPoint& point : prepared) {
    const Eigen::Vector3d map_point = rotation * Eigen::Vector3d(point.x, point.y, point.z) +
        map_T_lidar.position;
    const Point query(static_cast<float>(map_point.x()),
                      static_cast<float>(map_point.y()),
                      static_cast<float>(map_point.z()));
    if (impl_->target_tree->nearestKSearch(query, 1, nearest_index, squared_distance) != 1 ||
        !std::isfinite(squared_distance.front()) || squared_distance.front() < 0.0f)
      return fail(reason, "map_overlap_nearest_neighbor_failed");
    const double distance = std::sqrt(static_cast<double>(squared_distance.front()));
    distances.push_back(distance);
    distance_sum += distance;
    for (std::size_t index = 0; index < thresholds.size(); ++index)
      if (distance < thresholds[index]) ++counts[index];
  }
  std::sort(distances.begin(), distances.end());
  for (std::size_t index = 0; index < counts.size(); ++index)
    pending.fraction_within[index] =
        static_cast<double>(counts[index]) / static_cast<double>(prepared.size());
  pending.mean_nearest_distance_m = distance_sum / static_cast<double>(prepared.size());
  pending.median_nearest_distance_m = quantile(distances, 0.50);
  pending.p90_nearest_distance_m = quantile(distances, 0.90);
  pending.p95_nearest_distance_m = quantile(distances, 0.95);
  *result = pending;
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
