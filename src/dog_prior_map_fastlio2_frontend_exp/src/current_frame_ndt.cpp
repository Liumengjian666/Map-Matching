#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using ObservableNdt = ObservablePclNdt<Point>;

Eigen::Matrix4f poseCarrier(const Pose3d& pose) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix();
  transform.block<3, 1>(0, 3) = pose.position;
  return transform.cast<float>();
}

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
  Cloud::Ptr source;
  ObservableNdt ndt;
  CurrentFrameNdtResult nominal;
  bool nominal_available = false;
  CoupledNdtBackend shadowBackend() {
    CoupledNdtBackend backend;
    backend.jet = [this](const Eigen::Matrix4f& pose, const CoupledVector6& native) {
      return ndt.nativeJet(source, pose, native);
    };
    backend.score = [this](const Eigen::Matrix4f& pose) {
      return ndt.dynamicScore(source, pose);
    };
    backend.refine = [this](const Eigen::Matrix4f& initial) {
      CoupledRefinement refined;
      Cloud aligned;
      ndt.align(aligned, initial);
      refined.iterations = ndt.getFinalNumIteration();
      refined.converged = ndt.hasConverged();
      const Eigen::Matrix4f raw = ndt.getFinalTransformation();
      Pose3d pose;
      pose.position = raw.block<3, 1>(0, 3).cast<double>();
      pose.orientation = Eigen::Quaterniond(raw.block<3, 3>(0, 0).cast<double>()).normalized();
      const bool finite = raw.allFinite() && pose.orientation.coeffs().allFinite();
      refined.pose = finite ? poseCarrier(pose) : raw;
      const auto status = classifyNdtTerminal(refined.converged, refined.iterations,
          parameters.maximum_iterations, finite, std::isfinite(ndt.getFitnessScore()));
      refined.status = currentFrameNdtStatusName(status);
      refined.successful = status == CurrentFrameNdtStatus::SUCCESS;
      return refined;
    };
    return backend;
  }
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
  impl_->nominal_available = false;
  impl_->source.reset();
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
    impl_->nominal = *result;
    impl_->nominal_available = true;
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("ndt_internal_error:") + error.what());
  }
}

bool CurrentFrameNdtRegistration::validateShadowNominal(const CurrentFrameNdtResult& nominal,
    std::string* reason) const {
  if (!ready() || !impl_->source || !impl_->nominal_available)
    return fail(reason, "shadow_requires_current_nominal_source");
  const auto& stored = impl_->nominal;
  if (!nominal.effective || nominal.status != CurrentFrameNdtStatus::SUCCESS ||
      !stored.effective || stored.status != CurrentFrameNdtStatus::SUCCESS)
    return fail(reason, "shadow_requires_effective_nominal");
  if (nominal.stamp_ns != stored.stamp_ns ||
      nominal.source_cloud_hash != stored.source_cloud_hash ||
      nominal.source_point_count != impl_->source->size() ||
      nominal.target_point_count != targetPointCount() ||
      nominal.raw_map_T_lidar.position != stored.raw_map_T_lidar.position ||
      nominal.raw_map_T_lidar.orientation.coeffs() != stored.raw_map_T_lidar.orientation.coeffs() ||
      nominal.initial_map_T_lidar.position != stored.initial_map_T_lidar.position ||
      nominal.initial_map_T_lidar.orientation.coeffs() != stored.initial_map_T_lidar.orientation.coeffs())
    return fail(reason, "shadow_nominal_source_identity_mismatch");
  return true;
}

bool CurrentFrameNdtRegistration::shadow(const CurrentFrameNdtResult& nominal,
    const CoupledNdtConfig& config, CoupledShadowResult* result, std::string* reason) {
  if (reason) reason->clear();
  if (!result) return fail(reason, "null_shadow_result");
  if (!validateShadowNominal(nominal, reason)) return false;
  try {
    *result = runCoupledNdtShadow(poseCarrier(nominal.raw_map_T_lidar),
        poseCarrier(nominal.initial_map_T_lidar), nominal.source_point_count, impl_->shadowBackend(), config);
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("shadow_internal_error:") + error.what());
  }
}

bool CurrentFrameNdtRegistration::eventShadow(const CurrentFrameNdtResult& nominal,
    const CoupledEventConfig& config, PendingCandidate* pending,
    CoupledEventResult* result, std::string* reason,
    const Eigen::Matrix4d* causal_imu_interval) {
  if (reason) reason->clear();
  if (!result || !pending) return fail(reason, "null_event_shadow_state");
  if (nominal.effective && !validateShadowNominal(nominal, reason)) return false;
  try {
    *result = runEventCoupledNdtShadow(poseCarrier(nominal.raw_map_T_lidar),
        poseCarrier(nominal.initial_map_T_lidar), nominal.stamp_ns, nominal.source_point_count,
        nominal.effective, impl_->shadowBackend(), config, pending, causal_imu_interval);
    if (nominal.status == CurrentFrameNdtStatus::INSUFFICIENT_POINTS)
      result->shadow.complete_ndt_calls = 0;
    return true;
  } catch (const std::exception& error) {
    return fail(reason, std::string("event_shadow_internal_error:") + error.what());
  }
}

bool CurrentFrameNdtRegistration::anchoredEventShadow(const CurrentFrameNdtResult& nominal,
    const CoupledEventConfig& config, PendingCandidate* pending, CoupledEventResult* result,
    std::string* reason, const Eigen::Matrix4d& imu, CoupledAnchorState* anchor, CoupledAnchorReceipt* receipt) {
  if (reason) reason->clear();
  if (!result || !pending || !anchor || !receipt) return fail(reason,"null_anchored_shadow_state");
  if (nominal.effective && !validateShadowNominal(nominal,reason)) return false;
  try {
    *result=runAnchoredCoupledNdtShadow(poseCarrier(nominal.raw_map_T_lidar),
        poseCarrier(nominal.initial_map_T_lidar),nominal.stamp_ns,nominal.source_point_count,
        nominal.effective,impl_->shadowBackend(),config,pending,imu,anchor,receipt);
    if (nominal.status==CurrentFrameNdtStatus::INSUFFICIENT_POINTS) result->shadow.complete_ndt_calls=0;
    return true;
  } catch (const std::exception& error) {
    return fail(reason,std::string("anchored_shadow_internal_error:")+error.what());
  }
}

bool CurrentFrameNdtRegistration::weakRefinement(const CurrentFrameNdtResult& nominal,
    const CoupledAnchorState& anchor,const WeakCoupledConfig& config,
    WeakCoupledResult* result,std::string* reason) {
  if(reason) reason->clear();
  if(!result) return fail(reason,"null_weak_refinement_result");
  if(nominal.effective && !validateShadowNominal(nominal,reason)) return false;
  try {
    *result=runWeakCoupledRefinement(poseCarrier(nominal.raw_map_T_lidar),
        poseCarrier(nominal.initial_map_T_lidar),nominal.stamp_ns,nominal.source_point_count,
        nominal.effective,anchor,impl_->shadowBackend(),config);
    return true;
  }catch(const std::exception& error) {
    return fail(reason,std::string("weak_refinement_internal_error:")+error.what());
  }
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
