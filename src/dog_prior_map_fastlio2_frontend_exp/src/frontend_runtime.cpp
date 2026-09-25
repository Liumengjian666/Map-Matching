#include "dog_prior_map_fastlio2_frontend_exp/frontend_runtime.hpp"

#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

Eigen::Isometry3d toIsometry(const Pose3d& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.orientation.toRotationMatrix();
  transform.translation() = pose.position;
  return transform;
}

Pose3d fromIsometry(const Eigen::Isometry3d& transform) {
  Pose3d pose;
  pose.position = transform.translation();
  pose.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return pose;
}

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

}  // namespace

FrontendRuntime::FrontendRuntime(const RuntimeParameters& parameters)
    : parameters_(parameters), committed_(parameters) {}

bool FrontendRuntime::initializeStatic(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    const Pose3d& initial_map_T_lidar, const Pose3d& T_imu_lidar,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (fatal_) return fail(failure_reason, "frontend_is_fatal");
  if (initialized_) return fail(failure_reason, "frontend_already_initialized");
  if (!committed_.initializeStatic(imu, initial_map_T_lidar, T_imu_lidar,
                                   failure_reason))
    return false;
  T_imu_lidar_ = T_imu_lidar;
  initialized_ = true;
  return true;
}

bool FrontendRuntime::beginScan(
    uint64_t transaction_id, uint64_t scan_start_ns, uint64_t scan_end_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
    ScanEndResult* output, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!initialized_) return fail(failure_reason, "frontend_not_initialized");
  if (fatal_) return fail(failure_reason, "frontend_is_fatal");
  if (candidate_) return fail(failure_reason, "scan_transaction_already_pending");
  if (!output || transaction_id == 0 ||
      transaction_id != counters_.last_committed_transaction + 1)
    return fail(failure_reason, "invalid_or_nonsequential_transaction_id");

  std::unique_ptr<FastLio2IkfomFrontend> candidate = committed_.cloneCandidate();
  ScanEndResult result;
  if (!scan_processor_.process(candidate.get(), T_imu_lidar_, scan_start_ns,
                               scan_end_ns, imu, cloud, &result,
                               failure_reason))
    return false;
  candidate_ = std::move(candidate);
  pending_transaction_id_ = transaction_id;
  pending_output_ = result;
  *output = std::move(result);
  return true;
}

bool FrontendRuntime::finishScan(uint64_t transaction_id,
                                 RuntimeDisposition disposition,
                                 bool pose_valid,
                                 const Pose3d& used_map_T_lidar,
                                 PoseCorrectionDelta* update_delta,
                                 std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!candidate_ || transaction_id != pending_transaction_id_) {
    fatal_ = true;
    candidate_.reset();
    pending_transaction_id_ = 0;
    return fail(failure_reason, "terminal_result_does_not_match_pending_transaction");
  }

  if (disposition == RuntimeDisposition::SUCCESS) {
    if (!pose_valid) {
      fatal_ = true;
      candidate_.reset();
      pending_transaction_id_ = 0;
      return fail(failure_reason, "success_result_has_invalid_pose_flag");
    }
    const Eigen::Isometry3d map_T_imu =
        toIsometry(used_map_T_lidar) * toIsometry(T_imu_lidar_).inverse();
    std::unique_ptr<FastLio2IkfomFrontend> shadow = candidate_->cloneCandidate();
    PoseCorrectionDelta delta;
    if (!shadow->applyPoseMeasurement(fromIsometry(map_T_imu), &delta,
                                      failure_reason) ||
        shadow->getState().stamp_ns != pending_output_.scan_end_ns ||
        !shadow->postconditionsValid(failure_reason)) {
      fatal_ = true;
      candidate_.reset();
      pending_transaction_id_ = 0;
      return false;
    }
    if (!committed_.commitCandidate(*shadow, failure_reason)) {
      fatal_ = true;
      candidate_.reset();
      pending_transaction_id_ = 0;
      return false;
    }
    if (update_delta) *update_delta = delta;
    ++counters_.measurement_updates;
  } else if (disposition == RuntimeDisposition::REJECT_INSUFFICIENT_POINTS ||
             disposition == RuntimeDisposition::REJECT_NOT_CONVERGED) {
    if (pose_valid) {
      fatal_ = true;
      candidate_.reset();
      pending_transaction_id_ = 0;
      return fail(failure_reason, "non_success_result_has_valid_pose_flag");
    }
    if (!committed_.rejectCandidatePredictionOnly(*candidate_, failure_reason)) {
      fatal_ = true;
      candidate_.reset();
      pending_transaction_id_ = 0;
      return false;
    }
    ++counters_.prediction_only_commits;
  } else {
    fatal_ = true;
    candidate_.reset();
    pending_transaction_id_ = 0;
    return fail(failure_reason, "fatal_ndt_terminal_disposition");
  }

  if (committed_.getState().stamp_ns != pending_output_.scan_end_ns) {
    fatal_ = true;
    candidate_.reset();
    pending_transaction_id_ = 0;
    return fail(failure_reason, "committed_timestamp_differs_from_scan_end");
  }
  counters_.last_committed_transaction = transaction_id;
  candidate_.reset();
  pending_transaction_id_ = 0;
  return true;
}

bool FrontendRuntime::initialized() const { return initialized_; }
bool FrontendRuntime::hasPendingScan() const { return candidate_ != nullptr; }
bool FrontendRuntime::fatal() const { return fatal_; }
uint64_t FrontendRuntime::pendingTransactionId() const {
  return pending_transaction_id_;
}
FilterSnapshot FrontendRuntime::committedState() const {
  return committed_.getState();
}
RuntimeCounters FrontendRuntime::counters() const { return counters_; }

}  // namespace dog_prior_map_fastlio2_frontend_exp
