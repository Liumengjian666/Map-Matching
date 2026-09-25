#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <memory>
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp {

// Single-worker transaction owner shared by the ROS executable and the
// end-to-end runtime contract test. Callers must serialize all methods.
class FrontendRuntime {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FrontendRuntime(const RuntimeParameters& parameters);

  bool initializeStatic(
      const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
      const Pose3d& initial_map_T_lidar, const Pose3d& T_imu_lidar,
      std::string* failure_reason);
  bool beginScan(
      uint64_t transaction_id, uint64_t scan_start_ns, uint64_t scan_end_ns,
      const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
      const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
      ScanEndResult* output, std::string* failure_reason);
  bool finishScan(uint64_t transaction_id, RuntimeDisposition disposition,
                  bool pose_valid, const Pose3d& used_map_T_lidar,
                  PoseCorrectionDelta* update_delta,
                  std::string* failure_reason);

  bool initialized() const;
  bool hasPendingScan() const;
  bool fatal() const;
  uint64_t pendingTransactionId() const;
  FilterSnapshot committedState() const;
  RuntimeCounters counters() const;

 private:
  RuntimeParameters parameters_;
  FastLio2IkfomFrontend committed_;
  ScanEndProcessor scan_processor_;
  std::unique_ptr<FastLio2IkfomFrontend> candidate_;
  Pose3d T_imu_lidar_;
  ScanEndResult pending_output_;
  RuntimeCounters counters_;
  uint64_t pending_transaction_id_ = 0;
  bool initialized_ = false;
  bool fatal_ = false;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
