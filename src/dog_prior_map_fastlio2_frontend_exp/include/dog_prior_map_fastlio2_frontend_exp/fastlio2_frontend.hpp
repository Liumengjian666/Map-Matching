#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

class FastLio2IkfomFrontend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FastLio2IkfomFrontend(const RuntimeParameters& parameters);
  ~FastLio2IkfomFrontend();

  FastLio2IkfomFrontend(const FastLio2IkfomFrontend&) = delete;
  FastLio2IkfomFrontend& operator=(const FastLio2IkfomFrontend&) = delete;

  bool initializeStatic(
      const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
      const Pose3d& initial_map_T_lidar,
      const Pose3d& T_imu_lidar,
      std::string* failure_reason);

  bool predictInterval(const ImuSample& head, const ImuSample& tail,
                       std::string* failure_reason);
  bool predictImuSequence(
      const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
      uint64_t exact_end_stamp_ns,
      std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>* poses,
      std::string* failure_reason);
  bool predictHeldInputTo(uint64_t end_stamp_ns, const ImuSample& head,
                          const ImuSample& tail,
                          std::string* failure_reason);
  bool applyPoseMeasurement(const Pose3d& map_T_imu_measurement,
                            PoseCorrectionDelta* delta,
                            std::string* failure_reason);

  std::unique_ptr<FastLio2IkfomFrontend> cloneCandidate() const;
  bool commitCandidate(const FastLio2IkfomFrontend& candidate,
                       std::string* failure_reason);
  bool rejectCandidatePredictionOnly(
      const FastLio2IkfomFrontend& candidate,
      std::string* failure_reason);
  void reset();

  bool initialized() const;
  FilterSnapshot getState() const;
  Eigen::Matrix<double, 12, 12> getProcessNoiseCovariance() const;
  bool postconditionsValid(std::string* failure_reason) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
