#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

struct ProjectedPoseInnovation {
  bool valid = false;
  int rank = 0;
  double nis = std::numeric_limits<double>::quiet_NaN();
  double threshold = std::numeric_limits<double>::quiet_NaN();
  double residual_norm = 0.0;
  double projected_residual_norm = 0.0;
  double projected_noise_trace = 0.0;
  double innovation_covariance_trace = 0.0;
  double projected_noise_min_eigenvalue = 0.0;
  double projected_noise_max_eigenvalue = 0.0;
  std::string status = "UNINITIALIZED";
};

enum class ProjectedPoseLinearizationMode {
  LEGACY_IDENTITY_ROTATION,
  EXACT_LOG_RESIDUAL,
};


double chiSquare99Threshold(int rank);


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

  bool initializeMoving(const MovingInitializationState& initial,
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
  // Adaptive measurement covariance order follows PoseMeasurement's MTK
  // fields: [position XYZ, SO(3) residual]. The SO(3) block is in the filter's
  // right/body error coordinates. The original isotropic API remains intact.
  bool applyPoseMeasurement(
      const Pose3d& map_T_imu_measurement,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance,
      PoseCorrectionDelta* delta, std::string* failure_reason);
  // Linear map-frame XYZ observation. The update uses the complete IKFoM
  // covariance (including position cross-covariances with velocity/bias),
  // applies the manifold covariance reset, and rejects non-SPD noise.
  bool applyPositionMeasurement(
      const Eigen::Vector3d& map_position_measurement,
      const Eigen::Matrix3d& measurement_covariance,
      PoseCorrectionDelta* delta, std::string* failure_reason);
  bool applyProjectedPoseMeasurement(
      const Pose3d& map_T_imu_measurement,
      const Eigen::Matrix<double, 6, 6>& measurement_covariance,
      const Eigen::Matrix<double, 6, 6>& measurement_basis,
      int measurement_rank, PoseCorrectionDelta* delta,
      std::string* failure_reason);
  bool evaluateProjectedPoseInnovation(
      const Pose3d& map_T_imu_measurement,
      const Eigen::Matrix<double, 6, 6>& measurement_noise,
      const Eigen::Matrix<double, 6, 6>& measurement_basis,
      int rank, ProjectedPoseInnovation* output,
      std::string* reason) const;
  bool evaluateProjectedPoseInnovationLinearized(
      const Pose3d& map_T_imu_measurement,
      const Eigen::Matrix<double, 6, 6>& measurement_noise,
      const Eigen::Matrix<double, 6, 6>& measurement_basis,
      int rank, ProjectedPoseLinearizationMode linearization_mode,
      ProjectedPoseInnovation* output, std::string* reason) const;
  bool applyProjectedPoseMeasurementChecked(
      const Pose3d& map_T_imu_measurement,
      const Eigen::Matrix<double, 6, 6>& measurement_noise,
      const Eigen::Matrix<double, 6, 6>& measurement_basis,
      int rank, bool enforce_nis_gate, double nis_threshold,
      ProjectedPoseInnovation* diagnostic, PoseCorrectionDelta* delta,
      std::string* reason);
  bool applyProjectedPoseMeasurementLinearizedChecked(
      const Pose3d& map_T_imu_measurement,
      const Eigen::Matrix<double, 6, 6>& measurement_noise,
      const Eigen::Matrix<double, 6, 6>& measurement_basis,
      int rank, ProjectedPoseLinearizationMode linearization_mode,
      bool enforce_nis_gate, double nis_threshold,
      ProjectedPoseInnovation* diagnostic, PoseCorrectionDelta* delta,
      std::string* reason);
  // Applies only the position residual components spanned by the first
  // `measurement_rank` orthonormal map-frame basis columns. This is used to
  // keep visual corrections inside the LiDAR-weak measurement row-space.
  bool applyProjectedPositionMeasurement(
      const Eigen::Vector3d& map_position_measurement,
      const Eigen::Matrix3d& measurement_covariance,
      const Eigen::Matrix3d& measurement_basis, int measurement_rank,
      PoseCorrectionDelta* delta, std::string* failure_reason);

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
