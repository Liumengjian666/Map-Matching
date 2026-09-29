#pragma once

#include <Eigen/Core>

#include <string>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class MeasurementNoiseSemantic {
  PHYSICAL_LIDAR_PERTURBATION,
  EMPIRICAL_POSE_RESIDUAL,
};

const char* toString(MeasurementNoiseSemantic semantic);

struct MeasurementNoiseModel {
  MeasurementNoiseSemantic semantic =
      MeasurementNoiseSemantic::EMPIRICAL_POSE_RESIDUAL;
  Matrix6d covariance = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  std::string coordinate_definition;
  bool statistically_calibrated = false;
};

struct PhysicalCovariancePropagation {
  bool valid = false;
  Matrix6d residual_covariance = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  std::string status = "UNINITIALIZED";
};

// A_exact maps the physical normalized LiDAR perturbation chart
// [map-spatial rotation, normalized map translation] into the residual chart
// consumed by the EKF/window pose factor. Q_lidar is never diagonalized or
// block-truncated: rotational/translational cross covariance is retained.
PhysicalCovariancePropagation propagatePhysicalLidarCovariance(
    const Matrix6d& A_exact, const Matrix6d& Q_lidar);

// Empirical pose-residual covariance is already expressed in the target
// residual chart. It must not be propagated through A_exact again.
bool makeEmpiricalPoseResidualModel(
    const Matrix6d& residual_covariance,
    const std::string& coordinate_definition,
    MeasurementNoiseModel* output, std::string* reason = nullptr);

bool covarianceIsPSD(const Matrix6d& covariance, double tolerance,
                     std::string* reason = nullptr);

// Uses the same physical vector and covariance in two coordinate charts. This
// is a diagnostic helper for R4 NIS consistency, not a covariance estimator.
bool transformResidualAndCovariance(const Vector6d& residual_source,
                                    const Matrix6d& A_exact,
                                    const Matrix6d& Q_source,
                                    Vector6d* residual_target,
                                    Matrix6d* Q_target,
                                    std::string* reason = nullptr);

}  // namespace dog_prior_map_fastlio2_frontend_exp
