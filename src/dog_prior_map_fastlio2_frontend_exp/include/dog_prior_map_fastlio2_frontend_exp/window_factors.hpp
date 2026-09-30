#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/measurement_noise_model.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

using Vector15d = Eigen::Matrix<double, 15, 1>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;

// Local error order is [right rotation, position, velocity, gyro bias,
// accelerometer bias]. All poses remain on SO(3); only local increments are
// Euclidean vectors.
struct WindowState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint64_t stamp_ns = 0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
};

struct ImuNoiseParameters {
  double gyro_noise_density = 0.01;
  double accel_noise_density = 0.10;
  double gyro_bias_random_walk = 1e-4;
  double accel_bias_random_walk = 1e-3;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.809);
};

struct ImuPreintegratedMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint64_t start_stamp_ns = 0;
  std::uint64_t end_stamp_ns = 0;
  double dt_s = 0.0;
  Eigen::Matrix3d delta_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d jacobian_rotation_gyro_bias = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d jacobian_velocity_gyro_bias = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d jacobian_velocity_accel_bias = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d jacobian_position_gyro_bias = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d jacobian_position_accel_bias = Eigen::Matrix3d::Zero();
  Matrix15d covariance = Matrix15d::Zero();
  Matrix15d physical_covariance = Matrix15d::Zero();
  bool factor_covariance_regularized = false;
  double factor_covariance_regularization = 0.0;
  double physical_min_eigenvalue = 0.0;
  double factor_min_eigenvalue = 0.0;
  double information_change_norm = 0.0;
  Eigen::Vector3d linearization_gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearization_accel_bias = Eigen::Vector3d::Zero();
  bool valid = false;
  std::string status = "UNINITIALIZED";
};

enum class VisualFactorMode {
  FULL_TRANSLATION,
  LIDAR_WEAK_TRANSLATION,
  NOT_TRIGGERED,
};

const char* toString(VisualFactorMode mode);

bool preintegrateImu(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    std::uint64_t start_stamp_ns, std::uint64_t end_stamp_ns,
    const Eigen::Vector3d& linearization_gyro_bias,
    const Eigen::Vector3d& linearization_accel_bias,
    const ImuNoiseParameters& noise, ImuPreintegratedMeasurement* output,
    std::string* reason = nullptr);

// Both APIs execute the same bounded integration loop, including its bias,
// covariance and midpoint-input / left-orientation discretization.
WindowState propagateWindowState(const WindowState& anchor,
    const ImuPreintegratedMeasurement& measurement,
    const Eigen::Vector3d& gravity);
bool integrateWindowImuTrajectory(
    const WindowState& anchor, std::uint64_t end_stamp_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    const ImuNoiseParameters& noise, ImuPreintegratedMeasurement* measurement,
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>* trajectory,
    std::string* reason = nullptr);

bool applyLocalIncrement(WindowState* state, const Vector15d& increment,
                         std::string* reason = nullptr);
Vector15d localDifference(const WindowState& state,
                          const WindowState& reference);

bool buildImuResidual(const WindowState& from, const WindowState& to,
                      const ImuPreintegratedMeasurement& measurement,
                      const ImuNoiseParameters& noise, Vector15d* residual,
                      std::string* reason = nullptr);

bool linearizeImuFactor(
    const WindowState& from, const WindowState& to,
    const ImuPreintegratedMeasurement& measurement,
    const ImuNoiseParameters& noise, Eigen::Matrix<double, 15, 15>* jacobian_from,
    Eigen::Matrix<double, 15, 15>* jacobian_to, Vector15d* residual,
    std::string* reason = nullptr);

// Independent central-FD oracle. Not used by the runtime assembler.
bool linearizeImuFactorFiniteDifferenceReference(
    const WindowState& from, const WindowState& to,
    const ImuPreintegratedMeasurement& measurement,
    const ImuNoiseParameters& noise, Matrix15d* jacobian_from,
    Matrix15d* jacobian_to, Vector15d* residual,
    std::string* reason = nullptr);

struct LidarWindowMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint64_t observation_id = 0;
  std::uint64_t stamp_ns = 0;
  Eigen::Matrix3d measured_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d measured_position = Eigen::Vector3d::Zero();
  Matrix6d covariance = Matrix6d::Identity();
  Matrix6d measurement_basis = Matrix6d::Identity();
  int reliable_rank = 6;
  // For a degenerate (rank < 6) measurement, the owning NDT/U_obs adapter
  // must rebuild the reliable basis at every outer linearization point.  The
  // returned basis is frozen while the local Jacobian is evaluated.
  std::function<bool(const WindowState&, Matrix6d*, int*, std::string*)>
      basis_relinearizer;
  bool valid = false;
  std::string skipped_reason;
};

// Immutable reliability-subspace and projected-noise snapshot for one inner
// optimizer iteration. Both the local model and candidate acceptance must use
// this exact pair; a new snapshot is formed only at the next outer iteration.
struct FrozenLidarProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint64_t observation_id = 0;
  std::uint64_t stamp_ns = 0;
  Matrix6d basis = Matrix6d::Zero();
  int reliable_rank = 0;
  Eigen::MatrixXd selected_covariance;
  bool valid = false;
};

using FrozenLidarProjectionVector =
    std::vector<FrozenLidarProjection,
                Eigen::aligned_allocator<FrozenLidarProjection>>;

bool freezeLidarProjection(const WindowState& state,
                           const LidarWindowMeasurement& measurement,
                           FrozenLidarProjection* output,
                           std::string* reason = nullptr);

bool linearizeLidarFactorWithFrozenProjection(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    const FrozenLidarProjection& projection, Eigen::VectorXd* residual,
    Eigen::MatrixXd* jacobian, Eigen::MatrixXd* covariance,
    std::string* reason = nullptr);

bool buildLidarResidual(const WindowState& state,
                        const LidarWindowMeasurement& measurement,
                        Eigen::VectorXd* residual, std::string* reason = nullptr);

bool linearizeLidarFactor(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    Eigen::VectorXd* residual, Eigen::MatrixXd* jacobian,
    Eigen::MatrixXd* covariance, std::string* reason = nullptr);
bool linearizeLidarFactorFiniteDifferenceReference(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    Eigen::VectorXd* residual, Eigen::MatrixXd* jacobian,
    Eigen::MatrixXd* covariance, std::string* reason = nullptr);

struct VisualRelativeMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint64_t observation_id = 0;
  std::uint64_t reference_stamp_ns = 0;
  std::uint64_t current_stamp_ns = 0;
  Eigen::Vector3d reference_imu_translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d measurement_basis = Eigen::Matrix3d::Identity();
  int selected_rank = 3;
  VisualFactorMode mode = VisualFactorMode::FULL_TRANSLATION;
  std::string trigger_status = "FULL_TRANSLATION";
  std::uint64_t basis_source_lidar_stamp_ns = 0;
  Matrix6d admission_exact_jacobian = Matrix6d::Zero();
  bool valid = false;
  std::string source_semantic = "METRIC_PNP_RELATIVE_TRANSLATION_FACTOR";
};

bool buildVisualResidual(const WindowState& reference,
                         const WindowState& current,
                         const VisualRelativeMeasurement& measurement,
                         Eigen::Vector3d* residual,
                         std::string* reason = nullptr);

bool linearizeVisualFactor(
    const WindowState& reference, const WindowState& current,
    const VisualRelativeMeasurement& measurement, Eigen::Vector3d* residual,
    Eigen::Matrix<double, 3, 15>* jacobian_reference,
    Eigen::Matrix<double, 3, 15>* jacobian_current,
    std::string* reason = nullptr);

bool linearizeSelectedVisualFactor(
    const WindowState& reference, const WindowState& current,
    const VisualRelativeMeasurement& measurement, Eigen::VectorXd* residual,
    Eigen::MatrixXd* jacobian_reference, Eigen::MatrixXd* jacobian_current,
    Eigen::MatrixXd* covariance, std::string* reason = nullptr);

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
