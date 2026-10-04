#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <omp.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

// This is the only runtime translation unit allowed to include this pinned
// header: it defines non-inline free functions.
#include <use-ikfom.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

MTK_BUILD_MANIFOLD(PoseMeasurement,
((vect3, position))
((SO3, rotation))
);

typedef esekfom::esekf<state_ikfom, 12, input_ikfom, PoseMeasurement, 6>
    PoseFilter;

PoseMeasurement poseModel(state_ikfom& state, bool& valid) {
  valid = true;
  return PoseMeasurement(state.pos, state.rot);
}

Eigen::Matrix<double, 6, state_ikfom::DOF> poseJacobian(
    state_ikfom&, bool& valid) {
  valid = true;
  Eigen::Matrix<double, 6, state_ikfom::DOF> h =
      Eigen::Matrix<double, 6, state_ikfom::DOF>::Zero();
  h.template block<3, 3>(0, MTK::getStartIdx(&state_ikfom::pos)).setIdentity();
  h.template block<3, 3>(3, MTK::getStartIdx(&state_ikfom::rot)).setIdentity();
  return h;
}

Eigen::Matrix<double, 6, 6> poseNoiseJacobian(state_ikfom&, bool& valid) {
  valid = true;
  return Eigen::Matrix<double, 6, 6>::Identity();
}

Eigen::Matrix<double, 12, 12> makeProcessNoise(
    const RuntimeParameters& parameters) {
  Eigen::Matrix<double, 12, 12> q = Eigen::Matrix<double, 12, 12>::Zero();
  q.block<3, 3>(0, 0).diagonal().setConstant(
      parameters.gyro_noise_std_rad_s * parameters.gyro_noise_std_rad_s);
  q.block<3, 3>(3, 3).diagonal().setConstant(
      parameters.accel_noise_std_m_s2 * parameters.accel_noise_std_m_s2);
  q.block<3, 3>(6, 6).diagonal().setConstant(
      parameters.gyro_bias_rw_std_rad_s2 * parameters.gyro_bias_rw_std_rad_s2);
  q.block<3, 3>(9, 9).diagonal().setConstant(
      parameters.accel_bias_rw_std_m_s3 * parameters.accel_bias_rw_std_m_s3);
  return q;
}

Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> makeInitialCovariance() {
  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> p =
      Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();
  // Match FAST-LIO IMU_init() engineering priors for fixed extrinsic, biases,
  // and the S2 gravity tangent; pose and velocity start with unit variance.
  p.block<3, 3>(6, 6).diagonal().setConstant(1e-5);
  p.block<3, 3>(9, 9).diagonal().setConstant(1e-5);
  p.block<3, 3>(15, 15).diagonal().setConstant(1e-4);
  p.block<3, 3>(18, 18).diagonal().setConstant(1e-3);
  p.block<2, 2>(21, 21).diagonal().setConstant(1e-5);
  return p;
}

Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>
makeInitialCovariance(const InitialStateOverrides& initial_state) {
  auto p = makeInitialCovariance();
  if (!initial_state.use_covariance_overrides) return p;
  // state_ikfom tangent ordering is [pos(0), rot(3), extrinsic_R(6),
  // extrinsic_t(9), vel(12), gyro_bias(15), accel_bias(18), gravity_S2(21)].
  p.block<3, 3>(12, 12).diagonal().setConstant(
      initial_state.velocity_std_m_s * initial_state.velocity_std_m_s);
  p.block<3, 3>(15, 15).diagonal().setConstant(
      initial_state.gyro_bias_std_rad_s * initial_state.gyro_bias_std_rad_s);
  p.block<3, 3>(18, 18).diagonal().setConstant(
      initial_state.accel_bias_std_m_s2 * initial_state.accel_bias_std_m_s2);
  return p;
}

void enforceFixedExtrinsicConstraint(
    state_ikfom& state,
    Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>& covariance,
    const SO3& calibrated_rotation, const vect3& calibrated_translation) {
  const int rotation_index = MTK::getStartIdx(&state_ikfom::offset_R_L_I);
  const int translation_index = MTK::getStartIdx(&state_ikfom::offset_T_L_I);
  state.offset_R_L_I = calibrated_rotation;
  state.offset_T_L_I = calibrated_translation;
  for (int index : {rotation_index, translation_index}) {
    covariance.block(index, 0, 3, state_ikfom::DOF).setZero();
    covariance.block(0, index, state_ikfom::DOF, 3).setZero();
    covariance.block<3, 3>(index, index).diagonal().setConstant(1e-12);
  }
}

bool finitePose(const Pose3d& pose) {
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite())
    return false;
  const double norm = pose.orientation.norm();
  return std::isfinite(norm) && norm > 1e-12 && std::abs(norm - 1.0) <= 1e-6;
}

Eigen::Matrix4d poseMatrix(const Pose3d& pose) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
  transform.block<3, 1>(0, 3) = pose.position;
  return transform;
}

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}
bool finiteState(const state_ikfom& state) {
  return state.pos.allFinite() && state.rot.coeffs().allFinite() &&
         state.offset_R_L_I.coeffs().allFinite() &&
         state.offset_T_L_I.allFinite() && state.vel.allFinite() &&
         state.bg.allFinite() && state.ba.allFinite() && state.grav.vec.allFinite();
}

struct ProjectedPoseSystem {
  Eigen::MatrixXd H;
  Eigen::MatrixXd R;
  Eigen::MatrixXd S;
  Eigen::VectorXd residual;
  Eigen::VectorXd S_inverse_residual;
  Eigen::LDLT<Eigen::MatrixXd> innovation_factor;
  ProjectedPoseInnovation diagnostic;
};

Pose3d statePose(const state_ikfom& state) {
  Pose3d pose;
  pose.position = state.pos;
  pose.orientation = Eigen::Quaterniond(
      state.rot.w(), state.rot.x(), state.rot.y(), state.rot.z());
  return pose;
}

bool buildProjectedPoseSystem(
    const Pose3d& prior_pose,
    const Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>& prior_covariance,
    const Pose3d& measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_noise,
    const Eigen::Matrix<double, 6, 6>& measurement_basis,
    int rank, ProjectedPoseLinearizationMode linearization_mode,
    ProjectedPoseSystem* system, std::string* reason) {
  if (reason) reason->clear();
  if (!system) return fail(reason, "null_projected_pose_system");
  *system = ProjectedPoseSystem();
  system->diagnostic.rank = rank;
  system->diagnostic.threshold = chiSquare99Threshold(rank);
  const auto reject = [&](const char* status) {
    system->diagnostic.status = status;
    return fail(reason, status);
  };
  if (rank < 1 || rank > 6)
    return reject("projected_pose_rank_out_of_range");
  if (!finitePose(prior_pose) || !finitePose(measurement))
    return reject("invalid_projected_pose_measurement");
  if (!measurement_basis.allFinite() || !measurement_noise.allFinite() ||
      !prior_covariance.allFinite())
    return reject("nonfinite_projected_pose_input");
  if ((measurement_noise - measurement_noise.transpose())
          .cwiseAbs().maxCoeff() > 1e-10)
    return reject("asymmetric_projected_pose_covariance");
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> full_noise_factor(measurement_noise);
  if (full_noise_factor.info() != Eigen::Success ||
      !full_noise_factor.matrixL().toDenseMatrix().allFinite() ||
      full_noise_factor.matrixL().toDenseMatrix().diagonal().minCoeff() <= 0.0)
    return reject("projected_pose_full_covariance_not_spd");
  if ((prior_covariance - prior_covariance.transpose())
          .cwiseAbs().maxCoeff() > 1e-8)
    return reject("asymmetric_projected_pose_prior_covariance");

  const Eigen::MatrixXd basis = measurement_basis.leftCols(rank);
  if ((basis.transpose() * basis -
       Eigen::MatrixXd::Identity(rank, rank)).norm() > 1e-8)
    return reject("projected_pose_basis_not_orthonormal");

  system->R = basis.transpose() * measurement_noise * basis;
  Eigen::LLT<Eigen::MatrixXd> noise_factor(system->R);
  if (noise_factor.info() != Eigen::Success ||
      !noise_factor.matrixL().toDenseMatrix().allFinite() ||
      noise_factor.matrixL().toDenseMatrix().diagonal().minCoeff() <= 0.0)
    return reject("projected_pose_covariance_not_spd");
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> noise_eigenvalues(system->R);
  if (noise_eigenvalues.info() != Eigen::Success ||
      !noise_eigenvalues.eigenvalues().allFinite())
    return reject("projected_pose_covariance_eigendecomposition_failed");

  Eigen::Matrix<double, 6, 1> raw_residual;
  raw_residual.head<3>() = measurement.position - prior_pose.position;
  Eigen::Quaterniond rotation_residual =
      (prior_pose.orientation.conjugate() * measurement.orientation.normalized()).normalized();
  if (rotation_residual.w() < 0.0)
    rotation_residual.coeffs() *= -1.0;
  const Eigen::AngleAxisd rotation_error(rotation_residual);
  raw_residual.tail<3>() = rotation_error.axis() * rotation_error.angle();
  if (!raw_residual.allFinite())
    return reject("nonfinite_projected_pose_residual");

  Eigen::Matrix<double, 6, state_ikfom::DOF> selector =
      Eigen::Matrix<double, 6, state_ikfom::DOF>::Zero();
  selector.block<3, 3>(0, MTK::getStartIdx(&state_ikfom::pos)).setIdentity();
  if (linearization_mode ==
      ProjectedPoseLinearizationMode::LEGACY_IDENTITY_ROTATION) {
    selector.block<3, 3>(3, MTK::getStartIdx(&state_ikfom::rot)).setIdentity();
  } else if (linearization_mode ==
             ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL) {
    Eigen::Matrix3d left_inverse;
    std::string jacobian_reason;
    if (!so3LeftJacobianInverse(raw_residual.tail<3>(), &left_inverse,
                                &jacobian_reason))
      return reject(jacobian_reason.c_str());
    // The residual changes by -H*dx under the filter's right perturbation;
    // H therefore contains +J_l^{-1}(phi).
    selector.block<3, 3>(3, MTK::getStartIdx(&state_ikfom::rot)) =
        left_inverse;
  } else {
    return reject("unknown_projected_pose_linearization_mode");
  }
  system->H = basis.transpose() * selector;
  system->S = system->H * prior_covariance * system->H.transpose() + system->R;
  system->innovation_factor.compute(system->S);
  if (system->innovation_factor.info() != Eigen::Success ||
      !system->S.allFinite() || !system->innovation_factor.isPositive() ||
      !system->innovation_factor.vectorD().allFinite() ||
      system->innovation_factor.vectorD().minCoeff() <= 0.0)
    return reject("projected_pose_innovation_not_positive");

  system->residual = basis.transpose() * raw_residual;
  system->S_inverse_residual =
      system->innovation_factor.solve(system->residual);
  if (!system->S_inverse_residual.allFinite())
    return reject("projected_pose_innovation_solve_nonfinite");
  double nis = system->residual.dot(system->S_inverse_residual);
  if (!std::isfinite(nis) || nis < -1e-10)
    return reject("projected_pose_nis_invalid");

  system->diagnostic.valid = true;
  system->diagnostic.nis = std::max(0.0, nis);
  system->diagnostic.residual_norm = raw_residual.norm();
  system->diagnostic.projected_residual_norm = system->residual.norm();
  system->diagnostic.projected_noise_trace = system->R.trace();
  system->diagnostic.innovation_covariance_trace = system->S.trace();
  system->diagnostic.projected_noise_min_eigenvalue =
      noise_eigenvalues.eigenvalues().minCoeff();
  system->diagnostic.projected_noise_max_eigenvalue =
      noise_eigenvalues.eigenvalues().maxCoeff();
  system->diagnostic.status = "OK";
  return true;
}

}  // namespace


double chiSquare99Threshold(int rank) {
  static const double thresholds[] = {
      std::numeric_limits<double>::quiet_NaN(),
      6.635, 9.210, 11.345, 13.277, 15.086, 16.812};
  if (rank < 1 || rank > 6)
    return std::numeric_limits<double>::quiet_NaN();
  return thresholds[rank];
}

struct FastLio2IkfomFrontend::Impl {
  explicit Impl(const RuntimeParameters& runtime_parameters)
      : parameters(runtime_parameters), process_noise(makeProcessNoise(runtime_parameters)),
        filter(state_ikfom(), makeInitialCovariance()) {
    double limits[state_ikfom::DOF];
    std::fill(limits, limits + state_ikfom::DOF, 100.0);
    filter.init(get_f, df_dx, df_dw, poseModel, poseJacobian,
                poseNoiseJacobian, 4, limits);
  }

  RuntimeParameters parameters;
  Eigen::Matrix<double, 12, 12> process_noise;
  PoseFilter filter;
  uint64_t stamp_ns = 0;
  bool is_initialized = false;
  SO3 fixed_rotation = SO3::Identity();
  vect3 fixed_translation = vect3(Eigen::Vector3d::Zero());
};

FastLio2IkfomFrontend::FastLio2IkfomFrontend(
    const RuntimeParameters& parameters)
    : impl_(new Impl(parameters)) {}

FastLio2IkfomFrontend::~FastLio2IkfomFrontend() = default;

bool FastLio2IkfomFrontend::initializeStatic(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    const Pose3d& initial_map_T_lidar, const Pose3d& T_imu_lidar,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (impl_->parameters.static_init_samples <= 1 ||
      samples.size() < static_cast<std::size_t>(impl_->parameters.static_init_samples))
    return fail(failure_reason, "insufficient_static_imu_samples");
  if (!finitePose(initial_map_T_lidar) || !finitePose(T_imu_lidar))
    return fail(failure_reason, "invalid_initial_or_extrinsic_pose");
  StaticImuCalibration calibration;
  if (!calibrateStaticImu(samples, &calibration, failure_reason)) return false;
  const Eigen::Vector3d mean_specific_force = calibration.mean_specific_force;
  if (!mean_specific_force.allFinite() || mean_specific_force.norm() < 1e-6)
    return fail(failure_reason, "invalid_static_acceleration_mean");

  const Eigen::Matrix4d map_T_lidar = poseMatrix(initial_map_T_lidar);
  const Eigen::Matrix4d imu_T_lidar = poseMatrix(T_imu_lidar);
  const Eigen::Matrix4d prior_map_T_imu = map_T_lidar * imu_T_lidar.inverse();
  const Eigen::Matrix3d prior_rotation = prior_map_T_imu.block<3, 3>(0, 0);
  const Eigen::Vector3d acceleration_world_prior = prior_rotation * mean_specific_force;
  if (!acceleration_world_prior.allFinite() || acceleration_world_prior.norm() < 1e-6)
    return fail(failure_reason, "invalid_prior_frame_gravity_direction");

  // Preserve the no-GT map convention used by the audit: refine the supplied
  // initial tilt so the static specific-force direction maps to world +Z.
  const Eigen::Quaterniond gravity_alignment = Eigen::Quaterniond::FromTwoVectors(
      acceleration_world_prior.normalized(), Eigen::Vector3d::UnitZ());
  const Eigen::Matrix3d initialized_rotation =
      gravity_alignment.toRotationMatrix() * prior_rotation;
  const Eigen::Vector3d initialized_position =
      map_T_lidar.block<3, 1>(0, 3) -
      initialized_rotation * imu_T_lidar.block<3, 1>(0, 3);

  state_ikfom state;
  state.pos = vect3(initialized_position);
  state.rot = SO3(initialized_rotation);
  state.offset_R_L_I = SO3(imu_T_lidar.block<3, 3>(0, 0));
  state.offset_T_L_I = vect3(imu_T_lidar.block<3, 1>(0, 3));
  state.vel = vect3(Eigen::Vector3d::Zero());
  state.bg = vect3(calibration.gyro_bias);
  state.ba = vect3(calibration.accel_bias_prior);
  state.grav = S2(-initialized_rotation * mean_specific_force.normalized() *
                  impl_->parameters.gravity_mps2);

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> covariance =
      makeInitialCovariance();
  enforceFixedExtrinsicConstraint(state, covariance, state.offset_R_L_I,
                                  state.offset_T_L_I);
  impl_->filter.change_x(state);
  impl_->filter.change_P(covariance);
  impl_->fixed_rotation = state.offset_R_L_I;
  impl_->fixed_translation = state.offset_T_L_I;
  impl_->stamp_ns = samples[calibration.sample_count - 1].stamp_ns;
  impl_->is_initialized = true;
  return postconditionsValid(failure_reason);
}

bool FastLio2IkfomFrontend::calibrateStaticImu(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    StaticImuCalibration* calibration, std::string* failure_reason) const {
  if (failure_reason) failure_reason->clear();
  if (!calibration) return fail(failure_reason, "null_static_calibration_output");
  *calibration = StaticImuCalibration();
  if (impl_->parameters.static_init_samples <= 1 ||
      samples.size() < static_cast<std::size_t>(impl_->parameters.static_init_samples))
    return fail(failure_reason, "insufficient_static_imu_samples");
  if (!std::isfinite(impl_->parameters.gravity_mps2) ||
      impl_->parameters.gravity_mps2 <= 0.0 ||
      !impl_->parameters.initial_accel_bias.allFinite())
    return fail(failure_reason, "invalid_initialization_parameters");

  const std::size_t count =
      static_cast<std::size_t>(impl_->parameters.static_init_samples);
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_m2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_m2 = Eigen::Vector3d::Zero();
  uint64_t previous_stamp = 0;
  for (std::size_t index = 0; index < count; ++index) {
    const ImuSample& sample = samples[index];
    if (sample.stamp_ns == 0 || (index > 0 && sample.stamp_ns <= previous_stamp) ||
        !sample.acceleration.allFinite() || !sample.angular_velocity.allFinite())
      return fail(failure_reason, "invalid_or_nonmonotonic_static_imu_sample");
    previous_stamp = sample.stamp_ns;
    const double n = static_cast<double>(index + 1);
    const Eigen::Vector3d delta_acc = sample.acceleration - mean_acc;
    const Eigen::Vector3d delta_gyro = sample.angular_velocity - mean_gyro;
    mean_acc += delta_acc / n;
    mean_gyro += delta_gyro / n;
    acc_m2.array() += delta_acc.array() * (sample.acceleration - mean_acc).array();
    gyro_m2.array() += delta_gyro.array() * (sample.angular_velocity - mean_gyro).array();
  }

  const Eigen::Vector3d acc_std =
      (acc_m2 / static_cast<double>(count - 1)).cwiseMax(0.0).cwiseSqrt();
  const Eigen::Vector3d gyro_std =
      (gyro_m2 / static_cast<double>(count - 1)).cwiseMax(0.0).cwiseSqrt();
  if (acc_std.maxCoeff() > impl_->parameters.max_static_accel_std_m_s2 ||
      gyro_std.maxCoeff() > impl_->parameters.max_static_gyro_std_rad_s)
    return fail(failure_reason, "static_imu_variance_exceeds_gate");

  const Eigen::Vector3d specific_force = mean_acc - impl_->parameters.initial_accel_bias;
  if (!specific_force.allFinite() || specific_force.norm() < 1e-6)
    return fail(failure_reason, "invalid_static_acceleration_mean");

  calibration->gate_passed = true;
  calibration->sample_count = count;
  calibration->start_stamp_ns = samples.front().stamp_ns;
  calibration->end_stamp_ns = samples[count - 1].stamp_ns;
  calibration->mean_acceleration = mean_acc;
  calibration->mean_specific_force = specific_force;
  calibration->gyro_bias = mean_gyro;
  calibration->accel_bias_prior = impl_->parameters.initial_accel_bias;
  calibration->acceleration_std = acc_std;
  calibration->gyro_std = gyro_std;
  calibration->gravity_mps2 = impl_->parameters.gravity_mps2;
  return true;
}

bool FastLio2IkfomFrontend::initializeFromStaticCalibration(
    const StaticImuCalibration& calibration,
    const Pose3d& initial_map_T_imu, const Pose3d& T_imu_lidar,
    const Eigen::Vector3d& gravity_map,
    const Eigen::Vector3d& initial_velocity, uint64_t start_timestamp_ns,
    std::string* failure_reason) {
  InitialStateOverrides initial_state;
  initial_state.use_initial_velocity = true;
  initial_state.velocity_world_m_s = initial_velocity;
  return initializeFromStaticCalibration(calibration, initial_map_T_imu,
      T_imu_lidar, gravity_map, initial_state, start_timestamp_ns,
      failure_reason);
}

bool FastLio2IkfomFrontend::initializeFromStaticCalibration(
    const StaticImuCalibration& calibration,
    const Pose3d& initial_map_T_imu, const Pose3d& T_imu_lidar,
    const Eigen::Vector3d& gravity_map,
    const InitialStateOverrides& initial_state, uint64_t start_timestamp_ns,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!calibration.gate_passed ||
      calibration.sample_count != static_cast<std::size_t>(impl_->parameters.static_init_samples) ||
      calibration.start_stamp_ns == 0 ||
      calibration.end_stamp_ns <= calibration.start_stamp_ns ||
      start_timestamp_ns <= calibration.end_stamp_ns)
    return fail(failure_reason, "invalid_or_noncausal_static_calibration");
  if (!finitePose(initial_map_T_imu) || !finitePose(T_imu_lidar))
    return fail(failure_reason, "invalid_reanchor_or_extrinsic_pose");
  if (!calibration.mean_acceleration.allFinite() ||
      !calibration.mean_specific_force.allFinite() ||
      !calibration.gyro_bias.allFinite() ||
      !calibration.accel_bias_prior.allFinite() ||
      !calibration.acceleration_std.allFinite() || !calibration.gyro_std.allFinite() ||
      !gravity_map.allFinite() ||
      (initial_state.use_initial_velocity &&
       !initial_state.velocity_world_m_s.allFinite()) ||
      (initial_state.use_initial_biases &&
       (!initial_state.gyro_bias_rad_s.allFinite() ||
        !initial_state.accel_bias_m_s2.allFinite())) ||
      !std::isfinite(calibration.gravity_mps2) ||
      std::abs(calibration.gravity_mps2 - impl_->parameters.gravity_mps2) > 1e-9 ||
      (calibration.accel_bias_prior - impl_->parameters.initial_accel_bias).norm() > 1e-12 ||
      calibration.acceleration_std.maxCoeff() > impl_->parameters.max_static_accel_std_m_s2 ||
      calibration.gyro_std.maxCoeff() > impl_->parameters.max_static_gyro_std_rad_s ||
      std::abs(gravity_map.norm() - impl_->parameters.gravity_mps2) > 1e-6)
    return fail(failure_reason, "invalid_static_calibration_or_reanchor_gravity");
  if (initial_state.use_covariance_overrides &&
      (!std::isfinite(initial_state.velocity_std_m_s) ||
       initial_state.velocity_std_m_s <= 0.0 ||
       !std::isfinite(initial_state.gyro_bias_std_rad_s) ||
       initial_state.gyro_bias_std_rad_s <= 0.0 ||
       !std::isfinite(initial_state.accel_bias_std_m_s2) ||
       initial_state.accel_bias_std_m_s2 <= 0.0))
    return fail(failure_reason, "invalid_initial_state_covariance_override");

  state_ikfom state;
  state.pos = vect3(initial_map_T_imu.position);
  state.rot = SO3(initial_map_T_imu.orientation.toRotationMatrix());
  state.offset_R_L_I = SO3(T_imu_lidar.orientation.toRotationMatrix());
  state.offset_T_L_I = vect3(T_imu_lidar.position);
  state.vel = vect3(initial_state.use_initial_velocity
      ? initial_state.velocity_world_m_s : Eigen::Vector3d::Zero());
  state.bg = vect3(initial_state.use_initial_biases
      ? initial_state.gyro_bias_rad_s : calibration.gyro_bias);
  state.ba = vect3(initial_state.use_initial_biases
      ? initial_state.accel_bias_m_s2 : calibration.accel_bias_prior);
  state.grav = S2(gravity_map);

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> covariance =
      makeInitialCovariance(initial_state);
  enforceFixedExtrinsicConstraint(state, covariance, state.offset_R_L_I,
                                  state.offset_T_L_I);
  impl_->filter.change_x(state);
  impl_->filter.change_P(covariance);
  impl_->fixed_rotation = state.offset_R_L_I;
  impl_->fixed_translation = state.offset_T_L_I;
  impl_->stamp_ns = start_timestamp_ns;
  impl_->is_initialized = true;
  return postconditionsValid(failure_reason);
}

bool FastLio2IkfomFrontend::predictInterval(
    const ImuSample& head, const ImuSample& tail, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!impl_->is_initialized) return fail(failure_reason, "filter_not_initialized");
  if (head.stamp_ns == 0 || tail.stamp_ns <= head.stamp_ns ||
      !head.acceleration.allFinite() || !tail.acceleration.allFinite() ||
      !head.angular_velocity.allFinite() || !tail.angular_velocity.allFinite())
    return fail(failure_reason, "invalid_imu_interval");
  if (impl_->stamp_ns < head.stamp_ns || impl_->stamp_ns >= tail.stamp_ns)
    return fail(failure_reason, "imu_interval_does_not_cover_filter_time");

  const uint64_t interval_end_ns = tail.stamp_ns;
  const double dt = static_cast<double>(interval_end_ns - impl_->stamp_ns) * 1e-9;
  input_ikfom input;
  input.acc = vect3(0.5 * (head.acceleration + tail.acceleration));
  input.gyro = vect3(0.5 * (head.angular_velocity + tail.angular_velocity));
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(input.acc.norm()) ||
      !std::isfinite(input.gyro.norm()))
    return fail(failure_reason, "nonfinite_interval_input");

  double mutable_dt = dt;
  auto q = impl_->process_noise;
  impl_->filter.predict(mutable_dt, q, input);
  state_ikfom state = impl_->filter.get_x();
  auto covariance = impl_->filter.get_P();
  enforceFixedExtrinsicConstraint(state, covariance, impl_->fixed_rotation,
                                  impl_->fixed_translation);
  impl_->filter.change_x(state);
  impl_->filter.change_P(covariance);
  impl_->stamp_ns = interval_end_ns;
  return postconditionsValid(failure_reason);
}

bool FastLio2IkfomFrontend::predictImuSequence(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    uint64_t exact_end_stamp_ns,
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>* poses,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (poses) poses->clear();
  if (!poses) return fail(failure_reason, "null_imu_pose_output");
  if (!impl_->is_initialized) return fail(failure_reason, "filter_not_initialized");
  if (samples.size() < 2 || exact_end_stamp_ns < impl_->stamp_ns)
    return fail(failure_reason, "insufficient_imu_sequence_or_end_before_state");

  uint64_t previous_stamp = 0;
  for (const ImuSample& sample : samples) {
    if (sample.stamp_ns == 0 || sample.stamp_ns <= previous_stamp ||
        sample.stamp_ns > exact_end_stamp_ns ||
        !sample.acceleration.allFinite() || !sample.angular_velocity.allFinite())
      return fail(failure_reason, "invalid_or_future_imu_sequence_sample");
    previous_stamp = sample.stamp_ns;
  }

  const uint64_t initial_stamp = impl_->stamp_ns;
  if (samples.front().stamp_ns > initial_stamp ||
      samples.back().stamp_ns < std::min(initial_stamp, exact_end_stamp_ns))
    return fail(failure_reason, "imu_sequence_does_not_cover_state_time");

  auto sampleAt = [](const ImuSample& a, const ImuSample& b, uint64_t stamp) {
    ImuSample interpolated;
    interpolated.stamp_ns = stamp;
    const double alpha = static_cast<double>(stamp - a.stamp_ns) /
                         static_cast<double>(b.stamp_ns - a.stamp_ns);
    interpolated.acceleration = (1.0 - alpha) * a.acceleration + alpha * b.acceleration;
    interpolated.angular_velocity =
        (1.0 - alpha) * a.angular_velocity + alpha * b.angular_velocity;
    return interpolated;
  };
  auto appendPose = [this, poses](uint64_t stamp, const Eigen::Vector3d& accel,
                                  const Eigen::Vector3d& gyro) {
    const FilterSnapshot snapshot = getState();
    ImuPoseSample pose;
    pose.stamp_ns = stamp;
    pose.rotation = snapshot.map_T_imu.orientation.toRotationMatrix();
    pose.position = snapshot.map_T_imu.position;
    pose.velocity = snapshot.velocity;
    pose.world_acceleration = pose.rotation * (accel - snapshot.accel_bias) +
                              snapshot.gravity;
    pose.unbiased_gyro = gyro - snapshot.gyro_bias;
    poses->push_back(pose);
  };

  std::size_t bracket = 0;
  while (bracket + 1 < samples.size() &&
         samples[bracket + 1].stamp_ns <= initial_stamp)
    ++bracket;
  if (samples[bracket].stamp_ns > initial_stamp)
    return fail(failure_reason, "no_causal_imu_bracket_at_state_time");

  ImuSample state_time_sample = samples[bracket];
  if (state_time_sample.stamp_ns < initial_stamp)
    state_time_sample = sampleAt(samples[bracket], samples[bracket + 1], initial_stamp);
  const Eigen::Vector3d initial_accel = state_time_sample.acceleration;
  const Eigen::Vector3d initial_gyro = state_time_sample.angular_velocity;
  appendPose(initial_stamp, initial_accel, initial_gyro);

  uint64_t propagated_to = initial_stamp;
  for (std::size_t index = bracket; index + 1 < samples.size(); ++index) {
    const ImuSample& tail = samples[index + 1];
    if (tail.stamp_ns <= propagated_to) continue;
    const ImuSample head = propagated_to == samples[index].stamp_ns
                               ? samples[index]
                               : sampleAt(samples[index], tail, propagated_to);
    if (!predictInterval(head, tail, failure_reason)) return false;
    propagated_to = tail.stamp_ns;
    const Eigen::Vector3d accel_mid = 0.5 * (head.acceleration + tail.acceleration);
    const Eigen::Vector3d gyro_mid = 0.5 * (head.angular_velocity + tail.angular_velocity);
    appendPose(propagated_to, accel_mid, gyro_mid);
  }

  if (propagated_to < exact_end_stamp_ns) {
    const ImuSample& head = samples[samples.size() - 2];
    const ImuSample& tail = samples.back();
    if (!predictHeldInputTo(exact_end_stamp_ns, head, tail, failure_reason))
      return false;
    appendPose(exact_end_stamp_ns,
               0.5 * (head.acceleration + tail.acceleration),
               0.5 * (head.angular_velocity + tail.angular_velocity));
  }
  if (poses->empty() || poses->back().stamp_ns != exact_end_stamp_ns)
    return fail(failure_reason, "scan_end_pose_timestamp_not_exact");
  return postconditionsValid(failure_reason);
}

bool FastLio2IkfomFrontend::predictHeldInputTo(
    uint64_t end_stamp_ns, const ImuSample& head, const ImuSample& tail,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!impl_->is_initialized) return fail(failure_reason, "filter_not_initialized");
  if (tail.stamp_ns > end_stamp_ns || end_stamp_ns <= impl_->stamp_ns ||
      tail.stamp_ns > impl_->stamp_ns || head.stamp_ns >= tail.stamp_ns ||
      !head.acceleration.allFinite() || !tail.acceleration.allFinite() ||
      !head.angular_velocity.allFinite() || !tail.angular_velocity.allFinite())
    return fail(failure_reason, "invalid_causal_scan_end_tail");

  const double dt = static_cast<double>(end_stamp_ns - impl_->stamp_ns) * 1e-9;
  input_ikfom input;
  input.acc = vect3(0.5 * (head.acceleration + tail.acceleration));
  input.gyro = vect3(0.5 * (head.angular_velocity + tail.angular_velocity));
  double mutable_dt = dt;
  auto q = impl_->process_noise;
  impl_->filter.predict(mutable_dt, q, input);
  state_ikfom state = impl_->filter.get_x();
  auto covariance = impl_->filter.get_P();
  enforceFixedExtrinsicConstraint(state, covariance, impl_->fixed_rotation,
                                  impl_->fixed_translation);
  impl_->filter.change_x(state);
  impl_->filter.change_P(covariance);
  impl_->stamp_ns = end_stamp_ns;
  return postconditionsValid(failure_reason);
}

bool FastLio2IkfomFrontend::applyPoseMeasurement(
    const Pose3d& map_T_imu_measurement, PoseCorrectionDelta* delta,
    std::string* failure_reason) {
  Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
  noise.block<3, 3>(0, 0).diagonal().setConstant(
      impl_->parameters.pose_position_sigma_m * impl_->parameters.pose_position_sigma_m);
  noise.block<3, 3>(3, 3).diagonal().setConstant(
      impl_->parameters.pose_rotation_sigma_rad * impl_->parameters.pose_rotation_sigma_rad);
  return applyPoseMeasurement(map_T_imu_measurement, noise, delta, failure_reason);
}

bool FastLio2IkfomFrontend::applyPoseMeasurement(
    const Pose3d& map_T_imu_measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_covariance,
    PoseCorrectionDelta* delta, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!impl_->is_initialized) return fail(failure_reason, "filter_not_initialized");
  if (!finitePose(map_T_imu_measurement))
    return fail(failure_reason, "invalid_pose_measurement");
  if (!measurement_covariance.allFinite())
    return fail(failure_reason, "nonfinite_pose_measurement_covariance");
  if ((measurement_covariance - measurement_covariance.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return fail(failure_reason, "asymmetric_pose_measurement_covariance");
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> noise_factor(measurement_covariance);
  if (noise_factor.info() != Eigen::Success ||
      !noise_factor.matrixL().toDenseMatrix().allFinite() ||
      noise_factor.matrixL().toDenseMatrix().diagonal().minCoeff() <= 0.0)
    return fail(failure_reason, "pose_measurement_covariance_not_spd");

  const FilterSnapshot before = getState();
  // update_iterated mutates the filter object in place. Keep a transaction
  // snapshot so a failed postcondition cannot leak a partial correction.
  state_ikfom backup_x = impl_->filter.get_x();
  auto backup_P = impl_->filter.get_P();
  PoseMeasurement measurement(vect3(map_T_imu_measurement.position),
                              SO3(map_T_imu_measurement.orientation.toRotationMatrix()));
  // The pinned IKFoM API accepts a mutable R reference (and may reuse its
  // workspace), so keep the public caller's validated covariance immutable.
  Eigen::Matrix<double, 6, 6> update_noise = measurement_covariance;
  impl_->filter.update_iterated(measurement, update_noise);
  state_ikfom state = impl_->filter.get_x();
  auto covariance = impl_->filter.get_P();
  enforceFixedExtrinsicConstraint(state, covariance, impl_->fixed_rotation,
                                  impl_->fixed_translation);
  impl_->filter.change_x(state);
  impl_->filter.change_P(covariance);
  std::string postcondition_failure;
  if (!postconditionsValid(&postcondition_failure)) {
    impl_->filter.change_x(backup_x);
    impl_->filter.change_P(backup_P);
    return fail(failure_reason, postcondition_failure.c_str());
  }

  if (delta) {
    const FilterSnapshot after = getState();
    delta->position = after.map_T_imu.position - before.map_T_imu.position;
    const Eigen::Quaterniond q_before = before.map_T_imu.orientation.normalized();
    const Eigen::Quaterniond q_after = after.map_T_imu.orientation.normalized();
    const Eigen::Quaterniond dq = (q_before.conjugate() * q_after).normalized();
    const Eigen::AngleAxisd angle_axis(dq);
    delta->rotation = angle_axis.axis() * angle_axis.angle();
    delta->velocity = after.velocity - before.velocity;
    delta->gyro_bias = after.gyro_bias - before.gyro_bias;
    delta->accel_bias = after.accel_bias - before.accel_bias;
    const Eigen::Vector3d gravity_before = before.gravity;
    const Eigen::Vector3d gravity_after = after.gravity;
    const Eigen::Vector3d tangent_axis = gravity_before.normalized().unitOrthogonal();
    const Eigen::Vector3d tangent_axis_2 = gravity_before.normalized().cross(tangent_axis);
    delta->gravity_tangent << tangent_axis.dot(gravity_after - gravity_before),
                              tangent_axis_2.dot(gravity_after - gravity_before);
  }
  return true;
}

bool FastLio2IkfomFrontend::applyPositionMeasurement(
    const Eigen::Vector3d& map_position_measurement,
    const Eigen::Matrix3d& measurement_covariance,
    PoseCorrectionDelta* delta, std::string* failure_reason) {
  return applyProjectedPositionMeasurement(map_position_measurement,
      measurement_covariance, Eigen::Matrix3d::Identity(), 3, delta,
      failure_reason);
}

bool FastLio2IkfomFrontend::applyProjectedPositionMeasurement(
    const Eigen::Vector3d& map_position_measurement,
    const Eigen::Matrix3d& measurement_covariance,
    const Eigen::Matrix3d& measurement_basis, int measurement_rank,
    PoseCorrectionDelta* delta, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!impl_->is_initialized)
    return fail(failure_reason, "filter_not_initialized");
  if (measurement_rank < 1 || measurement_rank > 3)
    return fail(failure_reason, "projected_position_rank_out_of_range");
  if (!map_position_measurement.allFinite())
    return fail(failure_reason, "nonfinite_position_measurement");
  if (!measurement_basis.allFinite())
    return fail(failure_reason, "nonfinite_projected_position_basis");
  const Eigen::MatrixXd basis = measurement_basis.leftCols(measurement_rank);
  if ((basis.transpose() * basis -
       Eigen::MatrixXd::Identity(measurement_rank, measurement_rank))
          .norm() > 1e-8)
    return fail(failure_reason, "projected_position_basis_not_orthonormal");
  if (!measurement_covariance.allFinite())
    return fail(failure_reason, "nonfinite_position_measurement_covariance");
  if ((measurement_covariance - measurement_covariance.transpose())
          .cwiseAbs().maxCoeff() > 1e-10)
    return fail(failure_reason, "asymmetric_position_measurement_covariance");
  const Eigen::MatrixXd projected_noise = basis.transpose() *
      measurement_covariance * basis;
  Eigen::LLT<Eigen::MatrixXd> noise_factor(projected_noise);
  if (noise_factor.info() != Eigen::Success ||
      !noise_factor.matrixL().toDenseMatrix().allFinite() ||
      noise_factor.matrixL().toDenseMatrix().diagonal().minCoeff() <= 0.0)
    return fail(failure_reason, "projected_position_covariance_not_spd");

  const FilterSnapshot before = getState();
  state_ikfom backup_x = impl_->filter.get_x();
  auto backup_P = impl_->filter.get_P();
  state_ikfom prior = backup_x;
  state_ikfom state = prior;
  const Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> covariance = backup_P;
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(measurement_rank,
                                             state_ikfom::DOF);
  h.block(0, MTK::getStartIdx(&state_ikfom::pos), measurement_rank, 3) =
      basis.transpose();
  // Construct S in the selected measurement row-space only, not in the
  // discarded XYZ directions.
  const Eigen::MatrixXd selected_innovation_covariance =
      h * covariance * h.transpose() + projected_noise;
  Eigen::LDLT<Eigen::MatrixXd> innovation_factor(
      selected_innovation_covariance);
  if (innovation_factor.info() != Eigen::Success || !innovation_factor.isPositive())
    return fail(failure_reason, "projected_position_innovation_not_positive");
  const Eigen::MatrixXd gain =
      covariance * h.transpose() *
      innovation_factor.solve(Eigen::MatrixXd::Identity(
          measurement_rank, measurement_rank));
  const Eigen::VectorXd projected_residual = basis.transpose() *
      (map_position_measurement - prior.pos);
  const Eigen::Matrix<double, state_ikfom::DOF, 1> dx =
      gain * projected_residual;
  if (!dx.allFinite())
    return fail(failure_reason, "nonfinite_position_update_increment");
  state.boxplus(dx);

  const Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> residual_map =
      Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity() - gain * h;
  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> posterior =
      residual_map * covariance * residual_map.transpose() +
      gain * projected_noise * gain.transpose();
  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> reset =
      Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();
  for (const auto& item : state.SO3_state) {
    const int index = item.first;
    const MTK::vect<3, double> tangent = dx.template segment<3>(index);
    reset.template block<3, 3>(index, index) = A_matrix(tangent).transpose();
  }
  for (const auto& item : state.S2_state) {
    const int index = item.first;
    MTK::vect<2, double> tangent;
    tangent << dx[index], dx[index + 1];
    Eigen::Matrix<double, 2, 3> nx;
    Eigen::Matrix<double, 3, 2> mx;
    state.S2_Nx_yy(nx, index);
    prior.S2_Mx(mx, tangent, index);
    reset.template block<2, 2>(index, index) = nx * mx;
  }
  posterior = (reset * posterior * reset.transpose()).eval();
  posterior = (0.5 * (posterior + posterior.transpose())).eval();
  enforceFixedExtrinsicConstraint(state, posterior, impl_->fixed_rotation,
                                  impl_->fixed_translation);
  impl_->filter.change_x(state);
  impl_->filter.change_P(posterior);
  std::string postcondition_failure;
  if (!postconditionsValid(&postcondition_failure)) {
    impl_->filter.change_x(backup_x);
    impl_->filter.change_P(backup_P);
    return fail(failure_reason, postcondition_failure.c_str());
  }

  if (delta) {
    const FilterSnapshot after = getState();
    delta->position = after.map_T_imu.position - before.map_T_imu.position;
    const Eigen::Quaterniond q_before = before.map_T_imu.orientation.normalized();
    const Eigen::Quaterniond q_after = after.map_T_imu.orientation.normalized();
    const Eigen::Quaterniond dq = (q_before.conjugate() * q_after).normalized();
    const Eigen::AngleAxisd angle_axis(dq);
    delta->rotation = angle_axis.axis() * angle_axis.angle();
    delta->velocity = after.velocity - before.velocity;
    delta->gyro_bias = after.gyro_bias - before.gyro_bias;
    delta->accel_bias = after.accel_bias - before.accel_bias;
    const Eigen::Vector3d tangent_axis = before.gravity.normalized().unitOrthogonal();
    const Eigen::Vector3d tangent_axis_2 = before.gravity.normalized().cross(tangent_axis);
    delta->gravity_tangent << tangent_axis.dot(after.gravity - before.gravity),
                              tangent_axis_2.dot(after.gravity - before.gravity);
  }
  return true;
}

bool FastLio2IkfomFrontend::applyProjectedPoseMeasurement(
    const Pose3d& map_T_imu_measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_covariance,
    const Eigen::Matrix<double, 6, 6>& measurement_basis,
    int measurement_rank, PoseCorrectionDelta* delta,
    std::string* failure_reason) {
  return applyProjectedPoseMeasurementChecked(
      map_T_imu_measurement, measurement_covariance, measurement_basis,
      measurement_rank, false, std::numeric_limits<double>::quiet_NaN(),
      nullptr, delta, failure_reason);
}

bool FastLio2IkfomFrontend::evaluateProjectedPoseInnovation(
    const Pose3d& map_T_imu_measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_noise,
    const Eigen::Matrix<double, 6, 6>& measurement_basis, int rank,
    ProjectedPoseInnovation* output, std::string* reason) const {
  return evaluateProjectedPoseInnovationLinearized(
      map_T_imu_measurement, measurement_noise, measurement_basis, rank,
      ProjectedPoseLinearizationMode::LEGACY_IDENTITY_ROTATION, output,
      reason);
}

bool FastLio2IkfomFrontend::evaluateProjectedPoseInnovationLinearized(
    const Pose3d& map_T_imu_measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_noise,
    const Eigen::Matrix<double, 6, 6>& measurement_basis, int rank,
    ProjectedPoseLinearizationMode linearization_mode,
    ProjectedPoseInnovation* output, std::string* reason) const {
  if (reason) reason->clear();
  if (output) *output = ProjectedPoseInnovation();
  if (!impl_->is_initialized) {
    if (output) output->status = "filter_not_initialized";
    return fail(reason, "filter_not_initialized");
  }
  const state_ikfom& prior = impl_->filter.get_x();
  const auto prior_covariance = impl_->filter.get_P();
  ProjectedPoseSystem system;
  std::string local_reason;
  const bool valid = buildProjectedPoseSystem(
      statePose(prior), prior_covariance, map_T_imu_measurement,
      measurement_noise, measurement_basis, rank, linearization_mode, &system,
      &local_reason);
  if (output) *output = system.diagnostic;
  if (!valid) return fail(reason, local_reason.c_str());
  return true;
}

bool FastLio2IkfomFrontend::applyProjectedPoseMeasurementChecked(
    const Pose3d& map_T_imu_measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_noise,
    const Eigen::Matrix<double, 6, 6>& measurement_basis, int rank,
    bool enforce_nis_gate, double nis_threshold,
    ProjectedPoseInnovation* diagnostic, PoseCorrectionDelta* delta,
    std::string* reason) {
  return applyProjectedPoseMeasurementLinearizedChecked(
      map_T_imu_measurement, measurement_noise, measurement_basis, rank,
      ProjectedPoseLinearizationMode::LEGACY_IDENTITY_ROTATION,
      enforce_nis_gate, nis_threshold, diagnostic, delta, reason);
}

bool FastLio2IkfomFrontend::applyProjectedPoseMeasurementLinearizedChecked(
    const Pose3d& map_T_imu_measurement,
    const Eigen::Matrix<double, 6, 6>& measurement_noise,
    const Eigen::Matrix<double, 6, 6>& measurement_basis, int rank,
    ProjectedPoseLinearizationMode linearization_mode,
    bool enforce_nis_gate, double nis_threshold,
    ProjectedPoseInnovation* diagnostic, PoseCorrectionDelta* delta,
    std::string* reason) {
  if (reason) reason->clear();
  if (diagnostic) *diagnostic = ProjectedPoseInnovation();
  if (!impl_->is_initialized) {
    if (diagnostic) diagnostic->status = "filter_not_initialized";
    return fail(reason, "filter_not_initialized");
  }

  const FilterSnapshot before = getState();
  state_ikfom backup_x = impl_->filter.get_x();
  auto backup_P = impl_->filter.get_P();
  state_ikfom prior = backup_x;
  ProjectedPoseSystem system;
  std::string local_reason;
  if (!buildProjectedPoseSystem(
          statePose(prior), backup_P, map_T_imu_measurement,
          measurement_noise, measurement_basis, rank, linearization_mode, &system,
          &local_reason)) {
    if (diagnostic) *diagnostic = system.diagnostic;
    return fail(reason, local_reason.c_str());
  }
  if (enforce_nis_gate &&
      (!std::isfinite(nis_threshold) || nis_threshold <= 0.0)) {
    system.diagnostic.valid = false;
    system.diagnostic.status = "invalid_projected_pose_nis_threshold";
    if (diagnostic) *diagnostic = system.diagnostic;
    return fail(reason, "invalid_projected_pose_nis_threshold");
  }
  if (enforce_nis_gate) system.diagnostic.threshold = nis_threshold;
  if (enforce_nis_gate && system.diagnostic.nis > nis_threshold) {
    system.diagnostic.status = "SELECTED_NIS_REJECTED";
    if (diagnostic) *diagnostic = system.diagnostic;
    return fail(reason, "SELECTED_NIS_REJECTED");
  }

  state_ikfom state = prior;
  const Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> covariance = backup_P;
  const Eigen::MatrixXd gain = covariance * system.H.transpose() *
      system.innovation_factor.solve(Eigen::MatrixXd::Identity(rank, rank));
  const Eigen::Matrix<double, state_ikfom::DOF, 1> dx =
      gain * system.residual;
  if (!gain.allFinite() || !dx.allFinite()) {
    system.diagnostic.valid = false;
    system.diagnostic.status = "nonfinite_projected_pose_increment";
    if (diagnostic) *diagnostic = system.diagnostic;
    return fail(reason, "nonfinite_projected_pose_increment");
  }
  state.boxplus(dx);

  const Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> residual_map =
      Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity() -
      gain * system.H;
  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> posterior =
      residual_map * covariance * residual_map.transpose() +
      gain * system.R * gain.transpose();
  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> reset =
      Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();
  for (const auto& item : state.SO3_state) {
    const int index = item.first;
    const MTK::vect<3, double> tangent = dx.template segment<3>(index);
    reset.template block<3, 3>(index, index) = A_matrix(tangent).transpose();
  }
  for (const auto& item : state.S2_state) {
    const int index = item.first;
    MTK::vect<2, double> tangent;
    tangent << dx[index], dx[index + 1];
    Eigen::Matrix<double, 2, 3> nx;
    Eigen::Matrix<double, 3, 2> mx;
    state.S2_Nx_yy(nx, index);
    prior.S2_Mx(mx, tangent, index);
    reset.template block<2, 2>(index, index) = nx * mx;
  }
  posterior = (reset * posterior * reset.transpose()).eval();
  posterior = (0.5 * (posterior + posterior.transpose())).eval();
  enforceFixedExtrinsicConstraint(state, posterior, impl_->fixed_rotation,
                                  impl_->fixed_translation);
  impl_->filter.change_x(state);
  impl_->filter.change_P(posterior);
  std::string postcondition_failure;
  if (!postconditionsValid(&postcondition_failure)) {
    impl_->filter.change_x(backup_x);
    impl_->filter.change_P(backup_P);
    system.diagnostic.valid = false;
    system.diagnostic.status = postcondition_failure;
    if (diagnostic) *diagnostic = system.diagnostic;
    return fail(reason, postcondition_failure.c_str());
  }

  if (diagnostic) *diagnostic = system.diagnostic;
  if (delta) {
    const FilterSnapshot after = getState();
    delta->position = after.map_T_imu.position - before.map_T_imu.position;
    const Eigen::Quaterniond q_before = before.map_T_imu.orientation.normalized();
    const Eigen::Quaterniond q_after = after.map_T_imu.orientation.normalized();
    const Eigen::Quaterniond dq = (q_before.conjugate() * q_after).normalized();
    const Eigen::AngleAxisd angle_axis(dq);
    delta->rotation = angle_axis.axis() * angle_axis.angle();
    delta->velocity = after.velocity - before.velocity;
    delta->gyro_bias = after.gyro_bias - before.gyro_bias;
    delta->accel_bias = after.accel_bias - before.accel_bias;
    const Eigen::Vector3d tangent_axis = before.gravity.normalized().unitOrthogonal();
    const Eigen::Vector3d tangent_axis_2 = before.gravity.normalized().cross(tangent_axis);
    delta->gravity_tangent << tangent_axis.dot(after.gravity - before.gravity),
                              tangent_axis_2.dot(after.gravity - before.gravity);
  }
  return true;
}


std::unique_ptr<FastLio2IkfomFrontend>
FastLio2IkfomFrontend::cloneCandidate() const {
  std::unique_ptr<FastLio2IkfomFrontend> candidate(
      new FastLio2IkfomFrontend(impl_->parameters));
  if (!impl_->is_initialized) return candidate;
  state_ikfom candidate_state = impl_->filter.get_x();
  auto candidate_covariance = impl_->filter.get_P();
  candidate->impl_->filter.change_x(candidate_state);
  candidate->impl_->filter.change_P(candidate_covariance);
  candidate->impl_->stamp_ns = impl_->stamp_ns;
  candidate->impl_->is_initialized = true;
  candidate->impl_->fixed_rotation = impl_->fixed_rotation;
  candidate->impl_->fixed_translation = impl_->fixed_translation;
  return candidate;
}

bool FastLio2IkfomFrontend::commitCandidate(
    const FastLio2IkfomFrontend& candidate, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!candidate.impl_->is_initialized)
    return fail(failure_reason, "candidate_not_initialized");
  if (!candidate.postconditionsValid(failure_reason)) return false;
  state_ikfom candidate_state = candidate.impl_->filter.get_x();
  auto candidate_covariance = candidate.impl_->filter.get_P();
  impl_->filter.change_x(candidate_state);
  impl_->filter.change_P(candidate_covariance);
  impl_->stamp_ns = candidate.impl_->stamp_ns;
  impl_->is_initialized = true;
  impl_->fixed_rotation = candidate.impl_->fixed_rotation;
  impl_->fixed_translation = candidate.impl_->fixed_translation;
  return postconditionsValid(failure_reason);
}

bool FastLio2IkfomFrontend::rejectCandidatePredictionOnly(
    const FastLio2IkfomFrontend& candidate, std::string* failure_reason) {
  return commitCandidate(candidate, failure_reason);
}

void FastLio2IkfomFrontend::reset() {
  state_ikfom reset_state;
  auto reset_covariance = makeInitialCovariance();
  impl_->filter.change_x(reset_state);
  impl_->filter.change_P(reset_covariance);
  impl_->stamp_ns = 0;
  impl_->is_initialized = false;
  impl_->fixed_rotation = SO3::Identity();
  impl_->fixed_translation = vect3(Eigen::Vector3d::Zero());
}

bool FastLio2IkfomFrontend::initialized() const {
  return impl_->is_initialized;
}

FilterSnapshot FastLio2IkfomFrontend::getState() const {
  FilterSnapshot snapshot;
  if (!impl_->is_initialized) return snapshot;
  const state_ikfom& state = impl_->filter.get_x();
  snapshot.stamp_ns = impl_->stamp_ns;
  snapshot.map_T_imu.position = state.pos;
  snapshot.map_T_imu.orientation = Eigen::Quaterniond(
      state.rot.w(), state.rot.x(), state.rot.y(), state.rot.z());
  snapshot.velocity = state.vel;
  snapshot.gyro_bias = state.bg;
  snapshot.accel_bias = state.ba;
  snapshot.gravity = state.grav.vec;
  snapshot.T_imu_lidar_rotation = state.offset_R_L_I.toRotationMatrix();
  snapshot.T_imu_lidar_translation = state.offset_T_L_I;
  snapshot.covariance = impl_->filter.get_P();
  return snapshot;
}


Eigen::Matrix<double, 12, 12>
FastLio2IkfomFrontend::getProcessNoiseCovariance() const {
  return impl_->process_noise;
}

bool FastLio2IkfomFrontend::postconditionsValid(
    std::string* failure_reason) const {
  if (failure_reason) failure_reason->clear();
  if (!impl_->is_initialized) return fail(failure_reason, "filter_not_initialized");
  const state_ikfom& state = impl_->filter.get_x();
  const auto& covariance = impl_->filter.get_P();
  if (!finiteState(state) || !covariance.allFinite())
    return fail(failure_reason, "nonfinite_filter_state_or_covariance");
  if ((covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8 ||
      covariance.diagonal().minCoeff() < -1e-10)
    return fail(failure_reason, "invalid_covariance_postcondition");
  if (std::abs(state.grav.vec.norm() - impl_->parameters.gravity_mps2) > 1e-8)
    return fail(failure_reason, "gravity_norm_changed");
  if ((state.offset_R_L_I.coeffs().array() !=
       impl_->fixed_rotation.coeffs().array()).any() ||
      (state.offset_T_L_I.array() != impl_->fixed_translation.array()).any())
    return fail(failure_reason, "fixed_extrinsic_changed");
  if (impl_->stamp_ns == 0) return fail(failure_reason, "invalid_filter_timestamp");
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
