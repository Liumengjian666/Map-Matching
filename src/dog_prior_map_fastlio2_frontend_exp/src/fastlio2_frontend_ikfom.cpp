#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <omp.h>

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

}  // namespace

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

  const Eigen::Vector3d acc_std = (acc_m2 / static_cast<double>(count - 1)).cwiseMax(0.0).cwiseSqrt();
  const Eigen::Vector3d gyro_std = (gyro_m2 / static_cast<double>(count - 1)).cwiseMax(0.0).cwiseSqrt();
  if (acc_std.maxCoeff() > impl_->parameters.max_static_accel_std_m_s2 ||
      gyro_std.maxCoeff() > impl_->parameters.max_static_gyro_std_rad_s)
    return fail(failure_reason, "static_imu_variance_exceeds_gate");

  const Eigen::Vector3d mean_specific_force = mean_acc - impl_->parameters.initial_accel_bias;
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
  state.bg = vect3(mean_gyro);
  state.ba = vect3(impl_->parameters.initial_accel_bias);
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
  impl_->stamp_ns = samples[count - 1].stamp_ns;
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
  if (failure_reason) failure_reason->clear();
  if (!impl_->is_initialized) return fail(failure_reason, "filter_not_initialized");
  if (!finitePose(map_T_imu_measurement))
    return fail(failure_reason, "invalid_pose_measurement");

  const FilterSnapshot before = getState();
  PoseMeasurement measurement(vect3(map_T_imu_measurement.position),
                              SO3(map_T_imu_measurement.orientation.toRotationMatrix()));
  Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
  noise.block<3, 3>(0, 0).diagonal().setConstant(
      impl_->parameters.pose_position_sigma_m * impl_->parameters.pose_position_sigma_m);
  noise.block<3, 3>(3, 3).diagonal().setConstant(
      impl_->parameters.pose_rotation_sigma_rad * impl_->parameters.pose_rotation_sigma_rad);

  impl_->filter.update_iterated(measurement, noise);
  state_ikfom state = impl_->filter.get_x();
  auto covariance = impl_->filter.get_P();
  enforceFixedExtrinsicConstraint(state, covariance, impl_->fixed_rotation,
                                  impl_->fixed_translation);
  impl_->filter.change_x(state);
  impl_->filter.change_P(covariance);
  if (!postconditionsValid(failure_reason)) return false;

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
