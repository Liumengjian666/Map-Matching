#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d& vector) {
  const double angle = vector.norm();
  if (angle < 1e-10)
    return Eigen::Matrix3d::Identity() + skew(vector);
  return Eigen::AngleAxisd(angle, vector / angle).toRotationMatrix();
}

Eigen::Matrix3d so3RightJacobian(const Eigen::Vector3d& vector) {
  const double angle = vector.norm();
  const Eigen::Matrix3d hat = skew(vector);
  if (angle < 1e-8)
    return Eigen::Matrix3d::Identity() - 0.5 * hat +
        (1.0 / 6.0) * hat * hat;
  const double angle2 = angle * angle;
  return Eigen::Matrix3d::Identity() -
      ((1.0 - std::cos(angle)) / angle2) * hat +
      ((angle - std::sin(angle)) / (angle2 * angle)) * hat * hat;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  if (quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
  const double sine_half = quaternion.vec().norm();
  if (sine_half < 1e-12) return 2.0 * quaternion.vec();
  const double angle = 2.0 * std::atan2(sine_half,
      std::clamp(quaternion.w(), -1.0, 1.0));
  return quaternion.vec() * (angle / sine_half);
}

bool finiteSample(const ImuSample& sample) {
  return sample.stamp_ns > 0 && sample.acceleration.allFinite() &&
      sample.angular_velocity.allFinite();
}

ImuSample interpolate(const ImuSample& before, const ImuSample& after,
                      std::uint64_t stamp_ns) {
  ImuSample result;
  result.stamp_ns = stamp_ns;
  const double alpha = static_cast<double>(stamp_ns - before.stamp_ns) /
      static_cast<double>(after.stamp_ns - before.stamp_ns);
  result.acceleration = (1.0 - alpha) * before.acceleration +
      alpha * after.acceleration;
  result.angular_velocity = (1.0 - alpha) * before.angular_velocity +
      alpha * after.angular_velocity;
  return result;
}

bool collectBoundarySamples(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    std::uint64_t start_stamp_ns, std::uint64_t end_stamp_ns,
    std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>* output,
    std::string* reason) {
  if (!output) return fail(reason, "null_imu_boundary_output");
  output->clear();
  if (start_stamp_ns == 0 || end_stamp_ns <= start_stamp_ns || samples.size() < 2)
    return fail(reason, "invalid_imu_preintegration_interval");
  std::uint64_t previous = 0;
  for (const ImuSample& sample : samples) {
    if (!finiteSample(sample) || (previous != 0 && sample.stamp_ns <= previous))
      return fail(reason, "imu_samples_not_strictly_monotonic");
    previous = sample.stamp_ns;
  }
  if (start_stamp_ns < samples.front().stamp_ns ||
      end_stamp_ns > samples.back().stamp_ns)
    return fail(reason, "imu_samples_do_not_bracket_interval");

  auto appendAt = [&](std::uint64_t stamp) {
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (samples[i].stamp_ns == stamp) {
        output->push_back(samples[i]);
        return true;
      }
      if (samples[i].stamp_ns > stamp && i > 0) {
        output->push_back(interpolate(samples[i - 1], samples[i], stamp));
        return true;
      }
    }
    return false;
  };
  if (!appendAt(start_stamp_ns)) return fail(reason, "missing_imu_start_interpolation");
  for (const ImuSample& sample : samples) {
    if (sample.stamp_ns > start_stamp_ns && sample.stamp_ns < end_stamp_ns)
      output->push_back(sample);
  }
  if (!appendAt(end_stamp_ns)) return fail(reason, "missing_imu_end_interpolation");
  return output->size() >= 2;
}

}  // namespace

static bool integrateBoundedImuSequence(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    std::uint64_t start_stamp_ns, std::uint64_t end_stamp_ns,
    const Eigen::Vector3d& linearization_gyro_bias,
    const Eigen::Vector3d& linearization_accel_bias,
    const ImuNoiseParameters& noise, ImuPreintegratedMeasurement* output,
    std::string* reason, const WindowState* anchor = nullptr,
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>* trajectory = nullptr) {
  if (reason) reason->clear();
  if (!output) return fail(reason, "null_preintegration_output");
  *output = ImuPreintegratedMeasurement();
  if (!linearization_gyro_bias.allFinite() || !linearization_accel_bias.allFinite() ||
      !noise.gravity.allFinite() || !std::isfinite(noise.gyro_noise_density) ||
      !std::isfinite(noise.accel_noise_density) ||
      !std::isfinite(noise.gyro_bias_random_walk) ||
      !std::isfinite(noise.accel_bias_random_walk) ||
      noise.gyro_noise_density < 0.0 || noise.accel_noise_density < 0.0 ||
      noise.gyro_bias_random_walk < 0.0 || noise.accel_bias_random_walk < 0.0)
    return fail(reason, "invalid_imu_noise_or_bias");
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> bounded;
  if (!collectBoundarySamples(samples, start_stamp_ns, end_stamp_ns, &bounded,
                               reason)) return false;

  output->start_stamp_ns = start_stamp_ns;
  output->end_stamp_ns = end_stamp_ns;
  output->dt_s = static_cast<double>(end_stamp_ns - start_stamp_ns) * 1e-9;
  output->linearization_gyro_bias = linearization_gyro_bias;
  output->linearization_accel_bias = linearization_accel_bias;
  auto appendKnot = [&](std::uint64_t stamp, const Eigen::Vector3d& world_acceleration,
                        const Eigen::Vector3d& unbiased_gyro) {
    if (!trajectory) return;
    ImuPreintegratedMeasurement cumulative = *output;
    cumulative.end_stamp_ns = stamp;
    cumulative.dt_s = static_cast<double>(stamp - start_stamp_ns) * 1e-9;
    const WindowState state = propagateWindowState(*anchor, cumulative, noise.gravity);
    ImuPoseSample pose;
    pose.stamp_ns = stamp;
    pose.rotation = state.rotation;
    pose.position = state.position;
    pose.velocity = state.velocity;
    pose.world_acceleration = world_acceleration;
    pose.unbiased_gyro = unbiased_gyro;
    trajectory->push_back(pose);
  };
  if (trajectory) trajectory->clear();
  if (anchor) appendKnot(start_stamp_ns,
      anchor->rotation * (bounded.front().acceleration-linearization_accel_bias) + noise.gravity,
      bounded.front().angular_velocity-linearization_gyro_bias);
  for (std::size_t index = 0; index + 1 < bounded.size(); ++index) {
    const double dt = static_cast<double>(bounded[index + 1].stamp_ns -
                                          bounded[index].stamp_ns) * 1e-9;
    if (!(dt > 0.0) || !std::isfinite(dt))
      return fail(reason, "nonpositive_imu_interval");
    const Eigen::Vector3d omega = 0.5 *
        (bounded[index].angular_velocity + bounded[index + 1].angular_velocity) -
        linearization_gyro_bias;
    const Eigen::Vector3d acceleration = 0.5 *
        (bounded[index].acceleration + bounded[index + 1].acceleration) -
        linearization_accel_bias;
    const Eigen::Matrix3d old_rotation = output->delta_rotation;
    const Eigen::Vector3d old_velocity = output->delta_velocity;
    const Eigen::Matrix3d old_jr_bg = output->jacobian_rotation_gyro_bias;
    const Eigen::Matrix3d old_jv_bg = output->jacobian_velocity_gyro_bias;
    const Eigen::Matrix3d old_jv_ba = output->jacobian_velocity_accel_bias;
    const Eigen::Matrix3d old_jp_bg = output->jacobian_position_gyro_bias;
    const Eigen::Matrix3d old_jp_ba = output->jacobian_position_accel_bias;
    const Eigen::Vector3d rotated_acceleration = old_rotation * acceleration;
    output->delta_position += old_velocity * dt +
        0.5 * rotated_acceleration * dt * dt;
    output->delta_velocity += rotated_acceleration * dt;
    const Eigen::Vector3d rotation_vector = omega * dt;
    const Eigen::Matrix3d rotation_increment = so3Exp(rotation_vector);
    output->delta_rotation = old_rotation * rotation_increment;
    // Right-tangent bias Jacobian:
    // DeltaR(b + db) ~= DeltaR(b) Exp(J_R_bg db).
    output->jacobian_rotation_gyro_bias =
        rotation_increment.transpose() * old_jr_bg -
        so3RightJacobian(rotation_vector) * dt;
    output->jacobian_velocity_gyro_bias = old_jv_bg -
        old_rotation * skew(acceleration) * old_jr_bg * dt;
    output->jacobian_velocity_accel_bias = old_jv_ba - old_rotation * dt;
    output->jacobian_position_gyro_bias = old_jp_bg + old_jv_bg * dt -
        0.5 * old_rotation * skew(acceleration) * old_jr_bg * dt * dt;
    output->jacobian_position_accel_bias = old_jp_ba + old_jv_ba * dt -
        0.5 * old_rotation * dt * dt;

    Matrix15d transition = Matrix15d::Identity();
    transition.block<3, 3>(0, 0) = so3Exp(-omega * dt);
    transition.block<3, 3>(6, 0) = -old_rotation * skew(acceleration) * dt;
    transition.block<3, 3>(6, 12) = -old_rotation * dt;
    transition.block<3, 3>(3, 0) = -0.5 * old_rotation * skew(acceleration) * dt * dt;
    transition.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    transition.block<3, 3>(3, 12) = -0.5 * old_rotation * dt * dt;
    const Eigen::Matrix3d right_jacobian = so3RightJacobian(rotation_vector);
    transition.block<3, 3>(0, 9) = -right_jacobian * dt;
    Eigen::Matrix<double, 15, 12> noise_map =
        Eigen::Matrix<double, 15, 12>::Zero();
    noise_map.block<3, 3>(0, 0) = -right_jacobian * dt;
    noise_map.block<3, 3>(6, 3) = old_rotation * dt;
    noise_map.block<3, 3>(3, 3) = 0.5 * old_rotation * dt * dt;
    noise_map.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
    noise_map.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;
    Eigen::Matrix<double, 12, 12> continuous_noise =
        Eigen::Matrix<double, 12, 12>::Zero();
    continuous_noise.block<3, 3>(0, 0).diagonal().setConstant(
        noise.gyro_noise_density * noise.gyro_noise_density);
    continuous_noise.block<3, 3>(3, 3).diagonal().setConstant(
        noise.accel_noise_density * noise.accel_noise_density);
    continuous_noise.block<3, 3>(6, 6).diagonal().setConstant(
        noise.gyro_bias_random_walk * noise.gyro_bias_random_walk);
    continuous_noise.block<3, 3>(9, 9).diagonal().setConstant(
        noise.accel_bias_random_walk * noise.accel_bias_random_walk);
    // The parameters are continuous-time noise densities. noise_map contains
    // the interval-integrated state sensitivities, so the equivalent sampled
    // input covariance is Qc / dt (yielding, e.g., sigma_g^2 * dt for angle
    // and sigma_bg^2 * dt for a bias random walk).
    const Eigen::Matrix<double, 12, 12> sampled_noise =
        continuous_noise / dt;
    output->covariance = transition * output->covariance * transition.transpose() +
        noise_map * sampled_noise * noise_map.transpose();
    output->covariance = 0.5 *
        (output->covariance + output->covariance.transpose());
    if (anchor) appendKnot(bounded[index + 1].stamp_ns,
        anchor->rotation * rotated_acceleration + noise.gravity, omega);
  }
  Eigen::SelfAdjointEigenSolver<Matrix15d> covariance_solver(output->covariance);
  if (covariance_solver.info() != Eigen::Success ||
      !output->delta_rotation.allFinite() || !output->delta_velocity.allFinite() ||
      !output->delta_position.allFinite() ||
      covariance_solver.eigenvalues().minCoeff() < -1e-10)
    return fail(reason, "invalid_imu_preintegration_result");
  output->physical_covariance = output->covariance;
  output->physical_min_eigenvalue = covariance_solver.eigenvalues().minCoeff();
  output->factor_min_eigenvalue = output->physical_min_eigenvalue;
  output->valid = true;
  output->status = "PASS_PREINTEGRATED_IMU_WITH_BIAS_JACOBIANS";
  return true;
}

WindowState propagateWindowState(const WindowState& anchor,
    const ImuPreintegratedMeasurement& measurement,
    const Eigen::Vector3d& gravity) {
  WindowState result = anchor;
  const double dt = measurement.dt_s;
  result.stamp_ns = measurement.end_stamp_ns;
  result.rotation = anchor.rotation * measurement.delta_rotation;
  result.velocity = anchor.velocity + gravity * dt +
      anchor.rotation * measurement.delta_velocity;
  result.position = anchor.position + anchor.velocity * dt +
      0.5 * gravity * dt * dt + anchor.rotation * measurement.delta_position;
  return result;
}

bool preintegrateImu(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    std::uint64_t start, std::uint64_t end, const Eigen::Vector3d& bg,
    const Eigen::Vector3d& ba, const ImuNoiseParameters& noise,
    ImuPreintegratedMeasurement* output, std::string* reason) {
  return integrateBoundedImuSequence(samples, start, end, bg, ba, noise, output, reason);
}

bool integrateWindowImuTrajectory(
    const WindowState& anchor, std::uint64_t end,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    const ImuNoiseParameters& noise, ImuPreintegratedMeasurement* measurement,
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>* trajectory,
    std::string* reason) {
  if (!trajectory || !anchor.rotation.allFinite() || !anchor.position.allFinite() ||
      !anchor.velocity.allFinite() ||
      (anchor.rotation.transpose()*anchor.rotation-Eigen::Matrix3d::Identity()).norm()>1e-7 ||
      std::abs(anchor.rotation.determinant()-1.0)>1e-7)
    return fail(reason, "invalid_window_trajectory_anchor");
  return integrateBoundedImuSequence(samples, anchor.stamp_ns, end,
      anchor.gyro_bias, anchor.accel_bias, noise, measurement, reason, &anchor, trajectory);
}

bool applyLocalIncrement(WindowState* state, const Vector15d& increment,
                         std::string* reason) {
  if (!state) return fail(reason, "null_window_state");
  if (!increment.allFinite()) return fail(reason, "nonfinite_window_increment");
  state->rotation = state->rotation * so3Exp(increment.segment<3>(0));
  state->position += increment.segment<3>(3);
  state->velocity += increment.segment<3>(6);
  state->gyro_bias += increment.segment<3>(9);
  state->accel_bias += increment.segment<3>(12);
  if (!state->rotation.allFinite() || !state->position.allFinite() ||
      !state->velocity.allFinite() || !state->gyro_bias.allFinite() ||
      !state->accel_bias.allFinite())
    return fail(reason, "nonfinite_window_state_after_increment");
  return true;
}

Vector15d localDifference(const WindowState& state,
                          const WindowState& reference) {
  Vector15d result;
  result.segment<3>(0) = so3Log(reference.rotation.transpose() * state.rotation);
  result.segment<3>(3) = state.position - reference.position;
  result.segment<3>(6) = state.velocity - reference.velocity;
  result.segment<3>(9) = state.gyro_bias - reference.gyro_bias;
  result.segment<3>(12) = state.accel_bias - reference.accel_bias;
  return result;
}

bool buildImuResidual(const WindowState& from, const WindowState& to,
                      const ImuPreintegratedMeasurement& measurement,
                      const ImuNoiseParameters& noise, Vector15d* residual,
                      std::string* reason) {
  if (reason) reason->clear();
  if (!residual) return fail(reason, "null_imu_residual");
  if (!measurement.valid || from.stamp_ns != measurement.start_stamp_ns ||
      to.stamp_ns != measurement.end_stamp_ns || to.stamp_ns <= from.stamp_ns)
    return fail(reason, "invalid_imu_factor_endpoint");
  const Eigen::Vector3d delta_bg = from.gyro_bias -
      measurement.linearization_gyro_bias;
  const Eigen::Vector3d delta_ba = from.accel_bias -
      measurement.linearization_accel_bias;
  const Eigen::Vector3d dtheta = measurement.jacobian_rotation_gyro_bias * delta_bg;
  const Eigen::Vector3d dv = measurement.delta_velocity +
      measurement.jacobian_velocity_gyro_bias * delta_bg +
      measurement.jacobian_velocity_accel_bias * delta_ba;
  const Eigen::Vector3d dp = measurement.delta_position +
      measurement.jacobian_position_gyro_bias * delta_bg +
      measurement.jacobian_position_accel_bias * delta_ba;
  residual->setZero();
  residual->segment<3>(0) = so3Log(
      (measurement.delta_rotation * so3Exp(dtheta)).transpose() *
      from.rotation.transpose() * to.rotation);
  residual->segment<3>(3) = from.rotation.transpose() *
      (to.position - from.position - from.velocity * measurement.dt_s -
       0.5 * noise.gravity * measurement.dt_s * measurement.dt_s) - dp;
  residual->segment<3>(6) = from.rotation.transpose() *
      (to.velocity - from.velocity - noise.gravity * measurement.dt_s) - dv;
  residual->segment<3>(9) = to.gyro_bias - from.gyro_bias;
  residual->segment<3>(12) = to.accel_bias - from.accel_bias;
  if (!residual->allFinite()) return fail(reason, "nonfinite_imu_residual");
  return true;
}

bool linearizeImuFactorFiniteDifferenceReference(
    const WindowState& from, const WindowState& to,
    const ImuPreintegratedMeasurement& measurement,
    const ImuNoiseParameters& noise, Eigen::Matrix<double, 15, 15>* jacobian_from,
    Eigen::Matrix<double, 15, 15>* jacobian_to, Vector15d* residual,
    std::string* reason) {
  if (!jacobian_from || !jacobian_to || !residual)
    return fail(reason, "null_imu_linearization_output");
  if (!buildImuResidual(from, to, measurement, noise, residual, reason))
    return false;
  jacobian_from->setZero();
  jacobian_to->setZero();
  constexpr double h = 1e-7;
  for (int column = 0; column < 15; ++column) {
    Vector15d increment = Vector15d::Zero();
    increment(column) = h;
    WindowState from_plus = from, from_minus = from;
    WindowState to_plus = to, to_minus = to;
    applyLocalIncrement(&from_plus, increment);
    applyLocalIncrement(&from_minus, -increment);
    applyLocalIncrement(&to_plus, increment);
    applyLocalIncrement(&to_minus, -increment);
    Vector15d plus, minus;
    if (!buildImuResidual(from_plus, to, measurement, noise, &plus, reason) ||
        !buildImuResidual(from_minus, to, measurement, noise, &minus, reason))
      return false;
    jacobian_from->col(column) = (plus - minus) / (2.0 * h);
    if (!buildImuResidual(from, to_plus, measurement, noise, &plus, reason) ||
        !buildImuResidual(from, to_minus, measurement, noise, &minus, reason))
      return false;
    jacobian_to->col(column) = (plus - minus) / (2.0 * h);
  }
  return jacobian_from->allFinite() && jacobian_to->allFinite();
}

bool linearizeImuFactor(
    const WindowState& from, const WindowState& to,
    const ImuPreintegratedMeasurement& m, const ImuNoiseParameters& noise,
    Matrix15d* a, Matrix15d* b, Vector15d* residual, std::string* reason) {
  if (!a || !b || !residual) return fail(reason, "null_imu_linearization_output");
  if (!buildImuResidual(from, to, m, noise, residual, reason)) return false;
  const Eigen::Vector3d phi = residual->head<3>();
  // Principal Log is not differentiable exactly at pi. Do not hide this with
  // an arbitrary near-pi rejection band or a silently selected FD derivative.
  if (std::abs(phi.norm() - std::acos(-1.0)) < 1e-12)
    return fail(reason, "imu_rotation_log_branch_cut");
  auto leftInverse = [](const Eigen::Vector3d& x) {
    const double theta = x.norm();
    const Eigen::Matrix3d hat = skew(x);
    const double coefficient = theta < 1e-5 ?
        1.0 / 12.0 + theta * theta / 720.0 :
        (1.0 - 0.5 * theta / std::tan(0.5 * theta)) / (theta * theta);
    return Eigen::Matrix3d(Eigen::Matrix3d::Identity() - 0.5 * hat + coefficient * hat * hat);
  };
  const Eigen::Matrix3d li = leftInverse(phi);
  const Eigen::Matrix3d ri = leftInverse(-phi);
  const Eigen::Matrix3d rt = from.rotation.transpose();
  const Eigen::Vector3d beta = m.jacobian_rotation_gyro_bias *
      (from.gyro_bias - m.linearization_gyro_bias);
  const Eigen::Matrix3d corrected = m.delta_rotation * so3Exp(beta);
  a->setZero(); b->setZero();
  a->block<3,3>(0,0) = -li * corrected.transpose();
  b->block<3,3>(0,0) = ri;
  a->block<3,3>(0,9) = -li * so3RightJacobian(beta) * m.jacobian_rotation_gyro_bias;
  a->block<3,3>(3,0) = skew(rt * (to.position - from.position -
      from.velocity * m.dt_s - 0.5 * noise.gravity * m.dt_s * m.dt_s));
  a->block<3,3>(3,3) = -rt; b->block<3,3>(3,3) = rt;
  a->block<3,3>(3,6) = -rt * m.dt_s;
  a->block<3,3>(3,9) = -m.jacobian_position_gyro_bias;
  a->block<3,3>(3,12) = -m.jacobian_position_accel_bias;
  a->block<3,3>(6,0) = skew(rt * (to.velocity - from.velocity - noise.gravity * m.dt_s));
  a->block<3,3>(6,6) = -rt; b->block<3,3>(6,6) = rt;
  a->block<3,3>(6,9) = -m.jacobian_velocity_gyro_bias;
  a->block<3,3>(6,12) = -m.jacobian_velocity_accel_bias;
  a->block<3,3>(9,9) = -Eigen::Matrix3d::Identity();
  b->block<3,3>(9,9).setIdentity();
  a->block<3,3>(12,12) = -Eigen::Matrix3d::Identity();
  b->block<3,3>(12,12).setIdentity();
  return a->allFinite() && b->allFinite();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
