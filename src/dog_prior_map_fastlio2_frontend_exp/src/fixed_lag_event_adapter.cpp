#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool failLocal(std::string* reason, const std::string& message) {
  if (reason) *reason = message;
  return false;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
  return result;
}

Eigen::Matrix3d exp3(const Eigen::Vector3d& v) {
  const double angle = v.norm();
  if (angle < 1e-12) return Eigen::Matrix3d::Identity() + skew(v);
  return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

Eigen::Vector3d log3(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond q(rotation);
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double sine_half = q.vec().norm();
  if (sine_half < 1e-12) return 2.0 * q.vec();
  const double angle = 2.0 * std::atan2(sine_half,
      std::clamp(q.w(), -1.0, 1.0));
  return q.vec() * (angle / sine_half);
}

bool finitePose(const Pose3d& pose) {
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
      std::isfinite(pose.orientation.norm()) &&
      pose.orientation.norm() > 1e-12;
}

bool spd(const Eigen::MatrixXd& covariance) {
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return false;
  Eigen::LLT<Eigen::MatrixXd> factor(covariance);
  return factor.info() == Eigen::Success;
}

}  // namespace

const char* toString(AdapterEventDisposition disposition) {
  switch (disposition) {
    case AdapterEventDisposition::ACCEPTED: return "ACCEPTED";
    case AdapterEventDisposition::DUPLICATE_SOURCE: return "DUPLICATE_SOURCE";
    case AdapterEventDisposition::SKIPPED_INVALID_SOURCE:
      return "SKIPPED_INVALID_SOURCE";
    case AdapterEventDisposition::REJECTED_CAUSALITY:
      return "REJECTED_CAUSALITY";
    case AdapterEventDisposition::REJECTED_WINDOW:
      return "REJECTED_WINDOW";
    case AdapterEventDisposition::REJECTED_FACTOR:
    default: return "REJECTED_FACTOR";
  }
}

FixedLagEventAdapter::FixedLagEventAdapter(
    const FixedLagOptions& options, const ImuNoiseParameters& imu_noise,
    const FixedLagAdapterCalibration& calibration)
    : controller_(WindowExecutionMode::FULL_FIXED_LAG_EXPERIMENTAL, options,
                   imu_noise),
      calibration_(calibration), imu_noise_(imu_noise) {}

void FixedLagEventAdapter::setStatus(AdapterEventDisposition disposition,
                                     std::uint64_t id,
                                     const std::string& reason,
                                     std::string* output_reason) {
  last_status_.disposition = disposition;
  last_status_.observation_id = id;
  last_status_.reason = reason;
  if (output_reason) *output_reason = reason;
}

bool FixedLagEventAdapter::fail(AdapterEventDisposition disposition,
                                const std::string& reason,
                                std::string* output_reason) {
  setStatus(disposition, 0, reason, output_reason);
  return false;
}

bool FixedLagEventAdapter::initialize(const WindowState& initial_state,
                                      const Matrix15d& initial_information,
                                      const Vector15d& initial_gradient,
                                      std::string* reason) {
  if (reason) reason->clear();
  if (initialized_) return fail(AdapterEventDisposition::REJECTED_WINDOW,
                                "adapter_already_initialized", reason);
  if (!controller_.enabled())
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                "experimental_controller_disabled", reason);
  if (!controller_.addState(initial_state, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "initial_state_rejected", reason);
    return false;
  }
  if (!controller_.setInitialPrior(initial_state.stamp_ns,
                                   initial_information, initial_gradient,
                                   reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "initial_prior_rejected", reason);
    return false;
  }
  initialized_ = true;
  last_status_ = {AdapterEventDisposition::ACCEPTED, 0, "INITIALIZED"};
  return true;
}

bool FixedLagEventAdapter::appendImu(const ImuSample& sample,
                                     std::string* reason) {
  if (reason) reason->clear();
  if (!initialized_)
    return fail(AdapterEventDisposition::REJECTED_CAUSALITY,
                "adapter_not_initialized", reason);
  if (sample.stamp_ns == 0 || !sample.acceleration.allFinite() ||
      !sample.angular_velocity.allFinite())
    return fail(AdapterEventDisposition::REJECTED_CAUSALITY,
                "invalid_imu_sample", reason);
  if (last_imu_stamp_ns_ != 0 && sample.stamp_ns <= last_imu_stamp_ns_)
    return fail(AdapterEventDisposition::DUPLICATE_SOURCE,
                sample.stamp_ns == last_imu_stamp_ns_
                    ? "duplicate_imu_timestamp" : "imu_timestamp_regression",
                reason);
  imu_samples_.push_back(sample);
  last_imu_stamp_ns_ = sample.stamp_ns;
  last_status_ = {AdapterEventDisposition::ACCEPTED, 0, "IMU_BUFFERED"};
  return true;
}

bool FixedLagEventAdapter::sourceSeen(const SourceKey& key) const {
  return source_records_.count(key) != 0;
}

std::uint64_t FixedLagEventAdapter::allocateObservationId() {
  return next_observation_id_;
}

bool FixedLagEventAdapter::accept(std::uint64_t observation_id,
                                  std::string* output_reason) {
  if (observation_id == 0 || observation_id != next_observation_id_)
    return fail(AdapterEventDisposition::REJECTED_FACTOR,
                "internal_observation_id_allocation_error", output_reason);
  ++next_observation_id_;
  return true;
}

Pose3d FixedLagEventAdapter::lidarToImu(const Pose3d& map_T_lidar) const {
  Pose3d result;
  const Eigen::Quaterniond q_il = calibration_.T_imu_lidar.orientation.normalized();
  const Eigen::Quaterniond q_li = q_il.conjugate();
  result.orientation = (map_T_lidar.orientation.normalized() * q_li).normalized();
  result.position = map_T_lidar.position -
      result.orientation * calibration_.T_imu_lidar.position;
  return result;
}

bool FixedLagEventAdapter::buildPredictedState(
    std::uint64_t stamp_ns, WindowState* output,
    ImuPreintegratedMeasurement* preintegrated, std::string* reason) const {
  if (!output || !preintegrated)
    return failLocal(reason, "null_prediction_output");
  WindowState latest;
  if (!controller_.latestState(&latest, reason)) return false;
  if (stamp_ns <= latest.stamp_ns)
    return failLocal(reason, stamp_ns == latest.stamp_ns
                         ? "state_already_exists" : "event_timestamp_regression");
  if (imu_samples_.size() < 2 || last_imu_stamp_ns_ < stamp_ns)
    return failLocal(reason, "imu_history_not_causal_for_event");
  if (!preintegrateImu(imu_samples_, latest.stamp_ns, stamp_ns,
                       latest.gyro_bias, latest.accel_bias, imu_noise_,
                       preintegrated, reason)) return false;
  // The existing preintegrator returns a physically valid PSD covariance for
  // one interval (the 15D process noise has only 12 driving channels).  The
  // window factor currently requires an SPD solve.  Add only a numerical
  // solve floor; this is not a replacement covariance and is reported in the
  // measurement status for downstream audit.
  Eigen::LLT<Matrix15d> covariance_factor(preintegrated->covariance);
  if (covariance_factor.info() != Eigen::Success) {
    preintegrated->covariance.diagonal().array() += 1e-9;
    preintegrated->status += ";WINDOW_SOLVE_NUMERICAL_FLOOR_1E-9";
    covariance_factor.compute(preintegrated->covariance);
    if (covariance_factor.info() != Eigen::Success)
      return failLocal(reason, "imu_preintegration_covariance_not_solveable");
  }
  *output = latest;
  output->stamp_ns = stamp_ns;
  output->rotation = latest.rotation * preintegrated->delta_rotation;
  output->velocity = latest.velocity + imu_noise_.gravity * preintegrated->dt_s +
      latest.rotation * preintegrated->delta_velocity;
  output->position = latest.position + latest.velocity * preintegrated->dt_s +
      0.5 * imu_noise_.gravity * preintegrated->dt_s * preintegrated->dt_s +
      latest.rotation * preintegrated->delta_position;
  return output->rotation.allFinite() && output->position.allFinite() &&
      output->velocity.allFinite();
}

bool FixedLagEventAdapter::ensureStateAt(std::uint64_t stamp_ns,
                                         std::string* reason) {
  WindowState existing;
  if (controller_.stateAt(stamp_ns, &existing, nullptr)) return true;
  WindowState predicted;
  ImuPreintegratedMeasurement preintegrated;
  if (!buildPredictedState(stamp_ns, &predicted, &preintegrated, reason))
    return false;
  WindowState latest;
  if (!controller_.latestState(&latest, reason)) return false;
  if (!controller_.addState(predicted, reason)) return false;
  const SourceKey key(0, latest.stamp_ns, stamp_ns);
  if (sourceSeen(key)) return failLocal(reason, "duplicate_imu_interval_source");
  const std::uint64_t id = allocateObservationId();
  if (!controller_.addImuFactor(id, latest.stamp_ns, stamp_ns,
                                preintegrated, reason)) return false;
  if (!accept(id, reason)) return false;
  source_records_.insert(key);
  return true;
}

bool FixedLagEventAdapter::rawLidarResidual(
    const WindowState& state, const Pose3d& map_T_imu,
    Eigen::Matrix<double, 6, 1>* residual, std::string* reason) const {
  if (!residual) return failLocal(reason, "null_lidar_basis_residual");
  if (state.stamp_ns == 0 || !finitePose(map_T_imu))
    return failLocal(reason, "invalid_lidar_basis_pose");
  residual->head<3>() = map_T_imu.position - state.position;
  residual->tail<3>() = log3(state.rotation.transpose() *
                             map_T_imu.orientation.normalized().toRotationMatrix());
  return residual->allFinite();
}

bool FixedLagEventAdapter::rebuildLidarBasis(
    const FrozenLidarEvent& event, const WindowState& state, Matrix6d* basis,
    int* rank, std::string* reason) const {
  if (!basis || !rank) return failLocal(reason, "null_lidar_basis_output");
  const int weak_dimension = event.local_risk.weak_dimension;
  if (!event.local_risk.valid || weak_dimension < 0 || weak_dimension > 6 ||
      event.local_risk.reliable_dimension != 6 - weak_dimension ||
      !event.local_risk.joint_weak_basis.allFinite() ||
      !std::isfinite(event.local_risk.translation_length_scale_m) ||
      event.local_risk.translation_length_scale_m <= 0.0)
    return failLocal(reason, "invalid_lidar_weak_basis_input");
  if (weak_dimension == 0) {
    basis->setIdentity();
    *rank = 6;
    return true;
  }
  Eigen::Matrix<double, 6, 6> exact_jacobian = Matrix6d::Zero();
  constexpr double h = 1e-7;
  for (int axis = 0; axis < 6; ++axis) {
    FrozenLidarEvent plus_event = event;
    FrozenLidarEvent minus_event = event;
    const Eigen::Vector3d dphi = h * Eigen::Vector3d::Unit(axis);
    if (axis < 3) {
      plus_event.map_T_lidar.orientation =
          (Eigen::Quaterniond(exp3(dphi)) * event.map_T_lidar.orientation).normalized();
      minus_event.map_T_lidar.orientation =
          (Eigen::Quaterniond(exp3(-dphi)) * event.map_T_lidar.orientation).normalized();
    } else {
      plus_event.map_T_lidar.position += h *
          event.local_risk.translation_length_scale_m *
          Eigen::Vector3d::Unit(axis - 3);
      minus_event.map_T_lidar.position -= h *
          event.local_risk.translation_length_scale_m *
          Eigen::Vector3d::Unit(axis - 3);
    }
    Eigen::Matrix<double, 6, 1> plus_residual, minus_residual;
    if (!rawLidarResidual(state, lidarToImu(plus_event.map_T_lidar),
                          &plus_residual, reason) ||
        !rawLidarResidual(state, lidarToImu(minus_event.map_T_lidar),
                          &minus_residual, reason)) return false;
    exact_jacobian.col(axis) = (plus_residual - minus_residual) / (2.0 * h);
  }
  return reliability::buildReliableMeasurementBasisFromWeak(
      exact_jacobian, event.local_risk.joint_weak_basis, weak_dimension,
      basis, rank, reason);
}

bool FixedLagEventAdapter::convertLidarMeasurement(
    const FrozenLidarEvent& event, const WindowState& predicted_state,
    LidarWindowMeasurement* output, std::string* reason) const {
  if (!output) return failLocal(reason, "null_lidar_measurement_output");
  if (!finitePose(event.map_T_lidar) || !spd(event.residual_covariance))
    return failLocal(reason, "invalid_lidar_pose_or_covariance");
  *output = LidarWindowMeasurement();
  output->stamp_ns = event.stamp_ns;
  const Pose3d map_T_imu = lidarToImu(event.map_T_lidar);
  output->measured_rotation = map_T_imu.orientation.normalized().toRotationMatrix();
  output->measured_position = map_T_imu.position;
  output->covariance = event.residual_covariance;
  output->valid = true;
  if (!rebuildLidarBasis(event, predicted_state, &output->measurement_basis,
                         &output->reliable_rank, reason)) return false;
  if (event.local_risk.weak_dimension > 0) {
    output->basis_relinearizer =
        [this, event](const WindowState& state, Matrix6d* basis, int* rank,
                      std::string* local_reason) {
          return rebuildLidarBasis(event, state, basis, rank, local_reason);
        };
  }
  return true;
}

bool FixedLagEventAdapter::processLidarEvent(const FrozenLidarEvent& event,
                                             std::string* reason) {
  if (reason) reason->clear();
  if (!initialized_)
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                "adapter_not_initialized", reason);
  const SourceKey key(1, event.transaction_id, 0);
  if (event.transaction_id == 0)
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "invalid_lidar_transaction_id", reason);
  if (sourceSeen(key))
    return fail(AdapterEventDisposition::DUPLICATE_SOURCE,
                "duplicate_lidar_source", reason);
  if (!event.ndt_converged) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "ndt_not_converged", reason);
  }
  if (!event.map_support_valid) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "map_support_insufficient", reason);
  }
  if (!event.local_risk.valid || event.local_risk.reliable_dimension <= 0) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "lidar_reliable_rank_zero_or_invalid", reason);
  }
  if (event.stamp_ns == 0) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "invalid_lidar_timestamp", reason);
  }
  if (!ensureStateAt(event.stamp_ns, reason)) {
    const AdapterEventDisposition disposition =
        reason && reason->find("causal") != std::string::npos
            ? AdapterEventDisposition::REJECTED_CAUSALITY
            : AdapterEventDisposition::REJECTED_WINDOW;
    setStatus(disposition, 0, reason ? *reason : "state_creation_failed", reason);
    return false;
  }
  WindowState state;
  if (!controller_.stateAt(event.stamp_ns, &state, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_WINDOW, 0,
              reason ? *reason : "lidar_state_not_in_window", reason);
    return false;
  }
  LidarWindowMeasurement measurement;
  if (!convertLidarMeasurement(event, state, &measurement, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "lidar_conversion_failed", reason);
    return false;
  }
  const std::uint64_t id = allocateObservationId();
  measurement.observation_id = id;
  if (!controller_.addLidarFactor(measurement, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "lidar_factor_rejected", reason);
    return false;
  }
  if (!accept(id, reason)) return false;
  source_records_.insert(key);
  setStatus(AdapterEventDisposition::ACCEPTED, id, "LIDAR_FACTOR_ACCEPTED", reason);
  return true;
}

bool FixedLagEventAdapter::processVisualEvent(const FrozenVisualEvent& event,
                                              std::string* reason) {
  if (reason) reason->clear();
  if (!initialized_)
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                "adapter_not_initialized", reason);
  const SourceKey key(2, event.ref_ns, event.cur_ns);
  if (event.ref_ns == 0 || event.cur_ns <= event.ref_ns ||
      event.depth_ns == 0 || event.depth_ns > event.ref_ns) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "invalid_visual_timestamp_order", reason);
  }
  if (sourceSeen(key))
    return fail(AdapterEventDisposition::DUPLICATE_SOURCE,
                "duplicate_visual_source", reason);
  reliability::VisualQualityObservation quality = event.quality;
  quality.source_valid = event.source_valid;
  quality.reference_stamp_ns = event.ref_ns;
  quality.current_stamp_ns = event.cur_ns;
  quality.depth_stamp_ns = event.depth_ns;
  const reliability::VisualQualityDecision quality_decision =
      reliability::assessVisualQuality(quality, calibration_.reliability_config);
  if (!quality_decision.passed) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                quality_decision.rejection_reason, reason);
  }
  if (!event.translation_ref_imu.allFinite() ||
      !spd(event.measurement_covariance)) {
    source_records_.insert(key);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "invalid_visual_translation_or_covariance", reason);
  }
  WindowState reference;
  if (!controller_.stateAt(event.ref_ns, &reference, nullptr))
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                "visual_reference_state_not_in_window", reason);
  if (!ensureStateAt(event.cur_ns, reason)) {
    const AdapterEventDisposition disposition =
        reason && reason->find("causal") != std::string::npos
            ? AdapterEventDisposition::REJECTED_CAUSALITY
            : AdapterEventDisposition::REJECTED_WINDOW;
    setStatus(disposition, 0, reason ? *reason : "visual_state_creation_failed", reason);
    return false;
  }
  WindowState current;
  if (!controller_.stateAt(event.cur_ns, &current, reason))
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                "visual_current_state_not_in_window", reason);
  VisualRelativeMeasurement measurement;
  measurement.observation_id = allocateObservationId();
  measurement.reference_stamp_ns = event.ref_ns;
  measurement.current_stamp_ns = event.cur_ns;
  measurement.reference_imu_translation = event.translation_ref_imu;
  measurement.covariance = event.measurement_covariance;
  measurement.valid = true;
  if (!controller_.addVisualFactor(measurement, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "visual_factor_rejected", reason);
    return false;
  }
  const std::uint64_t id = measurement.observation_id;
  if (!accept(id, reason)) return false;
  source_records_.insert(key);
  setStatus(AdapterEventDisposition::ACCEPTED, id,
            "VISUAL_FACTOR_ACCEPTED", reason);
  return true;
}

bool FixedLagEventAdapter::optimizeCurrentWindow(std::string* reason) {
  if (!initialized_)
    return failLocal(reason, "adapter_not_initialized");
  return controller_.optimizeAndMarginalize(reason);
}

bool FixedLagEventAdapter::latestOptimizedState(WindowState* output,
                                                std::string* reason) const {
  if (!initialized_)
    return failLocal(reason, "adapter_not_initialized");
  return controller_.predictionFeedbackSeed(output, reason);
}

WindowSummary FixedLagEventAdapter::summary() const { return controller_.summary(); }

const AdapterEventStatus& FixedLagEventAdapter::lastEventStatus() const {
  return last_status_;
}

std::uint64_t FixedLagEventAdapter::nextObservationId() const {
  return next_observation_id_;
}

std::size_t FixedLagEventAdapter::sourceRecordCount() const {
  return source_records_.size();
}

std::size_t FixedLagEventAdapter::imuSampleCount() const {
  return imu_samples_.size();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
