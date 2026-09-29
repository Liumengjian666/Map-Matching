#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/lidar_residual_math.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool failLocal(std::string* reason, const std::string& message) {
  if (reason) *reason = message;
  return false;
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
  if (!controller_.initializeWithPriorAtomic(
          initial_state, initial_information, initial_gradient, reason)) {
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
  lifecycle_diagnostics_.peak_imu_buffer_size = std::max(
      lifecycle_diagnostics_.peak_imu_buffer_size, imu_samples_.size());
  last_imu_stamp_ns_ = sample.stamp_ns;
  last_status_ = {AdapterEventDisposition::ACCEPTED, 0, "IMU_BUFFERED"};
  return true;
}

bool FixedLagEventAdapter::sourceSeen(const SourceKey& key) const {
  return source_records_.count(key) != 0;
}

void FixedLagEventAdapter::recordSource(const SourceKey& key,
                                        std::uint64_t expiry_stamp_ns) {
  source_records_.insert(key);
  source_expiry_stamps_[key] = expiry_stamp_ns;
  lifecycle_diagnostics_.peak_active_source_records = std::max(
      lifecycle_diagnostics_.peak_active_source_records, source_records_.size());
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
    preintegrated->physical_covariance = preintegrated->covariance;
    Eigen::SelfAdjointEigenSolver<Matrix15d> physical_solver(
        preintegrated->physical_covariance);
    preintegrated->physical_min_eigenvalue =
        physical_solver.info() == Eigen::Success
            ? physical_solver.eigenvalues().minCoeff() :
              std::numeric_limits<double>::quiet_NaN();
    Matrix15d physical_information = Matrix15d::Zero();
    Eigen::CompleteOrthogonalDecomposition<Matrix15d> cod(
        preintegrated->physical_covariance);
    physical_information = cod.pseudoInverse();
    preintegrated->covariance.diagonal().array() += 1e-9;
    preintegrated->factor_covariance_regularized = true;
    preintegrated->factor_covariance_regularization = 1e-9;
    preintegrated->status +=
        ";NUMERICAL_FACTOR_COVARIANCE_REGULARIZATION_1E-9";
    covariance_factor.compute(preintegrated->covariance);
    if (covariance_factor.info() != Eigen::Success)
      return failLocal(reason, "imu_preintegration_covariance_not_solveable");
    Eigen::SelfAdjointEigenSolver<Matrix15d> factor_solver(
        preintegrated->covariance);
    preintegrated->factor_min_eigenvalue =
        factor_solver.eigenvalues().minCoeff();
    const Matrix15d factor_information =
        preintegrated->covariance.ldlt().solve(Matrix15d::Identity());
    preintegrated->information_change_norm =
        (factor_information - physical_information).norm();
  } else {
    preintegrated->physical_covariance = preintegrated->covariance;
    Eigen::SelfAdjointEigenSolver<Matrix15d> solver(preintegrated->covariance);
    preintegrated->physical_min_eigenvalue = solver.eigenvalues().minCoeff();
    preintegrated->factor_min_eigenvalue =
        preintegrated->physical_min_eigenvalue;
  }
  lifecycle_diagnostics_.last_imu_factor_covariance_regularized =
      preintegrated->factor_covariance_regularized;
  lifecycle_diagnostics_.last_imu_regularization =
      preintegrated->factor_covariance_regularization;
  lifecycle_diagnostics_.last_imu_physical_min_eigenvalue =
      preintegrated->physical_min_eigenvalue;
  lifecycle_diagnostics_.last_imu_factor_min_eigenvalue =
      preintegrated->factor_min_eigenvalue;
  lifecycle_diagnostics_.last_imu_information_change_norm =
      preintegrated->information_change_norm;
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
  const SourceKey key(0, latest.stamp_ns, stamp_ns);
  if (sourceSeen(key)) return failLocal(reason, "duplicate_imu_interval_source");
  const std::uint64_t id = allocateObservationId();
  if (!controller_.addStateWithImuFactorAtomic(
          predicted, id, latest.stamp_ns, preintegrated, reason)) return false;
  if (!accept(id, reason)) return false;
  recordSource(key, stamp_ns);
  return true;
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
  const Pose3d map_T_imu = lidarToImu(event.map_T_lidar);
  Matrix6d exact_jacobian;
  if (!normalizedLidarResidualJacobian(
          state, map_T_imu.orientation.normalized().toRotationMatrix(),
          calibration_.T_imu_lidar.position,
          event.local_risk.translation_length_scale_m, &exact_jacobian,
          reason)) return false;
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
  if (event.transaction_id <= lidar_transaction_watermark_ || sourceSeen(key))
    return fail(AdapterEventDisposition::DUPLICATE_SOURCE,
                event.transaction_id == lidar_transaction_watermark_
                    ? "duplicate_lidar_source" :
                      "lidar_transaction_below_monotonic_watermark", reason);
  if (event.stamp_ns == 0)
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "invalid_lidar_timestamp", reason);
  const auto consume_lidar = [&]() {
    lidar_transaction_watermark_ = event.transaction_id;
    recordSource(key, event.stamp_ns);
    lidar_risk_history_.push_back(
        {event.stamp_ns, event.local_risk, event.map_support_valid,
         event.ndt_converged, Matrix6d::Zero()});
  };
  if (!event.ndt_converged) {
    consume_lidar();
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "ndt_not_converged", reason);
  }
  if (!event.map_support_valid) {
    consume_lidar();
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "map_support_insufficient", reason);
  }
  if (!event.local_risk.valid || event.local_risk.reliable_dimension <= 0) {
    consume_lidar();
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "lidar_reliable_rank_zero_or_invalid", reason);
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
  consume_lidar();
  Matrix6d exact;
  const Pose3d map_T_imu = lidarToImu(event.map_T_lidar);
  if (normalizedLidarResidualJacobian(
          state, map_T_imu.orientation.normalized().toRotationMatrix(),
          calibration_.T_imu_lidar.position,
          event.local_risk.translation_length_scale_m, &exact, nullptr))
    lidar_risk_history_.back().exact_jacobian = exact;
  setStatus(AdapterEventDisposition::ACCEPTED, id, "LIDAR_FACTOR_ACCEPTED", reason);
  return true;
}

bool FixedLagEventAdapter::processVisualReferenceStamp(
    std::uint64_t ref_ns, std::string* reason) {
  if (reason) reason->clear();
  if (!initialized_ || ref_ns == 0)
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                initialized_ ? "invalid_visual_reference_timestamp" :
                               "adapter_not_initialized", reason);
  WindowState reference;
  if (controller_.stateAt(ref_ns, &reference, nullptr)) {
    setStatus(AdapterEventDisposition::ACCEPTED, 0,
              "VISUAL_REFERENCE_STATE_REUSED", reason);
    return true;
  }
  WindowState latest;
  if (!controller_.latestState(&latest, reason)) return false;
  WindowState oldest;
  if (!controller_.oldestState(&oldest, reason)) return false;
  if (ref_ns < oldest.stamp_ns)
    return fail(AdapterEventDisposition::REJECTED_WINDOW,
                "visual_reference_state_marginalized", reason);
  if (ref_ns < latest.stamp_ns)
    return fail(AdapterEventDisposition::REJECTED_CAUSALITY,
                "visual_reference_would_require_out_of_order_state", reason);
  if (!ensureStateAt(ref_ns, reason)) {
    const AdapterEventDisposition disposition =
        reason && reason->find("causal") != std::string::npos
            ? AdapterEventDisposition::REJECTED_CAUSALITY
            : AdapterEventDisposition::REJECTED_WINDOW;
    setStatus(disposition, 0,
              reason ? *reason : "visual_reference_state_creation_failed",
              reason);
    return false;
  }
  setStatus(AdapterEventDisposition::ACCEPTED, 0,
            "VISUAL_REFERENCE_STATE_CREATED", reason);
  return true;
}

bool FixedLagEventAdapter::configureVisualDirection(
    const FrozenVisualEvent& event, VisualRelativeMeasurement* measurement,
    std::string* reason) const {
  if (!measurement) return failLocal(reason, "null_visual_direction_output");
  const LidarRiskRecord* selected = nullptr;
  for (auto it = lidar_risk_history_.rbegin();
       it != lidar_risk_history_.rend(); ++it) {
    if (it->stamp_ns <= event.cur_ns) { selected = &*it; break; }
  }
  if (!selected) {
    measurement->mode = VisualFactorMode::NOT_TRIGGERED;
    measurement->trigger_status = "NO_CAUSAL_LIDAR_RISK";
    return true;
  }
  const double age_s = static_cast<double>(event.cur_ns - selected->stamp_ns) * 1e-9;
  if (age_s > calibration_.reliability_config.maximum_visual_pair_gap_s) {
    measurement->mode = VisualFactorMode::NOT_TRIGGERED;
    measurement->trigger_status = "STALE_LIDAR_RISK";
    return true;
  }
  if (!selected->ndt_converged || !selected->map_support_valid) {
    measurement->mode = VisualFactorMode::FULL_TRANSLATION;
    measurement->measurement_basis.setIdentity();
    measurement->selected_rank = 3;
    measurement->trigger_status = "RELATIVE_ONLY_NO_GLOBAL_RECOVERY";
    return true;
  }
  if (!selected->risk.valid || selected->risk.weak_dimension <= 0) {
    measurement->mode = VisualFactorMode::NOT_TRIGGERED;
    measurement->trigger_status = "NORMAL_LIDAR";
    return true;
  }
  const Eigen::MatrixXd mapped = selected->exact_jacobian *
      selected->risk.joint_weak_basis.leftCols(selected->risk.weak_dimension);
  const Eigen::MatrixXd translation = mapped.topRows(3);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      translation, Eigen::ComputeFullU | Eigen::ComputeThinV);
  if (!svd.singularValues().allFinite())
    return failLocal(reason, "visual_weak_translation_svd_failed");
  const double tolerance = 1e-9 * std::max(1.0, svd.singularValues()(0));
  int rank = 0;
  for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i)
    if (svd.singularValues()(i) > tolerance) ++rank;
  if (rank == 0) {
    measurement->mode = VisualFactorMode::NOT_TRIGGERED;
    measurement->trigger_status = "NO_TRANSLATIONAL_COMPLEMENT";
    return true;
  }
  measurement->mode = VisualFactorMode::LIDAR_WEAK_TRANSLATION;
  measurement->measurement_basis.setZero();
  measurement->measurement_basis.leftCols(rank) = svd.matrixU().leftCols(rank);
  measurement->selected_rank = rank;
  measurement->trigger_status = "LIDAR_WEAK_TRANSLATION_COMPLEMENT";
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
    recordSource(key, event.cur_ns);
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
      reliability::assessVisualSensorQuality(
          quality, calibration_.reliability_config);
  if (!quality_decision.passed) {
    recordSource(key, event.cur_ns);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                quality_decision.rejection_reason, reason);
  }
  if (!event.translation_ref_imu.allFinite() ||
      !spd(event.measurement_covariance)) {
    recordSource(key, event.cur_ns);
    return fail(AdapterEventDisposition::SKIPPED_INVALID_SOURCE,
                "invalid_visual_translation_or_covariance", reason);
  }
  if (!processVisualReferenceStamp(event.ref_ns, reason)) return false;
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
  if (!configureVisualDirection(event, &measurement, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "visual_direction_configuration_failed", reason);
    return false;
  }
  if (measurement.mode == VisualFactorMode::NOT_TRIGGERED) {
    recordSource(key, event.cur_ns);
    setStatus(AdapterEventDisposition::SKIPPED_INVALID_SOURCE, 0,
              measurement.trigger_status, reason);
    return false;
  }
  if (!controller_.addVisualFactor(measurement, reason)) {
    setStatus(AdapterEventDisposition::REJECTED_FACTOR, 0,
              reason ? *reason : "visual_factor_rejected", reason);
    return false;
  }
  const std::uint64_t id = measurement.observation_id;
  if (!accept(id, reason)) return false;
  recordSource(key, event.cur_ns);
  setStatus(AdapterEventDisposition::ACCEPTED, id,
            "VISUAL_FACTOR_ACCEPTED", reason);
  return true;
}

bool FixedLagEventAdapter::optimizeCurrentWindow(std::string* reason) {
  if (!initialized_)
    return failLocal(reason, "adapter_not_initialized");
  if (!controller_.optimizeAndMarginalize(reason)) return false;
  pruneExpiredHistory();
  return true;
}

bool FixedLagEventAdapter::latestOptimizedState(WindowState* output,
                                                std::string* reason) const {
  if (!initialized_)
    return failLocal(reason, "adapter_not_initialized");
  return controller_.predictionFeedbackSeed(output, reason);
}

void FixedLagEventAdapter::pruneExpiredHistory() {
  WindowState oldest;
  if (!controller_.oldestState(&oldest, nullptr)) return;
  auto upper = std::upper_bound(
      imu_samples_.begin(), imu_samples_.end(), oldest.stamp_ns,
      [](std::uint64_t stamp, const ImuSample& sample) {
        return stamp < sample.stamp_ns;
      });
  if (upper != imu_samples_.begin()) {
    auto keep = std::prev(upper);
    imu_samples_.erase(imu_samples_.begin(), keep);
  }
  for (auto it = source_expiry_stamps_.begin();
       it != source_expiry_stamps_.end();) {
    if (it->second < oldest.stamp_ns) {
      source_records_.erase(it->first);
      it = source_expiry_stamps_.erase(it);
      ++lifecycle_diagnostics_.expired_source_records_removed;
    } else {
      ++it;
    }
  }
  if (!lidar_risk_history_.empty()) {
    auto first_after = std::lower_bound(
        lidar_risk_history_.begin(), lidar_risk_history_.end(), oldest.stamp_ns,
        [](const LidarRiskRecord& record, std::uint64_t stamp) {
          return record.stamp_ns < stamp;
        });
    if (first_after != lidar_risk_history_.begin())
      lidar_risk_history_.erase(lidar_risk_history_.begin(),
                                std::prev(first_after));
  }
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

AdapterLifecycleDiagnostics FixedLagEventAdapter::lifecycleDiagnostics() const {
  return lifecycle_diagnostics_;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
