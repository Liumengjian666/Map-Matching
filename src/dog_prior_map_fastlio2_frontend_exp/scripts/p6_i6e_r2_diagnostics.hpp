// Included inside p6_i1 after the existing pose/basis helpers. Read-only
// diagnostics and one-step clones never replace the official frontend.
inline void writeR2PreupdateDiagnostics(
    FastLio2IkfomFrontend& frontend, const ScanAsset& asset, R2Policy policy,
    const Pose3d& nominal_lidar, const Pose3d& extrinsic,
    const reliability::LocalRisk& local, const reliability::LocalRisk& combined,
    const Eigen::Matrix<double, 6, 6>& base_noise,
    const Eigen::Matrix<double, 6, 6>& adaptive_noise, bool adaptive_valid,
    std::ostream& comparison, std::ostream& linearization,
    std::ostream& shadows) {
  const FilterSnapshot prior = frontend.getState();
  const Pose3d measurement = p4_i2::lidarMeasurementToImu(nominal_lidar, extrinsic);
  Eigen::Matrix<double, 6, 6> local_basis = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> combined_basis = local_basis;
  int local_rank = 0, combined_rank = 0;
  std::string local_reason, combined_reason;
  const bool local_valid = makeReliablePoseMeasurementBasis(
      local, measurement, extrinsic, &local_basis, &local_rank, &local_reason) &&
      local_rank > 0;
  const bool combined_valid = makeReliablePoseMeasurementBasis(
      combined, measurement, extrinsic, &combined_basis, &combined_rank,
      &combined_reason) && combined_rank > 0;
  const bool noise_valid = !r2PolicyUsesAdaptiveNoise(policy) || adaptive_valid;
  const auto& noise = r2PolicyUsesAdaptiveNoise(policy) ? adaptive_noise : base_noise;
  ProjectedPoseInnovation ld, cd;
  if (local_valid && noise_valid)
    frontend.evaluateProjectedPoseInnovation(measurement, noise, local_basis,
                                             local_rank, &ld, &local_reason);
  if (combined_valid && noise_valid)
    frontend.evaluateProjectedPoseInnovation(measurement, noise, combined_basis,
                                             combined_rank, &cd, &combined_reason);
  comparison << asset.transaction_id << ',' << asset.time_s << ',' << r2PolicyName(policy)
      << ',' << (ld.valid ? "OK" : (local_reason.empty() ? "NO_VALID_SELECTED_SYSTEM" : local_reason))
      << ',' << (cd.valid ? "OK" : (combined_reason.empty() ? "NO_VALID_SELECTED_SYSTEM" : combined_reason))
      << ',' << local.weak_dimension << ',' << combined.weak_dimension
      << ',' << local.reliable_dimension << ',' << combined.reliable_dimension
      << ',' << local.reliable_dimension - combined.reliable_dimension
      << ',' << local_rank << ',' << combined_rank << ',' << ld.nis
      << ',' << chiSquare99Threshold(local_rank) << ',' << cd.nis
      << ',' << chiSquare99Threshold(combined_rank) << ',' << ld.residual_norm
      << ',' << cd.residual_norm << ',' << ld.projected_residual_norm
      << ',' << cd.projected_residual_norm << ',' << ld.projected_noise_trace
      << ',' << cd.projected_noise_trace << ',' << ld.projected_noise_min_eigenvalue
      << ',' << ld.projected_noise_max_eigenvalue << ',' << cd.projected_noise_min_eigenvalue
      << ',' << cd.projected_noise_max_eigenvalue
      << ',' << (local_valid && noise_valid ? matrixField(Eigen::MatrixXd(
          local_basis.leftCols(local_rank).transpose() * noise * local_basis.leftCols(local_rank))) : "NA")
      << ',' << (combined_valid && noise_valid ? matrixField(Eigen::MatrixXd(
          combined_basis.leftCols(combined_rank).transpose() * noise * combined_basis.leftCols(combined_rank))) : "NA")
      << '\n';

  if (combined_valid && combined.weak_dimension > 0) {
    const Eigen::MatrixXd B = combined_basis.leftCols(combined_rank);
    const Eigen::MatrixXd Uw = combined.joint_weak_basis.leftCols(combined.weak_dimension);
    const auto A_nominal = normalizedLidarToPoseJacobian(
        measurement, extrinsic, combined.translation_length_scale_m);
    const auto A_predicted = normalizedLidarToPoseJacobian(
        prior.map_T_imu, extrinsic, combined.translation_length_scale_m);
    const auto gap = poseResidual(prior.map_T_imu, measurement);
    constexpr double epsilon = 1e-7;
    for (int axis = 0; axis < combined.weak_dimension; ++axis) {
      const Eigen::Matrix<double, 6, 1> w = Uw.col(axis);
      Pose3d plus = nominal_lidar, minus = nominal_lidar;
      const Eigen::Vector3d phi = epsilon * w.head<3>();
      Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
      if (phi.norm() > 0.0)
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(phi.norm(), phi.normalized()));
      plus.orientation = (dq * nominal_lidar.orientation).normalized();
      minus.orientation = (dq.conjugate() * nominal_lidar.orientation).normalized();
      plus.position += epsilon * combined.translation_length_scale_m * w.tail<3>();
      minus.position -= epsilon * combined.translation_length_scale_m * w.tail<3>();
      const Eigen::Matrix<double, 6, 1> fd =
          (poseResidual(prior.map_T_imu, p4_i2::lidarMeasurementToImu(plus, extrinsic)) -
           poseResidual(prior.map_T_imu, p4_i2::lidarMeasurementToImu(minus, extrinsic))) /
          (2.0 * epsilon);
      const Eigen::VectorXd selected = B.transpose() * fd;
      linearization << asset.transaction_id << ',' << asset.time_s << ',' << r2PolicyName(policy)
          << ',' << axis << ',' << matrixField(w) << ',' << (B.transpose() * A_nominal * Uw).norm()
          << ',' << (B.transpose() * A_predicted * Uw).norm() << ',' << gap.tail<3>().norm()
          << ',' << gap.head<3>().norm() << ',' << selected.norm() << ',' << matrixField(selected)
          << ',' << (selected.allFinite() ? "RECORDED_NONLINEAR_RESPONSE" : "NONFINITE_RESPONSE") << '\n';
    }
  } else {
    linearization << asset.transaction_id << ',' << asset.time_s << ',' << r2PolicyName(policy)
        << ",-1,NA,nan,nan,nan,nan,nan,NA,NO_VALID_WEAK_RELIABLE_BASIS\n";
  }

  static const std::array<std::uint64_t, 14> selected_transactions = {
      24, 29, 32, 34, 40, 47, 60, 79, 88, 95, 110, 120, 124, 127};
  if (policy != R2Policy::LEGACY_BASE_NO_GATE ||
      std::find(selected_transactions.begin(), selected_transactions.end(),
                asset.transaction_id) == selected_transactions.end()) return;
  const std::array<R2Policy, 4> policies = {R2Policy::LEGACY_BASE_NO_GATE,
      R2Policy::ADAPTIVE_NO_GATE, R2Policy::BASE_SELECTED_NIS,
      R2Policy::ADAPTIVE_SELECTED_NIS};
  std::array<std::unique_ptr<FastLio2IkfomFrontend>, 4> clones;
  if (combined_valid)
    for (auto& clone : clones) {
      clone = frontend.cloneCandidate();
      if (!sameSnapshotExactly(prior, clone->getState()))
        throw std::runtime_error("R2_SHADOW_CLONE_PRIOR_MISMATCH");
    }
  const std::string prior_hash = stateSnapshotHash(prior);
  for (std::size_t index = 0; index < policies.size(); ++index) {
    const R2Policy branch_policy = policies[index];
    ProjectedPoseInnovation d;
    PoseCorrectionDelta delta;
    bool attempted = false, committed = false;
    std::string reason = "SHADOW_SKIPPED_NO_VALID_BASIS";
    FilterSnapshot after = prior;
    if (combined_valid) {
      if (r2PolicyUsesAdaptiveNoise(branch_policy) && !adaptive_valid) {
        reason = "INVALID_ADAPTIVE_MEASUREMENT_NOISE";
      } else {
        const auto& branch_noise = r2PolicyUsesAdaptiveNoise(branch_policy) ? adaptive_noise : base_noise;
        attempted = true;
        if (clones[index]->evaluateProjectedPoseInnovation(measurement, branch_noise,
                combined_basis, combined_rank, &d, &reason)) {
          committed = clones[index]->applyProjectedPoseMeasurementChecked(
              measurement, branch_noise, combined_basis, combined_rank,
              r2PolicyUsesNisGate(branch_policy), chiSquare99Threshold(combined_rank),
              &d, &delta, &reason);
        }
        after = clones[index]->getState();
      }
    }
    const FilterSnapshot official_after = frontend.getState();
    const bool isolated = sameSnapshotExactly(prior, official_after);
    shadows << asset.transaction_id << ',' << asset.time_s << ',' << r2PolicyName(branch_policy)
        << ',' << prior_hash << ',' << vectorField(prior.map_T_imu.position)
        << ',' << vectorField(prior.velocity) << ',' << combined_rank << ',' << combined.weak_dimension
        << ',' << d.residual_norm << ',' << d.projected_residual_norm << ',' << d.projected_noise_trace
        << ',' << d.projected_noise_min_eigenvalue << ',' << d.projected_noise_max_eigenvalue
        << ',' << d.innovation_covariance_trace << ',' << d.nis << ',' << chiSquare99Threshold(combined_rank)
        << ',' << (d.valid && d.nis <= chiSquare99Threshold(combined_rank)) << ',' << attempted
        << ',' << committed << ',' << (committed ? "NONE" : reason)
        << ',' << delta.position.norm() << ',' << delta.rotation.norm() << ',' << delta.velocity.norm()
        << ',' << delta.gyro_bias.norm() << ',' << delta.accel_bias.norm()
        << ',' << maximumPositionSigma(after) << ',' << stateSnapshotHash(official_after)
        << ',' << isolated << '\n';
    if (!isolated) throw std::runtime_error("R2_SHADOW_MODIFIED_OFFICIAL_STATE");
  }
}

inline void writeR3ShadowComparison(
    FastLio2IkfomFrontend& frontend, const ScanAsset& asset, R2Policy policy,
    const Pose3d& nominal_lidar, const Pose3d& extrinsic,
    const reliability::LocalRisk& risk,
    const Eigen::Matrix<double, 6, 6>& base_noise,
    const Eigen::Matrix<double, 6, 6>& adaptive_noise, bool adaptive_valid,
    std::ostream& output) {
  static const std::array<std::uint64_t, 10> selected_transactions = {
      24, 32, 47, 60, 79, 88, 95, 120, 124, 127};
  if (std::find(selected_transactions.begin(), selected_transactions.end(),
                asset.transaction_id) == selected_transactions.end())
    return;
  const FilterSnapshot prior = frontend.getState();
  const Pose3d measurement = p4_i2::lidarMeasurementToImu(nominal_lidar,
                                                            extrinsic);
  Eigen::Matrix<double, 6, 6> legacy_basis =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> exact_basis = legacy_basis;
  int legacy_rank = 0, exact_rank = 0;
  std::string legacy_reason, exact_reason;
  const bool legacy_built = makeReliablePoseMeasurementBasis(
      risk, measurement, extrinsic, &legacy_basis, &legacy_rank,
      &legacy_reason);
  if (legacy_built && legacy_rank <= 0 && legacy_reason.empty())
    legacy_reason = "ZERO_RELIABLE_RANK";
  const bool legacy_valid = legacy_built && legacy_rank > 0;
  const bool exact_built = makeExactReliablePoseMeasurementBasis(
      risk, prior.map_T_imu, nominal_lidar, extrinsic, &exact_basis,
      &exact_rank, &exact_reason);
  if (exact_built && exact_rank <= 0 && exact_reason.empty())
    exact_reason = "ZERO_RELIABLE_RANK";
  const bool exact_valid = exact_built && exact_rank > 0;
  const auto legacy_A = normalizedLidarToPoseJacobian(
      measurement, extrinsic, risk.translation_length_scale_m);
  Eigen::Matrix<double, 6, 6> exact_A =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  std::string exact_jacobian_reason;
  const bool exact_jacobian_valid =
      normalizedLidarToExactResidualJacobian(
          prior.map_T_imu, nominal_lidar, extrinsic,
          risk.translation_length_scale_m, &exact_A,
          &exact_jacobian_reason);
  const Eigen::MatrixXd weak = risk.joint_weak_basis.leftCols(
      std::max(0, risk.weak_dimension));
  const double legacy_leakage = legacy_valid && risk.weak_dimension > 0
      ? (legacy_basis.leftCols(legacy_rank).transpose() * legacy_A * weak).norm()
      : std::numeric_limits<double>::quiet_NaN();
  const double exact_leakage = exact_valid && exact_jacobian_valid &&
      risk.weak_dimension > 0
      ? (exact_basis.leftCols(exact_rank).transpose() * exact_A * weak).norm()
      : std::numeric_limits<double>::quiet_NaN();
  constexpr double epsilon = 1e-7;
  double legacy_fd_response = 0.0;
  double exact_fd_response = 0.0;
  if (risk.weak_dimension > 0) {
    for (int index = 0; index < risk.weak_dimension; ++index) {
      const Eigen::Matrix<double, 6, 1> direction = weak.col(index);
      const Eigen::Vector3d phi = epsilon * direction.head<3>();
      Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();
      if (phi.norm() > 0.0)
        delta = Eigen::Quaterniond(Eigen::AngleAxisd(phi.norm(),
                                                      phi.normalized()));
      Pose3d plus = nominal_lidar, minus = nominal_lidar;
      plus.orientation = (delta * plus.orientation).normalized();
      minus.orientation = (delta.conjugate() * minus.orientation).normalized();
      plus.position += epsilon * risk.translation_length_scale_m *
          direction.tail<3>();
      minus.position -= epsilon * risk.translation_length_scale_m *
          direction.tail<3>();
      const Eigen::Matrix<double, 6, 1> fd =
          (poseResidual(prior.map_T_imu,
                        p4_i2::lidarMeasurementToImu(plus, extrinsic)) -
           poseResidual(prior.map_T_imu,
                        p4_i2::lidarMeasurementToImu(minus, extrinsic))) /
          (2.0 * epsilon);
      if (legacy_valid)
        legacy_fd_response = std::max(legacy_fd_response,
            (legacy_basis.leftCols(legacy_rank).transpose() * fd).norm());
      if (exact_valid)
        exact_fd_response = std::max(exact_fd_response,
            (exact_basis.leftCols(exact_rank).transpose() * fd).norm());
    }
  }
  const auto& noise = r2PolicyUsesAdaptiveNoise(policy) ? adaptive_noise :
                                                            base_noise;
  const bool noise_valid = !r2PolicyUsesAdaptiveNoise(policy) || adaptive_valid;
  ProjectedPoseInnovation legacy_diagnostic, exact_diagnostic;
  PoseCorrectionDelta legacy_delta, exact_delta;
  bool legacy_committed = false, exact_committed = false;
  if (legacy_valid && noise_valid) {
    auto clone = frontend.cloneCandidate();
    if (!sameSnapshotExactly(prior, clone->getState()))
      throw std::runtime_error("R3_LEGACY_CLONE_PRIOR_MISMATCH");
    legacy_committed = clone->applyProjectedPoseMeasurementLinearizedChecked(
        measurement, noise, legacy_basis, legacy_rank,
        ProjectedPoseLinearizationMode::LEGACY_IDENTITY_ROTATION,
        r2PolicyUsesNisGate(policy), chiSquare99Threshold(legacy_rank),
        &legacy_diagnostic, &legacy_delta, &legacy_reason);
  }
  if (exact_valid && noise_valid) {
    auto clone = frontend.cloneCandidate();
    if (!sameSnapshotExactly(prior, clone->getState()))
      throw std::runtime_error("R3_EXACT_CLONE_PRIOR_MISMATCH");
    exact_committed = clone->applyProjectedPoseMeasurementLinearizedChecked(
        measurement, noise, exact_basis, exact_rank,
        ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL,
        r2PolicyUsesNisGate(policy), chiSquare99Threshold(exact_rank),
        &exact_diagnostic, &exact_delta, &exact_reason);
  }
  const bool official_unchanged = sameSnapshotExactly(prior,
                                                       frontend.getState());
  const double rotation_gap = poseResidual(prior.map_T_imu,
                                           measurement).tail<3>().norm();
  std::string status = "OK";
  if (!legacy_valid) status = "SKIPPED_LEGACY:" + legacy_reason;
  if (!exact_valid) status = "SKIPPED_EXACT:" + exact_reason;
  if (!noise_valid) status = "SKIPPED_INVALID_ADAPTIVE_NOISE";
  output << asset.transaction_id << ',' << asset.time_s << ','
      << r2PolicyName(policy) << ',' << stateSnapshotHash(prior) << ','
      << rotation_gap << ',' << legacy_rank << ',' << exact_rank << ','
      << legacy_leakage << ',' << legacy_fd_response << ','
      << exact_leakage << ',' << exact_fd_response << ','
      << legacy_diagnostic.nis << ',' << exact_diagnostic.nis << ','
      << legacy_committed << ',' << exact_committed << ','
      << legacy_delta.position.norm() << ',' << exact_delta.position.norm()
      << ',' << legacy_delta.rotation.norm() << ','
      << exact_delta.rotation.norm() << ',' << legacy_delta.velocity.norm()
      << ',' << exact_delta.velocity.norm() << ',' << official_unchanged
      << ',' << status << '\n';
  if (!official_unchanged)
    throw std::runtime_error("R3_SHADOW_MODIFIED_OFFICIAL_STATE");
}
