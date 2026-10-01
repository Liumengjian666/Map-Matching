// Included after the mature P6 NDT/input helpers, inside namespace p6_i1.
// This independent producer owns only FixedLagEventAdapter after handoff.

#include "p6_a3g_r3_capture.hpp"

struct FixedLagProducerResult {
  std::size_t events = 0;
  std::size_t lidar_committed = 0;
  std::size_t visual_committed = 0;
  std::size_t covariance_available = 0;
  std::size_t probes = 0;
  std::uint64_t ndt_calls = 0;
  double ndt_ms = 0.0;
  double total_ms = 0.0;
  std::size_t window_deskew_count = 0;
  std::size_t raw_scans_before_handoff = 0;
};

using RawTimedScanProvider = std::function<bool(const ScanAsset&,
    fixed_lag::RawTimedScan*, std::string*)>;
struct WindowOwnedProducerInput {
  RawTimedScanProvider provider;
  std::vector<std::uint64_t> scan_start_ns;
  std::vector<fixed_lag::VisualMeasurementProvenance> visual_provenance;
};

void writeMarginalizationStatsHeader(std::ostream& output,
                                     const std::string& prefix) {
  output << ',' << prefix << "_available," << prefix << "_finite,"
         << prefix << "_dimension," << prefix << "_symmetry_fro,"
         << prefix << "_symmetry_max_abs,"
         << prefix << "_lambda_min," << prefix << "_lambda_max,"
         << prefix << "_min_abs_eigenvalue," << prefix << "_spectral_scale,"
         << prefix << "_frobenius," << prefix << "_relative_negative,"
         << prefix << "_negative_count," << prefix << "_numerical_rank";
}

void writeMarginalizationStats(std::ostream& output,
    const fixed_lag::MarginalizationMatrixStats& stats) {
  output << ',' << stats.available << ',' << stats.finite << ','
         << stats.dimension << ',' << stats.symmetry_frobenius_norm << ','
         << stats.symmetry_max_abs << ','
         << stats.lambda_min << ',' << stats.lambda_max << ','
         << stats.min_abs_eigenvalue << ',' << stats.spectral_scale << ','
         << stats.frobenius_norm << ',' << stats.relative_negative_ratio << ','
         << stats.negative_eigenvalue_count << ',' << stats.numerical_rank;
}

void writeMarginalizationTraceHeader(std::ostream& output) {
  output << std::setprecision(17)
      << "transaction_id,event_stamp_ns,enforcement_index,attempt_index,latest_state_stamp_ns,oldest_state_stamp_ns,nodes_before_attempt,span_before_attempt_s,removed_state_stamp_ns,oldest_state_removed,nodes_after_attempt,span_after_attempt_s,trigger_duration_limit,trigger_node_limit,incident_imu_factor_count,incident_lidar_factor_count,incident_visual_factor_count,incoming_prior_valid,incoming_prior_hash_fnv1a64";
  for (const char* name : {"incoming_prior", "charted_prior", "touching_factors",
       "imu_contribution", "lidar_contribution", "visual_contribution",
       "consumed_system", "hmm", "raw_schur", "symmetrized_schur",
       "new_prior"})
    writeMarginalizationStatsHeader(output, name);
  output << ",ldlt_initial_min_abs_D,ldlt_initial_max_abs_D,ldlt_initial_pivot_ratio,ldlt_initial_positive_D_count,ldlt_initial_negative_D_count,ldlt_initial_near_zero_D_count,ldlt_solve_min_abs_D,ldlt_solve_max_abs_D,ldlt_solve_pivot_ratio,ldlt_solve_positive_D_count,ldlt_solve_negative_D_count,ldlt_solve_near_zero_D_count,solve_jitter,Hmm_condition_proxy,solve_finite,solve_H_backward_error,solve_b_backward_error,raw_schur_asymmetry_fro,new_gradient_evaluated,new_gradient_finite,new_gradient_norm,new_gradient_max_abs,first_bad_stage,marginalization_result,marginalization_backend,qr_stack_rows,qr_columns,qr_marginalized_rank,qr_rank_threshold,qr_R_diag_min,qr_R_diag_max,qr_rows_before_compression,qr_rows_after_compression,qr_active_columns,qr_compression_rank,qr_compression_threshold,qr_discarded_row_jacobian_norm,qr_discarded_constant_squared,qr_status,qr_ms,legacy_shadow_status,legacy_shadow_lambda_min\n";
}

void writeMarginalizationTraceRow(std::ostream& output,
    std::uint64_t transaction_id, std::uint64_t event_stamp_ns,
    const fixed_lag::MarginalizationTraceRecord& row) {
  output << transaction_id << ',' << event_stamp_ns << ','
      << row.marginalization_enforcement_index << ','
      << row.attempt_index_within_enforcement << ','
      << row.latest_state_stamp_ns << ',' << row.oldest_state_stamp_ns << ','
      << row.nodes_before_attempt << ',' << row.span_before_attempt_s << ','
      << row.removed_state_stamp_ns << ',' << row.oldest_state_removed << ','
      << row.nodes_after_attempt << ',' << row.span_after_attempt_s << ','
      << row.trigger_duration_limit << ',' << row.trigger_node_limit << ','
      << row.incident_imu_factor_count << ','
      << row.incident_lidar_factor_count << ','
      << row.incident_visual_factor_count << ',' << row.incoming_prior_valid
      << ',' << row.incoming_prior_hash_fnv1a64;
  for (const auto* stats : {&row.incoming_prior, &row.charted_prior,
       &row.touching_factors, &row.imu_contribution, &row.lidar_contribution,
       &row.visual_contribution, &row.consumed_system, &row.hmm,
       &row.raw_schur, &row.symmetrized_schur, &row.new_prior})
    writeMarginalizationStats(output, *stats);
  output << ',' << row.ldlt_initial_min_abs_d << ','
      << row.ldlt_initial_max_abs_d << ',' << row.ldlt_initial_pivot_ratio
      << ',' << row.ldlt_initial_positive_d_count << ','
      << row.ldlt_initial_negative_d_count << ','
      << row.ldlt_initial_near_zero_d_count << ','
      << row.ldlt_solve_min_abs_d << ',' << row.ldlt_solve_max_abs_d << ','
      << row.ldlt_solve_pivot_ratio << ','
      << row.ldlt_solve_positive_d_count << ','
      << row.ldlt_solve_negative_d_count << ','
      << row.ldlt_solve_near_zero_d_count << ',' << row.solve_jitter << ','
      << row.hmm_condition_proxy << ',' << row.solve_finite << ','
      << row.solve_h_backward_error << ',' << row.solve_b_backward_error << ','
      << row.raw_schur_asymmetry_frobenius_norm << ','
      << row.new_gradient_evaluated << ','
      << row.new_gradient_finite << ',' << row.new_gradient_norm << ','
      << row.new_gradient_max_abs << ',' << row.first_bad_stage << ','
      << row.marginalization_result << ',' << row.backend << ','
      << row.qr.stack_rows << ',' << row.qr.columns << ','
      << row.qr.marginalized_rank << ',' << row.qr.marginalized_threshold << ','
      << row.qr.r_diagonal_min << ',' << row.qr.r_diagonal_max << ','
      << row.qr.rows_before_compression << ',' << row.qr.rows_after_compression << ','
      << row.qr.active_columns << ',' << row.qr.compression_rank << ','
      << row.qr.compression_threshold << ',' << row.qr.discarded_row_jacobian_norm << ','
      << row.qr.discarded_constant_squared << ',' << row.qr.status << ','
      << row.qr_ms << ',' << row.legacy_shadow_status << ','
      << row.legacy_shadow_lambda_min << '\n';
}

void writeCapsuleArray(std::ostream& file, const std::string& name,
                       const Eigen::MatrixXd& matrix) {
  const std::uint32_t name_length = static_cast<std::uint32_t>(name.size());
  const std::uint64_t rows = static_cast<std::uint64_t>(matrix.rows());
  const std::uint64_t columns = static_cast<std::uint64_t>(matrix.cols());
  file.write(reinterpret_cast<const char*>(&name_length), sizeof(name_length));
  file.write(name.data(), static_cast<std::streamsize>(name.size()));
  file.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
  file.write(reinterpret_cast<const char*>(&columns), sizeof(columns));
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      const double value = matrix(row, column);
      file.write(reinterpret_cast<const char*>(&value), sizeof(value));
    }
}

bool writeMarginalizationFailureCapsuleBinary(
  const std::string& path,
  const fixed_lag::MarginalizationFailureCapsule& capsule) {
  if (!capsule.valid || path.empty()) return false;
  std::ifstream existing(path, std::ios::binary);
  if (existing.good()) return false;
  std::ofstream file(path, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!file) return false;
  const char magic[16] = {'P','6','A','3','C','R','1','C','A','P','S','U','L','E','\0','\0'};
  const std::uint32_t version = 1;
  const bool square_root = capsule.trace.backend == "SQUARE_ROOT_QR";
  const std::uint32_t array_count = square_root ? 15 : 11;
  file.write(magic, sizeof(magic));
  file.write(reinterpret_cast<const char*>(&version), sizeof(version));
  file.write(reinterpret_cast<const char*>(&array_count), sizeof(array_count));
  writeCapsuleArray(file, "incoming_prior_information", capsule.incoming_prior_information);
  writeCapsuleArray(file, "incoming_prior_gradient", Eigen::MatrixXd(capsule.incoming_prior_gradient));
  writeCapsuleArray(file, "charted_prior_information", capsule.charted_prior_information);
  writeCapsuleArray(file, "charted_prior_gradient", Eigen::MatrixXd(capsule.charted_prior_gradient));
  writeCapsuleArray(file, "consumed_hessian", capsule.consumed_hessian);
  writeCapsuleArray(file, "consumed_gradient", Eigen::MatrixXd(capsule.consumed_gradient));
  writeCapsuleArray(file, "imu_hessian", capsule.imu_hessian);
  writeCapsuleArray(file, "lidar_hessian", capsule.lidar_hessian);
  writeCapsuleArray(file, "visual_hessian", capsule.visual_hessian);
  writeCapsuleArray(file, "correction_h", capsule.correction_h);
  writeCapsuleArray(file, "correction_b", Eigen::MatrixXd(capsule.correction_b));
  if (square_root) {
    writeCapsuleArray(file,"square_root_stack_A",capsule.square_root_stack.a);
    writeCapsuleArray(file,"square_root_stack_b",Eigen::MatrixXd(capsule.square_root_stack.b));
    writeCapsuleArray(file,"incoming_square_root_A",capsule.incoming_square_root_prior.a);
    writeCapsuleArray(file,"incoming_square_root_b",Eigen::MatrixXd(capsule.incoming_square_root_prior.b));
  }
  file.flush();
  return file.good();
}

std::string stampList(const std::vector<std::uint64_t>& stamps) {
  std::ostringstream output;
  for (std::size_t index = 0; index < stamps.size(); ++index) {
    if (index) output << ';';
    output << stamps[index];
  }
  return output.str();
}

FixedLagInitializationSeed initializeFixedLagProducer(
    const p4_i2::ImuVector& imu, const RuntimeParameters& parameters,
    const Pose3d& initial_lidar, const Pose3d& extrinsic,
    std::uint64_t initialization_stamp_ns) {
  auto end = initialization_stamp_ns == 0
      ? imu.begin() + std::min<std::size_t>(imu.size(), parameters.static_init_samples)
      : std::upper_bound(imu.begin(), imu.end(), initialization_stamp_ns,
          [](std::uint64_t t, const ImuSample& s) { return t < s.stamp_ns; });
  if (std::distance(imu.begin(), end) < parameters.static_init_samples)
    throw std::runtime_error("insufficient_causal_initialization_imu");
  p4_i2::ImuVector samples(end - parameters.static_init_samples, end);
  FastLio2IkfomFrontend initializer(parameters);
  std::string reason;
  if (!initializer.initializeStatic(samples, initial_lidar, extrinsic, &reason))
    throw std::runtime_error("fixed_lag_static_initialization:" + reason);
  if (initialization_stamp_ns > samples.back().stamp_ns) {
    std::vector<std::uint64_t> periods;
    for (std::size_t i=1; i<samples.size(); ++i)
      periods.push_back(samples[i].stamp_ns-samples[i-1].stamp_ns);
    std::sort(periods.begin(),periods.end());
    if (periods.empty() || periods[periods.size()/2]==0 ||
        initialization_stamp_ns-samples.back().stamp_ns > 2*periods[periods.size()/2])
      throw std::runtime_error("causal_initialization_epoch_gap_too_large");
    if (!initializer.predictHeldInputTo(initialization_stamp_ns,
            samples[samples.size()-2],samples.back(),&reason))
      throw std::runtime_error("fixed_lag_initialization_epoch:"+reason);
  }
  FixedLagInitializationSeed seed;
  if (!initializer.makeFixedLagInitializationSeed(&seed,&reason))
    throw std::runtime_error("fixed_lag_initialization_bridge:"+reason);
  return seed; // initializer is destroyed here, before the producer handoff.
}

fixed_lag::FrozenVisualEvent frozenVisual(const VisualMeasurement& v,
                                         double sigma) {
  fixed_lag::FrozenVisualEvent event;
  event.ref_ns=v.ref_ns; event.cur_ns=v.cur_ns; event.depth_ns=v.depth_stamp_ns;
  event.translation_ref_imu=v.translation;
  event.measurement_covariance=sigma*sigma*Eigen::Matrix3d::Identity();
  event.source_valid=v.source_valid;
  event.provenance=fixed_lag::VisualMeasurementProvenance::LEGACY_STATE_DERIVED_DEPTH;
  auto& q=event.quality;
  q.quality_metadata_available=v.quality_metadata_available;
  q.detected_count=v.detected_count; q.tracked_count=v.tracked_count;
  q.depth_associated_count=v.depth_associated_count; q.pnp_inlier_count=v.inliers;
  q.inlier_ratio=v.ratio; q.depth_fraction=v.depth_fraction;
  q.grid_occupancy=v.grid_occupancy; q.hull_fraction=v.hull_fraction;
  q.median_parallax_px=v.median_parallax_px; q.reprojection_rmse_px=v.reprojection;
  return event;
}

struct LidarCandidateEvaluation {
  fixed_lag::FrozenLidarEvent event;
  fixed_lag::LidarWindowMeasurement preview;
  fixed_lag::SelectedLidarNis nis;
  reliability::LocalObservability local;
  reliability::LocalRisk routed;
  reliability::NonlocalTerminalStability stability;
  GeometricSearchDiagnostic search;
  Eigen::VectorXd selected_residual;
  Eigen::MatrixXd selected_covariance;
  bool preview_valid=false,attempted=false,probed=false,admissible=false;
  bool same_factor_noise_allowed=false,production_probe_trigger=false,legacy_probe_trigger=false;
  bool effective=false;
  int rank=0;
  std::uint64_t probe_calls=0;
  double probe_ms=0;
  std::string nonlocal_status="NOT_PROBED";
};

reliability::TerminalCapture trackingRegistrationCapture(
    const Candidate& candidate,bool enforce_effective_registration,int maximum_iterations) {
  auto capture=terminalCapture(candidate);
  if (enforce_effective_registration)
    capture.converged=p6_tracking::effectiveRegistration(candidate.converged,
        candidate.iterations,candidate.objective,candidate.pose,maximum_iterations);
  return capture;
}

// Every recovery hypothesis uses the same measurement admission chain as M0.
// Preview is read-only; only the selected result is submitted to the adapter.
LidarCandidateEvaluation evaluateLidarCandidate(
    GeometricNdt& ndt,const Cloud::Ptr& source,const Candidate& terminal,
    const ScanAsset& asset,const fixed_lag::WindowState& predicted,
    const fixed_lag::WindowMarginalCovariance& prior,const Pose3d& extrinsic,
    const RuntimeParameters& parameters,R2Policy policy,
    const reliability::DualReliabilityConfig& config,
    fixed_lag::FixedLagEventAdapter& adapter,bool enforce_effective_registration,
    const Eigen::Matrix4d* recovery_seed_map_T_imu=nullptr) {
  using namespace fixed_lag;
  LidarCandidateEvaluation result;
  result.stability.status="NOT_PROBED";
  result.effective=p6_tracking::effectiveRegistration(
      terminal.converged,terminal.iterations,terminal.objective,terminal.pose,ndt.getMaximumIterations());
  const bool converged=terminal.converged && (!enforce_effective_registration || result.effective);
  const auto observations=converged ? ndt.geometricObservations(*source,terminal.pose.cast<float>(),0.8) :
      std::vector<reliability::GeometricObservation>{};
  result.local=reliability::analyzeGeometricObservability(observations,converged,0.8);
  if (enforce_effective_registration && !result.local.valid)
    result.search=ndt.diagnoseSearch(*source,terminal.pose.cast<float>(),0.8);
  const auto risk=reliability::assessLocalRisk(result.local,config);
  Pose3d predicted_pose; predicted_pose.position=predicted.position;
  predicted_pose.orientation=Eigen::Quaterniond(predicted.rotation);
  const Eigen::Matrix4d prediction=poseMatrix(predicted_pose),T_il=poseMatrix(extrinsic);
  // Registration perturbations must belong to this same candidate hypothesis.
  // The Window state/P15 still own innovation, covariance and selected NIS.
  const Eigen::Matrix4d probe_center=recovery_seed_map_T_imu ? *recovery_seed_map_T_imu : prediction;
  if (prior.valid && converged) {
    const auto measured=p4_i2::lidarMeasurementToImu(poseFromMatrix(terminal.pose),extrinsic);
    const auto innovation=reliability::mapProductInnovation(
        p4_i2::asIsometry(predicted_pose),p4_i2::asIsometry(measured));
    const auto trigger=reliability::shouldRunNonlocalProbes(
        asset.transaction_id,innovation,prior.map_pose_covariance6,config);
    result.production_probe_trigger=trigger.run_probes;
    if (prior.legacy_shadow_valid)
      result.legacy_probe_trigger=reliability::shouldRunNonlocalProbes(
          asset.transaction_id,innovation,prior.legacy_map_pose_covariance6,config).run_probes;
    const auto spectrum=p6_i4::analyzePoseCovariance(prior.map_pose_covariance6);
    if (trigger.run_probes && spectrum.valid && spectrum.effective_rank>0 && spectrum.eigenvalues(5)>0) {
      const Vector6d delta=config.probe_prior_sigma*std::sqrt(spectrum.eigenvalues(5))*spectrum.eigenvectors.col(5);
      const auto plus=runNdtCandidate(ndt,source,p6_i4::boxplusMapPose(probe_center,delta)*T_il,1,"WINDOW_M_PLUS");
      const auto minus=runNdtCandidate(ndt,source,p6_i4::boxplusMapPose(probe_center,-delta)*T_il,2,"WINDOW_M_MINUS");
      result.probe_calls=2; result.probe_ms=plus.runtime_ms+minus.runtime_ms; result.probed=true;
      result.stability=reliability::analyzeNonlocalTerminalStability(
          trackingRegistrationCapture(terminal,enforce_effective_registration,ndt.getMaximumIterations()),
          trackingRegistrationCapture(plus,enforce_effective_registration,ndt.getMaximumIterations()),
          trackingRegistrationCapture(minus,enforce_effective_registration,ndt.getMaximumIterations()),
          p4_i2::asIsometry(extrinsic),2);
      result.nonlocal_status=result.stability.status;
    } else if (trigger.run_probes) {
      result.stability.status="PROBE_PRIOR_COVARIANCE_INVALID";
      result.nonlocal_status=result.stability.status;
    } else result.nonlocal_status=trigger.reason;
  } else if (!prior.valid) result.nonlocal_status="UNONLOCAL_DISABLED_WINDOW_COVARIANCE_UNAVAILABLE";
  const auto decision=reliability::decideDualReliability(risk,result.stability,converged,
      predicted.rotation,parameters.pose_position_sigma_m,parameters.pose_rotation_sigma_rad,true,true,config);
  const auto& stability=result.stability;
  const bool terminal_risk=stability.response_valid &&
      (std::max(stability.delta_position_imu_positive.norm(),stability.delta_position_imu_negative.norm())>config.terminal_translation_limit_m ||
       std::max(stability.delta_rotation_positive.norm(),stability.delta_rotation_negative.norm())>config.terminal_rotation_limit_rad);
  result.routed=riskWithNonlocalResponse(risk,stability,decision.nonlocal_risk||terminal_risk,
      -predicted.rotation*extrinsic.position,config);
  auto& lidar=result.event;
  lidar.transaction_id=asset.transaction_id; lidar.stamp_ns=asset.stamp_ns;
  lidar.map_T_lidar=poseFromMatrix(terminal.pose); lidar.local_risk=result.routed;
  lidar.ndt_converged=converged;
  lidar.map_support_valid=result.routed.valid&&result.routed.map_support_sufficient;
  lidar.residual_covariance=makeBasePoseNoise(parameters);
  if (r2PolicyUsesAdaptiveNoise(policy)) {
    const auto noise=reliability::makePoseMeasurementNoise(parameters.pose_position_sigma_m,
        parameters.pose_rotation_sigma_rad,predicted.rotation,result.routed,stability,true,true,config);
    lidar.measurement_commit_allowed=noise.valid;
    if (noise.valid) lidar.residual_covariance=noise.covariance;
  }
  result.same_factor_noise_allowed=lidar.measurement_commit_allowed;
  result.attempted=lidar.ndt_converged&&lidar.map_support_valid&&result.routed.reliable_dimension>0;
  if (result.attempted && adapter.previewLidarMeasurement(lidar,&result.preview,nullptr)) {
    result.preview_valid=true; result.rank=result.preview.reliable_rank;
    if (buildLidarResidual(predicted,result.preview,&result.selected_residual,nullptr)) {
      const Eigen::MatrixXd basis=result.preview.measurement_basis.leftCols(result.rank);
      result.selected_covariance=basis.transpose()*result.preview.covariance*basis;
    }
    if (r2PolicyUsesNisGate(policy)) {
      result.nis=evaluateSelectedLidarNis(predicted,result.preview,prior,chiSquare99Threshold(result.rank));
      lidar.measurement_commit_allowed=lidar.measurement_commit_allowed&&result.nis.valid&&result.nis.accepted;
    }
  } else if (r2PolicyUsesNisGate(policy)) lidar.measurement_commit_allowed=false;
  result.admissible=result.attempted && result.preview_valid && lidar.measurement_commit_allowed;
  return result;
}

FixedLagProducerResult runFixedLagProducer(
    const p4_i2::Inputs& inputs, const std::vector<ScanAsset>& assets,
    const RuntimeParameters& parameters, const Pose3d& initial_lidar,
    const Pose3d& extrinsic, const Cloud::Ptr& target,
    const std::vector<VisualMeasurement>& visual,
    const std::function<Cloud::Ptr(const ScanAsset&)>& source_provider,
    std::ostream& trajectory, std::ostream& diagnostics, std::ostream& runtime,
    std::uint64_t initialization_stamp_ns, R2Policy policy,
    const reliability::DualReliabilityConfig& config = {},
    const fixed_lag::FixedLagOptions& options = {},
    const WindowOwnedProducerInput* window_owned = nullptr,
    fixed_lag::LidarCloudProvenance compatibility_cloud_provenance =
        fixed_lag::LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW,
    std::ostream* deskew_evidence = nullptr,
    std::ostream* preopt_capsule = nullptr,
    std::ostream* optimizer_trace_output = nullptr,
    std::ostream* directional_derivative_output = nullptr,
    std::ostream* damping_sweep_output = nullptr,
    std::ostream* optimizer_failure_summary = nullptr,
    std::ostream* marginalization_trace_output = nullptr,
    std::ostream* marginalization_failure_summary = nullptr,
    const std::string& marginalization_failure_capsule_path = {},
    std::ostream* covariance_request_output = nullptr,
    std::ostream* covariance_comparison_output = nullptr,
    std::ostream* soak_health_output = nullptr,
    A3gR3EvidenceCapture* a3g_r3_capture = nullptr,
    const p6_tracking::Config& tracking_config = {},
    std::ostream* tracking_output = nullptr) {
  using namespace fixed_lag;
  const auto started=std::chrono::steady_clock::now();
  if(assets.empty() || inputs.imu.empty() || !target || target->empty())
    throw std::runtime_error("invalid_fixed_lag_producer_input");
  const FixedLagInitializationSeed seed=initializeFixedLagProducer(
      inputs.imu,parameters,initial_lidar,extrinsic,initialization_stamp_ns);
  WindowState initial;
  initial.stamp_ns=seed.stamp_ns;
  initial.rotation=seed.map_T_imu.orientation.toRotationMatrix();
  initial.position=seed.map_T_imu.position; initial.velocity=seed.velocity;
  initial.gyro_bias=seed.gyro_bias; initial.accel_bias=seed.accel_bias;
  FixedLagAdapterCalibration calibration;
  calibration.T_imu_lidar=extrinsic; calibration.reliability_config=config;
  calibration.allow_compatibility_visual_inputs = window_owned == nullptr;
  FixedLagEventAdapter adapter(options,makeWindowImuNoise(parameters,seed.gravity),calibration);
  std::string reason;
  if(!adapter.initialize(initial,seed.information15,Vector15d::Zero(),&reason))
    throw std::runtime_error("fixed_lag_prior:"+reason);
  std::cout << "initial_square_root_factorization=" << adapter.summary().initial_square_root_status << '\n';
  // No IKFoM object exists below this boundary. The adapter is the state owner.
  std::vector<ProducerEvent> stream;
  std::size_t raw_scans_before_handoff=0;
  for(std::size_t i=0;i<assets.size();++i) {
    if(i>=inputs.scans.size() || assets[i].stamp_ns!=inputs.scans[i].stamp_ns ||
        assets[i].transaction_id!=inputs.scans[i].transaction_id ||
        (!window_owned && assets[i].stamp_ns<seed.stamp_ns))
      throw std::runtime_error("fixed_lag_scan_identity_or_time_mismatch");
    if (window_owned) {
      if (!window_owned->provider || window_owned->scan_start_ns.size()!=assets.size() ||
          !window_owned->scan_start_ns[i] ||
          window_owned->scan_start_ns[i]>=assets[i].stamp_ns)
        throw std::runtime_error("RAW_POINT_TIME_UNAVAILABLE:NOT_ELIGIBLE_FOR_WINDOW_OWNED_DESKEW");
      // Raw export preserves acquisition during initialization. These scans
      // have no Window-owned start state and cannot be deskewed after handoff.
      // Skip, never snap/reindex their physical timestamps or transaction IDs.
      if(window_owned->scan_start_ns[i]<seed.stamp_ns) {
        ++raw_scans_before_handoff; continue;
      }
      stream.push_back({window_owned->scan_start_ns[i],ProducerEventType::LIDAR_SCAN_START,i});
      stream.push_back({assets[i].stamp_ns,ProducerEventType::LIDAR_SCAN_END,i});
    } else stream.push_back({assets[i].stamp_ns,ProducerEventType::LIDAR_SCAN,i});
  }
  if(window_owned && stream.empty()) throw std::runtime_error("NO_RAW_SCAN_AFTER_WINDOW_HANDOFF");
  for(std::size_t i=0;i<visual.size();++i) {
    if(visual[i].ref_ns<seed.stamp_ns || visual[i].cur_ns>assets.back().stamp_ns ||
        visual[i].cur_ns<=visual[i].ref_ns) continue;
    stream.push_back({visual[i].ref_ns,ProducerEventType::VISUAL_REFERENCE,i});
    stream.push_back({visual[i].cur_ns,ProducerEventType::VISUAL_CURRENT,i});
  }
  sortProducerEvents(&stream);
  std::size_t imu_cursor=static_cast<std::size_t>(std::upper_bound(
      inputs.imu.begin(),inputs.imu.end(),seed.stamp_ns,
      [](std::uint64_t t,const ImuSample& s){return t<s.stamp_ns;})-inputs.imu.begin());
  if(imu_cursor==0) throw std::runtime_error("initial_imu_anchor_missing");
  --imu_cursor;
  std::uint64_t buffered_stamp=0;
  GeometricNdt ndt;
  configureNdt(ndt,target);
  ndt.setResolution(0.8); ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(1e-5); ndt.setMaximumIterations(80);
  const Eigen::Matrix4d T_il=poseMatrix(extrinsic);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  FixedLagProducerResult result;
  p6_tracking::Tracker tracker(tracking_config);
  if (tracking_output)
    *tracking_output << "transaction_id,stamp_ns,row_type,health,consecutive_failures,seed,effective_registration,ndt_iterations,objective,fitness,uobs_status,correspondences,reliable_rank,nis_valid,nis,nis_threshold,admissible,committed,recovery_attempts,last_reliable_stamp,map_support_status,rejected_covariances,sampled_neighbor_queries,queries_with_neighbor_0p8,queries_with_neighbor_1p6\n";
  result.raw_scans_before_handoff=raw_scans_before_handoff;
  trajectory<<std::setprecision(17)<<"transaction_id,stamp_ns,time_s,px,py,pz,qx,qy,qz,qw\n";
  diagnostics<<std::setprecision(17)<<"timestamp,event_type,window_nodes,window_span,optimizer_status,optimizer_cost_before,optimizer_cost_after,predicted_px,predicted_py,predicted_pz,predicted_qx,predicted_qy,predicted_qz,predicted_qw,ndt_converged,uobs_valid,weak_dimension,reliable_dimension,window_covariance_valid,window_position_sigma_max,window_rotation_sigma_max,unonlocal_probe_triggered,unonlocal_status,lidar_factor_attempted,lidar_factor_committed,lidar_selected_rank,lidar_nis,lidar_nis_threshold,visual_sensor_quality,visual_mode,visual_selected_rank,visual_trigger_status,visual_basis_source_lidar_stamp,imu_factor_count,lidar_factor_count,visual_factor_count,r2_policy,imu_buffer_last_stamp,lidar_source_provenance,visual_source_provenance,input_eligibility,post_handoff_ikfom_calls,marginalization_backend,initial_square_root_status,square_root_prior_rows,square_root_prior_columns,square_root_prior_bytes,qr_marginalization_ms\n";
  runtime<<"timestamp,event_type,ndt_calls,ndt_ms,event_ms,linearization_ms,solve_ms,marginal_covariance_ms,rank_diagnostic_ms,solver_status,sparse_fallback_count\n";
  if (deskew_evidence)
    *deskew_evidence<<std::setprecision(17)
        <<"transaction_id,scan_start_ns,scan_end_ns,raw_point_count,point_stamp_min_ns,point_stamp_max_ns,"
        <<"anchor_px,anchor_py,anchor_pz,anchor_qx,anchor_qy,anchor_qz,anchor_qw,"
        <<"predicted_end_px,predicted_end_py,predicted_end_pz,predicted_end_qx,predicted_end_qy,predicted_end_qz,predicted_end_qw,"
        <<"deskew_point_count,displacement_mean_m,displacement_p95_m,displacement_max_m,status\n";
  if (preopt_capsule)
    *preopt_capsule<<std::setprecision(17)
      <<"transaction_id,stamp_ns,window_nodes,window_span_s,predicted_position,predicted_rotation_xyzw,"
      <<"ndt_converged,ndt_fitness,ndt_objective,ndt_iterations,ndt_runtime_ms,ndt_terminal_pose,"
      <<"uobs_valid,uobs_status,uobs_weak_dimension,uobs_reliable_dimension,"
      <<"translation_eigenvalues,rotation_eigenvalues,translation_weak_ratio,rotation_weak_ratio,"
      <<"weak_translation_direction_map,weak_rotation_direction_map,weak_basis,"
      <<"pre_measurement_covariance_valid,p15_diagonal,p15_min_eigenvalue,p15_max_eigenvalue,"
      <<"p_map6_min_eigenvalue,p_map6_max_eigenvalue,unonlocal_triggered,unonlocal_status,"
      <<"unonlocal_positive_translation_delta,unonlocal_positive_rotation_delta,"
      <<"unonlocal_negative_translation_delta,unonlocal_negative_rotation_delta,"
      <<"measurement_noise_mode,adaptive_R6,R6_eigenvalues,measurement_preview_valid,"
      <<"reliable_rank,basis,raw_residual_r6,selected_residual_rs,selected_covariance_Rs,"
      <<"selected_nis_valid,selected_nis,nis_threshold,nis_accepted,"
      <<"lidar_attempted,lidar_committed,event_status,factor_counts_before_optimize\n";
  if (marginalization_trace_output)
    writeMarginalizationTraceHeader(*marginalization_trace_output);
  if (optimizer_trace_output)
    *optimizer_trace_output<<std::setprecision(17)
      <<"transaction_id,stamp_ns,iteration,lidar_snapshot_generation,max_projector_change_from_previous_outer,"
      <<"candidate_basis_relinearization_calls,damping_before,damping_after,surrogate_current_cost,"
      <<"gradient_inf_norm,solver_status,raw_step_norm,applied_step_norm,step_clipped,"
      <<"g_dot_step,step_H_step,predicted_reduction,surrogate_candidate_cost,actual_reduction,rho,rho_valid,accepted,"
      <<"current_prior,current_imu,current_lidar,current_visual,current_latest_lidar_cost,"
      <<"candidate_prior,candidate_imu,candidate_lidar,candidate_visual,candidate_latest_lidar_cost,"
      <<"applied_step_components,small_step_termination,termination_reason,candidate_rollback_state_difference\n";
  if (directional_derivative_output)
    *directional_derivative_output<<std::setprecision(17)
      <<"transaction_id,stamp_ns,direction_name,epsilon,diagnostic_relinearized_basis_fd,"
      <<"diagnostic_relinearized_basis_model,diagnostic_relinearized_basis_relative_error,"
      <<"frozen_basis_fd,frozen_basis_model,"
      <<"frozen_basis_relative_error,valid,status,direction_components\n";
  if (damping_sweep_output)
    *damping_sweep_output<<std::setprecision(17)
      <<"transaction_id,stamp_ns,damping,solved,solver_status,raw_step_norm,applied_step_norm,"
      <<"step_clipped,g_dot_step,step_H_step,predicted_reduction,diagnostic_relinearized_basis_candidate_cost,"
      <<"diagnostic_relinearized_basis_actual_reduction,frozen_basis_candidate_cost,frozen_basis_actual_reduction,"
      <<"diagnostic_relinearized_basis_rho,rho_valid,diagnostic_relinearized_prior,diagnostic_relinearized_imu,"
      <<"diagnostic_relinearized_lidar,diagnostic_relinearized_visual,"
      <<"diagnostic_relinearized_latest_lidar_cost,frozen_prior,frozen_imu,frozen_lidar,frozen_visual,"
      <<"frozen_latest_lidar_cost\n";
  const auto flush_optimizer_trace_rows = [&](
      std::uint64_t transaction_id, std::uint64_t stamp_ns,
      const std::vector<fixed_lag::OptimizerIterationTrace>& rows) {
    if (!optimizer_trace_output) return;
    for (const auto& row : rows)
      *optimizer_trace_output<<transaction_id<<','<<stamp_ns<<','
        <<row.iteration<<','<<row.lidar_snapshot_generation<<','
        <<row.max_projector_change_from_previous_outer<<','
        <<row.candidate_basis_relinearization_calls<<','
        <<row.damping_before<<','<<row.damping_after<<','
        <<row.surrogate_current_cost<<','<<row.gradient_inf_norm<<','
        <<row.solver_status<<','<<row.raw_step_norm<<','<<row.applied_step_norm<<','
        <<row.step_clipped<<','<<row.g_dot_step<<','<<row.step_H_step<<','
        <<row.predicted_reduction<<','<<row.surrogate_candidate_cost<<','
        <<row.actual_reduction<<','<<row.rho<<','<<row.rho_valid<<','
        <<row.accepted<<','<<row.current_breakdown.prior_cost<<','
        <<row.current_breakdown.imu_cost<<','<<row.current_breakdown.lidar_cost<<','
        <<row.current_breakdown.visual_cost<<','
        <<row.current_breakdown.latest_lidar_factor_cost<<','
        <<row.candidate_breakdown.prior_cost<<','
        <<row.candidate_breakdown.imu_cost<<','
        <<row.candidate_breakdown.lidar_cost<<','
        <<row.candidate_breakdown.visual_cost<<','
        <<row.candidate_breakdown.latest_lidar_factor_cost<<','
        <<matrixField(Eigen::MatrixXd(row.applied_step))<<','
        <<row.small_step_termination<<','<<row.termination_reason<<','
        <<row.candidate_rollback_state_difference<<'\n';
    optimizer_trace_output->flush();
    if (!*optimizer_trace_output)
      throw std::runtime_error("optimizer_trace_flush_failed");
  };
  std::size_t marginalization_trace_cursor = 0;
  if (covariance_request_output) p6_i6b::writeCovarianceRequestHeader(*covariance_request_output);
  if (covariance_comparison_output)
    *covariance_comparison_output<<std::setprecision(17)<<
        "transaction_id,stamp_ns,legacy_valid,P15_relative_error,Pmap_relative_error,"
        "production_NIS_valid,production_NIS,threshold,production_NIS_accepted,"
        "legacy_same_factor_NIS_valid,legacy_same_factor_NIS,legacy_same_factor_NIS_accepted,"
        "production_probe_trigger,legacy_probe_trigger,production_LiDAR_committed,"
        "legacy_same_factor_admission\n";
  const auto flushMarginalizationTraceRows = [&] (
      std::uint64_t transaction_id, std::uint64_t stamp_ns) {
    if (!marginalization_trace_output) return;
    const auto* diagnostic_window = adapter.debugWindowForDiagnostics();
    if (!diagnostic_window) return;
    const auto& rows = diagnostic_window->marginalizationTraceForDiagnostics();
    for (; marginalization_trace_cursor < rows.size();
         ++marginalization_trace_cursor)
      writeMarginalizationTraceRow(*marginalization_trace_output,
          transaction_id, stamp_ns, rows[marginalization_trace_cursor]);
    marginalization_trace_output->flush();
  };
  if (soak_health_output)
    *soak_health_output << std::setprecision(17)
        << "transaction_id,stamp_ns,event,completed,state_finite,so3_defect,det_defect,prior_finite,prior_rows,prior_columns,window_nodes,window_span,imu_factors,lidar_factors,visual_factors,active_ids,revision,optimized_revision,optimizer_status,optimizer_success,marginalization_status,dense_reference_requests,covariance_requests,sparse_fallbacks,optimizer_and_marginalization_ms,reason\n";
  for(const ProducerEvent& event:stream) {
    double optimizer_and_marginalization_ms = 0;
    const auto writeSoakHealth = [&](bool completed, const std::string& failure) {
      if (!soak_health_output) return;
      const auto* w = adapter.debugWindowForDiagnostics();
      const auto s = adapter.summary();
      bool finite = w != nullptr, prior_finite = false;
      double orthogonal = 0, determinant = 0;
      if (w) {
        for (const auto& x : w->states()) {
          finite = finite && x.rotation.allFinite() && x.position.allFinite() &&
              x.velocity.allFinite() && x.gyro_bias.allFinite() && x.accel_bias.allFinite();
          orthogonal = std::max(orthogonal,
              (x.rotation.transpose()*x.rotation-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff());
          determinant = std::max(determinant, std::abs(x.rotation.determinant()-1));
        }
        const auto& p = w->priorSquareRootRows();
        prior_finite = p.a.allFinite() && p.b.allFinite() &&
            p.a.rows()==p.b.size() && p.a.cols()==static_cast<Eigen::Index>(15*w->states().size());
      }
      const auto tx = (event.type==ProducerEventType::LIDAR_SCAN_START ||
          event.type==ProducerEventType::LIDAR_SCAN_END || event.type==ProducerEventType::LIDAR_SCAN)
          ? assets[event.source_index].transaction_id : 0;
      std::string clean = failure;
      std::replace(clean.begin(),clean.end(),',',';');
      std::replace(clean.begin(),clean.end(),'\n',' ');
      *soak_health_output << tx << ',' << event.stamp_ns << ',' << toString(event.type) << ','
          << completed << ',' << finite << ',' << orthogonal << ',' << determinant << ','
          << prior_finite << ',' << s.square_root_prior_rows << ',' << s.square_root_prior_columns << ','
          << s.window_node_count << ',' << s.window_time_span_s << ',' << s.imu_factor_count << ','
          << s.lidar_factor_count << ',' << s.visual_factor_count << ',' << s.active_observation_id_count << ','
          << s.window_revision << ',' << s.optimized_revision << ',' << s.optimizer_status << ','
          << s.optimizer_success << ',' << s.marginalization_status << ',' << s.dense_marginal_reference_requests << ','
          << s.marginal_covariance_requests << ',' << s.sparse_solver_fallback_count << ','
          << optimizer_and_marginalization_ms << ',' << clean << '\n';
      soak_health_output->flush();
    };
    try {
    const auto event_start=std::chrono::steady_clock::now();
    // Append through exactly the first right boundary required for this event.
    while(buffered_stamp<event.stamp_ns) {
      if(imu_cursor>=inputs.imu.size()) throw std::runtime_error("event_imu_right_boundary_missing");
      const auto& sample=inputs.imu[imu_cursor++];
      if(!adapter.appendImu(sample,&reason)) throw std::runtime_error("producer_imu:"+reason);
      buffered_stamp=sample.stamp_ns;
    }
    WindowState predicted;
    if(!adapter.prepareStateAt(event.stamp_ns,&predicted,&reason))
      throw std::runtime_error("producer_prediction:"+reason);
    WindowMarginalCovariance prior;
    const bool lidar_terminal = event.type==ProducerEventType::LIDAR_SCAN ||
        event.type==ProducerEventType::LIDAR_SCAN_END;
    // This producer enables U_nonlocal in every policy; P2/P3 also require P.
    // Request after prediction/IMU admission, before any current LiDAR factor.
    const auto pre_measurement = adapter.summary();
    if (lidar_terminal) adapter.latestMarginalCovariance(&prior,nullptr);
    else prior.status="NOT_REQUESTED_NON_LIDAR_EVENT";
    if (lidar_terminal && covariance_request_output)
      p6_i6b::writeCovarianceRequest(*covariance_request_output,
          assets[event.source_index].transaction_id,event.stamp_ns,prior);
    // The new backend cannot conceal an unavailable production P by silently
    // continuing the short-link gate, or by borrowing its legacy shadow P.
    if (lidar_terminal && options.marginal_covariance_backend==
        MarginalCovarianceBackend::SQUARE_ROOT_QR && !prior.valid)
      throw std::runtime_error("producer_square_root_covariance:"+prior.detail);
    const auto covariance_diagnostic = adapter.summary();
    if (covariance_diagnostic.dense_marginal_reference_requests != 0 ||
        covariance_diagnostic.marginal_covariance_requests !=
            pre_measurement.marginal_covariance_requests + (lidar_terminal ? 1 : 0) ||
        covariance_diagnostic.lidar_factor_count != pre_measurement.lidar_factor_count)
      throw std::runtime_error("producer_pre_measurement_covariance_contract");
    Candidate nominal;
    reliability::LocalRisk routed;
    reliability::LocalObservability local_observability;
    reliability::NonlocalTerminalStability stability;
    stability.status="NOT_PROBED";
    std::string unonlocal_status="NOT_PROBED";
    SelectedLidarNis nis;
    bool lidar_attempted=false,lidar_committed=false,probed=false,quality=false,uobs_valid=false;
    bool measurement_preview_valid=false;
    bool production_probe_trigger=false,legacy_probe_trigger=false;
    LidarWindowMeasurement preview_measurement;
    Eigen::VectorXd selected_residual;
    Eigen::MatrixXd selected_covariance;
    Matrix6d selected_measurement_covariance=Matrix6d::Constant(nan);
    std::uint64_t calls=0;
    double ndt_ms=0;
    std::size_t recovery_attempts=0;
    int lidar_rank=0,visual_rank=0;
    std::uint64_t basis_stamp=0;
    std::string visual_mode="NOT_APPLICABLE",visual_trigger="NOT_APPLICABLE";
    std::string lidar_provenance="NOT_APPLICABLE",visual_provenance="NOT_APPLICABLE";
    if(event.type==ProducerEventType::LIDAR_SCAN || event.type==ProducerEventType::LIDAR_SCAN_END) {
      const auto& asset=assets[event.source_index];
      Cloud::Ptr source;
      std::optional<RawTimedScan> raw_storage;
      std::optional<WindowState> anchor_storage;
      std::optional<WindowDeskewResult> deskew_storage;
      if (window_owned) {
        raw_storage.emplace();
        anchor_storage.emplace();
        deskew_storage.emplace();
        auto& raw=*raw_storage;
        auto& anchor=*anchor_storage;
        auto& deskew=*deskew_storage;
        if (!window_owned->provider(asset,&raw,&reason)) throw std::runtime_error("raw_timed_scan:"+reason);
        if (!rawDeskewInputAllowed(raw.provenance,&reason)) throw std::runtime_error(reason);
        if (raw.transaction_id!=asset.transaction_id || raw.scan_end_ns!=asset.stamp_ns ||
            raw.scan_start_ns!=window_owned->scan_start_ns[event.source_index])
          throw std::runtime_error("raw_timed_scan_identity_mismatch");
        if (!adapter.activeStateAt(raw.scan_start_ns,&anchor,&reason)) throw std::runtime_error(reason);
        auto left = std::lower_bound(inputs.imu.begin(),inputs.imu.begin()+imu_cursor,raw.scan_start_ns,
            [](const ImuSample& s,std::uint64_t t){return s.stamp_ns<t;});
        if(left!=inputs.imu.begin() && (left==inputs.imu.end() || left->stamp_ns>raw.scan_start_ns)) --left;
        const p4_i2::ImuVector causal_imu(left,inputs.imu.begin()+imu_cursor);
        if (!deskewScanWithWindowState(anchor,raw.scan_start_ns,raw.scan_end_ns,causal_imu,
              makeWindowImuNoise(parameters,seed.gravity),extrinsic,raw.points,&deskew,&reason))
          throw std::runtime_error("window_owned_deskew:"+reason);
        if (deskew.cloud_end_frame.size()!=raw.points.size())
          throw std::runtime_error("WINDOW_DESKEW_POINT_COUNT_CHANGED");
        if (deskew_evidence) {
          std::uint64_t point_stamp_min=std::numeric_limits<std::uint64_t>::max();
          std::uint64_t point_stamp_max=0;
          std::vector<double> displacement;
          displacement.reserve(raw.points.size());
          double displacement_sum=0.0;
          double displacement_max=0.0;
          for (std::size_t i=0;i<raw.points.size();++i) {
            point_stamp_min=std::min(point_stamp_min,raw.points[i].stamp_ns);
            point_stamp_max=std::max(point_stamp_max,raw.points[i].stamp_ns);
            const double d=(deskew.cloud_end_frame[i].position-raw.points[i].position).norm();
            displacement.push_back(d); displacement_sum+=d; displacement_max=std::max(displacement_max,d);
          }
          std::sort(displacement.begin(),displacement.end());
          const std::size_t p95_index=displacement.empty()?0:
              std::min(displacement.size()-1,static_cast<std::size_t>(std::ceil(0.95*displacement.size()))-1);
          const Eigen::Quaterniond q_anchor(anchor.rotation),q_end(deskew.predicted_end_state.rotation);
          *deskew_evidence<<asset.transaction_id<<','<<raw.scan_start_ns<<','<<raw.scan_end_ns<<','
              <<raw.points.size()<<','<<point_stamp_min<<','<<point_stamp_max<<','
              <<anchor.position.x()<<','<<anchor.position.y()<<','<<anchor.position.z()<<','
              <<q_anchor.x()<<','<<q_anchor.y()<<','<<q_anchor.z()<<','<<q_anchor.w()<<','
              <<deskew.predicted_end_state.position.x()<<','<<deskew.predicted_end_state.position.y()<<','
              <<deskew.predicted_end_state.position.z()<<','<<q_end.x()<<','<<q_end.y()<<','
              <<q_end.z()<<','<<q_end.w()<<','<<deskew.cloud_end_frame.size()<<','
              <<(displacement.empty()?0.0:displacement_sum/displacement.size())<<','
              <<(displacement.empty()?0.0:displacement[p95_index])<<','<<displacement_max<<','<<deskew.status<<'\n';
        }
        Cloud::Ptr end_cloud(new Cloud);
        end_cloud->reserve(deskew.cloud_end_frame.size());
        for (const auto& point : deskew.cloud_end_frame) {
          Point p; p.x=point.position.x(); p.y=point.position.y(); p.z=point.position.z();
          end_cloud->push_back(p);
        }
        source=preprocessSource(end_cloud);
        lidar_provenance=toString(deskew.provenance);
        ++result.window_deskew_count;
      } else {
        source=source_provider(asset);
        lidar_provenance=toString(compatibility_cloud_provenance);
      }
      if(!source || source->empty()) throw std::runtime_error("empty_preprocessed_source");
      if(!window_owned && asset.expected_source_hash_available && sourceCloudHash(source)!=asset.expected_source_hash)
        throw std::runtime_error("prepared_source_hash_mismatch");
      Pose3d predicted_pose;
      predicted_pose.position=predicted.position;
      predicted_pose.orientation=Eigen::Quaterniond(predicted.rotation);
      const Eigen::Matrix4d prediction=poseMatrix(predicted_pose);
      if (window_owned && a3g_r3_capture && a3g_r3_capture->selected(asset.transaction_id))
        a3g_r3_capture->captureInputs(raw_storage.value(),anchor_storage.value(),predicted,
                                      deskew_storage.value(),*source,extrinsic,
                                      prediction*T_il,pre_measurement);
      Eigen::Matrix4d selected_initial_seed=prediction*T_il;
      nominal=runNdtCandidate(ndt,source,selected_initial_seed,0,"WINDOW_M0");
      calls=1; ndt_ms=nominal.runtime_ms;
      auto evaluation=evaluateLidarCandidate(ndt,source,nominal,asset,predicted,prior,
          extrinsic,parameters,policy,config,adapter,tracking_config.enabled);
      calls+=evaluation.probe_calls; ndt_ms+=evaluation.probe_ms;
      const auto log_candidate=[&](const Candidate& candidate,const LidarCandidateEvaluation& e) {
        if (!tracking_output) return;
        *tracking_output<<std::setprecision(17)<<asset.transaction_id<<','<<asset.stamp_ns
            <<",CANDIDATE,"<<p6_tracking::name(tracker.health())<<','<<tracker.consecutiveFailures()
            <<','<<candidate.seed_name<<','<<e.effective<<','<<candidate.iterations<<','
            <<candidate.objective<<','<<candidate.fitness<<','<<e.local.status<<','
            <<e.local.valid_correspondence_count<<','<<e.rank<<','<<e.nis.valid<<','
            <<e.nis.nis<<','<<e.nis.threshold<<','<<e.admissible<<",0,"<<recovery_attempts
            <<','<<tracker.lastReliableStamp()<<','<<e.local.map_support_status<<','
            <<e.local.rejected_covariance_count<<','<<e.search.sampled_source_points<<','
            <<e.search.queries_with_neighbors_r<<','<<e.search.queries_with_neighbors_2r<<'\n';
        tracking_output->flush();
      };
      log_candidate(nominal,evaluation);
      if (tracker.needsRecovery(evaluation.admissible)) {
        for (const auto& seed_candidate : tracker.seeds(asset.stamp_ns,prediction)) {
          ++recovery_attempts;
          auto candidate=runNdtCandidate(ndt,source,seed_candidate.map_T_imu*T_il,
              static_cast<int>(recovery_attempts),seed_candidate.label);
          ++calls; ndt_ms+=candidate.runtime_ms;
          auto candidate_evaluation=evaluateLidarCandidate(ndt,source,candidate,asset,predicted,prior,
              extrinsic,parameters,policy,config,adapter,true,&seed_candidate.map_T_imu);
          calls+=candidate_evaluation.probe_calls; ndt_ms+=candidate_evaluation.probe_ms;
          log_candidate(candidate,candidate_evaluation);
          if (candidate_evaluation.admissible) {
            selected_initial_seed=seed_candidate.map_T_imu*T_il;
            nominal=std::move(candidate); evaluation=std::move(candidate_evaluation);
            break;
          }
        }
      }
      local_observability=evaluation.local; uobs_valid=local_observability.valid;
      routed=evaluation.routed; stability=evaluation.stability; nis=evaluation.nis;
      probed=evaluation.probed; unonlocal_status=evaluation.nonlocal_status;
      production_probe_trigger=evaluation.production_probe_trigger;
      legacy_probe_trigger=evaluation.legacy_probe_trigger;
      measurement_preview_valid=evaluation.preview_valid; preview_measurement=evaluation.preview;
      lidar_rank=evaluation.rank; lidar_attempted=evaluation.attempted;
      selected_residual=evaluation.selected_residual; selected_covariance=evaluation.selected_covariance;
      FrozenLidarEvent lidar=evaluation.event;
      selected_measurement_covariance=lidar.residual_covariance;
      const bool same_factor_noise_allowed=evaluation.same_factor_noise_allowed;
      lidar_committed=adapter.processLidarEvent(lidar,&reason);
      if(!lidar_committed && adapter.lastEventStatus().disposition!=AdapterEventDisposition::SKIPPED_INVALID_SOURCE)
        throw std::runtime_error("producer_lidar:"+reason);
      result.lidar_committed+=lidar_committed;
      if (window_owned && a3g_r3_capture && a3g_r3_capture->selected(asset.transaction_id)) {
        const auto after_lidar_summary=adapter.summary();
        a3g_r3_capture->captureTerminalMetadata(
            raw_storage.value(),anchor_storage.value(),predicted,deskew_storage.value(),
            *source,extrinsic,selected_initial_seed,nominal,
            local_observability,routed,lidar,measurement_preview_valid,lidar_rank,
            nis.valid,nis.nis,nis.threshold,nis.accepted,lidar_attempted,
            lidar_committed,toString(adapter.lastEventStatus().disposition),
            adapter.lastEventStatus().reason,pre_measurement,after_lidar_summary);
      }
      if (covariance_comparison_output) {
        WindowMarginalCovariance shadow;
        shadow.valid=prior.legacy_shadow_valid;
        shadow.covariance15=prior.legacy_covariance15;
        shadow.map_pose_covariance6=prior.legacy_map_pose_covariance6;
        SelectedLidarNis shadow_nis;
        if (measurement_preview_valid && shadow.valid && r2PolicyUsesNisGate(policy))
          shadow_nis=evaluateSelectedLidarNis(predicted,preview_measurement,shadow,
              chiSquare99Threshold(lidar_rank));
        const double p15_error=shadow.valid?(prior.covariance15-shadow.covariance15).norm()/shadow.covariance15.norm():nan;
        const double map_error=shadow.valid?(prior.map_pose_covariance6-shadow.map_pose_covariance6).norm()/shadow.map_pose_covariance6.norm():nan;
        // This is a same-raw-factor counterfactual, NOT an alternate replay.
        // If probe triggers differ, alternate NDT probe terminals are unknown.
        const bool shadow_admission=shadow.valid && lidar_attempted && same_factor_noise_allowed &&
            (!r2PolicyUsesNisGate(policy) || (shadow_nis.valid && shadow_nis.accepted));
        *covariance_comparison_output<<asset.transaction_id<<','<<event.stamp_ns<<','<<shadow.valid<<','
            <<p15_error<<','<<map_error<<','<<nis.valid<<','<<nis.nis<<','<<nis.threshold<<','<<nis.accepted<<','
            <<shadow_nis.valid<<','<<shadow_nis.nis<<','<<shadow_nis.accepted<<','
            <<production_probe_trigger<<','<<legacy_probe_trigger<<','<<lidar_committed<<','<<shadow_admission<<'\n';
        covariance_comparison_output->flush();
        if (!*covariance_comparison_output) throw std::runtime_error("covariance_comparison_flush_failed");
      }
    } else if(event.type==ProducerEventType::LIDAR_SCAN_START) {
      // prepareStateAt above created a real optimizable scan-start node.
      visual_mode="SCAN_START_STATE";
    } else if(event.type==ProducerEventType::VISUAL_REFERENCE) {
      if(!adapter.processVisualReferenceStamp(event.stamp_ns,&reason))
        throw std::runtime_error("producer_visual_reference:"+reason);
      visual_mode="REFERENCE_STATE"; visual_trigger=reason;
    } else {
      auto v=frozenVisual(visual[event.source_index],0.05);
      if(window_owned) v.provenance=event.source_index<window_owned->visual_provenance.size() ?
          window_owned->visual_provenance[event.source_index] : VisualMeasurementProvenance::UNKNOWN;
      visual_provenance=toString(v.provenance);
      if(!window_owned && !formalVisualInputAllowed(v.provenance))
        visual_provenance+=";VISUAL_PROVENANCE_COMPATIBILITY_ONLY";
      auto sensor=v.quality;
      sensor.source_valid=v.source_valid; sensor.reference_stamp_ns=v.ref_ns;
      sensor.current_stamp_ns=v.cur_ns; sensor.depth_stamp_ns=v.depth_ns;
      quality=reliability::assessVisualSensorQuality(sensor,config).passed;
      const bool committed=adapter.processVisualEvent(v,&reason);
      if(!committed && adapter.lastEventStatus().disposition!=AdapterEventDisposition::SKIPPED_INVALID_SOURCE &&
          adapter.lastEventStatus().disposition!=AdapterEventDisposition::REJECTED_WINDOW)
        throw std::runtime_error("producer_visual:"+reason);
      const auto& admission=adapter.lastVisualAdmission();
      visual_mode=toString(admission.mode); visual_rank=committed?admission.selected_rank:0;
      visual_trigger=committed?admission.trigger_status:reason;
      basis_stamp=admission.basis_source_lidar_stamp_ns;
      result.visual_committed+=committed;
    }
    if (preopt_capsule &&
        (event.type==ProducerEventType::LIDAR_SCAN ||
         event.type==ProducerEventType::LIDAR_SCAN_END)) {
      const auto summary_before_optimize=adapter.summary();
      const auto eigen_bounds=[](const Eigen::MatrixXd& matrix) {
        const double nan_value=std::numeric_limits<double>::quiet_NaN();
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
            0.5*(matrix+matrix.transpose()));
        if (!matrix.allFinite() || eigen.info()!=Eigen::Success ||
            !eigen.eigenvalues().allFinite())
          return std::pair<double,double>(nan_value,nan_value);
        return std::pair<double,double>(eigen.eigenvalues().minCoeff(),
                                        eigen.eigenvalues().maxCoeff());
      };
      double p15_min=std::numeric_limits<double>::quiet_NaN();
      double p15_max=p15_min, pmap_min=p15_min, pmap_max=p15_min;
      if (prior.valid) {
        const auto p15=eigen_bounds(prior.covariance15);
        const auto pmap=eigen_bounds(prior.map_pose_covariance6);
        p15_min=p15.first; p15_max=p15.second;
        pmap_min=pmap.first; pmap_max=pmap.second;
      }
      Eigen::VectorXd r_eigenvalues=Eigen::VectorXd::Constant(
          6,std::numeric_limits<double>::quiet_NaN());
      if (selected_measurement_covariance.allFinite()) {
        Eigen::SelfAdjointEigenSolver<Matrix6d> r_eigen(
            0.5*(selected_measurement_covariance+
                 selected_measurement_covariance.transpose()));
        if (r_eigen.info()==Eigen::Success && r_eigen.eigenvalues().allFinite())
          r_eigenvalues=r_eigen.eigenvalues();
      }
      Eigen::Matrix<double,6,1> raw_residual=
          Eigen::Matrix<double,6,1>::Constant(nan);
      if (measurement_preview_valid) {
        raw_residual.head<3>()=preview_measurement.measured_position-predicted.position;
        Eigen::Quaterniond dq(predicted.rotation.transpose()*
                              preview_measurement.measured_rotation);
        dq.normalize();
        if (dq.w()<0.0) dq.coeffs()*=-1.0;
        const double sine_half=dq.vec().norm();
        if (sine_half<1e-12) raw_residual.tail<3>()=2.0*dq.vec();
        else raw_residual.tail<3>()=dq.vec()*(2.0*std::atan2(sine_half,
                std::clamp(dq.w(),-1.0,1.0))/sine_half);
      }
      const Eigen::Quaterniond predicted_q(predicted.rotation);
      const auto& asset=assets[event.source_index];
      *preopt_capsule<<asset.transaction_id<<','<<event.stamp_ns<<','
        <<summary_before_optimize.window_node_count<<','
        <<summary_before_optimize.window_time_span_s<<','
        <<vectorField(predicted.position)<<','
        <<predicted_q.x()<<';'<<predicted_q.y()<<';'<<predicted_q.z()<<';'<<predicted_q.w()<<','
        <<nominal.converged<<','<<nominal.fitness<<','<<nominal.objective<<','
        <<nominal.iterations<<','<<nominal.runtime_ms<<','
        <<matrixField(nominal.pose)<<','
        <<local_observability.valid<<','<<local_observability.status<<','
        <<routed.weak_dimension<<','<<routed.reliable_dimension<<','
        <<matrixField(Eigen::MatrixXd(local_observability.translation_block_eigenvalues))<<','
        <<matrixField(Eigen::MatrixXd(local_observability.rotation_block_eigenvalues))<<','
        <<routed.translation_weak_ratio<<','<<routed.rotation_weak_ratio<<','
        <<vectorField(routed.map_translation_weak_direction)<<','
        <<vectorField(routed.map_rotation_weak_direction)<<','
        <<matrixField(routed.joint_weak_basis)<<','
        <<prior.valid<<','
        <<(prior.valid ? matrixField(Eigen::MatrixXd(prior.covariance15.diagonal())) : "")<<','
        <<p15_min<<','<<p15_max<<','<<pmap_min<<','<<pmap_max<<','
        <<probed<<','<<unonlocal_status<<','
        <<vectorField(stability.delta_position_imu_positive)<<','
        <<vectorField(stability.delta_rotation_positive)<<','
        <<vectorField(stability.delta_position_imu_negative)<<','
        <<vectorField(stability.delta_rotation_negative)<<','
        <<(r2PolicyUsesAdaptiveNoise(policy)?"ADAPTIVE":"BASE")<<','
        <<matrixField(selected_measurement_covariance)<<','
        <<matrixField(Eigen::MatrixXd(r_eigenvalues))<<','
        <<measurement_preview_valid<<','<<lidar_rank<<','
        <<(measurement_preview_valid ? matrixField(preview_measurement.measurement_basis) : "")<<','
        <<matrixField(Eigen::MatrixXd(raw_residual))<<','
        <<matrixField(Eigen::MatrixXd(selected_residual))<<','
        <<matrixField(selected_covariance)<<','
        <<nis.valid<<','<<nis.nis<<','<<nis.threshold<<','<<nis.accepted<<','
        <<lidar_attempted<<','<<lidar_committed<<','
        <<toString(adapter.lastEventStatus().disposition)<<';'
        <<adapter.lastEventStatus().reason<<','
        <<summary_before_optimize.imu_factor_count<<';'
        <<summary_before_optimize.lidar_factor_count<<';'
        <<summary_before_optimize.visual_factor_count<<'\n';
      preopt_capsule->flush();
      if (!*preopt_capsule) throw std::runtime_error("optimizer_preopt_capsule_flush_failed");
    }
    const auto optimizer_begin=std::chrono::steady_clock::now();
    const bool optimizer_completed=adapter.optimizeCurrentWindow(&reason);
    optimizer_and_marginalization_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-optimizer_begin).count();
    if(!optimizer_completed) {
      const std::uint64_t failed_transaction_id =
          (event.type==ProducerEventType::LIDAR_SCAN ||
           event.type==ProducerEventType::LIDAR_SCAN_END)
              ? assets[event.source_index].transaction_id : 0;
      flushMarginalizationTraceRows(failed_transaction_id, event.stamp_ns);
      if (marginalization_failure_summary) {
        *marginalization_failure_summary << std::setprecision(17);
        const auto* diagnostic_window = adapter.debugWindowForDiagnostics();
        const auto* capsule = diagnostic_window
            ? &diagnostic_window->marginalizationFailureCapsuleForDiagnostics()
            : nullptr;
        *marginalization_failure_summary
            << "transaction_id=" << failed_transaction_id << '\n'
            << "event=" << toString(event.type) << '\n'
            << "event_stamp_ns=" << event.stamp_ns << '\n'
            << "failure=producer_optimizer:" << reason << '\n';
        if (capsule && capsule->valid) {
          const bool partial_commit =
              capsule->state_stamps_before_enforcement !=
                  capsule->state_stamps_at_failure ||
              capsule->prior_hash_before_enforcement_fnv1a64 !=
                  capsule->prior_hash_at_failure_fnv1a64 ||
              capsule->factors_before_enforcement.imu !=
                  capsule->factors_at_failure.imu ||
              capsule->factors_before_enforcement.lidar !=
                  capsule->factors_at_failure.lidar ||
              capsule->factors_before_enforcement.visual !=
                  capsule->factors_at_failure.visual;
          const bool binary_written = writeMarginalizationFailureCapsuleBinary(
              marginalization_failure_capsule_path, *capsule);
          const auto& row = capsule->trace;
          *marginalization_failure_summary
              << "marginalization_failure_capsule_valid=1\n"
              << "marginalization_enforcement_index="
              << row.marginalization_enforcement_index << '\n'
              << "attempt_index_within_enforcement="
              << row.attempt_index_within_enforcement << '\n'
              << "successful_oldest_removals_before_failure_in_same_enforcement="
              << (row.attempt_index_within_enforcement - 1) << '\n'
              << "first_bad_stage=" << row.first_bad_stage << '\n'
              << "marginalization_result=" << row.marginalization_result << '\n'
              << "marginalization_backend=" << row.backend << '\n'
              << "square_root_stack_rows=" << row.qr.stack_rows << '\n'
              << "square_root_stack_columns=" << row.qr.columns << '\n'
              << "square_root_marginalized_rank=" << row.qr.marginalized_rank << '\n'
              << "square_root_rank_threshold=" << row.qr.marginalized_threshold << '\n'
              << "square_root_R_diag_min=" << row.qr.r_diagonal_min << '\n'
              << "square_root_R_diag_max=" << row.qr.r_diagonal_max << '\n'
              << "square_root_rows_before_compression=" << row.qr.rows_before_compression << '\n'
              << "square_root_rows_after_compression=" << row.qr.rows_after_compression << '\n'
              << "square_root_status=" << row.qr.status << '\n'
              << "state_stamps_before_enforcement="
              << stampList(capsule->state_stamps_before_enforcement) << '\n'
              << "state_stamps_at_failing_attempt="
              << stampList(capsule->state_stamps_at_attempt) << '\n'
              << "state_stamps_after_failure="
              << stampList(capsule->state_stamps_at_failure) << '\n'
              << "prior_hash_before_enforcement_fnv1a64="
              << capsule->prior_hash_before_enforcement_fnv1a64 << '\n'
              << "prior_hash_after_failure_fnv1a64="
              << capsule->prior_hash_at_failure_fnv1a64 << '\n'
              << "factor_counts_before_enforcement="
              << capsule->factors_before_enforcement.imu << ';'
              << capsule->factors_before_enforcement.lidar << ';'
              << capsule->factors_before_enforcement.visual << '\n'
              << "factor_counts_after_failure="
              << capsule->factors_at_failure.imu << ';'
              << capsule->factors_at_failure.lidar << ';'
              << capsule->factors_at_failure.visual << '\n'
              << "partial_marginalization_commit_on_enforcement_failure="
              << (partial_commit ? "YES" : "NO") << '\n'
              << "matrix_capsule_binary_path="
              << marginalization_failure_capsule_path << '\n'
              << "matrix_capsule_binary_written=" << binary_written << '\n'
              << "hessian_dimension=" << capsule->consumed_hessian.rows() << '\n'
              << "marginalized_dimension=15\n"
              << "retained_dimension="
              << std::max<Eigen::Index>(0, capsule->consumed_hessian.rows()-15) << '\n'
              << "solve_jitter=" << row.solve_jitter << '\n'
              << "hmm_lambda_min=" << row.hmm.lambda_min << '\n'
              << "hmm_lambda_max=" << row.hmm.lambda_max << '\n'
              << "production_correction_h_shape="
              << capsule->correction_h.rows() << 'x'
              << capsule->correction_h.cols() << '\n'
              << "hessian_dtype=float64_row_major\n"
              << "matrix_capsule_format=P6A3CR1CAPSULE_v1\n";
        }
        marginalization_failure_summary->flush();
      }
      if (optimizer_failure_summary) {
        const std::uint64_t transaction_id=
            (event.type==ProducerEventType::LIDAR_SCAN ||
             event.type==ProducerEventType::LIDAR_SCAN_END)
                ? assets[event.source_index].transaction_id : 0;
        const auto failed_summary=adapter.summary();
        const auto* debug_window=adapter.debugWindowForDiagnostics();
        ObjectiveBreakdown breakdown;
        std::vector<OptimizerIterationTrace> trace_rows;
        std::vector<DirectionalDerivativeTrace> derivative_rows;
        std::vector<DampingSweepTrace> damping_rows;
        double max_state_difference=std::numeric_limits<double>::quiet_NaN();
        bool diagnosis_ok=false;
        std::string diagnosis_reason="diagnostic_window_unavailable";
        if (debug_window) {
          trace_rows=debug_window->optimizerTraceForDebug();
          if (!soak_health_output) diagnosis_ok=debug_window->diagnoseOptimizerFailureForDebug(
              {1e-8,1e-7,1e-6,1e-5,1e-4},
              {1e-6,1e-5,1e-4,1e-3,1e-2,1e-1,1.0,10.0,100.0,
               1e3,1e4,1e5,1e6},
              &breakdown,&derivative_rows,&damping_rows,
              &max_state_difference,&diagnosis_reason);
        }
        flush_optimizer_trace_rows(transaction_id, event.stamp_ns, trace_rows);
        if (directional_derivative_output) {
          for (const auto& row : derivative_rows)
            *directional_derivative_output<<transaction_id<<','<<event.stamp_ns<<','
              <<row.direction_name<<','<<row.epsilon<<','
              <<row.diagnostic_relinearized_basis_fd<<','
              <<row.diagnostic_relinearized_basis_model<<','
              <<row.diagnostic_relinearized_basis_relative_error<<','
              <<row.frozen_basis_fd<<','<<row.frozen_basis_model<<','
              <<row.frozen_basis_relative_error<<','<<row.valid<<','<<row.status<<','
              <<matrixField(Eigen::MatrixXd(row.direction))<<'\n';
          directional_derivative_output->flush();
        }
        if (damping_sweep_output) {
          for (const auto& row : damping_rows)
            *damping_sweep_output<<transaction_id<<','<<event.stamp_ns<<','
              <<row.damping<<','<<row.solved<<','<<row.solver_status<<','
              <<row.raw_step_norm<<','<<row.applied_step_norm<<','<<row.step_clipped<<','
              <<row.g_dot_step<<','<<row.step_H_step<<','<<row.predicted_reduction<<','
              <<row.diagnostic_relinearized_basis_candidate_cost<<','
              <<row.diagnostic_relinearized_basis_actual_reduction<<','
              <<row.frozen_basis_candidate_cost<<','<<row.frozen_basis_actual_reduction<<','
              <<row.diagnostic_relinearized_basis_rho<<','<<row.rho_valid<<','
              <<row.diagnostic_relinearized_basis_breakdown.prior_cost<<','
              <<row.diagnostic_relinearized_basis_breakdown.imu_cost<<','
              <<row.diagnostic_relinearized_basis_breakdown.lidar_cost<<','
              <<row.diagnostic_relinearized_basis_breakdown.visual_cost<<','
              <<row.diagnostic_relinearized_basis_breakdown.latest_lidar_factor_cost<<','
              <<row.frozen_basis_breakdown.prior_cost<<','<<row.frozen_basis_breakdown.imu_cost<<','
              <<row.frozen_basis_breakdown.lidar_cost<<','<<row.frozen_basis_breakdown.visual_cost<<','
              <<row.frozen_basis_breakdown.latest_lidar_factor_cost<<'\n';
          damping_sweep_output->flush();
        }
        *optimizer_failure_summary<<std::setprecision(17)
          <<"transaction_id="<<transaction_id<<"\n"
          <<"event="<<toString(event.type)<<"\n"
          <<"stamp_ns="<<event.stamp_ns<<"\n"
          <<"failure=producer_optimizer:"<<reason<<"\n"
          <<"optimizer_status="<<failed_summary.optimizer_status<<"\n"
          <<"optimizer_iterations="<<failed_summary.optimizer_iterations<<"\n"
          <<"optimizer_initial_cost="<<failed_summary.optimizer_initial_cost<<"\n"
          <<"optimizer_final_cost="<<failed_summary.optimizer_final_cost<<"\n"
          <<"trace_rows="<<trace_rows.size()<<"\n"
          <<"diagnosis_ok="<<diagnosis_ok<<"\n"
          <<"diagnosis_status="<<diagnosis_reason<<"\n"
          <<"start_prior_cost="<<breakdown.prior_cost<<"\n"
          <<"start_imu_cost="<<breakdown.imu_cost<<"\n"
          <<"start_lidar_cost="<<breakdown.lidar_cost<<"\n"
          <<"start_visual_cost="<<breakdown.visual_cost<<"\n"
          <<"start_total_cost="<<breakdown.total_cost<<"\n"
          <<"latest_lidar_stamp_ns="<<breakdown.latest_lidar_stamp_ns<<"\n"
          <<"latest_lidar_observation_id="<<breakdown.latest_lidar_observation_id<<"\n"
          <<"latest_lidar_factor_cost="<<breakdown.latest_lidar_factor_cost<<"\n"
          <<"max_lidar_stamp_ns="<<breakdown.max_lidar_stamp_ns<<"\n"
          <<"max_lidar_observation_id="<<breakdown.max_lidar_observation_id<<"\n"
          <<"max_single_lidar_factor_cost="<<breakdown.max_single_lidar_factor_cost<<"\n"
          <<"diagnostic_state_local_difference_max="<<max_state_difference<<"\n"
          <<"diagnostic_transaction_test="
          <<((diagnosis_ok&&max_state_difference==0.0)?"PASS":"FAIL")<<"\n";
        optimizer_failure_summary->flush();
      }
      throw std::runtime_error("producer_optimizer:"+reason);
    }
    flushMarginalizationTraceRows(
        (event.type==ProducerEventType::LIDAR_SCAN ||
         event.type==ProducerEventType::LIDAR_SCAN_END)
            ? assets[event.source_index].transaction_id : 0,
        event.stamp_ns);
    if (optimizer_trace_output && !soak_health_output) {
      const auto* trace_window = adapter.debugWindowForDiagnostics();
      if (!trace_window)
        throw std::runtime_error("optimizer_trace_window_unavailable");
      const std::uint64_t transaction_id =
          (event.type==ProducerEventType::LIDAR_SCAN ||
           event.type==ProducerEventType::LIDAR_SCAN_END)
              ? assets[event.source_index].transaction_id : 0;
      flush_optimizer_trace_rows(transaction_id, event.stamp_ns,
          trace_window->optimizerTraceForDebug());
    }
    WindowState optimized;
    if(!adapter.latestOptimizedState(&optimized,&reason)) throw std::runtime_error("producer_optimized_state:"+reason);
    if(event.type==ProducerEventType::LIDAR_SCAN || event.type==ProducerEventType::LIDAR_SCAN_END) {
      const auto& asset=assets[event.source_index];
      tracker.finish(asset.stamp_ns,lidar_committed,nominal.pose*T_il.inverse());
      if (tracking_output) {
        *tracking_output<<std::setprecision(17)<<asset.transaction_id<<','<<asset.stamp_ns
            <<",RESULT,"<<p6_tracking::name(tracker.health())<<','<<tracker.consecutiveFailures()
            <<','<<nominal.seed_name<<','<<p6_tracking::effectiveRegistration(nominal.converged,
                 nominal.iterations,nominal.objective,nominal.pose)<<','<<nominal.iterations<<','
            <<nominal.objective<<','<<nominal.fitness<<','<<local_observability.status<<','
            <<local_observability.valid_correspondence_count<<','<<lidar_rank<<','<<nis.valid<<','
            <<nis.nis<<','<<nis.threshold<<','<<lidar_committed<<','<<lidar_committed<<','
            <<recovery_attempts<<','<<tracker.lastReliableStamp()<<','
            <<local_observability.map_support_status<<','
            <<local_observability.rejected_covariance_count<<",0,0,0\n";
        tracking_output->flush();
      }
      const Eigen::Quaterniond q(optimized.rotation);
      trajectory<<asset.transaction_id<<','<<asset.stamp_ns<<','<<asset.time_s<<','
          <<optimized.position.x()<<','<<optimized.position.y()<<','<<optimized.position.z()<<','
          <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
    }
    const auto summary=adapter.summary();
    const Eigen::Quaterniond q(predicted.rotation);
    const double position_sigma=prior.valid ? std::sqrt(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(
        prior.covariance15.block<3,3>(3,3)).eigenvalues().maxCoeff()) : nan;
    const double rotation_sigma=prior.valid ? std::sqrt(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(
        prior.covariance15.block<3,3>(0,0)).eigenvalues().maxCoeff()) : nan;
    diagnostics<<event.stamp_ns<<','<<toString(event.type)<<','<<summary.window_node_count<<','<<summary.window_time_span_s<<','
      <<summary.optimizer_status<<','<<summary.optimizer_initial_cost<<','<<summary.optimizer_final_cost<<','
      <<predicted.position.x()<<','<<predicted.position.y()<<','<<predicted.position.z()<<','
      <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','
      <<nominal.converged<<','<<uobs_valid<<','<<routed.weak_dimension<<','<<routed.reliable_dimension<<','
      <<(lidar_terminal ? (prior.valid ? "1" : "0") : prior.status)<<','
      <<position_sigma<<','<<rotation_sigma<<','<<probed<<','<<unonlocal_status<<','
      <<lidar_attempted<<','<<lidar_committed<<','<<lidar_rank<<','<<nis.nis<<','<<nis.threshold<<','
      <<quality<<','<<visual_mode<<','<<visual_rank<<','<<visual_trigger<<','<<basis_stamp<<','
      <<summary.imu_factor_count<<','<<summary.lidar_factor_count<<','<<summary.visual_factor_count<<','
      <<r2PolicyName(policy)<<','<<buffered_stamp<<','<<lidar_provenance<<','<<visual_provenance<<','
      <<(window_owned?"WINDOW_OWNED_EXPERIMENTAL_INPUT":"COMPATIBILITY_ONLY_NOT_FORMAL_INPUT")<<",0,"
      <<summary.marginalization_backend<<','<<summary.initial_square_root_status<<','
      <<summary.square_root_prior_rows<<','<<summary.square_root_prior_columns<<','
      <<summary.square_root_prior_bytes<<','<<summary.qr_marginalization_ms<<'\n';
    const double event_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-event_start).count();
    runtime<<event.stamp_ns<<','<<toString(event.type)<<','<<calls<<','<<ndt_ms<<','<<event_ms<<','
        <<summary.linearization_ms<<','<<summary.solve_ms<<','
        <<(lidar_terminal ? summary.marginal_covariance_ms : 0.0)<<','
        <<summary.rank_diagnostic_ms<<','<<summary.solver_status<<','<<summary.sparse_solver_fallback_count<<'\n';
    result.events++; result.probes+=probed; result.covariance_available+=prior.valid;
    result.ndt_calls+=calls; result.ndt_ms+=ndt_ms;
    writeSoakHealth(true, "");
    if (soak_health_output) { trajectory.flush(); diagnostics.flush(); runtime.flush(); }
    } catch (const std::exception& error) {
      const bool scan_event = event.type==ProducerEventType::LIDAR_SCAN_START ||
          event.type==ProducerEventType::LIDAR_SCAN_END || event.type==ProducerEventType::LIDAR_SCAN;
      flushMarginalizationTraceRows(scan_event ? assets[event.source_index].transaction_id : 0,event.stamp_ns);
      writeSoakHealth(false,error.what());
      trajectory.flush(); diagnostics.flush(); runtime.flush();
      throw;
    }
  }
  result.total_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  return result;
}

void runFixedLagExperimentalMode(const p4_i2::Inputs& inputs,
    const std::vector<ScanAsset>& assets,const std::string& cloud_path,
    const std::string& map_path,const std::string& params_path,
    const std::string& trajectory_path,const std::string& diagnostics_path,
    const std::string& runtime_path,const std::vector<VisualMeasurement>& visual,
    std::uint64_t init_stamp,const std::string& map_profile,R2Policy policy) {
  if(map_profile=="floor01") {
    p5_i1::requireFrozenMapSha256(map_path);
    for(const auto& asset:assets)
      if(!asset.expected_source_hash_available)
        throw std::runtime_error("fixed_lag_frozen_source_hash_missing");
  }
  else if(map_profile!="corridor01" || p5_i1::sha256File(map_path)!=
      "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f")
    throw std::runtime_error("fixed_lag_map_identity_mismatch");
  const Cloud::Ptr target=loadTarget(map_path);
  if(map_profile=="floor01"&&target->size()!=549606) throw std::runtime_error("target_count_mismatch");
  Pose3d initial,extrinsic;
  const auto parameters=p4_i2::readParameters(params_path,&initial,&extrinsic);
  std::ofstream trajectory(trajectory_path),diagnostics(diagnostics_path),runtime(runtime_path);
  if(!trajectory||!diagnostics||!runtime) throw std::runtime_error("producer_output_open_failed");
  const auto result=runFixedLagProducer(inputs,assets,parameters,initial,extrinsic,target,visual,
      [&](const ScanAsset& a){return preprocessSource(loadRawCloudAt(cloud_path,a));},
      trajectory,diagnostics,runtime,init_stamp,policy,{}, {},nullptr,
      map_profile=="floor01" ? fixed_lag::LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW :
          fixed_lag::LidarCloudProvenance::SENSOR_LOCAL_ROTATION_ONLY);
  std::cout<<"FULL_FIXED_LAG_V2_EXPERIMENTAL_COMPLETE events="<<result.events
      <<" lidar="<<result.lidar_committed<<" visual="<<result.visual_committed
      <<" ndt_calls="<<result.ndt_calls<<" total_ms="<<result.total_ms<<'\n';
}

void runFixedLagProductionFixture(bool window_owned_fixture = false,
                                  bool no_vision_fixture = false,
                                  bool a3g_r3_capture_parity_fixture = false,
                                  bool tracking_recovery_fixture = false) {
  Cloud::Ptr target(new Cloud);
  for(int x=-2;x<=2;++x) for(int y=-2;y<=2;++y) for(int z=-1;z<=1;++z)
    for(int i=0;i<40;++i) {
      Point p;
      p.x=x+0.25f+0.16f*std::sin(1.7*i+0.3*y);
      p.y=y+0.25f+0.17f*std::cos(2.3*i+0.2*z);
      p.z=z+0.25f+0.15f*std::sin(3.1*i+0.4*x);
      target->push_back(p);
    }
  p4_i2::Inputs inputs;
  for(std::uint64_t t=1'000'000'000;t<=1'405'000'000;t+=5'000'000) {
    ImuSample s; s.stamp_ns=t; s.acceleration=Eigen::Vector3d(0,0,9.809);
    inputs.imu.push_back(s);
  }
  std::vector<ScanAsset> assets;
  for(int i=0;i<3;++i) {
    ScanAsset a; a.transaction_id=25+i; a.stamp_ns=1'200'000'000+100'000'000*i;
    a.time_s=0.1*i; assets.push_back(a);
    p4_i2::PoseRecord r; r.transaction_id=a.transaction_id; r.stamp_ns=a.stamp_ns;
    inputs.scans.push_back(r);
  }
  std::vector<VisualMeasurement> visual(no_vision_fixture ? 0 : 2);
  if (!no_vision_fixture) {
    visual[0].ref_ns=1'250'000'000; visual[0].cur_ns=1'300'000'000;
    visual[1].ref_ns=1'300'000'000; visual[1].cur_ns=1'350'000'000;
    for(auto& v:visual) {
      v.depth_stamp_ns=v.ref_ns; v.source_valid=v.quality_metadata_available=true;
      v.detected_count=100; v.tracked_count=80; v.depth_associated_count=70;
      v.inliers=60; v.ratio=0.75; v.depth_fraction=0.7;
      v.grid_occupancy=0.6; v.hull_fraction=0.4;
      v.median_parallax_px=2; v.reprojection=0.5;
    }
  }
  RuntimeParameters parameters; parameters.static_init_samples=20;
  const Cloud::Ptr source=preprocessSource(target);
  WindowOwnedProducerInput owned;
  for(const auto& asset : assets) owned.scan_start_ns.push_back(asset.stamp_ns-50'000'000);
  owned.visual_provenance.assign(visual.size(),fixed_lag::VisualMeasurementProvenance::WINDOW_OWNED_DEPTH);
  std::size_t raw_loads=0,legacy_loads=0;
  owned.provider=[&](const ScanAsset& asset,fixed_lag::RawTimedScan* raw,std::string*) {
    ++raw_loads;
    raw->transaction_id=asset.transaction_id;
    raw->scan_start_ns=asset.stamp_ns-50'000'000; raw->scan_end_ns=asset.stamp_ns;
    // These are explicitly synthetic stationary-world observations, not
    // point-time reconstruction from a real XYZ-only prepared bundle.
    for(std::size_t i=0;i<target->size();++i) {
      TimedLidarPoint point;
      point.position=target->points[i].getVector3fMap().cast<double>();
      point.stamp_ns=raw->scan_start_ns+(i%11)*5'000'000;
      raw->points.push_back(point);
    }
    return true;
  };
  if (tracking_recovery_fixture) {
    using namespace fixed_lag;
    WindowState predicted; predicted.stamp_ns=1'200'000'000;
    FixedLagEventAdapter adapter;
    std::string reason;
    if (!adapter.initialize(predicted,Matrix15d::Identity()*1e6,Vector15d::Zero(),&reason))
      throw std::runtime_error("tracking_fixture_initialize:"+reason);
    WindowMarginalCovariance prior;
    if (!adapter.latestMarginalCovariance(&prior,&reason))
      throw std::runtime_error("tracking_fixture_covariance:"+reason);
    GeometricNdt ndt; configureNdt(ndt,target); ndt.setResolution(0.8);
    ndt.setStepSize(0.08); ndt.setTransformationEpsilon(1e-5); ndt.setMaximumIterations(80);
    Eigen::Matrix4d bad_seed=Eigen::Matrix4d::Identity(); bad_seed(0,3)=100;
    const auto bad=runNdtCandidate(ndt,source,bad_seed,0,"BAD_NOMINAL");
    const auto bad_eval=evaluateLidarCandidate(ndt,source,bad,assets.front(),predicted,
        prior,Pose3d(),parameters,R2Policy::ADAPTIVE_SELECTED_NIS,{},adapter,true);
    if (bad.iterations!=0 || bad_eval.effective || bad_eval.admissible ||
        adapter.summary().lidar_factor_count!=0)
      throw std::runtime_error("tracking_fixture_seed_passthrough_admitted");
    const auto recovered=runNdtCandidate(ndt,source,Eigen::Matrix4d::Identity(),1,"RELIABLE_SEED");
    const auto invalid_probe=trackingRegistrationCapture(bad,true,ndt.getMaximumIterations());
    const auto nominal_capture=trackingRegistrationCapture(recovered,true,ndt.getMaximumIterations());
    const auto invalid_response=reliability::analyzeNonlocalTerminalStability(nominal_capture,
        invalid_probe,invalid_probe,Eigen::Isometry3d::Identity(),2);
    if (invalid_probe.converged || invalid_response.response_valid ||
        invalid_response.status!="NDT_NOT_CONVERGED" ||
        !trackingRegistrationCapture(bad,false,ndt.getMaximumIterations()).converged)
      throw std::runtime_error("tracking_fixture_invalid_probe_became_valid_response");
    const auto good=evaluateLidarCandidate(ndt,source,recovered,assets.front(),predicted,
        prior,Pose3d(),parameters,R2Policy::ADAPTIVE_SELECTED_NIS,{},adapter,true);
    if (!good.admissible || !good.nis.valid || !good.nis.accepted || good.rank<=0 ||
        adapter.summary().lidar_factor_count!=0 || adapter.nextObservationId()!=1)
      throw std::runtime_error("tracking_fixture_recovery_preview_failed");
    auto shifted=recovered; shifted.pose(0,3)+=0.05;
    RuntimeParameters tight=parameters;
    tight.pose_position_sigma_m=0.001; tight.pose_rotation_sigma_rad=0.001;
    const auto rejected=evaluateLidarCandidate(ndt,source,shifted,assets.front(),predicted,
        prior,Pose3d(),tight,R2Policy::BASE_SELECTED_NIS,{},adapter,true);
    if (!rejected.preview_valid || !rejected.nis.valid || rejected.nis.accepted ||
        rejected.admissible || adapter.nextObservationId()!=1)
      throw std::runtime_error("tracking_fixture_recovery_NIS_control:preview="+
          std::to_string(rejected.preview_valid)+" valid="+std::to_string(rejected.nis.valid)+
          " accepted="+std::to_string(rejected.nis.accepted)+" rank="+std::to_string(rejected.rank)+
          " nis="+std::to_string(rejected.nis.nis)+" status="+rejected.local.status);
    if (!adapter.processLidarEvent(good.event,&reason) ||
        adapter.summary().lidar_factor_count!=1 || adapter.nextObservationId()!=2)
      throw std::runtime_error("tracking_fixture_commit_lifecycle:"+reason);
    WindowState displaced=predicted; displaced.position.x()=100;
    FixedLagEventAdapter displaced_adapter;
    if (!displaced_adapter.initialize(displaced,Matrix15d::Identity()*1e6,Vector15d::Zero(),&reason))
      throw std::runtime_error("tracking_fixture_displaced_initialize:"+reason);
    const auto mixed_center=evaluateLidarCandidate(ndt,source,recovered,assets.front(),displaced,
        prior,Pose3d(),parameters,R2Policy::ADAPTIVE_SELECTED_NIS,{},displaced_adapter,true);
    const Eigen::Matrix4d recovery_center=Eigen::Matrix4d::Identity();
    const auto paired_center=evaluateLidarCandidate(ndt,source,recovered,assets.front(),displaced,
        prior,Pose3d(),parameters,R2Policy::ADAPTIVE_SELECTED_NIS,{},displaced_adapter,true,&recovery_center);
    if (mixed_center.stability.response_valid || mixed_center.same_factor_noise_allowed ||
        !paired_center.stability.response_valid || !paired_center.same_factor_noise_allowed ||
        !paired_center.nis.valid || paired_center.nis.accepted || paired_center.admissible ||
        displaced_adapter.summary().lidar_factor_count!=0 || displaced_adapter.nextObservationId()!=1)
      throw std::runtime_error("tracking_fixture_candidate_probe_pairing_or_NIS_failed");
    std::cout<<"A3G_R4_PCL_RECOVERY_ADMISSION_PASS ineffective_nominal=REJECTED "
             <<"recovery_preview=READ_ONLY recovery_NIS=ENFORCED commit_count=1 "
             <<"invalid_probes=REJECTED candidate_probe_center=PAIRED\n";
  }
  if (a3g_r3_capture_parity_fixture || tracking_recovery_fixture) {
    struct CaptureParityRun {
      FixedLagProducerResult result;
      std::string trajectory,diagnostics,runtime,deskew,health;
      std::size_t provider_calls=0;
    };
    const auto execute=[&](A3gR3EvidenceCapture* capture,bool tracking=false) {
      std::ostringstream trajectory,diagnostics,runtime,deskew,health;
      const auto calls_before=raw_loads;
      CaptureParityRun run;
      p6_tracking::Config tracking_options; tracking_options.enabled=tracking;
      run.result=runFixedLagProducer(inputs,assets,parameters,Pose3d(),Pose3d(),target,visual,
          [&](const ScanAsset&){++legacy_loads;return source;},trajectory,diagnostics,runtime,
          0,R2Policy::ADAPTIVE_SELECTED_NIS,{}, {},&owned,
          fixed_lag::LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW,&deskew,
          nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,std::string(),
          nullptr,nullptr,&health,capture,tracking_options);
      run.trajectory=trajectory.str(); run.diagnostics=diagnostics.str();
      run.runtime=runtime.str(); run.deskew=deskew.str(); run.health=health.str();
      run.provider_calls=raw_loads-calls_before;
      return run;
    };
    const auto normalize_csv=[](const std::string& input,
                               const std::set<std::string>& excluded) {
      std::istringstream stream(input); std::string line,output;
      if (!std::getline(stream,line)) return output;
      const auto header=split(line,',');
      std::vector<std::size_t> keep;
      for (std::size_t i=0;i<header.size();++i)
        if (!excluded.count(header[i])) keep.push_back(i);
      const auto append=[&](const std::vector<std::string>& row) {
        for (std::size_t i=0;i<keep.size();++i) {
          if (i) output.push_back(',');
          if (keep[i]<row.size()) output+=row[keep[i]];
        }
        output.push_back('\n');
      };
      append(header);
      while (std::getline(stream,line)) append(split(line,','));
      return output;
    };
    const auto off=execute(nullptr);
    if (tracking_recovery_fixture) {
      const auto on=execute(nullptr,true);
      if (off.trajectory!=on.trajectory || off.deskew!=on.deskew ||
          off.result.lidar_committed!=on.result.lidar_committed ||
          off.result.ndt_calls!=on.result.ndt_calls ||
          off.result.probes!=on.result.probes ||
          normalize_csv(off.diagnostics,{"qr_marginalization_ms"})!=
              normalize_csv(on.diagnostics,{"qr_marginalization_ms"}) ||
          normalize_csv(off.health,{"optimizer_and_marginalization_ms"})!=
              normalize_csv(on.health,{"optimizer_and_marginalization_ms"}))
        throw std::runtime_error("tracking_fixture_healthy_path_parity_failed");
      std::cout<<"A3G_R4_HEALTHY_PATH_OFF_ON_PARITY_PASS extra_NDT_calls=0\n";
      return;
    }
    const auto serial=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto root=std::filesystem::temp_directory_path()/
        ("p6_a3g_r3_capture_parity_"+std::to_string(serial));
    if (!std::filesystem::create_directory(root))
      throw std::runtime_error("A3G_R3_FIXTURE_TEMP_DIRECTORY_CREATE_FAILED");
    A3gR3EvidenceCapture capture(root.string(),{25,26,27});
    const auto on=execute(&capture);
    if (off.result.events!=on.result.events || off.result.lidar_committed!=on.result.lidar_committed ||
        off.result.visual_committed!=on.result.visual_committed ||
        off.result.covariance_available!=on.result.covariance_available ||
        off.result.probes!=on.result.probes || off.result.ndt_calls!=on.result.ndt_calls ||
        off.result.window_deskew_count!=on.result.window_deskew_count ||
        off.provider_calls!=3 || on.provider_calls!=3 || legacy_loads!=0 ||
        off.trajectory!=on.trajectory || off.deskew!=on.deskew ||
        normalize_csv(off.diagnostics,{"qr_marginalization_ms"}) !=
            normalize_csv(on.diagnostics,{"qr_marginalization_ms"}) ||
        normalize_csv(off.health,{"optimizer_and_marginalization_ms"}) !=
            normalize_csv(on.health,{"optimizer_and_marginalization_ms"}) ||
        normalize_csv(off.runtime,{"ndt_ms","event_ms","linearization_ms","solve_ms",
             "marginal_covariance_ms","rank_diagnostic_ms"}) !=
            normalize_csv(on.runtime,{"ndt_ms","event_ms","linearization_ms","solve_ms",
             "marginal_covariance_ms","rank_diagnostic_ms"}) ||
        capture.completedSelected()!=3)
      throw std::runtime_error("A3G_R3_CAPTURE_OFF_ON_PRODUCER_PARITY_FAILED");
    for (const auto tx : {25ULL,26ULL,27ULL}) {
      const auto dir=root/("TX00"+std::to_string(tx));
      for (const char* name : {"RAW_TIMED_POINTS.bin","RAW_TIMED_POINTS_SCHEMA.json",
             "DESKEWED_END_FRAME.pcd","NDT_SOURCE.pcd","DESKEW_KNOTS.csv","METADATA.json"})
        if (!std::filesystem::is_regular_file(dir/name))
          throw std::runtime_error("A3G_R3_CAPTURE_FIXTURE_ARTIFACT_MISSING");
    }
    std::filesystem::remove_all(root);
    std::cout << "A3G_R3_CAPTURE_OFF_ON_PARITY_PASS states=trajectory+health "
              << "factors=revisions+health callbacks=raw_provider_count "
              << "deskew=evidence NDT_source=immutable_const_input optimizer=trajectory\n";
    return;
  }
  for(auto policy:{R2Policy::LEGACY_BASE_NO_GATE,R2Policy::ADAPTIVE_NO_GATE,
                   R2Policy::BASE_SELECTED_NIS,R2Policy::ADAPTIVE_SELECTED_NIS}) {
    std::ostringstream trajectory,diagnostics,runtime;
    const auto result=runFixedLagProducer(inputs,assets,parameters,Pose3d(),Pose3d(),target,visual,
        [&](const ScanAsset&){++legacy_loads;return source;},trajectory,diagnostics,runtime,0,policy,{}, {},
        window_owned_fixture?&owned:nullptr);
    const std::size_t expected_events=window_owned_fixture ? (no_vision_fixture ? 6 : 10) : 7;
    if(result.events!=expected_events || result.covariance_available!=3 || result.probes==0 ||
        result.lidar_committed!=3 || (no_vision_fixture && result.visual_committed!=0) ||
        result.ndt_calls!=3+2*result.probes)
      throw std::runtime_error("production_fixture_did_not_exercise_required_paths");
    std::istringstream rows(diagnostics.str());
    std::string row; std::getline(rows,row);
    const auto columns=std::count(row.begin(),row.end(),',');
    while(std::getline(rows,row)) {
      if(std::count(row.begin(),row.end(),',')!=columns || row.substr(row.size()-2)!=",0")
        throw std::runtime_error("production_diagnostic_shape_or_handoff_failed");
      std::istringstream values(row);
      std::vector<std::string> fields;
      std::string field;
      while(std::getline(values,field,',')) fields.push_back(field);
      if(no_vision_fixture && (fields[1]=="VISUAL_REFERENCE" ||
          fields[1]=="VISUAL_CURRENT" || fields[35]!="0"))
        throw std::runtime_error("A3B_NONE_CREATED_VISUAL_EVENT_OR_FACTOR");
      const auto stamp=std::stoull(fields[0]);
      const auto right=std::lower_bound(inputs.imu.begin(),inputs.imu.end(),stamp,
          [](const ImuSample& s,std::uint64_t t){return s.stamp_ns<t;});
      if(right==inputs.imu.end() || std::stoull(fields[37])!=right->stamp_ns)
        throw std::runtime_error("producer_imu_future_prefetch");
      if(fields[36]!=r2PolicyName(policy)) throw std::runtime_error("producer_policy_mismatch");
      if(fields[1]=="LIDAR_SCAN" || fields[1]=="LIDAR_SCAN_END") {
        const bool has_nis=std::isfinite(std::stod(fields[26]));
        if(has_nis!=r2PolicyUsesNisGate(policy) || fields[15]!="1" || fields[24]!="1")
          throw std::runtime_error("producer_r2_nis_or_geometric_admission_mismatch");
        if(window_owned_fixture && fields[38]!="WINDOW_OWNED_SE3_DESKEW")
          throw std::runtime_error("V3 did not deskew raw source");
      } else if(fields[18]!="NOT_REQUESTED_NON_LIDAR_EVENT" ||
          std::isfinite(std::stod(fields[19])) || std::isfinite(std::stod(fields[20]))) {
        throw std::runtime_error("non_lidar_event_requested_covariance");
      }
    }
    std::cout<<"A2C_REAL_PCL_PRODUCER_FIXTURE_PASS policy="<<r2PolicyName(policy)
        <<" events="<<result.events<<" lidar="<<result.lidar_committed
        <<" visual="<<result.visual_committed<<" probes="<<result.probes
        <<" ndt_calls="<<result.ndt_calls<<" ndt_ms="<<result.ndt_ms
        <<" total_ms="<<result.total_ms<<" post_handoff_ikfom_calls=0\n";
  }
  if(window_owned_fixture) {
    if(raw_loads!=12 || legacy_loads!=0)
      throw std::runtime_error("V3 legacy source provider was invoked");
    if(no_vision_fixture) {
      std::cout<<"A3B_NO_VISION_PRODUCER_FIXTURE_PASS policies=4 events_per_policy=6 "
          <<"visual_events=0 visual_factors=0 lidar_per_policy=3\n";
      return;
    }
    const auto good_provider=owned.provider;
    auto warmup_inputs=inputs; auto warmup_assets=assets; auto warmup_owned=owned;
    ScanAsset warmup; warmup.transaction_id=24; warmup.stamp_ns=1'050'000'000;
    warmup_assets.insert(warmup_assets.begin(),warmup);
    p4_i2::PoseRecord warmup_record; warmup_record.transaction_id=24; warmup_record.stamp_ns=warmup.stamp_ns;
    warmup_inputs.scans.insert(warmup_inputs.scans.begin(),warmup_record);
    warmup_owned.scan_start_ns.insert(warmup_owned.scan_start_ns.begin(),1'010'000'000);
    std::ostringstream warmup_trajectory,warmup_diagnostics,warmup_runtime;
    const auto loads_before=raw_loads;
    const auto warmup_result=runFixedLagProducer(warmup_inputs,warmup_assets,parameters,Pose3d(),Pose3d(),target,visual,
        [&](const ScanAsset&){++legacy_loads;return source;},warmup_trajectory,warmup_diagnostics,warmup_runtime,
        0,R2Policy::LEGACY_BASE_NO_GATE,{}, {},&warmup_owned);
    if(warmup_result.raw_scans_before_handoff!=1 || warmup_result.events!=10 ||
        raw_loads-loads_before!=3 || warmup_result.covariance_available!=3 ||
        warmup_result.lidar_committed!=3 || warmup_trajectory.str().find("1050000000")!=std::string::npos)
      throw std::runtime_error("V3_raw_warmup_scan_was_snapped_or_consumed");
    owned.provider=[&](const ScanAsset& a,fixed_lag::RawTimedScan* raw,std::string* reason) {
      good_provider(a,raw,reason);
      raw->provenance=fixed_lag::LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW;
      return true;
    };
    std::ostringstream trajectory,diagnostics,runtime;
    bool rejected=false;
    try {
      runFixedLagProducer(inputs,assets,parameters,Pose3d(),Pose3d(),target,visual,
          [&](const ScanAsset&){++legacy_loads;return source;},trajectory,diagnostics,runtime,
          0,R2Policy::LEGACY_BASE_NO_GATE,{}, {},&owned);
    } catch(const std::runtime_error& e) {
      rejected=std::string(e.what()).find("NOT_ELIGIBLE_FOR_WINDOW_OWNED_DESKEW")!=std::string::npos;
    }
    std::istringstream runtime_rows(runtime.str()); std::string runtime_row;
    std::getline(runtime_rows,runtime_row);
    std::uint64_t rejected_input_calls=0;
    while(std::getline(runtime_rows,runtime_row)) rejected_input_calls+=std::stoull(split(runtime_row,',')[2]);
    if(!rejected || legacy_loads!=0 || rejected_input_calls!=0)
      throw std::runtime_error("V3 legacy LiDAR gate failed");
    owned.provider=good_provider;
    owned.visual_provenance.assign(visual.size(),fixed_lag::VisualMeasurementProvenance::LEGACY_STATE_DERIVED_DEPTH);
    std::ostringstream trajectory2,diagnostics2,runtime2;
    const auto rejected_visual=runFixedLagProducer(inputs,assets,parameters,Pose3d(),Pose3d(),target,visual,
        [&](const ScanAsset&){++legacy_loads;return source;},trajectory2,diagnostics2,runtime2,
        0,R2Policy::LEGACY_BASE_NO_GATE,{}, {},&owned);
    if(rejected_visual.visual_committed!=0 || diagnostics2.str().find("VISUAL_PROVENANCE_REJECTED_NOT_FORMAL_INPUT")==std::string::npos)
      throw std::runtime_error("V3 legacy visual gate failed");
    std::cout<<"A2D_V3_RAW_DESKEW_AND_LEGACY_PROVENANCE_REJECTION_PASS\n";
  }
}
