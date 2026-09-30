// Included after the mature P6 NDT/input helpers, inside namespace p6_i1.
// This independent producer owns only FixedLagEventAdapter after handoff.

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
    std::ostream* deskew_evidence = nullptr) {
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
  result.raw_scans_before_handoff=raw_scans_before_handoff;
  trajectory<<std::setprecision(17)<<"transaction_id,stamp_ns,time_s,px,py,pz,qx,qy,qz,qw\n";
  diagnostics<<std::setprecision(17)<<"timestamp,event_type,window_nodes,window_span,optimizer_status,optimizer_cost_before,optimizer_cost_after,predicted_px,predicted_py,predicted_pz,predicted_qx,predicted_qy,predicted_qz,predicted_qw,ndt_converged,uobs_valid,weak_dimension,reliable_dimension,window_covariance_valid,window_position_sigma_max,window_rotation_sigma_max,unonlocal_probe_triggered,unonlocal_status,lidar_factor_attempted,lidar_factor_committed,lidar_selected_rank,lidar_nis,lidar_nis_threshold,visual_sensor_quality,visual_mode,visual_selected_rank,visual_trigger_status,visual_basis_source_lidar_stamp,imu_factor_count,lidar_factor_count,visual_factor_count,r2_policy,imu_buffer_last_stamp,lidar_source_provenance,visual_source_provenance,input_eligibility,post_handoff_ikfom_calls\n";
  runtime<<"timestamp,event_type,ndt_calls,ndt_ms,event_ms,linearization_ms,solve_ms,marginal_covariance_ms,rank_diagnostic_ms,solver_status,sparse_fallback_count\n";
  if (deskew_evidence)
    *deskew_evidence<<std::setprecision(17)
        <<"transaction_id,scan_start_ns,scan_end_ns,raw_point_count,point_stamp_min_ns,point_stamp_max_ns,"
        <<"anchor_px,anchor_py,anchor_pz,anchor_qx,anchor_qy,anchor_qz,anchor_qw,"
        <<"predicted_end_px,predicted_end_py,predicted_end_pz,predicted_end_qx,predicted_end_qy,predicted_end_qz,predicted_end_qw,"
        <<"deskew_point_count,displacement_mean_m,displacement_p95_m,displacement_max_m,status\n";
  for(const ProducerEvent& event:stream) {
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
    const auto covariance_diagnostic = adapter.summary();
    if (covariance_diagnostic.dense_marginal_reference_requests != 0 ||
        covariance_diagnostic.marginal_covariance_requests !=
            pre_measurement.marginal_covariance_requests + (lidar_terminal ? 1 : 0) ||
        covariance_diagnostic.lidar_factor_count != pre_measurement.lidar_factor_count)
      throw std::runtime_error("producer_pre_measurement_covariance_contract");
    Candidate nominal;
    reliability::LocalRisk routed;
    reliability::NonlocalTerminalStability stability;
    stability.status="NOT_PROBED";
    std::string unonlocal_status="NOT_PROBED";
    SelectedLidarNis nis;
    bool lidar_attempted=false,lidar_committed=false,probed=false,quality=false,uobs_valid=false;
    std::uint64_t calls=0;
    double ndt_ms=0;
    int lidar_rank=0,visual_rank=0;
    std::uint64_t basis_stamp=0;
    std::string visual_mode="NOT_APPLICABLE",visual_trigger="NOT_APPLICABLE";
    std::string lidar_provenance="NOT_APPLICABLE",visual_provenance="NOT_APPLICABLE";
    if(event.type==ProducerEventType::LIDAR_SCAN || event.type==ProducerEventType::LIDAR_SCAN_END) {
      const auto& asset=assets[event.source_index];
      Cloud::Ptr source;
      if (window_owned) {
        RawTimedScan raw;
        if (!window_owned->provider(asset,&raw,&reason)) throw std::runtime_error("raw_timed_scan:"+reason);
        if (!rawDeskewInputAllowed(raw.provenance,&reason)) throw std::runtime_error(reason);
        if (raw.transaction_id!=asset.transaction_id || raw.scan_end_ns!=asset.stamp_ns ||
            raw.scan_start_ns!=window_owned->scan_start_ns[event.source_index])
          throw std::runtime_error("raw_timed_scan_identity_mismatch");
        WindowState anchor;
        if (!adapter.activeStateAt(raw.scan_start_ns,&anchor,&reason)) throw std::runtime_error(reason);
        WindowDeskewResult deskew;
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
      nominal=runNdtCandidate(ndt,source,prediction*T_il,0,"WINDOW_M0");
      calls=1; ndt_ms=nominal.runtime_ms;
      const auto observations=nominal.converged ?
          ndt.geometricObservations(*source,nominal.pose.cast<float>(),0.8) :
          std::vector<reliability::GeometricObservation>{};
      const auto local=reliability::analyzeGeometricObservability(observations,nominal.converged,0.8);
      uobs_valid=local.valid;
      const auto risk=reliability::assessLocalRisk(local,config);
      if(prior.valid && nominal.converged) {
        const Pose3d measured_imu=p4_i2::lidarMeasurementToImu(poseFromMatrix(nominal.pose),extrinsic);
        const auto innovation=reliability::mapProductInnovation(
            p4_i2::asIsometry(predicted_pose),p4_i2::asIsometry(measured_imu));
        const auto trigger=reliability::shouldRunNonlocalProbes(
            asset.transaction_id,innovation,prior.map_pose_covariance6,config);
        const auto spectrum=p6_i4::analyzePoseCovariance(prior.map_pose_covariance6);
        if(trigger.run_probes && spectrum.valid && spectrum.effective_rank>0 && spectrum.eigenvalues(5)>0) {
          const Vector6d delta=config.probe_prior_sigma*std::sqrt(spectrum.eigenvalues(5))*spectrum.eigenvectors.col(5);
          const Candidate plus=runNdtCandidate(ndt,source,p6_i4::boxplusMapPose(prediction,delta)*T_il,1,"WINDOW_M_PLUS");
          const Candidate minus=runNdtCandidate(ndt,source,p6_i4::boxplusMapPose(prediction,-delta)*T_il,2,"WINDOW_M_MINUS");
          calls+=2; ndt_ms+=plus.runtime_ms+minus.runtime_ms; probed=true;
          stability=reliability::analyzeNonlocalTerminalStability(
              terminalCapture(nominal),terminalCapture(plus),terminalCapture(minus),p4_i2::asIsometry(extrinsic),2);
          unonlocal_status=stability.status;
        } else if(trigger.run_probes) {
          stability.status="PROBE_PRIOR_COVARIANCE_INVALID";
          unonlocal_status=stability.status;
        } else unonlocal_status=trigger.reason;
      } else if(!prior.valid) unonlocal_status="UNONLOCAL_DISABLED_WINDOW_COVARIANCE_UNAVAILABLE";
      const auto decision=reliability::decideDualReliability(risk,stability,nominal.converged,
          predicted.rotation,parameters.pose_position_sigma_m,parameters.pose_rotation_sigma_rad,true,true,config);
      const bool terminal_risk=stability.response_valid &&
          (std::max(stability.delta_position_imu_positive.norm(),stability.delta_position_imu_negative.norm())>config.terminal_translation_limit_m ||
           std::max(stability.delta_rotation_positive.norm(),stability.delta_rotation_negative.norm())>config.terminal_rotation_limit_rad);
      routed=riskWithNonlocalResponse(risk,stability,decision.nonlocal_risk||terminal_risk,
          -predicted.rotation*extrinsic.position,config);
      FrozenLidarEvent lidar;
      lidar.transaction_id=asset.transaction_id; lidar.stamp_ns=asset.stamp_ns;
      lidar.map_T_lidar=poseFromMatrix(nominal.pose); lidar.local_risk=routed;
      lidar.ndt_converged=nominal.converged;
      lidar.map_support_valid=routed.valid&&routed.map_support_sufficient;
      lidar.residual_covariance=makeBasePoseNoise(parameters);
      if(r2PolicyUsesAdaptiveNoise(policy)) {
        const auto noise=reliability::makePoseMeasurementNoise(parameters.pose_position_sigma_m,
            parameters.pose_rotation_sigma_rad,predicted.rotation,routed,stability,true,true,config);
        lidar.measurement_commit_allowed=noise.valid;
        if(noise.valid) lidar.residual_covariance=noise.covariance;
      }
      LidarWindowMeasurement measurement;
      lidar_attempted=lidar.ndt_converged&&lidar.map_support_valid&&routed.reliable_dimension>0;
      if(lidar_attempted && adapter.previewLidarMeasurement(lidar,&measurement,nullptr)) {
        lidar_rank=measurement.reliable_rank;
        if(r2PolicyUsesNisGate(policy)) {
          nis=evaluateSelectedLidarNis(predicted,measurement,prior,chiSquare99Threshold(lidar_rank));
          lidar.measurement_commit_allowed=lidar.measurement_commit_allowed&&nis.valid&&nis.accepted;
        }
      } else if(r2PolicyUsesNisGate(policy)) lidar.measurement_commit_allowed=false;
      lidar_committed=adapter.processLidarEvent(lidar,&reason);
      if(!lidar_committed && adapter.lastEventStatus().disposition!=AdapterEventDisposition::SKIPPED_INVALID_SOURCE)
        throw std::runtime_error("producer_lidar:"+reason);
      result.lidar_committed+=lidar_committed;
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
    if(!adapter.optimizeCurrentWindow(&reason))
      throw std::runtime_error("producer_optimizer:"+reason);
    WindowState optimized;
    if(!adapter.latestOptimizedState(&optimized,&reason)) throw std::runtime_error("producer_optimized_state:"+reason);
    if(event.type==ProducerEventType::LIDAR_SCAN || event.type==ProducerEventType::LIDAR_SCAN_END) {
      const auto& asset=assets[event.source_index];
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
      <<(window_owned?"WINDOW_OWNED_EXPERIMENTAL_INPUT":"COMPATIBILITY_ONLY_NOT_FORMAL_INPUT")<<",0\n";
    const double event_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-event_start).count();
    runtime<<event.stamp_ns<<','<<toString(event.type)<<','<<calls<<','<<ndt_ms<<','<<event_ms<<','
        <<summary.linearization_ms<<','<<summary.solve_ms<<','
        <<(lidar_terminal ? summary.marginal_covariance_ms : 0.0)<<','
        <<summary.rank_diagnostic_ms<<','<<summary.solver_status<<','<<summary.sparse_solver_fallback_count<<'\n';
    result.events++; result.probes+=probed; result.covariance_available+=prior.valid;
    result.ndt_calls+=calls; result.ndt_ms+=ndt_ms;
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
                                  bool no_vision_fixture = false) {
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
