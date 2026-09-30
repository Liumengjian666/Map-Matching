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
    const fixed_lag::FixedLagOptions& options = {}) {
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
  FixedLagEventAdapter adapter(options,makeWindowImuNoise(parameters,seed.gravity),calibration);
  std::string reason;
  if(!adapter.initialize(initial,seed.information15,Vector15d::Zero(),&reason))
    throw std::runtime_error("fixed_lag_prior:"+reason);
  // No IKFoM object exists below this boundary. The adapter is the state owner.
  std::vector<ProducerEvent> stream;
  for(std::size_t i=0;i<assets.size();++i) {
    if(i>=inputs.scans.size() || assets[i].stamp_ns!=inputs.scans[i].stamp_ns ||
        assets[i].transaction_id!=inputs.scans[i].transaction_id ||
        assets[i].stamp_ns<seed.stamp_ns)
      throw std::runtime_error("fixed_lag_scan_identity_or_time_mismatch");
    stream.push_back({assets[i].stamp_ns,ProducerEventType::LIDAR_SCAN,i});
  }
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
  trajectory<<std::setprecision(17)<<"transaction_id,stamp_ns,time_s,px,py,pz,qx,qy,qz,qw\n";
  diagnostics<<std::setprecision(17)<<"timestamp,event_type,window_nodes,window_span,optimizer_status,optimizer_cost_before,optimizer_cost_after,predicted_px,predicted_py,predicted_pz,predicted_qx,predicted_qy,predicted_qz,predicted_qw,ndt_converged,uobs_valid,weak_dimension,reliable_dimension,window_covariance_valid,window_position_sigma_max,window_rotation_sigma_max,unonlocal_probe_triggered,unonlocal_status,lidar_factor_attempted,lidar_factor_committed,lidar_selected_rank,lidar_nis,lidar_nis_threshold,visual_sensor_quality,visual_mode,visual_selected_rank,visual_trigger_status,visual_basis_source_lidar_stamp,imu_factor_count,lidar_factor_count,visual_factor_count,r2_policy,imu_buffer_last_stamp,post_handoff_ikfom_calls\n";
  runtime<<"timestamp,event_type,ndt_calls,ndt_ms,event_ms\n";
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
    adapter.latestMarginalCovariance(&prior,nullptr);
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
    if(event.type==ProducerEventType::LIDAR_SCAN) {
      const auto& asset=assets[event.source_index];
      const Cloud::Ptr source=source_provider(asset);
      if(!source || source->empty()) throw std::runtime_error("empty_preprocessed_source");
      if(asset.expected_source_hash_available && sourceCloudHash(source)!=asset.expected_source_hash)
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
    } else if(event.type==ProducerEventType::VISUAL_REFERENCE) {
      if(!adapter.processVisualReferenceStamp(event.stamp_ns,&reason))
        throw std::runtime_error("producer_visual_reference:"+reason);
      visual_mode="REFERENCE_STATE"; visual_trigger=reason;
    } else {
      const auto v=frozenVisual(visual[event.source_index],0.05);
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
    if(event.type==ProducerEventType::LIDAR_SCAN) {
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
      <<prior.valid<<','<<position_sigma<<','<<rotation_sigma<<','<<probed<<','<<unonlocal_status<<','
      <<lidar_attempted<<','<<lidar_committed<<','<<lidar_rank<<','<<nis.nis<<','<<nis.threshold<<','
      <<quality<<','<<visual_mode<<','<<visual_rank<<','<<visual_trigger<<','<<basis_stamp<<','
      <<summary.imu_factor_count<<','<<summary.lidar_factor_count<<','<<summary.visual_factor_count<<','
      <<r2PolicyName(policy)<<','<<buffered_stamp<<",0\n";
    const double event_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-event_start).count();
    runtime<<event.stamp_ns<<','<<toString(event.type)<<','<<calls<<','<<ndt_ms<<','<<event_ms<<'\n';
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
      trajectory,diagnostics,runtime,init_stamp,policy);
  std::cout<<"FULL_FIXED_LAG_V2_EXPERIMENTAL_COMPLETE events="<<result.events
      <<" lidar="<<result.lidar_committed<<" visual="<<result.visual_committed
      <<" ndt_calls="<<result.ndt_calls<<" total_ms="<<result.total_ms<<'\n';
}

void runFixedLagProductionFixture() {
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
  std::vector<VisualMeasurement> visual(2);
  visual[0].ref_ns=1'250'000'000; visual[0].cur_ns=1'300'000'000;
  visual[1].ref_ns=1'300'000'000; visual[1].cur_ns=1'350'000'000;
  for(auto& v:visual) {
    v.depth_stamp_ns=v.ref_ns; v.source_valid=v.quality_metadata_available=true;
    v.detected_count=100; v.tracked_count=80; v.depth_associated_count=70;
    v.inliers=60; v.ratio=0.75; v.depth_fraction=0.7;
    v.grid_occupancy=0.6; v.hull_fraction=0.4;
    v.median_parallax_px=2; v.reprojection=0.5;
  }
  RuntimeParameters parameters; parameters.static_init_samples=20;
  const Cloud::Ptr source=preprocessSource(target);
  for(auto policy:{R2Policy::LEGACY_BASE_NO_GATE,R2Policy::ADAPTIVE_NO_GATE,
                   R2Policy::BASE_SELECTED_NIS,R2Policy::ADAPTIVE_SELECTED_NIS}) {
    std::ostringstream trajectory,diagnostics,runtime;
    const auto result=runFixedLagProducer(inputs,assets,parameters,Pose3d(),Pose3d(),target,visual,
        [&](const ScanAsset&){return source;},trajectory,diagnostics,runtime,0,policy);
    if(result.events!=7 || result.covariance_available!=7 || result.probes==0 ||
        result.lidar_committed!=3 || result.ndt_calls!=3+2*result.probes)
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
      const auto stamp=std::stoull(fields[0]);
      const auto right=std::lower_bound(inputs.imu.begin(),inputs.imu.end(),stamp,
          [](const ImuSample& s,std::uint64_t t){return s.stamp_ns<t;});
      if(right==inputs.imu.end() || std::stoull(fields[37])!=right->stamp_ns)
        throw std::runtime_error("producer_imu_future_prefetch");
      if(fields[36]!=r2PolicyName(policy)) throw std::runtime_error("producer_policy_mismatch");
      if(fields[1]=="LIDAR_SCAN") {
        const bool has_nis=std::isfinite(std::stod(fields[26]));
        if(has_nis!=r2PolicyUsesNisGate(policy) || fields[15]!="1" || fields[24]!="1")
          throw std::runtime_error("producer_r2_nis_or_geometric_admission_mismatch");
      }
    }
    std::cout<<"A2C_REAL_PCL_PRODUCER_FIXTURE_PASS policy="<<r2PolicyName(policy)
        <<" events="<<result.events<<" lidar="<<result.lidar_committed
        <<" visual="<<result.visual_committed<<" probes="<<result.probes
        <<" ndt_calls="<<result.ndt_calls<<" ndt_ms="<<result.ndt_ms
        <<" total_ms="<<result.total_ms<<" post_handoff_ikfom_calls=0\n";
  }
}
