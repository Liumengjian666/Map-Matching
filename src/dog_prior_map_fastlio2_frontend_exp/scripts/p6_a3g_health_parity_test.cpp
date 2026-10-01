// Reuse the existing deterministic mathematical fixture, without changing it.
#define main a3e_r2_existing_test_main
#include "p6_a3e_r2_square_root_test.cpp"
#undef main
int main() {
  try {
    auto off_calls=std::make_shared<int>(0),on_calls=std::make_shared<int>(0);
    auto off=graph(false,off_calls),on=graph(false,on_calls,true);
    std::string a,b;
    require(off.optimize(&a)==on.optimize(&b) && a==b,"optimizer parity");
    require(off.marginalizeIfNeeded(&a)==on.marginalizeIfNeeded(&b) && a==b,"marginal parity");
    require((off.priorInformation()-on.priorInformation()).norm()==0 &&
        (off.priorGradient()-on.priorGradient()).norm()==0,"exact cache parity");
    require((off.priorSquareRootRows().a-on.priorSquareRootRows().a).norm()==0 &&
        (off.priorSquareRootRows().b-on.priorSquareRootRows().b).norm()==0,"exact A/b parity");
    require(off.states().size()==on.states().size(),"stamp count parity");
    for(std::size_t i=0;i<off.states().size();++i)
      require(off.states()[i].stamp_ns==on.states()[i].stamp_ns &&
          localDifference(off.states()[i],on.states()[i]).norm()==0,"state parity");
    const auto os=off.summary(),ns=on.summary();
    require(os.optimizer_status==ns.optimizer_status &&
        os.marginalization_status==ns.marginalization_status &&
        os.imu_factor_count==ns.imu_factor_count && os.lidar_factor_count==ns.lidar_factor_count &&
        os.visual_factor_count==ns.visual_factor_count && os.window_revision==ns.window_revision &&
        os.active_observation_id_count==ns.active_observation_id_count,"lifecycle parity");
    require(*off_calls==*on_calls,"callback parity");
    const auto& trace=on.marginalizationTraceForDiagnostics();
    require(trace.size()==1 && trace[0].qr.marginalized_rank==15 &&
        trace[0].marginalization_result=="SUCCESS" &&
        trace[0].legacy_shadow_status=="NOT_REQUESTED" && trace[0].attempt_index_within_enforcement==1,
        "light trace without shadow");
    const auto failureGraph=[](bool health) {
      FixedLagOptions o; o.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
      o.maximum_nodes=1; o.maximum_duration_s=100; o.capture_marginalization_health=health;
      ImuNoiseParameters noise; noise.gravity.setZero(); FixedLagWindow w(o,noise); std::string r;
      require(w.initializeWithPriorAtomic(stateAt(0),Matrix15d::Identity(),Vector15d::Zero(),&r),"fail init");
      require(w.addStateWithImuFactorAtomic(stateAt(1),1,stateAt(0).stamp_ns,imuBetween(stateAt(0),stateAt(1)),&r),"fail IMU");
      require(w.addState(stateAt(2),&r)&&w.addState(stateAt(3),&r),"isolated synthetic nodes");
      return w;
    };
    auto fo=failureGraph(false),fh=failureGraph(true);
    require(!fo.marginalizeIfNeeded(&a)&&!fh.marginalizeIfNeeded(&b)&&a==b,"failure parity");
    require(fo.states().size()==fh.states().size() &&
        (fo.priorInformation()-fh.priorInformation()).norm()==0 &&
        fo.summary().active_observation_id_count==fh.summary().active_observation_id_count,"partial lifecycle parity");
    const auto& capsule=fh.marginalizationFailureCapsuleForDiagnostics();
    require(capsule.valid && capsule.trace.attempt_index_within_enforcement==3 &&
        capsule.state_stamps_before_enforcement.size()==4 && capsule.state_stamps_at_failure.size()==2,
        "light failure capsule and partial attempt identity");
    std::cout<<"A3G_LIGHT_HEALTH_OFF_ON_PARITY_PASS exact_states_prior_lifecycle=YES shadow=OFF\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
