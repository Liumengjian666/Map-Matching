#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
namespace p=dog_prior_map_fastlio2_frontend_exp;
namespace {
void require(bool ok,const char* text) { if(!ok) throw std::runtime_error(text); }
Eigen::Matrix4f pose(float x) { auto T=Eigen::Matrix4f::Identity().eval();T(0,3)=x;return T; }
}
int main() {
  try {
    p::CoupledAnchorState anchor;
    p::settleCoupledAnchor(&anchor,1000000000,pose(.5f).cast<double>(),true,false);
    require(anchor.valid && anchor.status=="ESTABLISHED","stable seed after corrected update");
    p::settleCoupledAnchor(&anchor,1100000000,pose(10).cast<double>(),true,false);
    require(anchor.prediction(0,3)==.5,"valid anchor never absorbs latest NDT");
    p::advanceCoupledAnchor(&anchor,1100000000,pose(.1f).cast<double>());
    require(std::abs(anchor.prediction(0,3)-.6)<1e-7,"causal propagation adds inertial interval only");
    p::advanceCoupledAnchor(&anchor,1100000000,Eigen::Matrix4d::Identity());
    require(!anchor.valid,"duplicate propagation invalidates");
    p::settleCoupledAnchor(&anchor,1000000000,pose(.5f).cast<double>(),true,false);
    int jets=0,scores=0,aligns=0;
    bool follow_initial=false;
    p::CoupledNdtBackend backend;
    backend.jet=[&](const Eigen::Matrix4f&,const p::CoupledVector6&) {
      ++jets;p::CoupledNativeJet j;j.valid=true;j.score_sum=100;
      j.score_hessian.diagonal()<<-.1,-2,-3,-4,-5,-6;return j;};
    backend.score=[&](const Eigen::Matrix4f&){++scores;return 100.;};
    backend.refine=[&](const Eigen::Matrix4f& initial){++aligns;p::CoupledRefinement r;
      r.pose=follow_initial?initial:pose(.5f);r.successful=r.converged=true;r.iterations=5;return r;};
    p::PendingCandidate pending;p::CoupledAnchorReceipt receipt;p::CoupledEventConfig config;
    config.branch_admission=true;
    auto run=[&](float x,uint64_t stamp) {return p::runAnchoredCoupledNdtShadow(pose(x),pose(x+.2f),
      stamp,1,true,backend,config,&pending,pose(.1f).cast<double>(),&anchor,&receipt);};
    auto r=run(0,1100000000);
    require(r.event=="PENDING_CREATED" && anchor.frozen && receipt.evaluated && receipt.contributions==1,
        "creation freezes valid W and anchor");
    const auto W=anchor.weak_basis;follow_initial=true;
    r=run(.1f,1200000000);
    require(r.event=="PENDING_FIRST_SUPPORT" && !r.admitted && anchor.weak_basis==W,"fixed basis no early admission");
    r=run(.2f,1300000000);
    require(r.temporally_supported && r.admitted && receipt.contributions==3 && !pending.active &&
      receipt.nominal_sum>receipt.alternative_sum && r.branch_difference>=-1e-6,
      "anchor advantage admits without R4 score improvement and snapshots before clear");
    p::settleCoupledAnchor(&anchor,1300000000,pose(.7f).cast<double>(),false,true);
    require(!anchor.valid && !anchor.frozen,"actual feedback consumes anchor");
    aligns=scores=jets=0;
    r=run(.3f,1400000000);
    require(!r.admitted && r.mode=="ANCHOR_SKIP" && !aligns && !scores && !jets,"missing anchor retains nominal zero work");
    // Spatial map-product chart: never use body-frame inverse(reference)*candidate.
    Eigen::Matrix4d ref=Eigen::Matrix4d::Identity(),cur=ref;
    ref.block<3,3>(0,0)=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitY()).toRotationMatrix();
    cur.block<3,3>(0,0)=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitX()).toRotationMatrix()*ref.block<3,3>(0,0);
    cur(0,3)=.8;p::CoupledVector6 eta;
    require(p::coupledAnchorChart(cur,ref,&eta) && std::abs(eta(0)-1)<1e-12 &&
      std::abs(eta(3)-.2)<1e-12 && eta.tail<2>().norm()<1e-12,"spatial rotation and scaled map translation");
    cur(3,3)=2;require(!p::coupledAnchorChart(cur,ref,&eta),"invalid rigid chart rejected");
    // Even a frozen episode cannot extend the absolute anchor lifetime.
    anchor={};p::settleCoupledAnchor(&anchor,1000000000,pose(0).cast<double>(),true,false);
    for(int i=1;i<=20;++i) p::advanceCoupledAnchor(&anchor,1000000000+i*100000000ULL,Eigen::Matrix4d::Identity());
    require(anchor.valid,"2 second boundary inclusive");anchor.frozen=true;
    p::advanceCoupledAnchor(&anchor,3100000000,Eigen::Matrix4d::Identity());
    require(!anchor.valid && !anchor.frozen,"expired pending cannot renew anchor");
    p::settleCoupledAnchor(&anchor,3100000000,pose(0).cast<double>(),true,false);
    require(!anchor.valid,"no same-frame expiry reseed even on normal frame");
    p::settleCoupledAnchor(&anchor,3200000000,pose(0).cast<double>(),true,false);
    require(anchor.valid,"later stable frame may renew");
    // Exact/near-numerical equality cannot pass the meaningful cost margin.
    anchor={};p::settleCoupledAnchor(&anchor,1000000000,pose(.15f).cast<double>(),true,false);
    pending={};follow_initial=false;r=run(0,1100000000);follow_initial=true;
    run(.1f,1200000000);r=run(.2f,1300000000);
    require(r.temporally_supported && !r.admitted,"equal anchor costs retain nominal");
    // R4 score/motion arithmetic is diagnostic, not an implicit R5 veto.
    anchor={};p::settleCoupledAnchor(&anchor,1000000000,pose(.5f).cast<double>(),true,false);
    pending={};follow_initial=false;run(0,1100000000);follow_initial=true;
    pending.motion_difference=std::numeric_limits<double>::quiet_NaN();
    r=run(.1f,1200000000);require(pending.active && !receipt.diagnostic_valid,"unavailable R4 diagnostic does not abort anchor");
    r=run(.2f,1300000000);require(r.admitted && !receipt.diagnostic_valid,"anchor admission independent of R4 diagnostic validity");
    anchor={};p::settleCoupledAnchor(&anchor,1000000000,pose(.5f).cast<double>(),true,false);
    pending={};follow_initial=false;run(0,1100000000);follow_initial=true;
    pending.admission_valid=false;  // Simulate an omitted diagnostic interval, with finite sums.
    run(.1f,1200000000);r=run(.2f,1300000000);
    require(r.admitted && !receipt.diagnostic_valid,"diagnostic validity cannot recover after missing interval");
    // Two already-refined eligible terminals: IMU absolute merit prefers .5,
    // but the inertial anchor at .6 selects .6 without any new alignment.
    anchor={};p::settleCoupledAnchor(&anchor,1000000000,pose(.5f).cast<double>(),true,false);
    pending={};int rank_aligns=0;
    backend.refine=[&](const Eigen::Matrix4f&){p::CoupledRefinement t;
      t.pose=pose(++rank_aligns==1?.5f:.6f);t.successful=t.converged=true;t.iterations=5;return t;};
    r=run(0,1100000000);
    require(rank_aligns==2 && r.candidate_pose(0,3)>.59f && receipt.alternative_cost<1e-12,
        "anchor-aware terminal rank reuses same two calls, not old absolute prediction merit");
    std::cout<<"anchor lifecycle/chart/admission tests PASS\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
