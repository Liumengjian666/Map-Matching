#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

namespace p = dog_prior_map_fastlio2_frontend_exp;
namespace {
void require(bool condition, const char* label) { if (!condition) throw std::runtime_error(label); }
Eigen::Matrix4f pose(float x) { auto T = Eigen::Matrix4f::Identity().eval(); T(0,3)=x; return T; }
}
int main() {
  try {
    p::CoupledEventConfig config; config.branch_admission=true;
    p::PendingCandidate pending;
    int jets=0, scores=0, aligns=0;
    double score_gain=1;
    bool use_initial=false;
    bool rotated_alternative=false;
    p::CoupledNdtBackend backend;
    backend.jet = [&](const Eigen::Matrix4f&, const p::CoupledVector6&) {
      ++jets; p::CoupledNativeJet j; j.valid=true; j.score_sum=100;
      j.score_hessian.diagonal() << -.1,-2,-3,-4,-5,-6; return j;
    };
    backend.score = [&](const Eigen::Matrix4f& T) {
      ++scores; return T(0,3)>.3f ? 100+score_gain : 100;
    };
    backend.refine = [&](const Eigen::Matrix4f& initial) {
      ++aligns; p::CoupledRefinement r; r.pose=use_initial?initial:pose(.5f);
      if (rotated_alternative && !use_initial)
        r.pose.block<3,3>(0,0)=Eigen::AngleAxisf(.1f,Eigen::Vector3f::UnitX()).toRotationMatrix();
      r.successful=r.converged=true; r.iterations=5; r.status="SUCCESS"; return r;
    };
    const Eigen::Matrix4d increment=pose(.1f).cast<double>();
    auto run = [&](float x, uint64_t stamp, const Eigen::Matrix4d* delta) {
      return p::runEventCoupledNdtShadow(pose(x),pose(x+.2f),stamp,1,true,backend,config,&pending,delta);
    };
    auto advance = [&](float x, uint64_t stamp) { return run(x,stamp,&increment); };
    auto create = [&]() { use_initial=false; pending={}; return advance(0,1000000000); };
    auto r=create();
    require(r.event=="PENDING_CREATED" && pending.active && pending.admission_valid &&
        std::abs(r.energy_difference+.01)<1e-12 && r.origin_stamp_ns==1000000000,
        "creation score same-carrier normalized difference");
    use_initial=true; jets=scores=aligns=0;
    r=advance(.1f,1100000000);
    require(r.event=="PENDING_FIRST_SUPPORT" && !r.admitted && aligns==1 && scores==2 && jets==0 &&
        std::abs(r.energy_difference+.02)<1e-12, "first support cannot be admitted");
    r=advance(.2f,1200000000);
    require(r.temporally_supported && r.admitted && !pending.active && r.confirmation_count==2 &&
        std::abs(r.energy_difference+.03)<1e-12 && r.admitted_pose(0,3)>.6f &&
        r.shadow.recommended_pose==pose(.2f) && r.origin_stamp_ns==1000000000,
        "completed receipt snapshot before clearing; shadow remains nominal");
    jets=scores=aligns=0;
    r=p::runEventCoupledNdtShadow(pose(.2f),pose(.2f),1300000000,1,true,backend,config,&pending,&increment);
    require(!r.admitted && r.mode=="NORMAL" && jets==0 && scores==0 && aligns==0,
        "consumed admission cannot be reinjected; normal zero work");
    score_gain=0; create(); use_initial=true; advance(.1f,1100000000); r=advance(.2f,1200000000);
    require(r.temporally_supported && !r.admitted && r.admission_status=="NUMERICAL_TIE_NOMINAL",
        "numerical equality retains nominal");
    score_gain=-1; create(); use_initial=true; advance(.1f,1100000000); r=advance(.2f,1200000000);
    require(r.temporally_supported && !r.admitted && r.branch_difference>0,
        "temporal support is not admission");
    score_gain=1; create(); use_initial=true; advance(.1f,1100000000);
    score_gain=std::numeric_limits<double>::infinity();
    r=advance(.2f,1200000000);
    require(!pending.active && !r.admitted && r.event=="PENDING_MATCH_QUALITY_FAILED" &&
        std::abs(r.energy_difference+.02)<1e-12 && r.origin_stamp_ns==1000000000 && r.confirmation_count==1,
        "rejection preserves accumulated history before clearing");
    score_gain=1; create(); use_initial=true; aligns=scores=jets=0;
    r=run(.1f,1100000000,nullptr);
    require(!pending.active && r.event=="PENDING_ADMISSION_INPUT_FAILED" && aligns==0 && scores==0 && jets==0,
        "missing true IMU increment clears without search or backend");
    // Noncommuting frames, arbitrary common map transform, and lever-arm motion.
    auto a=pose(.5f), d=pose(.1f);
    a.block<3,3>(0,0)=Eigen::AngleAxisf(.4f,Eigen::Vector3f::UnitY()).toRotationMatrix();
    d.block<3,3>(0,0)=Eigen::AngleAxisf(.15f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Matrix4f b=a*d;
    double cost=0;
    require(p::coupledBranchMotionCost(a,b,d.cast<double>(),config.search,&cost) && cost<1e-12,
        "body interval SE3 logarithm order");
    auto left=pose(9); left.block<3,3>(0,0)=Eigen::AngleAxisf(.7f,Eigen::Vector3f::UnitX()).toRotationMatrix();
    double transformed_cost;
    require(p::coupledBranchMotionCost((left*a).eval(),(left*b).eval(),d.cast<double>(),config.search,&transformed_cost) &&
        transformed_cost<1e-10, "common map transform cannot alter motion cost");
    auto invalid=d; invalid(0,0)=2;
    require(!p::coupledBranchMotionCost(a,b,invalid.cast<double>(),config.search,&cost), "nonrigid IMU rejected");
    // Demonstrate log translation is not raw t for rotation+translation.
    double nonzero;
    require(p::coupledBranchMotionCost(pose(0),d,Eigen::Matrix4d::Identity(),config.search,&nonzero) &&
        nonzero>std::pow(.1/2,2), "true SE3 residual includes SO3 Jacobian");
    config.branch_rotation_guard=true; score_gain=1; rotated_alternative=true;
    create(); use_initial=true; advance(.1f,1100000000); r=advance(.2f,1200000000);
    require(r.temporally_supported && r.branch_difference<0 && !r.admitted &&
        !r.rotation_consistent && r.admission_status=="ROTATION_PREDICTION_DISAGREEMENT",
        "constant rotated branch may win incremental score but must pass independent rotation guard");
    std::cout << "three-frame admission tests PASS\n";
    return 0;
  } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
