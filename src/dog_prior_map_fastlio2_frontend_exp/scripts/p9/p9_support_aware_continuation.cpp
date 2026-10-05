#define P9_STRONG_ATTRACTOR_LIBRARY
#include "p9_strong_attractor_closure.cpp"
#include "p9_fixed_support_branch.hpp"

namespace {
struct SupportSnapshot {
  Support signature;
  ExactPclNdt::FrozenSupport leaves;
};
struct SupportRound {
  int index=0,inner=0;
  Eigen::VectorXd initial,endpoint;
  std::uint64_t before=0,after=0;
  double change=0,gradient=0,frozen_energy=0,dynamic_energy=0;
  bool spd=false,equal=false;
  std::string inner_status;
};
struct BranchPoint {
  Eigen::VectorXd u,v,predictor;
  Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  SupportSnapshot support;
  SupportSnapshot derivative_support;
  FrozenFd fd;
  double dynamic_energy=0,frozen_energy=0,alpha=0;
  int outer=0,inner=0,changes=0;
  bool certified=false,cycle=false;
  std::string status="UNINITIALIZED";
  std::vector<SupportRound> rounds;
};
struct BranchEngine : AttractorEngine {
  std::size_t frozen_calls=0,dynamic_calls=0;
  using AttractorEngine::AttractorEngine;
  SupportSnapshot nominal,canonical;
  std::map<std::uint64_t,Support> support_archive;
  SupportSnapshot snapshot(const Eigen::Matrix4f& p) {
    SupportSnapshot r;
    ++dynamic_calls;
    ndt.dynamicValueOnly(context.source,p,&r.signature,&r.leaves);
    const auto hash=supportSignature(r.signature);
    const auto old=support_archive.find(hash);
    if(old!=support_archive.end()&&old->second!=r.signature)throw std::runtime_error("support hash collision");
    support_archive.emplace(hash,r.signature);
    return r;
  }
  void selectBranch(const AttractorTarget& b) {
    context=frameContext(frames.at(b.tx),observations.at(b.tx),ndt,target->size());
    ++dynamic_calls; // frameContext evaluates the nominal dynamic support once.
    if(context.k!=2||context.strong.cols()!=4||
       (mapChartDisplacement(context.obs.pose,b.closed)-context.weak*b.u-context.strong*b.v).norm()>1e-6)
      throw std::runtime_error("R1C fixed W/S/chart contract mismatch");
    nominal=snapshot(context.obs.pose);canonical=snapshot(b.closed);
  }
  Eigen::Matrix4f at(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const {
    return poseAtEta(context.obs.pose,context.weak*u+context.strong*v);
  }
  double value(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const SupportSnapshot& s) {
    ++frozen_calls;
    return -ndt.frozenScore(context.source,at(u,v),s.leaves)/context.source->size();
  }
  double actual(const Eigen::Matrix4f& p) {
    ++dynamic_calls;return dynamic(p);
  }
  FrozenFd joint(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const SupportSnapshot& s) {
    Eigen::VectorXd x(6);x<<u,v;
    return frozenFiniteDifference(x,[&](const Eigen::VectorXd& z){return value(z.head(2),z.tail(4),s);});
  }
  BranchPoint correct(const Eigen::VectorXd& u,const Eigen::VectorXd& initial,SupportSnapshot support) {
    BranchPoint r;r.u=u;r.v=initial;r.predictor=initial;
    SupportFixedPointHistory history(support.signature);
    for(int outer=0;outer<8;++outer) {
      r.outer=outer+1;
      auto f=[&](const Eigen::VectorXd& v){return value(u,v,support);};
      const Eigen::VectorXd old_v=r.v;
      const auto minimum=minimizeFrozenStrong(r.v,f);
      r.inner+=minimum.iterations;r.v=minimum.v;r.fd=minimum.fd;
      r.frozen_energy=f(r.v);r.pose=at(u,r.v);
      auto next=snapshot(r.pose);r.dynamic_energy=actual(r.pose);
      r.derivative_support=support;
      SupportRound round;round.index=r.outer;round.inner=minimum.iterations;
      round.initial=old_v;round.endpoint=r.v;round.before=supportSignature(support.signature);
      round.after=supportSignature(next.signature);round.change=supportFraction(support.signature,next.signature);
      round.gradient=r.fd.gradient.norm();round.spd=r.fd.spd;round.equal=next.signature==support.signature;
      round.frozen_energy=r.frozen_energy;round.dynamic_energy=r.dynamic_energy;round.inner_status=minimum.status;
      r.rounds.push_back(round);
      const auto decision=history.observe(support.signature,next.signature);
      if(decision==SupportDecision::SAME) {
        r.support=std::move(next);
        r.certified=certifiesFrozenBranch(r.fd,true);
        r.status=r.certified?"CERTIFIED_LOCAL_BRANCH_POINT":minimum.status;
        // Repeating identical support without stationarity is not a cycle.
        return r;
      }
      ++r.changes;
      r.support=next;
      if(decision==SupportDecision::CYCLE){r.cycle=true;r.status="SUPPORT_FIXED_POINT_CYCLE";return r;}
      support=std::move(next);
    }
    r.status="SUPPORT_FIXED_POINT_NOT_CLOSED";return r;
  }
};

void probeBranches(BranchEngine& e,const std::string& prefix) {
  std::ofstream out(prefix+"_seed_probe.csv");
  writeHeader(out,{"tx","cluster","direction","certified","status","outer","inner","cycles",
    "support_changes","gradient_norm","Hvv_eigenvalues","dynamic_energy","frozen_energy",
    "initial_pose","endpoint_pose","endpoint_v","closure_translation_m","closure_rotation_deg",
    "frozen_evaluations","dynamic_evaluations"});
  for(const auto& item:e.basins) {
    const auto& b=item.second;e.selectBranch(b);
    for(const std::string direction:{"FORWARD","REVERSE"}) {
      const bool forward=direction=="FORWARD";
      const auto start=forward?e.context.obs.pose:b.closed;
      const Eigen::VectorXd u=forward?Eigen::VectorXd::Zero(2):b.u;
      const Eigen::VectorXd v=forward?Eigen::VectorXd::Zero(4):b.v;
      const auto f=e.frozen_calls,d=e.dynamic_calls;
      const auto r=e.correct(u,v,forward?e.nominal:e.canonical);
      writeRow(out,{std::to_string(b.tx),b.cluster,direction,r.certified?"1":"0",r.status,
        std::to_string(r.outer),std::to_string(r.inner),r.cycle?"1":"0",std::to_string(r.changes),
        number(r.fd.gradient.norm()),vectorText(r.fd.eigenvalues),number(r.dynamic_energy),number(r.frozen_energy),
        poseText(start),poseText(r.pose),vectorText(r.v),number(translationDistance(start,r.pose)),
        number(rotationDistanceDeg(start,r.pose)),std::to_string(e.frozen_calls-f),std::to_string(e.dynamic_calls-d)});
      out.flush();std::cout<<"R1C_SEED "<<b.tx<<"/"<<b.cluster<<" "<<direction<<" "<<r.status
        <<" grad="<<r.fd.gradient.norm()<<" support_rounds="<<r.outer<<std::endl;
    }
  }
}
}

#include "p9_support_continuation_io.hpp"

int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!runFrozenBranchSelfTest())throw std::runtime_error("frozen FD/strong/predictor self-test");
      std::cout<<"P9_R1C_FROZEN_BRANCH_SELF_TEST=PASS\n";return 0;
    }
    if(argc==7&&std::string(argv[1])=="--probe") {
      BranchEngine engine(argv[2],argv[3],argv[4],argv[5]);probeBranches(engine,argv[6]);return 0;
    }
    if(argc==7&&std::string(argv[1])=="--run") {
      BranchEngine engine(argv[2],argv[3],argv[4],argv[5]);runContinuation(engine,argv[6]);return 0;
    }
    std::cerr<<"usage: --self-test | --probe/--run MAP COHORT UOBS CANONICAL OUTPUT_PREFIX/DIRECTORY\n";return 2;
  }catch(const std::exception& e){std::cerr<<"P9_R1C_ERROR "<<e.what()<<'\n';return 1;}
}
