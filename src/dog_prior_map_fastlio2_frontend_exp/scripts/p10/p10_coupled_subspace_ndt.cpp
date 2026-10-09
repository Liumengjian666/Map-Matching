// Independent P10 experiment: frozen P9 objective/chart; no oracle inputs.
#define P9_TRUE_PROFILE_CLOSURE_LIBRARY
#include "../p9/p9_true_profile_closure.cpp"

namespace {
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
std::string bit(bool value) {return value?"1":"0";}
std::string optionalNumber(double value) {return std::isfinite(value)?number(value):std::string();}
struct Coupling {
  Eigen::VectorXd predicted,raw;
  double condition=std::numeric_limits<double>::quiet_NaN();
  double residual=std::numeric_limits<double>::quiet_NaN();
  bool fallback=true,capped=false;
  std::string reason="NOT_REQUESTED";
};
Coupling couplingStep(const Eigen::MatrixXd& H,int k,const Eigen::VectorXd& du,
                      const Eigen::VectorXd& warm) {
  Coupling c;c.predicted=warm;c.raw=Eigen::VectorXd::Zero(warm.size());
  c.reason="DERIVATIVE_NONFINITE";
  if(H.rows()!=k+warm.size()||H.cols()!=H.rows()||!H.allFinite()||!du.allFinite())return c;
  const auto hvv=H.bottomRightCorner(warm.size(),warm.size()).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(hvv);
  c.reason="HVV_NOT_SPD";
  if(eig.info()!=Eigen::Success||eig.eigenvalues().minCoeff()<=0)return c;
  c.condition=eig.eigenvalues().maxCoeff()/eig.eigenvalues().minCoeff();
  c.reason="HVV_ILL_CONDITIONED";
  if(!std::isfinite(c.condition)||c.condition>1e8)return c;
  const Eigen::VectorXd rhs=-H.bottomLeftCorner(warm.size(),k)*du;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(hvv);c.raw=ldlt.solve(rhs);
  c.reason="COUPLING_SOLVE_FAILED";
  if(ldlt.info()!=Eigen::Success||!c.raw.allFinite())return c;
  c.residual=(hvv*c.raw-rhs).norm()/std::max(1e-12,rhs.norm());
  if(!std::isfinite(c.residual)||!std::isfinite(c.raw.norm()))return c;
  Eigen::VectorXd move=c.raw;
  c.capped=move.norm()>.10;if(c.capped)move*=.10/move.norm();
  c.predicted=warm+move;c.fallback=false;c.reason="OK";return c;
}
struct JointJet {
  Eigen::MatrixXd H;Eigen::VectorXd gradient;
  int evaluations=0;bool valid=false;std::string reason="CURRENT_CHART_INVALID";
};
JointJet jointJet(ExactPclNdt& ndt,const FrameContext& context,const Eigen::VectorXd& u,
                  const Eigen::VectorXd& v) {
  JointJet result;
  FrameContext joint;joint.obs.pose=context.obs.pose;joint.strong.resize(6,6);
  joint.strong<<context.weak,context.strong;
  const Vector6d eta=context.weak*u+context.strong*v;
  Vector6d p;Eigen::MatrixXd J;std::array<Eigen::MatrixXd,6> K;
  if(!strongNativePullback(joint,eta,p,J,K))return result;
  const auto jet=evaluateNativeEnergy(ndt,context.source,poseAtEta(context.obs.pose,eta),p,true,false);
  ++result.evaluations;result.gradient=J.transpose()*jet.gradient;
  result.H=J.transpose()*jet.hessian*J;
  for(int a=0;a<6;++a)result.H+=jet.gradient(a)*K[a];
  result.H=.5*(result.H+result.H.transpose()).eval();
  result.valid=result.H.allFinite()&&result.gradient.allFinite();
  result.reason=result.valid?"OK":"DERIVATIVE_NONFINITE";return result;
}
struct GridNode {int index;Eigen::VectorXd u;};
std::vector<GridNode> orderedGrid(const FrameContext& context) {
  std::vector<GridNode> remaining,ordered;
  const int n=context.k==1?17:9,j_count=context.k==2?n:1;
  for(int i=0;i<n;++i)for(int j=0;j<j_count;++j) {
    Eigen::VectorXd u(context.k);u(0)=-context.bounds(0)+2*context.bounds(0)*i/(n-1);
    if(context.k==2)u(1)=-context.bounds(1)+2*context.bounds(1)*j/(n-1);
    if(insideOriginalBounds(context,u))remaining.push_back({i*j_count+j,u});
  }
  Eigen::VectorXd previous=Eigen::VectorXd::Zero(context.k);
  while(!remaining.empty()) {
    std::size_t best=0;double distance=(remaining.front().u-previous).squaredNorm();
    for(std::size_t i=1;i<remaining.size();++i) {
      const double d=(remaining[i].u-previous).squaredNorm();
      if(d<distance-1e-12||(std::abs(d-distance)<=1e-12&&remaining[i].index<remaining[best].index))
        {best=i;distance=d;}
    }
    ordered.push_back(remaining[best]);previous=remaining[best].u;
    remaining.erase(remaining.begin()+best);
  }
  if(ordered.empty()||ordered.front().u.norm()>1e-12)throw std::runtime_error("grid must begin at u=0");
  return ordered;
}
DynamicEnergy valueProbe(ExactPclNdt& ndt,const FrameContext& context,const Eigen::Matrix4f& pose) {
  DynamicEnergy r;r.score=ndt.dynamicValueOnly(context.source,pose,&r.support);
  r.mean_energy=-r.score/context.source->size();r.support_hash=supportSignature(r.support);return r;
}
void experiment(const std::string& map,const std::string& cohort,const std::string& uobs,
                const std::string& directory,int variant) {
  if(variant!=0)throw std::runtime_error("only frozen variant=0 is defined");
  const auto frames=readFrameSources(cohort);
  const auto observations=readArchivedUObs(uobs);
  const auto target=loadTarget(map);
  if(target->size()!=549606)throw std::runtime_error("P9 target provenance count mismatch");
  ExactPclNdt ndt;configureNdt(ndt,target);
  std::ofstream out(directory+"/probes.csv"),trace(directory+"/trace.csv");
  if(!out||!trace)throw std::runtime_error("cannot open P10 outputs");
  out.exceptions(std::ios::badbit|std::ios::failbit);trace.exceptions(std::ios::badbit|std::ios::failbit);
  const std::vector<std::string> header={"transaction_id","variant","method","k","grid_index","visit_order",
    "u","u_before","v_before","v_initial","v_pred","v_corrected","du","dv_raw","coupling_capped",
    "coupling_fallback","coupling_reason","Hvv_condition","coupling_raw_relative_residual","previous_strong_gradient_norm",
    "initial_pose_matrix16","predicted_pose_matrix16","pre_refine_pose_matrix16","refined_pose_matrix16",
    "initial_mean_energy","predicted_mean_energy","pre_refine_mean_energy","refined_mean_energy","refined_score_sum",
    "coupling_energy_evaluations","strong_energy_evaluations","diagnostic_energy_evaluations","frame_setup_energy_evaluations",
    "total_explicit_energy_evaluations","strong_iterations","strong_accepted_steps","solver_status","full_ndt_calls",
    "full_ndt_iterations","full_ndt_converged","full_ndt_status","coupling_ms","strong_ms","diagnostic_ms","refine_ms","total_ms",
    "source_hash","source_point_count","target_point_count","initial_support_hash","predicted_support_hash",
    "pre_refine_support_hash","refined_support_hash","support_change_previous_to_initial","support_change_initial_to_predicted",
    "support_change_predicted_to_corrected","support_change_pre_to_refined","previous_to_pre_translation_m",
    "previous_to_pre_rotation_deg","previous_to_pre_same_basin","pre_to_refine_translation_m","pre_to_refine_rotation_deg",
    "pre_to_refine_same_basin"};
  writeHeader(out,header);
  writeHeader(trace,{"transaction_id","variant","method","grid_index","visit_order","iteration","energy_evaluations",
    "accepted_move","v","mean_energy","support_from_T0","support_from_previous_accepted",
    "branch_strong_gradient_before_step","step_or_trust_scale"});
  for(const std::uint64_t tx:{616,2226}) {
    const auto context=frameContext(frames.at(tx),observations.at(tx),ndt,target->size());
    const auto grid=orderedGrid(context);
    for(const std::string method:{"ORIGINAL_NDT","A_WEAK_ONLY","B_WARM_NEWTON","C_COUPLED_PREDICTOR","D_COUPLED_CORRECTOR"}) {
      const bool baseline=method=="ORIGINAL_NDT",coupled=method[0]=='C'||method[0]=='D';
      const bool correct=method[0]=='B'||method[0]=='D';
      Eigen::VectorXd previous_u=Eigen::VectorXd::Zero(context.k),previous_v=Eigen::VectorXd::Zero(6-context.k);
      Eigen::Matrix4f previous_pose=context.obs.pose;Support previous_support=context.nominal_support;
      const std::vector<GridNode> nodes=baseline?std::vector<GridNode>{{-1,previous_u}}:grid;
      for(std::size_t order=0;order<nodes.size();++order) {
        const auto started=Clock::now();const auto& node=nodes[order];
        const Eigen::VectorXd du=node.u-previous_u;
        const Eigen::VectorXd initial=method[0]=='A'?Eigen::VectorXd::Zero(previous_v.size()).eval():previous_v;
        Coupling c;c.predicted=initial;c.raw=Eigen::VectorXd::Zero(initial.size());c.fallback=false;
        int coupling_evaluations=0;double coupling_ms=0,previous_gradient=std::numeric_limits<double>::quiet_NaN();
        if(coupled) {
          const auto begin=Clock::now();const auto jet=jointJet(ndt,context,previous_u,previous_v);
          coupling_evaluations=jet.evaluations;
          if(jet.valid) {previous_gradient=jet.gradient.tail(previous_v.size()).norm();
            c=couplingStep(jet.H,context.k,du,initial);}
          else {c.fallback=true;c.reason=jet.reason;}
          coupling_ms=milliseconds(begin);
        }
        auto at=[&](const Eigen::VectorXd& v){return poseAtEta(context.obs.pose,context.weak*node.u+context.strong*v);};
        double diagnostic_ms=0;
        auto sample=[&](const Eigen::Matrix4f& pose){const auto begin=Clock::now();
          auto r=valueProbe(ndt,context,pose);diagnostic_ms+=milliseconds(begin);return r;};
        const auto initial_pose=at(initial),predicted_pose=at(c.predicted);
        const auto ei=sample(initial_pose),ep=sample(predicted_pose);
        StrongSolution solution;solution.v=c.predicted;solution.status=baseline?"BASELINE":(coupled?"PREDICTOR_ONLY":"WEAK_ONLY");
        if(correct)solution=iterativeNewton(ndt,context,node.u,c.predicted);
        const auto pre=at(solution.v);const auto ec=sample(pre);
        Cloud aligned;const auto refine_start=Clock::now();ndt.align(aligned,pre);
        const double refine_ms=milliseconds(refine_start);const auto refined=ndt.getFinalTransformation();
        const int iterations=ndt.getFinalNumIteration();const bool converged=ndt.hasConverged();
        const auto status=ndtStatus(ndt);
        if(!refined.allFinite())throw std::runtime_error("nonfinite P10 full refinement endpoint");
        const auto er=sample(refined);const int setup=baseline?1:0;
        const std::vector<std::string> row={std::to_string(tx),std::to_string(variant),method,std::to_string(context.k),
          std::to_string(node.index),baseline?"-1":std::to_string(order),vectorText(node.u),vectorText(previous_u),vectorText(previous_v),
          vectorText(initial),vectorText(c.predicted),vectorText(solution.v),vectorText(du),vectorText(c.raw),bit(c.capped),bit(c.fallback),
          c.reason,optionalNumber(c.condition),optionalNumber(c.residual),optionalNumber(previous_gradient),poseText(initial_pose),poseText(predicted_pose),
          poseText(pre),poseText(refined),number(ei.mean_energy),number(ep.mean_energy),number(ec.mean_energy),number(er.mean_energy),number(er.score),
          std::to_string(coupling_evaluations),std::to_string(solution.evaluations),"4",std::to_string(setup),
          std::to_string(coupling_evaluations+solution.evaluations+4+setup),std::to_string(solution.iterations),
          std::to_string(solution.accepted_steps),solution.status,"1",std::to_string(iterations),bit(converged),status,
          number(coupling_ms),number(solution.runtime_ms),number(diagnostic_ms),number(refine_ms),number(milliseconds(started)),
          std::to_string(context.frame.expected_hash),std::to_string(context.source->size()),std::to_string(target->size()),
          std::to_string(ei.support_hash),std::to_string(ep.support_hash),std::to_string(ec.support_hash),std::to_string(er.support_hash),
          number(supportFraction(previous_support,ei.support)),number(supportFraction(ei.support,ep.support)),
          number(supportFraction(ep.support,ec.support)),number(supportFraction(ec.support,er.support)),
          number(translationDistance(previous_pose,pre)),number(rotationDistanceDeg(previous_pose,pre)),bit(sameBasin(previous_pose,pre)),
          number(translationDistance(pre,refined)),number(rotationDistanceDeg(pre,refined)),bit(sameBasin(pre,refined))};
        if(row.size()!=header.size())throw std::runtime_error("P10 probe schema mismatch");
        writeRow(out,row);
        for(const auto& event:solution.trace)writeRow(trace,{std::to_string(tx),std::to_string(variant),method,
          std::to_string(node.index),std::to_string(order),std::to_string(event.iteration),std::to_string(event.evaluations),
          std::to_string(event.accepted_move),vectorText(event.v),number(event.energy),number(event.support_from_T0),
          number(event.support_from_previous),optionalNumber(event.projected_gradient_before_step),number(event.scale)});
        out.flush();trace.flush();
        // Full refinement is an observation only; transport the fixed-u endpoint.
        previous_u=node.u;previous_v=solution.v;previous_pose=pre;previous_support=ec.support;
        std::cout<<"P10 tx="<<tx<<" method="<<method<<" node="<<order+1<<'/'<<nodes.size()
          <<" strong="<<solution.status<<" full="<<status<<std::endl;
      }
    }
  }
}
void evaluateArchive(const std::string& canonical,const std::string& probes,const std::string& path) {
  const auto references=readCsv(canonical),candidates=readCsv(probes);
  std::ofstream out(path);if(!out)throw std::runtime_error("cannot open posthoc output");
  out.exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(out,{"transaction_id","cluster_id","method","grid_index","visit_order","pre_dt","pre_dr",
    "refined_dt","refined_dr","pre_recovered","refined_recovered"});
  std::set<std::pair<std::uint64_t,std::string>> seen;
  for(const auto& reference:references.rows) {
    if(references.get(reference,"group_a")!="1")continue;
    const auto tx=parseU64(references.get(reference,"transaction_id"));
    const auto cluster=references.get(reference,"cluster_id");
    if((tx!=616&&tx!=2226)||!seen.emplace(tx,cluster).second)throw std::runtime_error("invalid posthoc GROUP A references");
    const auto target=parseMatrix16(references.get(reference,"closed_pose_matrix16"));
    for(const auto& row:candidates.rows) {
      if(parseU64(candidates.get(row,"transaction_id"))!=tx)continue;
      const auto pre=parseMatrix16(candidates.get(row,"pre_refine_pose_matrix16"));
      const auto refined=parseMatrix16(candidates.get(row,"refined_pose_matrix16"));
      writeRow(out,{std::to_string(tx),cluster,candidates.get(row,"method"),candidates.get(row,"grid_index"),
        candidates.get(row,"visit_order"),number(translationDistance(pre,target)),number(rotationDistanceDeg(pre,target)),
        number(translationDistance(refined,target)),number(rotationDistanceDeg(refined,target)),bit(sameBasin(pre,target)),
        bit(candidates.get(row,"full_ndt_status")=="SUCCESS"&&sameBasin(refined,target))});
    }
  }
  if(seen.size()!=7)throw std::runtime_error("posthoc requires seven frozen GROUP A references");
}
bool couplingSelfTest() {
  if(!optionalNumber(std::numeric_limits<double>::quiet_NaN()).empty()||
     !optionalNumber(std::numeric_limits<double>::infinity()).empty()||optionalNumber(.125)!=number(.125))return false;
  FrameContext grid_test;grid_test.k=1;grid_test.bounds=Eigen::VectorXd::Ones(1);
  grid_test.weak=Matrix6d::Identity().leftCols(1);
  const auto line=orderedGrid(grid_test);
  if(line.size()!=17||line.front().index!=8||line[1].index!=7)return false;
  grid_test.k=2;grid_test.bounds=Eigen::VectorXd::Ones(2);grid_test.weak=Matrix6d::Identity().leftCols(2);
  const auto grid=orderedGrid(grid_test);std::set<int> indices;
  for(const auto& n:grid)if(!indices.insert(n.index).second)return false;
  if(grid.size()!=81||grid.front().index!=40||grid[1].index!=31)return false;
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(6,6);H(0,2)=H(2,0)=.2;H(1,4)=H(4,1)=-.3;
  const Eigen::Vector2d du(.02,-.03);const Eigen::VectorXd warm=Eigen::VectorXd::Constant(4,.04);
  auto c=couplingStep(H,2,du,warm);
  if(c.fallback||c.capped||c.residual>1e-12||
     (c.predicted-warm+H.bottomLeftCorner(4,2)*du).norm()>1e-12)return false;
  H(2,2)=0;if(!couplingStep(H,2,du,warm).fallback)return false;
  H(2,2)=1e-10;if(couplingStep(H,2,du,warm).reason!="HVV_ILL_CONDITIONED")return false;
  H(2,2)=1;c=couplingStep(H,2,20*du,warm);
  if(!c.capped||std::abs((c.predicted-warm).norm()-.10)>1e-12||c.residual>1e-12)return false;
  // Nonidentity base and nonzero global eta: retain mixed chart derivatives.
  FrameContext context;context.obs.pose=Eigen::Matrix4f::Identity();
  context.obs.pose.block<3,3>(0,0)=expRotation(Eigen::Vector3d(.3,-.2,.4)).cast<float>();
  context.strong=Matrix6d::Identity();Vector6d eta;eta<<.1,-.2,.3,.16,-.12,.09;
  Vector6d p;Eigen::MatrixXd J;std::array<Eigen::MatrixXd,6> K;
  if(!strongNativePullback(context,eta,p,J,K))return false;
  const Vector6d g=(Vector6d()<<.2,-.3,.1,.4,-.2,.3).finished();
  Matrix6d native=Matrix6d::Identity();Eigen::MatrixXd pull=J.transpose()*native*J;
  for(int a=0;a<6;++a)pull+=g(a)*K[a];
  const Vector6d direction=(Vector6d()<<.2,-.1,.3,.4,.2,-.3).finished();
  auto f=[&](const Vector6d& x) {Vector6d q;
    if(!parametersAtEta(context.obs.pose,p.tail<3>(),x,&q,4.0))return std::numeric_limits<double>::quiet_NaN();
    const Vector6d z=q-p;return g.dot(z)+.5*z.dot(native*z);};
  constexpr double h=.001;
  return std::abs((f(eta+h*direction)-f(eta-h*direction))/(2*h)-direction.dot(J.transpose()*g))<1e-6&&
    std::abs((f(eta+h*direction)+f(eta-h*direction)-2*f(eta))/(h*h)-direction.dot(pull*direction))<1e-5&&
    runClosureSelfTests();
}
} // namespace
int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!couplingSelfTest())throw std::runtime_error("coupling/global-chart self-test failed");
      std::cout<<"P10_COUPLING_SELF_TEST=PASS\n";return 0;
    }
    if(argc==5&&std::string(argv[1])=="--evaluate") {evaluateArchive(argv[2],argv[3],argv[4]);return 0;}
    if(argc==5||argc==6) {experiment(argv[1],argv[2],argv[3],argv[4],argc==6?std::stoi(argv[5]):0);return 0;}
    std::cerr<<"usage: --self-test | MAP COHORT UOBS OUTPUT [variant=0] | --evaluate CANONICAL PROBES OUTPUT\n";return 2;
  }catch(const std::exception& e){std::cerr<<"P10_ERROR "<<e.what()<<'\n';return 1;}
}
