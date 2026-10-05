#pragma once
namespace {
std::string denseMatrixText(const Eigen::MatrixXd& matrix) {
  std::ostringstream out;out<<std::setprecision(17);
  for(int i=0;i<matrix.rows();++i)for(int j=0;j<matrix.cols();++j){if(i||j)out<<';';out<<matrix(i,j);}
  return out.str();
}
struct ContinuationOutput {
  std::ofstream forward,reverse,certificates,events,rounds,fd,refine;
  explicit ContinuationOutput(const std::string& dir):forward(dir+"/forward_continuation.csv"),
    reverse(dir+"/reverse_continuation.csv"),certificates(dir+"/branch_certificates.csv"),
    events(dir+"/support_events.csv"),rounds(dir+"/corrector_rounds.csv"),
    fd(dir+"/frozen_derivative_fd.csv"),refine(dir+"/full_refine_final_branches.csv") {
    if(!forward||!reverse||!certificates||!events||!rounds||!fd||!refine)throw std::runtime_error("R1C output directory required");
    const std::vector<std::string> columns={"tx","cluster","direction","attempt","alpha","u","v","pose_matrix16",
      "dynamic_energy","frozen_energy","support_hash","frozen_support_hash","support_equal","support_vs_previous",
      "support_vs_T0","support_vs_canonical","strong_gradient_norm","Hvv_eigenvalues","Hvv_condition",
      "predictor_v","predictor_error","outer_iterations","inner_iterations","certified","accepted","endpoint_closure",
      "status","support_cycles","support_changes","frozen_evaluations","dynamic_evaluations","runtime_ms",
      "root_translation_m","root_rotation_deg","H_uv_vu_relative_asymmetry","derivative_valid","strong_gradient","Hvv_matrix16"};
    writeHeader(forward,columns);writeHeader(reverse,columns);writeHeader(certificates,columns);
    writeHeader(events,{"tx","cluster","direction","attempt","alpha","round","before_hash","after_hash",
      "changed_fraction","event","corrector_certified"});
    writeHeader(rounds,{"tx","cluster","direction","attempt","alpha","round","initial_v","endpoint_v",
      "before_hash","after_hash","support_equal","gradient_norm","Hvv_spd","inner_iterations","inner_status",
      "frozen_energy","dynamic_energy"});
    writeHeader(fd,{"tx","cluster","support_origin","direction","h","gradient_absolute_error","gradient_relative_error",
      "curvature_absolute_error","curvature_relative_error","hessian_symmetry","H_uv_vu_relative_asymmetry","pass"});
    writeHeader(refine,{"tx","cluster","direction","pre_pose_matrix16","post_pose_matrix16","pre_distance_m",
      "pre_distance_deg","post_distance_m","post_distance_deg","escape","iterations","status"});
  }
};

void validateFrozenFd(BranchEngine& e,const AttractorTarget& b,ContinuationOutput& output) {
  for(const std::string origin:{"T0","CANONICAL"}) {
    const bool zero=origin=="T0";
    Eigen::VectorXd x(6);x<< (zero?Eigen::VectorXd::Zero(2):b.u),(zero?Eigen::VectorXd::Zero(4):b.v);
    const auto& support=zero?e.nominal:e.canonical;
    auto value=[&](const Eigen::VectorXd& z){return e.value(z.head(2),z.tail(4),support);};
    const auto jet=frozenFiniteDifference(x,value);
    const double block_asym=(jet.hessian.topRightCorner(2,4)-jet.hessian.bottomLeftCorner(4,2).transpose()).norm()/
      std::max(1e-12,jet.hessian.norm());
    for(int i=0;i<3;++i) {
      Eigen::VectorXd q(6);
      for(int j=0;j<6;++j)q(j)=std::sin(double((i+1)*(j+2)));
      q.normalize();constexpr double h=1e-3;
      const double fp=value(x+h*q),fm=value(x-h*q);
      const double g=(fp-fm)/(2*h),c=(fp-2*jet.energy+fm)/(h*h);
      const double gp=q.dot(jet.gradient),cp=q.dot(jet.hessian*q);
      const double ge=std::abs(g-gp),ce=std::abs(c-cp);
      const double gr=ge/std::max(1e-12,std::max(std::abs(g),std::abs(gp)));
      const double cr=ce/std::max(1e-12,std::max(std::abs(c),std::abs(cp)));
      // Independent-direction truncation/float diagnostics; not certificate tolerances.
      const bool pass=jet.valid&&block_asym<=1e-3&&(ge<=1e-4||gr<=.02)&&(ce<=.02||cr<=.05);
      writeRow(output.fd,{std::to_string(b.tx),b.cluster,origin,std::to_string(i),number(h),number(ge),number(gr),
        number(ce),number(cr),number(jet.asymmetry),number(block_asym),pass?"1":"0"});
    }
  }
}

void writeContinuationPoint(BranchEngine& e,const AttractorTarget& b,ContinuationOutput& o,const std::string& direction,
    int attempt,BranchPoint& p,const Support& previous,bool accepted,bool closure,
    std::size_t f,std::size_t d,std::chrono::steady_clock::time_point start) {
  const auto anchor=direction=="FORWARD"?e.context.obs.pose:b.closed;
  const auto full=e.joint(p.u,p.v,p.derivative_support);
  const double mixed_asym=(full.hessian.topRightCorner(2,4)-full.hessian.bottomLeftCorner(4,2).transpose()).norm()/
    std::max(1e-12,full.hessian.norm());
  if(!full.valid||mixed_asym>1e-3){p.certified=false;accepted=false;p.status="DERIVATIVE_INVALID";}
  const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  std::vector<std::string> row={std::to_string(b.tx),b.cluster,direction,std::to_string(attempt),number(p.alpha),
    vectorText(p.u),vectorText(p.v),poseText(p.pose),number(p.dynamic_energy),number(p.frozen_energy),
    std::to_string(supportSignature(p.support.signature)),std::to_string(supportSignature(p.derivative_support.signature)),
    p.support.signature==p.derivative_support.signature?"1":"0",number(supportFraction(previous,p.support.signature)),
    number(supportFraction(e.nominal.signature,p.support.signature)),number(supportFraction(e.canonical.signature,p.support.signature)),
    number(p.fd.gradient.norm()),vectorText(p.fd.eigenvalues),number(p.fd.condition),vectorText(p.predictor),
    number((p.v-p.predictor).norm()),std::to_string(p.outer),std::to_string(p.inner),p.certified?"1":"0",
    accepted?"1":"0",closure?"1":"0",p.status,p.cycle?"1":"0",std::to_string(p.changes),
    std::to_string(e.frozen_calls-f),std::to_string(e.dynamic_calls-d),number(ms),
    number(translationDistance(anchor,p.pose)),number(rotationDistanceDeg(anchor,p.pose)),number(mixed_asym),
    p.fd.valid&&full.valid?"1":"0",vectorText(p.fd.gradient),denseMatrixText(p.fd.hessian)};
  writeRow(direction=="FORWARD"?o.forward:o.reverse,row);writeRow(o.certificates,row);
  for(const auto& r:p.rounds) {
    writeRow(o.rounds,{std::to_string(b.tx),b.cluster,direction,std::to_string(attempt),number(p.alpha),std::to_string(r.index),
      vectorText(r.initial),vectorText(r.endpoint),std::to_string(r.before),std::to_string(r.after),r.equal?"1":"0",
      number(r.gradient),r.spd?"1":"0",std::to_string(r.inner),r.inner_status,number(r.frozen_energy),number(r.dynamic_energy)});
    if(!r.equal)writeRow(o.events,{std::to_string(b.tx),b.cluster,direction,std::to_string(attempt),number(p.alpha),
      std::to_string(r.index),std::to_string(r.before),std::to_string(r.after),number(r.change),
      accepted?"SUPPORT_TRANSITION_CONTINUED":"SUPPORT_BOUNDARY_UNRESOLVED",p.certified?"1":"0"});
  }
  o.forward.flush();o.reverse.flush();o.certificates.flush();o.rounds.flush();o.events.flush();
}

std::vector<BranchPoint> continueBranch(BranchEngine& e,const AttractorTarget& b,const std::string& direction,
    ContinuationOutput& o) {
  const bool forward=direction=="FORWARD";
  const double sign=forward?1:-1;
  double alpha=forward?0:1,step=.05;
  Eigen::VectorXd initial=forward?Eigen::VectorXd::Zero(4):b.v;
  SupportSnapshot support=forward?e.nominal:e.canonical;
  std::vector<BranchPoint> accepted;
  int attempt=0;
  while(attempt<500) {
    const std::size_t f=e.frozen_calls,d=e.dynamic_calls;
    const auto start=std::chrono::steady_clock::now();
    BranchPoint p=e.correct(alpha*b.u,initial,support);p.alpha=alpha;
    bool closure=true;
    if(attempt==0) {
      const auto anchor=forward?e.context.obs.pose:b.closed;
      closure=rootClosurePass(translationDistance(anchor,p.pose),rotationDistanceDeg(anchor,p.pose));
    }
    writeContinuationPoint(e,b,o,direction,attempt,p,support.signature,canPredictFromRoot(p.certified,closure),closure,f,d,start);
    std::cout<<"R1C_NODE "<<b.tx<<"/"<<b.cluster<<" "<<direction<<" alpha="<<alpha<<" "<<p.status<<std::endl;
    ++attempt;
    if(!p.certified||!closure) {
      if(accepted.empty())break; // No predictor from an uncertified root.
      step*=.5;
      if(step<.005)break;
    }else {
      accepted.push_back(p);
      if((forward&&alpha>=1)||( !forward&&alpha<=0))break;
      step=std::min(.05,2*step);
    }
    const auto& previous=accepted.back();
    alpha=std::max(0.0,std::min(1.0,previous.alpha+sign*step));
    const auto jet=e.joint(previous.u,previous.v,previous.support);
    const Eigen::MatrixXd hvv=jet.hessian.bottomRightCorner(4,4);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(hvv);
    if(!jet.valid||eig.info()!=Eigen::Success||eig.eigenvalues().minCoeff()<=0)break;
    const Eigen::VectorXd derivative=-hvv.ldlt().solve(jet.hessian.bottomLeftCorner(4,2)*b.u);
    if(!derivative.allFinite())throw std::runtime_error("nonfinite branch predictor");
    initial=previous.v+derivative*(alpha-previous.alpha);support=previous.support;
  }
  return accepted;
}

void runContinuation(BranchEngine& e,const std::string& dir) {
  ContinuationOutput o(dir);
  for(const auto& item:e.basins) {
    const auto& b=item.second;e.selectBranch(b);
    validateFrozenFd(e,b,o);
    continueBranch(e,b,"FORWARD",o);continueBranch(e,b,"REVERSE",o);
  }
  std::ofstream supports(dir+"/support_memberships.csv");
  writeHeader(supports,{"support_hash","point_index","cell_bits_xyz"});
  for(const auto& item:e.support_archive)for(std::size_t i=0;i<item.second.size();++i) {
    std::ostringstream cells;
    for(std::size_t j=0;j<item.second[i].size();++j) {
      if(j)cells<<';';const auto& c=item.second[i][j];cells<<c[0]<<':'<<c[1]<<':'<<c[2];
    }
    writeRow(supports,{std::to_string(item.first),std::to_string(i),cells.str()});
  }
  std::ofstream counts(dir+"/evaluation_counts.csv");
  writeHeader(counts,{"frozen_value_evaluations","dynamic_value_evaluations","full_ndt_calls"});
  writeRow(counts,{std::to_string(e.frozen_calls),std::to_string(e.dynamic_calls),"0"});
}
}
