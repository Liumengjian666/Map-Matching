// Offline R1C3. Canonical poses/v/support are not inputs to this engine.
#define P9_TRUE_PROFILE_CLOSURE_LIBRARY
#include "p9_true_profile_closure.cpp"
#include "p9_numeric_branch_certificate.hpp"

namespace {
struct GateSupport {Support signature;ExactPclNdt::FrozenSupport leaves;double energy=0;};
struct GatePoint {
  double alpha=0;Eigen::VectorXd u,v,predictor;Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  GateSupport support,derivative_support,event_support;
  BranchCertificate certificate;bool certified=false,event=false,cycle=false;
  int outer=0,inner=0,changes=0,id=0,parent=0;
  std::string status="UNINITIALIZED";
};
struct GateOutput {
  std::ofstream roots,nodes,events,splits,checks,multih,costs,dispositions,directional;
  explicit GateOutput(const std::string& dir):roots(dir+"/numeric_root_certificates.csv"),
    nodes(dir+"/branch_nodes.csv"),events(dir+"/support_events.csv"),splits(dir+"/branch_splits.csv"),
    checks(dir+"/numeric_resolution_checks.csv"),multih(dir+"/numeric_multih.csv"),costs(dir+"/case_costs.csv"),
    dispositions(dir+"/branch_dispositions.csv"),directional(dir+"/numeric_directional_fd.csv") {
    for(auto* s:{&roots,&nodes,&events,&splits,&checks,&multih,&costs,&dispositions,&directional}) {
      if(!*s)throw std::runtime_error("R1C3 output directory required");s->exceptions(std::ios::badbit|std::ios::failbit);
    }
    const std::vector<std::string> fields={"tx","cluster","attempt","branch_id","parent_id","alpha","u","v",
      "pose_matrix16","predictor_v","certified","accepted","root_closure","status","support_hash",
      "derivative_support_hash","support_equal","dynamic_energy","frozen_energy","outer","inner","changes",
      "cycle","dn","predicted_decrement","EPS_G","EPS_DV","EPS_E","FD_valid","Hvv_SPD","resolution_pass",
      "guard_parity","reference_status","Hvv_eigenvalues","predictor_error"};
    writeHeader(roots,fields);writeHeader(nodes,fields);
    writeHeader(events,{"tx","cluster","attempt","candidate_id","parent_id","hypothesis","alpha","round","before_hash","after_hash",
      "changed_fraction","event","certified"});
    writeHeader(splits,{"tx","cluster","attempt","candidate_A","candidate_B","parent_id","alpha","event_detected","certified_A","certified_B",
      "separation_m","separation_deg","split","merged","pruned","active_count"});
    writeHeader(checks,{"tx","cluster","attempt","candidate_id","parent_id","hypothesis","alpha","round","inner","u","v","support_hash",
      "status","resolved","FD_valid","Hvv_SPD","guard_parity","region_indices","EPS_G","EPS_DV","EPS_E",
      "dn","decrement","reference_move","reference_drop","reference_status","gradient_FD_error","curvature_FD_error",
      "reference_dN","reference_decrement","rho_E","d_resolution","reference_fd_displacement","reference_fd_eigen_relative"});
    writeHeader(multih,{"tx","cluster","attempt","candidate_id","parent_id","hypothesis","round","inner","precision","h","gradient",
      "H_eigenvalues","dN","decrement","SPD"});
    writeHeader(costs,{"tx","cluster","frozen_evals","double_evals","dynamic_evals","guard_evals","corrector_calls",
      "support_cycles","runtime_ms","full_ndt_calls"});
    writeHeader(dispositions,{"tx","cluster","attempt","alpha","candidate_id","parent_id","disposition","merge_target"});
    writeHeader(directional,{"tx","cluster","attempt","candidate_id","round","inner","precision","direction",
      "gradient_error","gradient_relative","curvature_error","curvature_relative","pass"});
  }
};

struct GateEngine {
  DoubleFrozenNdt ndt;Cloud::Ptr target;FrameContext ctx;
  std::map<std::uint64_t,FrameSource> frames;std::map<std::uint64_t,ArchivedUObs> observations;
  std::size_t frozen=0,doubles=0,dynamic=0,guards=0,correctors=0,cycles=0;
  std::uint64_t tx=0;std::string cluster,hypothesis;int attempt=0,round=0,inner=0,candidateId=0,parentId=0;
  double alpha=0;GateOutput& output;
  GateEngine(const std::string& map,const std::string& cohort,const std::string& obs,GateOutput& out):
    target(loadTarget(map)),frames(readFrameSources(cohort)),observations(readArchivedUObs(obs)),output(out) {
    if(target->size()!=549606)throw std::runtime_error("target count changed");configureNdt(ndt,target);
  }
  void select(std::uint64_t frame,const std::string& basin) {
    tx=frame;cluster=basin;ctx=frameContext(frames.at(tx),observations.at(tx),ndt,target->size());++dynamic;
    // This task fixes W2/S4, not the old automatic k rule.
    ctx.k=2;ctx.weak=ctx.obs.eigenvectors.leftCols(2);ctx.strong=ctx.obs.eigenvectors.rightCols(4);
    if((ctx.obs.eigenvectors.transpose()*ctx.obs.eigenvectors-Matrix6d::Identity()).norm()>1e-12)
      throw std::runtime_error("W/S not orthonormal");
  }
  Vector6d eta(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const{return ctx.weak*u+ctx.strong*v;}
  Eigen::Matrix4f pose(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const{return poseAtEta(ctx.obs.pose,eta(u,v));}
  GateSupport snapshot(const Eigen::Matrix4f& p) {
    GateSupport s;++dynamic;s.energy=-ndt.dynamicValueOnly(ctx.source,p,&s.signature,&s.leaves)/ctx.source->size();return s;
  }
  double ff(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const GateSupport& s) {
    ++frozen;return -ndt.frozenScore(ctx.source,pose(u,v),s.leaves)/ctx.source->size();
  }
  double dd(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const GateSupport& s) {
    ++doubles;return -ndt.frozenScoreDouble(ctx.source,continuousPose(ctx.obs.pose,eta(u,v)),s.leaves)/ctx.source->size();
  }
  BranchCertificate certify(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const GateSupport& s) {
    std::size_t terms=0;for(const auto& ls:s.leaves)terms+=ls.size();
    const auto c=numericBranchCertificate(u,v,terms,
      [&](const Eigen::VectorXd& z){return ff(u,z,s);},[&](const Eigen::VectorXd& z){return dd(u,z,s);},
      [&](const Eigen::VectorXd& x){return ff(x.head(2),x.tail(4),s);},
      [&](const Eigen::VectorXd& x){return dd(x.head(2),x.tail(4),s);},
      [&](const Eigen::VectorXd& z){++guards;return ndt.guardDisagreement(ctx.source,pose(u,z),continuousPose(ctx.obs.pose,eta(u,z)),s.leaves)==0;});
    Eigen::VectorXd region(c.region.size());for(int i=0;i<region.size();++i)region(i)=c.region[i];
    writeRow(output.checks,{std::to_string(tx),cluster,std::to_string(attempt),std::to_string(candidateId),std::to_string(parentId),hypothesis,number(alpha),
      std::to_string(round),std::to_string(inner),vectorText(u),vectorText(v),std::to_string(supportSignature(s.signature)),
      c.status,c.resolved?"1":"0",c.derivative_valid?"1":"0",c.spd?"1":"0",c.guards_equal?"1":"0",vectorText(region),
      number(c.eps_g),number(c.eps_dv),number(c.eps_e),number(c.dn),number(c.decrement),number(c.reference_move),
      number(c.reference_drop),c.reference.status,number(c.gradient_fd_error),number(c.curvature_fd_error),
      c.reference.jet.d.size()?number(c.reference.jet.d.norm()):"",number(c.reference.jet.decrement),number(c.reference.rho),
      number(c.reference.d_resolution),number(c.reference.fd_displacement_difference),number(c.reference.fd_eigen_relative)});
    for(const auto& a:c.fd_audits)writeRow(output.directional,{std::to_string(tx),cluster,std::to_string(attempt),
      std::to_string(candidateId),std::to_string(round),std::to_string(inner),a[0]?"DOUBLE":"FLOAT",number(a[1]),
      number(a[2]),number(a[3]),number(a[4]),number(a[5]),a[6]?"1":"0"});
    for(int precision=0;precision<2;++precision)for(int i=0;i<6;++i) {
      const auto& j=precision?c.doubles[i]:c.floats[i];
      writeRow(output.multih,{std::to_string(tx),cluster,std::to_string(attempt),std::to_string(candidateId),std::to_string(parentId),hypothesis,std::to_string(round),
        std::to_string(inner),precision?"DOUBLE":"FLOAT",number(branchSteps[i]),vectorText(j.g),vectorText(j.eig),
        vectorText(j.d),number(j.decrement),j.spd?"1":"0"});
    }
    return c;
  }
  void minimize(GatePoint& p,const GateSupport& s) {
    double trust=.10;
    for(inner=0;inner<=20;++inner) {
      auto value=[&](const Eigen::VectorXd& v){return ff(p.u,v,s);};
      auto j=numericalJet(p.v,.001,value);completeJet(j);
      p.certificate=certify(p.u,p.v,s);
      if(p.certificate.resolved||inner==20)break;
      ++p.inner;
      const double floor=1e-4*std::max(1.,j.H.diagonal().cwiseAbs().maxCoeff());
      const double damping=std::max(0.,floor-j.eig.minCoeff());
      Eigen::VectorXd step=-(j.H+damping*Eigen::Matrix4d::Identity()).ldlt().solve(j.g);
      if(!step.allFinite())throw std::runtime_error("nonfinite corrector step");
      if(step.norm()>trust)step*=trust/step.norm();
      bool accepted=false;
      for(int line=0;line<10;++line) {
        const Eigen::VectorXd move=std::ldexp(1.,-line)*step;
        const double candidate=value(p.v+move);
        if(candidate<j.energy-1e-12) {
          const double predicted=-j.g.dot(move)-.5*move.dot(j.H*move);
          const double ratio=predicted>0?(j.energy-candidate)/predicted:0;
          p.v+=move;accepted=true;
          if(ratio>.75&&move.norm()>.9*trust)trust=std::min(.50,2*trust);
          else if(ratio<.25)trust*=.5;
          break;
        }
      }
      if(!accepted)trust*=.5;
      // This final audit may be at an accepted new pose. Give it its own key;
      // p.inner still counts optimization steps, not certificate calls.
      if(trust<1e-6){++inner;p.certificate=certify(p.u,p.v,s);break;}
    }
  }
  GatePoint correct(double a,const Eigen::VectorXd& u,const Eigen::VectorXd& initial,
                    GateSupport support,const std::string& name,int id,int parent) {
    ++correctors;alpha=a;hypothesis=name;candidateId=id;parentId=parent;
    GatePoint p;p.alpha=a;p.u=u;p.v=initial;p.predictor=initial;p.id=id;p.parent=parent;
    std::vector<Support> seen{support.signature};
    for(round=1;round<=8;++round) {
      p.outer=round;minimize(p,support);p.pose=pose(p.u,p.v);p.derivative_support=support;
      p.support=snapshot(p.pose);const bool equal=p.support.signature==support.signature;
      p.certified=completeBranchCertificate(p.certificate,equal);
      if(equal){p.status=p.certified?"CERTIFIED":p.certificate.status;return p;}
      ++p.changes;
      if(!p.event){p.event=true;p.event_support=p.support;}
      p.cycle=std::find(seen.begin(),seen.end(),p.support.signature)!=seen.end();
      writeRow(output.events,{std::to_string(tx),cluster,std::to_string(attempt),std::to_string(id),std::to_string(parent),name,number(a),std::to_string(round),
        std::to_string(supportSignature(support.signature)),std::to_string(supportSignature(p.support.signature)),
        number(supportFraction(support.signature,p.support.signature)),p.cycle?"SUPPORT_FIXED_POINT_CYCLE":"SUPPORT_UPDATE","0"});
      if(p.cycle){++cycles;p.status="SUPPORT_FIXED_POINT_CYCLE";return p;}
      seen.push_back(p.support.signature);support=p.support;
    }
    p.status="SUPPORT_FIXED_POINT_NOT_CLOSED";return p;
  }
  void record(GatePoint& p,bool accepted,bool closure,bool root=false) {
    const auto& c=p.certificate;
    writeRow(root?output.roots:output.nodes,{std::to_string(tx),cluster,std::to_string(attempt),std::to_string(p.id),
      std::to_string(p.parent),number(p.alpha),vectorText(p.u),vectorText(p.v),poseText(p.pose),vectorText(p.predictor),
      p.certified?"1":"0",accepted?"1":"0",closure?"1":"0",p.status,
      std::to_string(supportSignature(p.support.signature)),std::to_string(supportSignature(p.derivative_support.signature)),
      p.support.signature==p.derivative_support.signature?"1":"0",number(p.support.energy),
      number(ff(p.u,p.v,p.derivative_support)),std::to_string(p.outer),std::to_string(p.inner),std::to_string(p.changes),
      p.cycle?"1":"0",number(c.dn),number(c.decrement),number(c.eps_g),number(c.eps_dv),number(c.eps_e),
      c.derivative_valid?"1":"0",c.spd?"1":"0",c.resolved?"1":"0",c.guards_equal?"1":"0",c.reference.status,
      c.floats.empty()?"":vectorText(c.floats[2].eig),number((p.v-p.predictor).norm())});
    output.roots.flush();output.nodes.flush();output.checks.flush();output.events.flush();
  }
};

bool distinct(const Eigen::Matrix4f& a,const Eigen::Matrix4f& b) {
  return translationDistance(a,b)>.05||rotationDistanceDeg(a,b)>.5;
}
bool pointOrder(const GatePoint& a,const GatePoint& b) {
  if(a.support.energy!=b.support.energy)return a.support.energy<b.support.energy;
  for(int i=0;i<16;++i)if(a.pose(i/4,i%4)!=b.pose(i/4,i%4))return a.pose(i/4,i%4)<b.pose(i/4,i%4);
  return a.id<b.id;
}
struct Disposition {GatePoint point;std::string action;int target=0;};
std::vector<GatePoint> prunePoints(std::vector<GatePoint> pool,int& merges,int& pruned,std::vector<Disposition>* audit=nullptr) {
  std::sort(pool.begin(),pool.end(),pointOrder);std::vector<GatePoint> kept;
  for(const auto& p:pool) {
    int duplicate=-1;for(const auto& q:kept)if(!distinct(p.pose,q.pose)){duplicate=q.id;break;}
    if(duplicate>=0){++merges;if(audit)audit->push_back({p,"MERGED",duplicate});continue;}
    if(kept.size()==4){++pruned;if(audit)audit->push_back({p,"PRUNED",0});continue;}
    kept.push_back(p);if(audit)audit->push_back({p,"ACCEPTED",0});
  }
  return kept;
}
double smallerGateStep(double step){return step<=.005+1e-12?0:std::max(.005,step*.5);}
bool splitPass(const GatePoint& a,const GatePoint& b){return a.certified&&b.certified&&distinct(a.pose,b.pose);}

void continueCase(GateEngine& e,const Eigen::VectorXd& ub) {
  e.attempt=0;int nextId=1;
  const auto initial=e.snapshot(e.ctx.obs.pose);
  auto root=e.correct(0,Eigen::VectorXd::Zero(2),Eigen::VectorXd::Zero(4),initial,"ROOT",nextId++,0);
  const bool closure=translationDistance(root.pose,e.ctx.obs.pose)<=.02&&rotationDistanceDeg(root.pose,e.ctx.obs.pose)<=.2;
  if(!closure)root.status="ROOT_CLOSURE_FAIL";
  e.record(root,root.certified&&closure,closure,true);e.record(root,root.certified&&closure,closure);
  if(!root.certified||!closure)return;
  std::vector<GatePoint> active{root};double step=.05;
  while(e.attempt<500&&active.front().alpha<1-1e-12) {
    ++e.attempt;const double targetAlpha=std::min(1.,active.front().alpha+step);
    std::vector<GatePoint> pool;bool parentFailed=false;
    for(const auto& old:active) {
      const auto& H=old.certificate.joint.H;
      const Eigen::Matrix4d hvv=H.bottomRightCorner(4,4);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(hvv);
      if(es.eigenvalues().minCoeff()<=0)throw std::runtime_error("predictor Hvv not SPD");
      const Eigen::VectorXd derivative=-hvv.ldlt().solve(H.bottomLeftCorner(4,2)*ub);
      const Eigen::VectorXd predictor=old.v+derivative*(targetAlpha-old.alpha);
      if(!predictor.allFinite())throw std::runtime_error("nonfinite predictor");
      auto a=e.correct(targetAlpha,targetAlpha*ub,predictor,old.support,"OLD_SUPPORT",nextId++,old.id);
      GatePoint b;bool haveB=a.event;
      if(haveB){b=e.correct(targetAlpha,targetAlpha*ub,predictor,a.event_support,"NEW_SUPPORT",nextId++,old.id);
        if((b.predictor-a.predictor).norm()!=0)throw std::runtime_error("spawn predictor identity violated");}
      const bool split=haveB&&splitPass(a,b);
      if(a.certified)pool.push_back(a);if(haveB&&b.certified)pool.push_back(b);
      if(!a.certified&&(!haveB||!b.certified))parentFailed=true;
      e.record(a,false,true);if(haveB)e.record(b,false,true);
      writeRow(e.output.splits,{std::to_string(e.tx),e.cluster,std::to_string(e.attempt),std::to_string(a.id),
        haveB?std::to_string(b.id):"",std::to_string(old.id),number(targetAlpha),
        haveB?"1":"0",a.certified?"1":"0",haveB&&b.certified?"1":"0",
        haveB?number(translationDistance(a.pose,b.pose)):"",haveB?number(rotationDistanceDeg(a.pose,b.pose)):"",
        split?"1":"0","0","0","0"});
    }
    if(parentFailed&&step>.005+1e-12) {
      for(const auto& p:pool)writeRow(e.output.dispositions,{std::to_string(e.tx),e.cluster,std::to_string(e.attempt),number(targetAlpha),
        std::to_string(p.id),std::to_string(p.parent),"RETRY_DISCARDED",""});
      step=smallerGateStep(step);continue;
    }
    if(pool.empty())break;
    int merges=0,pruned=0;std::vector<Disposition> decisions;active=prunePoints(pool,merges,pruned,&decisions);
    for(const auto& d:decisions)writeRow(e.output.dispositions,{std::to_string(e.tx),e.cluster,std::to_string(e.attempt),number(targetAlpha),
      std::to_string(d.point.id),std::to_string(d.point.parent),d.action,std::to_string(d.target)});
    if(active.empty())throw std::runtime_error("accepted empty branch pool");
    for(auto& p:active)e.record(p,true,true);
    writeRow(e.output.splits,{std::to_string(e.tx),e.cluster,std::to_string(e.attempt),"","","",number(targetAlpha),
      "POOL","","","","","0",std::to_string(merges),std::to_string(pruned),std::to_string(active.size())});
    step=std::min(.05,2*step);
    std::cout<<"R1C3_NODE "<<e.tx<<"/"<<e.cluster<<" alpha="<<targetAlpha<<" active="<<active.size()<<std::endl;
  }
}

void runGate(char** argv) {
  GateOutput out(argv[6]);GateEngine e(argv[2],argv[3],argv[4],out);
  const auto requests=readCsv(argv[5]);if(requests.rows.size()!=8)throw std::runtime_error("eight case requests required");
  std::set<CandidateKey> seen;
  for(const auto& row:requests.rows) {
    const auto tx=parseU64(requests.get(row,"tx"));const auto cluster=requests.get(row,"cluster");
    if(!seen.emplace(tx,cluster).second)throw std::runtime_error("duplicate case");
    const auto ub=parseVectorText(requests.get(row,"u_b"));if(ub.size()!=2||!ub.allFinite())throw std::runtime_error("u_b shape");
    const auto f=e.frozen,d=e.doubles,y=e.dynamic,g=e.guards,c=e.correctors,z=e.cycles;
    const auto start=std::chrono::steady_clock::now();e.select(tx,cluster);continueCase(e,ub);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    writeRow(out.costs,{std::to_string(tx),cluster,std::to_string(e.frozen-f),std::to_string(e.doubles-d),
      std::to_string(e.dynamic-y),std::to_string(e.guards-g),std::to_string(e.correctors-c),std::to_string(e.cycles-z),number(ms),"0"});
    out.costs.flush();std::cout<<"R1C3_CASE_DONE "<<tx<<"/"<<cluster<<std::endl;
  }
}

bool gateSelfTest() {
  if(!numericBranchSelfTest())return false;
  std::vector<GatePoint> points(6);
  for(int i=0;i<6;++i){points[i].id=i;points[i].pose(0,3)=.1*i;points[i].support.energy=i;}
  int merged=0,pruned=0;const auto kept=prunePoints(points,merged,pruned);
  if(kept.size()!=4||merged!=0||pruned!=2)return false;
  points[1].pose=points[0].pose;merged=pruned=0;prunePoints(points,merged,pruned);
  if(merged!=1)return false;
  double step=.05;for(double expected:{.025,.0125,.00625,.005,0.}) {
    step=smallerGateStep(step);if(std::abs(step-expected)>1e-14)return false;
  }
  GatePoint a,b;a.certified=true;b.certified=false;b.pose(0,3)=.1;
  if(splitPass(a,b))return false;b.certified=true;if(!splitPass(a,b))return false;
  b.predictor=a.predictor=Eigen::VectorXd::LinSpaced(4,-.02,.03);
  if((a.predictor-b.predictor).norm()!=0)return false;
  b.pose=a.pose;if(splitPass(a,b))return false;
  std::vector<Disposition> decisions;merged=pruned=0;prunePoints(points,merged,pruned,&decisions);
  if(decisions.size()!=6||decisions[1].action!="MERGED"||decisions[1].target!=0)return false;
  const Eigen::Matrix4d H=Eigen::Matrix4d::Identity();Eigen::Matrix<double,4,2> M;M.setConstant(.1);
  const Eigen::Vector2d ub(.3,-.2);const Eigen::Vector4d predictor=-H.ldlt().solve(M*ub);
  return (predictor+M*ub).norm()<1e-12;
}

void finalDiagnostic(char** argv) {
  const auto frames=readFrameSources(argv[3]);const auto requests=readCsv(argv[4]);
  auto target=loadTarget(argv[2]);if(target->size()!=549606)throw std::runtime_error("refine target changed");
  ExactPclNdt ndt;configureNdt(ndt,target);
  std::ofstream out(argv[5]);if(!out)throw std::runtime_error("refine output required");
  out.exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(out,{"tx","cluster","branch_id","pre_pose_matrix16","post_pose_matrix16","iterations",
    "converged","status","runtime_ms","pre_support_hash","post_support_hash","support_change"});
  std::set<std::tuple<std::uint64_t,std::string,int>> seen;
  for(const auto& row:requests.rows) {
    const auto tx=parseU64(requests.get(row,"tx"));const auto cluster=requests.get(row,"cluster");
    const int branch=std::stoi(requests.get(row,"branch_id"));
    if(requests.get(row,"certified")!="1"||requests.get(row,"accepted")!="1"||
       std::abs(std::stod(requests.get(row,"alpha"))-1)>1e-12||!seen.emplace(tx,cluster,branch).second)
      throw std::runtime_error("refine only once per final certified branch");
    auto source=preprocessSource(loadPackedSource(frames.at(tx).path));
    if(source->size()!=frames.at(tx).expected_points||sourceHash(*source)!=frames.at(tx).expected_hash)
      throw std::runtime_error("refine source changed");
    ndt.setInputSource(source);const auto pre=parseMatrix16(requests.get(row,"pose_matrix16"));
    Support a,b;ndt.dynamicValueOnly(source,pre,&a);
    if(supportSignature(a)!=parseU64(requests.get(row,"support_hash")))throw std::runtime_error("refine support changed");
    Cloud aligned;const auto start=std::chrono::steady_clock::now();ndt.align(aligned,pre);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    const auto post=ndt.getFinalTransformation();if(!post.allFinite())throw std::runtime_error("invalid final diagnostic terminal");
    ndt.dynamicValueOnly(source,post,&b);
    writeRow(out,{std::to_string(tx),cluster,std::to_string(branch),poseText(pre),poseText(post),
      std::to_string(ndt.getFinalNumIteration()),ndt.hasConverged()?"1":"0",ndtStatus(ndt),number(ms),
      std::to_string(supportSignature(a)),std::to_string(supportSignature(b)),number(supportFraction(a,b))});
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!gateSelfTest())throw std::runtime_error("numeric/continuation self-test");
      std::cout<<"P9_R1C3_SELF_TEST=PASS\n";return 0;
    }
    if(argc==7&&std::string(argv[1])=="--run"){runGate(argv);return 0;}
    if(argc==6&&std::string(argv[1])=="--refine"){finalDiagnostic(argv);return 0;}
    throw std::runtime_error("--self-test | --run MAP COHORT UOBS WEAK_REQUESTS OUTDIR");
  }catch(const std::exception& ex){std::cerr<<"R1C3_ERROR "<<ex.what()<<'\n';return 1;}
}
