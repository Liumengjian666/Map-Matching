// R1C3A offline numerical-contract diagnostics. This translation unit reuses
// the frozen R1C3 evaluator and chart, but never calls NDT align().
#define main p9_secondgen_branch_gate_main
#include "p9_secondgen_branch_gate.cpp"
#undef main

namespace {

const std::vector<double> kPredictorSteps={.004,.002,.001,.0005,.00025,.000125};

std::string vecText(const Eigen::VectorXd& values) {
  std::ostringstream out;out<<std::setprecision(17);
  for(Eigen::Index i=0;i<values.size();++i){if(i)out<<';';out<<values(i);}return out.str();
}

Eigen::VectorXd parseVec(const std::string& value,Eigen::Index expected) {
  const auto fields=split(value,';');
  if(static_cast<Eigen::Index>(fields.size())!=expected)throw std::runtime_error("vector field dimension mismatch");
  Eigen::VectorXd result(expected);
  for(Eigen::Index i=0;i<expected;++i)result(i)=std::stod(fields[static_cast<std::size_t>(i)]);
  if(!result.allFinite())throw std::runtime_error("nonfinite vector field");
  return result;
}

bool eq(const std::string& a,const std::string& b){return a==b;}
double numberOr(const std::string& value,double fallback=0) {return value.empty()?fallback:std::stod(value);}

struct DiagnosticEngine {
  DoubleFrozenNdt ndt;
  Cloud::Ptr target;
  std::map<std::uint64_t,FrameSource> frames;
  std::map<std::uint64_t,ArchivedUObs> observations;
  FrameContext ctx;
  std::uint64_t tx=0;
  std::string cluster;
  std::size_t dynamic_evals=0,frozen_evals=0,double_evals=0;

  DiagnosticEngine(const std::string& map,const std::string& cohort,const std::string& uobs)
    :target(loadTarget(map)),frames(readFrameSources(cohort)),observations(readArchivedUObs(uobs)) {
    if(target->size()!=549606)throw std::runtime_error("raw map target point count mismatch");
    configureNdt(ndt,target);
  }
  void select(std::uint64_t frame,const std::string& basin) {
    tx=frame;cluster=basin;
    ctx=frameContext(frames.at(tx),observations.at(tx),ndt,target->size());
    ++dynamic_evals; // frameContext performs one nominal dynamic support evaluation.
    ctx.k=2;ctx.weak=ctx.obs.eigenvectors.leftCols(2);ctx.strong=ctx.obs.eigenvectors.rightCols(4);
    if((ctx.obs.eigenvectors.transpose()*ctx.obs.eigenvectors-Matrix6d::Identity()).norm()>1e-12)
      throw std::runtime_error("frozen U/S orthogonality mismatch");
  }
  Vector6d eta(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const{return ctx.weak*u+ctx.strong*v;}
  Eigen::Matrix4f pose(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const{return poseAtEta(ctx.obs.pose,eta(u,v));}
  GateSupport snapshot(const Eigen::Matrix4f& p) {
    GateSupport s;++dynamic_evals;
    s.energy=-ndt.dynamicValueOnly(ctx.source,p,&s.signature,&s.leaves)/ctx.source->size();return s;
  }
  double ff(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const GateSupport& s) {
    ++frozen_evals;return -ndt.frozenScore(ctx.source,pose(u,v),s.leaves)/ctx.source->size();
  }
  double dd(const Eigen::VectorXd& u,const Eigen::VectorXd& v,const GateSupport& s) {
    ++double_evals;return -ndt.frozenScoreDouble(ctx.source,continuousPose(ctx.obs.pose,eta(u,v)),s.leaves)/ctx.source->size();
  }
};

struct PredictorRow {
  double h=0;
  NumericJet f,d;
  Eigen::Vector4d pf=Eigen::Vector4d::Zero(),pd=Eigen::Vector4d::Zero();
  Eigen::Vector4d eigf=Eigen::Vector4d::Zero(),eigd=Eigen::Vector4d::Zero();
  double condition_f=0,condition_d=0;
  bool hvvf=false,hvvd=false;
};

std::vector<PredictorRow> predictorJets(DiagnosticEngine& e,const Eigen::Vector2d& u,
    const Eigen::Vector4d& v,const Eigen::Vector2d& ub,const GateSupport& support) {
  Eigen::VectorXd x(6);x<<u,v;
  std::vector<PredictorRow> rows;
  for(double h:kPredictorSteps) {
    auto ff=[&](const Eigen::VectorXd& z){return e.ff(z.head(2),z.tail(4),support);};
    auto dd=[&](const Eigen::VectorXd& z){return e.dd(z.head(2),z.tail(4),support);};
    PredictorRow r;r.h=h;r.f=numericalJet(x,h,ff);r.d=numericalJet(x,h,dd);completeJet(r.f);completeJet(r.d);
    const Eigen::Matrix4d hvvf=r.f.H.bottomRightCorner(4,4), hvvd=r.d.H.bottomRightCorner(4,4);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> ef(hvvf),ed(hvvd);
    if(ef.info()!=Eigen::Success||ed.info()!=Eigen::Success)throw std::runtime_error("predictor Hvv eigensolver");
    r.eigf=ef.eigenvalues();r.eigd=ed.eigenvalues();
    r.hvvf=r.f.H.allFinite()&&r.eigf.minCoeff()>0;
    r.hvvd=r.d.H.allFinite()&&r.eigd.minCoeff()>0;
    if(r.hvvf) {
      r.pf=-hvvf.ldlt().solve(r.f.H.bottomLeftCorner(4,2)*ub);
      r.condition_f=r.eigf.maxCoeff()/r.eigf.minCoeff();
    } else r.pf.setConstant(std::numeric_limits<double>::quiet_NaN());
    if(r.hvvd) {
      r.pd=-hvvd.ldlt().solve(r.d.H.bottomLeftCorner(4,2)*ub);
      r.condition_d=r.eigd.maxCoeff()/r.eigd.minCoeff();
    } else r.pd.setConstant(std::numeric_limits<double>::quiet_NaN());
    rows.push_back(r);
  }
  return rows;
}

struct PredictorDecision {
  bool valid=false;
  int first=-1,last=-1;
  double max_float_double_step=0,max_float_spread_step=0,max_double_relative_step=0,max_double_eig_relative=0;
};

PredictorDecision assessPredictor(const std::vector<PredictorRow>& rows,double delta_alpha,double eps_dv) {
  PredictorDecision out;
  for(int a=0;a+2<static_cast<int>(rows.size());++a)for(int b=a+2;b<static_cast<int>(rows.size());++b) {
    bool pass=true;double fd=0,fs=0,dr=0,er=0;
    for(int i=a;i<=b;++i) {
      const auto& r=rows[i];
      if(!r.hvvf||!r.hvvd||!r.pf.allFinite()||!r.pd.allFinite()){pass=false;break;}
      fd=std::max(fd,(r.pf-r.pd).norm()*delta_alpha);
      if(i<b) {
        const auto& next=rows[i+1];
        const double drel=(r.pd-next.pd).norm()/std::max(1e-15,std::max(r.pd.norm(),next.pd.norm()));
        const Eigen::Vector4d ef=r.eigd,en=next.eigd;
        const double erel=((ef-en).cwiseAbs().array()/ef.cwiseAbs().cwiseMax(en.cwiseAbs()).array().max(1e-300)).maxCoeff();
        const double fstep=(r.pf-next.pf).norm()*delta_alpha;
        dr=std::max(dr,drel);er=std::max(er,erel);fs=std::max(fs,fstep);
        if(drel>.20||erel>.10){pass=false;break;}
      }
    }
    if(pass&&fd<=eps_dv&&fs<=eps_dv) {
      if(b-a+1>out.last-out.first+1){out.valid=true;out.first=a;out.last=b;
        out.max_float_double_step=fd;out.max_float_spread_step=fs;out.max_double_relative_step=dr;out.max_double_eig_relative=er;}
    }
  }
  return out;
}

void csvHeader(std::ofstream& o,const std::vector<std::string>& fields){writeHeader(o,fields);}

using SupportRegistry=std::map<std::uint64_t,GateSupport>;
using CsvRow=std::vector<std::string>;

struct CaseRequest {std::uint64_t tx=0;std::string cluster;Eigen::Vector2d ub=Eigen::Vector2d::Zero();};
struct RootRecord {CaseRequest request;const CsvRow* row=nullptr;Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  Eigen::Vector4d v=Eigen::Vector4d::Zero();std::uint64_t support=0,derivative_support=0;};

bool closeAlpha(const std::string& value,double wanted){return std::abs(std::stod(value)-wanted)<1e-10;}

const CsvRow* findRoot(const CsvTable& roots,std::uint64_t tx,const std::string& cluster) {
  const CsvRow* found=nullptr;
  for(const auto& row:roots.rows)if(parseU64(roots.get(row,"tx"))==tx&&roots.get(row,"cluster")==cluster&&
      closeAlpha(roots.get(row,"alpha"),0.0)&&roots.get(row,"branch_id")=="1"&&roots.get(row,"parent_id")=="0") {
    if(found)throw std::runtime_error("duplicate archived root record");found=&row;
  }
  if(!found)throw std::runtime_error("missing archived root record "+std::to_string(tx)+"/"+cluster);
  return found;
}

const CsvRow* findRootStartCheck(const CsvTable& checks,std::uint64_t tx,const std::string& cluster) {
  const CsvRow* found=nullptr;
  for(const auto& row:checks.rows)if(parseU64(checks.get(row,"tx"))==tx&&checks.get(row,"cluster")==cluster&&
      checks.get(row,"attempt")=="0"&&checks.get(row,"candidate_id")=="1"&&
      checks.get(row,"parent_id")=="0"&&checks.get(row,"hypothesis")=="ROOT"&&
      closeAlpha(checks.get(row,"alpha"),0.0)&&checks.get(row,"round")=="1"&&checks.get(row,"inner")=="0") {
    if(found)throw std::runtime_error("duplicate archived root-start check");found=&row;
  }
  if(!found)throw std::runtime_error("missing archived root-start check "+std::to_string(tx)+"/"+cluster);
  return found;
}

const CsvRow* findCheck(const CsvTable& checks,const CsvRow& event,bool first) {
  const CsvRow* selected=nullptr;int selected_inner=first?std::numeric_limits<int>::max():-1;
  for(const auto& row:checks.rows) {
    if(parseU64(checks.get(row,"tx"))!=parseU64(checks.get(event,"tx"))||
       checks.get(row,"cluster")!=checks.get(event,"cluster")||checks.get(row,"attempt")!=checks.get(event,"attempt")||
       checks.get(row,"candidate_id")!=checks.get(event,"candidate_id")||checks.get(row,"hypothesis")!=checks.get(event,"hypothesis")||
       !closeAlpha(checks.get(row,"alpha"),std::stod(checks.get(event,"alpha")))||
       checks.get(row,"round")!=checks.get(event,"round"))continue;
    const int inner=std::stoi(checks.get(row,"inner"));
    if((first&&inner<selected_inner)||(!first&&inner>=selected_inner)){selected=&row;selected_inner=inner;}
  }
  return selected;
}

void registerSnapshot(DiagnosticEngine& e,SupportRegistry& registry,const Eigen::Matrix4f& pose,
                      std::uint64_t expected,const std::string& context) {
  auto support=e.snapshot(pose);const auto actual=supportSignature(support.signature);
  if(actual!=expected)throw std::runtime_error("historical support hash not reproduced at "+context+
      " tx="+std::to_string(e.tx)+" cluster="+e.cluster+" expected="+
      std::to_string(expected)+" actual="+std::to_string(actual));
  const auto found=registry.find(actual);
  if(found!=registry.end()&&found->second.signature!=support.signature)
    throw std::runtime_error("support hash collision within a frame");
  registry[actual]=std::move(support);
}

void loadRelevantSupportHistory(DiagnosticEngine& e,const CsvTable& events,const CsvTable& checks,
    const CsvTable& nodes,std::map<std::uint64_t,SupportRegistry>& registry) {
  const std::set<std::pair<std::uint64_t,std::string>> wanted={{2226,"P05"},{2350,"P01"},{3341,"P02"},
    {2722,"P05"},{616,"P10"},{3796,"P06"}};
  std::set<std::pair<std::uint64_t,std::string>> selected_frames;
  for(const auto& key:wanted) {
    if(selected_frames.insert({key.first,""}).second) {
      e.select(key.first,key.second);auto& map=registry[key.first];
      auto nominal=e.snapshot(e.ctx.obs.pose);
      if(nominal.signature!=e.ctx.nominal_support)throw std::runtime_error("nominal support parity failure");
      map[supportSignature(nominal.signature)]=std::move(nominal);
    }
  }
  // Seed the registry with the already accepted TX616 alpha=.68125 support
  // before processing .68625 events: the OLD_SUPPORT pre-state is inherited
  // from that archived node and is not necessarily the support at the new
  // predictor pose.
  for(const auto& row:nodes.rows) {
    const auto tx=parseU64(nodes.get(row,"tx"));const std::string cluster=nodes.get(row,"cluster");
    const double alpha=std::stod(nodes.get(row,"alpha"));
    if(tx!=616||cluster!="P10"||(!closeAlpha(nodes.get(row,"alpha"),.68125)&&!closeAlpha(nodes.get(row,"alpha"),.68625)))continue;
    e.select(tx,cluster);const auto pose=parseMatrix16(nodes.get(row,"pose_matrix16"));
    const auto expected=parseU64(nodes.get(row,"support_hash"));
    registerSnapshot(e,registry[tx],pose,expected,"archived branch node alpha="+number(alpha));
  }
  // Every after-support is reconstructed only at the endpoint already logged
  // for that exact historical corrector round.
  for(const auto& event:events.rows) {
    const auto tx=parseU64(events.get(event,"tx"));const auto key=std::make_pair(tx,events.get(event,"cluster"));
    const std::string hyp=events.get(event,"hypothesis");const double alpha=std::stod(events.get(event,"alpha"));
    const bool root3341=tx==3341&&key.second=="P02"&&hyp=="ROOT"&&closeAlpha(events.get(event,"alpha"),0.0);
    const bool boundary616=tx==616&&key.second=="P10"&&(hyp=="OLD_SUPPORT"||hyp=="NEW_SUPPORT")&&closeAlpha(events.get(event,"alpha"),.68625);
    if(!root3341&&!boundary616)continue;
    e.select(tx,key.second);auto& map=registry[tx];
    const CsvRow* first=findCheck(checks,event,true);const CsvRow* last=findCheck(checks,event,false);
    if(!first||!last)throw std::runtime_error("support event has no archived numerical check");
    const auto before=parseU64(events.get(event,"before_hash"));
    if(!map.count(before)) {
      const Eigen::Vector2d u=parseVec(checks.get(*first,"u"),2);
      const Eigen::Vector4d v=parseVec(checks.get(*first,"v"),4);
      registerSnapshot(e,map,e.pose(u,v),before,"event pre-state");
    }
    const auto after=parseU64(events.get(event,"after_hash"));
    const Eigen::Vector2d u=parseVec(checks.get(*last,"u"),2);
    const Eigen::Vector4d v=parseVec(checks.get(*last,"v"),4);
    registerSnapshot(e,map,e.pose(u,v),after,"event post-state");
  }
}

void writePredictorRows(std::ofstream& out,DiagnosticEngine& e,const std::string& node,
    double alpha,double delta_alpha,double eps_dv,const Eigen::Vector2d& u,const Eigen::Vector4d& v,
    const Eigen::Vector2d& ub,const std::string& support_role,std::uint64_t support_hash,
    const GateSupport& support,const std::string& float_fd_status,const std::string& double_fd_status) {
  const auto rows=predictorJets(e,u,v,ub,support);const auto decision=assessPredictor(rows,delta_alpha,eps_dv);
  for(std::size_t row_index=0;row_index<rows.size();++row_index) {
    const auto& r=rows[row_index];
    const double disagreement=(r.pf.allFinite()&&r.pd.allFinite())?(r.pf-r.pd).norm()*delta_alpha:
      std::numeric_limits<double>::quiet_NaN();
    const bool has_next=row_index+1<rows.size();
    const double float_spread=has_next&&r.pf.allFinite()&&rows[row_index+1].pf.allFinite()?
      (r.pf-rows[row_index+1].pf).norm()*delta_alpha:std::numeric_limits<double>::quiet_NaN();
    const double double_spread=has_next&&r.pd.allFinite()&&rows[row_index+1].pd.allFinite()?
      (r.pd-rows[row_index+1].pd).norm()*delta_alpha:std::numeric_limits<double>::quiet_NaN();
    const auto ix=decision.first<0?std::string():number(kPredictorSteps[static_cast<std::size_t>(decision.first)]);
    const auto jx=decision.last<0?std::string():number(kPredictorSteps[static_cast<std::size_t>(decision.last)]);
    Eigen::VectorXd ef=r.eigf,ed=r.eigd;
    writeRow(out,{std::to_string(e.tx),e.cluster,node,number(alpha),support_role,std::to_string(support_hash),
      number(delta_alpha),number(eps_dv),number(r.h),r.hvvf?"1":"0",r.hvvd?"1":"0",
      r.pf.allFinite()?vecText(r.pf):"",r.pd.allFinite()?vecText(r.pd):"",
      r.pf.allFinite()?number(r.pf.norm()):"",r.pd.allFinite()?number(r.pd.norm()):"",
      number(disagreement),has_next?number(float_spread):"",has_next?number(double_spread):"",vecText(ef),vecText(ed),number(r.condition_f),number(r.condition_d),
      decision.valid?"1":"0",ix,jx,float_fd_status,double_fd_status});
  }
}

std::map<std::pair<std::uint64_t,std::string>,CaseRequest> readRequests(const std::string& path) {
  const CsvTable table=readCsv(path);std::map<std::pair<std::uint64_t,std::string>,CaseRequest> result;
  for(const auto& row:table.rows) {
    CaseRequest item;item.tx=parseU64(table.get(row,"tx"));item.cluster=table.get(row,"cluster");
    const auto u=parseVec(table.get(row,"u_b"),2);item.ub=u;
    if(!result.emplace(std::make_pair(item.tx,item.cluster),item).second)throw std::runtime_error("duplicate weak-path request");
  }
  if(result.size()!=8)throw std::runtime_error("expected frozen eight-case request set");
  return result;
}

const CsvRow* findNodeAt(const CsvTable& nodes,std::uint64_t tx,const std::string& cluster,
                         double alpha,int branch_id=-1) {
  const CsvRow* found=nullptr;
  for(const auto& row:nodes.rows)if(parseU64(nodes.get(row,"tx"))==tx&&nodes.get(row,"cluster")==cluster&&
      closeAlpha(nodes.get(row,"alpha"),alpha)&&(branch_id<0||std::stoi(nodes.get(row,"branch_id"))==branch_id)) {
    if(nodes.get(row,"accepted")=="1") {
      if(found&&branch_id>=0)throw std::runtime_error("duplicate accepted branch node");found=&row;
    }
  }
  return found;
}

const CsvRow* findFirstCheck(const CsvTable& checks,std::uint64_t tx,const std::string& cluster,
    int candidate,const std::string& hyp,double alpha,int round=1) {
  for(const auto& row:checks.rows)if(parseU64(checks.get(row,"tx"))==tx&&checks.get(row,"cluster")==cluster&&
      std::stoi(checks.get(row,"attempt"))>0&&std::stoi(checks.get(row,"candidate_id"))==candidate&&
      checks.get(row,"hypothesis")==hyp&&closeAlpha(checks.get(row,"alpha"),alpha)&&
      std::stoi(checks.get(row,"round"))==round&&std::stoi(checks.get(row,"inner"))==0)return &row;
  return nullptr;
}

std::string fdPassSummary(const CsvTable& directional,std::uint64_t tx,const std::string& cluster,
                         const std::string& precision,int candidate=1) {
  bool saw=false,all=true;
  for(const auto& row:directional.rows)if(parseU64(directional.get(row,"tx"))==tx&&
      directional.get(row,"cluster")==cluster&&std::stoi(directional.get(row,"attempt"))==0&&
      std::stoi(directional.get(row,"candidate_id"))==candidate&&directional.get(row,"precision")==precision) {
    saw=true;all=all&&directional.get(row,"pass")=="1";
  }
  return saw?(all?"PASS":"FAIL"):"NOT_RECORDED";
}

void runSupportPairDiagnostics(char** argv,DiagnosticEngine& e,const CsvTable& events,
    const CsvTable& checks,const std::map<std::uint64_t,SupportRegistry>& registry,
    const std::map<std::pair<std::uint64_t,std::string>,CaseRequest>& requests);

void runDiagnostics(char** argv) {
  const std::string outdir=argv[11];
  const CsvTable roots=readCsv(argv[6]),checks=readCsv(argv[7]);
  const CsvTable events=readCsv(argv[8]),nodes=readCsv(argv[9]),directional=readCsv(argv[10]);
  const auto requests=readRequests(argv[5]);
  DiagnosticEngine e(argv[2],argv[3],argv[4]);
  // argv contract: MAP COHORT UOBS REQUESTS ROOTS CHECKS EVENTS NODES DIRECTIONAL OUTDIR
  std::map<std::uint64_t,SupportRegistry> support_registry;
  loadRelevantSupportHistory(e,events,checks,nodes,support_registry);

  std::ofstream pred(outdir+"/predictor_multih.csv"),rawfd(outdir+"/raw_derivative_diagnostic.csv"),anchor(outdir+"/root_anchor.csv");
  for(auto* stream:{&pred,&rawfd,&anchor})if(!*stream)throw std::runtime_error("cannot open R1C3A output");
  pred.exceptions(std::ios::badbit|std::ios::failbit);rawfd.exceptions(std::ios::badbit|std::ios::failbit);anchor.exceptions(std::ios::badbit|std::ios::failbit);
  csvHeader(pred,{"tx","cluster","node_id","alpha","support_role","support_hash","delta_alpha","eps_dv","h",
    "float_Hvv_spd","double_Hvv_spd","float_predictor","double_predictor","float_predictor_norm","double_predictor_norm",
    "predictor_step_disagreement","float_adjacent_step_spread","double_adjacent_step_spread","float_Hvv_eigenvalues",
    "double_Hvv_eigenvalues","float_condition","double_condition","predictor_valid","stable_h_first","stable_h_last",
    "float_raw_fd_status","double_raw_fd_status"});
  csvHeader(rawfd,{"tx","cluster","attempt","candidate_id","round","inner","precision","direction","gradient_error",
    "gradient_relative","curvature_error","curvature_relative","pass"});
  csvHeader(anchor,{"tx","cluster","independent_root_id","root_v","root_v_norm","translation_from_T0_m","rotation_from_T0_deg",
    "old_002m_02deg_pass","nominal_anchor_02m_2deg_associated","T0_dynamic_energy","root_dynamic_energy",
    "T0_frozen_on_T0_support","root_frozen_on_derivative_support","support_hash_T0","support_hash_root",
    "derivative_support_hash","exact_support_equal","support_changed_point_fraction","energy_drop_dynamic",
    "root_start_u","root_start_v","root_start_support_hash","root_start_support_equals_T0",
    "corrector_start_u_zero_v_zero","oracle_information_used"});

  const std::vector<std::pair<std::uint64_t,std::string>> root_keys={{2226,"P05"},{2350,"P01"},{3341,"P02"},
    {2722,"P05"},{616,"P10"},{3796,"P06"}};
  std::map<std::pair<std::uint64_t,std::string>,RootRecord> root_records;
  for(const auto& key:root_keys) {
    const CaseRequest req=requests.at(key);const CsvRow* rr=findRoot(roots,key.first,key.second);
    RootRecord root;root.request=req;root.row=rr;root.pose=parseMatrix16(roots.get(*rr,"pose_matrix16"));
    root.v=parseVec(roots.get(*rr,"v"),4);root.support=parseU64(roots.get(*rr,"support_hash"));
    root.derivative_support=parseU64(roots.get(*rr,"derivative_support_hash"));
    e.select(key.first,key.second);auto& registry=support_registry[key.first];
    const auto t0=e.snapshot(e.ctx.obs.pose);const auto root_dynamic=e.snapshot(root.pose);
    if(supportSignature(root_dynamic.signature)!=root.support)throw std::runtime_error("root dynamic support parity fail");
    registry[supportSignature(t0.signature)]=t0;registry[root.support]=root_dynamic;
    if(!registry.count(root.derivative_support))throw std::runtime_error("cannot reconstruct root derivative support");
    const auto& support=registry.at(root.derivative_support);
    const CsvRow* start_check=findRootStartCheck(checks,key.first,key.second);
    const Eigen::Vector2d start_u=parseVec(checks.get(*start_check,"u"),2);
    const Eigen::Vector4d start_v=parseVec(checks.get(*start_check,"v"),4);
    const auto start_support=parseU64(checks.get(*start_check,"support_hash"));
    const bool zero_start=start_u.norm()==0.0&&start_v.norm()==0.0;
    const bool start_support_is_t0=start_support==supportSignature(t0.signature);
    const bool root_start_verified=zero_start&&start_support_is_t0;
    const Eigen::Vector2d u=parseVec(roots.get(*rr,"u"),2);
    const double eps=numberOr(roots.get(*rr,"EPS_DV"));
    const std::string float_fd_status=fdPassSummary(directional,key.first,key.second,"FLOAT");
    const std::string double_fd_status=fdPassSummary(directional,key.first,key.second,"DOUBLE");
    writePredictorRows(pred,e,"ROOT",0,.05,eps,u,root.v,req.ub,"DERIVATIVE",root.derivative_support,support,float_fd_status,double_fd_status);
    if(key.first==3341&&root.support!=root.derivative_support) {
      const auto& dynamic_support=registry.at(root.support);
      writePredictorRows(pred,e,"ROOT_DYNAMIC_SUPPORT",0,.05,eps,u,root.v,req.ub,"DYNAMIC",root.support,dynamic_support,float_fd_status,double_fd_status);
    }
    for(const auto& fr:directional.rows)if(parseU64(directional.get(fr,"tx"))==key.first&&
       directional.get(fr,"cluster")==key.second&&std::stoi(directional.get(fr,"attempt"))==0&&
       std::stoi(directional.get(fr,"candidate_id"))==1)
      writeRow(rawfd,{directional.get(fr,"tx"),directional.get(fr,"cluster"),directional.get(fr,"attempt"),
        directional.get(fr,"candidate_id"),directional.get(fr,"round"),directional.get(fr,"inner"),directional.get(fr,"precision"),
        directional.get(fr,"direction"),directional.get(fr,"gradient_error"),directional.get(fr,"gradient_relative"),
        directional.get(fr,"curvature_error"),directional.get(fr,"curvature_relative"),directional.get(fr,"pass")});
    const auto root_support=registry.at(root.support);const auto deriv_support=registry.at(root.derivative_support);
    const double t0_energy=t0.energy,root_energy=root_dynamic.energy;
    const double frozen_t0=e.ff(Eigen::Vector2d::Zero(),Eigen::Vector4d::Zero(),t0);
    const double frozen_root=e.ff(u,root.v,deriv_support);
    const double dt=translationDistance(root.pose,e.ctx.obs.pose),dr=rotationDistanceDeg(root.pose,e.ctx.obs.pose);
    const bool exact= root_support.signature==deriv_support.signature;
    const double changed=supportFraction(deriv_support.signature,root_support.signature);
    writeRow(anchor,{std::to_string(key.first),key.second,std::to_string(key.first)+"/"+key.second,vecText(root.v),
      number(root.v.norm()),number(dt),number(dr),(dt<=.02&&dr<=.2)?"1":"0",(dt<=.2&&dr<=2.)?"1":"0",
      number(t0_energy),number(root_energy),number(frozen_t0),number(frozen_root),
      std::to_string(supportSignature(t0.signature)),std::to_string(root.support),std::to_string(root.derivative_support),
      exact?"1":"0",number(changed),number(t0_energy-root_energy),vecText(start_u),vecText(start_v),
      std::to_string(start_support),start_support_is_t0?"1":"0",root_start_verified?"YES":"NO",
      root_start_verified?"NO":"UNVERIFIED"});
    root_records[key]=root;
  }

  // Existing 616 boundary endpoints only: accepted alpha=.68125 and the
  // already-failed alpha=.68625 old/new support hypotheses.
  e.select(616,"P10");
  const auto* accepted=findNodeAt(nodes,616,"P10",.68125);
  if(!accepted)throw std::runtime_error("missing pre-existing alpha=.68125 accepted node");
  const auto ub=requests.at({616,"P10"}).ub;
  {
    const auto v=parseVec(nodes.get(*accepted,"v"),4),u=parseVec(nodes.get(*accepted,"u"),2);
    const auto support_hash=parseU64(nodes.get(*accepted,"derivative_support_hash"));
    const double eps=numberOr(nodes.get(*accepted,"EPS_DV"));
    writePredictorRows(pred,e,"ALPHA_0P68125",.68125,.005,eps,u,v,ub,"ACCEPTED",support_hash,
      support_registry.at(616).at(support_hash),"NOT_RECORDED","NOT_RECORDED");
  }
  for(const auto& spec:std::vector<std::pair<int,std::string>>{{44,"OLD_SUPPORT"},{45,"NEW_SUPPORT"}}) {
    const auto* start=findFirstCheck(checks,616,"P10",spec.first,spec.second,.68625,1);
    if(!start)throw std::runtime_error("missing pre-existing alpha=.68625 predictor record");
    const Eigen::Vector2d u=parseVec(checks.get(*start,"u"),2);const Eigen::Vector4d v=parseVec(checks.get(*start,"v"),4);
    const auto hash=parseU64(checks.get(*start,"support_hash"));
    writePredictorRows(pred,e,spec.second+"_ALPHA_0P68625",.68625,.005,numberOr(checks.get(*start,"EPS_DV")),
      u,v,ub,spec.second,hash,support_registry.at(616).at(hash),"NOT_RECORDED","NOT_RECORDED");
  }
  pred.flush();rawfd.flush();anchor.flush();
  runSupportPairDiagnostics(argv,e,events,checks,support_registry,requests);
  std::cout<<"R1C3A_PREDICTOR_DIAGNOSTIC=COMPLETE dynamic="<<e.dynamic_evals
    <<" float="<<e.frozen_evals<<" double="<<e.double_evals<<"\n";
}

struct FixedSupportMinimum {
  ReferenceMinimum reference;
  Eigen::Vector4d v=Eigen::Vector4d::Zero();
  Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  GateSupport dynamic;
  bool dynamic_equal=false,double_predictor_stable=false;
  int stable_first=-1,stable_last=-1;
  Eigen::Vector4d predictor=Eigen::Vector4d::Zero();
};

FixedSupportMinimum solveFixedSupport(DiagnosticEngine& e,const Eigen::Vector2d& u,
    const Eigen::Vector4d& initial,const Eigen::Vector2d& ub,const GateSupport& support) {
  FixedSupportMinimum result;
  std::size_t terms=0;for(const auto& leaves:support.leaves)terms+=leaves.size();
  result.reference=referenceMinimum(initial,terms,[&](const Eigen::VectorXd& v){return e.dd(u,v,support);});
  if(result.reference.v.size()!=4)throw std::runtime_error("fixed support minimum returned invalid v");
  result.v=result.reference.v;
  Eigen::VectorXd dynamic_pose_v=result.v;
  result.pose=e.pose(u,dynamic_pose_v);
  result.dynamic=e.snapshot(result.pose);
  result.dynamic_equal=(result.dynamic.signature==support.signature);
  double best_length=0;
  std::vector<Eigen::Vector4d> predictors;std::vector<Eigen::Vector4d> eigs;std::vector<bool> spd;
  for(double h:kPredictorSteps) {
    Eigen::VectorXd x(6);x<<u,result.v;
    auto value=[&](const Eigen::VectorXd& z){return e.dd(z.head(2),z.tail(4),support);};
    const auto jet=numericalJet(x,h,value);const auto hvv=jet.H.bottomRightCorner(4,4);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(hvv);
    if(es.info()!=Eigen::Success)throw std::runtime_error("support-star Hvv eigensolver");
    const bool positive=hvv.allFinite()&&es.eigenvalues().minCoeff()>0;
    spd.push_back(positive);eigs.push_back(es.eigenvalues());
    if(positive)predictors.push_back(-hvv.ldlt().solve(jet.H.bottomLeftCorner(4,2)*ub));
    else predictors.push_back(Eigen::Vector4d::Constant(std::numeric_limits<double>::quiet_NaN()));
  }
  for(int first=0;first+2<static_cast<int>(kPredictorSteps.size());++first)
    for(int last=first+2;last<static_cast<int>(kPredictorSteps.size());++last) {
      bool stable=true;
      for(int i=first;i<=last;++i) {
        if(!spd[i]||!predictors[i].allFinite()){stable=false;break;}
        if(i<last) {
          const double dp=(predictors[i]-predictors[i+1]).norm()/
            std::max(1e-15,std::max(predictors[i].norm(),predictors[i+1].norm()));
          const double dh=((eigs[i]-eigs[i+1]).cwiseAbs().array()/
            eigs[i].cwiseAbs().cwiseMax(eigs[i+1].cwiseAbs()).array().max(1e-300)).maxCoeff();
          if(dp>.20||dh>.10){stable=false;break;}
        }
      }
      if(stable&&last-first+1>best_length){
        best_length=last-first+1;result.stable_first=first;result.stable_last=last;
        result.predictor=predictors[last];result.double_predictor_stable=true;
      }
    }
  return result;
}

struct SupportPairResult {
  bool equivalent=false;
  double dv=0,translation=0,rotation=0,energy_gap=0,eps_dv_pair=0,eps_e_pair=0;
  FixedSupportMinimum a,b;
};

bool numericallyEquivalentSupports(bool stationary_a,bool stationary_b,bool spd_a,bool spd_b,
    bool predictor_a,bool predictor_b,double delta_v,double eps_dv,double energy_gap,double eps_e) {
  return stationary_a&&stationary_b&&spd_a&&spd_b&&predictor_a&&predictor_b&&
    std::isfinite(delta_v)&&std::isfinite(eps_dv)&&std::isfinite(energy_gap)&&std::isfinite(eps_e)&&
    delta_v<=eps_dv&&energy_gap<=eps_e;
}

SupportPairResult evaluateSupportPair(DiagnosticEngine& e,const CsvRow& event,const CsvRow& first,
    const CsvRow& last,const GateSupport& a,const GateSupport& b,const Eigen::Vector2d& ub) {
  const Eigen::Vector2d u=parseVec(first.at(9),2);
  const Eigen::Vector4d v_initial=parseVec(first.at(10),4);
  const auto vA=parseVec(last.at(10),4);
  const auto A=solveFixedSupport(e,u,v_initial,ub,a),B=solveFixedSupport(e,u,v_initial,ub,b);
  SupportPairResult r;r.a=A;r.b=B;r.dv=(A.v-B.v).norm();
  r.translation=translationDistance(A.pose,B.pose);r.rotation=rotationDistanceDeg(A.pose,B.pose);
  const std::size_t termsA=std::accumulate(a.leaves.begin(),a.leaves.end(),std::size_t(0),
    [](std::size_t n,const std::vector<ExactPclNdt::Leaf>& leaves){return n+leaves.size();});
  const std::size_t termsB=std::accumulate(b.leaves.begin(),b.leaves.end(),std::size_t(0),
    [](std::size_t n,const std::vector<ExactPclNdt::Leaf>& leaves){return n+leaves.size();});
  const double energyA=A.reference.jet.energy,energyB=B.reference.jet.energy;
  r.energy_gap=std::abs(energyA-energyB);
  r.eps_dv_pair=std::max(A.reference.d_resolution,A.reference.fd_displacement_difference)+
                std::max(B.reference.d_resolution,B.reference.fd_displacement_difference);
  r.eps_e_pair=doubleEnergyResolution(energyA,termsA)+doubleEnergyResolution(energyB,termsB);
  const bool stationary=A.reference.status=="DOUBLE_RESOLUTION_STATIONARY"&&B.reference.status=="DOUBLE_RESOLUTION_STATIONARY";
  const bool spd=A.reference.jet.spd&&B.reference.jet.spd;
  const bool valid=A.double_predictor_stable&&B.double_predictor_stable;
  r.equivalent=numericallyEquivalentSupports(stationary,stationary,spd,spd,valid,valid,
    r.dv,r.eps_dv_pair,r.energy_gap,r.eps_e_pair);
  (void)vA;(void)event;
  return r;
}

void runSupportPairDiagnostics(char** argv,DiagnosticEngine& e,const CsvTable& events,
    const CsvTable& checks,const std::map<std::uint64_t,SupportRegistry>& registry,
    const std::map<std::pair<std::uint64_t,std::string>,CaseRequest>& requests) {
  const std::string outdir=argv[11];
  std::ofstream detail(outdir+"/support_pair_stationary.csv"),summary(outdir+"/support_equivalence.csv");
  if(!detail||!summary)throw std::runtime_error("cannot open support-pair outputs");
  detail.exceptions(std::ios::badbit|std::ios::failbit);summary.exceptions(std::ios::badbit|std::ios::failbit);
  csvHeader(detail,{"tx","cluster","hypothesis","alpha","round","before_hash","after_hash","changed_fraction",
    "u","v_common_initial","v_recorded_endpoint","E_A_common","E_B_common","E_dynamic_common",
    "vA_star","vB_star","status_A","status_B","Hvv_spd_A","Hvv_spd_B","double_predictor_stable_A",
    "double_predictor_stable_B","stable_h_A","stable_h_B","Delta_v_star","translation_separation_m",
    "rotation_separation_deg","E_A_star","E_B_star","energy_gap","EPS_DV_A","EPS_DV_B","EPS_DV_PAIR",
    "EPS_E_A","EPS_E_B","EPS_E_PAIR","dynamic_energy_A_star","dynamic_energy_B_star",
    "dynamic_support_equal_A","dynamic_support_equal_B","common_pose_dynamic_hash","runtime_ms","double_evaluations"});
  csvHeader(summary,{"tx","cluster","hypothesis","alpha","round","support_A","support_B","support_change_fraction",
    "stationary_A","stationary_B","Hvv_SPD_A","Hvv_SPD_B","double_predictor_valid_A","double_predictor_valid_B",
    "Delta_v","EPS_DV_PAIR","energy_gap","EPS_E_PAIR","numerically_equivalent","support_self_consistent_A",
    "support_self_consistent_B"});
  int pair_index=0;
  for(const auto& event:events.rows) {
    const auto tx=parseU64(events.get(event,"tx"));const std::string cluster=events.get(event,"cluster");
    const std::string hyp=events.get(event,"hypothesis");const double alpha=std::stod(events.get(event,"alpha"));
    const bool root3341=tx==3341&&cluster=="P02"&&hyp=="ROOT"&&closeAlpha(events.get(event,"alpha"),0.0);
    const bool boundary616=tx==616&&cluster=="P10"&&(hyp=="OLD_SUPPORT"||hyp=="NEW_SUPPORT")&&closeAlpha(events.get(event,"alpha"),.68625);
    if(!root3341&&!boundary616)continue;
    const CsvRow* first=findCheck(checks,event,true);const CsvRow* last=findCheck(checks,event,false);
    if(!first||!last)throw std::runtime_error("support-pair input checks missing");
    const auto ha=parseU64(events.get(event,"before_hash")),hb=parseU64(events.get(event,"after_hash"));
    const auto txreg=registry.find(tx);
    if(txreg==registry.end()||!txreg->second.count(ha)||!txreg->second.count(hb))
      throw std::runtime_error("support pair not reconstructible from existing nodes/events");
    e.select(tx,cluster);const auto& a=txreg->second.at(ha);const auto& b=txreg->second.at(hb);
    const Eigen::Vector2d u=parseVec(checks.get(*first,"u"),2);
    const Eigen::Vector4d v0=parseVec(checks.get(*first,"v"),4),vend=parseVec(checks.get(*last,"v"),4);
    const auto start=std::chrono::steady_clock::now();const auto eval_before=e.double_evals;
    const auto res=evaluateSupportPair(e,event,*first,*last,a,b,requests.at({tx,cluster}).ub);
    const double runtime=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    const double ea0=e.dd(u,v0,a),eb0=e.dd(u,v0,b);const auto dyn0=e.snapshot(e.pose(u,v0));
    const auto termsA=std::accumulate(a.leaves.begin(),a.leaves.end(),std::size_t(0),
      [](std::size_t n,const std::vector<ExactPclNdt::Leaf>& ls){return n+ls.size();});
    const auto termsB=std::accumulate(b.leaves.begin(),b.leaves.end(),std::size_t(0),
      [](std::size_t n,const std::vector<ExactPclNdt::Leaf>& ls){return n+ls.size();});
    const double epsA=std::max(res.a.reference.d_resolution,res.a.reference.fd_displacement_difference);
    const double epsB=std::max(res.b.reference.d_resolution,res.b.reference.fd_displacement_difference);
    const double energyA=res.a.reference.rho,energyB=res.b.reference.rho;
    const double gap=std::abs(res.a.reference.jet.energy-res.b.reference.jet.energy);
    const bool statA=res.a.reference.status=="DOUBLE_RESOLUTION_STATIONARY";
    const bool statB=res.b.reference.status=="DOUBLE_RESOLUTION_STATIONARY";
    const double change=numberOr(events.get(event,"changed_fraction"));
    const auto hAfirst=res.a.stable_first<0?std::string():number(kPredictorSteps[res.a.stable_first]);
    const auto hAlast=res.a.stable_last<0?std::string():number(kPredictorSteps[res.a.stable_last]);
    const auto hBfirst=res.b.stable_first<0?std::string():number(kPredictorSteps[res.b.stable_first]);
    const auto hBlast=res.b.stable_last<0?std::string():number(kPredictorSteps[res.b.stable_last]);
    const Eigen::Vector2d urow=u;const Eigen::Vector4d vrow=v0;
    const double dynA=res.a.dynamic.energy,dynB=res.b.dynamic.energy;
    writeRow(detail,{std::to_string(tx),cluster,hyp,number(alpha),events.get(event,"round"),std::to_string(ha),std::to_string(hb),
      number(change),vecText(urow),vecText(vrow),vecText(vend),number(ea0),number(eb0),number(dyn0.energy),
      vecText(res.a.v),vecText(res.b.v),res.a.reference.status,res.b.reference.status,
      res.a.reference.jet.spd?"1":"0",res.b.reference.jet.spd?"1":"0",res.a.double_predictor_stable?"1":"0",
      res.b.double_predictor_stable?"1":"0",hAfirst+";"+hAlast,hBfirst+";"+hBlast,number(res.dv),number(res.translation),
      number(res.rotation),number(res.a.reference.jet.energy),number(res.b.reference.jet.energy),number(gap),number(epsA),number(epsB),
      number(res.eps_dv_pair),number(energyA),number(energyB),number(res.eps_e_pair),number(dynA),number(dynB),
      res.a.dynamic_equal?"1":"0",res.b.dynamic_equal?"1":"0",std::to_string(supportSignature(dyn0.signature)),
      number(runtime),std::to_string(e.double_evals-eval_before)});
    writeRow(summary,{std::to_string(tx),cluster,hyp,number(alpha),events.get(event,"round"),std::to_string(ha),std::to_string(hb),
      number(change),statA?"1":"0",statB?"1":"0",res.a.reference.jet.spd?"1":"0",res.b.reference.jet.spd?"1":"0",
      res.a.double_predictor_stable?"1":"0",res.b.double_predictor_stable?"1":"0",number(res.dv),number(res.eps_dv_pair),
      number(gap),number(res.eps_e_pair),res.equivalent?"1":"0",res.a.dynamic_equal?"1":"0",res.b.dynamic_equal?"1":"0"});
    if(++pair_index%2==0)std::cout<<"R1C3A_SUPPORT_PAIR "<<tx<<"/"<<cluster<<"/"<<hyp<<" round="<<events.get(event,"round")
      <<" status="<<res.a.reference.status<<"/"<<res.b.reference.status<<" equivalent="<<res.equivalent<<std::endl;
  }
  detail.flush();summary.flush();
}

}

int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!gateSelfTest())throw std::runtime_error("R1C3 prerequisite self-test failed");
      const Eigen::Vector2d ub(.3,-.2);const Eigen::Matrix<double,4,2> cross=Eigen::Matrix<double,4,2>::Constant(.1);
      const Eigen::Vector4d p=-Eigen::Matrix4d::Identity().ldlt().solve(cross*ub);
      if((p+cross*ub).norm()>1e-12)throw std::runtime_error("predictor equation self-test");
      std::vector<PredictorRow> rows(6);
      for(std::size_t i=0;i<rows.size();++i){rows[i].h=kPredictorSteps[i];rows[i].hvvf=rows[i].hvvd=true;
        rows[i].pf=Eigen::Vector4d::Ones();rows[i].pd=rows[i].pf;rows[i].eigd=Eigen::Vector4d::Ones();}
      if(!assessPredictor(rows,.005,1e-10).valid)throw std::runtime_error("predictor multi-h self-test");
      for(auto& row:rows)row.pf(0)+=.1;
      if(assessPredictor(rows,.005,1e-10).valid)throw std::runtime_error("predictor numeric-envelope gate self-test");
      if(!numericallyEquivalentSupports(true,true,true,true,true,true,1e-8,2e-8,1e-9,2e-9)||
         numericallyEquivalentSupports(true,true,true,true,true,true,3e-8,2e-8,1e-9,2e-9)||
         numericallyEquivalentSupports(true,false,true,true,true,true,1e-8,2e-8,1e-9,2e-9))
        throw std::runtime_error("support-equivalence contract self-test");
      std::cout<<"P9_R1C3A_SELF_TEST=PASS\n";return 0;
    }
    if(argc==12&&std::string(argv[1])=="--run"){runDiagnostics(argv);return 0;}
    throw std::runtime_error("--self-test | --run MAP COHORT UOBS REQUESTS ROOTS CHECKS EVENTS NODES DIRECTIONAL OUTDIR");
  } catch(const std::exception& ex) {
    std::cerr<<"R1C3A_ERROR "<<ex.what()<<'\n';return 1;
  }
}
