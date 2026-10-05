// Offline R1B orchestration; all scores, charts and solvers are reused unchanged.
#define P9_TRUE_PROFILE_CLOSURE_LIBRARY
#include "p9_true_profile_closure.cpp"

namespace {
struct AttractorTarget {
  std::uint64_t tx;
  std::string cluster;
  Eigen::Matrix4f closed;
  Eigen::VectorXd u, v;
};

std::map<CandidateKey,AttractorTarget> attractorTargets(const std::string& path) {
  const CsvTable table=readCsv(path);
  const std::set<CandidateKey> expected={{616,"P02"},{616,"P03"},{616,"P10"},
    {616,"P12"},{616,"P13"},{616,"P17"},{2226,"P09"}};
  std::map<CandidateKey,AttractorTarget> result;
  for (const auto& row:table.rows) {
    if (table.get(row,"group_a")!="1") continue;
    AttractorTarget item;
    item.tx=parseU64(table.get(row,"transaction_id"));item.cluster=table.get(row,"cluster_id");
    item.closed=parseMatrix16(table.get(row,"closed_pose_matrix16"));
    item.u=parseVectorText(table.get(row,"u_b"));item.v=parseVectorText(table.get(row,"v_b"));
    if (!expected.count({item.tx,item.cluster}) || !result.emplace(CandidateKey(item.tx,item.cluster),item).second)
      throw std::runtime_error("R1B unexpected/duplicate GROUP A");
  }
  if (result.size()!=7) throw std::runtime_error("R1B requires all seven frozen targets");
  return result;
}

struct AttractorEngine {
  Cloud::Ptr target;
  ExactPclNdt ndt;
  std::map<std::uint64_t,FrameSource> frames;
  std::map<std::uint64_t,ArchivedUObs> observations;
  std::map<CandidateKey,AttractorTarget> basins;
  FrameContext context;
  ExactPclNdt::FrozenSupport frozen_t0,frozen_canonical;
  Support canonical_support;

  AttractorEngine(const std::string& map,const std::string& cohort,
      const std::string& uobs,const std::string& canonical)
      :target(loadTarget(map)),frames(readFrameSources(cohort)),
       observations(readArchivedUObs(uobs)),basins(attractorTargets(canonical)) {
    if (target->size()!=549606) throw std::runtime_error("R1B target count mismatch");
    configureNdt(ndt,target);
  }
  void select(const AttractorTarget& basin) {
    context=frameContext(frames.at(basin.tx),observations.at(basin.tx),ndt,target->size());
    if (basin.u.size()!=context.k || basin.v.size()!=context.strong.cols() ||
        (mapChartDisplacement(context.obs.pose,basin.closed)-
          (context.weak*basin.u+context.strong*basin.v)).norm()>1e-6)
      throw std::runtime_error("R1B frozen product chart mismatch");
    ndt.scoreJet(context.source,context.obs.pose,nullptr,nullptr,nullptr,&frozen_t0,false);
    ndt.scoreJet(context.source,basin.closed,nullptr,nullptr,&canonical_support,&frozen_canonical,false);
  }
  Eigen::Matrix4f pose(const AttractorTarget& basin,const Eigen::VectorXd& v) const {
    return poseAtEta(context.obs.pose,context.weak*basin.u+context.strong*v);
  }
  double dynamic(const Eigen::Matrix4f& p,Support* support=nullptr) {
    return -ndt.dynamicValueOnly(context.source,p,support)/context.source->size();
  }
  double frozen(const Eigen::Matrix4f& p,bool canonical) const {
    return -ndt.frozenScore(context.source,p,canonical?frozen_canonical:frozen_t0)/context.source->size();
  }
};

void sampleAttractors(AttractorEngine& engine,const std::string& requests_path,const std::string& prefix) {
  const CsvTable requests=readCsv(requests_path);
  std::ofstream out(prefix+".csv"),trace(prefix+"_trace.csv");
  if (!out || !trace) throw std::runtime_error("cannot open R1B sample outputs");
  writeHeader(out,{"request_id","transaction_id","cluster_id","stage","solver","objective","beta",
    "local_d","axis","sign","u_b","v_b","initial_v","endpoint_v","endpoint_pose_matrix16",
    "initial_energy","endpoint_objective_energy","initial_dynamic_energy","endpoint_dynamic_energy",
    "endpoint_frozen_t0_energy","endpoint_frozen_canonical_energy","closed_dynamic_energy",
    "initial_support_hash","endpoint_support_hash","initial_support_vs_T0","endpoint_support_vs_T0",
    "endpoint_support_vs_canonical","endpoint_distance_translation_m","endpoint_distance_rotation_deg",
    "energy_evaluations","strong_iterations","accepted_steps","solver_status","runtime_ms",
    "diagnostic_dynamic_evaluations"});
  writeHeader(trace,{"request_id","transaction_id","cluster_id","accepted_move","iteration","evaluations",
    "v","objective_energy","dynamic_energy","objective_support_hash","dynamic_support_hash",
    "dynamic_support_vs_previous","dynamic_support_vs_T0","branch_gradient_before_step","scale"});
  std::set<std::string> ids;
  CandidateKey current{0,""};
  for (const auto& row:requests.rows) {
    const std::string id=requests.get(row,"request_id");
    if (!ids.insert(id).second) throw std::runtime_error("duplicate R1B request");
    const CandidateKey key{parseU64(requests.get(row,"transaction_id")),requests.get(row,"cluster_id")};
    const auto& basin=engine.basins.at(key);
    if (key!=current) {engine.select(basin);current=key;}
    const std::string method=requests.get(row,"solver"),objective=requests.get(row,"objective");
    const Eigen::VectorXd initial=parseVectorText(requests.get(row,"initial_v"));
    if (initial.size()!=basin.v.size()) throw std::runtime_error("strong initialization size mismatch");
    StrongSolution solution;
    if (method=="NEWTON" && objective=="DYNAMIC")
      solution=iterativeNewton(engine.ndt,engine.context,basin.u,initial);
    else if (method=="PATTERN") {
      if (objective!="DYNAMIC" && objective!="FROZEN_T0" && objective!="FROZEN_CANONICAL")
        throw std::runtime_error("invalid pattern objective");
      solution=patternSearch(initial,[&](const Eigen::VectorXd& v,Support& support) {
        const auto pose=engine.pose(basin,v);
        if (objective=="DYNAMIC") return engine.dynamic(pose,&support);
        support=objective=="FROZEN_T0"?engine.context.nominal_support:engine.canonical_support;
        return engine.frozen(pose,objective=="FROZEN_CANONICAL");
      },engine.context);
    } else throw std::runtime_error("unsupported R1B solver/objective combination");
    if (solution.trace.size()!=static_cast<std::size_t>(solution.accepted_steps+1))
      throw std::runtime_error("incomplete strong accepted-event trace");
    const auto endpoint=engine.pose(basin,solution.v);
    Support si,se;
    const double initial_dynamic=engine.dynamic(engine.pose(basin,initial),&si);
    const double endpoint_dynamic=engine.dynamic(endpoint,&se);
    const double et0=engine.frozen(endpoint,false),ec=engine.frozen(endpoint,true);
    const double check=objective=="DYNAMIC"?endpoint_dynamic:(objective=="FROZEN_T0"?et0:ec);
    if (std::abs(check-solution.energy)>1e-10) throw std::runtime_error("R1B endpoint objective mismatch");
    Support previous;
    for (const auto& event:solution.trace) {
      Support support;
      const double energy=engine.dynamic(engine.pose(basin,event.v),&support);
      const Support& fixed=objective=="FROZEN_T0"?engine.context.nominal_support:engine.canonical_support;
      const auto objective_hash=objective=="DYNAMIC"?supportSignature(support):supportSignature(fixed);
      writeRow(trace,{id,std::to_string(basin.tx),basin.cluster,std::to_string(event.accepted_move),
        std::to_string(event.iteration),std::to_string(event.evaluations),vectorText(event.v),number(event.energy),
        number(energy),std::to_string(objective_hash),std::to_string(supportSignature(support)),
        number(previous.empty()?0:supportFraction(previous,support)),
        number(supportFraction(engine.context.nominal_support,support)),
        number(event.projected_gradient_before_step),number(event.scale)});
      previous=std::move(support);
    }
    writeRow(out,{id,std::to_string(basin.tx),basin.cluster,requests.get(row,"stage"),method,objective,
      requests.get(row,"beta"),requests.get(row,"local_d"),requests.get(row,"axis"),requests.get(row,"sign"),
      vectorText(basin.u),vectorText(basin.v),vectorText(initial),vectorText(solution.v),poseText(endpoint),
      number(solution.initial_energy),number(solution.energy),number(initial_dynamic),number(endpoint_dynamic),
      number(et0),number(ec),number(engine.dynamic(basin.closed)),std::to_string(supportSignature(si)),
      std::to_string(supportSignature(se)),number(supportFraction(engine.context.nominal_support,si)),
      number(supportFraction(engine.context.nominal_support,se)),number(supportFraction(engine.canonical_support,se)),
      number(translationDistance(endpoint,basin.closed)),number(rotationDistanceDeg(endpoint,basin.closed)),
      std::to_string(solution.evaluations),std::to_string(solution.iterations),std::to_string(solution.accepted_steps),
      solution.status,number(solution.runtime_ms),std::to_string(3+solution.trace.size())});
    out.flush();trace.flush();
    std::cout<<"R1B_SAMPLE "<<id<<" "<<solution.status<<" E="<<solution.initial_energy<<"->"<<solution.energy<<std::endl;
  }
}

void scanSupportPaths(AttractorEngine& engine,const std::string& path) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open support path");
  writeHeader(out,{"transaction_id","cluster_id","beta","v","pose_matrix16","dynamic_energy",
    "frozen_t0_energy","frozen_canonical_energy","support_hash","support_vs_previous","support_vs_T0",
    "support_vs_canonical","point_count"});
  for (const auto& pair:engine.basins) {
    const auto& basin=pair.second;engine.select(basin);
    Support previous;
    for (int i=0;i<=100;++i) {
      const double beta=i/100.0;
      const Eigen::VectorXd v=beta*basin.v;
      const auto pose=engine.pose(basin,v);
      Support support;
      const double energy=engine.dynamic(pose,&support);
      writeRow(out,{std::to_string(basin.tx),basin.cluster,number(beta),vectorText(v),poseText(pose),
        number(energy),number(engine.frozen(pose,false)),number(engine.frozen(pose,true)),
        std::to_string(supportSignature(support)),number(previous.empty()?0:supportFraction(previous,support)),
        number(supportFraction(engine.context.nominal_support,support)),
        number(supportFraction(engine.canonical_support,support)),std::to_string(engine.context.source->size())});
      previous=std::move(support);
    }
    out.flush();std::cout<<"R1B_SUPPORT_PATH "<<basin.tx<<"/"<<basin.cluster<<std::endl;
  }
}

void refineBranches(AttractorEngine& engine,const std::string& requests_path,const std::string& path) {
  const CsvTable requests=readCsv(requests_path);
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open branch refine output");
  writeHeader(out,{"transaction_id","cluster_id","strong_branch_id","representative_request_id",
    "pre_pose_matrix16","pre_dynamic_energy","pre_frozen_t0_energy","pre_frozen_canonical_energy",
    "pre_support_hash","pre_distance_translation_m","pre_distance_rotation_deg","pre_canonical",
    "post_pose_matrix16","post_dynamic_energy","post_distance_translation_m","post_distance_rotation_deg",
    "post_canonical","full_refine_basin_escape","iterations","pcl_converged","status",
    "post_support_hash","support_change","runtime_ms"});
  std::set<std::tuple<std::uint64_t,std::string,std::string>> unique;
  CandidateKey current{0,""};
  for (const auto& row:requests.rows) {
    const CandidateKey key{parseU64(requests.get(row,"transaction_id")),requests.get(row,"cluster_id")};
    const auto& basin=engine.basins.at(key);
    const std::string branch=requests.get(row,"strong_branch_id");
    if (!unique.emplace(key.first,key.second,branch).second) throw std::runtime_error("duplicate branch full refine");
    if (key!=current) {engine.select(basin);current=key;}
    const auto pre=parseMatrix16(requests.get(row,"endpoint_pose_matrix16"));
    const auto reconstructed=engine.pose(basin,parseVectorText(requests.get(row,"endpoint_v")));
    if ((pre-reconstructed).norm()>1e-7) throw std::runtime_error("full refine seed not an actual strong endpoint");
    Support sp,sq;
    const double ep=engine.dynamic(pre,&sp);
    const double ft=engine.frozen(pre,false),fc=engine.frozen(pre,true);
    Cloud aligned;
    const auto start=std::chrono::steady_clock::now();
    engine.ndt.align(aligned,pre);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    const auto post=engine.ndt.getFinalTransformation();
    if (!post.allFinite()) throw std::runtime_error("branch full refine nonfinite");
    const double eq=engine.dynamic(post,&sq);
    const bool a=sameBasin(pre,basin.closed),b=sameBasin(post,basin.closed);
    writeRow(out,{std::to_string(basin.tx),basin.cluster,branch,requests.get(row,"request_id"),poseText(pre),
      number(ep),number(ft),number(fc),std::to_string(supportSignature(sp)),
      number(translationDistance(pre,basin.closed)),number(rotationDistanceDeg(pre,basin.closed)),a?"1":"0",
      poseText(post),number(eq),number(translationDistance(post,basin.closed)),number(rotationDistanceDeg(post,basin.closed)),
      b?"1":"0",a&&!b?"1":"0",std::to_string(engine.ndt.getFinalNumIteration()),
      engine.ndt.hasConverged()?"1":"0",ndtStatus(engine.ndt),std::to_string(supportSignature(sq)),
      number(supportFraction(sp,sq)),number(ms)});
    out.flush();std::cout<<"R1B_FULL_REFINE "<<basin.tx<<"/"<<basin.cluster<<" "<<branch
      <<" escape="<<(a&&!b)<<std::endl;
  }
}
} // namespace

#ifndef P9_STRONG_ATTRACTOR_LIBRARY
int main(int argc,char** argv) {
  try {
    if (argc==2 && std::string(argv[1])=="--self-test") {
      if (!runClosureSelfTests()) throw std::runtime_error("R1B reuse self-test failed");
      std::cout<<"P9_R1B_REUSE_SELF_TEST=PASS\n";return 0;
    }
    if (argc==8 && (std::string(argv[1])=="--sample" || std::string(argv[1])=="--refine")) {
      AttractorEngine engine(argv[2],argv[3],argv[4],argv[5]);
      if (std::string(argv[1])=="--sample") sampleAttractors(engine,argv[6],argv[7]);
      else refineBranches(engine,argv[6],argv[7]);
      return 0;
    }
    if (argc==7 && std::string(argv[1])=="--path") {
      AttractorEngine engine(argv[2],argv[3],argv[4],argv[5]);scanSupportPaths(engine,argv[6]);return 0;
    }
    std::cerr<<"usage: --self-test | --sample/--refine MAP COHORT UOBS CANONICAL REQUESTS OUTPUT | --path MAP COHORT UOBS CANONICAL OUTPUT\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr<<"P9_R1B_ERROR: "<<error.what()<<'\n';return 1;
  }
}
#endif
