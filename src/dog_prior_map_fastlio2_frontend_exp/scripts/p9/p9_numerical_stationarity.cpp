#define P9_STRONG_ATTRACTOR_LIBRARY
// Independent R1C2 diagnostic entry point; never dispatches an inherited align.
#include "p9_strong_attractor_closure.cpp"
#include "p9_stationarity_numerics.hpp"

namespace {
using Row=std::vector<std::string>;
const std::vector<double> steps={.004,.002,.001,.0005,.00025,.000125};

std::map<std::uint64_t,Support> archivedSupports(const std::string& path) {
  const auto table=readCsv(path);std::map<std::uint64_t,Support> result;
  for(const auto& row:table.rows) {
    const auto hash=parseU64(table.get(row,"support_hash"));const auto index=parseU64(table.get(row,"point_index"));
    auto& s=result[hash];if(index!=s.size())throw std::runtime_error("support archive order/cardinality");
    PointSupport cells;
    for(const auto& cell:split(table.get(row,"cell_bits_xyz"),';')) {
      const auto xyz=split(cell,':');if(xyz.size()!=3)throw std::runtime_error("support cell shape");
      cells.push_back({{parseU64(xyz[0]),parseU64(xyz[1]),parseU64(xyz[2])}});
    }
    s.push_back(cells);
  }
  for(const auto& item:result)if(supportSignature(item.second)!=item.first)throw std::runtime_error("archived membership hash mismatch");
  return result;
}

struct Diagnostic {
  DoubleFrozenNdt ndt;
  Cloud::Ptr target;
  std::map<std::uint64_t,FrameSource> frames;
  std::map<std::uint64_t,ArchivedUObs> obs;
  std::map<CandidateKey,AttractorTarget> basins;
  FrameContext ctx;
  Diagnostic(const std::string& map,const std::string& cohort,const std::string& observations,const std::string& canonical)
    :target(loadTarget(map)),frames(readFrameSources(cohort)),obs(readArchivedUObs(observations)),basins(attractorTargets(canonical)) {
    if(target->size()!=549606)throw std::runtime_error("target cardinality changed");configureNdt(ndt,target);
  }
  Eigen::Matrix4f pf(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const {
    return poseAtEta(ctx.obs.pose,ctx.weak*u+ctx.strong*v);
  }
  Eigen::Matrix4d pd(const Eigen::VectorXd& u,const Eigen::VectorXd& v)const {
    return continuousPose(ctx.obs.pose,ctx.weak*u+ctx.strong*v);
  }
};

void jetRow(std::ofstream& out,const std::string& id,const std::string& stage,double h,NumericJet j) {
  completeJet(j);
  writeRow(out,{id,stage,number(h),number(j.energy),vectorText(j.g),number(j.g.norm()),
    vectorText(j.d),number(j.d.norm()),number(j.decrement),number(2*j.decrement),
    vectorText(j.eig),j.spd?"1":"0",number((j.H-j.H.transpose()).norm()),vectorText(Eigen::Map<Eigen::VectorXd>(j.H.data(),j.H.size()))});
}

void runDiagnostic(int argc,char** argv) {
  if(argc!=9)throw std::runtime_error("--run MAP COHORT UOBS CANONICAL R1C_DIRECTORY OUTPUT_DIRECTORY");
  Diagnostic e(argv[2],argv[3],argv[4],argv[5]);const std::string previous=argv[6],out=argv[7];
  // Last argument is an explicit protocol version, avoiding silent old invocation.
  if(std::string(argv[8])!="R1C2_V1")throw std::runtime_error("protocol version");
  const auto roots=readCsv(previous+"/branch_certificates.csv"),rounds=readCsv(previous+"/corrector_rounds.csv");
  if(roots.rows.size()!=14)throw std::runtime_error("exactly 14 prescribed roots required");
  const auto archive=archivedSupports(previous+"/support_memberships.csv");
  std::ofstream mf(out+"/multih_float.csv"),md(out+"/multih_double.csv"),parity(out+"/float_double_parity.csv"),
    reference(out+"/double_stationary_endpoints.csv"),metadata(out+"/root_metadata.csv"),
    directional(out+"/directional_fd.csv"),trace(out+"/reference_multih.csv");
  if(!mf||!md||!parity||!reference||!metadata||!directional||!trace)throw std::runtime_error("cannot open diagnostic outputs");
  const std::vector<std::string> jfields={"root_id","stage","h","energy","gradient","gradient_norm","newton_displacement","displacement_norm","predicted_decrement","newton_decrement_squared","H_eigenvalues","spd","asymmetry","H_columnmajor"};
  writeHeader(mf,jfields);writeHeader(md,jfields);writeHeader(trace,jfields);
  writeHeader(parity,{"root_id","probe","source_count","float_energy","double_energy","mean_gap","sum_gap","point_count","guard_disagreements"});
  writeHeader(metadata,{"root_id","independent_id","tx","cluster","direction","u","v","frozen_support_hash","actual_support_hash","support_equal","terms","old_gradient_norm","old_pass","reproduced_gradient_error","reproduced_H_error","reproduced_energy_error","original_pose"});
  writeHeader(reference,{"root_id","status","iterations","accepted_steps","root_v","star_v","delta_v_norm","translation_m","rotation_deg","energy_before","energy_after","energy_decrease","g_double","d_double","predicted_decrement","rho_E","d_resolution","fd_displacement_difference","fd_eigen_relative","star_float_g","star_float_d","star_float_decrement","star_float_H_eigen","star_support_equal","star_support_hash","star_dynamic_energy","star_float_double_translation_m","star_float_double_matrix_norm","star_pose_double","coarse_d","coarse_decrement","coarse_min_eig","no_tested_newton_descent"});
  writeHeader(directional,{"root_id","origin","precision","h","direction","gradient_error","gradient_relative","curvature_error","curvature_relative","pass"});
  std::set<std::string> root_ids;
  for(const auto& row:roots.rows) {
    const auto tx=parseU64(roots.get(row,"tx"));const auto cluster=roots.get(row,"cluster"),dir=roots.get(row,"direction");
    const std::string id=std::to_string(tx)+"/"+cluster+"/"+dir;
    if(!root_ids.insert(id).second||!e.basins.count({tx,cluster})||(dir!="FORWARD"&&dir!="REVERSE"))throw std::runtime_error("root identity");
    e.ctx=frameContext(e.frames.at(tx),e.obs.at(tx),e.ndt,e.target->size());
    const auto& b=e.basins.at({tx,cluster});const auto u=parseVectorText(roots.get(row,"u")),v=parseVectorText(roots.get(row,"v"));
    if(u.size()!=2||v.size()!=4)throw std::runtime_error("root chart dimension");
    const Eigen::VectorXd expected_u=dir=="FORWARD"?Eigen::VectorXd::Zero(2).eval():b.u;
    if((u-expected_u).norm()>1e-12)throw std::runtime_error("root weak coordinate mismatch");
    ExactPclNdt::FrozenSupport fixed;Support before;
    Eigen::Matrix4f capture=dir=="FORWARD"?e.ctx.obs.pose:b.closed;
    int count=0;
    for(const auto& round:rounds.rows) {
      if(rounds.get(round,"tx")!=std::to_string(tx)||rounds.get(round,"cluster")!=cluster||rounds.get(round,"direction")!=dir)continue;
      if(std::stoi(rounds.get(round,"round"))!=++count)throw std::runtime_error("support round order");
      e.ndt.dynamicValueOnly(e.ctx.source,capture,&before,&fixed);
      const auto expected=parseU64(rounds.get(round,"before_hash"));
      if(supportSignature(before)!=expected||before!=archive.at(expected))throw std::runtime_error("frozen support replay mismatch");
      capture=e.pf(u,parseVectorText(rounds.get(round,"endpoint_v")));
    }
    if(count!=std::stoi(roots.get(row,"outer_iterations"))||supportSignature(before)!=parseU64(roots.get(row,"frozen_support_hash")))throw std::runtime_error("wrong final derivative support");
    if((e.pf(u,v)-parseMatrix16(roots.get(row,"pose_matrix16"))).norm()!=0)throw std::runtime_error("original pose not reproduced");
    Support actual;e.ndt.dynamicValueOnly(e.ctx.source,e.pf(u,v),&actual);
    if(actual!=archive.at(parseU64(roots.get(row,"support_hash"))))throw std::runtime_error("actual support mismatch");
    std::size_t terms=0;for(const auto& leaves:fixed)terms+=leaves.size();
    const auto ff=[&](const Eigen::VectorXd& x){return -e.ndt.frozenScore(e.ctx.source,e.pf(u,x),fixed)/e.ctx.source->size();};
    const auto fd=[&](const Eigen::VectorXd& x){return -e.ndt.frozenScoreDouble(e.ctx.source,e.pd(u,x),fixed)/e.ctx.source->size();};
    auto old=numericalJet(v,.001,ff);completeJet(old);
    const auto archived_g=parseVectorText(roots.get(row,"strong_gradient")),archived_H=parseVectorText(roots.get(row,"Hvv_matrix16"));
    Eigen::Matrix4d old_H;for(int a=0;a<4;++a)for(int c=0;c<4;++c)old_H(a,c)=archived_H(4*a+c);
    const double g_error=(old.g-archived_g).norm(),H_error=(old.H-old_H).norm();
    const double energy_error=std::abs(old.energy-std::stod(roots.get(row,"frozen_energy")));
    if(g_error>1e-10||H_error>1e-7||energy_error>1e-12)throw std::runtime_error("old derivative/energy not reproduced");
    writeRow(metadata,{id,dir=="FORWARD"?std::to_string(tx)+"/SHARED_FORWARD":id,std::to_string(tx),cluster,dir,vectorText(u),vectorText(v),std::to_string(supportSignature(before)),std::to_string(supportSignature(actual)),before==actual?"1":"0",std::to_string(terms),number(old.g.norm()),roots.get(row,"certified"),number(g_error),number(H_error),number(energy_error),poseText(e.pf(u,v))});
    for(double h:steps){jetRow(mf,id,"ROOT",h,numericalJet(v,h,ff));jetRow(md,id,"ROOT",h,numericalJet(v,h,fd));}
    for(int probe=-1;probe<8;++probe) {
      Eigen::VectorXd x=v;if(probe>=0)x(probe/2)+=(probe%2?1:-1)*.001;
      const double a=ff(x),d=fd(x),gap=std::abs(a-d);
      writeRow(parity,{id,std::to_string(probe),std::to_string(e.ctx.source->size()),number(a),number(d),number(gap),number(gap*e.ctx.source->size()),std::to_string(e.ctx.source->size()),std::to_string(e.ndt.guardDisagreement(e.ctx.source,e.pf(u,x),e.pd(u,x),fixed))});
    }
    const auto star=referenceMinimum(v,terms,fd);auto sf=numericalJet(star.v,.001,ff);completeJet(sf);
    Support star_actual;const auto starpf=e.pf(u,star.v);const auto starpd=e.pd(u,star.v);
    const double dynamic=-e.ndt.dynamicValueOnly(e.ctx.source,starpf,&star_actual)/e.ctx.source->size();
    const Vector6d delta=e.ctx.strong*(star.v-v);
    const Vector6d root_eta=e.ctx.weak*u+e.ctx.strong*v,star_eta=e.ctx.weak*u+e.ctx.strong*star.v;
    const Eigen::Matrix3d relative_rotation=expRotation(star_eta.tail<3>())*expRotation(root_eta.tail<3>()).transpose();
    const double rotation=Eigen::AngleAxisd(relative_rotation).angle()*180/kPi;
    std::ostringstream doublepose;doublepose<<std::setprecision(17);
    for(int a=0;a<4;++a)for(int c=0;c<4;++c){if(a||c)doublepose<<';';doublepose<<starpd(a,c);}
    bool no_descent=true;for(int line=0;line<16;++line)if(fd(star.v+std::ldexp(1.0,-line)*star.jet.d)<star.jet.energy)no_descent=false;
    writeRow(reference,{id,star.status,std::to_string(star.iterations),std::to_string(star.accepted),vectorText(v),vectorText(star.v),number((star.v-v).norm()),number(kResolution*delta.head<3>().norm()),number(rotation),number(fd(v)),number(fd(star.v)),number(fd(v)-fd(star.v)),number(star.jet.g.norm()),number(star.jet.d.norm()),number(star.jet.decrement),number(star.rho),number(star.d_resolution),number(star.fd_displacement_difference),number(star.fd_eigen_relative),number(sf.g.norm()),number(sf.d.norm()),number(sf.decrement),vectorText(sf.eig),star_actual==before?"1":"0",std::to_string(supportSignature(star_actual)),number(dynamic),number((starpf.block<3,1>(0,3).cast<double>()-starpd.block<3,1>(0,3)).norm()),number((starpf.cast<double>()-starpd).norm()),doublepose.str(),number(star.coarse_d),number(star.coarse_decrement),number(star.coarse_min_eig),no_descent?"1":"0"});
    for(double h:steps){jetRow(trace,id,"STAR_FLOAT",h,numericalJet(star.v,h,ff));jetRow(trace,id,"STAR_DOUBLE",h,numericalJet(star.v,h,fd));}
    // Independent directions use original T0/canonical, not recentered endpoints.
    const auto origin_pose=dir=="FORWARD"?e.ctx.obs.pose:b.closed;
    Support os;ExactPclNdt::FrozenSupport ol;e.ndt.dynamicValueOnly(e.ctx.source,origin_pose,&os,&ol);
    Eigen::VectorXd origin_x(6);origin_x<<expected_u,(dir=="FORWARD"?Eigen::VectorXd::Zero(4).eval():b.v);
    for(const std::string stage:{"ORIGINAL","ENDPOINT"})for(const std::string precision:{"FLOAT","DOUBLE"})for(double h:steps) {
      Eigen::VectorXd x=origin_x;if(stage=="ENDPOINT")x<<u,v;
      const auto& selected_support=stage=="ENDPOINT"?fixed:ol;
      auto val=[&](const Eigen::VectorXd& x){const Vector6d eta=e.ctx.weak*x.head(2)+e.ctx.strong*x.tail(4);
        return precision=="FLOAT"?-e.ndt.frozenScore(e.ctx.source,poseAtEta(e.ctx.obs.pose,eta),selected_support)/e.ctx.source->size():-e.ndt.frozenScoreDouble(e.ctx.source,continuousPose(e.ctx.obs.pose,eta),selected_support)/e.ctx.source->size();};
      const auto j=numericalJet(x,h,val);
      for(int a=0;a<3;++a) {
        Eigen::VectorXd direction(6);for(int c=0;c<6;++c)direction(c)=std::sin((a+1)*(c+2));direction.normalize();
        const double plus=val(x+h*direction),minus=val(x-h*direction),g=(plus-minus)/(2*h),curv=(plus-2*j.energy+minus)/(h*h);
        const double gp=j.g.dot(direction),hp=direction.dot(j.H*direction);
        const double ge=std::abs(g-gp),he=std::abs(curv-hp),gr=ge/std::max(1e-12,std::max(std::abs(g),std::abs(gp))),hr=he/std::max(1e-12,std::max(std::abs(curv),std::abs(hp)));
        writeRow(directional,{id,stage=="ENDPOINT"?"ENDPOINT":(dir=="FORWARD"?"T0":"CANONICAL"),precision,number(h),std::to_string(a),number(ge),number(gr),number(he),number(hr),(ge<=1e-4||gr<=.02)&&(he<=.02||hr<=.05)?"1":"0"});
      }
    }
    mf.flush();md.flush();metadata.flush();reference.flush();std::cout<<"R1C2_ROOT "<<id<<" "<<star.status<<" delta="<<(star.v-v).norm()<<std::endl;
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!stationaritySelfTest())throw std::runtime_error("double evaluator/multi-h self-test");
      std::cout<<"P9_R1C2_NUMERICAL_SELF_TEST=PASS\n";return 0;
    }
    if(argc>1&&std::string(argv[1])=="--run"){runDiagnostic(argc,argv);return 0;}
    throw std::runtime_error("expected --self-test or --run");
  }catch(const std::exception& ex){std::cerr<<"R1C2_ERROR "<<ex.what()<<'\n';return 1;}
}
