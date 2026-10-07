// Offline P9-R1C3B support-transition evidence; no alignment or oracle labels.
#define P9_TRUE_PROFILE_CLOSURE_LIBRARY
#include "p9_true_profile_closure.cpp"
#include "p9_stationarity_numerics.hpp"

namespace {

using CsvRow=std::vector<std::string>;
std::size_t g_dynamic_support_evaluations=0;

struct Snapshot {
  Support signature;
  ExactPclNdt::FrozenSupport leaves;
  double dynamic_energy=0;
  std::uint64_t hash=0;
};

struct SideResult {
  ReferenceMinimum minimum;
  Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  Eigen::VectorXd eigenvalues;
  double condition=std::numeric_limits<double>::infinity();
  double energy=std::numeric_limits<double>::quiet_NaN();
  double dynamic_energy=std::numeric_limits<double>::quiet_NaN();
  std::uint64_t dynamic_hash=0;
  bool dynamic_equal=false;
  std::size_t terms=0;
};

struct PairResult {
  std::string classification="UNRESOLVED";
  std::string status_a,status_b;
  SideResult a,b;
  double delta_v=std::numeric_limits<double>::quiet_NaN();
  double translation=std::numeric_limits<double>::quiet_NaN();
  double rotation_deg=std::numeric_limits<double>::quiet_NaN();
  double energy_gap=std::numeric_limits<double>::quiet_NaN();
  double signed_energy_gap=std::numeric_limits<double>::quiet_NaN();
  double energy_a_at_b=std::numeric_limits<double>::quiet_NaN();
  double energy_b_at_a=std::numeric_limits<double>::quiet_NaN();
  double eps_dv_a=std::numeric_limits<double>::quiet_NaN();
  double eps_dv_b=std::numeric_limits<double>::quiet_NaN();
  double eps_dv_pair=std::numeric_limits<double>::quiet_NaN();
  double eps_e_a=std::numeric_limits<double>::quiet_NaN();
  double eps_e_b=std::numeric_limits<double>::quiet_NaN();
  double eps_e_pair=std::numeric_limits<double>::quiet_NaN();
  double impact_v=std::numeric_limits<double>::quiet_NaN();
  double impact_e=std::numeric_limits<double>::quiet_NaN();
  double impact_pose_t=std::numeric_limits<double>::quiet_NaN();
  double impact_pose_r=std::numeric_limits<double>::quiet_NaN();
  double changed_fraction=0;
  double membership_change_fraction=0;
  bool both_stationary=false;
  bool both_spd=false;
  bool support_equal_a=false,support_equal_b=false;
  double runtime_ms=0;
};

struct FrameData {
  FrameContext context;
  Eigen::Matrix<double,6,2> weak;
  Eigen::Matrix<double,6,4> strong;
  Snapshot nominal;
};

struct EdgeResult {
  int axis=0,edge=0;
  double left_offset=0,right_offset=0;
  bool transition=false;
  Snapshot left,right;
  PairResult pair;
};

std::string poseMatrixText(const Eigen::Matrix4f& pose) {
  std::ostringstream out;out<<std::setprecision(17);
  for(int r=0;r<4;++r)for(int c=0;c<4;++c){if(r||c)out<<';';out<<pose(r,c);}
  return out.str();
}

std::string vectorTextAny(const Eigen::VectorXd& vector) {
  std::ostringstream out;out<<std::setprecision(17);
  for(Eigen::Index i=0;i<vector.size();++i){if(i)out<<';';out<<vector(i);}
  return out.str();
}

Snapshot takeSnapshot(DoubleFrozenNdt& ndt,const Cloud::ConstPtr& source,
                      const Eigen::Matrix4f& pose) {
  ++g_dynamic_support_evaluations;
  Snapshot result;
  const double score=ndt.dynamicValueOnly(source,pose,&result.signature,&result.leaves);
  result.hash=supportSignature(result.signature);
  result.dynamic_energy=-score/static_cast<double>(source->size());
  return result;
}

std::size_t leafTerms(const ExactPclNdt::FrozenSupport& support) {
  std::size_t result=0;
  for(const auto& per_point:support)result+=per_point.size();
  return result;
}

Eigen::Matrix4f chartPose(const FrameData& frame,const Eigen::Vector2d& u,
                          const Eigen::Vector4d& v) {
  Vector6d eta=frame.weak*u+frame.strong*v;
  return poseAtEta(frame.context.obs.pose,eta);
}

double fixedEnergy(DoubleFrozenNdt& ndt,const FrameData& frame,const Eigen::Vector2d& u,
                   const Eigen::Vector4d& v,const Snapshot& support,
                   std::size_t* evaluations=nullptr) {
  if(evaluations)++*evaluations;
  const Vector6d eta=frame.weak*u+frame.strong*v;
  const Eigen::Matrix4d pose=continuousPose(frame.context.obs.pose,eta);
  return -ndt.frozenScoreDouble(frame.context.source,pose,support.leaves)/
      static_cast<double>(frame.context.source->size());
}

SideResult solveSide(DoubleFrozenNdt& ndt,const FrameData& frame,const Eigen::Vector2d& u,
                     const Eigen::Vector4d& initial,const Snapshot& support,
                     std::size_t* evaluations) {
  SideResult result;result.terms=leafTerms(support.leaves);
  const auto objective=[&](const Eigen::VectorXd& v) {
    if(evaluations)++*evaluations;
    const Vector6d eta=frame.weak*u+frame.strong*v;
    const Eigen::Matrix4d pose=continuousPose(frame.context.obs.pose,eta);
    return -ndt.frozenScoreDouble(frame.context.source,pose,support.leaves)/
        static_cast<double>(frame.context.source->size());
  };
  result.minimum=referenceMinimum(initial,result.terms,objective);
  result.energy=objective(result.minimum.v);
  result.eigenvalues=result.minimum.jet.eig;
  if(result.minimum.jet.spd&&result.eigenvalues.size()==4&&result.eigenvalues.minCoeff()>0)
    result.condition=result.eigenvalues.maxCoeff()/result.eigenvalues.minCoeff();
  result.pose=chartPose(frame,u,result.minimum.v);
  const Snapshot actual=takeSnapshot(ndt,frame.context.source,result.pose);
  result.dynamic_energy=actual.dynamic_energy;result.dynamic_hash=actual.hash;
  result.dynamic_equal=actual.signature==support.signature;
  return result;
}

std::string classifyPair(const PairResult& pair) {
  if(!pair.both_stationary||!pair.both_spd||!std::isfinite(pair.impact_v)||
     !std::isfinite(pair.impact_e))return "UNRESOLVED";
  if(pair.impact_v>3.0||pair.impact_e>3.0)return "EVENT_STRONGLY_MATERIAL";
  if(pair.impact_v>1.0||pair.impact_e>1.0)return "EVENT_MATERIAL";
  return "EVENT_NUMERICALLY_NEGLIGIBLE";
}

PairResult evaluatePair(DoubleFrozenNdt& ndt,const FrameData& frame,const Snapshot& support_a,
                        const Snapshot& support_b,const Eigen::Vector2d& u,
                        const Eigen::Vector4d& common_initial,double changed_fraction,
                        double membership_change_fraction,std::size_t* double_evaluations) {
  PairResult result;result.changed_fraction=changed_fraction;
  result.membership_change_fraction=membership_change_fraction;
  const auto started=std::chrono::steady_clock::now();
  result.a=solveSide(ndt,frame,u,common_initial,support_a,double_evaluations);
  result.b=solveSide(ndt,frame,u,common_initial,support_b,double_evaluations);
  result.status_a=result.a.minimum.status;result.status_b=result.b.minimum.status;
  result.both_stationary=result.status_a=="DOUBLE_RESOLUTION_STATIONARY"&&
      result.status_b=="DOUBLE_RESOLUTION_STATIONARY";
  result.both_spd=result.a.minimum.jet.spd&&result.b.minimum.jet.spd;
  result.support_equal_a=result.a.dynamic_equal;result.support_equal_b=result.b.dynamic_equal;
  result.delta_v=(result.a.minimum.v-result.b.minimum.v).norm();
  result.translation=translationDistance(result.a.pose,result.b.pose);
  result.rotation_deg=rotationDistanceDeg(result.a.pose,result.b.pose);
  result.energy_gap=std::abs(result.a.energy-result.b.energy);
  result.signed_energy_gap=result.a.energy-result.b.energy;
  result.energy_a_at_b=fixedEnergy(ndt,frame,u,result.b.minimum.v,support_a,double_evaluations);
  result.energy_b_at_a=fixedEnergy(ndt,frame,u,result.a.minimum.v,support_b,double_evaluations);
  result.eps_dv_a=std::max(result.a.minimum.d_resolution,
                           result.a.minimum.fd_displacement_difference);
  result.eps_dv_b=std::max(result.b.minimum.d_resolution,
                           result.b.minimum.fd_displacement_difference);
  result.eps_dv_pair=result.eps_dv_a+result.eps_dv_b;
  result.eps_e_a=doubleEnergyResolution(result.a.energy,result.a.terms);
  result.eps_e_b=doubleEnergyResolution(result.b.energy,result.b.terms);
  result.eps_e_pair=result.eps_e_a+result.eps_e_b;
  if(result.eps_dv_pair>0&&result.eps_e_pair>0) {
    result.impact_v=result.delta_v/result.eps_dv_pair;
    result.impact_e=result.energy_gap/result.eps_e_pair;
    result.impact_pose_t=result.translation/(kResolution*result.eps_dv_pair);
    result.impact_pose_r=(result.rotation_deg*kPi/180.0)/result.eps_dv_pair;
  }
  result.runtime_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
  result.classification=classifyPair(result);
  return result;
}

struct Outputs {
  std::ofstream raw,pairs;
  explicit Outputs(const std::string& directory):
    raw(directory+"/support_events_raw.csv"),
    pairs(directory+"/event_stationary_pairs.csv") {
    if(!raw||!pairs)throw std::runtime_error("cannot open R1C3B output directory");
    raw.exceptions(std::ios::badbit|std::ios::failbit);
    pairs.exceptions(std::ios::badbit|std::ios::failbit);
    writeHeader(raw,{"event_id","origin","tx","frame_id","source_point_count","axis","edge","left_offset","right_offset",
      "transition","classification","support_hash_A","support_hash_B","changed_point_fraction",
      "membership_change_fraction","dynamic_energy_A","dynamic_energy_B","u_mid","v_mid",
      "fixed_solves_performed"});
    writeHeader(pairs,{"event_id","origin","tx","frame_id","source_point_count","axis","edge","alpha","round",
      "support_hash_A","support_hash_B","changed_point_fraction","membership_change_fraction",
      "support_leaf_terms_A","support_leaf_terms_B",
      "u_mid","v_common_initial","v_A_star","v_B_star","pose_A_star","pose_B_star",
      "status_A","status_B","Hvv_spd_A","Hvv_spd_B","Hvv_eigenvalues_A","Hvv_eigenvalues_B",
      "Hvv_condition_A","Hvv_condition_B","E_A_common","E_B_common","E_dynamic_common",
      "E_A_star","E_B_star","E_A_at_B","E_B_at_A","signed_energy_gap","Delta_E",
      "Delta_v","translation_separation_m","rotation_separation_deg","EPS_DV_A","EPS_DV_B",
      "EPS_DV_PAIR","EPS_E_A","EPS_E_B","EPS_E_PAIR","I_v","I_E","I_pose_t","I_pose_r",
      "dynamic_energy_A_star","dynamic_energy_B_star","dynamic_hash_A_star","dynamic_hash_B_star",
      "support_equal_A","support_equal_B","classification","runtime_ms","double_evaluations"});
  }
};

std::string eventKey(const std::string& origin,std::uint64_t tx,int axis,int edge) {
  std::ostringstream out;out<<origin<<"_"<<tx<<"_"<<axis<<"_"<<edge;return out.str();
}

bool closeAlpha(const std::string& value,double wanted) {
  return std::abs(std::stod(value)-wanted)<1e-10;
}

Eigen::VectorXd parseVectorField(const std::string& text,int expected_size) {
  const auto values=split(text,';');
  if(static_cast<int>(values.size())!=expected_size)
    throw std::runtime_error("historical vector dimension mismatch");
  Eigen::VectorXd result(expected_size);
  for(int i=0;i<expected_size;++i) {
    result(i)=std::stod(values[i]);
    if(!std::isfinite(result(i)))throw std::runtime_error("nonfinite historical vector");
  }
  return result;
}

const CsvRow* findHistoricalCheck(const CsvTable& checks,const CsvRow& event,bool first) {
  const CsvRow* selected=nullptr;
  int selected_inner=first?std::numeric_limits<int>::max():-1;
  for(const auto& row:checks.rows) {
    if(parseU64(checks.get(row,"tx"))!=parseU64(checks.get(event,"tx"))||
       checks.get(row,"cluster")!=checks.get(event,"cluster")||
       checks.get(row,"attempt")!=checks.get(event,"attempt")||
       checks.get(row,"candidate_id")!=checks.get(event,"candidate_id")||
       checks.get(row,"hypothesis")!=checks.get(event,"hypothesis")||
       !closeAlpha(checks.get(row,"alpha"),std::stod(checks.get(event,"alpha")))||
       checks.get(row,"round")!=checks.get(event,"round"))continue;
    const int inner=std::stoi(checks.get(row,"inner"));
    if((first&&inner<selected_inner)||(!first&&inner>=selected_inner)) {
      selected=&row;selected_inner=inner;
    }
  }
  return selected;
}

using HistoricalSupportRegistry=std::map<std::uint64_t,std::map<std::uint64_t,Snapshot>>;

void registerHistoricalSupport(DoubleFrozenNdt& ndt,FrameData& frame,HistoricalSupportRegistry& registry,
                               const Eigen::Matrix4f& pose,std::uint64_t expected,
                               const std::string& context) {
  Snapshot snapshot=takeSnapshot(ndt,frame.context.source,pose);
  if(snapshot.hash!=expected)throw std::runtime_error("historical support hash not reproduced at "+
    context+" tx="+std::to_string(frame.context.frame.transaction)+" expected="+
    std::to_string(expected)+" actual="+std::to_string(snapshot.hash));
  auto& per_frame=registry[frame.context.frame.transaction];
  const auto found=per_frame.find(snapshot.hash);
  if(found!=per_frame.end()&&found->second.signature!=snapshot.signature)
    throw std::runtime_error("historical support hash collision");
  per_frame[snapshot.hash]=std::move(snapshot);
}

void loadHistoricalSupportRegistry(DoubleFrozenNdt& ndt,
    const std::map<std::uint64_t,FrameData*>& frames,const CsvTable& events,
    const CsvTable& checks,const CsvTable& nodes,HistoricalSupportRegistry& registry) {
  for(const auto& entry:frames) {
    FrameData& frame=*entry.second;
    if(entry.first!=3341&&entry.first!=616)continue;
    const Snapshot nominal=takeSnapshot(ndt,frame.context.source,frame.context.obs.pose);
    registry[entry.first][nominal.hash]=nominal;
  }
  // The OLD_SUPPORT at 616/.68625 inherits its pre-state from the archived
  // accepted .68125 node; rebuild stored nodes before following event links.
  for(const auto& row:nodes.rows) {
    const std::uint64_t tx=parseU64(nodes.get(row,"tx"));
    if(tx!=616||nodes.get(row,"cluster")!="P10"||
       (!closeAlpha(nodes.get(row,"alpha"),.68125)&&!closeAlpha(nodes.get(row,"alpha"),.68625)))continue;
    const auto found=frames.find(tx);
    if(found==frames.end())throw std::runtime_error("missing historical frame context");
    registerHistoricalSupport(ndt,*found->second,registry,
      parseMatrix16(nodes.get(row,"pose_matrix16")),parseU64(nodes.get(row,"support_hash")),
      "archived branch node alpha="+nodes.get(row,"alpha"));
  }
  for(const auto& event:events.rows) {
    const std::uint64_t tx=parseU64(events.get(event,"tx"));
    const std::string cluster=events.get(event,"cluster");
    const std::string hypothesis=events.get(event,"hypothesis");
    const bool root3341=tx==3341&&cluster=="P02"&&hypothesis=="ROOT"&&
      closeAlpha(events.get(event,"alpha"),0.0);
    const bool boundary616=tx==616&&cluster=="P10"&&
      (hypothesis=="OLD_SUPPORT"||hypothesis=="NEW_SUPPORT")&&
      closeAlpha(events.get(event,"alpha"),.68625);
    if(!root3341&&!boundary616)continue;
    const CsvRow* first=findHistoricalCheck(checks,event,true);
    const CsvRow* last=findHistoricalCheck(checks,event,false);
    if(!first||!last)throw std::runtime_error("historical support event lacks endpoint checks");
    const auto found=frames.find(tx);
    if(found==frames.end())throw std::runtime_error("missing historical frame context");
    auto& per_frame=registry[tx];
    const std::uint64_t before=parseU64(events.get(event,"before_hash"));
    if(!per_frame.count(before)) {
      const Eigen::Vector2d u=parseVectorField(checks.get(*first,"u"),2);
      const Eigen::Vector4d v=parseVectorField(checks.get(*first,"v"),4);
      registerHistoricalSupport(ndt,*found->second,registry,
        chartPose(*found->second,u,v),before,"event pre-state");
    }
    const std::uint64_t after=parseU64(events.get(event,"after_hash"));
    const Eigen::Vector2d u=parseVectorField(checks.get(*last,"u"),2);
    const Eigen::Vector4d v=parseVectorField(checks.get(*last,"v"),4);
    registerHistoricalSupport(ndt,*found->second,registry,
      chartPose(*found->second,u,v),after,"event post-state");
  }
}

void writePairRow(Outputs& output,const std::string& id,const std::string& origin,
    std::uint64_t tx,const std::string& frame_id,std::size_t source_points,int axis,int edge,double alpha,int round,
    const Snapshot& a,const Snapshot& b,const Eigen::Vector2d& u,const Eigen::Vector4d& v0,
    const PairResult& r,std::size_t evals,double energy_a_common,double energy_b_common,
    double dynamic_common) {
  writeRow(output.pairs,{id,origin,std::to_string(tx),frame_id,std::to_string(source_points),
    std::to_string(axis),std::to_string(edge),
    number(alpha),std::to_string(round),std::to_string(a.hash),std::to_string(b.hash),
    number(r.changed_fraction),number(r.membership_change_fraction),std::to_string(r.a.terms),
    std::to_string(r.b.terms),vectorText(u),vectorText(v0),
    vectorText(r.a.minimum.v),vectorText(r.b.minimum.v),poseMatrixText(r.a.pose),poseMatrixText(r.b.pose),
    r.status_a,r.status_b,r.a.minimum.jet.spd?"1":"0",r.b.minimum.jet.spd?"1":"0",
    vectorTextAny(r.a.eigenvalues),vectorTextAny(r.b.eigenvalues),number(r.a.condition),number(r.b.condition),
    number(energy_a_common),number(energy_b_common),number(dynamic_common),number(r.a.energy),number(r.b.energy),
    number(r.energy_a_at_b),number(r.energy_b_at_a),number(r.signed_energy_gap),number(r.energy_gap),
    number(r.delta_v),number(r.translation),number(r.rotation_deg),number(r.eps_dv_a),number(r.eps_dv_b),
    number(r.eps_dv_pair),number(r.eps_e_a),number(r.eps_e_b),number(r.eps_e_pair),number(r.impact_v),
    number(r.impact_e),number(r.impact_pose_t),number(r.impact_pose_r),number(r.a.dynamic_energy),
    number(r.b.dynamic_energy),std::to_string(r.a.dynamic_hash),std::to_string(r.b.dynamic_hash),
    r.support_equal_a?"1":"0",r.support_equal_b?"1":"0",r.classification,number(r.runtime_ms),
    std::to_string(evals)});
}

void writeRawRow(Outputs& output,const std::string& id,const std::string& origin,
    const FrameData& frame,int axis,int edge,double left,double right,const Snapshot& a,
    const Snapshot& b,const PairResult* pair) {
  SupportChange change=compareSupport(a.signature,b.signature);
  const double point_fraction=frame.context.source->empty()?0.0:
      static_cast<double>(change.changed_points)/frame.context.source->size();
  const double membership_denom=std::max<std::size_t>(1,a.leaves.empty()?0:
      [&](){std::size_t n=0;for(const auto& x:a.leaves)n+=x.size();return n;}()+
      [&](){std::size_t n=0;for(const auto& x:b.leaves)n+=x.size();return n;}());
  const double membership_fraction=static_cast<double>(change.symmetric_difference)/membership_denom;
  const bool transition=a.signature!=b.signature;
  writeRow(output.raw,{id,origin,std::to_string(frame.context.frame.transaction),frame.context.frame.frame_id,
    std::to_string(frame.context.source->size()),
    std::to_string(axis),std::to_string(edge),number(left),number(right),transition?"1":"0",
    pair?pair->classification:(transition?"NOT_SOLVED":"NO_SUPPORT_CHANGE"),std::to_string(a.hash),
    std::to_string(b.hash),number(point_fraction),number(membership_fraction),number(a.dynamic_energy),
    number(b.dynamic_energy),"","",transition?"2":"0"});
}

std::vector<double> stencilOffsets() {return {-0.02,-0.01,0.0,0.01,0.02};}

void processCohortFrame(DoubleFrozenNdt& ndt,FrameData& frame,Outputs& output,
    std::size_t& double_evaluations,std::vector<EdgeResult>* frame_edges) {
  const std::vector<double> offsets=stencilOffsets();
  std::size_t transition_count=0,material_count=0,strong_count=0,unresolved_count=0;
  std::vector<EdgeResult> edges;
  for(int axis=0;axis<6;++axis) {
    std::vector<Snapshot> samples(offsets.size());
    for(std::size_t oi=0;oi<offsets.size();++oi) {
      Vector6d eta=Vector6d::Zero();eta(axis)=offsets[oi];
      samples[oi]=takeSnapshot(ndt,frame.context.source,poseAtEta(frame.context.obs.pose,eta));
    }
    for(int edge=0;edge<4;++edge) {
      EdgeResult item;item.axis=axis;item.edge=edge;item.left_offset=offsets[edge];
      item.right_offset=offsets[edge+1];item.left=samples[edge];item.right=samples[edge+1];
      item.transition=item.left.signature!=item.right.signature;
      item.pair.classification=item.transition?"UNRESOLVED":"NO_SUPPORT_CHANGE";
      const std::string id=eventKey("STENCIL",frame.context.frame.transaction,axis,edge);
      if(item.transition) {
        ++transition_count;
        const double point_fraction=static_cast<double>(compareSupport(item.left.signature,
            item.right.signature).changed_points)/frame.context.source->size();
        const Vector6d eta_mid=Vector6d::Unit(axis)*
            ((item.left_offset+item.right_offset)*0.5);
        const Eigen::Vector2d u=frame.weak.transpose()*eta_mid;
        const Eigen::Vector4d v=frame.strong.transpose()*eta_mid;
        const double ea=fixedEnergy(ndt,frame,u,v,item.left,&double_evaluations);
        const double eb=fixedEnergy(ndt,frame,u,v,item.right,&double_evaluations);
        const Snapshot mid=takeSnapshot(ndt,frame.context.source,chartPose(frame,u,v));
        const auto before=double_evaluations;
        item.pair=evaluatePair(ndt,frame,item.left,item.right,u,v,point_fraction,
            static_cast<double>(compareSupport(item.left.signature,item.right.signature).symmetric_difference)/
            std::max<std::size_t>(1,leafTerms(item.left.leaves)+leafTerms(item.right.leaves)),
            &double_evaluations);
        const std::size_t evals=double_evaluations-before;
        writePairRow(output,id,"DETERMINISTIC_STENCIL",frame.context.frame.transaction,
            frame.context.frame.frame_id,frame.context.source->size(),axis,edge,0.0,edge+1,
            item.left,item.right,u,v,item.pair,
            evals,ea,eb,mid.dynamic_energy);
        if(item.pair.classification=="UNRESOLVED")++unresolved_count;
        else if(item.pair.classification=="EVENT_MATERIAL")++material_count;
        else if(item.pair.classification=="EVENT_STRONGLY_MATERIAL")++strong_count;
      }
      writeRawRow(output,id,"DETERMINISTIC_STENCIL",frame,axis,edge,item.left_offset,
          item.right_offset,item.left,item.right,item.transition?&item.pair:nullptr);
      // Repeatability needs only endpoint hashes and the solved response; do
      // not retain per-point support vectors for the whole 32-frame cohort.
      item.left.signature.clear();item.left.leaves.clear();
      item.right.signature.clear();item.right.leaves.clear();
      edges.push_back(std::move(item));
    }
  }
  if(frame_edges)*frame_edges=edges;
  (void)transition_count;(void)material_count;(void)strong_count;(void)unresolved_count;
}

void processHistoricalPairs(DoubleFrozenNdt& ndt,const std::map<std::uint64_t,FrameSource>& sources,
    const std::map<std::uint64_t,ArchivedUObs>& observations,const std::string& pair_path,
    const std::string& event_path,const std::string& check_path,const std::string& node_path,
    Cloud::Ptr target,Outputs& output,std::size_t& double_evaluations) {
  const CsvTable history=readCsv(pair_path),events=readCsv(event_path);
  const CsvTable checks=readCsv(check_path),nodes=readCsv(node_path);
  if(history.rows.size()!=24)throw std::runtime_error("R1C3A historical support-pair count must be 24");
  std::map<std::uint64_t,FrameData> frames;
  for(const auto& row:history.rows) {
    const std::uint64_t tx=parseU64(history.get(row,"tx"));
    if(tx!=3341&&tx!=616)throw std::runtime_error("unexpected R1C3A historical support frame");
    if(!frames.count(tx)) {
      auto found=sources.find(tx);if(found==sources.end())throw std::runtime_error("historical frame absent from cohort");
      FrameData f;f.context=frameContext(found->second,observations.at(tx),ndt,target->size());
      ++g_dynamic_support_evaluations;
      f.weak=f.context.obs.eigenvectors.leftCols(2);f.strong=f.context.obs.eigenvectors.rightCols(4);
      frames.emplace(tx,std::move(f));
    }
  }
  std::map<std::uint64_t,FrameData*> frame_ptrs;
  for(auto& entry:frames)frame_ptrs[entry.first]=&entry.second;
  HistoricalSupportRegistry registry;
  loadHistoricalSupportRegistry(ndt,frame_ptrs,events,checks,nodes,registry);
  for(const auto& row:history.rows) {
    const std::uint64_t tx=parseU64(history.get(row,"tx"));
    FrameData& frame=frames.at(tx);
    const Eigen::Vector2d archived_u=parseVectorField(history.get(row,"u"),2);
    const Eigen::Vector4d v0=parseVectorField(history.get(row,"v_common_initial"),4);
    const Eigen::Vector4d vend=parseVectorField(history.get(row,"v_recorded_endpoint"),4);
    const double alpha=std::stod(history.get(row,"alpha"));
    const int round=std::stoi(history.get(row,"round"));
    if(!std::isfinite(alpha)||round<1)throw std::runtime_error("invalid historical transition index");
    const auto expected_a=parseU64(history.get(row,"before_hash"));
    const auto expected_b=parseU64(history.get(row,"after_hash"));
    const auto per_frame=registry.find(tx);
    if(per_frame==registry.end()||!per_frame->second.count(expected_a)||!per_frame->second.count(expected_b))
      throw std::runtime_error("historical support pair absent from reconstructed support registry");
    const Snapshot& a=per_frame->second.at(expected_a);
    const Snapshot& b=per_frame->second.at(expected_b);
    if(a.signature==b.signature)throw std::runtime_error("historical support pair is not a transition");
    const double changed=std::stod(history.get(row,"changed_fraction"));
    const auto measured=compareSupport(a.signature,b.signature);
    const double measured_fraction=static_cast<double>(measured.changed_points)/frame.context.source->size();
    if(std::abs(changed-measured_fraction)>1e-8)
      throw std::runtime_error("historical support changed-fraction parity mismatch");
    const double ea=fixedEnergy(ndt,frame,archived_u,v0,a,&double_evaluations);
    const double eb=fixedEnergy(ndt,frame,archived_u,v0,b,&double_evaluations);
    const Snapshot common=takeSnapshot(ndt,frame.context.source,chartPose(frame,archived_u,v0));
    const auto before=double_evaluations;
    const PairResult pair=evaluatePair(ndt,frame,a,b,archived_u,v0,measured_fraction,
        static_cast<double>(measured.symmetric_difference)/
        std::max<std::size_t>(1,leafTerms(a.leaves)+leafTerms(b.leaves)),&double_evaluations);
    const std::size_t evals=double_evaluations-before;
    const std::string id="HISTORY_"+std::to_string(tx)+"_"+history.get(row,"hypothesis")+
        "_"+std::to_string(round);
    writePairRow(output,id,"R1C3A_ARCHIVED",tx,frame.context.frame.frame_id,
        frame.context.source->size(),-1,-1,alpha,round,
        a,b,archived_u,v0,pair,evals,ea,eb,common.dynamic_energy);
    writeRawRow(output,id,"R1C3A_ARCHIVED",frame,-1,-1,alpha,alpha,a,b,&pair);
  }
}

void validateHistoricalRegistry(const std::string& map_path,const std::string& cohort_path,
    const std::string& uobs_path,const std::string& pair_path,const std::string& event_path,
    const std::string& check_path,const std::string& node_path) {
  const auto sources=readFrameSources(cohort_path);const auto observations=readArchivedUObs(uobs_path);
  const CsvTable history=readCsv(pair_path),events=readCsv(event_path);
  const CsvTable checks=readCsv(check_path),nodes=readCsv(node_path);
  if(history.rows.size()!=24)throw std::runtime_error("R1C3A historical support-pair count must be 24");
  Cloud::Ptr target=loadTarget(map_path);
  if(target->size()!=549606)throw std::runtime_error("target point count mismatch");
  DoubleFrozenNdt ndt;configureNdt(ndt,target);
  std::map<std::uint64_t,FrameData> frames;
  for(const auto& row:history.rows) {
    const std::uint64_t tx=parseU64(history.get(row,"tx"));
    if(tx!=3341&&tx!=616)throw std::runtime_error("unexpected historical support frame");
    if(frames.count(tx))continue;
    const auto source=sources.find(tx);const auto obs=observations.find(tx);
    if(source==sources.end()||obs==observations.end())throw std::runtime_error("historical frame input missing");
    FrameData f;f.context=frameContext(source->second,obs->second,ndt,target->size());
    f.weak=f.context.obs.eigenvectors.leftCols(2);f.strong=f.context.obs.eigenvectors.rightCols(4);
    frames.emplace(tx,std::move(f));
  }
  std::map<std::uint64_t,FrameData*> frame_ptrs;
  for(auto& entry:frames)frame_ptrs[entry.first]=&entry.second;
  HistoricalSupportRegistry registry;
  loadHistoricalSupportRegistry(ndt,frame_ptrs,events,checks,nodes,registry);
  for(const auto& row:history.rows) {
    const std::uint64_t tx=parseU64(history.get(row,"tx"));
    const std::uint64_t ha=parseU64(history.get(row,"before_hash"));
    const std::uint64_t hb=parseU64(history.get(row,"after_hash"));
    const auto per_frame=registry.find(tx);
    if(per_frame==registry.end()||!per_frame->second.count(ha)||!per_frame->second.count(hb))
      throw std::runtime_error("historical support pair not reconstructed");
    const auto& a=per_frame->second.at(ha);const auto& b=per_frame->second.at(hb);
    if(a.signature==b.signature)throw std::runtime_error("historical pair has identical support signatures");
    const double fraction=static_cast<double>(compareSupport(a.signature,b.signature).changed_points)/
      frames.at(tx).context.source->size();
    if(std::abs(fraction-std::stod(history.get(row,"changed_fraction")))>1e-8)
      throw std::runtime_error("historical support changed-point fraction mismatch");
  }
  std::cout<<"P9_R1C3B_HISTORICAL_SUPPORT_REGISTRY=PASS pairs="<<history.rows.size()
    <<" frames="<<frames.size()<<" NDT_ALIGN_CALLS=0\n";
}

bool repeatableAxis(const std::vector<const EdgeResult*>& candidates,std::string* class_out,
                    std::string* mechanism_out) {
  for(const std::string cls:{"EVENT_MATERIAL","EVENT_STRONGLY_MATERIAL"}) {
    std::vector<const EdgeResult*> selected;
    for(const auto* edge:candidates)if(edge->transition&&edge->pair.classification==cls)selected.push_back(edge);
    if(selected.size()<3)continue;
    bool vector_consistent=true;
    for(std::size_t i=0;i<selected.size();++i)for(std::size_t j=i+1;j<selected.size();++j) {
      const Eigen::Vector4d di=selected[i]->pair.b.minimum.v-selected[i]->pair.a.minimum.v;
      const Eigen::Vector4d dj=selected[j]->pair.b.minimum.v-selected[j]->pair.a.minimum.v;
      if(di.norm()==0||dj.norm()==0||di.dot(dj)<0)vector_consistent=false;
    }
    std::vector<double> iv,ie;
    for(const auto* edge:selected) {iv.push_back(edge->pair.impact_v);ie.push_back(edge->pair.impact_e);}
    const auto scale_consistent=[](const std::vector<double>& values) {
      if(values.empty())return false;
      double lo=std::numeric_limits<double>::infinity(),hi=0;
      for(double value:values){if(!(std::isfinite(value)&&value>1.0))return false;lo=std::min(lo,value);hi=std::max(hi,value);}
      return hi/lo<=10.0;
    };
    bool energy_sign=true;int sign=0;
    for(const auto* edge:selected) {
      const double value=edge->pair.signed_energy_gap;
      if(!std::isfinite(value)||std::abs(value)<=edge->pair.eps_e_pair){energy_sign=false;break;}
      const int this_sign=value>0?1:-1;if(sign&&sign!=this_sign){energy_sign=false;break;}sign=this_sign;
    }
    if((vector_consistent&&scale_consistent(iv))||(energy_sign&&scale_consistent(ie))) {
      if(class_out)*class_out=cls;
      if(mechanism_out)*mechanism_out=vector_consistent&&scale_consistent(iv)?"I_V_DIRECTION_AND_SCALE":"I_E_SIGN_AND_SCALE";
      return true;
    }
  }
  return false;
}

void writeRepeatabilityAndUnlabeled(const std::string& outdir,const std::vector<FrameData>& frames,
    const std::vector<std::vector<EdgeResult>>& all_edges) {
  std::ofstream rep(outdir+"/event_repeatability.csv"),features(outdir+"/frame_support_features_unlabeled.csv");
  if(!rep||!features)throw std::runtime_error("cannot open repeatability/frame-feature outputs");
  rep.exceptions(std::ios::badbit|std::ios::failbit);features.exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(rep,{"tx","frame_id","axis","edge_count","support_event_count","resolved_count",
    "material_count","strong_material_count","repeatable_material_event","repeatable_class",
    "consistency_mechanism","exact_support_pair_recurrence_count","max_I_v","max_I_E"});
  writeHeader(features,{"tx","frame_id","probe_count","support_event_count","resolved_event_count",
    "material_event_count","strong_material_event_count","repeatable_event_count",
    "exact_support_pair_recurrence_count","FRAME_MAX_Iv","FRAME_MEDIAN_Iv","FRAME_MAX_IE",
    "FRAME_MEDIAN_IE","max_translation_separation_m","max_rotation_separation_deg"});
  for(std::size_t fi=0;fi<frames.size();++fi) {
    const auto& frame=frames[fi];const auto& edges=all_edges[fi];
    std::size_t total_events=0,resolved=0,material=0,strong=0,repeatable_count=0;
    std::map<std::pair<std::uint64_t,std::uint64_t>,int> pair_counts;
    std::vector<double> iv,ie;double max_t=0,max_r=0;
    for(const auto& edge:edges)if(edge.transition) {
      ++total_events;++pair_counts[{edge.left.hash,edge.right.hash}];
      if(edge.pair.classification!="UNRESOLVED")++resolved;
      if(edge.pair.classification=="EVENT_MATERIAL")++material;
      if(edge.pair.classification=="EVENT_STRONGLY_MATERIAL")++strong;
      if(edge.pair.classification!="UNRESOLVED") {
        iv.push_back(edge.pair.impact_v);ie.push_back(edge.pair.impact_e);
        max_t=std::max(max_t,edge.pair.translation);max_r=std::max(max_r,edge.pair.rotation_deg);
      }
    }
    std::size_t exact_repeats=0;for(const auto& item:pair_counts)if(item.second>1)exact_repeats+=item.second-1;
    for(int axis=0;axis<6;++axis) {
      std::vector<const EdgeResult*> candidates;
      for(const auto& edge:edges)if(edge.axis==axis)candidates.push_back(&edge);
      std::string cls="NONE",mechanism="NONE";
      const bool repeatable=repeatableAxis(candidates,&cls,&mechanism);
      if(repeatable)++repeatable_count;
      std::size_t axis_events=0,axis_resolved=0,axis_material=0,axis_strong=0;
      double axis_max_iv=0,axis_max_ie=0;std::map<std::pair<std::uint64_t,std::uint64_t>,int> axis_pairs;
      for(const auto* edge:candidates)if(edge->transition) {
        ++axis_events;++axis_pairs[{edge->left.hash,edge->right.hash}];
        if(edge->pair.classification!="UNRESOLVED")++axis_resolved;
        if(edge->pair.classification=="EVENT_MATERIAL")++axis_material;
        if(edge->pair.classification=="EVENT_STRONGLY_MATERIAL")++axis_strong;
        if(edge->pair.classification!="UNRESOLVED") {
          axis_max_iv=std::max(axis_max_iv,edge->pair.impact_v);
          axis_max_ie=std::max(axis_max_ie,edge->pair.impact_e);
        }
      }
      std::size_t axis_repeats=0;for(const auto& item:axis_pairs)if(item.second>1)axis_repeats+=item.second-1;
      writeRow(rep,{std::to_string(frame.context.frame.transaction),frame.context.frame.frame_id,
        std::to_string(axis),"4",std::to_string(axis_events),std::to_string(axis_resolved),
        std::to_string(axis_material),std::to_string(axis_strong),repeatable?"1":"0",cls,mechanism,
        std::to_string(axis_repeats),number(axis_max_iv),number(axis_max_ie)});
    }
    const auto median=[](std::vector<double> values) {
      if(values.empty())return std::numeric_limits<double>::quiet_NaN();
      std::sort(values.begin(),values.end());const std::size_t n=values.size();
      return n%2?values[n/2]:(values[n/2-1]+values[n/2])*0.5;
    };
    const double max_iv=iv.empty()?0.0:*std::max_element(iv.begin(),iv.end());
    const double max_ie=ie.empty()?0.0:*std::max_element(ie.begin(),ie.end());
    writeRow(features,{std::to_string(frame.context.frame.transaction),frame.context.frame.frame_id,"24",
      std::to_string(total_events),std::to_string(resolved),std::to_string(material),std::to_string(strong),
      std::to_string(repeatable_count),std::to_string(exact_repeats),number(max_iv),number(median(iv)),
      number(max_ie),number(median(ie)),number(max_t),number(max_r)});
  }
}

void run(const std::string& map_path,const std::string& cohort_path,const std::string& uobs_path,
         const std::string& history_path,const std::string& event_path,const std::string& check_path,
         const std::string& node_path,const std::string& output_dir) {
  const auto sources=readFrameSources(cohort_path);const auto observations=readArchivedUObs(uobs_path);
  if(sources.size()!=32)throw std::runtime_error("frozen cohort must contain 32 unique frames");
  Cloud::Ptr target=loadTarget(map_path);if(target->size()!=549606)throw std::runtime_error("target point count mismatch");
  DoubleFrozenNdt ndt;configureNdt(ndt,target);Outputs output(output_dir);
  std::vector<FrameData> frames;frames.reserve(sources.size());
  std::vector<std::vector<EdgeResult>> all_edges;all_edges.reserve(sources.size());
  std::size_t double_evaluations=0;std::size_t total_transitions=0;
  g_dynamic_support_evaluations=0;
  for(const auto& entry:sources) {
    const auto obs=observations.find(entry.first);if(obs==observations.end())throw std::runtime_error("cohort frame has no U_obs row");
    FrameData frame;frame.context=frameContext(entry.second,obs->second,ndt,target->size());
    ++g_dynamic_support_evaluations;
    frame.weak=frame.context.obs.eigenvectors.leftCols(2);frame.strong=frame.context.obs.eigenvectors.rightCols(4);
    const Matrix6d reconstructed=frame.weak*frame.weak.transpose()+frame.strong*frame.strong.transpose();
    if((reconstructed-Matrix6d::Identity()).norm()>1e-3)throw std::runtime_error("W2/S4 chart not orthogonal");
    frame.nominal=takeSnapshot(ndt,frame.context.source,frame.context.obs.pose);
    std::vector<EdgeResult> edges;processCohortFrame(ndt,frame,output,double_evaluations,&edges);
    total_transitions+=std::count_if(edges.begin(),edges.end(),[](const EdgeResult& e){return e.transition;});
    std::cout<<"R1C3B_STENCIL_FRAME tx="<<entry.first<<" events="
      <<std::count_if(edges.begin(),edges.end(),[](const EdgeResult& e){return e.transition;})<<"/24\n";
    frames.push_back(std::move(frame));all_edges.push_back(std::move(edges));
  }
  processHistoricalPairs(ndt,sources,observations,history_path,event_path,check_path,node_path,
      target,output,double_evaluations);
  writeRepeatabilityAndUnlabeled(output_dir,frames,all_edges);
  std::cout<<"R1C3B_EVENT_ENGINE=COMPLETE frames="<<frames.size()<<" edges="<<frames.size()*24
    <<" transitions="<<total_transitions<<" historical_pairs=24 double_evaluations="
    <<double_evaluations<<" dynamic_support_evaluations="<<g_dynamic_support_evaluations
    <<" ndt_align_calls=0\n";
}

bool selfTest() {
  if(stencilOffsets().size()!=5||5*6!=30||4*6!=24)return false;
  PairResult p;p.both_stationary=p.both_spd=true;p.impact_v=1;p.impact_e=1;
  if(classifyPair(p)!="EVENT_NUMERICALLY_NEGLIGIBLE")return false;
  p.impact_v=1.00001;if(classifyPair(p)!="EVENT_MATERIAL")return false;
  p.impact_e=3.00001;if(classifyPair(p)!="EVENT_STRONGLY_MATERIAL")return false;
  p.status_a="NOT_STATIONARY";p.both_stationary=false;
  if(classifyPair(p)!="UNRESOLVED")return false;
  Eigen::Vector4d d0(1,0,0,0),d1(0,1,0,0),d2(1,1,0,0);
  if(d0.dot(d1)<0||d0.dot(d2)<0||d1.dot(d2)<0)return false;
  std::vector<EdgeResult> edges(4);
  for(int i=0;i<3;++i) {
    edges[i].transition=true;edges[i].pair.classification="EVENT_MATERIAL";
    edges[i].pair.impact_v=1.5+.1*i;edges[i].pair.impact_e=.5;
    edges[i].pair.a.minimum.v=Eigen::Vector4d::Zero();
    edges[i].pair.b.minimum.v=Eigen::Vector4d::Unit(0)*(1.0+.1*i);
    edges[i].pair.signed_energy_gap=.01;edges[i].pair.eps_e_pair=.001;
  }
  std::string repeat_class,repeat_mechanism;
  if(!repeatableAxis({&edges[0],&edges[1],&edges[2],&edges[3]},&repeat_class,&repeat_mechanism)||
     repeat_class!="EVENT_MATERIAL")return false;
  edges[2].pair.b.minimum.v=-Eigen::Vector4d::Unit(0);
  if(repeatableAxis({&edges[0],&edges[1],&edges[2],&edges[3]},nullptr,nullptr))return false;
  return stationaritySelfTest();
}

}  // namespace

int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!selfTest())throw std::runtime_error("discrete support evidence self-test failed");
      std::cout<<"P9_R1C3B_DISCRETE_SUPPORT_SELF_TEST=PASS\n";return 0;
    }
    if(argc==9&&std::string(argv[1])=="--validate-history") {
      validateHistoricalRegistry(argv[2],argv[3],argv[4],argv[5],argv[6],argv[7],argv[8]);return 0;
    }
    if(argc==10&&std::string(argv[1])=="--run") {
      run(argv[2],argv[3],argv[4],argv[5],argv[6],argv[7],argv[8],argv[9]);return 0;
    }
    std::cerr<<"usage: p9_discrete_support_evidence --self-test\n"
             <<"   or: p9_discrete_support_evidence --validate-history MAP COHORT UOBS R1C3A_PAIRS R1C3A_EVENTS R1C3A_CHECKS R1C3A_NODES\n"
             <<"   or: p9_discrete_support_evidence --run MAP COHORT UOBS R1C3A_PAIRS R1C3A_EVENTS R1C3A_CHECKS R1C3A_NODES OUTPUT_DIR\n";
    return 2;
  } catch(const std::exception& error) {
    std::cerr<<"P9_R1C3B_ERROR: "<<error.what()<<'\n';return 1;
  }
}
