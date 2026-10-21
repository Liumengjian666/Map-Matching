#include "pool_v3.hpp"
#include <fstream>
#include <map>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/resource.h>
using namespace pool_v3;
using Row=std::map<std::string,std::string>;
std::vector<Row> read(const std::string& path){
  std::ifstream in(path);std::string line;require(bool(std::getline(in,line)),"missing CSV");
  auto keys=csvFields(line);std::vector<Row> result;
  while(std::getline(in,line)){auto values=csvFields(line);require(values.size()==keys.size(),"CSV width mismatch");
    Row r;for(size_t i=0;i<keys.size();++i)require(r.emplace(keys[i],values[i]).second,"duplicate CSV column");result.push_back(r);}
  require(in.eof(),"CSV read error");return result;
}
double number(const Row& r,const std::string& key){return r.at(key)=="NA"?NAN:std::stod(r.at(key));}
int integer(const Row& r,const std::string& key){return std::stoi(r.at(key));}
Pose pose(const Row& r,const std::string& prefix){Pose t;for(int i=0;i<4;++i)for(int j=0;j<4;++j)t(i,j)=number(r,prefix+std::to_string(i)+std::to_string(j));return t;}
std::string key(const Row& r){return r.at("arm")+":"+r.at("scan_index");}
std::string ids(const std::vector<int>& indices,const std::vector<Candidate>& raw){std::string out;for(int j:indices){if(!out.empty())out+='|';out+=std::to_string(raw.at(j).id);}return out.empty()?"NA":out;}
struct Csv {
  std::ofstream f;
  Csv(const std::string& path,const std::string& header){f.exceptions(std::ios::badbit|std::ios::failbit);f.open(path);f<<std::setprecision(17)<<header<<'\n';}
};
int main(int argc,char** argv){try{
  require(argc==3,"usage: replay FROZEN_R11_DOC NEW_OUTPUT_DIR");
  const std::string input=argv[1],out=argv[2];require(::mkdir(out.c_str(),0775)==0,"never overwrite an output directory");
  std::map<std::string,std::vector<Row>> candidates,blocks;
  for(const auto& r:read(input+"/CANDIDATE_POOL.csv"))candidates[key(r)].push_back(r);
  for(const auto& r:read(input+"/BLOCK_LOSSES.csv"))blocks[key(r)].push_back(r);
  std::map<std::string,Row> predictions,old_admission,old_select;
  for(const auto& r:read(input+"/FINAL_OBSERVATIONS.csv"))require(predictions.emplace(key(r),r).second,"duplicate frame prediction");
  for(const auto& r:read(input+"/ADMISSION_AND_SELECTION.csv"))require(old_admission.emplace(key(r),r).second,"duplicate old admission");
  for(const auto& r:read(input+"/SELECTORS.csv"))require(old_select.emplace(key(r)+":"+r.at("selector"),r).second,"duplicate selector");
  std::map<std::string,std::map<std::pair<int,int>,double>> original_pairs;
  for(const auto& r:read(input+"/PAIRWISE_EVIDENCE.csv"))require(original_pairs[key(r)].emplace(std::make_pair(integer(r,"j"),integer(r,"k")),number(r,"D_j_k")).second,"duplicate original pair");
  auto coverage_rows=read(input+"/COVERAGE.csv");require(coverage_rows.size()==100&&candidates.size()==200&&predictions.size()==200&&old_admission.size()==200,"fixed100x2 input");
  Csv mapping(out+"/RAW_TO_GROUP_MAPPING.csv","arm,scan_index,raw_index,candidate_id,native_call_id,old_duplicate_of,new_group_id,anchor_id,representative_id,raw_eligible,representative_eligible,is_representative,is_raw_nominal_anchor");
  Csv groups(out+"/REPRESENTATIVE_COMPARISON.csv","arm,scan_index,group_id,anchor_id,member_ids,old_representative_id,representative_id,representative_eligible,all_raw_eligible_ids,discarded_member_ids,FEASIBLE_REPRESENTATIVE_PRESERVED,PHYSICALLY_DISTINCT_FROM_NOMINAL");
  Csv pairs(out+"/PAIRWISE_EVIDENCE.csv","variant,arm,scan_index,B,j,k,D_j_k,j_eligible,k_eligible,j_group,k_group,k_is_raw_nominal_anchor");
  Csv decisions(out+"/ACCEPTANCE_COMPARISON.csv","arm,scan_index,raw_index,raw_count,raw_eligible,old_eligible_discarded,new_eligible_representatives,old_pool_ids,new_representative_ids,new_evidence_ids,old_native,new_native,old_mean,new_mean,old_robust,new_robust,old_selected,new_selected,old_status,new_status,old_fallback,new_fallback,FEASIBLE_REPRESENTATIVE_PRESERVED,ROBUST_EVIDENCE_SUPPORTED,PHYSICALLY_DISTINCT_RECOVERY,ACCEPTED_REPLACEMENT");
  Csv timing(out+"/RUNTIME_AND_MEMORY.csv","arm,scan_index,control_ms,proposed_ms,process_high_water_RSS_KiB");
  int total_raw=0,total_eligible=0,total_lost=0,total_new_eligible=0,recovered=0,changed=0,physical=0;
  const auto start=Clock::now();
  for(int i=1;i<=100;++i)for(const std::string arm:{"BASELINE_WEAK","PROPOSED_DISAGREEMENT"}){
    const std::string k=arm+":"+std::to_string(i);const auto& cov=coverage_rows.at(i-1);
    require(integer(cov,"scan_index")==i&&integer(cov,"raw_index")==10640+i,"frozen scan identity");
    Split s;s.training->resize(integer(cov,"training_points"));for(int b=0;b<8;++b)s.blocks[b]->resize(integer(cov,"val"+std::to_string(b)));
    Mask m(s);require(m.evaluable()&&int(m.active.size())==integer(cov,"B"),"coverage identity");
    const auto& prediction_row=predictions.at(k);const Pose prediction=pose(prediction_row,"prediction_");require(rigid(prediction),"invalid frozen prediction");
    std::vector<Candidate> raw;std::vector<int> native_calls,recorded_duplicates;std::map<std::pair<int,int>,Row> losses;
    for(const auto& row:blocks.at(k))require(losses.emplace(std::make_pair(integer(row,"candidate_id"),integer(row,"block_id")),row).second,"duplicate loss row");
    for(const auto& row:candidates.at(k)){
      Candidate c;c.id=integer(row,"candidate_id");require(c.id==int(raw.size())&&integer(row,"raw_index")==10640+i,"raw ID ordering");
      c.pose=pose(row,"final_");c.initial=pose(row,"seed_");c.finite=integer(row,"finite");c.converged=integer(row,"converged");
      c.iterations=integer(row,"iterations");c.score=number(row,"native_score");c.fitness=number(row,"fitness");c.overlap=number(row,"overlap");
      for(int b=0;b<8;++b){const auto& l=losses.at({c.id,b});const bool active=std::find(m.active.begin(),m.active.end(),b)!=m.active.end();
        require(integer(l,"active")==active&&integer(l,"points")==int(s.blocks[b]->size()),"frozen block provenance");c.loss[b]=number(l,"loss");if(!active)require(!std::isfinite(c.loss[b]),"inactive loss must remain NA");}
      c.loss_valid=m.valid(c.loss);c.eligible=admissible(c,m,prediction);require(c.eligible==bool(integer(row,"eligible")),"recomputed eligibility differs");
      raw.push_back(c);native_calls.push_back(integer(row,"native_call_id"));recorded_duplicates.push_back(integer(row,"duplicate_of"));
    }
    require(losses.size()==raw.size()*8,"all8 original records per candidate required");
    auto control=raw;auto phase=Clock::now();shadow::deduplicate(control);const auto old_pool=shadow::pool(control);
    const auto old_selection=coverage::select(control,m);const auto old_decision=feasibility::decide(control,m,prediction,old_selection);const double old_ms=milliseconds(phase);
    const auto& frozen_pairs=original_pairs[k];require(frozen_pairs.size()==old_pool.size()*(old_pool.size()-1),"CONTROL pair set differs");
    for(int j:old_pool)for(int l:old_pool)if(j!=l)require(coverage::paired(raw[j].loss,raw[l].loss,m)==frozen_pairs.at({j,l}),"CONTROL paired value differs");
    const auto& original=old_admission.at(k);
    for(size_t j=0;j<control.size();++j)require(control[j].duplicate_of==recorded_duplicates[j],"CONTROL must reproduce every old duplicate");
    require(old_decision.status==original.at("final_status")&&old_decision.selected==integer(original,"final_selected")&&old_decision.prediction_fallback==bool(integer(original,"prediction_fallback")),"CONTROL must reproduce original decision");
    require(old_selection.native==integer(old_select.at(k+":NATIVE_SCORE"),"selected")&&old_selection.heldout_mean==integer(old_select.at(k+":HELDOUT_MEAN"),"selected")&&old_selection.robust==integer(old_select.at(k+":COVERAGE_ROBUST"),"selected"),"CONTROL selectors differ");
    require(old_decision.observation_valid==bool(integer(prediction_row,"observation_valid")),"original observation validity");
    if(old_decision.observation_valid)require((pose(prediction_row,"accepted_")-raw[old_decision.selected].pose).norm()==0,"original accepted matrix");
    else for(int a=0;a<4;++a)for(int b=0;b<4;++b)require(prediction_row.at("accepted_"+std::to_string(a)+std::to_string(b))=="NA","rejected matrix cannot become observation");
    phase=Clock::now();const Pool p=construct(raw,m,prediction);const auto selection=pool_v3::select(p,m,prediction);const auto decision=pool_v3::decide(p,m,prediction,selection);const double new_ms=milliseconds(phase);
    int eligible=0,lost=0,new_eligible=0;bool preserved=true;
    for(size_t j=0;j<raw.size();++j){eligible+=raw[j].eligible;lost+=raw[j].eligible&&control[j].duplicate_of>=0;
      const int g=p.group_of[j],rep=g<0?-1:p.groups[g].representative;
      if(raw[j].eligible)preserved=preserved&&g>=0&&p.raw[rep].eligible;
      mapping.f<<arm<<','<<i<<','<<10640+i<<','<<raw[j].id<<','<<native_calls[j]<<','<<recorded_duplicates[j]<<','<<g<<','<<(g<0?-1:p.groups[g].anchor)<<','<<rep<<','<<raw[j].eligible<<','<<(rep>=0&&p.raw[rep].eligible)<<','<<(rep==int(j))<<','<<(j==0)<<'\n';
    }
    for(size_t g=0;g<p.groups.size();++g){const auto& group=p.groups[g];std::vector<int> good,discarded;
      for(int j:group.members){if(p.raw[j].eligible)good.push_back(j);if(j!=group.representative)discarded.push_back(j);}
      const bool legal=p.raw[group.representative].eligible;new_eligible+=legal;
      if(int(g)!=p.group_of[0])++physical;
      groups.f<<arm<<','<<i<<','<<g<<','<<group.anchor<<','<<ids(group.members,raw)<<','<<group.anchor<<','<<group.representative<<','<<legal<<','<<ids(good,raw)<<','<<ids(discarded,raw)<<','<<(!good.empty()&&legal)<<','<<(int(g)!=p.group_of[0])<<'\n';
    }
    auto record_pairs=[&](const std::string& variant,const std::vector<int>& evidence){for(int j:evidence)for(int l:evidence)if(j!=l)
      pairs.f<<variant<<','<<arm<<','<<i<<','<<m.active.size()<<','<<raw[j].id<<','<<raw[l].id<<','<<coverage::paired(raw[j].loss,raw[l].loss,m)<<','<<raw[j].eligible<<','<<raw[l].eligible<<','<<p.group_of[j]<<','<<p.group_of[l]<<','<<(l==0)<<'\n';};
    record_pairs("CONTROL",old_pool);record_pairs("PROPOSED",p.evidence);
    require(preserved,"feasibility preservation invariant");
    if(decision.observation_valid)require(admissible(p.raw[decision.selected],m,prediction),"invalid final map observation");
    if(arm=="PROPOSED_DISAGREEMENT"&&i==64){require(raw.size()==3&&!raw[0].eligible&&raw[1].eligible&&p.groups[p.group_of[1]].representative==1,"fixed64 counterexample not repaired");
      require(coverage::paired(raw[1].loss,raw[0].loss,m)<.02&&!decision.recovery_success,"fixed64 evidence cannot be bypassed");}
    decisions.f<<arm<<','<<i<<','<<10640+i<<','<<raw.size()<<','<<eligible<<','<<lost<<','<<new_eligible<<','<<ids(old_pool,raw)<<','<<ids(p.representatives,raw)<<','<<ids(p.evidence,raw)<<','<<old_selection.native<<','<<selection.native<<','<<old_selection.heldout_mean<<','<<selection.heldout_mean<<','<<old_selection.robust<<','<<selection.robust<<','<<old_decision.selected<<','<<decision.selected<<','<<old_decision.status<<','<<decision.status<<','<<old_decision.prediction_fallback<<','<<decision.prediction_fallback<<','<<preserved<<','<<(selection.robust>=0)<<','<<(decision.recovery_success&&p.group_of[decision.selected]!=p.group_of[0])<<','<<decision.recovery_success<<'\n';
    struct rusage usage;getrusage(RUSAGE_SELF,&usage);timing.f<<arm<<','<<i<<','<<old_ms<<','<<new_ms<<','<<usage.ru_maxrss<<'\n';
    total_raw+=raw.size();total_eligible+=eligible;total_lost+=lost;total_new_eligible+=new_eligible;recovered+=decision.recovery_success;changed+=old_decision.selected!=decision.selected||old_decision.status!=decision.status;
  }
  require(total_raw==212,"frozen212 candidate rows");
  for(Csv* file:{&mapping,&groups,&pairs,&decisions,&timing})file->f.flush();
  std::cout<<"R12_REPLAY_COMPLETE frames=100 arms=2 raw="<<total_raw<<" raw_eligible="<<total_eligible<<" old_discarded_eligible="<<total_lost<<" new_eligible_representatives="<<total_new_eligible<<" physical_non_nominal_groups="<<physical<<" accepted_replacements="<<recovered<<" final_changes="<<changed<<" wall_ms="<<milliseconds(start)<<" new_ndt_align=0\n";
}catch(const std::exception& e){std::cerr<<"R12_STOP "<<e.what()<<'\n';return 1;}}
