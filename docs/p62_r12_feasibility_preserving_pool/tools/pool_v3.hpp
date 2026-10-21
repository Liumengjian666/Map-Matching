#pragma once
#include "../../p62_r10_feasibility_event_recovery/tools/events.hpp"
#include <set>

namespace pool_v3 {
using namespace feasibility;
struct Group {
  int anchor=-1,representative=-1;
  std::vector<int> members;
};
struct Pool {
  std::vector<Candidate> original;
  std::vector<Candidate> raw;
  std::vector<Group> groups;
  std::vector<int> group_of,representatives,evidence;
};
inline bool evidenceValid(const Candidate& c,const Mask& m){
  return safeNative(c)&&c.loss_valid&&m.valid(c.loss);
}
inline Pool construct(const std::vector<Candidate>& input,const Mask& mask,const Pose& prediction){
  require(!input.empty()&&rigid(prediction),"raw nominal and valid prediction required");
  Pool p;p.original=input;p.raw=input; // original metadata untouched; raw is derived canonical view
  // Stable IDs are provenance, never the caller's container order or scores.
  std::sort(p.raw.begin(),p.raw.end(),[](const Candidate& a,const Candidate& b){return a.id<b.id;});
  require(p.raw.front().id==0,"RAW_NOMINAL_ANCHOR id0 required");
  std::set<int> ids;
  for(auto& c:p.raw){require(c.id>=0&&ids.insert(c.id).second,"unique nonnegative IDs required");
    c.eligible=admissible(c,mask,prediction);c.duplicate_of=-1;}
  p.group_of.assign(p.raw.size(),-1);
  for(size_t j=0;j<p.raw.size();++j){
    // Unsafe raw outputs remain in raw, not in physical comparisons or evidence.
    if(!p.raw[j].finite||!rigid(p.raw[j].pose))continue;
    int group=-1;
    for(size_t g=0;g<p.groups.size();++g)
      if(duplicate(p.raw[j].pose,p.raw[p.groups[g].anchor].pose)){group=int(g);break;}
    if(group<0){group=int(p.groups.size());p.groups.push_back(Group{int(j),int(j),{}});}
    p.group_of[j]=group;p.groups[group].members.push_back(int(j));
  }
  for(auto& g:p.groups){
    // Lowest immutable candidate ID among eligible members, otherwise anchor.
    for(int j:g.members)if(p.raw[j].eligible){g.representative=j;break;}
    const int rep=g.representative;
    for(int j:g.members)if(j!=rep)p.raw[j].duplicate_of=rep;
    if(evidenceValid(p.raw[rep],mask))p.representatives.push_back(rep);
  }
  p.evidence=p.representatives;
  // A promoted duplicate never wins by deleting its illegal raw nominal rival.
  if(evidenceValid(p.raw[0],mask)&&std::find(p.evidence.begin(),p.evidence.end(),0)==p.evidence.end())p.evidence.push_back(0);
  std::sort(p.evidence.begin(),p.evidence.end());
  return p;
}
inline bool supported(const Pool& p,int j,const Mask& mask,const Pose& prediction){
  if(j<=0||size_t(j)>=p.raw.size()||!admissible(p.raw[j],mask,prediction)||
     std::find(p.representatives.begin(),p.representatives.end(),j)==p.representatives.end())return false;
  // An unavailable nominal cannot silently disappear and create certainty.
  if(std::find(p.evidence.begin(),p.evidence.end(),0)==p.evidence.end())return false;
  int opponents=0;
  for(int k:p.evidence)if(k!=j){++opponents;if(!(coverage::paired(p.raw[j].loss,p.raw[k].loss,mask)>.02))return false;}
  return opponents>0;
}
inline Selection select(const Pool& p,const Mask& mask,const Pose& prediction){
  Selection out;
  for(int j:p.representatives){const auto& c=p.raw[j];if(!admissible(c,mask,prediction))continue;
    if(out.native<0||c.score>p.raw[out.native].score)out.native=j;
    if(out.heldout_mean<0||coverage::mean(c.loss,mask)<coverage::mean(p.raw[out.heldout_mean].loss,mask))out.heldout_mean=j;
    if(supported(p,j,mask,prediction)){require(out.robust<0,"nonunique robust winner");out.robust=j;}
  }
  return out;
}
inline Decision decide(const Pool& p,const Mask& mask,const Pose& prediction,const Selection& selection){
  Decision out;out.prediction_fallback=rigid(prediction);
  if(supported(p,selection.robust,mask,prediction)){
    out.selected=selection.robust;out.recovery_success=true;out.status="ACCEPT_REPLACEMENT";
  }else if(admissible(p.raw[0],mask,prediction)){out.selected=0;out.status="ACCEPT_NOMINAL";}
  if(out.selected>=0){out.observation_valid=true;out.prediction_fallback=false;}
  return out;
}
} // namespace pool_v3
