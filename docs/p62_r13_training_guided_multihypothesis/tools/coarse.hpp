#pragma once
#include "../../p62_r11_cause_aware_recovery/tools/se3.hpp"
#include "../../p62_r6_dog_shadow_prototype/tools/native_kernel.hpp"
#include <numeric>

namespace training_grid {
using namespace shadow;
struct Seed {
  int id=0,selected_rank=0;double dx=0,dy=0,yaw_deg=0;
  Pose pose=Pose::Identity(),native_pose=Pose::Identity();
  double loss=NAN,overlap=NAN;bool local_excluded=false;
  std::string reason="UNSCORED";
};
inline bool locallyExcluded(double translation_m,double rotation_deg){return translation_m<.25&&rotation_deg<4.;}
inline bool sufficientlySeparated(double translation_m,double rotation_deg){return translation_m>=.30||rotation_deg>=4.;}
inline std::vector<Seed> grid(const Pose& prediction){
  require(rigid(prediction),"invalid common prediction");std::vector<Seed> result;
  for(double x:{-.4,0.,.4})for(double y:{-.4,0.,.4})for(double yaw:{-10.,-5.,0.,5.,10.}){
    Seed s;s.id=result.size();s.dx=x;s.dy=y;s.yaw_deg=yaw;
    Vector6 xi=Vector6::Zero();xi<<x,y,0,0,0,yaw*PI/180;
    s.pose=prediction*cause::expSE3(xi);s.native_pose=s.pose.cast<float>().cast<double>();
    require(rigid(s.pose)&&translation(s.pose,prediction)<=.60&&rotation(s.pose,prediction)<=12.,"invalid double grid seed STOP");
    require(rigid(s.native_pose)&&translation(s.native_pose,prediction)<=.60&&rotation(s.native_pose,prediction)<=12.,"invalid actual float grid seed STOP");
    result.push_back(s);
  }
  require(result.size()==45,"fixed45 grid");return result;
}
inline std::vector<size_t> subset(const Cloud& training){
  require(!training.empty(),"no training points");
  for(const auto& p:training)require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&sector(p.x,p.y)%2==0,"coarse input must contain only finite even-sector training points");
  const size_t n=std::min(size_t(256),training.size());std::vector<size_t> indices;
  for(size_t i=0;i<n;++i)indices.push_back(n==1?0:size_t(std::round(double(i)*(training.size()-1)/(n-1))));
  require(std::adjacent_find(indices.begin(),indices.end())==indices.end(),"duplicate subset index");return indices;
}
template<class SquaredNearest> inline size_t score(std::vector<Seed>& seeds,const Cloud& training,const std::vector<size_t>& sample,const SquaredNearest& nearest){
  require(seeds.size()==45&&!sample.empty()&&sample.size()<=256,"fixed training score shape");size_t queries=0;
  for(auto& s:seeds){double sum=0;size_t hits=0;
    for(size_t i:sample){require(i<training.size(),"bad training index");const auto& p=training[i];
      require(sector(p.x,p.y)%2==0,"heldout point in coarse score");
      const Eigen::Vector3d y=s.native_pose.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+s.native_pose.block<3,1>(0,3);
      require(y.allFinite(),"nonfinite coarse transform");const double d2=nearest(y);++queries;
      require(std::isfinite(d2)&&d2>=0,"invalid coarse squared nearest distance");sum+=std::min(d2/.25,1.);hits+=d2<=.25;
    }
    s.loss=sum/sample.size();s.overlap=double(hits)/sample.size();s.reason="SCORED_NOT_SELECTED";
  }
  return queries;
}
inline std::vector<int> select(std::vector<Seed>& seeds,const Pose& nominal){
  require(rigid(nominal)&&seeds.size()==45,"invalid nominal/grid selection");std::vector<int> ranked,selected;
  for(size_t i=0;i<seeds.size();++i){auto& s=seeds[i];
    require(s.id==int(i)&&std::isfinite(s.loss)&&s.loss>=0&&s.loss<=1&&std::isfinite(s.overlap)&&s.overlap>=0&&s.overlap<=1,"unscored or invalid grid");
    s.selected_rank=0;s.local_excluded=locallyExcluded(translation(s.native_pose,nominal),rotation(s.native_pose,nominal));
    s.reason=s.local_excluded?"LOCAL_EXCLUDED":"LOWER_RANK";if(!s.local_excluded)ranked.push_back(i);
  }
  std::sort(ranked.begin(),ranked.end(),[&](int i,int j){const auto& a=seeds[i];const auto& b=seeds[j];
    if(a.loss!=b.loss)return a.loss<b.loss;if(a.overlap!=b.overlap)return a.overlap>b.overlap;return a.id<b.id;});
  for(int i:ranked){
    if(selected.empty()){selected.push_back(i);continue;}
    const auto& first=seeds[selected[0]].native_pose;
    if(sufficientlySeparated(translation(seeds[i].native_pose,first),rotation(seeds[i].native_pose,first))){
      if(selected.size()<2)selected.push_back(i);
    }else seeds[i].reason="DIVERSITY_TOO_CLOSE";
  }
  for(size_t k=0;k<selected.size();++k){seeds[selected[k]].selected_rank=k+1;seeds[selected[k]].reason="SELECTED";}
  require(selected.size()<=2,"maximum2 selected seeds");return selected;
}
} // namespace training_grid
