#pragma once
#include "../../p62_r6_dog_shadow_prototype/tools/native_kernel.hpp"

namespace coverage {
using namespace shadow;
struct Mask {
  std::vector<int> active;
  size_t points=0,all_validation=0;bool training_ok=false;
  explicit Mask(const Split& s):training_ok(s.training->size()>=300){
    for(int b=0;b<8;++b){all_validation+=s.blocks[b]->size();if(s.blocks[b]->size()>=10){active.push_back(b);points+=s.blocks[b]->size();}}
  }
  bool evaluable()const{return training_ok&&active.size()>=4;}
  bool valid(const Loss& a)const{if(!evaluable())return false;for(int b:active)if(!std::isfinite(a[b])||a[b]<0||a[b]>1)return false;return true;}
};
inline double mean(const Loss& a,const Mask& m){require(m.valid(a),"invalid active loss");double sum=0;for(int b:m.active)sum+=a[b];return sum/m.active.size();}
inline double median(const Loss& a,const Mask& m){require(m.valid(a),"invalid active loss");std::vector<double> v;for(int b:m.active)v.push_back(a[b]);std::sort(v.begin(),v.end());const int n=v.size();return n%2?v[n/2]:(v[n/2-1]+v[n/2])/2;}
inline double paired(const Loss& j,const Loss& k,const Mask& m){
  require(m.valid(j)&&m.valid(k),"unavailable common block evidence");
  if(m.active.size()==8)return shadow::paired(j,k); // strict V1 double arithmetic
  std::vector<double> d;for(int b:m.active)d.push_back(k[b]-j[b]);std::sort(d.begin(),d.end());
  const double cap=2./d.size();const size_t full=d.size()/2;double sum=0;
  for(size_t i=0;i<full;++i)sum+=cap*d[i];if(d.size()%2)sum+=(1.-cap*full)*d[full];return sum;
}
inline void validate(Candidate& c,const Split& s,const Mask& m,pcl::KdTreeFLANN<pcl::PointXYZ>& tree,const Pose& prediction){
  if(!c.finite||!m.evaluable())return;
  if(m.active.size()==8){shadow::validate(c,s,tree,prediction);return;}
  const auto start=Clock::now();size_t hits=0;std::vector<int> ids(1);std::vector<float> distance(1);
  for(int b:m.active){double sum=0;
    for(const auto& p:*s.blocks[b]){const Eigen::Vector3d y=c.pose.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+c.pose.block<3,1>(0,3);
      require(y.allFinite(),"nonfinite active transform");require(tree.nearestKSearch(pcl::PointXYZ(y.x(),y.y(),y.z()),1,ids,distance)==1&&std::isfinite(distance[0]),"invalid active NN");
      sum+=std::min(double(distance[0])/.25,1.);if(distance[0]<=.25f)++hits;}
    c.loss[b]=sum/s.blocks[b]->size();}
  c.loss_valid=m.valid(c.loss);c.overlap=double(hits)/m.points;
  c.eligible=c.finite&&c.converged&&bounded(c.pose,prediction)&&c.loss_valid&&c.overlap>=.50;c.validation_ms=milliseconds(start);
}
inline Selection select(const std::vector<Candidate>& c,const Mask& m){
  Selection out;if(!m.evaluable())return out;const auto ids=pool(c);
  for(int j:ids){if(!c[j].eligible)continue;
    if(out.native<0||c[j].score>c[out.native].score)out.native=j;
    if(out.heldout_mean<0||mean(c[j].loss,m)<mean(c[out.heldout_mean].loss,m))out.heldout_mean=j;
    if(j==0||ids.size()<2)continue;bool dominates=true;for(int k:ids)if(k!=j&&!(paired(c[j].loss,c[k].loss,m)>.02))dominates=false;
    if(dominates){require(out.robust<0,"nonunique strict winner");out.robust=j;}}
  return out;
}
inline Pose scaledSeed(const Pose& center,const Vector6& direction,double sign,bool large){
  if(!large)return shadow::seed(center,direction,sign);
  require(rigid(center)&&direction.allFinite()&&direction.norm()>1e-12,"invalid large direction");
  const double t=.8*direction.head<3>().norm(),r=direction.tail<3>().norm();double scale=std::numeric_limits<double>::infinity();
  if(t>1e-12)scale=std::min(scale,1.2/t);if(r>1e-12)scale=std::min(scale,(8*PI/180)/r);require(std::isfinite(scale),"zero large direction");
  Pose result=center;result.block<3,1>(0,3)+=.8*scale*sign*direction.head<3>();const Eigen::Vector3d rv=scale*sign*direction.tail<3>();
  if(rv.norm()>0)result.block<3,3>(0,0)=Eigen::AngleAxisd(rv.norm(),rv.normalized()).toRotationMatrix()*center.block<3,3>(0,0);return result;
}
} // namespace coverage
