#pragma once
#include "contract.hpp"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace shadow {
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
using Clock=std::chrono::steady_clock;
inline double milliseconds(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
inline void finish(const Cloud::Ptr& c){c->width=c->size();c->height=1;c->is_dense=true;}
inline Cloud::Ptr voxel(const Cloud::Ptr& c,float leaf,int cap=0){
  Cloud::Ptr out(new Cloud);pcl::VoxelGrid<pcl::PointXYZ> filter;filter.setLeafSize(leaf,leaf,leaf);filter.setInputCloud(c);filter.filter(*out);
  if(cap>0&&out->size()>size_t(cap)){Cloud::Ptr sampled(new Cloud);const double step=double(out->size()-1)/std::max(cap-1,1);
    for(int i=0;i<cap;++i)sampled->push_back(out->points[size_t(std::round(i*step))]);out=sampled;}
  finish(out);return out;
}
inline std::string cloudHash(const Cloud::Ptr& c){
  uint64_t h=1469598103934665603ULL;
  auto byte=[&](uint8_t x){h=(h^x)*1099511628211ULL;};
  auto u32=[&](uint32_t x){for(int b=0;b<32;b+=8)byte(uint8_t(x>>b));};
  u32(c->width);u32(c->height);u32(c->is_dense?1:0);u32(c->size());
  for(const auto& p:*c)for(float x:{p.x,p.y,p.z}){uint32_t bits;std::memcpy(&bits,&x,4);u32(bits);}
  std::ostringstream o;o<<std::hex<<std::setw(16)<<std::setfill('0')<<h;return o.str();
}
struct Split {
  Cloud::Ptr training{new Cloud};std::array<Cloud::Ptr,8> blocks;
  Split(){for(auto& b:blocks)b.reset(new Cloud);}
  bool evaluable()const{if(training->size()<300)return false;for(const auto& b:blocks)if(b->size()<10)return false;return true;}
};
inline Split splitSource(const Cloud::Ptr& c){
  Split s;for(const auto& p:*c){const int b=sector(p.x,p.y);if(b%2)s.blocks[b/2]->push_back(p);else s.training->push_back(p);}
  finish(s.training);for(auto& b:s.blocks)finish(b);return s;
}
class NativeNdt : public pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> {
 public:
  void configure(const Cloud::Ptr& map){setResolution(.8f);setStepSize(.08);setTransformationEpsilon(.001);setMaximumIterations(40);setOulierRatio(.55);setInputTarget(map);checkGrid();}
  void checkGrid()const{const auto leaf=this->target_cells_.getLeafSize();require(leaf.x()==.8f&&leaf.y()==.8f&&leaf.z()==.8f,"actual Gaussian Grid not0.8m");}
  Matrix6 geometry(const Cloud::Ptr& training,const Pose& t,size_t& supported){
    require(rigid(t)&&!training->empty(),"invalid G inputs");Matrix6 g=Matrix6::Zero();supported=0;
    std::vector<TargetGrid::LeafConstPtr> leaves;std::vector<float> distances;
    for(const auto& p:*training){
      const Eigen::Vector3d rx=t.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z),y=rx+t.block<3,1>(0,3);
      pcl::PointXYZ query(y.x(),y.y(),y.z());leaves.clear();distances.clear();
      this->target_cells_.radiusSearch(query,this->resolution_,leaves,distances);if(!leaves.empty())++supported;
      Eigen::Matrix<double,3,6> jac;Eigen::Matrix3d skew;
      skew<<0,-rx.z(),rx.y(),rx.z(),0,-rx.x(),-rx.y(),rx.x(),0;
      jac.leftCols<3>()=.8*Eigen::Matrix3d::Identity();jac.rightCols<3>()=-skew;
      for(const auto& leaf:leaves){const Eigen::Vector3d r=y-leaf->getMean();const auto& cov=leaf->getInverseCov();
        const double exponential=std::exp(-this->gauss_d2_*r.dot(cov*r)/2);
        const double guard=this->gauss_d2_*exponential;
        if(!std::isfinite(guard)||guard<0||guard>1)continue;
        const double a=-this->gauss_d1_*this->gauss_d2_*exponential;
        if(a>=0&&std::isfinite(a))g.noalias()+=a*jac.transpose()*cov*jac;
      }
    }
    g/=training->size();return .5*(g+g.transpose()).eval();
  }
};
inline Candidate align(NativeNdt& ndt,const Cloud::Ptr& source,const Pose& initial,int id){
  require(rigid(initial)&&source->size()>0,"invalid alignment input");ndt.checkGrid();ndt.setInputSource(source);Cloud aligned;
  const auto start=Clock::now();ndt.align(aligned,initial.cast<float>());
  Candidate c;c.id=id;c.initial=initial;c.align_ms=milliseconds(start);c.pose=ndt.getFinalTransformation().cast<double>();
  c.converged=ndt.hasConverged();c.iterations=ndt.getFinalNumIteration();c.score=ndt.getTransformationProbability();c.fitness=ndt.getFitnessScore();
  c.finite=rigid(c.pose)&&std::isfinite(c.score)&&std::isfinite(c.fitness);ndt.checkGrid();return c;
}
inline void validate(Candidate& c,const Split& source,pcl::KdTreeFLANN<pcl::PointXYZ>& tree,const Pose& prediction){
  if(!c.finite||!source.evaluable())return;const auto start=Clock::now();size_t all=0,hits=0;
  std::vector<int> ids(1);std::vector<float> distance(1);
  for(int b=0;b<8;++b){double sum=0;
    for(const auto& p:*source.blocks[b]){const Eigen::Vector3d y=c.pose.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+c.pose.block<3,1>(0,3);
      require(y.allFinite(),"nonfinite heldout transform");pcl::PointXYZ q(y.x(),y.y(),y.z());
      require(tree.nearestKSearch(q,1,ids,distance)==1&&std::isfinite(distance[0]),"invalid heldout NN");
      sum+=std::min(double(distance[0])/.25,1.);++all;if(distance[0]<=.25f)++hits;}
    c.loss[b]=sum/source.blocks[b]->size();
  }
  c.loss_valid=validLoss(c.loss);c.overlap=double(hits)/all;
  c.eligible=c.finite&&c.converged&&bounded(c.pose,prediction)&&c.loss_valid&&c.overlap>=.50;
  c.validation_ms=milliseconds(start);
}
} // namespace shadow
