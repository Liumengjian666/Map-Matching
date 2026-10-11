#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include <pcl/point_types.h>
#include <iostream>
int main() {
  using Point=pcl::PointXYZ;
  using Ndt=dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<Point>;
  pcl::PointCloud<Point>::Ptr target(new pcl::PointCloud<Point>);
  uint32_t seed=0x13579bdfU;
  auto random=[&]() {seed=1664525U*seed+1013904223U;return float(seed)/float(UINT32_MAX);};
  for(int i=0;i<6000;++i) target->push_back(Point(6*random()-3,6*random()-3,3*random()-1.5));
  target->width=target->size();target->height=1;target->is_dense=true;
  // Two maps ONLY in this synthetic unit test; runtime owns one NDT/map.
  Ndt a,b;
  for(auto* n:{&a,&b}) {
    n->setInputTarget(target);n->setInputSource(target);n->setResolution(.8);
    n->setStepSize(.08);n->setTransformationEpsilon(.001);n->setMaximumIterations(40);
  }
  pcl::PointCloud<Point> aligned;
  Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();guess(0,3)=.08;
  for(int frame=0;frame<2;++frame) {
    a.align(aligned,guess);b.align(aligned,guess);
    if(a.getFinalTransformation()!=b.getFinalTransformation() ||
       a.getFinalNumIteration()!=b.getFinalNumIteration() ||
       a.hasConverged()!=b.hasConverged() || a.getFitnessScore()!=b.getFitnessScore()) return 1;
    const auto raw=b.getFinalTransformation();
    const auto jet=b.nativeJet(target,raw,dog_prior_map_fastlio2_frontend_exp::CoupledVector6::Zero());
    if(!jet.valid || b.getFinalTransformation()!=raw) return 2;
    guess=a.getFinalTransformation();
  }
  std::cout<<"TWO_FRAME_NDT_READONLY_EXACT_PARITY_PASS\n";return 0;
}
