#pragma once
#include <dcreg.hpp>

struct DirectionFit {
  bool converged=false;
  bool triggered=false;
  Eigen::Vector3f axis=Eigen::Vector3f::Zero();
  double schur_condition=0.;
  int retained=0;
  int iterations=0;
  double selection_s=0;
  double fit_s=0;
  Pose pose;
};

DirectionFit fit_direction(const Cloud::ConstPtr& cloud,const Cloud::ConstPtr& world,
                          pcl::KdTreeFLANN<Point>& tree,InstrumentedScorer& ndt,
                          const Pose& native,const Eigen::Vector3f& axis,bool author_gate=false) {
  const auto begin=Clock::now();
  DirectionFit result;
  result.pose=native;
  result.axis=axis;
  result.triggered=!author_gate;
  Mat6 plane_information=Mat6::Zero();
  Eigen::Vector3f zero=Eigen::Vector3f::Zero();
  std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> normals(cloud->size(),zero);
  std::vector<float> contribution(cloud->size(),0.f);
  std::vector<int> neighbors(20);
  std::vector<float> distances(20);
  for(std::size_t i=0;i<cloud->size();++i){
    Point world_point=(*cloud)[i];
    world_point.getVector3fMap()=native.topLeftCorner<3,3>()*world_point.getVector3fMap()+native.topRightCorner<3,1>();
    const int n=tree.nearestKSearch(world_point,20,neighbors,distances);
    if(n!=20||distances.front()>.30f*.30f)continue;
    Eigen::Matrix<float,3,20> points;
    for(int j=0;j<20;++j)points.col(j)=(*world)[neighbors[j]].getVector3fMap();
    const auto center=points.rowwise().mean().eval();
    points.colwise()-=center;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(points*points.transpose());
    if(es.info()!=Eigen::Success)continue;
    normals[i]=es.eigenvectors().col(0);
    if(author_gate){
      const Eigen::Vector3d n=native.topLeftCorner<3,3>().transpose().cast<double>()*normals[i].cast<double>();
      const Eigen::Vector3d p=(*cloud)[i].getVector3fMap().cast<double>();
      Eigen::Matrix<double,6,1> j;
      j<<p.cross(n)/5.,n;
      plane_information.noalias()+=j*j.transpose();
    }
  }
  if(author_gate){
    const dcreg::SolverParameters params;
    const auto reg=dcreg::DetectDegeneracy(plane_information,params);
    result.schur_condition=reg.cond_schur_trans;
    result.triggered=reg.factorization_ok&&result.schur_condition>params.degeneracy_condition_threshold;
    if(!result.triggered){
      result.selection_s=elapsed(begin);
      return result;
    }
    result.axis=native.topLeftCorner<3,3>()*reg.raw_trans_basis.col(0).cast<float>();
  }
  for(std::size_t i=0;i<cloud->size();++i)
    contribution[i]=std::pow(normals[i].dot(result.axis),2.f);
  std::vector<std::size_t> ids(cloud->size());
  std::iota(ids.begin(),ids.end(),0);
  std::stable_sort(ids.begin(),ids.end(),[&](auto a,auto b){return contribution[a]>contribution[b];});
  ids.resize((ids.size()+1)/2);
  std::sort(ids.begin(),ids.end());
  auto chosen=Cloud::Ptr(new Cloud);
  chosen->reserve(ids.size());
  for(const auto i:ids)chosen->push_back((*cloud)[i]);
  result.retained=chosen->size();
  result.selection_s=elapsed(begin);
  const auto started=Clock::now();
  ndt.setInputSource(chosen);
  Cloud output;
  ndt.align(output,native);
  result.pose=ndt.getFinalTransformation();
  result.iterations=ndt.getFinalNumIteration();
  result.converged=ndt.hasConverged()&&result.iterations>0&&result.pose.allFinite();
  result.fit_s=elapsed(started);
  return result;
}
