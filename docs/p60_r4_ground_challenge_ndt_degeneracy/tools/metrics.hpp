#pragma once
#include "cloud_contract.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>

namespace screen {
using namespace dog_prior_map_fastlio2_frontend_exp;
inline double ms(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();
}
inline bool rigid(const Eigen::Matrix4f& p) {
  if (!p.allFinite()) return false;
  const Eigen::Matrix3f R=p.block<3,3>(0,0);
  return (R.transpose()*R-Eigen::Matrix3f::Identity()).norm()<1e-3 && std::abs(R.determinant()-1)<1e-3;
}
struct Csv {
  std::ofstream out; bool first=true;
  explicit Csv(const std::string& path):out(path) { if (!out) throw std::runtime_error("output open failed"); out<<std::setprecision(17); }
  template<class T> void col(const T& value) { if (!first) out<<','; out<<value; first=false; }
  void end() { out<<'\n'; out.flush(); first=true; }
};
inline void metricHeader(Csv& csv) {
  for (auto s : {"scan_index","stamp_ns","arm","evaluation_pose","source_points","target_points","source_hash","target_hash",
      "pose_rigid","reference_converged","jet_valid","supported_points","support_fraction","comparable","spd",
      "score_sum","mean_energy","gradient_norm","ratio6","condition6","ratio_translation","ratio_rotation","jet_ms","jet_calls"}) csv.col(s);
  for(int i=0;i<6;++i) csv.col("lambda"+std::to_string(i));
  for(int i=0;i<6;++i) csv.col("weak"+std::to_string(i));
  for(int i=0;i<3;++i) csv.col("translation_lambda"+std::to_string(i));
  for(int i=0;i<3;++i) csv.col("rotation_lambda"+std::to_string(i));
  for(int i=0;i<3;++i) csv.col("translation_weak"+std::to_string(i));
  for(int i=0;i<6;++i) for(int j=0;j<6;++j) csv.col("H"+std::to_string(i)+std::to_string(j));
  csv.end();
}
inline CoupledLocalJet measure(Csv& csv, int index, const std::string& stamp, const std::string& arm,
    const std::string& evaluation, Ndt& ndt, Cloud::Ptr src, Cloud::Ptr dst,
    const Eigen::Matrix4f& pose, bool converged) {
  CoupledLocalJet jet;
  size_t support=0, calls=0;
  const auto start=std::chrono::steady_clock::now();
  if (rigid(pose) && src->size()>=30) {
    // computeDerivatives also dereferences internal input_: set it for EACH arm.
    ndt.setInputSource(src);
    support=ndt.supported(src,pose);
    CoupledNdtBackend backend;
    backend.jet=[&](const Eigen::Matrix4f& p,const CoupledVector6& parameters) {
      ++calls; return ndt.nativeJet(src,p,parameters);
    };
    jet=coupledNominalJet(pose,src->size(),backend,.8);
  }
  const double elapsed=ms(start), fraction=src->empty()?0.:double(support)/src->size();
  Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> tr,rot;
  bool valid=jet.valid;
  if (valid) {
    eig.compute(jet.H); tr.compute(jet.H.topLeftCorner<3,3>()); rot.compute(jet.H.bottomRightCorner<3,3>());
    valid=eig.info()==Eigen::Success && tr.info()==Eigen::Success && rot.info()==Eigen::Success && eig.eigenvalues().allFinite();
  }
  const bool spd=valid && eig.eigenvalues()[0]>0;
  for(auto x:{std::to_string(index),stamp,arm,evaluation,std::to_string(src->size()),std::to_string(dst->size()),hash(src),hash(dst)}) csv.col(x);
  csv.col(rigid(pose)); csv.col(converged); csv.col(valid); csv.col(support); csv.col(fraction);
  csv.col(valid && rigid(pose) && converged && src->size()>=30 && fraction>=.5 && std::abs(jet.score)>1e-12);
  csv.col(spd); csv.col(jet.score); csv.col(-jet.score/std::max<size_t>(src->size(),1)); csv.col(jet.gradient.norm());
  csv.col(spd?eig.eigenvalues()[0]/eig.eigenvalues()[5]:NAN);
  csv.col(spd?eig.eigenvalues()[5]/eig.eigenvalues()[0]:NAN);
  csv.col(valid && tr.eigenvalues()[0]>0?tr.eigenvalues()[0]/tr.eigenvalues()[2]:NAN);
  csv.col(valid && rot.eigenvalues()[0]>0?rot.eigenvalues()[0]/rot.eigenvalues()[2]:NAN);
  csv.col(elapsed); csv.col(calls);
  for(int i=0;i<6;++i) csv.col(valid?eig.eigenvalues()[i]:NAN);
  for(int i=0;i<6;++i) csv.col(valid?eig.eigenvectors()(i,0):NAN);
  for(int i=0;i<3;++i) csv.col(valid?tr.eigenvalues()[i]:NAN);
  for(int i=0;i<3;++i) csv.col(valid?rot.eigenvalues()[i]:NAN);
  for(int i=0;i<3;++i) csv.col(valid?tr.eigenvectors()(i,0):NAN);
  for(int i=0;i<6;++i) for(int j=0;j<6;++j) csv.col(valid?jet.H(i,j):NAN);
  csv.end(); return jet;
}
} // namespace screen
