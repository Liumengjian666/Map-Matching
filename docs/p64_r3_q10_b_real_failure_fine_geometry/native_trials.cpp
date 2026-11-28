// Use the exact R1 native target and NDT configuration for isolated trials.
#include "scoring_profile.hpp"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

q1::Cloud::Ptr read_cloud(const std::string& path) {
  q1::Cloud::Ptr result(new q1::Cloud());
  if (pcl::io::loadPCDFile(path,*result)<0 || result->empty()) throw std::runtime_error(path);
  return result;
}

int main(int argc,char** argv) {
  if (argc!=4) return 2;
  try {
    q1::Scorer native(read_cloud(argv[1]));
    std::ifstream jobs(argv[2]); std::ofstream out(argv[3]);
    if (!jobs || !out) return 2;
    out<<"id,points,converged,iterations,seed_associated,associated,seed_native,native,fit_s,pose,information,exact_native_hessian\n"<<std::setprecision(17);
    std::string tag,source,mode;
    while(jobs>>tag>>mode>>source) {
      const auto original=read_cloud(source);
      if(mode=="DUMP") {
        std::string target; jobs>>target;
        if (pcl::io::savePCDFileBinary(target,*q1::downsample(original))<0) return 3;
        continue;
      }
      q1::Pose t;
      for(int i=0;i<4;++i) for(int j=0;j<4;++j) if(!(jobs>>t(i,j))) return 2;
      const auto seed=native.fixed(*original,t);
      const auto start=std::chrono::steady_clock::now();
      if(mode=="FIT") {
        q1::Cloud final;
        native.setInputSource(original);
        native.align(final,t);
        t=native.getFinalTransformation();
      }
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      std::vector<q1::PointScore> points;
      auto fit=native.fixed(*original,t,&points);
      const auto it=mode=="FIT"?native.getFinalNumIteration():0;
      const auto ok=mode=="FIT"&&native.hasConverged();
      q1::Mat6 H=q1::Mat6::Zero(), exact=q1::Mat6::Zero();
      if(mode=="AUDIT") {
        native.oracle(original,t,1,&exact);
        for(const auto& p:points) {
          const Eigen::Vector3d local=p.transformed.getVector3fMap().cast<double>()-t.block<3,1>(0,3).cast<double>();
          Eigen::Matrix3d cross;
          cross<<0,-local.z(),local.y(),local.z(),0,-local.x(),-local.y(),local.x(),0;
          Eigen::Matrix<double,3,6> J; J<<Eigen::Matrix3d::Identity(),-cross/5.;
          for(const auto& term:p.neighborhoods)
            H.noalias()+=term.score*J.transpose()*native.cells().at(term.cell).getInverseCov()*J;
        }
      }
      out<<tag<<','<<original->size()<<','<<ok<<','<<it<<','<<seed.associated<<','<<fit.associated<<','
        <<seed.native<<','<<fit.native<<','<<elapsed<<',';
      auto dump=[&](const auto& x) {for(int i=0;i<x.rows();++i)for(int j=0;j<x.cols();++j)
        out<<(i||j?";":"")<<x(i,j);};
      dump(t);out<<',';dump(H);out<<',';dump(exact);out<<'\n';
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
