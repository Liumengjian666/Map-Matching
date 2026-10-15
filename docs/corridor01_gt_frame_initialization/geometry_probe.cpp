// Bounded P34 diagnostic: two fixed NN queries, at most one nominal PCL NDT.
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include "../../src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Eigen::Matrix4d readPose(std::istream &in) {
  Eigen::Matrix4d T;
  for(int r=0;r<4;++r) for(int c=0;c<4;++c)
    if(!in.read(reinterpret_cast<char*>(&T(r,c)),sizeof(double))) throw std::runtime_error("POSE_BYTES");
  const auto R=T.block<3,3>(0,0);
  if(!T.allFinite() || (T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()!=0 ||
      (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()>1e-5 || std::abs(R.determinant()-1)>1e-5)
    throw std::runtime_error("INVALID_RIGID_POSE");
  return T;
}

void writePose(std::ostream &out,const Eigen::Matrix4d &T) {
  for(int r=0;r<4;++r) for(int c=0;c<4;++c) out<<','<<T(r,c);
}

void writeStats(std::ostream &out,const Cloud::ConstPtr &source,const Eigen::Matrix4d &T,
                pcl::KdTreeFLANN<pcl::PointXYZ> &tree) {
  Cloud transformed; pcl::transformPointCloud(*source,transformed,T.cast<float>());
  std::vector<double> distances; std::vector<int> id(1);std::vector<float> sq(1);
  Eigen::Vector3f low=Eigen::Vector3f::Constant(INFINITY),high=-low;
  size_t half=0,two=0,eight=0;
  for(const auto &p:transformed) {
    if(!pcl::isFinite(p) || tree.nearestKSearch(p,1,id,sq)!=1) throw std::runtime_error("NN_FAILED");
    const double d=std::sqrt(sq[0]); distances.push_back(d);
    half+=d<=.5;two+=d<=2.;eight+=d<=8.;
    low=low.cwiseMin(p.getVector3fMap());high=high.cwiseMax(p.getVector3fMap());
  }
  std::sort(distances.begin(),distances.end());
  out<<','<<distances[distances.size()/2]<<','<<distances[size_t(.9*(distances.size()-1))]
     <<','<<double(half)/distances.size()<<','<<double(two)/distances.size()<<','<<double(eight)/distances.size();
  for(int k=0;k<3;++k) out<<','<<low(k)<<','<<high(k);
}

int main(int argc,char **argv) {
  if(argc!=5) return 2;
  try {
    const std::string input=argv[1],map=argv[2],output=argv[3];
    const bool run_ndt=std::string(argv[4])=="--one-H2-ndt";
    if(!run_ndt && std::string(argv[4])!="--NN-only") throw std::runtime_error("MODE");
    if(std::ifstream(output+"/geometry.csv") || std::ifstream(output+"/ndt.csv")) throw std::runtime_error("NO_OVERWRITE");
    std::ifstream poses(input+"/hypotheses.f64",std::ios::binary);
    const Eigen::Matrix4d H1=readPose(poses),H2=readPose(poses);
    if(poses.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("EXTRA_POSES");
    std::ifstream raw(input+"/selected_raw.xyz",std::ios::binary|std::ios::ate);
    if(!raw || raw.tellg()<=0 || raw.tellg()%12!=0) throw std::runtime_error("XYZ_BYTES");
    raw.seekg(0);Cloud::Ptr filtered(new Cloud);float p[3];size_t raw_count=0;
    while(raw.read(reinterpret_cast<char*>(p),12)) {
      ++raw_count;
      const double range=std::sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
      if(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) && range>=.5 && range<=80.)
        filtered->push_back(pcl::PointXYZ(p[0],p[1],p[2]));
    }
    p23::finalizeCloud(filtered);p23::FrozenNdtRules rules;
    const auto source=rules.voxelDown(filtered,.25,.25,1400);
    Cloud::Ptr original(new Cloud),finite(new Cloud);
    if(pcl::io::loadPCDFile(map,*original)<0) throw std::runtime_error("MAP_READ");
    for(const auto &p:*original) if(pcl::isFinite(p)) finite->push_back(p);
    p23::finalizeCloud(finite);
    const auto coarse=rules.voxelDown(finite,.15,.15,0);
    const auto target=rules.voxelDown(coarse,.15,.15,0);
    if(source->size()<50 || target->size()<50) throw std::runtime_error("POINT_COUNT");
    std::ofstream xyz(output+"/preprocessed_source.xyz",std::ios::binary);
    for(const auto &p:*source) for(float v:{p.x,p.y,p.z}) xyz.write(reinterpret_cast<const char*>(&v),4);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(target);
    std::ofstream out(output+"/geometry.csv");out<<std::setprecision(17)
      <<"hypothesis,source_points,target_points,nn_median_m,nn_p90_m,coverage_0_5m,coverage_2m,coverage_8m,xmin,xmax,ymin,ymax,zmin,zmax";
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) out<<",T"<<r<<c;out<<'\n';
    int queries=0;
    for(const auto &pair:std::vector<std::pair<const char*,Eigen::Matrix4d>>{{"H1_GT_LIDAR_W_EQ_M",H1},{"H2_GT_IMU_W_EQ_M",H2}}) {
      ++queries;out<<pair.first<<','<<source->size()<<','<<target->size();
      writeStats(out,source,pair.second,tree);writePose(out,pair.second);out<<'\n';
    }
    int aligns=0;
    if(run_ndt) {
      pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt;
      ndt.setInputTarget(target);ndt.setInputSource(source);
      ndt.setResolution(.8);ndt.setStepSize(.08);ndt.setTransformationEpsilon(.001);ndt.setMaximumIterations(40);
      Cloud aligned;const auto tick=std::chrono::steady_clock::now();
      ++aligns;ndt.align(aligned,H2.cast<float>());
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
      const Eigen::Matrix4d result=ndt.getFinalTransformation().cast<double>();
      if(!result.allFinite()) throw std::runtime_error("NONFINITE_RESULT");
      std::ofstream result_csv(output+"/ndt.csv");result_csv<<std::setprecision(17)
        <<"hypothesis,converged,iterations,fitness,translation_correction_m,rotation_correction_deg,align_ms";
      for(const char *tag:{"initial_","final_"}) for(int r=0;r<4;++r) for(int c=0;c<4;++c) result_csv<<','<<tag<<r<<c;
      result_csv<<"\nH2_GT_IMU_W_EQ_M,"<<ndt.hasConverged()<<','<<ndt.getFinalNumIteration()<<','<<ndt.getFitnessScore()
        <<','<<(result.block<3,1>(0,3)-H2.block<3,1>(0,3)).norm()<<','
        <<Eigen::AngleAxisd(H2.block<3,3>(0,0).transpose()*result.block<3,3>(0,0)).angle()*180./M_PI<<','<<ms;
      writePose(result_csv,H2);writePose(result_csv,result);result_csv<<'\n';
    }
    if(queries!=2 || aligns>1) throw std::runtime_error("BUDGET");
    std::cout<<"NN_queries="<<queries<<" full_NDT_aligns="<<aligns<<" source_points="<<source->size()
      <<" target_points="<<target->size()<<" source_stable_hash="<<std::hex<<p23::FrozenNdtRules::stableCloudHash(source)<<'\n';
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
  return 0;
}
