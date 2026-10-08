// Startup-only local odometry. No NDT, GT, baseline update, or source deskew.
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace p = dog_prior_map_fastlio2_frontend_exp;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
Cloud::Ptr cloud(const p::P7TimedLidarVector& raw) {
  Cloud::Ptr in(new Cloud), out(new Cloud);
  for(const auto& point:raw) {
    const double range=point.position.norm();
    if(point.position.allFinite() && range>=.5 && range<=80.)
      in->push_back(pcl::PointXYZ(point.position.x(),point.position.y(),point.position.z()));
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(in); voxel.setLeafSize(.25f,.25f,.25f); voxel.filter(*out);
  return out;
}
int main(int argc,char** argv) {
  try {
    if(argc!=3 && (argc!=4 || std::string(argv[3])!="--diagnostic-only"))
      throw std::runtime_error("usage: INPUT_DIRECTORY NEW_ODOMETRY_CSV [--diagnostic-only]");
    const bool diagnostic_only=argc==4;
    std::ifstream exists(argv[2]); if(exists.good())throw std::runtime_error("output_already_exists");
    const std::string dir=argv[1];
    const auto scans=p::readP7TimedScans(dir+"/filter_scans.csv",dir+"/raw_timed_scan_index.csv");
    const uint64_t deadline=scans.front().scan_start_ns+10000000000ULL;
    std::ofstream out(argv[2]); if(!out)throw std::runtime_error("output_write_failed");
    out<<"transaction_id,scan_start_ns,stamp_ns,raw_points,voxel_points,converged,solver_converged,diagnostic_only,fitness_m2,pair_translation_m,pair_rotation_deg,wall_s";
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<",T"<<i<<j;
    out<<'\n'<<std::setprecision(17);
    Cloud::Ptr previous; Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
    for(const auto& scan:scans) {
      if(scan.scan_end_ns>deadline)break;
      const auto start=std::chrono::steady_clock::now();
      auto current=cloud(p::readP7PackedTimedCloud(dir+"/raw_timed_points.bin",scan));
      bool ok=true,solver_ok=true; double fitness=0,translation=0,rotation=0;
      if(previous) {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
        gicp.setInputSource(current); gicp.setInputTarget(previous);
        gicp.setMaxCorrespondenceDistance(2.5); gicp.setMaximumIterations(40);
        gicp.setTransformationEpsilon(1e-6); Cloud aligned;
        gicp.align(aligned,Eigen::Matrix4f::Identity());
        const Eigen::Matrix4f relative=gicp.getFinalTransformation();
        translation=relative.block<3,1>(0,3).norm();
        rotation=Eigen::AngleAxisf(relative.block<3,3>(0,0)).angle()*180./M_PI;
        fitness=gicp.getFitnessScore();
        solver_ok=gicp.hasConverged()&&relative.allFinite();
        ok=solver_ok&&fitness<=.10&&translation<3&&rotation<45;
        if(ok || (diagnostic_only&&solver_ok))pose=pose*relative;
      }
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      out<<scan.transaction_id<<','<<scan.scan_start_ns<<','<<scan.scan_end_ns<<','<<scan.cloud_point_count<<','
         <<current->size()<<','<<ok<<','<<solver_ok<<','<<diagnostic_only<<','<<fitness<<','<<translation<<','<<rotation<<','<<elapsed;
      for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<pose(i,j);
      out<<'\n'; out.flush();
      std::cout<<"BOOTSTRAP_ONLY tx="<<scan.transaction_id<<" accepted="<<ok<<" wall="<<elapsed<<'\n';
      if(!solver_ok || (!ok&&!diagnostic_only))
        throw std::runtime_error("local_odometry_admission_failed; no gap skipping permitted");
      previous=current;
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
