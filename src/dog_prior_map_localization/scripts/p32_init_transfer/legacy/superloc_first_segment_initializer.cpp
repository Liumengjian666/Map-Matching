#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Transform = Eigen::Affine3f;

namespace {

Cloud::Ptr voxel(const Cloud::ConstPtr &in, float leaf) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(in); vg.setLeafSize(leaf, leaf, leaf); vg.filter(*out);
  return out;
}

double rotationAngle(const Eigen::Matrix3f &R) {
  const double c = std::max(-1.0, std::min(1.0, (static_cast<double>(R.trace()) - 1.0) * 0.5));
  return std::acos(c);
}

std::vector<Eigen::Matrix3f> properAxes() {
  std::vector<Eigen::Matrix3f> out;
  const std::array<int,3> p{0,1,2};
  std::array<int,3> a = p;
  do {
    int inv = 0; for (int i=0;i<3;++i) for (int j=i+1;j<3;++j) if (a[i]>a[j]) ++inv;
    for (int sx : {-1,1}) for (int sy : {-1,1}) for (int sz : {-1,1}) {
      Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
      M(0,a[0])=static_cast<float>(sx); M(1,a[1])=static_cast<float>(sy); M(2,a[2])=static_cast<float>(sz);
      if (M.determinant() > 0.5f) out.push_back(M);
    }
  } while (std::next_permutation(a.begin(), a.end()));
  return out;
}

void writeMatrix(std::ostream &os, const Eigen::Matrix3f &R) {
  os << "[" << std::setprecision(10)
     << R(0,0) << ", " << R(0,1) << ", " << R(0,2) << "; "
     << R(1,0) << ", " << R(1,1) << ", " << R(1,2) << "; "
     << R(2,0) << ", " << R(2,1) << ", " << R(2,2) << "]";
}

bool loadOfficialPrior(const std::string &path, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
  std::ifstream f(path); if (!f) return false;
  std::string line, all; while (std::getline(f,line)) all += line + "\n";
  auto data = [&](const std::string &key, int n) {
    std::vector<float> v; const auto pos = all.find(key); if (pos == std::string::npos) return v;
    const auto b = all.find("data:", pos); if (b == std::string::npos) return v;
    const auto e = all.find(']', b); if (e == std::string::npos) return v;
    std::stringstream ss(all.substr(b + 5, e - b - 5)); char c;
    while (ss && static_cast<int>(v.size()) < n) { float x; if (ss >> x) v.push_back(x); else ss.clear(); ss >> c; }
    return v;
  };
  const auto rv = data("extrinsicRotation_world_darpa",9), tv = data("extrinsicTranslation_world_darpa",3);
  if (rv.size()!=9 || tv.size()!=3) return false;
  R << rv[0],rv[1],rv[2],rv[3],rv[4],rv[5],rv[6],rv[7],rv[8];
  t << tv[0],tv[1],tv[2];
  return true;
}

struct Candidate { int id; Transform T; bool converged; double fitness, nn_median, nn_p90, inlier; };

Candidate registerCandidate(int id, const Transform &seed, const Cloud::ConstPtr &source, const Cloud::ConstPtr &target) {
  pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
  icp.setInputSource(source); icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(8.0); icp.setMaximumIterations(60); icp.setTransformationEpsilon(1e-7); icp.setEuclideanFitnessEpsilon(1e-6);
  Cloud aligned; icp.align(aligned, seed.matrix());
  Candidate c{id, Transform(icp.getFinalTransformation()), icp.hasConverged(), icp.getFitnessScore(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(target);
  std::vector<float> d; d.reserve(aligned.size()); int in=0; std::vector<int> idx(1); std::vector<float> dist(1);
  for (const auto &p : aligned.points) if (pcl::isFinite(p) && tree.nearestKSearch(p,1,idx,dist)>0) { const double x=std::sqrt(dist[0]); d.push_back(x); if (x<1.0) ++in; }
  if (!d.empty()) { std::sort(d.begin(),d.end()); c.nn_median=d[d.size()/2]; c.nn_p90=d[std::min(d.size()-1,static_cast<std::size_t>(0.90*(d.size()-1)))]; c.inlier=static_cast<double>(in)/d.size(); }
  return c;
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "superloc_first_segment_initializer", ros::init_options::AnonymousName);
  if (argc < 5) {
    std::cerr << "usage: superloc_first_segment_initializer ADAPTER_BAG MAP_PCD OFFICIAL_INIT OUT_DIR [duration_sec]\n";
    return 2;
  }
  const std::string bag_path=argv[1], map_path=argv[2], prior_path=argv[3], out_dir=argv[4];
  const double duration=argc>5 ? std::stod(argv[5]) : 5.0;
  Cloud::Ptr map(new Cloud); if (pcl::io::loadPCDFile(map_path,*map)<0) { std::cerr<<"map read failed\n"; return 3; }
  Cloud::Ptr map_ds=voxel(map,0.20f);
  rosbag::Bag bag; try { bag.open(bag_path,rosbag::bagmode::Read); } catch (...) { std::cerr<<"bag open failed\n"; return 4; }
  const std::vector<std::string> topics{"/superloc_adapter/points_deskewed"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<Cloud::Ptr> scans; std::vector<double> stamps; double first=-1;
  for (const auto &m : view) {
    auto msg=m.instantiate<sensor_msgs::PointCloud2>(); if (!msg) continue;
    const double s=msg->header.stamp.toSec(); if (first<0) first=s; if (s>first+duration) break;
    Cloud::Ptr c(new Cloud); pcl::fromROSMsg(*msg,*c); if (c->empty()) continue;
    scans.push_back(voxel(c,0.20f)); stamps.push_back(s);
  }
  bag.close();
  std::ofstream reg(out_dir+"/initializer_registration.csv");
  reg << "index,stamp,converged,fitness,translation_m,rotation_deg,used\n";
  if (scans.size()<2) { std::cerr<<"not enough scans\n"; return 5; }
  Cloud::Ptr local(new Cloud); *local += *scans.front(); Transform T_first=Transform::Identity(); local->clear(); *local += *scans.front();
  Cloud::Ptr previous=scans.front(); int used=1;
  for (std::size_t i=1;i<scans.size();++i) {
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp; icp.setInputSource(scans[i]); icp.setInputTarget(previous); icp.setMaxCorrespondenceDistance(2.5); icp.setMaximumIterations(40); icp.setTransformationEpsilon(1e-6);
    Cloud aligned; icp.align(aligned,Eigen::Matrix4f::Identity()); const Transform d(icp.getFinalTransformation());
    const double tr=d.translation().norm(), rd=rotationAngle(d.rotation())*180.0/M_PI; const bool ok=icp.hasConverged() && tr<3.0 && rd<45.0 && std::isfinite(icp.getFitnessScore());
    reg << i << ',' << stamps[i] << ',' << (icp.hasConverged()?1:0) << ',' << icp.getFitnessScore() << ',' << tr << ',' << rd << ',' << (ok?1:0) << '\n';
    if (ok) { T_first = T_first * d; Cloud::Ptr transformed(new Cloud); pcl::transformPointCloud(*scans[i],*transformed,T_first); *local += *transformed; previous=scans[i]; ++used; }
  }
  local=voxel(local,0.25f); pcl::io::savePCDFileBinary(out_dir+"/first_segment_local_submap.pcd",*local);
  Eigen::Matrix3f R; Eigen::Vector3f t; if (!loadOfficialPrior(prior_path,R,t)) { std::cerr<<"official prior parse failed\n"; return 6; }
  const auto axes=properAxes(); std::vector<Candidate> cands; cands.reserve(axes.size());
  for (std::size_t i=0;i<axes.size();++i) { Transform seed=Transform::Identity(); seed.linear()=R*axes[i]; seed.translation()=t; cands.push_back(registerCandidate(static_cast<int>(i),seed,local,map_ds)); }
  std::sort(cands.begin(),cands.end(),[](const Candidate&a,const Candidate&b){ if(a.converged!=b.converged)return a.converged>b.converged; return a.fitness<b.fitness; });
  std::ofstream summary(out_dir+"/initializer_candidates.csv"); summary<<"rank,candidate_id,converged,fitness,nn_median,nn_p90,inlier,translation_x,translation_y,translation_z,rotation_angle_deg\n";
  for (std::size_t r=0;r<cands.size();++r) summary<<r+1<<','<<cands[r].id<<','<<(cands[r].converged?1:0)<<','<<cands[r].fitness<<','<<cands[r].nn_median<<','<<cands[r].nn_p90<<','<<cands[r].inlier<<','<<cands[r].T.translation().x()<<','<<cands[r].T.translation().y()<<','<<cands[r].T.translation().z()<<','<<rotationAngle(cands[r].T.rotation())*180.0/M_PI<<'\n';
  const Candidate &best=cands.front(); std::ofstream init(out_dir+"/corridor01_init.yaml");
  init<<"source: FIRST_SEGMENT_WITH_OFFICIAL_PRIOR\n"<<"t_init_start: "<<std::setprecision(15)<<first<<"\n"<<"initialization_interval_sec: "<<duration<<"\n"<<"scans_available: "<<scans.size()<<"\n"<<"scans_used: "<<used<<"\n"<<"GT_used: false\n"<<"future_after_5s_used: false\n"<<"official_prior_semantics: OTHER_OFFICIAL_FRAME_TRANSFORM_UNRESOLVED\n"<<"selected_candidate_id: "<<best.id<<"\n"<<"T_map_lidar:\n  translation: ["<<best.T.translation().x()<<", "<<best.T.translation().y()<<", "<<best.T.translation().z()<<"]\n  rotation_matrix: "; writeMatrix(init,best.T.rotation()); init<<"\n  quaternion_xyzw: ["<<Eigen::Quaternionf(best.T.rotation()).x()<<", "<<Eigen::Quaternionf(best.T.rotation()).y()<<", "<<Eigen::Quaternionf(best.T.rotation()).z()<<", "<<Eigen::Quaternionf(best.T.rotation()).w()<<"]\n";
  std::cout<<"FIRST_SEGMENT_INITIALIZER_PASS scans="<<scans.size()<<" used="<<used<<" candidates="<<cands.size()<<" best_converged="<<best.converged<<" best_fitness="<<best.fitness<<"\n";
  return best.converged ? 0 : 7;
}
