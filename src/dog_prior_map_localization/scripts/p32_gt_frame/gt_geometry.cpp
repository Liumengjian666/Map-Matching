// Fixed hypothesis NN diagnostics only; no registration/ICP/NDT dependency.
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <chrono>
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
Eigen::Matrix4d readMatrix(std::istream& stream) {
  Eigen::Matrix4d T;for(int i=0;i<4;++i)for(int j=0;j<4;++j)
    if(!stream.read(reinterpret_cast<char*>(&T(i,j)),8))throw std::runtime_error("MATRIX_BYTES");
  if(!T.allFinite()||(T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()!=0||std::abs(T.block<3,3>(0,0).determinant()-1)>1e-5
    ||(T.block<3,3>(0,0).transpose()*T.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>1e-5)throw std::runtime_error("ILLEGAL_SE3");
  return T;
}
int main(int argc,char** argv) {
  if(argc!=4)return 2;
  try {
    const std::string root=argv[1],output=argv[3];
    Cloud::Ptr map(new Cloud),target(new Cloud);if(pcl::io::loadPCDFile(argv[2],*map)<0)return 3;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(map);voxel.setLeafSize(.20f,.20f,.20f);voxel.filter(*target);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
    std::ifstream ext(root+"/T_I_L.f64",std::ios::binary),yaml(root+"/T_yaml.f64",std::ios::binary),poses(root+"/poses.f64",std::ios::binary);
    const Eigen::Matrix4d T_I_L=readMatrix(ext),T_yaml=readMatrix(yaml);
    const Eigen::Matrix4d C[3]={Eigen::Matrix4d::Identity(),T_yaml,T_yaml.inverse()};
    std::ifstream frames(root+"/frames.txt");int frame_index;uint64_t scan_ns;size_t count;std::string name;int scans=0,queries=0;
    std::ofstream out(output+"/geometry.csv");out<<std::setprecision(17)<<"frame_index,scan_ns,H,C,source_points,nn_median_m,nn_p90_m,coverage_0_5m,coverage_2m,coverage_8m,bbox_xmin,bbox_xmax,bbox_ymin,bbox_ymax,bbox_zmin,bbox_zmax,nn_ms";
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<",T"<<i<<j;out<<'\n';
    while(frames>>frame_index>>scan_ns>>count>>name) {
      if(++scans>5)throw std::runtime_error("FRAME_BUDGET");const Eigen::Matrix4d GT=readMatrix(poses);
      std::ifstream points(root+"/"+name,std::ios::binary|std::ios::ate);
      if(!points||points.tellg()!=static_cast<long long>(count*12)||!count)throw std::runtime_error("POINT_FILE");points.seekg(0);
      Cloud::Ptr source(new Cloud);source->resize(count);float xyz[3];
      for(auto& p:*source){points.read(reinterpret_cast<char*>(xyz),12);p.x=xyz[0];p.y=xyz[1];p.z=xyz[2];if(!pcl::isFinite(p))throw std::runtime_error("NONFINITE_RAW");}
      for(int h=1;h<=2;++h)for(int c=0;c<3;++c) {
        if(++queries>30)throw std::runtime_error("NN_BUDGET");
        const Eigen::Matrix4d T=C[c]*GT*(h==1?T_I_L:Eigen::Matrix4d::Identity());
        Cloud moved;pcl::transformPointCloud(*source,moved,T.cast<float>());
        const auto tick=std::chrono::steady_clock::now();std::vector<float> distances;std::vector<int> ids(1);std::vector<float> d2(1);
        Eigen::Vector3f low=Eigen::Vector3f::Constant(INFINITY),high=Eigen::Vector3f::Constant(-INFINITY);size_t near_half=0,near_two=0,near_eight=0;
        for(const auto& p:moved) {
          if(!pcl::isFinite(p)||tree.nearestKSearch(p,1,ids,d2)!=1)throw std::runtime_error("NN_INVALID");
          const Eigen::Vector3f v(p.x,p.y,p.z);low=low.cwiseMin(v);high=high.cwiseMax(v);
          const float d=std::sqrt(d2[0]);distances.push_back(d);near_half+=d<=.5f;near_two+=d<=2.f;near_eight+=d<=8.f;
        }
        std::sort(distances.begin(),distances.end());const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
        out<<frame_index<<','<<scan_ns<<','<<h<<','<<c<<','<<count<<','<<distances[count/2]<<','<<distances[size_t(.9*(count-1))]
          <<','<<double(near_half)/count<<','<<double(near_two)/count<<','<<double(near_eight)/count;
        for(int axis=0;axis<3;++axis)out<<','<<low(axis)<<','<<high(axis);out<<','<<ms;
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<T(i,j);out<<'\n';
      }
    }
    if(scans!=5||queries!=30)throw std::runtime_error("INCOMPLETE_FIXED_TEST");
    std::cout<<"GT_DIAGNOSTIC_COMPLETE scans="<<scans<<" frame_hypothesis_NN="<<queries<<" ICP=0 NDT=0 target_points="<<target->size()<<'\n';return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 5;}
}
