// Offline binding of raw RC1 sensors to the P2B rotation-only bootstrap input.
#include "gyro_input.hpp"
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/SVD>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
int main(int argc,char** argv) {
  if (argc!=2) return 2;
  try {
    const std::string root=argv[1];
    int fd=::open((root+"/bridge.started").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    if (fd<0) throw std::runtime_error("BRIDGE_ALREADY_ATTEMPTED");::close(fd);
    const uint64_t reference=1645999726479818106ULL,cutoff=reference+5000000000ULL;
    std::ifstream input_imu(root+"/imu.txt");std::vector<p32::Imu> imu;
    p32::Imu sample;double ax,ay,az;
    while(input_imu>>sample.ns>>sample.gyro.x()>>sample.gyro.y()>>sample.gyro.z()>>ax>>ay>>az) {
      if(sample.ns>cutoff) throw std::runtime_error("POST_INIT_CUTOFF_IMU");
      imu.push_back(sample);
    }
    if(imu.size()<2) throw std::runtime_error("IMU_MISSING");
    p32::integrate(imu);
    Eigen::Matrix4d T;std::ifstream ext(root+"/T_imu_lidar.f64",std::ios::binary);
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)
      if(!ext.read(reinterpret_cast<char*>(&T(i,j)),8)) throw std::runtime_error("CALIBRATION_READ");
    if(!T.allFinite()||(T.block<3,3>(0,0).transpose()*T.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>1e-5)
      throw std::runtime_error("INVALID_CALIBRATION");
    // Existing adapter's decimal-rounding handling, never alters the raw file.
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(T.block<3,3>(0,0),Eigen::ComputeFullU|Eigen::ComputeFullV);
    const Eigen::Matrix3d R=svd.matrixU()*svd.matrixV().transpose();
    const Eigen::Vector3d t=T.block<3,1>(0,3);
    rosbag::Bag bag;bag.open(root+"/init_rotation_input.bag",rosbag::bagmode::Write);
    std::ifstream scans(root+"/scans.txt");uint64_t start,end;size_t count;std::string name;int index=0;
    std::ofstream ledger(root+"/bridge.csv");
    ledger<<"scan_ns,end_ns,points,max_point_change_m,translation_model,max_imu_available_ns\n"<<std::setprecision(17);
    while(scans>>start>>end>>count>>name) {
      if(start<reference||end>cutoff||(!index&&start!=reference)) throw std::runtime_error("SCAN_WINDOW");
      std::ifstream points(root+"/"+name,std::ios::binary|std::ios::ate);
      if(!points||points.tellg()!=static_cast<long long>(count*16)) throw std::runtime_error("POINT_BYTE_COUNT");
      points.seekg(0);
      const auto q0=p32::orientation(imu,start);
      sensor_msgs::PointCloud2 cloud;cloud.header.seq=++index;
      cloud.header.stamp=ros::Time(start/1000000000ULL,start%1000000000ULL);
      cloud.header.frame_id="cmu_rc1_velodyne";cloud.height=1;cloud.width=count;cloud.is_dense=true;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::PointField::FLOAT32,"y",1,sensor_msgs::PointField::FLOAT32,
        "z",1,sensor_msgs::PointField::FLOAT32);modifier.resize(count);
      float xyzt[4];double change=0;
      for(size_t k=0;k<count;++k) {
        if(!points.read(reinterpret_cast<char*>(xyzt),16)) throw std::runtime_error("POINT_TRUNCATED");
        const Eigen::Vector3d p(xyzt[0],xyzt[1],xyzt[2]);
        if(!p.allFinite()||!std::isfinite(xyzt[3])||xyzt[3]<0||xyzt[3]>.15) throw std::runtime_error("INVALID_POINT");
        const uint64_t ns=start+static_cast<uint64_t>(std::llround(double(xyzt[3])*1e9));
        if(ns>end||ns>cutoff) throw std::runtime_error("POINT_AFTER_SCAN_OR_CUTOFF");
        const auto qi=p32::orientation(imu,ns);
        const Eigen::Matrix3d relative=q0.toRotationMatrix().transpose()*qi.toRotationMatrix();
        const Eigen::Vector3d corrected=p32::rotatePoint(R,t,relative,p);
        if(!corrected.allFinite()||corrected.norm()>1000) throw std::runtime_error("NONFINITE_OR_EXPLODED_POINT");
        const float xyz[3]={float(corrected.x()),float(corrected.y()),float(corrected.z())};
        std::memcpy(cloud.data.data()+k*12,xyz,12);change=std::max(change,(corrected-p).norm());
      }
      bag.write("/superloc_adapter/points_deskewed",cloud.header.stamp,cloud);
      ledger<<start<<','<<end<<','<<count<<','<<change<<",P2B_ROTATION_ONLY_NO_TRANSLATION_CV,"<<imu.back().ns<<'\n';
    }
    bag.close();std::cout<<"BOOTSTRAP_BRIDGE_PASS scans="<<index<<" reference_ns="<<reference<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
