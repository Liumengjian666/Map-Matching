// Offline source-recovery probe; no NDT align, ROS node or filter updates.
// finalizeCloud/voxelDown/preprocess/stableCloudHash below are verbatim
// function bodies from the frozen production NDT node, not new algorithms.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include "dog_prior_map_localization/core/imu_deskew.hpp"
using namespace dog_prior_map_localization;
void finalizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}
struct NdtPreprocessing {
  double min_range_=.5,max_range_=80.,min_z_=-std::numeric_limits<double>::infinity(),max_z_=std::numeric_limits<double>::infinity();
  double scan_voxel_size_=.25,source_voxel_z_size_=.25;
  int max_scan_points_=1400;
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                double voxel_size,
                                                double voxel_z_size,
                                                int max_points) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
    if (voxel_size > 0.01)
    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(voxel_size, voxel_size, voxel_z_size);
      voxel.setInputCloud(cloud);
      voxel.filter(*down);
    }
    else
    {
      *down = *cloud;
    }

    if (max_points > 0 && static_cast<int>(down->size()) > max_points)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
      sampled->reserve(max_points);
      const double step = static_cast<double>(down->size() - 1) / static_cast<double>(std::max(max_points - 1, 1));
      for (int i = 0; i < max_points; ++i)
      {
        sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
      }
      finalizeCloud(sampled);
      return sampled;
    }
    finalizeCloud(down);
    return down;
  }
pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->size());
    for (const auto &pt : cloud->points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r < min_range_ || r > max_range_) continue;
      if (pt.z < min_z_ || pt.z > max_z_) continue;
      filtered->push_back(pt);
    }
    finalizeCloud(filtered);
    return voxelDown(filtered, scan_voxel_size_, source_voxel_z_size_, max_scan_points_);
  }
static uint64_t stableCloudHash(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {
    constexpr uint64_t kOffset = 1469598103934665603ULL;
    constexpr uint64_t kPrime = 1099511628211ULL;
    uint64_t hash = kOffset;
    const auto mixByte = [&hash](uint8_t byte) {
      hash ^= static_cast<uint64_t>(byte);
      hash *= kPrime;
    };
    const auto mixU32 = [&mixByte](uint32_t value) {
      for (int shift = 0; shift < 32; shift += 8)
      {
        mixByte(static_cast<uint8_t>((value >> shift) & 0xffU));
      }
    };
    if (!cloud) return hash;
    mixU32(cloud->width);
    mixU32(cloud->height);
    mixU32(cloud->is_dense ? 1U : 0U);
    mixU32(static_cast<uint32_t>(cloud->size()));
    for (const auto &point : cloud->points)
    {
      uint32_t bits = 0;
      std::memcpy(&bits, &point.x, sizeof(bits));
      mixU32(bits);
      std::memcpy(&bits, &point.y, sizeof(bits));
      mixU32(bits);
      std::memcpy(&bits, &point.z, sizeof(bits));
      mixU32(bits);
    }
    return hash;
  }
};
std::vector<double> parse(const std::string& line) {
  std::vector<double> row;std::stringstream s(line);std::string v;
  while(std::getline(s,v,','))row.push_back(std::stod(v));
  return row;
}
int main(int argc,char**argv) {
  if(argc!=6){std::cerr<<"raw_xyzt states.csv scan_start expected_hash output_prefix\n";return 2;}
  try {
    std::ifstream states(argv[2]);std::string line;std::getline(states,line);
    StateHistory history;
    while(std::getline(states,line)) {
      const auto v=parse(line);if(v.size()!=17)throw std::runtime_error("state layout");
      FilterStateSnapshot s;s.stamp=v[0];s.p=Eigen::Vector3d(v[1],v[2],v[3]);s.v=Eigen::Vector3d(v[4],v[5],v[6]);
      s.R=Eigen::Quaterniond(v[10],v[7],v[8],v[9]).normalized().toRotationMatrix();
      s.acc_measurement=Eigen::Vector3d(v[11],v[12],v[13]);s.gyro_measurement=Eigen::Vector3d(v[14],v[15],v[16]);
      history.insertMonotonic(s);
    }
    std::ifstream input(argv[1],std::ios::binary);std::vector<std::array<float,4>> points;std::array<float,4> pt;
    while(input.read(reinterpret_cast<char*>(pt.data()),sizeof(pt)))points.push_back(pt);
    if(points.empty())throw std::runtime_error("empty raw scan");
    const double start=std::stod(argv[3]);
    double offset=0.;for(const auto&p:points)offset=std::max(offset,static_cast<double>(p[3]));
    const double end=start+offset;
    Eigen::Isometry3d Til=Eigen::Isometry3d::Identity();Eigen::Matrix3d Ril;
    Ril<<.9999918597233475,-.0005161381080032746,.004001760674214531,
        .0005196241852023933,.9999994864189835,-.0008701450879252283,
        -.004001309503946919,.0008722174163330545,.9999916143443562;
    Til.linear()=Eigen::Quaterniond(Ril).normalized().toRotationMatrix();Til.translation()=Eigen::Vector3d(.08,.029,.03);
    const Eigen::Vector3d gravity(-.34546457,-.73858365,-9.77269321);
    ImuKinematicsConfig config;config.use_acc_for_position=true;config.midpoint_interval_input_enable=true;
    ImuDeskewCoverage coverage;std::string reason;
    if(!inspectImuDeskewCoverage(history,start,end,.02,coverage,reason))throw std::runtime_error(reason);
    ImuDeskewPose reference;
    if(!interpolateImuDeskewPose(history,start,.02,gravity,config,reference,reason,end))throw std::runtime_error(reason);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto&p:points) {
      ImuDeskewPose pose;Eigen::Vector3d xyz;
      if(!interpolateImuDeskewPose(history,start+static_cast<double>(p[3]),.02,gravity,config,pose,reason,end))throw std::runtime_error(reason);
      if(!transformPointToReferenceLidar(pose,reference,Til,Eigen::Vector3d(p[0],p[1],p[2]),xyz,reason))throw std::runtime_error(reason);
      cloud->push_back(pcl::PointXYZ(static_cast<float>(xyz.x()),static_cast<float>(xyz.y()),static_cast<float>(xyz.z())));
    }
    finalizeCloud(cloud);NdtPreprocessing preprocessing;const auto source=preprocessing.preprocess(cloud);
    const auto hash=NdtPreprocessing::stableCloudHash(source);
    std::ofstream xyz(std::string(argv[5])+".xyz",std::ios::binary);
    for(const auto&p:source->points){xyz.write(reinterpret_cast<const char*>(&p.x),sizeof(float));xyz.write(reinterpret_cast<const char*>(&p.y),sizeof(float));xyz.write(reinterpret_cast<const char*>(&p.z),sizeof(float));}
    std::ostringstream h;h<<std::hex<<std::setw(16)<<std::setfill('0')<<hash;
    std::cout<<source->size()<<","<<h.str()<<","<<(h.str()==argv[4]?"SOURCE_HASH_PASS":"SOURCE_RECONSTRUCTION_FAILED")<<"\n";
    return 0;
  } catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
