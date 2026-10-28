// R7 reuses the R6 preparation and native scoring path; only seeds/budget change.
// The runtime input contains no reference poses or R6 prediction results.
#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/property_tree/json_parser.hpp>
#include <openssl/sha.h>
#include <omp.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pclomp::NormalDistributionsTransform<Point, Point>;
using Tree = boost::property_tree::ptree;
using Clock = std::chrono::steady_clock;
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
std::string xyz_sha(const Cloud& cloud) {
  std::vector<float> xyz; xyz.reserve(cloud.size() * 3);
  for (const auto& p : cloud) { xyz.push_back(p.x); xyz.push_back(p.y); xyz.push_back(p.z); }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(xyz.data()), xyz.size()*sizeof(float), digest);
  std::ostringstream out;
  for (auto b : digest) out << std::hex << std::setw(2) << std::setfill('0') << int(b);
  return out.str();
}
Cloud::Ptr voxel(const Cloud::ConstPtr& input) {
  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(0.1f, 0.1f, 0.1f); grid.setInputCloud(input);
  Cloud::Ptr out(new Cloud); grid.filter(*out); return out;
}
struct Frame {
  int raw; uint64_t stamp; Cloud::Ptr source;
  std::string original_xyz_sha; double prepare_ms;
};
Frame prepare(const Tree& config) {
  const auto begin = Clock::now();
  pcl::PCLPointCloud2 blob;
  require(pcl::io::loadPCDFile(config.get<std::string>("path"), blob) == 0, "PCD load failed");
  const size_t n = blob.width * blob.height;
  require(n == config.get<size_t>("points"), "Raw point count differs from R5");
  std::vector<uint32_t> offsets;
  for (const auto& name : {"x", "y", "z"}) {
    auto f = std::find_if(blob.fields.begin(), blob.fields.end(), [&](const auto& f){return f.name == name;});
    require(f != blob.fields.end() && f->datatype == pcl::PCLPointField::FLOAT32 && f->count == 1, "XYZ format mismatch");
    offsets.push_back(f->offset);
  }
  sensor_msgs::PointCloud2 message;
  const auto ns = config.get<uint64_t>("stamp_ns");
  message.header.stamp.sec = ns / 1000000000ULL; message.header.stamp.nsec = ns % 1000000000ULL;
  message.header.frame_id = config.get<std::string>("frame_id");
  require(message.header.frame_id == "depth_camera_link", "Frame mismatch");
  message.width = n; message.height = 1; message.point_step = 16; message.row_step = n * 16;
  message.is_bigendian = false; message.is_dense = true; message.data.resize(n * 16, 0);
  for (size_t k = 0; k < 4; ++k) {
    sensor_msgs::PointField f; f.name = std::vector<std::string>{"x", "y", "z", "intensity"}[k];
    f.offset = k * 4; f.count = 1; f.datatype = sensor_msgs::PointField::FLOAT32; message.fields.push_back(f);
  }
  Cloud::Ptr direct(new Cloud); direct->resize(n); direct->is_dense = true;
  for (size_t i = 0; i < n; ++i) {
    const auto* from = blob.data.data() + (i / blob.width) * blob.row_step + (i % blob.width) * blob.point_step;
    for (int k = 0; k < 3; ++k) std::memcpy(message.data.data() + i*16 + k*4, from + offsets[k], 4);
    float xyz[3]; std::memcpy(xyz, message.data.data() + i*16, 12);
    require(std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]), "Unexpected nonfinite raw point");
    auto& p = direct->points[i]; p.x=xyz[0]; p.y=xyz[1]; p.z=xyz[2]; p.intensity=0;
  }
  Cloud::Ptr converted(new Cloud); pcl::fromROSMsg(message, *converted);
  // pcl_ros::transformPointCloud(target == input.frame_id) returns cloud_out=cloud_in.
  Cloud::Ptr same_frame(new Cloud(*converted));
  const auto raw_sha = xyz_sha(*same_frame);
  require(raw_sha == config.get<std::string>("xyz_sha256"), "R5 input XYZ SHA differs");
  auto source = voxel(same_frame); auto independent = voxel(direct);
  require(xyz_sha(*source)==xyz_sha(*independent), "Independent serialization paths differ");
  return {config.get<int>("raw_ordinal"), ns, source, raw_sha, std::chrono::duration<double,std::milli>(Clock::now()-begin).count()};
}
int main(int argc,char** argv) {
 try {
  require(argc==3,"usage: kernel_preflight INPUT_FREEZE.json OUTPUT_DIRECTORY");
  Tree config;boost::property_tree::read_json(argv[1],config);const std::string directory=argv[2];
  Cloud::Ptr raw(new Cloud);require(pcl::io::loadPCDFile(config.get<std::string>("map_path"),*raw)==0,"Map load failed");
  require(raw->size()==config.get<size_t>("map_points"),"Map input count differs");
  auto target=voxel(raw);std::ofstream map(directory+"/TARGET.json");
  map<<"{\"points\":"<<target->size()<<",\"xyz_sha256\":\""<<xyz_sha(*target)<<"\"}\n";map.close();
  std::ofstream out(directory+"/SOURCES.csv");
  out<<"raw_ordinal,stamp_ns,source_points,source_xyz_sha256\n";
  size_t n=0;
  for(const auto& entry:config.get_child("frames")) {
    auto f=prepare(entry.second);require(f.raw==5+int(n),"Frozen raw order differs");
    out<<f.raw<<','<<f.stamp<<','<<f.source->size()<<','<<xyz_sha(*f.source)<<'\n';++n;
  }
  require(n==2375,"Exactly 2375 sources required");std::cout<<"PASS 2375 native preprocessing identities; target="<<target->size()<<" align=0\n";return 0;
 }catch(const std::exception& e){std::cerr<<"DATA_OR_FRAME_CONTRACT_BLOCKED "<<e.what()<<'\n';return 2;}
}
