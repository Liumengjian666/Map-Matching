#include "dog_prior_map_localization/experimental_source_capture.hpp"
#include <fstream>
#include <iostream>
#include <cassert>
using namespace dog_prior_map_localization;
int main() {
  char directory[] = "/tmp/p23_capture_test.XXXXXX";
  if (!mkdtemp(directory)) return 2;
  ExperimentalSourceCapture capture;
  const uint64_t ns = 1517157219289839001ULL;
  assert(!capture.requested(ns));
  capture.configure(directory, {"2:" + std::to_string(ns)});
  assert(capture.requested(ns) && !capture.requested(ns+1));
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.125f,-2.25f,3.5f)); cloud.is_dense=true;
  capture.save(2, ns, cloud, 42);
  assert(cloud[0].x==1.125f && cloud[0].y==-2.25f && cloud[0].z==3.5f);
  std::ifstream binary(std::string(directory)+"/frame_2.xyz",std::ios::binary);
  float values[3];binary.read(reinterpret_cast<char*>(values),sizeof(values));
  assert(binary.gcount()==12 && values[0]==1.125f && values[1]==-2.25f && values[2]==3.5f);
  bool rejected=false;
  try { capture.save(2,ns,cloud,42); } catch(const std::exception&) { rejected=true; }
  assert(rejected);
  capture.savePoses(2,std::vector<Eigen::Matrix4d>(5,Eigen::Matrix4d::Identity()));
  rejected=false;
  try { capture.savePoses(2,std::vector<Eigen::Matrix4d>(5,Eigen::Matrix4d::Identity())); }
  catch(const std::exception&) { rejected=true; }
  assert(rejected);
  ExperimentalSourceCapture conflict;
  conflict.configure(directory,{"2:"+std::to_string(ns)});
  rejected=false;try { conflict.save(2,ns,cloud,42); } catch(const std::exception&) { rejected=true; }
  assert(rejected);
  ExperimentalSourceCapture invalid;
  rejected=false;try { invalid.configure(directory,{"2:1.5"}); } catch(const std::exception&) { rejected=true; }
  assert(rejected);
  std::cout << "CAPTURE_CONTRACT_PASS directory=" << directory << '\n';
}
