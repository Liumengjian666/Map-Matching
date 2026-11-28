// Paired original NDT configuration and research scorer at the same initial pose.
#include "scoring_profile.hpp"
#include "input_receipt.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <omp.h>

namespace {
q1::Cloud::Ptr load(const char* file) {
  q1::Cloud::Ptr result(new q1::Cloud());
  if (pcl::io::loadPCDFile(file,*result)<0 || result->empty())
    throw std::runtime_error("cannot load point cloud");
  return result;
}
using Native = pclomp::NormalDistributionsTransform<q1::Point, q1::Point>;
class OriginalProbe : public Native {
 public:
  explicit OriginalProbe(const q1::Cloud::ConstPtr& target) {
    setResolution(1.);
    setTransformationEpsilon(.01);
    setNeighborhoodSearchMethod(pclomp::DIRECT7);
    setInputTarget(target);
  }
  std::size_t support(const q1::Cloud& cloud) {
    std::vector<TargetGridLeafConstPtr> hits;
    std::size_t connected=0;
    for (const auto& p : cloud)
      connected += target_cells_.getNeighborhoodAtPoint7(p,hits) != 0;
    return connected;
  }
};

void metrics(const char* name, Native& registration, const q1::Cloud::ConstPtr& source,
             const Eigen::Matrix4f& seed, OriginalProbe& oracle) {
  registration.setInputSource(source);
  q1::Cloud out;
  registration.align(out,seed);
  const auto pose=registration.getFinalTransformation();
  std::cout << name << ',' << registration.getFinalNumIteration() << ',' << registration.hasConverged()
            << ',' << registration.getTransformationProbability() << ',' << oracle.support(out);
  for (int i=0;i<4;++i)
    for (int j=0;j<4;++j) std::cout << ',' << pose(i,j);
  std::cout << '\n';
}
}

int main(int argc, char** argv) {
  if (argc!=4 || omp_get_max_threads()!=1) return 2;
  auto map=load(argv[1]);
  auto source=q1::downsample(load(argv[2]));
  Eigen::Vector3f t;
  float yaw,qx,qy,qz,qw;
  std::ifstream anchor(argv[3]);
  if (!(anchor>>t.x()>>t.y()>>t.z()>>yaw>>qx>>qy>>qz>>qw)) return 2;
  Eigen::Matrix4f seed=Eigen::Matrix4f::Identity();
  seed.block<3,3>(0,0)=Eigen::Quaternionf(qw,qx,qy,qz).normalized().toRotationMatrix();
  seed.block<3,1>(0,3)=t;
  OriginalProbe native(map);
  q1::Scorer research(map);
  std::cout << std::setprecision(17);
  std::cout << "arm,iterations,converged,transformation_probability,associated";
  for (int i=0;i<4;++i)
    for (int j=0;j<4;++j) std::cout << ",t" << i << j;
  std::cout << '\n';
  metrics("A_ORIGINAL_CONFIG",native,source,seed,native);
  metrics("B_RESEARCH",research,source,seed,native);
}
