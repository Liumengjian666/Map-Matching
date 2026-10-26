#include "dog_prior_map_localization/ndt_target_configuration.hpp"
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <iostream>

class InspectNdt : public pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>
{
public:
  Eigen::Vector3f leaf() const { return target_cells_.getLeafSize(); }
};

int main()
{
  auto target = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int x = 0; x < 8; ++x)
    for (int y = 0; y < 8; ++y)
      for (int z = 0; z < 8; ++z)
        target->push_back(pcl::PointXYZ(.03f * x, .04f * y, .05f * z));

  // Verify the actual PCL failure without running align() or reading a GT pose.
  InspectNdt legacy;
  legacy.setInputTarget(target);
  legacy.setResolution(.8f);
  if (legacy.getResolution() != .8f || legacy.leaf().x() != 1.f)
  {
    std::cerr << "PCL premise changed; reassess target construction order\n";
    return 1;
  }

  InspectNdt fixed;
  dog_prior_map_localization::setNdtTargetAtResolution(fixed, target, .8f);
  const auto valid = [&]() {
    return fixed.getResolution() == .8f &&
        (fixed.leaf() - Eigen::Vector3f::Constant(.8f)).norm() == 0.f;
  };
  if (!valid()) return 2;  // First global PCD with no Source installed.
  fixed.setInputSource(target);
  dog_prior_map_localization::setNdtTargetAtResolution(fixed, target, .8f);
  if (!valid()) return 3;  // Later target replacement.
  InspectNdt tiled;
  tiled.setResolution(.8f);
  dog_prior_map_localization::setNdtTargetAtResolution(tiled, target, .8f);
  if ((tiled.leaf() - fixed.leaf()).norm() != 0.f) return 4;

  std::cout << "NDT_TARGET_RESOLUTION_CONTRACT_PASS legacy_leaf="
            << legacy.leaf().transpose() << " fixed_leaf="
            << fixed.leaf().transpose() << " native_align_calls=0\n";
  return 0;
}
