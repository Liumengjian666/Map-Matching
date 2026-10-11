// Compile against the original upstream checkout, not the adapted library.
// Boundary/profile evidence only; no alternative descriptor is implemented.
#include <iostream>
#include <iomanip>
#include <solid_module.h>

int main(int argc, char **argv)
{
  if (argc < 2) return 2;
  SOLiDModule module;
  std::cout << "path,raw_points,range_points,down_points,finite,range_norm,top_height_fraction\n";
  for (int i = 1; i < argc; ++i) {
    pcl::PointCloud<PointType> raw;
    if (pcl::io::loadPCDFile<PointType>(argv[i], raw) != 0) return 3;
    pcl::PointCloud<PointType>::Ptr far(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr range(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr down(new pcl::PointCloud<PointType>());
    module.remove_far_points(raw, far);
    module.remove_closest_points(*far, range);
    module.down_sampling(*range, down);
    const auto descriptor = module.makeSolid(*down);
    std::size_t top = 0;
    for (auto point : down->points) {
      auto bin = module.pt2rah(point, 360.0f/module.NUM_ANGLE,
          static_cast<float>(module.MAX_DISTANCE)/module.NUM_RANGE,
          (module.FOV_u-module.FOV_d)/module.NUM_HEIGHT);
      if (bin.idx_height == module.NUM_HEIGHT-1) ++top;
    }
    std::cout << argv[i] << ',' << raw.size() << ',' << range->size() << ','
              << down->size() << ',' << descriptor.allFinite() << ','
              << std::setprecision(17) << descriptor.head(module.NUM_RANGE).norm() << ','
              << (down->empty() ? 0.0 : static_cast<double>(top)/down->size()) << '\n';
  }
  pcl::PointCloud<PointType> empty;
  const auto empty_descriptor = module.makeSolid(empty);
  std::cout << "EMPTY_INPUT,0,0,0," << empty_descriptor.allFinite() << ','
            << empty_descriptor.head(module.NUM_RANGE).norm() << ",0\n";
  return 0;
}
