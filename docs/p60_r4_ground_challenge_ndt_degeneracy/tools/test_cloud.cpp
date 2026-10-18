#include "cloud_contract.hpp"
#include <iostream>
#include <stdexcept>
void check(bool ok) { if (!ok) throw std::runtime_error("cloud contract test failed"); }
int main() {
  screen::Cloud::Ptr cloud(new screen::Cloud);
  for (auto p : {pcl::PointXYZ(1,0,0), pcl::PointXYZ(1,1,0), pcl::PointXYZ(1,-1,0),
                 pcl::PointXYZ(-1,0,0), pcl::PointXYZ(0,1,0)}) cloud->push_back(p);
  screen::finish(cloud);
  check(screen::crop(cloud)->size() == 3);
  auto a=screen::matched(cloud,3,2), b=screen::matched(cloud,3,2);
  check(a->size()==3 && screen::hash(a)==screen::hash(b));
  check(screen::hash(screen::matched(cloud,5,2))==screen::hash(cloud));
  bool rejected=false;
  try { screen::matched(cloud,6,2); } catch (const std::runtime_error&) { rejected=true; }
  check(rejected);
  cloud->push_back(pcl::PointXYZ(NAN,0,0));
  cloud->push_back(pcl::PointXYZ(.1,0,0));
  check(screen::finite(cloud,true)->size()==5);
  screen::Cloud::Ptr cells(new screen::Cloud);
  for (int x=0;x<3;++x) for(int y=0;y<3;++y) for(int z=0;z<3;++z)
    cells->push_back(pcl::PointXYZ(.1*x,.1*y,.1*z));
  screen::finish(cells);
  screen::Ndt ndt; ndt.configureTarget(cells);
  check(std::abs(ndt.getResolution()-.8)<1e-6);
  check(ndt.targetGridLeafSizeMeters()[0]==1.0f);
  std::cout << "cloud crop/boundary/deterministic count/order/range/finite guards PASS\n";
}
