#include <pcl/registration/ndt.h>
#include <pcl/point_types.h>
#include <iostream>
class Inspect : public pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> {
 public: Eigen::Vector3f leaf()const{return target_cells_.getLeafSize();}
};
int main(){
 auto cloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
 for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z)cloud->push_back(pcl::PointXYZ(.03f*x,.04f*y,.05f*z));
 Inspect old;
 // Exact first-target order in non-local loadMap() with geode disabled.
 old.setInputTarget(cloud);old.setResolution(.8f);
 std::cout<<"configured_resolution="<<old.getResolution()<<" actual_leaf="<<old.leaf().transpose()<<'\n';
 if((old.leaf()-Eigen::Vector3f::Constant(.8f)).norm()!=0)return 1;
 return 0;
}
