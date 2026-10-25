// R29 data adapter: original map -> frozen P45 two-pass 0.15m XYZ voxels / 20m tiles.
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <map>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
int main(int argc,char**argv){
 if(argc!=3)return 2;
 auto cloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
 if(pcl::io::loadPLYFile(argv[1],*cloud)!=0)return 3;
 for(int i=0;i<2;++i){auto next=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> v;v.setInputCloud(cloud);v.setLeafSize(.15f,.15f,.15f);v.filter(*next);cloud=next;}
 std::map<std::pair<int,int>,pcl::PointCloud<pcl::PointXYZ>> tiles;
 for(auto&p:*cloud)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))tiles[{int(std::floor(p.x/20)),int(std::floor(p.y/20))}].push_back(p);
 std::ofstream out(std::string(argv[2])+"/tile_table.csv");
 out<<"tile_id,tile_x,tile_y,path,point_count,x_min,x_max,y_min,y_max,z_min,z_max\n"<<std::setprecision(17);
 for(auto& item:tiles){auto&c=item.second;auto xy=item.first;std::string id="tile_"+std::to_string(xy.first)+"_"+std::to_string(xy.second);
  std::string path=std::string(argv[2])+"/"+id+".pcd";if(pcl::io::savePCDFileBinary(path,c))return 4;
  double lo[3]={INFINITY,INFINITY,INFINITY},hi[3]={-INFINITY,-INFINITY,-INFINITY};
  for(auto&p:c)for(int j=0;j<3;++j){lo[j]=std::min(lo[j],double(p.data[j]));hi[j]=std::max(hi[j],double(p.data[j]));}
  out<<id<<','<<xy.first<<','<<xy.second<<','<<path<<','<<c.size();for(int j=0;j<3;++j)out<<','<<lo[j]<<','<<hi[j];out<<'\n';
 }
}
