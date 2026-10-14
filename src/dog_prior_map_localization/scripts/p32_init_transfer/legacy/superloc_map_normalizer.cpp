#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  if (argc < 4) { std::cerr<<"usage: superloc_map_normalizer SOURCE_PCD INIT_YAML OUTPUT_PCD\n"; return 2; }
  pcl::PointCloud<pcl::PointXYZ> src; if (pcl::io::loadPCDFile(argv[1],src)<0) return 3;
  std::ifstream f(argv[2]); std::string line, all; while (std::getline(f,line)) all += line+"\n";
  const auto pos=all.find("translation: ["); if(pos==std::string::npos) return 4; const auto b=pos+14; const auto e=all.find(']',b); std::stringstream ss(all.substr(b,e-b)); float x,y,z; char c; if(!(ss>>x>>c>>y>>c>>z)) return 5;
  // The initializer writes a rotation matrix as [r00, ...; r10, ...; r20, ...].
  const auto rp=all.find("rotation_matrix: ["); if(rp==std::string::npos) return 6; const auto rb=rp+18; const auto re=all.find(']',rb); std::string rs=all.substr(rb,re-rb); for(char &q:rs) if(q==';') q=','; std::stringstream rr(rs); Eigen::Matrix3f R; for(int i=0;i<9;++i){ if(!(rr>>R(i/3,i%3))) return 7; rr>>c; }
  Eigen::Affine3f T=Eigen::Affine3f::Identity(); T.linear()=R; T.translation()<<x,y,z; Eigen::Affine3f inv=T.inverse(); pcl::PointCloud<pcl::PointXYZ> out; pcl::transformPointCloud(src,out,inv); if(pcl::io::savePCDFileBinary(argv[3],out)<0)return 8;
  std::cout<<"MAP_NORMALIZATION_PASS points="<<out.size()<<" translation_norm="<<T.translation().norm()<<" rotation_angle_deg="<<Eigen::AngleAxisf(R).angle()*180.0/M_PI<<"\n"; return 0;
}
