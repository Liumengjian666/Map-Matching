#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using Cloud=pcl::PointCloud<pcl::PointXYZ>; using Transform=Eigen::Affine3f;
Cloud::Ptr voxel(const Cloud::ConstPtr& c,float l){Cloud::Ptr o(new Cloud);pcl::VoxelGrid<pcl::PointXYZ>v;v.setInputCloud(c);v.setLeafSize(l,l,l);v.filter(*o);return o;}
bool parse(const std::string&p,Transform&T){std::ifstream f(p);std::string s,l;while(std::getline(f,l))s+=l+"\n";auto tp=s.find("translation: [");auto te=s.find(']',tp);auto rp=s.find("rotation_matrix: [");auto re=s.find(']',rp);if(tp==std::string::npos||rp==std::string::npos)return false;float x,y,z;char c;std::stringstream ts(s.substr(tp+14,te-tp-14));if(!(ts>>x>>c>>y>>c>>z))return false;std::string rs=s.substr(rp+18,re-rp-18);for(char&q:rs)if(q==';')q=',';std::stringstream rr(rs);Eigen::Matrix3f R;for(int i=0;i<9;++i){if(!(rr>>R(i/3,i%3))){return false;}rr>>c;}T=Transform::Identity();T.linear()=R;T.translation()<<x,y,z;return true;}
int main(int argc,char**argv){ros::init(argc,argv,"superloc_holdout_sanity",ros::init_options::AnonymousName);if(argc<5){std::cerr<<"usage: superloc_holdout_sanity ADAPTER_BAG NORMALIZED_MAP INIT_YAML OUT_DIR [init_sec] [end_sec]\n";return 2;}double init_s=argc>5?std::stod(argv[5]):5.,end_s=argc>6?std::stod(argv[6]):10.;Cloud::Ptr map(new Cloud);if(pcl::io::loadPCDFile(argv[2],*map)<0)return 3;map=voxel(map,0.2f);Transform Tmap0;if(!parse(argv[3],Tmap0))return 4;rosbag::Bag bag;bag.open(argv[1],rosbag::bagmode::Read);const std::vector<std::string>topics{"/superloc_adapter/points_deskewed"};rosbag::View view(bag,rosbag::TopicQuery(topics));std::vector<Cloud::Ptr>ss;std::vector<double>st;double first=-1;for(const auto&m:view){auto msg=m.instantiate<sensor_msgs::PointCloud2>();if(!msg)continue;double t=msg->header.stamp.toSec();if(first<0)first=t;if(t>first+end_s)break;Cloud::Ptr c(new Cloud);pcl::fromROSMsg(*msg,*c);if(!c->empty()){ss.push_back(voxel(c,0.2f));st.push_back(t);}}bag.close();if(ss.size()<3)return 5;pcl::KdTreeFLANN<pcl::PointXYZ>tree;tree.setInputCloud(map);std::ofstream out(std::string(argv[4])+"/holdout_sanity.csv");out<<"index,stamp,phase,scan_to_scan_converged,scan_to_scan_fitness,pose_translation_norm_m,map_nn_median,map_nn_p90,map_inlier_ratio\n";Transform T=Transform::Identity();Cloud::Ptr prev=ss.front();for(std::size_t i=1;i<ss.size();++i){pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>icp;icp.setInputSource(ss[i]);icp.setInputTarget(prev);icp.setMaxCorrespondenceDistance(2.5);icp.setMaximumIterations(40);Cloud a;icp.align(a,Eigen::Matrix4f::Identity());Transform d(icp.getFinalTransformation());bool ok=icp.hasConverged()&&d.translation().norm()<3.0&&std::isfinite(icp.getFitnessScore());if(ok){T=T*d;prev=ss[i];}if(st[i]>=first+init_s&&st[i]<=first+end_s){Cloud::Ptr tr(new Cloud);pcl::transformPointCloud(*ss[i],*tr,Tmap0*T);std::vector<float>ds;ds.reserve(tr->size());int in=0;std::vector<int>ix(1);std::vector<float>di(1);for(const auto&p:*tr)if(pcl::isFinite(p)&&tree.nearestKSearch(p,1,ix,di)>0){float d0=std::sqrt(di[0]);ds.push_back(d0);if(d0<1.)++in;}std::sort(ds.begin(),ds.end());double med=ds.empty()?0.:ds[ds.size()/2],p90=ds.empty()?0.:ds[std::min(ds.size()-1,(std::size_t)(.9*(ds.size()-1)))];out<<i<<','<<std::setprecision(15)<<st[i]<<",holdout,"<<(icp.hasConverged()?1:0)<<','<<icp.getFitnessScore()<<','<<T.translation().norm()<<','<<med<<','<<p90<<','<<(ds.empty()?0.:double(in)/ds.size())<<'\n';}}std::cout<<"HOLDOUT_SANITY_PASS scans="<<ss.size()<<" holdout_interval="<<init_s<<".."<<end_s<<"\n";return 0;}
