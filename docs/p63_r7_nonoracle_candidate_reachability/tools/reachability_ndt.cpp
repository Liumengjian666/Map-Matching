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
  Eigen::Matrix4f prediction; std::string original_xyz_sha; double prepare_ms;
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
  require(source->size() == config.get<size_t>("R5_source_points"), "R5 native source count differs");
  require(xyz_sha(*source) == xyz_sha(*independent), "Direct-PCD versus PointCloud2 preprocessing differs");
  require(xyz_sha(*source) == config.get<std::string>("R6_source_xyz_sha256"), "R6 source geometry differs");
  Eigen::Matrix4f prediction; int k=0;
  for (const auto& value : config.get_child("pred_initial_row_major")) {
    require(k < 16, "Oversized seed"); prediction(k/4,k%4)=value.second.get_value<float>(); ++k;
  }
  require(k==16 && prediction.allFinite(), "Invalid seed");
  require((prediction.row(3)-Eigen::RowVector4f(0,0,0,1)).norm()<1e-6, "Nonhomogeneous seed");
  require((prediction.block<3,3>(0,0).transpose()*prediction.block<3,3>(0,0)-Eigen::Matrix3f::Identity()).norm()<1e-5, "Invalid seed rotation");
  return {config.get<int>("raw_ordinal"),ns,source,prediction,raw_sha,
          std::chrono::duration<double,std::milli>(Clock::now()-begin).count()};
}
int main(int argc, char** argv) {
  size_t calls=0;
  try {
    require(argc==4,"usage: reachability_ndt --prepare-only|--run INPUT.json OUTPUT_DIRECTORY");
    const std::string mode=argv[1],directory=argv[3];
    require(mode=="--prepare-only" || mode=="--run","Invalid mode");
    Tree config;boost::property_tree::read_json(argv[2],config);
    Cloud::Ptr raw_map(new Cloud);
    require(pcl::io::loadPCDFile(config.get<std::string>("map_path"),*raw_map)==0 && raw_map->size()==93146,"R5 map mismatch");
    auto target=voxel(raw_map);const auto target_sha=xyz_sha(*target);
    require(target->size()==89656 && target_sha==config.get<std::string>("target_xyz_sha256"),"R5/R6 target mismatch");
    std::vector<Frame> frames;std::vector<std::vector<Eigen::Matrix4f>> seeds;
    for(const auto& entry:config.get_child("frames")) {
      frames.push_back(prepare(entry.second));std::vector<Eigen::Matrix4f> group;int id=1;
      for(const auto& item:entry.second.get_child("seeds")) {
        require(item.second.get<int>("candidate_id")==id,"Seed order differs");
        Eigen::Matrix4f seed;int k=0;
        for(const auto& v:item.second.get_child("matrix_row_major")) {require(k<16,"Oversized seed");seed(k/4,k%4)=v.second.get_value<float>();++k;}
        require(k==16 && seed.allFinite(),"Invalid seed");
        const double angle=(id-1)*45.0*std::acos(-1.0)/180.0;
        Eigen::Matrix4f expected=frames.back().prediction;
        expected(0,3)=double(expected(0,3))+0.75*std::cos(angle);
        expected(1,3)=double(expected(1,3))+0.75*std::sin(angle);
        require((seed-expected).cwiseAbs().maxCoeff()<1e-7,"Frozen map-XY translation formula differs");
        require((seed.block<3,3>(0,0)-frames.back().prediction.block<3,3>(0,0)).norm()==0 && seed(2,3)==frames.back().prediction(2,3),"Seed changed rotation or Z");
        group.push_back(seed);++id;
      }
      require(group.size()==8,"Exactly eight fixed seeds required");seeds.push_back(group);
    }
    require(frames.size()==6,"Exactly six frames required");
    for(int i=0;i<6;++i)require(frames[i].raw==78+i,"Frozen raw order differs");
    std::ofstream prep(directory+"/PREPROCESS.csv");require(prep.good(),"Cannot open preprocessing evidence");
    prep<<"raw_ordinal,stamp_ns,raw_xyz_sha256,source_points,source_xyz_sha256,target_points,target_xyz_sha256,source_prepare_ms\n";
    for(const auto& f:frames)prep<<f.raw<<','<<f.stamp<<','<<f.original_xyz_sha<<','<<f.source->size()<<','<<xyz_sha(*f.source)<<','<<target->size()<<','<<target_sha<<','<<std::setprecision(17)<<f.prepare_ms<<'\n';
    prep.close();
    Ndt ndt;ndt.setTransformationEpsilon(.01);ndt.setResolution(1.f);ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    require(ndt.getMaximumIterations()==35 && ndt.getStepSize()==.1 && ndt.getOutlierRatio()==.55,"Original defaults differ");
    std::cout<<"PREFLIGHT_PASS all sources/target, 48 fixed seeds, omp_max_threads="<<omp_get_max_threads()<<std::endl;
    if(mode=="--prepare-only")return 0;
    const int fd=open((directory+"/ALIGN_STARTED").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    require(fd>=0,"Align trial already started");close(fd);
    const auto map_begin=Clock::now();ndt.setInputTarget(target);
    std::cout<<"TARGET_GRID_BUILD_MS "<<std::chrono::duration<double,std::milli>(Clock::now()-map_begin).count()<<std::endl;
    std::ofstream out(directory+"/NATIVE_CANDIDATES.csv"),events(directory+"/EVENT_TIMING.csv");
    require(out.good()&&events.good(),"Cannot open result files");
    out<<"raw_ordinal,stamp_ns,candidate_id,angle_deg,call_number,source_points,source_xyz_sha256,target_xyz_sha256,converged,iterations,native_probability,fitness_all_nn_m2,native_matching_error_m2,inlier_fraction,inlier_points,valid_points,ndt_ms,scoring_ms,diagnostic_ms,peak_RSS_KiB,final_qx,final_qy,final_qz,final_qw";
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<",initial_"<<i<<j;
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<",final_"<<i<<j;
    out<<'\n'<<std::setprecision(17);
    events<<"raw_ordinal,extra_calls,ndt_sum_ms,scoring_sum_ms,diagnostic_sum_ms,event_wall_ms\n"<<std::setprecision(17);
    for(size_t fi=0;fi<frames.size();++fi) {
      const auto& f=frames[fi];const auto event_begin=Clock::now();double ndt_sum=0,score_sum=0,diag_sum=0;
      ndt.setInputSource(f.source);
      for(int id=1;id<=8;++id) {
        require(calls<48,"48-call budget exhausted");const auto& seed=seeds[fi][id-1];
        std::cout<<"BEFORE_ALIGN raw="<<f.raw<<" id="<<id<<" call="<<calls+1<<std::endl;
        Cloud aligned;const auto begin=Clock::now();++calls;ndt.align(aligned,seed);const auto end=Clock::now();
        const auto final=ndt.getFinalTransformation();
        if(!final.allFinite())std::cerr<<"NONFINITE_FINAL raw="<<f.raw<<" id="<<id<<'\n'<<final<<std::endl;
        require(final.allFinite(),"Nonfinite core; stop subsequent calls without retry");
        const Eigen::Quaternionf q(final.block<3,3>(0,0));
        const double fitness=ndt.getFitnessScore();int inliers=0,valid=0;double error=0;
        std::vector<int> indices;std::vector<float> distances;
        for(const auto& p:aligned) {
          if(p.getVector3fMap().norm()>25.)continue;
          ++valid;ndt.getSearchMethodTarget()->nearestKSearch(p,1,indices,distances);
          require(!distances.empty(),"Native status NN unavailable");
          if(distances[0]<.5*.5){error+=distances[0];++inliers;}
        }
        const double matching=inliers?error/inliers:std::numeric_limits<double>::quiet_NaN();
        const float fraction=static_cast<float>(inliers)/std::max(1,valid);
        const auto scored=Clock::now();require(xyz_sha(*target)==target_sha,"Target changed");
        struct rusage usage;getrusage(RUSAGE_SELF,&usage);
        const double ms=std::chrono::duration<double,std::milli>(end-begin).count();
        const double scoring=std::chrono::duration<double,std::milli>(scored-end).count();
        const double diagnostic=std::chrono::duration<double,std::milli>(Clock::now()-end).count();
        ndt_sum+=ms;score_sum+=scoring;diag_sum+=diagnostic;
        out<<f.raw<<','<<f.stamp<<','<<id<<','<<(id-1)*45<<','<<calls<<','<<f.source->size()<<','<<xyz_sha(*f.source)<<','<<target_sha<<','<<ndt.hasConverged()<<','<<ndt.getFinalNumIteration()<<','<<ndt.getTransformationProbability()<<','<<fitness<<','<<matching<<','<<fraction<<','<<inliers<<','<<valid<<','<<ms<<','<<scoring<<','<<diagnostic<<','<<usage.ru_maxrss<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<seed(i,j);
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<final(i,j);
        out<<'\n';out.flush();
        std::cout<<"AFTER_ALIGN raw="<<f.raw<<" id="<<id<<" call="<<calls<<" converged="<<ndt.hasConverged()<<std::endl;
      }
      events<<f.raw<<",8,"<<ndt_sum<<','<<score_sum<<','<<diag_sum<<','<<std::chrono::duration<double,std::milli>(Clock::now()-event_begin).count()<<'\n';events.flush();
    }
    require(calls==48,"Incomplete calls");std::cout<<"COMPLETE align=48 UKF=0 ROS_replay=0"<<std::endl;return 0;
  }catch(const std::exception& e){std::cerr<<"CONTRACT_OR_EXECUTION_BLOCKED calls="<<calls<<" reason="<<e.what()<<std::endl;return 2;}
}
