// Reuse the original voxel/map ICP, never its initializer/search main.
#define main historicalUnusedMain
#include "../p32_init_transfer/legacy/superloc_first_segment_initializer.cpp"
#undef main
#include <chrono>

namespace {
bool legal(const Eigen::Matrix4d& T) {
  const Eigen::Matrix3d R=T.block<3,3>(0,0);
  return T.allFinite()&&(T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()==0
    &&std::abs(R.determinant()-1)<=1e-5
    &&(R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<=1e-5;
}
struct Geometry { double median,p90,half,two,eight; Eigen::Vector3f low,high; };
Geometry geometry(const Cloud::ConstPtr& source,const Transform& T,pcl::KdTreeFLANN<pcl::PointXYZ>& tree) {
  Cloud moved;pcl::transformPointCloud(*source,moved,T);
  std::vector<float> distances;std::vector<int> ids(1);std::vector<float> squared(1);
  size_t half=0,two=0,eight=0;const float inf=std::numeric_limits<float>::infinity();
  Eigen::Vector3f low=Eigen::Vector3f::Constant(inf),high=Eigen::Vector3f::Constant(-inf);
  for(const auto& p:moved) {
    if(!pcl::isFinite(p)||tree.nearestKSearch(p,1,ids,squared)!=1||!std::isfinite(squared[0])||squared[0]<0)
      throw std::runtime_error("INVALID_NN");
    const Eigen::Vector3f xyz(p.x,p.y,p.z);low=low.cwiseMin(xyz);high=high.cwiseMax(xyz);
    const float d=std::sqrt(squared[0]);distances.push_back(d);half+=d<=.5f;two+=d<=2.f;eight+=d<=8.f;
  }
  if(distances.empty())throw std::runtime_error("EMPTY_SOURCE");
  std::sort(distances.begin(),distances.end());const size_t n=distances.size();
  return {distances[n/2],distances[size_t(.9*(n-1))],double(half)/n,double(two)/n,double(eight)/n,low,high};
}
void writeGeometry(const std::string& path,const Geometry& g) {
  std::ofstream out(path);out<<std::setprecision(17);
  out<<"nn_median_m,nn_p90_m,coverage_0_5m,coverage_2m,coverage_8m,bbox_xmin,bbox_xmax,bbox_ymin,bbox_ymax,bbox_zmin,bbox_zmax\n";
  out<<g.median<<','<<g.p90<<','<<g.half<<','<<g.two<<','<<g.eight;
  for(int axis=0;axis<3;++axis)out<<','<<g.low(axis)<<','<<g.high(axis);out<<'\n';
  if(!out)throw std::runtime_error("GEOMETRY_WRITE_FAILED");
}
}

int main(int argc,char** argv) {
  if(argc!=5)return 2;
  try {
    std::ifstream input(argv[3],std::ios::binary|std::ios::ate);
    if(!input||input.tellg()!=128)throw std::runtime_error("MATRIX_BYTES");input.seekg(0);
    Eigen::Matrix4d matrix;
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)input.read(reinterpret_cast<char*>(&matrix(i,j)),8);
    if(!input||!legal(matrix))throw std::runtime_error("ILLEGAL_ANNOTATED_MATRIX");
    Transform seed=Transform::Identity();seed.matrix()=matrix.cast<float>();
    if(!legal(seed.matrix().cast<double>()))throw std::runtime_error("ILLEGAL_FLOAT_MATRIX");
    Cloud::Ptr source(new Cloud),map(new Cloud);
    if(pcl::io::loadPCDFile(argv[1],*source)<0||pcl::io::loadPCDFile(argv[2],*map)<0)return 3;
    if(source->size()!=2569||map->empty())throw std::runtime_error("FROZEN_POINT_COUNT");
    for(const auto& p:*source)if(!pcl::isFinite(p))throw std::runtime_error("NONFINITE_SOURCE");
    const auto target=voxel(map,.20f);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
    const std::string output=argv[4];const auto tick=std::chrono::steady_clock::now();
    const auto initial=geometry(source,seed,tree); // the sole initial NN screen
    writeGeometry(output+"/initial_nn.csv",initial);
    const double initial_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
    const bool screen=initial.median<=8&&initial.eight>=.5;
    bool admitted=false;
    if(screen) {
      const auto start=std::chrono::steady_clock::now();
      const auto result=registerCandidate(1,seed,source,target); // sole original map ICP
      const double icp_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      const auto final=geometry(source,result.T,tree);
      writeGeometry(output+"/final_nn.csv",final);
      const bool final_legal=legal(result.T.matrix().cast<double>());
      admitted=final_legal&&result.converged&&std::isfinite(result.fitness)&&result.fitness<=10
        &&final.median<=1&&final.p90<=3&&final.half>=.5;
      std::ofstream out(output+"/map_icp.csv");out<<std::setprecision(17);
      out<<"converged,fitness,final_legal,admitted,correction_m,correction_deg,icp_and_legacy_score_ms";
      for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<",T"<<i<<j;
      out<<'\n'<<result.converged<<','<<result.fitness<<','<<final_legal<<','<<admitted<<','
        <<(result.T.translation()-seed.translation()).norm()<<','
        <<rotationAngle(seed.linear().transpose()*result.T.linear())*180/M_PI<<','<<icp_ms;
      for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<result.T.matrix()(i,j);out<<'\n';
      if(!out)throw std::runtime_error("ICP_WRITE_FAILED");
    }
    std::ofstream ledger(output+"/ledger.json");
    ledger<<"{\"initial_NN\":1,\"map_ICP\":"<<int(screen)<<",\"admitted\":"<<int(admitted)
      <<",\"source_points\":"<<source->size()<<",\"target_points\":"<<target->size()
      <<",\"initial_NN_ms\":"<<std::setprecision(17)<<initial_ms<<",\"GT_reads\":0}\n";
    if(!ledger)throw std::runtime_error("LEDGER_WRITE_FAILED");
    std::cout<<"ANNOTATED_POSE_LEDGER initial_NN=1 map_ICP="<<screen<<" admitted="<<admitted<<" GT=0\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 5;}
}
