// Reuse the historical loader, voxel, and map ICP without executing its main.
#define main historicalUnusedMain
#include "../p32_init_transfer/legacy/superloc_first_segment_initializer.cpp"
#undef main
#include <chrono>

namespace {
Eigen::Matrix3f worldAxis(int id) {
  Eigen::Matrix3f Q;
  if(id==0)Q=Eigen::Matrix3f::Identity();
  else if(id==1)Q<<0,-1,0,1,0,0,0,0,1;
  else if(id==2)Q<<0,1,0,-1,0,0,0,0,1;
  else if(id==3)Q<<-1,0,0,0,-1,0,0,0,1;
  else throw std::runtime_error("UNREGISTERED_Q");
  return Q;
}
Transform seed(int id,const Eigen::Matrix3f& R,const Eigen::Vector3f& t) {
  Transform out=Transform::Identity();const auto Q=worldAxis(id);
  out.linear()=Q*R;out.translation()=Q*t;return out;
}
bool legal(const Transform& T) {
  const Eigen::Matrix3f R=T.linear();
  return T.matrix().allFinite()&&(T.matrix().row(3)-Eigen::RowVector4f(0,0,0,1)).norm()==0
    &&std::abs(R.determinant()-1)<=1e-5
    &&(R.transpose()*R-Eigen::Matrix3f::Identity()).norm()<=1e-5;
}
void matrix(std::ostream& out,const Transform& T) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<T.matrix()(i,j);
}
struct Geometry {
  double median,p90,half,eight;
  Eigen::Vector3f low,high;
};
Geometry geometry(const Cloud::ConstPtr& source,const Transform& T,pcl::KdTreeFLANN<pcl::PointXYZ>& tree) {
  Cloud moved;pcl::transformPointCloud(*source,moved,T);
  std::vector<float> d;std::vector<int> ids(1);std::vector<float> squared(1);
  size_t half=0,eight=0;const float inf=std::numeric_limits<float>::infinity();
  Eigen::Vector3f low=Eigen::Vector3f::Constant(inf),high=Eigen::Vector3f::Constant(-inf);
  for(const auto& p:moved) {
    if(!pcl::isFinite(p))throw std::runtime_error("NONFINITE_TRANSFORMED_POINT");
    const Eigen::Vector3f v(p.x,p.y,p.z);low=low.cwiseMin(v);high=high.cwiseMax(v);
    if(tree.nearestKSearch(p,1,ids,squared)!=1)throw std::runtime_error("NN_QUERY_FAILED");
    const float x=std::sqrt(squared[0]);d.push_back(x);half+=x<=.5f;eight+=x<=8.f;
  }
  if(d.empty())throw std::runtime_error("EMPTY_SOURCE");
  std::sort(d.begin(),d.end());
  return {d[d.size()/2],d[size_t(.9*(d.size()-1))],double(half)/d.size(),double(eight)/d.size(),low,high};
}
void metrics(std::ostream& out,const Geometry& g) {
  out<<','<<g.median<<','<<g.p90<<','<<g.half<<','<<g.eight;
  for(int i=0;i<3;++i)out<<','<<g.low(i)<<','<<g.high(i);
}
void columns(std::ostream& out,const std::string& prefix) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<prefix<<i<<j;
}
int mathTest() {
  const Eigen::Vector3f t(39.929523f,-176.217148f,-5.225427f);
  Eigen::Matrix3f R=Eigen::AngleAxisf(.6f,Eigen::Vector3f::UnitX()).toRotationMatrix();
  for(int id=0;id<4;++id) {
    const auto Q=worldAxis(id);const auto T=seed(id,R,t);
    if(Q.determinant()!=1||(Q.transpose()*Q-Eigen::Matrix3f::Identity()).norm()!=0||!legal(T))return 1;
    if((T.linear()-Q*R).norm()!=0||(T.translation()-Q*t).norm()!=0)return 2;
  }
  if((seed(1,R,t).translation()-Eigen::Vector3f(176.217148f,39.929523f,-5.225427f)).norm()!=0)return 3;
  if((worldAxis(1)*R-R*worldAxis(1)).norm()<.1)return 4;
  Transform bad=Transform::Identity();bad.linear()(0,0)=2;if(legal(bad))return 5;
  bad=Transform::Identity();bad.matrix()(3,0)=1;if(legal(bad))return 6;
  std::cout<<"WORLD_FRAME_MATH_PASS four_Q=PASS left_rotation_and_translation=PASS no_offset=PASS final_SE3=PASS\n";return 0;
}
}

int main(int argc,char** argv) {
  if(argc==2&&std::string(argv[1])=="--math-test")return mathTest();
  if(argc!=5)return 2;
  try {
    Cloud::Ptr source(new Cloud),map(new Cloud);
    if(pcl::io::loadPCDFile(argv[1],*source)<0||pcl::io::loadPCDFile(argv[2],*map)<0)return 3;
    if(source->size()!=2569||map->empty())throw std::runtime_error("FROZEN_POINT_COUNT");
    const auto target=voxel(map,.20f);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
    Eigen::Matrix3f R;Eigen::Vector3f t;if(!loadOfficialPrior(argv[3],R,t))return 4;
    Eigen::Matrix3f expected;expected<<.823800f,-.566853f,-.005503f,.565879f,.822884f,-.051418f,.033675f,.039243f,.998662f;
    if((R-expected).norm()!=0||(t-Eigen::Vector3f(39.929523f,-176.217148f,-5.225427f)).norm()!=0)
      throw std::runtime_error("ACTIVE_PRIOR_BINDING");
    const std::string root=argv[4];
    std::ofstream screens(root+"/world_candidates.csv");screens<<std::setprecision(17);
    screens<<"candidate_id,yaw_deg,legal,nn_median_m,nn_p90_m,coverage_0_5m,coverage_8m,bbox_xmin,bbox_xmax,bbox_ymin,bbox_ymax,bbox_zmin,bbox_zmax,screen_pass,nn_ms";columns(screens,"seed");screens<<'\n';
    std::vector<std::pair<double,int>> eligible;
    const int yaw[4]={0,90,-90,180};
    for(int id=0;id<4;++id) {
      const auto T=seed(id,R,t);if(!legal(T))throw std::runtime_error("ILLEGAL_SEED");
      const auto tick=std::chrono::steady_clock::now();const auto g=geometry(source,T,tree);
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
      const bool pass=g.median<=8&&g.eight>=.5;
      screens<<id<<','<<yaw[id]<<",1";metrics(screens,g);screens<<','<<pass<<','<<ms;matrix(screens,T);screens<<'\n';
      if(pass)eligible.emplace_back(g.median,id);
    }
    screens.close();std::sort(eligible.begin(),eligible.end());if(eligible.size()>2)eligible.resize(2);
    std::ofstream results(root+"/map_icp.csv");results<<std::setprecision(17);
    results<<"candidate_id,converged,fitness,nn_median_m,nn_p90_m,coverage_0_5m,coverage_8m,bbox_xmin,bbox_xmax,bbox_ymin,bbox_ymax,bbox_zmin,bbox_zmax,final_legal,admitted,correction_m,correction_deg,icp_and_legacy_score_ms,final_nn_ms";columns(results,"result");results<<'\n';
    size_t passed=0;
    for(const auto& entry:eligible) {
      const int id=entry.second;const auto T=seed(id,R,t);const auto tick=std::chrono::steady_clock::now();
      const auto c=registerCandidate(id,T,source,target); // exactly one original map ICP
      const double icp_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
      const auto g=geometry(source,c.T,tree);
      const double final_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count()-icp_ms;
      const bool admitted=c.converged&&std::isfinite(c.fitness)&&c.fitness<=10&&g.median<=1&&g.p90<=3&&g.half>=.5&&legal(c.T);
      passed+=admitted;
      results<<id<<','<<c.converged<<','<<c.fitness;metrics(results,g);
      results<<','<<legal(c.T)<<','<<admitted<<','<<(c.T.translation()-T.translation()).norm()<<','
        <<rotationAngle(T.linear().transpose()*c.T.linear())*180/M_PI<<','<<icp_ms<<','<<final_ms;
      matrix(results,c.T);results<<'\n';results.flush();
    }
    std::cout<<"WORLD_FRAME_LEDGER nn_screens=4 map_icp="<<eligible.size()<<" admitted="<<passed
      <<" source="<<source->size()<<" target="<<target->size()<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 5;}
}
