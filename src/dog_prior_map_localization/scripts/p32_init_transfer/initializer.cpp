// The complete historical main executes unchanged; subclass only observes align.
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <iostream>
namespace p32 {
std::ofstream ledger;
int map_calls=0,local_calls=0;
template<typename M> void matrix(std::ostream& out,const M& T) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<T(i,j);
}
}
namespace pcl {
template<typename Source,typename Target>
class P32LoggedICP : public IterativeClosestPoint<Source,Target> {
  using Base=IterativeClosestPoint<Source,Target>;
public:
  void align(pcl::PointCloud<Source>& output,const Eigen::Matrix4f& guess) {
    const bool map=this->getMaxCorrespondenceDistance()==8.;
    if(map&&++p32::map_calls>24)throw std::runtime_error("CANDIDATE_BUDGET");
    if(!map)++p32::local_calls;
    const auto tick=std::chrono::steady_clock::now();
    Base::align(output,guess); // Same solver, same source/target and seed.
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
    if(!map)return;
    const Eigen::Matrix4f final=this->getFinalTransformation();
    pcl::KdTreeFLANN<Target> tree;tree.setInputCloud(this->getInputTarget());
    std::vector<double> distances;std::vector<int> ids(1);std::vector<float> squared(1);size_t inside=0;
    for(const auto& p:output) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      if(tree.nearestKSearch(p,1,ids,squared)==1) {
        const double d=std::sqrt(squared[0]);distances.push_back(d);if(d<=.5)++inside;
      }
    }
    std::sort(distances.begin(),distances.end());
    const double median=distances.empty()?std::numeric_limits<double>::infinity():distances[distances.size()/2];
    const double p90=distances.empty()?std::numeric_limits<double>::infinity():distances[size_t(.9*(distances.size()-1))];
    const double fraction=distances.empty()?0.:double(inside)/distances.size();
    const double error=(guess.block<3,3>(0,0).transpose()*guess.block<3,3>(0,0)-Eigen::Matrix3f::Identity()).norm();
    const double det=guess.block<3,3>(0,0).determinant();
    const double translation=(final.block<3,1>(0,3)-guess.block<3,1>(0,3)).norm();
    const double angle=Eigen::AngleAxisf(guess.block<3,3>(0,0).transpose()*final.block<3,3>(0,0)).angle()*180/M_PI;
    const double diag=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count()-ms;
    p32::ledger<<p32::map_calls-1<<','<<this->getInputSource()->size()<<','<<this->hasConverged()<<','
      <<this->getFitnessScore()<<','<<median<<','<<p90<<','<<fraction<<','<<translation<<','<<angle<<','
      <<det<<','<<error<<','<<(guess.allFinite()&&det>0&&error<=1e-5)<<','<<ms<<','<<diag;
    p32::matrix(p32::ledger,guess);p32::matrix(p32::ledger,final);p32::ledger<<'\n';p32::ledger.flush();
  }
};
}
#define IterativeClosestPoint P32LoggedICP
#define main historicalInitializerMain
#include "legacy/superloc_first_segment_initializer.cpp"
#undef main
#undef IterativeClosestPoint
int main(int argc,char** argv) {
  if(argc<5)return 2;
  p32::ledger.open(std::string(argv[4])+"/candidate_diagnostics.csv");
  p32::ledger<<std::setprecision(17)<<"candidate_id,source_points,converged,fitness,nn_median_m,nn_p90_m,inlier_0_5m,correction_m,correction_deg,seed_determinant,seed_orthogonality_error,seed_SE3_float_tolerance,icp_ms,diagnostics_ms";
  for(const auto prefix:{"seed","result"})for(int i=0;i<4;++i)for(int j=0;j<4;++j)p32::ledger<<','<<prefix<<i<<j;
  p32::ledger<<'\n';
  const int code=historicalInitializerMain(argc,argv);
  std::cout<<"P32_CALL_LEDGER local_icp="<<p32::local_calls<<" map_icp="<<p32::map_calls<<'\n';
  return code;
}
