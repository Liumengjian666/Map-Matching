#include <pcl/kdtree/kdtree_flann.h>
#define P64_DIRECTIONAL_TOOL
#include "pipeline.cpp"
#undef P64_DIRECTIONAL_TOOL

struct OriginalProbe : InstrumentedScorer {
  using InstrumentedScorer::InstrumentedScorer;
  double native_d2() const { return gauss_d2_; }
};

int main(int argc,char** argv) {
  try {
    need(argc==6,"directional SOURCE MAP SEED_TSV OUT MODE");
    const auto mode=std::string(argv[5]);
    const auto cloud=load(argv[1]);
    OriginalProbe scorer(downsample(load(argv[2])));
    Pose seed;
    std::ifstream in(argv[3]);
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)need(bool(in>>seed(i,j)),"pose");
    need(seed.allFinite(),"finite pose");
    if(mode=="fit"){
      scorer.setInputSource(cloud);
      Cloud terminal;
      const auto start=Clock::now();
      scorer.align(terminal,seed);
      const auto pose=scorer.getFinalTransformation();
      const auto runtime=elapsed(start);
      const auto s0=scorer.fixed(*cloud,seed);
      const auto s1=pose.allFinite()?scorer.fixed(*cloud,pose):Score{};
      auto out=file(argv[4],"converged,iterations,n,current_associations,seed_reward,end_reward,fit_s,seed,terminal");
      out<<scorer.hasConverged()<<','<<scorer.getFinalNumIteration()<<','<<cloud->size()<<','<<s1.associated<<','
        <<s0.native<<','<<s1.native<<','<<runtime<<',';
      list(out,seed);out<<',';list(out,pose);out<<'\n';
      return 0;
    }
    need(mode=="export","bad mode");
    Eigen::Vector3f axis;
    need(bool(in>>axis.x()>>axis.y()>>axis.z())&&std::abs(axis.norm()-1)<1e-4,"unit world weak axis");
    const float eps=.01;
    Pose plus=seed,minus=seed;
    plus.topRightCorner<3,1>()+=eps*axis;
    minus.topRightCorner<3,1>()-=eps*axis;
    std::vector<q1::PointScore> nominal,left,right;
    scorer.fixed(*cloud,seed,&nominal);scorer.fixed(*cloud,minus,&left);scorer.fixed(*cloud,plus,&right);
    const Eigen::Matrix3d R=seed.topLeftCorner<3,3>().cast<double>();
    auto out=file(argv[4],"point,x,y,z,native_reward,native_gradient_weak,fd_gradient_weak,fd_curvature_weak,g_assoc,cells_changed,max_term,mean_mahal,nearest_mean_m,regularized_like,ndt_information");
    for(std::size_t i=0;i<cloud->size();++i) {
      const Eigen::Vector3d p=(*cloud)[i].getVector3fMap().cast<double>();
      const Eigen::Vector3d world=nominal[i].transformed.getVector3fMap().cast<double>();
      Eigen::Matrix3d p_x;
      p_x<<0,-p.z(),p.y(),p.z(),0,-p.x(),-p.y(),p.x(),0;
      Eigen::Matrix<double,3,6> J;
      J<<R,-R*p_x/5.;
      Mat6 h=Mat6::Zero(); double analytic=0,max_term=0,mahal=0,dist=std::numeric_limits<double>::infinity();
      int clamped=0;
      auto ids=[](const auto& terms){std::vector<int> v;for(const auto& t:terms)v.push_back(t.cell);std::sort(v.begin(),v.end());return v;};
      const auto central=ids(nominal[i].neighborhoods);
      const bool changed=central!=ids(left[i].neighborhoods)||central!=ids(right[i].neighborhoods);
      for(const auto& term:nominal[i].neighborhoods){
        const auto& cell=scorer.cells().at(term.cell);
        const Eigen::Matrix3d P=cell.getInverseCov();
        const Eigen::Vector3d d=world-cell.getMean();
        h.noalias()+=term.score*J.transpose()*P*J;
        analytic-=scorer.native_d2()*term.score*axis.cast<double>().dot(P*d);
        max_term=std::max(max_term,term.score);mahal+=term.mahalanobis;dist=std::min(dist,d.norm());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(P,Eigen::EigenvaluesOnly);
        if(eig.eigenvalues()(2)>99.*eig.eigenvalues()(0))++clamped;
      }
      const double fd=(right[i].total-left[i].total)/(2*eps);
      const double curve=(right[i].total+left[i].total-2*nominal[i].total)/(eps*eps);
      out<<i<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<nominal[i].total<<','<<analytic<<','<<fd<<','<<curve<<','
        <<central.size()<<','<<changed<<','<<max_term<<','<<(central.empty()?0:mahal/central.size())<<','
        <<dist<<','<<clamped<<',';list(out,h);out<<'\n';
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
