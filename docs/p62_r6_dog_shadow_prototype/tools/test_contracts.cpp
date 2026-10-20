#include "native_kernel.hpp"
#include <pcl/registration/ndt.h>
#include <pcl/point_types.h>
#include <iostream>
using namespace shadow;
class NumericalOracle : public NativeNdt {
 public:
  Matrix6 finiteDifferenceG(const Cloud::Ptr& source,const Pose& t){
    Matrix6 g=Matrix6::Zero();std::vector<TargetGrid::LeafConstPtr> leaves;std::vector<float> distances;
    for(const auto& p:*source){const Eigen::Vector3d x(p.x,p.y,p.z),y=t.block<3,3>(0,0)*x+t.block<3,1>(0,3);
      Eigen::Matrix<double,3,6> jac;const double epsilon=1e-6;
      for(int col=0;col<6;++col){std::array<Eigen::Vector3d,2> value;
        for(int sign=0;sign<2;++sign){const double step=(sign==0?-1:1)*epsilon;Pose perturbed=t;
          if(col<3)perturbed(col,3)+=.8*step;
          else perturbed.block<3,3>(0,0)=Eigen::AngleAxisd(step,Eigen::Vector3d::Unit(col-3)).toRotationMatrix()*t.block<3,3>(0,0);
          value[sign]=perturbed.block<3,3>(0,0)*x+perturbed.block<3,1>(0,3);}
        jac.col(col)=(value[1]-value[0])/(2*epsilon);}
      leaves.clear();distances.clear();this->target_cells_.radiusSearch(pcl::PointXYZ(y.x(),y.y(),y.z()),this->resolution_,leaves,distances);
      for(const auto& leaf:leaves){const auto residual=(y-leaf->getMean()).eval();const auto& inv=leaf->getInverseCov();const double ex=std::exp(-this->gauss_d2_*residual.dot(inv*residual)/2),guard=this->gauss_d2_*ex;
        if(std::isfinite(guard)&&guard>=0&&guard<=1)g+=(-this->gauss_d1_*this->gauss_d2_*ex)*jac.transpose()*inv*jac;}
    }
    return g/source->size();
  }
};
int main(){try{
  require(csvFields("tx,qw\r").back()=="qw"&&csvFields("0,1\r").back()=="1","frozen CRLF CSV parser");
  require(sector(-1,0)==0&&sector(-1,-0.)==0&&sector(1,0)==8&&sector(0,1)==12&&sector(0,-1)==4,"sector half-open axes");
  int train=0,validation=0;std::array<int,8> counts{};
  for(int i=0;i<16;++i){const double a=-PI+(i+.5)*2*PI/16;const int s=sector(std::cos(a),std::sin(a));require(s==i,"16 sector centers");
    if(s%2){++validation;++counts[s/2];}else ++train;}
  require(train==8&&validation==8&&std::all_of(counts.begin(),counts.end(),[](int n){return n==1;}),"zero training/validation overlap");
  Loss j{{0,0,.5,.5,.5,.5,1,1}},k{{.5,.5,.5,.5,.5,.5,.5,.5}};
  double exhaustive=1e9;
  for(unsigned m=0;m<256;++m)if(__builtin_popcount(m)==4){double v=0;for(int b=0;b<8;++b)if(m&(1U<<b))v+=.25*(k[b]-j[b]);exhaustive=std::min(exhaustive,v);}
  require(std::abs(paired(j,k)-exhaustive)<1e-15&&paired(j,k)<0,"common-weight worst case, not difference of robust scores");
  Loss lo{{.1,.1,.1,.1,.1,.1,.1,.1}},hi{{.4,.4,.4,.4,.4,.4,.4,.4}};
  require(paired(lo,hi)>.02&&paired(hi,lo)<-.02&&paired(lo,lo)==0,"positive negative tied paired examples");
  std::vector<Candidate> c(3);for(int i=0;i<3;++i){c[i].id=i;c[i].pose(0,3)=i*.3;c[i].finite=c[i].loss_valid=c[i].eligible=true;c[i].score=i;c[i].loss=hi;}
  c[1].loss=lo;auto s=select(c);require(s.native==2&&s.heldout_mean==1&&s.robust==1,"same-pool three selectors");
  c[2].loss=lo;require(select(c).robust<0,"tied alternatives abstain");
  c[0].eligible=false;c[0].loss=lo;c[1].loss=hi;c[2].eligible=false;
  require(select(c).robust<0,"ineligible T0 remains a comparison rival");
  c[0].loss=hi;c[1].loss.fill(.2);c[2].loss=lo;
  require(select(c).robust<0,"alternative that beats T0 but loses ineligible rival abstains");
  c[1].pose=c[0].pose;deduplicate(c);require(c[1].duplicate_of==0&&pool(c).size()==2,"duplicate physical candidate");
  for(auto& a:c)a.eligible=false;require(select(c).native<0&&select(c).heldout_mean<0&&select(c).robust<0,"no supported recovery abstention");
  Vector6 v;v<<.2,.4,-.1,1,2,-3;
  for(double sign:{-1.,1.}){const auto t=seed(Pose::Identity(),v,sign);require(translation(t,Pose::Identity())<=.30+1e-12&&rotation(t,Pose::Identity())<=3+1e-10,"coupled seed physical limits");}
  Budget budget;for(int i=0;i<20;++i)require(budget.consume(),"budget token");require(!budget.consume()&&budget.used==20,"budget exhausted");
  require(!trigger(true,2,lo)&&budget.used==20,"ordinary frame no additional align token");
  bool rejected=false;try{sector(NAN,1);}catch(...){rejected=true;}require(rejected,"nonfinite rejection");
  auto missing=lo;missing[3]=NAN;require(!validLoss(missing),"invalid block not zero");
  using Cloud=pcl::PointCloud<pcl::PointXYZ>;Cloud::Ptr target(new Cloud),source(new Cloud);
  unsigned state=0x13579bdf;auto rand=[&](){state=1664525U*state+1013904223U;return double(state)/double(UINT32_MAX);};
  for(int i=0;i<8000;++i)target->push_back(pcl::PointXYZ(float(5*rand()),float(4*rand()),float(3*rand())));
  for(int i=0;i<800;++i)source->push_back(target->points[i*7]);
  pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> a;NumericalOracle b;b.configure(target);b.checkGrid();
  a.setResolution(.8f);a.setStepSize(.08);a.setTransformationEpsilon(.001);a.setMaximumIterations(40);a.setOulierRatio(.55);a.setInputTarget(target);a.setInputSource(source);b.setInputSource(source);
  Cloud x,y;a.align(x,Eigen::Matrix4f::Identity());b.align(y,Eigen::Matrix4f::Identity());
  require((a.getFinalTransformation()-b.getFinalTransformation()).norm()==0&&a.getTransformationProbability()==b.getTransformationProbability(),"native deterministic synthetic optimizer fixture");
  size_t support=0;const Matrix6 g=b.geometry(source,b.getFinalTransformation().cast<double>(),support);Eigen::SelfAdjointEigenSolver<Matrix6> eig(g);
  require(support>0&&g.allFinite()&&(g-g.transpose()).norm()<1e-10&&eig.eigenvalues()[0]>-1e-8,"PSD direction matrix from native Gaussian grid");
  Pose perturbed=Pose::Identity();perturbed.block<3,3>(0,0)=Eigen::AngleAxisd(.1,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();perturbed.block<3,1>(0,3)=Eigen::Vector3d(.2,-.1,.15);
  const Matrix6 analytic=b.geometry(source,perturbed,support),oracle=b.finiteDifferenceG(source,perturbed);
  require((analytic-oracle).norm()/std::max(1.,oracle.norm())<1e-8,"independent chart finite difference G oracle");
  Cloud::Ptr circular(new Cloud);for(int s=0;s<16;++s)for(int n=0;n<80;++n){const double theta=-PI+(s+.5)*2*PI/16;circular->push_back(pcl::PointXYZ(std::cos(theta),std::sin(theta),0));}
  auto split=splitSource(circular);require(split.evaluable()&&split.training->size()==640,"complete validation blocks");split.blocks[0]->resize(9);require(!split.evaluable(),"under10 block not evaluable");
  split.blocks[0]->resize(80);split.training->resize(299);require(!split.evaluable(),"training299 rejected");split.training->resize(300);require(split.evaluable(),"training300 accepted");
  auto original_split=splitSource(circular);const auto hash_before=cloudHash(original_split.training);
  for(auto& p:*circular)if(sector(p.x,p.y)%2){p.x*=1.1;p.y*=1.1;}
  auto changed_split=splitSource(circular);require(cloudHash(changed_split.training)==hash_before,"heldout changes do not enter training");
  const Matrix6 g1=b.geometry(original_split.training,Pose::Identity(),support),g2=b.geometry(changed_split.training,Pose::Identity(),support);require((g1-g2).norm()==0,"heldout changes do not enter G");
  Cloud::Ptr one(new Cloud);one->push_back(pcl::PointXYZ(0,0,0));pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(one);Split nearest;
  nearest.training->resize(300);for(auto& block:nearest.blocks)for(int i=0;i<10;++i)block->push_back(pcl::PointXYZ(0,0,0));
  Candidate test;test.finite=test.converged=true;test.pose(0,3)=.25;validate(test,nearest,tree,Pose::Identity());require(test.eligible&&std::abs(mean(test.loss)-.25)<1e-12&&test.overlap==1,"NN squared loss/overlap gate");
  test.pose(0,3)=.6;validate(test,nearest,tree,Pose::Identity());require(!test.eligible&&mean(test.loss)==1&&test.overlap==0,"clipped loss and poor support rejection");
  Pose far=Pose::Identity();far(0,3)=.801;require(!bounded(far,Pose::Identity()),"output translation rejection");
  far=Pose::Identity();far.block<3,3>(0,0)=Eigen::AngleAxisd(5.01*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();require(!bounded(far,Pose::Identity()),"output rotation rejection");
  std::cout<<"CONTRACT_TESTS_PASS categories=18 synthetic_native_aligns=2 real_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
