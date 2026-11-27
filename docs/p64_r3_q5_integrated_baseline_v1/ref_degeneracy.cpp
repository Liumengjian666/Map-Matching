#include <Eigen/Eigenvalues>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <dcreg.hpp>
#define MELO_DEBUG_STREAM(x) ((void)0)
#define MELO_INFO_STREAM(x) ((void)0)

struct XCore {
  using Vector=Eigen::Vector3d;
  using AlignmentVector=std::vector<Vector,Eigen::aligned_allocator<Vector>>;
  using AlignmentList=std::vector<std::pair<Eigen::Index,double>>;
  enum class LocalizabilityCategory{kNonLocalizable=0,kLocalizable=1};
  enum class LocalizabilitySamplingType{kInsufficientPoints=0,kUnnecessary=1,
    kMixedContributionPoints=2,kHighContributionPoints=3};
  struct LocalizabilityParametersForErrorMinimization {
    struct {
      Vector localizabilityXyz_=Vector::Zero(),localizabilityRpy_=Vector::Zero();
      struct {Vector translationConstraintValues_=Vector::Zero(),rotationConstraintValues_=Vector::Zero();}
        localizabilityConstraints_;
    } localizabilityAnalysisResults_;
  };
  struct {
    bool isPrintingEnabled_=false;
    double highInformationThreshold=350,enoughInformationThreshold=150,insufficientInformationThreshold=40;
    double point2NormalMinimalAlignmentCosineThreshold=std::cos(80*M_PI/180);
    double point2NormalStrongAlignmentCosineThreshold=std::cos(45*M_PI/180);
    double combinedContribution=0,highContribution=0;
    int highlyContributingNumberOfPoints=0,contributingNumberOfPoints=0;
    Eigen::Index numberOfPoints=0;
    AlignmentList samplingAlignmentList;
  } localizabilityDetectionParameters;
  bool countContributionValuesAndCheckWhetherProblemIsConstrainedVeryWell(const AlignmentVector&,const Vector&);
  LocalizabilitySamplingType decideLocalizabilityLevel(LocalizabilityParametersForErrorMinimization&,
      const Vector&,const AlignmentVector&,const Eigen::Index&,bool);
};
using T = double;
#include "xicp_extracted.inc"

template<typename A>
void vector(std::ostream& out,const A& columns) {
  for(int k=0;k<columns.size();++k){
    if(k)out<<';';
    out<<columns(k);
  }
}
int main(int argc,char** argv) {
  if(argc!=3)return 1;
  std::ifstream in(argv[1]);std::ofstream out(argv[2]);
  out<<std::setprecision(12)<<"case,method,stage,n,full_spectrum,rot,trans,factor_ok,classification,rot_basis,trans_basis,elapsed_ms\n";
  int n;std::string name,stage;
  while(in>>name>>stage>>n){
    XCore::AlignmentVector normals,rotations;
    dcreg::Matrix6d h=dcreg::Matrix6d::Zero();
    for(int i=0;i<n;++i){
      Eigen::Vector3d p,norm;
      if(!(in>>p[0]>>p[1]>>p[2]>>norm[0]>>norm[1]>>norm[2]))throw std::runtime_error("bad row");
      norm.normalize();
      Eigen::Vector3d rotation=p.cross(norm);
      Eigen::Matrix<double,6,1> j;j<<rotation/5.,norm;
      h+=j*j.transpose();
      normals.push_back(norm);
      rotations.push_back(rotation.norm()>1e-9 ? rotation.normalized().eval() : Eigen::Vector3d::Zero());
    }
    const auto begin=std::chrono::steady_clock::now();
    Eigen::SelfAdjointEigenSolver<dcreg::Matrix6d> es(h);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> er(h.topLeftCorner<3,3>()),et(h.bottomRightCorner<3,3>());
    XCore::LocalizabilityParametersForErrorMinimization params;
    Eigen::Matrix<int,6,1> categories;
    for(int a=0;a<6;++a){
      XCore engine;
      engine.localizabilityDetectionParameters.numberOfPoints=n;
      const XCore::Vector e=a<3?er.eigenvectors().col(a):et.eigenvectors().col(a-3);
      categories[a]=static_cast<int>(engine.decideLocalizabilityLevel(params,e,a<3?rotations:normals,a%3,a<3));
    }
    const auto ms=[&](auto start){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();};
    const double x=ms(begin);
    out<<name<<",X-ICP_CORE,"<<stage<<','<<n<<',';vector(out,es.eigenvalues());out<<',';
    vector(out,er.eigenvalues());out<<',';vector(out,et.eigenvalues());out<<",1,";vector(out,categories);out<<',';
    vector(out,Eigen::Map<const Eigen::VectorXd>(er.eigenvectors().data(),9));out<<',';
    vector(out,Eigen::Map<const Eigen::VectorXd>(et.eigenvectors().data(),9));out<<','<<x<<'\n';
    const auto tick=std::chrono::steady_clock::now();
    const auto result=dcreg::DetectDegeneracy(h,dcreg::SolverParameters{});
    const double d=ms(tick);
    out<<name<<",DCReg_CORE,"<<stage<<','<<n<<',';vector(out,es.eigenvalues());out<<',';
    vector(out,result.lambda_schur_rot);out<<',';vector(out,result.lambda_schur_trans);
    out<<','<<result.factorization_ok<<",NA,";
    vector(out,Eigen::Map<const Eigen::VectorXd>(result.raw_rot_basis.data(),9));out<<',';
    vector(out,Eigen::Map<const Eigen::VectorXd>(result.raw_trans_basis.data(),9));out<<','<<d<<'\n';
  }
}
