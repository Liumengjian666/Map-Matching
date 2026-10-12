#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "dog_prior_map_localization/core/positive_covariance_factor.hpp"
#include "dog_prior_map_localization/core/psd_covariance_damping.hpp"
#include "dog_prior_map_localization/core/selective_relative_visual_update.hpp"

using namespace dog_prior_map_localization;
void require(bool good,const char* message) { if(!good) throw std::runtime_error(message); }
Matrix15d loadStage(const char* path,const std::string& selected)
{
  std::ifstream file(path);require(file.good(),"fixture missing");
  Matrix15d P=Matrix15d::Zero();std::string line;std::getline(file,line);int count=0;
  while(std::getline(file,line)) {
    std::stringstream stream(line);std::string stage,r,c,value;
    std::getline(stream,stage,',');std::getline(stream,r,',');std::getline(stream,c,',');std::getline(stream,value,',');
    if(stage==selected) {P(std::stoi(r),std::stoi(c))=std::stod(value);++count;}
  }
  require(count==225,"fixture incomplete");return P;
}
int main(int argc,char** argv)
{
  try {
    require(argc==4,"need eigensolver, double-LLT and actual-indefinite fixtures");
    require(std::numeric_limits<long double>::digits>std::numeric_limits<double>::digits,"extended precision unavailable");
    const Matrix15d before=loadStage(argv[1],"before"),after=loadStage(argv[1],"after");
    const Matrix15d llt_failure=loadStage(argv[2],"after");
    Eigen::LLT<Matrix15d> double_factor(llt_failure);
    require(double_factor.info()!=Eigen::Success,"double LLT failure no longer reproduced");
    require(factorPositiveCovariance(llt_failure).valid,"positive sample rejected by extended-precision factor");
    const Matrix15d actual_indefinite=loadStage(argv[3],"after");
    require(!factorPositiveCovariance(actual_indefinite).valid,"actual indefinite high-range sample incorrectly accepted");
    const Matrix15d symmetric=(0.5*(after+after.transpose())).eval();
    Eigen::SelfAdjointEigenSolver<Matrix15d> raw(symmetric);
    require(raw.eigenvalues().minCoeff() < -1e-10,"old PSD predicate no longer reproduces");
    require(factorPositiveCovariance(before).valid,"before factor failed");
    const auto certificate=factorPositiveCovariance(after);
    require(certificate.valid,"actual PSD sample rejected");
    Matrix15d legacy=after, stable=after;
    require(!applyPsdPoseCovarianceDamping(&legacy,0.85,1e-4,1e-5),"legacy rejection not reproduced");
    require(applyPsdPoseCovarianceDamping(&stable,0.85,1e-4,1e-5,true),"stable DPD failed");
    require(factorPositiveCovariance(stable).valid,"stable DPD output invalid");
    Matrix15d indefinite=Matrix15d::Identity(); indefinite(0,3)=indefinite(3,0)=1.5;
    require(!factorPositiveCovariance(indefinite).valid,"indefinite input wrongly accepted");
    require(!applyPsdPoseCovarianceDamping(&indefinite,0.85,1e-4,1e-5,true),"invalid input repaired instead of rejected");
    FilterStateSnapshot ref,current; ref.P=Matrix15d::Identity();current.P=stable;
    Eigen::Matrix4d relative=Eigen::Matrix4d::Identity();
    relative.block<3,3>(0,0)=Eigen::AngleAxisd(0.002,Eigen::Vector3d::UnitX()).toRotationMatrix();
    const auto update=applySelectiveRelativeVisualUpdate(&current,ref,relative,
        Matrix6d::Identity()*0.01,false,Eigen::Matrix3d::Identity(),true,Eigen::Matrix3d::Identity(),true);
    require(update.accepted,"stable CI update failed");
    require(factorPositiveCovariance(current.P).valid,"stable CI output invalid");
    std::cout<<"COVARIANCE_STABILITY_REGRESSION_PASS raw_eigen="<<raw.eigenvalues().minCoeff()
             <<" certified_lower_bound="<<certificate.minimum_eigenvalue_lower_bound<<"\n";
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
