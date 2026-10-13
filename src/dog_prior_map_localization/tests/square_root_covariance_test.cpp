#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "dog_prior_map_localization/core/square_root_covariance.hpp"
#include "dog_prior_map_localization/core/positive_covariance_factor.hpp"
#include "dog_prior_map_localization/core/psd_covariance_damping.hpp"
#include "dog_prior_map_localization/core/selective_relative_visual_update.hpp"
#include "dog_prior_map_localization/core/state_history.hpp"
using namespace dog_prior_map_localization;
void require(bool good,const char* text) { if(!good) throw std::runtime_error(text); }
Matrix15d load(const char* path,const std::string& selected) {
  std::ifstream file(path); require(file.good(),"real fixture missing");
  Matrix15d P=Matrix15d::Zero(); std::string line; std::getline(file,line); int count=0;
  while(std::getline(file,line)) {
    std::stringstream stream(line); std::string stage,r,c,v;
    std::getline(stream,stage,','); std::getline(stream,r,','); std::getline(stream,c,','); std::getline(stream,v,',');
    if(stage==selected) { P(std::stoi(r),std::stoi(c))=std::stod(v); ++count; }
  }
  require(count==225,"fixture incomplete"); return P;
}
int main(int argc,char** argv) {
  try {
    require(argc==2,"need actual P26 before/after fixture");
    const auto real=factorPositiveCovariance(load(argv[1],"before"));
    require(real.valid,"P26 before sample is not PSD");
    require(!factorPositiveCovariance(load(argv[1],"after")).valid,"P26 true illegal write not detected");
    Matrix15d F=Matrix15d::Identity(); F.block<3,3>(0,3)=Eigen::Matrix3d::Identity()*0.005;
    F.block<3,3>(3,9)=-Eigen::Matrix3d::Identity()*0.005;
    F.block<3,3>(6,12)=-Eigen::Matrix3d::Identity()*0.005;
    F(4,6)=0.049; F(3,7)=-0.049;
    const Matrix15d noise=diagonalImuNoiseRoot(0.005,0.1,0.02,0.001,0.001);
    Matrix15d normal=Matrix15d::Identity(); normal(1,0)=0.1;
    const Matrix15d expected=(F*normal*normal.transpose()*F.transpose()+noise*noise.transpose()).eval();
    require(propagateCovarianceRoot(F,noise,&normal),"normal QR prediction failed");
    require((normal*normal.transpose()-expected).norm()<1e-12,"normal prediction differs from FPFt+Q");
    const Matrix15d no_noise=diagonalImuNoiseRoot(0.005,0,0,0,0);
    require(no_noise.isZero(0.0),"zero noise changed");
    require(propagateCovarianceRoot(F,no_noise,&normal),"PSD zero Q rejected");
    // Causal partial-interval rank-one noise, including cross-position/velocity.
    Eigen::Matrix<double,15,17> partial=Eigen::Matrix<double,15,17>::Zero();
    partial.leftCols<15>()=noise; partial(0,15)=0.0001; partial(3,15)=0.01; partial(6,16)=0.001;
    const Matrix15d partial_expected=(F*normal*normal.transpose()*F.transpose()+partial*partial.transpose()).eval();
    require(propagateCovarianceRoot(F,partial,&normal),"partial Q failed");
    require((normal*normal.transpose()-partial_expected).norm()<1e-12,"partial Q changed");
    FilterStateSnapshot reference,current; reference.square_root_authoritative=current.square_root_authoritative=true;
    reference.P.setZero(); current.P.setZero(); reference.S=Matrix15d::Identity(); current.S=normal;
    FilterStateSnapshot dense_ref=reference,dense_current=current;
    dense_ref.P=reference.covarianceView(); dense_current.P=current.covarianceView();
    dense_ref.square_root_authoritative=dense_current.square_root_authoritative=false;
    Eigen::Matrix4d relative=Eigen::Matrix4d::Identity();
    relative(0,3)=0.002;
    relative.block<3,3>(0,0)=Eigen::AngleAxisd(0.002,Eigen::Vector3d::UnitX()).toRotationMatrix();
    const auto sr=applySelectiveRelativeVisualUpdate(&current,reference,relative,Matrix6d::Identity()*0.01,
        true,Eigen::Matrix3d::Identity(),true,Eigen::Matrix3d::Identity(),true);
    const auto dense=applySelectiveRelativeVisualUpdate(&dense_current,dense_ref,relative,Matrix6d::Identity()*0.01,
        true,Eigen::Matrix3d::Identity(),true,Eigen::Matrix3d::Identity(),true);
    require(sr.accepted && dense.accepted,"SKF update not applied");
    require(validCovarianceRoot(current.S),"SKF factor invalid");
    require((current.covarianceView()-dense_current.P).norm()<1e-10,"SKF Joseph formula changed");
    require((current.p-dense_current.p).norm()<1e-12 && (current.v-dense_current.v).norm()<1e-12,"SKF correction changed");
    Matrix15d damp=current.covarianceView();
    require(applyPsdPoseCovarianceDamping(&damp,.85,1e-4,1e-5,true),"dense damping failed");
    require(dampPoseCovarianceRoot(&current.S,.85,1e-4,1e-5),"root damping failed");
    require((damp-current.covarianceView()).norm()<1e-12,"DPD changed");
    current.S=real.root; current.stamp=10; current.P.setZero();
    StateHistory history; history.insertMonotonic(current); Matrix15d direct=current.S;
    for(int i=1;i<=20;++i) {
      require(propagateCovarianceRoot(F,noise,&current.S),"real-matrix QR failed");
      current.stamp=10+0.005*i; history.insertMonotonic(current);
    }
    const Matrix15d replay_expected=current.S;
    std::size_t index=0; double age=0;
    require(history.findAtOrBefore(10,index,age) && age==0,"causal history query failed");
    current=history.at(index); require((current.S-direct).norm()==0,"history factor changed");
    history.eraseAfter(10);
    for(int i=1;i<=20;++i) {
      require(propagateCovarianceRoot(F,noise,&current.S),"replay failed");
      current.stamp=10+0.005*i; history.insertMonotonic(current);
    }
    require((current.S-replay_expected).norm()==0,"rollback/replay factor not bit-identical");
    require(current.P.isZero(0.0),"legacy P acquired authority");
    const auto high_range_skf=applySelectiveRelativeVisualUpdate(&current,reference,relative,
        Matrix6d::Identity()*0.01,false,Eigen::Matrix3d::Identity(),true,Eigen::Matrix3d::Identity(),true);
    require(high_range_skf.accepted && validCovarianceRoot(current.S),"high-range real matrix SKF failed");
    current.S(0,0)=-1; require(!validCovarianceRoot(current.S),"invalid factor accepted");
    std::cout<<"SQUARE_ROOT_COVARIANCE_PASS real_P26_fixture normal_prediction zero_Q partial_Q SKF_Joseph DPD OOSM_factor_replay invalid_factor\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
