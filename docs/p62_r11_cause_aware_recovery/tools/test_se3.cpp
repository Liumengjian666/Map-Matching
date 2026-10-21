#include "se3.hpp"
#include <iostream>
using namespace cause;
int main(){try{
  Vector6 xi=Vector6::Zero();require((expSE3(xi)-Pose::Identity()).norm()==0,"zero exp");
  require(logSE3(Pose::Identity()).norm()==0,"zero log");
  for(double a:{1e-12,1e-6,.1,1.2,PI-1e-8,PI}){
    xi<<.7,-.2,.4,a*.3,a*.4,a*std::sqrt(.75);
    Pose t=expSE3(xi);require((expSE3(logSE3(t))-t).norm()<1e-9,"Exp Log roundtrip");
  }
  xi<<1,0,0,0,0,PI/2;auto coupled=expSE3(xi);
  require(std::abs(coupled(0,3)-2/PI)<1e-12&&std::abs(coupled(1,3)-2/PI)<1e-12,"full SE3 translation Jacobian");
  Pose prediction=Pose::Identity();prediction.block<3,3>(0,0)=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitY()).toRotationMatrix();prediction(0,3)=3;
  for(int i=1;i<=200;++i){
    xi<<std::sin(i),std::cos(i),.1*i,std::sin(2*i),.4*std::cos(i),.1;
    Pose nominal=prediction*expSE3(xi);auto s=disagreement(prediction,nominal);require(s.valid,"valid direction");
    for(const auto& seed:s.poses)require(translation(seed,prediction)<=.3&&rotation(seed,prediction)<=3,"actual physical bounds");
    const Pose plus=prediction.inverse()*s.poses[0],minus=prediction.inverse()*s.poses[1];
    require((plus*minus-Pose::Identity()).norm()<1e-12,"opposite right exponentials");
    Pose map_change=expSE3(.2*xi);auto transformed=disagreement(map_change*prediction,map_change*nominal);
    require((transformed.poses[0]-map_change*s.poses[0]).norm()<1e-11,"right tangent map-frame equivariance");
  }
  auto zero=disagreement(prediction,prediction);require(!zero.valid,"zero residual must reject");
  for(double world_x:{0.,20.,-30.,1000.}){
    Pose p=Pose::Identity();p(0,3)=world_x;Pose n=p;n(0,3)+=1;
    auto s=disagreement(p,n);require(s.valid,"pure translation valid");
    for(const auto& seed:s.poses)require(translation(seed,p)<=.30&&rotation(seed,p)<=3.,"world-coordinate translation regression");
    n=p;n.block<3,3>(0,0)=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitX()).toRotationMatrix();
    s=disagreement(p,n);require(s.valid,"pure rotation valid");
    for(const auto& seed:s.poses)require(translation(seed,p)<=.30&&rotation(seed,p)<=3.,"pure rotation limit");
  }
  Pose bad=prediction;bad(0,0)=NAN;require(!disagreement(prediction,bad).valid,"NaN rejects");
  bad=prediction;bad(0,0)=4;require(!disagreement(prediction,bad).valid,"invalid SO3 rejects");
  Pose rounded_p=Pose::Identity();rounded_p.block<3,3>(0,0)=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<float>().cast<double>();
  for(const Vector6& delta:std::vector<Vector6>{(Vector6()<<1,0,0,0,0,0).finished(),(Vector6()<<0,0,0,0,0,.2).finished()}){
    auto s=disagreement(rounded_p,rounded_p*expSE3(delta));require(s.valid,"rounded prediction valid");
    for(const auto& seed:s.poses)require(translation(seed,rounded_p)<=.30&&rotation(seed,rounded_p)<=3.,"native rounded prediction limits");
  }
  xi<<.2,.1,-.1,.02,.03,.04;Pose rounded=expSE3(xi).cast<float>().cast<double>();
  require((expSE3(logSE3(rounded))-rounded).norm()<1e-6,"native float rounding");
  std::cout<<"R11_SE3_TESTS_PASS new_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
