#include "gyro_input.hpp"
#define main historicalTestUnusedMain
#include "legacy/superloc_first_segment_initializer.cpp"
#undef main
int main(int argc,char** argv) {
  if(argc!=2)return 8;
  Eigen::Matrix3f R;Eigen::Vector3f parsed_t;
  if(!loadOfficialPrior(argv[1],R,parsed_t))return 9;
  Eigen::Matrix3f expected;
  expected<<.823800f,-.566853f,-.005503f,.565879f,.822884f,-.051418f,.033675f,.039243f,.998662f;
  const Eigen::Vector3f t(39.929523f,-176.217148f,-5.225427f);
  if((R-expected).norm()!=0||(parsed_t-t).norm()!=0)return 10;
  if(!R.allFinite()||R.determinant()<=0||(R.transpose()*R-Eigen::Matrix3f::Identity()).norm()>1e-5)return 11;
  const auto axes=properAxes();if(axes.size()!=24)return 1;
  for(size_t i=0;i<axes.size();++i) {
    if((axes[i].transpose()*axes[i]-Eigen::Matrix3f::Identity()).norm()!=0||axes[i].determinant()!=1)return 2;
    for(size_t j=0;j<i;++j)if((axes[i]-axes[j]).norm()==0)return 3;
  }
  for(const auto& A:axes){Transform seed=Transform::Identity();seed.linear()=R*A;seed.translation()=t;if(seed.translation()!=t)return 4;}
  std::vector<p32::Imu> imu(11);for(size_t i=0;i<imu.size();++i){imu[i].ns=1000000000+i*10000000;imu[i].gyro=Eigen::Vector3d(0,0,1);}
  p32::integrate(imu);
  const auto q=p32::orientation(imu,1055000000);
  if(std::abs(Eigen::AngleAxisd(q).angle()-.055)>1e-12)return 5;
  const auto zero=p32::rotatePoint(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),Eigen::Vector3d(1,2,3));
  if((zero-Eigen::Vector3d(1,2,3)).norm()>1e-12)return 6;
  bool refused=false;try{p32::orientation(imu,999999999);}catch(...){refused=true;}if(!refused)return 7;
  std::cout<<"P32_TRANSFER_MATH_PASS axes=24 unique_proper fixed_translation gyro_and_point=PASS no_extrapolation=PASS active_yaml=PASS det="<<R.determinant()<<" orthogonality="<<(R.transpose()*R-Eigen::Matrix3f::Identity()).norm()<<'\n';return 0;
}
