// Math-only matrix carrier extraction of P9 mapChartDisplacement(). No PCL/NDT.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

extern "C" void p9_r2b_displacement(const double* from16,const double* to16,double* output) {
  Eigen::Matrix4f from,to;
  for(int i=0;i<16;++i) {from(i/4,i%4)=static_cast<float>(from16[i]);to(i/4,i%4)=static_cast<float>(to16[i]);}
  Eigen::Matrix<double,6,1> eta;
  eta.head<3>()=(to.block<3,1>(0,3)-from.block<3,1>(0,3)).cast<double>()/0.8;
  const Eigen::Quaterniond q_from(from.block<3,3>(0,0).cast<double>());
  const Eigen::Quaterniond q_to(to.block<3,3>(0,0).cast<double>());
  Eigen::Quaterniond delta=q_to*q_from.conjugate();delta.normalize();
  if(delta.w()<0.0)delta.coeffs()*=-1.0;
  const double norm=delta.vec().norm(),angle=2.0*std::atan2(norm,delta.w());
  eta.tail<3>().setZero();if(norm>=1e-12)eta.tail<3>()=angle*delta.vec()/norm;
  for(int i=0;i<6;++i)output[i]=eta(i);
}

#ifdef P9_R2B_CHART_TEST_MAIN
#include <iostream>
int main() {
  Eigen::Matrix<double,4,4,Eigen::RowMajor> a=Eigen::Matrix4d::Identity(),b=a;
  a.block<3,3>(0,0)=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  b=a;double out[6];p9_r2b_displacement(a.data(),b.data(),out);
  if(Eigen::Map<Eigen::Matrix<double,6,1>>(out).norm()>1e-12)return 1;
  b(0,3)=.8;
  b.block<3,3>(0,0)=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ()).toRotationMatrix()*a.block<3,3>(0,0);
  p9_r2b_displacement(a.data(),b.data(),out);
  if(std::abs(out[0]-1.)>2e-7||std::abs(out[5]-.2)>2e-7||
     std::abs(out[3])>2e-7||std::abs(out[4])>2e-7)return 1;
  std::cout<<"P9_R2B_MATRIX_PRODUCT_CHART_SELF_TEST=PASS\n";
}
#endif
