#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_directional_covariance.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <chrono>
#include <algorithm>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
bool poseValid(const Pose3d& p) {
  return p.position.allFinite() && p.orientation.coeffs().allFinite() &&
      std::abs(p.orientation.norm()-1.)<=1e-8;
}
}
DirectionalCovarianceResult buildDirectionalCovariance(
    const CoupledVector6& l, const CoupledMatrix6& Q, int k,
    const CoupledMatrix6& R0, const Pose3d& nominal, const Pose3d& lidar, const Pose3d& predicted,
    const Pose3d& extrinsic) {
  const auto began=std::chrono::steady_clock::now();
  DirectionalCovarianceResult out;out.baseline=R0;out.covariance=R0;
  auto finish=[&](const std::string& status) {
    out.status=status;out.total_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-began).count();return out;
  };
  if((k!=1 && k!=2) || !l.allFinite() || l.minCoeff()<=0 ||
      !Q.allFinite() || (Q.transpose()*Q-CoupledMatrix6::Identity()).norm()>1e-8 ||
      !R0.allFinite() || (R0-R0.transpose()).cwiseAbs().maxCoeff()>1e-10 ||
      !poseValid(nominal) || !poseValid(lidar) || !poseValid(predicted) || !poseValid(extrinsic))
    return finish("INVALID_INPUT_NOMINAL");
  for(int i=1;i<6;++i) if(l(i)<l(i-1)) return finish("UNSORTED_CURVATURE_NOMINAL");
  Eigen::LLT<CoupledMatrix6> baseline_factor(R0);
  if(baseline_factor.info()!=Eigen::Success) return finish("BASELINE_NOT_SPD_NOMINAL");
  const Eigen::Matrix3d measured_R=lidar.orientation.toRotationMatrix()*
      extrinsic.orientation.toRotationMatrix().transpose();
  CoupledMatrix6 rotation_first;
  std::string reason;
  if(!normalizedRegistrationToPoseResidualJacobian(predicted.orientation.toRotationMatrix(),
      measured_R,extrinsic.position,.8,&rotation_first,&reason))
    return finish("RESIDUAL_JACOBIAN_INVALID_NOMINAL");
  out.jacobian.leftCols<3>()=rotation_first.rightCols<3>();
  out.jacobian.rightCols<3>()=rotation_first.leftCols<3>();
  // Q lives in the ORIGINAL nominal product chart, not a recentered candidate
  // chart. Exp(theta+dtheta) requires the SO(3) left-Jacobian transport.
  const Eigen::Vector3d theta=so3Log(lidar.orientation.toRotationMatrix()*
      nominal.orientation.toRotationMatrix().transpose());
  Eigen::Matrix3d left_inverse;
  if(!so3LeftJacobianInverse(theta,&left_inverse,&reason))
    return finish("CHART_TRANSPORT_INVALID_NOMINAL");
  const Eigen::Matrix3d left_jacobian=left_inverse.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
  out.jacobian.rightCols<3>()=(out.jacobian.rightCols<3>()*left_jacobian).eval();
  Eigen::ColPivHouseholderQR<CoupledMatrix6> solver(out.jacobian);
  if(solver.rank()!=6) return finish("JACOBIAN_RANK_FAILED_NOMINAL");
  const CoupledMatrix6 left=solver.solve(R0);
  out.chart_baseline=solver.solve(left.transpose()).transpose();
  out.chart_baseline=(.5*(out.chart_baseline+out.chart_baseline.transpose())).eval();
  out.solve_residual=(out.jacobian*out.chart_baseline*out.jacobian.transpose()-R0).norm()/R0.norm();
  if(!out.chart_baseline.allFinite() || !std::isfinite(out.solve_residual) || out.solve_residual>1e-10)
    return finish("CHART_SOLVE_FAILED_NOMINAL");
  out.epsilon=1e-6*l(k);out.multipliers.resize(k);out.weak_variances.resize(k);
  for(int i=0;i<k;++i) {
    const double m=std::max(1.,std::min(20.,l(k)/std::max(l(i),out.epsilon)));
    const double variance=Q.col(i).dot(out.chart_baseline*Q.col(i));
    if(!std::isfinite(variance) || variance<=0) return finish("WEAK_VARIANCE_INVALID_NOMINAL");
    out.multipliers(i)=m;out.weak_variances(i)=variance;
    const Eigen::Matrix<double,6,1> direction=out.jacobian*Q.col(i);
    out.covariance+=(m-1)*variance*(direction*direction.transpose());
  }
  out.covariance=(.5*(out.covariance+out.covariance.transpose())).eval();
  if(!out.covariance.allFinite()) return finish("COVARIANCE_NONFINITE_NOMINAL");
  Eigen::LLT<CoupledMatrix6> factor(out.covariance);
  Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig(out.covariance);
  if(factor.info()!=Eigen::Success || eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff()<=0)
    return finish("COVARIANCE_NOT_SPD_NOMINAL");
  out.minimum_eigenvalue=eig.eigenvalues().minCoeff();out.valid=true;
  const CoupledMatrix6 emitted_left=solver.solve(out.covariance);
  const CoupledMatrix6 emitted_chart=solver.solve(emitted_left.transpose()).transpose();
  out.strong_block_difference=(Q.rightCols(6-k).transpose()*
      (emitted_chart-out.chart_baseline)*Q.rightCols(6-k)).norm();
  return finish("DIRECTIONAL_HEURISTIC_SPD");
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
