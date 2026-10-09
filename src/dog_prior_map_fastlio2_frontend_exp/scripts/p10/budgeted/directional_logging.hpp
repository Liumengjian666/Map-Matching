#pragma once
#include "shadow_logging.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_directional_covariance.hpp"
namespace p10log {
struct DirectionalLogger {
  static Eigen::Matrix4d precise(const paper::Pose3d& p) {
    Eigen::Matrix4d T=Eigen::Matrix4d::Identity();T.block<3,3>(0,0)=p.orientation.toRotationMatrix();
    T.block<3,1>(0,3)=p.position;return T;
  }
  std::ofstream rows;
  explicit DirectionalLogger(const std::string& dir):rows(output(dir+"/directional_covariance.csv")) {
    rows<<"transaction_id,mode,attempted,valid,status,weak_dimension,epsilon,multipliers,weak_variances,jacobian,baseline_covariance,chart_baseline,covariance,minimum_eigenvalue,solve_residual,strong_block_difference,covariance_ms,alternative_used,update_success,correction_t_m,correction_r_deg,weak_pose,candidate_pose,weak_score,candidate_score,strong_selected,predicted_imu_pose,candidate_imu_pose,weak_paired_covariance,weak_paired_valid,actual_imu_pose,consumed_covariance\n";
  }
  void write(uint64_t tx,const std::string& mode,const paper::WeakCoupledResult& weak,
      const paper::DirectionalCovarianceResult& r,const paper::DirectionalCovarianceResult& paired,
      const paper::Pose3d& predicted,const paper::Pose3d& measured,
      const paper::Pose3d& actual,
      bool used,bool success,const paper::PoseCorrectionDelta& delta) {
    rows<<tx<<','<<mode<<','<<weak.recommended<<','<<r.valid<<','<<r.status<<','<<weak.weak_dimension
      <<','<<r.epsilon<<','<<values(r.multipliers)<<','<<values(r.weak_variances)<<','<<values(r.jacobian)
      <<','<<values(r.baseline)<<','<<values(r.chart_baseline)<<','<<values(r.covariance)
      <<','<<r.minimum_eigenvalue<<','<<r.solve_residual<<','<<r.strong_block_difference<<','<<r.total_ms+paired.total_ms
      <<','<<used<<','<<success<<','<<delta.position.norm()<<','<<delta.rotation.norm()*180/std::acos(-1.)
      <<','<<values(weak.weak_pose)<<','<<values(weak.candidate)<<',';scalar(rows,weak.weak_score);
    rows<<',';scalar(rows,weak.candidate_score);
    rows<<','<<weak.strong_selected<<','<<values(precise(predicted))<<','<<values(precise(measured))
      <<','<<values(paired.covariance)<<','<<paired.valid<<','<<values(precise(actual))
      <<','<<values(used ? r.covariance:r.baseline)<<'\n';
  }
};
}  // namespace p10log
