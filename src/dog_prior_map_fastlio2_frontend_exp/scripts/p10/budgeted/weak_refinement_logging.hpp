#pragma once
#include "shadow_logging.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp"
namespace p10log {
struct WeakRefinementLogger {
  std::ofstream rows;
  explicit WeakRefinementLogger(const std::string& dir):rows(output(dir+"/weak_refinement.csv")) {
    rows<<"transaction_id,experiment_mode,status,triggered,innovation_t_m,innovation_r_deg,anchor_valid,anchor_age_s,anchor_origin_stamp_ns,anchor_origin,anchor_prediction,anchor_after_valid,anchor_after_status,anchor_after_origin_stamp_ns,attempted,weak_solve_valid,weak_quality_valid,strong_solve_valid,strong_selected,half_step,recommended,weak_dimension,eigenvalues,eigenvectors,anchor_eta,u_anchor,delta_u,delta_v,weak_eta,strong_eta,candidate_eta,nominal_pose,prediction_pose,weak_pose,coupled_pose,candidate_pose,nominal_score,full_weak_score,weak_score,displaced_score,displaced_score_gap,coupled_score,candidate_score,rho,weak_damping,strong_damping,schur_condition,nominal_strong_condition,strong_condition,weak_solve_residual,strong_solve_residual,nominal_cross_norm,displaced_cross_norm,nominal_strong_gradient,displaced_strong_gradient,nominal_objective,candidate_objective,strong_status,jet_calls,value_calls,extra_align_calls,jet_ms,value_ms,solve_ms,total_ms,cpu_ms,actual_measurement,alternative_used,update_success,correction_t_m,correction_r_deg,anchor_after_origin,anchor_after_prediction\n";
  }
  void write(uint64_t tx,const std::string& mode,const paper::WeakCoupledResult& r,
      const paper::CoupledAnchorState& before,const paper::CoupledAnchorState& after,
      const paper::Pose3d& actual,bool used,bool success,const paper::PoseCorrectionDelta& delta) {
    rows<<tx<<','<<mode<<','<<r.status<<','<<r.triggered<<','<<r.innovation_translation_m<<','<<r.innovation_rotation_deg
      <<','<<r.anchor_valid<<','<<r.anchor_age_s<<','<<before.origin_stamp_ns<<','<<values(before.origin)<<','<<values(r.anchor_prediction)
      <<','<<after.valid<<','<<after.status<<','<<after.origin_stamp_ns<<','<<r.attempted<<','<<r.weak_solve_valid<<','<<r.weak_quality_valid
      <<','<<r.strong_solve_valid<<','<<r.strong_selected<<','<<r.half_step<<','<<r.recommended<<','<<r.weak_dimension
      <<','<<values(r.eigenvalues)<<','<<values(r.eigenvectors)<<','<<values(r.anchor_eta)<<','<<values(r.u_anchor)
      <<','<<values(r.delta_u)<<','<<values(r.delta_v)<<','<<values(r.weak_eta)<<','<<values(r.strong_eta)<<','<<values(r.candidate_eta)
      <<','<<values(r.nominal)<<','<<values(r.prediction)<<','<<values(r.weak_pose)<<','<<values(r.coupled_pose)<<','<<values(r.candidate);
    for(double value:{r.nominal_score,r.full_weak_score,r.weak_score,r.displaced_score,r.displaced_score_gap,r.coupled_score,r.candidate_score}) {
      rows<<',';scalar(rows,value);
    }
    rows<<','<<r.rho<<','<<r.weak_damping<<','<<r.strong_damping<<','<<r.schur_condition<<','<<r.nominal_strong_condition
      <<','<<r.strong_condition<<','<<r.weak_solve_residual<<','<<r.strong_solve_residual<<','<<r.nominal_cross_norm<<','<<r.displaced_cross_norm
      <<','<<r.nominal_strong_gradient<<','<<r.displaced_strong_gradient<<','<<r.nominal_objective<<','<<r.candidate_objective
      <<','<<r.strong_status<<','<<r.jet_calls<<','<<r.value_calls<<",0,"<<r.jet_ms<<','<<r.value_ms<<','<<r.solve_ms
      <<','<<r.total_ms<<','<<r.cpu_ms<<','<<values(matrix(actual))<<','<<used<<','<<success
      <<','<<delta.position.norm()<<','<<delta.rotation.norm()*180/std::acos(-1.)
      <<','<<values(after.origin)<<','<<values(after.prediction)<<'\n';
  }
};
}  // namespace p10log
