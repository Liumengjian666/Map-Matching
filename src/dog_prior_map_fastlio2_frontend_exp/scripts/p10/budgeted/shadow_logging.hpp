#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/resource.h>

namespace p10log {
namespace paper = dog_prior_map_fastlio2_frontend_exp;
inline Eigen::Matrix4f matrix(const paper::Pose3d& pose) {
  Eigen::Matrix4f value = Eigen::Matrix4f::Identity();
  value.block<3,3>(0,0) = pose.orientation.normalized().toRotationMatrix().cast<float>();
  value.block<3,1>(0,3) = pose.position.cast<float>();
  return value;
}
template<class Derived> std::string values(const Eigen::MatrixBase<Derived>& value) {
  std::ostringstream out; out << std::setprecision(17);
  for (int r=0; r<value.rows(); ++r) for (int c=0; c<value.cols(); ++c) {
    if (r || c) out << ';'; out << value(r,c);
  }
  return out.str();
}
inline double rotation(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  Eigen::Quaterniond qa(a.block<3,3>(0,0).cast<double>()), qb(b.block<3,3>(0,0).cast<double>());
  return qa.normalized().angularDistance(qb.normalized()) * 180.0 / std::acos(-1.0);
}
inline double translation(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3,1>(0,3)-b.block<3,1>(0,3)).cast<double>().norm();
}
inline void scalar(std::ostream& out, double v) { if(std::isfinite(v)) out << v; }
inline std::ofstream output(const std::string& path) {
  std::ifstream exists(path); if(exists.good()) throw std::runtime_error("output_exists:"+path);
  std::ofstream out(path); if(!out) throw std::runtime_error("cannot_create:"+path);
  out.exceptions(std::ios::badbit|std::ios::failbit); out << std::setprecision(17); return out;
}
struct Logger {
  std::ofstream frames, candidates, models;
  explicit Logger(const std::string& dir): frames(output(dir+"/frames.csv")),
      candidates(output(dir+"/candidates.csv")), models(output(dir+"/models.csv")) {
    frames << "transaction_id,stamp_ns,source_count,source_hash,target_count,nominal_status,nominal_iterations,nominal_probability,nominal_alignment_ms,nominal_pose,prediction_pose,recommended_pose,recommended_id,weak_dimension,preview_count,expanded,full_ndt_calls,jet_calls,preview_score_calls,terminal_score_calls,nominal_energy,nominal_merit,shadow_status,model_ms,jet_ms,solve_ms,preview_ms,refinement_ms,shadow_ms,shadow_cpu_ms,frame_total_ms,frame_cpu_ms,recommended_dt_m,recommended_dr_deg,nonfinite_recommended\n";
    candidates << "transaction_id,candidate_id,stage,rank,u,previous_u,previous_v,v_predicted,v_corrected,coupling_delta,gradient_delta,initial_pose,predicted_pose,preview_pose,preview_score,preview_energy,preview_merit,prediction_dt_m,prediction_dr_deg,strong_gradient_norm,damping,condition,solve_residual,fallback,capped,finite,status,selected_for_refinement,refined_pose,refined_score,refined_iterations,refined_converged,refined_successful,refined_status,refine_dt_m,refine_dr_deg\n";
    models << "transaction_id,weak_dimension,eigenvalues,eigenvectors\n";
  }
  void write(uint64_t tx, const paper::CurrentFrameNdtResult& nominal,
      const paper::CoupledShadowResult& result, double total_ms, double cpu_ms) {
    frames << tx << ',' << nominal.stamp_ns << ',' << nominal.source_point_count << ','
      << nominal.source_cloud_hash << ',' << nominal.target_point_count << ','
      << paper::currentFrameNdtStatusName(nominal.status) << ',' << nominal.iterations << ',';
    scalar(frames,nominal.transformation_probability); frames << ','; scalar(frames,nominal.alignment_ms);
    frames << ',' << values(matrix(nominal.raw_map_T_lidar)) << ',' << values(matrix(nominal.initial_map_T_lidar))
      << ',' << values(result.recommended_pose) << ',' << result.recommended_id << ',' << result.weak_dimension
      << ',' << result.candidates.size() << ',' << result.expanded << ',' << result.complete_ndt_calls << ','
      << result.jet_calls << ',' << result.preview_score_calls << ',' << result.terminal_score_calls << ','
      << result.nominal_energy << ',' << result.nominal_merit << ',' << result.status << ',' << result.model_ms
      << ',' << result.jet_ms << ',' << result.solve_ms << ',' << result.preview_ms << ',' << result.refinement_ms
      << ',' << result.total_ms << ',' << result.cpu_ms << ',' << total_ms << ',' << cpu_ms << ','
      << translation(matrix(nominal.raw_map_T_lidar),result.recommended_pose) << ','
      << rotation(matrix(nominal.raw_map_T_lidar),result.recommended_pose) << ','
      << !result.recommended_pose.allFinite() << '\n';
    models << tx << ',' << result.weak_dimension << ',' << values(result.eigenvalues) << ',' << values(result.eigenvectors) << '\n';
    for(const auto& c:result.candidates) {
      candidates << tx << ',' << c.id << ',' << c.stage << ',' << c.rank << ',' << values(c.u) << ','
        << values(c.previous_u) << ',' << values(c.previous_v) << ',' << values(c.v_predicted) << ','
        << values(c.v_corrected) << ',' << values(c.coupling_delta) << ',' << values(c.gradient_delta) << ','
        << values(c.initial_pose) << ',' << values(c.predicted_pose) << ',' << values(c.pose) << ',';
      scalar(candidates,c.score_sum); candidates << ','; scalar(candidates,c.energy); candidates << ',';
      scalar(candidates,c.merit); candidates << ',' << c.prediction_translation_m << ',' << c.prediction_rotation_deg
        << ',' << c.strong_gradient_norm << ',' << c.damping << ',' << c.condition << ',' << c.solve_residual
        << ',' << c.fallback << ',' << c.capped << ',' << c.finite << ',' << c.status << ','
        << c.selected_for_refinement << ',' << values(c.refinement.pose) << ',';
      scalar(candidates,c.refinement.score_sum); candidates << ',' << c.refinement.iterations << ','
        << c.refinement.converged << ',' << c.refinement.successful << ',' << c.refinement.status << ',';
      if(c.selected_for_refinement) candidates << translation(c.pose,c.refinement.pose);
      candidates << ','; if(c.selected_for_refinement) candidates << rotation(c.pose,c.refinement.pose);
      candidates << '\n';
    }
  }
};
// Opt-in R3 receipts; the historical R2 logger and its column meanings stay intact.
struct EventLogger {
  std::ofstream events, costs, end;
  explicit EventLogger(const std::string& dir): events(output(dir+"/events.csv")),
      costs(output(dir+"/frame_cost.csv")), end(output(dir+"/pending_end.csv")) {
    events << "transaction_id,mode,event,innovation_trigger,innovation_translation_m,innovation_rotation_deg,recommendation_available,pending_before,pending_after,confirmation_count,pending_candidate_id,nonlocal_terminals,eligible_nonlocal_terminals,propagated_alternative,temporal_terminal,temporal_score,temporal_iterations,temporal_converged,temporal_successful,temporal_status,temporal_translation_residual_m,temporal_rotation_residual_deg,pending_pose,pending_prediction,pending_stamp_ns,pending_confirmations\n";
    costs << "transaction_id,processing_and_logging_ms,cpu_ms,logging_ms\n";
    end << "active,status,stamp_ns,confirmations,pose,prediction\n";
  }
  void write(uint64_t tx, const paper::CoupledEventResult& r, const paper::PendingCandidate& pending) {
    events << tx << ',' << r.mode << ',' << r.event << ',' << r.innovation_trigger << ','
      << r.innovation_translation_m << ',' << r.innovation_rotation_deg << ',' << r.recommendation_available
      << ',' << r.pending_before << ',' << r.pending_after << ',' << r.confirmation_count << ','
      << r.pending_candidate_id << ',' << r.nonlocal_terminals << ',' << r.eligible_nonlocal_terminals << ','
      << values(r.propagated_alternative) << ',' << values(r.temporal_terminal.pose) << ',';
    scalar(events,r.temporal_terminal.score_sum);
    events << ',' << r.temporal_terminal.iterations << ',' << r.temporal_terminal.converged << ','
      << r.temporal_terminal.successful << ',' << r.temporal_terminal.status << ','
      << r.temporal_translation_residual_m << ',' << r.temporal_rotation_residual_deg << ','
      << values(pending.pose) << ',' << values(pending.imu_prediction) << ',' << pending.stamp_ns << ','
      << pending.confirmations << '\n';
  }
  void cost(uint64_t tx, double total, double cpu, double logging) {
    costs << tx << ',' << total << ',' << cpu << ',' << logging << '\n';
  }
  void finish(const paper::PendingCandidate& pending) {
    end << pending.active << ',' << (pending.active ? "UNCONFIRMED_END_OF_SEQUENCE" : "NONE") << ','
      << pending.stamp_ns << ',' << pending.confirmations << ',' << values(pending.pose) << ','
      << values(pending.imu_prediction) << '\n';
  }
};
// R4 experiment receipt. R2/R3 column meanings are preserved above.
struct AdmissionLogger {
  std::ofstream rows;
  explicit AdmissionLogger(const std::string& dir): rows(output(dir+"/admission.csv")) {
    rows << "transaction_id,experiment_mode,event,origin_stamp_ns,confirmation_count,admission_valid,temporally_supported,admitted,admission_status,D_E,D_M,D,nominal_energy,alternative_energy,nominal_motion_cost,alternative_motion_cost,previous_nominal,previous_alternative,imu_interval,candidate_pose,admitted_pose,actual_measurement,alternative_used,update_success,correction_translation_m,correction_rotation_deg,rotation_consistent\n";
  }
  void write(uint64_t tx, const std::string& mode, const paper::CoupledEventResult& r,
      const paper::Pose3d& measurement, bool used, bool success, const paper::PoseCorrectionDelta& delta) {
    rows << tx << ',' << mode << ',' << r.event << ',' << r.origin_stamp_ns << ','
      << r.confirmation_count << ',' << r.admission_valid << ',' << r.temporally_supported << ','
      << r.admitted << ',' << r.admission_status << ',' << r.energy_difference << ','
      << r.motion_difference << ',' << r.branch_difference << ',' << r.nominal_branch_energy << ','
      << r.alternative_branch_energy << ',' << r.nominal_motion_cost << ',' << r.alternative_motion_cost << ','
      << values(r.previous_nominal) << ',' << values(r.previous_alternative) << ',' << values(r.imu_interval)
      << ',' << values(r.candidate_pose) << ',' << values(r.admitted_pose) << ',' << values(matrix(measurement))
      << ',' << used << ',' << success << ',' << delta.position.norm() << ','
      << delta.rotation.norm()*180/std::acos(-1.0) << ',' << r.rotation_consistent << '\n';
  }
};
inline paper::CoupledNdtConfig config(const std::string& method, bool fixed) {
  paper::CoupledNdtConfig c; c.fixed_maximum_budget=fixed;
  if(method=="A") c.method=paper::CoupledMethod::WEAK_ONLY;
  else if(method=="B") c.method=paper::CoupledMethod::R1_PREDICTOR;
  else if(method!="C") throw std::runtime_error("invalid_method");
  return c;
}
inline void resourceReceipt(const std::string& dir) {
  rusage r; if(getrusage(RUSAGE_SELF,&r)!=0) throw std::runtime_error("resource_query_failed");
  auto out=output(dir+"/resources.csv");
  out << "peak_rss_kib,user_s,system_s\n" << r.ru_maxrss << ','
    << r.ru_utime.tv_sec+r.ru_utime.tv_usec/1e6 << ',' << r.ru_stime.tv_sec+r.ru_stime.tv_usec/1e6 << '\n';
}
} // namespace p10log
