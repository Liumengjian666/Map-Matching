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
