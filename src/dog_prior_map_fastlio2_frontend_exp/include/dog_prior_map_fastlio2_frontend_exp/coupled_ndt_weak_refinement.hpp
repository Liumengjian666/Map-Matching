#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"

namespace dog_prior_map_fastlio2_frontend_exp {
struct WeakCoupledConfig {
  bool coupled = true;
  double translation_limit_m = .15, rotation_limit_deg = 2.0;
  double strong_step_cap = .10, near_quality_fraction = .05;
  double minimum_rho = 1e-4, objective_margin = 1e-8;
  double maximum_condition = 1e8, solve_residual_limit = 1e-6;
};
struct WeakCoupledResult {
  Eigen::Matrix4f nominal = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  Eigen::Matrix4d anchor_prediction = Eigen::Matrix4d::Identity();
  Eigen::Matrix4f weak_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f coupled_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f candidate = Eigen::Matrix4f::Identity();
  CoupledVector6 eigenvalues = CoupledVector6::Zero();
  CoupledMatrix6 eigenvectors = CoupledMatrix6::Identity();
  CoupledVector6 anchor_eta = CoupledVector6::Zero();
  CoupledVector6 weak_eta = CoupledVector6::Zero(), strong_eta = CoupledVector6::Zero();
  CoupledVector6 candidate_eta = CoupledVector6::Zero();
  Eigen::VectorXd u_anchor, delta_u, delta_v;
  bool triggered = false, anchor_valid = false, attempted = false;
  bool weak_solve_valid = false, weak_quality_valid = false;
  bool strong_solve_valid = false, strong_selected = false;
  bool half_step = false, recommended = false;
  int weak_dimension = 0, jet_calls = 0, value_calls = 0;
  double innovation_translation_m = 0, innovation_rotation_deg = 0, anchor_age_s = 0;
  double rho = 0, weak_damping = 0, strong_damping = 0, schur_condition = 0;
  double strong_condition = 0, nominal_strong_condition = 0, weak_solve_residual = 0, strong_solve_residual = 0;
  double nominal_cross_norm = 0, displaced_cross_norm = 0;
  double nominal_strong_gradient = 0, displaced_strong_gradient = 0;
  double nominal_score = std::numeric_limits<double>::quiet_NaN();
  double weak_score = std::numeric_limits<double>::quiet_NaN();
  double full_weak_score = std::numeric_limits<double>::quiet_NaN();
  double coupled_score = std::numeric_limits<double>::quiet_NaN();
  double displaced_score = std::numeric_limits<double>::quiet_NaN(), displaced_score_gap = 0;
  double candidate_score = std::numeric_limits<double>::quiet_NaN();
  double nominal_objective = 0, candidate_objective = 0;
  double jet_ms = 0, value_ms = 0, solve_ms = 0, total_ms = 0, cpu_ms = 0;
  std::string status = "NOT_RUN", strong_status = "NOT_RUN";
};
WeakCoupledResult runWeakCoupledRefinement(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, uint64_t stamp_ns, std::size_t source_count,
    bool nominal_effective, const CoupledAnchorState&, const CoupledNdtBackend&,
    const WeakCoupledConfig&);
}  // namespace dog_prior_map_fastlio2_frontend_exp
