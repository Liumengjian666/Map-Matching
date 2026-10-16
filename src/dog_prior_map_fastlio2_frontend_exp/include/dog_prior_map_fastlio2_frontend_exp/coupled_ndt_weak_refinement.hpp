#pragma once
#include <cmath>
#include <limits>
#include <Eigen/Geometry>
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
struct WeakCoupledInnovation {
  double translation_m = std::numeric_limits<double>::quiet_NaN();
  double rotation_deg = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool triggered = false;
};
inline bool weakCoupledInnovationTriggered(double translation_m, double rotation_deg) {
  return translation_m > 0.12 || rotation_deg > 3.0;
}

// Shared pure gate for production Shadow and CONTROL Record-Only. Keep the
// frozen strict comparisons in one place so observation cannot drift from the
// refinement path. This performs no NDT, score, jet, or candidate work.
inline WeakCoupledInnovation evaluateWeakCoupledInnovation(
    const Eigen::Matrix4f& nominal, const Eigen::Matrix4f& prediction) {
  WeakCoupledInnovation out;
  if (!nominal.allFinite() || !prediction.allFinite()) return out;
  const Eigen::Quaterniond nominal_q(nominal.block<3,3>(0,0).cast<double>());
  const Eigen::Quaterniond prediction_q(prediction.block<3,3>(0,0).cast<double>());
  if (!nominal_q.coeffs().allFinite() || !prediction_q.coeffs().allFinite() ||
      nominal_q.norm() < 1e-12 || prediction_q.norm() < 1e-12) return out;
  out.translation_m = (nominal.block<3,1>(0,3).cast<double>() -
                       prediction.block<3,1>(0,3).cast<double>()).norm();
  out.rotation_deg = nominal_q.normalized().angularDistance(prediction_q.normalized()) *
                     180.0 / std::acos(-1.0);
  out.valid = std::isfinite(out.translation_m) && std::isfinite(out.rotation_deg);
  if (out.valid)
    out.triggered = weakCoupledInnovationTriggered(out.translation_m, out.rotation_deg);
  return out;
}

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
// R6 local-feedback lifecycle: keep an existing inertial reference until its
// original expiry. This never copies the local corrected pose into a live anchor.
void settleWeakRefinementAnchor(CoupledAnchorState*, uint64_t stamp_ns,
    const Eigen::Matrix4d& corrected_lidar, bool ordinary_stable_frame);
}  // namespace dog_prior_map_fastlio2_frontend_exp
