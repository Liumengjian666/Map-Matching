#pragma once

#include <Eigen/Core>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {
using CoupledVector6 = Eigen::Matrix<double, 6, 1>;
using CoupledMatrix6 = Eigen::Matrix<double, 6, 6>;

enum class CoupledMethod { WEAK_ONLY, R1_PREDICTOR, R2_RESIDUAL };
struct CoupledNdtConfig {
  CoupledMethod method = CoupledMethod::R2_RESIDUAL;
  int initial_candidates = 8, maximum_candidates = 16, maximum_extra_aligns = 2;
  // Fixed maximum budget only for the two-frame controlled ablation.
  bool fixed_maximum_budget = false;
  double translation_scale_m = .8, strong_step_cap = .10;
  double weak_translation_bound_m = 2.0, weak_rotation_bound_deg = 15.0;
  double separation_m = .2, separation_deg = 2.0;
  double motion_translation_scale_m = 2.0, motion_rotation_scale_deg = 15.0;
  double motion_weight = .05, near_quality_fraction = .05;
  double raw_score_tolerance = 2.747604276e-4;
};

struct CoupledNativeJet {
  double score_sum = std::numeric_limits<double>::quiet_NaN();
  CoupledVector6 score_gradient = CoupledVector6::Zero();
  CoupledMatrix6 score_hessian = CoupledMatrix6::Zero();
  bool valid = false;
};
struct CoupledRefinement {
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double score_sum = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  bool converged = false, successful = false;
  std::string status = "NOT_RUN";
};
// The owner supplies its existing NDT/source/target. No PCL/map object is
// created, copied or loaded by this algorithm module.
struct CoupledNdtBackend {
  std::function<CoupledNativeJet(const Eigen::Matrix4f&, const CoupledVector6&)> jet;
  std::function<double(const Eigen::Matrix4f&)> score;
  std::function<CoupledRefinement(const Eigen::Matrix4f&)> refine;
};
struct CoupledCandidate {
  int id = -1, stage = 1, rank = -1;
  Eigen::VectorXd u, previous_u, previous_v, v_predicted, v_corrected;
  Eigen::VectorXd coupling_delta, gradient_delta;
  Eigen::Matrix4f initial_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f predicted_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  CoupledRefinement refinement;
  double score_sum = std::numeric_limits<double>::quiet_NaN();
  double energy = std::numeric_limits<double>::quiet_NaN(), merit = std::numeric_limits<double>::quiet_NaN();
  double prediction_translation_m = 0, prediction_rotation_deg = 0;
  double strong_gradient_norm = 0, damping = 0, condition = 0, solve_residual = 0;
  bool finite = false, fallback = false, capped = false, selected_for_refinement = false;
  std::string status = "NOT_RUN";
};
struct CoupledShadowResult {
  Eigen::Matrix4f nominal_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prediction_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f recommended_pose = Eigen::Matrix4f::Identity();
  CoupledVector6 eigenvalues = CoupledVector6::Zero();
  CoupledMatrix6 eigenvectors = CoupledMatrix6::Identity();
  std::vector<CoupledCandidate> candidates;
  int weak_dimension = 0, recommended_id = -1, complete_ndt_calls = 1;
  int jet_calls = 0, preview_score_calls = 0, terminal_score_calls = 0;
  bool expanded = false;
  double nominal_energy = 0, nominal_merit = 0;
  double model_ms = 0, jet_ms = 0, solve_ms = 0, preview_ms = 0, refinement_ms = 0;
  double total_ms = 0, cpu_ms = 0;
  std::string status = "NOT_RUN";
};

CoupledShadowResult runCoupledNdtShadow(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, std::size_t source_count,
    const CoupledNdtBackend& backend, const CoupledNdtConfig& config);
bool coupledNdtSelfTest();
}  // namespace dog_prior_map_fastlio2_frontend_exp
