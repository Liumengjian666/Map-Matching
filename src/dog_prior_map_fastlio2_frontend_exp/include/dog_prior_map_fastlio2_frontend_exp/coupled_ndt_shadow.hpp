#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
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

// R3 scheduling is opt-in; R2 search and the production state update stay intact.
struct CoupledEventConfig {
  CoupledNdtConfig search;
  double trigger_translation_m = .12, trigger_rotation_deg = 3.0;
  double confirmation_translation_m = .2, confirmation_rotation_deg = 2.0;
  double maximum_prediction_gap_s = .25;
  double local_rotation_tolerance_deg = 1e-6;
  int required_confirmations = 2;
  // Explicit R4 experiment only; legacy R3/production defaults stay unchanged.
  bool branch_admission = false;
  double branch_tie_tolerance = 1e-6;
  bool branch_rotation_guard = false;
};
struct PendingCandidate {
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f imu_prediction = Eigen::Matrix4f::Identity();
  uint64_t stamp_ns = 0;
  int confirmations = 0;
  bool active = false;
  Eigen::Matrix4f created_nominal = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f created_alternative = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f previous_nominal = Eigen::Matrix4f::Identity();
  uint64_t origin_stamp_ns = 0;
  double energy_difference = 0, motion_difference = 0;
  bool admission_valid = false;
  bool rotation_consistent = true;
};
struct CoupledEventResult {
  CoupledShadowResult shadow;
  double innovation_translation_m = 0, innovation_rotation_deg = 0;
  bool innovation_trigger = false, pending_before = false, pending_after = false;
  bool recommendation_available = false;
  int confirmation_count = 0, pending_candidate_id = -1;
  int nonlocal_terminals = 0, eligible_nonlocal_terminals = 0;
  Eigen::Matrix4f propagated_alternative = Eigen::Matrix4f::Identity();
  CoupledRefinement temporal_terminal;
  double temporal_translation_residual_m = 0, temporal_rotation_residual_deg = 0;
  std::string mode = "INVALID", event = "NOT_RUN";
  bool temporally_supported = false, admitted = false;
  bool admission_valid = false;
  bool rotation_consistent = true;
  uint64_t origin_stamp_ns = 0;
  Eigen::Matrix4f candidate_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f admitted_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f previous_nominal = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f previous_alternative = Eigen::Matrix4f::Identity();
  Eigen::Matrix4d imu_interval = Eigen::Matrix4d::Identity();
  double energy_difference = 0, motion_difference = 0, branch_difference = 0;
  double nominal_motion_cost = 0, alternative_motion_cost = 0;
  double nominal_branch_energy = 0, alternative_branch_energy = 0;
  std::string admission_status = "DISABLED";
};
CoupledEventResult runEventCoupledNdtShadow(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, uint64_t stamp_ns, std::size_t source_count,
    bool nominal_effective, const CoupledNdtBackend& backend,
    const CoupledEventConfig& config, PendingCandidate* pending,
    const Eigen::Matrix4d* causal_imu_interval = nullptr);

// Reuses the existing SO(3) geometry for a true SE(3) logarithmic residual.
bool coupledBranchMotionCost(const Eigen::Matrix4f& previous,
    const Eigen::Matrix4f& current, const Eigen::Matrix4d& imu_interval,
    const CoupledNdtConfig& config, double* cost);
}  // namespace dog_prior_map_fastlio2_frontend_exp
