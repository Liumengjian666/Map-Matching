#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"

namespace dog_prior_map_localization {

// NDT-adapted observability report. The input Hessian is the existing P10
// product-chart energy Hessian, ordered [map translation / 0.8 m, map-spatial
// rotation / rad]. This is an adapted detector, not the DCReg ICP solver.
struct DcregAdaptedReport {
  bool valid = false;
  bool translation_degenerate = false;
  bool rotation_degenerate = false;
  bool translation_nonconvex = false;
  bool rotation_nonconvex = false;
  double translation_ratio = 0.0;
  double rotation_ratio = 0.0;
  double translation_condition = 0.0;
  double rotation_condition = 0.0;
  Eigen::Vector3d translation_eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Matrix3d translation_axes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rotation_axes = Eigen::Matrix3d::Identity();
  std::string status = "NOT_RUN";
};

struct MatureNdtConfig {
  double translation_scale_m = 0.8;
  double degeneracy_eigen_ratio = 0.03;
  double weak_translation_step_m = 0.20;
  double weak_rotation_step_deg = 2.0;
  double score_improvement_margin = 2.747604276e-4;
  double motion_weight = 0.05;
  double motion_translation_scale_m = 2.0;
  double motion_rotation_scale_deg = 15.0;
  double suspect_fitness_threshold = 10.0;
  int maximum_quick_candidates = 4;
  int maximum_extra_aligns = 2;
};

struct MatureNdtCandidate {
  int id = -1;
  std::string kind;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double score = 0.0;
  double merit = 0.0;
  bool finite = false;
};

struct SkfGateDecision {
  std::string status = "DISABLED";
  bool direction_selected = false;
  Eigen::Matrix3d selected_rotation_axes = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d selected_translation_axes = Eigen::Matrix3d::Identity();
};

DcregAdaptedReport analyzeDcregAdapted(
    const dog_prior_map_fastlio2_frontend_exp::CoupledMatrix6& energy_hessian,
    double degeneracy_eigen_ratio);

std::vector<MatureNdtCandidate> makeWeakDirectionCandidates(
    const DcregAdaptedReport& report, const Eigen::Matrix4f& nominal,
    double translation_step_m, double rotation_step_deg);

double candidateMotionMerit(double score, std::size_t source_count,
    const Eigen::Matrix4f& candidate, const Eigen::Matrix4f& prediction,
    double nominal_energy_scale, const MatureNdtConfig& config);

void rankMatureCandidates(std::vector<MatureNdtCandidate>* candidates);

// The current P4 input is a causal relative displacement already consumed by
// the NDT initial guess, and carries no calibrated covariance. This gate
// deliberately cannot authorize an EKF measurement update in that contract.
SkfGateDecision evaluateSkfGate(bool enabled, const DcregAdaptedReport& report,
    bool valid_visual_row, bool visual_consumed_by_initial_guess,
    bool calibrated_covariance_available);

}  // namespace dog_prior_map_localization
