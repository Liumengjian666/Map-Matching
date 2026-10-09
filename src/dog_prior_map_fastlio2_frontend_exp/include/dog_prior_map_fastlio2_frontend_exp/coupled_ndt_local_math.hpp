#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"

namespace dog_prior_map_fastlio2_frontend_exp {
// Small research interfaces to the EXISTING R2 pullback and strong-step code.
// The old search and native objective are not replaced or reimplemented.
struct CoupledLocalJet {
  CoupledVector6 gradient = CoupledVector6::Zero();
  CoupledMatrix6 H = CoupledMatrix6::Zero();
  double score = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};
struct CoupledLocalStrongStep {
  Eigen::VectorXd delta;
  double damping = 0, condition = 0, residual = 0;
  bool valid = false, capped = false;
  std::string status = "NOT_RUN";
};
Eigen::Matrix4f coupledPoseAtEta(const Eigen::Matrix4f&, const CoupledVector6&, double);
CoupledLocalJet coupledNominalJet(const Eigen::Matrix4f&, double source_count,
    const CoupledNdtBackend&, double length_scale);
CoupledLocalJet coupledJointJet(const Eigen::Matrix4f&, const CoupledVector6&,
    const CoupledMatrix6&, double source_count, const CoupledNdtBackend&, double length_scale);
CoupledLocalStrongStep coupledStrongStep(const CoupledLocalJet&, int weak_dimension,
    const Eigen::VectorXd& delta_u, double cap);
}  // namespace dog_prior_map_fastlio2_frontend_exp
