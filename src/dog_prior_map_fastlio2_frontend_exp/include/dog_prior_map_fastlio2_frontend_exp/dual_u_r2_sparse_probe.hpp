#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/dual_u_architecture.hpp"

#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

struct SparseProbeSupportSample {
  double chart_radius = 0.0;
  double plus_changed_fraction = 0.0;
  double minus_changed_fraction = 0.0;
};

struct SparseProbePolicy {
  double weak_to_strong_curvature_ratio = 0.01;
  double support_exit_fraction = 0.10;
  double post_exit_chart_margin = 1.0;
  double maximum_exit_chart_radius = 0.5;
  std::vector<double> exit_search_radii{
      0.005, 0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.5};
};

// Return the first one or two ascending U_obs eigen-directions whose
// curvature is <= 1% of the strongest direction. This is a fixed, GT-free
// 100:1 anisotropy trigger; it is not a calibrated probability threshold.
bool selectSparseProbeDirections(const DualUVector6d& ascending_eigenvalues,
    const SparseProbePolicy& policy, std::vector<int>* eigen_indices,
    std::string* reason);

// The local branch exit is the first tested chart radius where either signed
// perturbation changes at least the configured fraction of source-point
// Gaussian-cell memberships. A found radius is only an active-support branch
// transition, not proof that another NDT basin exists.
bool findSparseProbeBranchExit(const std::vector<SparseProbeSupportSample>& samples,
    const SparseProbePolicy& policy, double* exit_radius, std::string* reason);

double sparseProbeRadiusBeyondExit(double exit_radius,
    const SparseProbePolicy& policy);

}  // namespace dog_prior_map_fastlio2_frontend_exp
