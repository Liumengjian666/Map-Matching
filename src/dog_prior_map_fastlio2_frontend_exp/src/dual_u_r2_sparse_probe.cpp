#include "dog_prior_map_fastlio2_frontend_exp/dual_u_r2_sparse_probe.hpp"

#include <algorithm>
#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}
}  // namespace

bool selectSparseProbeDirections(const DualUVector6d& eigenvalues,
    const SparseProbePolicy& policy, std::vector<int>* indices,
    std::string* reason) {
  if (reason) reason->clear();
  if (!indices) return fail(reason, "NULL_SPARSE_PROBE_DIRECTION_OUTPUT");
  indices->clear();
  if (!eigenvalues.allFinite() || eigenvalues(5) <= 0.0 ||
      !std::isfinite(policy.weak_to_strong_curvature_ratio) ||
      policy.weak_to_strong_curvature_ratio <= 0.0 ||
      policy.weak_to_strong_curvature_ratio >= 1.0)
    return fail(reason, "INVALID_SPARSE_PROBE_SPECTRUM_OR_POLICY");
  for (int index = 0; index < 2; ++index) {
    if (eigenvalues(index) < 0.0) continue;
    if (eigenvalues(index) / eigenvalues(5) <=
        policy.weak_to_strong_curvature_ratio)
      indices->push_back(index);
  }
  return true;
}

bool findSparseProbeBranchExit(
    const std::vector<SparseProbeSupportSample>& samples,
    const SparseProbePolicy& policy, double* exit_radius,
    std::string* reason) {
  if (reason) reason->clear();
  if (!exit_radius || samples.empty() ||
      !std::isfinite(policy.support_exit_fraction) ||
      policy.support_exit_fraction <= 0.0 || policy.support_exit_fraction > 1.0 ||
      !std::isfinite(policy.maximum_exit_chart_radius) ||
      policy.maximum_exit_chart_radius <= 0.0)
    return fail(reason, "INVALID_SPARSE_PROBE_SUPPORT_INPUT");
  double previous_radius = 0.0;
  for (const SparseProbeSupportSample& sample : samples) {
    if (!std::isfinite(sample.chart_radius) || sample.chart_radius <= previous_radius ||
        !std::isfinite(sample.plus_changed_fraction) ||
        !std::isfinite(sample.minus_changed_fraction) ||
        sample.plus_changed_fraction < 0.0 || sample.plus_changed_fraction > 1.0 ||
        sample.minus_changed_fraction < 0.0 || sample.minus_changed_fraction > 1.0)
      return fail(reason, "NONMONOTONIC_OR_INVALID_SPARSE_PROBE_SUPPORT_SAMPLES");
    previous_radius = sample.chart_radius;
  }
  for (const SparseProbeSupportSample& sample : samples) {
    if (sample.chart_radius <= policy.maximum_exit_chart_radius &&
        std::max(sample.plus_changed_fraction, sample.minus_changed_fraction) >=
            policy.support_exit_fraction) {
      *exit_radius = sample.chart_radius;
      return true;
    }
  }
  if (reason) *reason = "NO_SUPPORT_EXIT_WITHIN_BOUNDED_SEARCH_RADIUS";
  return false;
}

double sparseProbeRadiusBeyondExit(double exit_radius,
    const SparseProbePolicy& policy) {
  if (!std::isfinite(exit_radius) || exit_radius < 0.0 ||
      !std::isfinite(policy.post_exit_chart_margin) ||
      policy.post_exit_chart_margin <= 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  return exit_radius + policy.post_exit_chart_margin;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
