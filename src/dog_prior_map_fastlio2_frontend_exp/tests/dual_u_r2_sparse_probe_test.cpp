#include "dog_prior_map_fastlio2_frontend_exp/dual_u_r2_sparse_probe.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace paper = dog_prior_map_fastlio2_frontend_exp;

namespace {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance,
    const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
      message);
}
}  // namespace

int main() {
  try {
    const paper::SparseProbePolicy policy;
    paper::DualUVector6d spectrum;
    spectrum << 0.01, 0.01, 0.1, 0.3, 0.7, 1.0;
    std::vector<int> directions;
    std::string reason;
    require(paper::selectSparseProbeDirections(spectrum, policy,
        &directions, &reason), "valid_spectrum_rejected");
    require(directions == std::vector<int>({0, 1}),
        "weakest_two_directions_not_selected");

    spectrum(1) = 0.0100001;
    require(paper::selectSparseProbeDirections(spectrum, policy,
        &directions, &reason), "single_weak_direction_spectrum_rejected");
    require(directions == std::vector<int>({0}),
        "nonweak_second_direction_selected");

    std::vector<paper::SparseProbeSupportSample> samples{
        {0.005, 0.01, 0.02}, {0.01, 0.04, 0.08},
        {0.02, 0.11, 0.07}, {0.04, 0.2, 0.2}};
    double exit_radius = 0.0;
    require(paper::findSparseProbeBranchExit(samples, policy,
        &exit_radius, &reason), "support_exit_not_found");
    near(exit_radius, 0.02, 1e-12, "wrong_first_support_exit_radius");
    near(paper::sparseProbeRadiusBeyondExit(exit_radius, policy),
        1.02, 1e-12, "probe_not_one_chart_unit_beyond_exit");

    samples = {{0.005, 0.01, 0.02}, {0.5, 0.03, 0.04},
               {1.0, 0.5, 0.5}};
    require(!paper::findSparseProbeBranchExit(samples, policy,
        &exit_radius, &reason), "out_of_bound_support_exit_accepted");
    require(reason == "NO_SUPPORT_EXIT_WITHIN_BOUNDED_SEARCH_RADIUS",
        "wrong_no_exit_reason");

    samples = {{0.01, 0.2, 0.1}, {0.005, 0.3, 0.3}};
    require(!paper::findSparseProbeBranchExit(samples, policy,
        &exit_radius, &reason), "unsorted_support_radii_accepted");

    std::cout << "dual_u_r2_sparse_probe_test PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "dual_u_r2_sparse_probe_test FAIL: " << error.what() << '\n';
    return 1;
  }
}
