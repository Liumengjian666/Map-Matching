#include "dog_prior_map_localization/mature_ndt_algorithms.hpp"

#include <iostream>
#include <limits>

using dog_prior_map_fastlio2_frontend_exp::CoupledMatrix6;
using dog_prior_map_localization::analyzeDcregAdapted;
using dog_prior_map_localization::candidateMotionMerit;
using dog_prior_map_localization::evaluateSkfGate;
using dog_prior_map_localization::makeWeakDirectionCandidates;
using dog_prior_map_localization::rankMatureCandidates;
using dog_prior_map_localization::MatureNdtCandidate;
using dog_prior_map_localization::MatureNdtConfig;

int main() {
  CoupledMatrix6 h = CoupledMatrix6::Zero();
  h.diagonal() << 1.0, 100.0, 120.0, 2.0, 80.0, 90.0;
  h(0, 3) = h(3, 0) = 0.5;
  const auto report = analyzeDcregAdapted(h, 0.03);
  if (!report.valid || !report.translation_degenerate || !report.rotation_degenerate ||
      !report.translation_eigenvalues.allFinite() || !report.rotation_eigenvalues.allFinite()) return 1;
  const auto candidates = makeWeakDirectionCandidates(
      report, Eigen::Matrix4f::Identity(), 0.2, 2.0);
  if (candidates.size() != 4) return 2;
  for (const auto& candidate : candidates) if (!candidate.finite) return 3;

  auto invalid = h;
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  if (analyzeDcregAdapted(invalid, 0.03).valid) return 4;
  invalid = h;
  invalid(0, 0) = 0.0;
  if (analyzeDcregAdapted(invalid, 0.03).valid) return 5;
  auto saddle = h;
  saddle(3, 3) = -2.0;
  const auto signed_report = analyzeDcregAdapted(saddle, 0.03);
  if (!signed_report.valid || !signed_report.rotation_nonconvex) {
    std::cerr << "signed Schur test failed status=" << signed_report.status
              << " valid=" << signed_report.valid
              << " rotation_nonconvex=" << signed_report.rotation_nonconvex << '\n';
    return 6;
  }
  CoupledMatrix6 off_diagonal_saddle = CoupledMatrix6::Zero();
  off_diagonal_saddle(0, 1) = off_diagonal_saddle(1, 0) = 1.0;
  off_diagonal_saddle(2, 2) = 2.0;
  off_diagonal_saddle(3, 3) = 4.0;
  off_diagonal_saddle(4, 4) = 5.0;
  off_diagonal_saddle(5, 5) = 6.0;
  const auto pivoted_report = analyzeDcregAdapted(off_diagonal_saddle, 0.03);
  if (!pivoted_report.valid || !pivoted_report.translation_nonconvex ||
      pivoted_report.rotation_nonconvex) {
    std::cerr << "off-diagonal indefinite Schur fallback failed status="
              << pivoted_report.status << " valid=" << pivoted_report.valid
              << " translation_nonconvex=" << pivoted_report.translation_nonconvex << '\n';
    return 11;
  }

  MatureNdtConfig config;
  const double nominal_merit = candidateMotionMerit(100.0, 100,
      Eigen::Matrix4f::Identity(), Eigen::Matrix4f::Identity(), 1.0, config);
  MatureNdtCandidate a, b;
  a.id = 4; a.finite = true; a.merit = nominal_merit + 0.1;
  b.id = 1; b.finite = true; b.merit = nominal_merit - 0.1;
  std::vector<MatureNdtCandidate> ranking{a, b};
  rankMatureCandidates(&ranking);
  if (ranking.front().id != 1) return 7;
  const auto skipped = evaluateSkfGate(true, report, true, true, false);
  if (skipped.status != "SKIPPED_DOUBLE_COUNT_PREVENTED" || !skipped.direction_selected) return 8;
  const auto no_cov = evaluateSkfGate(true, report, true, false, false);
  if (no_cov.status != "INTERFACE_READY_RELATIVE_VISUAL_NO_COVARIANCE") return 9;
  const auto off = evaluateSkfGate(false, report, true, false, true);
  if (off.status != "DISABLED") return 10;
  std::cout << "mature_ndt_algorithms_test PASS\n";
  return 0;
}
