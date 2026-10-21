#include "gradient_only.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

static void requireTest(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  try {
    Eigen::Matrix2d hss;
    hss << 2.0, 0.2, 0.2, 1.5;
    Eigen::Matrix<double, 2, 1> gs;
    Eigen::VectorXd dw(1);
    Eigen::Matrix<double, 2, 1> hsw;
    gs << 0.4, -0.3;
    hsw << 0.6, -0.25;
    dw << 0.1;
    const auto solved = r15c::solveAttribution(hss, hsw, gs, dw, 0.01);
    if(!solved.ok)throw std::runtime_error("finite block solve rejected: "+solved.status);
    requireTest((solved.ds_gradient_raw + solved.ds_coupling_raw - solved.ds_coupled_raw).norm() < 1e-12,
                "gradient/coupling decomposition mismatch");
    requireTest(solved.ds_coupling_raw.norm() > 1e-5, "nonzero Hsw produced no coupling term");

    const auto uncoupled = r15c::solveAttribution(hss, Eigen::MatrixXd::Zero(2, 1), gs, dw, 0.01);
    requireTest(uncoupled.ok, "zero-Hsw case rejected");
    requireTest((uncoupled.ds_gradient_trusted - uncoupled.ds_coupled_trusted).norm() < 1e-12,
                "zero-Hsw generated a false coupling response");

    Eigen::VectorXd out_of_ball(1); out_of_ball << 1.1;
    const auto rejected = r15c::solveAttribution(hss, hsw, gs, out_of_ball, 0.01);
    requireTest(!rejected.ok && rejected.status == "WEAK_STEP_OUTSIDE_TRUST_REGION",
                "out-of-ball weak displacement was not rejected");

    Eigen::Matrix2d bad; bad << 1, 2, 2, 1;
    const auto bad_hessian = r15c::solveAttribution(bad, hsw, gs, dw, 0.01);
    requireTest(!bad_hessian.ok, "indefinite regularized Hss must be rejected");

    Eigen::VectorXd nonfinite_gs = gs;
    nonfinite_gs[0] = std::numeric_limits<double>::quiet_NaN();
    const auto nonfinite = r15c::solveAttribution(hss, hsw, nonfinite_gs, dw, 0.01);
    requireTest(!nonfinite.ok && nonfinite.status == "INVALID_INPUT",
                "nonfinite gradient must be rejected before solving");
    std::cout << "R15C_GRADIENT_ONLY_TESTS_PASS lambda=0.01 zero_Hsw=pass decomposition=pass trust=pass nonfinite_guard=pass\n";
  } catch (const std::exception& e) {
    std::cerr << "R15C_GRADIENT_ONLY_TESTS_FAIL " << e.what() << '\n';
    return 1;
  }
  return 0;
}
