#include "dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace reliability = dog_prior_map_fastlio2_frontend_exp::reliability;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  try {
    Eigen::Matrix<double, 6, 6> curvature = Eigen::Matrix<double, 6, 6>::Zero();
    curvature.block<3, 3>(0, 0).diagonal() << 1.0, 4.0, 9.0;
    curvature.block<3, 3>(3, 3).diagonal() << 16.0, 25.0, 36.0;
    const auto local = reliability::analyzeLocalObservability(curvature, true, true);
    require(local.valid && local.status == "VALID_BLOCK_CURVATURE",
            "local observability should be valid");
    require((local.rotation_block_eigenvalues - Eigen::Vector3d(1, 4, 9)).norm() < 1e-12,
            "rotation block eigenvalues mismatch");
    require((local.translation_block_eigenvalues - Eigen::Vector3d(16, 25, 36)).norm() < 1e-12,
            "translation block eigenvalues mismatch");
    require(!reliability::analyzeLocalObservability(curvature, false, true).valid,
            "nonconverged NDT must not be exposed as valid local observability");
    const auto unverified = reliability::analyzeLocalObservability(curvature, true, false);
    require(!unverified.valid &&
                unverified.status == "SCORE_GRADIENT_COORDINATE_CHECK_UNVERIFIED",
            "unverified derivative convention must gate local curvature validity");
    curvature(0, 0) = std::numeric_limits<double>::quiet_NaN();
    require(reliability::analyzeLocalObservability(curvature, true, true).status ==
                "NONFINITE_CURVATURE",
            "nonfinite curvature should be rejected");

    reliability::TerminalCapture nominal, positive, negative;
    nominal.fixed_objective = 10.0;
    positive.fixed_objective = 12.0;
    negative.fixed_objective = 9.0;
    nominal.converged = true;
    positive.converged = false;
    negative.converged = true;
    positive.map_T_lidar.translation().x() = 1.0;
    negative.map_T_lidar.translation().x() = -1.0;
    positive.map_T_lidar.linear() = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    negative.map_T_lidar.linear() = Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto nonlocal = reliability::analyzeNonlocalTerminalStability(
        nominal, positive, negative, 2);
    require(nonlocal.geometry_valid && nonlocal.objectives_finite,
            "finite terminal pair should remain recordable");
    require(std::abs(nonlocal.positive_negative_translation_gap_m - 2.0) < 1e-12,
            "translation terminal gap mismatch");
    require(std::abs(nonlocal.positive_negative_rotation_gap_rad - 0.2) < 1e-12,
            "rotation terminal gap mismatch");
    require(nonlocal.positive_minus_nominal_objective == 2.0 &&
                nonlocal.negative_minus_nominal_objective == -1.0 &&
                nonlocal.positive_minus_negative_objective == 3.0,
            "fixed-objective difference mismatch");
    require(nonlocal.extra_ndt_calls == 2 && !nonlocal.positive.converged,
            "call count and per-terminal convergence must be retained");
    require(nonlocal.status == "RECORDED_NO_BASIN_CLASSIFICATION",
            "terminal record must not classify a basin");
    std::cout << "P6_I6A_RELIABILITY_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I6A_RELIABILITY_TEST_FAIL: " << error.what() << '\n';
    return 1;
  }
}
