#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

namespace {
bool check(bool value, const char* label) {
  if (!value) std::cerr << "FAIL: " << label << '\n';
  return value;
}

bool nonsymmetricRegression() {
  Eigen::MatrixXd a(3, 3);
  a << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  const Eigen::MatrixXd safe = evaluateSymmetricInformation(a);
  const Eigen::MatrixXd oracle = (0.5 * (a + a.transpose())).eval();
  if (!check((safe - safe.transpose()).norm() == 0 &&
                 (safe - oracle).norm() == 0,
             "evaluated symmetry is exact and formula unchanged")) return false;
#ifdef NDEBUG
  // Demonstration only: unsafe behavior is build-dependent, not a PASS gate.
  Eigen::MatrixXd unsafe = a;
  unsafe = 0.5 * (unsafe + unsafe.transpose());
  std::cout << "nonsymmetric_unsafe_difference=" << (unsafe - safe).norm() << '\n';
#else
  std::cout << "unsafe_demo=SKIPPED_DEBUG_ASSERTION_STRATEGY\n";
#endif
  return true;
}

LidarWindowMeasurement rankFive(std::uint64_t id, const WindowState& state) {
  LidarWindowMeasurement m;
  m.observation_id = id;
  m.stamp_ns = state.stamp_ns;
  m.measured_rotation = state.rotation;
  m.measured_position = state.position;
  m.measurement_basis.setIdentity();
  m.covariance.setIdentity();
  m.reliable_rank = 5;
  m.valid = true;
  m.basis_relinearizer = [](const WindowState&, Matrix6d* b, int* rank,
                            std::string*) {
    b->setIdentity(); *rank = 5; return true;
  };
  return m;
}

bool rankFiveContract() {
  WindowState x;
  x.stamp_ns = 1'000'000'000;
  auto m = rankFive(1, x);
  m.measured_position = Eigen::Vector3d(0.1, -0.2, 0.3);
  const Eigen::Vector3d rotation_vector(0.04, 0.05, 0.06);
  m.measured_rotation = Eigen::AngleAxisd(rotation_vector.norm(),
      rotation_vector.normalized()).toRotationMatrix();
  FrozenLidarProjection frozen;
  Eigen::VectorXd r; Eigen::MatrixXd j, c;
  std::string reason;
  if (!freezeLidarProjection(x, m, &frozen, &reason) ||
      !linearizeLidarFactorWithFrozenProjection(x, m, frozen, &r, &j, &c, &reason))
    return check(false, reason.c_str());
  Eigen::Matrix<double, 6, 1> raw;
  raw.head<3>() = m.measured_position - x.position;
  raw.tail<3>() = rotation_vector;
  const Eigen::MatrixXd info = j.transpose() * c.inverse() * j;
  Eigen::SelfAdjointEigenSolver<Matrix6d> eig(info.topLeftCorner(6, 6));
  int rank = 0;
  for (Eigen::Index i = 0; i < eig.eigenvalues().size(); ++i)
    if (eig.eigenvalues()(i) > 1e-10) ++rank;
  const Eigen::Matrix<double, 6, 1> weak_pose = eig.eigenvectors().col(0);
  return check(frozen.reliable_rank == 5 && rank == 5,
               "rank-five unary information remains rank five") &&
      check((r - m.measurement_basis.leftCols(5).transpose() * raw).norm() < 1e-15,
            "B transpose residual unchanged") &&
      check((c - m.measurement_basis.leftCols(5).transpose() * m.covariance *
                  m.measurement_basis.leftCols(5)).norm() == 0,
            "projected covariance unchanged") &&
      check((j.leftCols(6) * weak_pose).norm() < 1e-10,
            "LiDAR pose null direction not filled (not velocity or bias nullspace)");
}

bool build(bool capture, FixedLagWindow* window) {
  FixedLagOptions options;
  options.maximum_duration_s = 0.15;
  options.capture_marginalization_diagnostics = capture;
  *window = FixedLagWindow(options);
  std::string reason;
  for (std::uint64_t i = 0; i < 4; ++i) {
    WindowState x;
    x.stamp_ns = 1'000'000'000 + i * 100'000'000;
    if (!window->addState(x, &reason)) return check(false, reason.c_str());
    if (i == 0 && !window->setInitialPrior(x.stamp_ns, Matrix15d::Identity(),
                                          Vector15d::Zero(), &reason))
      return check(false, reason.c_str());
    if (i > 0) {
      ImuPreintegratedMeasurement imu;
      imu.start_stamp_ns = x.stamp_ns - 100'000'000;
      imu.end_stamp_ns = x.stamp_ns;
      imu.dt_s = 0.1;
      // Near-stationary deltas; default gravity leaves small nonzero residuals.
      imu.delta_velocity = Eigen::Vector3d(0, 0, 0.981);
      imu.delta_position = Eigen::Vector3d(0, 0, 0.04905);
      imu.covariance = Matrix15d::Identity();
      imu.valid = true;
      if (!window->addImuFactor(2*i, imu.start_stamp_ns, imu.end_stamp_ns,
                                imu, &reason)) return check(false, reason.c_str());
    }
    if (!window->addLidarFactor(rankFive(2*i+1, x), &reason))
      return check(false, reason.c_str());
  }
  return window->optimize(&reason) || check(false, reason.c_str());
}

bool multiRemovalAndParity() {
  FixedLagWindow off, on;
  if (!build(false, &off) || !build(true, &on)) return false;
  Eigen::MatrixXd h; Eigen::VectorXd g; double cost;
  std::string reason;
  if (!on.linearizedSystem(&h, &g, &cost, &reason)) return check(false, reason.c_str());
  // Independent complete-graph Schur oracle eliminates the first two nodes.
  Eigen::LDLT<Eigen::MatrixXd> factor(h.topLeftCorner(30, 30));
  const Eigen::MatrixXd cross = h.topRightCorner(30, 30);
  const Eigen::MatrixXd expected_h = evaluateSymmetricInformation(
      h.bottomRightCorner(30, 30) - cross.transpose() * factor.solve(cross));
  const Eigen::VectorXd expected_g = g.tail(30) - cross.transpose() * factor.solve(g.head(30));
  const auto revision = on.summary().window_revision;
  if (!off.marginalizeIfNeeded(&reason) || !on.marginalizeIfNeeded(&reason))
    return check(false, reason.c_str());
  const auto& traces = on.marginalizationTraceForDiagnostics();
  if (!check(traces.size() == 2 && traces[0].oldest_state_removed &&
                 traces[1].oldest_state_removed &&
                 traces[0].marginalization_enforcement_index == traces[1].marginalization_enforcement_index &&
                 traces[0].nodes_after_attempt == traces[1].nodes_before_attempt &&
                 traces[0].new_prior.lambda_min == traces[1].incoming_prior.lambda_min &&
                 traces[0].new_prior.lambda_max == traces[1].incoming_prior.lambda_max &&
                 traces[0].incident_imu_factor_count == 1 && traces[1].incident_imu_factor_count == 1 &&
                 traces[0].incident_lidar_factor_count == 1 && traces[1].incident_lidar_factor_count == 1,
             "one enforcement completes two consistent removals")) return false;
  for (const auto& t : traces) {
    if (!check(t.consumed_system.symmetry_max_abs == 0 && t.hmm.symmetry_max_abs == 0 &&
                   t.new_prior.symmetry_max_abs == 0 && t.new_prior.finite,
               "production consumed Hessian and stored prior exact symmetry")) return false;
    std::cout << "attempt=" << t.attempt_index_within_enforcement
              << " raw_schur_max_asymmetry=" << t.raw_schur.symmetry_max_abs
              << " stored_prior_max_asymmetry=" << t.new_prior.symmetry_max_abs << '\n';
  }
  if (!on.linearizedSystem(&h, &g, &cost, &reason)) return false;
  const double he = (h - expected_h).norm() / expected_h.norm();
  const double ge = (g - expected_g).norm();
  std::cout << "multi_removal_H_relative_error=" << he << " g_error=" << ge << '\n';
  const auto s = on.summary(); const auto o = off.summary();
  bool identical_states = on.states().size() == off.states().size();
  for (std::size_t i = 0; identical_states && i < on.states().size(); ++i)
    identical_states = on.states()[i].stamp_ns == off.states()[i].stamp_ns &&
        localDifference(on.states()[i], off.states()[i]).norm() == 0;
  return check(he < 1e-10 && ge < 1e-10, "multi-removal information conservation") &&
      check(s.window_revision == revision + 2 && s.optimized_revision == s.window_revision,
            "revision and optimized feedback advance with removals") &&
      check(s.imu_factor_count == 1 && s.lidar_factor_count == 2 &&
                 s.active_observation_id_count == 3,
            "incident factors retired, retained-only factors stay active") &&
      check(identical_states && (on.priorInformation() - off.priorInformation()).norm() == 0 &&
                 (on.priorGradient() - off.priorGradient()).norm() == 0 &&
                 s.optimizer_status == o.optimizer_status && s.marginalization_status == o.marginalization_status &&
                 s.imu_factor_count == o.imu_factor_count && s.lidar_factor_count == o.lidar_factor_count &&
                 off.marginalizationTraceForDiagnostics().empty(),
            "diagnostics OFF ON identical estimation and lifecycle");
}
}  // namespace

int main() {
  std::cout << std::setprecision(17);
  if (!nonsymmetricRegression() || !rankFiveContract() || !multiRemovalAndParity()) return 1;
  std::cout << "A3C_R2_ALIAS_REGRESSION_PASS\n";
}
