#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

namespace {

bool require(bool condition, const std::string& label,
             const std::string& reason = {}) {
  if (condition) return true;
  std::cerr << "FAIL: " << label;
  if (!reason.empty()) std::cerr << ": " << reason;
  std::cerr << '\n';
  return false;
}

WindowState stateAt(std::uint64_t stamp_ns) {
  WindowState state;
  state.stamp_ns = stamp_ns;
  state.rotation = Eigen::Matrix3d::Identity();
  state.position.setZero();
  state.velocity.setZero();
  state.gyro_bias.setZero();
  state.accel_bias.setZero();
  return state;
}

ImuPreintegratedMeasurement zeroImu(std::uint64_t start_ns,
                                    std::uint64_t end_ns) {
  ImuPreintegratedMeasurement measurement;
  measurement.start_stamp_ns = start_ns;
  measurement.end_stamp_ns = end_ns;
  measurement.dt_s = static_cast<double>(end_ns - start_ns) * 1e-9;
  measurement.delta_rotation.setIdentity();
  measurement.delta_velocity.setZero();
  measurement.delta_position.setZero();
  measurement.covariance = Matrix15d::Identity();
  measurement.valid = true;
  measurement.status = "A3C_R1_SYNTHETIC";
  return measurement;
}

LidarWindowMeasurement rankFiveLidar(const WindowState& state) {
  LidarWindowMeasurement measurement;
  measurement.observation_id = 1;
  measurement.stamp_ns = state.stamp_ns;
  measurement.measured_rotation = state.rotation;
  measurement.measured_position = state.position;
  measurement.covariance = Matrix6d::Identity();
  measurement.measurement_basis = Matrix6d::Identity();
  measurement.reliable_rank = 5;
  measurement.valid = true;
  const Matrix6d basis = measurement.measurement_basis;
  measurement.basis_relinearizer = [basis](const WindowState&, Matrix6d* out,
      int* rank, std::string* reason) {
    if (!out || !rank) {
      if (reason) *reason = "null_synthetic_basis_output";
      return false;
    }
    *out = basis;
    *rank = 5;
    return true;
  };
  return measurement;
}

bool buildRankFiveRepeatedWindow(bool capture,
                                 FixedLagWindow* output,
                                 std::string* reason) {
  if (!output) return false;
  FixedLagOptions options;
  options.maximum_duration_s = 0.15;
  options.maximum_nodes = 48;
  options.capture_marginalization_diagnostics = capture;
  *output = FixedLagWindow(options);
  const WindowState x0 = stateAt(1'000'000'000ULL);
  const WindowState x1 = stateAt(1'200'000'000ULL);
  const WindowState x2 = stateAt(1'400'000'000ULL);
  if (!output->addState(x0, reason) || !output->addState(x1, reason) ||
      !output->addState(x2, reason) ||
      !output->addLidarFactor(rankFiveLidar(x0), reason) ||
      !output->addImuFactor(2, x1.stamp_ns, x2.stamp_ns,
                            zeroImu(x1.stamp_ns, x2.stamp_ns), reason))
    return false;
  return true;
}

bool captureParityAndRepeatedHistory() {
  FixedLagWindow trace_off, trace_on;
  std::string reason;
  if (!buildRankFiveRepeatedWindow(false, &trace_off, &reason) ||
      !buildRankFiveRepeatedWindow(true, &trace_on, &reason))
    return require(false, "rank-five repeated fixture construction", reason);
  if (!trace_off.optimize(&reason) || !trace_on.optimize(&reason))
    return require(false, "trace parity optimizer status", reason);
  if (!require(trace_off.summary().optimizer_status ==
                   trace_on.summary().optimizer_status,
               "trace parity optimizer result") ||
      !require(trace_off.summary().optimizer_success &&
                   trace_on.summary().optimizer_success,
               "both optimizers succeed"))
    return false;
  if (!trace_off.marginalizeIfNeeded(&reason) ||
      !trace_on.marginalizeIfNeeded(&reason))
    return require(false, "rank-five repeated fixture marginalization", reason);

  if (!require(trace_off.states().size() == trace_on.states().size(),
               "trace parity state count") ||
      !require(trace_off.priorInformation().rows() ==
                   trace_on.priorInformation().rows(),
               "trace parity prior dimensions") ||
      !require((trace_off.priorInformation() -
                trace_on.priorInformation()).norm() == 0.0,
               "trace parity prior information") ||
      !require((trace_off.priorGradient() - trace_on.priorGradient()).norm() ==
                   0.0,
               "trace parity prior gradient"))
    return false;
  for (std::size_t i = 0; i < trace_off.states().size(); ++i) {
    const auto delta = localDifference(trace_off.states()[i],
                                       trace_on.states()[i]);
    if (!require(delta.norm() == 0.0, "trace parity state values")) return false;
  }
  const auto off_summary = trace_off.summary();
  const auto on_summary = trace_on.summary();
  if (!require(off_summary.optimizer_status == on_summary.optimizer_status &&
                   off_summary.marginalization_status ==
                       on_summary.marginalization_status &&
                   off_summary.marginalization_psd ==
                       on_summary.marginalization_psd,
               "trace parity optimizer and marginalization statuses") ||
      !require(off_summary.imu_factor_count == on_summary.imu_factor_count &&
                   off_summary.lidar_factor_count == on_summary.lidar_factor_count &&
                   off_summary.visual_factor_count == on_summary.visual_factor_count,
               "trace parity factor lifecycle") ||
      !require(trace_off.marginalizationTraceForDiagnostics().empty(),
               "trace off retains no trace rows") ||
      !require(trace_on.marginalizationTraceForDiagnostics().size() == 2,
               "two attempts captured in one enforcement"))
    return false;

  const auto& first = trace_on.marginalizationTraceForDiagnostics()[0];
  const auto& second = trace_on.marginalizationTraceForDiagnostics()[1];
  return require(first.marginalization_enforcement_index ==
                     second.marginalization_enforcement_index,
                 "attempts share enforcement identity") &&
      require(first.attempt_index_within_enforcement == 1 &&
                  second.attempt_index_within_enforcement == 2,
              "attempt indices are one-based and ordered") &&
      require(first.oldest_state_removed && second.oldest_state_removed,
              "both oldest removals committed") &&
      require(first.incident_lidar_factor_count == 1 &&
                  first.incident_imu_factor_count == 0,
              "rank-five LiDAR is the first consumed factor") &&
      require(first.hmm.numerical_rank == 5,
              "rank-five LiDAR-like Hmm diagnosed rank deficient") &&
      require(first.solve_jitter > 0.0,
              "existing solve-only jitter is observed, not changed") &&
      require(second.incident_imu_factor_count == 1,
              "retained IMU link becomes incident on second removal") &&
      require(first.new_prior.available && second.new_prior.available,
              "prior spectrum recorded after each repeated Schur");
}

bool positiveDefinitePivotCase() {
  FixedLagOptions options;
  options.maximum_duration_s = 0.5;
  options.capture_marginalization_diagnostics = true;
  FixedLagWindow window(options);
  const WindowState x0 = stateAt(2'000'000'000ULL);
  const WindowState x1 = stateAt(2'600'000'000ULL);
  Matrix15d information = 3.0 * Matrix15d::Identity();
  std::string reason;
  if (!window.initializeWithPriorAtomic(x0, information, Vector15d::Zero(),
                                        &reason) ||
      !window.addState(x1, &reason) || !window.marginalizeIfNeeded(&reason))
    return require(false, "well-conditioned SPD marginalization", reason);
  const auto& trace = window.marginalizationTraceForDiagnostics();
  return require(trace.size() == 1, "SPD case has one attempt") &&
      require(trace[0].hmm.numerical_rank == 15,
              "well-conditioned Hmm full rank") &&
      require(trace[0].solve_jitter == 0.0,
              "well-conditioned Hmm does not enter jitter path") &&
      require(trace[0].ldlt_initial_positive_d_count == 15 &&
                  trace[0].ldlt_initial_negative_d_count == 0,
              "SPD signed LDLT pivots reported");
}

bool pivotDiagnosticsCases() {
  Eigen::Matrix2d scale_disparity = Eigen::Matrix2d::Zero();
  scale_disparity(0, 0) = 1e8;
  scale_disparity(1, 1) = 1e-10;
  const auto scale_stats = marginalizationMatrixStatsForDiagnostics(
      scale_disparity);
  Eigen::LDLT<Eigen::Matrix2d> scale_ldlt(scale_disparity);
  const auto scale_pivots = marginalizationLdltStatsForDiagnostics(
      scale_ldlt.vectorD());
  if (!require(scale_stats.min_abs_eigenvalue > 1e-12,
               "absolute gate passes scale-disparity minimum") ||
      !require(scale_pivots.available && scale_pivots.min_abs_d > 1e-12,
               "absolute pivot gate would pass scale-disparity case") ||
      !require(scale_pivots.pivot_ratio < 1e-16,
               "relative pivot ratio exposes catastrophic scale disparity"))
    return false;

  Eigen::Matrix2d indefinite;
  indefinite << 1.0, 2.0, 2.0, 1.0;
  Eigen::LDLT<Eigen::Matrix2d> indefinite_ldlt(indefinite);
  const auto signed_pivots = marginalizationLdltStatsForDiagnostics(
      indefinite_ldlt.vectorD());
  const auto indefinite_stats = marginalizationMatrixStatsForDiagnostics(
      indefinite);
  return require(indefinite_stats.lambda_min < 0.0,
                 "signed-inertia fixture is indefinite") &&
      require(signed_pivots.negative_d_count == 1,
              "negative LDLT pivot is not hidden by absolute values");
}

bool inspectProductionSchurGate(const std::string& capsule_path) {
  std::ifstream input(capsule_path, std::ios::binary);
  if (!input) return require(false, "open real failure capsule");
  char magic[16] = {};
  const char expected_magic[16] = {
      'P','6','A','3','C','R','1','C','A','P','S','U','L','E','\0','\0'};
  input.read(magic, sizeof(magic));
  std::uint32_t version = 0, count = 0;
  input.read(reinterpret_cast<char*>(&version), sizeof(version));
  input.read(reinterpret_cast<char*>(&count), sizeof(count));
  if (!input || std::string(magic, sizeof(magic)) !=
                    std::string(expected_magic, sizeof(expected_magic)) ||
      version != 1 || count != 11)
    return require(false, "failure capsule header is valid");

  Eigen::MatrixXd consumed_hessian, correction_h;
  for (std::uint32_t index = 0; index < count; ++index) {
    std::uint32_t name_size = 0;
    std::uint64_t rows = 0, columns = 0;
    input.read(reinterpret_cast<char*>(&name_size), sizeof(name_size));
    if (!input || name_size == 0 || name_size > 256)
      return require(false, "failure capsule array name length");
    std::string name(name_size, '\0');
    input.read(name.data(), static_cast<std::streamsize>(name_size));
    input.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    input.read(reinterpret_cast<char*>(&columns), sizeof(columns));
    if (!input || rows > 10000 || columns > 10000 ||
        rows * columns > 100000000ULL)
      return require(false, "failure capsule matrix dimensions");
    std::vector<double> payload(static_cast<std::size_t>(rows * columns));
    input.read(reinterpret_cast<char*>(payload.data()),
        static_cast<std::streamsize>(payload.size() * sizeof(double)));
    if (!input) return require(false, "failure capsule matrix payload");
    if (name == "consumed_hessian" || name == "correction_h") {
      Eigen::MatrixXd matrix(static_cast<Eigen::Index>(rows),
                             static_cast<Eigen::Index>(columns));
      for (std::uint64_t row = 0; row < rows; ++row)
        for (std::uint64_t column = 0; column < columns; ++column)
          matrix(static_cast<Eigen::Index>(row),
                 static_cast<Eigen::Index>(column)) =
              payload[static_cast<std::size_t>(row * columns + column)];
      if (name == "consumed_hessian") consumed_hessian = std::move(matrix);
      else correction_h = std::move(matrix);
    }
  }
  if (consumed_hessian.rows() <= 15 || correction_h.rows() != 15 ||
      consumed_hessian.rows() - 15 != correction_h.cols())
    return require(false, "failure capsule contains Schur inputs");

  const Eigen::Index retained = consumed_hessian.rows() - 15;
  const Eigen::MatrixXd hmr = consumed_hessian.topRightCorner(15, retained);
  const Eigen::MatrixXd hrr = consumed_hessian.bottomRightCorner(retained,
                                                                  retained);
  const Eigen::MatrixXd raw = hrr - hmr.transpose() * correction_h;
  Eigen::MatrixXd production = raw;
  production = 0.5 * (production + production.transpose());
  const Eigen::MatrixXd out_of_place =
      (0.5 * (raw + raw.transpose())).eval();
  const double production_max_asymmetry =
      (production - production.transpose()).cwiseAbs().maxCoeff();
  const double production_fro_asymmetry =
      (production - production.transpose()).norm();
  const double out_of_place_max_asymmetry =
      (out_of_place - out_of_place.transpose()).cwiseAbs().maxCoeff();

  const auto productionValidator = [](const Eigen::MatrixXd& matrix) {
    if (!matrix.allFinite() ||
        (matrix - matrix.transpose()).cwiseAbs().maxCoeff() > 1e-8)
      return false;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(matrix);
    return solver.info() == Eigen::Success &&
        solver.eigenvalues().allFinite() &&
        solver.eigenvalues().minCoeff() >= -1e-6;
  };
  std::cout << std::setprecision(17)
      << "TX90_CPP_CAPSULE_EMULATION raw_asymmetry_fro="
      << (raw - raw.transpose()).norm()
      << " production_symmetry_fro=" << production_fro_asymmetry
      << " production_symmetry_max_abs=" << production_max_asymmetry
      << " out_of_place_symmetry_max_abs=" << out_of_place_max_asymmetry
      << " production_validator_pass=" << productionValidator(production)
      << " out_of_place_validator_pass=" << productionValidator(out_of_place)
      << '\n';
  return require(!productionValidator(production),
                 "TX90 Eigen in-place Schur expression reproduces validator rejection") &&
      require(productionValidator(out_of_place),
              "out-of-place symmetric Schur passes unchanged PSD thresholds");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2) return inspectProductionSchurGate(argv[1]) ? 0 : 1;
  if (argc != 1) return 2;
  if (!captureParityAndRepeatedHistory() || !positiveDefinitePivotCase() ||
      !pivotDiagnosticsCases())
    return 1;
  std::cout << "A3C_R1_MARGINALIZATION_DIAGNOSTICS_TEST_PASS\n";
  return 0;
}
