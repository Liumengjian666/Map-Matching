#include "dog_prior_map_fastlio2_frontend_exp/dual_u_architecture.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp;
namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

Pose3d poseFrom(const Eigen::Vector3d& position, const Eigen::Vector3d& axis,
                double angle) {
  Pose3d pose;
  pose.position = position;
  pose.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(angle, axis.normalized()));
  return pose;
}

Pose3d composePose(const Pose3d& lhs, const Pose3d& rhs) {
  Pose3d result;
  result.position = lhs.position + lhs.orientation * rhs.position;
  result.orientation = (lhs.orientation * rhs.orientation).normalized();
  return result;
}

Eigen::Matrix<double, 6, 1> exactPoseResidual(const Pose3d& prediction,
                                               const Pose3d& measurement) {
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = measurement.position - prediction.position;
  Eigen::Quaterniond q = (prediction.orientation.conjugate() *
      measurement.orientation).normalized();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const Eigen::AngleAxisd angle_axis(q);
  residual.tail<3>() = angle_axis.axis() * angle_axis.angle();
  return residual;
}

Eigen::Vector3d eulerAtEta(const Pose3d& base, const DualUVector6d& eta) {
  const double angle = eta.tail<3>().norm();
  const Eigen::Matrix3d delta = angle < 1e-14
      ? Eigen::Matrix3d::Identity()
      : Eigen::AngleAxisd(angle, eta.tail<3>() / angle).toRotationMatrix();
  const Eigen::Vector3d base_euler =
      base.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  Eigen::Vector3d euler = (delta * base.orientation.toRotationMatrix()).eulerAngles(0, 1, 2);
  for (int i = 0; i < 3; ++i)
    euler(i) += 2.0 * M_PI * std::round((base_euler(i) - euler(i)) / (2.0 * M_PI));
  return euler;
}

void testPullbackAndObjectiveSignScale() {
  Pose3d pose = poseFrom(Eigen::Vector3d(2.1, -0.8, 0.4),
      Eigen::Vector3d(0.2, -0.5, 1.0), 0.61);
  PclNdtScoreJet jet;
  jet.valid = true;
  jet.status = "PASS_PCL_SCORE_JET";
  jet.score_sum = 230.0;
  jet.source_point_count = 23;
  jet.pcl_euler_xyz = pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  jet.score_gradient << 0.3, -0.2, 0.15, 0.06, 0.04, -0.03;
  DualUMatrix6d positive_curvature = DualUMatrix6d::Identity();
  positive_curvature.diagonal() << 2.0, 2.5, 3.0, 4.0, 4.5, 5.0;
  positive_curvature(0, 4) = positive_curvature(4, 0) = 0.12;
  positive_curvature(2, 3) = positive_curvature(3, 2) = -0.08;
  jet.score_hessian = -positive_curvature;

  WithinBasinObservability result;
  std::string reason;
  require(analyzeWithinBasinObservability(jet, pose,
      Eigen::Vector3d::Constant(0.8), 0.8, &result, &reason),
      "valid synthetic U_obs rejected");
  require(result.valid && result.locally_convex, "synthetic U_obs not locally convex");
  require(std::abs(result.objective_per_source + 10.0) < 1e-12,
      "PCL score-to-objective sign/normalization changed");

  // Independent scalar evaluation in the declared product chart. The toy
  // native objective has a nonzero gradient, so this also checks the g_i K_i
  // term rather than only a congruence transform of the Hessian.
  const double source_count = static_cast<double>(jet.source_point_count);
  const auto objective = [&](double scale, const DualUVector6d& direction) {
    const DualUVector6d eta = scale * direction;
    Eigen::Matrix<double, 6, 1> p;
    p.head<3>() = pose.position + 0.8 * eta.head<3>();
    p.tail<3>() = eulerAtEta(pose, eta);
    const Eigen::Matrix<double, 6, 1> delta = p -
        (Eigen::Matrix<double, 6, 1>() << pose.position,
         jet.pcl_euler_xyz).finished();
    const double score = jet.score_sum + jet.score_gradient.dot(delta) +
        0.5 * delta.dot(jet.score_hessian * delta);
    return -score / source_count;
  };
  const double h = 0.01;
  for (int index : {0, 5}) {
    const DualUVector6d direction = result.curvature_eigenvectors.col(index);
    const double f0 = objective(0.0, direction);
    const double finite_difference =
        (objective(h, direction) - 2.0 * f0 + objective(-h, direction)) / (h * h);
    const double analytic = direction.dot(result.local_curvature * direction);
    require(std::isfinite(finite_difference) && std::isfinite(analytic) &&
        std::abs(finite_difference - analytic) < 2e-3 * std::max(1.0, std::abs(analytic)),
        "weak/strong second-order product-chart pullback disagrees with objective finite difference");
  }
}

void testChartSingularityFailsClosed() {
  Pose3d pose;
  pose.position.setZero();
  pose.orientation = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d euler = pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  ProductChartPullback chart;
  std::string reason;
  require(!buildMapProductChartPullback(pose, euler, 0.8, &chart, &reason),
      "Euler singularity was accepted");
  require(!reason.empty(), "Euler singularity did not return a fail-closed reason");
}

void testEulerBranchContinuityNearZeroPitch() {
  Pose3d pose;
  pose.position = Eigen::Vector3d(0.4, -0.2, 0.1);
  // Representative small-angle orientation from a Floor01 terminal. Eigen's
  // XYZ extraction may choose its alternate equivalent branch on one side of
  // zero pitch; the local chart must unwrap to the branch nearest the base.
  pose.orientation = Eigen::Quaterniond(0.99999866122818259,
      -0.00041050590748135593, -0.001274928241306067,
      -0.0009399918732831134).normalized();
  PclNdtScoreJet jet;
  jet.valid = true;
  jet.status = "PASS_PCL_SCORE_JET";
  jet.score_sum = 1400.0;
  jet.source_point_count = 1400;
  jet.pcl_euler_xyz = pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  jet.score_gradient.setZero();
  jet.score_hessian = -DualUMatrix6d::Identity();
  WithinBasinObservability result;
  std::string reason;
  if (!analyzeWithinBasinObservability(jet, pose,
      Eigen::Vector3d::Constant(0.8), 0.8, &result, &reason))
    throw std::runtime_error("small-angle Euler alternate branch was not handled: " + reason);
  require(result.valid && result.locally_convex,
      "small-angle branch chart did not produce finite local curvature");
}

void testLidarImuReferenceTransport() {
  const double length_scale = 0.8;
  const Eigen::Vector3d r_map(0.42, -0.17, 0.08);
  DualUVector6d eta_lidar;
  eta_lidar << 0.08, -0.03, 0.02, 0.025, -0.04, 0.015;
  const DualUMatrix6d g = lidarOriginToImuOriginTangentMap(r_map, length_scale);
  const DualUVector6d eta_imu = g * eta_lidar;
  const Eigen::Vector3d dt_lidar = length_scale * eta_lidar.head<3>();
  const Eigen::Vector3d dt_imu_expected = dt_lidar + r_map.cross(eta_lidar.tail<3>());
  require((length_scale * eta_imu.head<3>() - dt_imu_expected).norm() < 1e-12,
      "LiDAR-to-IMU lever-arm perturbation sign is wrong");

  DualUMatrix6d h_lidar = DualUMatrix6d::Identity();
  h_lidar(0, 4) = h_lidar(4, 0) = 0.3;
  h_lidar(2, 5) = h_lidar(5, 2) = -0.2;
  const DualUMatrix6d g_inverse = g.inverse();
  const DualUMatrix6d h_imu = g_inverse.transpose() * h_lidar * g_inverse;
  require(std::abs(eta_lidar.dot(h_lidar * eta_lidar) -
                   eta_imu.dot(h_imu * eta_imu)) < 1e-12,
      "reference-point Hessian transport changed physical directional curvature");
}

void testUobsChartToEkfResidualJacobian() {
  const double length_scale = 0.8;
  const Pose3d map_T_lidar = poseFrom(Eigen::Vector3d(2.0, -0.7, 0.4),
      Eigen::Vector3d(0.3, -0.4, 1.0), 0.42);
  const Pose3d lidar_T_imu = poseFrom(Eigen::Vector3d(0.23, -0.11, 0.06),
      Eigen::Vector3d(1.0, 0.2, -0.1), 0.17);
  const Pose3d map_T_imu_measurement = composePose(map_T_lidar, lidar_T_imu);
  Pose3d map_T_imu_prediction = map_T_imu_measurement;
  map_T_imu_prediction.position += Eigen::Vector3d(-0.12, 0.06, 0.03);
  map_T_imu_prediction.orientation =
      (Eigen::Quaterniond(Eigen::AngleAxisd(0.31,
          Eigen::Vector3d(0.4, -0.1, 0.8).normalized())) *
       map_T_imu_measurement.orientation).normalized();

  WithinBasinObservability u_obs;
  u_obs.valid = true;
  u_obs.locally_convex = true;
  u_obs.length_scale_m = length_scale;
  u_obs.curvature_eigenvalues << 0.01, 0.02, 0.1, 0.3, 0.7, 1.0;
  // Use a genuinely coupled orthonormal basis, not the canonical axes.
  u_obs.curvature_eigenvectors.setIdentity();
  for (int index = 0; index < 5; ++index) {
    const double angle = 0.13 * static_cast<double>(index + 1);
    DualUMatrix6d plane_rotation = DualUMatrix6d::Identity();
    plane_rotation(index, index) = std::cos(angle);
    plane_rotation(index + 1, index + 1) = std::cos(angle);
    plane_rotation(index, index + 1) = -std::sin(angle);
    plane_rotation(index + 1, index) = std::sin(angle);
    u_obs.curvature_eigenvectors *= plane_rotation;
  }

  DualUMatrix6d baseline = DualUMatrix6d::Zero();
  baseline.diagonal() << 0.04, 0.05, 0.06, 0.01, 0.012, 0.014;
  UobsCovarianceInflation result;
  std::string reason;
  require(buildUobsInflatedPoseMeasurementCovariance(u_obs, map_T_lidar,
      map_T_imu_prediction, map_T_imu_measurement, baseline, 0.25,
      &result, &reason), "U_obs covariance mapping rejected valid inputs");
  require(result.valid && result.applied && reason.empty(),
      "valid weak-direction spectrum did not produce an inflation");
  require((result.effective_covariance - result.effective_covariance.transpose()).norm() < 1e-12,
      "effective measurement covariance is not symmetric");
  Eigen::LLT<DualUMatrix6d> effective_factor(result.effective_covariance);
  require(effective_factor.info() == Eigen::Success,
      "effective measurement covariance is not positive definite");
  Eigen::SelfAdjointEigenSolver<DualUMatrix6d> added_eigenvalues(
      result.effective_covariance - baseline);
  require(added_eigenvalues.info() == Eigen::Success &&
      added_eigenvalues.eigenvalues().minCoeff() >= -1e-11,
      "U_obs reduced baseline measurement covariance in some direction");

  const DualUMatrix6d analytic = result.chart_to_residual_jacobian;
  Eigen::Matrix<double, 6, 6> finite_difference;
  const double step = 2e-7;
  for (int column = 0; column < 6; ++column) {
    DualUVector6d eta = DualUVector6d::Zero();
    eta(column) = step;
    const Pose3d plus_lidar = applyMapProductChartIncrement(
        map_T_lidar, eta, length_scale);
    const Pose3d minus_lidar = applyMapProductChartIncrement(
        map_T_lidar, -eta, length_scale);
    const Pose3d plus_imu = composePose(plus_lidar, lidar_T_imu);
    const Pose3d minus_imu = composePose(minus_lidar, lidar_T_imu);
    finite_difference.col(column) =
        (exactPoseResidual(map_T_imu_prediction, plus_imu) -
         exactPoseResidual(map_T_imu_prediction, minus_imu)) / (2.0 * step);
  }
  require((finite_difference - analytic).norm() < 2e-6,
      "chart-to-EKF residual Jacobian disagrees with central finite differences");

  const DualUMatrix6d residual_to_chart = analytic.inverse();
  const DualUMatrix6d baseline_chart = residual_to_chart * baseline *
      residual_to_chart.transpose();
  const DualUMatrix6d effective_chart = residual_to_chart * result.effective_covariance *
      residual_to_chart.transpose();
  for (int index = 0; index < 6; ++index) {
    const DualUVector6d q = u_obs.curvature_eigenvectors.col(index);
    const double before = q.dot(baseline_chart * q);
    const double after = q.dot(effective_chart * q);
    require(std::abs(after / before - result.directional_variance_inflation(index)) < 1e-9,
        "chart directional variance did not follow the declared bounded inflation");
  }
  require(result.directional_variance_inflation.minCoeff() >= 1.0 &&
      result.directional_variance_inflation.maxCoeff() <= 4.0 + 1e-12 &&
      std::abs(result.directional_variance_inflation(5) - 1.0) < 1e-12,
      "directional inflation exceeded the fixed floor/cap contract");
}

void testUobsInvalidFailsClosedToBaselineCovariance() {
  const Pose3d map_T_lidar = poseFrom(Eigen::Vector3d(0.5, 0.1, -0.2),
      Eigen::Vector3d::UnitZ(), 0.2);
  const Pose3d map_T_imu = poseFrom(Eigen::Vector3d(0.6, 0.0, -0.1),
      Eigen::Vector3d::UnitZ(), 0.25);
  DualUMatrix6d baseline = DualUMatrix6d::Identity();
  baseline.diagonal() << 0.04, 0.04, 0.04, 0.01, 0.01, 0.01;
  WithinBasinObservability invalid;
  invalid.valid = true;
  invalid.locally_convex = false;
  invalid.length_scale_m = 0.8;
  UobsCovarianceInflation result;
  std::string reason;
  require(buildUobsInflatedPoseMeasurementCovariance(invalid, map_T_lidar,
      map_T_imu, map_T_imu, baseline, 0.25, &result, &reason),
      "invalid U_obs should fail closed rather than reject the baseline measurement");
  require(result.valid && !result.applied &&
      result.status == "UOBS_INVALID_OR_NONCONVEX_BASELINE_COVARIANCE_USED" &&
      result.effective_covariance.isApprox(baseline, 0.0),
      "invalid U_obs did not preserve the exact baseline covariance");
}

NdtObjectiveProvenance matchingObjective() {
  NdtObjectiveProvenance p;
  p.map_sha256 = "map-sha";
  p.source_cloud_hash = 101;
  p.source_point_count = 1400;
  p.target_point_count = 549606;
  p.configured_resolution_m = 0.8;
  p.actual_target_grid_leaf_m = Eigen::Vector3d::Constant(0.8);
  p.step_size = 0.08;
  p.transformation_epsilon = 1e-5;
  p.maximum_iterations = 80;
  return p;
}

NonlocalReliability classify(const NdtObjectiveProvenance& objective,
    int planned, int attempted, int converged, std::vector<int> cluster_support,
    int selected_cluster_index = 0) {
  NonlocalSearchEvidence evidence;
  evidence.provenance = objective;
  evidence.finite_seed_domain = "deterministic-fixed-seed-set";
  evidence.planned_seed_count = planned;
  evidence.attempted_seed_count = attempted;
  evidence.converged_seed_count = converged;
  for (std::size_t index = 0; index < cluster_support.size(); ++index)
    evidence.represented_clusters.push_back(
        {cluster_support[index], static_cast<int>(index) == selected_cluster_index});
  return classifyCandidateConditionedNonlocalEvidence(objective, evidence);
}

void testCandidateConditionedStatusesAndProvenance() {
  const NdtObjectiveProvenance objective = matchingObjective();
  const auto single = classify(objective, 10, 10, 10, {10});
  require(single.status == NonlocalStatus::SINGLE_REPRESENTED &&
      single.valid_for_objective && single.finite_seed_domain_exhausted,
      "single represented basin classification failed");
  require(single.unrepresented_basin_possible &&
      !single.exact_global_completeness_proven && single.selected_basin_supported,
      "finite seed set falsely claimed global completeness");

  const auto multi = classify(objective, 10, 10, 10, {5, 5});
  require(multi.status == NonlocalStatus::MULTI_REPRESENTED &&
      multi.supported_cluster_count == 2 && multi.selected_basin_supported,
      "multiple represented basins were not distinguished");

  const auto incomplete = classify(objective, 10, 8, 8, {8});
  require(incomplete.status == NonlocalStatus::POSSIBLY_UNREPRESENTED &&
      incomplete.search_coverage_incomplete,
      "truncated candidate search did not retain unrepresented-basin risk");

  const auto small_cluster = classify(objective, 10, 10, 10, {8, 2});
  require(small_cluster.status == NonlocalStatus::POSSIBLY_UNREPRESENTED &&
      small_cluster.subthreshold_cluster_count == 1 &&
      small_cluster.cluster_fraction_of_attempted.size() == 2,
      "sub-threshold cluster evidence was discarded");

  const auto missing_cluster = classify(objective, 10, 10, 10, {8});
  require(missing_cluster.status == NonlocalStatus::INDETERMINATE &&
      missing_cluster.diagnostic == "CONVERGED_TERMINAL_CLUSTER_COVERAGE_INCOMPLETE",
      "unclustered converged terminals were silently omitted from the basin evidence");

  const auto nominal_missing = classify(objective, 10, 10, 10, {5, 5}, -1);
  require(nominal_missing.status == NonlocalStatus::POSSIBLY_UNREPRESENTED &&
      nominal_missing.diagnostic == "SELECTED_TERMINAL_NOT_REPRESENTED_IN_CANDIDATE_SET",
      "unrelated represented clusters were mistaken for the selected nominal basin");

  NdtObjectiveProvenance old_grid = objective;
  old_grid.actual_target_grid_leaf_m.setConstant(1.0);
  NonlocalSearchEvidence old_evidence;
  old_evidence.provenance = old_grid;
  old_evidence.finite_seed_domain = "old-grid-fixed-seeds";
  old_evidence.planned_seed_count = 10;
  old_evidence.attempted_seed_count = 10;
  old_evidence.converged_seed_count = 10;
  old_evidence.represented_clusters.push_back({10});
  const auto mismatch = classifyCandidateConditionedNonlocalEvidence(objective, old_evidence);
  require(mismatch.status == NonlocalStatus::INDETERMINATE &&
      mismatch.diagnostic.find("actual_target_grid_leaf_m") != std::string::npos,
      "objective/grid provenance mismatch was not fail-closed");
}

void testNoGroundTruthInputSurface() {
  // This API accepts only registration objective/provenance and candidate
  // terminal support. No truth/label object is part of the computation.
  const auto objective = matchingObjective();
  const auto result = classify(objective, 4, 4, 4, {4});
  require(result.status == NonlocalStatus::POSSIBLY_UNREPRESENTED,
      "GT-free candidate interface failed its empty/under-supported case");
}
}  // namespace

int main() {
  try {
    testPullbackAndObjectiveSignScale();
    testChartSingularityFailsClosed();
    testEulerBranchContinuityNearZeroPitch();
    testLidarImuReferenceTransport();
    testUobsChartToEkfResidualJacobian();
    testUobsInvalidFailsClosedToBaselineCovariance();
    testCandidateConditionedStatusesAndProvenance();
    testNoGroundTruthInputSurface();
    std::cout << "dual_u_architecture_test PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "dual_u_architecture_test FAIL: " << error.what() << '\n';
    return 1;
  }
}
