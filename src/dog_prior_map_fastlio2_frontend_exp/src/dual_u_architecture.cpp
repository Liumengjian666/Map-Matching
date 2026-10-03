#include "dog_prior_map_fastlio2_frontend_exp/dual_u_architecture.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
constexpr double kPi = 3.14159265358979323846;

bool reject(std::string* reason, const std::string& status) {
  if (reason) *reason = status;
  return false;
}

bool validRotation(const Eigen::Matrix3d& rotation) {
  return rotation.allFinite() &&
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-8 &&
      std::abs(rotation.determinant() - 1.0) <= 1e-8;
}

Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(),
          vector.z(), 0.0, -vector.x(),
          -vector.y(), vector.x(), 0.0;
  return skew;
}

bool validPose(const Pose3d& pose) {
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12)
    return false;
  return validRotation(pose.orientation.normalized().toRotationMatrix());
}

Eigen::Matrix3d so3LeftJacobianInverse(const Eigen::Vector3d& tangent) {
  const double angle = tangent.norm();
  const Eigen::Matrix3d tangent_skew = skewMatrix(tangent);
  const double coefficient = angle < 1e-5
      ? 1.0 / 12.0 + angle * angle / 720.0
      : 1.0 / (angle * angle) -
          (1.0 + std::cos(angle)) / (2.0 * angle * std::sin(angle));
  return Eigen::Matrix3d::Identity() - 0.5 * tangent_skew +
      coefficient * tangent_skew * tangent_skew;
}

Eigen::Matrix3d pclEulerRotation(const Eigen::Vector3d& angles) {
  return (Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

bool unwrapEulerNear(const Eigen::Vector3d& raw, const Eigen::Vector3d& reference,
                     Eigen::Vector3d* unwrapped) {
  if (!unwrapped || !raw.allFinite() || !reference.allFinite()) return false;
  double best_squared_distance = std::numeric_limits<double>::infinity();
  bool found = false;
  // XYZ Tait-Bryan angles have a second equivalent branch in addition to
  // independent 2*pi wraps. Eigen::eulerAngles() can switch between these
  // branches around zero pitch, which is common in planar mobile-robot motion.
  for (int branch = 0; branch < 2; ++branch) {
    Eigen::Vector3d candidate = raw;
    if (branch == 1) {
      candidate.x() += kPi;
      candidate.y() = kPi - candidate.y();
      candidate.z() += kPi;
    }
    for (int axis = 0; axis < 3; ++axis)
      candidate(axis) += 2.0 * kPi *
          std::round((reference(axis) - candidate(axis)) / (2.0 * kPi));
    const double squared_distance = (candidate - reference).squaredNorm();
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      *unwrapped = candidate;
      found = true;
    }
  }
  return found && unwrapped->allFinite();
}

bool closeRelative(double lhs, double rhs, double tolerance = 1e-8) {
  return std::isfinite(lhs) && std::isfinite(rhs) &&
      std::abs(lhs - rhs) <= tolerance * std::max({1.0, std::abs(lhs), std::abs(rhs)});
}

bool sameProvenance(const NdtObjectiveProvenance& a,
                    const NdtObjectiveProvenance& b,
                    std::string* mismatch) {
  const auto fail = [mismatch](const char* field) {
    if (mismatch) *mismatch = std::string("OBJECTIVE_PROVENANCE_MISMATCH:") + field;
    return false;
  };
  if (a.backend.empty() || a.backend != b.backend) return fail("backend");
  if (a.map_sha256.empty() || a.map_sha256 != b.map_sha256) return fail("map_sha256");
  if (a.source_cloud_hash != b.source_cloud_hash) return fail("source_cloud_hash");
  if (a.source_point_count != b.source_point_count) return fail("source_point_count");
  if (a.target_point_count != b.target_point_count) return fail("target_point_count");
  if (!closeRelative(a.configured_resolution_m, b.configured_resolution_m, 1e-6))
    return fail("configured_resolution_m");
  if (!a.actual_target_grid_leaf_m.allFinite() || !b.actual_target_grid_leaf_m.allFinite() ||
      (a.actual_target_grid_leaf_m - b.actual_target_grid_leaf_m).cwiseAbs().maxCoeff() > 1e-6)
    return fail("actual_target_grid_leaf_m");
  if (!closeRelative(a.step_size, b.step_size)) return fail("step_size");
  if (!closeRelative(a.transformation_epsilon, b.transformation_epsilon))
    return fail("transformation_epsilon");
  if (a.maximum_iterations != b.maximum_iterations) return fail("maximum_iterations");
  if (mismatch) mismatch->clear();
  return true;
}

}  // namespace

bool buildMapProductChartPullback(const Pose3d& pose,
    const Eigen::Vector3d& pcl_euler_xyz, double length_scale_m,
    ProductChartPullback* output, std::string* reason) {
  if (reason) reason->clear();
  if (!output) return reject(reason, "NULL_PULLBACK_OUTPUT");
  *output = ProductChartPullback();
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() < 1e-12 ||
      !pcl_euler_xyz.allFinite() || !std::isfinite(length_scale_m) || length_scale_m <= 0.0)
    return reject(reason, "INVALID_CHART_INPUT");

  const Eigen::Matrix3d rotation = pose.orientation.normalized().toRotationMatrix();
  if (!validRotation(rotation)) return reject(reason, "INVALID_POSE_ROTATION");
  output->base_rotation_reconstruction_error =
      (pclEulerRotation(pcl_euler_xyz) - rotation).norm();
  if (!std::isfinite(output->base_rotation_reconstruction_error) ||
      output->base_rotation_reconstruction_error > 1e-5)
    return reject(reason, "PCL_EULER_BASE_BRANCH_MISMATCH");

  const Eigen::Vector3d base_angles = pcl_euler_xyz;
  const auto parameterAt = [&](const DualUVector6d& eta, Eigen::Matrix<double, 6, 1>* p) {
    if (!p || !eta.allFinite()) return false;
    p->head<3>() = pose.position + length_scale_m * eta.head<3>();
    const double angle = eta.tail<3>().norm();
    const Eigen::Matrix3d delta_rotation = angle < 1e-14
        ? Eigen::Matrix3d::Identity()
        : Eigen::AngleAxisd(angle, eta.tail<3>() / angle).toRotationMatrix();
    const Eigen::Vector3d raw = (delta_rotation * rotation).eulerAngles(0, 1, 2);
    Eigen::Vector3d unwrapped;
    if (!unwrapEulerNear(raw, base_angles, &unwrapped)) return false;
    const Eigen::Matrix3d reconstructed = pclEulerRotation(unwrapped);
    if (!unwrapped.allFinite() || (reconstructed - delta_rotation * rotation).norm() > 1e-7 ||
        (unwrapped - base_angles).norm() > 0.25)
      return false;
    p->tail<3>() = unwrapped;
    return p->allFinite();
  };

  output->dp_deta.setZero();
  output->dp_deta.block<3, 3>(0, 0) = length_scale_m * Eigen::Matrix3d::Identity();
  constexpr double kJacobianStep = 1e-5;
  for (int axis = 0; axis < 3; ++axis) {
    DualUVector6d positive = DualUVector6d::Zero();
    DualUVector6d negative = DualUVector6d::Zero();
    positive(3 + axis) = kJacobianStep;
    negative(3 + axis) = -kJacobianStep;
    Eigen::Matrix<double, 6, 1> p_positive, p_negative;
    if (!parameterAt(positive, &p_positive) || !parameterAt(negative, &p_negative))
      return reject(reason, "EULER_BRANCH_DISCONTINUITY_IN_JACOBIAN");
    output->dp_deta.block<3, 1>(3, 3 + axis) =
        (p_positive.tail<3>() - p_negative.tail<3>()) / (2.0 * kJacobianStep);
  }
  const Eigen::Matrix3d rotation_jacobian = output->dp_deta.block<3, 3>(3, 3);
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation_jacobian);
  if (!svd.singularValues().allFinite() || svd.singularValues().minCoeff() <= 1e-10)
    return reject(reason, "SINGULAR_EULER_PRODUCT_CHART");
  output->euler_chart_condition = svd.singularValues().maxCoeff() /
      svd.singularValues().minCoeff();
  if (!std::isfinite(output->euler_chart_condition) || output->euler_chart_condition > 1e4)
    return reject(reason, "ILL_CONDITIONED_EULER_PRODUCT_CHART");

  output->d2p_deta2.assign(6, DualUMatrix6d::Zero());
  constexpr double kHessianStep = 1e-3;
  for (int a = 0; a < 3; ++a) {
    DualUVector6d positive = DualUVector6d::Zero();
    DualUVector6d negative = DualUVector6d::Zero();
    positive(3 + a) = kHessianStep;
    negative(3 + a) = -kHessianStep;
    Eigen::Matrix<double, 6, 1> p_positive, p_negative, p_zero;
    if (!parameterAt(positive, &p_positive) || !parameterAt(negative, &p_negative) ||
        !parameterAt(DualUVector6d::Zero(), &p_zero))
      return reject(reason, "EULER_BRANCH_DISCONTINUITY_IN_HESSIAN");
    output->d2p_deta2[3](3 + a, 3 + a) =
        (p_positive(3) - 2.0 * p_zero(3) + p_negative(3)) /
        (kHessianStep * kHessianStep);
    output->d2p_deta2[4](3 + a, 3 + a) =
        (p_positive(4) - 2.0 * p_zero(4) + p_negative(4)) /
        (kHessianStep * kHessianStep);
    output->d2p_deta2[5](3 + a, 3 + a) =
        (p_positive(5) - 2.0 * p_zero(5) + p_negative(5)) /
        (kHessianStep * kHessianStep);
    for (int b = a + 1; b < 3; ++b) {
      DualUVector6d pp = DualUVector6d::Zero(), pm = DualUVector6d::Zero();
      DualUVector6d mp = DualUVector6d::Zero(), mm = DualUVector6d::Zero();
      pp(3 + a) = kHessianStep; pp(3 + b) = kHessianStep;
      pm(3 + a) = kHessianStep; pm(3 + b) = -kHessianStep;
      mp(3 + a) = -kHessianStep; mp(3 + b) = kHessianStep;
      mm(3 + a) = -kHessianStep; mm(3 + b) = -kHessianStep;
      Eigen::Matrix<double, 6, 1> ppp, ppm, pmp, pmm;
      if (!parameterAt(pp, &ppp) || !parameterAt(pm, &ppm) ||
          !parameterAt(mp, &pmp) || !parameterAt(mm, &pmm))
        return reject(reason, "EULER_BRANCH_DISCONTINUITY_IN_MIXED_HESSIAN");
      for (int parameter = 3; parameter < 6; ++parameter) {
        const double value = (ppp(parameter) - ppm(parameter) -
                              pmp(parameter) + pmm(parameter)) /
            (4.0 * kHessianStep * kHessianStep);
        output->d2p_deta2[static_cast<std::size_t>(parameter)](3 + a, 3 + b) = value;
        output->d2p_deta2[static_cast<std::size_t>(parameter)](3 + b, 3 + a) = value;
      }
    }
  }
  for (const auto& second_derivative : output->d2p_deta2)
    if (!second_derivative.allFinite())
      return reject(reason, "NONFINITE_PRODUCT_CHART_SECOND_DERIVATIVE");
  output->valid = output->dp_deta.allFinite();
  output->status = output->valid ? "PASS_MAP_PRODUCT_CHART" : "NONFINITE_PRODUCT_CHART";
  if (!output->valid) return reject(reason, output->status);
  return true;
}

bool analyzeWithinBasinObservability(const PclNdtScoreJet& jet,
    const Pose3d& pose, const Eigen::Vector3d& actual_leaf_m,
    double configured_resolution_m, WithinBasinObservability* output,
    std::string* reason) {
  if (reason) reason->clear();
  if (!output) return reject(reason, "NULL_UOBS_OUTPUT");
  *output = WithinBasinObservability();
  if (!jet.valid || jet.status != "PASS_PCL_SCORE_JET" || jet.source_point_count == 0 ||
      !std::isfinite(jet.score_sum) || !jet.score_gradient.allFinite() ||
      !jet.score_hessian.allFinite() || !jet.pcl_euler_xyz.allFinite())
    return reject(reason, "INVALID_PCL_SCORE_JET");
  if (!actual_leaf_m.allFinite() || (actual_leaf_m.array() <= 0.0).any() ||
      !std::isfinite(configured_resolution_m) || configured_resolution_m <= 0.0)
    return reject(reason, "INVALID_NDT_GRID_PROVENANCE");
  if ((actual_leaf_m.array() - configured_resolution_m).abs().maxCoeff() > 1e-6 ||
      (actual_leaf_m.array() - actual_leaf_m.x()).abs().maxCoeff() > 1e-6)
    return reject(reason, "TARGET_GRID_RESOLUTION_MISMATCH");

  ProductChartPullback chart;
  if (!buildMapProductChartPullback(pose, jet.pcl_euler_xyz,
          actual_leaf_m.x(), &chart, reason)) {
    output->status = chart.status;
    return false;
  }
  const double n = static_cast<double>(jet.source_point_count);
  const DualUVector6d objective_gradient = -jet.score_gradient / n;
  const DualUMatrix6d objective_hessian_native = -jet.score_hessian / n;
  output->score_hessian_relative_asymmetry =
      (objective_hessian_native - objective_hessian_native.transpose()).norm() /
      std::max(1.0, objective_hessian_native.norm());
  if (!std::isfinite(output->score_hessian_relative_asymmetry) ||
      output->score_hessian_relative_asymmetry > 1e-6)
    return reject(reason, "PCL_SCORE_HESSIAN_MATERIALLY_ASYMMETRIC");
  const DualUMatrix6d hessian_native =
      (0.5 * (objective_hessian_native + objective_hessian_native.transpose())).eval();
  DualUMatrix6d pulled = chart.dp_deta.transpose() * hessian_native * chart.dp_deta;
  for (int parameter = 0; parameter < 6; ++parameter)
    pulled += objective_gradient(parameter) *
        chart.d2p_deta2[static_cast<std::size_t>(parameter)];
  output->pulled_hessian_relative_asymmetry =
      (pulled - pulled.transpose()).norm() / std::max(1.0, pulled.norm());
  if (!pulled.allFinite() || !std::isfinite(output->pulled_hessian_relative_asymmetry) ||
      output->pulled_hessian_relative_asymmetry > 1e-6)
    return reject(reason, "PULLED_HESSIAN_MATERIALLY_ASYMMETRIC");
  output->objective_gradient = chart.dp_deta.transpose() * objective_gradient;
  output->local_curvature = (0.5 * (pulled + pulled.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<DualUMatrix6d> eigensolver(output->local_curvature);
  if (eigensolver.info() != Eigen::Success || !eigensolver.eigenvalues().allFinite() ||
      !eigensolver.eigenvectors().allFinite())
    return reject(reason, "UOBS_EIGENSOLVE_FAILED");

  output->length_scale_m = actual_leaf_m.x();
  output->objective_per_source = -jet.score_sum / n;
  output->curvature_eigenvalues = eigensolver.eigenvalues();
  output->curvature_eigenvectors = eigensolver.eigenvectors();
  output->euler_chart_condition = chart.euler_chart_condition;
  const double spectral_scale = std::max(1.0,
      output->curvature_eigenvalues.cwiseAbs().maxCoeff());
  output->locally_convex = output->curvature_eigenvalues.minCoeff() >=
      -1e-10 * spectral_scale;
  output->valid = output->objective_gradient.allFinite() &&
      output->local_curvature.allFinite();
  output->status = output->locally_convex ? "PASS_LOCAL_NDT_CURVATURE" :
                                           "LOCAL_NDT_CURVATURE_NONCONVEX";
  if (!output->valid) return reject(reason, "NONFINITE_UOBS_RESULT");
  return true;
}

Pose3d applyMapProductChartIncrement(const Pose3d& pose,
    const DualUVector6d& eta, double length_scale_m) {
  Pose3d result = pose;
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
      !eta.allFinite() || !std::isfinite(length_scale_m) || length_scale_m <= 0.0) {
    result.position.setConstant(std::numeric_limits<double>::quiet_NaN());
    result.orientation.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
    return result;
  }
  const double angle = eta.tail<3>().norm();
  const Eigen::Quaterniond delta = angle < 1e-14
      ? Eigen::Quaterniond::Identity()
      : Eigen::Quaterniond(Eigen::AngleAxisd(angle, eta.tail<3>() / angle));
  result.position = pose.position + length_scale_m * eta.head<3>();
  result.orientation = (delta * pose.orientation.normalized()).normalized();
  return result;
}

DualUMatrix6d lidarOriginToImuOriginTangentMap(
    const Eigen::Vector3d& r_map_imu_to_lidar, double length_scale_m) {
  DualUMatrix6d map = DualUMatrix6d::Identity();
  if (!r_map_imu_to_lidar.allFinite() || !std::isfinite(length_scale_m) ||
      length_scale_m <= 0.0)
    return DualUMatrix6d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d skew;
  skew << 0.0, -r_map_imu_to_lidar.z(), r_map_imu_to_lidar.y(),
          r_map_imu_to_lidar.z(), 0.0, -r_map_imu_to_lidar.x(),
          -r_map_imu_to_lidar.y(), r_map_imu_to_lidar.x(), 0.0;
  // t_lidar = t_imu + delta_theta x r; hence delta_t_imu =
  // delta_t_lidar + [r]x delta_theta.
  map.block<3, 3>(0, 3) = skew / length_scale_m;
  return map;
}

bool buildUobsInflatedPoseMeasurementCovariance(
    const WithinBasinObservability& u_obs,
    const Pose3d& map_T_lidar_measurement,
    const Pose3d& map_T_imu_prediction,
    const Pose3d& map_T_imu_measurement,
    const DualUMatrix6d& baseline_covariance,
    double weak_weight_floor,
    UobsCovarianceInflation* output,
    std::string* reason) {
  if (reason) reason->clear();
  if (!output) return reject(reason, "NULL_UOBS_COVARIANCE_OUTPUT");
  *output = UobsCovarianceInflation();
  if (!baseline_covariance.allFinite() ||
      (baseline_covariance - baseline_covariance.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return reject(reason, "INVALID_BASELINE_MEASUREMENT_COVARIANCE");
  Eigen::LLT<DualUMatrix6d> baseline_factor(baseline_covariance);
  if (baseline_factor.info() != Eigen::Success ||
      baseline_factor.matrixL().toDenseMatrix().diagonal().minCoeff() <= 0.0)
    return reject(reason, "BASELINE_MEASUREMENT_COVARIANCE_NOT_SPD");

  output->baseline_covariance = baseline_covariance;
  output->effective_covariance = baseline_covariance;
  output->weak_weight_floor = weak_weight_floor;
  const auto fallback = [&](const char* status) {
    output->valid = true;
    output->applied = false;
    output->status = status;
    output->effective_covariance = baseline_covariance;
    output->chart_to_residual_jacobian.setConstant(
        std::numeric_limits<double>::quiet_NaN());
    output->relative_curvature.setConstant(std::numeric_limits<double>::quiet_NaN());
    output->directional_reliability.setConstant(std::numeric_limits<double>::quiet_NaN());
    output->directional_variance_inflation.setConstant(
        std::numeric_limits<double>::quiet_NaN());
    if (reason) *reason = status;
    return true;
  };
  if (!std::isfinite(weak_weight_floor) || weak_weight_floor <= 0.0 ||
      weak_weight_floor > 1.0)
    return fallback("INVALID_UOBS_WEIGHT_FLOOR_BASELINE_COVARIANCE_USED");
  if (!u_obs.valid || !u_obs.locally_convex || !u_obs.curvature_eigenvalues.allFinite() ||
      !u_obs.curvature_eigenvectors.allFinite() || !std::isfinite(u_obs.length_scale_m) ||
      u_obs.length_scale_m <= 0.0)
    return fallback("UOBS_INVALID_OR_NONCONVEX_BASELINE_COVARIANCE_USED");
  if (!validPose(map_T_lidar_measurement) || !validPose(map_T_imu_prediction) ||
      !validPose(map_T_imu_measurement))
    return fallback("INVALID_POSE_FOR_UOBS_MAPPING_BASELINE_COVARIANCE_USED");
  if ((u_obs.curvature_eigenvectors.transpose() * u_obs.curvature_eigenvectors -
       DualUMatrix6d::Identity()).norm() > 1e-7 ||
      (u_obs.curvature_eigenvalues.tail<5>().array() + 1e-10 <
       u_obs.curvature_eigenvalues.head<5>().array()).any())
    return fallback("INVALID_UOBS_EIGENBASIS_BASELINE_COVARIANCE_USED");

  const double curvature_reference = u_obs.curvature_eigenvalues.maxCoeff();
  const double curvature_scale = std::max(1.0,
      u_obs.curvature_eigenvalues.cwiseAbs().maxCoeff());
  if (!std::isfinite(curvature_reference) || curvature_reference <=
      64.0 * std::numeric_limits<double>::epsilon() * curvature_scale)
    return fallback("UOBS_CURVATURE_BELOW_NUMERICAL_RESOLUTION_BASELINE_COVARIANCE_USED");

  // IKFoM's exact pose residual is [p_meas-p_pred, Log(R_pred^-1 R_meas)].
  // A map-spatial left perturbation of the measured LiDAR pose moves the IMU
  // origin by delta_t + delta_theta x (p_imu-p_lidar). The rotation block is
  // the differential of Log under a left perturbation of R_pred^-1 R_meas.
  Eigen::Quaterniond residual_rotation =
      (map_T_imu_prediction.orientation.normalized().conjugate() *
       map_T_imu_measurement.orientation.normalized()).normalized();
  if (residual_rotation.w() < 0.0) residual_rotation.coeffs() *= -1.0;
  const Eigen::AngleAxisd residual_angle_axis(residual_rotation);
  const Eigen::Vector3d rotation_residual =
      residual_angle_axis.axis() * residual_angle_axis.angle();
  if (!rotation_residual.allFinite() ||
      residual_angle_axis.angle() >= kPi - 1e-5)
    return fallback("SO3_RESIDUAL_NEAR_LOG_CUT_BASELINE_COVARIANCE_USED");

  const Eigen::Vector3d r_lidar_to_imu_map =
      map_T_imu_measurement.position - map_T_lidar_measurement.position;
  DualUMatrix6d chart_to_residual = DualUMatrix6d::Zero();
  chart_to_residual.topLeftCorner<3, 3>() =
      u_obs.length_scale_m * Eigen::Matrix3d::Identity();
  chart_to_residual.topRightCorner<3, 3>() = -skewMatrix(r_lidar_to_imu_map);
  chart_to_residual.bottomRightCorner<3, 3>() =
      so3LeftJacobianInverse(rotation_residual) *
      map_T_imu_prediction.orientation.normalized().toRotationMatrix().transpose();
  if (!chart_to_residual.allFinite())
    return fallback("NONFINITE_UOBS_CHART_TO_RESIDUAL_JACOBIAN_BASELINE_COVARIANCE_USED");

  Eigen::FullPivLU<DualUMatrix6d> chart_factor(chart_to_residual);
  if (chart_factor.rank() != 6)
    return fallback("SINGULAR_UOBS_CHART_TO_RESIDUAL_JACOBIAN_BASELINE_COVARIANCE_USED");
  const DualUMatrix6d residual_to_chart = chart_factor.inverse();
  const DualUMatrix6d baseline_chart_covariance =
      residual_to_chart * baseline_covariance * residual_to_chart.transpose();
  const DualUMatrix6d symmetric_baseline_chart_covariance =
      (0.5 * (baseline_chart_covariance + baseline_chart_covariance.transpose())).eval();
  if (!symmetric_baseline_chart_covariance.allFinite())
    return fallback("NONFINITE_CHART_COVARIANCE_BASELINE_COVARIANCE_USED");

  DualUVector6d added_chart_variance = DualUVector6d::Zero();
  for (int index = 0; index < 6; ++index) {
    const double relative = std::max(0.0,
        u_obs.curvature_eigenvalues(index) / curvature_reference);
    const double reliability = std::max(weak_weight_floor, std::min(1.0, relative));
    const DualUVector6d direction = u_obs.curvature_eigenvectors.col(index);
    const double baseline_directional_variance =
        direction.dot(symmetric_baseline_chart_covariance * direction);
    if (!std::isfinite(baseline_directional_variance) ||
        baseline_directional_variance <= 0.0)
      return fallback("INVALID_BASELINE_DIRECTIONAL_VARIANCE_BASELINE_COVARIANCE_USED");
    output->relative_curvature(index) = relative;
    output->directional_reliability(index) = reliability;
    output->directional_variance_inflation(index) = 1.0 / reliability;
    added_chart_variance(index) =
        (1.0 / reliability - 1.0) * baseline_directional_variance;
  }

  const DualUMatrix6d added_chart_covariance =
      u_obs.curvature_eigenvectors * added_chart_variance.asDiagonal() *
      u_obs.curvature_eigenvectors.transpose();
  const DualUMatrix6d added_residual_covariance =
      chart_to_residual * added_chart_covariance * chart_to_residual.transpose();
  output->chart_to_residual_jacobian = chart_to_residual;
  output->effective_covariance = baseline_covariance +
      0.5 * (added_residual_covariance + added_residual_covariance.transpose());
  output->effective_covariance =
      (0.5 * (output->effective_covariance + output->effective_covariance.transpose())).eval();
  if (!output->effective_covariance.allFinite())
    return fallback("NONFINITE_EFFECTIVE_COVARIANCE_BASELINE_COVARIANCE_USED");
  Eigen::LLT<DualUMatrix6d> effective_factor(output->effective_covariance);
  Eigen::SelfAdjointEigenSolver<DualUMatrix6d> added_eigenvalues(
      output->effective_covariance - baseline_covariance);
  if (effective_factor.info() != Eigen::Success || added_eigenvalues.info() != Eigen::Success ||
      !added_eigenvalues.eigenvalues().allFinite() ||
      added_eigenvalues.eigenvalues().minCoeff() < -1e-10 *
          std::max(1.0, output->effective_covariance.norm()))
    return fallback("UOBS_COVARIANCE_POSTCONDITION_FAILED_BASELINE_COVARIANCE_USED");

  output->valid = true;
  output->applied = output->directional_variance_inflation.maxCoeff() > 1.0 + 1e-12;
  output->status = output->applied ? "UOBS_DIRECTIONAL_COVARIANCE_INFLATION_APPLIED" :
                                     "UOBS_NO_DIRECTIONAL_INFLATION";
  return true;
}

const char* nonlocalStatusName(NonlocalStatus status) {
  switch (status) {
    case NonlocalStatus::INDETERMINATE: return "INDETERMINATE";
    case NonlocalStatus::POSSIBLY_UNREPRESENTED: return "POSSIBLY_UNREPRESENTED";
    case NonlocalStatus::SINGLE_REPRESENTED: return "SINGLE_REPRESENTED";
    case NonlocalStatus::MULTI_REPRESENTED: return "MULTI_REPRESENTED";
  }
  return "INVALID_NONLOCAL_STATUS";
}

NonlocalReliability classifyCandidateConditionedNonlocalEvidence(
    const NdtObjectiveProvenance& selected_objective,
    const NonlocalSearchEvidence& evidence,
    int minimum_cluster_seeds, double minimum_fraction_of_converged) {
  NonlocalReliability result;
  result.planned_seed_count = evidence.planned_seed_count;
  result.attempted_seed_count = evidence.attempted_seed_count;
  result.converged_seed_count = evidence.converged_seed_count;
  result.represented_cluster_count = static_cast<int>(evidence.represented_clusters.size());
  result.exact_global_completeness_proven = false;
  result.unrepresented_basin_possible = true;
  result.finite_seed_domain_exhausted = evidence.planned_seed_count > 0 &&
      evidence.attempted_seed_count == evidence.planned_seed_count;
  result.search_coverage_incomplete = !result.finite_seed_domain_exhausted;

  std::string provenance_status;
  if (!sameProvenance(selected_objective, evidence.provenance, &provenance_status)) {
    result.status = NonlocalStatus::INDETERMINATE;
    result.diagnostic = provenance_status;
    return result;
  }
  if (minimum_cluster_seeds <= 0 || !std::isfinite(minimum_fraction_of_converged) ||
      minimum_fraction_of_converged <= 0.0 || minimum_fraction_of_converged > 1.0 ||
      evidence.finite_seed_domain.empty() || evidence.planned_seed_count <= 0 ||
      evidence.attempted_seed_count < 0 ||
      evidence.attempted_seed_count > evidence.planned_seed_count ||
      evidence.converged_seed_count < 0 ||
      evidence.converged_seed_count > evidence.attempted_seed_count) {
    result.status = NonlocalStatus::INDETERMINATE;
    result.diagnostic = "INVALID_CANDIDATE_COVERAGE_METADATA";
    return result;
  }
  if (evidence.converged_seed_count == 0 && !evidence.represented_clusters.empty()) {
    result.status = NonlocalStatus::INDETERMINATE;
    result.diagnostic = "CLUSTERS_WITH_ZERO_CONVERGED_SEEDS";
    return result;
  }

  int clustered_converged = 0;
  int selected_membership_count = 0;
  for (std::size_t cluster_index = 0;
       cluster_index < evidence.represented_clusters.size(); ++cluster_index) {
    const BasinSupportEvidence& basin = evidence.represented_clusters[cluster_index];
    if (basin.converged_seed_count <= 0 ||
        basin.converged_seed_count > evidence.converged_seed_count) {
      result.status = NonlocalStatus::INDETERMINATE;
      result.diagnostic = "INVALID_CLUSTER_SUPPORT_COUNT";
      return result;
    }
    if (basin.contains_selected_terminal) {
      ++selected_membership_count;
      result.selected_terminal_cluster_index = static_cast<int>(cluster_index);
    }
    clustered_converged += basin.converged_seed_count;
    const double fraction_converged = static_cast<double>(basin.converged_seed_count) /
        static_cast<double>(evidence.converged_seed_count);
    const double fraction_attempted = evidence.attempted_seed_count > 0
        ? static_cast<double>(basin.converged_seed_count) /
              static_cast<double>(evidence.attempted_seed_count)
        : 0.0;
    result.cluster_support_converged.push_back(basin.converged_seed_count);
    result.cluster_fraction_of_converged.push_back(fraction_converged);
    result.cluster_fraction_of_attempted.push_back(fraction_attempted);
    if (basin.converged_seed_count >= minimum_cluster_seeds &&
        fraction_converged >= minimum_fraction_of_converged)
      ++result.supported_cluster_count;
    else
      ++result.subthreshold_cluster_count;
  }
  if (clustered_converged > evidence.converged_seed_count) {
    result.status = NonlocalStatus::INDETERMINATE;
    result.diagnostic = "CLUSTER_SUPPORT_EXCEEDS_CONVERGED_SEEDS";
    return result;
  }
  if (clustered_converged != evidence.converged_seed_count) {
    result.status = NonlocalStatus::INDETERMINATE;
    result.diagnostic = "CONVERGED_TERMINAL_CLUSTER_COVERAGE_INCOMPLETE";
    return result;
  }
  if (selected_membership_count > 1) {
    result.status = NonlocalStatus::INDETERMINATE;
    result.diagnostic = "SELECTED_TERMINAL_ASSIGNED_TO_MULTIPLE_CLUSTERS";
    return result;
  }
  if (selected_membership_count == 0) {
    result.status = NonlocalStatus::POSSIBLY_UNREPRESENTED;
    result.diagnostic = "SELECTED_TERMINAL_NOT_REPRESENTED_IN_CANDIDATE_SET";
    return result;
  }
  result.selected_basin_supported =
      result.cluster_support_converged[static_cast<std::size_t>(
          result.selected_terminal_cluster_index)] >= minimum_cluster_seeds &&
      result.cluster_fraction_of_converged[static_cast<std::size_t>(
          result.selected_terminal_cluster_index)] >= minimum_fraction_of_converged;
  result.valid_for_objective = true;
  if (!result.selected_basin_supported) {
    result.status = NonlocalStatus::POSSIBLY_UNREPRESENTED;
    result.diagnostic = "SELECTED_BASIN_CLUSTER_BELOW_SUPPORT_THRESHOLD";
  } else if (result.supported_cluster_count >= 2) {
    result.status = NonlocalStatus::MULTI_REPRESENTED;
    result.diagnostic = "MULTIPLE_SUPPORTED_CLUSTERS_IN_SUPPLIED_CANDIDATE_SET";
  } else if (result.supported_cluster_count == 1 &&
             result.finite_seed_domain_exhausted &&
             result.subthreshold_cluster_count == 0) {
    result.status = NonlocalStatus::SINGLE_REPRESENTED;
    result.diagnostic = "ONE_SUPPORTED_CLUSTER_IN_EXHAUSTED_FINITE_SEED_DOMAIN_NOT_GLOBAL_PROOF";
  } else {
    result.status = NonlocalStatus::POSSIBLY_UNREPRESENTED;
    result.diagnostic = result.search_coverage_incomplete
        ? "PLANNED_CANDIDATE_SEARCH_NOT_EXHAUSTED"
        : (result.supported_cluster_count == 0
              ? "NO_SUPPORTED_CLUSTER_IN_FINITE_CANDIDATE_SET"
              : "SUBTHRESHOLD_CLUSTER_SUPPORT_RETAINED");
  }
  return result;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
