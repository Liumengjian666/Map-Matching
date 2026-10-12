#include "dog_prior_map_localization/mature_ndt_algorithms.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace dog_prior_map_localization {
namespace {
using Matrix3d = Eigen::Matrix3d;
constexpr double kPi = 3.14159265358979323846;

bool schurComplement(const Matrix3d& eliminated, const Matrix3d& cross,
                     const Matrix3d& retained,
                     Matrix3d* output, double* condition) {
  if (!output || !condition || !eliminated.allFinite() || !cross.allFinite() ||
      !retained.allFinite()) return false;
  const Matrix3d solve_block = 0.5 * (eliminated + eliminated.transpose());
  const Matrix3d output_block = 0.5 * (retained + retained.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen(solve_block);
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()) return false;
  const double minimum_magnitude = eigen.eigenvalues().cwiseAbs().minCoeff();
  const double maximum_magnitude = eigen.eigenvalues().cwiseAbs().maxCoeff();
  if (minimum_magnitude <= 1e-12 || maximum_magnitude <= 1e-12) return false;
  *condition = maximum_magnitude / minimum_magnitude;
  if (!std::isfinite(*condition) || *condition > 1e8) return false;
  Matrix3d solved;
  bool solved_ok = false;
  Eigen::LDLT<Matrix3d> factor(solve_block);
  if (factor.info() == Eigen::Success) {
    const Matrix3d ldlt_solution = factor.solve(cross);
    const double residual = (solve_block * ldlt_solution - cross).norm() /
        std::max(1e-12, cross.norm());
    if (ldlt_solution.allFinite() && std::isfinite(residual) && residual <= 1e-6) {
      solved = ldlt_solution;
      solved_ok = true;
    }
  }
  // Eigen's LDLT uses 1x1 pivots; an invertible indefinite block can still
  // need a 2x2 pivot. Fall back to the already-computed signed eigensystem.
  // The condition gate above bounds reciprocal eigenvalues, and the residual
  // check keeps this from turning a numerically singular block into a direction.
  if (!solved_ok) {
    const Eigen::Vector3d inverse_values = eigen.eigenvalues().cwiseInverse();
    const Matrix3d spectral_inverse = eigen.eigenvectors() *
        inverse_values.asDiagonal() * eigen.eigenvectors().transpose();
    solved = spectral_inverse * cross;
    const double residual = (solve_block * solved - cross).norm() /
        std::max(1e-12, cross.norm());
    if (!solved.allFinite() || !std::isfinite(residual) || residual > 1e-6) return false;
  }
  *output = 0.5 * ((output_block - cross.transpose() * solved) +
                   (output_block - cross.transpose() * solved).transpose());
  return output->allFinite();
}

bool properPose(const Eigen::Matrix4f& pose) {
  if (!pose.allFinite() || (pose.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() > 1e-5)
    return false;
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0).cast<double>();
  return (rotation.transpose() * rotation - Matrix3d::Identity()).norm() < 1e-4 &&
      std::abs(rotation.determinant() - 1.0) < 1e-4;
}

double rotationDistanceDeg(const Eigen::Matrix3f& a, const Eigen::Matrix3f& b) {
  const Eigen::Quaterniond qa(a.cast<double>()), qb(b.cast<double>());
  if (qa.norm() < 1e-12 || qb.norm() < 1e-12) return std::numeric_limits<double>::infinity();
  return qa.normalized().angularDistance(qb.normalized()) * 180.0 / kPi;
}
}  // namespace

DcregAdaptedReport analyzeDcregAdapted(
    const dog_prior_map_fastlio2_frontend_exp::CoupledMatrix6& hessian,
    double threshold) {
  DcregAdaptedReport report;
  if (!hessian.allFinite() || !std::isfinite(threshold) || threshold <= 0.0 || threshold >= 1.0) {
    report.status = "INVALID_INPUT";
    return report;
  }
  const double asymmetry = (hessian - hessian.transpose()).norm();
  if (asymmetry > 1e-6 * std::max(1.0, hessian.norm())) {
    report.status = "HESSIAN_NOT_SYMMETRIC";
    return report;
  }

  // The shared P10 chart is [translation, map-spatial rotation]. Compute the
  // two Schur complements so each physical block is assessed after accounting
  // for coupling to the other block.
  const Matrix3d Htt = hessian.topLeftCorner<3, 3>();
  const Matrix3d Htr = hessian.topRightCorner<3, 3>();
  const Matrix3d Hrt = hessian.bottomLeftCorner<3, 3>();
  const Matrix3d Hrr = hessian.bottomRightCorner<3, 3>();
  Matrix3d translation_schur, rotation_schur;
  double translation_eliminated_condition = 0.0, rotation_eliminated_condition = 0.0;
  if (!schurComplement(Htt, Htr, Hrr, &rotation_schur, &translation_eliminated_condition) ||
      !schurComplement(Hrr, Hrt, Htt, &translation_schur, &rotation_eliminated_condition)) {
    report.status = "SCHUR_BLOCK_INVALID";
    return report;
  }
  Eigen::SelfAdjointEigenSolver<Matrix3d> translation_eigen(translation_schur);
  Eigen::SelfAdjointEigenSolver<Matrix3d> rotation_eigen(rotation_schur);
  if (translation_eigen.info() != Eigen::Success || rotation_eigen.info() != Eigen::Success ||
      !translation_eigen.eigenvalues().allFinite() || !rotation_eigen.eigenvalues().allFinite() ||
      !translation_eigen.eigenvectors().allFinite() || !rotation_eigen.eigenvectors().allFinite()) {
    report.status = "SCHUR_EIGEN_INVALID";
    return report;
  }
  // NDT's exact objective Hessian can be indefinite even at a PCL-converged
  // pose. Preserve its signs, but use singular curvature magnitudes for the
  // Schur rank/conditioning test (the analogous DCReg normal matrix is PSD).
  // Negative curvature is separately surfaced and never relabeled positive.
  auto orderByMagnitude = [](const Eigen::Vector3d& values, const Matrix3d& vectors,
                             Eigen::Vector3d* ordered_values, Matrix3d* ordered_vectors) {
    std::array<int, 3> order{{0, 1, 2}};
    std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      return std::abs(values(lhs)) < std::abs(values(rhs));
    });
    for (int column = 0; column < 3; ++column) {
      (*ordered_values)(column) = values(order[column]);
      ordered_vectors->col(column) = vectors.col(order[column]);
    }
  };
  orderByMagnitude(translation_eigen.eigenvalues(), translation_eigen.eigenvectors(),
      &report.translation_eigenvalues, &report.translation_axes);
  orderByMagnitude(rotation_eigen.eigenvalues(), rotation_eigen.eigenvectors(),
      &report.rotation_eigenvalues, &report.rotation_axes);
  const double translation_max_magnitude = report.translation_eigenvalues.cwiseAbs().maxCoeff();
  const double rotation_max_magnitude = report.rotation_eigenvalues.cwiseAbs().maxCoeff();
  if (translation_max_magnitude <= 1e-12 || rotation_max_magnitude <= 1e-12) {
    report.status = "ZERO_SCHUR_INFORMATION";
    return report;
  }
  report.translation_nonconvex = (report.translation_eigenvalues.array() < 0.0).any();
  report.rotation_nonconvex = (report.rotation_eigenvalues.array() < 0.0).any();
  report.translation_ratio = report.translation_eigenvalues.cwiseAbs().minCoeff() /
      translation_max_magnitude;
  report.rotation_ratio = report.rotation_eigenvalues.cwiseAbs().minCoeff() /
      rotation_max_magnitude;
  report.translation_condition = report.translation_ratio > 0.0 ? 1.0 / report.translation_ratio :
      std::numeric_limits<double>::infinity();
  report.rotation_condition = report.rotation_ratio > 0.0 ? 1.0 / report.rotation_ratio :
      std::numeric_limits<double>::infinity();
  report.translation_degenerate = report.translation_ratio <= threshold;
  report.rotation_degenerate = report.rotation_ratio <= threshold;
  report.valid = std::isfinite(report.translation_ratio) && std::isfinite(report.rotation_ratio);
  report.status = report.valid ? ((report.translation_nonconvex || report.rotation_nonconvex) ?
      "DCREG_DETECTION_ADAPTED_NONCONVEX" : "DCREG_DETECTION_ADAPTED") :
      "NONFINITE_SCHUR_DIAGNOSTIC";
  return report;
}

std::vector<MatureNdtCandidate> makeWeakDirectionCandidates(
    const DcregAdaptedReport& report, const Eigen::Matrix4f& nominal,
    double translation_step, double rotation_step_deg) {
  std::vector<MatureNdtCandidate> candidates;
  if (!report.valid || !properPose(nominal) || !std::isfinite(translation_step) ||
      !std::isfinite(rotation_step_deg) || translation_step <= 0.0 || rotation_step_deg <= 0.0)
    return candidates;
  int id = 0;
  if (report.translation_degenerate) {
    const Eigen::Vector3d axis = report.translation_axes.col(0).normalized();
    for (const double sign : {1.0, -1.0}) {
      MatureNdtCandidate candidate;
      candidate.id = id++;
      candidate.kind = sign > 0.0 ? "TRANSLATION_WEAK_PLUS" : "TRANSLATION_WEAK_MINUS";
      candidate.pose = nominal;
      candidate.pose.block<3, 1>(0, 3) += (sign * translation_step * axis).cast<float>();
      candidate.finite = properPose(candidate.pose);
      if (candidate.finite) candidates.push_back(candidate);
    }
  }
  if (report.rotation_degenerate) {
    const Eigen::Vector3d axis = report.rotation_axes.col(0).normalized();
    const double angle = rotation_step_deg * kPi / 180.0;
    for (const double sign : {1.0, -1.0}) {
      MatureNdtCandidate candidate;
      candidate.id = id++;
      candidate.kind = sign > 0.0 ? "ROTATION_WEAK_PLUS" : "ROTATION_WEAK_MINUS";
      candidate.pose = nominal;
      candidate.pose.block<3, 3>(0, 0) =
          (Eigen::AngleAxisd(sign * angle, axis).toRotationMatrix() *
           nominal.block<3, 3>(0, 0).cast<double>()).cast<float>();
      candidate.finite = properPose(candidate.pose);
      if (candidate.finite) candidates.push_back(candidate);
    }
  }
  return candidates;
}

double candidateMotionMerit(double score, std::size_t source_count,
    const Eigen::Matrix4f& candidate, const Eigen::Matrix4f& prediction,
    double nominal_energy_scale, const MatureNdtConfig& config) {
  if (source_count == 0 || !std::isfinite(score) || !properPose(candidate) ||
      !properPose(prediction) || !std::isfinite(nominal_energy_scale) || nominal_energy_scale < 1.0 ||
      config.motion_translation_scale_m <= 0.0 ||
      config.motion_rotation_scale_deg <= 0.0)
    return std::numeric_limits<double>::infinity();
  const double energy = -score / static_cast<double>(source_count);
  const double translation = (candidate.block<3, 1>(0, 3) - prediction.block<3, 1>(0, 3)).norm();
  const double rotation = rotationDistanceDeg(candidate.block<3, 3>(0, 0),
                                               prediction.block<3, 3>(0, 0));
  if (!std::isfinite(energy) || !std::isfinite(translation) || !std::isfinite(rotation))
    return std::numeric_limits<double>::infinity();
  return energy / nominal_energy_scale + config.motion_weight *
      (std::pow(translation / config.motion_translation_scale_m, 2.0) +
       std::pow(rotation / config.motion_rotation_scale_deg, 2.0));
}

void rankMatureCandidates(std::vector<MatureNdtCandidate>* candidates) {
  if (!candidates) return;
  std::stable_sort(candidates->begin(), candidates->end(),
      [](const MatureNdtCandidate& lhs, const MatureNdtCandidate& rhs) {
        if (lhs.finite != rhs.finite) return lhs.finite;
        if (lhs.merit != rhs.merit) return lhs.merit < rhs.merit;
        return lhs.id < rhs.id;
      });
}

SkfGateDecision evaluateSkfGate(bool enabled, const DcregAdaptedReport& report,
    bool visual_valid, bool visual_consumed, bool covariance_available) {
  SkfGateDecision decision;
  if (!enabled) return decision;
  if (!report.valid) {
    decision.status = "SKIPPED_INVALID_DEGENERACY_REPORT";
    return decision;
  }
  if (!report.translation_degenerate && !report.rotation_degenerate) {
    decision.status = "SKIPPED_NO_LIDAR_DEGENERATION";
    return decision;
  }
  decision.direction_selected = true;
  decision.selected_translation_axes = report.translation_axes;
  decision.selected_rotation_axes = report.rotation_axes;
  if (!visual_valid) {
    decision.status = "SKIPPED_NO_VALID_VISUAL_ROW";
    return decision;
  }
  if (visual_consumed) {
    decision.status = "SKIPPED_DOUBLE_COUNT_PREVENTED";
    return decision;
  }
  if (!covariance_available) {
    decision.status = "INTERFACE_READY_RELATIVE_VISUAL_NO_COVARIANCE";
    return decision;
  }
  decision.status = "READY_RELATIVE_VISUAL_MEASUREMENT";
  return decision;
}

}  // namespace dog_prior_map_localization
