#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace p6_i5c {

using Pose = Eigen::Matrix4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

inline bool sha256Matches(const std::string& actual, const std::string& expected) {
  return !actual.empty() && !expected.empty() && actual == expected;
}

inline bool sameFrozenSelectionIdentity(const std::string& first_selected_sha,
                                        const std::string& first_manifest_sha,
                                        const std::string& second_selected_sha,
                                        const std::string& second_manifest_sha) {
  return sha256Matches(first_selected_sha, second_selected_sha) &&
         sha256Matches(first_manifest_sha, second_manifest_sha);
}

inline void validateSelectionIdentity(
    const std::vector<std::pair<std::string, uint64_t>>& identities) {
  if (identities.empty() || identities.size() > 7)
    throw std::runtime_error("selected_case_count_outside_1_to_7");
  std::set<std::string> frames;
  std::set<uint64_t> transactions;
  for (const auto& identity : identities) {
    if (identity.first.empty() || identity.second == 0 ||
        !frames.insert(identity.first).second ||
        !transactions.insert(identity.second).second)
      throw std::runtime_error("duplicate_selected_frame_or_transaction");
  }
}

inline Pose parsePose7(const std::string& text) {
  std::vector<double> values;
  std::stringstream stream(text);
  std::string part;
  while (std::getline(stream, part, ';')) values.push_back(std::stod(part));
  if (values.size() != 7) throw std::runtime_error("pose7_field_count");
  for (double value : values)
    if (!std::isfinite(value)) throw std::runtime_error("pose7_nonfinite");
  Eigen::Quaterniond q(values[6], values[3], values[4], values[5]);
  if (q.norm() < 1e-12) throw std::runtime_error("pose7_zero_quaternion");
  q.normalize();
  Pose pose = Pose::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3d(values[0], values[1], values[2]);
  return pose;
}

inline std::string serializePose7(const Pose& pose) {
  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  std::ostringstream out;
  out << std::setprecision(17) << pose(0, 3) << ';' << pose(1, 3) << ';'
      << pose(2, 3) << ';' << q.x() << ';' << q.y() << ';' << q.z() << ';'
      << q.w();
  return out.str();
}

inline double rotationDistanceRad(const Eigen::Matrix3d& first,
                                  const Eigen::Matrix3d& second) {
  const Eigen::Quaterniond delta(first.transpose() * second);
  Eigen::Quaterniond normalized = delta.normalized();
  return 2.0 * std::atan2(normalized.vec().norm(), std::abs(normalized.w()));
}

inline double translationDistance(const Pose& first, const Pose& second) {
  return (first.block<3, 1>(0, 3) - second.block<3, 1>(0, 3)).norm();
}

inline bool terminalReturn(bool converged, double translation_error_m,
                           double rotation_error_deg,
                           double translation_tolerance_m,
                           double rotation_tolerance_deg) {
  return converged && std::isfinite(translation_error_m) &&
         std::isfinite(rotation_error_deg) &&
         translation_error_m <= translation_tolerance_m &&
         rotation_error_deg <= rotation_tolerance_deg;
}

struct NeighborScoreAudit {
  std::size_t finite_count = 0;
  std::size_t nonfinite_count = 0;
  bool no_positive_increase = true;
  std::vector<std::string> nonfinite_locations;

  void observe(const std::string& location, double center_score,
               double neighbor_score, double repeat_tolerance) {
    const double delta = neighbor_score - center_score;
    if (!std::isfinite(center_score) || !std::isfinite(neighbor_score) ||
        !std::isfinite(delta)) {
      ++nonfinite_count;
      nonfinite_locations.push_back(location);
      no_positive_increase = false;
      return;
    }
    ++finite_count;
    if (delta > repeat_tolerance) no_positive_increase = false;
  }

  bool determinate(std::size_t expected_count) const {
    return nonfinite_count == 0 && finite_count == expected_count;
  }
};

struct RepeatedScoreAudit {
  std::size_t finite_count = 0;
  std::size_t nonfinite_count = 0;
  std::vector<std::string> nonfinite_locations;
  double minimum = std::numeric_limits<double>::quiet_NaN();
  double maximum = std::numeric_limits<double>::quiet_NaN();
  double range = std::numeric_limits<double>::quiet_NaN();

  bool determinate() const { return nonfinite_count == 0 && finite_count == 3; }
};

inline RepeatedScoreAudit auditRepeatedScores(const std::array<double, 3>& scores) {
  RepeatedScoreAudit audit;
  for (std::size_t index = 0; index < scores.size(); ++index) {
    if (std::isfinite(scores[index])) {
      ++audit.finite_count;
    } else {
      ++audit.nonfinite_count;
      audit.nonfinite_locations.push_back("center_score_repeat_" + std::to_string(index));
    }
  }
  if (audit.determinate()) {
    const auto bounds = std::minmax_element(scores.begin(), scores.end());
    audit.minimum = *bounds.first;
    audit.maximum = *bounds.second;
    audit.range = audit.maximum - audit.minimum;
  }
  return audit;
}

inline double symmetricSpectralNorm(const Eigen::MatrixXd& matrix) {
  if (matrix.rows() != matrix.cols() || !matrix.allFinite())
    throw std::runtime_error("invalid_symmetric_norm_input");
  const Eigen::MatrixXd symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("symmetric_norm_eigendecomposition_failed");
  return solver.eigenvalues().cwiseAbs().maxCoeff();
}

inline Pose leftPerturbMap(const Pose& nominal, const Eigen::Vector3d& dp,
                           const Eigen::Vector3d& dphi) {
  Pose result = nominal;
  const double angle = dphi.norm();
  const Eigen::Matrix3d delta_rotation = angle == 0.0
      ? Eigen::Matrix3d::Identity()
      : Eigen::AngleAxisd(angle, dphi / angle).toRotationMatrix();
  result.block<3, 3>(0, 0) = delta_rotation * nominal.block<3, 3>(0, 0);
  result.block<3, 1>(0, 3) += dp;
  return result;
}

inline Eigen::Matrix<double, 6, 1> pclVector(const Eigen::Matrix4f& pose) {
  Eigen::Transform<float, 3, Eigen::Affine> transform;
  transform.matrix() = pose;
  const Eigen::Vector3f angles = transform.rotation().eulerAngles(0, 1, 2);
  Eigen::Matrix<double, 6, 1> out;
  out << transform.translation().x(), transform.translation().y(),
      transform.translation().z(), angles.x(), angles.y(), angles.z();
  return out;
}

inline Eigen::Matrix4f poseFromPclVector(const Eigen::Matrix<double, 6, 1>& p) {
  return (Eigen::Translation3f(static_cast<float>(p(0)), static_cast<float>(p(1)),
                               static_cast<float>(p(2))) *
          Eigen::AngleAxisf(static_cast<float>(p(3)), Eigen::Vector3f::UnitX()) *
          Eigen::AngleAxisf(static_cast<float>(p(4)), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(static_cast<float>(p(5)), Eigen::Vector3f::UnitZ())).matrix();
}

inline int64_t alphaKeyPositive(double alpha) {
  if (!std::isfinite(alpha) || alpha < 0.0)
    throw std::runtime_error("invalid_nonnegative_alpha");
  return static_cast<int64_t>(std::floor(alpha * 1e6 + 0.5));
}

struct EndpointKey {
  uint64_t transaction_id = 0;
  std::string ray_type;
  int ray_id = 0;
  int sign = 0;
  int64_t alpha_key = 0;

  bool operator<(const EndpointKey& other) const {
    return std::tie(transaction_id, ray_type, ray_id, sign, alpha_key) <
           std::tie(other.transaction_id, other.ray_type, other.ray_id,
                    other.sign, other.alpha_key);
  }
};

}  // namespace p6_i5c
