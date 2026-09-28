#include "p6_i5c_stationary_math.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <set>

namespace {
void require(bool condition, const char* name) {
  if (!condition) throw std::runtime_error(std::string("FAIL:") + name);
  std::cout << "PASS:" << name << '\n';
}

double poseRotationError(const Eigen::Matrix4f& first, const Eigen::Matrix4f& second) {
  return p6_i5c::rotationDistanceRad(first.block<3, 3>(0, 0).cast<double>(),
                                     second.block<3, 3>(0, 0).cast<double>());
}
}  // namespace

int main() {
  try {
    require(p6_i5c::sha256Matches("abc123", "abc123") &&
            !p6_i5c::sha256Matches("abc123", "changed") &&
            !p6_i5c::sha256Matches("", "abc123"),
            "frozen_sha_identity_match_and_mismatch");
    require(p6_i5c::sameFrozenSelectionIdentity("sel", "manifest", "sel", "manifest") &&
            !p6_i5c::sameFrozenSelectionIdentity("sel", "manifest", "different", "manifest") &&
            !p6_i5c::sameFrozenSelectionIdentity("sel", "manifest", "sel", "different"),
            "smoke_formal_selection_and_manifest_identity");

    p6_i5c::validateSelectionIdentity({{"frame-1", 1}});
    std::vector<std::pair<std::string, uint64_t>> seven_cases;
    for (uint64_t index = 1; index <= 7; ++index)
      seven_cases.emplace_back("frame-" + std::to_string(index), index);
    p6_i5c::validateSelectionIdentity(seven_cases);
    bool rejected_empty = false;
    bool rejected_eight = false;
    bool rejected_duplicate_frame = false;
    bool rejected_duplicate_transaction = false;
    try { p6_i5c::validateSelectionIdentity({}); }
    catch (const std::runtime_error&) { rejected_empty = true; }
    seven_cases.emplace_back("frame-8", 8);
    try { p6_i5c::validateSelectionIdentity(seven_cases); }
    catch (const std::runtime_error&) { rejected_eight = true; }
    try { p6_i5c::validateSelectionIdentity({{"same", 1}, {"same", 2}}); }
    catch (const std::runtime_error&) { rejected_duplicate_frame = true; }
    try { p6_i5c::validateSelectionIdentity({{"frame-1", 1}, {"frame-2", 1}}); }
    catch (const std::runtime_error&) { rejected_duplicate_transaction = true; }
    require(rejected_empty && rejected_eight && rejected_duplicate_frame &&
            rejected_duplicate_transaction,
            "selected_case_count_1_to_7_and_uniqueness");

    Eigen::Quaterniond q = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()) *
                           Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY());
    p6_i5c::Pose pose = p6_i5c::Pose::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(1.25, -2.5, 3.75);
    const p6_i5c::Pose decoded = p6_i5c::parsePose7(p6_i5c::serializePose7(pose));
    require(p6_i5c::translationDistance(pose, decoded) < 1e-14 &&
            p6_i5c::rotationDistanceRad(pose.block<3, 3>(0, 0),
                                        decoded.block<3, 3>(0, 0)) < 1e-14,
            "pose7_serialization_roundtrip");
    require(p6_i5c::terminalReturn(true, 0.01, 0.1, 0.02, 0.2),
            "converged_terminal_within_return_gate");
    require(!p6_i5c::terminalReturn(false, 0.0, 0.0, 0.02, 0.2),
            "nonconverged_terminal_not_counted_as_return");
    require(!p6_i5c::terminalReturn(true, 0.01,
                                    std::numeric_limits<double>::quiet_NaN(), 0.02, 0.2),
            "nonfinite_terminal_not_counted_as_return");
    p6_i5c::NeighborScoreAudit neighbor_audit;
    neighbor_audit.observe("x+", 10.0, 10.1, 1e-8);
    neighbor_audit.observe("y-", 10.0,
                           std::numeric_limits<double>::quiet_NaN(), 1e-8);
    neighbor_audit.observe("z+", 10.0,
                           std::numeric_limits<double>::infinity(), 1e-8);
    require(neighbor_audit.finite_count == 1 &&
            neighbor_audit.nonfinite_count == 2 &&
            neighbor_audit.nonfinite_locations == std::vector<std::string>({"y-", "z+"}) &&
            !neighbor_audit.determinate(3) && !neighbor_audit.no_positive_increase,
            "nonfinite_neighbor_scores_force_indeterminate_with_locations");
    const auto finite_repeats = p6_i5c::auditRepeatedScores({10.0, 10.0, 10.0});
    require(finite_repeats.determinate() && finite_repeats.finite_count == 3 &&
            finite_repeats.nonfinite_count == 0 && finite_repeats.range == 0.0,
            "finite_center_repeats_are_determinate");
    const auto nonfinite_repeats = p6_i5c::auditRepeatedScores({
        10.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()});
    require(!nonfinite_repeats.determinate() && nonfinite_repeats.finite_count == 1 &&
            nonfinite_repeats.nonfinite_count == 2 &&
            nonfinite_repeats.nonfinite_locations == std::vector<std::string>(
                {"center_score_repeat_1", "center_score_repeat_2"}),
            "nonfinite_center_repeats_force_indeterminate_with_locations");

    Eigen::Quaterniond q_neg(-q.w(), -q.x(), -q.y(), -q.z());
    p6_i5c::Pose sign_flip = pose;
    sign_flip.block<3, 3>(0, 0) = q_neg.toRotationMatrix();
    require(p6_i5c::rotationDistanceRad(pose.block<3, 3>(0, 0),
                                        sign_flip.block<3, 3>(0, 0)) < 1e-14,
            "quaternion_sign_invariance");

    const Eigen::Vector3d dphi(0.03, -0.02, 0.01);
    const Eigen::Matrix3d expected = Eigen::AngleAxisd(dphi.norm(), dphi.normalized()).toRotationMatrix() *
                                     pose.block<3, 3>(0, 0);
    const p6_i5c::Pose left = p6_i5c::leftPerturbMap(pose, Eigen::Vector3d::Zero(), dphi);
    require(p6_i5c::rotationDistanceRad(expected, left.block<3, 3>(0, 0)) < 1e-14,
            "map_frame_left_perturbation");
    const Eigen::Matrix3d right = pose.block<3, 3>(0, 0) *
        Eigen::AngleAxisd(dphi.norm(), dphi.normalized()).toRotationMatrix();
    require(p6_i5c::rotationDistanceRad(right, left.block<3, 3>(0, 0)) > 1e-5,
            "left_perturbation_not_right_perturbation");

    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d yaw90 = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    require(std::abs(p6_i5c::rotationDistanceRad(identity, yaw90) - M_PI / 2.0) < 1e-14,
            "so3_geodesic_angle");

    Eigen::Matrix2d hessian;
    hessian << 3.0, 0.0, 0.0, 4.0;
    require(std::abs(p6_i5c::symmetricSpectralNorm(hessian) - 4.0) < 1e-14,
            "symmetric_hessian_spectral_norm");

    std::set<p6_i5c::EndpointKey> keys;
    keys.insert({42, "PRINCIPAL", 4, -1, p6_i5c::alphaKeyPositive(0.5)});
    keys.insert({42, "EXTRA_MARGIN", 4, -1, p6_i5c::alphaKeyPositive(0.5)});
    keys.insert({42, "PRINCIPAL", 4, 1, p6_i5c::alphaKeyPositive(0.5)});
    keys.insert({42, "PRINCIPAL", 4, -1, p6_i5c::alphaKeyPositive(0.500001)});
    require(keys.size() == 4 && p6_i5c::alphaKeyPositive(0.0000005) == 1,
            "endpoint_key_uniqueness_and_alpha_quantization");

    const Eigen::Matrix4f pcl_pose = pose.cast<float>();
    const Eigen::Matrix4f pcl_roundtrip = p6_i5c::poseFromPclVector(p6_i5c::pclVector(pcl_pose));
    require((pcl_roundtrip.block<3, 1>(0, 3) - pcl_pose.block<3, 1>(0, 3)).norm() < 1e-6f &&
            poseRotationError(pcl_pose, pcl_roundtrip) < 1e-5,
            "pcl_euler_012_parameter_roundtrip");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
