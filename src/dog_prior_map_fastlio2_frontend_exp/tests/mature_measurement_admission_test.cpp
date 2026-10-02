#include "dog_prior_map_fastlio2_frontend_exp/mature_measurement_admission.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void sameState(const paper::FilterSnapshot& a, const paper::FilterSnapshot& b) {
  check(a.stamp_ns == b.stamp_ns && a.map_T_imu.position == b.map_T_imu.position &&
        a.map_T_imu.orientation.coeffs() == b.map_T_imu.orientation.coeffs() &&
        a.velocity == b.velocity && a.gyro_bias == b.gyro_bias &&
        a.accel_bias == b.accel_bias && a.gravity == b.gravity &&
        a.T_imu_lidar_rotation == b.T_imu_lidar_rotation &&
        a.T_imu_lidar_translation == b.T_imu_lidar_translation &&
        a.covariance == b.covariance, "state/covariance changed");
}
}

int main() {
  try {
    using Policy = paper::MatureMeasurementAdmission;
    paper::ProjectedPoseInnovation good;
    good.valid = true; good.rank = 6; good.nis = 1.0;
    for (double distance : {2.9, 3.0}) {
      Policy policy;
      check(policy.observe(true, distance, good).accepted, "inclusive distance boundary");
    }
    Policy policy;
    check(policy.observe(true, 3.000001, good).reason ==
          "AUTOWARE_INITIAL_TO_RESULT_DISTANCE_REJECT", "distance reject");
    check(!policy.observe(false, 0.0, good).accepted, "ineffective rejected");
    check(policy.observe(true, 0.0, good).consecutive_rejections == 0, "accepted resets counter");
    for (int i = 1; i <= 5; ++i) {
      const auto result = policy.observe(false, 0.0, good);
      check(result.consecutive_rejections == i, "rejection counter");
      check((result.tracking_state == paper::AdmissionTrackingState::LOST) == (i == 5), "fifth LOST");
    }
    check(!policy.observe(true, 0.0, good).accepted, "LOST is terminal");
    for (double distance : {-1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()})
      check(!Policy::distanceGatePass(distance), "invalid distance fails closed");
    for (double nis : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
      Policy invalid;
      auto bad = good; bad.nis = nis;
      check(invalid.observe(true, 0.0, bad).reason == "NIS_EVALUATION_INVALID", "invalid NIS");
    }
    Policy threshold;
    good.nis = paper::chiSquare99Threshold(6);
    check(good.nis == 16.812 && threshold.observe(true, 0.0, good).accepted, "inclusive NIS boundary");
    good.nis += 1e-6;
    check(!threshold.observe(true, 0.0, good).accepted, "NIS above boundary");

    paper::RuntimeParameters parameters;
    paper::FastLio2IkfomFrontend frontend(parameters), baseline(parameters);
    std::vector<paper::ImuSample, Eigen::aligned_allocator<paper::ImuSample>> imu;
    for (int i = 0; i < parameters.static_init_samples; ++i) {
      paper::ImuSample sample;
      sample.stamp_ns = 1000000ULL + i * 5000000ULL;
      sample.acceleration.z() = parameters.gravity_mps2;
      imu.push_back(sample);
    }
    paper::Pose3d initial, extrinsic;
    extrinsic.position = Eigen::Vector3d(0.08, 0.029, 0.03);
    std::string reason;
    check(frontend.initializeStatic(imu, initial, extrinsic, &reason), reason.c_str());
    check(baseline.initializeStatic(imu, initial, extrinsic, &reason), reason.c_str());
    // Initialization has unit pose variance; first establish a tracked pose via
    // the public update, not by injecting or changing the filter covariance.
    paper::PoseCorrectionDelta initial_delta;
    const auto initial_imu_pose = frontend.getState().map_T_imu;
    check(frontend.applyPoseMeasurement(initial_imu_pose, &initial_delta, &reason), reason.c_str());
    check(baseline.applyPoseMeasurement(initial_imu_pose, &initial_delta, &reason), reason.c_str());
    auto tail = imu.back(); tail.stamp_ns += 5000000ULL;
    check(frontend.predictInterval(imu.back(), tail, &reason), reason.c_str());
    check(baseline.predictInterval(imu.back(), tail, &reason), reason.c_str());
    const auto predicted = frontend.getState();
    Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
    noise.diagonal().head<3>().setConstant(std::pow(parameters.pose_position_sigma_m, 2));
    noise.diagonal().tail<3>().setConstant(std::pow(parameters.pose_rotation_sigma_rad, 2));
    auto evaluate = [&](const paper::Pose3d& measurement) {
      paper::ProjectedPoseInnovation innovation;
      check(frontend.evaluateProjectedPoseInnovationLinearized(measurement, noise,
          Eigen::Matrix<double, 6, 6>::Identity(), 6,
          paper::ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL, &innovation, &reason), reason.c_str());
      check(innovation.valid && std::isfinite(innovation.nis), "real frontend finite NIS");
      sameState(predicted, frontend.getState());
      return innovation;
    };
    auto far = predicted.map_T_imu;
    far.position.x() += 2.9;  // Passes distance gate, fails statistical consistency.
    Policy integration;
    const auto far_innovation = evaluate(far);
    check(far_innovation.nis > paper::chiSquare99Threshold(6), "far innovation must reject");
    const auto rejected = integration.observe(true, 2.9, far_innovation);
    check(!rejected.accepted && rejected.reason == "MAHALANOBIS_NIS_REJECT", "real NIS rejected");
    sameState(baseline.getState(), frontend.getState());
    auto near = predicted.map_T_imu;
    near.position.x() += 0.001;
    const auto accepted = integration.observe(true, 0.001, evaluate(near));
    check(accepted.accepted && accepted.consecutive_rejections == 0, "near accepted");
    paper::PoseCorrectionDelta delta;
    check(frontend.applyPoseMeasurement(near, &delta, &reason), reason.c_str());
    check(baseline.applyPoseMeasurement(near, &delta, &reason), reason.c_str());
    sameState(baseline.getState(), frontend.getState());
    std::cout << "MATURE_MEASUREMENT_ADMISSION_PASS far_nis=" << far_innovation.nis << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
