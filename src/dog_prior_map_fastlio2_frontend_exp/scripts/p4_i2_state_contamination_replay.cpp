// Offline-only replay for PAPER-P4-I2. The include keeps FULL_UPDATE on the
// exact runtime implementation; this executable is never linked into a ROS
// node or installed as a runtime target.
#include "../src/fastlio2_frontend_ikfom.cpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace p4_i2 {
using dog_prior_map_fastlio2_frontend_exp::FastLio2IkfomFrontend;
using dog_prior_map_fastlio2_frontend_exp::FilterSnapshot;
using dog_prior_map_fastlio2_frontend_exp::ImuSample;
using dog_prior_map_fastlio2_frontend_exp::Pose3d;
using dog_prior_map_fastlio2_frontend_exp::PoseCorrectionDelta;
using dog_prior_map_fastlio2_frontend_exp::RuntimeParameters;
using ImuVector = std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>;

struct PoseRecord {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  Pose3d saved_predictor_lidar;
  Pose3d used_lidar;
  Pose3d saved_corrected_lidar;
};

struct Inputs {
  ImuVector imu;
  std::vector<PoseRecord, Eigen::aligned_allocator<PoseRecord>> scans;
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) fields.push_back(field);
  return fields;
}

bool readInputs(const std::string& imu_path, const std::string& scan_path,
                Inputs* output, std::string* reason) {
  if (!output) return false;
  std::ifstream imu_file(imu_path);
  if (!imu_file) {
    if (reason) *reason = "cannot_open_imu_csv";
    return false;
  }
  std::string line;
  std::getline(imu_file, line);  // header
  uint64_t previous_stamp = 0;
  while (std::getline(imu_file, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != 7) {
      if (reason) *reason = "invalid_imu_csv_field_count";
      return false;
    }
    ImuSample sample;
    sample.stamp_ns = std::stoull(fields[0]);
    sample.acceleration = Eigen::Vector3d(std::stod(fields[1]), std::stod(fields[2]),
                                           std::stod(fields[3]));
    sample.angular_velocity = Eigen::Vector3d(std::stod(fields[4]), std::stod(fields[5]),
                                              std::stod(fields[6]));
    if (sample.stamp_ns <= previous_stamp || !sample.acceleration.allFinite() ||
        !sample.angular_velocity.allFinite()) {
      if (reason) *reason = "imu_csv_not_strictly_monotonic_or_nonfinite";
      return false;
    }
    previous_stamp = sample.stamp_ns;
    output->imu.push_back(sample);
  }

  std::ifstream scan_file(scan_path);
  if (!scan_file) {
    if (reason) *reason = "cannot_open_scan_csv";
    return false;
  }
  std::getline(scan_file, line);  // header
  previous_stamp = 0;
  uint64_t expected_transaction = 1;
  while (std::getline(scan_file, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != 23) {
      if (reason) *reason = "invalid_scan_csv_field_count";
      return false;
    }
    PoseRecord record;
    record.transaction_id = std::stoull(fields[0]);
    record.stamp_ns = std::stoull(fields[1]);
    auto parsePose = [&fields](std::size_t offset, Pose3d* pose) {
      pose->position = Eigen::Vector3d(std::stod(fields[offset]),
                                       std::stod(fields[offset + 1]),
                                       std::stod(fields[offset + 2]));
      pose->orientation = Eigen::Quaterniond(std::stod(fields[offset + 6]),
                                              std::stod(fields[offset + 3]),
                                              std::stod(fields[offset + 4]),
                                              std::stod(fields[offset + 5]));
      const double norm = pose->orientation.norm();
      if (!pose->position.allFinite() || !pose->orientation.coeffs().allFinite() ||
          !std::isfinite(norm) || norm < 1e-12) return false;
      pose->orientation.normalize();
      return true;
    };
    if (record.transaction_id != expected_transaction || record.stamp_ns <= previous_stamp ||
        !parsePose(2, &record.saved_predictor_lidar) ||
        !parsePose(9, &record.used_lidar) ||
        !parsePose(16, &record.saved_corrected_lidar)) {
      if (reason) *reason = "invalid_scan_sequence_or_pose";
      return false;
    }
    previous_stamp = record.stamp_ns;
    ++expected_transaction;
    output->scans.push_back(record);
  }
  if (output->imu.empty() || output->scans.empty()) {
    if (reason) *reason = "empty_input_csv";
    return false;
  }
  return true;
}

RuntimeParameters readParameters(const std::string& path, Pose3d* initial_map_T_lidar,
                                 Pose3d* T_imu_lidar) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_parameter_file");
  std::vector<double> values;
  double value = 0.0;
  while (input >> value) values.push_back(value);
  if (values.size() != 27) throw std::runtime_error("expected_27_runtime_parameters");
  std::size_t i = 0;
  RuntimeParameters parameters;
  parameters.static_init_samples = static_cast<int>(values[i++]);
  parameters.gravity_mps2 = values[i++];
  parameters.initial_accel_bias = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]);
  i += 3;
  parameters.gyro_noise_std_rad_s = values[i++];
  parameters.accel_noise_std_m_s2 = values[i++];
  parameters.gyro_bias_rw_std_rad_s2 = values[i++];
  parameters.accel_bias_rw_std_m_s3 = values[i++];
  parameters.pose_position_sigma_m = values[i++];
  parameters.pose_rotation_sigma_rad = values[i++];
  parameters.max_static_accel_std_m_s2 = values[i++];
  parameters.max_static_gyro_std_rad_s = values[i++];
  auto readPose = [&values, &i](Pose3d* pose) {
    pose->position = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]);
    i += 3;
    pose->orientation = Eigen::Quaterniond(values[i + 3], values[i], values[i + 1],
                                            values[i + 2]);
    i += 4;
    pose->orientation.normalize();
  };
  readPose(initial_map_T_lidar);
  readPose(T_imu_lidar);
  return parameters;
}

Eigen::Isometry3d asIsometry(const Pose3d& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.orientation.toRotationMatrix();
  result.translation() = pose.position;
  return result;
}

Pose3d fromIsometry(const Eigen::Isometry3d& transform) {
  Pose3d result;
  result.position = transform.translation();
  result.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return result;
}

Pose3d lidarMeasurementToImu(const Pose3d& map_T_lidar, const Pose3d& T_imu_lidar) {
  return fromIsometry(asIsometry(map_T_lidar) * asIsometry(T_imu_lidar).inverse());
}

ImuVector imuWindow(const ImuVector& all, uint64_t state_stamp_ns,
                    uint64_t end_stamp_ns) {
  auto after_state = std::upper_bound(
      all.begin(), all.end(), state_stamp_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  if (after_state == all.begin()) throw std::runtime_error("no_imu_sample_before_filter_state");
  auto first = after_state - 1;
  auto after_end = std::upper_bound(
      all.begin(), all.end(), end_stamp_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  if (after_end - first < 2) throw std::runtime_error("fewer_than_two_scan_imu_samples");
  return ImuVector(first, after_end);
}

Eigen::Quaterniond poseDelta(const Pose3d& before, const Pose3d& after) {
  return (before.orientation.normalized().conjugate() *
          after.orientation.normalized()).normalized();
}

double gravityAngleDeg(const Eigen::Vector3d& before, const Eigen::Vector3d& after) {
  const double denominator = before.norm() * after.norm();
  if (denominator < 1e-12) return std::numeric_limits<double>::quiet_NaN();
  const double cosine = std::max(-1.0, std::min(1.0, before.dot(after) / denominator));
  return std::acos(cosine) * 180.0 / M_PI;
}

void writeVector(std::ostream& output, const Eigen::Vector3d& vector) {
  output << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
}

void writePose(std::ostream& output, const Pose3d& pose) {
  output << ',' << pose.position.x() << ',' << pose.position.y() << ','
         << pose.position.z() << ',' << pose.orientation.x() << ','
         << pose.orientation.y() << ',' << pose.orientation.z() << ','
         << pose.orientation.w();
}

void writeHeader(std::ostream& output) {
  output << "frame_index,transaction_id,stamp_ns"
         << ",predictor_imu_tx,predictor_imu_ty,predictor_imu_tz,predictor_imu_qx,predictor_imu_qy,predictor_imu_qz,predictor_imu_qw"
         << ",corrected_imu_tx,corrected_imu_ty,corrected_imu_tz,corrected_imu_qx,corrected_imu_qy,corrected_imu_qz,corrected_imu_qw"
         << ",saved_predictor_lidar_tx,saved_predictor_lidar_ty,saved_predictor_lidar_tz,saved_predictor_lidar_qx,saved_predictor_lidar_qy,saved_predictor_lidar_qz,saved_predictor_lidar_qw"
         << ",saved_corrected_lidar_tx,saved_corrected_lidar_ty,saved_corrected_lidar_tz,saved_corrected_lidar_qx,saved_corrected_lidar_qy,saved_corrected_lidar_qz,saved_corrected_lidar_qw"
         << ",used_lidar_tx,used_lidar_ty,used_lidar_tz,used_lidar_qx,used_lidar_qy,used_lidar_qz,used_lidar_qw"
         << ",delta_position_x,delta_position_y,delta_position_z,delta_rotation_x_rad,delta_rotation_y_rad,delta_rotation_z_rad"
         << ",velocity_pre_x,velocity_pre_y,velocity_pre_z,velocity_post_x,velocity_post_y,velocity_post_z,delta_velocity_x,delta_velocity_y,delta_velocity_z"
         << ",gyro_bias_pre_x,gyro_bias_pre_y,gyro_bias_pre_z,gyro_bias_post_x,gyro_bias_post_y,gyro_bias_post_z,delta_gyro_bias_x,delta_gyro_bias_y,delta_gyro_bias_z"
         << ",accel_bias_pre_x,accel_bias_pre_y,accel_bias_pre_z,accel_bias_post_x,accel_bias_post_y,accel_bias_post_z,delta_accel_bias_x,delta_accel_bias_y,delta_accel_bias_z"
         << ",gravity_pre_x,gravity_pre_y,gravity_pre_z,gravity_post_x,gravity_post_y,gravity_post_z,gravity_delta_deg\n";
}

void writeRow(std::ostream& output, std::size_t index, const PoseRecord& record,
              const Pose3d& predictor, const FilterSnapshot& before,
              const FilterSnapshot& after, const Pose3d& T_imu_lidar) {
  output << index + 1 << ',' << record.transaction_id << ',' << record.stamp_ns;
  writePose(output, predictor);
  writePose(output, after.map_T_imu);
  writePose(output, fromIsometry(asIsometry(record.saved_predictor_lidar) *
                                 asIsometry(T_imu_lidar).inverse()));
  writePose(output, fromIsometry(asIsometry(record.saved_corrected_lidar) *
                                 asIsometry(T_imu_lidar).inverse()));
  writePose(output, record.used_lidar);
  writeVector(output, after.map_T_imu.position - before.map_T_imu.position);
  const Eigen::AngleAxisd rotation_delta(poseDelta(before.map_T_imu, after.map_T_imu));
  writeVector(output, rotation_delta.axis() * rotation_delta.angle());
  writeVector(output, before.velocity);
  writeVector(output, after.velocity);
  writeVector(output, after.velocity - before.velocity);
  writeVector(output, before.gyro_bias);
  writeVector(output, after.gyro_bias);
  writeVector(output, after.gyro_bias - before.gyro_bias);
  writeVector(output, before.accel_bias);
  writeVector(output, after.accel_bias);
  writeVector(output, after.accel_bias - before.accel_bias);
  writeVector(output, before.gravity);
  writeVector(output, after.gravity);
  output << ',' << gravityAngleDeg(before.gravity, after.gravity) << '\n';
}

// The custom path is deliberately confined to this offline executable. It
// uses the same pinned state type, propagation callbacks, process noise,
// initialization snapshot, measurement manifold and pose Jacobians as the
// runtime. Only the measurement gain rows and corresponding covariance update
// differ for the two counterfactual modes.
class SelectiveFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Filter = dog_prior_map_fastlio2_frontend_exp::PoseFilter;
  using Covariance = Filter::cov;

  bool initializeFromRuntimeSeed(const FastLio2IkfomFrontend& seed,
                                 const RuntimeParameters& parameters,
                                 const bool allow_velocity,
                                 std::string* reason) {
    const FilterSnapshot snapshot = seed.getState();
    if (snapshot.stamp_ns == 0 || snapshot.covariance.rows() != state_ikfom::DOF ||
        snapshot.covariance.cols() != state_ikfom::DOF) {
      if (reason) *reason = "invalid_runtime_seed_snapshot";
      return false;
    }
    state_ikfom state;
    state.pos = vect3(snapshot.map_T_imu.position);
    state.rot = SO3(snapshot.map_T_imu.orientation.toRotationMatrix());
    state.offset_R_L_I = SO3(snapshot.T_imu_lidar_rotation);
    state.offset_T_L_I = vect3(snapshot.T_imu_lidar_translation);
    state.vel = vect3(snapshot.velocity);
    state.bg = vect3(snapshot.gyro_bias);
    state.ba = vect3(snapshot.accel_bias);
    state.grav = S2(snapshot.gravity);
    filter_.change_x(state);
    Covariance initial_covariance = snapshot.covariance;
    filter_.change_P(initial_covariance);

    double limits[state_ikfom::DOF];
    std::fill(limits, limits + state_ikfom::DOF, 100.0);
    filter_.init(
        get_f, df_dx, df_dw,
        dog_prior_map_fastlio2_frontend_exp::poseModel,
        dog_prior_map_fastlio2_frontend_exp::poseJacobian,
        dog_prior_map_fastlio2_frontend_exp::poseNoiseJacobian,
        4, limits);
    process_noise_ = seed.getProcessNoiseCovariance();
    pose_position_variance_ = parameters.pose_position_sigma_m *
                              parameters.pose_position_sigma_m;
    pose_rotation_variance_ = parameters.pose_rotation_sigma_rad *
                              parameters.pose_rotation_sigma_rad;
    stamp_ns_ = snapshot.stamp_ns;
    fixed_rotation_ = state.offset_R_L_I;
    fixed_translation_ = state.offset_T_L_I;
    allow_velocity_ = allow_velocity;
    initialized_ = true;
    return true;
  }

  uint64_t stampNs() const { return stamp_ns_; }

  FilterSnapshot snapshot() const {
    FilterSnapshot result;
    if (!initialized_) return result;
    const state_ikfom& state = filter_.get_x();
    result.stamp_ns = stamp_ns_;
    result.map_T_imu.position = state.pos;
    result.map_T_imu.orientation = Eigen::Quaterniond(
        state.rot.w(), state.rot.x(), state.rot.y(), state.rot.z()).normalized();
    result.velocity = state.vel;
    result.gyro_bias = state.bg;
    result.accel_bias = state.ba;
    result.gravity = state.grav.vec;
    result.T_imu_lidar_rotation = state.offset_R_L_I.toRotationMatrix();
    result.T_imu_lidar_translation = state.offset_T_L_I;
    result.covariance = filter_.get_P();
    return result;
  }

  bool predictImuSequence(const ImuVector& samples, uint64_t end_stamp_ns,
                          std::string* reason) {
    auto fail = [reason](const char* text) {
      if (reason) *reason = text;
      return false;
    };
    if (reason) reason->clear();
    if (!initialized_ || samples.size() < 2 || end_stamp_ns < stamp_ns_)
      return fail("filter_uninitialized_or_invalid_prediction_interval");
    uint64_t previous_stamp = 0;
    for (const ImuSample& sample : samples) {
      if (sample.stamp_ns == 0 || sample.stamp_ns <= previous_stamp ||
          sample.stamp_ns > end_stamp_ns || !sample.acceleration.allFinite() ||
          !sample.angular_velocity.allFinite())
        return fail("invalid_or_future_imu_sequence_sample");
      previous_stamp = sample.stamp_ns;
    }
    const uint64_t initial_stamp = stamp_ns_;
    if (samples.front().stamp_ns > initial_stamp ||
        samples.back().stamp_ns < std::min(initial_stamp, end_stamp_ns))
      return fail("imu_sequence_does_not_cover_filter_time");

    auto sampleAt = [](const ImuSample& a, const ImuSample& b, uint64_t stamp) {
      ImuSample interpolated;
      interpolated.stamp_ns = stamp;
      const double alpha = static_cast<double>(stamp - a.stamp_ns) /
                           static_cast<double>(b.stamp_ns - a.stamp_ns);
      interpolated.acceleration = (1.0 - alpha) * a.acceleration + alpha * b.acceleration;
      interpolated.angular_velocity =
          (1.0 - alpha) * a.angular_velocity + alpha * b.angular_velocity;
      return interpolated;
    };

    std::size_t bracket = 0;
    while (bracket + 1 < samples.size() &&
           samples[bracket + 1].stamp_ns <= initial_stamp)
      ++bracket;
    if (samples[bracket].stamp_ns > initial_stamp)
      return fail("no_causal_imu_bracket_at_state_time");
    uint64_t propagated_to = initial_stamp;
    for (std::size_t index = bracket; index + 1 < samples.size(); ++index) {
      const ImuSample& tail = samples[index + 1];
      if (tail.stamp_ns <= propagated_to) continue;
      const ImuSample head = propagated_to == samples[index].stamp_ns
                                 ? samples[index]
                                 : sampleAt(samples[index], tail, propagated_to);
      if (!predictInterval(head, tail, reason)) return false;
      propagated_to = tail.stamp_ns;
    }
    if (propagated_to < end_stamp_ns) {
      if (!predictHeldInputTo(end_stamp_ns, samples[samples.size() - 2],
                              samples.back(), reason))
        return false;
    }
    if (stamp_ns_ != end_stamp_ns) return fail("scan_end_pose_timestamp_not_exact");
    return true;
  }

  bool applyPoseMeasurement(const Pose3d& measurement_pose,
                            std::string* reason) {
    if (reason) reason->clear();
    if (!initialized_) {
      if (reason) *reason = "filter_not_initialized";
      return false;
    }
    const FilterSnapshot before = snapshot();
    dog_prior_map_fastlio2_frontend_exp::PoseMeasurement measurement(
        vect3(measurement_pose.position),
        SO3(measurement_pose.orientation.toRotationMatrix()));
    Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
    noise.block<3, 3>(0, 0).diagonal().setConstant(pose_position_variance_);
    noise.block<3, 3>(3, 3).diagonal().setConstant(pose_rotation_variance_);

    state_ikfom current = filter_.get_x();
    const state_ikfom propagated = current;
    const Covariance covariance_propagated = filter_.get_P();
    int convergence_count = 0;
    bool valid = true;
    const int maximum_iterations = 4;
    double limits[state_ikfom::DOF];
    std::fill(limits, limits + state_ikfom::DOF, 100.0);

    for (int iteration = -1; iteration < maximum_iterations; ++iteration) {
      Eigen::Matrix<double, state_ikfom::DOF, 1> dx;
      Eigen::Matrix<double, state_ikfom::DOF, 1> dx_new;
      current.boxminus(dx, propagated);
      dx_new = dx;
      auto h_x = dog_prior_map_fastlio2_frontend_exp::poseJacobian(current, valid);
      if (!valid) {
        if (reason) *reason = "pose_jacobian_invalid";
        return false;
      }

      Covariance covariance = covariance_propagated;
      transportPriorCovariance(current, propagated, dx, &dx_new, &covariance);
      const Eigen::Matrix<double, 6, 6> innovation_covariance =
          h_x * covariance * h_x.transpose() + noise;
      Eigen::Matrix<double, state_ikfom::DOF, 6> gain =
          covariance * h_x.transpose() * innovation_covariance.ldlt().solve(
              Eigen::Matrix<double, 6, 6>::Identity());
      for (int row = 0; row < state_ikfom::DOF; ++row) {
        const bool pose_state = row >= 0 && row < 6;
        const bool velocity_state = allow_velocity_ && row >= 12 && row < 15;
        if (!pose_state && !velocity_state) gain.row(row).setZero();
      }

      Eigen::Matrix<double, 6, 1> innovation;
      measurement.boxminus(
          innovation,
          dog_prior_map_fastlio2_frontend_exp::poseModel(current, valid));
      if (!valid || !innovation.allFinite() || !gain.allFinite()) {
        if (reason) *reason = "nonfinite_or_invalid_pose_update_terms";
        return false;
      }
      const Covariance gain_h = gain * h_x;
      const Eigen::Matrix<double, state_ikfom::DOF, 1> correction =
          gain * innovation + (gain_h - Covariance::Identity()) * dx_new;
      current.boxplus(correction);
      bool converged = true;
      for (int index = 0; index < state_ikfom::DOF; ++index) {
        if (std::fabs(correction[index]) > limits[index]) {
          converged = false;
          break;
        }
      }
      if (converged) ++convergence_count;

      if (convergence_count > 1 || iteration == maximum_iterations - 1) {
        const Covariance identity = Covariance::Identity();
        const Covariance residual_map = identity - gain_h;
        Covariance posterior = residual_map * covariance * residual_map.transpose() +
                               gain * noise * gain.transpose();
        transportPosteriorAfterInjection(current, propagated, correction, &posterior);
        filter_.change_x(current);
        filter_.change_P(posterior);
        enforceFixedExtrinsic();
        const FilterSnapshot after = snapshot();
        if (!after.covariance.allFinite() ||
            (after.covariance - after.covariance.transpose()).cwiseAbs().maxCoeff() > 1e-7 ||
            after.covariance.diagonal().minCoeff() < -1e-9 ||
            (after.T_imu_lidar_rotation - before.T_imu_lidar_rotation).norm() > 1e-12 ||
            (after.T_imu_lidar_translation - before.T_imu_lidar_translation).norm() > 1e-12) {
          if (reason) *reason = "selective_update_covariance_or_extrinsic_postcondition";
          return false;
        }
        return true;
      }
    }
    if (reason) *reason = "iterated_pose_update_did_not_terminate";
    return false;
  }

 private:
  bool predictInterval(const ImuSample& head, const ImuSample& tail,
                       std::string* reason) {
    if (tail.stamp_ns <= head.stamp_ns || stamp_ns_ < head.stamp_ns ||
        stamp_ns_ >= tail.stamp_ns || !head.acceleration.allFinite() ||
        !tail.acceleration.allFinite() || !head.angular_velocity.allFinite() ||
        !tail.angular_velocity.allFinite()) {
      if (reason) *reason = "invalid_imu_interval";
      return false;
    }
    const double dt = static_cast<double>(tail.stamp_ns - stamp_ns_) * 1e-9;
    input_ikfom input;
    input.acc = vect3(0.5 * (head.acceleration + tail.acceleration));
    input.gyro = vect3(0.5 * (head.angular_velocity + tail.angular_velocity));
    double mutable_dt = dt;
    auto noise = process_noise_;
    filter_.predict(mutable_dt, noise, input);
    stamp_ns_ = tail.stamp_ns;
    enforceFixedExtrinsic();
    return true;
  }

  bool predictHeldInputTo(uint64_t end_stamp_ns, const ImuSample& head,
                          const ImuSample& tail, std::string* reason) {
    if (tail.stamp_ns > end_stamp_ns || end_stamp_ns <= stamp_ns_ ||
        tail.stamp_ns > stamp_ns_ || head.stamp_ns >= tail.stamp_ns) {
      if (reason) *reason = "invalid_causal_scan_end_tail";
      return false;
    }
    const double dt = static_cast<double>(end_stamp_ns - stamp_ns_) * 1e-9;
    input_ikfom input;
    input.acc = vect3(0.5 * (head.acceleration + tail.acceleration));
    input.gyro = vect3(0.5 * (head.angular_velocity + tail.angular_velocity));
    double mutable_dt = dt;
    auto noise = process_noise_;
    filter_.predict(mutable_dt, noise, input);
    stamp_ns_ = end_stamp_ns;
    enforceFixedExtrinsic();
    return true;
  }

  static void transportPriorCovariance(state_ikfom current,
                                       state_ikfom propagated,
                                       const Eigen::Matrix<double, state_ikfom::DOF, 1>& dx,
                                       Eigen::Matrix<double, state_ikfom::DOF, 1>* dx_new,
                                       Covariance* covariance) {
    for (const auto& item : current.SO3_state) {
      const int index = item.first;
      const MTK::vect<3, double> segment = dx.template segment<3>(index);
      const Eigen::Matrix3d reset = A_matrix(segment).transpose();
      dx_new->template block<3, 1>(index, 0) =
          reset * dx.template block<3, 1>(index, 0);
      covariance->template block<3, state_ikfom::DOF>(index, 0) =
          reset * covariance->template block<3, state_ikfom::DOF>(index, 0);
      covariance->template block<state_ikfom::DOF, 3>(0, index) =
          covariance->template block<state_ikfom::DOF, 3>(0, index) * reset.transpose();
    }
    for (const auto& item : current.S2_state) {
      const int index = item.first;
      MTK::vect<2, double> segment;
      segment << dx[index], dx[index + 1];
      Eigen::Matrix<double, 2, 3> nx;
      Eigen::Matrix<double, 3, 2> mx;
      current.S2_Nx_yy(nx, index);
      propagated.S2_Mx(mx, segment, index);
      const Eigen::Matrix2d reset = nx * mx;
      dx_new->template block<2, 1>(index, 0) =
          reset * dx.template block<2, 1>(index, 0);
      covariance->template block<2, state_ikfom::DOF>(index, 0) =
          reset * covariance->template block<2, state_ikfom::DOF>(index, 0);
      covariance->template block<state_ikfom::DOF, 2>(0, index) =
          covariance->template block<state_ikfom::DOF, 2>(0, index) * reset.transpose();
    }
  }

  static void transportPosteriorAfterInjection(
      state_ikfom current, state_ikfom propagated,
      const Eigen::Matrix<double, state_ikfom::DOF, 1>& correction,
      Covariance* covariance) {
    Covariance reset = Covariance::Identity();
    for (const auto& item : current.SO3_state) {
      const int index = item.first;
      const MTK::vect<3, double> segment = correction.template segment<3>(index);
      reset.template block<3, 3>(index, index) = A_matrix(segment).transpose();
    }
    for (const auto& item : current.S2_state) {
      const int index = item.first;
      MTK::vect<2, double> segment;
      segment << correction[index], correction[index + 1];
      Eigen::Matrix<double, 2, 3> nx;
      Eigen::Matrix<double, 3, 2> mx;
      current.S2_Nx_yy(nx, index);
      propagated.S2_Mx(mx, segment, index);
      reset.template block<2, 2>(index, index) = nx * mx;
    }
    *covariance = reset * *covariance * reset.transpose();
  }

  void enforceFixedExtrinsic() {
    state_ikfom state = filter_.get_x();
    Covariance covariance = filter_.get_P();
    const int rotation_index = MTK::getStartIdx(&state_ikfom::offset_R_L_I);
    const int translation_index = MTK::getStartIdx(&state_ikfom::offset_T_L_I);
    state.offset_R_L_I = fixed_rotation_;
    state.offset_T_L_I = fixed_translation_;
    for (int index : {rotation_index, translation_index}) {
      covariance.block(index, 0, 3, state_ikfom::DOF).setZero();
      covariance.block(0, index, state_ikfom::DOF, 3).setZero();
      covariance.block<3, 3>(index, index).diagonal().setConstant(1e-12);
    }
    filter_.change_x(state);
    filter_.change_P(covariance);
  }

  Filter filter_;
  Eigen::Matrix<double, 12, 12> process_noise_ =
      Eigen::Matrix<double, 12, 12>::Zero();
  double pose_position_variance_ = 0.0;
  double pose_rotation_variance_ = 0.0;
  uint64_t stamp_ns_ = 0;
  bool initialized_ = false;
  bool allow_velocity_ = false;
  SO3 fixed_rotation_ = SO3::Identity();
  vect3 fixed_translation_ = vect3(Eigen::Vector3d::Zero());
};

int runFullUpdate(const Inputs& inputs, const RuntimeParameters& parameters,
                  const Pose3d& initial_map_T_lidar, const Pose3d& T_imu_lidar,
                  const std::string& output_path) {
  if (inputs.imu.size() < static_cast<std::size_t>(parameters.static_init_samples))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  FastLio2IkfomFrontend frontend(parameters);
  ImuVector initialization_imu(inputs.imu.begin(),
                               inputs.imu.begin() + parameters.static_init_samples);
  std::string reason;
  if (!frontend.initializeStatic(initialization_imu, initial_map_T_lidar,
                                 T_imu_lidar, &reason))
    throw std::runtime_error("FULL_UPDATE_static_initialization_failed:" + reason);

  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot_create_replay_output");
  output << std::setprecision(17);
  writeHeader(output);
  for (std::size_t index = 0; index < inputs.scans.size(); ++index) {
    const PoseRecord& record = inputs.scans[index];
    const FilterSnapshot start = frontend.getState();
    ImuVector window = imuWindow(inputs.imu, start.stamp_ns, record.stamp_ns);
    std::vector<dog_prior_map_fastlio2_frontend_exp::ImuPoseSample,
                Eigen::aligned_allocator<dog_prior_map_fastlio2_frontend_exp::ImuPoseSample>>
        unused_poses;
    if (!frontend.predictImuSequence(window, record.stamp_ns, &unused_poses, &reason))
      throw std::runtime_error("FULL_UPDATE_imu_prediction_failed:" + reason);
    const FilterSnapshot predicted = frontend.getState();
    const Pose3d used_imu = lidarMeasurementToImu(record.used_lidar, T_imu_lidar);
    PoseCorrectionDelta ignored_delta;
    if (!frontend.applyPoseMeasurement(used_imu, &ignored_delta, &reason))
      throw std::runtime_error("FULL_UPDATE_pose_update_failed:" + reason);
    const FilterSnapshot corrected = frontend.getState();
    if (corrected.stamp_ns != record.stamp_ns)
      throw std::runtime_error("FULL_UPDATE_timestamp_mismatch");
    writeRow(output, index, record, predicted.map_T_imu, predicted, corrected,
             T_imu_lidar);
  }
  output.close();
  std::cout << "FULL_UPDATE_REPLAY_COMPLETE frames=" << inputs.scans.size()
            << " init_stamp_ns=" << initialization_imu.back().stamp_ns << '\n';
  return 0;
}

int runSelectiveUpdate(const Inputs& inputs, const RuntimeParameters& parameters,
                       const Pose3d& initial_map_T_lidar,
                       const Pose3d& T_imu_lidar, bool allow_velocity,
                       const std::string& mode, const std::string& output_path) {
  if (inputs.imu.size() < static_cast<std::size_t>(parameters.static_init_samples))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  FastLio2IkfomFrontend seed(parameters);
  ImuVector initialization_imu(inputs.imu.begin(),
                               inputs.imu.begin() + parameters.static_init_samples);
  std::string reason;
  if (!seed.initializeStatic(initialization_imu, initial_map_T_lidar,
                             T_imu_lidar, &reason))
    throw std::runtime_error(mode + "_static_initialization_failed:" + reason);
  SelectiveFilter filter;
  if (!filter.initializeFromRuntimeSeed(seed, parameters, allow_velocity, &reason))
    throw std::runtime_error(mode + "_filter_construction_failed:" + reason);

  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot_create_replay_output");
  output << std::setprecision(17);
  writeHeader(output);
  for (std::size_t index = 0; index < inputs.scans.size(); ++index) {
    const PoseRecord& record = inputs.scans[index];
    ImuVector window = imuWindow(inputs.imu, filter.stampNs(), record.stamp_ns);
    if (!filter.predictImuSequence(window, record.stamp_ns, &reason))
      throw std::runtime_error(mode + "_imu_prediction_failed:" + reason);
    const FilterSnapshot predicted = filter.snapshot();
    const Pose3d used_imu = lidarMeasurementToImu(record.used_lidar, T_imu_lidar);
    if (!filter.applyPoseMeasurement(used_imu, &reason))
      throw std::runtime_error(mode + "_pose_update_failed_at_tx_" +
                               std::to_string(record.transaction_id) + ":" + reason);
    const FilterSnapshot corrected = filter.snapshot();
    if (corrected.stamp_ns != record.stamp_ns)
      throw std::runtime_error(mode + "_timestamp_mismatch");
    writeRow(output, index, record, predicted.map_T_imu, predicted, corrected,
             T_imu_lidar);
  }
  output.close();
  std::cout << mode << "_REPLAY_COMPLETE frames=" << inputs.scans.size()
            << " init_stamp_ns=" << initialization_imu.back().stamp_ns << '\n';
  return 0;
}

}  // namespace p4_i2

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "usage: p4_i2_state_contamination_replay MODE imu.csv scan.csv params.txt output.csv\n";
    return 2;
  }
  try {
    const std::string mode(argv[1]);
    p4_i2::Inputs inputs;
    std::string reason;
    if (!p4_i2::readInputs(argv[2], argv[3], &inputs, &reason))
      throw std::runtime_error("input_load_failed:" + reason);
    p4_i2::Pose3d initial_map_T_lidar, T_imu_lidar;
    const p4_i2::RuntimeParameters parameters =
        p4_i2::readParameters(argv[4], &initial_map_T_lidar, &T_imu_lidar);
    if (mode == "FULL_UPDATE")
      return p4_i2::runFullUpdate(inputs, parameters, initial_map_T_lidar,
                                  T_imu_lidar, argv[5]);
    if (mode == "POSE_VEL_UPDATE")
      return p4_i2::runSelectiveUpdate(inputs, parameters, initial_map_T_lidar,
                                       T_imu_lidar, true, mode, argv[5]);
    if (mode == "POSE_ONLY_UPDATE")
      return p4_i2::runSelectiveUpdate(inputs, parameters, initial_map_T_lidar,
                                       T_imu_lidar, false, mode, argv[5]);
    throw std::runtime_error("unknown_replay_mode:" + mode);
  } catch (const std::exception& exception) {
    std::cerr << "P4_I2_REPLAY_FAILED: " << exception.what() << '\n';
    return 1;
  }
}
