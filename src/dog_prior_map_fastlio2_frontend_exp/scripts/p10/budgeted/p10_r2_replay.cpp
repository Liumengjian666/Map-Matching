#include "shadow_logging.hpp"
#include "anchor_logging.hpp"
#include <ctime>
#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Clock = std::chrono::steady_clock;
double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
uint64_t unsignedArgument(const char* argument) {
  const std::string value(argument);
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("invalid_unsigned_argument");
  return std::stoull(value);
}
paper::P7ImuVector initializationSamples(const paper::P7ImuVector& all,
                                        int sample_count, uint64_t stamp) {
  if (all.size() < static_cast<std::size_t>(sample_count))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  if (stamp == 0) return paper::P7ImuVector(all.begin(), all.begin() + sample_count);
  auto end = std::lower_bound(all.begin(), all.end(), stamp,
      [](const paper::ImuSample& sample, uint64_t time) { return sample.stamp_ns < time; });
  if (end != all.end() && end->stamp_ns == stamp) ++end;
  if (std::distance(all.begin(), end) < sample_count)
    throw std::runtime_error("insufficient_causal_initialization_imu");
  paper::P7ImuVector samples(end - sample_count, end);
  const uint64_t gap = stamp - samples.back().stamp_ns;
  if (gap != 0) {
    std::vector<uint64_t> periods;
    const std::size_t first = samples.size() > 101 ? samples.size() - 101 : 1;
    for (std::size_t i = first; i < samples.size(); ++i)
      periods.push_back(samples[i].stamp_ns - samples[i - 1].stamp_ns);
    std::sort(periods.begin(), periods.end());
    const uint64_t median = periods.at(periods.size() / 2);
    if (median == 0 || gap > 2 * median)
      throw std::runtime_error("initialization_epoch_extrapolation_exceeds_two_median_imu_periods");
  }
  return samples;
}
void poseColumns(std::ostream& out, const paper::Pose3d& pose) {
  out << ',' << pose.position.x() << ',' << pose.position.y() << ',' << pose.position.z()
      << ',' << pose.orientation.x() << ',' << pose.orientation.y() << ','
      << pose.orientation.z() << ',' << pose.orientation.w();
}
void vectorColumns(std::ostream& out, const Eigen::Vector3d& vector) {
  out << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
}
Eigen::Matrix4d preciseMatrix(const paper::Pose3d& pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = pose.orientation.normalized().toRotationMatrix();
  T.block<3,1>(0,3) = pose.position;
  return T;
}
paper::Pose3d carrierPose(const Eigen::Matrix4f& T) {
  paper::Pose3d pose;
  pose.position = T.block<3,1>(0,3).cast<double>();
  pose.orientation = Eigen::Quaterniond(T.block<3,3>(0,0).cast<double>()).normalized();
  return pose;
}
}  // namespace

int main(int argc, char** argv) {
  uint64_t transaction = 0;
  try {
    if (argc != 14)
      throw std::runtime_error("usage: p7_single_state_runner IMU_CSV FILTER_SCANS_CSV "
          "RAW_TIMED_SCAN_INDEX_CSV RAW_TIMED_POINTS_BIN MAP_PCD PARAMS_TXT "
          "TRAJECTORY_CSV REGISTRATION_CSV RUNTIME_CSV FRAME_LIMIT INITIALIZATION_STAMP_NS SHADOW_OR_CONTROL METHOD");
    const auto all_imu = paper::readP7Imu(argv[1]);
    const auto scans = paper::readP7TimedScans(argv[2], argv[3]);
    const uint64_t limit = unsignedArgument(argv[10]);
    const uint64_t initialization_stamp = unsignedArgument(argv[11]);
    const bool enable_shadow = std::string(argv[12]) == "shadow";
    const std::string experiment_mode = argv[12];
    const bool anchored = experiment_mode == "anchored_shadow" || experiment_mode == "anchored_guarded_feedback";
    const bool rotation_guard = experiment_mode == "guarded_feedback_rotation_guard" ||
        experiment_mode == "event_admission_rotation_guard";
    const bool guarded_feedback = experiment_mode == "guarded_feedback" ||
        experiment_mode == "guarded_feedback_rotation_guard" || experiment_mode == "anchored_guarded_feedback";
    const bool branch_admission = guarded_feedback || experiment_mode == "event_admission" ||
        experiment_mode == "event_admission_rotation_guard" || anchored;
    const bool enable_event = experiment_mode == "event" || branch_admission;
    if (!enable_shadow && !enable_event && std::string(argv[12]) != "control") throw std::runtime_error("invalid_mode");
    const std::string log_directory = std::string(argv[7]).substr(0,std::string(argv[7]).find_last_of('/'));
    p10log::Logger shadow_log(log_directory);
    p10log::EventLogger event_log(log_directory);
    std::unique_ptr<p10log::AdmissionLogger> admission_log;
    if (branch_admission) admission_log.reset(new p10log::AdmissionLogger(log_directory));
    std::unique_ptr<p10log::AnchorLogger> anchor_log;
    if (anchored) anchor_log.reset(new p10log::AnchorLogger(log_directory));
    const auto shadow_config = p10log::config(argv[13],false);
    paper::CoupledEventConfig event_config;
    event_config.search = shadow_config;
    event_config.branch_admission = branch_admission;
    event_config.branch_rotation_guard = rotation_guard;
    if (enable_event && std::string(argv[13]) != "C") throw std::runtime_error("event_requires_R2_residual_predictor");
    paper::PendingCandidate pending;
    paper::CoupledAnchorState anchor;
    if (limit == 0 || limit > scans.size()) throw std::runtime_error("invalid_frame_limit");
    paper::Pose3d initial_lidar, extrinsic;
    const auto parameters = paper::readP7Parameters(argv[6], &initial_lidar, &extrinsic);
    const auto initialization_imu = initializationSamples(
        all_imu, parameters.static_init_samples, initialization_stamp);
    paper::FastLio2IkfomFrontend frontend(parameters);
    std::string reason;
    if (!frontend.initializeStatic(initialization_imu, initial_lidar, extrinsic, &reason))
      throw std::runtime_error("static_initialization_failed:" + reason);
    if (initialization_stamp != 0 && frontend.getState().stamp_ns < initialization_stamp) {
      if (!frontend.predictHeldInputTo(initialization_stamp,
          initialization_imu[initialization_imu.size() - 2], initialization_imu.back(), &reason))
        throw std::runtime_error("initialization_epoch_propagation_failed:" + reason);
    }
    if (initialization_stamp != 0 && frontend.getState().stamp_ns != initialization_stamp)
      throw std::runtime_error("initialization_timestamp_mismatch");
    paper::CurrentFrameNdtRegistration registration{paper::CurrentFrameNdtParameters{}};
    if (!registration.loadMap(argv[5], &reason)) throw std::runtime_error("map_load_failed:" + reason);
    std::ofstream trajectory(argv[7]), observations(argv[8]), runtime(argv[9]);
    if (!trajectory || !observations || !runtime) throw std::runtime_error("cannot_create_outputs");
    trajectory.exceptions(std::ios::badbit | std::ios::failbit);
    observations.exceptions(std::ios::badbit | std::ios::failbit);
    runtime.exceptions(std::ios::badbit | std::ios::failbit);
    trajectory << std::setprecision(17)
        << "transaction_id,stamp_ns,predicted_imu_x,predicted_imu_y,predicted_imu_z,"
           "predicted_imu_qx,predicted_imu_qy,predicted_imu_qz,predicted_imu_qw,"
           "corrected_imu_x,corrected_imu_y,corrected_imu_z,corrected_imu_qx,corrected_imu_qy,"
           "corrected_imu_qz,corrected_imu_qw,velocity_x,velocity_y,velocity_z,"
           "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
           "gravity_x,gravity_y,gravity_z,lidar_update_applied\n";
    observations << std::setprecision(17)
        << "transaction_id,scan_start_ns,scan_effective_start_ns,stamp_ns,raw_source_points,"
           "overlap_points_dropped,source_points,target_points,source_cloud_hash,converged,effective,status,"
           "iterations,fitness,transformation_probability,alignment_ms,"
           "initial_x,initial_y,initial_z,initial_qx,initial_qy,initial_qz,initial_qw,"
           "raw_x,raw_y,raw_z,raw_qx,raw_qy,raw_qz,raw_qw,lidar_update_applied\n";
    runtime << std::setprecision(17)
        << "transaction_id,prediction_and_deskew_ms,cloud_io_ms,ndt_total_ms,ndt_alignment_ms,"
           "ikfom_update_ms,frame_total_ms,ndt_effective,lidar_update_applied\n";
    uint64_t updates = 0;
    paper::ScanEndProcessor scan_processor;
    for (uint64_t index = 0; index < limit; ++index) {
      const auto& scan = scans.at(index);
      transaction = scan.transaction_id;
      const auto frame_start = Clock::now();
      const auto frame_cpu = std::clock();
      const auto start = frontend.getState();
      const auto io_start = Clock::now();
      auto timed_cloud = paper::readP7PackedTimedCloud(argv[4], scan);
      const double cloud_io_ms = elapsedMs(io_start);
      paper::ScanWindowDecision window_decision;
      paper::ScanWindowStats window_stats;
      if (!paper::prepareScanWindow(scan.scan_start_ns, scan.scan_end_ns,
              start.stamp_ns, &timed_cloud, &window_decision, &window_stats, &reason))
        throw std::runtime_error("scan_window_failed:" + reason);
      if (window_decision != paper::ScanWindowDecision::PROCESS)
        throw std::runtime_error("unexpected_stale_raw_scan");
      const auto prediction_start = Clock::now();
      const auto causal_imu = paper::imuWindow(all_imu, start.stamp_ns, scan.scan_end_ns);
      paper::ScanEndResult scan_end;
      if (!scan_processor.process(&frontend, extrinsic,
              window_stats.effective_scan_start_ns, scan.scan_end_ns,
              causal_imu, timed_cloud, &scan_end, &reason))
        throw std::runtime_error("scan_end_prediction_or_deskew_failed:" + reason);
      const double prediction_ms = elapsedMs(prediction_start);
      if (scan_end.scan_end_ns != scan.scan_end_ns ||
          frontend.getState().stamp_ns != scan.scan_end_ns)
        throw std::runtime_error("scan_end_prediction_timestamp_mismatch");
      paper::RegistrationCloud cloud;
      cloud.reserve(scan_end.cloud_end_frame.size());
      for (const auto& point : scan_end.cloud_end_frame)
        cloud.push_back({static_cast<float>(point.position.x()),
                         static_cast<float>(point.position.y()),
                         static_cast<float>(point.position.z())});
      const auto ndt_start = Clock::now();
      paper::CurrentFrameNdtResult result;
      if (!registration.align(scan.scan_end_ns, cloud,
                              scan_end.predicted_map_T_lidar, &result, &reason))
        throw std::runtime_error("registration_internal_error:" + reason);
      const double ndt_total_ms = elapsedMs(ndt_start);
      paper::CoupledShadowResult shadow;
      shadow.nominal_pose = p10log::matrix(result.raw_map_T_lidar);
      shadow.prediction_pose = p10log::matrix(result.initial_map_T_lidar);
      shadow.recommended_pose = shadow.nominal_pose;
      shadow.status = "CONTROL_NO_SHADOW";
      shadow.complete_ndt_calls = result.status == paper::CurrentFrameNdtStatus::INSUFFICIENT_POINTS ? 0 : 1;
      if(enable_shadow && result.effective && !registration.shadow(result,shadow_config,&shadow,&reason))
        throw std::runtime_error("shadow_failed:"+reason);
      if(enable_shadow && !result.effective) shadow.status="NOMINAL_INEFFECTIVE_SHADOW_SKIPPED";
      paper::CoupledEventResult event;
      paper::CoupledAnchorReceipt anchor_receipt;
      event.mode="CONTROL"; event.event="CONTROL_NO_SHADOW";
      event.recommendation_available=result.effective;
      if (enable_event) {
        // Actual previous corrected state -> current causal IMU-only prediction.
        // Unlike pred_prev^-1*pred_cur, this excludes the previous measurement update.
        const Eigen::Matrix4d previous_lidar = preciseMatrix(start.map_T_imu) * preciseMatrix(extrinsic);
        const Eigen::Matrix4d imu_interval = previous_lidar.inverse() * preciseMatrix(scan_end.predicted_map_T_lidar);
        const bool event_ok = anchored ? registration.anchoredEventShadow(result,event_config,
            &pending,&event,&reason,imu_interval,&anchor,&anchor_receipt) :
            registration.eventShadow(result,event_config,&pending,&event,&reason,
                branch_admission ? &imu_interval : nullptr);
        if (!event_ok)
          throw std::runtime_error("event_shadow_failed:"+reason);
        shadow=event.shadow;
      }
      const auto update_start = Clock::now();
      paper::Pose3d actual_measurement = result.raw_map_T_lidar;
      const bool alternative_used = guarded_feedback && event.admitted;
      if (alternative_used) {
        if (!event.temporally_supported || !event.admission_valid || pending.active)
          throw std::runtime_error("invalid_admission_consumption");
        actual_measurement = carrierPose(event.admitted_pose);
      }
      paper::PoseCorrectionDelta delta;
      bool update_success = false;
      if (result.effective) {
        if (!frontend.applyPoseMeasurement(
            paper::lidarMeasurementToImu(actual_measurement, extrinsic), &delta, &reason)) {
          if (admission_log) admission_log->write(transaction,experiment_mode,event,
              actual_measurement,alternative_used,false,delta);
          throw std::runtime_error("pose_update_failed:" + reason);
        }
        update_success = true;
        ++updates;
      }
      const double update_ms = elapsedMs(update_start);
      const auto corrected = frontend.getState();
      if (corrected.stamp_ns != scan.scan_end_ns) throw std::runtime_error("state_timestamp_mismatch");
      if (!frontend.postconditionsValid(&reason)) throw std::runtime_error("postconditions_failed:" + reason);
      if (anchored) {
        const bool stable=update_success && event.mode=="NORMAL" && !event.innovation_trigger &&
            !event.pending_before && !event.pending_after;
        paper::settleCoupledAnchor(&anchor,scan.scan_end_ns,
            preciseMatrix(corrected.map_T_imu)*preciseMatrix(extrinsic),stable,alternative_used);
      }
      const double total_ms = elapsedMs(frame_start);
      const auto logging_start = Clock::now();
      shadow_log.write(transaction,result,shadow,total_ms,1000.0*(std::clock()-frame_cpu)/CLOCKS_PER_SEC);
      event_log.write(transaction,event,pending);
      if (admission_log) admission_log->write(transaction,experiment_mode,event,
          actual_measurement,alternative_used,update_success,delta);
      if (anchor_log) anchor_log->write(transaction,anchor_receipt,anchor);
      trajectory << transaction << ',' << scan.scan_end_ns;
      poseColumns(trajectory, scan_end.predicted_map_T_imu); poseColumns(trajectory, corrected.map_T_imu);
      vectorColumns(trajectory, corrected.velocity); vectorColumns(trajectory, corrected.gyro_bias);
      vectorColumns(trajectory, corrected.accel_bias); vectorColumns(trajectory, corrected.gravity);
      trajectory << ',' << result.effective << '\n';
      observations << transaction << ',' << scan.scan_start_ns << ','
          << window_stats.effective_scan_start_ns << ',' << scan.scan_end_ns << ','
          << scan.cloud_point_count << ',' << window_stats.overlap_points_dropped << ','
          << result.source_point_count << ',' << result.target_point_count << ','
          << result.source_cloud_hash
          << ',' << result.converged << ',' << result.effective << ','
          << paper::currentFrameNdtStatusName(result.status) << ',' << result.iterations << ','
          << result.fitness << ',' << result.transformation_probability << ',' << result.alignment_ms;
      poseColumns(observations, result.initial_map_T_lidar); poseColumns(observations, result.raw_map_T_lidar);
      observations << ',' << result.effective << '\n';
      runtime << transaction << ',' << prediction_ms << ',' << cloud_io_ms << ',' << ndt_total_ms
          << ',' << result.alignment_ms << ',' << update_ms << ',' << total_ms << ','
          << result.effective << ',' << result.effective << '\n';
      event_log.cost(transaction,elapsedMs(frame_start),1000.0*(std::clock()-frame_cpu)/CLOCKS_PER_SEC,
          elapsedMs(logging_start));
    }
    event_log.finish(pending);
    trajectory.close(); observations.close(); runtime.close();
    p10log::resourceReceipt(log_directory);
    std::cout << "frames=" << limit << " lidar_updates=" << updates
              << " prediction_only=" << limit - updates << " state_finite=true\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FIRST_BAD_TX=" << transaction << " error=" << error.what() << '\n';
    return 1;
  }
}
