#include "dog_prior_map_localization/external_ndt_transaction_server.hpp"
#include "dog_prior_map_localization/ndt_request_identity.hpp"

#include <dog_prior_map_interfaces/NdtScanRequest.h>
#include <dog_prior_map_interfaces/NdtScanResult.h>
#include <dog_prior_map_interfaces/NdtServerStatus.h>
#include <dog_prior_map_interfaces/NdtSessionControl.h>
#include <dog_prior_map_interfaces/NdtSessionControlAck.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/serialization.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {

using Request = dog_prior_map_interfaces::NdtScanRequest;
using Result = dog_prior_map_interfaces::NdtScanResult;
using Status = dog_prior_map_interfaces::NdtServerStatus;
using Control = dog_prior_map_interfaces::NdtSessionControl;
using ControlAck = dog_prior_map_interfaces::NdtSessionControlAck;

static_assert(sizeof(dog_prior_map_localization::detail::NdtRequestIdentity) <= 256,
              "terminal request identity must stay compact");

class Probe {
 public:
  void status(const Status::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex);
    latest_status = *message;
    got_status = true;
    condition.notify_all();
  }
  void ack(const ControlAck::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex);
    control_ack = *message;
    got_ack = true;
    condition.notify_all();
  }
  void result(const Result::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex);
    results[message->transaction_id] = *message;
    ++result_counts[message->transaction_id];
    condition.notify_all();
  }
  bool waitStatus(Status* output) {
    std::unique_lock<std::mutex> lock(mutex);
    if (!condition.wait_for(lock, std::chrono::seconds(5), [this] { return got_status; }))
      return false;
    *output = latest_status;
    return true;
  }
  bool waitAck(ControlAck* output) {
    std::unique_lock<std::mutex> lock(mutex);
    if (!condition.wait_for(lock, std::chrono::seconds(5), [this] { return got_ack; }))
      return false;
    *output = control_ack;
    return true;
  }
  bool waitResult(uint64_t tx, Result* output) {
    return waitResultAfter(tx, 0, output);
  }
  bool waitResultAfter(uint64_t tx, uint64_t previous_count, Result* output) {
    std::unique_lock<std::mutex> lock(mutex);
    if (!condition.wait_for(lock, std::chrono::seconds(10), [this, tx, previous_count] {
          return result_counts[tx] > previous_count;
        })) return false;
    *output = results.at(tx);
    return true;
  }
  uint64_t resultCount(uint64_t tx) {
    std::lock_guard<std::mutex> lock(mutex);
    return result_counts[tx];
  }

  std::mutex mutex;
  std::condition_variable condition;
  Status latest_status;
  ControlAck control_ack;
  std::map<uint64_t, Result> results;
  std::map<uint64_t, uint64_t> result_counts;
  bool got_status = false;
  bool got_ack = false;
};

uint64_t cloudHash(const sensor_msgs::PointCloud2& cloud) {
  constexpr uint64_t offset_basis = 14695981039346656037ULL;
  constexpr uint64_t prime = 1099511628211ULL;
  uint64_t hash = offset_basis;
  auto byte = [&hash](uint8_t value) { hash = (hash ^ value) * prime; };
  auto u32 = [&byte](uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8)
      byte(static_cast<uint8_t>(value >> shift));
  };
  auto string = [&u32, &byte](const std::string& value) {
    u32(static_cast<uint32_t>(value.size()));
    for (unsigned char c : value) byte(c);
  };
  string(cloud.header.frame_id);
  u32(cloud.header.stamp.sec);
  u32(cloud.header.stamp.nsec);
  u32(cloud.width);
  u32(cloud.height);
  u32(static_cast<uint32_t>(cloud.fields.size()));
  for (const auto& field : cloud.fields) {
    string(field.name);
    u32(field.offset);
    byte(field.datatype);
    u32(field.count);
  }
  u32(cloud.point_step);
  u32(cloud.row_step);
  byte(cloud.is_bigendian ? 1U : 0U);
  u32(static_cast<uint32_t>(cloud.data.size()));
  for (uint8_t value : cloud.data) byte(value);
  return hash;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makeMap() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  for (int iy = -40; iy <= 40; ++iy) {
    for (int iz = 0; iz <= 24; ++iz) {
      map->push_back(pcl::PointXYZ(3.0f, 0.10f * iy, 0.10f * iz));
      map->push_back(pcl::PointXYZ(0.10f * iy, 4.0f, 0.10f * iz));
    }
  }
  for (int ix = -30; ix <= 30; ++ix)
    for (int iy = -40; iy <= 40; ++iy)
      map->push_back(pcl::PointXYZ(0.10f * ix, 0.10f * iy, 2.4f));
  map->width = static_cast<uint32_t>(map->size());
  map->height = 1;
  map->is_dense = true;
  return map;
}

Eigen::Matrix4d pose(double x, double y, double z, double yaw) {
  Eigen::Matrix4d value = Eigen::Matrix4d::Identity();
  value.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  value.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return value;
}

void setPose(geometry_msgs::Pose* output, const Eigen::Matrix4d& transform) {
  output->position.x = transform(0, 3);
  output->position.y = transform(1, 3);
  output->position.z = transform(2, 3);
  Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
  q.normalize();
  output->orientation.x = q.x();
  output->orientation.y = q.y();
  output->orientation.z = q.z();
  output->orientation.w = q.w();
}

Request requestFor(uint64_t tx, const Status& status, const std::string& session,
                   const Eigen::Matrix4d& guess,
                   const sensor_msgs::PointCloud2& cloud) {
  Request request;
  request.header.stamp.fromNSec(2000000000ULL + tx * 100000000ULL);
  request.header.frame_id = "map";
  request.protocol_version = Status::PROTOCOL_VERSION;
  request.frontend_session_id = session;
  request.epoch = 0;
  request.transaction_id = tx;
  request.server_instance_id = status.server_instance_id;
  request.scan_start.fromNSec(request.header.stamp.toNSec() - 100000000ULL);
  request.scan_end = request.header.stamp;
  request.scan_start_ns = request.scan_start.toNSec();
  request.scan_end_ns = request.scan_end.toNSec();
  request.map_frame = "map";
  request.lidar_frame = "lidar";
  request.cloud_end_frame = cloud;
  request.cloud_end_frame.header.stamp = request.scan_end;
  request.cloud_end_frame.header.frame_id = "lidar";
  request.predicted_map_T_lidar.header = request.header;
  setPose(&request.predicted_map_T_lidar.pose, guess);
  request.request_cloud_hash = cloudHash(request.cloud_end_frame);
  return request;
}

template <typename T>
void setParam(const std::string& key, const T& value) {
  ros::param::set(key, value);
}

sensor_msgs::PointCloud2 makeSource(const pcl::PointCloud<pcl::PointXYZ>& map,
                                    const Eigen::Matrix4d& map_T_lidar,
                                    const std::string& frame,
                                    const ros::Time& stamp,
                                    const Eigen::Vector3d& local_offset =
                                        Eigen::Vector3d::Zero(),
                                    std::size_t cap = 0) {
  pcl::PointCloud<pcl::PointXYZ> source;
  const Eigen::Matrix4d lidar_T_map = map_T_lidar.inverse();
  for (std::size_t i = 0; i < map.size(); ++i) {
    if (cap && source.size() >= cap) break;
    Eigen::Vector4d point(map[i].x, map[i].y, map[i].z, 1.0);
    point = lidar_T_map * point;
    point.head<3>() += local_offset;
    source.push_back(pcl::PointXYZ(static_cast<float>(point.x()),
                                   static_cast<float>(point.y()),
                                   static_cast<float>(point.z())));
  }
  source.width = static_cast<uint32_t>(source.size());
  source.height = 1;
  source.is_dense = true;
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(source, message);
  message.header.frame_id = frame;
  message.header.stamp = stamp;
  return message;
}

std::vector<uint8_t> serializeResultPayload(Result result) {
  // roscpp assigns Header.seq on each publish; it is transport metadata, not
  // part of the cached NDT terminal payload.
  result.header.seq = 0;
  std::vector<uint8_t> bytes(ros::serialization::serializationLength(result));
  ros::serialization::OStream stream(bytes.data(), static_cast<uint32_t>(bytes.size()));
  ros::serialization::serialize(stream, result);
  return bytes;
}

Eigen::Vector3d usedTranslation(const Result& result) {
  return Eigen::Vector3d(result.used_map_T_lidar.pose.position.x,
                         result.used_map_T_lidar.pose.position.y,
                         result.used_map_T_lidar.pose.position.z);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "external_ndt_server_integration_test");
  if (!ros::master::check()) {
    std::cerr << "FAIL: ROS master is not running; start an isolated roscore for this test\n";
    return 2;
  }

  char path[] = "/tmp/p3_r10a_ndt_map_XXXXXX.pcd";
  const int fd = mkstemps(path, 4);
  if (fd < 0) return 2;
  close(fd);
  const pcl::PointCloud<pcl::PointXYZ>::Ptr map = makeMap();
  if (pcl::io::savePCDFileBinary(path, *map) != 0) {
    std::cerr << "FAIL: cannot save synthetic PCD\n";
    return 2;
  }

  setParam("/map/pcd_fallback_path", std::string(path));
  setParam("/frames/map_frame", std::string("map"));
  setParam("/frames/base_frame", std::string("lidar"));
  setParam("/map/voxel_size", 0.05);
  setParam("/lidar_update/ndt_target_voxel_size", 0.05);
  setParam("/lidar_update/ndt_source_voxel_size", 0.08);
  setParam("/lidar_update/min_effective_points", 30);
  setParam("/lidar_update/scan_min_range", 0.1);
  setParam("/lidar_update/scan_max_range", 1000.0);
  setParam("/lidar_update/ndt_resolution", 1.0);
  setParam("/lidar_update/ndt_step_size", 0.1);
  setParam("/lidar_update/ndt_transformation_epsilon", 0.001);
  setParam("/lidar_update/ndt_max_iterations", 1);
  setParam("/lidar_update/ndt_step_limit_enable", true);
  setParam("/lidar_update/ndt_step_limit_max_translation", 0.15);
  setParam("/external_transaction/queue_capacity", 8);
  setParam("/external_transaction/terminal_cache_max_entries", 100);
  setParam("/external_transaction/request_topic", std::string("/test/ndt/request"));
  setParam("/external_transaction/result_topic", std::string("/test/ndt/result"));
  setParam("/external_transaction/control_topic", std::string("/test/ndt/control"));
  setParam("/external_transaction/control_ack_topic", std::string("/test/ndt/control_ack"));
  setParam("/external_transaction/status_topic", std::string("/test/ndt/status"));
  setParam("/external_transaction/ack_topic", std::string("/test/ndt/ack"));

  ros::NodeHandle nh;
  Probe probe;
  auto status_sub = nh.subscribe("/test/ndt/status", 4, &Probe::status, &probe);
  auto control_ack_sub = nh.subscribe("/test/ndt/control_ack", 4, &Probe::ack, &probe);
  auto result_sub = nh.subscribe("/test/ndt/result", 16, &Probe::result, &probe);
  auto request_pub = nh.advertise<Request>("/test/ndt/request", 8, false);
  auto control_pub = nh.advertise<Control>("/test/ndt/control", 4, false);
  dog_prior_map_localization::ExternalNdtTransactionServer server;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  Status status;
  if (!probe.waitStatus(&status) || !status.server_ready ||
      !status.external_mode_enabled || status.map_frame != "map" ||
      status.lidar_frame != "lidar") {
    std::cerr << "FAIL: external NDT status did not become ready\n";
    return 1;
  }
  const std::string session = "00000000-0000-4000-8000-000000000001";
  Control control;
  control.command = Control::BEGIN_SESSION;
  control.protocol_version = Status::PROTOCOL_VERSION;
  control.server_instance_id = status.server_instance_id;
  control.frontend_session_id = session;
  control.epoch = 0;
  control.reason = Control::REASON_FRONTEND_START;
  control.expected_map_sha256 = status.map_sha256;
  control.expected_ndt_config_sha256 = status.canonical_ndt_config_sha256;
  control_pub.publish(control);
  ControlAck control_ack;
  if (!probe.waitAck(&control_ack) || control_ack.result != ControlAck::ACCEPTED) {
    std::cerr << "FAIL: BEGIN_SESSION was not accepted\n";
    return 1;
  }

  while (request_pub.getNumSubscribers() == 0 && ros::ok())
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const Eigen::Matrix4d true_map_T_lidar = pose(0.28, -0.16, 0.11, 0.035);
  const uint64_t first_end_ns = 2100000000ULL;
  ros::Time first_stamp;
  first_stamp.fromNSec(first_end_ns);
  const sensor_msgs::PointCloud2 success_cloud = makeSource(
      *map, true_map_T_lidar, "lidar", first_stamp);
  const Request first_request = requestFor(1, status, session, true_map_T_lidar,
                                           success_cloud);
  const auto first_identity =
      dog_prior_map_localization::detail::makeNdtRequestIdentity(first_request);
  if (!dog_prior_map_localization::detail::exactNdtRequestIdentity(
          first_identity, first_identity)) {
    std::cerr << "FAIL: compact request identity is not reflexive\n";
    return 1;
  }
  request_pub.publish(first_request);
  Result success;
  if (!probe.waitResult(1, &success) || success.disposition != Result::SUCCESS ||
      !success.pose_valid || !success.converged) {
    std::cerr << "FAIL: synthetic NDT registration did not succeed\n";
    return 1;
  }
  const Eigen::Vector3d result_translation(
      success.used_map_T_lidar.pose.position.x,
      success.used_map_T_lidar.pose.position.y,
      success.used_map_T_lidar.pose.position.z);
  const Eigen::Quaterniond result_rotation(
      success.used_map_T_lidar.pose.orientation.w,
      success.used_map_T_lidar.pose.orientation.x,
      success.used_map_T_lidar.pose.orientation.y,
      success.used_map_T_lidar.pose.orientation.z);
  const Eigen::Vector3d expected_translation = true_map_T_lidar.block<3, 1>(0, 3);
  const Eigen::Quaterniond expected_rotation(true_map_T_lidar.block<3, 3>(0, 0));
  const double translation_error = (result_translation - expected_translation).norm();
  const double rotation_error = Eigen::AngleAxisd(
      expected_rotation.conjugate() * result_rotation).angle();
  if (translation_error > 0.12 || rotation_error > 0.08) {
    std::cerr << "FAIL: NDT result did not retain/use predicted initial guess: "
              << translation_error << " m / " << rotation_error << " rad\n";
    return 1;
  }

  Request cloud_payload_conflict = first_request;
  cloud_payload_conflict.cloud_end_frame.data.at(0) ^= 0x01U;
  uint64_t prior_result_count = probe.resultCount(1);
  request_pub.publish(cloud_payload_conflict);
  Result cloud_payload_conflict_result;
  if (!probe.waitResultAfter(1, prior_result_count, &cloud_payload_conflict_result) ||
      cloud_payload_conflict_result.disposition != Result::ERROR_PROTOCOL ||
      cloud_payload_conflict_result.pose_valid ||
      cloud_payload_conflict_result.reason != "conflicting_duplicate_request_identity") {
    std::cerr << "FAIL: changed point-cloud bytes with unchanged declared hash were not rejected as a duplicate conflict\n";
    return 1;
  }

  Request cloud_conflict = first_request;
  ++cloud_conflict.request_cloud_hash;
  prior_result_count = probe.resultCount(1);
  request_pub.publish(cloud_conflict);
  Result cloud_conflict_result;
  if (!probe.waitResultAfter(1, prior_result_count, &cloud_conflict_result) ||
      cloud_conflict_result.disposition != Result::ERROR_PROTOCOL ||
      cloud_conflict_result.reason != "conflicting_duplicate_request_identity") {
    std::cerr << "FAIL: changed request cloud hash was not rejected as a duplicate conflict\n";
    return 1;
  }

  Request pose_conflict = first_request;
  pose_conflict.predicted_map_T_lidar.pose.position.x += 0.01;
  prior_result_count = probe.resultCount(1);
  request_pub.publish(pose_conflict);
  Result pose_conflict_result;
  if (!probe.waitResultAfter(1, prior_result_count, &pose_conflict_result) ||
      pose_conflict_result.disposition != Result::ERROR_PROTOCOL ||
      pose_conflict_result.reason != "conflicting_duplicate_request_identity") {
    std::cerr << "FAIL: changed predicted pose was not rejected as a duplicate conflict\n";
    return 1;
  }

  const Eigen::Matrix4d second_true_pose = pose(0.80, -0.16, 0.11, 0.035);
  ros::Time second_scan_stamp;
  second_scan_stamp.fromNSec(2200000000ULL);
  const auto second_cloud = makeSource(*map, second_true_pose, "lidar", second_scan_stamp);
  request_pub.publish(requestFor(2, status, session, second_true_pose, second_cloud));
  Result second_success;
  if (!probe.waitResult(2, &second_success) ||
      second_success.disposition != Result::SUCCESS || !second_success.pose_valid) {
    std::cerr << "FAIL: second scan did not succeed for duplicate idempotence check\n";
    return 1;
  }

  const Eigen::Matrix4d third_true_pose = pose(1.32, -0.16, 0.11, 0.035);
  ros::Time third_scan_stamp;
  third_scan_stamp.fromNSec(2300000000ULL);
  const auto third_cloud = makeSource(*map, third_true_pose, "lidar", third_scan_stamp);
  request_pub.publish(requestFor(3, status, session, third_true_pose, third_cloud));
  Result third_success;
  if (!probe.waitResult(3, &third_success) ||
      third_success.disposition != Result::SUCCESS || !third_success.pose_valid ||
      (usedTranslation(third_success) - usedTranslation(success)).norm() < 0.20) {
    std::cerr << "FAIL: later transactions did not advance NDT limiter state\n";
    return 1;
  }

  prior_result_count = probe.resultCount(1);
  request_pub.publish(first_request);
  Result cached_duplicate;
  const bool got_cached_duplicate =
      probe.waitResultAfter(1, prior_result_count, &cached_duplicate);
  const auto cached_bytes = got_cached_duplicate ? serializeResultPayload(cached_duplicate)
                                                  : std::vector<uint8_t>();
  const auto first_bytes = serializeResultPayload(success);
  if (!got_cached_duplicate || cached_bytes != first_bytes ||
      (usedTranslation(cached_duplicate) - usedTranslation(success)).norm() > 0.0) {
    std::size_t mismatch = 0;
    while (mismatch < cached_bytes.size() && mismatch < first_bytes.size() &&
           cached_bytes[mismatch] == first_bytes[mismatch]) ++mismatch;
    std::cerr << "FAIL: identical duplicate did not return exact cached result; disposition="
              << static_cast<int>(cached_duplicate.disposition)
              << " reason=" << cached_duplicate.reason
              << " first_x=" << success.used_map_T_lidar.pose.position.x
              << " tx3_x=" << third_success.used_map_T_lidar.pose.position.x
              << " duplicate_x=" << cached_duplicate.used_map_T_lidar.pose.position.x
              << " fitness=" << success.fitness << "/" << cached_duplicate.fitness
              << " iter=" << success.iterations << "/" << cached_duplicate.iterations
              << " bytes=" << first_bytes.size() << "/" << cached_bytes.size()
              << " mismatch_offset=" << mismatch
              << " result_count=" << probe.resultCount(1)
              << " ndt may have rerun\n";
    return 1;
  }

  const uint64_t insufficient_end_ns = 2400000000ULL;
  ros::Time insufficient_stamp;
  insufficient_stamp.fromNSec(insufficient_end_ns);
  const sensor_msgs::PointCloud2 small_cloud = makeSource(
      *map, true_map_T_lidar, "lidar", insufficient_stamp,
      Eigen::Vector3d::Zero(), 10);
  request_pub.publish(requestFor(4, status, session, true_map_T_lidar,
                                 small_cloud));
  Result insufficient;
  if (!probe.waitResult(4, &insufficient) ||
      insufficient.disposition != Result::REJECT_INSUFFICIENT_POINTS ||
      insufficient.pose_valid) {
    std::cerr << "FAIL: insufficient-point transaction disposition mismatch\n";
    return 1;
  }

  std::cout << "EXTERNAL_NDT_SERVER_INTEGRATION_PASS"
            << " success_guess_translation_error_m=" << translation_error
            << " success_guess_rotation_error_rad=" << rotation_error
            << " success=PASS insufficient=PASS\n";
  return 0;
}
