#include "dog_prior_map_fastlio2_frontend_exp/frontend_runtime.hpp"

#include <dog_prior_map_interfaces/NdtScanAck.h>
#include <dog_prior_map_interfaces/NdtScanRequest.h>
#include <dog_prior_map_interfaces/NdtScanResult.h>
#include <dog_prior_map_interfaces/NdtServerStatus.h>
#include <dog_prior_map_interfaces/NdtSessionControl.h>
#include <dog_prior_map_interfaces/NdtSessionControlAck.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

using Request = dog_prior_map_interfaces::NdtScanRequest;
using Result = dog_prior_map_interfaces::NdtScanResult;
using Status = dog_prior_map_interfaces::NdtServerStatus;
using Control = dog_prior_map_interfaces::NdtSessionControl;
using ControlAck = dog_prior_map_interfaces::NdtSessionControlAck;
using ScanAck = dog_prior_map_interfaces::NdtScanAck;
using ImuBuffer = std::deque<ImuSample, Eigen::aligned_allocator<ImuSample>>;

bool finitePose(const Pose3d& pose) {
  const double norm = pose.orientation.norm();
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
         std::isfinite(norm) && norm > 1e-12 && std::abs(norm - 1.0) <= 1e-6;
}

Eigen::Isometry3d toIsometry(const Pose3d& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.orientation.toRotationMatrix();
  transform.translation() = pose.position;
  return transform;
}

Pose3d fromIsometry(const Eigen::Isometry3d& transform) {
  Pose3d pose;
  pose.position = transform.translation();
  pose.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return pose;
}

void fillPose(geometry_msgs::Pose* output, const Pose3d& pose) {
  output->position.x = pose.position.x();
  output->position.y = pose.position.y();
  output->position.z = pose.position.z();
  output->orientation.x = pose.orientation.x();
  output->orientation.y = pose.orientation.y();
  output->orientation.z = pose.orientation.z();
  output->orientation.w = pose.orientation.w();
}

Pose3d readPose(const geometry_msgs::Pose& input) {
  Pose3d pose;
  pose.position = Eigen::Vector3d(input.position.x, input.position.y,
                                  input.position.z);
  pose.orientation = Eigen::Quaterniond(input.orientation.w,
                                        input.orientation.x,
                                        input.orientation.y,
                                        input.orientation.z);
  if (pose.orientation.norm() > 1e-12) pose.orientation.normalize();
  return pose;
}

bool isUuidV4(const std::string& value) {
  if (value.size() != 36 || value[8] != '-' || value[13] != '-' ||
      value[18] != '-' || value[23] != '-' || value[14] != '4') return false;
  const char variant = static_cast<char>(std::tolower(value[19]));
  if (variant != '8' && variant != '9' && variant != 'a' && variant != 'b')
    return false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 8 || i == 13 || i == 18 || i == 23) continue;
    if (!std::isxdigit(static_cast<unsigned char>(value[i]))) return false;
  }
  return true;
}

std::string makeUuidV4() {
  std::random_device random;
  std::mt19937_64 generator(random());
  std::array<uint8_t, 16> bytes{};
  for (std::size_t offset = 0; offset < bytes.size(); offset += 8) {
    const uint64_t value = generator();
    for (int byte = 0; byte < 8; ++byte)
      bytes[offset + byte] = static_cast<uint8_t>(value >> (8 * byte));
  }
  bytes[6] = static_cast<uint8_t>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<uint8_t>((bytes[8] & 0x3fU) | 0x80U);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) output << '-';
    output << std::setw(2) << static_cast<unsigned>(bytes[i]);
  }
  return output.str();
}

bool parseSha256(const std::string& value, std::array<uint8_t, 32>* bytes) {
  if (!bytes || value.size() != 64) return false;
  auto nibble = [](char character) -> int {
    if (character >= '0' && character <= '9') return character - '0';
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    return -1;
  };
  for (std::size_t i = 0; i < bytes->size(); ++i) {
    const int high = nibble(value[2 * i]);
    const int low = nibble(value[2 * i + 1]);
    if (high < 0 || low < 0) return false;
    (*bytes)[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

bool statusHashMatches(const boost::array<uint8_t, 32>& actual,
                       const std::array<uint8_t, 32>& expected) {
  return std::equal(actual.begin(), actual.end(), expected.begin());
}

std::vector<uint8_t> serialized(const Result& result) {
  std::vector<uint8_t> bytes(ros::serialization::serializationLength(result));
  ros::serialization::OStream stream(bytes.data(),
                                     static_cast<uint32_t>(bytes.size()));
  ros::serialization::serialize(stream, result);
  return bytes;
}

uint64_t cloudHash(const sensor_msgs::PointCloud2& cloud) {
  constexpr uint64_t kOffset = 14695981039346656037ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  auto byte = [&hash](uint8_t value) { hash = (hash ^ value) * kPrime; };
  auto u32 = [&byte](uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8)
      byte(static_cast<uint8_t>(value >> shift));
  };
  auto string = [&u32, &byte](const std::string& value) {
    u32(static_cast<uint32_t>(value.size()));
    for (unsigned char character : value) byte(character);
  };
  string(cloud.header.frame_id);
  u32(cloud.header.stamp.sec);
  u32(cloud.header.stamp.nsec);
  u32(cloud.width);
  u32(cloud.height);
  u32(static_cast<uint32_t>(cloud.fields.size()));
  for (const sensor_msgs::PointField& field : cloud.fields) {
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

const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                         const std::string& name) {
  for (const sensor_msgs::PointField& field : cloud.fields)
    if (field.name == name) return &field;
  return nullptr;
}

bool fieldValue(const sensor_msgs::PointCloud2& cloud, std::size_t point_index,
                const sensor_msgs::PointField& field, double* output) {
  if (!output || cloud.is_bigendian || field.count != 1) return false;
  if (cloud.width == 0) return false;
  const std::size_t row = point_index / cloud.width;
  const std::size_t column = point_index % cloud.width;
  const std::size_t offset = row * cloud.row_step + column * cloud.point_step + field.offset;
  if (offset >= cloud.data.size()) return false;
  switch (field.datatype) {
    case sensor_msgs::PointField::FLOAT32: {
      float value;
      if (offset + sizeof(value) > cloud.data.size()) return false;
      std::memcpy(&value, cloud.data.data() + offset, sizeof(value));
      *output = value;
      return true;
    }
    case sensor_msgs::PointField::FLOAT64: {
      double value;
      if (offset + sizeof(value) > cloud.data.size()) return false;
      std::memcpy(&value, cloud.data.data() + offset, sizeof(value));
      *output = value;
      return true;
    }
    case sensor_msgs::PointField::UINT32: {
      uint32_t value;
      if (offset + sizeof(value) > cloud.data.size()) return false;
      std::memcpy(&value, cloud.data.data() + offset, sizeof(value));
      *output = value;
      return true;
    }
    default:
      return false;
  }
}

bool makeTimedCloud(const sensor_msgs::PointCloud2& input,
                    const std::string& lidar_frame,
                    const std::string& point_time_field,
                    double point_time_scale_seconds,
                    double max_scan_duration_seconds,
                    uint64_t* scan_start_ns, uint64_t* scan_end_ns,
                    std::vector<TimedLidarPoint,
                                Eigen::aligned_allocator<TimedLidarPoint>>* points,
                    std::string* reason) {
  auto fail = [reason](const char* text) {
    if (reason) *reason = text;
    return false;
  };
  if (!scan_start_ns || !scan_end_ns || !points || input.header.stamp.isZero() ||
      input.header.frame_id != lidar_frame || input.height == 0 ||
      input.point_step == 0 ||
      input.data.size() < static_cast<std::size_t>(input.row_step) * input.height)
    return fail("invalid_pointcloud2_envelope");
  const auto* x_field = findField(input, "x");
  const auto* y_field = findField(input, "y");
  const auto* z_field = findField(input, "z");
  const auto* intensity_field = findField(input, "intensity");
  const auto* time_field = findField(input, point_time_field);
  if (!x_field || !y_field || !z_field || !time_field || input.is_bigendian)
    return fail("pointcloud2_requires_little_endian_xyz_and_point_time_fields");
  const uint64_t start = input.header.stamp.toNSec();
  points->clear();
  points->reserve(static_cast<std::size_t>(input.width) * input.height);
  double max_offset_seconds = -std::numeric_limits<double>::infinity();
  const std::size_t point_count = static_cast<std::size_t>(input.width) * input.height;
  for (std::size_t index = 0; index < point_count; ++index) {
    double x = 0.0, y = 0.0, z = 0.0, time_value = 0.0, intensity = 0.0;
    if (!fieldValue(input, index, *x_field, &x) ||
        !fieldValue(input, index, *y_field, &y) ||
        !fieldValue(input, index, *z_field, &z) ||
        !fieldValue(input, index, *time_field, &time_value) ||
        (intensity_field && !fieldValue(input, index, *intensity_field, &intensity)))
      return fail("unsupported_pointcloud2_field_layout");
    const double offset_seconds = time_value * point_time_scale_seconds;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(intensity) || !std::isfinite(offset_seconds) ||
        offset_seconds < 0.0 || offset_seconds > max_scan_duration_seconds)
      return fail("nonfinite_point_or_point_time_outside_scan");
    const long double offset_ns_real = static_cast<long double>(offset_seconds) * 1e9L;
    if (offset_ns_real > static_cast<long double>(std::numeric_limits<uint64_t>::max() - start))
      return fail("point_time_overflows_ros_timestamp");
    const uint64_t offset_ns = static_cast<uint64_t>(std::llround(offset_ns_real));
    TimedLidarPoint point;
    point.position = Eigen::Vector3d(x, y, z);
    point.intensity = intensity;
    point.stamp_ns = start + offset_ns;
    points->push_back(point);
    max_offset_seconds = std::max(max_offset_seconds, offset_seconds);
  }
  if (points->empty() || max_offset_seconds <= 0.0)
    return fail("empty_cloud_or_zero_scan_duration");
  const long double end_ns_real = static_cast<long double>(start) +
                                  max_offset_seconds * 1e9L;
  if (end_ns_real > static_cast<long double>(std::numeric_limits<uint64_t>::max()))
    return fail("scan_end_overflows_ros_timestamp");
  *scan_start_ns = start;
  *scan_end_ns = static_cast<uint64_t>(std::llround(end_ns_real));
  if (*scan_end_ns <= *scan_start_ns) return fail("invalid_scan_interval");
  return true;
}

sensor_msgs::PointCloud2 makeCloudMessage(
    const ScanEndResult& scan, const std::string& lidar_frame) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp.fromNSec(scan.scan_end_ns);
  cloud.header.frame_id = lidar_frame;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(scan.cloud_end_frame.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(4);
  const char* names[] = {"x", "y", "z", "intensity"};
  for (std::size_t i = 0; i < 4; ++i) {
    cloud.fields[i].name = names[i];
    cloud.fields[i].offset = static_cast<uint32_t>(4 * i);
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  for (std::size_t i = 0; i < scan.cloud_end_frame.size(); ++i) {
    const auto& point = scan.cloud_end_frame[i];
    const float fields[4] = {static_cast<float>(point.position.x()),
                             static_cast<float>(point.position.y()),
                             static_cast<float>(point.position.z()),
                             static_cast<float>(point.intensity)};
    std::memcpy(cloud.data.data() + i * cloud.point_step, fields, sizeof(fields));
  }
  return cloud;
}

class FastLio2FrontendNode {
 public:
  FastLio2FrontendNode()
      : nh_(), pnh_("~"), runtime_parameters_(readRuntimeParameters()),
        runtime_(runtime_parameters_) {
    readConfiguration();
    imu_sub_ = nh_.subscribe(imu_topic_, 5000,
                             &FastLio2FrontendNode::imuCallback, this);
    lidar_sub_ = nh_.subscribe(lidar_topic_, 32,
                               &FastLio2FrontendNode::lidarCallback, this);
    result_sub_ = nh_.subscribe(result_topic_, 100,
                                &FastLio2FrontendNode::resultCallback, this);
    status_sub_ = nh_.subscribe(status_topic_, 4,
                                &FastLio2FrontendNode::statusCallback, this);
    control_ack_sub_ = nh_.subscribe(control_ack_topic_, 16,
                                     &FastLio2FrontendNode::controlAckCallback, this);
    request_pub_ = nh_.advertise<Request>(request_topic_, 10, false);
    control_pub_ = nh_.advertise<Control>(control_topic_, 4, false);
    ack_pub_ = nh_.advertise<ScanAck>(ack_topic_, 20, false);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 100, false);
    worker_ = std::thread(&FastLio2FrontendNode::workerLoop, this);
    ROS_INFO("[FAST-LIO2 frontend] waiting for valid NDT status, explicit initial pose, and IMU initialization");
  }

  ~FastLio2FrontendNode() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  RuntimeParameters readRuntimeParameters() {
    RuntimeParameters parameters;
    int static_samples = parameters.static_init_samples;
    pnh_.param("imu/static_init_samples", static_samples, static_samples);
    parameters.static_init_samples = static_samples;
    pnh_.param("imu/gravity_mps2", parameters.gravity_mps2,
               parameters.gravity_mps2);
    pnh_.param("imu/max_static_gyro_std_rad_s", parameters.max_static_gyro_std_rad_s,
               parameters.max_static_gyro_std_rad_s);
    pnh_.param("imu/max_static_accel_std_m_s2", parameters.max_static_accel_std_m_s2,
               parameters.max_static_accel_std_m_s2);
    pnh_.param("imu/gyro_noise_std_rad_s", parameters.gyro_noise_std_rad_s,
               parameters.gyro_noise_std_rad_s);
    pnh_.param("imu/accel_noise_std_m_s2", parameters.accel_noise_std_m_s2,
               parameters.accel_noise_std_m_s2);
    pnh_.param("imu/gyro_bias_rw_std_rad_s2", parameters.gyro_bias_rw_std_rad_s2,
               parameters.gyro_bias_rw_std_rad_s2);
    pnh_.param("imu/accel_bias_rw_std_m_s3", parameters.accel_bias_rw_std_m_s3,
               parameters.accel_bias_rw_std_m_s3);
    pnh_.param("update/pose_position_sigma_m", parameters.pose_position_sigma_m,
               parameters.pose_position_sigma_m);
    pnh_.param("update/pose_rotation_sigma_rad", parameters.pose_rotation_sigma_rad,
               parameters.pose_rotation_sigma_rad);
    return parameters;
  }

  bool readPoseParameters(const std::string& prefix, Pose3d* pose) {
    if (!pose) return false;
    return pnh_.getParam(prefix + "/x", pose->position.x()) &&
           pnh_.getParam(prefix + "/y", pose->position.y()) &&
           pnh_.getParam(prefix + "/z", pose->position.z()) &&
           pnh_.getParam(prefix + "/qx", pose->orientation.x()) &&
           pnh_.getParam(prefix + "/qy", pose->orientation.y()) &&
           pnh_.getParam(prefix + "/qz", pose->orientation.z()) &&
           pnh_.getParam(prefix + "/qw", pose->orientation.w()) && finitePose(*pose);
  }

  void readConfiguration() {
    pnh_.param<std::string>("topics/imu", imu_topic_, "/imu/data");
    pnh_.param<std::string>("topics/lidar", lidar_topic_, "/points_raw");
    pnh_.param<std::string>("topics/request", request_topic_, "/dog_livo/ndt/scan_request");
    pnh_.param<std::string>("topics/result", result_topic_, "/dog_livo/ndt/scan_result");
    pnh_.param<std::string>("topics/status", status_topic_, "/dog_livo/ndt/server_status");
    pnh_.param<std::string>("topics/control", control_topic_, "/dog_livo/ndt/session_control");
    pnh_.param<std::string>("topics/control_ack", control_ack_topic_, "/dog_livo/ndt/session_control_ack");
    pnh_.param<std::string>("topics/ack", ack_topic_, "/dog_livo/ndt/scan_ack");
    pnh_.param<std::string>("topics/odometry", odom_topic_, "/dog_livo/fastlio2_ndt_odom");
    pnh_.param<std::string>("frames/map", map_frame_, "camera_init");
    pnh_.param<std::string>("frames/lidar", lidar_frame_, "velodyne");
    pnh_.param<std::string>("lidar/point_time_field", point_time_field_, "time");
    pnh_.param("lidar/point_time_scale_to_seconds", point_time_scale_seconds_, 1.0);
    pnh_.param("lidar/max_scan_duration_seconds", max_scan_duration_seconds_, 0.25);
    pnh_.param("queues/max_imu_samples", max_imu_samples_, 20000);
    pnh_.param("queues/max_scans", max_scans_, 64);
    pnh_.param("queues/max_results", max_results_, 64);
    int protocol_version = static_cast<int>(protocol_version_);
    int cache_entries = static_cast<int>(expected_cache_entries_);
    pnh_.param("protocol/version", protocol_version, protocol_version);
    pnh_.param("protocol/terminal_cache_max_entries", cache_entries, cache_entries);
    protocol_version_ = static_cast<uint32_t>(std::max(protocol_version, 0));
    expected_cache_entries_ = static_cast<uint64_t>(std::max(cache_entries, 0));
    pnh_.param("protocol/result_timeout_wall_seconds", result_timeout_seconds_, 5.0);
    std::string map_sha, config_sha;
    pnh_.param<std::string>("protocol/map_sha256", map_sha, "");
    pnh_.param<std::string>("protocol/canonical_ndt_config_sha256", config_sha, "");
    hashes_valid_ = parseSha256(map_sha, &expected_map_sha_) &&
                    parseSha256(config_sha, &expected_config_sha_);
    initial_pose_valid_ = readPoseParameters("initial_map_T_lidar", &initial_map_T_lidar_);
    extrinsic_valid_ = readPoseParameters("T_imu_lidar", &T_imu_lidar_);
    if (!hashes_valid_)
      ROS_WARN("[FAST-LIO2 frontend] WAIT_SERVER: expected map/config SHA-256 parameters are missing or malformed");
    if (!initial_pose_valid_)
      ROS_WARN("[FAST-LIO2 frontend] WAIT_INIT: explicit initial_map_T_lidar is absent or invalid; identity is not substituted");
    if (!extrinsic_valid_)
      ROS_WARN("[FAST-LIO2 frontend] WAIT_INIT: calibrated T_imu_lidar is absent or invalid; identity is not substituted");
    if (runtime_parameters_.static_init_samples < 2 || max_imu_samples_ < 2 ||
        max_scans_ < 1 || max_results_ < 1 ||
        !std::isfinite(point_time_scale_seconds_) || point_time_scale_seconds_ <= 0.0 ||
        !std::isfinite(max_scan_duration_seconds_) || max_scan_duration_seconds_ <= 0.0 ||
        !std::isfinite(result_timeout_seconds_) || result_timeout_seconds_ <= 0.0)
      configuration_valid_ = false;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    if (!message) return;
    ImuSample sample;
    sample.stamp_ns = message->header.stamp.toNSec();
    sample.acceleration = Eigen::Vector3d(message->linear_acceleration.x,
        message->linear_acceleration.y, message->linear_acceleration.z);
    sample.angular_velocity = Eigen::Vector3d(message->angular_velocity.x,
        message->angular_velocity.y, message->angular_velocity.z);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) return;
      if (imu_buffer_.size() >= static_cast<std::size_t>(max_imu_samples_)) {
        callback_fatal_ = "IMU input buffer capacity exceeded";
      } else {
        imu_buffer_.push_back(sample);
      }
    }
    condition_.notify_all();
  }

  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    if (!message) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) return;
      if (scan_queue_.size() >= static_cast<std::size_t>(max_scans_))
        callback_fatal_ = "LiDAR scan queue capacity exceeded";
      else
        scan_queue_.push_back(message);
    }
    condition_.notify_all();
  }

  void resultCallback(const Result::ConstPtr& message) {
    if (!message) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) return;
      if (result_queue_.size() >= static_cast<std::size_t>(max_results_))
        callback_fatal_ = "NDT result queue capacity exceeded";
      else
        result_queue_.push_back(*message);
    }
    condition_.notify_all();
  }

  void statusCallback(const Status::ConstPtr& message) {
    if (!message) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_status_ = *message;
      have_status_ = true;
    }
    condition_.notify_all();
  }

  void controlAckCallback(const ControlAck::ConstPtr& message) {
    if (!message) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (control_ack_queue_.size() >= 16)
        callback_fatal_ = "NDT control-ACK queue capacity exceeded";
      else
        control_ack_queue_.push_back(*message);
    }
    condition_.notify_all();
  }

  bool currentStatus(Status* status) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_status_) return false;
    *status = latest_status_;
    return true;
  }

  bool statusIsReadyAndExpected(const Status& status, std::string* reason) const {
    if (!configuration_valid_) *reason = "invalid frontend ROS parameters";
    else if (!hashes_valid_) *reason = "expected map/config SHA-256 missing";
    else if (!status.server_ready) *reason = status.fatal_latched
        ? "NDT server has fatal_latched" : "NDT server is not ready";
    else if (!status.external_mode_enabled) *reason = "NDT external transaction mode is disabled";
    else if (status.protocol_version != protocol_version_) *reason = "protocol version mismatch";
    else if (!statusHashMatches(status.map_sha256, expected_map_sha_)) *reason = "map SHA-256 mismatch";
    else if (!statusHashMatches(status.canonical_ndt_config_sha256, expected_config_sha_))
      *reason = "canonical NDT config SHA-256 mismatch";
    else if (!isUuidV4(status.server_instance_id)) *reason = "server_instance_id is not UUIDv4";
    else if (status.map_frame != map_frame_) *reason = "map frame mismatch";
    else if (status.lidar_frame != lidar_frame_) *reason = "LiDAR frame mismatch";
    else if (status.terminal_cache_max_entries != expected_cache_entries_ ||
             status.terminal_cache_max_entries == 0)
      *reason = "terminal cache capacity mismatch";
    else {
      if (reason) reason->clear();
      return true;
    }
    return false;
  }

  bool bindSession() {
    Status status;
    std::string reason;
    if (!currentStatus(&status)) return false;
    if (!statusIsReadyAndExpected(status, &reason)) {
      ROS_WARN_THROTTLE(2.0, "[FAST-LIO2 frontend] WAIT_SERVER: %s", reason.c_str());
      return false;
    }
    bound_server_instance_ = status.server_instance_id;
    frontend_session_id_ = makeUuidV4();
    ROS_INFO("[FAST-LIO2 frontend] bound NDT server %s; waiting for static IMU initialization",
             bound_server_instance_.c_str());
    return true;
  }

  bool takeStaticImu(std::vector<ImuSample,
                                  Eigen::aligned_allocator<ImuSample>>* samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!samples || imu_buffer_.size() <
        static_cast<std::size_t>(runtime_parameters_.static_init_samples)) return false;
    samples->clear();
    for (int i = 0; i < runtime_parameters_.static_init_samples; ++i)
      samples->push_back(imu_buffer_[static_cast<std::size_t>(i)]);
    return true;
  }

  bool sendBeginSession() {
    const ros::WallTime connect_deadline =
        ros::WallTime::now() + ros::WallDuration(3.0);
    while (ros::ok() && control_pub_.getNumSubscribers() == 0 &&
           (ros::WallTime::now() - connect_deadline).toSec() < 0.0)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (control_pub_.getNumSubscribers() == 0) {
      setFatal("NDT session-control subscriber did not connect");
      return false;
    }
    Control control;
    control.header.stamp = ros::Time::now();
    control.header.frame_id = map_frame_;
    control.command = Control::BEGIN_SESSION;
    control.protocol_version = protocol_version_;
    control.server_instance_id = bound_server_instance_;
    control.frontend_session_id = frontend_session_id_;
    control.epoch = 0;
    control.reason = Control::REASON_FRONTEND_START;
    std::copy(expected_map_sha_.begin(), expected_map_sha_.end(),
              control.expected_map_sha256.begin());
    std::copy(expected_config_sha_.begin(), expected_config_sha_.end(),
              control.expected_ndt_config_sha256.begin());
    control_pub_.publish(control);
    ROS_INFO("[FAST-LIO2 frontend] sent BEGIN_SESSION for %s",
             frontend_session_id_.c_str());
    return true;
  }

  bool waitForSessionBinding() {
    const ros::WallTime bind_deadline =
        ros::WallTime::now() + ros::WallDuration(5.0);
    bool accepted_ack = false;
    while (ros::ok() && !worker_fatal_) {
      if (ros::WallTime::now() >= bind_deadline) {
        setFatal("NDT BEGIN_SESSION wall-time timeout");
        return false;
      }
      if (checkServerRestart()) return false;
      ControlAck ack;
      bool have_ack = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!control_ack_queue_.empty()) {
          ack = control_ack_queue_.front();
          control_ack_queue_.pop_front();
          have_ack = true;
        } else {
          condition_.wait_for(lock, std::chrono::milliseconds(100));
        }
      }
      if (have_ack) {
        if (ack.frontend_session_id != frontend_session_id_ ||
            ack.server_instance_id != bound_server_instance_) continue;
        if (ack.command != Control::BEGIN_SESSION ||
            ack.protocol_version != protocol_version_ ||
            ack.epoch != 0 ||
            ack.result != ControlAck::ACCEPTED) {
          setFatal("NDT BEGIN_SESSION ACK rejected or identity mismatch: " + ack.reason);
          return false;
        }
        accepted_ack = true;
      }
      Status status;
      if (currentStatus(&status)) {
        if (status.fatal_latched || !status.server_ready) {
          setFatal("NDT server became not-ready/fatal during session bind");
          return false;
        }
        if (status.server_instance_id != bound_server_instance_) {
          setFatal("SERVER_RESTART_DETECTED during session bind");
          return false;
        }
        if (accepted_ack && status.bound_frontend_session_id == frontend_session_id_ &&
            status.epoch == 0) return true;
      }
    }
    return false;
  }

  bool checkServerRestart() {
    Status status;
    if (currentStatus(&status) && !bound_server_instance_.empty()) {
      if (status.server_instance_id != bound_server_instance_) {
        setFatal("SERVER_RESTART_DETECTED: NDT server instance changed");
        return true;
      }
      if (status.fatal_latched || !status.server_ready) {
        setFatal("bound NDT server became fatal or not ready");
        return true;
      }
      std::string reason;
      if (!statusIsReadyAndExpected(status, &reason)) {
        setFatal("bound NDT status identity changed: " + reason);
        return true;
      }
    }
    return false;
  }

  bool initializeFilter() {
    if (!initial_pose_valid_ || !extrinsic_valid_) return false;
    std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> samples;
    if (!takeStaticImu(&samples)) return false;
    std::string failure;
    if (!runtime_.initializeStatic(samples, initial_map_T_lidar_, T_imu_lidar_,
                                   &failure)) {
      setFatal("FAST-LIO2 static IMU initialization failed: " + failure);
      return false;
    }
    ROS_INFO("[FAST-LIO2 frontend] initialized at sensor time %llu ns",
             static_cast<unsigned long long>(runtime_.committedState().stamp_ns));
    if (!sendBeginSession()) return false;
    if (!waitForSessionBinding()) return false;
    session_bound_ = true;
    return true;
  }

  bool parseScan(const sensor_msgs::PointCloud2& message, uint64_t* start_ns,
                 uint64_t* end_ns,
                 std::vector<TimedLidarPoint,
                             Eigen::aligned_allocator<TimedLidarPoint>>* points) {
    std::string reason;
    if (!makeTimedCloud(message, lidar_frame_, point_time_field_,
                        point_time_scale_seconds_, max_scan_duration_seconds_,
                        start_ns, end_ns, points, &reason)) {
      setFatal("invalid raw LiDAR scan: " + reason);
      return false;
    }
    return true;
  }

  bool imuForScan(uint64_t state_stamp_ns, uint64_t scan_end_ns,
                  std::vector<ImuSample,
                              Eigen::aligned_allocator<ImuSample>>* output) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto has_watermark = [this, scan_end_ns] {
      return stopping_ || callback_fatal_.size() ||
          (!imu_buffer_.empty() && imu_buffer_.back().stamp_ns >= scan_end_ns);
    };
    condition_.wait(lock, has_watermark);
    if (stopping_ || !callback_fatal_.empty()) return false;
    output->clear();
    std::size_t first = 0;
    while (first + 1 < imu_buffer_.size() &&
           imu_buffer_[first + 1].stamp_ns <= state_stamp_ns) ++first;
    if (imu_buffer_[first].stamp_ns > state_stamp_ns) {
      setFatal("IMU buffer no longer brackets committed filter time");
      return false;
    }
    for (std::size_t index = first; index < imu_buffer_.size(); ++index) {
      if (imu_buffer_[index].stamp_ns > scan_end_ns) break;
      if (!output->empty() && imu_buffer_[index].stamp_ns <= output->back().stamp_ns) {
        setFatal("IMU timestamps are not strictly increasing");
        return false;
      }
      output->push_back(imu_buffer_[index]);
    }
    lock.unlock();
    if (output->size() < 2) {
      setFatal("fewer than two causal IMU samples cover scan interval");
      return false;
    }
    return true;
  }

  bool makeRequest(const ScanEndResult& processed, uint64_t scan_start_ns,
                   uint64_t transaction_id, Request* request) {
    if (!request) return false;
    Request message;
    message.header.stamp.fromNSec(processed.scan_end_ns);
    message.header.frame_id = map_frame_;
    message.protocol_version = protocol_version_;
    message.frontend_session_id = frontend_session_id_;
    message.epoch = 0;
    message.transaction_id = transaction_id;
    message.server_instance_id = bound_server_instance_;
    message.scan_start.fromNSec(scan_start_ns);
    message.scan_end.fromNSec(processed.scan_end_ns);
    message.scan_start_ns = scan_start_ns;
    message.scan_end_ns = processed.scan_end_ns;
    message.map_frame = map_frame_;
    message.lidar_frame = lidar_frame_;
    message.cloud_end_frame = makeCloudMessage(processed, lidar_frame_);
    message.predicted_map_T_lidar.header = message.header;
    fillPose(&message.predicted_map_T_lidar.pose,
             processed.predicted_map_T_lidar);
    message.request_cloud_hash = cloudHash(message.cloud_end_frame);
    *request = std::move(message);
    return true;
  }

  bool validateTerminal(const Result& result, const Request& request,
                        std::string* reason) const {
    if (result.protocol_version != protocol_version_ ||
        result.frontend_session_id != frontend_session_id_ || result.epoch != 0 ||
        result.transaction_id != request.transaction_id ||
        result.server_instance_id != bound_server_instance_)
      *reason = "current transaction identity/version/server mismatch";
    else if (result.header.stamp.toNSec() != request.scan_end_ns ||
             result.scan_start.toNSec() != request.scan_start_ns ||
             result.scan_end.toNSec() != request.scan_end_ns ||
             result.scan_start_ns != request.scan_start_ns ||
             result.scan_end_ns != request.scan_end_ns ||
             result.raw_map_T_lidar.header.stamp.toNSec() != request.scan_end_ns ||
             result.used_map_T_lidar.header.stamp.toNSec() != request.scan_end_ns)
      *reason = "current transaction timestamp mirror mismatch";
    else if (result.header.frame_id != map_frame_ || result.map_frame != map_frame_ ||
             result.lidar_frame != lidar_frame_ ||
             result.raw_map_T_lidar.header.frame_id != map_frame_ ||
             result.used_map_T_lidar.header.frame_id != map_frame_)
      *reason = "current transaction frame mismatch";
    else if (result.request_cloud_hash != request.request_cloud_hash ||
             result.step_limited != (result.translation_limited || result.rotation_limited))
      *reason = "current transaction cloud hash or step-limit mirror mismatch";
    else if (result.disposition == Result::SUCCESS) {
      const Pose3d raw = readPose(result.raw_map_T_lidar.pose);
      const Pose3d used = readPose(result.used_map_T_lidar.pose);
      if (!result.pose_valid || !result.converged || !finitePose(raw) || !finitePose(used) ||
          !std::isfinite(result.fitness))
        *reason = "successful NDT result has invalid pose/fitness payload";
      else {
        if (reason) reason->clear();
        return true;
      }
    } else if (result.pose_valid) {
      *reason = "FATAL_CURRENT_TRANSACTION_CORRUPTION: non-success with pose_valid=true";
    } else if (result.disposition != Result::REJECT_INSUFFICIENT_POINTS &&
               result.disposition != Result::REJECT_NOT_CONVERGED) {
      *reason = "fatal NDT disposition=" + std::to_string(result.disposition);
    } else {
      if (reason) reason->clear();
      return true;
    }
    return false;
  }

  bool waitForResult(const Request& request, Result* output) {
    const ros::WallTime started = ros::WallTime::now();
    const double timeout = result_timeout_seconds_;
    while (ros::ok() && !worker_fatal_) {
      if (checkServerRestart()) return false;
      Result result;
      bool have_result = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!result_queue_.empty()) {
          result = std::move(result_queue_.front());
          result_queue_.pop_front();
          have_result = true;
        } else {
          const double elapsed = (ros::WallTime::now() - started).toSec();
          if (elapsed >= timeout) break;
          condition_.wait_for(lock, std::chrono::milliseconds(50));
        }
      }
      if (!have_result) continue;
      if (result.frontend_session_id != frontend_session_id_) continue;
      if (result.epoch < request.epoch) continue;
      if (result.epoch > request.epoch || result.transaction_id > request.transaction_id) {
        setFatal("unsolicited future NDT result");
        return false;
      }
      if (result.transaction_id < request.transaction_id) {
        const auto previous = terminal_ledger_.find(result.transaction_id);
        if (previous != terminal_ledger_.end() && previous->second != serialized(result)) {
          setFatal("FATAL_NONDETERMINISTIC_SERVER_RESULT for completed transaction");
          return false;
        }
        continue;
      }
      std::string reason;
      if (!validateTerminal(result, request, &reason)) {
        setFatal("invalid current NDT terminal: " + reason);
        return false;
      }
      *output = std::move(result);
      return true;
    }
    setFatal("NDT transaction wall-time timeout");
    return false;
  }

  void publishOdometry(const FilterSnapshot& state, uint64_t stamp_ns) {
    Pose3d map_T_imu = state.map_T_imu;
    const Pose3d map_T_lidar = fromIsometry(toIsometry(map_T_imu) *
                                            toIsometry(T_imu_lidar_));
    nav_msgs::Odometry odometry;
    odometry.header.stamp.fromNSec(stamp_ns);
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = lidar_frame_;
    fillPose(&odometry.pose.pose, map_T_lidar);
    odometry.twist.twist.linear.x = state.velocity.x();
    odometry.twist.twist.linear.y = state.velocity.y();
    odometry.twist.twist.linear.z = state.velocity.z();
    odom_pub_.publish(odometry);
  }

  void publishAck(const Request& request) {
    ScanAck ack;
    ack.frontend_session_id = request.frontend_session_id;
    ack.epoch = request.epoch;
    ack.transaction_id = request.transaction_id;
    ack.server_instance_id = request.server_instance_id;
    ack.scan_start_ns = request.scan_start_ns;
    ack.scan_end_ns = request.scan_end_ns;
    ack.request_cloud_hash = request.request_cloud_hash;
    ack_pub_.publish(ack);
  }

  bool processOneScan(const sensor_msgs::PointCloud2::ConstPtr& message) {
    uint64_t raw_scan_start_ns = 0, scan_end_ns = 0;
    std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> points;
    if (!parseScan(*message, &raw_scan_start_ns, &scan_end_ns, &points)) return false;
    const FilterSnapshot committed = runtime_.committedState();
    // Static initialization consumes an initial IMU-only window. Scans that
    // overlap it cannot be deskewed from that later initialized state.
    if (raw_scan_start_ns < committed.stamp_ns) {
      if (runtime_.counters().last_committed_transaction == 0) {
        ++initial_skipped_scans_;
        ROS_WARN_THROTTLE(2.0, "[FAST-LIO2 frontend] skipping scan overlapping static-init window: count=%llu start=%llu init=%llu",
                 static_cast<unsigned long long>(initial_skipped_scans_),
                 static_cast<unsigned long long>(raw_scan_start_ns),
                 static_cast<unsigned long long>(committed.stamp_ns));
        return true;
      }
    }

    ScanWindowDecision window_decision = ScanWindowDecision::PROCESS;
    ScanWindowStats window_stats;
    std::string failure;
    if (!prepareScanWindow(raw_scan_start_ns, scan_end_ns, committed.stamp_ns,
                           &points, &window_decision, &window_stats, &failure)) {
      setFatal("invalid scan time window: " + failure);
      return false;
    }
    if (window_decision == ScanWindowDecision::SKIP_STALE) {
      ++stale_skipped_scans_;
      ROS_WARN_THROTTLE(2.0, "[FAST-LIO2 frontend] skipping stale LiDAR scan: count=%llu raw_start=%llu end=%llu committed=%llu",
                        static_cast<unsigned long long>(stale_skipped_scans_),
                        static_cast<unsigned long long>(raw_scan_start_ns),
                        static_cast<unsigned long long>(scan_end_ns),
                        static_cast<unsigned long long>(committed.stamp_ns));
      return true;
    }
    const uint64_t scan_start_ns = window_stats.effective_scan_start_ns;
    if (window_stats.overlap_duration_ns > 0) {
      ++partial_overlap_scans_;
      overlap_points_dropped_ += window_stats.overlap_points_dropped;
      overlap_duration_ns_ += window_stats.overlap_duration_ns;
      ROS_WARN_THROTTLE(2.0, "[FAST-LIO2 frontend] trimming LiDAR overlap: duration_us=%.3f dropped=%zu remaining=%zu partial_count=%llu",
                        static_cast<double>(window_stats.overlap_duration_ns) / 1e3,
                        window_stats.overlap_points_dropped,
                        window_stats.remaining_points,
                        static_cast<unsigned long long>(partial_overlap_scans_));
    }
    if (terminal_ledger_.size() >= expected_cache_entries_) {
      setFatal("completed-result terminal ledger capacity exhausted before scan");
      return false;
    }
    std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> imu;
    if (!imuForScan(committed.stamp_ns, scan_end_ns, &imu)) return false;
    const uint64_t transaction_id = runtime_.counters().last_committed_transaction + 1;
    ScanEndResult processed;
    if (!runtime_.beginScan(transaction_id, scan_start_ns, scan_end_ns, imu,
                            points, &processed, &failure)) {
      setFatal("candidate prediction/deskew failed: " + failure);
      return false;
    }
    Request request;
    if (!makeRequest(processed, scan_start_ns, transaction_id, &request)) {
      setFatal("failed to construct NDT request");
      return false;
    }
    const ros::WallTime connect_deadline =
        ros::WallTime::now() + ros::WallDuration(3.0);
    while (ros::ok() && request_pub_.getNumSubscribers() == 0 &&
           (ros::WallTime::now() - connect_deadline).toSec() < 0.0)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (request_pub_.getNumSubscribers() == 0 || checkServerRestart()) {
      if (!worker_fatal_) setFatal("NDT scan-request subscriber did not connect");
      return false;
    }
    request_pub_.publish(request);
    Result result;
    if (!waitForResult(request, &result)) return false;

    RuntimeDisposition disposition = RuntimeDisposition::FATAL;
    if (result.disposition == Result::SUCCESS)
      disposition = RuntimeDisposition::SUCCESS;
    else if (result.disposition == Result::REJECT_INSUFFICIENT_POINTS)
      disposition = RuntimeDisposition::REJECT_INSUFFICIENT_POINTS;
    else if (result.disposition == Result::REJECT_NOT_CONVERGED)
      disposition = RuntimeDisposition::REJECT_NOT_CONVERGED;
    const Pose3d used = readPose(result.used_map_T_lidar.pose);
    PoseCorrectionDelta delta;
    if (!runtime_.finishScan(transaction_id, disposition, result.pose_valid,
                             used, &delta, &failure)) {
      setFatal("terminal transaction commit failed: " + failure);
      return false;
    }
    const auto payload = serialized(result);
    terminal_ledger_.emplace(transaction_id, payload);
    publishAck(request);
    const FilterSnapshot state = runtime_.committedState();
    pruneImuThrough(state.stamp_ns);
    publishOdometry(state, scan_end_ns);
    ++processed_scans_;
    if (result.disposition == Result::SUCCESS)
      ++ndt_success_count_;
    else
      ++ndt_reject_count_;
    const RuntimeCounters counters = runtime_.counters();
    ROS_INFO("[FAST-LIO2 frontend] tx=%llu disposition=%u committed=%llu updates=%llu prediction_only=%llu processed=%llu init_skip=%llu stale_skip=%llu partial_overlap=%llu overlap_dropped=%llu overlap_us_total=%.3f ndt_success=%llu ndt_reject=%llu p=(%.3f %.3f %.3f)",
             static_cast<unsigned long long>(transaction_id), result.disposition,
             static_cast<unsigned long long>(state.stamp_ns),
             static_cast<unsigned long long>(counters.measurement_updates),
             static_cast<unsigned long long>(counters.prediction_only_commits),
             static_cast<unsigned long long>(processed_scans_),
             static_cast<unsigned long long>(initial_skipped_scans_),
             static_cast<unsigned long long>(stale_skipped_scans_),
             static_cast<unsigned long long>(partial_overlap_scans_),
             static_cast<unsigned long long>(overlap_points_dropped_),
             static_cast<double>(overlap_duration_ns_) / 1e3,
             static_cast<unsigned long long>(ndt_success_count_),
             static_cast<unsigned long long>(ndt_reject_count_),
             state.map_T_imu.position.x(), state.map_T_imu.position.y(),
             state.map_T_imu.position.z());
    return true;
  }

  void pruneImuThrough(uint64_t committed_stamp_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    while (imu_buffer_.size() > 2 &&
           imu_buffer_[1].stamp_ns <= committed_stamp_ns)
      imu_buffer_.pop_front();
  }

  void setFatal(const std::string& reason) {
    if (worker_fatal_) return;
    worker_fatal_ = true;
    ROS_FATAL("[FAST-LIO2 frontend] sticky FATAL: %s", reason.c_str());
  }

  void workerLoop() {
    if (!configuration_valid_) {
      setFatal("invalid LiDAR/queue/runtime parameters");
      return;
    }
    while (ros::ok() && !worker_fatal_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!callback_fatal_.empty()) {
          setFatal(callback_fatal_);
          return;
        }
      }
      if (checkServerRestart()) return;
      if (bound_server_instance_.empty() && !bindSession()) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(250));
        continue;
      }
      if (!session_bound_ && !runtime_.initialized()) {
        if (!initial_pose_valid_ || !extrinsic_valid_) {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait_for(lock, std::chrono::milliseconds(250));
          continue;
        }
        if (!initializeFilter()) {
          if (worker_fatal_) return;
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait_for(lock, std::chrono::milliseconds(250));
          continue;
        }
      }
      if (!session_bound_) continue;
      sensor_msgs::PointCloud2::ConstPtr scan;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (scan_queue_.empty()) {
          condition_.wait_for(lock, std::chrono::milliseconds(250));
          continue;
        }
        scan = scan_queue_.front();
        scan_queue_.pop_front();
      }
      if (!processOneScan(scan)) return;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  RuntimeParameters runtime_parameters_;
  FrontendRuntime runtime_;
  ros::Subscriber imu_sub_, lidar_sub_, result_sub_, status_sub_, control_ack_sub_;
  ros::Publisher request_pub_, control_pub_, ack_pub_, odom_pub_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  ImuBuffer imu_buffer_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> scan_queue_;
  std::deque<Result> result_queue_;
  std::deque<ControlAck> control_ack_queue_;
  Status latest_status_;
  bool have_status_ = false;
  bool stopping_ = false;
  std::string callback_fatal_;
  bool worker_fatal_ = false;
  bool session_bound_ = false;
  bool configuration_valid_ = true;
  bool hashes_valid_ = false;
  bool initial_pose_valid_ = false;
  bool extrinsic_valid_ = false;
  uint32_t protocol_version_ = 1;
  uint64_t expected_cache_entries_ = 15000;
  uint64_t processed_scans_ = 0;
  uint64_t initial_skipped_scans_ = 0;
  uint64_t stale_skipped_scans_ = 0;
  uint64_t partial_overlap_scans_ = 0;
  uint64_t overlap_points_dropped_ = 0;
  uint64_t overlap_duration_ns_ = 0;
  uint64_t ndt_success_count_ = 0;
  uint64_t ndt_reject_count_ = 0;
  int max_imu_samples_ = 20000, max_scans_ = 64, max_results_ = 64;
  double point_time_scale_seconds_ = 1.0;
  double max_scan_duration_seconds_ = 0.25;
  double result_timeout_seconds_ = 5.0;
  std::string imu_topic_, lidar_topic_, request_topic_, result_topic_, status_topic_;
  std::string control_topic_, control_ack_topic_, ack_topic_, odom_topic_;
  std::string map_frame_, lidar_frame_, point_time_field_;
  std::string bound_server_instance_, frontend_session_id_;
  std::array<uint8_t, 32> expected_map_sha_{}, expected_config_sha_{};
  Pose3d initial_map_T_lidar_, T_imu_lidar_;
  std::map<uint64_t, std::vector<uint8_t>> terminal_ledger_;
};

}  // namespace
}  // namespace dog_prior_map_fastlio2_frontend_exp

int main(int argc, char** argv) {
  ros::init(argc, argv, "dog_prior_map_fastlio2_frontend");
  dog_prior_map_fastlio2_frontend_exp::FastLio2FrontendNode node;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
