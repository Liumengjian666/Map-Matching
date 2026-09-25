#include "dog_prior_map_localization/external_ndt_transaction_server.hpp"
#include "dog_prior_map_localization/ndt_request_identity.hpp"

#include <dog_prior_map_interfaces/NdtScanAck.h>
#include <dog_prior_map_interfaces/NdtScanRequest.h>
#include <dog_prior_map_interfaces/NdtScanResult.h>
#include <dog_prior_map_interfaces/NdtServerStatus.h>
#include <dog_prior_map_interfaces/NdtSessionControl.h>
#include <dog_prior_map_interfaces/NdtSessionControlAck.h>
#include <geometry_msgs/PoseStamped.h>
#include <openssl/evp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace dog_prior_map_localization {
namespace {

using Request = dog_prior_map_interfaces::NdtScanRequest;
using Result = dog_prior_map_interfaces::NdtScanResult;
using Control = dog_prior_map_interfaces::NdtSessionControl;
using ControlAck = dog_prior_map_interfaces::NdtSessionControlAck;
using Status = dog_prior_map_interfaces::NdtServerStatus;
using Ack = dog_prior_map_interfaces::NdtScanAck;
using Key = std::tuple<std::string, uint32_t, uint64_t>;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

void finalizeCloud(const Cloud::Ptr& cloud) {
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Eigen::Matrix4d poseMatrix(const geometry_msgs::Pose& pose) {
  const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  return matrix;
}

void fillPose(geometry_msgs::PoseStamped* output, const Eigen::Matrix4d& matrix,
              const std_msgs::Header& header) {
  output->header = header;
  output->header.frame_id = header.frame_id;
  output->pose.position.x = matrix(0, 3);
  output->pose.position.y = matrix(1, 3);
  output->pose.position.z = matrix(2, 3);
  Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  q.normalize();
  output->pose.orientation.x = q.x();
  output->pose.orientation.y = q.y();
  output->pose.orientation.z = q.z();
  output->pose.orientation.w = q.w();
}

bool validPose(const geometry_msgs::Pose& pose) {
  const Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Vector4d q(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w);
  const double norm = q.norm();
  return p.allFinite() && q.allFinite() && std::isfinite(norm) && norm > 1e-12 &&
         std::abs(norm - 1.0) <= 1e-6;
}

std::string makeUuidV4() {
  std::random_device random;
  std::mt19937_64 generator(random());
  std::array<uint8_t, 16> bytes{};
  for (std::size_t i = 0; i < bytes.size(); i += 8) {
    const uint64_t value = generator();
    for (int j = 0; j < 8; ++j) bytes[i + j] = static_cast<uint8_t>(value >> (j * 8));
  }
  bytes[6] = static_cast<uint8_t>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<uint8_t>((bytes[8] & 0x3fU) | 0x80U);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) out << '-';
    out << std::setw(2) << static_cast<unsigned>(bytes[i]);
  }
  return out.str();
}

bool isUuidV4(const std::string& value) {
  if (value.size() != 36 || value[8] != '-' || value[13] != '-' ||
      value[18] != '-' || value[23] != '-' || value[14] != '4') return false;
  const char variant = static_cast<char>(std::tolower(value[19]));
  if (variant != '8' && variant != '9' && variant != 'a' && variant != 'b') return false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 8 || i == 13 || i == 18 || i == 23) continue;
    if (!std::isxdigit(static_cast<unsigned char>(value[i]))) return false;
  }
  return true;
}

bool sha256File(const std::string& path, std::array<uint8_t, 32>* output) {
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input || !output) return false;
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (!context) return false;
  bool ok = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1;
  std::array<char, 1 << 16> buffer{};
  while (ok && input.good()) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize bytes = input.gcount();
    if (bytes > 0) ok = EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(bytes)) == 1;
  }
  unsigned int length = 0;
  ok = ok && EVP_DigestFinal_ex(context, output->data(), &length) == 1 && length == output->size();
  EVP_MD_CTX_free(context);
  return ok;
}

bool sha256Text(const std::string& text, std::array<uint8_t, 32>* output) {
  unsigned int length = 0;
  return output && EVP_Digest(text.data(), text.size(), output->data(), &length,
                              EVP_sha256(), nullptr) == 1 && length == output->size();
}

uint64_t cloudHash(const sensor_msgs::PointCloud2& cloud) {
  constexpr uint64_t kOffset = 14695981039346656037ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  auto byte = [&hash](uint8_t value) { hash = (hash ^ value) * kPrime; };
  auto u32 = [&byte](uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8) byte(static_cast<uint8_t>(value >> shift));
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

std::string canonicalCloudConfig(double map_voxel, double map_z_voxel,
                                 double source_voxel, double source_z_voxel,
                                 double target_voxel, double target_z_voxel,
                                 int max_source, int max_target, double min_range,
                                 double max_range, double min_z, double max_z,
                                 int min_points, double resolution, double step,
                                 double epsilon, int iterations, bool limiter,
                                 double max_translation, double max_rotation) {
  std::ostringstream out;
  out << std::setprecision(17)
      << "map_voxel=" << map_voxel << '\n' << "map_z_voxel=" << map_z_voxel << '\n'
      << "source_voxel=" << source_voxel << '\n' << "source_z_voxel=" << source_z_voxel << '\n'
      << "target_voxel=" << target_voxel << '\n' << "target_z_voxel=" << target_z_voxel << '\n'
      << "max_source=" << max_source << '\n' << "max_target=" << max_target << '\n'
      << "min_range=" << min_range << '\n' << "max_range=" << max_range << '\n'
      << "min_z=" << min_z << '\n' << "max_z=" << max_z << '\n'
      << "min_points=" << min_points << '\n' << "resolution=" << resolution << '\n'
      << "step=" << step << '\n' << "epsilon=" << epsilon << '\n'
      << "iterations=" << iterations << '\n' << "limiter=" << (limiter ? 1 : 0) << '\n'
      << "max_translation=" << max_translation << '\n' << "max_rotation_deg=" << max_rotation << '\n';
  return out.str();
}

}  // namespace

struct ExternalNdtTransactionServer::Impl {
  struct Work {
    enum Kind { REQUEST, CONTROL, ACK } kind = REQUEST;
    Request request;
    Control control;
    Ack ack;
  };
  struct Cached {
    detail::NdtRequestIdentity request_identity;
    Result result;
  };

  ros::NodeHandle nh;
  ros::NodeHandle pnh{"~"};
  ros::Subscriber request_sub;
  ros::Subscriber control_sub;
  ros::Subscriber ack_sub;
  ros::Publisher result_pub;
  ros::Publisher status_pub;
  ros::Publisher control_ack_pub;
  std::mutex mutex;
  std::condition_variable condition;
  std::deque<Work> queue;
  std::thread worker;
  bool stopping = false;
  bool queue_overflow_pending = false;
  bool fatal = false;
  uint8_t fatal_reason = Status::FATAL_NONE;
  Status status;
  std::map<Key, Cached> terminal_cache;
  std::map<Key, Ack> acknowledgements;
  std::string active_session;
  uint32_t active_epoch = 0;
  bool session_bound = false;
  bool has_previous_pose = false;
  Eigen::Matrix4d previous_pose = Eigen::Matrix4d::Identity();
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  Cloud::Ptr target{new Cloud};

  std::string map_path, map_frame, lidar_frame;
  std::string request_topic, result_topic, control_topic, control_ack_topic, status_topic, ack_topic;
  double map_voxel = 0.30, map_z_voxel = 0.30;
  double source_voxel = 0.25, source_z_voxel = 0.15;
  double target_voxel = 0.15, target_z_voxel = 0.15;
  int max_source = 1400, max_target = 0, min_points = 50;
  double min_range = 0.5, max_range = 80.0;
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
  double resolution = 0.8, step = 0.08, epsilon = 0.001;
  int max_iterations = 40;
  bool limiter = false;
  double max_translation = 0.5, max_rotation_deg = 5.0;
  uint32_t queue_capacity = 8;

  template <class T>
  T param(const std::string& name, const T& fallback) {
    T value;
    if (nh.getParam(name, value) || pnh.getParam(name, value)) return value;
    return fallback;
  }

  Impl() {
    map_path = param<std::string>("map/pcd_fallback_path", "");
    map_frame = param<std::string>("frames/map_frame", "camera_init");
    lidar_frame = param<std::string>("frames/base_frame", "livox_frame");
    map_voxel = param<double>("map/voxel_size", 0.30);
    map_z_voxel = param<double>("map/voxel_z_size", map_voxel);
    source_voxel = param<double>("lidar_update/ndt_source_voxel_size", 0.25);
    source_z_voxel = param<double>("lidar_update/ndt_source_voxel_z_size", source_voxel);
    target_voxel = param<double>("lidar_update/ndt_target_voxel_size", 0.15);
    target_z_voxel = param<double>("lidar_update/ndt_target_voxel_z_size", target_voxel);
    max_source = param<int>("lidar_update/ndt_max_source_points", 1400);
    max_target = param<int>("lidar_update/ndt_max_target_points", 0);
    min_points = param<int>("lidar_update/min_effective_points", 50);
    min_range = param<double>("lidar_update/scan_min_range", 0.5);
    max_range = param<double>("lidar_update/scan_max_range", 80.0);
    min_z = param<double>("lidar_update/scan_min_z", min_z);
    max_z = param<double>("lidar_update/scan_max_z", max_z);
    resolution = param<double>("lidar_update/ndt_resolution", 0.8);
    step = param<double>("lidar_update/ndt_step_size", 0.08);
    epsilon = param<double>("lidar_update/ndt_transformation_epsilon", 0.001);
    max_iterations = param<int>("lidar_update/ndt_max_iterations", 40);
    limiter = param<bool>("lidar_update/ndt_step_limit_enable", false);
    max_translation = param<double>("lidar_update/ndt_step_limit_max_translation", 0.5);
    max_rotation_deg = param<double>("lidar_update/ndt_step_limit_max_rotation_deg", 5.0);
    queue_capacity = static_cast<uint32_t>(std::max(1, param<int>("external_transaction/queue_capacity", 8)));
    status.terminal_cache_max_entries = static_cast<uint64_t>(std::max(1,
        param<int>("external_transaction/terminal_cache_max_entries", 15000)));
    request_topic = param<std::string>("external_transaction/request_topic", "/dog_livo/ndt/scan_request");
    result_topic = param<std::string>("external_transaction/result_topic", "/dog_livo/ndt/scan_result");
    control_topic = param<std::string>("external_transaction/control_topic", "/dog_livo/ndt/session_control");
    control_ack_topic = param<std::string>("external_transaction/control_ack_topic", "/dog_livo/ndt/session_control_ack");
    status_topic = param<std::string>("external_transaction/status_topic", "/dog_livo/ndt/server_status");
    ack_topic = param<std::string>("external_transaction/ack_topic", "/dog_livo/ndt/scan_ack");

    if (map_path.empty()) throw std::runtime_error("external NDT map path is empty");
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile(map_path, *raw) != 0)
      throw std::runtime_error("external NDT failed to load map: " + map_path);
    Cloud::Ptr finite(new Cloud);
    finite->reserve(raw->size());
    for (const auto& point : raw->points)
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite->push_back(point);
    finalizeCloud(finite);
    Cloud::Ptr map = voxelDown(finite, map_voxel, map_z_voxel, 0);
    target = voxelDown(map, target_voxel, target_z_voxel, max_target);
    if (target->empty()) throw std::runtime_error("external NDT target map is empty");
    ndt.setInputTarget(target);
    ndt.setResolution(resolution);
    ndt.setStepSize(step);
    ndt.setTransformationEpsilon(epsilon);
    ndt.setMaximumIterations(max_iterations);

    std::array<uint8_t, 32> map_hash{}, config_hash{};
    if (!sha256File(map_path, &map_hash)) throw std::runtime_error("cannot SHA256 map file");
    const std::string canonical_config = canonicalCloudConfig(
        map_voxel, map_z_voxel, source_voxel, source_z_voxel, target_voxel,
        target_z_voxel, max_source, max_target, min_range, max_range, min_z,
        max_z, min_points, resolution, step, epsilon, max_iterations,
        limiter, max_translation, max_rotation_deg);
    if (!sha256Text(canonical_config, &config_hash)) throw std::runtime_error("cannot SHA256 NDT config");
    status.protocol_version = Status::PROTOCOL_VERSION;
    status.server_ready = true;
    status.fatal_latched = false;
    status.fatal_reason = Status::FATAL_NONE;
    status.external_mode_enabled = true;
    std::copy(map_hash.begin(), map_hash.end(), status.map_sha256.begin());
    std::copy(config_hash.begin(), config_hash.end(), status.canonical_ndt_config_sha256.begin());
    status.map_frame = map_frame;
    status.lidar_frame = lidar_frame;
    status.server_instance_id = makeUuidV4();
    status.bound_frontend_session_id.clear();
    status.epoch = 0;

    result_pub = nh.advertise<Result>(result_topic, 100, false);
    status_pub = nh.advertise<Status>(status_topic, 1, true);
    control_ack_pub = nh.advertise<ControlAck>(control_ack_topic, 10, false);
    request_sub = nh.subscribe(request_topic, queue_capacity,
                               &Impl::requestCallback, this);
    control_sub = nh.subscribe(control_topic, 10, &Impl::controlCallback, this);
    ack_sub = nh.subscribe(ack_topic, 100, &Impl::ackCallback, this);
    publishStatus();
    worker = std::thread(&Impl::workerLoop, this);
    ROS_INFO("[DogPriorMap NDT] external transaction worker ready: map=%zu frames=%s->%s cache=%llu",
             target->size(), map_frame.c_str(), lidar_frame.c_str(),
             static_cast<unsigned long long>(status.terminal_cache_max_entries));
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
    }
    condition.notify_all();
    if (worker.joinable()) worker.join();
  }

  Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf, double zleaf, int cap) {
    Cloud::Ptr down(new Cloud);
    if (leaf > 0.01) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(zleaf));
      voxel.setInputCloud(input);
      voxel.filter(*down);
    } else {
      *down = *input;
    }
    if (cap > 0 && static_cast<int>(down->size()) > cap) {
      Cloud::Ptr sampled(new Cloud);
      sampled->reserve(static_cast<std::size_t>(cap));
      const double increment = static_cast<double>(down->size() - 1) /
                              static_cast<double>(std::max(cap - 1, 1));
      for (int i = 0; i < cap; ++i)
        sampled->push_back(down->points[static_cast<std::size_t>(std::llround(i * increment))]);
      finalizeCloud(sampled);
      return sampled;
    }
    finalizeCloud(down);
    return down;
  }

  Cloud::Ptr preprocess(const Cloud::Ptr& input) {
    Cloud::Ptr filtered(new Cloud);
    filtered->reserve(input->size());
    for (const auto& point : input->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (range < min_range || range > max_range || point.z < min_z || point.z > max_z) continue;
      filtered->push_back(point);
    }
    finalizeCloud(filtered);
    return voxelDown(filtered, source_voxel, source_z_voxel, max_source);
  }

  Eigen::Matrix4d limitPose(const Eigen::Matrix4d& raw, bool* translation_limited,
                            bool* rotation_limited) const {
    *translation_limited = false;
    *rotation_limited = false;
    Eigen::Matrix4d used = raw;
    if (!limiter || !has_previous_pose) return used;
    const Eigen::Vector3d previous_p = previous_pose.block<3, 1>(0, 3);
    const Eigen::Vector3d dp = raw.block<3, 1>(0, 3) - previous_p;
    if (max_translation > 0.0 && dp.norm() > max_translation) {
      used.block<3, 1>(0, 3) = previous_p + dp.normalized() * max_translation;
      *translation_limited = true;
    }
    const Eigen::Matrix3d previous_r = previous_pose.block<3, 3>(0, 0);
    const Eigen::Matrix3d raw_r = raw.block<3, 3>(0, 0);
    const Eigen::AngleAxisd diff(previous_r.transpose() * raw_r);
    const double angle_deg = std::abs(diff.angle()) * 180.0 / M_PI;
    if (max_rotation_deg > 0.0 && angle_deg > max_rotation_deg) {
      const double ratio = max_rotation_deg / std::max(angle_deg, 1e-6);
      used.block<3, 3>(0, 0) = Eigen::Quaterniond(previous_r).normalized().slerp(
          ratio, Eigen::Quaterniond(raw_r).normalized()).toRotationMatrix();
      *rotation_limited = true;
    }
    return used;
  }

  Result baseResult(const Request& request, uint8_t disposition,
                    const std::string& reason) const {
    Result result;
    result.header.stamp = request.scan_end;
    result.header.frame_id = map_frame;
    result.protocol_version = Status::PROTOCOL_VERSION;
    result.frontend_session_id = request.frontend_session_id;
    result.epoch = request.epoch;
    result.transaction_id = request.transaction_id;
    result.server_instance_id = status.server_instance_id;
    result.scan_start = request.scan_start;
    result.scan_end = request.scan_end;
    result.scan_start_ns = request.scan_start_ns;
    result.scan_end_ns = request.scan_end_ns;
    result.disposition = disposition;
    result.reason = reason;
    result.pose_valid = false;
    result.map_frame = map_frame;
    result.lidar_frame = lidar_frame;
    result.raw_map_T_lidar.header.stamp = request.scan_end;
    result.raw_map_T_lidar.header.frame_id = map_frame;
    result.used_map_T_lidar = result.raw_map_T_lidar;
    result.fitness = std::numeric_limits<double>::quiet_NaN();
    result.converged = false;
    result.request_cloud_hash = request.request_cloud_hash;
    result.ndt_source_cloud_hash = 0;
    result.translation_limited = false;
    result.rotation_limited = false;
    result.step_limited = false;
    return result;
  }

  void requestCallback(const Request::ConstPtr& message) {
    if (!message) return;
    Work work;
    work.kind = Work::REQUEST;
    work.request = *message;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping) return;
      if (queue.size() >= queue_capacity) {
        queue_overflow_pending = true;
        condition.notify_one();
        return;
      }
      queue.push_back(work);
    }
    condition.notify_one();
  }

  void controlCallback(const Control::ConstPtr& message) {
    if (!message) return;
    Work work;
    work.kind = Work::CONTROL;
    work.control = *message;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping) return;
      if (queue.size() >= queue_capacity) {
        queue_overflow_pending = true;
        condition.notify_one();
        return;
      }
      queue.push_back(work);
    }
    condition.notify_one();
  }

  void ackCallback(const Ack::ConstPtr& message) {
    if (!message) return;
    Work work;
    work.kind = Work::ACK;
    work.ack = *message;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping) return;
      if (queue.size() >= queue_capacity) {
        queue_overflow_pending = true;
        condition.notify_one();
        return;
      }
      queue.push_back(work);
    }
    condition.notify_one();
  }

  void workerLoop() {
    while (true) {
      Work work;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this] { return stopping || queue_overflow_pending || !queue.empty(); });
        if (stopping && queue.empty() && !queue_overflow_pending) return;
        if (queue_overflow_pending) {
          queue_overflow_pending = false;
          lock.unlock();
          latchFatal(Status::FATAL_QUEUE_OVERFLOW);
          continue;
        }
        work = std::move(queue.front());
        queue.pop_front();
      }
      if (work.kind == Work::CONTROL) processControl(work.control);
      else if (work.kind == Work::REQUEST) processRequest(work.request);
      else processAck(work.ack);
    }
  }

  void publishStatus() {
    status.header.stamp = ros::Time::now();
    status.header.frame_id = map_frame;
    status_pub.publish(status);
  }

  void latchFatal(uint8_t reason) {
    if (fatal) return;
    fatal = true;
    fatal_reason = reason;
    status.fatal_latched = true;
    status.fatal_reason = reason;
    status.server_ready = false;
    publishStatus();
    ROS_ERROR("[DogPriorMap NDT] external transaction fatal latch=%u", reason);
  }

  bool acceptedControl(const Control& control, std::string* reason) {
    if (fatal) { *reason = "fatal_latched"; return false; }
    if (control.protocol_version != Status::PROTOCOL_VERSION) { *reason = "protocol_version_mismatch"; return false; }
    if (control.server_instance_id != status.server_instance_id) { *reason = "server_instance_mismatch"; return false; }
    if (!isUuidV4(control.frontend_session_id)) { *reason = "invalid_frontend_session_uuid"; return false; }
    if (control.expected_map_sha256 != status.map_sha256) { *reason = "map_sha256_mismatch"; return false; }
    if (control.expected_ndt_config_sha256 != status.canonical_ndt_config_sha256) {
      *reason = "ndt_config_sha256_mismatch"; return false;
    }
    if (control.command == Control::BEGIN_SESSION) {
      if (control.epoch != 0 || control.reason != Control::REASON_FRONTEND_START) {
        *reason = "invalid_begin_session_transition"; return false;
      }
      active_session = control.frontend_session_id;
      active_epoch = 0;
      session_bound = true;
      terminal_cache.clear();
      acknowledgements.clear();
      has_previous_pose = false;
    } else if (control.command == Control::BEGIN_EPOCH) {
      if (!session_bound || control.frontend_session_id != active_session ||
          control.epoch != active_epoch + 1 ||
          (control.reason != Control::REASON_EXPLICIT_FILTER_RESET &&
           control.reason != Control::REASON_SENSOR_TIME_REWIND &&
           control.reason != Control::REASON_DATASET_RESTART)) {
        *reason = "invalid_begin_epoch_transition"; return false;
      }
      active_epoch = control.epoch;
      terminal_cache.clear();
      acknowledgements.clear();
      has_previous_pose = false;
    } else if (control.command == Control::END_SESSION) {
      if (!session_bound || control.frontend_session_id != active_session ||
          control.epoch != active_epoch || control.reason != Control::REASON_FRONTEND_END) {
        *reason = "invalid_end_session_transition"; return false;
      }
      session_bound = false;
      active_session.clear();
      terminal_cache.clear();
      acknowledgements.clear();
      has_previous_pose = false;
    } else {
      *reason = "unknown_control_command"; return false;
    }
    status.bound_frontend_session_id = active_session;
    status.epoch = active_epoch;
    *reason = "accepted";
    return true;
  }

  void processControl(const Control& control) {
    ControlAck ack;
    ack.header.stamp = ros::Time::now();
    ack.header.frame_id = map_frame;
    ack.command = control.command;
    ack.protocol_version = Status::PROTOCOL_VERSION;
    ack.server_instance_id = status.server_instance_id;
    ack.frontend_session_id = control.frontend_session_id;
    ack.epoch = control.epoch;
    bool accepted = false;
    accepted = acceptedControl(control, &ack.reason);
    ack.result = accepted ? ControlAck::ACCEPTED : ControlAck::REJECT_INVALID_TRANSITION;
    if (fatal) ack.result = ControlAck::REJECT_FATAL_LATCHED;
    else if (control.protocol_version != Status::PROTOCOL_VERSION) ack.result = ControlAck::REJECT_PROTOCOL_VERSION;
    else if (control.server_instance_id != status.server_instance_id) ack.result = ControlAck::REJECT_SERVER_INSTANCE;
    else if (control.expected_map_sha256 != status.map_sha256) ack.result = ControlAck::REJECT_MAP_IDENTITY;
    else if (control.expected_ndt_config_sha256 != status.canonical_ndt_config_sha256) ack.result = ControlAck::REJECT_CONFIG_IDENTITY;
    control_ack_pub.publish(ack);
    publishStatus();
  }

  bool validTimestampEnvelope(const Request& request) const {
    return request.scan_start_ns > 0 && request.scan_end_ns > request.scan_start_ns &&
        request.scan_start.toNSec() == request.scan_start_ns &&
        request.scan_end.toNSec() == request.scan_end_ns &&
        request.header.stamp.toNSec() == request.scan_end_ns &&
        request.cloud_end_frame.header.stamp.toNSec() == request.scan_end_ns &&
        request.predicted_map_T_lidar.header.stamp.toNSec() == request.scan_end_ns;
  }

  void processRequest(const Request& request) {
    Result result = baseResult(request, Result::ERROR_PROTOCOL, "invalid_request_envelope");
    const Key key(request.frontend_session_id, request.epoch, request.transaction_id);
    if (request.transaction_id == 0 || !isUuidV4(request.frontend_session_id) ||
        !validTimestampEnvelope(request) || request.protocol_version != Status::PROTOCOL_VERSION ||
        request.server_instance_id != status.server_instance_id ||
        request.map_frame != map_frame || request.lidar_frame != lidar_frame ||
        request.header.frame_id != map_frame ||
        request.cloud_end_frame.header.frame_id != lidar_frame ||
        request.predicted_map_T_lidar.header.frame_id != map_frame ||
        !validPose(request.predicted_map_T_lidar.pose)) {
      result.reason = "invalid_request_envelope_or_pose";
      result_pub.publish(result);
      return;
    }
    if (!session_bound || request.frontend_session_id != active_session || request.epoch != active_epoch) {
      result.reason = "inactive_frontend_session_or_epoch";
      result_pub.publish(result);
      return;
    }
    const uint64_t actual_cloud_hash = cloudHash(request.cloud_end_frame);
    const detail::NdtRequestIdentity request_identity =
        detail::makeNdtRequestIdentity(request);
    const auto existing = terminal_cache.find(key);
    if (existing != terminal_cache.end()) {
      if (actual_cloud_hash == request.request_cloud_hash &&
          detail::exactNdtRequestIdentity(existing->second.request_identity,
                                          request_identity)) {
        result_pub.publish(existing->second.result);
      } else {
        result.reason = "conflicting_duplicate_request_identity";
        result_pub.publish(result);
      }
      return;
    }
    if (fatal) return;
    if (terminal_cache.size() >= status.terminal_cache_max_entries) {
      result = baseResult(request, Result::ERROR_TERMINAL_CACHE_EXHAUSTED,
                          "terminal_cache_capacity_exhausted");
      result_pub.publish(result);
      latchFatal(Status::FATAL_CACHE_EXHAUSTED);
      return;
    }
    if (actual_cloud_hash != request.request_cloud_hash) {
      result.reason = "request_cloud_hash_mismatch";
      result_pub.publish(result);
      return;
    }

    Cloud::Ptr raw(new Cloud);
    pcl::fromROSMsg(request.cloud_end_frame, *raw);
    bool valid = !raw->empty();
    for (const auto& point : raw->points)
      valid = valid && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    if (!valid) {
      result = baseResult(request, Result::REJECT_INVALID_SOURCE, "invalid_or_nonfinite_source_cloud");
    } else {
      Cloud::Ptr source = preprocess(raw);
      result.ndt_source_cloud_hash = pclCloudHash(source);
      if (static_cast<int>(source->size()) < min_points) {
        result = baseResult(request, Result::REJECT_INSUFFICIENT_POINTS,
                            "source_below_min_effective_points");
        result.ndt_source_cloud_hash = pclCloudHash(source);
      } else {
        ndt.setInputSource(source);
        Cloud aligned;
        const Eigen::Matrix4d guess = poseMatrix(request.predicted_map_T_lidar.pose);
        const ros::WallTime start = ros::WallTime::now();
        ndt.align(aligned, guess.cast<float>());
        const double elapsed_ms = (ros::WallTime::now() - start).toSec() * 1000.0;
        result.fitness = ndt.getFitnessScore();
        result.iterations = static_cast<uint32_t>(std::max(0, ndt.getFinalNumIteration()));
        result.converged = ndt.hasConverged();
        if (!result.converged) {
          result = baseResult(request, Result::REJECT_NOT_CONVERGED, "ndt_did_not_converge");
          result.fitness = ndt.getFitnessScore();
          result.iterations = static_cast<uint32_t>(std::max(0, ndt.getFinalNumIteration()));
          result.ndt_source_cloud_hash = pclCloudHash(source);
          result.converged = false;
        } else {
          const Eigen::Matrix4d raw_pose = ndt.getFinalTransformation().cast<double>();
          bool translation_limited = false;
          bool rotation_limited = false;
          const Eigen::Matrix4d used_pose = limitPose(raw_pose, &translation_limited, &rotation_limited);
          fillPose(&result.raw_map_T_lidar, raw_pose, result.header);
          fillPose(&result.used_map_T_lidar, used_pose, result.header);
          result.disposition = Result::SUCCESS;
          result.reason = "ndt_converged";
          result.pose_valid = true;
          result.translation_limited = translation_limited;
          result.rotation_limited = rotation_limited;
          result.step_limited = translation_limited || rotation_limited;
          result.ndt_source_cloud_hash = pclCloudHash(source);
          previous_pose = used_pose;
          has_previous_pose = true;
          ROS_DEBUG("[DogPriorMap NDT] external transaction=%llu align=%.2fms fit=%.5f iter=%u",
                    static_cast<unsigned long long>(request.transaction_id), elapsed_ms,
                    result.fitness, result.iterations);
        }
      }
    }
    terminal_cache.emplace(key, Cached{request_identity, result});
    result_pub.publish(result);
  }

  uint64_t pclCloudHash(const Cloud::Ptr& cloud) const {
    constexpr uint64_t kOffset = 1469598103934665603ULL;
    constexpr uint64_t kPrime = 1099511628211ULL;
    uint64_t hash = kOffset;
    auto mix_byte = [&hash](uint8_t value) { hash = (hash ^ value) * kPrime; };
    auto mix_u32 = [&mix_byte](uint32_t value) {
      for (int shift = 0; shift < 32; shift += 8) mix_byte(static_cast<uint8_t>(value >> shift));
    };
    if (!cloud) return hash;
    mix_u32(cloud->width); mix_u32(cloud->height); mix_u32(cloud->is_dense ? 1U : 0U);
    mix_u32(static_cast<uint32_t>(cloud->size()));
    for (const auto& point : cloud->points) {
      uint32_t bits;
      std::memcpy(&bits, &point.x, sizeof(bits)); mix_u32(bits);
      std::memcpy(&bits, &point.y, sizeof(bits)); mix_u32(bits);
      std::memcpy(&bits, &point.z, sizeof(bits)); mix_u32(bits);
    }
    return hash;
  }

  void processAck(const Ack& ack) {
    const Key key(ack.frontend_session_id, ack.epoch, ack.transaction_id);
    const auto found = terminal_cache.find(key);
    if (found == terminal_cache.end()) return;
    const Result& result = found->second.result;
    if (ack.server_instance_id != status.server_instance_id ||
        ack.scan_start_ns != result.scan_start_ns || ack.scan_end_ns != result.scan_end_ns ||
        ack.request_cloud_hash != result.request_cloud_hash) {
      ROS_WARN("[DogPriorMap NDT] external ACK identity mismatch for transaction %llu",
               static_cast<unsigned long long>(ack.transaction_id));
      return;
    }
    acknowledgements[key] = ack;
  }
};

ExternalNdtTransactionServer::ExternalNdtTransactionServer() : impl_(new Impl) {}
ExternalNdtTransactionServer::~ExternalNdtTransactionServer() = default;

}  // namespace dog_prior_map_localization
