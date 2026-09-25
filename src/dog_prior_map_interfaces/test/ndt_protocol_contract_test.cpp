#include <dog_prior_map_interfaces/NdtScanAck.h>
#include <dog_prior_map_interfaces/NdtScanRequest.h>
#include <dog_prior_map_interfaces/NdtScanResult.h>
#include <dog_prior_map_interfaces/NdtServerStatus.h>
#include <dog_prior_map_interfaces/NdtSessionControl.h>
#include <dog_prior_map_interfaces/NdtSessionControlAck.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const std::string& name) {
  if (!condition) {
    std::cerr << "FAIL: " << name << '\n';
    ++failures;
  }
}

void reportTest(const std::string& test_id, bool condition,
                const std::string& description) {
  expect(condition, description);
  if (condition) std::cout << test_id << "_PASS\n";
}

bool isUuidV4(const std::string& value) {
  if (value.size() != 36 || value[8] != '-' || value[13] != '-' ||
      value[18] != '-' || value[23] != '-') {
    return false;
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 8 || i == 13 || i == 18 || i == 23) continue;
    if (!std::isxdigit(static_cast<unsigned char>(value[i])) ||
        std::isupper(static_cast<unsigned char>(value[i]))) {
      return false;
    }
  }
  return value[14] == '4' &&
         (value[19] == '8' || value[19] == '9' || value[19] == 'a' ||
          value[19] == 'b');
}

const std::string kServerA = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa";
const std::string kServerB = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb";
const std::string kFrontendA = "11111111-1111-4111-8111-111111111111";
const std::string kFrontendB = "22222222-2222-4222-8222-222222222222";

struct StatusExpectation {
  uint32_t protocol_version = 1;
  std::array<uint8_t, 32> map_hash{};
  std::array<uint8_t, 32> config_hash{};
  uint64_t cache_capacity = 1001;
  uint64_t maximum_expected_scans = 1000;
  std::string frontend_session_id = kFrontendA;
  std::string map_frame = "map";
  std::string lidar_frame = "lidar";
};

bool statusMatches(const dog_prior_map_interfaces::NdtServerStatus& status,
                   const StatusExpectation& expected,
                   std::string* reason) {
  if (status.fatal_latched ||
      status.fatal_reason != dog_prior_map_interfaces::NdtServerStatus::FATAL_NONE)
    *reason = "server_fatal_latched";
  else if (!status.server_ready) *reason = "server_not_ready";
  else if (!status.external_mode_enabled) *reason = "external_mode_disabled";
  else if (status.protocol_version != expected.protocol_version)
    *reason = "protocol_version_mismatch";
  else if (!std::equal(status.map_sha256.begin(), status.map_sha256.end(),
                       expected.map_hash.begin()))
    *reason = "map_hash_mismatch";
  else if (!std::equal(status.canonical_ndt_config_sha256.begin(),
                       status.canonical_ndt_config_sha256.end(),
                       expected.config_hash.begin()))
    *reason = "ndt_config_hash_mismatch";
  else if (status.map_frame != expected.map_frame)
    *reason = "map_frame_mismatch";
  else if (status.lidar_frame != expected.lidar_frame)
    *reason = "lidar_frame_mismatch";
  else if (!isUuidV4(status.server_instance_id))
    *reason = "invalid_server_instance_id";
  else if (status.terminal_cache_max_entries != expected.cache_capacity ||
           status.terminal_cache_max_entries <= expected.maximum_expected_scans)
    *reason = "terminal_cache_capacity_invalid";
  else {
    reason->clear();
    return true;
  }
  return false;
}

bool statusBindsSession(const dog_prior_map_interfaces::NdtServerStatus& status,
                        const std::string& server_id,
                        const std::string& frontend_id, uint32_t epoch) {
  return status.server_ready && !status.fatal_latched &&
         status.fatal_reason == dog_prior_map_interfaces::NdtServerStatus::FATAL_NONE &&
         status.server_instance_id == server_id &&
         status.bound_frontend_session_id == frontend_id && status.epoch == epoch;
}

enum class InstanceDecision { kInitialBind, kSameInstance, kFatalRestart, kWait };

class InstanceGuard {
 public:
  InstanceDecision observe(const dog_prior_map_interfaces::NdtServerStatus& status,
                           bool valid_handshake) {
    if (!valid_handshake) return InstanceDecision::kWait;
    if (!bound_) {
      server_id_ = status.server_instance_id;
      bound_ = true;
      return InstanceDecision::kInitialBind;
    }
    if (status.server_instance_id == server_id_) return InstanceDecision::kSameInstance;
    return InstanceDecision::kFatalRestart;
  }

 private:
  std::string server_id_;
  bool bound_ = false;
};

struct Key {
  std::string frontend_session_id;
  uint32_t epoch = 0;
  uint64_t transaction_id = 0;

  bool operator<(const Key& other) const {
    return std::tie(frontend_session_id, epoch, transaction_id) <
           std::tie(other.frontend_session_id, other.epoch, other.transaction_id);
  }
  bool operator==(const Key& other) const {
    return frontend_session_id == other.frontend_session_id &&
           epoch == other.epoch && transaction_id == other.transaction_id;
  }
};

void setTimeNs(ros::Time* stamp, uint64_t nanoseconds);
bool exactRequestIdentity(const dog_prior_map_interfaces::NdtScanRequest& lhs,
                          const dog_prior_map_interfaces::NdtScanRequest& rhs);

struct FrontendIdentity {
  std::string session = kFrontendA;
  uint32_t epoch = 0;
  uint64_t next_transaction = 1;

  Key allocate() { return Key{session, epoch, next_transaction++}; }
  bool beginNextEpoch() {
    if (epoch == std::numeric_limits<uint32_t>::max()) return false;
    ++epoch;
    next_transaction = 1;
    return true;
  }
};

dog_prior_map_interfaces::NdtScanRequest makeRequest(const Key& key) {
  dog_prior_map_interfaces::NdtScanRequest request;
  request.protocol_version = 1;
  request.frontend_session_id = key.frontend_session_id;
  request.epoch = key.epoch;
  request.transaction_id = key.transaction_id;
  request.server_instance_id = kServerA;
  setTimeNs(&request.scan_start, 10);
  setTimeNs(&request.scan_end, 20);
  request.scan_start_ns = 10;
  request.scan_end_ns = 20;
  request.header.stamp = request.scan_end;
  request.header.frame_id = "map";
  request.map_frame = "map";
  request.lidar_frame = "lidar";
  request.cloud_end_frame.header.stamp = request.scan_end;
  request.cloud_end_frame.header.frame_id = "lidar";
  request.cloud_end_frame.height = 1;
  request.cloud_end_frame.width = 1;
  request.cloud_end_frame.point_step = 4;
  request.cloud_end_frame.row_step = 4;
  request.cloud_end_frame.data = {1, 2, 3, 4};
  request.cloud_end_frame.is_dense = true;
  request.predicted_map_T_lidar.header.stamp = request.scan_end;
  request.predicted_map_T_lidar.header.frame_id = "map";
  request.predicted_map_T_lidar.pose.orientation.w = 1.0;
  request.request_cloud_hash = 30;
  return request;
}

struct TerminalResult {
  uint8_t disposition = dog_prior_map_interfaces::NdtScanResult::SUCCESS;
  uint64_t deterministic_token = 0;

  bool operator==(const TerminalResult& other) const {
    return disposition == other.disposition &&
           deterministic_token == other.deterministic_token;
  }
};

enum class CacheAction { kProcessed, kCached, kConflict, kExhausted };

class TerminalCacheModel {
 public:
  explicit TerminalCacheModel(std::size_t max_entries) : max_entries_(max_entries) {}

  CacheAction submit(const Key& key,
                    const dog_prior_map_interfaces::NdtScanRequest& request,
                    TerminalResult* result) {
    auto found = entries_.find(key);
    if (found != entries_.end()) {
      if (!exactRequestIdentity(found->second.request, request))
        return CacheAction::kConflict;
      *result = found->second.result;
      return CacheAction::kCached;
    }
    if (exhausted_ || entries_.size() >= max_entries_) {
      exhausted_ = true;
      return CacheAction::kExhausted;
    }
    ++ndt_process_count_;
    TerminalResult generated;
    generated.deterministic_token = ndt_process_count_;
    entries_.emplace(key, Entry{request, generated, false});
    *result = generated;
    return CacheAction::kProcessed;
  }

  bool acknowledge(const dog_prior_map_interfaces::NdtScanAck& ack) {
    const Key key{ack.frontend_session_id, ack.epoch, ack.transaction_id};
    auto found = entries_.find(key);
    if (found == entries_.end() ||
        ack.server_instance_id != found->second.request.server_instance_id ||
        ack.scan_start_ns != found->second.request.scan_start_ns ||
        ack.scan_end_ns != found->second.request.scan_end_ns ||
        ack.request_cloud_hash != found->second.request.request_cloud_hash) {
      return false;
    }
    found->second.acked = true;
    return true;
  }

  void resetEpoch(const std::string& frontend_id, uint32_t epoch) {
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (it->first.frontend_session_id == frontend_id && it->first.epoch == epoch)
        it = entries_.erase(it);
      else
        ++it;
    }
    exhausted_ = false;
  }

  void resetSession(const std::string& frontend_id) {
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (it->first.frontend_session_id == frontend_id)
        it = entries_.erase(it);
      else
        ++it;
    }
    exhausted_ = false;
  }

  void clear() {
    entries_.clear();
    exhausted_ = false;
  }

  std::size_t size() const { return entries_.size(); }
  uint64_t ndtProcessCount() const { return ndt_process_count_; }

 private:
  struct Entry {
    dog_prior_map_interfaces::NdtScanRequest request;
    TerminalResult result;
    bool acked;
  };
  std::size_t max_entries_;
  bool exhausted_ = false;
  uint64_t ndt_process_count_ = 0;
  std::map<Key, Entry> entries_;
};

enum class ResultDecision {
  kFatalMalformedResult,
  kAcceptCurrent,
  kDropForeignSession,
  kDropStaleEpoch,
  kFatalFutureEpoch,
  kDropOldTransaction,
  kFatalFutureTransaction,
  kFatalServerInstanceChanged,
  kFatalCurrentPayloadCorruption,
  kDropDuplicateResult,
  kFatalNondeterministicResult
};

void setResultKey(const Key& key,
                  dog_prior_map_interfaces::NdtScanResult* result) {
  result->frontend_session_id = key.frontend_session_id;
  result->epoch = key.epoch;
  result->transaction_id = key.transaction_id;
}

Key resultKey(const dog_prior_map_interfaces::NdtScanResult& result) {
  return Key{result.frontend_session_id, result.epoch, result.transaction_id};
}

bool exactPoseStamped(const geometry_msgs::PoseStamped& lhs,
                      const geometry_msgs::PoseStamped& rhs) {
  return lhs.header.stamp == rhs.header.stamp &&
         lhs.header.frame_id == rhs.header.frame_id &&
         lhs.pose.position.x == rhs.pose.position.x &&
         lhs.pose.position.y == rhs.pose.position.y &&
         lhs.pose.position.z == rhs.pose.position.z &&
         lhs.pose.orientation.x == rhs.pose.orientation.x &&
         lhs.pose.orientation.y == rhs.pose.orientation.y &&
         lhs.pose.orientation.z == rhs.pose.orientation.z &&
         lhs.pose.orientation.w == rhs.pose.orientation.w;
}

void setTimeNs(ros::Time* stamp, uint64_t nanoseconds) {
  stamp->fromNSec(nanoseconds);
}

bool requestTimestampsConsistent(
    const dog_prior_map_interfaces::NdtScanRequest& request) {
  const uint64_t start_ns = request.scan_start.toNSec();
  const uint64_t end_ns = request.scan_end.toNSec();
  return start_ns == request.scan_start_ns && end_ns == request.scan_end_ns &&
         request.header.stamp.toNSec() == request.scan_end_ns &&
         request.scan_start_ns < request.scan_end_ns &&
         request.cloud_end_frame.header.stamp.toNSec() == request.scan_end_ns &&
         request.predicted_map_T_lidar.header.stamp.toNSec() == request.scan_end_ns;
}

bool resultTimestampsConsistent(
    const dog_prior_map_interfaces::NdtScanResult& result) {
  const uint64_t start_ns = result.scan_start.toNSec();
  const uint64_t end_ns = result.scan_end.toNSec();
  return start_ns == result.scan_start_ns && end_ns == result.scan_end_ns &&
         result.header.stamp.toNSec() == result.scan_end_ns &&
         result.scan_start_ns < result.scan_end_ns &&
         result.raw_map_T_lidar.header.stamp.toNSec() == result.scan_end_ns &&
         result.used_map_T_lidar.header.stamp.toNSec() == result.scan_end_ns;
}

constexpr double kQuaternionUnitNormTolerance = 1e-6;

bool finiteAndUnitQuaternion(const geometry_msgs::Quaternion& quaternion) {
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
      !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w)) {
    return false;
  }
  const double norm = std::sqrt(quaternion.x * quaternion.x +
                                quaternion.y * quaternion.y +
                                quaternion.z * quaternion.z +
                                quaternion.w * quaternion.w);
  return std::isfinite(norm) && norm > 1e-12 &&
         std::fabs(norm - 1.0) <= kQuaternionUnitNormTolerance;
}

bool finitePose(const geometry_msgs::PoseStamped& pose) {
  return std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y) &&
         std::isfinite(pose.pose.position.z) &&
         finiteAndUnitQuaternion(pose.pose.orientation);
}

bool limiterFlagsConsistent(
    const dog_prior_map_interfaces::NdtScanResult& result) {
  return result.step_limited ==
         (result.translation_limited || result.rotation_limited);
}

dog_prior_map_interfaces::NdtScanResult makeValidTerminalResult(const Key& key) {
  dog_prior_map_interfaces::NdtScanResult result;
  setResultKey(key, &result);
  result.protocol_version = 1;
  result.server_instance_id = kServerA;
  result.scan_start_ns = 10;
  result.scan_end_ns = 20;
  setTimeNs(&result.scan_start, result.scan_start_ns);
  setTimeNs(&result.scan_end, result.scan_end_ns);
  setTimeNs(&result.header.stamp, result.scan_end_ns);
  result.header.frame_id = "map";
  result.request_cloud_hash = 30;
  result.map_frame = "map";
  result.lidar_frame = "lidar";
  result.disposition = dog_prior_map_interfaces::NdtScanResult::SUCCESS;
  result.pose_valid = true;
  result.fitness = 0.25;
  result.raw_map_T_lidar.header.stamp = result.header.stamp;
  result.raw_map_T_lidar.header.frame_id = "map";
  result.raw_map_T_lidar.pose.orientation.w = 1.0;
  result.used_map_T_lidar.header.stamp = result.header.stamp;
  result.used_map_T_lidar.header.frame_id = "map";
  result.used_map_T_lidar.pose.orientation.w = 1.0;
  return result;
}

bool exactPointCloud(const sensor_msgs::PointCloud2& lhs,
                     const sensor_msgs::PointCloud2& rhs) {
  if (lhs.header.seq != rhs.header.seq || lhs.header.stamp != rhs.header.stamp ||
      lhs.header.frame_id != rhs.header.frame_id || lhs.height != rhs.height ||
      lhs.width != rhs.width || lhs.fields.size() != rhs.fields.size() ||
      lhs.is_bigendian != rhs.is_bigendian || lhs.point_step != rhs.point_step ||
      lhs.row_step != rhs.row_step || lhs.data != rhs.data ||
      lhs.is_dense != rhs.is_dense) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.fields.size(); ++i) {
    const auto& a = lhs.fields[i];
    const auto& b = rhs.fields[i];
    if (a.name != b.name || a.offset != b.offset || a.datatype != b.datatype ||
        a.count != b.count) {
      return false;
    }
  }
  return true;
}

bool exactRequestIdentity(const dog_prior_map_interfaces::NdtScanRequest& lhs,
                          const dog_prior_map_interfaces::NdtScanRequest& rhs) {
  return lhs.protocol_version == rhs.protocol_version &&
         lhs.frontend_session_id == rhs.frontend_session_id &&
         lhs.epoch == rhs.epoch && lhs.transaction_id == rhs.transaction_id &&
         lhs.server_instance_id == rhs.server_instance_id &&
         lhs.scan_start == rhs.scan_start && lhs.scan_end == rhs.scan_end &&
         lhs.scan_start_ns == rhs.scan_start_ns && lhs.scan_end_ns == rhs.scan_end_ns &&
         lhs.map_frame == rhs.map_frame && lhs.lidar_frame == rhs.lidar_frame &&
         exactPointCloud(lhs.cloud_end_frame, rhs.cloud_end_frame) &&
         exactPoseStamped(lhs.predicted_map_T_lidar, rhs.predicted_map_T_lidar) &&
         lhs.request_cloud_hash == rhs.request_cloud_hash;
}

bool sameServerConfiguration(const dog_prior_map_interfaces::NdtServerStatus& lhs,
                             const dog_prior_map_interfaces::NdtServerStatus& rhs) {
  return lhs.protocol_version == rhs.protocol_version &&
         lhs.external_mode_enabled == rhs.external_mode_enabled &&
         lhs.server_instance_id == rhs.server_instance_id &&
         lhs.map_sha256 == rhs.map_sha256 &&
         lhs.canonical_ndt_config_sha256 == rhs.canonical_ndt_config_sha256 &&
         lhs.map_frame == rhs.map_frame && lhs.lidar_frame == rhs.lidar_frame &&
         lhs.terminal_cache_max_entries == rhs.terminal_cache_max_entries;
}

enum class ServerRequestOutcome {
  kProcessed,
  kCached,
  kPayloadConflict,
  kCacheExhausted,
  kServerNotReady,
  kMalformed,
  kTimestampInconsistent,
  kQueueOverflow,
  kFatalLatched,
  kProtocolMismatch,
  kServerInstanceMismatch,
  kUnboundSession,
  kForeignSession,
  kStaleEpoch,
  kFutureEpoch
};

class ProtocolTestServerHarness {
 public:
  ProtocolTestServerHarness(const dog_prior_map_interfaces::NdtServerStatus& status,
                            std::size_t cache_capacity,
                            std::size_t queue_capacity = 16)
      : immutable_status_(status), status_(status), cache_(cache_capacity),
        queue_capacity_(queue_capacity),
        ready_(status.server_ready && status.external_mode_enabled &&
               !status.fatal_latched &&
               status.fatal_reason ==
                   dog_prior_map_interfaces::NdtServerStatus::FATAL_NONE),
        fatal_latched_(status.fatal_latched),
        fatal_reason_(status.fatal_reason) {}

  bool updateStatus(const dog_prior_map_interfaces::NdtServerStatus& incoming) {
    if (incoming.fatal_latched) {
      latchFatal(incoming.fatal_reason);
      return false;
    }
    if (!sameServerConfiguration(immutable_status_, incoming) ||
        !incoming.server_ready || !incoming.external_mode_enabled) {
      ready_ = false;
      status_.server_ready = false;
      return false;
    }
    return ready_;
  }

  bool applyControl(const dog_prior_map_interfaces::NdtSessionControl& control,
                    dog_prior_map_interfaces::NdtSessionControlAck* ack) {
    ack->command = control.command;
    ack->protocol_version = immutable_status_.protocol_version;
    ack->server_instance_id = immutable_status_.server_instance_id;
    ack->frontend_session_id = control.frontend_session_id;
    ack->epoch = control.epoch;
    ack->result = dog_prior_map_interfaces::NdtSessionControlAck::ACCEPTED;
    ack->reason.clear();
    if (fatal_latched_) return rejectControl(ack,
        dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED,
        "fatal_latched_requires_process_restart");
    if (!ready_) return rejectControl(ack,
        dog_prior_map_interfaces::NdtSessionControlAck::REJECT_INVALID_TRANSITION,
        "server_not_ready");
    if (control.protocol_version != immutable_status_.protocol_version)
      return rejectControl(ack,
          dog_prior_map_interfaces::NdtSessionControlAck::REJECT_PROTOCOL_VERSION,
          "protocol_version_mismatch");
    if (control.server_instance_id != immutable_status_.server_instance_id)
      return rejectControl(ack,
          dog_prior_map_interfaces::NdtSessionControlAck::REJECT_SERVER_INSTANCE,
          "server_instance_mismatch");
    if (control.expected_map_sha256 != immutable_status_.map_sha256)
      return rejectControl(ack,
          dog_prior_map_interfaces::NdtSessionControlAck::REJECT_MAP_IDENTITY,
          "map_hash_mismatch");
    if (control.expected_ndt_config_sha256 !=
        immutable_status_.canonical_ndt_config_sha256)
      return rejectControl(ack,
          dog_prior_map_interfaces::NdtSessionControlAck::REJECT_CONFIG_IDENTITY,
          "ndt_config_hash_mismatch");

    switch (control.command) {
      case dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION:
        if (!isUuidV4(control.frontend_session_id) || control.epoch != 0 ||
            control.reason != dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START ||
            (session_bound_ && control.frontend_session_id == active_session_)) {
          return rejectControl(ack,
              dog_prior_map_interfaces::NdtSessionControlAck::REJECT_INVALID_TRANSITION,
              "invalid_begin_session");
        }
        clearEpochState();
        active_session_ = control.frontend_session_id;
        active_epoch_ = 0;
        session_bound_ = true;
        break;
      case dog_prior_map_interfaces::NdtSessionControl::BEGIN_EPOCH:
        if (!session_bound_ || control.frontend_session_id != active_session_)
          return rejectControl(ack,
              dog_prior_map_interfaces::NdtSessionControlAck::REJECT_SESSION_OR_EPOCH,
              "session_not_bound");
        if (active_epoch_ == std::numeric_limits<uint32_t>::max() ||
            control.epoch != active_epoch_ + 1 ||
            (control.reason != dog_prior_map_interfaces::NdtSessionControl::REASON_EXPLICIT_FILTER_RESET &&
             control.reason != dog_prior_map_interfaces::NdtSessionControl::REASON_SENSOR_TIME_REWIND &&
             control.reason != dog_prior_map_interfaces::NdtSessionControl::REASON_DATASET_RESTART)) {
          return rejectControl(ack,
              dog_prior_map_interfaces::NdtSessionControlAck::REJECT_INVALID_TRANSITION,
              "epoch_must_increment_exactly_once");
        }
        clearEpochState();
        active_epoch_ = control.epoch;
        break;
      case dog_prior_map_interfaces::NdtSessionControl::END_SESSION:
        if (!session_bound_ || control.frontend_session_id != active_session_ ||
            control.epoch != active_epoch_)
          return rejectControl(ack,
              dog_prior_map_interfaces::NdtSessionControlAck::REJECT_SESSION_OR_EPOCH,
              "end_session_binding_mismatch");
        clearEpochState();
        active_session_.clear();
        active_epoch_ = 0;
        session_bound_ = false;
        break;
      default:
        return rejectControl(ack,
            dog_prior_map_interfaces::NdtSessionControlAck::REJECT_INVALID_TRANSITION,
            "unknown_control_command");
    }
    status_.bound_frontend_session_id = session_bound_ ? active_session_ : "";
    status_.epoch = active_epoch_;
    return true;
  }

  bool enqueue(const dog_prior_map_interfaces::NdtScanRequest& request) {
    if (fatal_latched_) return false;
    if (requests_.size() >= queue_capacity_) {
      latchFatal(dog_prior_map_interfaces::NdtServerStatus::FATAL_QUEUE_OVERFLOW);
      return false;
    }
    requests_.push_back(request);
    return true;
  }

  bool acknowledge(const dog_prior_map_interfaces::NdtScanAck& ack) {
    if (fatal_latched_) return false;
    return cache_.acknowledge(ack);
  }

  bool processNext(ServerRequestOutcome* outcome,
                   dog_prior_map_interfaces::NdtScanResult* result) {
    if (fatal_latched_)
      return setOutcome(outcome, ServerRequestOutcome::kFatalLatched);
    if (requests_.empty()) return false;
    const auto request = requests_.front();
    requests_.pop_front();
    if (!ready_) return setOutcome(outcome, ServerRequestOutcome::kServerNotReady);
    if (request.transaction_id == 0 || request.map_frame != immutable_status_.map_frame ||
        request.lidar_frame != immutable_status_.lidar_frame ||
        request.header.frame_id != request.map_frame ||
        request.cloud_end_frame.header.frame_id != request.lidar_frame ||
        request.predicted_map_T_lidar.header.frame_id != request.map_frame)
      return setOutcome(outcome, ServerRequestOutcome::kMalformed);
    if (!requestTimestampsConsistent(request)) {
      result->protocol_version = request.protocol_version;
      result->frontend_session_id = request.frontend_session_id;
      result->epoch = request.epoch;
      result->transaction_id = request.transaction_id;
      result->server_instance_id = immutable_status_.server_instance_id;
      result->scan_start = request.scan_start;
      result->scan_end = request.scan_end;
      result->scan_start_ns = request.scan_start_ns;
      result->scan_end_ns = request.scan_end_ns;
      result->disposition = dog_prior_map_interfaces::NdtScanResult::ERROR_REQUEST_TIMESTAMP_INCONSISTENT;
      result->reason = "request_timestamp_inconsistent";
      return setOutcome(outcome, ServerRequestOutcome::kTimestampInconsistent);
    }
    if (request.protocol_version != immutable_status_.protocol_version)
      return setOutcome(outcome, ServerRequestOutcome::kProtocolMismatch);
    if (request.server_instance_id != immutable_status_.server_instance_id)
      return setOutcome(outcome, ServerRequestOutcome::kServerInstanceMismatch);
    if (!session_bound_)
      return setOutcome(outcome, ServerRequestOutcome::kUnboundSession);
    if (request.frontend_session_id != active_session_)
      return setOutcome(outcome, ServerRequestOutcome::kForeignSession);
    if (request.epoch < active_epoch_)
      return setOutcome(outcome, ServerRequestOutcome::kStaleEpoch);
    if (request.epoch > active_epoch_)
      return setOutcome(outcome, ServerRequestOutcome::kFutureEpoch);

    const Key key{request.frontend_session_id, request.epoch, request.transaction_id};
    TerminalResult cached;
    const CacheAction action = cache_.submit(key, request, &cached);
    if (action == CacheAction::kConflict)
      return setOutcome(outcome, ServerRequestOutcome::kPayloadConflict);
    if (action == CacheAction::kExhausted) {
      latchFatal(dog_prior_map_interfaces::NdtServerStatus::FATAL_CACHE_EXHAUSTED);
      fillCacheExhaustedResult(request, result);
      return setOutcome(outcome, ServerRequestOutcome::kCacheExhausted);
    }
    if (action == CacheAction::kProcessed) {
      external_history_valid_ = true;
      fillSyntheticTerminal(request, cached, result);
      cached_results_.emplace(key, *result);
    } else {
      auto cached_result = cached_results_.find(key);
      if (cached_result == cached_results_.end())
        return setOutcome(outcome, ServerRequestOutcome::kPayloadConflict);
      *result = cached_result->second;
    }
    *outcome = action == CacheAction::kProcessed ? ServerRequestOutcome::kProcessed
                                                : ServerRequestOutcome::kCached;
    return true;
  }

  uint64_t ndtProcessCount() const { return cache_.ndtProcessCount(); }
  std::size_t queuedCount() const { return requests_.size(); }
  std::size_t workerCount() const { return 1; }
  bool sessionBound() const { return session_bound_; }
  const std::string& activeSession() const { return active_session_; }
  uint32_t activeEpoch() const { return active_epoch_; }
  bool ready() const { return ready_; }
  bool fatalLatched() const { return fatal_latched_; }
  uint8_t fatalReason() const { return fatal_reason_; }
  const dog_prior_map_interfaces::NdtServerStatus& status() const { return status_; }
  bool externalHistoryValid() const { return external_history_valid_; }
  bool queueOverflowFatal() const {
    return fatal_latched_ &&
           fatal_reason_ == dog_prior_map_interfaces::NdtServerStatus::FATAL_QUEUE_OVERFLOW;
  }

 private:
  bool rejectControl(dog_prior_map_interfaces::NdtSessionControlAck* ack,
                     uint8_t result, const std::string& reason) {
    ack->result = result;
    ack->reason = reason;
    return false;
  }

  void latchFatal(uint8_t reason) {
    if (fatal_latched_) return;
    fatal_latched_ = true;
    fatal_reason_ = reason;
    ready_ = false;
    status_.server_ready = false;
    status_.fatal_latched = true;
    status_.fatal_reason = reason;
  }

  bool setOutcome(ServerRequestOutcome* outcome, ServerRequestOutcome value) {
    *outcome = value;
    return true;
  }

  void clearEpochState() {
    requests_.clear();
    cache_.clear();
    cached_results_.clear();
    external_history_valid_ = false;
  }

  void fillSyntheticTerminal(
      const dog_prior_map_interfaces::NdtScanRequest& request,
      const TerminalResult& token,
      dog_prior_map_interfaces::NdtScanResult* result) {
    result->protocol_version = request.protocol_version;
    result->frontend_session_id = request.frontend_session_id;
    result->epoch = request.epoch;
    result->transaction_id = request.transaction_id;
    result->server_instance_id = request.server_instance_id;
    result->scan_start = request.scan_start;
    result->scan_end = request.scan_end;
    result->scan_start_ns = request.scan_start_ns;
    result->scan_end_ns = request.scan_end_ns;
    result->header.stamp = request.scan_end;
    result->header.frame_id = request.map_frame;
    result->map_frame = request.map_frame;
    result->lidar_frame = request.lidar_frame;
    result->raw_map_T_lidar = request.predicted_map_T_lidar;
    result->used_map_T_lidar = request.predicted_map_T_lidar;
    result->request_cloud_hash = request.request_cloud_hash;
    result->iterations = static_cast<uint32_t>(token.deterministic_token);
    result->fitness = 0.0;
    result->pose_valid = true;
    result->converged = true;
    result->disposition = dog_prior_map_interfaces::NdtScanResult::SUCCESS;
  }

  void fillCacheExhaustedResult(
      const dog_prior_map_interfaces::NdtScanRequest& request,
      dog_prior_map_interfaces::NdtScanResult* result) {
    result->protocol_version = request.protocol_version;
    result->frontend_session_id = request.frontend_session_id;
    result->epoch = request.epoch;
    result->transaction_id = request.transaction_id;
    result->server_instance_id = immutable_status_.server_instance_id;
    result->scan_start = request.scan_start;
    result->scan_end = request.scan_end;
    result->scan_start_ns = request.scan_start_ns;
    result->scan_end_ns = request.scan_end_ns;
    result->header.stamp = request.scan_end;
    result->header.frame_id = request.map_frame;
    result->map_frame = request.map_frame;
    result->lidar_frame = request.lidar_frame;
    result->raw_map_T_lidar.header = result->header;
    result->used_map_T_lidar.header = result->header;
    result->request_cloud_hash = request.request_cloud_hash;
    result->pose_valid = false;
    result->disposition =
        dog_prior_map_interfaces::NdtScanResult::ERROR_TERMINAL_CACHE_EXHAUSTED;
    result->reason = "terminal_cache_exhausted";
  }

  dog_prior_map_interfaces::NdtServerStatus immutable_status_;
  dog_prior_map_interfaces::NdtServerStatus status_;
  TerminalCacheModel cache_;
  std::size_t queue_capacity_;
  std::deque<dog_prior_map_interfaces::NdtScanRequest> requests_;
  std::map<Key, dog_prior_map_interfaces::NdtScanResult> cached_results_;
  std::string active_session_;
  uint32_t active_epoch_ = 0;
  bool session_bound_ = false;
  bool ready_ = false;
  bool external_history_valid_ = false;
  bool fatal_latched_ = false;
  uint8_t fatal_reason_ = dog_prior_map_interfaces::NdtServerStatus::FATAL_NONE;
};

bool exactTerminalIdentity(const dog_prior_map_interfaces::NdtScanResult& lhs,
                           const dog_prior_map_interfaces::NdtScanResult& rhs) {
  return lhs.protocol_version == rhs.protocol_version &&
         lhs.frontend_session_id == rhs.frontend_session_id &&
         lhs.epoch == rhs.epoch && lhs.transaction_id == rhs.transaction_id &&
         lhs.server_instance_id == rhs.server_instance_id &&
         lhs.scan_start == rhs.scan_start && lhs.scan_end == rhs.scan_end &&
         lhs.scan_start_ns == rhs.scan_start_ns &&
         lhs.scan_end_ns == rhs.scan_end_ns &&
         lhs.disposition == rhs.disposition && lhs.reason == rhs.reason &&
         lhs.pose_valid == rhs.pose_valid && lhs.map_frame == rhs.map_frame &&
         lhs.lidar_frame == rhs.lidar_frame &&
         exactPoseStamped(lhs.raw_map_T_lidar, rhs.raw_map_T_lidar) &&
         exactPoseStamped(lhs.used_map_T_lidar, rhs.used_map_T_lidar) &&
         lhs.fitness == rhs.fitness && lhs.iterations == rhs.iterations &&
         lhs.converged == rhs.converged &&
         lhs.request_cloud_hash == rhs.request_cloud_hash &&
         lhs.ndt_source_cloud_hash == rhs.ndt_source_cloud_hash &&
         lhs.translation_limited == rhs.translation_limited &&
         lhs.rotation_limited == rhs.rotation_limited &&
         lhs.step_limited == rhs.step_limited &&
         lhs.header.stamp == rhs.header.stamp &&
         lhs.header.frame_id == rhs.header.frame_id;
}

enum class FrontendFatalReason {
  kNone,
  kQueueOverflow,
  kLedgerExhausted,
  kNondeterministicResult
};

class CompletedTerminalLedger {
 public:
  explicit CompletedTerminalLedger(std::size_t max_entries)
      : max_entries_(max_entries) {}

  bool remember(const dog_prior_map_interfaces::NdtScanResult& result) {
    if (fatal_latched_) return false;
    const Key key = resultKey(result);
    auto found = entries_.find(key);
    if (found != entries_.end()) {
      if (exactTerminalIdentity(found->second, result)) return true;
      latchFatal(FrontendFatalReason::kNondeterministicResult);
      return false;
    }
    if (entries_.size() >= max_entries_) {
      latchFatal(FrontendFatalReason::kLedgerExhausted);
      return false;
    }
    entries_.emplace(key, result);
    return true;
  }

  const dog_prior_map_interfaces::NdtScanResult* find(const Key& key) const {
    auto found = entries_.find(key);
    return found == entries_.end() ? nullptr : &found->second;
  }

  bool resetEpoch(const std::string& frontend_id, uint32_t epoch) {
    if (fatal_latched_) return false;
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (it->first.frontend_session_id == frontend_id && it->first.epoch == epoch)
        it = entries_.erase(it);
      else
        ++it;
    }
    return true;
  }

  bool exhausted() const {
    return fatal_reason_ == FrontendFatalReason::kLedgerExhausted;
  }
  bool fatalLatched() const { return fatal_latched_; }
  FrontendFatalReason fatalReason() const { return fatal_reason_; }
  bool acceptsTransactions() const { return !fatal_latched_; }
  std::size_t size() const { return entries_.size(); }

 private:
  void latchFatal(FrontendFatalReason reason) {
    if (fatal_latched_) return;
    fatal_latched_ = true;
    fatal_reason_ = reason;
  }

  std::size_t max_entries_;
  bool fatal_latched_ = false;
  FrontendFatalReason fatal_reason_ = FrontendFatalReason::kNone;
  std::map<Key, dog_prior_map_interfaces::NdtScanResult> entries_;
};

class FrontendQueueFatalHarness {
 public:
  FrontendQueueFatalHarness(std::size_t scan_capacity,
                            std::size_t result_capacity)
      : scan_capacity_(scan_capacity), result_capacity_(result_capacity) {}

  bool enqueueScan() { return enqueue(&scan_queue_size_, scan_capacity_); }
  bool enqueueResult() { return enqueue(&result_queue_size_, result_capacity_); }
  bool acceptsTransactions() const { return !fatal_latched_; }
  bool ready() const { return !fatal_latched_; }
  bool beginEpoch() const { return !fatal_latched_; }
  bool beginSession() const { return !fatal_latched_; }
  bool fatalLatched() const { return fatal_latched_; }
  FrontendFatalReason fatalReason() const { return fatal_reason_; }

 private:
  bool enqueue(std::size_t* queue_size, std::size_t capacity) {
    if (fatal_latched_) return false;
    if (*queue_size >= capacity) {
      fatal_latched_ = true;
      fatal_reason_ = FrontendFatalReason::kQueueOverflow;
      return false;
    }
    ++*queue_size;
    return true;
  }

  std::size_t scan_capacity_;
  std::size_t result_capacity_;
  std::size_t scan_queue_size_ = 0;
  std::size_t result_queue_size_ = 0;
  bool fatal_latched_ = false;
  FrontendFatalReason fatal_reason_ = FrontendFatalReason::kNone;
};

class FrontendTransactionHarness {
 public:
  explicit FrontendTransactionHarness(std::size_t ledger_capacity)
      : ledger_(ledger_capacity) {}

  bool beginTransaction() {
    if (!ledger_.acceptsTransactions()) return false;
    ++ndt_process_count_;
    candidate_active_ = true;
    return true;
  }

  bool completeTerminal(const dog_prior_map_interfaces::NdtScanResult& result) {
    if (!candidate_active_ || !ledger_.remember(result)) {
      candidate_active_ = false;
      return false;
    }
    candidate_active_ = false;
    ++committed_count_;
    return true;
  }

  bool beginEpoch(const std::string& session, uint32_t epoch) {
    return ledger_.resetEpoch(session, epoch);
  }
  bool beginSession() const { return ledger_.acceptsTransactions(); }
  bool fatalLatched() const { return ledger_.fatalLatched(); }
  bool acceptsTransactions() const { return ledger_.acceptsTransactions(); }
  bool candidateActive() const { return candidate_active_; }
  std::size_t committedCount() const { return committed_count_; }
  uint64_t ndtProcessCount() const { return ndt_process_count_; }
  FrontendFatalReason fatalReason() const { return ledger_.fatalReason(); }
  std::string fatalReasonName() const {
    if (ledger_.fatalReason() == FrontendFatalReason::kLedgerExhausted)
      return "FATAL_LEDGER_EXHAUSTED";
    if (ledger_.fatalReason() == FrontendFatalReason::kNondeterministicResult)
      return "FATAL_NONDETERMINISTIC_SERVER_RESULT";
    if (ledger_.fatalReason() == FrontendFatalReason::kQueueOverflow)
      return "FATAL_QUEUE_OVERFLOW";
    return "FATAL_NONE";
  }

 private:
  CompletedTerminalLedger ledger_;
  uint64_t ndt_process_count_ = 0;
  std::size_t committed_count_ = 0;
  bool candidate_active_ = false;
};

ResultDecision classifyResult(const Key& current,
                              const dog_prior_map_interfaces::NdtScanResult& result,
                              const std::string& bound_server_id,
                              const CompletedTerminalLedger& completed) {
  if (!isUuidV4(result.frontend_session_id) || result.transaction_id == 0 ||
      !isUuidV4(result.server_instance_id))
    return ResultDecision::kFatalMalformedResult;
  const Key incoming = resultKey(result);
  const auto* saved = completed.find(incoming);
  if (saved != nullptr) {
    return exactTerminalIdentity(*saved, result)
               ? ResultDecision::kDropDuplicateResult
               : ResultDecision::kFatalNondeterministicResult;
  }
  if (incoming.frontend_session_id != current.frontend_session_id)
    return ResultDecision::kDropForeignSession;
  if (incoming.epoch < current.epoch) return ResultDecision::kDropStaleEpoch;
  if (incoming.epoch > current.epoch) return ResultDecision::kFatalFutureEpoch;
  if (incoming.transaction_id < current.transaction_id)
    return ResultDecision::kDropOldTransaction;
  if (incoming.transaction_id > current.transaction_id)
    return ResultDecision::kFatalFutureTransaction;
  if (result.server_instance_id != bound_server_id)
    return ResultDecision::kFatalServerInstanceChanged;
  if (!resultTimestampsConsistent(result) || result.scan_start_ns != 10 ||
      result.scan_end_ns != 20 ||
      result.request_cloud_hash != 30 || result.map_frame != "map" ||
      result.lidar_frame != "lidar" || result.header.frame_id != "map" ||
      result.raw_map_T_lidar.header.frame_id != "map" ||
      result.used_map_T_lidar.header.frame_id != "map" ||
      result.protocol_version != 1 || !limiterFlagsConsistent(result))
    return ResultDecision::kFatalCurrentPayloadCorruption;
  if (result.disposition == dog_prior_map_interfaces::NdtScanResult::SUCCESS) {
    if (!result.pose_valid || !finitePose(result.raw_map_T_lidar) ||
        !finitePose(result.used_map_T_lidar) || !std::isfinite(result.fitness)) {
      return ResultDecision::kFatalCurrentPayloadCorruption;
    }
    return ResultDecision::kAcceptCurrent;
  }
  if (result.pose_valid) return ResultDecision::kFatalCurrentPayloadCorruption;
  if (result.disposition ==
          dog_prior_map_interfaces::NdtScanResult::REJECT_INSUFFICIENT_POINTS ||
      result.disposition ==
          dog_prior_map_interfaces::NdtScanResult::REJECT_NOT_CONVERGED) {
    return ResultDecision::kAcceptCurrent;
  }
  return ResultDecision::kFatalCurrentPayloadCorruption;
}

dog_prior_map_interfaces::NdtServerStatus makeReadyStatus(
    const StatusExpectation& expected) {
  dog_prior_map_interfaces::NdtServerStatus status;
  status.protocol_version = expected.protocol_version;
  status.server_ready = true;
  status.external_mode_enabled = true;
  std::copy(expected.map_hash.begin(), expected.map_hash.end(),
            status.map_sha256.begin());
  std::copy(expected.config_hash.begin(), expected.config_hash.end(),
            status.canonical_ndt_config_sha256.begin());
  status.server_instance_id = kServerA;
  status.map_frame = expected.map_frame;
  status.lidar_frame = expected.lidar_frame;
  status.bound_frontend_session_id.clear();
  status.epoch = 0;
  status.terminal_cache_max_entries = expected.cache_capacity;
  return status;
}

dog_prior_map_interfaces::NdtSessionControl makeControl(
    const dog_prior_map_interfaces::NdtServerStatus& status, uint8_t command,
    const std::string& frontend_id, uint32_t epoch, uint8_t reason) {
  dog_prior_map_interfaces::NdtSessionControl control;
  control.command = command;
  control.protocol_version = status.protocol_version;
  control.server_instance_id = status.server_instance_id;
  control.frontend_session_id = frontend_id;
  control.epoch = epoch;
  control.reason = reason;
  control.expected_map_sha256 = status.map_sha256;
  control.expected_ndt_config_sha256 = status.canonical_ndt_config_sha256;
  return control;
}

void testWaitServerAdversarial() {
  StatusExpectation expected;
  expected.map_hash.fill(0x12);
  expected.config_hash.fill(0x34);
  auto status = makeReadyStatus(expected);
  std::string reason;

  auto checkWait = [&](dog_prior_map_interfaces::NdtServerStatus candidate,
                       const std::string& label) {
    reason.clear();
    expect(!statusMatches(candidate, expected, &reason) && !reason.empty(), label);
  };

  auto candidate = status;
  candidate.server_ready = false;
  checkWait(candidate, "ready=false stays WAIT_SERVER");
  candidate = status;
  candidate.fatal_latched = true;
  candidate.fatal_reason = dog_prior_map_interfaces::NdtServerStatus::FATAL_QUEUE_OVERFLOW;
  checkWait(candidate, "fatal server status cannot pass startup handshake");
  candidate = status;
  ++candidate.protocol_version;
  checkWait(candidate, "protocol mismatch stays WAIT_SERVER");
  candidate = status;
  candidate.map_sha256[0] ^= 0x01;
  checkWait(candidate, "map hash mismatch stays WAIT_SERVER");
  candidate = status;
  candidate.canonical_ndt_config_sha256[0] ^= 0x01;
  checkWait(candidate, "config hash mismatch stays WAIT_SERVER");
  candidate = status;
  candidate.lidar_frame = "wrong_lidar";
  checkWait(candidate, "LiDAR frame mismatch stays WAIT_SERVER");
  candidate = status;
  candidate.external_mode_enabled = false;
  checkWait(candidate, "external mode disabled stays WAIT_SERVER");
  candidate = status;
  candidate.server_instance_id.clear();
  checkWait(candidate, "missing server instance stays WAIT_SERVER");
  candidate = status;
  candidate.terminal_cache_max_entries = expected.maximum_expected_scans;
  checkWait(candidate, "cache capacity must exceed expected scan count");
  expect(statusMatches(status, expected, &reason) && reason.empty(),
         "all startup guards match");
  candidate = status;
  candidate.bound_frontend_session_id = kFrontendB;
  expect(statusMatches(candidate, expected, &reason),
         "server readiness check permits an explicit session handover");
  expect(!statusBindsSession(candidate, kServerA, kFrontendA, 0),
         "previous client binding cannot authorize this frontend's scans");

  dog_prior_map_interfaces::NdtServerStatus bound = status;
  bound.bound_frontend_session_id = kFrontendA;
  expect(statusBindsSession(bound, kServerA, kFrontendA, 0),
         "server status must echo exact frontend session and epoch");
  expect(!statusBindsSession(bound, kServerA, kFrontendB, 0),
         "wrong frontend session cannot pass binding");
  bound.fatal_latched = true;
  bound.fatal_reason = dog_prior_map_interfaces::NdtServerStatus::FATAL_CACHE_EXHAUSTED;
  expect(!statusBindsSession(bound, kServerA, kFrontendA, 0),
         "fatal server status cannot authorize a session binding");

  InstanceGuard guard;
  expect(guard.observe(status, true) == InstanceDecision::kInitialBind,
         "bind initial valid server");
  candidate = status;
  candidate.server_instance_id = kServerB;
  expect(guard.observe(candidate, true) == InstanceDecision::kFatalRestart,
         "server restart after start is fatal");

  InstanceGuard waiting_guard;
  auto not_ready = status;
  not_ready.server_ready = false;
  expect(waiting_guard.observe(not_ready, false) == InstanceDecision::kWait,
         "not-ready status keeps WAIT_SERVER unbound");
  auto later_server = status;
  later_server.server_instance_id = kServerB;
  expect(waiting_guard.observe(later_server, true) == InstanceDecision::kInitialBind,
         "WAIT_SERVER binds a later valid server instance");
  std::cout << "WAIT_SERVER_ADVERSARIAL_PASS\n";
}

void testSessionEpochAndResultClassification() {
  FrontendIdentity identity;
  expect(isUuidV4(identity.session), "frontend session is a valid UUIDv4");
  expect(isUuidV4(kServerA), "server instance is a valid UUIDv4");
  Key first = identity.allocate();
  Key second = identity.allocate();
  expect(first.frontend_session_id == kFrontendA && first.epoch == 0 &&
             first.transaction_id == 1 && second.transaction_id == 2,
         "frontend owns one stable session and monotonic transaction IDs");
  expect(identity.beginNextEpoch(), "frontend can increment epoch");
  Key reset_key = identity.allocate();
  expect(reset_key.frontend_session_id == kFrontendA && reset_key.epoch == 1 &&
             reset_key.transaction_id == 1,
         "epoch reset is frontend-owned and transaction IDs restart per epoch");
  identity.session = kFrontendB;
  identity.epoch = 0;
  identity.next_transaction = 1;
  Key restarted = identity.allocate();
  expect(restarted.frontend_session_id == kFrontendB,
         "frontend restart creates a new session ID");

  const Key current{kFrontendA, 2, 10};
  auto result = makeValidTerminalResult(current);
  CompletedTerminalLedger completed(4);
  result.server_instance_id.clear();
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kFatalMalformedResult,
         "structurally invalid identity is fatal before ledger/key classification");
  result.server_instance_id = kServerA;
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kAcceptCurrent,
         "exact current result accepted");
  result.frontend_session_id = kFrontendB;
  ++result.protocol_version;
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kDropForeignSession,
         "foreign session result is dropped before protocol mismatch handling");
  result.protocol_version = 1;
  setResultKey(Key{kFrontendA, 1, 99}, &result);
  ++result.protocol_version;
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kDropStaleEpoch,
         "old epoch result is dropped before protocol mismatch handling");
  result.protocol_version = 1;
  setResultKey(Key{kFrontendA, 3, 1}, &result);
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kFatalFutureEpoch,
         "future epoch result is fatal");
  setResultKey(Key{kFrontendA, 2, 9}, &result);
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kDropOldTransaction,
         "old transaction result dropped");
  setResultKey(Key{kFrontendA, 2, 11}, &result);
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kFatalFutureTransaction,
         "unsolicited future transaction is fatal");
  setResultKey(current, &result);
  result.server_instance_id = kServerB;
  expect(classifyResult(current, result, kServerA, completed) ==
             ResultDecision::kFatalServerInstanceChanged,
         "current result from changed server is fatal");
  result.server_instance_id = kServerA;
  auto corrupted = result;
  corrupted.scan_start_ns++;
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key start-stamp mismatch is fatal corruption");
  corrupted = result;
  corrupted.scan_end_ns++;
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key end-stamp mismatch is fatal corruption");
  corrupted = result;
  corrupted.request_cloud_hash++;
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key cloud-hash mismatch is fatal corruption");
  corrupted = result;
  corrupted.map_frame = "odom";
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key map-frame mismatch is fatal corruption");
  corrupted = result;
  corrupted.lidar_frame = "base_link";
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key LiDAR-frame mismatch is fatal corruption");
  corrupted = result;
  corrupted.header.frame_id = "odom";
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key header frame mismatch is fatal corruption");
  corrupted = result;
  ++corrupted.protocol_version;
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current-key protocol mismatch is fatal corruption");
  corrupted = result;
  setTimeNs(&corrupted.header.stamp, 21);
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current result with header stamp differing from scan_end_ns is fatal");
  corrupted = result;
  setTimeNs(&corrupted.scan_end, 21);
  expect(classifyResult(current, corrupted, kServerA, completed) ==
             ResultDecision::kFatalCurrentPayloadCorruption,
         "current result with scan_end differing from scan_end_ns is fatal");

  expect(completed.remember(result), "current terminal identity is recorded on commit");
  const Key waiting_for_next{kFrontendA, 2, 11};
  expect(classifyResult(waiting_for_next, result, kServerA, completed) ==
             ResultDecision::kDropDuplicateResult,
         "waiting T+1 drops an identical completed T before stale classification");
  corrupted = result;
  corrupted.used_map_T_lidar.pose.position.x = 0.5;
  expect(classifyResult(waiting_for_next, corrupted, kServerA, completed) ==
             ResultDecision::kFatalNondeterministicResult,
         "waiting T+1 treats changed completed-T pose as nondeterministic");
  corrupted = result;
  corrupted.disposition = dog_prior_map_interfaces::NdtScanResult::REJECT_NOT_CONVERGED;
  expect(classifyResult(waiting_for_next, corrupted, kServerA, completed) ==
             ResultDecision::kFatalNondeterministicResult,
         "changed completed-T disposition is nondeterministic");
  corrupted = result;
  corrupted.request_cloud_hash++;
  expect(classifyResult(waiting_for_next, corrupted, kServerA, completed) ==
             ResultDecision::kFatalNondeterministicResult,
         "changed completed-T hash is nondeterministic");
  corrupted = result;
  corrupted.fitness = 0.5;
  expect(classifyResult(waiting_for_next, corrupted, kServerA, completed) ==
             ResultDecision::kFatalNondeterministicResult,
         "changed completed-T fitness is nondeterministic");
  corrupted = result;
  corrupted.translation_limited = true;
  corrupted.step_limited = true;
  expect(classifyResult(waiting_for_next, corrupted, kServerA, completed) ==
             ResultDecision::kFatalNondeterministicResult,
         "changed completed-T translation limiter outcome is nondeterministic");
  corrupted = result;
  corrupted.rotation_limited = true;
  corrupted.step_limited = true;
  expect(classifyResult(waiting_for_next, corrupted, kServerA, completed) ==
             ResultDecision::kFatalNondeterministicResult,
         "changed completed-T rotation limiter outcome is nondeterministic");
  CompletedTerminalLedger bounded_ledger(1);
  expect(bounded_ledger.remember(result), "completed ledger stores terminal identity");
  auto overflow = result;
  overflow.transaction_id++;
  expect(!bounded_ledger.remember(overflow) && bounded_ledger.exhausted() &&
             bounded_ledger.size() == 1,
         "completed ledger exhaustion is explicit and does not evict old identity");
  bounded_ledger.resetEpoch(kFrontendA, 2);
  expect(bounded_ledger.size() == 1 && bounded_ledger.exhausted() &&
             bounded_ledger.fatalLatched(),
         "ledger exhaustion cannot be cleared by an epoch reset");
  CompletedTerminalLedger resettable_ledger(1);
  expect(resettable_ledger.remember(result) &&
             resettable_ledger.resetEpoch(kFrontendA, 2) &&
             resettable_ledger.size() == 0 && !resettable_ledger.fatalLatched(),
         "nonfatal explicit epoch reset clears only that epoch's ledger entries");
  std::cout << "SESSION_EPOCH_TRANSACTION_PASS\n";
  std::cout << "RESULT_CLASSIFICATION_PASS\n";
}

void testSuccessPoseAndLimiterContracts() {
  const Key current{kFrontendA, 0, 1};
  CompletedTerminalLedger completed(4);
  const std::string& server_id = kServerA;
  auto result = makeValidTerminalResult(current);

  auto invalid = result;
  invalid.pose_valid = false;
  reportTest("M1", classifyResult(current, invalid, server_id, completed) ==
                        ResultDecision::kFatalCurrentPayloadCorruption,
             "SUCCESS requires pose_valid=true");
  invalid = result;
  invalid.raw_map_T_lidar.pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  reportTest("M2", classifyResult(current, invalid, server_id, completed) ==
                        ResultDecision::kFatalCurrentPayloadCorruption,
             "SUCCESS rejects a non-finite raw pose");
  invalid = result;
  invalid.used_map_T_lidar.pose.position.y =
      std::numeric_limits<double>::infinity();
  reportTest("M3", classifyResult(current, invalid, server_id, completed) ==
                        ResultDecision::kFatalCurrentPayloadCorruption,
             "SUCCESS rejects a non-finite used pose");
  invalid = result;
  invalid.raw_map_T_lidar.pose.orientation = geometry_msgs::Quaternion();
  const bool zero_quaternion_rejected =
      classifyResult(current, invalid, server_id, completed) ==
      ResultDecision::kFatalCurrentPayloadCorruption;
  invalid = result;
  invalid.used_map_T_lidar.pose.orientation.x =
      std::numeric_limits<double>::quiet_NaN();
  const bool nonfinite_quaternion_rejected =
      classifyResult(current, invalid, server_id, completed) ==
      ResultDecision::kFatalCurrentPayloadCorruption;
  invalid = result;
  invalid.raw_map_T_lidar.pose.orientation.w = 2.0;
  reportTest("M4", zero_quaternion_rejected && nonfinite_quaternion_rejected &&
                        classifyResult(current, invalid, server_id, completed) ==
                            ResultDecision::kFatalCurrentPayloadCorruption,
             "SUCCESS rejects zero, non-unit, or non-finite quaternions");
  invalid = result;
  invalid.disposition =
      dog_prior_map_interfaces::NdtScanResult::REJECT_NOT_CONVERGED;
  invalid.pose_valid = true;
  reportTest("M5", classifyResult(current, invalid, server_id, completed) ==
                        ResultDecision::kFatalCurrentPayloadCorruption,
             "non-SUCCESS terminal results require pose_valid=false");
  reportTest("M6", classifyResult(current, result, server_id, completed) ==
                        ResultDecision::kAcceptCurrent,
             "finite success poses with normalized quaternions follow success path");
  auto ordinary_reject = result;
  ordinary_reject.disposition =
      dog_prior_map_interfaces::NdtScanResult::REJECT_INSUFFICIENT_POINTS;
  ordinary_reject.pose_valid = false;
  const bool ordinary_reject_is_prediction_only =
      classifyResult(current, ordinary_reject, server_id, completed) ==
      ResultDecision::kAcceptCurrent;
  ordinary_reject.disposition =
      dog_prior_map_interfaces::NdtScanResult::REJECT_INVALID_SOURCE;
  reportTest("REJECT_POSE_SEMANTICS",
             ordinary_reject_is_prediction_only &&
                 classifyResult(current, ordinary_reject, server_id, completed) ==
                     ResultDecision::kFatalCurrentPayloadCorruption,
             "ordinary rejects are prediction-only; invalid-source remains fatal");

  auto limiterCase = [&](bool translation_limited, bool rotation_limited,
                         bool step_limited) {
    auto candidate = result;
    candidate.translation_limited = translation_limited;
    candidate.rotation_limited = rotation_limited;
    candidate.step_limited = step_limited;
    return candidate;
  };
  reportTest("LIM1", classifyResult(current, limiterCase(false, false, false),
                                      server_id, completed) ==
                          ResultDecision::kAcceptCurrent,
             "no limiter flags agree with derived step_limited=false");
  reportTest("LIM2", classifyResult(current, limiterCase(true, false, true),
                                      server_id, completed) ==
                          ResultDecision::kAcceptCurrent,
             "translation-only limiter flags are consistent");
  reportTest("LIM3", classifyResult(current, limiterCase(false, true, true),
                                      server_id, completed) ==
                          ResultDecision::kAcceptCurrent,
             "rotation-only limiter flags are consistent");
  reportTest("LIM4", classifyResult(current, limiterCase(true, true, true),
                                      server_id, completed) ==
                          ResultDecision::kAcceptCurrent,
             "both limiter flags derive step_limited=true");
  reportTest("LIM5", classifyResult(current, limiterCase(true, false, false),
                                      server_id, completed) ==
                          ResultDecision::kFatalCurrentPayloadCorruption,
             "missing derived step_limited is fatal corruption");
  reportTest("LIM6", classifyResult(current, limiterCase(false, false, true),
                                      server_id, completed) ==
                          ResultDecision::kFatalCurrentPayloadCorruption,
             "spurious step_limited is fatal corruption");
}

void testLedgerFatalContract() {
  constexpr std::size_t capacity = 2;
  FrontendTransactionHarness frontend(capacity);
  bool filled_capacity = true;
  for (uint64_t transaction_id = 1; transaction_id <= capacity; ++transaction_id) {
    const Key key{kFrontendA, 0, transaction_id};
    filled_capacity = filled_capacity && frontend.beginTransaction() &&
                      frontend.completeTerminal(makeValidTerminalResult(key));
  }
  reportTest("G1", filled_capacity && frontend.committedCount() == capacity &&
                        !frontend.fatalLatched(),
             "completed ledger accepts exactly its configured capacity");

  const Key overflow_key{kFrontendA, 0, capacity + 1};
  const bool overflow_started = frontend.beginTransaction();
  const bool overflow_rejected =
      !frontend.completeTerminal(makeValidTerminalResult(overflow_key));
  reportTest("G2", overflow_started && overflow_rejected &&
                        frontend.fatalLatched() &&
                        frontend.fatalReason() == FrontendFatalReason::kLedgerExhausted &&
                        !frontend.candidateActive() &&
                        frontend.committedCount() == capacity,
             "terminal ledger exhaustion rejects the current candidate and latches fatal");
  reportTest("G3", frontend.fatalLatched() &&
                        frontend.fatalReason() == FrontendFatalReason::kLedgerExhausted,
             "ledger exhaustion enters sticky FATAL state");
  const uint64_t count_at_fatal = frontend.ndtProcessCount();
  reportTest("G4", !frontend.acceptsTransactions() && !frontend.beginTransaction(),
             "frontend refuses all new transactions after ledger exhaustion");
  reportTest("G5", frontend.ndtProcessCount() == count_at_fatal,
             "transaction processing count cannot increase after ledger fatal");
  reportTest("G6", !frontend.beginEpoch(kFrontendA, 1) && frontend.fatalLatched(),
             "BEGIN_EPOCH cannot recover a ledger fatal");
  reportTest("G7", !frontend.beginSession() && frontend.fatalLatched(),
             "BEGIN_SESSION cannot recover a ledger fatal");
  reportTest("G8", frontend.fatalReasonName() == "FATAL_LEDGER_EXHAUSTED",
             "frontend exposes FATAL_LEDGER_EXHAUSTED as the retained first reason");
}

void testCacheIdempotenceAndCapacity() {
  TerminalCacheModel cache(2);
  const Key key_a{kFrontendA, 0, 1};
  auto request_a = makeRequest(key_a);
  TerminalResult first_result;
  expect(cache.submit(key_a, request_a, &first_result) == CacheAction::kProcessed &&
             cache.ndtProcessCount() == 1,
         "first request runs NDT once and caches terminal result");
  TerminalResult duplicate_result;
  expect(cache.submit(key_a, request_a, &duplicate_result) == CacheAction::kCached &&
             duplicate_result == first_result && cache.ndtProcessCount() == 1,
         "identical duplicate returns exact cached result without rerun");
  dog_prior_map_interfaces::NdtScanAck ack;
  ack.frontend_session_id = key_a.frontend_session_id;
  ack.epoch = key_a.epoch;
  ack.transaction_id = key_a.transaction_id;
  ack.server_instance_id = request_a.server_instance_id;
  ack.scan_start_ns = request_a.scan_start_ns;
  ack.scan_end_ns = request_a.scan_end_ns;
  ack.request_cloud_hash = request_a.request_cloud_hash;
  expect(cache.acknowledge(ack), "matching ACK accepted");
  expect(cache.acknowledge(ack), "identical ACK is idempotent");
  expect(cache.size() == 1, "ACK does not evict terminal cache entry");
  duplicate_result = TerminalResult{};
  expect(cache.submit(key_a, request_a, &duplicate_result) == CacheAction::kCached &&
             duplicate_result == first_result && cache.ndtProcessCount() == 1,
         "duplicate after ACK returns same cached result without rerun");

  auto conflict = request_a;
  conflict.cloud_end_frame.data[0] ^= 0x01;
  expect(cache.submit(key_a, conflict, &duplicate_result) == CacheAction::kConflict &&
             cache.ndtProcessCount() == 1,
         "conflicting same-key cloud payload is protocol error, no rerun");
  conflict = request_a;
  conflict.predicted_map_T_lidar.pose.position.x = 0.25;
  expect(cache.submit(key_a, conflict, &duplicate_result) == CacheAction::kConflict &&
             cache.ndtProcessCount() == 1,
         "same-key predicted pose conflict is protocol error, no rerun");
  auto conflicting_ack = ack;
  conflicting_ack.request_cloud_hash++;
  expect(!cache.acknowledge(conflicting_ack), "conflicting ACK is rejected");

  const Key key_b{kFrontendA, 0, 2};
  auto request_b = makeRequest(key_b);
  setTimeNs(&request_b.scan_start, 20);
  setTimeNs(&request_b.scan_end, 30);
  request_b.scan_start_ns = 20;
  request_b.scan_end_ns = 30;
  request_b.header.stamp = request_b.scan_end;
  request_b.cloud_end_frame.header.stamp = request_b.scan_end;
  request_b.predicted_map_T_lidar.header.stamp = request_b.scan_end;
  TerminalResult result_b;
  expect(cache.submit(key_b, request_b, &result_b) == CacheAction::kProcessed &&
             cache.ndtProcessCount() == 2,
         "second unique request fills configured cache");
  const Key key_c{kFrontendA, 0, 3};
  auto request_c = makeRequest(key_c);
  setTimeNs(&request_c.scan_start, 30);
  setTimeNs(&request_c.scan_end, 40);
  request_c.scan_start_ns = 30;
  request_c.scan_end_ns = 40;
  request_c.header.stamp = request_c.scan_end;
  request_c.cloud_end_frame.header.stamp = request_c.scan_end;
  request_c.predicted_map_T_lidar.header.stamp = request_c.scan_end;
  expect(cache.submit(key_c, request_c, &duplicate_result) == CacheAction::kExhausted &&
             cache.ndtProcessCount() == 2,
         "cache exhaustion refuses unseen work before NDT");
  expect(cache.submit(key_a, request_a, &duplicate_result) == CacheAction::kCached &&
             duplicate_result == first_result,
         "cached duplicate remains available after capacity exhaustion");
  cache.resetEpoch(kFrontendA, 0);
  expect(cache.size() == 0, "epoch reset clears that epoch cache");
  TerminalCacheModel session_cache(2);
  expect(session_cache.submit(Key{kFrontendA, 0, 1}, request_a, &first_result) ==
             CacheAction::kProcessed,
         "session cache accepts initial key");
  session_cache.resetSession(kFrontendA);
  expect(session_cache.size() == 0, "session replacement clears prior session cache");
  std::cout << "CACHE_IDEMPOTENCE_PASS\n";
}

void testCanonicalRequestTimestamps() {
  const auto valid = makeRequest(Key{kFrontendA, 0, 1});
  expect(requestTimestampsConsistent(valid),
         "generated request carries consistent canonical nanosecond timestamps");
  auto corrupted = valid;
  setTimeNs(&corrupted.header.stamp, 21);
  expect(!requestTimestampsConsistent(corrupted),
         "request header stamp mismatch is rejected before NDT");
  corrupted = valid;
  setTimeNs(&corrupted.scan_end, 21);
  expect(!requestTimestampsConsistent(corrupted),
         "request ROS scan_end mismatch is rejected before NDT");
  corrupted = valid;
  corrupted.scan_end_ns++;
  expect(!requestTimestampsConsistent(corrupted),
         "request canonical scan_end_ns mismatch is rejected before NDT");
  corrupted = valid;
  setTimeNs(&corrupted.scan_start, 11);
  expect(!requestTimestampsConsistent(corrupted),
         "request scan_start mirror mismatch is rejected before NDT");
  std::cout << "REQUEST_TIMESTAMP_CONSISTENCY_PASS\n";
}

void testServerBindingQueueAndImmutableConfig() {
  StatusExpectation expected;
  expected.map_hash.fill(0x12);
  expected.config_hash.fill(0x34);
  auto status = makeReadyStatus(expected);
  ProtocolTestServerHarness server(status, 10);
  dog_prior_map_interfaces::NdtSessionControlAck ack;
  auto begin = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION,
      kFrontendA, 0, dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START);
  expect(server.applyControl(begin, &ack) &&
             ack.result == dog_prior_map_interfaces::NdtSessionControlAck::ACCEPTED &&
             server.sessionBound() && server.activeSession() == kFrontendA &&
             server.activeEpoch() == 0 &&
             server.status().bound_frontend_session_id == kFrontendA &&
             server.status().epoch == 0,
         "BEGIN_SESSION explicitly establishes frontend-owned binding");

  auto malformed = makeRequest(Key{kFrontendA, 0, 1});
  setTimeNs(&malformed.header.stamp, 21);
  server.enqueue(malformed);
  ServerRequestOutcome outcome;
  dog_prior_map_interfaces::NdtScanResult terminal;
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kTimestampInconsistent &&
             terminal.disposition == dog_prior_map_interfaces::NdtScanResult::ERROR_REQUEST_TIMESTAMP_INCONSISTENT &&
             server.ndtProcessCount() == 0,
         "request timestamp inconsistency is rejected before cache and NDT");

  auto wrong_protocol = makeRequest(Key{kFrontendA, 0, 1});
  ++wrong_protocol.protocol_version;
  server.enqueue(wrong_protocol);
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kProtocolMismatch &&
             server.ndtProcessCount() == 0,
         "request protocol must match the negotiated active version");

  auto foreign = makeRequest(Key{kFrontendB, 0, 1});
  server.enqueue(foreign);
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kForeignSession &&
             server.activeSession() == kFrontendA && server.ndtProcessCount() == 0,
         "foreign request cannot change binding or run NDT");
  auto future = makeRequest(Key{kFrontendA, 1, 1});
  server.enqueue(future);
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kFutureEpoch && server.ndtProcessCount() == 0,
         "unbound future epoch request is rejected before NDT");

  const Key duplicate_key{kFrontendA, 0, 1};
  auto first = makeRequest(duplicate_key);
  auto duplicate = first;
  server.enqueue(first);
  server.enqueue(duplicate);
  expect(server.workerCount() == 1 && server.queuedCount() == 2,
         "request callbacks enqueue duplicates for one serial worker");
  dog_prior_map_interfaces::NdtScanResult first_terminal;
  dog_prior_map_interfaces::NdtScanResult cached_terminal;
  server.processNext(&outcome, &first_terminal);
  expect(outcome == ServerRequestOutcome::kProcessed && server.ndtProcessCount() == 1,
         "first queued key reaches the sole worker and runs once");
  expect(server.externalHistoryValid(),
         "synthetic external previous-pose/limiter history exists after processing");
  server.processNext(&outcome, &cached_terminal);
  expect(outcome == ServerRequestOutcome::kCached && server.ndtProcessCount() == 1 &&
             exactTerminalIdentity(first_terminal, cached_terminal),
         "queued identical duplicate returns the exact cached terminal result");

  auto conflicting = makeRequest(Key{kFrontendA, 0, 2});
  auto conflicting_duplicate = conflicting;
  conflicting_duplicate.predicted_map_T_lidar.pose.position.x = 0.75;
  server.enqueue(conflicting);
  server.enqueue(conflicting_duplicate);
  server.processNext(&outcome, &first_terminal);
  expect(outcome == ServerRequestOutcome::kProcessed && server.ndtProcessCount() == 2,
         "first payload for a new queued key is processed once");
  server.processNext(&outcome, &cached_terminal);
  expect(outcome == ServerRequestOutcome::kPayloadConflict && server.ndtProcessCount() == 2,
         "queued conflicting same-key payload is rejected without second NDT call");

  server.enqueue(makeRequest(Key{kFrontendA, 0, 3}));
  auto begin_epoch = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_EPOCH, kFrontendA, 1,
      dog_prior_map_interfaces::NdtSessionControl::REASON_SENSOR_TIME_REWIND);
  expect(server.applyControl(begin_epoch, &ack) && server.activeEpoch() == 1 &&
             server.queuedCount() == 0 && !server.externalHistoryValid(),
         "BEGIN_EPOCH increments once and clears cache, external history and queue");
  server.enqueue(makeRequest(Key{kFrontendA, 0, 4}));
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kStaleEpoch && server.ndtProcessCount() == 2,
         "delayed old-epoch request after cache reset is rejected before NDT");
  server.enqueue(makeRequest(Key{kFrontendA, 2, 1}));
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kFutureEpoch && server.ndtProcessCount() == 2,
         "future unbound epoch request is rejected without automatic epoch advance");
  server.enqueue(makeRequest(Key{kFrontendA, 1, 1}));
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kProcessed && server.ndtProcessCount() == 3,
         "only explicitly bound epoch requests reach NDT");

  auto end = makeControl(status, dog_prior_map_interfaces::NdtSessionControl::END_SESSION,
                         kFrontendA, 1,
                         dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_END);
  expect(server.applyControl(end, &ack) && !server.sessionBound(),
         "END_SESSION clears binding and epoch state");
  server.enqueue(makeRequest(Key{kFrontendA, 1, 2}));
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kUnboundSession && server.ndtProcessCount() == 3,
         "unbound session request cannot reach NDT");
  auto begin_new_session = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION,
      kFrontendB, 0, dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START);
  expect(server.applyControl(begin_new_session, &ack) &&
             server.activeSession() == kFrontendB && server.activeEpoch() == 0,
         "new frontend process establishes its own fresh session");
  server.enqueue(makeRequest(Key{kFrontendA, 1, 3}));
  server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kForeignSession && server.ndtProcessCount() == 3,
         "ended frontend session cannot submit requests after replacement");

  ProtocolTestServerHarness immutable_server(status, 10);
  auto immutable_begin = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION,
      kFrontendA, 0, dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START);
  expect(immutable_server.applyControl(immutable_begin, &ack),
         "immutable-config test begins a bound session");
  auto changed_config = status;
  changed_config.canonical_ndt_config_sha256[0] ^= 1;
  expect(!immutable_server.updateStatus(changed_config) && !immutable_server.ready(),
         "effective config change in one server instance forces not-ready");
  immutable_server.enqueue(makeRequest(Key{kFrontendA, 0, 1}));
  immutable_server.processNext(&outcome, &terminal);
  expect(outcome == ServerRequestOutcome::kServerNotReady &&
             immutable_server.ndtProcessCount() == 0,
         "server cannot process transactions after immutable identity changes");

  ProtocolTestServerHarness bounded_queue(status, 10, 1);
  const bool first_queue_insert =
      bounded_queue.enqueue(makeRequest(Key{kFrontendA, 0, 1}));
  const bool overflow_rejected =
      !bounded_queue.enqueue(makeRequest(Key{kFrontendA, 0, 2}));
  auto later_cache_failure = status;
  later_cache_failure.fatal_latched = true;
  later_cache_failure.fatal_reason =
      dog_prior_map_interfaces::NdtServerStatus::FATAL_CACHE_EXHAUSTED;
  bounded_queue.updateStatus(later_cache_failure);
  reportTest("Q1", first_queue_insert && overflow_rejected &&
                        bounded_queue.fatalLatched(),
             "request queue overflow latches FATAL");
  reportTest("Q2", !bounded_queue.ready() && !bounded_queue.status().server_ready &&
                        bounded_queue.status().fatal_latched &&
                        bounded_queue.status().fatal_reason ==
                            dog_prior_map_interfaces::NdtServerStatus::FATAL_QUEUE_OVERFLOW,
             "queue overflow clears READY and publishes fatal reason");
  bounded_queue.processNext(&outcome, &terminal);
  const uint64_t overflow_count = bounded_queue.ndtProcessCount();
  reportTest("Q3", outcome == ServerRequestOutcome::kFatalLatched &&
                        !bounded_queue.enqueue(makeRequest(Key{kFrontendA, 0, 3})) &&
                        overflow_count == 0,
             "overflow rejects requests without NDT");
  const auto begin_epoch_after_overflow = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_EPOCH, kFrontendA, 1,
      dog_prior_map_interfaces::NdtSessionControl::REASON_SENSOR_TIME_REWIND);
  const bool epoch_rejected =
      !bounded_queue.applyControl(begin_epoch_after_overflow, &ack) &&
      ack.result == dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  reportTest("Q4", epoch_rejected && !bounded_queue.ready(),
             "BEGIN_EPOCH cannot recover a sticky fatal latch");
  const auto begin_session_after_overflow = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION, kFrontendB, 0,
      dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START);
  const bool session_rejected =
      !bounded_queue.applyControl(begin_session_after_overflow, &ack) &&
      ack.result == dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  const auto end_session_after_overflow = makeControl(status,
      dog_prior_map_interfaces::NdtSessionControl::END_SESSION, kFrontendA, 0,
      dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_END);
  const bool end_rejected =
      !bounded_queue.applyControl(end_session_after_overflow, &ack) &&
      ack.result == dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  dog_prior_map_interfaces::NdtScanAck ack_after_fatal;
  reportTest("Q5", session_rejected && end_rejected &&
                        !bounded_queue.acknowledge(ack_after_fatal) &&
                        !bounded_queue.ready(),
             "BEGIN_SESSION, END_SESSION, and ACK cannot clear fatal state");
  bounded_queue.processNext(&outcome, &terminal);
  reportTest("Q6", outcome == ServerRequestOutcome::kFatalLatched &&
                        bounded_queue.ndtProcessCount() == overflow_count,
             "NDT process count stays fixed after overflow");

  auto restarted_status = status;
  restarted_status.server_instance_id = kServerB;
  ProtocolTestServerHarness restarted_server(restarted_status, 10, 1);
  auto restart_begin = makeControl(restarted_status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION,
      kFrontendB, 0,
      dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START);
  reportTest("Q7", restarted_server.applyControl(restart_begin, &ack) &&
                        restarted_server.ready() &&
                        restarted_server.status().server_instance_id == kServerB &&
                        !restarted_server.status().fatal_latched,
             "only a new server instance can establish a fresh ready session");

  FrontendQueueFatalHarness frontend_scan_queue(1, 1);
  expect(frontend_scan_queue.enqueueScan() &&
             !frontend_scan_queue.enqueueScan() &&
             frontend_scan_queue.fatalLatched() &&
             frontend_scan_queue.fatalReason() == FrontendFatalReason::kQueueOverflow &&
             !frontend_scan_queue.acceptsTransactions() &&
             !frontend_scan_queue.beginEpoch() &&
             !frontend_scan_queue.beginSession(),
         "frontend scan queue overflow is sticky and blocks controls and scans");
  FrontendQueueFatalHarness frontend_result_queue(1, 1);
  expect(frontend_result_queue.enqueueResult() &&
             !frontend_result_queue.enqueueResult() &&
             frontend_result_queue.fatalLatched() &&
             frontend_result_queue.fatalReason() == FrontendFatalReason::kQueueOverflow &&
             !frontend_result_queue.ready(),
         "frontend result queue overflow enters FATAL and clears transaction ready");

  auto cache_status = status;
  cache_status.terminal_cache_max_entries = 1;
  ProtocolTestServerHarness cache_full_server(cache_status, 1);
  auto cache_begin = makeControl(cache_status,
      dog_prior_map_interfaces::NdtSessionControl::BEGIN_SESSION,
      kFrontendA, 0,
      dog_prior_map_interfaces::NdtSessionControl::REASON_FRONTEND_START);
  expect(cache_full_server.applyControl(cache_begin, &ack),
         "cache fatal test establishes initial session");
  cache_full_server.enqueue(makeRequest(Key{kFrontendA, 0, 1}));
  cache_full_server.processNext(&outcome, &terminal);
  const uint64_t count_before_cache_overflow = cache_full_server.ndtProcessCount();
  cache_full_server.enqueue(makeRequest(Key{kFrontendA, 0, 2}));
  cache_full_server.processNext(&outcome, &terminal);
  reportTest("CACHE_FATAL", outcome == ServerRequestOutcome::kCacheExhausted &&
                                terminal.disposition ==
                                    dog_prior_map_interfaces::NdtScanResult::ERROR_TERMINAL_CACHE_EXHAUSTED &&
                                !terminal.pose_valid &&
                                cache_full_server.fatalLatched() &&
                                !cache_full_server.status().server_ready &&
                                cache_full_server.status().fatal_reason ==
                                    dog_prior_map_interfaces::NdtServerStatus::FATAL_CACHE_EXHAUSTED &&
                                cache_full_server.ndtProcessCount() ==
                                    count_before_cache_overflow,
             "cache exhaustion latches server fatal before rerunning NDT");
  const bool cache_epoch_rejected =
      !cache_full_server.applyControl(begin_epoch_after_overflow, &ack) &&
      ack.result == dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  const bool cache_session_rejected =
      !cache_full_server.applyControl(begin_session_after_overflow, &ack) &&
      ack.result == dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  const bool cache_end_rejected =
      !cache_full_server.applyControl(end_session_after_overflow, &ack) &&
      ack.result == dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  reportTest("CACHE_FATAL_STICKY", cache_epoch_rejected && cache_session_rejected &&
                                         cache_end_rejected &&
                                         !cache_full_server.acknowledge(ack_after_fatal) &&
                                         !cache_full_server.enqueue(
                                             makeRequest(Key{kFrontendA, 0, 3})) &&
                                         cache_full_server.ndtProcessCount() ==
                                             count_before_cache_overflow,
             "cache fatal cannot be cleared by session/epoch controls");
  std::cout << "SERVER_BINDING_AND_SERIAL_QUEUE_PASS\n";
}

void testMessageFieldContract() {
  dog_prior_map_interfaces::NdtScanRequest request;
  dog_prior_map_interfaces::NdtScanResult result;
  dog_prior_map_interfaces::NdtScanAck ack;
  dog_prior_map_interfaces::NdtSessionControl control;
  dog_prior_map_interfaces::NdtSessionControlAck control_ack;
  dog_prior_map_interfaces::NdtServerStatus status;
  request.protocol_version = 1;
  request.frontend_session_id = kFrontendA;
  request.server_instance_id = kServerA;
  request.request_cloud_hash = 7;
  result.protocol_version = request.protocol_version;
  result.frontend_session_id = request.frontend_session_id;
  result.server_instance_id = request.server_instance_id;
  result.request_cloud_hash = request.request_cloud_hash;
  result.translation_limited = true;
  result.rotation_limited = false;
  result.step_limited = true;
  control.protocol_version = request.protocol_version;
  control.frontend_session_id = request.frontend_session_id;
  control.server_instance_id = request.server_instance_id;
  control_ack.protocol_version = request.protocol_version;
  control_ack.frontend_session_id = request.frontend_session_id;
  control_ack.server_instance_id = request.server_instance_id;
  control_ack.result =
      dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED;
  result.disposition = dog_prior_map_interfaces::NdtScanResult::ERROR_TERMINAL_CACHE_EXHAUSTED;
  ack.frontend_session_id = request.frontend_session_id;
  ack.server_instance_id = request.server_instance_id;
  status.protocol_version = request.protocol_version;
  status.server_instance_id = kServerA;
  status.bound_frontend_session_id = kFrontendA;
  status.terminal_cache_max_entries = 1001;
  status.fatal_latched = true;
  status.fatal_reason = dog_prior_map_interfaces::NdtServerStatus::FATAL_QUEUE_OVERFLOW;
  expect(result.translation_limited && !result.rotation_limited &&
             limiterFlagsConsistent(result) &&
             status.fatal_latched &&
             status.fatal_reason == dog_prior_map_interfaces::NdtServerStatus::FATAL_QUEUE_OVERFLOW &&
             control_ack.result ==
                 dog_prior_map_interfaces::NdtSessionControlAck::REJECT_FATAL_LATCHED,
         "generated ROS messages expose limiter and sticky-fatal fields used by validators");
  expect(request.protocol_version == 1 && result.protocol_version == 1 &&
             status.protocol_version == 1 && control.protocol_version == 1 &&
             control_ack.protocol_version == 1,
         "generated ROS request/result/status/control messages expose version fields");
  expect(request.frontend_session_id == result.frontend_session_id &&
             ack.frontend_session_id == control.frontend_session_id &&
             control_ack.frontend_session_id == request.frontend_session_id &&
             status.server_instance_id == request.server_instance_id,
         "generated ROS schemas use explicit, non-overloaded IDs");
  std::cout << "MESSAGE_SCHEMA_CONTRACT_PASS\n";
}

}  // namespace

int main() {
  testWaitServerAdversarial();
  testSessionEpochAndResultClassification();
  testSuccessPoseAndLimiterContracts();
  testLedgerFatalContract();
  testCacheIdempotenceAndCapacity();
  testCanonicalRequestTimestamps();
  testServerBindingQueueAndImmutableConfig();
  testMessageFieldContract();
  if (failures != 0) {
    std::cerr << "NDT_PROTOCOL_CONTRACT_FAIL count=" << failures << '\n';
    return 1;
  }
  std::cout << "TEST_A_PASS\nTEST_B_PASS\nTEST_C_PASS\nTEST_D_PASS\n"
               "TEST_E_PASS\nTEST_F_PASS\nTEST_G_PASS\nTEST_H_PASS\n"
               "TEST_I_PASS\nTEST_J_PASS\nTEST_K_PASS\nTEST_L_PASS\n";
  std::cout << "NDT_PROTOCOL_CONTRACT_PASS\n";
  return 0;
}
