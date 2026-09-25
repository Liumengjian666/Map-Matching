#pragma once

#include <dog_prior_map_interfaces/NdtScanRequest.h>

#include <cstdint>
#include <string>

namespace dog_prior_map_localization {
namespace detail {

using NdtRequest = dog_prior_map_interfaces::NdtScanRequest;

struct NdtRequestIdentity {
  uint32_t protocol_version = 0;
  std::string frontend_session_id;
  uint32_t epoch = 0;
  uint64_t transaction_id = 0;
  std::string server_instance_id;
  uint64_t scan_start_ns = 0;
  uint64_t scan_end_ns = 0;
  std::string map_frame;
  std::string lidar_frame;
  uint64_t request_cloud_hash = 0;
  double px = 0.0;
  double py = 0.0;
  double pz = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

inline NdtRequestIdentity makeNdtRequestIdentity(const NdtRequest& request) {
  NdtRequestIdentity identity;
  identity.protocol_version = request.protocol_version;
  identity.frontend_session_id = request.frontend_session_id;
  identity.epoch = request.epoch;
  identity.transaction_id = request.transaction_id;
  identity.server_instance_id = request.server_instance_id;
  identity.scan_start_ns = request.scan_start_ns;
  identity.scan_end_ns = request.scan_end_ns;
  identity.map_frame = request.map_frame;
  identity.lidar_frame = request.lidar_frame;
  identity.request_cloud_hash = request.request_cloud_hash;
  identity.px = request.predicted_map_T_lidar.pose.position.x;
  identity.py = request.predicted_map_T_lidar.pose.position.y;
  identity.pz = request.predicted_map_T_lidar.pose.position.z;
  identity.qx = request.predicted_map_T_lidar.pose.orientation.x;
  identity.qy = request.predicted_map_T_lidar.pose.orientation.y;
  identity.qz = request.predicted_map_T_lidar.pose.orientation.z;
  identity.qw = request.predicted_map_T_lidar.pose.orientation.w;
  return identity;
}

inline bool exactNdtRequestIdentity(const NdtRequestIdentity& lhs,
                                    const NdtRequestIdentity& rhs) {
  return lhs.protocol_version == rhs.protocol_version &&
         lhs.frontend_session_id == rhs.frontend_session_id &&
         lhs.epoch == rhs.epoch && lhs.transaction_id == rhs.transaction_id &&
         lhs.server_instance_id == rhs.server_instance_id &&
         lhs.scan_start_ns == rhs.scan_start_ns &&
         lhs.scan_end_ns == rhs.scan_end_ns && lhs.map_frame == rhs.map_frame &&
         lhs.lidar_frame == rhs.lidar_frame &&
         lhs.request_cloud_hash == rhs.request_cloud_hash && lhs.px == rhs.px &&
         lhs.py == rhs.py && lhs.pz == rhs.pz && lhs.qx == rhs.qx &&
         lhs.qy == rhs.qy && lhs.qz == rhs.qz && lhs.qw == rhs.qw;
}

}  // namespace detail
}  // namespace dog_prior_map_localization
