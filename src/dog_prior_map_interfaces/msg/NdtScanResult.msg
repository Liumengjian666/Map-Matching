uint8 SUCCESS=0
uint8 REJECT_INSUFFICIENT_POINTS=1
uint8 REJECT_NOT_CONVERGED=2
uint8 REJECT_INVALID_SOURCE=3
uint8 ERROR_NDT_INTERNAL=4
uint8 ERROR_PROTOCOL=5
uint8 ERROR_TERMINAL_CACHE_EXHAUSTED=6
uint8 ERROR_REQUEST_TIMESTAMP_INCONSISTENT=7

std_msgs/Header header
# Echo of the negotiated request protocol version.
uint32 protocol_version
# Echo of the frontend-owned transaction identity.
string frontend_session_id
uint32 epoch
uint64 transaction_id
# Echo of the server process that handled the request.
string server_instance_id
time scan_start
time scan_end
uint64 scan_start_ns
uint64 scan_end_ns
uint8 disposition
string reason
bool pose_valid
string map_frame
string lidar_frame
geometry_msgs/PoseStamped raw_map_T_lidar
geometry_msgs/PoseStamped used_map_T_lidar
float64 fitness
uint32 iterations
bool converged
uint64 request_cloud_hash
uint64 ndt_source_cloud_hash
# Separate limiter outcomes are part of terminal-result identity.
bool translation_limited
bool rotation_limited
# Derived diagnostic: exactly translation_limited || rotation_limited.
bool step_limited
