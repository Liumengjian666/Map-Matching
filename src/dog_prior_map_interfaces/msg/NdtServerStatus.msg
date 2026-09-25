std_msgs/Header header
uint32 PROTOCOL_VERSION=1
uint8 FATAL_NONE=0
uint8 FATAL_QUEUE_OVERFLOW=1
uint8 FATAL_CACHE_EXHAUSTED=2
uint32 protocol_version
bool server_ready
bool fatal_latched
# FATAL_NONE, FATAL_QUEUE_OVERFLOW, or FATAL_CACHE_EXHAUSTED.
uint8 fatal_reason
bool external_mode_enabled
uint8[32] map_sha256
uint8[32] canonical_ndt_config_sha256
string map_frame
string lidar_frame
# Created by this NDT server process at startup.
string server_instance_id
# Empty before a frontend reset/session-start control has been accepted.
string bound_frontend_session_id
uint32 epoch
# Must exceed the run-manifest upper bound on scans in the formal input.
uint64 terminal_cache_max_entries
