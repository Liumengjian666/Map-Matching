std_msgs/Header header
# Must equal the version negotiated in NdtServerStatus.
uint32 protocol_version
# Created once by the frontend process; never substituted by the server.
string frontend_session_id
uint32 epoch
uint64 transaction_id
# Exact NDT server process instance bound by the frontend handshake.
string server_instance_id
time scan_start
time scan_end
uint64 scan_start_ns
uint64 scan_end_ns
string map_frame
string lidar_frame
sensor_msgs/PointCloud2 cloud_end_frame
geometry_msgs/PoseStamped predicted_map_T_lidar
uint64 request_cloud_hash
