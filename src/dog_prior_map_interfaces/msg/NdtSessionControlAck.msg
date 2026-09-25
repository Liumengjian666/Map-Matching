std_msgs/Header header
uint8 ACCEPTED=0
uint8 REJECT_PROTOCOL_VERSION=1
uint8 REJECT_SERVER_INSTANCE=2
uint8 REJECT_MAP_IDENTITY=3
uint8 REJECT_CONFIG_IDENTITY=4
uint8 REJECT_INVALID_TRANSITION=5
uint8 REJECT_SESSION_OR_EPOCH=6
uint8 REJECT_FATAL_LATCHED=7
uint32 PROTOCOL_VERSION=1

uint8 command
uint8 result
uint32 protocol_version
string reason
string server_instance_id
string frontend_session_id
uint32 epoch
