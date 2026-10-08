# ProcessedTSList.msg
# Batch of processed target ships from TS State Manager.
# Published at 10Hz on /processed_ts_list.

std_msgs/Header header
# Identifies one coherent OS/TS calculation. Invalid input publishes valid=false,
# never a valid empty/no-threat snapshot. UUID changes on every publication.
unique_identifier_msgs/UUID snapshot_id
bool valid
string status_message
geometry_msgs/Pose os_pose
geometry_msgs/Twist os_twist
float64 os_radius
ProcessedTS[] ships
