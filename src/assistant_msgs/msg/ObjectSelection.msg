# Authoritative target after gaze/detection temporal confirmation.
std_msgs/Header header
string object_id
string source
float32 confidence
geometry_msgs/Pose target_pose
bool has_target_pose
string class_name
geometry_msgs/Point image_center
geometry_msgs/Point workspace_center
bool has_workspace_pose
