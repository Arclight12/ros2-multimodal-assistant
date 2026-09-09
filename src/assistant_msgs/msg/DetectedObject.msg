# One object from the fixed workspace camera.
std_msgs/Header header
string object_id
string class_name
float32 confidence
geometry_msgs/Point image_center
geometry_msgs/Point workspace_center
geometry_msgs/Pose robot_pose
bool has_workspace_pose
bool has_robot_pose
float32 bbox_width
float32 bbox_height
