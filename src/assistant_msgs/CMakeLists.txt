cmake_minimum_required(VERSION 3.8)
project(assistant_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- Messages ---
set(msg_files
  "msg/GazePoint.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/ObjectSelection.msg"
)

# --- Services ---
set(srv_files
  "srv/CalibrateWorkspace.srv"
  "srv/ScanWorkspace.srv"
)

# --- Actions ---
set(action_files
  "action/ExecuteGrasp.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
