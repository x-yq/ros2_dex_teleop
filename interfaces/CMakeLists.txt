cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/ArmJointStates.msg"
  "msg/HandJointStates.msg"
  "msg/HandKeypoint.msg"
  "msg/CmdArmJointAngles.msg"
  "msg/CmdHandJointAngles.msg"
  "msg/VizSensorData.msg"
  # Services
  "srv/StartDataCollection.srv"
  "srv/StopDataCollection.srv"
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()