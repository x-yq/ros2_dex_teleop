cmake_minimum_required(VERSION 3.8)
project(component_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/input_control/RawHandKeypoint.msg"
  "msg/input_control/ProcessedHandKeypoint.msg"
  "msg/input_control/HandImage.msg"
  "msg/feedback/SceneImage.msg"
  "msg/feedback/RobotState.msg"
  "msg/feedback/Hand2DPlotData.msg"
  "msg/cmd_robot_space/JointAngles.msg"
  "msg/dynamic_transform/DynamicTransform.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()