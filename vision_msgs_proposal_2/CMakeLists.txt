cmake_minimum_required(VERSION 3.5)
project(vision_msgs)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/ObjectHypothesis.msg"
  "msg/ObjectHypothesis2D.msg"
  "msg/ObjectHypothesis3D.msg"
  "msg/VisionInfo.msg"
  "msg/Classification2D.msg"
  "msg/Classification3D.msg"
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/Detection3D.msg"
  "msg/Detection3DArray.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
