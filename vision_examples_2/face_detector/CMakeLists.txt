cmake_minimum_required(VERSION 3.5)

project(face_detector)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(intra_process_demo REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${intra_process_demo_INCLUDE_DIRS}
)
message(STATUS "foobar")
message(STATUS ${intra_process_demo_INCLUDE_DIRS})

add_executable(face_detector_node
  src/face_detector_node.cpp)
ament_target_dependencies(face_detector_node
  "rclcpp"
  "sensor_msgs"
  "vision_msgs"
  "intra_process_demo"
  "OpenCV")

add_executable(face_detector_pipeline
  src/face_detector_pipeline.cpp)
ament_target_dependencies(face_detector_pipeline
  "rclcpp"
  "sensor_msgs"
  "vision_msgs"
  "intra_process_demo"
  "OpenCV")

install(TARGETS
  face_detector_pipeline
  face_detector_node
  DESTINATION bin)

ament_package()
