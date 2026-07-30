cmake_minimum_required(VERSION 3.8)
project(aemot_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)

add_executable(aeb_node
  src/main_node.cpp
  src/kalman.cpp
  src/TrackManager.cpp
  src/SAEdetector.cpp
  src/decoding.cpp
  src/parameters.cpp
)

ament_target_dependencies(aeb_node
  rclcpp
  std_msgs
  event_camera_msgs
  event_camera_codecs
  ament_index_cpp
)

target_include_directories(aeb_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(aeb_node
  ${OpenCV_LIBS}
  yaml-cpp
  Eigen3::Eigen
)

install(TARGETS aeb_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs
)

ament_package()