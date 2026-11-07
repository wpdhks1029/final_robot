cmake_minimum_required(VERSION 3.16)
project(follower_lidar_sdk_trace C CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
add_library(trace_sdk STATIC
  sdk/core/base/timer.cpp
  sdk/core/common/YdDataStream.cpp
  sdk/core/common/ydlidar_def.cpp
  sdk/core/json/cJSON.c
  sdk/core/network/ActiveSocket.cpp
  sdk/core/network/PassiveSocket.cpp
  sdk/core/network/SimpleSocket.cpp
  sdk/core/serial/serial.cpp
  sdk/core/serial/impl/unix/lock.c
  sdk/core/serial/impl/unix/list_ports_linux.cpp
  sdk/core/serial/impl/unix/unix_serial.cpp
  sdk/src/CYdLidar.cpp
  sdk/src/DTSLidarDriver.cpp
  sdk/src/ETLidarDriver.cpp
  sdk/src/GSLidarDriver.cpp
  sdk/src/SDMLidarDriver.cpp
  sdk/src/TiaLidarDriver.cpp
  sdk/src/YDlidarDriver.cpp
  sdk/src/ydlidar_sdk.cpp
  sdk/src/filters/NoiseFilter.cpp
  sdk/src/filters/StrongLightFilter.cpp
)
target_include_directories(trace_sdk PUBLIC sdk sdk/src sdk/core)
target_compile_options(trace_sdk PRIVATE -fPIC)
target_link_libraries(trace_sdk pthread rt)
add_executable(lidar_sdk_trace_node lidar_sdk_trace_node.cpp)
ament_target_dependencies(lidar_sdk_trace_node rclcpp sensor_msgs std_srvs)
target_link_libraries(lidar_sdk_trace_node trace_sdk)

add_executable(test_packet_format test_packet_format.cpp)
target_link_libraries(test_packet_format trace_sdk)
target_compile_options(test_packet_format PRIVATE -UNDEBUG)
