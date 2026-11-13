cmake_minimum_required(VERSION 3.8)
project(convoy_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  urdf
  launch
  worlds
  rviz
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes
install(PROGRAMS
  convoy_robot/path_recorder_node.py
  convoy_robot/pure_pursuit_node.py
  convoy_robot/apf_avoidance_node.py
  convoy_robot/real_leader_bridge_node.py
  convoy_robot/real_follower_bridge_node.py
  convoy_robot/real_pure_pursuit_node.py
  convoy_robot/real_apf_avoidance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
