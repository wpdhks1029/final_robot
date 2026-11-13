#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
convoy_launch_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$convoy_launch_dir/leader_nav2_convoy.launch.py" \
  params_file:="$HOME/leader_diagnostics/nav2_015_rotation_shim_check.yaml" \
  use_sim_time:=false use_composition:=False convoy_max_linear:=0.1 \
  convoy_motor_status_topic:=/follower/motor_candidate/status_unicast \
  convoy_follow_status_topic:=/follower/local_avoidance/status_unicast \
  convoy_follower_pose_topic:=/follower/odom_map_unicast
