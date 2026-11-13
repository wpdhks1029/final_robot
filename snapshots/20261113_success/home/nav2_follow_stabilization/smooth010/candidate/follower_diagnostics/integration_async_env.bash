# Source in each integration driver/sensor terminal BEFORE starting ROS nodes.
# Process-local setting; successful baseline scripts and YAML are unchanged.
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
