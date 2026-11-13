#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
success_tf_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$success_tf_dir/follower_success_tf_stack.launch.py" \
  map:="${1:-$HOME/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml}" \
  ekf_frequency:=20.0
