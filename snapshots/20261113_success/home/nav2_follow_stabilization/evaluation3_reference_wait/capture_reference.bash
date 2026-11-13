#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
eval3_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
eval3_reference="$eval3_dir/references/reference_$(date +%Y%m%d_%H%M%S).json"
python3 -B "$eval3_dir/candidate/follower_diagnostics/capture_shared_map_reference.py" \
  --map "$HOME/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml" \
  --output "$eval3_reference"
printf '%s\n' "$eval3_reference" > "$eval3_dir/latest_reference_path.txt"
