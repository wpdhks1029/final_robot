#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
convoy_source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -B "$convoy_source_dir/candidate/follower_diagnostics/follower_drive_node.py" \
  --run-hardware --ack-hardware-zero-rpm-tx \
  --input-kind local_avoidance --ack-bounded-pivot-candidate \
  --enable-motion-input --disable-clearance-stop \
  --ack-extended-follow-speed --trial-max-rpm 15 --trial-seconds 0 \
  --fault-snapshot-dir "$convoy_source_dir/motor_faults"
