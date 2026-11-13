#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
# Dedicated 10cm plan / 5cm stop profile with current device fault guards.
eval3_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -B "$eval3_dir/driver/follower_drive_node.py" \
  --run-hardware --ack-hardware-zero-rpm-tx \
  --input-kind local_avoidance --ack-bounded-pivot-candidate \
  --ack-continuous20-candidate --enable-motion-input \
  --trial-max-rpm 10 --trial-seconds 0
