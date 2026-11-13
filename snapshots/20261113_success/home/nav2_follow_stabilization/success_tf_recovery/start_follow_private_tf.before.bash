#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
convoy_follow_speed="${1:-0.1}"
case "$convoy_follow_speed" in
  0.08|0.1) ;;
  *) echo 'Allowed speed: 0.08 or 0.1' >&2; exit 2 ;;
esac
convoy_source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -B "$convoy_source_dir/candidate/follower_diagnostics/run_local_avoidance_trial.py" \
  --run --ack-clear-both-power-cut-ready --nav2-leader --follow-only \
  --auto-start-on-goal --direct-unicast --exclude-body-internal-scan \
  --disable-clearance-stop --follow-max-linear "$convoy_follow_speed" \
  --ack-extended-follow-speed --smooth-follow --pose-source filtered \
  --tf-static-topic /follower/tf_static_internal --seconds 0
