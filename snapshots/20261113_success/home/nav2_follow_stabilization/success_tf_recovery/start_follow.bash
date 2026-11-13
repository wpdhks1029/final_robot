#!/usr/bin/env bash
set -e
export CONVOY_TF_STATIC_TOPIC=/tf_static
exec bash "$HOME/nav2_follow_stabilization/smooth010/start_follow.bash" "${1:-0.1}"
