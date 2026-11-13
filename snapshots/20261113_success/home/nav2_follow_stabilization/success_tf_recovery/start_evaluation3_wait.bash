#!/usr/bin/env bash
set -e
export CONVOY_TF_STATIC_TOPIC=/tf_static
exec bash "$HOME/nav2_follow_stabilization/evaluation3_reference_wait/start_evaluation3.bash" "$@"
