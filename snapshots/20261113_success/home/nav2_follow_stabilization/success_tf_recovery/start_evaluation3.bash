#!/usr/bin/env bash
set -e
# Match the success-backup public static TF used by the recovery stack.
export CONVOY_TF_STATIC_TOPIC=/tf_static
exec bash "$HOME/nav2_follow_stabilization/evaluation3_clearance10_stop05/start_evaluation3.bash" "$@"
