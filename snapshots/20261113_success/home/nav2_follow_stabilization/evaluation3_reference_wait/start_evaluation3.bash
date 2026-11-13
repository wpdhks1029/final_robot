#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
eval3_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
eval3_reference="${1:-}"
if test -z "$eval3_reference"; then
  if ! test -f "$eval3_dir/latest_reference_path.txt"; then
    echo '먼저 양쪽 정지·후발 비무장 상태에서 capture_reference.bash를 실행하세요.' >&2
    exit 2
  fi
  IFS= read -r eval3_reference < "$eval3_dir/latest_reference_path.txt"
fi
if ! test -f "$eval3_reference"; then
  echo "기준선 파일 없음: $eval3_reference" >&2
  exit 2
fi
exec python3 -B "$eval3_dir/candidate/follower_diagnostics/run_local_avoidance_trial.py" \
  --run --ack-clear-both-power-cut-ready --apf-pivot --gentle-pivot --map-rejoin \
  --continue-follow-after-rejoin --continuous20 --dynamic-box --post-rejoin-follow-m 2 \
  --pose-source filtered --reference-plan "$eval3_reference" \
  --tf-static-topic "${CONVOY_TF_STATIC_TOPIC:-/follower/tf_static_internal}" --seconds 0 --reference-wait
