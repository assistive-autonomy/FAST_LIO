#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
Usage:
  run_bag.sh humble <bag-path> [rate]
  run_bag.sh jazzy  <bag-path> [rate]

Examples:
  run_bag.sh humble bag1.mcap 0.25
  run_bag.sh jazzy  bag1.mcap 0.25
USAGE
  exit 2
}

[[ $# -ge 2 && $# -le 3 ]] || usage

recording_kind="$1"
bag_path="$2"
rate="${3:-0.25}"
repo_root="${FAST_LIO_REPO:-${HOME}/ros2_ws/src/FAST_LIO}"

[[ -e "$bag_path" ]] || {
  echo >&2 "Bag path does not exist: $bag_path"
  exit 3
}

case "$recording_kind" in
  humble)
    qos_file="$repo_root/config/fastlio_playback_qos.yaml"
    ;;
  jazzy)
    qos_file="$repo_root/config/jazzy_mcap_fastlio_qos.yaml"
    ;;
  *)
    echo >&2 "Recording kind must be 'humble' or 'jazzy'."
    usage
    ;;
esac

[[ -f "$qos_file" ]] || {
  echo >&2 "QoS override file not found: $qos_file"
  exit 4
}

exec ros2 bag play -s mcap "$bag_path" \
  --clock \
  --rate "$rate" \
  --qos-profile-overrides-path "$qos_file"
