#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo >&2 "Usage: validate_bag.sh <front|rear> <humble|jazzy> <bag-path> [lidar-frame]"
  exit 2
}

[[ $# -ge 3 && $# -le 4 ]] || usage
imu_side="$1"
recording_kind="$2"
bag_path="$3"
lidar_frame="${4:-lidar_top}"

case "$imu_side" in
  front) config=top_autoware_front_imu.yaml ;;
  rear)  config=top_autoware_rear_imu.yaml ;;
  *) usage ;;
esac

stop_group() {
  local pid="${1:-}"

  [[ -n "$pid" ]] || return 0

  set +e
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null
    for _ in {1..30}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
  fi

  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in {1..30}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
  fi

  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null
  fi

  wait "$pid" 2>/dev/null
}

cleanup() {
  trap - EXIT
  stop_group "${bag_pid:-}"
  stop_group "${lio_pid:-}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch fast_lio mapping.launch.py \
  config_file:="$config" rviz:=false use_sim_time:=true \
  >/tmp/fast_lio_validation.log 2>&1 &
lio_pid=$!

sleep 3
setsid fast-lio-play-bag \
  "$recording_kind" "$bag_path" 0.25 \
  >/tmp/rosbag_validation.log 2>&1 &
bag_pid=$!

sleep 12

if grep -qi 'yaml-cpp:.*bad conversion' /tmp/rosbag_validation.log; then
  cat /tmp/rosbag_validation.log >&2
  exit 3
fi

for topic in /cloud_registered /Odometry /path; do
  timeout 30s ros2 topic echo --once "$topic" >/dev/null
  echo "Observed $topic"
done

set +e
timeout 20s ros2 topic hz /cloud_registered >/tmp/cloud_registered_hz.log 2>&1
hz_status=$?
set -e
if [[ $hz_status -ne 0 && $hz_status -ne 124 ]] || ! grep -qi 'average rate' /tmp/cloud_registered_hz.log; then
  cat /tmp/cloud_registered_hz.log >&2
  echo >&2 "/cloud_registered did not publish at a measurable nonzero rate."
  exit 4
fi

for topic in \
  /sensor/lidar/top/points \
  /sensor/lidar/front/points \
  /sensor/lidar/left/points \
  /sensor/lidar/right/points; do
  info_file="/tmp/$(echo "$topic" | tr '/' '_')_info.log"
  ros2 topic info --verbose "$topic" >"$info_file"
  if ! grep -Eqi 'Reliability:[[:space:]]*RELIABLE' "$info_file"; then
    cat "$info_file" >&2
    echo >&2 "No reliable publisher was reported for $topic."
    exit 5
  fi
  echo "Reliable publisher observed for $topic"
done

set +e
timeout 20s ros2 run tf2_ros tf2_echo map "$lidar_frame" >/tmp/tf_validation.log 2>&1
tf_status=$?
set -e
if [[ $tf_status -ne 0 && $tf_status -ne 124 ]] || \
   ! grep -Eqi 'Translation:|At time' /tmp/tf_validation.log; then
  cat /tmp/tf_validation.log >&2
  echo >&2 "Could not resolve map -> $lidar_frame."
  exit 6
fi

echo "Resolved map -> $lidar_frame"
echo "Bag validation passed for $imu_side IMU and $recording_kind recording."
