#!/usr/bin/env bash
set -euo pipefail

headless_bin=/opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_headless
driver_lib=/opt/ws_livox/install/livox_ros_driver2/lib/liblivox_ros_driver2.so

ros2 pkg prefix fast_lio
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix rosbag2_storage_mcap
ros2 interface show livox_ros_driver2/msg/CustomMsg >/dev/null

[[ -x "$headless_bin" ]]
[[ -f "$driver_lib" ]]
[[ -f /opt/fast_lio_ws/install/fast_lio/share/fast_lio/config/top_autoware_front_imu.yaml ]]
[[ -f /opt/fast_lio_ws/install/fast_lio/share/fast_lio/config/top_autoware_rear_imu.yaml ]]

if ldd "$headless_bin" | grep -q 'not found'; then
  echo >&2 "fastlio_headless has missing shared-library dependencies:"
  ldd "$headless_bin" | grep 'not found' >&2
  exit 1
fi

if ldd "$driver_lib" | grep -q 'not found'; then
  echo >&2 "Livox driver has missing shared-library dependencies:"
  ldd "$driver_lib" | grep 'not found' >&2
  exit 2
fi

"$headless_bin" --help | grep -q -- '--input <bag.mcap|bag_directory>'

echo "Headless container smoke checks passed."
