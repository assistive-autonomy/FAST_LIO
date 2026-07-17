#!/usr/bin/env bash
set -euo pipefail

mapping_bin=/opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping
driver_lib=/opt/ws_livox/install/livox_ros_driver2/lib/liblivox_ros_driver2.so
driver_node=/opt/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node

ros2 pkg prefix fast_lio
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix rosbag2_storage_mcap
ros2 interface show livox_ros_driver2/msg/CustomMsg >/dev/null
ldconfig -p | grep -E 'livox(_lidar)?_sdk'

[[ -L "${HOME}/ros2_ws" ]] || {
  echo >&2 "The compatibility workspace link is missing: ${HOME}/ros2_ws"
  exit 1
}

[[ -f "${HOME}/ros2_ws/src/FAST_LIO/config/fastlio_playback_qos.yaml" ]]
[[ -f "${HOME}/ros2_ws/src/FAST_LIO/config/jazzy_mcap_fastlio_qos.yaml" ]]
[[ -f "${HOME}/ros2_ws/install/fast_lio/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz" ]]

[[ -x "$mapping_bin" ]] || {
  echo >&2 "FAST_LIO executable not found: $mapping_bin"
  exit 1
}

if ldd "$mapping_bin" | grep -q 'not found'; then
  echo >&2 "FAST_LIO has missing shared-library dependencies:"
  ldd "$mapping_bin" | grep 'not found' >&2
  exit 2
fi

[[ -f "$driver_lib" ]] || {
  echo >&2 "Livox driver library not found: $driver_lib"
  exit 2
}

if ldd "$driver_lib" | grep -q 'not found'; then
  echo >&2 "Livox driver has missing shared-library dependencies:"
  ldd "$driver_lib" | grep 'not found' >&2
  exit 2
fi

[[ -x "$driver_node" ]] || {
  echo >&2 "Livox driver executable not found: $driver_node"
  exit 2
}

if ldd "$driver_node" | grep -q 'not found'; then
  echo >&2 "Livox driver executable has missing shared-library dependencies:"
  ldd "$driver_node" | grep 'not found' >&2
  exit 2
fi

for config in top_autoware_front_imu.yaml top_autoware_rear_imu.yaml; do
  log_file="/tmp/${config%.yaml}.log"
  set +e
  timeout --signal=INT 12s ros2 launch fast_lio mapping.launch.py \
    config_file:="$config" rviz:=false use_sim_time:=true \
    >"$log_file" 2>&1
  status=$?
  set -e

  # timeout(1) returns 124 when the launch stayed alive for the smoke-test window.
  if [[ $status -ne 0 && $status -ne 124 && $status -ne 130 ]]; then
    cat "$log_file" >&2
    echo >&2 "Launch smoke test failed for $config (status $status)."
    exit 3
  fi

done

echo "Container smoke checks passed."
