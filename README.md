# FAST-LIO2 Humble headless bag parser

Branch: `headless`

This ROS 2 Humble executable reads an MCAP bag directly and runs FAST-LIO2 offline.
It does not play the bag, use RViz, or send sensor data through DDS.

## Build

```bash
cd ~/ros2_ws/src/FAST_LIO
git checkout headless
git submodule update --init --recursive

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
colcon build --packages-select fast_lio
source install/setup.bash
```

## Parse a bag

`--input` accepts an MCAP file or a rosbag2 directory. `--output` must name a
directory that does not already exist; the parser never overwrites an output.
`--config` must be a full path.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

INPUT=~/data/2026_02_25-12_32_53_dean_village-st_andrew_sq_82/2026_02_25-12_32_53_dean_village-st_andrew_sq_82.mcap
OUTPUT=~/data/2026_02_25-12_32_53_dean_village-st_andrew_sq_82_fastlio
CONFIG="$(ros2 pkg prefix fast_lio)/share/fast_lio/config/top_autoware_front_imu.yaml"

ros2 run fast_lio fastlio_headless \
  --input "$INPUT" \
  --output "$OUTPUT" \
  --config "$CONFIG"
```

Headless mode automatically writes one result for every input LiDAR scan. Scans
used by FAST-LIO to initialize its time base, IMU state, and map are represented
by bootstrap estimates; subsequent frames are normal optimized solutions. No
warm-up bag or additional runtime option is required.

The output contains every original serialized bag record with its original
payload, topic, single Humble rosbag timestamp, and existing `/tf` and
`/tf_static` messages unchanged. It adds:

- FAST-LIO `map -> body` transforms on `/tf`
- the `body -> base_footprint` bridge on `/tf_static`
- the estimated trajectory on `/path`
- one registered scan per input LiDAR frame on `/cloud_registered`

Generated SLAM message headers use the corresponding LiDAR scan timestamps.
Bootstrap frames without complete IMU coverage use the preprocessed raw cloud;
the completion summary reports bootstrap and optimized counts separately.
Before reporting success, the parser re-reads both bags in ROS 2 storage order
and byte-compares every original record's topic, payload, and rosbag timestamp.

The parser feeds deserialized messages directly into FAST-LIO, so DDS QoS does
not affect transformation. The normal `fastlio_mapping` executable keeps the
Humble branch's QoS: `SensorDataQoS` for standard point clouds, the default
reliable depth-10 IMU subscription, and the existing queue depths for Livox and
published point clouds.

## Check the result

```bash
ros2 bag info "$INPUT"
ros2 bag info "$OUTPUT"
```

The original topic counts in the output must match the input; only `/tf`,
`/tf_static`, `/path`, and `/cloud_registered` gain generated records.

## Play and view the output bag

The headless parser has already run FAST-LIO, so playback does not need to be
slowed down to let a mapping node catch up. The default RViz profile uses
`lidar_top` as its fixed frame and enables the top, front, left, and right raw
LiDAR topics. This vehicle-fixed view displays raw scans immediately without
waiting for the time-varying `map -> body` transform. Start RViz first so it is
ready for the simulated clock.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

RVIZ_CONFIG="$(ros2 pkg prefix fast_lio)/share/fast_lio/rviz_cfg/fastlio_headless_sensor.rviz"
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

OUTPUT=~/data/2026_02_25-12_32_53_dean_village-st_andrew_sq_82_fastlio
QOS="$(ros2 pkg prefix fast_lio)/share/fast_lio/config/headless_rviz_playback_qos.yaml"

ros2 bag play -s mcap "$OUTPUT" --clock --rate 1.0 \
  --qos-profile-overrides-path "$QOS" \
  --topics \
    /sensor/lidar/top/points \
    /sensor/lidar/front/points \
    /sensor/lidar/left/points \
    /sensor/lidar/right/points \
    /tf /tf_static /path /cloud_registered
```

Filtering the playback prevents unrelated high-bandwidth camera and radar data
from competing with RViz. The player overrides raw LiDAR streams to Humble's
reliable, volatile, `Keep Last(10)` convention from
`config/fastlio_playback_qos.yaml`; RViz requests `Best Effort`, `Keep Last(1)`
for responsive visual display, which is compatible with a reliable publisher.
The generated `/cloud_registered` stream remains best effort with depth 1. A
10 Hz LiDAR therefore updates at 10 Hz wall time; using `--rate 0.25`
intentionally reduces that to 2.5 Hz.

For a world-fixed SLAM view, load `fastlio_headless_map.rviz` instead. That
profile displays the smaller, de-skewed `/cloud_registered` stream in `map` and
leaves the four full-density raw scans disabled by default. The full cumulative
path appears near the end of playback by design. Raw scans enabled manually in
the map profile can briefly wait for stamped TF during initialization or at the
newest edge of the trajectory.

Unfiltered playback can warn that unrelated camera, radar, GPS, or vehicle
topics have missing custom message packages. Those topics are irrelevant to
this visualization and are excluded by the command above.
