# FAST-LIO2 Jazzy headless bag parser

Branch: `headless`

This ROS 2 Jazzy executable reads a bag directly and runs FAST-LIO2 offline.
It does not play the bag, use RViz, or send sensor data through DDS.

## Build

```bash
cd ~/ros2_ws/src/FAST_LIO
git checkout headless
git submodule update --init --recursive

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
colcon build --packages-select fast_lio
source install/setup.bash
```

## Parse a bag

`--input` accepts an MCAP file or a rosbag2 directory. `--output` must name a
directory that does not already exist; the parser never overwrites an output.
`--config` must be a full path.

```bash
source /opt/ros/jazzy/setup.bash
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

The output contains every original serialized bag record with its original
receive timestamp, send timestamp, payload, topic, and existing `/tf` and
`/tf_static` messages unchanged. It adds:

- FAST-LIO `map -> body` transforms on `/tf`
- the `body -> base_footprint` bridge on `/tf_static`
- the estimated trajectory on `/path`
- one final registered scan on `/cloud_registered`

Generated SLAM message headers use the corresponding LiDAR scan timestamps.
Before reporting success, the parser re-reads both bags in MCAP file order and
byte-compares every original record's topic, payload, receive timestamp, and
send timestamp.

## Check the result

```bash
ros2 bag info "$INPUT"
ros2 bag info "$OUTPUT"
```

The original topic counts in the output must match the input; only `/tf`,
`/tf_static`, `/path`, and `/cloud_registered` gain generated records.

## Play and view the output bag

The supplied RViz profile uses `map` as its fixed frame and enables the top,
front, left, and right raw LiDAR topics. Start RViz first so it is ready for
the simulated clock.

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

RVIZ_CONFIG="$(ros2 pkg prefix fast_lio)/share/fast_lio/rviz_cfg/fastlio_headless_map.rviz"
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash

OUTPUT=~/data/2026_02_25-12_32_53_dean_village-st_andrew_sq_82_fastlio
QOS="$(ros2 pkg prefix fast_lio)/share/fast_lio/config/headless_rviz_playback_qos.yaml"

ros2 bag play -s mcap "$OUTPUT" --clock --rate 0.25 \
  --qos-profile-overrides-path "$QOS"
```

The profile gives every raw LiDAR display `Best Effort`, `Keep Last`, depth
`1`, matching the player overrides. The final registered scan and the full
path appear near the end of playback; raw LiDAR frames become map-transformable
once FAST-LIO has initialized.

The player can warn that unrelated camera, radar, GPS, or vehicle topics have
missing custom message packages. Those topics are skipped; the four LiDARs,
`/tf`, `/tf_static`, `/path`, and `/cloud_registered` still play normally.
