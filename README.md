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
