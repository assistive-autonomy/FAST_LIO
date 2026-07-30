# FAST_LIO ROS2 AD Branch

Custom ROS 2 Humble FAST-LIO branch for Autoware-style topics using the top LiDAR and front/rear IMU configs.

Branch: `fast_lio_ros2_AD`  
Repo: `https://github.com/assistive-autonomy/FAST_LIO.git`

## 1. Clone

```bash
cd ~/ros2_ws/src
git clone --recursive -b fast_lio_ros2_AD https://github.com/assistive-autonomy/FAST_LIO.git
```

If the repo was already cloned without submodules:

```bash
cd ~/ros2_ws/src/FAST_LIO
git checkout fast_lio_ros2_AD
git submodule update --init --recursive
```

Check the submodule:

```bash
ls include/ikd-Tree/ikd_Tree.cpp
```

## 2. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
colcon build
source install/setup.bash
```

## 3. Run with front IMU

Terminal 1 — FAST-LIO:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_front_imu.yaml rviz:=false use_sim_time:=true
```

Terminal 2 — Humble-recorded full-bag playback:

```bash
source /opt/ros/humble/setup.bash
BAG=~/path-to-the-bag
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/fastlio_playback_qos.yaml
```

The command replays every topic in the bag. The QoS override is limited to the
four LiDAR streams and the front IMU input; all remaining topics use their
recorded QoS profiles. Message packages for any recorded custom types must be
available in the sourced ROS environment.

### Jazzy-recorded MCAP bags on Humble

Jazzy records the offered QoS profiles using string-valued policies, which the
Humble rosbag2 player cannot decode. For an MCAP bag recorded with Jazzy, use
the full-topic compatibility override instead:

```bash
source /opt/ros/humble/setup.bash
BAG=~/path-to-the-jazzy-recorded-bag.mcap
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/jazzy_mcap_fastlio_qos.yaml
```

The Jazzy compatibility file covers the complete 105-topic Autoware bag
layout and makes all four LiDAR publishers reliable. Humble requires an
override entry for every topic in a Jazzy-recorded bag; if a bag contains an
additional topic, add that topic to the compatibility file before playback.
Warnings about ignored custom message types indicate that their message
packages are not installed and are separate from QoS compatibility.

The Autoware configurations connect FAST-LIO's `map -> body` estimate to the
sensor tree rooted at `base_footprint` using the IMU calibration from
`/tf_static`. Once FAST-LIO initializes, stamped LiDAR, camera, IMU, radar, and
other sensor topics in that tree can be displayed in RViz with `map` as the
fixed frame.

Terminal 3 — RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
RVIZ_CONFIG="$(ros2 pkg prefix fast_lio)/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz"
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
```

### Optional cumulative SLAM map

RViz now treats `/cloud_registered` as the latest registered scan only. To
also build and display a cumulative point-cloud map for the complete run,
use the following three-terminal workflow. RViz is launched separately so it
uses the bag's simulated clock. The large-data transport setting must be
exported in every terminal before starting any ROS process.

Terminal 1 — FAST-LIO with cumulative `/slam` output:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 launch fast_lio mapping.launch.py \
  config_file:=top_autoware_front_imu.yaml \
  rviz:=false \
  use_sim_time:=true \
  slam:=true
```

Terminal 2 — RViz using simulated time:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

RVIZ_CONFIG="$(ros2 pkg prefix fast_lio)/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz"
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
```

Terminal 3 — input-bag playback:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

BAG=~/path-to-the-bag
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/fastlio_playback_qos.yaml
```

Start a fresh set of all three processes before replaying a bag. This resets
the TF buffers, simulated clock, and cumulative map. Warnings about ignored
topics only mean that optional custom-message packages for those unrelated
bag topics are not installed.

When `slam:=false` (the default), FAST-LIO does not create the `/slam`
publisher or accumulate map points. When enabled, every successfully
registered scan, plus the initial map-seed scan, contributes to a world-frame
voxel map. The node publishes cumulative `/slam` snapshots with reliable,
transient-local QoS, so RViz can join late and immediately receive the newest
cumulative snapshot.

The Autoware YAML files provide two tuning parameters:

```yaml
publish:
  slam_voxel_size: 0.5       # metres; smaller values retain more detail and use more RAM
  slam_publish_period: 5.0   # seconds of LiDAR time between live preview snapshots
```

The voxel map preserves the whole driven area without retaining billions of
duplicate raw points. The preview rate follows LiDAR time, so changing the bag
playback rate does not multiply the number of full-map messages. When input
stops, the node publishes any remaining changes after approximately one
wall-clock second; leave FAST-LIO running for that final update.

## 4. Run with rear IMU

Use the rear config in Terminal 1:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_rear_imu.yaml rviz:=false use_sim_time:=true
```

Add `slam:=true` to this command when the cumulative `/slam` map is required.

## 5. Docker

A prebuilt ROS 2 Humble environment is provided for FAST-LIO2, Livox-SDK2,
`livox_ros_driver2`, MCAP playback, and RViz. It supports running FAST-LIO,
rosbag2, and RViz in three shells attached to one container.

### Build the image

Run these commands from the repository root:

```bash
cd ~/ros2_ws/src/FAST_LIO
git submodule update --init --recursive
mkdir -p bag data

# Example: copy your bag into the repository's bag/ directory.
cp /absolute/path/to/bag1.mcap bag/bag1.mcap

export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

docker compose build workspace
```

The repository's `bag/` directory is always mounted read-only at `/bag` in the
container. Container shells start in `/bag`, so use only the MCAP file name in
`BAG`; for the example above, use `BAG=bag1.mcap`.

### Start the container

```bash
docker compose up -d workspace
docker compose ps
```

Open three host terminals. In each terminal, run:

```bash
cd ~/ros2_ws/src/FAST_LIO
docker compose exec workspace bash
```

ROS 2 Humble, the Livox workspace, and FAST-LIO are sourced automatically in
every container shell.

Terminal 1 — FAST-LIO:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_front_imu.yaml rviz:=false use_sim_time:=true
```

Terminal 2 — Humble-recorded bag:

```bash
BAG=bag1.mcap
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/fastlio_playback_qos.yaml
```

For a Jazzy-recorded MCAP bag, use:

```bash
BAG=bag1.mcap
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/jazzy_mcap_fastlio_qos.yaml
```

Terminal 3 — RViz:

First, run this on the host:

```bash
xhost +SI:localuser:"$(id -un)"
```

Then run RViz in the third container shell:

```bash
rviz2 -d ~/ros2_ws/install/fast_lio/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz
```

### Stop the container

```bash
docker compose down
xhost -SI:localuser:"$(id -un)"
```

See [docker/README.md](docker/README.md) for validation, troubleshooting,
output persistence, live Livox hardware, and custom-message details.


## Notes

- Always clone with `--recursive`, or run `git submodule update --init --recursive` after cloning.
- The required submodule is `include/ikd-Tree`.
- The RViz config is `rviz_cfg/fastlio_map_ros2.rviz`.
- This branch expects Autoware-style LiDAR and IMU topics.
- Full sensor-frame visualization requires the bag's `/tf` and `/tf_static`
  topics.
