# FAST-LIO2 Humble container

This image contains a built ROS 2 Humble environment with FAST-LIO2,
Livox-SDK2, `livox_ros_driver2`, MCAP playback support, the bag-specific QoS
files, and RViz. No build or ROS setup command is required after the container
starts.

The supported workflow uses one persistent container and three interactive
shells. Linux host networking allows ROS 2 discovery and Livox UDP traffic,
while the repository's `bag/` directory is mounted read-only at `/bag`.

## 1. Prepare the repository

Clone the branch recursively, or initialize the submodule in an existing
checkout:

```bash
git clone --recursive -b fast_lio_ros2_AD \
  https://github.com/assistive-autonomy/FAST_LIO.git
cd FAST_LIO
```

For an existing checkout:

```bash
git submodule update --init --recursive
test -f include/ikd-Tree/ikd_Tree.cpp
```

Docker Engine with the Compose plugin is required. The primary supported host
is Linux with an X11 or XWayland display for RViz.

## 2. Put the MCAP file in `bag/`

From the repository root, create the `bag/` and `data/` directories:

```bash
mkdir -p bag data
```

Copy the MCAP file into `bag/`. For example, if the file is currently at
`/absolute/path/to/bag1.mcap`:

```bash
cp /absolute/path/to/bag1.mcap bag/bag1.mcap
ls -lh bag/bag1.mcap
```

The fixed Docker mapping is:

| Host path | Container path | Access |
| --- | --- | --- |
| `./bag` | `/bag` | read-only |
| `./data` | `/data` | read/write |

The container shell starts in `/bag`, so playback commands use only the file
name. For the example above, set `BAG=bag1.mcap`—not a host path and not
`/bag/bag1.mcap`.

Set the image user IDs before building so files written to `data/` belong to
the current host user:

```bash
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"
```

The user IDs and other image/runtime settings can instead be kept in a local
`.env` file by copying `.env.example` and editing it. `.env`, `bag/`, and
`data/` are ignored by Git.

## 3. Build and start

Run these commands from the repository root:

```bash
docker compose build workspace
docker compose up -d workspace
docker compose ps
```

The image is built as `fast-lio2-humble:local` by default. FAST-LIO2 and the
Livox workspace are already compiled in `/opt`. The container also provides
these compatibility links:

```text
~/ros2_ws  -> /opt/fast_lio_ws
~/ws_livox -> /opt/ws_livox
```

Every new Bash shell automatically sources ROS 2 Humble, the Livox overlay,
and the FAST-LIO overlay in that order.

## 4. Run FAST-LIO2, the bag, and RViz

Open three host terminals. In each terminal, enter the same running container:

```bash
docker compose exec workspace bash
```

No `source .../setup.bash` command is needed.

### Terminal 1: FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_front_imu.yaml rviz:=false use_sim_time:=true
```

For the rear IMU, use
`config_file:=top_autoware_rear_imu.yaml` instead.

### Terminal 2: full-bag playback

For a Humble-recorded bag:

```bash
BAG=bag1.mcap
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/fastlio_playback_qos.yaml
```

For a Jazzy-recorded MCAP bag played by Humble:

```bash
BAG=bag1.mcap
ros2 bag play -s mcap "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/ros2_ws/src/FAST_LIO/config/jazzy_mcap_fastlio_qos.yaml
```

Neither command uses `--topics`, so rosbag2 attempts to replay the complete
recording. The Jazzy compatibility file avoids Humble's
`yaml-cpp: bad conversion` error and contains an override for every topic in
the recorded 105-topic layout used to create it.

The optional helper selects the same QoS files:

```bash
fast-lio-play-bag humble bag1.mcap 0.25
fast-lio-play-bag jazzy bag1.mcap 0.25
```

### Terminal 3: RViz

Before starting RViz, allow the UID-matched container process to use the host
display. Run this once on the host, outside the container:

```bash
xhost +SI:localuser:"$(id -un)"
```

Then run RViz inside Terminal 3:

```bash
rviz2 -d ~/ros2_ws/install/fast_lio/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz
```

The supplied RViz configuration uses `map` as its fixed frame. Software OpenGL
rendering is enabled by default, so a GPU is not required.

After RViz and the container are stopped, revoke the temporary display access
on the host:

```bash
xhost -SI:localuser:"$(id -un)"
```

## 5. Output files

FAST-LIO's compile-time output locations are connected to the writable mount:

```text
~/ros2_ws/src/FAST_LIO/Log -> /data/Log
~/ros2_ws/src/FAST_LIO/PCD -> /data/PCD
```

Files created there persist in the repository's `data/` directory. Map/PCD
saving remains controlled by the selected FAST-LIO configuration.

## 6. Validation

Run repository checks before building:

```bash
docker/tests/static_checks.sh
```

After starting the container, verify the prebuilt packages and fresh-shell
environment:

```bash
docker compose exec workspace bash -ic \
  'ros2 pkg prefix fast_lio && ros2 pkg prefix livox_ros_driver2 && ros2 pkg prefix rosbag2_storage_mcap'

docker compose exec workspace bash -ic \
  '~/ros2_ws/src/FAST_LIO/docker/tests/container_smoke.sh'
```

The smoke test checks both FAST-LIO configurations, the Livox custom message,
MCAP support, the SDK library, the Livox driver library, the requested QoS
paths, and the installed RViz configuration.

To validate mapping against a mounted bag from inside an interactive container
shell:

```bash
~/ros2_ws/src/FAST_LIO/docker/tests/validate_bag.sh \
  front humble bag1.mcap lidar_top
```

Use `rear` for the rear IMU configuration or `jazzy` for a Jazzy-recorded bag.
The validation observes `/cloud_registered`, `/Odometry`, `/path`, all four
LiDAR publishers, and the `map -> lidar_top` transform.

## 7. Livox hardware

Livox-SDK2 and `livox_ros_driver2` are installed even when only standard
`sensor_msgs/PointCloud2` bags are used. Live Ethernet sensors use host
networking and do not require a privileged container. Before launching a live
sensor, configure the host network interface, firewall, sensor IP, host IP,
and the driver's JSON configuration for that machine. Store editable hardware
configuration under `/data` rather than rebuilding the image for each IP
address.

## 8. Recorded custom message types

The image installs the Humble packages needed by the inspected Autoware bags,
including `dbw_ford_msgs`, `flir_camera_msgs`, `gps_msgs`,
`microstrain_inertial_msgs`, and `novatel_gps_msgs`.

`pdk_msgs` is not available from the Humble apt repositories and no public
source is included in this repository. Rosbag2 will report and skip a recorded
`pdk_msgs` topic unless that message package is added to the image. This does
not affect FAST-LIO's LiDAR, IMU, TF, odometry, map, or registered-cloud data.

## 9. Stop or reconfigure

Stop the persistent container:

```bash
docker compose down
```

Rebuild after changing source code or Docker dependencies:

```bash
docker compose build --no-cache workspace
docker compose up -d --force-recreate workspace
```

Common checks:

```bash
docker compose exec workspace bash -ic 'echo "$AMENT_PREFIX_PATH"'
docker compose exec workspace bash -ic 'ldconfig -p | grep livox'
docker compose logs workspace
```

If RViz cannot open the display, confirm that `DISPLAY` is correct,
`/tmp/.X11-unix` exists on the host, the image was built with the host UID/GID,
and the `xhost` command above succeeded. ROS 2 discovery requires the same
`ROS_DOMAIN_ID` for every participant.
