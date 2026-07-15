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
rviz2 -d ~/ros2_ws/install/fast_lio/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz
```

## 4. Run with rear IMU

Use the rear config in Terminal 1:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_rear_imu.yaml rviz:=false use_sim_time:=true
```


## Notes

- Always clone with `--recursive`, or run `git submodule update --init --recursive` after cloning.
- The required submodule is `include/ikd-Tree`.
- The RViz config is `rviz_cfg/fastlio_map_ros2.rviz`.
- This branch expects Autoware-style LiDAR and IMU topics.
- Full sensor-frame visualization requires the bag's `/tf` and `/tf_static`
  topics.
