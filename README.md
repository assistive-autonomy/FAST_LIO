# FAST_LIO ROS2 AD Branch

Custom ROS 2 Humble FAST-LIO branch for Autoware-style topics using the top LiDAR and front/rear IMU configs.

Branch: `fast_lio_ros2_AD`  
Repo: `https://github.com/assistive-autonomy/FAST_LIO.git`

## 1. Clone

```bash
cd ~/fastlio2_ws/src
git clone --recursive -b fast_lio_ros2_AD https://github.com/assistive-autonomy/FAST_LIO.git
```

If the repo was already cloned without submodules:

```bash
cd ~/fastlio2_ws/src/FAST_LIO
git checkout fast_lio_ros2_AD
git submodule update --init --recursive
```

Check the submodule:

```bash
ls include/ikd-Tree/ikd_Tree.cpp
```

## 2. Build

```bash
cd ~/fastlio2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
colcon build
source install/setup.bash
```

## 3. Run with front IMU

Terminal 1 — FAST-LIO:

```bash
cd ~/fastlio2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_front_imu.yaml rviz:=false use_sim_time:=true
```

Terminal 2 — rosbag:

```bash
source /opt/ros/humble/setup.bash
BAG=~/odometry_test/2026_02_25-12_32_53_dean_village-st_andrew_sq_82_top_only
ros2 bag play "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/odometry_test/fastlio_playback_qos.yaml \
  --topics /sensor/lidar/top/points /sensor/imu/front/data /tf /tf_static
```

Terminal 3 — RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/fastlio2_ws/install/setup.bash
rviz2 -d ~/fastlio2_ws/install/fast_lio/share/fast_lio/rviz_cfg/fastlio_map_ros2.rviz
```

## 4. Run with rear IMU

Use the rear config in Terminal 1:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=top_autoware_rear_imu.yaml rviz:=false use_sim_time:=true
```

Use the rear IMU topic in Terminal 2:

```bash
ros2 bag play "$BAG" --clock --rate 0.25 \
  --qos-profile-overrides-path ~/odometry_test/fastlio_playback_qos.yaml \
  --topics /sensor/lidar/top/points /sensor/imu/rear/data /tf /tf_static
```

## 5. Validation

```bash
ros2 topic hz /sensor/lidar/top/points --wall-time
ros2 topic hz /sensor/imu/front/data --wall-time
ros2 topic hz /cloud_registered --wall-time
ros2 topic echo --once /Odometry --field header
ros2 topic info /cloud_registered --verbose
```

For rear IMU, replace:

```bash
/sensor/imu/front/data
```

with:

```bash
/sensor/imu/rear/data
```

## Notes

- Always clone with `--recursive`, or run `git submodule update --init --recursive` after cloning.
- The required submodule is `include/ikd-Tree`.
- The RViz config is `rviz_cfg/fastlio_map_ros2.rviz`.
- This branch expects Autoware-style LiDAR and IMU topics.
