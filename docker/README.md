# Headless Docker

Requires Docker Engine with the Compose plugin.

```bash
git clone --recursive -b headless https://github.com/assistive-autonomy/FAST_LIO.git
cd FAST_LIO
```

Edit `docker/headless.yaml` with absolute host paths:

```yaml
input_bag: /home/user/data/input.mcap
output_bag: /home/user/data/input_fastlio
config_file: top_autoware_front_imu.yaml
```

Use `top_autoware_rear_imu.yaml` for the rear IMU. The input may be an MCAP
file or rosbag2 directory; the output directory must not already exist.

Run everything with one command:

```bash
./docker/run_headless.sh
```

The first run builds the image. The output preserves every original message
and bag timestamp, then adds FAST-LIO `/tf`, `/tf_static`, `/path`, and
`/cloud_registered` records.
