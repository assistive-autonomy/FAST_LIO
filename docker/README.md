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
Headless mode automatically emits one timestamp-matched result for every input
LiDAR scan, including explicit bootstrap estimates during FAST-LIO startup.

Run everything with one command:

```bash
./docker/run_headless.sh
```

The first run builds `fast-lio2-headless-humble:local` if that exact image is
not available on the current Docker host. Later runs reuse the image, start a
temporary container, process the bag, and remove the container when it exits.

Rebuild intentionally after changing FAST-LIO source code or the Dockerfile:

```bash
./docker/run_headless.sh --rebuild
```

You can combine `--rebuild` with a custom workflow file. The output preserves
every original message and bag timestamp, then adds FAST-LIO `/tf`,
`/tf_static`, `/path`, and `/cloud_registered` records. The completion summary
reports bootstrap estimates separately from optimized solutions.
