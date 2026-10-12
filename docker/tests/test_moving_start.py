#!/usr/bin/env python3
"""Exercise headless moving-start initialization with tiny synthetic MCAPs.

Run inside a sourced FAST_LIO headless image:
  python3 docker/tests/test_moving_start.py

Uses the installed, unchanged top_autoware_front_imu.yaml. Four tiny scans are
necessary: three bootstrap frames followed by one optimized frame. Assertions
cover initialization conventions and exact record preservation, not SLAM
accuracy on this deliberately artificial scene. No real input data is needed.
"""

import argparse
from collections import Counter
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage


NS = 1_000_000_000
START_NS = NS
SCAN_COUNT = 4
IMU_FRAME = "imu_fsp_r_sensor"
LIDAR_TOPIC = "/sensor/lidar/top/points"
IMU_TOPIC = "/sensor/imu/front/data"
TWIST_TOPIC = "/vehicle/twist"
CONFIG = None
BINARY = None


def stamp(message, ns, frame):
    message.header.stamp.sec = ns // NS
    message.header.stamp.nanosec = ns % NS
    message.header.frame_id = frame
    return message


def static_tf():
    transform = stamp(TransformStamped(), START_NS - 20_000_000, IMU_FRAME)
    transform.child_frame_id = "base_footprint"
    # imu_from_base: a 180-degree yaw and a nonzero lever arm.
    transform.transform.rotation.z = 1.0
    transform.transform.rotation.w = 0.0
    transform.transform.translation.x = 0.3
    transform.transform.translation.y = 0.4
    transform.transform.translation.z = 0.2
    return TFMessage(transforms=[transform])


def cloud(ns):
    message = stamp(PointCloud2(), ns, "lidar_top")
    count = 1537  # Includes the final point after the configured stride of 3.
    message.height = 1
    message.width = count
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        PointField(name="time_stamp", offset=16, datatype=PointField.UINT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 20
    message.row_step = count * 20
    message.is_dense = True
    records = bytearray()
    for index in range(count):
        angle = 2 * math.pi * index / (count - 1)
        radius = 12.0 + 2.0 * math.sin(3.0 * angle)
        records.extend(struct.pack(
            "<fffBBHI", radius * math.cos(angle), radius * math.sin(angle),
            -1.2 + (index % 16) * 0.15, 40, 0, index % 128,
            index * 100_000_000 // (count - 1),
        ))
    message.data = bytes(records)
    return message


def imu(ns, orientation_available):
    message = stamp(Imu(), ns, IMU_FRAME)
    if orientation_available:
        # Known world_from_imu pitch, plus dynamic acceleration and real yaw
        # rate: these must not be reclassified as gravity or gyro bias.
        message.orientation.y = math.sin(0.1 / 2)
        message.orientation.w = math.cos(0.1 / 2)
        message.orientation_covariance[0] = 0.001
        message.linear_acceleration.x = 2.5
        message.linear_acceleration.z = 9.81
        message.angular_velocity.z = -0.35
    else:
        # Explicitly unavailable attitude; stationary samples exercise the
        # existing fallback without making claims about a moving fallback.
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.z = 9.81
    return message


def twist(ns, alternate=False):
    message = stamp(TwistStamped(), ns, "base_footprint")
    message.twist.linear.x = 99.0 if alternate else 7.0
    message.twist.linear.y = 1.0
    message.twist.angular.z = 0.4
    return message


def fixture_records(orientation_available=True, velocity="valid"):
    records = []

    def add(topic, record_ns, message):
        records.append((topic, bytes(serialize_message(message)), record_ns))

    add("/tf_static", START_NS - 20_000_000, static_tf())
    for index in range(SCAN_COUNT):
        ns = START_NS + index * 100_000_000
        add(LIDAR_TOPIC, ns, cloud(ns))
    for index in range(91):
        ns = START_NS + index * 5_000_000
        add(IMU_TOPIC, ns, imu(ns, orientation_available))
    # The second scan, which initializes the IMU, ends at START_NS + 200 ms.
    if velocity in ("valid", "future"):
        ns = START_NS + 195_000_000
        add(TWIST_TOPIC, ns, twist(ns))
        if velocity == "future":
            # Bag record order can deliver a future ROS header before the
            # initializer runs. It must choose the earlier true prior.
            add(TWIST_TOPIC, START_NS + 196_000_000,
                twist(START_NS + 250_000_000, alternate=True))
    elif velocity == "stale":
        ns = START_NS - 10_000_000
        add(TWIST_TOPIC, ns, twist(ns))  # Age 210 ms at initialization.
    elif velocity == "boundary":
        add(TWIST_TOPIC, START_NS, twist(START_NS))  # Exactly 200 ms old.
    elif velocity != "missing":
        raise ValueError(velocity)
    return sorted(records, key=lambda item: item[2])


def write_bag(path, records):
    types = {
        LIDAR_TOPIC: "sensor_msgs/msg/PointCloud2",
        IMU_TOPIC: "sensor_msgs/msg/Imu",
        TWIST_TOPIC: "geometry_msgs/msg/TwistStamped",
        "/tf_static": "tf2_msgs/msg/TFMessage",
    }
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic in dict.fromkeys(record[0] for record in records):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=types[topic], serialization_format="cdr"))
    for topic, data, ns in records:
        writer.write(topic, data, ns)
    if hasattr(writer, "close"):
        writer.close()
    del writer


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    result = []
    while reader.has_next():
        topic, data, ns = reader.read_next()
        result.append((topic, bytes(data), ns))
    if hasattr(reader, "close"):
        reader.close()
    del reader
    return result


def initialization_fields(stdout):
    line = next(line for line in stdout.splitlines()
                if "Headless IMU initialization:" in line)
    return dict(item.strip().split("=", 1) for item in
                line.split("Headless IMU initialization:", 1)[1].split(","))


class MovingStartInitializerTest(unittest.TestCase):
    def run_case(self, *, orientation_available=True, velocity="valid"):
        with tempfile.TemporaryDirectory(prefix="fastlio-moving-start-") as directory:
            root = Path(directory)
            input_bag, output_bag = root / "input", root / "result"
            write_bag(input_bag, fixture_records(orientation_available, velocity))
            original = read_bag(input_bag)
            completed = subprocess.run(
                [str(BINARY), "--input", str(input_bag), "--output", str(output_bag),
                 "--config", str(CONFIG)],
                capture_output=True, text=True, timeout=45,
            )
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
            result = read_bag(output_bag)
            # Original topics, serialized payloads, timestamps and ordering
            # must remain exact; additions are restricted to generated topics.
            remaining = iter(original)
            expected = next(remaining, None)
            generated_topics = {"/tf", "/tf_static", "/path", "/cloud_registered"}
            for record in result:
                if record == expected:
                    expected = next(remaining, None)
                else:
                    self.assertIn(record[0], generated_topics)
            self.assertIsNone(expected, "an original record changed or disappeared")
            result_counts = Counter(result)
            for record, count in Counter(original).items():
                self.assertEqual(result_counts[record], count)
            self.assertEqual(sum(topic == LIDAR_TOPIC for topic, _, _ in original), SCAN_COUNT)
            self.assertEqual(sum(topic == "/cloud_registered" for topic, _, _ in result), SCAN_COUNT)
            poses = [transform for topic, data, _ in result if topic == "/tf"
                     for transform in deserialize_message(data, TFMessage).transforms
                     if transform.header.frame_id == "map" and transform.child_frame_id == "body"]
            self.assertEqual(len(poses), SCAN_COUNT)
            for label in ("lidar records fed", "all result frames", "registered scans"):
                self.assertRegex(completed.stdout, rf"{re.escape(label)}:\s*{SCAN_COUNT}\b")
            self.assertRegex(completed.stdout, r"bootstrap estimates:\s*3\b")
            self.assertRegex(completed.stdout, r"optimized solutions:\s*1\b")
            self.assertRegex(completed.stdout, rf"verified input records:\s*{len(original)}\b")
            fields = initialization_fields(completed.stdout)
            print(f"{velocity}, orientation_available={orientation_available}: {fields}")
            return fields, completed.stderr

    def assert_vector(self, value, expected):
        actual = [float(item) for item in value.split()]
        self.assertEqual(len(actual), 3)
        for component, wanted in zip(actual, expected):
            self.assertAlmostEqual(component, wanted, delta=1e-4)

    def assert_moving_seed(self, fields):
        self.assertEqual(fields["gravity_method"], "provided_orientation")
        self.assertEqual(fields["gyro_bias_method"], "zero_prior")
        self.assertEqual(float(fields["acceleration_scale"]), 1.0)
        self.assert_vector(fields["gyro_bias"], (0, 0, 0))
        self.assert_vector(fields["gravity"],
                           (9.809 * math.sin(0.1), 0, -9.809 * math.cos(0.1)))
        self.assertEqual(fields["velocity_method"], "vehicle_twist_prior")
        # Rz(pi)*[7,1,0] - [0,0,.4] cross [.3,.4,.2].
        self.assert_vector(fields["initial_velocity"], (-6.84, -1.12, 0))

    def test_moving_orientation_and_lever_arm_velocity(self):
        fields, _ = self.run_case()
        self.assert_moving_seed(fields)

    def test_future_velocity_does_not_displace_valid_prior(self):
        fields, _ = self.run_case(velocity="future")
        self.assert_moving_seed(fields)

    def test_velocity_exactly_200_ms_old_is_accepted(self):
        fields, _ = self.run_case(velocity="boundary")
        self.assert_moving_seed(fields)

    def test_stale_velocity_falls_back_to_zero(self):
        fields, stderr = self.run_case(velocity="stale")
        self.assertEqual(fields["gravity_method"], "provided_orientation")
        self.assertEqual(fields["velocity_method"], "zero_prior")
        self.assert_vector(fields["initial_velocity"], (0, 0, 0))
        self.assertEqual(stderr.count("no valid /vehicle/twist prior"), 1)

    def test_unavailable_orientation_retains_stationary_fallback(self):
        fields, stderr = self.run_case(orientation_available=False, velocity="missing")
        self.assertEqual(fields["gravity_method"], "stationary_acceleration")
        self.assertEqual(fields["gyro_bias_method"], "stationary_mean")
        self.assertAlmostEqual(float(fields["acceleration_scale"]), 1.0, delta=1e-5)
        self.assert_vector(fields["gravity"], (0, 0, -9.809))
        self.assert_vector(fields["gyro_bias"], (0, 0, 0))
        self.assertEqual(fields["velocity_method"], "zero_prior")
        self.assertIn("moving split-bag start is ambiguous", stderr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--config", type=Path)
    options, remaining = parser.parse_known_args()
    BINARY = options.binary or Path(get_package_prefix("fast_lio")) / "lib/fast_lio/fastlio_headless"
    CONFIG = options.config or Path(get_package_share_directory("fast_lio")) / "config/top_autoware_front_imu.yaml"
    unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
