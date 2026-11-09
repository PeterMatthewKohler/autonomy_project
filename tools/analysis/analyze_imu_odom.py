#!/usr/bin/env python3

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from statistics import fmean, pstdev
import math


@dataclass
class ImuSample:
    timestamp_ns: int
    angular_velocity_z: float
    ax: float
    ay: float
    az: float


@dataclass
class OdomSample:
    timestamp_ns: int
    yaw: float


@dataclass
class BagSamples:
    imu: list[ImuSample] = field(default_factory=list)
    odom: list[OdomSample] = field(default_factory=list)
    imu_frames: set[str] = field(default_factory=set)
    odom_frames: set[tuple[str, str]] = field(default_factory=set)


def calculate_acceleration_magnitude(ax: float, ay: float, az: float) -> float:
    return math.sqrt(ax**2 + ay**2 + az**2)

def calculate_odom_yaw_rates(samples: list[OdomSample]) -> list[float]:
    yaw_rates = []
    prev_sample = None

    for odom_sample in samples:
        if prev_sample is None:
            prev_sample = odom_sample
            continue

        delta_yaw = math.atan2(
            math.sin(odom_sample.yaw - prev_sample.yaw),
            math.cos(odom_sample.yaw - prev_sample.yaw)
        )
        dt = (odom_sample.timestamp_ns - prev_sample.timestamp_ns) * 1e-9
        prev_sample = odom_sample
        if dt > 0:
            yaw_rates.append(delta_yaw / dt)
    return yaw_rates
        


def compare_yaw_rate_signs(imu_mean: float, odom_mean: float) -> bool:
    # Noise on a zero mean sample could say their signs don't agree, we don't really care about that
    if (abs(imu_mean - odom_mean) < 0.01):
        return True

    if (imu_mean > 0 and odom_mean > 0) or (imu_mean < 0 and odom_mean < 0):
        return True
    else:
        return False


def read_bag(
    bag: Path, imu_topic: str, odom_topic: str, storage_id: str
) -> BagSamples:
    # Lazy imports keep --help usable outside a ROS shell.
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python imports failed. Source your ROS 2 setup.bash and use "
            f"its compatible Python interpreter.\n{exc}"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    expected = {imu_topic: "sensor_msgs/msg/Imu", odom_topic: "nav_msgs/msg/Odometry"}
    for topic, message_type in expected.items():
        if topics.get(topic) != message_type:
            raise ValueError(
                f"{bag}: expected {topic} with type {message_type}; "
                f"found {topics.get(topic, 'no such topic')}. "
                "Check your topic arguments or record a bag containing both "
                f"{imu_topic} and {odom_topic}."
            )

    try:
        from tf_transformations import euler_from_quaternion
    except ImportError as exc:
        raise RuntimeError(
            "The quaternion helper tf_transformations is missing. On ROS 2 "
            "Humble, install ros-humble-tf-transformations and source setup.bash."
        ) from exc

    reader.set_filter(rosbag2_py.StorageFilter(topics=[imu_topic, odom_topic]))
    samples = BagSamples()
    while reader.has_next():
        topic, serialized_data, _bag_timestamp_ns = reader.read_next()
        message_type = Imu if topic == imu_topic else Odometry
        message = deserialize_message(serialized_data, message_type)
        stamp = message.header.stamp
        # Use measurement timestamps consistently, never bag receipt timestamps.
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if topic == imu_topic:
            acceleration = message.linear_acceleration
            samples.imu.append(
                ImuSample(
                    timestamp_ns,
                    message.angular_velocity.z,
                    acceleration.x,
                    acceleration.y,
                    acceleration.z,
                )
            )
            samples.imu_frames.add(message.header.frame_id)
        else:
            q = message.pose.pose.orientation
            # tf expects quaternion components in x, y, z, w order.
            _roll, _pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            samples.odom.append(OdomSample(timestamp_ns, yaw))
            samples.odom_frames.add((message.header.frame_id, message.child_frame_id))

    for topic, values in ((imu_topic, samples.imu), (odom_topic, samples.odom)):
        if not values:
            raise ValueError(f"{bag}: no messages found on {topic}; check ros2 bag info.")
    return samples


def print_summary(label: str, samples: BagSamples, stationary: bool) -> None:
    print(f"\n{label}")
    print(f"Samples: IMU={len(samples.imu)}, odometry={len(samples.odom)}")
    print(f"IMU frame_id(s): {sorted(samples.imu_frames)!r}")
    print(f"Odometry (frame_id, child_frame_id): {sorted(samples.odom_frames)!r}")
    print("Timestamps: message header.stamp (nanoseconds)")
    gyro_z = [sample.angular_velocity_z for sample in samples.imu]
    imu_mean = fmean(gyro_z)
    print(f"Mean IMU angular_velocity.z: {imu_mean:.6f} rad/s")

    if stationary:
        print(f"IMU angular_velocity.z population stddev: {pstdev(gyro_z):.6f} rad/s")
        try:
            accel_mean = fmean(
                calculate_acceleration_magnitude(sample.ax, sample.ay, sample.az)
                for sample in samples.imu
            )
        except NotImplementedError as exc:
            print(f"Mean acceleration magnitude: {exc}")
        else:
            print(f"Mean acceleration magnitude: {accel_mean:.6f} m/s^2")
        return

    try:
        yaw_rates = calculate_odom_yaw_rates(samples.odom)
    except NotImplementedError as exc:
        print(f"Mean odometry-derived yaw rate: {exc}")
        return
    if not yaw_rates:
        print("Mean odometry-derived yaw rate: unavailable (no valid sample pairs).")
        print("Sign agreement: unavailable without odometry yaw rates.")
        return
    odom_mean = fmean(yaw_rates)
    print(f"Mean odometry-derived yaw rate: {odom_mean:.6f} rad/s")
    try:
        agreement = compare_yaw_rate_signs(imu_mean, odom_mean)
    except NotImplementedError as exc:
        print(f"Sign agreement: {exc}")
    else:
        print(f"Sign agreement: {agreement}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stationary_bag", type=Path, help="Stationary bag directory")
    parser.add_argument("positive_yaw_bag", type=Path, help="Positive-yaw bag directory")
    parser.add_argument("--imu-topic", default="/chassis/imu")
    parser.add_argument("--odom-topic", default="/chassis/odom")
    parser.add_argument(
        "--storage-id", default="", help="Storage plugin; default: detect from metadata"
    )
    args = parser.parse_args()
    for bag in (args.stationary_bag, args.positive_yaw_bag):
        if not bag.is_dir():
            parser.error(f"Bag directory does not exist: {bag}")
    if args.imu_topic == args.odom_topic:
        parser.error("IMU and odometry topics must be different.")

    try:
        stationary = read_bag(
            args.stationary_bag, args.imu_topic, args.odom_topic, args.storage_id
        )
        positive_yaw = read_bag(
            args.positive_yaw_bag, args.imu_topic, args.odom_topic, args.storage_id
        )
    except (RuntimeError, ValueError) as exc:
        parser.exit(1, f"Error: {exc}\n")

    print_summary("Stationary", stationary, stationary=True)
    print_summary("Positive yaw", positive_yaw, stationary=False)


if __name__ == "__main__":
    main()
