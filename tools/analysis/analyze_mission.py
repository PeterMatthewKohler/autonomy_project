#!/usr/bin/env python3
"""Week 5 bag-reading scaffold; the measurement and interpretation are yours.

From the repository root, in a shell with your ROS 2 environment sourced:
    python3 tools/analysis/analyze_mission.py bags/week5_mission

TODOs deliberately leave the AI-use boundary's personally owned work unfinished.
"""

import argparse
from pathlib import Path
import math


ODOM_TOPIC = "/chassis/odom"
MISSION_FRAME = "odom"
BASE_FRAME = "base_link"

GOALS = [[1.0, 1.0], [3.0, -1.0], [0.0, 0.0]]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Recorded bag directory")
    parser.add_argument(
        "--storage-id",
        default="",
        help="Storage plugin, e.g. sqlite3 or mcap; default: detect from metadata",
    )
    args = parser.parse_args()
    if not args.bag.is_dir():
        parser.error(f"Bag directory does not exist: {args.bag}")

    # Import after argument parsing so --help also works outside a ROS shell.
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python imports failed. Source your ROS 2 setup.bash and use "
            f"its compatible Python interpreter.\n{exc}"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topics.get(ODOM_TOPIC) != "nav_msgs/msg/Odometry":
        raise SystemExit(f"Expected {ODOM_TOPIC} with type nav_msgs/msg/Odometry.")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[ODOM_TOPIC]))

    # TODO (you): Set up your own analysis state and mission goal sequence here.
    longitudinal_dist = 0.0
    sample_count = 0
    first_odom: tuple[float, float] | None = None
    odom_xy_prev: tuple[float, float] | None = None
    odom_xy_curr = odom_xy_prev
    while reader.has_next():
        _topic, serialized_data, bag_timestamp_ns = reader.read_next()
        odom: Odometry = deserialize_message(serialized_data, Odometry)
        if sample_count == 0:
            print(f"First header.frame_id: {odom.header.frame_id!r}")
            print(f"First child_frame_id:  {odom.child_frame_id!r}")
            if odom.header.frame_id != MISSION_FRAME or odom.child_frame_id != BASE_FRAME:
                raise SystemExit(f"Recorded odometry frames incorrect. Mission frame must be {MISSION_FRAME} and base frame must be {BASE_FRAME}")
        sample_count += 1
        # First loop only
        if odom_xy_prev is None:
            odom_xy_prev = (odom.pose.pose.position.x, odom.pose.pose.position.y)
            first_odom = odom_xy_prev
            continue
        # Every other loop
        odom_xy_curr = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        longitudinal_dist += math.sqrt(
            (odom_xy_curr[0] - odom_xy_prev[0])**2 +
            (odom_xy_curr[1] - odom_xy_prev[1])**2
        )
        odom_xy_prev = odom_xy_curr

    if sample_count == 0:
        raise SystemExit(f"No messages found on {ODOM_TOPIC}.")
    print(f"Deserialized {sample_count} messages from {ODOM_TOPIC}.")

    ideal_dist = 0.0
    prev_goal = None
    for goal in GOALS:
        if prev_goal is None:
            prev_goal = goal
            ideal_dist += math.sqrt(
                (prev_goal[0] - first_odom[0])**2 +
                (prev_goal[1] - first_odom[1])**2
            )
            continue
        ideal_dist += math.sqrt(
            (goal[0] - prev_goal[0])**2 +
            (goal[1] - prev_goal[1])**2
        )
        prev_goal = goal
    # Implement path stretch and print your three metric results.
    path_stretch = longitudinal_dist / ideal_dist
    # Print results
    print(f"Longitudinal dist: {longitudinal_dist}m, ideal dist: {ideal_dist}m, path stretch: {path_stretch}.")


if __name__ == "__main__":
    main()
