# Offline analysis

## Week 5 mission analyzer

Run from the repository root with the ROS 2 Python environment sourced:

```bash
source /opt/ros/humble/setup.bash
python3 tools/analysis/analyze_mission.py bags/week5_mission
```

The script opens the bag, filters `/chassis/odom`, deserializes Odometry
messages, and prints the first message's frame identifiers and total sample
count. It uses the storage plugin from the bag metadata; override it with
`--storage-id sqlite3` or `--storage-id mcap` if needed. The matching plugin
must be installed in your ROS environment.

Complete the `TODO (you)` comments in the script for your measurement data and
frame choices, geometry, ideal distance, path stretch, and interpretation.
The sample count checks the reader; it does not confirm full mission coverage.
No performance metric is calculated by this scaffold.

The assignment and AI-use boundary are in
[the week 5 plan](../../docs/plans/2026-10-19-week-05.md).

## Week 7 IMU/odometry comparison scaffold

Use a ROS 2 Python environment with `rosbag2_py`, `rclpy`, `sensor_msgs`,
`nav_msgs`, the bag's storage plugin, and `tf_transformations`. On Humble,
the quaternion helper is provided by `ros-humble-tf-transformations` (install
it if missing). `--help` also works without sourcing ROS.

From the repository root:

```bash
source /opt/ros/humble/setup.bash
python3 tools/analysis/analyze_imu_odom.py \
  bags/week7_imu_stationary/stationary \
  bags/week7_imu_stationary/positive_yaw
```

These are the current recordings' directory names. Both currently contain
IMU messages but no `/chassis/odom`; their `/cmd_vel` topics have zero messages.
Record new stationary and positive-yaw bags containing both `/chassis/imu`
and `/chassis/odom` before running the comparison. `/cmd_vel` is optional
and is not read by this script. Use the new bag directories as the two arguments.

Override discovered topic names with `--imu-topic /your/imu` and
`--odom-topic /your/odom`. Storage is detected from metadata, or override it
with `--storage-id sqlite3` or `--storage-id mcap`. Missing topics, incorrect
message types, and topics with no samples produce errors.

The scaffold reads the streams separately and collects IMU angular velocity
and acceleration components plus odometry yaw from the standard tf helper.
It retains integer nanosecond timestamps from **message `header.stamp`**,
not bag receipt times, and preserves recorded order. There is no fallback to
another clock, sorting, interpolation, frame transformation, or synchronization.
Frame identifiers are printed for your reasoning; the script does not decide
whether the recorded axes are comparable.

Complete these three `TODO (you)` functions:

- `calculate_acceleration_magnitude(ax, ay, az)`: return one acceleration
  magnitude; stationary means are already wired to this function.
- `calculate_odom_yaw_rates(samples)`: use each `OdomSample.timestamp_ns` and
  `.yaw` to implement wrapped angle differences and finite-difference yaw rate.
  Skip the first sample and nonpositive time intervals. Return a list of rates.
- `compare_yaw_rate_signs(imu_mean, odom_mean)`: report sign agreement after
  considering frames, axes, and what zero or near-zero values mean.

Until these are implemented, the corresponding results explicitly say TODO.
With valid bags, stationary gyro mean and population standard deviation and
positive-yaw gyro mean are available immediately. The scaffold supplies the
remaining summary statistics once your functions return values. These means
cover each entire recording, so you own whether it represents a steady interval.

Keep predictions, gravity/bias reasoning, debugging hypotheses, and conclusions
in your own response. The scope and AI-use boundary are in
[the week 7 plan](../../docs/plans/2026-11-02-week-07.md).
