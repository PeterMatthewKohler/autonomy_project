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

The analyzer also reports the traveled planar distance, ideal route distance,
their ratio, and the final goal error. The sample count checks the reader.
Use the recorded path and mission goals to verify full mission coverage.

The assignment and AI-use boundary are in
[the week 5 plan](../../docs/plans/2026-10-19-week-05.md).

## Week 7 IMU/odometry comparison

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

Both recordings contain `/chassis/imu` and `/chassis/odom`. The analyzer
uses those two topics. For other recordings, pass the corresponding stationary
and positive-yaw bag directories as the two arguments.

Override discovered topic names with `--imu-topic /your/imu` and
`--odom-topic /your/odom`. Storage is detected from metadata, or override it
with `--storage-id sqlite3` or `--storage-id mcap`. Missing topics, incorrect
message types, and topics with no samples produce errors.

The analyzer reads the streams separately and collects IMU angular velocity
and acceleration components plus odometry yaw from the standard tf helper.
It retains integer nanosecond timestamps from **message `header.stamp`**,
not bag receipt times, and preserves recorded order. There is no fallback to
another clock, sorting, interpolation, frame transformation, or synchronization.
Frame identifiers are printed for your reasoning; the script does not decide
whether the recorded axes are comparable.

The Week 7 calculation functions are implemented. Their responsibilities are:

- `calculate_acceleration_magnitude(ax, ay, az)`: return one acceleration
  magnitude; stationary means are already wired to this function.
- `calculate_odom_yaw_rates(samples)`: use each `OdomSample.timestamp_ns` and
  `.yaw` to implement wrapped angle differences and finite-difference yaw rate.
  Skip the first sample and nonpositive time intervals. Return a list of rates.
- `compare_yaw_rate_signs(imu_mean, odom_mean)`: report sign agreement after
  considering frames, axes, and what zero or near-zero values mean.

The summary reports stationary gyro mean, population standard deviation, and
mean acceleration magnitude. For the turn, it reports mean gyro and odometry
yaw rates and sign agreement. These means cover each entire recording.

Keep predictions, gravity/bias reasoning, debugging hypotheses, and conclusions
in your own response. The scope and AI-use boundary are in
[the week 7 plan](../../docs/plans/2026-11-02-week-07.md).

## Week 8 heading comparison

Run with just the verified turn bag (the older two-bag command also works):

```bash
python3 tools/analysis/analyze_imu_odom.py bags/week7_imu_stationary/positive_yaw
```

The analyzer selects the shared header-timestamp interval, keeps IMU
samples within it in recorded order, and finds odometry samples nearest
its endpoints. Ties use the first sample in recorded order. Missing streams,
nonpositive overlap, or fewer than two selected IMU samples produce an
unavailable message. No samples are interpolated or sorted. The printed
duration covers the shared interval; selected IMU timestamps and reference
odometry timestamps are also printed so endpoint offsets remain visible.

`propagate_imu_heading()` starts from the reference odometry yaw and integrates
the IMU's z-axis angular velocity across positive timestamp intervals. The
analyzer prints the final heading, interval count, and wrapped final error in
radians and degrees. See the [Week 8 plan](../../docs/plans/2026-11-09-week-08.md).

## Week 9 heading observation update

Run the Week 8 command to see the raw result and measured IMU and odometry
header periods. After inspecting those periods, document your chosen maximum
observation age and pass it in seconds using `--max-observation-age-s`.
For example, this command uses a 0.10 s limit:

```bash
python3 tools/analysis/analyze_imu_odom.py \
  bags/week7_imu_stationary/positive_yaw --max-observation-age-s 0.10
```

The gain defaults to the plan's fixed `0.05`. `--correction-gain` accepts
finite values from zero to one.

`match_heading_observations()` offers the latest odometry sample at or before
each IMU timestamp, once per odometry timestamp. If several samples become
eligible in one step, only the latest is offered. For equal odometry timestamps,
the last recorded sample wins. Decreasing odometry timestamps produce an error.

`estimate_corrected_heading()` carries one heading forward through gyro
prediction and applies `correct_heading()` for observations that pass the age
check. The first IMU sample establishes the timestamp for the heading initialized
from odometry. The report includes the gain, age limit, accepted and stale
counts, mean absolute wrapped innovation, and raw and corrected final headings
and errors over the same comparison interval. See the
[Week 9 plan](../../docs/plans/2026-11-16-week-09.md).

## Tests

Run the ROS-independent analysis tests:

```bash
python3 -m unittest discover -s tools/analysis -p 'test_*.py'
```
