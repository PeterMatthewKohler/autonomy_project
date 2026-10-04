# Nova Carter waypoint controller

The node subscribes to `/chassis/odom`, accepts one fixed goal through
`/goal_service`, and publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.
Goal coordinates are metres in the configured `odom` frame.

The control loop runs at 20 Hz. Forward motion and heading correction run
together, with limits from `max_linear_speed` and `max_angular_speed`. When the
robot is within `goal_tolerance`, it publishes zero velocity and releases the
goal so another request can be accepted. Requests received during an active
goal are rejected.

## PID inputs and logging

The linear PID compares zero remaining distance with measured remaining distance:
`latPID.update(0.0, distanceError)`. Its negative output is mapped to forward
velocity. The angular PID compares desired heading with measured yaw:
`rotPID.updateAngle(desiredHeading, measuredYaw)`. Angle wrapping happens inside
the PID.

The default gains are `Kp=1, Ki=0, Kd=0` for each controller. The existing
integral and derivative calculations remain available in the PID class.

The PID receives logging support with:

```cpp
pid.setLogging(get_logger(), get_clock(), "Angular");
```

Its class methods use `RCLCPP_INFO_STREAM_THROTTLE` to print reference,
measurement, signed error, and output once per second. Internal angle
calculations use radians; angular log values use degrees and degrees/second.
Linear values use metres and metres/second.

A separate traversal log shows positive remaining distance, wrapped heading
error, and the actual velocity commands after limiting, for example:

```text
Traversal: distance error=0.8 m, heading error=-12 deg, aligned=1, v=0.3 m/s, w=-12 deg/s
```

`heading_tolerance` controls the displayed `aligned` flag; it does not gate
motion. `goal_distance` is retained from the original skeleton; service-provided
coordinates determine the goal.

## Build and run

Build and source the same workspace to avoid selecting the older installation
at the repository root:

```bash
cd ~/Development/personal/autonomy_project/nova_carter
source /opt/ros/humble/setup.bash
colcon build --packages-select simple_waypoint_node --symlink-install
source install/local_setup.bash
ros2 launch simple_waypoint_node waypoint_node_launch.py
```

If Isaac Sim publishes `/clock`, add `use_sim_time:=true`.

Submit a goal from another shell with the same ROS environment:

```bash
ros2 service call /goal_service simple_waypoint_node/srv/Goal "{x: 1.0, y: 0.5}"
```

Frame names, position tolerance, and velocity limits are in
`params/waypoint_node_params.yaml`. Topic names can be remapped with ROS
arguments.

## Verification

```bash
colcon test --packages-select simple_waypoint_node
colcon test-result --test-result-base build/simple_waypoint_node --verbose
```

The regression tests run the real ROS node in a localhost-only test domain,
check turn direction, angle wrapping, goal rejection, logging units, arrival
and stopping, and integrate commanded motion in a planar simulation.
