# Nova Carter mission skeleton

This package provides `MissionNode_exe` separately from the waypoint controller.
It subscribes to `/chassis/odom` and `/cmd_vel`, and creates a client for `/goal_service` using
`nova_carter_interfaces/srv/Goal`. Both node packages depend on the shared
`nova_carter_interfaces` package; neither owns a duplicate service definition.

Entry points:

- `include/mission_node/MissionNode_impl.hpp`: declarations and member TODOs.
- `src/MissionNode_impl.cpp`: ROS wiring and callback TODOs.
- `src/MissionNode_main.cpp`: explicit static single-threaded executor.

The node has a 200 ms timer using the node clock. `sendGoal(x, y)` contains only
asynchronous request boilerplate and forwards responses to
`goalResponseCallback()`; nothing calls it yet. The skeleton is intentionally
idle and has no velocity publisher. Waypoint storage, indexing, mission states,
transitions, distance and frame reasoning, and retries remain your Week 4 work.

The command callback caches the latest `Twist` in `currCmd_` and the node-clock
receipt time in `lastCmdTime_`, under `mutex_`. Both are empty until a command
arrives. Interpreting command activity and freshness belongs to the mission logic.

## Build and run

```bash
cd ~/Development/personal/autonomy_project/nova_carter
source /opt/ros/humble/setup.bash
colcon build --packages-up-to simple_waypoint_node simple_mission_node --symlink-install
source install/local_setup.bash
ros2 launch simple_mission_node mission_node_launch.py
```

Launch the controller in another terminal with the same environment:

```bash
ros2 launch simple_waypoint_node waypoint_node_launch.py
```

Use `use_sim_time:=true` for both nodes when Isaac Sim publishes `/clock`.
`params/mission_node_params.yaml` declares `odom_frame` and `goal_tolerance`;
keep these consistent with the controller's configuration.

## Checks

```bash
colcon test --packages-select nova_carter_interfaces simple_waypoint_node simple_mission_node
colcon test-result --verbose
```
