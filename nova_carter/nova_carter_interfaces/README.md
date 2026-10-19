# Nova Carter interfaces

Shared ROS 2 service definitions for the waypoint controller and mission node.
`srv/Goal.srv` is generated once in this package; consumers declare a dependency
on `nova_carter_interfaces`.

The `Goal` request contains `x` and `y` in metres in the controller's configured
`odom` frame. The response contains `success` (request accepted) and `message`.
Acceptance does not report arrival at the goal.

C++ includes `<nova_carter_interfaces/srv/goal.hpp>` and uses
`nova_carter_interfaces::srv::Goal`. Python imports
`Goal` from `nova_carter_interfaces.srv`.

Build it directly, or let `colcon build --packages-up-to` build it before either
node package:

```bash
colcon build --packages-select nova_carter_interfaces --symlink-install
source install/local_setup.bash
ros2 interface show nova_carter_interfaces/srv/Goal
```
