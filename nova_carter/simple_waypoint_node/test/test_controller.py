"""Exercise the real node through odometry, goal requests, and velocity output."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from simple_waypoint_node.srv import Goal


class ControllerTest(unittest.TestCase):
    def setUp(self):
        # Every process in this test shares an isolated, localhost-only domain.
        os.environ['ROS_DOMAIN_ID'] = '211'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        for name in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            os.environ.pop(name, None)
        self.directory = tempfile.TemporaryDirectory(prefix='waypoint-controller-test-')
        self.output = Path(self.directory.name) / 'output.log'
        self.stream = self.output.open('w')
        executable = Path(get_package_prefix('simple_waypoint_node')) / (
            'lib/simple_waypoint_node/WaypointNode_exe')
        self.process = subprocess.Popen(
            [str(executable)], stdout=self.stream, stderr=subprocess.STDOUT,
            start_new_session=True, env=dict(os.environ, ROS_LOG_DIR=self.directory.name))
        rclpy.init()
        self.probe = rclpy.create_node('controller_test_probe')
        self.commands = []
        self.subscription = self.probe.create_subscription(
            Twist, '/cmd_vel', self.commands.append, 10)
        self.publisher = self.probe.create_publisher(Odometry, '/chassis/odom', 10)
        self.client = self.probe.create_client(Goal, '/goal_service')
        self.assertTrue(self.client.wait_for_service(timeout_sec=8))
        deadline = time.monotonic() + 8
        while self.publisher.get_subscription_count() == 0:
            self.assertLess(time.monotonic(), deadline, 'Odometry discovery timed out')
            rclpy.spin_once(self.probe, timeout_sec=0.02)
        self.pose(0.0, 0.0, 0.0)

    def tearDown(self):
        self.probe.destroy_node()
        rclpy.shutdown()
        os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait(timeout=5)
        self.stream.close()
        self.directory.cleanup()

    def pose(self, x, y, yaw, duration=0.25):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(yaw / 2)
        message.pose.pose.orientation.w = math.cos(yaw / 2)
        self.commands.clear()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self.probe, timeout_sec=0.02)

    def goal(self, x, y):
        request = Goal.Request()
        request.x = float(x)
        request.y = float(y)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.probe, future, timeout_sec=5)
        self.assertTrue(future.done(), 'Goal service timed out')
        return future.result()

    def latest(self):
        self.assertTrue(self.commands, 'No velocity command received')
        self.assertIsNone(self.process.poll(), 'Controller exited')
        return self.commands[-1]

    def test_turns_clockwise_while_driving_and_stops_inside_tolerance(self):
        self.assertTrue(self.goal(0, -1).success)
        self.pose(0, 0, 0)
        command = self.latest()
        self.assertLess(command.angular.z, 0)
        self.assertGreaterEqual(command.angular.z, -0.6)
        self.assertGreater(command.linear.x, 0, 'Allow forward motion while turning')
        self.pose(0, 0, -math.pi / 2)
        command = self.latest()
        self.assertGreater(command.linear.x, 0)
        self.assertLessEqual(command.linear.x, 0.3)
        # Arrival must take priority even when the robot faces away from the goal.
        self.pose(0.1, -0.95, math.pi)
        command = self.latest()
        self.assertEqual(command.linear.x, 0)
        self.assertEqual(command.angular.z, 0)
        self.assertTrue(self.goal(0, -2).success, 'Completed goal must release the service')

    def test_wraps_heading_across_zero_and_logs_degree_error(self):
        self.pose(0, 0, math.radians(358))
        self.assertTrue(self.goal(math.cos(math.radians(2)),
                                  math.sin(math.radians(2))).success)
        self.assertFalse(self.goal(-10, -10).success, 'Active goal must not be replaced')
        self.pose(0, 0, math.radians(358), duration=1.3)
        command = self.latest()
        self.assertGreater(command.linear.x, 0)
        self.assertGreater(command.angular.z, 0)
        self.assertLess(command.angular.z, 0.2, 'Four-degree correction should stay small')
        output = self.output.read_text()
        self.assertRegex(output, r'heading error=4(?:\.0*)? deg')
        self.assertRegex(output, r'distance error=1(?:\.0*)? m')

    def test_reaches_an_off_axis_goal_in_a_planar_simulation(self):
        x, y, yaw = 0.0, 0.0, math.radians(170)
        self.pose(x, y, yaw)
        self.assertTrue(self.goal(1.0, 0.7).success)
        for _ in range(500):
            self.pose(x, y, yaw, duration=0.05)
            command = self.latest()
            self.assertTrue(math.isfinite(command.linear.x))
            self.assertTrue(math.isfinite(command.angular.z))
            self.assertGreaterEqual(command.linear.x, 0)
            self.assertLessEqual(command.linear.x, 0.3)
            self.assertLessEqual(abs(command.angular.z), 0.6)
            x += command.linear.x * math.cos(yaw) * 0.05
            y += command.linear.x * math.sin(yaw) * 0.05
            yaw += command.angular.z * 0.05
            if math.hypot(1.0 - x, 0.7 - y) <= 0.15:
                self.pose(x, y, yaw)
                command = self.latest()
                self.assertEqual(command.linear.x, 0)
                self.assertEqual(command.angular.z, 0)
                return
        self.fail('Controller did not reach the off-axis goal')


if __name__ == '__main__':
    unittest.main()
