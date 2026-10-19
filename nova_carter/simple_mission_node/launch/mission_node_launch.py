import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('simple_mission_node')
    params_file = os.path.join(package_share, 'params', 'mission_node_params.yaml')

    mission_node = Node(
        package='simple_mission_node',
        executable='MissionNode_exe',
        name='mission_node',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[params_file, {
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Logging level for the mission node.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from the simulator when available.'),
        mission_node,
    ])
