from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('simple_odom_node')
    #params_file = os.path.join(package_share, 'params', 'odom_node_params.yaml')

    simple_odom_node = Node(
        package='simple_odom_node',
        executable='OdomNode_exe',
        name='odom_node',
        output='screen',
        #parameters=[params_file],
    )

    return LaunchDescription([
        simple_odom_node
    ])