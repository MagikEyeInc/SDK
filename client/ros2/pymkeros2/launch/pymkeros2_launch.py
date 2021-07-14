import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
        get_package_share_directory('pymkeros2'),
        'config',
        'pymkeros2_config.yaml'
        )

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pymkeros2',
            executable='pymkeros2_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])