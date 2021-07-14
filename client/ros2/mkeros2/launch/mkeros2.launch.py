from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
        get_package_share_directory('mkeros2'),
        'config',
        'mkeros2_config.yaml'
        )

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package="mkeros2",
            executable="mkeros2_node",
            output="screen",
            emulate_tty=True,
            parameters = [config]
        )
    ])