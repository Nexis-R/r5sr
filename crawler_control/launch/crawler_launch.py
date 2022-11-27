import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('crawler_control'),
      'config',
      'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='crawler_control',
            executable='crawler_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])