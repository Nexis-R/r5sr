import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_file_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)


def generate_launch_description():
    config_file = get_file_path('r5sr_bringup', 'config/livox_lio.yaml')
    hardware_config_file = get_file_path(
        'r5sr_bringup', 'config/MID360_config.json')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_ros_driver2',
        parameters=[config_file,
                    {'user_config_path': hardware_config_file}],
    )

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=[config_file],
    )
    return LaunchDescription([
        livox_driver,
        fast_lio
    ])
