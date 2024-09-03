import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def get_file_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    teleop_yaml_file = get_file_path('r5sr_teleop', 'config/r5sr_teleop.yaml')

    # Nodes
    hazmat_label_yaml_file = get_file_path(
        'r5sr_teleop', 'config/yolo/robocup-hazmat-label-2022.yaml')

    darknet_group = GroupAction(
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='vision_front_repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'vision_front_camera/image_raw/compressed'),
                        ('out', 'vision_front_camera/image_raw/uncompressed'),
                ],
            ),
            Node(
                package='darknet_ros',
                executable='darknet_ros',
                name='darknet_ros',
                output='screen',
                parameters=[
                        teleop_yaml_file,
                        hazmat_label_yaml_file,
                        {'config_path': get_file_path(
                            'r5sr_teleop', 'config/yolo/cfg')},
                        {'weights_path': get_file_path(
                            'r5sr_teleop', 'config/yolo/weights')},
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            darknet_group,
        ]
    )
