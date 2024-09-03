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

def generate_launch_description():

    hand_motion_image_group = GroupAction(
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'hand_camera/image_raw/compressed'),
                        ('out', 'hand_camera/image_raw/uncompressed'),
                ],
            ),
            Node(
                package='r5sr_image_proccesing',
                executable='image_motion_detect_node',
                name='image_motion_detect_node',
                remappings=[
                        ('image_raw', 'hand_camera/image_raw/uncompressed'),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            hand_motion_image_group,
        ]
    )
