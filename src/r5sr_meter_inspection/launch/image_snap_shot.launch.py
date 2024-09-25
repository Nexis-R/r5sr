import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    # USB Cam Nodeのパラメータ直書き
    usb_cam_params = {
        'video_device': '/dev/video0',
        'framerate': 10.0,
        'image_width': 640,
        'image_height': 480,
    }

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[usb_cam_params],
        name='usb_cam'
    )

    image_snapshot_publisher_node = Node(
        package='r5sr_meter_inspection',
        executable='image_snapshot_publisher_node',
        name='image_snapshot_publisher_node'
    )

    return LaunchDescription([
        usb_cam_node,
        image_snapshot_publisher_node,

    ])
