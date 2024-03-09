import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    # yolov8.launch.pyファイルのインクルード
    yolo_v8_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/yolov8.launch.py'])
    )

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

    image_converter_node = Node(
        package='r5sr_meter_inspection',
        executable='image_converter_node',
        name='image_converter_node'
    )
    
    meter_value_calculator_node = Node(
        package='r5sr_meter_inspection',
        executable='meter_value_calculator_node',
        name='meter_value_calculator_node'
    )

    inspection_result_node = Node(
        package='r5sr_meter_inspection',
        executable='inspection_result_node',
        name='inspection_result_node'
    )

    return LaunchDescription([
        image_converter_node,
        yolo_v8_launch_file,
        usb_cam_node,
        meter_value_calculator_node,
        inspection_result_node
    ])
