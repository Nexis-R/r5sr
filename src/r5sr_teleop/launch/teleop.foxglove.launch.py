import os
from launch import Condition, LaunchDescription
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
    # Arguments
    use_darknet_arg = DeclareLaunchArgument(
        'use_darknet', default_value='false', description='Use darknet or not')
    use_audio_arg = DeclareLaunchArgument(
        'use_audio', default_value='false', description='Use audio or not')

    exp_arg = DeclareLaunchArgument(
        'exp', default_value='false', description='Exploration mode')
    vsting_arg = DeclareLaunchArgument(
        'vsting', default_value='false', description='vsting mode')

    teleop_yaml_file = get_file_path('r5sr_teleop', 'config/r5sr_teleop.yaml')

    # Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[teleop_yaml_file],
    )
    joy2command_node = Node(
        package='r5sr_teleop',
        executable='joy2command_node',
        parameters=[teleop_yaml_file],
        remappings=[
            ('delta_twist_cmds', '/servo_node/delta_twist_cmds'),
            ('joint_jog', '/servo_node/delta_joint_cmds'),
            ('joint_jog_overhead', '/overhead_servo_node/delta_joint_cmds'),
            ('flipper_joint_trajectory', '/flipper_controller/joint_trajectory'),
            ('flippers', 'flippers_rpm'),
        ],
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[teleop_yaml_file]
    )

    flir_node = Node(
        package='r5sr_teleop',
        executable='flir_ax8_rtsp',
        namespace='thermo',
        parameters=[teleop_yaml_file],
    )

    hazmat_label_yaml_file = get_file_path(
        'r5sr_teleop', 'config/yolo/robocup-hazmat-label-2022.yaml')

    yolo_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_yolo')),
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='vision_front_repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'hand_camera/image_raw/compressed'),
                        ('out', 'hand_camera/image_raw/uncompressed'),
                ],
            ),
            Node(
                package='r5sr_meter_inspection',
                executable='image_converter_node',
                name='image_converter_node',
                remappings=[
                        ('/image_raw', 'hand_camera/image_raw/uncompressed'),
                        ('/image_processed', 'hand_camera/image_processed'),
                ],
            ),
            
           Node(
                package='r5sr_meter_inspection',
                executable='meter_value_calculator_node',
                name='meter_value_calculator_node',
                remappings=[
                        ('/image_processed', 'hand_camera/image_processed'),
                ],
            ),

            Node(
                package='r5sr_meter_inspection',
                executable='inspection_result_node',
                name='inspection_result_node'
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('r5sr_meter_inspection', 'launch/yolov8.launch.py')]),
                launch_arguments={'input_image_topic': 'hand_camera/image_processed'}.items(),
            ),
        ],
    )

    audio_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_audio')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('audio_play', 'launch/play.launch.xml')]),
                launch_arguments={'ns': 'audio_robot_to_ope'}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('audio_capture', 'launch/capture.launch.xml')]),
                launch_arguments={'ns': 'audio_ope_to_robot'}.items(),
            ),
        ],
    )

    cloud_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('r5sr_teleop', 'launch/cloud.launch.py')]
                )
    )

    dronecam_yaml_file = get_file_path('r5sr_teleop', 'config/drone_cam.yaml')
    dronecam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="drone_camera",
        namespace="drone_camera",
        parameters=[dronecam_yaml_file]
        )

    return LaunchDescription(
        [
            use_darknet_arg,
            use_audio_arg,
            exp_arg,
            vsting_arg,

            joy_node,
            joy2command_node,
            foxglove_node,
            flir_node,

            darknet_group,
            audio_group,

            cloud_launch,
            dronecam_node,
        ]
    )
