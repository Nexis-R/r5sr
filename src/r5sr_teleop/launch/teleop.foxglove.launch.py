import os

from blinker import Namespace
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


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
    use_wrs_arg = DeclareLaunchArgument(
        "use_wrs", default_value="false", description='when wrs or not')
    use_audio_arg = DeclareLaunchArgument(
        "use_audio", default_value="false", description='Use audio or not')

    exp_arg = DeclareLaunchArgument(
        "exp", default_value="false", description='Exploration mode')
    vsting_arg = DeclareLaunchArgument(
        "vsting", default_value="false", description='vsting mode')

    teleop_yaml_file = get_file_path('r5sr_teleop', 'config/r5sr_teleop.yaml')

    # Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[teleop_yaml_file],
        remappings=[
            ('/joy', '/joy_raw'),
        ],
    )
    joy_throttle = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'throttle',
             'messages', 'joy_raw', '100.0', 'joy']
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

    emergency_stop_node = Node(
        package='r5sr_emergency_stop',
        executable='r5sr_emergency_stop_node',
        name='r5sr_emergency_stop_node'
    )


    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_file_path('r5sr_teleop', 'launch/servo.launch.py')]),
    )

    audio_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_audio')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('audio_play', 'launch/play.launch.py')]),
                launch_arguments={'ns': 'audio_robot_to_ope'}.items(),
            ),
            # audio_ope_to_robotを入れると音が聞こえなくなる
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [get_file_path('audio_capture', 'launch/capture.launch.py')]),
            #     launch_arguments={'ns': 'audio_ope_to_robot'}.items(),
            # ),
        ],
    )

    hand_yolo_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_wrs")),
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'hand_camera/image_raw/compressed'),
                        ('out', 'yolo/hand/image_raw/uncompressed'),
                ],
            ),
            Node(
                package='r5sr_meter_inspection',
                executable='image_converter_node',
                name='image_converter_node',
                namespace='yolo/hand',
                remappings=[
                        ('/image_raw', '/yolo/hand/image_raw/uncompressed'),
                        ('/image_processed', '/yolo/hand/image_processed'),
                ],
            ),

            Node(
                package='r5sr_meter_inspection',
                executable='meter_value_calculator_node',
                name='meter_value_calculator_node',
                namespace='yolo/hand',
                remappings=[
                        ('/image_processed', '/yolo/hand/image_processed'),
                        ('/meter_value', '/yolo/hand/meter_value'),
                        ('/detections', '/yolo/hand/detections'),
                ],
            ),

            Node(
                package='r5sr_meter_inspection',
                executable='inspection_result_node',
                name='inspection_result_node',
                namespace='yolo/hand',
                remappings=[
                        ('/yolo/dbg_image', '/yolo/hand/dbg_image'),
                        ('/meter_value', '/yolo/hand/meter_value'),
                        ('/meter_inspection_image', '/yolo/hand/meter_inspection_image'),
                        ('/meter_inspection_snapshot', '/yolo/meter_inspection_snapshot'),
                        ('/compress_and_publish_image', '/yolo/hand/compress_and_publish_image'),
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('r5sr_meter_inspection', 'launch/yolov8.launch.py')]),
                launch_arguments={'namespace':'yolo/hand',
                                  'input_image_topic': 'image_processed',
                                  'model': get_file_path('r5sr_teleop', 'config/yolo/gauge-analysis-wrs-trial-2024.pt'),
                                  'threshold': '0.5'}.items(),

            ),
        ],
    )

    vision_yolo_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_wrs")),
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'vision_front_camera/image_raw/compressed'),
                        ('out', 'yolo/vision/image_raw/uncompressed'),
                ],
            ),
            Node(
                package='r5sr_meter_inspection',
                executable='image_converter_node',
                name='image_converter_node',
                namespace='yolo/vision',
                remappings=[
                        ('/image_raw', '/yolo/vision/image_raw/uncompressed'),
                        ('/image_processed', '/yolo/vision/image_processed'),
                ],
            ),

            Node(
                package='r5sr_meter_inspection',
                executable='meter_value_calculator_node',
                name='meter_value_calculator_node',
                namespace='yolo/vision',
                remappings=[
                        ('/image_processed', '/yolo/vision/image_processed'),
                        ('/meter_value', '/yolo/vision/meter_value'),
                        ('/detections', '/yolo/vision/detections'),
                ],
            ),

            Node(
                package='r5sr_meter_inspection',
                executable='inspection_result_node',
                name='inspection_result_node',
                namespace='yolo/vision',
                remappings=[
                        ('/yolo/dbg_image', '/yolo/vision/dbg_image'),
                        ('/meter_value', '/yolo/vision/meter_value'),
                        ('/meter_inspection_image', '/yolo/vision/meter_inspection_image'),
                        ('/meter_inspection_snapshot', '/yolo/meter_inspection_snapshot'),
                        ('/compress_and_publish_image', '/yolo/vision/compress_and_publish_image'),
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('r5sr_meter_inspection', 'launch/yolov8.launch.py')]),
                launch_arguments={'namespace':'yolo/vision',
                                  'input_image_topic': 'image_processed',
                                  'model': get_file_path('r5sr_teleop', 'config/yolo/gauge-analysis-wrs-trial-2024.pt'),
                                  'threshold': '0.5'}.items(),

            ),
        ],
    )

    cloud_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_wrs")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('r5sr_teleop', 'launch/cloud.launch.py')]),
            ),
        ],
    )

    dronecam_yaml_file = get_file_path('r5sr_teleop', 'config/drone_cam.yaml')

    drone_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_wrs")),
        actions=[
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="drone_camera",
                namespace="drone_camera",
                parameters=[dronecam_yaml_file]
            ),
            Node(
                package='r5sr_meter_inspection',
                executable='image_converter_node',
                name='drone_image_converter',
                remappings=[
                        ('/image_raw', '/drone_camera/image_raw'),
                        ('/image_processed', '/drone_camera/image_processed'),
                ],
            ),
        ]
    )

    hazmat_group = GroupAction(
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'hand_camera/image_raw/compressed'),
                        ('out', 'yolo/hand/image_raw/uncompressed'),
                ],
            ),

            Node(
                package='r5sr_meter_inspection',
                executable='image_converter_node',
                name='image_converter_node',
                namespace='yolo/vision',
                remappings=[
                        ('/image_raw', '/yolo/hand/image_raw/uncompressed'),
                        ('/image_processed', '/yolo/hand/image_processed'),
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_file_path('yolov8_bringup', 'launch/yolov8.launch.py')]),
                launch_arguments={'namespace':'yolo/hand',
                                  'input_image_topic': 'image_processed',
                                  'model': get_file_path('r5sr_teleop', 'config/yolo/hazmat2024-0421-yolov8m.pt'),
                                  'threshold': '0.9'}.items(),

            ),
        ]
    )

    hand_qr_detector_group = GroupAction(
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
                executable='qr_detector_node',
                name='qr_detector_node',
                remappings=[
                        ('image_raw', 'hand_camera/image_raw/uncompressed'),
                        ('image_processed', 'hand_camera/image_raw/qr_detector_image'),
                ],
            ),
        ]
    )

    vision_qr_detector_group = GroupAction(
        actions=[
            Node(
                package='image_transport',
                executable='republish',
                name='repub',
                arguments=['compressed', 'raw'],
                remappings=[
                        ('in/compressed', 'vision_front_camera/image_raw/compressed'),
                        ('out', 'vision_front_camera/image_raw/uncompressed'),
                ],
            ),
            Node(
                package='r5sr_image_proccesing',
                executable='qr_detector_node',
                name='qr_detector_node',
                remappings=[
                        ('image_raw', 'vision_front_camera/image_raw/uncompressed'),
                        ('image_processed', 'vision_front_camera/image_raw/qr_detector_image'),
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            use_wrs_arg,
            use_audio_arg,
            exp_arg,
            vsting_arg,

            joy_node,
            joy_throttle,
            joy2command_node,
            foxglove_node,
            flir_node,
            emergency_stop_node,

            servo_launch,

            hand_yolo_group,
            vision_yolo_group,
            audio_group,

            hazmat_group,

            cloud_group,
            drone_group,

            hand_qr_detector_group,
            vision_qr_detector_group,
            
        ]
    )
