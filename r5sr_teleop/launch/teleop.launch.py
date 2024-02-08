import os
import yaml
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
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


def rviz_opaque_function(context):
    exp_arg_value = LaunchConfiguration('exp').perform(context)
    rviz_file_path = get_file_path('r5sr_teleop', 'rviz/r5sr_teleop_exp.rviz') if exp_arg_value == 'true' else get_file_path('r5sr_teleop', 'rviz/r5sr_teleop.rviz')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_file_path('r5sr_teleop', 'launch/rviz_moveit.launch.py')]),
            launch_arguments={'rviz_config_file': rviz_file_path}.items(),
        ),
    ]

def rqt_opaque_function(context):
    exp_arg_value = LaunchConfiguration('exp').perform(context)
    vsting_arg_value = LaunchConfiguration('vsting').perform(context)

    if exp_arg_value == 'true' and vsting_arg_value == 'false':
        rqt_file_path = get_file_path('r5sr_teleop', 'rqt/r5sr_cam_exp.perspective')
    elif exp_arg_value == 'false' and vsting_arg_value == 'true':
        rqt_file_path = get_file_path('r5sr_teleop', 'rqt/r5sr_cam_vsting.perspective')
    else :
        rqt_file_path = get_file_path('r5sr_teleop', 'rqt/r5sr_cam.perspective')

    return [
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            output='screen',
            arguments=[
                '--clear-config',
                '--perspective-file', rqt_file_path
            ],
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui_front',
            output='screen',
            arguments=[
                '--clear-config',
                '--perspective-file', get_file_path('r5sr_teleop', 'rqt/vision_front.perspective')
            ],
        ),
    ]

def generate_launch_description():
    # Arguments
    use_darknet_arg = DeclareLaunchArgument('use_darknet', default_value='false', description='Use darknet or not')
    use_audio_arg = DeclareLaunchArgument('use_audio', default_value='false', description='Use audio or not')

    exp_arg = DeclareLaunchArgument('exp', default_value='false', description='Exploration mode') 
    vsting_arg = DeclareLaunchArgument('vsting', default_value='false', description='vsting mode')

    teleop_yaml_file = get_file_path('r5sr_teleop', 'config/r5sr_teleop.yaml')

    # Nodes
    base_control_container = ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='teleop_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='joy',
                    plugin='joy::Joy',
                    name='joy',
                    parameters=[teleop_yaml_file],
                ),
                ComposableNode(
                    package='r5sr_teleop',
                    plugin='r5sr_teleop::Joy2Command',
                    name='joy2command',
                    remappings=[
                        ('delta_twist_cmds', '/servo_node/delta_twist_cmds'),
                        ('joint_jog', '/servo_node/delta_joint_cmds'),
                        ('joint_jog_overhead', '/overhead_servo_node/delta_joint_cmds'),
                        ('flipper_joint_trajectory', '/flipper_controller/joint_trajectory'),
                        ('flippers', 'flippers_rpm'),
                    ],
                ),
                ComposableNode(
                    package='r5sr_crawler_control',
                    plugin='crawler_control::Crawler_Control',
                    name='crawler_control',
                    parameters=[teleop_yaml_file],
                    remappings=[
                        ('/command_crawler_left', '/epos/motor4/move_with_velocity'),
                        ('/command_crawler_right', '/epos/motor5/move_with_velocity'),
                        ('/command_flipper_left_front', '/epos/motor7/move_to_position'),
                        ('/command_flipper_right_front', '/epos/motor6/move_to_position'),
                        ('/command_flipper_left_back', '/epos/motor3/move_to_position'),
                        ('/command_flipper_right_back', '/epos/motor2/move_to_position'),
                    ],
                ),
            ],
        )

    battery_alert_node = Node(
            package='r5sr_teleop',
            executable='alert_battery',
        )

    flir_node = Node(
            package='r5sr_teleop',
            executable='flir_ax8_rtsp',
            parameters=[teleop_yaml_file],
        )

    hazmat_label_yaml_file = get_file_path('r5sr_teleop', 'config/yolo/robocup-hazmat-label-2022.yaml')

    darknet_group = GroupAction(
            condition=IfCondition(LaunchConfiguration('use_darknet')),
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
                        {'config_path':get_file_path('r5sr_teleop', 'config/yolo/cfg')},
                        {'weights_path':get_file_path('r5sr_teleop', 'config/yolo/weights')},
                    ],
                ),
            ],
        )
    
    audio_group = GroupAction(
            condition=IfCondition(LaunchConfiguration('use_audio')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([get_file_path('audio_play','launch/play.launch.xml')]),
                    launch_arguments={'ns': 'audio_robot_to_ope'}.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([get_file_path('audio_capture', 'launch/capture.launch.xml')]),
                    launch_arguments={'ns': 'audio_ope_to_robot'}.items(),
                ),
            ],
        )
    
    servo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_file_path('r5sr_teleop', 'launch/servo.launch.py')]),
        )
    
    return LaunchDescription(
        [
            use_darknet_arg,
            use_audio_arg,
            exp_arg,
            vsting_arg,

            base_control_container,
            battery_alert_node,
            flir_node,
            darknet_group,
            audio_group,

            servo_launch,

            OpaqueFunction(function=rqt_opaque_function),
            OpaqueFunction(function=rviz_opaque_function),
        ]
    )