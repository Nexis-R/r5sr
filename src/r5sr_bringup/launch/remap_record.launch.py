import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_maincam',
            parameters=[
                {
                    'input_topic': '/vision_front_camera/image_raw/compressed',
                    'output_topic': '/inspection_main_camera/image/compressed'
                }
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_subcam',
            parameters=[
                {
                    'input_topic': '/hand_camera/image_raw/compressed',
                    'output_topic': '/inspection_sub_camera/image/compressed'
                }
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_chassiscam',
            parameters=[
                {
                    'input_topic': '/camera/color/image_raw/compressed',
                    'output_topic': '/chassis_main_camera/image/compressed'
                }
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_chassissubcam',
            parameters=[
                {
                    'input_topic': '/vision_rear_camera/image_raw/compressed',
                    'output_topic': '/chassis_sub_camera/image/compressed'
                }
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_thermocam',
            parameters=[
                {
                    'input_topic': '/thermo/image_raw/compressed',
                    'output_topic': '/thermo_camera/image/compressed'
                }
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_map',
            parameters=[
                {
                    'input_topic': '/map',
                    'output_topic': '/cloud_1'
                }
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='topic_relay_lidar',
            parameters=[
                {
                    'input_topic': '/livox/lidar',
                    'output_topic': '/cloud_2'
                }
            ]
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/tf', '/tf_static',
                 '/inspection_main_camera/image/compressed',
                 '/inspection_sub_camera/image/compressed',
                 '/chassis_main_camera/image/compressed',
                 '/chassis_sub_camera/image/compressed',
                 '/thermo_camera/image/compressed',
                 '/cloud_1', '/cloud_2']
        )
    ])
