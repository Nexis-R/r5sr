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
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/tf', '/tf_static',
                 '/inspection_main_camera/image/compressed',
                 '/inspection_sub_camera/image/compressed',
                 ]
        )
    ])
