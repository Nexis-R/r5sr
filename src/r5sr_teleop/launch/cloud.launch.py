import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = launch.substitutions.LaunchConfiguration(
        'config_dir',
        default=os.path.join(
            get_package_share_directory('r5sr_teleop'),
            'config',
            'rms_cloud.yaml'
        ))

    rms_plant = launch_ros.actions.Node(
        package='r5sr_cloud',
        executable='rms_plant_node',
        name='rms_plant',
        parameters=[config_dir],
        output='screen'
    )

    rms_robot = launch_ros.actions.Node(
        package='r5sr_cloud',
        executable='rms_robot_node',
        name='rms_robot',
        parameters=[config_dir],
        remappings=[('/camera2', '/vision_front_camera/image_raw/compressed'),
                    ('/camera1', '/meter_inspection_snapshot'),
                    ('/camera3', '/thermo/image_raw/compressed')],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'config_dir',
            default_value=config_dir,
            description='Full path to config file to load'),

        rms_plant,
        rms_robot
    ])
