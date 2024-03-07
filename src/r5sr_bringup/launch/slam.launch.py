import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            get_package_share_directory('r5sr_bringup'),
            'config',
            'mapping.yaml'))

    mid360_param_dir = launch.substitutions.LaunchConfiguration(
        'user_config_path',
        default=os.path.join(
            get_package_share_directory('r5sr_bringup'),
            'config',
            'MID360_config.json'))

    livox_driver = launch_ros.actions.Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_ros_driver2',
        parameters=[mapping_param_dir,
                    {'user_config_path': mid360_param_dir}],
    )

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[mapping_param_dir],
        remappings=[('/input_cloud', '/livox/lidar'), ('/imu', '/livox/imu')],
        output='screen'
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[mapping_param_dir],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        launch.actions.DeclareLaunchArgument(
            'user_config_path',
            default_value=mid360_param_dir,
            description='Full path to livox parameter file to load'),
        livox_driver,
        mapping,
        graphbasedslam,
    ])
