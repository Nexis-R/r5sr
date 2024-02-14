import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_file_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)


def camera_opaque_function(context):
    vsting_arg_value = LaunchConfiguration("vsting").perform(context)

    if vsting_arg_value == "true":
        camera_yaml_file = get_file_path(
            "r5sr_bringup", "config/r5sr_camera_vsting.yaml"
        )
    else:
        camera_yaml_file = get_file_path(
            "r5sr_bringup", "config/r5sr_camera.yaml")

    camera_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_camera")),
        actions=[
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="vision_front_camera",
                namespace="vision_front_camera",
                parameters=[camera_yaml_file],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="vision_rear_camera",
                namespace="vision_rear_camera",
                parameters=[camera_yaml_file],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="fisheye_front_camera",
                namespace="fisheye_front_camera",
                parameters=[camera_yaml_file],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="hand_camera",
                namespace="hand_camera",
                parameters=[camera_yaml_file],
            ),
            Node(
                package="zbar_ros",
                executable="barcode_reader",
                remappings=[("image", "/vision_front_camera/image_raw")],
            ),
        ],
    )

    return [camera_group]


def generate_launch_description():
    # Arguments
    use_camera_arg = DeclareLaunchArgument(
        "use_camera", default_value="true", description="Use camera or not"
    )
    use_audio_arg = DeclareLaunchArgument(
        "use_audio", default_value="true", description="Use audio or not"
    )
    use_slam_arg = DeclareLaunchArgument(
        "use_slam", default_value="true", description="Use slam or not"
    )
    use_rplidar_arg = DeclareLaunchArgument(
        "use_rplidar", default_value="true", description="Use rplidar or not"
    )

    exp_arg = DeclareLaunchArgument(
        "exp", default_value="false", description="Exploration mode"
    )
    vsting_arg = DeclareLaunchArgument(
        "vsting", default_value="false", description="vsting mode"
    )

    bringup_yaml_file = get_file_path(
        "r5sr_bringup", "config/r5sr_bringup.yaml")

    # Nodes
    epos_node = Node(
        package="epos",
        executable="epos",
        parameters=[bringup_yaml_file],
    )
    move_with_jointstate_node = Node(
        package="r5sr_manipulator_control",
        executable="move_with_jointstate",
    )

    rplidar_node = Node(
        package="rplidar_ros2",
        executable="rplidar_scan_publisher",
        name="rplidar",
        parameters=[bringup_yaml_file],
    )

    audio_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_audio")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_file_path("audio_capture", "launch/capture.launch.py")
                ),
                launch_arguments=[("ns", "audio_robot_to_ope")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_file_path("audio_play", "launch/play.launch.py")
                ),
                launch_arguments=[("ns", "audio_ope_to_robot")],
            ),
        ],
    )

    slam_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_slam")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_file_path("realsense2_camera", "launch/rs_launch.py")
                ),
                launch_arguments=[
                    ('enable_gyro', 'true'),
                    ('enable_accel', 'true'),
                    ('unite_imu_method', '1'),
                    ('enable_sync', 'true'),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_file_path("r5sr_bringup",
                                  "launch/rtabmap_realsense.launch.py")
                ),
            ),
            # The IMU frame is mising in TF tree, add it:
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="log",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "camera_gyro_optical_frame",
                    "camera_imu_optical_frame",
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            use_camera_arg,
            use_audio_arg,
            use_slam_arg,
            use_rplidar_arg,
            exp_arg,
            vsting_arg,
            epos_node,
            move_with_jointstate_node,
            rplidar_node,
            audio_group,
            slam_group,
            OpaqueFunction(function=camera_opaque_function),
        ]
    )
