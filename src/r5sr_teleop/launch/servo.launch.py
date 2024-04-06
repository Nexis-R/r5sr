import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("r5sr")
        .robot_description(file_path="config/r5sr.urdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("r5sr_moveit_teleop",
                           "config/arm_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    overhead_servo_yaml = load_yaml("r5sr_moveit_teleop",
                           "config/overhead_arm_servo.yaml")
    overhead_servo_params = {"moveit_servo": overhead_servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="overhead_servo_node",
                parameters=[
                    overhead_servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            ),
            ComposableNode(
                package="r5sr_moveit_teleop",
                plugin="r5sr_moveit_teleop::PresetPose",
                name="preset_pose_node",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            container,
        ]
    )
