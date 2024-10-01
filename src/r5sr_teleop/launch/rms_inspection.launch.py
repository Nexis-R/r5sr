import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import ExecuteProcess, GroupAction

def get_file_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)

def load_file(package_name, file_path):
    absolute_file_path = get_file_path(package_name, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def generate_launch_description():
    # パッケージディレクトリのパスを取得
    rms_ros2_client_dir = get_package_share_directory('rms_ros2_client')
    
    # Node
    repub_node = Node(
        package='image_transport',
        executable='republish',
        name='repub_node',
        arguments=['compressed', 'raw'],
        remappings=[
                ('in/compressed', '/vision_front_camera/image_raw/compressed'),
                ('out', '/vision_front_camera/image_raw/uncompressed'),
        ],
    )

    # QRコード認識ノード
    qr_detector_node = Node(
        package='rms_ros2_client',
        executable='qr_detector_node',
        name='qr_detector_node',
        arguments=['compressed', 'raw'],
        remappings=[
            ('/image_raw', '/vision_front_camera/image_raw/uncompressed'),
            ('/qrcode_info', '/vision_front_camera/qrcode_info'),
        ]
    )

    # スナップショットノード
    image_snapshot_publisher_node = Node(
        package='rms_ros2_client',
        executable='image_snapshot_publisher_node',
        name='image_snapshot_publisher_node',
        arguments=['compressed', 'raw'],
        remappings=[
            ('/image_raw', '/vision_front_camera/meter_inspection_image'),
            ('/image_snap_shot', '/vision_front_camera/image_snap_shot'),
            ('/compress_and_publish_image', '/vision_front_camera/compress_and_publish_image'),
        ],
        parameters=[
            {'save_directory': os.path.join(rms_ros2_client_dir, 'snaps')},
            {'file_name': 'result.jpg'}
        ]
    )

    input_to_recognition_value_node = Node(
        package='rms_ros2_client',
        executable='input_to_recognition_value_node',
        name='input_to_recognition_value_node',
        remappings=[
            ('/input_string', '/vision_front_camera/input_string'), 
            ('/recognition_value', '/vision_front_camera/recognition_value'),
        ]
    )

    # 点検結果報告ノード
    topic_relay_maincam_node = Node(
        package='topic_tools',
        executable='relay',
        name='topic_relay_maincam',
        parameters=[
            {
                'input_topic': '/vision_front_camera/image_raw/uncompressed',
                'output_topic': '/vision_front_camera/meter_inspection_image'
            }
        ]
    )

    # RMSクラウドノード
    rms_ros_client_eqpt_updater = Node(
        package='rms_ros2_client',
        executable='rms_ros2_client_eqpt_updater',
        name='rms_ros2_client_eqpt_updater',
        remappings=[
            ('/qrcode_info', '/vision_front_camera/qrcode_info'),
            ('/recognition_value', '/vision_front_camera/recognition_value'),
            ('/update_result', '/vision_front_camera/update_result'),
            ('/send_request_service', '/vision_front_camera/send_request_service'),
        ],
        parameters=[
            {'ip': '52.193.111.81'},
            {'robot_id': 16},
            {'image_path': os.path.join(rms_ros2_client_dir, 'snaps/result.jpg')}
        ]
    )

    return LaunchDescription([
        repub_node,
        qr_detector_node,
        image_snapshot_publisher_node,
        rms_ros_client_eqpt_updater,
        input_to_recognition_value_node,
        topic_relay_maincam_node,
    ])
