#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class ImageSnapshotPublisherNode(Node):
    def __init__(self):
        super().__init__('image_snapshot_publisher_node')

        # サブスクライブするトピック
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # 圧縮画像をパブリッシュするためのパブリッシャー
        self.compressed_publisher_ = self.create_publisher(
            CompressedImage,
            '/image_snap_shot',
            10)

        # サービスの作成
        self.service = self.create_service(
            Trigger,
            '/compress_and_publish_image',
            self.handle_compress_and_publish)

        # CVBridgeのインスタンス作成
        self.bridge = CvBridge()

        # 最新の画像を保存するための変数
        self.last_image = None

        # 画像を保存するディレクトリを設定
        try:
            default_save_directory = os.path.join(get_package_share_directory('r5sr_meter_inspection'), 'snaps')
        except Exception as e:
            self.get_logger().error(f"Error getting package path: {str(e)}")
            default_save_directory = './snaps'
        
        self.save_directory = self.declare_parameter('save_directory', default_save_directory).get_parameter_value().string_value
        self.file_name = self.declare_parameter('file_name', 'cam1.jpg').get_parameter_value().string_value

        # 保存先ディレクトリが存在しない場合は作成
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

    def handle_compress_and_publish(self, request, response):
        if self.last_image is not None:
            self.get_logger().info('Image found, proceeding to save, compress, and publish.')
            try:
                # ROS ImageメッセージをOpenCV画像に変換
                cv_image = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")

                # 保存するファイル名を生成
                timestamp = self.get_clock().now().to_msg().sec
                filename = self.file_name.format(timestamp=timestamp)
                
                # 画像を保存
                filepath = os.path.join(self.save_directory, filename)
                cv2.imwrite(filepath, cv_image)
                self.get_logger().info(f"Image saved: {filename}")
                
                # JPEG形式で画像を圧縮
                compress_success, buffer = cv2.imencode('.jpg', cv_image)
                if compress_success:
                    compressed_image_msg = CompressedImage()
                    compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
                    compressed_image_msg.format = "jpeg"
                    compressed_image_msg.data = buffer.tobytes()

                    # 圧縮された画像をパブリッシュ
                    self.compressed_publisher_.publish(compressed_image_msg)
                    self.get_logger().info('Image compressed and published successfully.')
                    
                    response.success = True
                    response.message = "Image saved, compressed, and published successfully."
                else:
                    self.get_logger().error('Failed to compress image.')
                    response.success = False
                    response.message = "Failed to compress image."
            except Exception as e:
                self.get_logger().error(f'Error during image processing: {e}')
                response.success = False
                response.message = f"Error during image processing: {e}"
        else:
            self.get_logger().warn('No image available to save, compress, or publish.')
            response.success = False
            response.message = "No image available to save, compress, or publish."

        return response

    def image_callback(self, msg):
        try:
            self.last_image = msg  # 最新の画像を保存
        except Exception as e:
            self.get_logger().error(f'Error during image storage: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSnapshotPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
