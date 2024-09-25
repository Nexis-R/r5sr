import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger
import cv2

class SnapshotPublisherNode(Node):
    def __init__(self):
        super().__init__('snapshot_publisher_node')

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

        self.bridge = CvBridge()
        self.last_image = None

    def handle_compress_and_publish(self, request, response):
        if self.last_image is not None:
            self.get_logger().info('Image found, proceeding to compress and publish.')
            try:
                # ROS ImageメッセージをOpenCV画像に変換
                cv_image = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")
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
                    response.message = "Image compressed and published successfully."
                else:
                    self.get_logger().error('Failed to compress image.')
                    response.success = False
                    response.message = "Failed to compress image."
            except Exception as e:
                self.get_logger().error(f'Error during image compression: {e}')
                response.success = False
                response.message = f"Error during image compression: {e}"
        else:
            self.get_logger().warn('No image available to compress and publish.')
            response.success = False
            response.message = "No image available to compress and publish."

        return response

    def image_callback(self, msg):
        # 画像が届いたことをログ出力
        # self.get_logger().info(f'Received new image with timestamp: {msg.header.stamp}')
        try:
            self.last_image = msg  # 最新の画像を保存
        except Exception as e:
            self.get_logger().error(f'Error during image storage: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SnapshotPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
