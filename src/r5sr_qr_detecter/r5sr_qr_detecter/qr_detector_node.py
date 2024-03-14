import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class ImageConverterNode(Node):
    def __init__(self):
        super().__init__('image_converter_node')
        self.declare_parameter('display_image', True)
        self.display_image = self.get_parameter('display_image').get_parameter_value().bool_value
        self.get_logger().info(f"Display image: {self.display_image}")

        self.bridge = CvBridge()

        self.subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            '/image_processed',
            10)

    def image_callback(self, msg):
        # ROSのImageメッセージをOpenCV形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        ## ここにcv_imageの映像を使ったQRコードの認識プログラムを作成する

        # 加工した画像をROSのImageメッセージに変換してパブリッシュ
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher.publish(image_message)


        # 画像の表示 Trueで表示　Falseで非表示
        if self.display_image:
            cv2.imshow('QR Detector', cv_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
