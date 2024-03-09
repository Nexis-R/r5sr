import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class ImageConverterNode(Node):
    def __init__(self):
        super().__init__('image_converter_node')
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

        # 画像を正方形に加工
        height, width, _ = cv_image.shape
        min_edge = min(height, width)
        top = int((height - min_edge) / 2)
        left = int((width - min_edge) / 2)
        square_image = cv_image[top:top+min_edge, left:left+min_edge]

        # ホワイトバランスの自動調整
        def adjust_white_balance(image):
            result = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            avg_a = np.average(result[:, :, 1])
            avg_b = np.average(result[:, :, 2])
            result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
            result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
            return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)

        white_balanced_image = adjust_white_balance(square_image)

        # 加工した画像をROSのImageメッセージに変換してパブリッシュ
        image_message = self.bridge.cv2_to_imgmsg(white_balanced_image, encoding="bgr8")
        self.publisher.publish(image_message)


def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
