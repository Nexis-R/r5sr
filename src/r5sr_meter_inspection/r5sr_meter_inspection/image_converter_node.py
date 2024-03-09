import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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

        # HSV色空間に変換
        hsv_image = cv2.cvtColor(square_image, cv2.COLOR_BGR2HSV)

        # CLAHEをVチャンネルに適用
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        hsv_image[:, :, 2] = clahe.apply(hsv_image[:, :, 2])

        # HSVからBGRに変換
        contrast_adjusted_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        # 加工した画像をROSのImageメッセージに変換してパブリッシュ
        image_message = self.bridge.cv2_to_imgmsg(contrast_adjusted_image, encoding="bgr8")
        self.publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
