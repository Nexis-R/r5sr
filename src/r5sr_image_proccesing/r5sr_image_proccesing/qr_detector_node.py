import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageConverterNode(Node):
    def __init__(self):
        super().__init__('image_converter_node')
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

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
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # QRコードの検出とデコード
        try:
            data, bbox, _ = self.qr_detector.detectAndDecode(cv_image)
            if bbox is not None and len(bbox) > 0:
                # Ensure the bounding box has valid dimensions
                if cv2.contourArea(bbox) > 0:
                    # テキストのサイズを取得して、中心位置を計算
                    text_size = cv2.getTextSize(data, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
                    text_width = text_size[0][0]
                    text_height = text_size[0][1]

                    # 画像の幅と高さから中心位置を計算
                    height, width = cv_image.shape[:2]
                    text_org_x = int((width - text_width) / 2)
                    text_org_y = int(height / 12)  # 上部から1/12の位置

                    # 白いテキストで背景を描画（少しオフセットをつけて）
                    cv2.putText(cv_image, data, (text_org_x, text_org_y + text_height + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    # 緑のテキストを描画
                    cv2.putText(cv_image, data, (text_org_x, text_org_y + text_height + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    pass
                    # self.get_logger().warn('Detected QR code contour has invalid area, skipping.')
            else:
                pass
                # self.get_logger().info('No QR code detected.')
        except cv2.error as e:
            self.get_logger().error(f'CV error during QR code detection: {str(e)}')

        # 加工後の画像をROSのImageメッセージに変換してパブリッシュ
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
