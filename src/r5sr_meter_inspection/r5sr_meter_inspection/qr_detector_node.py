import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class QrDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.last_qr_data = None  # 最初はNoneで初期化

        self.subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        # String publisher for QR code information
        self.qr_publisher = self.create_publisher(
            String,
            '/qrcode_info',
            10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # QRコードの検出とデコード
        try:
            data, bbox, _ = self.qr_detector.detectAndDecode(cv_image)
            if bbox is not None and len(bbox) > 0 and data:
                self.last_qr_data = data  # 新しいQRコードデータを保持

            # 前回のQRコードデータを発行（検出されていない場合は前回の値を使用）
            if self.last_qr_data is not None:
                qr_msg = String()
                qr_msg.data = self.last_qr_data
                self.qr_publisher.publish(qr_msg)

        except cv2.error as e:
            self.get_logger().error(f'CV error during QR code detection: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = QrDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
