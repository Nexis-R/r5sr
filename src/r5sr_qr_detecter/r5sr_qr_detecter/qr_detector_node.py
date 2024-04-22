import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageConverterNode(Node):
    def __init__(self):
        super().__init__('image_converter_node')
        # パラメータを宣言し、ノードが画像を表示するかどうかを設定
        self.declare_parameter('display_image', False)
        self.display_image = self.get_parameter('display_image').get_parameter_value().bool_value
        self.get_logger().info(f"Display image: {self.display_image}")

        self.bridge = CvBridge()  # CvBridgeのインスタンス作成
        self.qr_detector = cv2.QRCodeDetector()  # QRコード検出器の初期化

        # ROSのSubscriber設定：生の画像データをサブスクライブ
        self.subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        # ROSのPublisher設定：処理後の画像データをパブリッシュ
        self.publisher = self.create_publisher(
            Image,
            '/image_processed',
            10)

    def image_callback(self, msg):
        # ROSのImageメッセージをOpenCV形式の画像に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # QRコードの検出とデコード
        data, bbox, _ = self.qr_detector.detectAndDecode(cv_image)
        if bbox is not None and data:
            # QRコードが検出された場合、境界ボックスとデコードされたテキストを画像に描画
            bbox = np.int32(bbox)
            for i in range(len(bbox)):
                cv2.line(cv_image, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]), color=(255, 0, 0), thickness=2)
            cv2.putText(cv_image, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 1, cv2.LINE_AA)

        # 加工後の画像をROSのImageメッセージに変換してパブリッシュ
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher.publish(image_message)

        # パラメータに応じて画像を表示
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
