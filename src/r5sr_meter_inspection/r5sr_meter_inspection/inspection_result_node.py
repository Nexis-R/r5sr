import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
from example_interfaces.srv import Trigger

class InspectionResultNode(Node):
    def __init__(self):
        super().__init__('inspection_result_node')

        self.image_subscription = self.create_subscription(
            Image,
            '/yolo/dbg_image',
            self.image_callback,
            10)
        
        self.result_subscription = self.create_subscription(
            Float32,
            '/meter_value',
            self.result_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            Image, 
            '/meter_inspection_image', 
            10)
        
        self.compressed_publisher_ = self.create_publisher(
            CompressedImage,
            '/meter_inspection_image/compressed',
            10)
        
        self.service = self.create_service(
            Trigger,
            '/compress_and_publish_image',
            self.handle_compress_and_publish)
        
        self.result_value = None
        self.bridge = CvBridge()
        self.last_image = None

    def handle_compress_and_publish(self, request, response):
        if self.last_image is not None:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")
                # Compress the image using JPEG format
                compress_success, buffer = cv2.imencode('.jpg', cv_image)
                if compress_success:
                    compressed_image_msg = CompressedImage()
                    compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
                    compressed_image_msg.format = "jpeg"
                    compressed_image_msg.data = buffer.tobytes()
                    # Publish the compressed image
                    self.compressed_publisher_.publish(compressed_image_msg)
                    response.success = True
                    response.message = "Image compressed and published successfully."
                else:
                    response.success = False
                    response.message = "Failed to compress image."
            except CvBridgeError as e:
                self.get_logger().error('CvBridge Error: {}'.format(e))
                response.success = False
                response.message = "CvBridge Error: {}".format(e)
        else:
            response.success = False
            response.message = "No image available to compress and publish."
        
        return response
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.result_value is not None:
            formatted_value = "Meter: {:.3f}".format(self.result_value)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_color = (0, 0, 255)  # テキストの色：赤
            background_color = (255, 255, 255)  # 背景の色：白
            thickness = 2

            # 画像のサイズを取得
            (img_height, img_width) = cv_image.shape[:2]

            # テキストのサイズとベースラインを取得
            (text_width, text_height), baseline = cv2.getTextSize(formatted_value, font, font_scale, thickness)

            # テキストの横位置を画面の中央に設定
            x = img_width // 2 - text_width // 2
            # テキストの縦位置を画面の上部に設定（例：画面高さの5%）
            y = int(img_height * 0.1) + text_height // 2

            # 背景の長方形を描画
            cv2.rectangle(cv_image, (x - 5, y - text_height - 15), (x + text_width + 5, y + baseline - 5), background_color, -1)

            # テキストを描画
            cv2.putText(cv_image, formatted_value, (x, y), font, font_scale, font_color, thickness)

        # 変更された画像をパブリッシュ
        try:
            new_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.last_image = new_msg
            self.publisher_.publish(new_msg)
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {}'.format(e))

    def result_callback(self, msg):
        self.result_value = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = InspectionResultNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
