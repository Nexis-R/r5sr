import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import collections

class ImageMotionDetectNode(Node):
    def __init__(self):
        super().__init__('image_motion_detect_node')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.frame_rate = 30  # 1秒間のフレーム数、適宜調整してください
        self.previous_images = collections.deque(maxlen=self.frame_rate)  # 1秒間の画像を保存するキュー
        self.area_threshold = 10  # 動きを検出するための面積のしきい値

    def listener_callback(self, msg):
        current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        display_image = current_image.copy()
        text = "No Motion."

        # 1秒前の画像が存在するか確認
        if len(self.previous_images) >= self.frame_rate:
            one_second_ago_image = self.previous_images[0]  # 1秒前の画像を取得

            # 映像の差分を計算
            diff = cv2.absdiff(one_second_ago_image, current_image)
            gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 面積がしきい値以上の輪郭を検出
            for contour in contours:
                if cv2.contourArea(contour) > self.area_threshold:
                    text = "Motion Detected!"
                    break

        # 画像にテキストを追加
        cv2.putText(display_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 現在の画像をキューに追加
        self.previous_images.append(current_image)

        # 画像をウィンドウに表示
        cv2.imshow('Motion Detection', display_image)
        cv2.waitKey(1)  # 少し待機してウィンドウが更新されるのを確認

def main(args=None):
    rclpy.init(args=args)
    node = ImageMotionDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # OpenCVのウィンドウをすべて閉じる

if __name__ == '__main__':
    main()
