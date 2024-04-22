import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from yolov8_msgs.msg import DetectionArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import pyocr
import pyocr.builders
import re
from PIL import Image as PILImage

class MeterValueCalculatorNode(Node):
    def __init__(self):
        super().__init__('meter_value_calculator_node')
        self.subscription = self.create_subscription(
            DetectionArray,
            '/detections',
            self.detection_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            '/image_processed',
            self.image_callback,
            10)

        self.publisher_ = self.create_publisher(
            Float32, 
            'meter_value',
            10)
        
        self.bridge = CvBridge()
        
        self.maximum_bbox = None
        
        tools = pyocr.get_available_tools()
        if len(tools) == 0:
            self.ocr_tool = None
        else:
            self.ocr_tool = tools[0]

        self.last_decimal_number = None
        self.last_mapped_value = None

    def image_callback(self, msg):
        if self.maximum_bbox is None or self.ocr_tool is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        bbox = self.maximum_bbox
        x_min = int(bbox.center.position.x - bbox.size.x / 2)
        y_min = int(bbox.center.position.y - bbox.size.y / 2)
        x_max = int(bbox.center.position.x + bbox.size.x / 2)
        y_max = int(bbox.center.position.y + bbox.size.y / 2)

        cropped_image = cv_image[y_min:y_max, x_min:x_max]
        gray_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        _, binary_cropped_image = cv2.threshold(gray_cropped_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        pil_binary_cropped_image = PILImage.fromarray(binary_cropped_image)

        ocr_result = self.ocr_tool.image_to_string(
            pil_binary_cropped_image,
            lang='eng',
            builder=pyocr.builders.TextBuilder()
        )

        decimal_numbers = re.findall(r'[0-9]+(?:\.[0-9]+)?', ocr_result)
        if not decimal_numbers:
            if self.last_decimal_number is not None:
                decimal_numbers = [self.last_decimal_number]
            # else:
            #     self.get_logger().info('No numbers found and no last number to fallback.')
        else:
            processed_numbers = []
            for number in decimal_numbers:
                if number.startswith('0') and len(number) > 1 and not number.startswith('0.'):
                    number = '0.' + number[1:]
                processed_numbers.append(number)
            
            decimal_numbers = processed_numbers
            if decimal_numbers:
                self.last_decimal_number = float(decimal_numbers[-1])

    def detection_callback(self, msg):
        minimum_x = minimum_y = maximum_x = maximum_y = base_x = base_y = tip_x = tip_y = 0.0

        for detection in msg.detections:
            if detection.class_name == 'minimum':
                minimum_x = detection.bbox.center.position.x
                minimum_y = detection.bbox.center.position.y
            elif detection.class_name == 'maximum':
                maximum_x = detection.bbox.center.position.x
                maximum_y = detection.bbox.center.position.y
                self.maximum_bbox = detection.bbox
            elif detection.class_name == 'base':
                base_x = detection.bbox.center.position.x
                base_y = detection.bbox.center.position.y
            elif detection.class_name == 'tip':
                tip_x = detection.bbox.center.position.x
                tip_y = detection.bbox.center.position.y

        if all(value != 0.0 for value in [minimum_x, minimum_y, maximum_x, maximum_y, base_x, base_y, tip_x, tip_y]):
            angle1 = self.calculate_angle(base_x, base_y, minimum_x, minimum_y, maximum_x, maximum_y)
            angle2 = self.calculate_angle(base_x, base_y, minimum_x, minimum_y, tip_x, tip_y)
            
            ratio = angle2 / (360.0 - angle1)
            mapped_value = self.map_value(ratio, 0.0, 1.0, 0.0, self.last_decimal_number if self.last_decimal_number is not None else 1.0)
            
            self.last_mapped_value = mapped_value
            self.publish_meter_value(mapped_value)

    def publish_meter_value(self, value):
        msg = Float32()
        msg.data = value  # Assign the floating point value directly
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published Meter Value: {msg.data}')


    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def calculate_angle(self, base_x, base_y, min_x, min_y, max_x, max_y):
        vec_min_x = min_x - base_x
        vec_min_y = min_y - base_y
        vec_max_x = max_x - base_x
        vec_max_y = max_y - base_y

        dot_product = vec_min_x * vec_max_x + vec_min_y * vec_max_y
        norm_min = math.sqrt(vec_min_x**2 + vec_min_y**2)
        norm_max = math.sqrt(vec_max_x**2 + vec_max_y**2)
        cos_theta = dot_product / (norm_min * norm_max)
        cos_theta = max(-1.0, min(1.0, cos_theta))

        angle_radians = math.acos(cos_theta)
        angle_degrees = angle_radians * (180.0 / math.pi)

        return angle_degrees

def main(args=None):
    rclpy.init(args=args)
    node = MeterValueCalculatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
