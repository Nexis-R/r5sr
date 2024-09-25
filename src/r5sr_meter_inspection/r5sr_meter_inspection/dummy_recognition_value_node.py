import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class DummyRecognitionValueNode(Node):

    def __init__(self):
        super().__init__('dummy_recognition_value_node')
        self.publisher_ = self.create_publisher(String, 'recognition_value', 10)
        self.timer = self.create_timer(1.0, self.publish_random_value)  # Publish every second

    def publish_random_value(self):
        msg = String()
        # Generate a random float between 0 and 1, rounded to 3 decimal places
        random_value = round(random.uniform(0, 1), 3)
        msg.data = str(random_value)
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyRecognitionValueNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
