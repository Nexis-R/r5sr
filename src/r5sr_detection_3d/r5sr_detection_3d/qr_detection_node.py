import rclpy
from rclpy.time import Time, Duration
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PointStamped
from pyzbar import pyzbar
import numpy as np
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener


class QRDetectionNode(Node):
    def __init__(self):
        super().__init__('qr_detection_node')
        self.initialize_subscribers()
        self.initialize_publisher()
        self.initialize_parameters()

    def initialize_subscribers(self):
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

    def initialize_publisher(self):
        self.publisher_ = self.create_publisher(
            MarkerArray, 'qr_code_markers', 10)

    def initialize_parameters(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.set_default_camera_intrinsics()
        self.qr_code_positions = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def set_default_camera_intrinsics(self):
        self.fx = 614.1699
        self.fy = 614.9002
        self.cx = 329.9493
        self.cy = 237.2788

    def image_callback(self, msg):
        if self.depth_image is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.decode_qr(cv_image)
        marker_array = self.create_marker_array()

        self.publisher_.publish(marker_array)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def decode_qr(self, img):
        qr_codes = pyzbar.decode(img)
        img_center = np.array([img.shape[1] / 2, img.shape[0] / 2])

        color_width, color_height = 1280, 720
        depth_width, depth_height = 848, 480
        width_ratio = depth_width / color_width
        height_ratio = depth_height / color_height

        for qr_code in qr_codes:
            center = np.mean(qr_code.polygon, axis=0).astype(int) + np.array([150, 60])
            distance = np.linalg.norm(center - img_center)

            if distance > 150:
                continue

            depth_center = (
                center * np.array([width_ratio, height_ratio])).astype(int)
            world_center = self.pixel_to_world(depth_center)

            if world_center is None:
                continue

            camera_point = self.create_camera_point(world_center)

            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', 'camera_link', Time(), timeout=Duration(seconds=1.0))
                map_point = tf2_geometry_msgs.do_transform_point(
                    camera_point, trans)
            except Exception as e:
                self.get_logger().warning(f'Could not transform point: {e}')
                continue

            self.update_qr_code_position(qr_code, map_point)

    def create_marker_array(self):
        marker_array = MarkerArray()

        for text_markers in self.qr_code_positions.values():
            for markers in text_markers.values():
                for marker in markers:
                    marker_array.markers.append(marker)

        return marker_array

    def create_camera_point(self, world_center):
        camera_point = PointStamped()
        camera_point.header.stamp = self.get_clock().now().to_msg()
        camera_point.header.frame_id = 'camera_link'
        camera_point.point.x = world_center[0]
        camera_point.point.y = world_center[1]
        camera_point.point.z = world_center[2]

        return camera_point

    def update_qr_code_position(self, qr_code, map_point):
        min_distance = None
        closest_id = None
        text = qr_code.data.decode("utf-8")

        for unique_id, (existing_marker, _) in self.qr_code_positions.get(text, {}).items():
            distance = np.linalg.norm(np.array([existing_marker.pose.position.x - map_point.point.x,
                                                existing_marker.pose.position.y - map_point.point.y,
                                                existing_marker.pose.position.z - map_point.point.z]))
            if min_distance is None or distance < min_distance:
                min_distance = distance
                closest_id = unique_id

        if min_distance is None or min_distance > 0.5:
            unique_id = len(self.qr_code_positions.setdefault(
                text, {})) + hash(text) % 2147483647
            print("new marker: " + text + " ,  " + str(unique_id))
            new_marker, new_text_marker = self.create_qr_markers(
                qr_code.data, map_point, unique_id)
            self.qr_code_positions[text][unique_id] = (
                new_marker, new_text_marker)
        else:
            closest_marker, closest_text_marker = self.qr_code_positions[text][closest_id]
            closest_marker.pose.position.x = map_point.point.x
            closest_marker.pose.position.y = map_point.point.y
            closest_marker.pose.position.z = map_point.point.z
            closest_text_marker.pose.position.x = map_point.point.x
            closest_text_marker.pose.position.y = map_point.point.y
            closest_text_marker.pose.position.z = map_point.point.z + \
                0.15  # Add a small offset in the z direction

    def pixel_to_world(self, pixel):
        depth_idx = int(pixel[1]) * self.depth_image.shape[1] + int(pixel[0])
        if len(self.depth_image.flat) < depth_idx:
            return None
        depth = self.depth_image.flat[depth_idx]

        if depth == 0:
            return None

        depth = depth * 0.001  # Convert depth to meters
        x = (pixel[0] - self.cx) * depth / self.fx
        y = (pixel[1] - self.cy) * depth / self.fy
        z = depth

        # Coordinate transformation from Realsense to ROS
        ros_x = z
        ros_y = -x
        ros_z = -y

        return np.array([ros_x, ros_y, ros_z])

    def create_qr_markers(self, qr_code_data, map_point, unique_id):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'qr_codes'
        marker.id = unique_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = map_point.point.x
        marker.pose.position.y = map_point.point.y
        marker.pose.position.z = map_point.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        text_marker = self.create_text_marker(
            qr_code_data, map_point, unique_id)

        return marker, text_marker

    def create_text_marker(self, qr_code_data, map_point, unique_id):
        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.id = unique_id
        text_marker.ns = "qr_code_text"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = map_point.point.x
        text_marker.pose.position.y = map_point.point.y
        text_marker.pose.position.z = map_point.point.z
        text_marker.pose.position.z += 0.15  # Add a small offset in the z direction
        text_marker.scale.z = 0.1  # Text size
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.text = qr_code_data.decode("utf-8")

        return text_marker


def main(args=None):
    rclpy.init(args=args)
    qr_detection_node = QRDetectionNode()
    rclpy.spin(qr_detection_node)

    qr_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
