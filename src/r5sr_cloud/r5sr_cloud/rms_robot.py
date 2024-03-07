import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import requests
import json
import io


class RMSRobot(Node):

    def __init__(self):
        super().__init__(node_name='rms_robot', parameter_overrides=[])
        self.declare_parameter('server_ip', '0.0.0.0')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('mrt_id', '')
        self.declare_parameter('user_id', 0)
        self.declare_parameter('polling_interval_ms', 100)

        self.robot_id = self.get_parameter('robot_id').value
        self.robot_name = self.get_parameter('mrt_id').value

        self.get_logger().info('Robot ID: %d' % self.robot_id)
        self.get_logger().info('Robot Name: %s' % self.robot_name)

        self.server_uri = 'http://%s' % self.get_parameter('server_ip').value
        self.pos_changed = True
        self.last_req_id = 0
        self.img1_msg = None
        self.img2_msg = None
        self.img3_msg = None
        self.session = requests.Session()

        self.image1_sub = self.create_subscription(
            CompressedImage, '/camera1', self.image1_callback, 10)
        self.image2_sub = self.create_subscription(
            CompressedImage, '/camera2', self.image2_callback, 10)
        self.image3_sub = self.create_subscription(
            CompressedImage, '/camera3', self.image3_callback, 10)

        polling_interval_ms = self.get_parameter(
            'polling_interval_ms').value
        if polling_interval_ms is not None:
            polling_interval_ms = float(polling_interval_ms) / 1000.0
            self.timer = self.create_timer(
                polling_interval_ms, self.timer_callback)

    def image1_callback(self, msg):
        self.img1_msg = msg
    def image2_callback(self, msg):
        self.img2_msg = msg
    def image3_callback(self, msg):
        self.img3_msg = msg

    def timer_callback(self):
        if self.pos_changed:
            cur_pos = {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0,
                'pitch': 0.0,
                'roll': 0.0
            }
            values = {
                'ent_type': 'rt',
                'ent_id': self.robot_id,
                'last_req_id': self.last_req_id,
                'pos_json': json.dumps(cur_pos)
            }
            self.pos_changed = False
        else:
            values = {
                'ent_type': 'rt',
                'ent_id': self.robot_id,
                'last_req_id': self.last_req_id
            }

        try:
            ret = self.session.post(
                '%s/rms_v2/api/get_ent_req.php' % self.server_uri, json=values)
            if ret.ok and ret.json()['status'] == 'ok':
                ret_json = ret.json()
                self.last_req_id = ret_json['req_id']

                req_jsn = json.loads(ret_json['req_jsn'])

                match req_jsn['req_id']:
                    case 'ser_cur_pos':
                        pass
                    case 'set_home_pos':
                        pass
                    case 'set_max_speed':
                        pass
                    case 'set_light':
                        pass
                    case 'send_camera_image':
                        self.get_logger().info('send_camera_image')
                        cam_id = req_jsn['cam_id']
                        self.get_logger().info(cam_id)
                        if cam_id == 'cam1' and self.img1_msg is not None:
                            img_bytes = io.BytesIO(self.img1_msg.data)
                            files = {
                                'file': ('image.' + self.img1_msg.format, img_bytes, 'image/' + self.img1_msg.format)}
                        elif cam_id == 'cam2' and self.img2_msg is not None:
                            img_bytes = io.BytesIO(self.img2_msg.data)
                            files = {
                                'file': ('image.' + self.img2_msg.format, img_bytes, 'image/' + self.img2_msg.format)}
                        elif cam_id == 'cam3' and self.img3_msg is not None:
                            img_bytes = io.BytesIO(self.img3_msg.data)
                            files = {
                                'file': ('image.' + self.img3_msg.format, img_bytes, 'image/' + self.img3_msg.format)}
                        else :
                            img_bytes = None

                        if img_bytes is not None:
                            r_val = {
                                'rob_id': self.robot_id,
                                'req_id': ret_json['req_id'],
                                'erq_id': ret_json['erq_id'],
                                'from_ent_type': ret_json['from_ent_type'],
                                'from_ent_id': ret_json['from_ent_id'],
                                'res_jsn': json.dumps({'exe': 1, 'cam': cam_id}),
                            }
                            res = self.session.post(
                                '%s/rms_v2/api/res_rob_image.php' % self.server_uri, files=files, data=r_val)
                            self.get_logger().info('send_camera_image: %s' % res.text)

                        else:
                            self.get_logger().info('send_camera_image: No Image')
                    case 'send_cur_pos':
                        pass
                    case 'send_home_pos':
                        pass
                    case 'send_rob_info':
                        pass
                    case 'move':
                        pass
                    case 'turn':
                        pass
                    case 'go_to_goal':
                        pass
                    case 'go_to_home':
                        pass
                    case 'send_alert':
                        pass
                    case 'stop':
                        pass
                    case _:
                        pass

        except Exception as e:
            self.get_logger().error('Cannot Connect to Server: %s' % e)
            return


def main(args=None):
    rclpy.init(args=args)
    rms_robot = RMSRobot()
    rclpy.spin(rms_robot)
    rms_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
