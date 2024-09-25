#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
import uuid
from ament_index_python.packages import get_package_share_directory
import time
from std_msgs.msg import String

class EqptValUpdater(Node):
    def __init__(self):
        super().__init__('eqpt_val_updater')

        # グローバル変数
        self.eqpt_id = None
        self.prim_value = None
        self.id_timestamp = None
        self.prim_value_timestamp = None

        # パラメータ
        self.rob_id = self.declare_parameter('robot_id', 0).value
        self.ip = self.declare_parameter('ip', '52.193.111.81').value
        self.file_path = self.declare_parameter('image_path', get_package_share_directory('r5sr_cloud') + '/snaps/cam2.jpg').value
        self.mac_id = self.declare_parameter('mac_id', '%012x' % uuid.getnode()).value
        self.time_tolerance = self.declare_parameter('time_tolerance', 300.0).value

        # パブリッシャー
        self.result_pub = self.create_publisher(String, 'update_result', 10)

        # サブスクライバー
        self.create_subscription(String, 'qrcode_info', self.id_callback, 10)
        self.create_subscription(String, 'recognition_value', self.prim_value_callback, 10)

        # ロボットのmac_idを登録
        self.session = requests.Session()
        self.register_mac_id()

    # 同じリクエストセッションでクラウドAPIにJSONデータをPOST
    # リクエスト制限を避けるため、単一のセッションを維持
    def post(self, url, data):
        response = self.session.post(url, json=data)
        return response.json()

    def id_callback(self, msg):
        if msg.data != '':
            self.eqpt_id = str(msg.data)
            self.id_timestamp = time.time()
            self.get_logger().info(f'id received: {self.eqpt_id}')
            # check_and_send_request() はここでは必要なし

    def prim_value_callback(self, msg):
        self.prim_value = float(msg.data)
        self.prim_value_timestamp = time.time()
        self.get_logger().info(f'Value received: {self.prim_value}')
        time.sleep(0.5)
        self.check_and_send_request()

    def check_and_send_request(self):
        if self.eqpt_id is not None and self.prim_value is not None:
            time_diff = abs(self.id_timestamp - self.prim_value_timestamp)
            if time_diff <= self.time_tolerance:
                self.get_logger().info("receive recognition result")
                # 修正: self.send_request() として関数を呼び出す
                self.send_request()

    def send_request(self):
        # eqpt_id、prim_value、mac_idを送信
        self.get_logger().info(f'eqpt_id: {self.eqpt_id}, Value: {self.prim_value}, mac_id: {self.mac_id}')
        url_send_value = f'http://{self.ip}/rms_wrs/api/set_eqpt_val.php'
        values = {
            "eqpt_nm": self.eqpt_id,
            "value": self.prim_value,
            "mac_id": self.mac_id
        }
        files = {'image': open(self.file_path, 'rb')}

        try:
            ret = requests.post(url_send_value, files=files, data=values)
            self.get_logger().info('posted')
            # レスポンスが'success'の場合のみ処理を進める
            if json.loads(ret.text)['status'] == 'success':
                report = String()
                report.data = f'ID: {self.eqpt_id}, Value: {self.prim_value}, mac_id: {self.mac_id}'
                self.result_pub.publish(report)
                self.get_logger().info(json.dumps(json.loads(ret.text), indent=4))
                self.get_logger().info('post success')
        except Exception as e:
            self.get_logger().error(f'Cannot Connect to Server: {e}')
            exit(3)

    def register_mac_id(self):
        # mac_id を登録するためのリクエストを送信
        url_rob_reg = f'http://{self.ip}/rms_wrs/api/set_mac_id.php'
        values = {
            "rob_id": self.rob_id,
            "mac_id": self.mac_id
        }

        try:
            ret = self.post(url_rob_reg, values)
            if ret['status'] == 'success':
                self.get_logger().info(json.dumps(ret, indent=4))
        except Exception as e:
            self.get_logger().error(f'Cannot Connect to Server: {e}')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = EqptValUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()