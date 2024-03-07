import rclpy
from rclpy.node import Node
import requests
import json


class RMSPlant(Node):

    def __init__(self):
        super().__init__(node_name='rms_plant', parameter_overrides=[])
        self.declare_parameter('server_ip', '0.0.0.0')
        self.declare_parameter('plant_name', '')
        self.declare_parameter('site.id', 0)
        self.declare_parameter('site.name', '')
        self.declare_parameter('user_id', 0)
        self.declare_parameter('polling_interval_ms', 1000)

        self.site_id = self.get_parameter('site.id').value
        self.site_name = self.get_parameter('site.name').value
        self.plant_name = self.get_parameter('plant_name').value
        self.last_req_id = 0

        self.get_logger().info('Site ID: %d' % self.site_id)
        self.get_logger().info('Site Name: %s' % self.site_name)
        self.get_logger().info('Plant Name: %s' % self.plant_name)

        self.server_uri = 'http://%s' % self.get_parameter('server_ip').value
        self.session = requests.Session()

        init_val = {'site_id': self.site_id}
        try:
            ret = self.session.post('%s/rms_v2/api/get_site_init.php' %
                                    self.server_uri, json=init_val)

            if ret.ok and ret.json()['status'] == 'ok':
                ret_json = ret.json()
                self.floor_length = ret_json['length_x']
                self.floor_width = ret_json['length_y']
                self.origin_x = ret_json['origin_x']
                self.origin_y = ret_json['origin_y']

                self.url_floor_map = '%s/rms_v2/uploads/%s' % (self.server_uri,
                                                               ret_json['fmap'])
                self.url_pnid = '%s/rms_v2/uploads/%s' % (self.server_uri,
                                                          ret_json['pnid'])

                polling_interval_ms = self.get_parameter(
                    'polling_interval_ms').value
                if polling_interval_ms is not None:
                    polling_interval_ms = float(polling_interval_ms) / 1000.0
                    self.timer = self.create_timer(
                        polling_interval_ms, self.timer_callback)

                self.get_logger().info('Successfully Connected to Server')

        except Exception as e:
            self.get_logger().error('Cannot Connect to Server: %s' % e)
            self.destroy_node()
            return

    def timer_callback(self):
        values = {'ent_type': 'pt', 'ent_id': self.site_id,
                  'last_req_id': self.last_req_id}

        try:
            ret = self.session.post('%s/rms_v2/api/get_ent_req.php' %
                                    self.server_uri, json=values)
            if ret.ok and ret.json()['status'] == 'ok':
                ret_json = ret.json()
                self.last_req_id = ret_json['req_id']

                req_jsn = json.loads(ret_json['req_jsn'])
                if req_jsn['req_id'] == 'send_site_info':
                    site_info = {'site_id': self.site_id, 'site_name': self.site_name, 'plant_name': self.plant_name, 'lngth': self.floor_length,
                                 'width': self.floor_width, 'x0': self.origin_x, 'y0': self.origin_y, 'map2d': self.url_floor_map, 'pnid': self.url_pnid}
                    rvalues = {'req_id': ret_json['req_id'], 'erq_id': ret_json['erq_id'],
                               'from_ent_type': ret_json['from_ent_type'],
                               'from_ent_id': ret_json['from_ent_id'],
                               'res_jsn': json.dumps(site_info)}

                    rret = self.session.post('%s/rms_v2/api/res_site_info.php' %
                                             self.server_uri, json=rvalues)

                    self.get_logger().info(json.dumps(rret.json(), indent=4))

        except Exception as e:
            self.get_logger().error('Cannot Connect to Server: %s' % e)
            return


def main(args=None):
    rclpy.init(args=args)

    rms_plant = RMSPlant()

    rclpy.spin(rms_plant)

    rms_plant.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
