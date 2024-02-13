# r5sr_crawler_control

足回りのクローラとフリッパー機構制御

## Node

### r5sr_craler_control
モータドライバ(Maxon Epos4)のドライバパッケージではなく, 速度指令値をpublishする.

#### Subscribed Topics
- **joy([sensor_msgs/Joy](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))**  フリッパー指令値に変換されるジョイスティック入力
- **cmd_vel([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))** クローラ移動機構の移動速度指令値

#### Published Topics
- **command_crawler_left([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** クローラ左モータへの速度指令値
- **command_crawler_right([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** クローラ右モータへの速度指令値
- **command_flipper_left_front([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** フリッパー左正面モータへの速度指令値
- **command_flipper_right_front([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** フリッパー右正面モータへの速度指令値
- **command_flipper_left_back([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** フリッパー左後面モータへの速度指令値
- **command_flipper_right_back([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** フリッパー右後面モータへの速度指令値

#### Parameters
- **wheel_diameter(double, default: 1.0)** ホイールの直径
- **wheel_base(double, default: 1.0)** ホイールベース

## Launch

### r5sr_crawler_control.launch.xml
r5sr_craler_controlとeposを起動する。

```bash
ros2 launch r5sr_crawler_control r5sr_crawler_control.launch.xml
```