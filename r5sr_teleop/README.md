# r5sr_teleop

オペレーターPC用起動パッケージ

## Node

### remap_joy
joyを実際の複雑なキーバインドにremapする

#### Subscribed Topics
- **joy([sensor_msgs/Joy](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))**  コントローラからのjoy信号
- **is_emergency_stopped([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))**  非常停止状態かどうか

#### Published Topics
- **out([sensor_msgs/Joy](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))**  remapされたJoy
- **teleop_mode([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html))** 遠隔操作モード

## Launch

### r5sr_teleop.launch.xml
通常の起動launchファイル。使用するすべての機能が含まれる
```bash
ros2 launch r5sr_teleop r5sr_teleop.launch.xml
```