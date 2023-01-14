# r5sr_moveit_teleop

moveit2でアームを順・逆運動学で動かす指令値を出すパッケージ

## Node

### joystick_servo
ジョイスティックの値(joy)をSubscribeし、moveit2のrealtimeservoの操縦に受け付ける指令値をpublishする

#### Subscribed Topics
- **joy_sub_([sensor_msgs/Joy](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))**  moveit2のアーム指令値に変換されるジョイスティック入力
#### Published Topics
- **twist_pub_([geometry_msgs/TwistStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html))**  moveit2のアームを逆運動学的動作を行う指令値

- **joint_pub_([control_msgs/JointJog](https://docs.ros.org/en/api/control_msgs/html/msg/JointJog.html))**  moveit2のアームを順運動学的動作を行う指令値

- **collision_pub_([moveit_msgs/PlanningScene](https://docs.ros.org/en/api/moveit_msgs/html/msg/PlanningScene.html))**  rviz上で衝突認識のある物体を出現させるための指令値(現在使われていない)

- **servo_start_client_([std_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html))**  moveit2のreal time servoを起動させるための指令値

## Launch

なし