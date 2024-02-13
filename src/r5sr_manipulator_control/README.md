# r5sr_manipulator_control
マニピュレーターモータ(Dynemixel)制御パッケージ

## Node

### move_with_jointstate

joy,jointstateをサブスクライブしてdynamixelを制御する

#### Subscribed Topics
- **joy([sensor_msgs/Joy](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))**  moveitで制御されないジョイントの動作用入力
- **joint_states([sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))** moveitで計算した軸の角度情報

#### Published Topics
- **hand_current([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))** ハンドモータの電流値(mA)

#### Parameters
- **portname(string, default: "/dev/ttyUSB-Dynamixel")** dynamixelのttyポート
- **baudrate(int, default: 1'000'000)** ボーレート
