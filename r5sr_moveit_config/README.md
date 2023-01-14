# r5sr_moveit_config

moveit2設定ファイル保管＆moveit2起動(アーム制御)パッケージ

## Node
設定ファイル保管＆moveit2起動(アーム制御)パッケージなのでノードなし

#### Subscribed Topics

#### Published Topics

## Launch

### moveit_demo.launch.py
moveit2のデモパッケージ
```bash
ros2 launch r5sr_moveit_config moveit_demo.launch.py
```

### r5sr_servo_teleop.launch.py
moveit2のreal time servo(順・逆運動学動作)起動パッケージ
```bash
ros2 launch r5sr_moveit_config r5sr_servo_teleop.launch.py
```