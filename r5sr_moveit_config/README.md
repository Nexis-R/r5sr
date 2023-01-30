# r5sr_moveit_config

moveit2設定ファイル保管＆moveit2起動(アーム制御)パッケージ

## Launch

### moveit_demo.launch.py
moveit2のデモ
```bash
ros2 launch r5sr_moveit_config moveit_demo.launch.py
```

### r5sr_servo_teleop.launch.py
moveit2のreal time servo(順・逆運動学動作)起動
```bash
ros2 launch r5sr_moveit_config r5sr_servo_teleop.launch.py
```