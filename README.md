# R5SR
災害対応ロボット**R5S**のROS2パッケージ
![r5s](images/r5s.JPG)

## 環境構築
### 要件
- Ubuntu20.04
- ROS2 foxy

### 依存関係
```bash
source /opt/ros/foxy/setup.bash
rosdep update

cd ws/src
git clone git@gitlab.com:nexis2/r5sr/r5sr.git
./r5sr/install-scripts/install-all 
vcs import < r5sr/r5sr.rosinstall
rosdep install -r -y -i --from-paths .
```

### ビルド
```bash
cd ws
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### オペレーターPC
```bash
ros2 launch r5sr_teleop r5sr_teleop.launch.xml
```

### ロボット側NUC PC
```bash
ros2 launch r5sr_bringup r5sr_bringup.launch.xml
```


## パッケージ

| パッケージ                                            | 概要                                                        |
| ----------------------------------------------------- | ----------------------------------------------------------- |
| r5sr                                                  | メタパッケージ                                              |
| [r5sr_bringup](/r5sr_bringup)                         | ロボット内蔵NUC起動パッケージ                               |
| [r5sr_crawler_control](/r5sr_crawler_control)         | ロボット実機のクローラ(足回り)制御                          |
| [r5sr_description](/r5sr_description)                 | urdfやros_contorol・rvizの設定ファイル保管パッケージ        |
| [r5sr_manipulator_control](/r5sr_manipulator_control) | マニピュレータのDynamixelの制御パッケージ                   |
| [r5sr_moveit_config](/r5sr_moveit_config)             | moveit2設定ファイル保管＆moveit2起動(アーム制御)パッケージ  |
| [r5sr_moveit_teleop](/r5sr_moveit_teleop)             | moveit2でアームを順・逆運動学で動かす指令値を出すパッケージ |
| [r5sr_rviz_plugins](/r5sr_rviz_plugins)               | rvizプラグイン                                              |
| [r5sr_teleop](/r5sr_teleop)                           | オペレーターPC用起動パッケージ                              |