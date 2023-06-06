# R5SR
災害対応ロボット**R5S**のROS2パッケージ \
![r5s](images/r5s.JPG)

## 環境構築
### 要件
- Ubuntu22.04
- ros 2 humble

### 依存関係
```bash
source /opt/ros/humble/setup.bash
rosdep update

cd ws/src
git clone git@gitlab.com:nexis2/r5sr/r5sr.git

# NUC側の時は下記のスクリプトを実行
./r5sr/install-scripts/install-robot
vcs import < r5sr/r5sr_robot.rosinstall

# 操縦卓側の時は下記のスクリプトを実行
./r5sr/install-scripts/install-control_teleop
vcs import --recursive < r5sr/r5sr_control_teleop.rosinstall 

rosdep install -r -y -i --from-paths .
```

### ビルド
```bash
cd ws
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### ロボット側NUC PC
```bash
ros2 launch r5sr_bringup r5sr_bringup.launch.xml
```

### オペレーターPC
```bash
ros2 launch r5sr_teleop r5sr_teleop.launch.xml
```