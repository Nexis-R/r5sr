set shell := ["bash", "-c"]

ros_distro := env_var('ROS_DISTRO')

default:
  @just --list

alias dep := rosdep-install
alias b := build
alias c := clean
alias bu := bringup
alias t := teleop-foxglove
alias teleop := teleop-foxglove
alias tf := teleop-foxglove

_cd:
  @cd {{justfile_directory()}}

vcs-import: _cd
  vcs import --input depends.yaml --recursive src

rosdep-update:
  rosdep update --rosdistro {{ros_distro}}

rosdep-install: _cd rosdep-update vcs-import
  @sudo apt update
  rosdep install --from-paths src --ignore-src --rosdistro {{ros_distro}} -y
  @sudo apt upgrade -y

# 依存関係と開発環境のセットアップ. target: robot or teleop
setup target='teleop' : _cd
  scripts/setup.sh {{target}}
  just dep

# colcon build
build parallel='1' : _cd
  source /opt/ros/{{ros_distro}}/setup.bash && \
  colcon build --parallel-workers {{parallel}} --symlink-install --mixin ccache clang release compile-commands

# launch teleop with foxglove
teleop-foxglove args='use_wrs:=false use_audio:=true exp:=false vsting:=false': _cd
  source install/setup.bash
  ros2 launch r5sr_teleop teleop.foxglove.launch.py {{args}}

# launch bringup (robot)
bringup args='use_camera:=true use_audio:=true use_slam:=true use_rplidar:=false': _cd
  source install/setup.bash
  ros2 launch r5sr_bringup bringup.launch.py {{args}}

# clean build, install, log
clean: _cd
  -rm -rf build install log