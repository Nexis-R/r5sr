set shell := ["bash", "-c"]

ros_distro := env_var('ROS_DISTRO')

default:
  @just --list

alias dep := rosdep-install
alias b := build
alias c := clean

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

build executor='sequential' mixin='ninja ccache mold clang release compile-commands': _cd
  source /opt/ros/{{ros_distro}}/setup.bash
  colcon build --symlink-install --executor {{executor}} --mixin {{mixin}} --event-handlers console_direct+

[confirm]
clean: _cd
  -rm -rf build install log